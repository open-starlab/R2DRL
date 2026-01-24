# robocup2d/ipc/handshake.py
from __future__ import annotations
from collections import Counter
import time
import struct
from typing import Final, Tuple, Iterable, Callable, Optional
from ..protocols.common import FLAG_READY, FLAG_REQ, ShmProtocolError
from ..protocols import P
# ============ Named flag states ============
# (A, B) are 2 uint8 values in shared memory.
# Convention in this project:
#   A: Python -> C++   (request bit)
#   B: C++   -> Python (ready/ack bit)

FLAGS_READY: Final[Tuple[int, int]] = FLAG_READY  # (0,1) C++ idle/ready
FLAGS_REQ:   Final[Tuple[int, int]] = FLAG_REQ    # (1,0) Python submitted a request

U8: Final[str] = "B"


def read_flags(buf, off_a: int, off_b: int) -> Tuple[int, int]:
    """Read (A,B) uint8 flags from shm."""
    a = struct.unpack_from(U8, buf, off_a)[0]
    b = struct.unpack_from(U8, buf, off_b)[0]
    return int(a), int(b)

def write_flags(buf, off_a: int, off_b: int, flags: Tuple[int, int]) -> None:
    """
    Write (A,B) flags with safe order: write B then A.
    This avoids the peer observing a “half-written” state.
    """
    a, b = int(flags[0]), int(flags[1])
    struct.pack_into(U8, buf, off_b, b)
    struct.pack_into(U8, buf, off_a, a)

def wait_flags(
    buf,
    off_a: int,
    off_b: int,
    want: Tuple[int, int] = FLAGS_READY,
    timeout: float = 2.0,
    poll: float = 0.001,
    log=None,
    tag: str = "",
) -> bool:
    """Spin until flags == want (True) or timeout (False)."""
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        if read_flags(buf, off_a, off_b) == want:
            return True
        time.sleep(poll)

    if log:
        a, b = read_flags(buf, off_a, off_b)
        head16 = list(bytes(buf[:16]))   # Note: Only convert to bytes() for printing, do not use it as the buffer itself.
        log.info(
            f"[handshake]{tag} wait_flags timeout want={want} got=({a},{b}) "
            f"off_a={off_a} off_b={off_b} head16={head16} buf_type={type(buf)}"
        )
    return False

def wait_all_ready(
    *,
    player_bufs,                 # [(buf, name), ...]
    off_a: int,
    off_b: int,
    tbuf,                        # ✅trainer shm buf（必须传；trainer 也要 READY 才返回）

    timeout: float = 60.0,
    poll: float = 0.0005,
    log=None,
    tag: str = "",

    # --- steady-state detection ---
    stuck_window: float = 10.0,      # ✅分布 dist 多久不变算“稳态/卡住”
    rescue_cooldown: float = 0.5,    # ✅两次 rescue 的最小间隔（防止狂推）

    # --- actions to push one transition ---
    is_hybrid: bool = False,
    default_action_base: int = 2,                 # ✅player 默认离散动作
    default_action_hybrid=(0, 0.5, 0.5),          # ✅player 默认连续动作 (a,u0,u1)
    trainer_opcode: int = 8,                      # ✅trainer 默认 opcode
    run_id = None,
):
    """
    ✅目标：直到（players + trainer）全员 flags == READY(0,1) 才返回。

    ✅不用 coach cycle。
    ✅只看 flags 的“分布是否稳定”：
       - 统计（players+trainer）的 flags 分布 dist
       - dist 在 stuck_window 内不变 => 认为进入稳态（可能卡住）
    ✅rescue（推进）策略：
       - 只对当前为 READY(0,1) 的实体写“默认动作 + REQ(1,0)”
       - player 写 action / hybrid_action；trainer 写 opcode
       - 然后继续等待直到全员 READY
    """
    FLAGS_READY = (0, 1)   # ✅READY: (A=0, B=1)
    FLAGS_REQ   = (1, 0)   # ✅REQ:   (A=1, B=0) 触发对端处理一次并回到 READY

    # ✅把 player 和 trainer 统一成一个 entities 列表（待遇一致，只有“写动作”不同）
    entities = [("player", buf, name) for (buf, name) in (player_bufs or [])]
    entities.append(("trainer", tbuf, "trainer"))

    total = len(entities)
    if total == 0:
        return True

    t_end = time.monotonic() + float(timeout)

    # ✅用于稳态检测：dist 变化则刷新 last_change_t
    last_dist = None
    last_change_t = time.monotonic()

    # ✅用于限频：两次 rescue 至少间隔 rescue_cooldown
    last_rescue_t = 0.0

    while True:
        now = time.monotonic()

        # =========================================================
        # (1) 读取 flags：构建 dist + 统计 READY 的实体
        # =========================================================
        pairs = []           # ✅所有实体的 flags 列表，用来统计分布
        ready_entities = []  # ✅当前处于 READY(01) 的实体（player + trainer）

        for kind, buf, name in entities:
            if kind == "trainer":
                a, b = P.trainer.read_flags(buf)
            else:
                a, b = read_flags(buf, off_a, off_b)

            ab = (int(a), int(b))
            pairs.append(ab)

            if ab == FLAGS_READY:
                ready_entities.append((kind, buf, name))

        # =========================================================
        # (2) 退出条件：全员 READY(01)
        # =========================================================
        if len(ready_entities) == total:
            return False

        # =========================================================
        # (3) 超时：打印 dist/ready 状态，直接抛错
        # =========================================================
        if now >= t_end:
            dist = Counter(pairs)
            msg = (
                f"[wait_all_ready]{tag} timeout: "
                f"ready={len(ready_entities)}/{total} dist={dict(dist)}"
                f"run_id={run_id}"
            )
            if log:
                log.info(msg)
            return True

        # =========================================================
        # (4) 稳态检测：dist 不变持续 stuck_window => stuck
        # =========================================================
        dist = Counter(pairs)
        if dist != last_dist:
            last_dist = dist
            last_change_t = now  # ✅分布变化，说明系统在动

        stuck = (now - last_change_t) >= float(stuck_window)

        # =========================================================
        # (5) rescue：stuck + 存在 READY 实体 + 通过 cooldown 限频
        #     只推进 READY(01) 的实体（谁是01就推谁）
        # =========================================================
        if stuck and ready_entities and (now - last_rescue_t) >= float(rescue_cooldown):
            pushed = 0

            for kind, buf, name in ready_entities:
                # ✅再读一次，避免刚好状态变了
                if kind == "trainer":
                    ta, tb = P.trainer.read_flags(buf)
                    if (int(ta), int(tb)) != FLAGS_READY:
                        continue

                    # ✅trainer 的“动作”是 opcode
                    P.trainer.write_opcode(buf, int(trainer_opcode))
                    # ✅trainer 置 REQ
                    write_flags(buf, P.trainer.T_FLAG_A, P.trainer.T_FLAG_B, P.common.FLAG_REQ)

                else:
                    a, b = read_flags(buf, off_a, off_b)
                    if (int(a), int(b)) != FLAGS_READY:
                        continue

                    # ✅player 的“动作”是 action / hybrid_action
                    if is_hybrid:
                        aa, u0, u1 = default_action_hybrid
                        P.player.write_hybrid_action(buf, int(aa), float(u0), float(u1), clamp=True)
                    else:
                        P.player.write_action(buf, int(default_action_base))

                    # ✅player 置 REQ
                    write_flags(buf, off_a, off_b, FLAGS_REQ)

                pushed += 1

            # ✅更新限频与稳态计时：给系统一个“推进窗口”
            last_rescue_t = now
            last_change_t = now

            if log:
                log.info(
                    f"[rescue]{tag} stuck>{stuck_window}s pushed={pushed} "
                    f"ready={len(ready_entities)}/{total} dist={dict(dist)}"
                )

        time.sleep(float(poll))
