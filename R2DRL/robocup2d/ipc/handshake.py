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

def wait_all_ready_or_done(
    *,
    player_bufs,          # [(buf, name), ...]
    off_a: int,
    off_b: int,
    cbuf = None,            # coach shm buf；为 None 时只等 READY + 超时
    poll: float = 0.001,
    stall_timeout: float = 20.0,   # cbuf=None 时把它当作 ready 超时；cbuf!=None 时当作 stall 超时
    current_cycle: int = 0,
    log=None,
):
    """
    If cbuf is None:
      - wait until all READY, or timeout -> raise ShmProtocolError
      - return None

    If cbuf is provided:
      - ensure cycle > current_cycle (至少前进一帧)
      - then: GOAL first, READY second
      - stall_timeout: coach cycle 长时间不前进 -> raise RuntimeError
    """

    if current_cycle is None:
        current_cycle = -1
    current_cycle = int(current_cycle)

    # 0) wait until cycle > current_cycle (至少前进一帧)
    t0 = time.monotonic()
    while True:
        cycle = int(P.coach.read_cycle(cbuf))
        gm    = int(P.coach.read_gamemode(cbuf))
        if cycle > current_cycle or gm == 9:
            last_cycle = cycle
            break
        time.sleep(float(poll))
        if time.monotonic() - t0 > float(stall_timeout):
            msg = f"[stall] waiting for new coach cycle timed out: current_cycle={current_cycle} now_cycle={cycle}"
            if log:
                log.info(msg)
            raise RuntimeError(msg)

    pending = list(player_bufs)
    last_change_t = time.monotonic()

    while True:
        progressed = False

        # 1) poll players READY
        i = 0
        while i < len(pending):
            buf, name = pending[i]
            if read_flags(buf, off_a, off_b) == FLAGS_READY:
                pending.pop(i)
                progressed = True
                continue
            i += 1

        # 2) read coach
        cycle = int(P.coach.read_cycle(cbuf))
        gm    = int(P.coach.read_gamemode(cbuf))
        goal  = int(P.coach.read_goal_flag(cbuf))

        # # priority: GOAL first
        # if goal != 0:
        #     return "GOAL", cycle, gm, goal

        # then READY
        if not pending:
            return "READY", cycle, gm, goal

        # stall (lowest priority)
        now = time.monotonic()
        if cycle != last_cycle:
            last_cycle = cycle
            last_change_t = now
        elif (now - last_change_t) > float(stall_timeout):
            # dump pending players flags
            pending_details = []
            pairs = []
            for buf, name in pending:
                a, b = read_flags(buf, off_a, off_b)
                a, b = int(a), int(b)
                pairs.append((a, b))
                pending_details.append(f"{name}=({a},{b})")

            dist = Counter(pairs)

            msg = (
                f"[stall] coach cycle not advancing: "
                f"cycle={cycle} gm={gm} goal={goal} current_cycle={current_cycle} "
                f"pending={len(pending)} dist={dict(dist)} "
                f"details=" + ", ".join(pending_details)
            )
            if log:
                log.info(msg)
            raise RuntimeError(msg)

        if not progressed:
            time.sleep(float(poll))
def wait_all_ready(
    *,
    player_bufs,                 # [(buf, name), ...]
    off_a: int,
    off_b: int,
    timeout: float = 20.0,
    poll: float = 0.0005,
    log=None,
    tag: str = "",

    # --- stable-state rescue (no coach / no cycle) ---
    tbuf=None,                   # trainer shm buf (optional)
    trainer_opcode: int = 8,

    stuck_window: float = 0.01,  # 00/01 混态分布保持不变超过该时间 -> 触发救援
    rescue_cooldown: float = 0.02,  # rescue 后给系统一点推进时间（防止狂推）

    # default action to push one transition
    is_hybrid: bool = False,
    default_action_base: int = 2,                 # doIntercept
    default_action_hybrid=(0, 0.5, 0.5),          # (a,u0,u1)
):
    """
    等待所有 player flags == READY(0,1)；满足则直接 return（与原版一致）。

    若未全 READY，但进入稳定态（pending 的 flags 只出现 {00,01} 且同时存在 00/01，
    且该分布在 stuck_window 内不变），则执行“推进救援”：
      - 对仍为 READY(01) 的 pending 球员：写入默认动作 + 置 REQ(10)
      - trainer：仅用于一起推进/诊断；trainer==11 忽略；仅 trainer==01 时写 opcode+REQ
    然后继续等待直到全 READY 或 timeout。
    """
    pending = list(player_bufs)
    if not pending:
        return

    t_end = time.monotonic() + float(timeout)

    last_dist = None
    last_change_t = time.monotonic()

    last_rescue_t = 0.0
    rescued_names = set()     # 本次稳定窗口内已推过的 READY 球员
    trainer_sent_in_window = False

    while True:
        # 1) pop READY
        i = 0
        while i < len(pending):
            buf, name = pending[i]
            if read_flags(buf, off_a, off_b) == FLAGS_READY:
                pending.pop(i)
                rescued_names.discard(name)
                continue
            i += 1

        if not pending:
            return

        now = time.monotonic()
        if now >= t_end:
            pairs, details = [], []
            for buf, name in pending:
                a, b = read_flags(buf, off_a, off_b)
                a = int(a); b = int(b)
                pairs.append((a, b))
                details.append(f"{name}=({a},{b})")
            dist = Counter(pairs)

            trainer_str = ""
            if tbuf is not None:
                ta, tb = P.trainer.read_flags(tbuf)
                trainer_str = f" trainer_flags=({int(ta)},{int(tb)})"

            msg = (
                f"[wait_all_ready]{tag} timeout: pending={len(pending)} "
                f"dist={dict(dist)} details=" + ", ".join(details) + trainer_str
            )
            if log:
                log.info(msg)
            raise ShmProtocolError(msg)

        # 2) compute pending dist + detect stable mixed {00,01}
        pairs = []
        ready_unserved = []  # pending 中当前为 01 且还没推过的
        for buf, name in pending:
            a, b = read_flags(buf, off_a, off_b)
            a = int(a); b = int(b)
            pairs.append((a, b))
            if (a, b) == FLAGS_READY and name not in rescued_names:
                ready_unserved.append((buf, name))

        dist = Counter(pairs)

        # dist 变化 -> 新窗口：刷新计时/状态
        if dist != last_dist:
            last_dist = dist
            last_change_t = now
            rescued_names.clear()
            trainer_sent_in_window = False

        stuck = ((now - last_change_t) >= float(stuck_window))

        # 3) rescue: stable + 有可推的 READY(01)
        if stuck and ready_unserved and (now - last_rescue_t) >= float(rescue_cooldown):
            # trainer push (optional)
            if tbuf is not None and (not trainer_sent_in_window):
                ta, tb = P.trainer.read_flags(tbuf)
                ta = int(ta); tb = int(tb)
                # 11 ignore；01 才推
                if (ta, tb) != (1, 1) and (ta, tb) == (0, 1):
                    P.trainer.write_opcode(tbuf, int(trainer_opcode))
                    write_flags(tbuf, P.trainer.T_FLAG_A, P.trainer.T_FLAG_B, P.common.FLAG_REQ)
                    trainer_sent_in_window = True

            pushed = 0
            for buf, name in ready_unserved:
                # 再确认一次仍为 READY
                if read_flags(buf, off_a, off_b) != FLAGS_READY:
                    continue
                if is_hybrid:
                    a, u0, u1 = default_action_hybrid
                    P.player.write_hybrid_action(buf, int(a), float(u0), float(u1), clamp=True)
                else:
                    P.player.write_action(buf, int(default_action_base))

                write_flags(buf, off_a, off_b, FLAGS_REQ)
                rescued_names.add(name)
                pushed += 1

            last_rescue_t = now
            last_change_t = now  # 给推进一点 grace

            if log:
                trainer_str = ""
                if tbuf is not None:
                    ta2, tb2 = P.trainer.read_flags(tbuf)
                    trainer_str = f" trainer_flags=({int(ta2)},{int(tb2)})"
                log.info(
                    f"[rescue]{tag} stuck(mixed00/01) pushed={pushed} "
                    f"dist={dict(dist)} pending={len(pending)}{trainer_str}"
                )

        time.sleep(float(poll))

def wait_all_ready_with_rescue(
    *,
    player_bufs,                 # [(buf, name), ...]
    off_a: int,
    off_b: int,
    cbuf,                        # coach shm buf (must exist)
    tbuf,                        # trainer shm buf (must exist)
    current_cycle: int,
    poll: float = 0.0005,
    stall_timeout: float = 1.0,
    log=None,

    # --- rescue policy ---
    is_hybrid: bool = False,
    default_action_base: int = 2,                 # 2 = doIntercept (追球/拦截)
    default_action_hybrid=(0, 0.5, 0.5),
    trainer_opcode: int = 8,
):
    """
    目标：直到所有 player flags == READY(0,1) 才返回。

    stall 时若 players flags 只呈现 {00,01} 且同时存在 00/01：
      - 对当前 01 的球员补写默认动作并置 REQ(10) 推进一帧
      - trainer 只有在 READY(01) 才发 REQ；其他状态不动
    """

    current_cycle = int(current_cycle)

    last_cycle = int(P.coach.read_cycle(cbuf))
    last_change_t = time.monotonic()

    # 防止同一 cycle 内重复 rescue
    last_rescue_cycle = -1
    trainer_sent_cycle = -1

    # 可选：避免同一 stall 窗口里反复推同一个名字（你也可以删掉，仅靠 last_rescue_cycle 即可）
    rescued_names = set()

    # 分层轮询：常态更省 CPU，临近 stall 更敏感
    slow_poll = max(float(poll), 0.002)   # 2ms
    mid_poll  = max(float(poll), 0.0008)  # 0.8ms
    fast_poll = float(poll)              # 用户给的快轮询

    while True:
        now = time.monotonic()

        # ---- coach ----
        cycle = int(P.coach.read_cycle(cbuf))
        gm    = int(P.coach.read_gamemode(cbuf))
        goal  = int(P.coach.read_goal_flag(cbuf))

        if cycle != last_cycle:
            last_cycle = cycle
            last_change_t = now
            rescued_names.clear()

        # ---- players flags（用计数替代 Counter，加速）----
        n01 = 0
        n00 = 0
        n_other = 0
        ready_list = []  # [(buf,name)] 仅收集 01 且未被推过的人

        for buf, name in player_bufs:
            a, b = read_flags(buf, off_a, off_b)
            a = int(a); b = int(b)

            if (a, b) == (0, 1):
                n01 += 1
                if name not in rescued_names:
                    ready_list.append((buf, name))
            elif (a, b) == (0, 0):
                n00 += 1
            else:
                n_other += 1

        all_ready = (n01 == len(player_bufs))
        advanced  = (cycle > current_cycle)

        # ✅退出条件：至少推进了一帧 且 全员 READY
        if advanced and all_ready:
            return cycle, gm, goal

        # ---- 未 stall：正常等（分层 sleep）----
        age = now - last_change_t
        stalled = age > float(stall_timeout)
        if not stalled:
            if age < stall_timeout * 0.5:
                time.sleep(slow_poll)
            elif age < stall_timeout:
                time.sleep(mid_poll)
            else:
                time.sleep(fast_poll)
            continue

        # ---- stall：判断是否可 rescue ----
        can_rescue_players = (n01 > 0 and not all_ready)

        # 同一 cycle 已经 rescue 过：给它时间推进，不要立刻报错/重复推
        if can_rescue_players and cycle == last_rescue_cycle:
            last_change_t = now
            time.sleep(fast_poll)
            continue

        if can_rescue_players:
            # trainer：每个 cycle 最多发一次；且只在 READY(01) 才发
            ta, tb = P.trainer.read_flags(tbuf)
            ta = int(ta); tb = int(tb)
            if (ta, tb) == (0, 1) and cycle != trainer_sent_cycle:
                P.trainer.write_opcode(tbuf, int(trainer_opcode))
                write_flags(
                    tbuf,
                    P.trainer.T_FLAG_A,
                    P.trainer.T_FLAG_B,
                    P.common.FLAG_REQ
                )
                trainer_sent_cycle = cycle

            pushed = 0
            for buf, name in ready_list:
                if is_hybrid:
                    a, u0, u1 = default_action_hybrid
                    P.player.write_hybrid_action(buf, int(a), float(u0), float(u1), clamp=True)
                else:
                    P.player.write_action(buf, int(default_action_base))

                write_flags(buf, off_a, off_b, P.common.FLAG_REQ)
                rescued_names.add(name)
                pushed += 1

            last_rescue_cycle = cycle
            last_change_t = now  # 给系统推进的 grace time

            if log:
                ta2, tb2 = P.trainer.read_flags(tbuf)
                dist_dbg = {"00": n00, "01": n01, "other": n_other}
                log.info(
                    f"[rescue] stalled: cycle={cycle} gm={gm} goal={goal} "
                    f"dist={dist_dbg} pushed={pushed} trainer_flags={(int(ta2),int(tb2))}"
                )

            time.sleep(fast_poll)
            continue

        # ---- stall 但不可救：报错 ----
        ta, tb = P.trainer.read_flags(tbuf)
        dist_dbg = {"00": n00, "01": n01, "other": n_other}
        msg = (
            f"[stall][fatal] coach cycle not advancing: "
            f"cycle={cycle} gm={gm} goal={goal} current_cycle={current_cycle} "
            f"dist={dist_dbg} trainer_flags={(int(ta),int(tb))}"
        )
        if log:
            log.info(msg)
        raise RuntimeError(msg)
