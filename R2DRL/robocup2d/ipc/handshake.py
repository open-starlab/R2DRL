# robocup2d/ipc/handshake.py
from __future__ import annotations

import time
import struct
from typing import Final, Tuple
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

def wait_ready_or_raise(
    buf,
    off_a: int,
    off_b: int,
    timeout: float = 2.0,
    poll: float = 0.001,
    log=None,
    tag: str = "",
) -> None:
    """Common helper: wait for READY else raise."""
    if not wait_flags(buf, off_a, off_b, want=FLAGS_READY, timeout=timeout, poll=poll, log=log, tag=tag):
        a, b = read_flags(buf, off_a, off_b)
        raise ShmProtocolError(f"wait READY timeout: got=({a},{b}) tag={tag}")

def wait_all_ready_or_raise(
    bufs,  # Iterable[Tuple[Buf, str]]  -> (buf, name)
    off_a: int,
    off_b: int,
    timeout: float = 2.0,
    poll: float = 0.001,
    log=None,
    tag: str = "",
) -> None:
    """
    Barrier: Poll flags of all bufs until all are READY; otherwise raise ShmProtocolError on timeout.
    bufs: [(buf, name), ...]
    """
    pending = list(bufs)
    if not pending:
        return

    t_end = time.monotonic() + timeout

    while True:
        progressed = False

        i = 0
        while i < len(pending):
            buf, name = pending[i]
            if read_flags(buf, off_a, off_b) == FLAGS_READY:
                pending.pop(i)
                progressed = True
                continue
            i += 1

        if not pending:
            return

        if time.monotonic() >= t_end:
            details = []
            for buf, name in pending:
                a, b = read_flags(buf, off_a, off_b)
                details.append(f"{name}=({a},{b})")
            msg = "wait ALL READY timeout: pending=" + ", ".join(details) + f" tag={tag}"
            if log:
                log.info(f"[handshake]{tag} {msg}")
            raise ShmProtocolError(msg)

        if not progressed:
            time.sleep(poll)

def wait_until_playon_or_done(
    *,
    cbuf,
    begin_cycle: int,
    episode_limit: int,
    goal_x: float,
    goal_y: float,
    max_stall_sec: float = 20.0,   # If cycle does not advance for this many seconds, consider it stalled.
    poll: float = 0.05,
    log=None,
    tag: str = "",
    current_coach_cycle=None,
):
    """
    Wait until:
      - gm==2 (PlayOn) => ready_to_act=True
      - OR gm==1/timeover OR timeout OR goal => ready_to_act=False

    Additionally: Detect if cycle advances; if not advancing for a long time, consider it stalled/server disconnected.
    Returns: (ready_to_act, cycle, gm, ball)
    """

    last_cycle = None
    last_change_t = time.monotonic()

    while True:
        cycle = int(P.coach.read_cycle(cbuf))
        gm = int(P.coach.read_gamemode(cbuf))
        goal = int(P.coach.read_goal_flag(cbuf))

        # ---- cycle stall detection ----
        if last_cycle is None:
            last_cycle = cycle
            last_change_t = time.monotonic()
        else:
            if cycle != last_cycle:
                last_cycle = cycle
                last_change_t = time.monotonic()
            else:
                if (time.monotonic() - last_change_t) > float(max_stall_sec):
                    msg = f"[stall]{tag} coach cycle not advancing: cycle={cycle} gm={gm}"
                    if log: 
                        log.info(msg)
                    raise RuntimeError(msg)

        # ✅ After action: Must wait for the next frame (cycle > current_coach_cycle) before returning.
        if current_coach_cycle is not None and cycle <= int(current_coach_cycle):
            time.sleep(float(poll))
            continue

        timeout = (begin_cycle >= 0) and ((cycle - begin_cycle) >= int(episode_limit))
        
        
        if gm == 1 or timeout or (goal != 0):
            return False, cycle, gm

        if gm == 2:
            return True, cycle, gm
        
        time.sleep(float(poll))

