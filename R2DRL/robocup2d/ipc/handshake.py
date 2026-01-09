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

def wait_all_ready_or_done(
    *,
    player_bufs,          # [(buf, name), ...]
    off_a: int,
    off_b: int,
    cbuf = None,            # coach shm buf；为 None 时只等 READY + 超时
    poll: float = 0.001,
    stall_timeout: float = 200.0,   # cbuf=None 时把它当作 ready 超时；cbuf!=None 时当作 stall 超时
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

    # ----------------------------
    # Case A) cbuf is None: only READY + timeout, no return value
    # ----------------------------
    if cbuf is None:
        pending = list(player_bufs)
        if not pending:
            return  # no return value

        t_end = time.monotonic() + float(stall_timeout)

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
                return  # READY reached, no return value

            if time.monotonic() >= t_end:
                details = []
                for buf, name in pending:
                    a, b = read_flags(buf, off_a, off_b)
                    details.append(f"{name}=({a},{b})")
                msg = "wait_all_ready timeout: pending=" + ", ".join(details)
                if log:
                    log.info(msg)
                raise ShmProtocolError(msg)

            if not progressed:
                time.sleep(float(poll))

    # ----------------------------
    # Case B) cbuf is provided: your original logic (GOAL > READY) + stall check
    # ----------------------------
    if current_cycle is None:
        current_cycle = -1
    current_cycle = int(current_cycle)

    # 0) wait until cycle > current_cycle (至少前进一帧)
    t0 = time.monotonic()
    while True:
        cycle = int(P.coach.read_cycle(cbuf))
        if cycle > current_cycle:
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

        # priority: GOAL first
        if goal != 0:
            return "GOAL", cycle, gm, goal

        # then READY
        if not pending:
            return "READY", cycle, gm, goal

        # stall (lowest priority)
        now = time.monotonic()
        if cycle != last_cycle:
            last_cycle = cycle
            last_change_t = now
        elif (now - last_change_t) > float(stall_timeout):
            msg = f"[stall] coach cycle not advancing: cycle={cycle} gm={gm} goal={goal} current_cycle={current_cycle}"
            if log:
                log.info(msg)
            raise RuntimeError(msg)

        if not progressed:
            time.sleep(float(poll))

def wait_all_ready(
    *,
    player_bufs,          # [(buf, name), ...]
    off_a: int,
    off_b: int,
    timeout: float = 2.0,
    poll: float = 0.001,
    log=None,
):
    """只等所有 player 到 READY；成功 return；超时 raise ShmProtocolError。"""
    pending = list(player_bufs)
    if not pending:
        return

    t_end = time.monotonic() + float(timeout)

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
            msg = "wait_all_ready timeout: pending=" + ", ".join(details)
            if log:
                log.info(msg)
            raise ShmProtocolError(msg)

        if not progressed:
            time.sleep(float(poll))