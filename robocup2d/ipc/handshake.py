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
