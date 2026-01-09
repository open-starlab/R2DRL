# robocup2d/protocols/common.py
from __future__ import annotations
import struct
from typing import Final, Tuple


# ---- alignment helpers ----
def align4(x: int) -> int:
    """Round up x to the next multiple of 4 bytes (4-byte alignment for int32/float32 offsets)."""
    return (int(x) + 3) & ~3


def align8(x: int) -> int:
    """Round up x to the next multiple of 8 bytes (8-byte alignment for int64/double or stricter layouts)."""
    return (int(x) + 7) & ~7


def sizeof(fmt: str) -> int:
    """Return the byte size of a struct format string (e.g., 'B'=1, 'i'=4, 'f'=4)."""
    return struct.calcsize(fmt)


# ---- struct format aliases (optional) ----
U8 = "B"  # uint8:  1 byte unsigned integer
I32 = "i"  # int32:  4 bytes signed integer (platform-native size; usually 4)
F32 = "f"  # float32: 4 bytes IEEE-754 float

# ---- shared flag semantics (only if you truly share it across modules) ----
# Two-byte handshake flags (A, B) used in shared memory:
# - FLAG_READY: peer is idle/ready to accept a new request (safe to write payload + opcode)
# - FLAG_REQ:   requester has written payload/opcode and is requesting the peer to handle it
FLAG_READY: Final[Tuple[int, int]] = (0, 1)  # C++ ready/idle
FLAG_REQ: Final[Tuple[int, int]] = (1, 0)  # Python request submitted


def clamp01(v: float) -> float:
    """
    Clamp a numeric value into [0.0, 1.0] (useful for normalized continuous action params).
    """
    v = float(v)
    if v < 0.0:
        return 0.0
    if v > 1.0:
        return 1.0
    return v


class ShmProtocolError(RuntimeError):
    """
    Raised when shared-memory protocol invariants are violated (size/offset/flags/timeout/layout).
    """

    pass

def _close_unlink(d):
    for shm in d.values():
        _safe(shm.close)
        _safe(shm.unlink)

def _as_popen(x):
    return x.p if hasattr(x, "p") else x

def _safe(fn, *args, **kwargs):
    try:
        return fn(*args, **kwargs)
    except Exception:
        return None