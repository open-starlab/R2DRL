# robocup2d/protocols/common.py
from __future__ import annotations
import struct
from typing import Final
import time

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
_U8:  Final[str] = "<B"
_I32: Final[str] = "<i"
_F32: Final[str] = "<f"

PHASE_NON_PLAY_ON: Final[int] = 0
PHASE_PLAY_ON: Final[int] = 1

TRAINER_PHASE_IDLE: Final[int] = 0
TRAINER_PHASE_BUSY: Final[int] = 1

# Deprecated compatibility aliases for older debug/IPC helpers.
# The active sync protocol is seq-based; do not use these as handshake truth.
FLAG_IDLE: Final[tuple[int, int]] = (0, 0)
FLAG_READY: Final[tuple[int, int]] = (0, 1)
FLAG_REQ: Final[tuple[int, int]] = (1, 0)
FLAG_DONE: Final[tuple[int, int]] = (1, 1)

WAIT_READY_TIMEOUT_MS: Final[int] = 30000
WAIT_DONE_TIMEOUT_MS:  Final[int] = 30000
POLL_US:               Final[int] = 100

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
