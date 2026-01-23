# robocup2d/protocols/player_shm.py
from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Final, Tuple, Optional, Union

import numpy as np

from .common import align4, clamp01

Buf = Union[memoryview, bytearray, bytes]

# =============================================================================
# Player SHM layout (must match C++ SamplePlayer exactly)
# Units: bytes. Alignment: int32/float32 fields start at 4-byte boundaries.
# =============================================================================

STATE_NUM: Final[int] = 97
BASE_MASK_NUM: Final[int] = 17

OFFSET_FLAG_A: Final[int] = 0
OFFSET_FLAG_B: Final[int] = 1
OFFSET_MASK:   Final[int] = align4(OFFSET_FLAG_B + 1)                 # 4
OFFSET_CYCLE:  Final[int] = align4(OFFSET_MASK + BASE_MASK_NUM)       # 24
OFFSET_STATE:  Final[int] = align4(OFFSET_CYCLE + 4)                  # 28
OFFSET_ACTION: Final[int] = align4(OFFSET_STATE + STATE_NUM * 4)       # 416

OFFSET_HYBRID_MASK: Final[int] = align4(OFFSET_ACTION + 4)            # 420
OFFSET_HYBRID_ACT:  Final[int] = align4(OFFSET_HYBRID_MASK + 4)       # 424
OFFSET_HYBRID_U0:   Final[int] = align4(OFFSET_HYBRID_ACT + 4)        # 428
OFFSET_HYBRID_U1:   Final[int] = align4(OFFSET_HYBRID_U0 + 4)         # 432

PLAYER_SHM_SIZE: Final[int] = align4(OFFSET_HYBRID_U1 + 4)                   # 436

# ---- struct formats (force little-endian for cross-platform consistency) ----
_U8:  Final[str] = "<B"
_I32: Final[str] = "<i"
_F32: Final[str] = "<f"
_MASK17: Final[str] = f"<{BASE_MASK_NUM}B"  # 17 uint8
_MASK4:  Final[str] = "<4B"                 # 4 uint8


# =============================================================================
# Data containers (optional but nice)
# =============================================================================

@dataclass(frozen=True)
class Flags:
    a: int
    b: int


@dataclass(frozen=True)
class HybridAction:
    a: int
    u0: float
    u1: float


# =============================================================================
# Basic helpers
# =============================================================================

def assert_player_shm_size(size: int) -> None:
    """Raise if shared memory segment size doesn't match expected SHM_SIZE."""
    if int(size) != int(PLAYER_SHM_SIZE):
        raise RuntimeError(f"player shm size mismatch: got={size} expected={PLAYER_SHM_SIZE}")


# =============================================================================
# Flags / cycle
# =============================================================================

def read_cycle(buf: Buf) -> int:
    """Read simulation cycle (int32)."""
    return int(struct.unpack_from(_I32, buf, OFFSET_CYCLE)[0])

# =============================================================================
# Masks
# =============================================================================

def read_base_mask(buf: Buf) -> np.ndarray:
    """Read base action availability mask: uint8[17] -> np.int32[17]."""
    raw = struct.unpack_from(_MASK17, buf, OFFSET_MASK)  # tuple(len=17)
    return np.asarray(raw, dtype=np.int32)

def read_hybrid_mask(buf: Buf) -> np.ndarray:
    """Read hybrid action availability mask: uint8[4] -> np.int32[4]."""
    raw = struct.unpack_from(_MASK4, buf, OFFSET_HYBRID_MASK)  # tuple(len=4)
    return np.asarray(raw, dtype=np.int32)


# =============================================================================
# Observation
# =============================================================================

def read_obs(buf: Buf, *, copy: bool = True) -> np.ndarray:
    """
    Read observation vector float32[97] from OFFSET_STATE.

    copy=True:
      - returns a standalone numpy array (safe; shm can change later)
    copy=False:
      - returns a view into shm buffer (fast; but buffer must stay alive)
    """
    arr = np.frombuffer(buf, dtype=np.float32, count=STATE_NUM, offset=OFFSET_STATE)
    return arr.copy() if copy else arr


# =============================================================================
# Base action
# =============================================================================

def write_action(buf: Buf, act: int) -> None:
    """Write base discrete action (int32) to OFFSET_ACTION."""
    struct.pack_into(_I32, buf, OFFSET_ACTION, int(act))


# =============================================================================
# Hybrid action
# =============================================================================

def write_hybrid_action(
    buf: Buf,
    a: int,
    u0: float,
    u1: float,
    *,
    clamp: bool = True,
) -> None:
    """
    Write hybrid action:
      - hybrid_act: int32
      - hybrid_u0/u1: float32

    clamp=True will clamp u0/u1 into [0,1].
    """
    if clamp:
        u0 = clamp01(u0)
        u1 = clamp01(u1)
    struct.pack_into(_I32, buf, OFFSET_HYBRID_ACT, int(a))
    struct.pack_into(_F32, buf, OFFSET_HYBRID_U0, float(u0))
    struct.pack_into(_F32, buf, OFFSET_HYBRID_U1, float(u1))


# robocup2d/protocols/player_shm.py
def read_obs_norm(
    buf,
    *,
    half_field_length: float,   # e.g. 52.5
    half_field_width: float,    # e.g. 34.0
    stamina_max: float = 8000.0,
    copy: bool = True,
) -> np.ndarray:
    """
    Normalize 97-dim obs from SamplePlayer::getAllState().

    Layout (as your comment implies):
      self: [0:6]   -> x,y,vx,vy,stamina,kickable
      ball: [6:10]  -> x,y,vx,vy
      opp:  [10:54] -> 11x4 (x,y,vx,vy)
      mate: [54:94] -> 10x4 (x,y,vx,vy)
      rest: [94:97] -> flags/gamemode etc (kept raw)
    """
    # hard-coded from your server config
    PLAYER_VMAX = 1.05
    BALL_VMAX   = 3.0

    o = read_obs(buf, copy=copy).astype(np.float32, copy=False)

    HL = float(half_field_length)
    HW = float(half_field_width)
    sm = float(stamina_max) 

    # --- self ---
    o[0] /= HL          # x
    o[1] /= HW          # y
    o[2] /= PLAYER_VMAX # vx
    o[3] /= PLAYER_VMAX # vy
    o[4] /= sm          # stamina
    # o[5] kickable keep

    # --- ball ---
    o[6] /= HL          # x
    o[7] /= HW          # y
    o[8] /= BALL_VMAX   # vx
    o[9] /= BALL_VMAX   # vy

    # --- opponents 11x4: [x,y,vx,vy] ---
    opp = o[10:54].reshape(11, 4)
    opp[:, 0] /= HL
    opp[:, 1] /= HW
    opp[:, 2] /= PLAYER_VMAX
    opp[:, 3] /= PLAYER_VMAX

    # --- mates 10x4: [x,y,vx,vy] ---
    mate = o[54:94].reshape(10, 4)
    mate[:, 0] /= HL
    mate[:, 1] /= HW
    mate[:, 2] /= PLAYER_VMAX
    mate[:, 3] /= PLAYER_VMAX

    return o