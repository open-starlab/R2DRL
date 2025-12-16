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
    buf: Buf,
    *,
    field_length: float,   # HALF_LENGTH=52.5
    field_width: float,    # HALF_WIDTH =34.0
    player_vmax: float = 1.2,   # 经验默认：player_speed_max 常见约 1.2（不准就自己改成 ServerParam 读到的值）
    ball_vmax: float = 2.7,     # 经验默认：ball_speed_max 常见约 2.7
    stamina_max: float = 4000.0,# 经验默认：stamina_max 常见约 4000
    copy: bool = True,
    clip: bool = True,
) -> np.ndarray:
    """
    Normalize your 97-dim obs produced by SamplePlayer::getAllState().

    - positions: x/field_length, y/field_width
    - velocities: /player_vmax (players), /ball_vmax (ball)
    - stamina: /stamina_max
    - flags kept as {0,1}
    - gamemode kept raw (float int)
    """
    # 注意：归一化最好别对 shm view in-place，所以这里强制拿 copy 来改
    o = read_obs(buf, copy=True).astype(np.float32, copy=False)

    L = float(field_length)
    W = float(field_width)
    pv = float(player_vmax)
    bv = float(ball_vmax)
    sm = float(stamina_max) if stamina_max else 1.0

    # --- self ---
    o[0] /= L;  o[1] /= W
    o[2] /= pv; o[3] /= pv
    o[4] /= sm
    # o[5] kickable keep

    # --- ball ---
    o[6] /= L;  o[7] /= W
    o[8] /= bv; o[9] /= bv

    # --- opponents 11x4: [x,y,vx,vy] ---
    opp = o[10:54].reshape(11, 4)
    opp[:, 0] /= L; opp[:, 1] /= W
    opp[:, 2] /= pv; opp[:, 3] /= pv

    # --- mates 10x4: [x,y,vx,vy] ---
    mate = o[54:94].reshape(10, 4)
    mate[:, 0] /= L; mate[:, 1] /= W
    mate[:, 2] /= pv; mate[:, 3] /= pv

    if clip:
        # 位置/速度/体力做个裁剪，避免偶发异常值把网络炸掉
        o[0:4] = np.clip(o[0:4], -1.0, 1.0)
        o[4]   = np.clip(o[4],  0.0, 1.0)
        o[6:10]= np.clip(o[6:10], -1.0, 1.0)
        o[10:94] = np.clip(o[10:94], -1.0, 1.0)

    return o
