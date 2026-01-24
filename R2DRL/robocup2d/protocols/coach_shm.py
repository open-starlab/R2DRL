# robocup2d/protocols/coach_shm.py
from __future__ import annotations

import struct
from typing import Final, Union, Tuple
import numpy as np

Buf = Union[memoryview, bytearray, bytes]

COACH_STATE_FLOAT: Final[int] = 136
COACH_SHM_SIZE: Final[int] = 1 + 4 + COACH_STATE_FLOAT * 4 + 4 + 4  # 557

# byte offsets (NOTE: intentionally unaligned)
OFFSET_FLAG: Final[int] = 0
OFFSET_CYCLE: Final[int] = 1          # int32: bytes 1..4
OFFSET_STATE: Final[int] = 5          # float32[136]: bytes 5..548
OFFSET_GAMEMODE: Final[int] = 549     # int32: bytes 549..552
OFFSET_GOAL: Final[int] = OFFSET_GAMEMODE + 4  # 553..556


_U8:  Final[str] = "<B"
_I32: Final[str] = "<i"
_F32_136: Final[str] = f"<{COACH_STATE_FLOAT}f"


def assert_coach_shm_size(size: int) -> None:
    if int(size) != int(COACH_SHM_SIZE):
        raise RuntimeError(f"coach shm size mismatch: got={size} expected={COACH_SHM_SIZE}")


def read_flag(buf: Buf) -> int:
    return int(struct.unpack_from(_U8, buf, OFFSET_FLAG)[0])


def read_cycle(buf: Buf) -> int:
    return int(struct.unpack_from(_I32, buf, OFFSET_CYCLE)[0])


def read_gamemode(buf: Buf) -> int:
    return int(struct.unpack_from(_I32, buf, OFFSET_GAMEMODE)[0])


def read_state_floats(buf: Buf, *, copy: bool = True) -> np.ndarray:
    """
    Returns float32[136] = ball(4) + players(22*6)
    copy=True returns a standalone numpy array.
    """
    arr = np.frombuffer(buf, dtype=np.float32, count=COACH_STATE_FLOAT, offset=OFFSET_STATE)
    return arr.copy() if copy else arr


def read_ball(buf: Buf, *, copy: bool = True) -> np.ndarray:
    s = read_state_floats(buf, copy=False)
    ball = s[0:4]
    return ball.copy() if copy else ball


def read_players(buf: Buf, *, copy: bool = True) -> np.ndarray:
    """
    Returns (22,6) float32:
      (x,y,vx,vy,dir_deg,team_id)
    """
    s = read_state_floats(buf, copy=False)
    players = s[4:].reshape(22, 6)
    return players.copy() if copy else players


def read_snapshot(buf: Buf) -> Tuple[int, np.ndarray, np.ndarray, int]:
    """
    Convenience: (cycle, ball(4,), players(22,6), gamemode)
    """
    cycle = read_cycle(buf)
    gm = read_gamemode(buf)
    s = read_state_floats(buf, copy=False)
    ball = s[0:4].copy()
    players = s[4:].reshape(22, 6).copy()
    return cycle, ball, players, gm

def read_state_norm(
    buf,
    *,
    half_field_length: float,
    half_field_width: float,
    n_team1: int = 11,
    n_team2: int = 11,
) -> np.ndarray:
    """Normalized state float32[136] from coach shm (vmax hard-coded)."""
    # hard-coded from your server config
    PLAYER_VMAX = 1.05
    BALL_VMAX   = 3.0

    s = read_state_floats(buf, copy=False)  # view (float32[136])
    hl = float(half_field_length)
    hw = float(half_field_width)

    out = s.copy().astype(np.float32, copy=False)

    # --- ball ---
    out[0] /= hl
    out[1] /= hw
    out[2] /= BALL_VMAX
    out[3] /= BALL_VMAX

    # --- players ---
    p = out[4:].reshape(22, 6)
    p[:, 0] /= hl
    p[:, 1] /= hw
    p[:, 2] /= PLAYER_VMAX
    p[:, 3] /= PLAYER_VMAX
    p[:, 4] = np.clip(p[:, 4], -180.0, 180.0) / 180.0

    # --- team_id -> {-1,+1} ---
    tid = p[:, 5].copy()
    if np.all((tid == 0) | (tid == 1)):
        p[:, 5] = tid * 2.0 - 1.0
    elif np.all((tid == 1) | (tid == 2)):
        p[:, 5] = (tid - 1.0) * 2.0 - 1.0
    else:
        p[:, 5] = np.clip(tid, -1.0, 1.0)

    # --- Slice and Concat ---
    # ball(4) + team1(n_team1*6) + team2(n_team2*6)
    # Team1 is at indices 0..n_team1-1 (in p)
    # Team2 is at indices 11..11+n_team2-1 (in p)
    
    # In flat 'out':
    # ball: 0..4
    # team1: 4 .. 4 + n_team1*6
    # team2: 4 + 66 .. 4 + 66 + n_team2*6
    
    return np.concatenate([
        out[0:4],
        out[4 : 4 + n_team1*6],
        out[70 : 70 + n_team2*6]
    ])

def read_goal_flag(buf: Buf) -> int:
    """+1=左队进球; -1=右队进球; 0=无/已清零"""
    return int(struct.unpack_from(_I32, buf, OFFSET_GOAL)[0])

def clear_goal_flag(buf: Buf) -> None:
    struct.pack_into(_I32, buf, OFFSET_GOAL, 0)
