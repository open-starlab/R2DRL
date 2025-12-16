# robocup2d/protocols/coach_shm.py
from __future__ import annotations

import struct
from typing import Final, Union, Tuple
import numpy as np

Buf = Union[memoryview, bytearray, bytes]

COACH_STATE_FLOAT: Final[int] = 136
COACH_SHM_SIZE: Final[int] = 1 + 4 + COACH_STATE_FLOAT * 4 + 4  # 553

# byte offsets (NOTE: intentionally unaligned)
OFFSET_FLAG: Final[int] = 0
OFFSET_CYCLE: Final[int] = 1          # int32: bytes 1..4
OFFSET_STATE: Final[int] = 5          # float32[136]: bytes 5..548
OFFSET_GAMEMODE: Final[int] = 549     # int32: bytes 549..552

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
    buf: Buf,
    *,
    field_length: float,
    field_width: float,
    copy: bool = True,
) -> np.ndarray:
    """
    Returns normalized float32[136] in a stable range:
      ball: (x,y,vx,vy) -> [-1,1]
      players: (x,y,vx,vy,dir_deg,team_id) with:
        x,y,v -> [-1,1], dir_deg -> [-1,1], team_id -> {-1,+1} (heuristic mapping)
    """
    s = read_state_floats(buf, copy=False)  # view

    hl = float(field_length) * 0.5
    hw = float(field_width) * 0.5
    vp = 1.0
    vb = 1.0

    out = s.copy() if copy else s  # if copy=False, you accept modifying underlying view (usually don't)
    if not copy:
        # 强烈建议 copy=True；这里防止误用
        out = s.copy()

    # ball
    out[0] = np.clip(out[0] / hl, -1.0, 1.0)
    out[1] = np.clip(out[1] / hw, -1.0, 1.0)
    out[2] = np.clip(out[2] / vb, -1.0, 1.0)
    out[3] = np.clip(out[3] / vb, -1.0, 1.0)

    # players view
    p = out[4:].reshape(22, 6)
    p[:, 0] = np.clip(p[:, 0] / hl, -1.0, 1.0)
    p[:, 1] = np.clip(p[:, 1] / hw, -1.0, 1.0)
    p[:, 2] = np.clip(p[:, 2] / vp, -1.0, 1.0)
    p[:, 3] = np.clip(p[:, 3] / vp, -1.0, 1.0)
    p[:, 4] = np.clip(p[:, 4], -180.0, 180.0) / 180.0

    tid = p[:, 5].copy()
    if np.all((tid == 0) | (tid == 1)):
        p[:, 5] = tid * 2.0 - 1.0
    elif np.all((tid == 1) | (tid == 2)):
        p[:, 5] = (tid - 1.0) * 2.0 - 1.0
    else:
        p[:, 5] = np.clip(tid, -1.0, 1.0)

    return out.astype(np.float32, copy=False)
