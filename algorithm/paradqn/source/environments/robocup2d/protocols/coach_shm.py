from __future__ import annotations

import struct
from typing import Final, Union, Tuple
from .common import _U8, _I32
import numpy as np

Buf = Union[memoryview, bytearray, bytes]

# =========================================================
# Coach SHM layout (must match C++ exactly)
# =========================================================

COACH_STATE_FLOAT: Final[int] = 136
COACH_SHM_SIZE: Final[int] = 1 + 4 + COACH_STATE_FLOAT * 4 + 4 + 4  # 557 bytes

OFFSET_FLAG: Final[int] = 0
OFFSET_CYCLE: Final[int] = 1
OFFSET_STATE: Final[int] = 5
OFFSET_GAMEMODE: Final[int] = 549
OFFSET_GOAL: Final[int] = OFFSET_GAMEMODE + 4



# =========================================================
# Basic helper
# =========================================================

def assert_coach_shm_size(size: int) -> None:
    if int(size) != int(COACH_SHM_SIZE):
        raise RuntimeError(
            f"coach shm size mismatch: got={size} expected={COACH_SHM_SIZE}"
        )


# =========================================================
# Coach SHM View (Object-Oriented Access)
# =========================================================

class Coach:

    def __init__(self, buf: Buf):
        self.buf = buf

    # =========================
    # meta
    # =========================
    def flag(self) -> int:
        return struct.unpack_from(_U8, self.buf, OFFSET_FLAG)[0]

    def cycle(self) -> int:
        return struct.unpack_from(_I32, self.buf, OFFSET_CYCLE)[0]

    def gamemode(self) -> int:
        return struct.unpack_from(_I32, self.buf, OFFSET_GAMEMODE)[0]

    # =========================
    # raw state
    # =========================

    def ball(self, copy: bool = True) -> np.ndarray:
        s = self.state(copy=False)
        b = s[0:4]
        return b.copy() if copy else b

    def players(self, copy: bool = True) -> np.ndarray:
        s = self.state(copy=False)
        p = s[4:].reshape(22, 6)
        return p.copy() if copy else p

    def snapshot(self) -> Tuple[int, np.ndarray, np.ndarray, int]:
        cycle = self.cycle()
        gm = self.gamemode()
        s = self.state(copy=False)
        ball = s[0:4].copy()
        players = s[4:].reshape(22, 6).copy()
        return cycle, ball, players, gm

    # =========================
    # normalized state
    # =========================
    def state(self, copy: bool = True) -> np.ndarray:
        arr = np.frombuffer(
            self.buf,
            dtype=np.float32,
            count=COACH_STATE_FLOAT,
            offset=OFFSET_STATE,
        )
        return arr.copy() if copy else arr
    
    def state_norm(
        self,
        *,
        half_field_length: float,
        half_field_width: float,
    ) -> np.ndarray:

        PLAYER_VMAX = 1.05
        BALL_VMAX = 3.0

        s = self.state(copy=False)
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

        # direction normalize
        p[:, 4] = np.clip(p[:, 4], -180.0, 180.0) / 180.0

        # team id -> {-1,+1}
        tid = p[:, 5].copy()

        if np.all((tid == 0) | (tid == 1)):
            p[:, 5] = tid * 2.0 - 1.0
        elif np.all((tid == 1) | (tid == 2)):
            p[:, 5] = (tid - 1.0) * 2.0 - 1.0
        else:
            p[:, 5] = np.clip(tid, -1.0, 1.0)

        return out

    # =========================
    # goal
    # =========================
    def goal(self) -> int:
        return struct.unpack_from(_I32, self.buf, OFFSET_GOAL)[0]

    def clear_goal_flag(self) -> None:
        struct.pack_into(_I32, self.buf, OFFSET_GOAL, 0)

