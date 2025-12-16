# robocup2d/protocols/trainer_shm.py
from __future__ import annotations

from typing import Final, Sequence, Tuple, Union
import struct

from .common import align4

Buf = Union[memoryview, bytearray, bytes]

# ===================== Trainer SHM (shared memory) =====================
# Fixed byte layout shared between Python and C++ Trainer.
#
# Python workflow:
#   1) wait flags == (0,1)  (READY/IDLE)
#   2) write payload + opcode
#   3) submit request by flipping flags to (1,0)  (REQUEST)
#   4) wait flags == (0,1) again (DONE/READY)
#
# Alignment rule:
#   - int32/float32 should start at 4-byte aligned offsets.
#   - use align4(...) when placing int32/float32 fields.
# =========================================================

TRAINER_SHM_SIZE: Final[int] = 4096  # total bytes allocated for trainer control block

# ---- flags (2 bytes total) ----
T_FLAG_A: Final[int] = 0  # uint8: request flag from Python -> C++
T_FLAG_B: Final[int] = 1  # uint8: ready/ack flag from C++ -> Python

# bytes [2..3] padding, so int32 starts aligned
T_OPCODE: Final[int] = 4  # int32

# ---- ball payload (4 float32) ----
T_BALL_X:  Final[int] = align4(T_OPCODE + 4)  # 8
T_BALL_Y:  Final[int] = T_BALL_X + 4          # 12
T_BALL_VX: Final[int] = T_BALL_Y + 4          # 16
T_BALL_VY: Final[int] = T_BALL_VX + 4         # 20

# ---- players payload ----
N_LEFT: Final[int] = 11
N_RIGHT: Final[int] = 11
PLAYER_STRIDE: Final[int] = 3 * 4  # (x,y,dir_deg): 3 float32 = 12 bytes

T_PLAYERS_BASE: Final[int] = align4(T_BALL_VY + 4)  # 24

def T_LPX(i: int) -> int: return T_PLAYERS_BASE + i * PLAYER_STRIDE + 0 * 4
def T_LPY(i: int) -> int: return T_PLAYERS_BASE + i * PLAYER_STRIDE + 1 * 4
def T_LPD(i: int) -> int: return T_PLAYERS_BASE + i * PLAYER_STRIDE + 2 * 4

T_R_BASE: Final[int] = T_PLAYERS_BASE + N_LEFT * PLAYER_STRIDE  # 156

def T_RPX(i: int) -> int: return T_R_BASE + i * PLAYER_STRIDE + 0 * 4
def T_RPY(i: int) -> int: return T_R_BASE + i * PLAYER_STRIDE + 1 * 4
def T_RPD(i: int) -> int: return T_R_BASE + i * PLAYER_STRIDE + 2 * 4

# ---- opcode values ----
OP_NOP: Final[int] = 0
OP_RESET_FROM_PY: Final[int] = 10


# ---- struct formats (little-endian, explicit) ----
_U8:  Final[str] = "<B"
_I32: Final[str] = "<i"
_F32: Final[str] = "<f"


# ===================== Basic helpers =====================

def assert_trainer_shm_size(size: int) -> None:
    if int(size) != int(TRAINER_SHM_SIZE):
        raise RuntimeError(f"trainer shm size mismatch: got={size} expected={TRAINER_SHM_SIZE}")

def write_opcode(buf: Buf, opcode: int) -> None:
    struct.pack_into(_I32, buf, T_OPCODE, int(opcode))


def write_ball(buf: Buf, bx: float, by: float, bvx: float, bvy: float) -> None:
    struct.pack_into(_F32, buf, T_BALL_X, float(bx))
    struct.pack_into(_F32, buf, T_BALL_Y, float(by))
    struct.pack_into(_F32, buf, T_BALL_VX, float(bvx))
    struct.pack_into(_F32, buf, T_BALL_VY, float(bvy))


def write_left_players(buf: Buf, left_players: Sequence[Tuple[float, float, float]]) -> None:
    if len(left_players) != N_LEFT:
        raise ValueError(f"left_players must be length {N_LEFT}, got {len(left_players)}")
    for i, (x, y, deg) in enumerate(left_players):
        struct.pack_into(_F32, buf, T_LPX(i), float(x))
        struct.pack_into(_F32, buf, T_LPY(i), float(y))
        struct.pack_into(_F32, buf, T_LPD(i), float(deg))


def write_right_players(buf: Buf, right_players: Sequence[Tuple[float, float, float]]) -> None:
    if len(right_players) != N_RIGHT:
        raise ValueError(f"right_players must be length {N_RIGHT}, got {len(right_players)}")
    for i, (x, y, deg) in enumerate(right_players):
        struct.pack_into(_F32, buf, T_RPX(i), float(x))
        struct.pack_into(_F32, buf, T_RPY(i), float(y))
        struct.pack_into(_F32, buf, T_RPD(i), float(deg))


def write_players(
    buf: Buf,
    left_players: Sequence[Tuple[float, float, float]],
    right_players: Sequence[Tuple[float, float, float]],
) -> None:
    write_left_players(buf, left_players)
    write_right_players(buf, right_players)


def submit_request(buf: Buf, opcode: int) -> None:
    """
    Write opcode only. Flipping flags is handled by ipc/handshake.py.
    Payload should be written before calling this.
    """
    write_opcode(buf, opcode)

def write_reset_payload(
    buf: Buf,
    *,
    ball: Tuple[float, ...],
    left_players: Sequence[Tuple[float, float, float]],
    right_players: Sequence[Tuple[float, float, float]],
    opcode: int = OP_RESET_FROM_PY,
) -> None:
    if len(ball) == 2:
        bx, by = ball
        bvx, bvy = 0.0, 0.0
    elif len(ball) == 4:
        bx, by, bvx, bvy = ball
    else:
        raise ValueError(f"ball must be (x,y) or (x,y,vx,vy), got len={len(ball)}")

    write_ball(buf, bx, by, bvx, bvy)
    write_players(buf, left_players, right_players)
    write_opcode(buf, opcode)
