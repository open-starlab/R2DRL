# robocup2d/protocols/trainer_shm.py
from __future__ import annotations

from typing import Final, Sequence, Tuple, Union
import struct
import time

from .common import align4, FLAG_READY, FLAG_REQ


Buf = Union[memoryview, bytearray]

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


# ---- opcode values (extend) ----
OP_RANDOM_RESET_ALL: Final[int] = 5   # 对应 C++ 里的 case 5（你现在的“全场随机站位”）


# ---- polling config ----
TRAINER_WAIT_READY_TIMEOUT_MS: Final[int] = 30000     # 30s
TRAINER_WAIT_DONE_TIMEOUT_MS:  Final[int] = 30000     # 30s
TRAINER_POLL_US:               Final[int] = 100       # 100us = 0.1ms （不是0.1s）

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

def write_fixed_reset(buf):
    write_reset_payload(
        buf,
        ball=BALL,
        left_players=LEFT_FIXED,
        right_players=RIGHT_FIXED,
        opcode=OP_RESET_FROM_PY,
    )
# frame 2738
BALL = (0.0, 0.0, 0.0, 0.0)  # (x, y, vx, vy)

LEFT_FIXED = [  # (x, y, body_deg)  # vx,vy 固定为 0
    (-49.4387,   0.0290,   94.2210),   # 1
    (-15.8660,  -4.7522,   16.9550),   # 2
    (-15.2561,   4.4188,  -16.2620),   # 3
    (-10.2079, -14.4404,   55.6470),   # 4
    (-11.5515,  13.1860,  -48.5820),   # 5
    ( -6.6729,  -0.2970,   -2.0820),   # 6
    ( -1.2355,  -6.6095,   77.0410),   # 7
    ( -0.9691,   6.7240,  -81.6370),   # 8
    ( -1.7809, -19.9735,   85.0590),   # 9
    ( -1.1504,  22.6600,   42.1780),   # 10
    ( -0.3979,  -0.0031, -178.0480),   # 11
]

RIGHT_FIXED = [  # (x, y, body_deg)  # vx,vy 固定为 0
    ( 49.3900,   0.0862,  -90.1690),   # 1
    ( 13.4133,   4.7750, -170.3380),   # 2
    ( 13.1628,  -4.1750,   71.8900),   # 3
    ( 13.1666,  15.2338,  160.2810),   # 4
    ( 12.8703, -13.3634, -163.4690),   # 5
    ( 13.9040,   0.9535,  176.0900),   # 6
    (  7.4706,   7.3084,  -46.1100),   # 7
    (  8.4330,  -7.8449,   46.7290),   # 8
    (  1.7011,  11.1080,  175.1860),   # 9
    (  2.4630, -11.3121,    8.9270),   # 10
    (  9.7244,   4.6590,  -63.9550),   # 11
]


def read_flags(buf: Buf) -> Tuple[int, int]:
    """读取 trainer flags: (A,B)"""
    return int(buf[T_FLAG_A]), int(buf[T_FLAG_B])

def write_flags(buf: Buf, a: int, b: int) -> None:
    """
    写 trainer flags: (A,B)
    写入顺序：先写 B 再写 A（与 C++/你之前约定一致）
    """
    buf[T_FLAG_B] = int(b) & 0xFF
    buf[T_FLAG_A] = int(a) & 0xFF

def wait_flags(
    buf: Buf,
    target: Tuple[int, int],
    *,
    timeout_ms: int,
    poll_us: int = TRAINER_POLL_US,
) -> bool:
    """轮询等待 flags==target，超时返回 False"""
    deadline = time.monotonic() + (timeout_ms / 1000.0)
    while time.monotonic() < deadline:
        if read_flags(buf) == target:
            return True
        time.sleep(poll_us / 1_000_000.0)
    return False

def wait_ready(
    buf: Buf,
    *,
    timeout_ms: int = TRAINER_WAIT_READY_TIMEOUT_MS,
    poll_us: int = TRAINER_POLL_US,
) -> bool:
    """等待 trainer 进入 READY(0,1)"""
    return wait_flags(buf, FLAG_READY, timeout_ms=timeout_ms, poll_us=poll_us)

def submit_opcode_request(buf: Buf, opcode: int) -> None:
    """
    写 opcode + 翻 flags 到 REQUEST(1,0)
    注意：是否等待 READY 由外层决定
    """
    write_opcode(buf, int(opcode))
    write_flags(buf, *FLAG_REQ)

def submit_opcode_and_wait_done(
    buf: Buf,
    opcode: int,
    *,
    ready_timeout_ms: int = TRAINER_WAIT_READY_TIMEOUT_MS,
    done_timeout_ms: int = TRAINER_WAIT_DONE_TIMEOUT_MS,
    poll_us: int = TRAINER_POLL_US,
) -> None:
    """
    常用封装：
      1) 等 READY(0,1)（否则直接报错）
      2) 写 opcode + 翻到 REQUEST(1,0)
      3) 等 trainer 处理完成回到 READY(0,1)
    """
    a, b = read_flags(buf)
    if (a, b) != FLAG_READY:
        # 你要“不是执行分支都报错”，这里直接抛
        raise RuntimeError(f"[trainer] not READY before submit: flags=({a},{b})")

    submit_opcode_request(buf, int(opcode))

    if not wait_ready(buf, timeout_ms=done_timeout_ms, poll_us=poll_us):
        a2, b2 = read_flags(buf)
        raise RuntimeError(f"[trainer] wait DONE/READY timeout: flags=({a2},{b2}), opcode={opcode}")

def submit_case5_and_wait_done(buf: Buf, **kwargs) -> None:
    """提交 C++ case 5，并等待处理完成"""
    return submit_opcode_and_wait_done(buf, OP_RANDOM_RESET_ALL, **kwargs)
