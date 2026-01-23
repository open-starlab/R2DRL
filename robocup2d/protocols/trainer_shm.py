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


LEFT_FIXED = [
    (-45,  0,   0),  # 1 GK
    (-30, -10,  0),
    (-30,  10,  0),
    (-20, -20,  0),
    (-20,  20,  0),
    (-10, -15,  0),
    (-10,  15,  0),
    (  0, -10,  0),
    (  0,  10,  0),
    ( 10,  -5,  0),
    ( 10,   5,  0),
]
RIGHT_FIXED = [
    ( 45,  0, 180),  # 1 GK
    ( 30, -10, 180),
    ( 30,  10, 180),
    ( 20, -20, 180),
    ( 20,  20, 180),
    ( 10, -15, 180),
    ( 10,  15, 180),
    (  0, -10, 180),
    (  0,  10, 180),
    (-10,  -5, 180),
    (-10,   5, 180),
]

def write_fixed_reset(buf):
    write_reset_payload(
        buf,
        ball=BALL_201,
        left_players=LEFT_FIXED_201,
        right_players=RIGHT_FIXED_201,
        opcode=OP_RESET_FROM_PY,
    )

# swapped teams + mirrored across y-axis (x -> -x)
BALL_201 = (44.6042, 1.1437, 0, 0)

LEFT_FIXED_201 = [  # from original RIGHT_FIXED_201, mirrored
    (-49.4234,  0.2634,  89.778),   # 1
    (  0.6372, 11.3897,  -6.547),   # 2
    ( -0.8365, -3.8472,   7.368),   # 3
    (  7.5608, 24.3036, -21.431),   # 4
    (  8.6894,-14.7653, -51.874),   # 5
    ( 22.7476,  4.6502, -35.091),   # 6
    ( 31.3146, 17.6325, -62.962),   # 7
    ( 34.7294,  2.6678, -64.991),   # 8
    ( 36.9538, 25.0532, -55.418),   # 9
    ( 44.0104,  0.9512,  24.640),   # 10
    ( 36.2119, 16.3141, -64.104),   # 11
]

RIGHT_FIXED_201 = [  # from original LEFT_FIXED_201, mirrored
    ( 49.8224,  5.9949, -104.387),  # 1
    ( 41.5248,  6.0471,  178.048),  # 2
    ( 41.6839, 13.6533,  -87.564),  # 3
    ( 37.6738, -4.0914,  179.966),  # 4
    ( 42.3590, 33.1811,   86.107),  # 5
    ( 32.4109, 12.4692, -128.110),  # 6
    ( 21.9719, -1.1957,   84.286),  # 7
    ( 24.1698, 17.9297,   91.171),  # 8
    ( 10.6027,-15.2309,   92.451),  # 9
    ( 11.7947, 26.0258,  -59.406),  # 10
    ( 10.4533, 10.4779,  110.443),  # 11
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