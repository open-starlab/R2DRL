from __future__ import annotations

from typing import Final, Sequence, Tuple, Union
import struct
import time

from .common import TRAINER_PHASE_BUSY, TRAINER_PHASE_IDLE, align4, _I32, _F32

Buf = Union[memoryview, bytearray]

# ===================== Trainer SHM layout =====================

TRAINER_SHM_SIZE: Final[int] = 4096

T_PHASE: Final[int] = 0
T_REQ_SEQ: Final[int] = 4
T_DONE_SEQ: Final[int] = 8
T_OPCODE: Final[int] = 12

T_BALL_X:  Final[int] = align4(T_OPCODE + 4)
T_BALL_Y:  Final[int] = T_BALL_X + 4
T_BALL_VX: Final[int] = T_BALL_Y + 4
T_BALL_VY: Final[int] = T_BALL_VX + 4

N_LEFT: Final[int] = 11
N_RIGHT: Final[int] = 11
PLAYER_STRIDE: Final[int] = 5 * 4

T_PLAYERS_BASE: Final[int] = align4(T_BALL_VY + 4)

def T_LPX(i: int) -> int: return T_PLAYERS_BASE + i * PLAYER_STRIDE + 0 * 4
def T_LPY(i: int) -> int: return T_PLAYERS_BASE + i * PLAYER_STRIDE + 1 * 4
def T_LPD(i: int) -> int: return T_PLAYERS_BASE + i * PLAYER_STRIDE + 2 * 4
def T_LVX(i: int) -> int: return T_PLAYERS_BASE + i * PLAYER_STRIDE + 3 * 4
def T_LVY(i: int) -> int: return T_PLAYERS_BASE + i * PLAYER_STRIDE + 4 * 4

T_R_BASE: Final[int] = T_PLAYERS_BASE + N_LEFT * PLAYER_STRIDE

def T_RPX(i: int) -> int: return T_R_BASE + i * PLAYER_STRIDE + 0 * 4
def T_RPY(i: int) -> int: return T_R_BASE + i * PLAYER_STRIDE + 1 * 4
def T_RPD(i: int) -> int: return T_R_BASE + i * PLAYER_STRIDE + 2 * 4
def T_RVX(i: int) -> int: return T_R_BASE + i * PLAYER_STRIDE + 3 * 4
def T_RVY(i: int) -> int: return T_R_BASE + i * PLAYER_STRIDE + 4 * 4

OP_NOOP: Final[int] = 0
OP_RESET_RANDOMLY: Final[int] = 1
OP_RESET_FROM_PY: Final[int] = 2
OP_PLAY_ON: Final[int] = 3


TRAINER_WAIT_READY_TIMEOUT_MS: Final[int] = 30000
TRAINER_WAIT_DONE_TIMEOUT_MS:  Final[int] = 30000
TRAINER_POLL_US:               Final[int] = 100

def assert_trainer_shm_size(size: int) -> None:
    if int(size) != int(TRAINER_SHM_SIZE):
        raise RuntimeError(
            f"trainer shm size mismatch: got={size} expected={TRAINER_SHM_SIZE}"
        )


# ===================== Trainer View =====================

class Trainer:

    def __init__(self, buf: Buf):
        self.buf = buf

    # ================= flags =================

    def phase(self) -> int:
        return int(self.buf[T_PHASE])

    def req_seq(self) -> int:
        return struct.unpack_from(_I32, self.buf, T_REQ_SEQ)[0]

    def done_seq(self) -> int:
        return struct.unpack_from(_I32, self.buf, T_DONE_SEQ)[0]

    def is_idle(self) -> bool:
        return self.done_seq() >= self.req_seq() and self.phase() != TRAINER_PHASE_BUSY

    def flags(self) -> Tuple[int, int]:
        """
        Backward-compatible debug projection of the old trainer flags.
        """
        if self.is_idle():
            return 0, 1
        return 1, 0

    def wait_ready(
        self,
        timeout_ms: int = TRAINER_WAIT_READY_TIMEOUT_MS,
        poll_us: int = TRAINER_POLL_US,
    ) -> bool:

        deadline = time.monotonic() + timeout_ms / 1000.0

        while time.monotonic() < deadline:
            if self.is_idle():
                return True
            time.sleep(poll_us / 1_000_000.0)

        return False

    def wait_done_seq(
        self,
        target_seq: int,
        timeout_ms: int = TRAINER_WAIT_DONE_TIMEOUT_MS,
        poll_us: int = TRAINER_POLL_US,
    ) -> bool:
        deadline = time.monotonic() + timeout_ms / 1000.0
        target_seq = int(target_seq)

        while time.monotonic() < deadline:
            if int(self.done_seq()) >= target_seq:
                return True
            time.sleep(poll_us / 1_000_000.0)

        return int(self.done_seq()) >= target_seq

    # ================= opcode =================

    def write_opcode(self, opcode: int) -> None:
        struct.pack_into(_I32, self.buf, T_OPCODE, int(opcode))

    def submit_opcode(self, opcode: int) -> int:
        next_seq = max(int(self.req_seq()), int(self.done_seq())) + 1
        self.write_opcode(opcode)
        struct.pack_into(_I32, self.buf, T_REQ_SEQ, int(next_seq))
        return next_seq

    def _zero_player_slot(self) -> Tuple[float, float, float, float, float]:
        return (0.0, 0.0, 0.0, 0.0, 0.0)

    def _pad_players(
        self,
        players: Sequence[Tuple[float, float, float, float, float]],
        target_len: int,
        side_name: str,
    ):
        if len(players) > target_len:
            raise ValueError(
                f"{side_name} players too many: got {len(players)}, max {target_len}"
            )

        out = [tuple(map(float, p)) for p in players]

        for p in out:
            if len(p) != 5:
                raise ValueError(
                    f"{side_name} player entry must have 5 values, got {p}"
                )

        while len(out) < target_len:
            out.append(self._zero_player_slot())

        return out
    
    # ================= payload =================

    def write_ball(self, bx, by, bvx, bvy):
        struct.pack_into(_F32, self.buf, T_BALL_X, float(bx))
        struct.pack_into(_F32, self.buf, T_BALL_Y, float(by))
        struct.pack_into(_F32, self.buf, T_BALL_VX, float(bvx))
        struct.pack_into(_F32, self.buf, T_BALL_VY, float(bvy))

    def write_left_players(self, left_players: Sequence[Tuple[float, float, float, float, float]]):
        left_players = self._pad_players(left_players, N_LEFT, "left")
        for i, (x, y, deg, vx, vy) in enumerate(left_players):
            struct.pack_into(_F32, self.buf, T_LPX(i), float(x))
            struct.pack_into(_F32, self.buf, T_LPY(i), float(y))
            struct.pack_into(_F32, self.buf, T_LPD(i), float(deg))
            struct.pack_into(_F32, self.buf, T_LVX(i), float(vx))
            struct.pack_into(_F32, self.buf, T_LVY(i), float(vy))

    def write_right_players(self, right_players: Sequence[Tuple[float, float, float, float, float]]):
        right_players = self._pad_players(right_players, N_RIGHT, "right")
        for i, (x, y, deg, vx, vy) in enumerate(right_players):
            struct.pack_into(_F32, self.buf, T_RPX(i), float(x))
            struct.pack_into(_F32, self.buf, T_RPY(i), float(y))
            struct.pack_into(_F32, self.buf, T_RPD(i), float(deg))
            struct.pack_into(_F32, self.buf, T_RVX(i), float(vx))
            struct.pack_into(_F32, self.buf, T_RVY(i), float(vy))

    def write_reset_payload(
        self,
        ball: Tuple[float, ...],
        left_players: Sequence[Tuple[float, float, float, float, float]],
        right_players: Sequence[Tuple[float, float, float, float, float]],
    ):
        if len(ball) == 2:
            bx, by = ball
            bvx, bvy = 0.0, 0.0
        elif len(ball) == 4:
            bx, by, bvx, bvy = ball
        else:
            raise ValueError

        self.write_ball(bx, by, bvx, bvy)
        self.write_left_players(left_players)
        self.write_right_players(right_players)

    def reset_players_and_ball(self, ball, left_players, right_players) -> int:
        """
        Write reset payload (ball + players), trigger OP_RESET_FROM_PY,
        and return the submitted request sequence for an exact ACK wait.
        left/right player counts may be <= 11; remaining slots are zero-padded.
        """
        self.write_reset_payload(ball, left_players, right_players)
        return self.submit_opcode(OP_RESET_FROM_PY)

    def noop(self) -> int:
        return self.submit_opcode(OP_NOOP)
