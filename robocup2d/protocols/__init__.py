#protocols/__init__.py
from dataclasses import dataclass
from . import common, player_shm, coach_shm, trainer_shm

@dataclass(frozen=True)
class Protocols:
    common = common
    player = player_shm
    coach  = coach_shm
    trainer = trainer_shm

P = Protocols()

__all__ = ["P"]
