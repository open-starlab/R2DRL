from __future__ import annotations

import re
from multiprocessing import shared_memory
from typing import Dict, List, Tuple

def create_shm(
    name: str,
    size: int,
    *,
    zero_fill: bool = False,
    log=None,
) -> shared_memory.SharedMemory:
    try:
        shm = shared_memory.SharedMemory(name=name, create=True, size=int(size))
        if zero_fill:
            shm.buf[:] = b"\x00" * size  # 广播填充，不用 * size
        # if log:
        #     log.info(f"[shm] created name={name} size={size} zero_fill={zero_fill}")
        return shm
    except FileExistsError:
        msg = f"[shm] already exists name={name} (refuse to attach/reuse)"
        if log:
            log.info(msg)
        raise RuntimeError(msg)

def create_shm_many_3lists(
    *,
    coach_names: Iterable[str],
    trainer_names: Iterable[str],
    player_names: Iterable[str],
    coach_size: int,
    trainer_size: int,
    player_size: int,
    zero_fill: bool = True,
    log=None,
) -> Tuple[
    Dict[str, shared_memory.SharedMemory],
    Dict[str, shared_memory.SharedMemory],
    Dict[str, shared_memory.SharedMemory],
]:
    """
    返回 (coach_owners, trainer_owners, player_owners)
    其中每个都是 {name: SharedMemory}（owner 句柄）。
    任意一个创建失败：回滚已创建的全部 shm。
    """
    coach_owners: Dict[str, shared_memory.SharedMemory] = {}
    trainer_owners: Dict[str, shared_memory.SharedMemory] = {}
    player_owners: Dict[str, shared_memory.SharedMemory] = {}

    def _create_group(dst: Dict[str, shared_memory.SharedMemory], names: Iterable[str], size: int, tag: str):
        size = int(size)
        for name in names:
            dst[name] = create_shm(name=name, size=size, zero_fill=zero_fill, log=log)
            # if log:
            #     log.info(f"[shm] group={tag} name={name} size={size}")

    try:
        _create_group(coach_owners, coach_names, coach_size, "coach")
        _create_group(trainer_owners, trainer_names, trainer_size, "trainer")
        _create_group(player_owners, player_names, player_size, "player")
        return coach_owners, trainer_owners, player_owners

    except Exception:
        # 回滚：把已经创建的 shm 全部清掉
        for d in (coach_owners, trainer_owners, player_owners):
            for shm in d.values():
                try: shm.close()
                except Exception: pass
                try: shm.unlink()
                except Exception: pass
        raise


def cleanup_shm(
    name: str,
    *,
    unlink: bool = True,
    log=None,
) -> None:
    try:
        shm = shared_memory.SharedMemory(name=name, create=False)
    except FileNotFoundError:
        # if log:
        #     log.info(f"[shm] cleanup skip (not found) name={name}")
        return
    except Exception as e:
        if log:
            log.info(f"[shm] cleanup attach failed name={name}: {e!r}")
        return

    try:
        shm.close()
        if log:
            log.info(f"[shm] closed name={name}")
    except Exception as e:
        if log:
            log.info(f"[shm] close failed name={name}: {e!r}")

    if unlink:
        try:
            shm.unlink()
            if log:
                log.info(f"[shm] unlinked name={name}")
        except FileNotFoundError:
            if log:
                log.info(f"[shm] unlink skip (already gone) name={name}")
        except Exception as e:
            if log:
                log.info(f"[shm] unlink failed name={name}: {e!r}")


def make_shm_name(
    run_id: str,
    base_port: int,
    kind: str,          # "player" | "coach" | "trainer"
    team1: str,
    team2: str,
    team_idx: int = 1,  # 1 or 2 (player/coach 用)
    unum: int = 1,      # 1..11 (player 用)
    prefix: str = "robocup2drl_",
) -> str:
    def san(s: str) -> str:
        s = (s or "").strip()
        s = re.sub(r"\W+", "_", s).strip("_")
        return s or "Team"

    t1, t2 = san(team1), san(team2)
    if t1.lower() == t2.lower():
        t1, t2 = f"{t1}_1", f"{t2}_2"

    if kind == "trainer":
        return f"/{prefix}{run_id}_trainer_p{int(base_port)}"

    if kind == "coach":
        team = t1 if team_idx == 1 else t2
        return f"/{prefix}{run_id}_coach_p{int(base_port)}_{team}"

    if kind == "player":
        team = t1 if team_idx == 1 else t2
        return f"/{prefix}{run_id}_player_p{int(base_port)}_{team}_u{int(unum):02d}"

    raise ValueError(f"kind must be player/coach/trainer, got {kind!r}")

def make_shm_plan(
    *,
    run_id: str,
    base_port: int,
    team1: str,
    team2: str,
    n1: int,
    n2: int,
    prefix: str = "robocup2drl_",
) -> Tuple[str, str, Dict[Tuple[int, int], str], List[str]]:
    n1 = int(n1)
    n2 = int(n2)

    mode1 = str(team1).lower()
    mode2 = str(team2).lower()

    coach_name = make_shm_name(
        run_id=run_id, base_port=base_port, kind="coach",
        team1=team1, team2=team2, team_idx=1, prefix=prefix
    )

    trainer_name = make_shm_name(
        run_id=run_id, base_port=base_port, kind="trainer",
        team1=team1, team2=team2, prefix=prefix
    )

    player_names: Dict[Tuple[int, int], str] = {}
    shm_names: List[str] = [coach_name, trainer_name]

    if mode1 != "helios":
        for unum in range(1, n1 + 1):
            name = make_shm_name(
                run_id=run_id, base_port=base_port, kind="player",
                team1=team1, team2=team2, team_idx=1, unum=unum, prefix=prefix
            )
            player_names[(1, unum)] = name
            shm_names.append(name)

    if mode2 != "helios":
        for unum in range(1, n2 + 1):
            name = make_shm_name(
                run_id=run_id, base_port=base_port, kind="player",
                team1=team1, team2=team2, team_idx=2, unum=unum, prefix=prefix
            )
            player_names[(2, unum)] = name
            shm_names.append(name)

    return coach_name, trainer_name, player_names, shm_names