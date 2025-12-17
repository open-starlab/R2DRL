from __future__ import annotations

import os
import re
import subprocess
from dataclasses import dataclass
from typing import Dict, IO, List, Optional, Sequence, Tuple, Union

LogFP = Union[int, IO[str]]

@dataclass
class ProcInfo:
    p: subprocess.Popen
    kind: str
    team: str
    unum: int
    shm_name: str
    port: int
    log_path: str

def popen_logged(
    args: Sequence[str],
    *,
    cwd: Optional[str] = None,
    env: Optional[dict] = None,
    log_path: Optional[str] = None,
) -> Tuple[subprocess.Popen, LogFP]:

    if not log_path:
        p = subprocess.Popen(
            list(map(str, args)),
            cwd=cwd,
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        return p, subprocess.DEVNULL

    d = os.path.dirname(log_path)
    if d:
        os.makedirs(d, exist_ok=True)

    fd = os.open(log_path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o644)
    try:
        p = subprocess.Popen(
            list(map(str, args)),
            cwd=cwd,
            env=env,
            stdout=fd,
            stderr=fd,  # 合并到同一个文件
            start_new_session=True,
        )
        return p, subprocess.DEVNULL
    finally:
        os.close(fd)  # ✅ 父进程立刻关闭

# ----------------------------
# High-level launch helpers
# ----------------------------

@dataclass(frozen=True)
class Ports:
    server: int
    trainer: int   # offline coach port (trainer connects here)
    coach: int     # online coach port
    debug: int

def _safe_team(team: str) -> str:
    return re.sub(r"\W+", "_", str(team or "Team")).strip("_") or "Team"

def _join_logs_dir(logs_dir: str, filename: str) -> str:
    os.makedirs(logs_dir, exist_ok=True)
    return os.path.join(logs_dir, filename)

def launch_server(
    *,
    server_path: str,
    server_port: int,
    trainer_port: int,
    coach_port: int,
    logs_dir: str,
    rcg_dir: str,
    env: Optional[dict] = None,
    log_tag: str = "",
    extra_args: Optional[List[str]] = None,
) -> Tuple[subprocess.Popen, LogFP]:
    """
    Launch rcssserver with standard flags (synch/auto/coach) + ports.
    """
    os.makedirs(rcg_dir, exist_ok=True)

    args = [
        server_path,
        f"--server::port={int(server_port)}",
        f"--server::coach_port={int(trainer_port)}",     # offline coach port
        f"--server::olcoach_port={int(coach_port)}",     # online coach port
        "--server::coach=true",
        "--server::coach_w_referee=true",
        "--server::auto_mode=true",
        "--server::synch_mode=true",
        "--server::synch_offset=60",
        "--server::synch_see_offset=60",
        "--server::simulator_step=100",
        "--server::send_step=100",
        f"--server::game_log_dir={rcg_dir}",
        f"--server::text_log_dir={rcg_dir}",
        "--server::text_logging=false",
    ]
    if extra_args:
        args.extend(list(map(str, extra_args)))

    log_name = f"{log_tag}server_{server_port}.log" if log_tag else f"server_{server_port}.log"
    return popen_logged(args, env=env, log_path=_join_logs_dir(logs_dir, log_name))

def launch_trainer(
    *,
    trainer_dir: str,
    trainer_exe: str,
    host: str,
    trainer_port: int,
    team1: str,
    team2: str,
    trainer_shm_name: str,
    logs_dir: str,
    env: Optional[dict] = None,
    log_tag: str = "",
    # you can pass extra flags for sample_trainer here if needed
    extra_args: Optional[List[str]] = None,
) -> Tuple[subprocess.Popen, LogFP]:
    """
    Launch sample_trainer. Pass shm name via env var RCSC_TRAINER_SHM.
    """
    env2 = dict(env or os.environ.copy())
    env2["RCSC_TRAINER_SHM"] = str(trainer_shm_name)

    args = [
        trainer_exe,
        "-h", str(host),
        "-p", str(int(trainer_port)),
        "--teaml", str(team1),
        "--teamr", str(team2),
    ]
    if extra_args:
        args.extend(list(map(str, extra_args)))

    log_name = f"{log_tag}trainer_{trainer_port}.log" if log_tag else f"trainer_{trainer_port}.log"
    return popen_logged(args, cwd=trainer_dir, env=env2, log_path=_join_logs_dir(logs_dir, log_name))

def launch_coach(
    *,
    coach_dir: str,
    coach_exe: str,
    host: str,
    coach_port: int,
    coach_team: str,
    coach_shm_name: str,
    logs_dir: str,
    env: Optional[dict] = None,
    log_tag: str = "",
    extra_args: Optional[List[str]] = None,
) -> Tuple[subprocess.Popen, LogFP]:
    """
    Launch sample_coach with explicit --shm-name.
    """
    args = [
        coach_exe,
        "-h", str(host),
        "-p", str(int(coach_port)),
        "-t", str(coach_team),
        "--shm-name", str(coach_shm_name),
        "--server_wait_seconds", "30"
    ]
    if extra_args:
        args.extend(list(map(str, extra_args)))

    log_name = f"{log_tag}coach_{coach_port}.log" if log_tag else f"coach_{coach_port}.log"
    return popen_logged(args, cwd=coach_dir, env=env, log_path=_join_logs_dir(logs_dir, log_name))

def launch_players(
    *,
    player_dir: str,
    player_exe: str,
    host: str,
    server_port: int,
    debug_host: str,
    debug_port: int,
    team1: str,
    team2: str,
    n1: int,
    n2: int,
    # only players in this mapping get --shm-name
    # key: (team_idx, unum) -> shm_name
    player_shm_by_key: Dict[Tuple[int, int], str],
    logs_dir: str,
    env: Optional[dict] = None,
    log_tag: str = "",
    extra_args_common: Optional[List[str]] = None,
) -> Tuple[List[ProcInfo], List[LogFP]]:
    """
    Launch all players of both teams.
    - Adds --shm-name only if (team_idx,unum) exists in player_shm_by_key
    - Adds --goalie for unum==1
    """
    procs: List[ProcInfo] = []
    fps: List[LogFP] = []

    def _launch_one(team_idx: int, team: str, unum: int, mode: str):
        args = [
            player_exe,
            "-h", str(host),
            "-p", str(int(server_port)),
            "-t", str(team),
            "-n", str(int(unum)),
            "--player-config", "../player.conf",
            "--config_dir", "../formations-dt",
            "--debug_server_host", str(debug_host),
            "--debug_server_port", str(int(debug_port)),
            "--mode", str(mode),
            "--server_wait_seconds", "30"
        ]

        shm_name = player_shm_by_key.get((team_idx, int(unum)))
        if shm_name is not None:
            args += ["--shm-name", str(shm_name)]

        if int(unum) == 1:
            args.append("--goalie")

        if extra_args_common:
            args.extend(list(map(str, extra_args_common)))

        safe = _safe_team(team)
        log_name = f"{log_tag}player_{safe}_u{int(unum):02d}_p{int(server_port)}.log" if log_tag else \
                   f"player_{safe}_u{int(unum):02d}_p{int(server_port)}.log"

        p, fp = popen_logged(
            args,
            cwd=player_dir,
            env=env,
            log_path=_join_logs_dir(logs_dir, log_name),
        )

        procs.append(
            ProcInfo(
                p=p,
                kind="player",
                team=team,                      # 如 base_L 或 base_R
                unum=int(unum),                 # 1~11
                shm_name=shm_name or "",        # 对方可能没有 shm
                port=int(server_port),
                log_path=_join_logs_dir(logs_dir, log_name),
            )
        )
        fps.append(fp)


    mode1 = team1
    mode2 = team2
    if str(team1) == str(team2):
        team1 = f"{team1}_L"
        team2 = f"{team2}_R"

    # team1
    for unum in range(1, int(n1) + 1):
        _launch_one(1, team1, unum, mode1)

    # team2
    for unum in range(1, int(n2) + 1):
        _launch_one(2, team2, unum, mode2)
    return procs, fps
