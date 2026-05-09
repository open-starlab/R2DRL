# robocup2d/runtime.py

from __future__ import annotations

import os
import uuid
import time
import signal
from typing import List, Optional, Dict, Tuple
import socket
from . import process as proc
from .protocols import P
from . import ipc

class Runtime:
    """
    Responsible for:
      - run_id
      - port selection
      - shm id generation
      - process lifecycle
      - restart / close

    Does NOT:
      - create shm objects
      - handle obs/state
      - compute reward
    """

    def __init__(self, config, log, child_env):
        self.config = config
        self.log = log
        self.child_env = child_env

        # run identity
        self.run_id: Optional[str] = None
        self._lock_fd = None

        # ports
        self.base_port = None
        self.server_port = None
        self.trainer_port = None
        self.coach_port = None
        self.debug_port = None
        self._port_lock_fd = None
        self._port_lock_path = None

        # shm ids (Runtime owns them)
        self.coach_shm_id: Optional[str] = None
        self.trainer_shm_id: Optional[str] = None
        self.player_shm_ids: Optional[Dict[Tuple[int, int], str]] = None

        # processes
        self.procs: List = []

        # log dirs
        self.log_dir = None
        self.rcg_dir = None

    # ==========================================================
    # Run initialization
    # ==========================================================

    def initialize_session(self, where: str = "init") -> None:

        if self.run_id is not None:
            raise RuntimeError("Runtime already started")

        # run id
        self.run_id = uuid.uuid4().hex
        self._lock_fd = proc.acquire_run_lock(self.run_id, log=self.log)
        self.log.info(f"[{where}] run_id={self.run_id}")

        # ports
        (
            self.base_port,
            self.server_port,
            self.trainer_port,
            self.coach_port,
            self.debug_port,
            self._port_lock_fd,
            self._port_lock_path,
        ) = proc.pick_ports(
            self.config.auto_port_start,
            self.config.auto_port_end,
            self.config.auto_port_step,
            self.config.trainer_port_offset,
            self.config.coach_port_offset,
            self.config.debug_port_offset,
            timeout=self.config.ports_wait_timeout,
        )

        # log directories
        base_logs_dir = os.path.abspath(self.config.logs_dir)
        self.log_dir = os.path.join(base_logs_dir, self.run_id)
        self.rcg_dir = os.path.join(self.log_dir, "rcg")

        os.makedirs(self.log_dir, exist_ok=True)
        os.makedirs(self.rcg_dir, exist_ok=True)

        # --------------------------------------------------
        # Generate shm ids (Runtime owns them)
        # --------------------------------------------------
        (
            self.coach_shm_id,
            self.trainer_shm_id,
            self.player_shm_ids,
            _,
        ) = ipc.build_shm_layout(
            run_id=self.run_id,
            base_port=self.base_port,
            team1=self.config.team1,
            team2=self.config.team2,
            n1=self.config.n1,
            n2=self.config.n2,
        )

    # ==========================================================
    # Process control
    # ==========================================================

    def start_procs(self) -> None:

        if self.run_id is None:
            raise RuntimeError("initialize_session() must be called first")

        env = dict(self.child_env)
        env["ROBOCUP2DRL_RUN_ID"] = self.run_id

        self.procs = []

        # ---------- server ----------
        p, _ = proc.launch_server(
            server_path=self.config.server_path,
            server_port=self.server_port,
            trainer_port=self.trainer_port,
            coach_port=self.coach_port,
            logs_dir=self.log_dir,
            rcg_dir=self.rcg_dir,
            half_time=self.config.half_time,
            env=env,
            log_tag=f"{self.run_id}_",
            text_logging=self.config.text_logging,
            game_logging=self.config.game_logging,
        )
        self.procs.append(p)

        # ---------- players ----------
        player_procs, _ = proc.launch_players(
            player_dir=self.config.player_dir,
            player_exe=self.config.player_exe,
            host=self.config.host,
            server_port=self.server_port,
            player_config=self.config.player_config,
            config_dir=self.config.config_dir,
            debug_host=self.config.host,
            debug_port=self.debug_port,
            team1=self.config.team1,
            team2=self.config.team2,
            n1=self.config.n1,
            n2=self.config.n2,
            opponent_level=self.config.opponent_level,
            player_shm_by_key=self.player_shm_ids,
            logs_dir=self.log_dir,
            env=env,
            log_tag=f"{self.run_id}_",
            server_wait_seconds=self.config.server_wait_seconds,
        )
        self.procs.extend(player_procs)

        time.sleep(5)
        # ---------- trainer ----------
        p, _ = proc.launch_trainer(
            trainer_dir=self.config.trainer_dir,
            trainer_exe=self.config.trainer_exe,
            host=self.config.host,
            trainer_port=self.trainer_port,
            team1=(self.config.team1 if self.config.team1 != self.config.team2 else self.config.team1 + "_L"),
            team2=(self.config.team2 if self.config.team1 != self.config.team2 else self.config.team2 + "_R"),
            logs_dir=self.log_dir,
            trainer_shm_name=self.trainer_shm_id,
            env=env,
            log_tag=f"{self.run_id}_",
            server_wait_seconds=self.config.server_wait_seconds,
        )
        self.procs.append(p)


        # ---------- coach ----------
        p, _ = proc.launch_coach(
            coach_dir=self.config.coach_dir,
            coach_exe=self.config.coach_exe,
            host=self.config.host,
            coach_port=self.coach_port,
            coach_team=(self.config.team1 if self.config.team1 != self.config.team2 else self.config.team1 + "_L"),
            coach_shm_name=self.coach_shm_id,
            logs_dir=self.log_dir,
            env=env,
            log_tag=f"{self.run_id}_",
            server_wait_seconds=self.config.server_wait_seconds,
        )
        self.procs.append(p)

    # ==========================================================
    # Restart
    # ==========================================================

    def restart(self) -> None:
        self.kill_all()
        self.start_procs()

    def restart_session(self, where: str = "restart") -> None:
        old_run_id = self.run_id
        old_lock_fd = self._lock_fd
        old_names = []

        if self.coach_shm_id:
            old_names.append(self.coach_shm_id)
        if self.trainer_shm_id:
            old_names.append(self.trainer_shm_id)
        if self.player_shm_ids:
            old_names.extend(self.player_shm_ids.values())

        self.kill_all()

        for name in old_names:
            ipc.cleanup_shm(name, unlink=True, log=self.log)

        if old_lock_fd is not None and old_run_id is not None:
            P.common._safe(
                proc.release_run_lock,
                old_run_id,
                old_lock_fd,
                log=self.log,
            )

        self.run_id = uuid.uuid4().hex
        self._lock_fd = proc.acquire_run_lock(self.run_id, log=self.log)
        self.log.info(f"[{where}] run_id={self.run_id}")

        base_logs_dir = os.path.abspath(self.config.logs_dir)
        self.log_dir = os.path.join(base_logs_dir, self.run_id)
        self.rcg_dir = os.path.join(self.log_dir, "rcg")

        os.makedirs(self.log_dir, exist_ok=True)
        os.makedirs(self.rcg_dir, exist_ok=True)

        (
            self.coach_shm_id,
            self.trainer_shm_id,
            self.player_shm_ids,
            _,
        ) = ipc.build_shm_layout(
            run_id=self.run_id,
            base_port=self.base_port,
            team1=self.config.team1,
            team2=self.config.team2,
            n1=self.config.n1,
            n2=self.config.n2,
        )

    # ==========================================================
    # Health
    # ==========================================================

    def has_live_procs(self) -> bool:
        for item in self.procs:
            p = P.common._as_popen(item)
            if p is None:
                continue
            if P.common._safe(p.poll) is None:
                return True
        return False

    # ==========================================================
    # Kill / Close
    # ==========================================================

    def kill_all(self) -> None:

        popens = []
        for item in self.procs:
            p = P.common._as_popen(item)
            if p is not None:
                popens.append(p)

        py_pgid = os.getpgrp()
        run_pids, run_pgids = set(), set()

        for p in popens:
            if P.common._safe(p.poll) is None:
                pid = int(p.pid)
                run_pids.add(pid)
                pgid = P.common._safe(os.getpgid, pid)
                if pgid is not None and int(pgid) != py_pgid:
                    run_pgids.add(int(pgid))

        P.common._safe(proc.kill_run_process_groups,
                       signal.SIGTERM,
                       run_pgids,
                       run_pids,
                       log=self.log)

        t_end = time.time() + 2.0
        for p in popens:
            if p.poll() is None:
                P.common._safe(p.wait,
                               timeout=max(0.0, t_end - time.time()))

        P.common._safe(proc.kill_run_process_groups,
                       signal.SIGKILL,
                       run_pgids,
                       run_pids,
                       log=self.log)

        self.procs = []

    def close(self) -> None:

        self.kill_all()

        if self._lock_fd is not None and self.run_id is not None:
            P.common._safe(proc.release_run_lock,
                           self.run_id,
                           self._lock_fd,
                           log=self.log)
            self._lock_fd = None

        if self._port_lock_fd is not None:
            P.common._safe(os.close, self._port_lock_fd)
            self._port_lock_fd = None
            self._port_lock_path = None
