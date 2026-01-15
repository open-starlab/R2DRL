# robocup2d/env.py
from __future__ import annotations
import torch
import uuid
from .protocols import P
from . import process as proc
from . import ipc as ipc
from .logging_utils import get_env_logger
from .config import load_env_args
import os
import time
import numpy as np
import signal


class Robocup2dEnv:
    """PyMARL-friendly env wrapper."""

    def __init__(self, cfg="robocup.yaml", **env_args):
        """
        Create shared memory and child processes.
        """
        
        # Setup main program output
        self.log = get_env_logger("robocup_env")
        self.log.info("[Robocup2dEnv] init scaffold OK")

        # Load yaml configuration
        self.cfg = cfg
        self.env_args = env_args
        self.args = load_env_args(cfg, env_args)        

        self.n1 = int(self.args["n1"])
        self.n2 = int(self.args["n2"])

        self.team1 = self.args["team1"].lower()
        if self.team1 == "base":
            self.n_actions = int(P.player.BASE_MASK_NUM)   # 17
        elif self.team1 == "hybrid":
            self.n_actions = 4
        else:
            raise ValueError(f"Unknown team='{self.team1}', cannot infer action mode")
        
        self.team2 = self.args["team2"]

        self.episode_limit = int(self.args["episode_limit"])
        self.goal_x = float(self.args["goal_x"])
        self.goal_y = float(self.args["goal_y"])
        self.half_length = float(self.args["HALF_LENGTH"])
        self.half_width = float(self.args["HALF_WIDTH"])

        self.wait_ready_timeout = float(self.args["wait_ready_timeout"])
        self.playon_timeout = float(self.args["playon_timeout"])
        self.trainer_ready_timeout_ms = int(self.args["trainer_ready_timeout_ms"])
        self.ports_wait_timeout = float(self.args["ports_wait_timeout"])
        self.server_wait_seconds = float(self.args["server_wait_seconds"])

        self._lock_fd = None
        self.log_dir = None
        self.rcg_dir = None
        self.base_port = None
        self.server_port = None
        self.trainer_port = None
        self.coach_port = None
        self.debug_port = None
        self.coach_name = None
        self.trainer_name = None
        self.player_names = {}
        self._shm_names = []
        self.coach_shms = {}
        self.trainer_shms = {}
        self.player_shms = {}
        self.run_id = None

        self.begin_cycle = -1
        self.procs = []
        self.last_state = None
        self.done = 0
        self.last_obs = None
        self.last_avail_actions = None  # Likely to be used later
        self._port_lock_fd = None
        self._port_lock_path = None
        self.episode_steps = 0
        self.turn_count = 0

        # Child process environment variables (configure LD_LIBRARY_PATH)
        self.child_env = os.environ.copy()
        lib_paths = self.args.get("lib_paths", [])
        base_ld = os.environ.get("LD_LIBRARY_PATH", "")

        if lib_paths:
            merged = ":".join(map(str, lib_paths))
            if base_ld:
                merged = merged + ":" + base_ld
            self.child_env["LD_LIBRARY_PATH"] = merged
        else:
            # If not configured in yaml, use the external one
            self.child_env["LD_LIBRARY_PATH"] = base_ld

        self.log.info(f"[child_env] LD_LIBRARY_PATH={self.child_env.get('LD_LIBRARY_PATH', '')}")
        self._start_run(where="init")

        self._act_keys = sorted(self.player_names.keys())
        self._obs_keys = sorted(self.player_names.keys())

        #bufs
        self.player_bufs = []
        for k in self._obs_keys:
            name = self.player_names[k]
            self.player_bufs.append((self.player_shms[name].buf, f"{k} {name}"))

        if self.team1 == "hybrid":
            self._mask_len = 4
            self._mask_off = P.player.OFFSET_HYBRID_MASK
        else:
            self._mask_len = 17
            self._mask_off = P.player.OFFSET_MASK

        self._mask_views = [
            np.frombuffer(buf_mv, dtype=np.uint8, count=self._mask_len, offset=self._mask_off)
            for (buf_mv, _tag) in self.player_bufs
        ]

        self._avail_out = np.empty((len(self._obs_keys), self._mask_len), dtype=np.int32)

        self.cbuf = self.coach_shms[self.coach_name].buf
        self.tbuf = self.trainer_shms[self.trainer_name].buf
        self._obs_bufs = [buf_mv for (buf_mv, _tag) in self.player_bufs]
        self._obs_out = np.empty((len(self._obs_bufs), P.player.STATE_NUM), dtype=np.float32)
        self._closed = False
        self.skip_trainer = False

    def get_avail_actions(self):
        if self.done == 1 and self.last_avail_actions is not None:
            return self.last_avail_actions

        for i, v in enumerate(self._mask_views):
            self._avail_out[i, :] = v

        self.last_avail_actions = self._avail_out
        return self._avail_out.copy()

    def reset(self):
        self.turn_count += 1
        print("Turn:", self.turn_count)

        if not self.procs:             # 第一次（或你 teardown 过）才会进来
            self.start_procs(where="reset")

        self._reset_once_no_reconnect()


    def _reset_once_no_reconnect(self):

        # 1) Clear cache/flags
        self.last_state = None
        self.last_obs = None
        self.last_avail_actions = None
        self.done = 0
        self.begin_cycle = -1
        self.episode_steps = 0

        ipc.wait_all_ready (
            player_bufs=self.player_bufs,
            off_a=P.player.OFFSET_FLAG_A,
            off_b=P.player.OFFSET_FLAG_B,
            log=self.log,
            tbuf=self.tbuf,
        )

        tflags=P.trainer.read_flags(self.tbuf)
        if not P.trainer.wait_flags(self.tbuf, P.common.FLAG_READY, timeout_ms=self.trainer_ready_timeout_ms, poll_us=500):
            raise P.common.ShmProtocolError(f"[trainer] not READY before submit, flags={tflags}")
        P.trainer.write_fixed_reset(self.tbuf)
        P.trainer.write_opcode(self.tbuf, 10)
        ipc.write_flags(self.tbuf, P.trainer.T_FLAG_A, P.trainer.T_FLAG_B, P.common.FLAG_REQ)
        self.skip_trainer = True

        P.coach.clear_goal_flag(self.cbuf)

        print("Reset Over!!!")
        return

    def _start_run(self, where: str = "init") -> None:

        # Start function is not responsible for teardown
        if self.run_id is not None or self.procs or self._shm_names:
            raise RuntimeError("run already started; call _reconnect_run or _teardown_run first")

        # run lock
        self.run_id = uuid.uuid4().hex
        self._lock_fd = proc.acquire_run_lock(self.run_id, log=self.log)
        self.log.info(f"[{where}] run_id={self.run_id}")

        # Select ports
        (self.base_port, self.server_port, self.trainer_port, self.coach_port, self.debug_port,
        self._port_lock_fd, self._port_lock_path) = proc.pick_ports(
            self.args["auto_port_start"], self.args["auto_port_end"],
            self.args["auto_port_step"], self.args["trainer_port_offset"],
            self.args["coach_port_offset"], self.args["debug_port_offset"],
        )

        # log/rcg directories
        base_logs_dir = os.path.abspath(str(self.args["logs_dir"]))
        self.log_dir = os.path.join(base_logs_dir, self.run_id)
        os.makedirs(self.log_dir, exist_ok=True)
        self.rcg_dir = os.path.join(self.log_dir, "rcg")
        os.makedirs(self.rcg_dir, exist_ok=True)

        # Clear runtime fields
        self.begin_cycle = -1
        self.last_state = None
        self.last_obs = None
        self.last_avail_actions = None
        self.done = 0

        # shm plan + create
        self.coach_name, self.trainer_name, self.player_names, self._shm_names = ipc.make_shm_plan(
            run_id=self.run_id,
            base_port=int(self.base_port),
            team1=self.team1,
            team2=self.team2,
            n1=int(self.n1),
            n2=int(self.n2),
            prefix="robocup2drl_",
        )

        self.coach_shms, self.trainer_shms, self.player_shms = ipc.create_shm_many_3lists(
            coach_names=[self.coach_name],
            trainer_names=[self.trainer_name],
            player_names=list(self.player_names.values()),
            coach_size=P.coach.COACH_SHM_SIZE,
            trainer_size=P.trainer.TRAINER_SHM_SIZE,
            player_size=P.player.PLAYER_SHM_SIZE,
            zero_fill=True,
            log=self.log,
        )

    def start_procs(self, where: str = "reset") -> None:

        env = dict(self.child_env)
        env["ROBOCUP2DRL_RUN_ID"] = self.run_id

        # Clear runtime fields
        self.begin_cycle = -1
        self.procs = []
        self.last_state = None
        self.last_obs = None
        self.last_avail_actions = None
        self.done = 0

        # server
        p, _ = proc.launch_server(
            server_path=str(self.args["server_path"]),
            server_port=int(self.server_port),
            trainer_port=int(self.trainer_port),
            coach_port=int(self.coach_port),
            logs_dir=self.log_dir,
            rcg_dir=self.rcg_dir,
            half_time=int(self.args["half_time"]),
            env=env,
            log_tag=f"{self.run_id}_",
        )
        self.procs.append(p)
        self.log.info(f"[{where}][server] pid={p.pid} port={int(self.server_port)} pgid={os.getpgid(p.pid)}")
        time.sleep(1.0)

        # trainer
        p, _ = proc.launch_trainer(
            trainer_dir=str(self.args["trainer_dir"]),
            trainer_exe=str(self.args["trainer_exe"]),
            host=str(self.args["host"]),
            trainer_port=int(self.trainer_port),
            team1=self.team1,
            team2=self.team2,
            logs_dir=self.log_dir,
            trainer_shm_name=str(self.trainer_name),
            env=env,
            log_tag=f"{self.run_id}_",
            server_wait_seconds=self.server_wait_seconds,
        )
        self.procs.append(p)

        # players
        player_procs, _ = proc.launch_players(
            player_dir=str(self.args["player_dir"]),
            player_exe=str(self.args["player_exe"]),
            host=str(self.args["host"]),
            server_port=int(self.server_port),
            player_config=str(self.args["player_config"]),
            config_dir=str(self.args["config_dir"]),
            debug_host=str(self.args["host"]),
            debug_port=int(self.debug_port),
            team1=self.team1,
            team2=self.team2,
            n1=int(self.n1),
            n2=int(self.n2),
            player_shm_by_key=dict(self.player_names),
            logs_dir=self.log_dir,
            env=env,
            log_tag=f"{self.run_id}_",
            server_wait_seconds=self.server_wait_seconds,
        )
        self.procs.extend(player_procs)
        self.log.info(f"[{where}][players] launched n={len(player_procs)}")
        # time.sleep(10)

        ipc.wait_all_ready (
            player_bufs=self.player_bufs,
            off_a=P.player.OFFSET_FLAG_A,
            off_b=P.player.OFFSET_FLAG_B,
            log=self.log,
            tbuf=self.tbuf,
        )

        # coach
        p, _ = proc.launch_coach(
            coach_dir=str(self.args["coach_dir"]),
            coach_exe=str(self.args["coach_exe"]),
            host=str(self.args["host"]),
            coach_port=int(self.coach_port),
            coach_team=str(self.team1),
            coach_shm_name=str(self.coach_name),
            logs_dir=self.log_dir,
            env=env,
            log_tag=f"{self.run_id}_",
            server_wait_seconds=self.server_wait_seconds,
        )
        self.procs.append(p)
        self.log.info(f"[{where}][coach] pid={p.pid} port={int(self.coach_port)} shm={self.coach_name}")
        time.sleep(0.2)

        print("start_procs")

    def step(self, actions):

        self.episode_steps += 1

        if isinstance(actions, torch.Tensor):
            actions = actions.detach().cpu().numpy()
        actions = np.asarray(actions)

        is_hybrid = (actions.ndim == 2 and actions.shape[1] == 3)
        if is_hybrid:
            if not (actions.shape[0] == self.n1 and actions.shape[1] == 3):
                raise RuntimeError(f"Hybrid expects actions.shape=(n_agents,3), received {actions.shape}")
        else:
            actions = actions.reshape(-1)
            if actions.shape[0] != self.n1:
                raise RuntimeError(f"Base expects {self.n1} actions, received {actions.shape[0]}")

        current_coach_cycle = int(P.coach.read_cycle(self.cbuf))

        #trainer
        if self.skip_trainer is False:
            tflags=P.trainer.read_flags(self.tbuf)
            if not P.trainer.wait_flags(self.tbuf, P.common.FLAG_READY, timeout_ms=self.trainer_ready_timeout_ms, poll_us=500):
                raise P.common.ShmProtocolError(f"[trainer] not READY before submit, flags={tflags}")
            P.trainer.write_opcode(self.tbuf, 8)
            ipc.write_flags(self.tbuf, P.trainer.T_FLAG_A, P.trainer.T_FLAG_B, P.common.FLAG_REQ)
        self.skip_trainer = False

        # 2) Write actions (only write at PlayOn sync point)
        for idx, k in enumerate(self._act_keys):
            shm_name = self.player_names[k]
            shm = self.player_shms[shm_name]
            buf = shm.buf

            if is_hybrid:
                a, u0, u1 = actions[idx]
                P.player.write_hybrid_action(buf, int(a), float(u0), float(u1), clamp=True)
            else:
                P.player.write_action(buf, int(actions[idx]))
                # print("wrote action:", int(actions[idx]))
            ipc.write_flags(buf,P.player.OFFSET_FLAG_A,P.player.OFFSET_FLAG_B,P.common.FLAG_REQ)

        cycle, gm, goal = ipc.wait_all_ready_with_rescue(
            player_bufs=self.player_bufs,
            off_a=P.player.OFFSET_FLAG_A,
            off_b=P.player.OFFSET_FLAG_B,
            cbuf=self.cbuf,
            tbuf=self.tbuf,
            current_cycle=current_coach_cycle,
            log=self.log,
            is_hybrid=is_hybrid,
        )


        # 4) Settlement (read coach)

        timeout = (self.episode_steps >= self.episode_limit)

        reward = 0.0
        self.done = 0

        if timeout:
            self.done = 1
            reward = 0.0
        elif goal==1:
            self.done = 1
            reward = 1.0
        elif goal==-1:
            self.done = 1
            reward = -1.0
        else:
            self.done = 0
            reward = 0.0        

        info = {
            "episode_limit": float(timeout),
            "cycle": int(cycle),
            "gamemode": int(gm),
        }
        # print("done:", self.done, " reward:", reward, " info:", info,"turn_cycle:", self.episode_steps, "gm:", gm, "goal:", goal)
        return float(reward), bool(self.done), info

    def get_obs(self):
        if self.done == 1 and self.last_obs is not None:
            return self.last_obs

        for i, buf in enumerate(self._obs_bufs):
            self._obs_out[i, :] = P.player.read_obs_norm(
                buf=buf,
                field_length=self.half_length,
                field_width=self.half_width,
            )

        self.last_obs = self._obs_out
        return self._obs_out.copy()


    def get_state(self):

        # 1) done cache
        if self.done == 1 and self.last_state is not None:
            return self.last_state

        # 2) Read coach shm

        state = P.coach.read_state_norm(
            buf=self.cbuf,
            field_length=self.half_length,
            field_width=self.half_width,
            copy=True,
        )
        self.last_state = state
        return state

    def __del__(self):
        # Do not raise exceptions during destruction
        try:
            self.close()
        except Exception:
            pass

    def get_env_info(self):

        return {
            "n_agents": int(self.n1),                 # Only control the number of players in team1
            "n_actions": int(self.n_actions),
            "state_shape": int(P.coach.COACH_STATE_FLOAT),  # 136
            "obs_shape": int(P.player.STATE_NUM),           # 97
            "episode_limit": int(self.episode_limit),
        }
    
    def close(self) -> None:
        if getattr(self, "_closed", False):
            return
        self._closed = True
        self._mask_views = []
        self.player_bufs = []
        self._obs_bufs = []
        self.cbuf = None
        self.tbuf = None
        self._avail_out = None
        self._obs_out = None
        self.last_obs = None
        self.last_state = None
        self.last_avail_actions = None

        popens = []
        for item in self.procs:
            p = P.common._as_popen(item)
            if p is not None:
                popens.append(p)

        py_pgid = os.getpgrp()
        run_pids, run_pgids = set(), set()
        for p in popens:
            st = P.common._safe(p.poll)
            if st is None:
                pid = int(p.pid)
                run_pids.add(pid)
                pgid = P.common._safe(os.getpgid, pid)
                if pgid is not None and int(pgid) != py_pgid:
                    run_pgids.add(int(pgid))

        P.common._safe(proc.kill_run_process_groups, signal.SIGTERM, run_pgids, run_pids, log=self.log)

        t_end = time.time() + 2.0
        for p in popens:
            if p.poll() is None:
                P.common._safe(p.wait, timeout=max(0.0, t_end - time.time()))

        P.common._safe(proc.kill_run_process_groups, signal.SIGKILL, run_pgids, run_pids, log=self.log)

        P.common._safe(P.common._close_unlink, self.coach_shms)
        P.common._safe(P.common._close_unlink, self.trainer_shms)
        P.common._safe(P.common._close_unlink, self.player_shms)

        if self._lock_fd is not None and self.run_id is not None:
            P.common._safe(proc.release_run_lock, self.run_id, self._lock_fd, log=self.log)
            self._lock_fd = None

        if self._port_lock_fd is not None:
            P.common._safe(os.close, self._port_lock_fd)
            self._port_lock_fd = None
            self._port_lock_path = None