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
        self._closed = False            # Protection suggested in close()
        self._port_lock_fd = None
        self._port_lock_path = None
        self.pass_trainer = False
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
        self.n_agents = self.n1
        self.first_time=True
        
    def get_avail_actions(self):
        if self.done == 1 and self.last_avail_actions is not None:
            return self.last_avail_actions

        bufs = []
        for k in self._obs_keys:
            name = self.player_names[k]
            bufs.append((self.player_shms[name].buf, f"{k} {name}"))

        ipc.wait_all_ready_or_raise(
            bufs,
            off_a=P.player.OFFSET_FLAG_A,
            off_b=P.player.OFFSET_FLAG_B,
            timeout=self.wait_ready_timeout,
            poll=0.0005,
            log=self.log,
            tag="get_avail_actions barrier",
        )

        masks = []
        for k in self._obs_keys:
            name = self.player_names[k]
            buf = self.player_shms[name].buf

            # Base mask (17) or Hybrid mask (4)
            if self.team1 == "hybrid":
                m = np.frombuffer(buf, dtype=np.uint8, count=4, offset=P.player.OFFSET_HYBRID_MASK)
            else:
                m = np.frombuffer(buf, dtype=np.uint8, count=17, offset=P.player.OFFSET_MASK)

            masks.append(m.astype(np.int32, copy=False))

        out = np.stack(masks, axis=0)
        self.last_avail_actions = out
        return out

    def reset(self):
        # print("reset!!!")
        self.turn_count += 1
        print("Turn:", self.turn_count)
        if self.first_time:
            if self._closed:
                raise RuntimeError("env already closed; create a new env instance")

            # Critical: Clean up old processes for each attempt (keep shm/run_id)
            self._teardown_procs_only()
            self._clear_shm_all()

            self._start_procs_only(where="reset")
            # Actual reset logic
            self._reset_once_no_reconnect()
            self.first_time = False
        else:
            self._reset_once_no_reconnect()
            

    def _clear_shm_all(self, *, clear_players=True, clear_coach=True, clear_trainer=True) -> None:
        """
        Zero out all created/attached shm buffers.
        Note: Best called after old processes have stopped (e.g., after _teardown_procs_only in reset).
        """
        def _zero_one(shm_obj, tag: str):
            if shm_obj is None:
                return
            try:
                buf = shm_obj.buf  # memoryview
                # Do not allocate huge bytes; overwrite directly with numpy
                arr = np.frombuffer(buf, dtype=np.uint8)
                arr[:] = 0
            except Exception as e:
                self.log.warning(f"[shm_clear] failed: {tag} err={e!r}")

        if clear_coach:
            for name, shm in getattr(self, "coach_shms", {}).items():
                _zero_one(shm, f"coach:{name}")

        if clear_trainer:
            for name, shm in getattr(self, "trainer_shms", {}).items():
                _zero_one(shm, f"trainer:{name}")

        if clear_players:
            for name, shm in getattr(self, "player_shms", {}).items():
                _zero_one(shm, f"player:{name}")

        # Safety: Clear Python side cache (optional, but usually done together)
        self.begin_cycle = -1
        self.last_state = None
        self.last_obs = None
        self.last_avail_actions = None
        self.done = 0

    def _reset_once_no_reconnect(self):
        # 0) watchdog
        # print("# 0) watchdog")
        alive, dead, dead_info = proc.check_child_processes(self.procs, where="_reset_once_no_reconnect0")
        self.procs = alive
        if dead:
            raise RuntimeError("[watchdog] child process died:\n" + dead_info)

        # 1) Clear cache/flags
        self.last_state = None
        self.last_obs = None
        self.last_avail_actions = None
        self.done = 0
        self.begin_cycle = -1
        self.episode_steps = 0


        # 2) coach/trainer buf (trainer not used yet, can be kept)
        cbuf = self.coach_shms[self.coach_name].buf
        tbuf = self.trainer_shms[self.trainer_name].buf

        # 3) Wait for gm==PlayOn(2)
        # print("# 3) Wait for gm==PlayOn(2)")
        t_end = time.monotonic() + float(self.playon_timeout)
        while time.monotonic() < t_end:
            alive, dead, dead_info = proc.check_child_processes(self.procs, where="_reset_once_no_reconnect1")
            self.procs = alive
            if dead:
                raise RuntimeError("[watchdog] child process died:\n" + dead_info)


            gm = int(P.coach.read_gamemode(cbuf))
            cycle = int(P.coach.read_cycle(cbuf))

            if gm == 1:
                raise RuntimeError("[reset] gamemode=TimeOver(1)")

            if gm == 2:
                self.begin_cycle = cycle
                P.coach.clear_goal_flag(cbuf)
                break

            time.sleep(0.05)

        if self.begin_cycle < 0:
            raise TimeoutError("[reset] Timeout waiting for PlayOn")

        bufs = []
        for k in self._obs_keys:
            shm_name = self.player_names[k]
            shm = self.player_shms[shm_name]
            bufs.append((shm.buf, f"{k} {shm_name}"))
        # print("wait_all_ready_or_raise")
        ipc.wait_all_ready_or_raise(
            bufs,
            off_a=P.player.OFFSET_FLAG_A,
            off_b=P.player.OFFSET_FLAG_B,
            timeout=self.wait_ready_timeout,
            poll=0.0005,
            log=self.log,
            tag="reset first-frame barrier",
        )

        tflags=P.trainer.read_flags(tbuf)
        if not P.trainer.wait_flags(tbuf, P.common.FLAG_READY, timeout_ms=self.trainer_ready_timeout_ms, poll_us=500):
            raise P.common.ShmProtocolError(f"[trainer] not READY before submit, flags={tflags}")
        P.trainer.write_fixed_reset(tbuf)
        P.trainer.write_opcode(tbuf, 10)
        # 3) Flip flags -> REQ(1,0) Submit request
        ipc.write_flags(tbuf, P.trainer.T_FLAG_A, P.trainer.T_FLAG_B, P.common.FLAG_REQ)
        self.pass_trainer = True

        print("Reset Over!!!")
        return

    def _start_run(self, where: str = "init") -> None:
        if self._closed:
            raise RuntimeError("env already closed; create a new env instance")

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

    def _start_procs_only(self, where: str = "reset") -> None:
        """
        Responsibilities:
        - Teardown if old run exists
        - Generate run_id + lock
        - Select ports, create log/rcg directories
        - Generate shm names and create shm
        - Launch server/players/coach
        - Collect pgid/pid
        - Clear cache fields (last_state/obs/avail/done/begin_cycle)
        """
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
        time.sleep(0.2)

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
        # self.log.info(f"[{where}][trainer] pid={p.pid} port={int(self.trainer_port)} shm={self.trainer_name}")
        time.sleep(0.2)

        self._collect_run_pgids()

    def step(self, actions):
        """
        1) Watchdog: Raise error if child process dies
        2) Wait for PlayOn and all controlled agents READY
        3) Write actions + submit_req
        4) Wait for next frame all READY
        5) Read coach settlement: goal / timeover / timeout
        """
        # print("Step!!!")
        # 0) Watchdog
        self.episode_steps += 1
        alive, dead, dead_info = proc.check_child_processes(self.procs, where="step0")
        self.procs = alive
        if dead:
            raise RuntimeError("[watchdog] child process died:\n" + dead_info)

        # Step even if done=1: Keep old habit, return directly
        if self.done == 1:
            return 0.0, True, {"episode_limit": 0.0}

        # actions -> numpy (compatible with torch)
        if isinstance(actions, torch.Tensor):
            actions = actions.detach().cpu().numpy()
        actions = np.asarray(actions)
        # print("actions:", actions)

        # Determine Base/Hybrid (Hybrid is (n,3))
        is_hybrid = (actions.ndim == 2 and actions.shape[1] == 3)
        # print("is_hybrid:", is_hybrid)
        if is_hybrid:
            if not (actions.shape[0] == self.n_agents and actions.shape[1] == 3):
                raise RuntimeError(f"Hybrid expects actions.shape=(n_agents,3), received {actions.shape}")
        else:
            actions = actions.reshape(-1)
            if actions.shape[0] != self.n_agents:
                raise RuntimeError(f"Base expects {self.n_agents} actions, received {actions.shape[0]}")

        # coach buf
        coach_shm = self.coach_shms[self.coach_name]
        cbuf = coach_shm.buf

        # episode_limit = self.episode_limit

        # Prepare bufs in advance (for barrier)
        bufs = []
        for k in self._act_keys:
            shm_name = self.player_names[k]
            shm = self.player_shms[shm_name]
            bufs.append((shm.buf, f"{k} {shm_name}"))

        ready_to_act, cycle0, gm = ipc.wait_until_playon_or_done(
            cbuf=cbuf,
            begin_cycle=self.begin_cycle,
            episode_limit=self.episode_limit,
            goal_x=self.goal_x,
            goal_y=self.goal_y,
            max_stall_sec=self.playon_timeout,
            log=self.log,
            tag="step wait until playon or done",
        )
        # print("ready_to_act:", ready_to_act)

        if ready_to_act:
            ipc.wait_all_ready_or_raise(
                bufs,
                off_a=P.player.OFFSET_FLAG_A,
                off_b=P.player.OFFSET_FLAG_B,
                timeout=self.wait_ready_timeout,
                poll=0.0005,
                log=self.log,
                tag="step pre-ready barrier",
            )
        if ready_to_act and not self.pass_trainer:    
            trainer_shm = self.trainer_shms[self.trainer_name]   # 按你项目里的实际字段名
            tbuf = trainer_shm.buf
            tflags=P.trainer.read_flags(tbuf)

            if not P.trainer.wait_flags(tbuf, P.common.FLAG_READY, timeout_ms=self.trainer_ready_timeout_ms, poll_us=500):
                raise P.common.ShmProtocolError(f"[trainer] not READY before submit, flags={tflags}")
            # P.trainer.write_fixed_reset(tbuf)
            P.trainer.write_opcode(tbuf, 8)
            # 3) Flip flags -> REQ(1,0) Submit request
            ipc.write_flags(tbuf, P.trainer.T_FLAG_A, P.trainer.T_FLAG_B, P.common.FLAG_REQ)
            
        self.pass_trainer = False

        # 2) Write actions (only write at PlayOn sync point)
        if ready_to_act and self.n_agents > 0:
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
            # print("submitted all actions")
            ready_to_act, cycle, gm = ipc.wait_until_playon_or_done(
                cbuf=cbuf,
                begin_cycle=self.begin_cycle,
                episode_limit=self.episode_limit,
                goal_x=self.goal_x,
                goal_y=self.goal_y,
                max_stall_sec=self.playon_timeout,
                log=self.log,
                tag="step wait until playon or done",
                current_coach_cycle=cycle0
            )

            alive, dead, dead_info = proc.check_child_processes(self.procs, where="step1")
            self.procs = alive
            if dead:
                raise RuntimeError("[watchdog] child process died:\n" + dead_info)
        
            # print("after watchdog check")
            if ready_to_act:
                # 3) After writing actions: Wait for next frame all READY
                ipc.wait_all_ready_or_raise(
                    bufs,
                    off_a=P.player.OFFSET_FLAG_A,
                    off_b=P.player.OFFSET_FLAG_B,
                    timeout=self.wait_ready_timeout,
                    poll=0.0005,
                    log=self.log,
                    tag="step post-ready barrier",
                )
            else:
                print("not ready to act")

        # 4) Settlement (read coach)
        cycle = int(P.coach.read_cycle(cbuf))
        gm = int(P.coach.read_gamemode(cbuf))
        # ball = P.coach.read_ball(cbuf, copy=True)
        goal = P.coach.read_goal_flag(cbuf)
        timeout = (self.episode_steps >= self.episode_limit)
        # scored = (abs(float(ball[0])) >= self.goal_x) and (abs(float(ball[1])) <= self.goal_y)

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
        """
        Returns obs: np.ndarray shape = (n_agents, 97)

        Logic:
        0) Watchdog: Raise error if child process dies (avoid getting stuck in wait)
        1) Barrier: wait_ready_or_raise for all players one by one (ensure all READY)
        2) Read all obs at once (second round read, reduce frame mixing probability)
        """
        # done cache
        if self.done == 1 and self.last_obs is not None:
            return self.last_obs

        # 0) Watchdog: Check if child processes are alive
        alive, dead, dead_info = proc.check_child_processes(self.procs, where="get_obs")
        self.procs = alive
        if dead:
            raise RuntimeError("[watchdog] child process died:\n" + dead_info)

        # 1) Barrier: Ensure "all players" are READY first
        # Collect all shm bufs needed for barrier at once
        bufs = []
        for k in self._obs_keys:
            shm_name = self.player_names[k]
            shm = self.player_shms[shm_name]
            bufs.append((shm.buf, f"{k} {shm_name}"))

        # Poll all until all READY (or raise error on total timeout)
        ipc.wait_all_ready_or_raise(
            bufs,
            off_a=P.player.OFFSET_FLAG_A,
            off_b=P.player.OFFSET_FLAG_B,
            timeout=self.wait_ready_timeout,
            poll=0.0005,
            log=self.log,
            tag="get_obs barrier",
        )

        # 2) Read obs at once (second round read)
        obs_list = []
        for k in self._obs_keys:
            shm_name = self.player_names[k]
            shm = self.player_shms[shm_name]
            o = P.player.read_obs_norm(buf=shm.buf,field_length=self.half_length, field_width=self.half_width)  # float32[97]
            obs_list.append(o)

        obs = np.stack(obs_list, axis=0).astype(np.float32, copy=False)
        self.last_obs = obs
        return obs

    def get_state(self):
        """
        Read global state from coach shm:
        state = ball(4) + players(22*6) => (136,)
        No stable-read (no cycle check), read once and return directly.
        """

        # 0) Watchdog: Check if child processes are alive
        alive, dead, dead_info = proc.check_child_processes(self.procs, where="get_state")
        self.procs = alive
        if dead:
            raise RuntimeError("[watchdog] child process died:\n" + dead_info)

        # 1) done cache
        if self.done == 1 and self.last_state is not None:
            return self.last_state

        # 2) Read coach shm
        shm = self.coach_shms[self.coach_name]
        buf = shm.buf

        state = P.coach.read_state_norm(
            buf,
            field_length=self.half_length,
            field_width=self.half_width,
            copy=True,
        )
        self.last_state = state
        return state

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._teardown_run()

    def __del__(self):
        # Do not raise exceptions during destruction
        try:
            self.close()
        except Exception:
            pass

    def register_shm(self, name: str) -> None:
        """Call immediately after create_shm() succeeds to ensure close() can clean up."""
        if not name.startswith("/"):
            name = "/" + name
        self._shm_names.append(name)

    def get_env_info(self):
        """
        Return env_info for PyMARL.
        Action mode is determined entirely by team1 name:
          - team1 starts with "base" -> use base actions (17)
          - team1 starts with "hybrid" -> use hybrid actions (e.g., 4, define yourself)
        """
        team_name = self.team1

        # 1) Determine action count based on team1
        if team_name.startswith("base"):
            n_actions = P.player.BASE_MASK_NUM        # 17
        elif team_name.startswith("hybrid"):
            n_actions = 4                             # Your current hybrid action count, change to real one
        else:
            raise ValueError(f"Unknown team1='{self.team1}', cannot infer action mode")

        # 2) Assemble env_info
        return {
            "n_agents": int(self.n1),                 # Only control the number of players in team1
            "n_actions": int(n_actions),
            "state_shape": int(P.coach.COACH_STATE_FLOAT),  # 136
            "obs_shape": int(P.player.STATE_NUM),           # 97
            "episode_limit": int(self.episode_limit),
        }
    def _teardown_procs_only(self) -> None:
        # 0) Resample pgid/pid
        try:
            self._collect_run_pgids()
        except Exception:
            pass

        def _as_popen(x):
            return x.p if hasattr(x, "p") else x

        popens = []
        for item in list(getattr(self, "procs", [])):
            p = _as_popen(item)
            if p is not None:
                popens.append(p)

        # 1) TERM
        try:
            self._kill_run_process_groups(signal.SIGTERM)
        except Exception:
            pass

        # 2) wait
        t_end = time.time() + 2.0
        for p in popens:
            try:
                if p.poll() is None:
                    p.wait(timeout=max(0.0, t_end - time.time()))
            except Exception:
                pass

        # 3) KILL
        try:
            self._kill_run_process_groups(signal.SIGKILL)
        except Exception:
            pass

        # 5) Only clear "runtime cache", do not clear shm/ports/run_id
        self.procs = []
        self.begin_cycle = -1
        self.last_state = None
        self.last_obs = None
        self.last_avail_actions = None
        self.done = 0
        
        # At the end of _teardown_procs_only()
        ports = [self.server_port, self.coach_port, self.trainer_port, self.debug_port]
        ports = [p for p in ports if p is not None]
        if ports:
            ok = proc.wait_ports_free(ports, timeout=self.ports_wait_timeout, poll=0.05, hold=0.3)
            if not ok:
                self.log.warning(f"[teardown] ports still busy after wait: {ports}")

    def _teardown_run(self) -> None:
        # 0) Resample once (prevent _run_pgids/_run_pids from being empty or expired)
        try:
            self._collect_run_pgids()
        except Exception:
            pass

        def _as_popen(x):
            return x.p if hasattr(x, "p") else x
        popens = []
        for item in list(getattr(self, "procs", [])):
            p = _as_popen(item)
            if p is not None:
                popens.append(p)

        # 1) TERM
        try:
            self._kill_run_process_groups(signal.SIGTERM)
        except Exception:
            pass

        # 2) wait
        t_end = time.time() + 2.0
        for p in popens:
            try:
                if p.poll() is None:
                    p.wait(timeout=max(0.0, t_end - time.time()))
            except Exception:
                pass

        # 3) KILL
        try:
            self._kill_run_process_groups(signal.SIGKILL)
        except Exception:
            pass

        # 4) (optional) port-based cleanup: only kill processes with same run_id
        try:
            if getattr(self, "run_id", None) is not None:
                ports = []
                for p in (self.server_port, self.trainer_port, self.coach_port, self.debug_port):
                    if p is not None:
                        ports.append(int(p))
                for port in ports:
                    proc.kill_port_by_run_id(port, run_id=self.run_id, log=self.log)
        except Exception:
            pass

        # 5) shm close/unlink
        def _destroy_owner_dict(d):
            for name, shm in list(d.items()):
                try: 
                    shm.close()
                except Exception: 
                    pass
                try: 
                    shm.unlink()
                except Exception: 
                    pass
            d.clear()

        try: 
            _destroy_owner_dict(getattr(self, "coach_shms", {}))
        except Exception: 
            pass
        try: 
            _destroy_owner_dict(getattr(self, "trainer_shms", {}))
        except Exception: 
            pass
        try: 
            _destroy_owner_dict(getattr(self, "player_shms", {}))
        except Exception: 
            pass

        for name in list(getattr(self, "_shm_names", [])):
            try:
                if not str(name).startswith("/"):
                    name = "/" + str(name)
                ipc.cleanup_shm(name, unlink=True, log=self.log)
            except Exception:
                pass

        # 6) release run lock
        if getattr(self, "_lock_fd", None) is not None and getattr(self, "run_id", None) is not None:
            try:
                proc.release_run_lock(self.run_id, self._lock_fd, log=self.log)
            except Exception:
                pass
            self._lock_fd = None

        # 7) clear
        self.procs = []
        self.coach_shms = {}
        self.trainer_shms = {}
        self.player_shms = {}
        self._shm_names = []
        self.coach_name = None
        self.trainer_name = None
        self.player_names = {}
        self.log_dir = None
        self.rcg_dir = None
        self.base_port = None
        self.server_port = None
        self.trainer_port = None
        self.coach_port = None
        self.debug_port = None
        self.run_id = None
        self.begin_cycle = -1
        self.last_state = None
        self.last_obs = None
        self.last_avail_actions = None
        self.done = 0
        self._run_pgids = set()
        self._run_pids = set()

        if self._port_lock_fd is not None:
            try: 
                os.close(self._port_lock_fd)
            except Exception: 
                pass
            self._port_lock_fd = None
            self._port_lock_path = None

    def _collect_run_pgids(self) -> None:
        def _as_popen(x):
            return x.p if hasattr(x, "p") else x
        self._run_pgids = set()
        self._run_pids = set()
        py_pgid = os.getpgrp()

        for item in list(getattr(self, "procs", [])):
            p = _as_popen(item)
            if p is None:
                continue
            if p.poll() is None:
                pid = int(p.pid)
                self._run_pids.add(pid)
                pgid = os.getpgid(pid)
                if pgid != py_pgid:
                    self._run_pgids.add(pgid)


        self.log.info(f"[run] python_pgid={py_pgid} run_pgids={sorted(self._run_pgids)} run_pids={sorted(self._run_pids)[:8]}{'...' if len(self._run_pids)>8 else ''}")

    def _kill_run_process_groups(self, sig: int) -> None:
        py_pgid = os.getpgrp()
        pgids = set(getattr(self, "_run_pgids", set()))
        pids  = set(getattr(self, "_run_pids", set()))

        for pgid in sorted(pgids):
            if pgid == py_pgid:
                # self.log.warning(f"[teardown] skip killpg(pgid={pgid}) because it equals python pgid")
                continue

            # ✅ Critical: Confirm that "alive pids of this run" still exist in this pgid
            alive_member = False
            for pid in list(pids):
                try:
                    if os.getpgid(pid) == pgid:
                        alive_member = True
                        break
                except ProcessLookupError:
                    continue
                except Exception:
                    continue

            if not alive_member:
                # pgid may be empty/reused; do not kill
                # self.log.info(f"[teardown] skip killpg pgid={pgid} sig={sig} (no alive member pid in this run)")
                continue

            try:
                os.killpg(pgid, sig)
                # self.log.info(f"[teardown] killpg pgid={pgid} sig={sig}")
            except ProcessLookupError:
                pass
            except Exception as e:
                self.log.warning(f"[teardown] killpg failed pgid={pgid} sig={sig} err={e}")
