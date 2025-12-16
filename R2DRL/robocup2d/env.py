# robocup2d/env.py
from __future__ import annotations
import torch
import uuid
from typing import List, Optional
from .protocols import P
from . import process as proc
from . import ipc as ipc
from .logging_utils import setup_line_buffering, get_env_logger
from .config import load_env_args
import os
import time
import numpy as np
import signal


class Robocup2dEnv:
    """PyMARL-friendly env wrapper (scaffold)."""

    def __init__(self, cfg="robocup.yaml", **env_args):
        """
        创建shm
        创建子进程
        """
        
        #主程序输出
        setup_line_buffering()
        self.log = get_env_logger("robocup_env")
        self.log.info("[Robocup2dEnv] init scaffold OK")

        # 读取yaml配置
        self.cfg = cfg
        self.env_args = env_args
        self.args = load_env_args(cfg, env_args)        

        self.n1 = int(self.args["n1"])
        self.n2 = int(self.args["n2"])
        self.team1 = self.args["team1"].lower()
        self.team2 = self.args["team2"]


        self.episode_limit=int(self.args["episode_limit"])
        self.goal_x = float(self.args["goal_x"])
        self.goal_y = float(self.args["goal_y"])
        self.half_length = float(self.args["HALF_LENGTH"])
        self.half_width = float(self.args["HALF_WIDTH"])

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
        self.last_avail_actions = None  # 你后面大概率也会用
        self._closed = False            # close() 里也建议加保护
        self.t0=time.time()

        # 子进程环境变量（把 LD_LIBRARY_PATH 配好）
        self.child_env = os.environ.copy()
        lib_paths = self.args.get("lib_paths", [])
        base_ld = os.environ.get("LD_LIBRARY_PATH", "")

        if lib_paths:
            merged = ":".join(map(str, lib_paths))
            if base_ld:
                merged = merged + ":" + base_ld
            self.child_env["LD_LIBRARY_PATH"] = merged
        else:
            # 如果 yaml 里没配，就沿用外面的
            self.child_env["LD_LIBRARY_PATH"] = base_ld

        self.log.info(f"[child_env] LD_LIBRARY_PATH={self.child_env.get('LD_LIBRARY_PATH', '')}")

        
    def get_avail_actions(self):
        if self.done == 1 and self.last_avail_actions is not None:
            return self.last_avail_actions

        # 先确保所有球员 READY（和 get_obs 一样）
        keys = getattr(self, "_obs_keys", None)
        if keys is None:
            keys = sorted(self.player_names.keys())
            self._obs_keys = keys

        bufs = []
        for k in keys:
            name = self.player_names[k]
            bufs.append((self.player_shms[name].buf, f"{k} {name}"))

        ipc.wait_all_ready_or_raise(
            bufs,
            off_a=P.player.OFFSET_FLAG_A,
            off_b=P.player.OFFSET_FLAG_B,
            timeout=20,
            poll=0.0005,
            log=self.log,
            tag="get_avail_actions barrier",
        )

        masks = []
        for k in keys:
            name = self.player_names[k]
            buf = self.player_shms[name].buf

            # Base mask (17) or Hybrid mask (4)
            if getattr(self, "action_mode", "base") == "hybrid":
                m = np.frombuffer(buf, dtype=np.uint8, count=4, offset=P.player.OFFSET_HYBRID_MASK)
            else:
                m = np.frombuffer(buf, dtype=np.uint8, count=17, offset=P.player.OFFSET_MASK)

            masks.append(m.astype(np.int32, copy=False))

        out = np.stack(masks, axis=0)
        self.last_avail_actions = out
        return out

    # ----------------- API placeholders -----------------
    def reset(self):
        if self._closed:
            raise RuntimeError("env already closed; create a new env instance")

        max_retries = int(self.args.get("reset_retries", 5))  # 没配就默认 2 次
        last_err = None

        for attempt in range(max_retries + 1):
            try:
                # 每次 attempt 都重拉一套（最稳）
                self._reconnect_run(where=f"reset attempt={attempt}")

                # 真正 reset 逻辑
                self._reset_once_no_reconnect()
                return

            except Exception as e:
                last_err = e
                self.log.warning(f"[reset] attempt={attempt} failed: {e!r}")

                # 失败就彻底拆掉，准备下一次重连
                try:
                    self._teardown_run()
                except Exception:
                    pass

                # 小睡一下，避免端口/进程组/内核回收抖动（可选）
                time.sleep(0.2)

        # 所有重试都失败
        raise RuntimeError(f"[reset] failed after {max_retries+1} attempts. last_err={last_err!r}")

    
    def _reset_once_no_reconnect(self):
        # 0) watchdog
        alive, dead = proc.check_child_processes(self.procs, where="reset")
        self.procs = alive
        if dead:
            detail = []
            for info, rc in dead:
                if hasattr(info, "p"):
                    detail.append(f"[{info.kind}] pid={info.p.pid} rc={rc} log={getattr(info,'log_path','')}")
                else:
                    detail.append(f"[proc] pid={info.pid} rc={rc}")
            raise RuntimeError("[reset] watchdog dead:\n" + "\n".join(detail))

        # 1) 清缓存/标志
        self.last_state = None
        self.last_obs = None
        self.last_avail_actions = None
        self.done = 0
        self.begin_cycle = -1

        # 2) coach/trainer buf（你现在 trainer 还没用到，可以先保留）
        cbuf = self.coach_shms[self.coach_name].buf
        # tbuf = self.trainer_shms[self.trainer_name].buf

        # 3) 等 gm==PlayOn(2)
        t_end = time.monotonic() + 20.0
        while time.monotonic() < t_end:
            alive, dead = proc.check_child_processes(self.procs, where="reset")
            self.procs = alive
            if dead:
                raise RuntimeError("[reset] watchdog dead while waiting PlayOn")

            gm = int(P.coach.read_gamemode(cbuf))
            cycle = int(P.coach.read_cycle(cbuf))

            if gm == 1:
                raise RuntimeError("[reset] gamemode=TimeOver(1)")

            if gm == 2:
                self.begin_cycle = cycle
                break

            time.sleep(0.05)

        if self.begin_cycle < 0:
            raise TimeoutError("[reset] 等待进入 PlayOn 超时")

        # 4) 首帧 barrier：全体 player READY
        keys = getattr(self, "_obs_keys", None)
        if keys is None:
            keys = sorted(self.player_names.keys())
            self._obs_keys = keys

        bufs = []
        for k in keys:
            shm_name = self.player_names[k]
            shm = self.player_shms[shm_name]
            bufs.append((shm.buf, f"{k} {shm_name}"))

        ipc.wait_all_ready_or_raise(
            bufs,
            off_a=P.player.OFFSET_FLAG_A,
            off_b=P.player.OFFSET_FLAG_B,
            timeout=20.0,
            poll=0.0005,
            log=self.log,
            tag="reset first-frame barrier",
        )

        print("Reset Over!!!")
        return

    def _reconnect_run(self, where: str = "reset") -> None:
        """
        负责：
        - 如有旧 run：teardown
        - 生成 run_id + lock
        - 选端口、建 log/rcg 目录
        - 生成 shm 名字并创建 shm
        - 拉起 server/players/coach
        - 收集 pgid/pid
        - 清空缓存字段（last_state/obs/avail/done/begin_cycle）
        """
        if self._closed:
            raise RuntimeError("env already closed; create a new env instance")

        # 旧 run 清理
        if self.run_id is not None or self.procs or self._shm_names:
            self._teardown_run()

        # run lock
        self.run_id = uuid.uuid4().hex
        self._lock_fd = proc.acquire_run_lock(self.run_id, log=self.log)
        self.log.info(f"[{where}] run_id={self.run_id}")

        # 选端口
        self.base_port, self.server_port, self.trainer_port, self.coach_port, self.debug_port = proc.pick_ports(
            self.args["auto_port_start"], self.args["auto_port_end"],
            self.args["auto_port_step"], self.args["trainer_port_offset"],
            self.args["coach_port_offset"], self.args["debug_port_offset"],
        )

        # log/rcg 目录
        base_logs_dir = os.path.abspath(str(self.args["logs_dir"]))
        self.log_dir = os.path.join(base_logs_dir, self.run_id)
        os.makedirs(self.log_dir, exist_ok=True)

        self.rcg_dir = os.path.join(self.log_dir, "rcg")
        os.makedirs(self.rcg_dir, exist_ok=True)

        self.log.info(f"[{where}] log_dir={self.log_dir}")
        self.log.info(f"[{where}] rcg_dir={self.rcg_dir}")

        # 清运行期字段
        self.begin_cycle = -1
        self.procs = []
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

        # 子进程 env
        env = self.child_env
        env["ROBOCUP2DRL_RUN_ID"] = self.run_id

        # server
        p, _ = proc.launch_server(
            server_path=str(self.args["server_path"]),
            server_port=int(self.server_port),
            trainer_port=int(self.trainer_port),
            coach_port=int(self.coach_port),
            logs_dir=self.log_dir,
            rcg_dir=self.rcg_dir,
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
        )
        self.procs.append(p)
        self.log.info(f"[{where}][coach] pid={p.pid} port={int(self.coach_port)} shm={self.coach_name}")
        time.sleep(0.2)

        self._collect_run_pgids()

        # 删除缓存 key（避免顺序/keys 继承上一次）
        if hasattr(self, "_obs_keys"):
            delattr(self, "_obs_keys")
        if hasattr(self, "_act_keys"):
            delattr(self, "_act_keys")

    def step(self, actions):
        """
        1) watchdog：子进程死了直接抛错
        2) 等待进入 PlayOn 且所有受控 agent READY
        3) 写动作 + submit_req
        4) 再等下一帧所有 READY
        5) 读 coach 结算：goal / timeover / timeout
        """
        # print("Step!!!")
        # 0) watchdog
        self.t1=time.time()
        # print(f"[time]{self.t1-self.t0}")
        alive, dead = proc.check_child_processes(self.procs, where="step")
        self.procs = alive
        if dead:
            items = []
            for info, rc in dead:
                if hasattr(info, "p"):  # ProcInfo
                    p = info.p
                    kind = info.kind
                    team = getattr(info, "team", "")
                    unum = getattr(info, "unum", "")
                    shm = getattr(info, "shm_name", "")
                    log = getattr(info, "log_path", "")
                    items.append(
                        f"[{kind}] pid={p.pid} rc={rc} team={team} unum={unum} shm={shm} log={log}"
                    )
                else:
                    # 原始 Popen（server, trainer, coach）
                    p = info
                    items.append(f"[server/trainer/coach] pid={p.pid} rc={rc}")

            detail = "\n".join(items)
            raise RuntimeError(f"[watchdog] child process died during get_state:\n{detail}")

        # done=1 还 step：保持旧版习惯直接返回
        if self.done == 1:
            return 0.0, True, {"episode_limit": 0.0}

        # keys 稳定顺序（缓存，避免每步 sort）
        keys = getattr(self, "_act_keys", None)
        if keys is None:
            keys = sorted(self.player_names.keys())  # (team_idx, unum)
            self._act_keys = keys
        n_agents = len(keys)

        # actions -> numpy（兼容 torch）
        if isinstance(actions, torch.Tensor):
            actions = actions.detach().cpu().numpy()
        actions = np.asarray(actions)
        # print("actions:", actions)

        # 判定 Base/Hybrid（Hybrid 为 (n,3)）
        is_hybrid = (actions.ndim == 2 and actions.shape[1] == 3)
        # print("is_hybrid:", is_hybrid)
        if is_hybrid:
            if not (actions.shape[0] == n_agents and actions.shape[1] == 3):
                raise RuntimeError(f"Hybrid 期望 actions.shape=(n_agents,3)，收到 {actions.shape}")
        else:
            actions = actions.reshape(-1)
            if actions.shape[0] != n_agents:
                raise RuntimeError(f"Base 期望 {n_agents} 个动作，收到 {actions.shape[0]}")

        # coach buf
        coach_shm = self.coach_shms[self.coach_name]
        cbuf = coach_shm.buf

        episode_limit = self.episode_limit
        GOAL_X = self.goal_x
        GOAL_Y = self.goal_y

        # 提前准备 bufs（barrier 用）
        bufs = []
        for k in keys:
            shm_name = self.player_names[k]
            shm = self.player_shms[shm_name]
            bufs.append((shm.buf, f"{k} {shm_name}"))

        ready_to_act, cycle, gm, ball = ipc.wait_until_playon_or_done(
            cbuf=cbuf,
            begin_cycle=self.begin_cycle,
            episode_limit=episode_limit,
            goal_x=GOAL_X,
            goal_y=GOAL_Y,
            log=self.log,
            tag="step wait until playon or done",
        )
        # print("ready_to_act:", ready_to_act)

        if ready_to_act:
            ipc.wait_all_ready_or_raise(
                bufs,
                off_a=P.player.OFFSET_FLAG_A,
                off_b=P.player.OFFSET_FLAG_B,
                timeout=20.0,
                poll=0.0005,
                log=self.log,
                tag="step pre-ready barrier",
            )

        # print("ready_to_write_act")

        # 2) 写动作（只在 PlayOn 同步点写）
        if ready_to_act and n_agents > 0:
            for idx, k in enumerate(keys):
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
            ready_to_act, cycle, gm, ball = ipc.wait_until_playon_or_done(
                cbuf=cbuf,
                begin_cycle=self.begin_cycle,
                episode_limit=episode_limit,
                goal_x=GOAL_X,
                goal_y=GOAL_Y,
                log=self.log,
                tag="step wait until playon or done",
            )
            # print("ready_to_act after action submit:", ready_to_act)
            # 0) 看门狗：检查子进程是否都活着
            alive, dead = proc.check_child_processes(self.procs, where="get_obs")
            self.procs = alive

            if dead:
                items = []
                for info, rc in dead:
                    if hasattr(info, "p"):  # ProcInfo
                        p = info.p
                        kind = info.kind
                        team = getattr(info, "team", "")
                        unum = getattr(info, "unum", "")
                        shm = getattr(info, "shm_name", "")
                        log = getattr(info, "log_path", "")
                        items.append(
                            f"[{kind}] pid={p.pid} rc={rc} team={team} unum={unum} shm={shm} log={log}"
                        )
                    else:
                        # 原始 Popen（server, trainer, coach）
                        p = info
                        items.append(f"[server/trainer/coach] pid={p.pid} rc={rc}")

                detail = "\n".join(items)
                raise RuntimeError(f"[watchdog] child process died during get_state:\n{detail}")
            
            # print("after watchdog check")
            if ready_to_act:
                # 3) 写完动作后：等下一帧全体 READY
                ipc.wait_all_ready_or_raise(
                    bufs,
                    off_a=P.player.OFFSET_FLAG_A,
                    off_b=P.player.OFFSET_FLAG_B,
                    timeout=20,
                    poll=0.0005,
                    log=self.log,
                    tag="step post-ready barrier",
                )

        # 4) 结算（读 coach）
        cycle = int(P.coach.read_cycle(cbuf))
        gm = int(P.coach.read_gamemode(cbuf))
        ball = P.coach.read_ball(cbuf, copy=True)

        timeout = (self.begin_cycle >= 0) and ((cycle - self.begin_cycle) >= episode_limit)
        scored = (abs(float(ball[0])) >= GOAL_X) and (abs(float(ball[1])) <= GOAL_Y)

        reward = 0.0
        self.done = 0

        if scored:
            self.done = 1
            reward = 1.0 if float(ball[0]) >= GOAL_X else -1.0
        elif gm == 1 or timeout:
            self.done = 1
            reward = 0.0

        info = {
            "episode_limit": float(timeout),
            "cycle": int(cycle),
            "gamemode": int(gm),
        }
        return float(reward), bool(self.done), info

    def get_obs(self):
        """
        返回 obs: np.ndarray shape = (n_agents, 97)

        逻辑：
        0) 看门狗：子进程挂了就抛错（避免卡在 wait）
        1) barrier：对所有球员逐个 wait_ready_or_raise（确保都 READY）
        2) 一口气读取所有 obs（第二轮读，减少混帧概率）
        """
        # done 缓存
        if self.done == 1 and self.last_obs is not None:
            return self.last_obs

        # 0) 看门狗：检查子进程是否都活着
        alive, dead = proc.check_child_processes(self.procs, where="get_obs")
        self.procs = alive
        if dead:
            items = []
            for info, rc in dead:
                if hasattr(info, "p"):  # ProcInfo
                    p = info.p
                    kind = info.kind
                    team = getattr(info, "team", "")
                    unum = getattr(info, "unum", "")
                    shm = getattr(info, "shm_name", "")
                    log = getattr(info, "log_path", "")
                    items.append(
                        f"[{kind}] pid={p.pid} rc={rc} team={team} unum={unum} shm={shm} log={log}"
                    )
                else:
                    # 原始 Popen（server, trainer, coach）
                    p = info
                    items.append(f"[server/trainer/coach] pid={p.pid} rc={rc}")

            detail = "\n".join(items)
            raise RuntimeError(f"[watchdog] child process died during get_state:\n{detail}")

        # keys 稳定顺序（并缓存，避免每步排序开销/顺序抖动）
        keys = getattr(self, "_obs_keys", None)
        if keys is None:
            keys = sorted(self.player_names.keys())  # (team_idx, unum)
            self._obs_keys = keys

        # 1) barrier：先确保“所有球员”READYdd
        # 一次性收集所有需要 barrier 的 shm buf
        bufs = []
        for k in keys:
            shm_name = self.player_names[k]
            shm = self.player_shms[shm_name]
            bufs.append((shm.buf, f"{k} {shm_name}"))

        # 全体轮询直到全部 READY（或总超时抛错）
        ipc.wait_all_ready_or_raise(
            bufs,
            off_a=P.player.OFFSET_FLAG_A,
            off_b=P.player.OFFSET_FLAG_B,
            timeout=20,
            poll=0.0005,
            log=self.log,
            tag="get_obs barrier",
        )

        # 2) 一口气读取 obs（第二轮读）
        obs_list = []
        for k in keys:
            shm_name = self.player_names[k]
            shm = self.player_shms[shm_name]
            o = P.player.read_obs_norm(buf=shm.buf,field_length=self.half_length, field_width=self.half_width)  # float32[97]
            obs_list.append(o)

        obs = np.stack(obs_list, axis=0).astype(np.float32, copy=False)
        self.last_obs = obs
        return obs

    def get_state(self):
        """
        从 coach shm 读全局 state：
        state = ball(4) + players(22*6) => (136,)
        不做 stable-read（不检查 cycle），直接读一次返回。
        """

        # 0) 看门狗：检查子进程是否都活着
        alive, dead = proc.check_child_processes(self.procs, where="get_state")
        self.procs = alive
        if dead:
            items = []
            for info, rc in dead:
                if hasattr(info, "p"):  # ProcInfo
                    p = info.p
                    kind = info.kind
                    team = getattr(info, "team", "")
                    unum = getattr(info, "unum", "")
                    shm = getattr(info, "shm_name", "")
                    log = getattr(info, "log_path", "")
                    items.append(
                        f"[{kind}] pid={p.pid} rc={rc} team={team} unum={unum} shm={shm} log={log}"
                    )
                else:
                    # 原始 Popen（server, trainer, coach）
                    p = info
                    items.append(f"[server/trainer/coach] pid={p.pid} rc={rc}")

            detail = "\n".join(items)
            raise RuntimeError(f"[watchdog] child process died during get_state:\n{detail}")
        
        # 1) done 缓存
        if self.done == 1 and self.last_state is not None:
            return self.last_state

        # 2) 读 coach shm
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
        if getattr(self, "_closed", False):
            return
        self._closed = True
        self._teardown_run()

    def __del__(self):
        # 析构阶段不要抛异常
        try:
            self.close()
        except Exception:
            pass

    def register_shm(self, name: str) -> None:
        """在你 create_shm() 成功后立刻调用，确保 close() 能清理到。"""
        if not name.startswith("/"):
            name = "/" + name
        self._shm_names.append(name)

    def get_env_info(self):
        """
        返回给 PyMARL 的 env_info。
        动作模式完全由 team1 名字决定：
          - team1 以 "base" 开头 -> 使用 base 动作 (17 个)
          - team1 以 "hybrid" 开头 -> 使用 hybrid 动作 (比如 4 个，你自己定)
        """
        team_name = self.team1

        # 1) 根据 team1 决定动作数
        if team_name.startswith("base"):
            n_actions = P.player.BASE_MASK_NUM        # 17
        elif team_name.startswith("hybrid"):
            n_actions = 4                             # 你目前的 hybrid 动作数，改成你真实的
        else:
            raise ValueError(f"未知的 team1='{self.team1}'，无法推断动作模式")

        # 2) 组装 env_info
        return {
            "n_agents": int(self.n1),                 # 只控制 team1 的球员数量
            "n_actions": int(n_actions),
            "state_shape": int(P.coach.COACH_STATE_FLOAT),  # 136
            "obs_shape": int(P.player.STATE_NUM),           # 97
            "episode_limit": int(self.episode_limit),
        }
    
    def _teardown_run(self) -> None:
        # 0) 重新采样一次（防止 _run_pgids/_run_pids 为空或过期）
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
                try: shm.close()
                except Exception: pass
                try: shm.unlink()
                except Exception: pass
            d.clear()

        try: _destroy_owner_dict(getattr(self, "coach_shms", {}))
        except Exception: pass
        try: _destroy_owner_dict(getattr(self, "trainer_shms", {}))
        except Exception: pass
        try: _destroy_owner_dict(getattr(self, "player_shms", {}))
        except Exception: pass

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

        if hasattr(self, "_obs_keys"):
            delattr(self, "_obs_keys")
        if hasattr(self, "_act_keys"):
            delattr(self, "_act_keys")

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
            try:
                if p.poll() is None:
                    pid = int(p.pid)
                    self._run_pids.add(pid)
                    pgid = os.getpgid(pid)
                    if pgid != py_pgid:
                        self._run_pgids.add(pgid)
            except Exception:
                pass

        self.log.info(f"[run] python_pgid={py_pgid} run_pgids={sorted(self._run_pgids)} run_pids={sorted(self._run_pids)[:8]}{'...' if len(self._run_pids)>8 else ''}")

    def _kill_run_process_groups(self, sig: int) -> None:
        py_pgid = os.getpgrp()
        pgids = set(getattr(self, "_run_pgids", set()))
        pids  = set(getattr(self, "_run_pids", set()))

        for pgid in sorted(pgids):
            if pgid == py_pgid:
                # self.log.warning(f"[teardown] skip killpg(pgid={pgid}) because it equals python pgid")
                continue

            # ✅ 关键：确认这个 pgid 里还存在“本 run 的存活 pid”
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
                # pgid 可能已空/被复用；不杀
                # self.log.info(f"[teardown] skip killpg pgid={pgid} sig={sig} (no alive member pid in this run)")
                continue

            try:
                os.killpg(pgid, sig)
                # self.log.info(f"[teardown] killpg pgid={pgid} sig={sig}")
            except ProcessLookupError:
                pass
            except Exception as e:
                self.log.warning(f"[teardown] killpg failed pgid={pgid} sig={sig} err={e}")

