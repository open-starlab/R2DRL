import os, time, signal, struct, subprocess, random, re, psutil
from multiprocessing import shared_memory
import numpy as np
import time
import torch
import sys, logging
# 让 print 逐行立刻刷新（Python 3.7+ 支持 reconfigure）
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(line_buffering=True)

# 建一个环境专用 logger，直接写 stdout、不带前缀
ENV_LOG = logging.getLogger("robocup_env")
if not ENV_LOG.handlers:
    h = logging.StreamHandler(sys.stdout)
    h.setFormatter(logging.Formatter("%(message)s"))
    ENV_LOG.propagate = False
    ENV_LOG.setLevel(logging.INFO)
    ENV_LOG.handlers = [h]

def align4(x): return (x + 3) & ~3

STATE_NUM  = 97
BASE_MASK_NUM   = 17

# 与 C++ SamplePlayer 的 OFFSET_* 完全一致（含 CYCLE，逐段对齐）
OFFSET_FLAG_A  = 0
OFFSET_FLAG_B  = 1
OFFSET_MASK    = align4(OFFSET_FLAG_B + 1)               # = 4
OFFSET_CYCLE   = align4(OFFSET_MASK + BASE_MASK_NUM)          # = align4(4 + 17) = 24
OFFSET_STATE   = align4(OFFSET_CYCLE + 4)                # = align4(28) = 28
OFFSET_ACTION  = align4(OFFSET_STATE + STATE_NUM * 4)    # = align4(28 + 388 = 416) = 416
OFFSET_HYBRID_MASK = align4(OFFSET_ACTION + 4)        # = 420
OFFSET_HYBRID_ACT  = align4(OFFSET_HYBRID_MASK + 4)   # = 424
OFFSET_HYBRID_U0   = align4(OFFSET_HYBRID_ACT + 4)    # = 428
OFFSET_HYBRID_U1   = align4(OFFSET_HYBRID_U0 + 4)     # = 432
SHM_SIZE = align4(OFFSET_HYBRID_U1 + 4)               # = 436

COACH_STATE_FLOAT = 136
COACH_SHM_SIZE    = 1 + 4 + COACH_STATE_FLOAT * 4 + 4
COACH_SHM_NAME    = "/coach_global_state"

# 运行前打印一眼，快速自检
print(f"[PY] OFF_A={OFFSET_FLAG_A} OFF_B={OFFSET_FLAG_B} OFF_MASK={OFFSET_MASK} "
      f"OFF_CYCLE={OFFSET_CYCLE} OFF_STATE={OFFSET_STATE} OFF_ACTION={OFFSET_ACTION} "
      f"SHM_SIZE={SHM_SIZE}")

# =================================================
    # 动作编号说明：
    # --- 有球动作 ---
    #  0: 铲球         → Bhv_BasicTackle
    #  1: 射门         → 严格射门 (isDoShootExecutable → doShoot)，否则降级为强制射门 (doForceKick)
    #  2: 拦截/追球     → move_behavior.doIntercept()
    #  3: 推进/解围     → advance_ball_action.execute()
    #  4: 传球         → pass_action.execute()
    #  5: 控球         → hold_ball.execute()
    #  6: 接球 (Catch) → doCatch() （仅门将、在禁区内可用）

    # --- 带球动作 ---
    #  7: 带球向上     → Body_Dribble2008(target=↑)
    #  8: 带球向下     → Body_Dribble2008(target=↓)
    #  9: 带球向左     → Body_Dribble2008(target=←)
    # 10: 带球向右     → Body_Dribble2008(target=→)

    # --- 无球移动 ---
    # 11: 无球移动 ↑   → doMoveTo(0)
    # 12: 无球移动 ↓   → doMoveTo(1)
    # 13: 无球移动 ←   → doMoveTo(2)
    # 14: 无球移动 →   → doMoveTo(3)
#===============================================================================

# 0 = BeforeKickOff
# 1 = TimeOver
# 2 = PlayOn
# 3 = KickOff_（带 side）
# 4 = KickIn_（带 side）
# 5 = FreeKick_（带 side）
# 6 = CornerKick_（带 side）
# 7 = GoalKick_（带 side）
# 8 = AfterGoal_（带 side）← 常规进球后
# 9 = OffSide_
# 10 = PenaltyKick_
# 11 = FirstHalfOver
# 12 = Pause
# 13 = Human
# 14 = FoulCharge_
# 15 = FoulPush_
# 16 = FoulMultipleAttacker_
# 17 = FoulBallOut_
# 18 = BackPass_
# 19 = FreeKickFault_
# 20 = CatchFault_
# 21 = IndFreeKick_
# 22 = PenaltySetup_
# 23 = PenaltyReady_
# 24 = PenaltyTaken_
# 25 = PenaltyMiss_ 
# 26 = PenaltyScore_
# 27 = IllegalDefense_
# 28 = PenaltyOnfield_
# 29 = PenaltyFoul_
# 30 = GoalieCatch_
# 31 = ExtendHalf
# 32 = MODE_MAX（不是实际模式，只是枚举上限）

import re

def _sanitize_team_name(name: str) -> str:
    """只保留字母数字与下划线，避免服务器或日志里奇怪字符。"""
    name = name.strip()
    name = re.sub(r'\W+', '_', name)  # 非字母数字替换为 _
    return name or "Team"

_ALLOWED_MODES = {"base": "Base", "helios": "Helios", "hybrid": "Hybrid"}

def _norm_mode(s: str) -> str:
    return _ALLOWED_MODES.get(str(s).lower(), "Helios")

class Robocup2d_Python:
    def __init__(self, n1=1, n2=1,m1="base", m2="Helios", seed=None,episode_limit=400):
        self.all_processes = []
        self.shm_refs = {}
        self.coach_shm = None
        
        # 1️⃣ 先罗列所有可能的动态库目录
        raw_paths = [
            "/fsws1/h_qin/libs",            # <-- 新增到最前
            "/fsws1/h_qin/hfo/protobuf-3.6.1/install/lib",
            "/home/h_qin/local/lib",
            "/home/h_qin/.local/lib",
        ]

        # 2️⃣ 过滤出真实存在的目录
        self.lib_paths = [p for p in raw_paths if os.path.isdir(p)]
        # ENV_LOG.info("✅ 有效 lib_paths:", self.lib_paths)

        # 3️⃣ 构造独立环境变量（一定放在过滤之后）
        self.env = os.environ.copy()
        self.env["LD_LIBRARY_PATH"] = ":".join(self.lib_paths) + ":" + self.env.get("LD_LIBRARY_PATH", "")
        # ENV_LOG.info("✅ LD_LIBRARY_PATH:", self.env["LD_LIBRARY_PATH"])

        # ---------- 其余参数保持原样 ----------
        self.player_dir  = "/fsws1/h_qin/robocup/robocup/helios-base/src/player"
        self.player_exe  = "./sample_player"
        self.coach_dir   = "/fsws1/h_qin/robocup/robocup/helios-base/src/coach"
        self.coach_exe   = "./sample_coach"
        self.server_path = "/fsws1/h_qin/robocup/robocup/rcssserver/build/rcssserver"
        self.host, self.port = "localhost", "6000"

        self.run_dir = os.path.dirname(os.path.abspath(__file__)) if '__file__' in globals() else os.getcwd()
        self.logs_dir = os.path.join(self.run_dir, "logs"); os.makedirs(self.logs_dir, exist_ok=True)


        # 记录两队的模式（仍然独立存放）
        self.m1 = _norm_mode(m1)
        self.m2 = _norm_mode(m2)

        # 队伍名 = 模式名（按你的要求）
        t1 = _sanitize_team_name(self.m1)
        t2 = _sanitize_team_name(self.m2)

        # 如果两队模式相同，自动加后缀区分，避免“同队名”被服务器拒绝 & 避免 SHM 名冲突
        if t1.lower() == t2.lower():
            self.team1_name = f"{t1}_A"
            self.team2_name = f"{t2}_B"
        else:
            self.team1_name = t1
            self.team2_name = t2

        self.team1_num_players = n1
        self.team2_num_players = n2

        self.cycle = -1 
        self.episode_limit = episode_limit #上限cycles

        self.gm_log_file = f"gamemode_{int(time.time())}.txt"

        self.last_state = None
        self.last_obs = None
        self.last_avail_actions = None
        

        self._closed = False
        self.done = 0

        self.GOAL_X = 52.5
        self.GOAL_Y = 7.01


        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)

        self.coach_team = self.team1_name
        if self.m1 == "Helios" and self.m2 != "Helios":
            self.coach_team = self.team2_name
        
        self._mode_by_key = {}

    # ========== 子进程管理 ==========
    def _popen(self, args, cwd=None, team=None, n=None):
        log_path = None
        if team is not None and n is not None:
            log_path = os.path.join(self.player_dir, f"player_{team}_{n}.log")
            log_file = open(log_path, "w")
        else:
            log_file = subprocess.DEVNULL  # 默认不写文件

        p = subprocess.Popen(
            args,
            cwd=cwd,
            env=self.env,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True
        )
        self.all_processes.append(p)
        return p

    # —— 1) helper：按模式给出动作数 ——
    def _n_actions_by_mode(self, mode: str) -> int:
        return BASE_MASK_NUM if mode == "Base" else (4 if mode == "Hybrid" else 0)

    def _launch_player_process(self, team, n, mode="Helios"):
        mode = _norm_mode(mode)
        shm_name = self._get_shm_name(team, n)

        args = [
            self.player_exe, "-h", self.host, "-p", self.port,
            "-t", team, "-n", str(n),
            "--player-config", "../player.conf",
            "--config_dir", "../formations-dt",
            "--debug_server_host", self.host,
            "--debug_server_port", "6032",
            "--mode", mode,
        ]
        # 只有 Base/Hybrid 才需要共享内存
        need_shm = (mode in ("Base", "Hybrid"))
        if need_shm:
            args.extend(["--shm-name", shm_name])
            self._mode_by_key[(team, n)] = mode

        if n == 1:
            args.append("--goalie")

        py_dir = os.path.dirname(os.path.abspath(__file__))
        log_path = os.path.join(py_dir, f"player_{team}_{n}.log")
        log_file = open(log_path, "w")

        p = subprocess.Popen(
            args, cwd=self.player_dir, env=self.env,
            stdout=log_file, stderr=subprocess.STDOUT, start_new_session=True
        )
        self.all_processes.append(p)

        # 返回 (是否需要attach, shm_name 或 None, key)
        return need_shm, (shm_name if need_shm else None), (team, n)



    def auto_kill_processes(self, names=["rcssserver", "rcssmonitor", "sample_player", "sample_coach"]):
        for proc in psutil.process_iter(['pid', 'name']):
            if proc.info['name'] in names:
                try:
                    proc.kill()
                    # ENV_LOG.info(f"🗢 杀死旧进程 {proc.info['name']} (pid={proc.pid})")
                except Exception:
                    continue

    def auto_kill_port(self, port=6000):
        for proc in psutil.process_iter(['pid', 'name']):
            try:
                for conn in proc.connections(kind='inet'):
                    if conn.laddr.port == port:
                        proc.kill()
                        # ENV_LOG.info(f"🔌 杀死占用端口的进程 {proc.pid}")
            except:
                continue

    def _read_hybrid_mask(self, team, n):
        """当 flag==(0,1) 时读取 4 位 Hybrid 掩码 [TURN,DASH,KICK,CATCH]"""
        shm = self.shm_refs[(team, n)]
        if self._flag(shm) != (0, 1):
            return None
        return struct.unpack_from('4B', shm.buf, OFFSET_HYBRID_MASK)

    def _write_hybrid_and_clear_flag(self, team, n, a: int, u0: float, u1: float):
        """
        写入 Hybrid 动作 (a,u0,u1)，然后置 flag_A=1, flag_B=0
        a ∈ {0,1,2,3}; u0,u1 ∈ [0,1]
        """
        shm = self.shm_refs[(team, n)]
        # 截断/校验
        a = int(a)
        u0 = float(max(0.0, min(1.0, u0)))
        u1 = float(max(0.0, min(1.0, u1)))

        struct.pack_into('i', shm.buf, OFFSET_HYBRID_ACT, a)
        struct.pack_into('f', shm.buf, OFFSET_HYBRID_U0, u0)
        struct.pack_into('f', shm.buf, OFFSET_HYBRID_U1, u1)

        # 和 Base 一样的握手：清 flag -> C++ 端执行
        struct.pack_into('B', shm.buf, OFFSET_FLAG_A, 1)
        struct.pack_into('B', shm.buf, OFFSET_FLAG_B, 0)
        time.sleep(0.0005)

    def _wait_all_cycle_advance(self, timeout=3.0, poll_dt=0.005):
        if not hasattr(self, "_last_cycles"):
            self._last_cycles = {k: -1 for k in self.shm_refs.keys()}

        t0 = time.time()
        while True:
            curr_cycles = {}
            flags = {}
            for (team, n), shm in self.shm_refs.items():
                (cyc,) = struct.unpack_from('i', shm.buf, OFFSET_CYCLE)
                curr_cycles[(team, n)] = cyc
                flags[(team, n)] = self._flag(shm)

            all_advanced = all(
                curr_cycles[k] > self._last_cycles.get(k, -1) for k in self.shm_refs.keys()
            )
            any_advanced = any(
                curr_cycles[k] > self._last_cycles.get(k, -1) for k in self.shm_refs.keys()
            )

            if all_advanced and any_advanced:
                self._last_cycles = curr_cycles
                return curr_cycles

            if time.time() - t0 > timeout:
                diff = []
                for k in self.shm_refs.keys():
                    last = self._last_cycles.get(k, -1)
                    curr = curr_cycles[k]
                    flg  = flags[k]
                    diff.append(f"{k[0]}#{k[1]} last={last} curr={curr} flag={flg}")
                raise TimeoutError("[ERROR] 等待球员 cycle 前进超时:\n  " + "\n  ".join(diff))

            time.sleep(poll_dt)


    def launch_rcss_server(self):
        ENV_LOG.info("🛠 启动 rcssserver ...")
        server_path = os.path.expanduser(self.server_path)
        args = [
            server_path,
            "server::auto_mode=true",        # 自动控制
            "server::synch_offset=60",
            "server::synch_see_offset=60",
            "server::simulator_step=100",
            "server::send_step=100",
            "server::synch_mode=true",
            f"server::half_time={self.episode_limit}",
        ]
        self._popen(args)
        time.sleep(3)

    def launch_coach(self):
        ENV_LOG.info("🛠 启动 coach 进程 ...")

        # 和球员一样：把日志放到当前 Python 文件夹下
        py_dir = os.path.dirname(os.path.abspath(__file__)) if '__file__' in globals() else os.getcwd()
        ts = time.strftime("%Y%m%d_%H%M%S")
        # 两个命名方案，二选一：
        # 方案1：带时间戳，避免覆盖
        log_path = os.path.join(py_dir, f"coach_{ts}.log")
        # 方案2：固定文件名，如果你希望始终覆盖同一个：
        # log_path = os.path.join(py_dir, f"coach_{self.team1_name}.log")

        log_file = open(log_path, "w")

        args = [
            self.coach_exe,
            "-h", self.host,           # 例如 "localhost"
            "-p", "6002",              # 与 server::online_coach_port 保持一致
            "-t", self.coach_team,     # 关注的队名，避免 no_such_team
            "--shm-name", COACH_SHM_NAME
        ]

        p = subprocess.Popen(
            args,
            cwd=self.coach_dir,        # 在 coach 目录下运行二进制
            env=self.env,
            stdout=log_file,           # ✅ 日志写到当前目录
            stderr=subprocess.STDOUT,
            start_new_session=True
        )
        self.all_processes.append(p)
        ENV_LOG.info(f"📝 coach 日志文件: {log_path}")



    def _get_shm_name(self, team, n):
        safe = re.sub(r'\W+', '_', team)
        return f"/{safe}_shm_{n-1}"

    def launch_sample_player(self, team, n):
        shm_name = self._get_shm_name(team, n)
        ENV_LOG.info(f"🎮 launch {team} #{n}  shm={shm_name}")
        args = [
            self.player_exe, "-h", self.host, "-p", self.port,
            "-t", team, "-n", str(n),
            "--shm-name", shm_name,
            "--player-config", "../player.conf",
            "--config_dir", "../formations-dt",
            "--debug_server_host", self.host,
            "--debug_server_port", "6032"
        ]
        # ✅ 只给 1 号加守门员（与旧逻辑保持一致）
        if n == 1:
            args.append("--goalie")

        self._popen(args, cwd=self.player_dir)

        shm = self._wait_for_shm(name=shm_name, expected_size=SHM_SIZE)
        self.shm_refs[(team, n)] = shm

    def _wait_for_shm(self, name, expected_size, retries=80, delay=0.05):
        last_err = None
        for _ in range(retries):
            try:
                ENV_LOG.info(f"尝试 attach shm: {name} (期望大小={expected_size})")
                shm = shared_memory.SharedMemory(name=name)
                if shm.size != expected_size:
                    shm.close()
                    raise RuntimeError(f"shm {name} size mismatch: got {shm.size}, expected {expected_size}")
                ENV_LOG.info(f"✅ 成功 attach shm: {name}, size={shm.size}")
                return shm
            except FileNotFoundError as e:
                last_err = e
                time.sleep(delay)
        raise RuntimeError(f"❌ shm {name} not found after {retries} retries (last={last_err})")

    def _attach_coach_shm(self):
        ENV_LOG.info("🛠 连接 coach 内存 ...")
        self.coach_shm = self._wait_for_shm(name=COACH_SHM_NAME, expected_size=COACH_SHM_SIZE)

    # ========== 内存交互 ==========
    def _flag(self, shm):
        flag_A = struct.unpack_from('B', shm.buf, OFFSET_FLAG_A)[0]
        flag_B = struct.unpack_from('B', shm.buf, OFFSET_FLAG_B)[0]
        return (flag_A, flag_B)

    def _read_mask(self, team, n):
        shm = self.shm_refs[(team, n)]
        flag = self._flag(shm)
        if flag != (0,1):
            return None
        try:
            return struct.unpack_from(f'{BASE_MASK_NUM}B', shm.buf, OFFSET_MASK)
        except Exception as e:
            ENV_LOG.info(f"[ERROR] read_mask failed: team={team}, n={n}, flag={flag}, err={e}")
            return None

    def _read_obs(self, team, n):
        shm = self.shm_refs[(team, n)]
        # 最多等 2~3ms 就放弃本帧，避免卡全局循环
        t_end = time.time() + 0.05
        while self._flag(shm) != (0,1) and time.time() < t_end:
            time.sleep(0.0005)  # 0.5ms

        try:
            return struct.unpack_from(f'{STATE_NUM}f', shm.buf, OFFSET_STATE)
        except Exception as e:
            raise RuntimeError(f"[ERROR] 读取 {team}#{n} 失败: {e}")

    def _write_action_and_clear_the_flag(self, team, n, act: int):
        shm = self.shm_refs[(team, n)]
        struct.pack_into('i', shm.buf, OFFSET_ACTION, int(act))
        struct.pack_into('B', shm.buf, OFFSET_FLAG_A, 1)
        struct.pack_into('B', shm.buf, OFFSET_FLAG_B, 0)
        time.sleep(0.0005)
        

    def _wait_all_ready(self, timeout=10.0, poll_dt=0.05):
        """
        等待所有球员 flag=(0,1)。
        - timeout: 最大等待时间
        - poll_dt: 轮询间隔
        自动跳过非 PlayOn、进球、超时或环境已关闭的情况。
        """
        if not self.shm_refs:
            return  # 没有受控球员，直接跳过

        # 非PlayOn模式列表
        NON_PLAYON_MODES = {0, 1, 3, 4, 6, 7, 8}

        t0 = time.time()
        while True:
            #关闭就退出
            if getattr(self, "_closed", False):
                ENV_LOG.info("[WAIT] 环境已关闭，跳过等待")
                return
            #先检查最新的cycle和gm
            cycle, ball, players, gm = self.read_coach_state()

            # ✅ 进球和超时返回
            if (cycle >= self.episode_limit) or (abs(ball[0]) >= self.GOAL_X and abs(ball[1]) <= self.GOAL_Y) :
                ENV_LOG.info(f"[WAIT] 检测到 gm={gm} 或终止条件，提前返回")
                return
            
            if gm != 2:  # 非PlayOn但不在终止集合（例如 SetPlay）
                ENV_LOG.info(f"[WAIT] 当前GameMode={gm}, 暂停等待PlayOn恢复")
                time.sleep(0.05)
                continue
            #正常逻辑
            if gm ==2:
                flags = {k: self._flag(shm) for k, shm in self.shm_refs.items()}
                not_ready = {f"{team}#{n}": flag for (team, n), flag in flags.items() if flag != (0, 1)}
                if not not_ready:
                    return  # 全部准备好了

                if time.time() - t0 > timeout:
                    detail = ", ".join(f"{k}: flag={flag}" for k, flag in not_ready.items())
                    raise TimeoutError(f"[ERROR] 超时等待共享内存 flag=(0,1)：未就绪={detail}")

                time.sleep(poll_dt)



    # ========== State ==========
    def read_coach_state(self):
        if self.coach_shm is None:
            raise RuntimeError("共享内存未初始化")

        # ① cycle (int32)，偏移 = 1
        cycle, = struct.unpack_from('i', self.coach_shm.buf, 1)

        # ② floats 区域 (136 floats = 544 bytes)，偏移 = 5
        floats = struct.unpack_from(f'{COACH_STATE_FLOAT}f', self.coach_shm.buf, 5)

        # ③ game_mode (int32)，偏移 = 1 + 4 + 136*4 = 549
        mode_offset = 1 + 4 + COACH_STATE_FLOAT * 4
        game_mode, = struct.unpack_from('i', self.coach_shm.buf, mode_offset)

        # ④ 解析球和球员
        ball = floats[:4]
        players = [floats[4 + i * 6: 4 + (i + 1) * 6] for i in range(22)]

        return cycle, ball, players, game_mode

    def get_state(self):
        """
        等待所有球员obs写入(flag=1)然后读取，虽然是教练
        """
        if self.done == 1:
            return self.last_state
        else:
            self._wait_all_ready()
            cycle, ball, players, game_mode = self.read_coach_state()
            ball = np.array(ball, dtype=np.float32)
            players = np.array(players, dtype=np.float32)
            state = np.concatenate([ball, players.flatten()])  # 136
            self.last_state = state
            return self.last_state


    # ========== 环境接口 ==========
    def reset(self):
        """
        严格马尔可夫 reset：
        - 清理旧进程/端口，重启 server/coach/player 并 attach 共享内存
        - 等待进入 PlayOn + 全员首帧 flag==1
        """
        ENV_LOG.info("🔄 reset 环境: 清理进程和端口")
        self.auto_kill_processes()
        self.auto_kill_port(6000)
        time.sleep(0.5)  # 给端口释放一点时间

        # 关闭旧资源句柄
        self.close()

        # 启动 server
        self.launch_rcss_server()
        time.sleep(1)

        # 清内部缓存
        self.last_state = None
        self.last_obs = None
        self.last_avail_actions = None

        self._closed = False
        self.sp_done = False
        self.done = 0

        # 1) 并行起所有 player 进程（不指定 -n 的那套，这里用你新的 launch_player_process）
        shm_wait_list = []
        for i in range(1, self.team1_num_players + 1):
            shm_wait_list.append(self._launch_player_process(self.team1_name, i, mode=self.m1))
        for i in range(1, self.team2_num_players + 1):
            shm_wait_list.append(self._launch_player_process(self.team2_name, i, mode=self.m2))

        # 统一 attach 仅 need_shm==True 的球员
        for need_shm, shm_name, key in shm_wait_list:
            if not need_shm:
                continue  # Helios 球员不 attach、不进 shm_refs
            shm = self._wait_for_shm(name=shm_name, expected_size=SHM_SIZE)
            self.shm_refs[key] = shm

        ENV_LOG.info(f"✅ 受控球员数(需要通信) = {len(self.shm_refs)}")

        # 3) 起 coach 并连接 coach shm
        time.sleep(1)
        self.launch_coach()
        self._attach_coach_shm()

        # 4) 记录初始 cycle
        cycle, _, _, _ = self.read_coach_state()
        self.cycle = cycle
        ENV_LOG.info(f"✅ 初始cycle = {self.cycle}")

        # 5) 等待进入 PlayOn
        while True:
            cycle, ball, players, game_mode = self.read_coach_state()
            print(f"cycle = {cycle}, gm={game_mode}")
            if game_mode == 2:  # play_on
                break
            time.sleep(0.5)

        # print("play on!!")

        # 6) 等待所有球员的首帧 flag==1
        for (team, n), shm in self.shm_refs.items():
            t0 = time.time()
            while True:
                if self._flag(shm) == (0, 1):
                    ENV_LOG.info(f"✅ 首帧 ready: {team}#{n} flag=1")
                    break
                if time.time() - t0 > 20.0:
                    raise TimeoutError(f"[ERROR] 首帧未准备好: {team}#{n} flag仍为{self._flag(shm)}")
                time.sleep(0.01)

        return

    def step(self, actions):
        keys = sorted(self.shm_refs)
        if keys:
            # 张量→numpy
            if torch is not None and isinstance(actions, torch.Tensor):
                actions = actions.detach().cpu().numpy()
            actions = np.asarray(actions)

            # 先等本帧 ready
            self._wait_all_ready()
            cycle, ball, players, game_mode = self.read_coach_state()

            if game_mode == 2:
                # 判断是否 Hybrid
                is_hybrid = (self.m1 == "Hybrid")
                if is_hybrid:
                    # 形状应该是 [n_agents, 3]
                    assert actions.ndim == 2 and actions.shape[0] == len(keys) and actions.shape[1] == 3, \
                        f"Hybrid 期望动作形状 (n_agents,3)，收到 {actions.shape}"
                    for (team, n), (a, u0, u1) in zip(keys, actions):
                        # 可选：对 a/u0/u1 做校验/裁剪
                        a  = int(a)
                        u0 = float(np.clip(u0, 0.0, 1.0))
                        u1 = float(np.clip(u1, 0.0, 1.0))
                        # 如果该 agent 不是 Hybrid（极端混搭场景），退回整数写法
                        mode = self._mode_by_key.get((team, n), self.m1)
                        if mode == "Hybrid":
                            self._write_hybrid_and_clear_flag(team, n, a, u0, u1)
                        else:
                            self._write_action_and_clear_the_flag(team, n, a)
                else:
                    # Base：接受 [n_agents] 的整数动作
                    actions = actions.flatten()
                    assert len(actions) == len(keys), f"env.step 期望 {len(keys)} 个动作，收到 {len(actions)} 个"
                    for (team, n), act in zip(keys, actions):
                        self._write_action_and_clear_the_flag(team, n, int(act))

                # 等下一帧写好
                self._wait_all_ready()
        else:
            time.sleep(0.005)

        # ---------- 结算 ----------
        cycle, ball, players, game_mode = self.read_coach_state()
        timeout = (cycle >= self.episode_limit)

        if abs(ball[0]) >= self.GOAL_X and abs(ball[1]) <= self.GOAL_Y:
            scorer_side = 0 if ball[0] >= self.GOAL_X else 1
            team1_side = getattr(self, "_team1_side", 0)
            reward = 1.0 if scorer_side == team1_side else -1.0
            self.done = 1
        else:
            reward = 0.0
            self.done = 1 if timeout else 0

        return float(reward), bool(self.done), {"episode_limit": float(timeout)}

    def get_obs(self):
        if self.done == 1:
            return self.last_obs
        else:
            self._wait_all_ready()

            keys = sorted(self.shm_refs)
            obs_list = []

            for k in keys:
                st = self._read_obs(*k)
                obs_list.append(st)

            obs = np.asarray(obs_list, dtype=np.float32)
            self.last_obs = obs
            return self.last_obs

    class NotReadyThisFrame(RuntimeError):
        pass   

    def get_avail_actions(self):
        """
        等待所有球员 flag=1,然后一次性读取动作 mask。
        """
        if self.done == 1:
            return self.last_avail_actions
        
        self._wait_all_ready()
        if self.m1 == "Hybrid":
           masks = [struct.unpack_from('4B', shm.buf, OFFSET_HYBRID_MASK)
                     for shm in self.shm_refs.values()]
        else:  # Base（或其它默认按 Base）
            masks = [struct.unpack_from(f'{BASE_MASK_NUM}B', shm.buf, OFFSET_MASK)
                     for shm in self.shm_refs.values()]
        self.last_avail_actions = np.asarray(masks, dtype=np.int32)
        return self.last_avail_actions


    def close(self):
        ENV_LOG.info("close the env !!!")

        for p in self.all_processes:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except Exception:
                try: p.terminate()
                except: pass

        for shm in self.shm_refs.values():
            try:
                shm.close()
                shm.unlink()
            except FileNotFoundError:
                pass
            except Exception as e:
                ENV_LOG.info(f"⚠️ 清理共享内存失败: {e}")

        # ✅ 补充：关闭 coach shm
        if self.coach_shm is not None:
            try:
                self.coach_shm.close()
                self.coach_shm.unlink()
            except FileNotFoundError:
                pass
            except Exception as e:
                ENV_LOG.info(f"⚠️ 清理 coach shm 失败: {e}")

        self.all_processes = []
        self.shm_refs = {}
        self.coach_shm = None
        self._closed = True
        
    def get_env_info(self):
        return {
            "n_agents": len(self.shm_refs),
            "n_actions": self._n_actions_by_mode(self.m1),
            "state_shape": COACH_STATE_FLOAT,
            "obs_shape": STATE_NUM,
            "episode_limit": self.episode_limit,
        }
    
    def save_replay(self):
        ENV_LOG.info("Replay not supported for this environment.")

    def get_obs_agent(self, agent_id):
        keys = sorted(self.shm_refs)
        k = keys[agent_id]
        return np.array(self._read_obs(*k), dtype=np.float32)

    def get_obs_size(self):
        return STATE_NUM

    def get_state_size(self):
        return COACH_STATE_FLOAT

    def get_avail_agent_actions(self, agent_id):
        keys = sorted(self.shm_refs)
        k = keys[agent_id]
        shm = self.shm_refs[k]
        self._wait_all_ready()

        if self.m1 == "Hybrid":
            mask = struct.unpack_from('4B', shm.buf, OFFSET_HYBRID_MASK)
        else:
            mask = struct.unpack_from(f'{BASE_MASK_NUM}B', shm.buf, OFFSET_MASK)
        return np.asarray(mask, dtype=np.int32)


    def get_total_actions(self):
        return self._n_actions_by_mode(self.m1)

    def render(self):
        pass  # 或者 ENV_LOG.info("渲染不支持")

    def seed(self, seed=None):
        random.seed(seed)
        np.random.seed(seed)