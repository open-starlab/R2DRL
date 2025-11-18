import os, time, signal, struct, subprocess, random, re, psutil
from multiprocessing import shared_memory
import numpy as np
import torch
import sys, logging
import socket
import fcntl  # ← 新增：用于文件锁

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

# === Trainer SHM (与Helios Trainer改造版一致) ===
TRAINER_SHM_SIZE = 4096  # 4KB 控制面，足够 RESET 负载

# COACH_SHM_NAME    = "/coach_global_state"
# 偏移
T_FLAG_A = 0
T_FLAG_B = 1
T_OPCODE = 4  # int32

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

# ===== 严格配置加载 & 校验 =====
def _load_yaml_cfg(cfg):
    if isinstance(cfg, dict):
        return cfg
    if isinstance(cfg, str):
        base = os.path.dirname(os.path.abspath(__file__)) if '__file__' in globals() else os.getcwd()
        path = cfg if os.path.isabs(cfg) else os.path.join(base, cfg)
        import yaml  # 需要 pyyaml
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    raise TypeError("cfg 必须是 YAML 文件路径或 dict")

def _extract_env_args(root: dict) -> dict:
    # 兼容两种写法：平铺 或 {env: robocup, env_args: {...}}
    return root.get("env_args", root) or {}

def _sanitize_team_name(name: str) -> str:
    """只保留字母数字与下划线，避免服务器或日志里奇怪字符。"""
    name = name.strip()
    name = re.sub(r'\W+', '_', name)  # 非字母数字替换为 _
    return name or "Team"

_ALLOWED_MODES = {"base": "Base", "helios": "Helios", "hybrid": "Hybrid"}

def _norm_mode(s: str) -> str:
    return _ALLOWED_MODES.get(str(s).lower(), "Helios")

class Robocup2d_Python:
    def __init__(self, cfg="robocup.yaml", **env_args):
        # 如果 PyMARL 以关键字形式传入 n1/n2/...，就直接用它们
        if env_args:
            self.cfg = {"env_args": env_args}
        else:
            self.cfg = _load_yaml_cfg(cfg)

        self.args = _extract_env_args(self.cfg)  # 保持你的调用

        n1 = self.args["n1"]
        n2 = self.args["n2"]
        m1 = self.args["m1"]
        m2 = self.args["m2"]
        seed = self.args["seed"]
        self.episode_limit = self.args["episode_limit"]

        self.all_processes = []
        self.shm_refs = {}
        self.coach_shm = None

        self.begin_cycle = -1
        self.cycle = -1  
        self.absolute_cycle=0

        # 1) 动态库目录 ← 从 YAML 读
        raw_paths = self.args["lib_paths"]
        self.lib_paths = [p for p in raw_paths if os.path.isdir(p)]

        # 2) 构造独立环境变量
        self.env = os.environ.copy()
        self.env["LD_LIBRARY_PATH"] = ":".join(self.lib_paths) + ":" + self.env.get("LD_LIBRARY_PATH", "")

        # 端口/自动分配参数（严格模式：不设默认值）
        self._port_locks = []

        # 自动分配所需参数，必须在 YAML 提供，否则 KeyError
        coach_offset = int(self.args["coach_port_offset"])
        debug_offset = int(self.args["debug_port_offset"])
        trainer_offset = int(self.args["trainer_port_offset"])
        auto_start   = int(self.args["auto_port_start"])
        auto_end     = int(self.args["auto_port_end"])
        auto_step    = int(self.args["auto_port_step"])

        # —— 端口偏移校验 —— #
        # 三个偏移均非 0、互不相等，且都落在一个 step 段内
        assert all(isinstance(x, int) for x in (trainer_offset, coach_offset, debug_offset))
        assert trainer_offset != 0 and coach_offset != 0 and debug_offset != 0
        assert len({trainer_offset, coach_offset, debug_offset}) == 3, "trainer/coach/debug 偏移必须互不相等"
        assert auto_start < auto_end and auto_step > 0
        assert trainer_offset < auto_step and coach_offset < auto_step and debug_offset < auto_step

        # 忽略手动端口（即便给了也不用），统一自动分配+加锁
        srv, coach, debug = self._alloc_ports(
            base_start=auto_start,
            base_end=auto_end,
            step=auto_step,
            coach_offset=coach_offset,
            debug_offset=debug_offset,
            trainer_offset=trainer_offset,   # ← 新增：把 trainer 偏移传进去
        )
        # 保存 4 个端口
        self.port          = str(srv)                              # BASE（server）
        self.trainer_port  = int(self.port) + trainer_offset       # trainer = BASE + trainer_offset
        self.coach_port    = coach                                  # online coach
        self.debug_port    = debug                                  # debug

        # 两块共享内存名
        self.coach_shm_name   = f"/coach_global_state_{self.coach_port}"
        self.trainer_shm_name = f"/trainer_ctrl_{self.trainer_port}"
        ENV_LOG.info(
            f"[ports] server={self.port} trainer={self.trainer_port} "
            f"coach={self.coach_port} debug={self.debug_port} "
            f"shm_coach={self.coach_shm_name} shm_tr={self.trainer_shm_name}"
        )   

        # 记录两队的模式
        self.m1 = _norm_mode(m1)
        self.m2 = _norm_mode(m2)
        
        # 队名 = 模式名（不变）
        t1 = _sanitize_team_name(self.m1)
        t2 = _sanitize_team_name(self.m2)
        if t1.lower() == t2.lower():
            self.team1_name = f"{t1}_A"
            self.team2_name = f"{t2}_B"
        else:
            self.team1_name = t1
            self.team2_name = t2

        self.team1_num_players = n1
        self.team2_num_players = n2

        
        self.last_state = None
        self.last_obs = None
        self.last_avail_actions = None
        self._closed = False
        self.done = 0
        self.round = 0

        # 4) 球门尺寸 ← 从 YAML 读
        self.GOAL_X = float(self.args["goal_x"])
        self.GOAL_Y = float(self.args["goal_y"])

        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)

        self.coach_team = self.team1_name
        if self.m1 == "Helios" and self.m2 != "Helios":
            self.coach_team = self.team2_name

        self._mode_by_key = {}
        self.player_dir  = self.args["player_dir"]
        self.player_exe  = self.args["player_exe"]
        self.coach_dir   = self.args["coach_dir"]
        self.coach_exe   = self.args["coach_exe"]
        self.trainer_dir = self.args["trainer_dir"]
        self.trainer_exe = self.args["trainer_exe"]
        self.server_path = self.args["server_path"]
        self.host        = self.args["host"]

        # 5) 日志目录 ← 从 YAML 读
        root_logs = self.args["logs_dir"]
        self.logs_dir = os.path.join(root_logs, f"port_{self.port}")
        os.makedirs(self.logs_dir, exist_ok=True)
        self.rcg_dir = os.path.join(root_logs, f"rcg_{self.port}")
        os.makedirs(self.rcg_dir, exist_ok=True)
        self.launch_processes()
    
    # ========== 子进程管理 ==========
    def _popen(self, args, cwd=None, log_name=None):
        os.makedirs(self.logs_dir, exist_ok=True)
        log_file = subprocess.DEVNULL
        try:
            if log_name:  # 显式指定
                log_path = os.path.join(self.logs_dir, log_name)
                log_file = open(log_path, "w")

            args = list(map(str, args))
            p = subprocess.Popen(
                args, cwd=cwd, env=self.env,
                stdout=log_file, stderr=subprocess.STDOUT,
                start_new_session=True
            )
            self.all_processes.append(p)
            return p
        except Exception:
            if hasattr(log_file, "close"):
                try: log_file.close()
                except: pass
            raise

    # —— 1) helper：按模式给出动作数 ——
    def _n_actions_by_mode(self, mode: str) -> int:
        return BASE_MASK_NUM if mode == "Base" else (4 if mode == "Hybrid" else 0)

    def _launch_player_process(self, team, n, mode="Helios"):
        mode = _norm_mode(mode)
        shm_name = self._get_shm_name(team, n)
        print(f"shm_name={shm_name}")


        args = [
            self.player_exe, "-h", self.host, "-p", self.port,
            "-t", team, "-n", str(n),
            "--player-config", "../player.conf",
            "--config_dir", "../formations-dt",
            "--debug_server_host", self.host,
            "--debug_server_port", str(self.debug_port),
            "--mode", mode,
        ]
        # 只有 Base/Hybrid 才需要共享内存
        need_shm = (mode in ("Base", "Hybrid"))
        if need_shm:
            args.extend(["--shm-name", shm_name])
            self._mode_by_key[(team, n)] = mode

        if n == 1:
            args.append("--goalie")

        safe_team = re.sub(r"\W+", "_", team)
        self._popen(args, cwd=self.player_dir, log_name=f"player_{safe_team}_{n}_{self.port}.log")

        return need_shm, (shm_name if need_shm else None), (team, n)



    def auto_kill_processes(self):
        """
        只杀占用当前实例端口(server/coach/debug)的目标进程，避免误杀别的实例。
        """
        target_names = {"rcssserver", "rcssmonitor", "sample_player", "sample_coach","sample_trainer"}
        my_ports = {int(self.port), self.trainer_port, self.coach_port, self.debug_port}

        def _uses_my_ports(proc):
            # 1) 首选：进程级 net_connections（新版）
            try:
                get_conns = getattr(proc, "net_connections", None) or getattr(proc, "connections", None)
                if get_conns:
                    for c in get_conns(kind='inet'):
                        if c.laddr and getattr(c.laddr, "port", None) in my_ports:
                            return True
            except Exception:
                pass
            # 2) 兜底：命令行特征（端口/SHM 名）
            try:
                cmd = " ".join(proc.cmdline()).lower()
                if str(int(self.port)) in cmd or str(self.coach_port) in cmd or str(self.debug_port) in cmd:
                    return True
                if self.coach_shm_name.lower() in cmd:
                    return True
            except Exception:
                pass
            return False

        my_pid = os.getpid()
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                name = (proc.info.get('name') or '').lower()
                if proc.pid == my_pid:
                    continue
                if name in target_names and _uses_my_ports(proc):
                    try:
                        proc.terminate()
                        proc.wait(timeout=1.0)
                    except Exception:
                        try:
                            proc.kill()
                        except Exception:
                            pass
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
            except Exception:
                continue



    def auto_kill_port(self, port):
        """
        只清理占用指定端口(TCP/UDP)的进程：
        - 优先使用系统级 psutil.net_connections()（更快更新）
        - 无结果时回退到逐进程扫描（兼容旧 psutil）
        - 先 terminate，失败再 kill
        - 会跳过当前进程
        """
        victims = set()

        # ① 系统级扫描（推荐）
        try:
            for c in psutil.net_connections(kind='inet'):
                try:
                    if c.laddr and getattr(c.laddr, "port", None) == port and c.pid:
                        victims.add(c.pid)
                except Exception:
                    continue
        except Exception:
            pass

        # ② 回退：逐进程扫描（兼容旧版本）
        if not victims:
            for proc in psutil.process_iter(['pid', 'name']):
                try:
                    get_conns = getattr(proc, "net_connections", None) or getattr(proc, "connections", None)
                    if not get_conns:
                        continue
                    for c in get_conns(kind='inet'):
                        if c.laddr and getattr(c.laddr, "port", None) == port:
                            victims.add(proc.pid)
                            break
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
                except Exception:
                    continue

        # ③ 终止目标进程
        my_pid = os.getpid()
        for pid in victims:
            if pid == my_pid:
                continue
            try:
                p = psutil.Process(pid)
                try:
                    p.terminate()
                    p.wait(timeout=1.0)
                except Exception:
                    try:
                        p.kill()
                    except Exception:
                        pass
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
            except Exception:
                continue



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

    def launch_rcss_server(self):
        server_path = os.path.expanduser(self.server_path)

        base = int(self.port)

        # 防御：绝不能相等
        assert self.trainer_port != self.coach_port
        assert base not in (self.trainer_port, self.coach_port)

        args = [
            server_path,
            f"--server::port={base}",
            f"--server::coach_port={self.trainer_port}",        # <- 离线教练
            f"--server::olcoach_port={self.coach_port}", # <- 在线教练(给 sample_coach 用)
            "--server::coach=true",                        # ← 必须：允许离线教练/Trainer 连接
            "--server::coach_w_referee=true",
            "--server::auto_mode=true",
            "--server::synch_mode=true",
            "--server::synch_offset=60",
            "--server::synch_see_offset=60",
            "--server::simulator_step=100",
            "--server::send_step=100",
            f"--server::half_time={self.episode_limit}",
            f"--server::game_log_dir={self.rcg_dir}",
            f"--server::text_log_dir={self.rcg_dir}",
            "--server::text_logging=false", 
        ]

        # ENV_LOG.info("server cmd: " + " ".join(map(str, args)))
        self._popen(args, log_name=f"server_{self.port}.log")
        time.sleep(3)

    def launch_trainer(self):
        # 让 C++ 端按环境变量取 shm 名（与默认名一致）
        self.env["RCSC_TRAINER_SHM"] = self.trainer_shm_name

        args = [
            self.trainer_exe,
            "-h", self.host,
            "-p", str(self.trainer_port),
            # C++ 现在不解析 --shm-name 了，可以删掉这行，省得混淆
            # "--shm-name", self.trainer_shm_name,
            "--teaml", self.team1_name,
            "--teamr", self.team2_name,
        ]
        # 日志名用 trainer_port 更直观
        self._popen(args, cwd=self.trainer_dir, log_name=f"trainer_{self.trainer_port}.log")

        self.trainer_shm = self._wait_for_shm(
            name=self.trainer_shm_name,
            expected_size=TRAINER_SHM_SIZE,
            retries=200,
            delay=0.05
        )
        ENV_LOG.info(f"[trainer] up: port={self.trainer_port} shm={self.trainer_shm_name}")


    def launch_coach(self):
        args = [
            self.coach_exe,
            "-h", self.host,
            "-p", str(self.coach_port),      # 这里用 online coach 口
            "-t", self.coach_team,
            "--shm-name", self.coach_shm_name
        ]
        self._popen(args, cwd=self.coach_dir, log_name=f"coach_{self.coach_port}.log")

    def _get_shm_name(self, team, n):
        safe = re.sub(r'\W+', '_', team)
        return f"/{safe}_{self.port}_shm_{n-1}"


    def _wait_for_shm(self, name, expected_size, retries=200, delay=0.1):
        last_err = None
        for _ in range(retries):
            try:
                # ENV_LOG.info(f"尝试 attach shm: {name} (期望大小={expected_size})")
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
        # ENV_LOG.info("🛠 连接 coach 内存 ...")
        self.coach_shm = self._wait_for_shm(name=self.coach_shm_name, expected_size=COACH_SHM_SIZE)

    # ========== 内存交互 ==========
    def _flag(self, shm):
        flag_A = struct.unpack_from('B', shm.buf, OFFSET_FLAG_A)[0]
        flag_B = struct.unpack_from('B', shm.buf, OFFSET_FLAG_B)[0]

        return (flag_A, flag_B)
        
    def _read_obs(self, team, n, normalize: bool = True):
        shm = self.shm_refs[(team, n)]
        # 最多等 ~50ms，避免卡全局循环
        t_end = time.time() + 0.05
        while self._flag(shm) != (0, 1) and time.time() < t_end:
            time.sleep(0.0005)  # 0.5ms

        try:
            raw = struct.unpack_from(f'{STATE_NUM}f', shm.buf, OFFSET_STATE)  # tuple,len=97
        except Exception as e:
            raise RuntimeError(f"[ERROR] 读取 {team}#{n} 失败: {e}")

        # 不归一化：直接返回 float32
        if not normalize:
            return np.asarray(raw, dtype=np.float32)

        # ===== 归一化（可从 self.env_norm 读配置；否则用默认）=====
        cfg = getattr(self, "env_norm", None) or {}
        L  = float(cfg.get("L", 52.5))                 # pitchHalfLength
        W  = float(cfg.get("W", 34.0))                 # pitchHalfWidth
        m  = float(cfg.get("margin", 5.0))             # 球允许越界余量
        ps = float(cfg.get("player_speed_max", 1.2))   # 球员速上限
        bs = float(cfg.get("ball_speed_max", 3.0))     # 足球速上限
        sM = float(cfg.get("stamina_max", 8000.0))     # 体力上限(按你当前obs=8000)
        gM = float(cfg.get("game_mode_max", 50.0))     # game_mode 粗缩放上限

        o = np.asarray(raw, dtype=np.float32)
        out = np.zeros(STATE_NUM, dtype=np.float32)

        def clip(v, lo, hi):
            return lo if v < lo else (hi if v > hi else v)

        # --- self 0..5 ---
        out[0] = clip(o[0], -L, L) / L
        out[1] = clip(o[1], -W, W) / W
        out[2] = clip(o[2], -ps, ps) / ps
        out[3] = clip(o[3], -ps, ps) / ps
        out[4] = clip(o[4], 0.0, sM) / sM               # [0,1]
        out[5] = 1.0 if o[5] > 0.5 else 0.0             # 0/1

        # --- ball 6..9 ---
        out[6] = clip(o[6], -(L + m), (L + m)) / (L + m)
        out[7] = clip(o[7], -(W + m), (W + m)) / (W + m)
        out[8] = clip(o[8], -bs, bs) / bs
        out[9] = clip(o[9], -bs, bs) / bs

        # --- opponents 10..53 (11人 *4) ---
        base = 10
        for k in range(11):
            i = base + 4 * k
            out[i + 0] = clip(o[i + 0], -L, L) / L
            out[i + 1] = clip(o[i + 1], -W, W) / W
            out[i + 2] = clip(o[i + 2], -ps, ps) / ps
            out[i + 3] = clip(o[i + 3], -ps, ps) / ps

        # --- mates 54..93 (10人 *4，不含自己) ---
        base = 54
        for k in range(10):
            i = base + 4 * k
            out[i + 0] = clip(o[i + 0], -L, L) / L
            out[i + 1] = clip(o[i + 1], -W, W) / W
            out[i + 2] = clip(o[i + 2], -ps, ps) / ps
            out[i + 3] = clip(o[i + 3], -ps, ps) / ps

        # --- tail 94..96 ---
        out[94] = clip(o[94], 0.0, gM) / gM
        out[95] = 1.0 if o[95] > 0.5 else 0.0
        out[96] = 1.0 if o[96] > 0.5 else 0.0

        return out

    def _write_action_and_clear_the_flag(self, team, n, act: int):
        shm = self.shm_refs[(team, n)]
        struct.pack_into('i', shm.buf, OFFSET_ACTION, int(act))
        struct.pack_into('B', shm.buf, OFFSET_FLAG_A, 1)
        struct.pack_into('B', shm.buf, OFFSET_FLAG_B, 0)
        time.sleep(0.0005)
        

    def _wait_all_ready(self, timeout=10.0, poll_dt=0.01):
        """
        等待所有球员 flag=(0,1)。
        - timeout: 最大等待时间
        - poll_dt: 轮询间隔
        自动跳过非 PlayOn、进球、超时或环境已关闭的情况。
        """
        
        if not self.shm_refs:
            return  # 没有受控球员，直接跳过

        t0 = time.time()
        while True:
            self._check_child_processes("wait_all_ready")
            #关闭就退出
            if getattr(self, "_closed", False):
                # ENV_LOG.info("[WAIT] 环境已关闭，跳过等待")
                return
            #先检查最新的cycle和gm
            cycle, ball, players, gm = self.read_coach_state()
            # print(f"get_state:cycle={cycle, cycle-self.begin_cycle}")

            # ✅ 进球和超时返回
            if ((cycle-self.begin_cycle) >= self.episode_limit):
                # ENV_LOG.info(f"[WAIT] 检测到 gm={gm} 或终止条件，提前返回")
                # print("超时")
                return
            if (abs(ball[0]) >= self.GOAL_X and abs(ball[1]) <= self.GOAL_Y):
                # print("被进球")
                return

            if gm != 2:  # 非PlayOn但不在终止集合（例如 SetPlay）
                time.sleep(0.05)
                if time.time() - t0 > 300:
                   raise RuntimeError("gm!=2, _wait_all_ready time out")
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
    def read_coach_state(self, normalize: bool = False):
        self._check_child_processes("wait_all_ready")
        if self.coach_shm is None:
            raise RuntimeError("共享内存未初始化")

        # ① cycle (int32)，偏移 = 1
        cycle, = struct.unpack_from('i', self.coach_shm.buf, 1)
        if cycle > self.absolute_cycle:
            self.absolute_cycle = cycle

        # ② floats 区域 (COACH_STATE_FLOAT 个 float)，偏移 = 5
        floats = struct.unpack_from(f'{COACH_STATE_FLOAT}f', self.coach_shm.buf, 5)

        # ③ game_mode (int32)，偏移 = 1 + 4 + COACH_STATE_FLOAT*4
        mode_offset = 1 + 4 + COACH_STATE_FLOAT * 4
        game_mode, = struct.unpack_from('i', self.coach_shm.buf, mode_offset)

        # ④ 解析球和球员
        ball = list(floats[:4])  # [x, y, vx, vy]
        players = [list(floats[4 + i * 6: 4 + (i + 1) * 6]) for i in range(22)]
        # 每个 player: [x, y, vx, vy, dir_deg, team_id(0/1)]

        if not normalize:
            # 兼容原返回类型：ball tuple, players list of tuples
            return cycle, tuple(ball), [tuple(p) for p in players], game_mode

        # ====== 归一化参数（沿用你 env 的配置）======
        cfg = getattr(self, "env_norm", None) or {}
        L  = float(cfg.get("L", 52.5))                 # pitchHalfLength
        W  = float(cfg.get("W", 34.0))                 # pitchHalfWidth
        m  = float(cfg.get("margin", 5.0))             # 球允许越界余量
        ps = float(cfg.get("player_speed_max", 1.2))   # 球员速上限
        bs = float(cfg.get("ball_speed_max", 3.0))     # 足球速上限
        dir_max = 180.0                                # 方向角度范围 [-180,180]

        def clip(v, lo, hi):
            return lo if v < lo else (hi if v > hi else v)

        # ---- 球 [x,y,vx,vy] ----
        bx, by, bvx, bvy = ball
        bx_n  = clip(bx, -(L + m), (L + m)) / (L + m)
        by_n  = clip(by, -(W + m), (W + m)) / (W + m)
        bvx_n = clip(bvx, -bs, bs) / bs
        bvy_n = clip(bvy, -bs, bs) / bs
        ball_n = (bx_n, by_n, bvx_n, bvy_n)

        # ---- 22 名球员，每人 [x,y,vx,vy,dir_deg,team_id] ----
        players_n = []
        for p in players:
            x, y, vx, vy, deg, team = p
            x_n   = clip(x, -L, L) / L
            y_n   = clip(y, -W, W) / W
            vx_n  = clip(vx, -ps, ps) / ps
            vy_n  = clip(vy, -ps, ps) / ps
            deg_n = clip(deg, -dir_max, dir_max) / dir_max   # [-1,1]
            team_n = 1.0 if team > 0.5 else 0.0              # 保持 0/1
            players_n.append((x_n, y_n, vx_n, vy_n, deg_n, team_n))

        # ✅ cycle 与 game_mode 不做归一化，按你的要求保持原值返回
        return cycle, ball_n, players_n, game_mode

    def get_state(self):
        """
        等待所有球员obs写入(flag=1)然后读取，虽然是教练
        """
        self._check_child_processes("wait_all_ready")
        if self.done == 1:
            return self.last_state
        else:
            self._wait_all_ready()
            cycle, ball, players, game_mode = self.read_coach_state(normalize=True)
            ball = np.array(ball, dtype=np.float32)
            players = np.array(players, dtype=np.float32)
            state = np.concatenate([ball, players.flatten()])  # 136
            self.last_state = state
            return self.last_state

    def _cleanup_stale_shm(self):
        patterns = []
        if self.m1 in ("Base","Hybrid"):
            patterns += [f"/{_sanitize_team_name(self.m1)}_{self.port}_shm_{i}" for i in range(self.team1_num_players)]
        if self.m2 in ("Base","Hybrid"):
            patterns += [f"/{_sanitize_team_name(self.m2)}_{self.port}_shm_{i}" for i in range(self.team2_num_players)]
        for name in patterns:
            try:
                shared_memory.SharedMemory(name=name).unlink()
            except FileNotFoundError:
                pass
            except Exception as e:
                ENV_LOG.info(f"[shm cleanup] {name}: {e}")

    def _attach_trainer_shm(self, retries=200, delay=0.05):
        """
        按约定名附着 trainer 共享内存；成功后把句柄放到 self.trainer_shm 并返回。
        """
        shm = self._wait_for_shm(
            name=self.trainer_shm_name,
            expected_size=TRAINER_SHM_SIZE,
            retries=retries,
            delay=delay,
        )
        self.trainer_shm = shm
        ENV_LOG.info(f"[trainer] shm attached: {self.trainer_shm_name} size={shm.size}")
        return shm

    # ========== 环境接口 ==========
    def reset(self):
        # 清内部缓存
        self.last_state = None
        self.last_obs = None
        self.last_avail_actions = None

        self._closed = False
        self.done = 0
        self.round = self.round +1
        print(f"[Reset] Round {self.round}")

        # 5) 等待进入 PlayOn（加超时）
        if not self._wait_playon(timeout=200.0):
            raise TimeoutError(
                "等待进入 PlayOn 超时。请检查 server/coach/player 是否正常启动（见 ./log/*.log）。"
            )

        # # 6) 等待所有球员的首帧 flag==1
        # for (team, n), shm in self.shm_refs.items():
        #     t0 = time.time()
        #     while True:
        #         if self._flag(shm) == (0, 1):
        #             # ENV_LOG.info(f"✅ 首帧 ready: {team}#{n} flag=1")
        #             break
        #         if time.time() - t0 > 20.0:
        #             raise TimeoutError(f"[ERROR] 首帧未准备好: {team}#{n} flag仍为{self._flag(shm)}")
        #         time.sleep(0.01)
        

        return


    def _lock_port(self, port: int):
        """
        用 flock 对 /tmp/robocup_port_<port>.lock 加排他锁。
        成功返回文件描述符；失败返回 None。
        """
        lock_path = f"/tmp/robocup_port_{port}.lock"
        fd = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o666)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            # 可写入一点标识信息（可选）
            try:
                os.write(fd, f"pid={os.getpid()}\n".encode())
            except Exception:
                pass
            return fd
        except BlockingIOError:
            os.close(fd)
            return None
        
    def _try_lock_ports(self, ports):
        fds = []
        for p in ports:
            fd = self._lock_port(int(p))
            if fd is None:
                for x in fds:
                    try:
                        fcntl.flock(x, fcntl.LOCK_UN)
                        os.close(x)
                    except Exception:
                        pass
                return False
            fds.append(fd)
        self._port_locks = fds
        return True

    def _release_port_locks(self):
        for fd in getattr(self, "_port_locks", []):
            try:
                fcntl.flock(fd, fcntl.LOCK_UN)
                os.close(fd)
            except Exception:
                pass
        self._port_locks = []

    def __del__(self):
        try:
            # 如果还没关，就兜底关一下
            if not getattr(self, "_closed", True):
                self.close(release_port_locks=True)
        except Exception:
            # 析构阶段不要让异常往外冒
            pass
    
    def _check_child_processes(self, where: str = ""):
        """
        看门狗：如果发现有任何一个子进程已经退出（poll()!=None），
        就认为环境已破坏，立刻 close 掉所有进程，并抛异常。
        where: 字符串标签，方便日志定位是哪一步发现的。
        """
        # env 已经关了，就别再动了
        if getattr(self, "_closed", False):
            return

        alive = []
        dead  = []

        for p in self.all_processes:
            try:
                rc = p.poll()
            except Exception:
                # 极端情况（进程对象坏了），也当成死了
                rc = -999

            if rc is None:
                alive.append(p)
            else:
                dead.append((p, rc))

        # 更新一下当前进程列表（去掉已经死掉的）
        self.all_processes = alive

        if dead:
            detail_list = []
            for (p, rc) in dead:
                # 这是 subprocess.Popen 对象，不是 psutil.Process
                try:
                    # 程序名
                    prog = os.path.basename(str(p.args[0])) if p.args else "<??>"
                except Exception:
                    prog = "<??>"
                try:
                    # 完整命令行
                    cmdline = " ".join(map(str, p.args)) if p.args else "<no-args>"
                except Exception:
                    cmdline = "<no-args>"

                detail_list.append(
                    f"pid={p.pid}, rc={rc}, prog={prog}, cmd=\"{cmdline}\""
                )

            detail_str = " ; ".join(detail_list)

            # 日志里也打一份
            ENV_LOG.info(f"[watchdog] 子进程异常退出({where}): {detail_str}")

            # 一旦发现有挂的，直接把全家干掉
            try:
                self.close(release_port_locks=True)
            except Exception:
                # 这里别让异常再炸掉，看门狗只负责兜底
                pass

            # 告诉上层这局 env 已经废了（附带详细信息）
            raise RuntimeError(
                f"Robocup2d_Python: 子进程在 {where} 阶段异常退出，环境已自动关闭。"
                f" 详细信息: {detail_str}"
            )


    def _can_bind_all(self, port: int, check_ipv6: bool = True) -> bool:
        families = [("0.0.0.0", socket.AF_INET)]
        if check_ipv6:
            families.append(("::", socket.AF_INET6))
        for host, af in families:
            for typ in (socket.SOCK_DGRAM, socket.SOCK_STREAM):
                s = None
                try:
                    s = socket.socket(af, typ)
                    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    s.bind((host, port))
                except OSError:
                    if s:
                        try: s.close()
                        except: pass
                    # IPv6 不可用时，把它当成“忽略”，而不是“端口被占”
                    if af == socket.AF_INET6:
                        continue
                    return False
                finally:
                    try: s.close()
                    except: pass
        return True


    def _alloc_ports(self, base_start=6000, base_end=7000, step=40,
                    coach_offset=2, debug_offset=32, trainer_offset=1, tries=128):
        import random
        bases = list(range(base_start, base_end, step))
        random.shuffle(bases)
        count = 0
        for base in bases:
            srv     = base
            trainer = base + trainer_offset
            coach   = base + coach_offset
            debug   = base + debug_offset

            if max(srv, trainer, coach, debug) >= 65535:
                continue

            if (self._can_bind_all(srv, True) and
                self._can_bind_all(trainer, True) and
                self._can_bind_all(coach, True) and
                self._can_bind_all(debug, True) and
                self._try_lock_ports((srv, trainer, coach, debug))):
                return srv, coach, debug

            count += 1
            if count >= tries:
                break
        raise RuntimeError("自动分配端口失败：可用端口不足或被锁定。请调整范围/step 或改用手动端口。")



    def _wait_ports_free(self, ports=None, timeout=8.0, poll=0.1, hold=0.3):
        if ports is None:
            ports = (int(self.port), self.trainer_port, self.coach_port, self.debug_port)
        deadline = time.time() + timeout
        while time.time() < deadline:
            if all(self._can_bind_all(p, check_ipv6=True) for p in ports):
                time.sleep(hold)
                return True
            time.sleep(poll)
        return False

    def trainer_cmd(self, opcode: int, timeout=2.0):
        shm = self.trainer_shm  # 已经 attach 好的 4KB
        t_end = time.time() + timeout

        # 1) 等空闲 (0,0)
        while True:
            A = struct.unpack_from('B', shm.buf, T_FLAG_A)[0]
            B = struct.unpack_from('B', shm.buf, T_FLAG_B)[0]
            if (A, B) == (0, 0):
                break
            if time.time() > t_end:
                raise TimeoutError("trainer busy too long")
            time.sleep(0.001)

        # 2) 写入 opcode，再置 (0,1) 发起请求
        struct.pack_into('i', shm.buf, T_OPCODE, int(opcode))
        struct.pack_into('B', shm.buf, T_FLAG_B, 1)  # 请求
        # A 由 C++ 端来置

        # 3) 等待完成：看到恢复 (0,0) 即一轮结束
        t_end = time.time() + timeout
        while True:
            A = struct.unpack_from('B', shm.buf, T_FLAG_A)[0]
            B = struct.unpack_from('B', shm.buf, T_FLAG_B)[0]
            if (A, B) == (0, 0):
                break
            if time.time() > t_end:
                raise TimeoutError("trainer op timeout")
            time.sleep(0.001)


    def step(self, actions):
        self._check_child_processes("wait_all_ready")
        keys = sorted(self.shm_refs)
        if keys:
            # 张量→numpy
            if torch is not None and isinstance(actions, torch.Tensor):
                actions = actions.detach().cpu().numpy()
            actions = np.asarray(actions)

            # 先等本帧 ready
            self._wait_all_ready()
            _, ball, players, game_mode = self.read_coach_state()

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
        # if cycle % 10 == 0:
        #     self.trainer_cmd(5)
        timeout = cycle-self.begin_cycle > self.episode_limit

        if (abs(ball[0]) >= self.GOAL_X) and (abs(ball[1]) <= self.GOAL_Y):
            self.done = 1
            if ball[0] >= self.GOAL_X:
                # 进右边门（+X） → +1
                reward = 1.0
                print("[Done] Score.")
            else:
                # 进左边门（-X） → -1
                reward = -1.0
                print("[Done] Conceded.")
        else:
            reward = 0.0
            if timeout :
                self.done = 1
                print("[Done] Time out.")
            else:
                self.done = 0 

        return float(reward), bool(self.done), {"episode_limit": float(timeout)}

    def get_obs(self):
        self._check_child_processes("wait_all_ready")
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


    def close(self,release_port_locks=True):
        # ENV_LOG.info("close the env !!!")

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

        if getattr(self, "trainer_shm", None) is not None:
            try:
                self.trainer_shm.close()
                self.trainer_shm.unlink() 
            except FileNotFoundError:
                pass
            except Exception as e:
                ENV_LOG.info(f"⚠️ 清理 trainer shm 失败: {e}")
            self.trainer_shm = None

        self.all_processes = []
        self.shm_refs = {}
        self.coach_shm = None
        self._closed = True
        if release_port_locks:
            self._release_port_locks()  # ← 加这个判断
 

        if not self._wait_ports_free(timeout=8.0):
            ENV_LOG.info("[WARN] 端口未能在超时内释放，可能被其他用户进程占用")

    
    def get_env_info(self):
        return {
            "n_agents": 11,
            "n_actions": self._n_actions_by_mode(self.m1),
            "state_shape": COACH_STATE_FLOAT,
            "obs_shape": STATE_NUM,
            "episode_limit": 6000,
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
    
    def _wait_playon(self, timeout=30.0, poll=0.2):
        """等待进入 PlayOn；超时返回 False。"""
        t0 = time.time()
        last_gm = None
        while time.time() - t0 < timeout:
            cycle, _, _, gm = self.read_coach_state()
            if gm != last_gm:
                last_gm = gm
            if gm == 2:  # PlayOn
                self.begin_cycle=cycle
                return True
            time.sleep(poll)
            
        return False

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

    def launch_processes(self):
        """
        严格马尔可夫 reset：
        - 清理旧进程/端口，重启 server/coach/player 并 attach 共享内存
        - 等待进入 PlayOn + 全员首帧 flag==1
        """
        # ENV_LOG.info("🔄 reset 环境: 清理进程和端口")
        self.close(release_port_locks=False)
        if bool(self.args["aggressive_kill"]):
            self.auto_kill_processes()

        if not self._wait_ports_free((int(self.port), self.trainer_port, self.coach_port, self.debug_port), timeout=8.0):
            self.auto_kill_port(int(self.port))
            self.auto_kill_port(self.trainer_port)      # ← 新增
            self.auto_kill_port(self.coach_port)
            self.auto_kill_port(self.debug_port)
            if not self._wait_ports_free((int(self.port), self.trainer_port, self.coach_port, self.debug_port), timeout=8.0):
                raise RuntimeError("端口仍被占用（可能是其他用户进程），放弃本次重启")

        time.sleep(0.5)  # 给端口释放一点时间
        self._cleanup_stale_shm()
        # 关闭旧资源句柄
        
        # 启动 server
        self.launch_rcss_server()
        time.sleep(1)

        # 启动 trainer（并等待其 SHM ready）
        self.launch_trainer()
        self._attach_trainer_shm()
        time.sleep(0.5)

        # 清内部缓存
        self.last_state = None
        self.last_obs = None
        self.last_avail_actions = None

        self._closed = False
        self.done = 0

        # 1) 并行起所有 player 进程（不指定 -n 的那套，这里用你新的 launch_player_process）
        shm_wait_list = []
        shm_wait_list.append(self._launch_player_process(self.team1_name, 1, mode=self.m1))
        shm_wait_list.append(self._launch_player_process(self.team2_name, 1, mode=self.m2))
        time.sleep(0.2)
        for i in range(2, self.team1_num_players + 1):
            shm_wait_list.append(self._launch_player_process(self.team1_name, i, mode=self.m1))
        for i in range(2, self.team2_num_players + 1):
            shm_wait_list.append(self._launch_player_process(self.team2_name, i, mode=self.m2))

        # 统一 attach 仅 need_shm==True 的球员
        for need_shm, shm_name, key in shm_wait_list:
            if not need_shm:
                continue  # Helios 球员不 attach、不进 shm_refs
            shm = self._wait_for_shm(name=shm_name, expected_size=SHM_SIZE)
            self.shm_refs[key] = shm

        # 3) 起 coach 并连接 coach shm
        time.sleep(1)
        self.launch_coach()
        self._attach_coach_shm()
