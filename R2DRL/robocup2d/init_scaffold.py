# init_scaffold.py
from __future__ import annotations
import os
from pathlib import Path

PROJECT_NAME = "robocup_env_scaffold"

FILES: dict[str, str] = {
    "pyproject.toml": """\
[build-system]
requires = ["setuptools>=68", "wheel"]
build-backend = "setuptools.build_meta"

[project]
name = "robocup-env"
version = "0.1.0"
requires-python = ">=3.9"
dependencies = ["numpy", "psutil", "pyyaml", "torch"]

[tool.setuptools]
package-dir = {"" = "src"}

[tool.setuptools.packages.find]
where = ["src"]
""",

    "README.md": """\
# robocup_env

Python package scaffold for RoboCup2D env (server/players/coach/trainer + SHM IPC).
""",

    "src/robocup2d/__init__.py": """\
from .env import Robocup2dEnv

__all__ = ["Robocup2dEnv"]
""",

    "src/robocup2d/env.py": """\
from __future__ import annotations

# 这里先放“薄壳”入口：以后你把原 Robocup2d_Python 主类搬进来即可
# 建议：env.py 只负责 reset/step/get_obs/get_state 等接口，具体实现委托给 process/ipc/protocols

from .logging_utils import setup_line_buffering, get_env_logger
from .protocols import player_shm as P, coach_shm as C, trainer_shm as T

setup_line_buffering()
ENV_LOG = get_env_logger("robocup_env")

class Robocup2dEnv:
    \"\"\"PyMARL-friendly env wrapper (placeholder).\"\"\"

    def __init__(self, cfg="robocup.yaml", **env_args):
        ENV_LOG.info("[Robocup2dEnv] init scaffold OK")
        self.cfg = cfg
        self.env_args = env_args

    def reset(self):
        raise NotImplementedError("把你原来的 reset/launch_processes 等逻辑搬进来")

    def step(self, actions):
        raise NotImplementedError

    def get_obs(self):
        raise NotImplementedError

    def get_state(self):
        raise NotImplementedError

    def close(self):
        pass
""",

    "src/robocup2d/logging_utils.py": """\
from __future__ import annotations
import sys
import logging

def setup_line_buffering():
    # 让 print 逐行立刻刷新（Python 3.7+ 支持 reconfigure）
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(line_buffering=True)

def get_env_logger(name: str = "robocup_env") -> logging.Logger:
    # 环境专用 logger，直接写 stdout、不带前缀
    log = logging.getLogger(name)
    if not log.handlers:
        h = logging.StreamHandler(sys.stdout)
        h.setFormatter(logging.Formatter("%(message)s"))
        log.propagate = False
        log.setLevel(logging.INFO)
        log.handlers = [h]
    return log
""",

    "src/robocup2d/config.py": """\
from __future__ import annotations
import os
from dataclasses import dataclass
from typing import Any, Dict

def load_yaml_cfg(cfg: Any) -> Dict[str, Any]:
    if isinstance(cfg, dict):
        return cfg
    if isinstance(cfg, str):
        base = os.getcwd()
        path = cfg if os.path.isabs(cfg) else os.path.join(base, cfg)
        import yaml
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    raise TypeError("cfg 必须是 YAML 文件路径或 dict")

def extract_env_args(root: Dict[str, Any]) -> Dict[str, Any]:
    return root.get("env_args", root) or {}
""",

    "src/robocup2d/protocols/__init__.py": """\
# protocol definitions (offsets/sizes/opcodes)
""",

    "src/robocup2d/protocols/common.py": """\
def align4(x: int) -> int:
    return (x + 3) & ~3
""",

    "src/robocup2d/protocols/player_shm.py": """\
from .common import align4

STATE_NUM = 97
BASE_MASK_NUM = 17

OFFSET_FLAG_A = 0
OFFSET_FLAG_B = 1
OFFSET_MASK   = align4(OFFSET_FLAG_B + 1)

OFFSET_CYCLE  = align4(OFFSET_MASK + BASE_MASK_NUM)
OFFSET_STATE  = align4(OFFSET_CYCLE + 4)
OFFSET_ACTION = align4(OFFSET_STATE + STATE_NUM * 4)

OFFSET_HYBRID_MASK = align4(OFFSET_ACTION + 4)
OFFSET_HYBRID_ACT  = align4(OFFSET_HYBRID_MASK + 4)
OFFSET_HYBRID_U0   = align4(OFFSET_HYBRID_ACT + 4)
OFFSET_HYBRID_U1   = align4(OFFSET_HYBRID_U0 + 4)

SHM_SIZE = align4(OFFSET_HYBRID_U1 + 4)
""",

    "src/robocup2d/protocols/coach_shm.py": """\
COACH_STATE_FLOAT = 136
COACH_SHM_SIZE = 1 + 4 + COACH_STATE_FLOAT * 4 + 4
""",

    "src/robocup2d/protocols/trainer_shm.py": """\
from .common import align4

TRAINER_SHM_SIZE = 4096  # 4KB 控制面

T_FLAG_A = 0
T_FLAG_B = 1
T_OPCODE = 4  # int32

T_BALL_X  = align4(T_OPCODE + 4)
T_BALL_Y  = T_BALL_X  + 4
T_BALL_VX = T_BALL_Y  + 4
T_BALL_VY = T_BALL_VX + 4

N_LEFT = 11
N_RIGHT = 11
PLAYER_STRIDE = 3 * 4
T_PLAYERS_BASE = align4(T_BALL_VY + 4)

def T_LPX(i): return T_PLAYERS_BASE + i*PLAYER_STRIDE + 0*4
def T_LPY(i): return T_PLAYERS_BASE + i*PLAYER_STRIDE + 1*4
def T_LPD(i): return T_PLAYERS_BASE + i*PLAYER_STRIDE + 2*4

T_R_BASE = T_PLAYERS_BASE + N_LEFT*PLAYER_STRIDE
def T_RPX(i): return T_R_BASE + i*PLAYER_STRIDE + 0*4
def T_RPY(i): return T_R_BASE + i*PLAYER_STRIDE + 1*4
def T_RPD(i): return T_R_BASE + i*PLAYER_STRIDE + 2*4

OP_RESET_FROM_PY = 10
OP_NOP = 0
""",

    "src/robocup2d/process/__init__.py": """\
# process management: ports/locks/launcher/killer/watchdog
""",

    "src/robocup2d/process/ports.py": """\
from __future__ import annotations
import socket
import time

def can_bind_all(port: int, check_ipv6: bool = True) -> bool:
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
                if af == socket.AF_INET6:
                    continue
                return False
            finally:
                try: s.close()
                except: pass
    return True

def wait_ports_free(ports, timeout=8.0, poll=0.1, hold=0.3) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if all(can_bind_all(int(p), check_ipv6=True) for p in ports):
            time.sleep(hold)
            return True
        time.sleep(poll)
    return False
""",

    "src/robocup2d/process/locks.py": """\
from __future__ import annotations
import os

# 注意：fcntl 在 Windows 不可用；如果你在 Windows 本机跑，需要改为 portalocker 等
try:
    import fcntl  # type: ignore
except Exception:
    fcntl = None

def lock_port(port: int):
    if fcntl is None:
        raise RuntimeError("fcntl 不可用（Windows）。在 Linux/WSL/服务器上用即可。")
    lock_path = f"/tmp/robocup_port_{port}.lock"
    fd = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o666)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            os.write(fd, f"pid={os.getpid()}\\n".encode())
        except Exception:
            pass
        return fd
    except BlockingIOError:
        os.close(fd)
        return None

def unlock_fds(fds):
    if fcntl is None:
        return
    for fd in fds:
        try:
            fcntl.flock(fd, fcntl.LOCK_UN)
            os.close(fd)
        except Exception:
            pass
""",

    "src/robocup2d/process/launcher.py": """\
from __future__ import annotations
import subprocess
import os

def popen(args, cwd=None, env=None, log_path=None):
    log_file = subprocess.DEVNULL
    if log_path:
        os.makedirs(os.path.dirname(log_path), exist_ok=True)
        log_file = open(log_path, "w", encoding="utf-8")
    args = list(map(str, args))
    p = subprocess.Popen(
        args, cwd=cwd, env=env,
        stdout=log_file, stderr=subprocess.STDOUT,
        start_new_session=True
    )
    return p
""",

    "src/robocup2d/process/killer.py": """\
from __future__ import annotations
import psutil
import os

def auto_kill_port(port: int):
    victims = set()
    try:
        for c in psutil.net_connections(kind='inet'):
            if c.laddr and getattr(c.laddr, "port", None) == port and c.pid:
                victims.add(c.pid)
    except Exception:
        pass

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
                p.kill()
        except Exception:
            pass
""",

    "src/robocup2d/process/watchdog.py": """\
from __future__ import annotations

def check_child_processes(processes, where=""):
    dead = []
    alive = []
    for p in processes:
        rc = p.poll()
        if rc is None:
            alive.append(p)
        else:
            dead.append((p, rc))
    return alive, dead
""",

    "src/robocup2d/ipc/__init__.py": """\
# shared memory manager + handshake
""",

    "src/robocup2d/ipc/shm_manager.py": """\
from __future__ import annotations
import time
from multiprocessing import shared_memory

def wait_for_shm(name: str, expected_size: int, retries=200, delay=0.05) -> shared_memory.SharedMemory:
    last_err = None
    for _ in range(retries):
        try:
            shm = shared_memory.SharedMemory(name=name)
            if shm.size != expected_size:
                shm.close()
                raise RuntimeError(f"shm {name} size mismatch: got {shm.size}, expected {expected_size}")
            return shm
        except FileNotFoundError as e:
            last_err = e
            time.sleep(delay)
    raise RuntimeError(f"shm {name} not found after {retries} retries (last={last_err})")

def cleanup_shm(name: str):
    try:
        shm = shared_memory.SharedMemory(name=name)
    except FileNotFoundError:
        return
    try:
        shm.close()
    except Exception:
        pass
    try:
        shm.unlink()
    except FileNotFoundError:
        pass
""",

    "src/robocup2d/ipc/handshake.py": """\
from __future__ import annotations
import time
import struct

def read_flags(buf, off_a: int, off_b: int):
    A = struct.unpack_from('B', buf, off_a)[0]
    B = struct.unpack_from('B', buf, off_b)[0]
    return A, B

def wait_ready(buf, off_a: int, off_b: int, want=(0,1), timeout=2.0, poll=0.001):
    t_end = time.time() + timeout
    while time.time() < t_end:
        if read_flags(buf, off_a, off_b) == want:
            return True
        time.sleep(poll)
    return False

def submit_req(buf, off_opcode: int, opcode: int, off_a: int, off_b: int):
    # 推荐顺序：先写 opcode，再 B=0，再 A=1
    struct.pack_into('i', buf, off_opcode, int(opcode))
    struct.pack_into('B', buf, off_b, 0)
    struct.pack_into('B', buf, off_a, 1)
""",
}

def write_file(path: Path, content: str):
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        return False
    path.write_text(content, encoding="utf-8")
    return True

def main():
    root = Path.cwd()
    created = []
    skipped = []

    for rel, content in FILES.items():
        p = root / rel
        ok = write_file(p, content)
        (created if ok else skipped).append(rel)

    print("== Scaffold done ==")
    print(f"created: {len(created)}")
    for x in created:
        print("  +", x)
    print(f"skipped (already exists): {len(skipped)}")
    for x in skipped:
        print("  =", x)

if __name__ == "__main__":
    main()
