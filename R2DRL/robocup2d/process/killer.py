from __future__ import annotations
import os, psutil
from typing import Iterable, Optional, Set

def _get_env_value(pid: int, key: str) -> Optional[str]:
    """
    Read /proc/<pid>/environ for linux, return env value of key if present.
    psutil does没有稳定跨平台读 env 的 API，这里直接读 /proc 更稳。
    """
    try:
        with open(f"/proc/{int(pid)}/environ", "rb") as f:
            data = f.read()
    except Exception:
        return None

    prefix = (key + "=").encode()
    for item in data.split(b"\0"):
        if item.startswith(prefix):
            try:
                return item[len(prefix):].decode(errors="ignore")
            except Exception:
                return None
    return None

def kill_port_by_run_id(
    port: int,
    *,
    run_id: str,
    env_key: str = "ROBOCUP2DRL_RUN_ID",
    sigterm_timeout: float = 1.0,
    log=None,
) -> Set[int]:
    """
    Kill processes occupying `port`, but ONLY those whose environ contains:
        ROBOCUP2DRL_RUN_ID == run_id

    Returns a set of pids that were killed (or attempted).
    """
    port = int(port)
    my_pid = os.getpid()
    my_uid = os.getuid()

    victims: Set[int] = set()

    # 1) 找到占用该端口的 pid（TCP/UDP 都可能出现）
    try:
        for c in psutil.net_connections(kind="inet"):
            if not c.pid:
                continue
            if c.laddr and getattr(c.laddr, "port", None) == port:
                victims.add(int(c.pid))
    except Exception:
        pass

    killed: Set[int] = set()

    # 2) 过滤并杀
    for pid in sorted(victims):
        if pid == my_pid:
            continue

        # 只操作同一用户（额外安全网）
        try:
            p = psutil.Process(pid)
            if p.uids().real != my_uid:
                continue
        except Exception:
            continue

        # 关键：只杀带同 run_id 的
        v = _get_env_value(pid, env_key)
        if v != str(run_id):
            if log:
                try:
                    log.info(f"[kill_port] skip pid={pid} port={port} {env_key}={v}")
                except Exception:
                    pass
            continue

        # 杀进程（优雅→强制）
        try:
            name = ""
            try:
                name = p.name()
            except Exception:
                pass

            if log:
                log.info(f"[kill_port] port={port} kill pid={pid} name={name} {env_key}={v}")

            try:
                p.terminate()
                p.wait(timeout=float(sigterm_timeout))
            except Exception:
                p.kill()

            killed.add(pid)
        except Exception:
            pass

    return killed

def auto_kill_port(port: int, *, allow_names=None, log=None):
    allow_names = set(n.lower() for n in (allow_names or []))
    my_pid = os.getpid()
    my_uid = os.getuid()

    victims = set()
    try:
        for c in psutil.net_connections(kind="inet"):
            if c.laddr and getattr(c.laddr, "port", None) == port and c.pid:
                victims.add(c.pid)
    except Exception:
        pass

    for pid in victims:
        if pid == my_pid:
            continue
        try:
            p = psutil.Process(pid)

            # 过滤：只杀自己用户
            try:
                if p.uids().real != my_uid:
                    continue
            except Exception:
                continue

            # 过滤：只杀指定进程名（你可以传 allow_names=None 来跳过这个过滤）
            if allow_names:
                try:
                    name = (p.name() or "").lower()
                    if name not in allow_names:
                        continue
                except Exception:
                    continue

            if log: log.info(f"[kill_port] port={port} kill pid={pid} name={p.name()}")
            try:
                p.terminate()
                p.wait(timeout=1.0)
            except Exception:
                p.kill()
        except Exception:
            pass
