from __future__ import annotations
import os
import psutil
from typing import Optional, Set

def _get_env_value(pid: int, key: str) -> Optional[str]:
    """
    Read /proc/<pid>/environ for Linux, return the value of the given environment variable key if present.
    psutil does not provide a stable cross-platform API for reading env, so reading /proc is more reliable here.
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

    # 1) Find pids occupying the port (could be TCP/UDP)
    try:
        for c in psutil.net_connections(kind="inet"):
            if not c.pid:
                continue
            if c.laddr and getattr(c.laddr, "port", None) == port:
                victims.add(int(c.pid))
    except Exception:
        pass

    killed: Set[int] = set()

    # 2) Filter and kill
    for pid in sorted(victims):
        if pid == my_pid:
            continue

        # Only operate on processes of the same user (extra safety)
        try:
            p = psutil.Process(pid)
            if p.uids().real != my_uid:
                continue
        except Exception:
            continue

        # Key: Only kill processes with the same run_id
        v = _get_env_value(pid, env_key)
        if v != str(run_id):
            if log:
                try:
                    log.info(f"[kill_port] skip pid={pid} port={port} {env_key}={v}")
                except Exception:
                    pass
            continue

        # Kill process (graceful → forceful)
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
