from __future__ import annotations
import os
import psutil
from typing import Iterable, Optional, Sequence, Set
import signal
import time

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

def _same_real_uid(proc: psutil.Process, uid: int) -> bool:
    try:
        return int(proc.uids().real) == int(uid)
    except Exception:
        return False

def find_pids_by_run_id(
    run_id: str,
    *,
    env_key: str = "ROBOCUP2DRL_RUN_ID",
    log=None,
) -> Set[int]:
    """
    Find same-user processes whose inherited environment marks them as part
    of this RoboCup session. This catches children that outlive our Popen
    handles after a restart timeout or Slurm signal.
    """
    run_id = str(run_id or "")
    if not run_id:
        return set()

    my_pid = os.getpid()
    my_uid = os.getuid()
    pids: Set[int] = set()

    for proc in psutil.process_iter(["pid", "name"]):
        try:
            pid = int(proc.pid)
            if pid == my_pid:
                continue
            if not _same_real_uid(proc, my_uid):
                continue
            if _get_env_value(pid, env_key) == run_id:
                pids.add(pid)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
        except Exception as exc:
            if log:
                try:
                    log.info(f"[kill_run_id] inspect skip pid={getattr(proc, 'pid', '?')} err={exc!r}")
                except Exception:
                    pass

    return pids

def kill_run_processes_by_run_id(
    run_id: str,
    *,
    env_key: str = "ROBOCUP2DRL_RUN_ID",
    sigterm_timeout: float = 2.0,
    log=None,
) -> Set[int]:
    """
    Terminate all same-user processes that belong to a RoboCup session.

    The Popen-based teardown is the normal path. This is the stronger sweep
    for restart/scancel cases where a child process escaped the original
    handles but still carries ROBOCUP2DRL_RUN_ID in /proc/<pid>/environ.
    """
    pids = find_pids_by_run_id(run_id, env_key=env_key, log=log)
    if not pids:
        return set()

    protect_pgid = os.getpgrp()
    pgids: Set[int] = set()
    procs = []

    for pid in sorted(pids):
        try:
            proc = psutil.Process(pid)
            procs.append(proc)
            pgid = os.getpgid(pid)
            if int(pgid) != int(protect_pgid):
                pgids.add(int(pgid))
        except (psutil.NoSuchProcess, ProcessLookupError):
            continue
        except Exception:
            continue

    if log:
        try:
            log.info(
                f"[kill_run_id] run_id={run_id} pids={sorted(pids)} pgids={sorted(pgids)}"
            )
        except Exception:
            pass

    kill_run_process_groups(signal.SIGTERM, pgids, pids, log=log, protect_pgid=protect_pgid)

    deadline = time.time() + float(sigterm_timeout)
    for proc in list(procs):
        try:
            proc.wait(timeout=max(0.0, deadline - time.time()))
        except psutil.TimeoutExpired:
            pass
        except Exception:
            pass

    survivors = find_pids_by_run_id(run_id, env_key=env_key, log=log)
    if survivors:
        survivor_pgids: Set[int] = set()
        for pid in sorted(survivors):
            try:
                pgid = os.getpgid(pid)
                if int(pgid) != int(protect_pgid):
                    survivor_pgids.add(int(pgid))
            except Exception:
                pass
        kill_run_process_groups(
            signal.SIGKILL,
            survivor_pgids,
            survivors,
            log=log,
            protect_pgid=protect_pgid,
        )
        for pid in sorted(survivors):
            try:
                psutil.Process(pid).kill()
            except Exception:
                pass

    return pids | survivors

def kill_run_ports_by_run_id(
    ports: Sequence[int],
    *,
    run_id: str,
    env_key: str = "ROBOCUP2DRL_RUN_ID",
    sigterm_timeout: float = 1.0,
    log=None,
) -> Set[int]:
    killed: Set[int] = set()
    for port in ports or ():
        try:
            killed.update(
                kill_port_by_run_id(
                    int(port),
                    run_id=run_id,
                    env_key=env_key,
                    sigterm_timeout=sigterm_timeout,
                    log=log,
                )
            )
        except Exception:
            pass
    return killed

def kill_run_process_groups(
    sig: int,
    run_pgids: Set[int],
    run_pids: Set[int],
    *,
    log=None,
    protect_pgid: Optional[int] = None,
) -> None:
    """
    Kill process groups in run_pgids with signal sig, but only if:
      - pgid != protect_pgid (default: current python pgid)
      - at least one alive pid in run_pids is still a member of that pgid

    Notes:
      - Uses os.killpg => works on POSIX (Linux/macOS). Not for native Windows.
    """
    if protect_pgid is None:
        protect_pgid = os.getpgrp()

    pgids = set(run_pgids or [])
    pids = set(run_pids or [])

    for pgid in sorted(pgids):
        if pgid == protect_pgid:
            continue

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
            continue

        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            pass
        except Exception as e:
            if log is not None:
                log.warning(f"[teardown] killpg failed pgid={pgid} sig={sig} err={e}")



def kill_current_procs(
    procs: Iterable,
    *,
    log=None,
    sigterm_wait: float = 2.0,
) -> None:
    """
    Gracefully terminate then force kill process groups
    belonging to the given process list.

    This function:
        - sends SIGTERM to process groups
        - waits up to `sigterm_wait`
        - sends SIGKILL if still alive
    """

    popens = []

    # --- unwrap possible wrapper objects ---
    for item in procs:
        if item is None:
            continue
        p = getattr(item, "p", item)  # support wrapper.p or raw Popen
        if p is not None:
            popens.append(p)

    py_pgid = os.getpgrp()
    run_pids: Set[int] = set()
    run_pgids: Set[int] = set()

    # --- collect alive pid / pgid ---
    for p in popens:
        try:
            if p.poll() is None:
                pid = int(p.pid)
                run_pids.add(pid)

                try:
                    pgid = os.getpgid(pid)
                    if pgid != py_pgid:
                        run_pgids.add(pgid)
                except Exception:
                    pass
        except Exception:
            continue

    # --- SIGTERM ---
    for pgid in run_pgids:
        try:
            if log:
                log.info(f"[killer] SIGTERM pgid={pgid}")
            os.killpg(pgid, signal.SIGTERM)
        except Exception:
            pass

    # --- wait ---
    t_end = time.time() + float(sigterm_wait)
    for p in popens:
        try:
            if p.poll() is None:
                p.wait(timeout=max(0.0, t_end - time.time()))
        except Exception:
            pass

    # --- SIGKILL ---
    for pgid in run_pgids:
        try:
            if log:
                log.info(f"[killer] SIGKILL pgid={pgid}")
            os.killpg(pgid, signal.SIGKILL)
        except Exception:
            pass