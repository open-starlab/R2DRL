from __future__ import annotations
import os
import fcntl
import re
from typing import Optional

# ----------------------------
# create and release run lock
# ----------------------------

RUN_RE = re.compile(r"^robocup2drl_([0-9a-f]{32})_")  # /dev/shm 里文件名不带前导 /

def acquire_run_lock(run_id: str, *, log=None) -> int:
    path = f"/tmp/robocup2drl_{run_id}.lock"
    try:
        fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o600)
    except OSError as e:
        raise RuntimeError(f"[lock] cannot open lock file: {path} ({e!r})")

    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        try:
            os.close(fd)
        except Exception:
            pass
        raise RuntimeError(f"[lock] run_id already active: {run_id}")

    # 可选：写入 pid，方便你之后 cat 这个文件看是谁拿着锁
    try:
        os.ftruncate(fd, 0)
        os.write(fd, f"pid={os.getpid()}\n".encode())
    except Exception:
        pass

    if log:
        log.info(f"[lock] acquired {path}")
    return fd  # 不要 close，fd 活着锁就一直在


def release_run_lock(run_id: str, fd: int, *, unlink_file: bool = True, log=None) -> None:
    path = f"/tmp/robocup2drl_{run_id}.lock"

    # 1) unlock (best-effort)
    try:
        fcntl.flock(fd, fcntl.LOCK_UN)
    except Exception:
        pass

    # 2) close fd (best-effort)
    try:
        os.close(fd)
    except Exception:
        pass

    # 3) delete lock file (optional, best-effort)
    if unlink_file:
        try:
            os.unlink(path)
        except FileNotFoundError:
            pass
        except Exception:
            pass

    if log:
        log.info(f"[lock] released run_id={run_id} unlink_file={unlink_file} path={path}")


def extract_run_id_from_devshm_filename(fn: str) -> Optional[str]:
    m = RUN_RE.match(fn)
    return m.group(1) if m else None

def is_run_active(run_id: str) -> bool:
    path = f"/tmp/robocup2drl_{run_id}.lock"
    if not os.path.exists(path):
        return False
    try:
        fd = os.open(path, os.O_RDWR)
    except OSError:
        return True  # 保守：打不开锁文件就别删
    try:
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.flock(fd, fcntl.LOCK_UN)
            return False  # 能抢到锁 => 没活实例持有
        except BlockingIOError:
            return True   # 抢不到 => 有活实例持有
    finally:
        try:
            os.close(fd)
        except Exception:
            pass
