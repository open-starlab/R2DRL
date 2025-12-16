from __future__ import annotations
import os
from multiprocessing import shared_memory
from typing import List
from .locks import extract_run_id_from_devshm_filename, is_run_active

def list_project_shm(prefix: str = "robocup2drl_") -> List[str]:
    try:
        return [fn for fn in os.listdir("/dev/shm") if fn.startswith(prefix)]
    except (FileNotFoundError, PermissionError):
        return []

def safe_cleanup_project_shm(
    prefix: str = "robocup2drl_",
    *,
    dry_run: bool = True,
    log=None,
) -> List[str]:
    deleted: List[str] = []
    for fn in list_project_shm(prefix):
        run_id = extract_run_id_from_devshm_filename(fn)
        if run_id is None:
            if log: log.info(f"[shm] skip (bad name) /{fn}")
            continue

        if is_run_active(run_id):
            if log: log.info(f"[shm] keep (run active) run_id={run_id} /{fn}")
            continue

        name = "/" + fn
        if dry_run:
            if log: log.info(f"[shm] would delete {name} (run_id={run_id})")
            deleted.append(name)
            continue

        # race guard: re-check right before unlink
        if is_run_active(run_id):
            if log: log.info(f"[shm] keep (became active) run_id={run_id} /{fn}")
            continue

        try:
            shm = shared_memory.SharedMemory(name=name, create=False)
        except FileNotFoundError:
            continue
        except Exception as e:
            if log: log.info(f"[shm] attach failed {name}: {e!r}")
            continue

        try:
            try:
                shm.close()
            except Exception:
                pass
            try:
                shm.unlink()
            except FileNotFoundError:
                pass
            if log: log.info(f"[shm] deleted {name} (run_id={run_id})")
            deleted.append(name)
        except Exception as e:
            if log: log.info(f"[shm] delete failed {name}: {e!r}")

    return deleted
