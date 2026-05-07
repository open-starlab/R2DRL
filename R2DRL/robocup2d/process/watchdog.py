from __future__ import annotations
from typing import Any, List, Tuple


def check_child_processes(processes, where: str = "") -> Tuple[List[Any], List[Tuple[Any, Any]], str]:
    """
    Returns:
        alive: list of alive info objects (original objects preserved)
        dead : [(info, rc), ...] (dead process info and return code)
        dead_info_str: formatted multi-line string for logging dead processes
    """
    dead: List[Tuple[Any, Any]] = []
    alive: List[Any] = []

    lines: List[str] = []

    for info in list(processes):
        p = info.p if hasattr(info, "p") else info

        try:
            rc = p.poll()
        except Exception as e:
            rc = f"poll_error:{e!r}"

        if rc is None:
            alive.append(info)
            continue

        dead.append((info, rc))

        pid = getattr(p, "pid", None)
        if hasattr(info, "p"):  # ProcInfo
            kind = getattr(info, "kind", "proc")
            team = getattr(info, "team", "")
            unum = getattr(info, "unum", "")
            shm  = getattr(info, "shm_name", "")
            log  = getattr(info, "log_path", "")
            lines.append(
                f"[{where}][{kind}] pid={pid} rc={rc} team={team} unum={unum} shm={shm} log={log}"
            )
        else:  # Raw Popen (server/trainer/coach)
            lines.append(
                f"[{where}][server/trainer/coach] pid={pid} rc={rc}"
            )

    dead_info_str = "\n".join(lines)
    return alive, dead, dead_info_str
