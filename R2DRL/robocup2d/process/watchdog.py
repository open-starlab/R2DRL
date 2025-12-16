from __future__ import annotations
from typing import List, Tuple

def check_child_processes(processes, where: str = ""):
    dead = []
    alive = []
    
    for info in list(processes):
        # ----------- 支持 ProcInfo / Popen 两种类型 ------------
        if hasattr(info, "p"):        # ProcInfo
            p = info.p
        else:                         # subprocess.Popen
            p = info
        
        try:
            rc = p.poll()
        except Exception as e:
            # p 不是 Popen 或 poll() 出错
            dead.append((info, f"poll_error:{e!r}"))
            continue

        if rc is None:
            alive.append(info)
        else:
            dead.append((info, rc))

    return alive, dead
