from __future__ import annotations
import socket
import time
from typing import Iterable

def _bind_addr(host: str, port: int, af: int):
    # 把 (host, port) 变成 socket.bind() 需要的地址格式。
    if af == socket.AF_INET6:
        return (host, port, 0, 0)
    return (host, port)

def can_bind_all(port: int, check_ipv6: bool = True) -> bool:
    families = [("0.0.0.0", socket.AF_INET)]
    if check_ipv6:
        families.append(("::", socket.AF_INET6))

    for host, af in families:
        for typ in (socket.SOCK_DGRAM, socket.SOCK_STREAM):
            s = None
            try:
                s = socket.socket(af, typ)
                # 探测用：别强行 SO_REUSEADDR，更“严格”
                s.bind(_bind_addr(host, port, af))
            except (OSError, TypeError):
                # IPv6 不可用：当作忽略，而不是端口被占
                if af == socket.AF_INET6:
                    continue
                return False
            finally:
                if s is not None:
                    try:
                        s.close()
                    except Exception:
                        pass
    return True

def wait_ports_free(ports: Iterable[int], timeout: float = 8.0, poll: float = 0.1, hold: float = 0.3) -> bool:
    #给一组端口 ports，在 timeout 时间内反复检查它们是否全部空闲。

    deadline = time.time() + timeout
    ports = [int(p) for p in ports]
    while time.time() < deadline:
        if all(can_bind_all(p, check_ipv6=True) for p in ports):
            time.sleep(hold)
            return True
        time.sleep(poll)
    return False

def pick_ports(auto_start, auto_end, auto_step,
               trainer_offset, coach_offset, debug_offset,
               timeout: float = 8.0, poll: float = 0.05, hold: float = 0.05):

    auto_start, auto_end, auto_step = map(int, (auto_start, auto_end, auto_step))
    trainer_offset, coach_offset, debug_offset = map(int, (trainer_offset, coach_offset, debug_offset))

    assert auto_start < auto_end
    assert auto_step > 0
    assert 0 < trainer_offset < auto_step
    assert 0 < coach_offset   < auto_step
    assert 0 < debug_offset   < auto_step
    assert len({trainer_offset, coach_offset, debug_offset}) == 3

    for base in range(auto_start, auto_end, auto_step):
        block = range(base, base + auto_step)  # 整段保留/探测
        if wait_ports_free(block, timeout=timeout, poll=poll, hold=hold):
            server  = base
            trainer = base + trainer_offset
            coach   = base + coach_offset
            debug   = base + debug_offset
            return base, server, trainer, coach, debug

    raise RuntimeError("no free port block found")
