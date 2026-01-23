from __future__ import annotations
import socket
import errno
import time
from typing import Iterable
import os
import fcntl
from typing import Optional, Tuple

def _bind_addr(host: str, port: int, af: int):
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

                # Key: When probing IPv6, try to avoid v4-mapped interference
                if af == socket.AF_INET6:
                    try:
                        s.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 1)
                    except OSError:
                        pass

                s.bind(_bind_addr(host, port, af))

            except TypeError:
                # IPv6 address family not available, etc.
                if af == socket.AF_INET6:
                    continue
                return False

            except OSError as e:
                if af == socket.AF_INET6:
                    # Only ignore "IPv6 not supported/unavailable", do not ignore "address in use"
                    if e.errno in (errno.EAFNOSUPPORT, errno.EPROTONOSUPPORT, errno.EADDRNOTAVAIL):
                        continue
                    return False
                return False

            finally:
                if s is not None:
                    try: 
                        s.close()
                    except Exception: 
                        pass

    return True

def wait_ports_free(ports: Iterable[int], timeout: float = 8.0, poll: float = 0.1, hold: float = 0.3) -> bool:
    # For a set of ports, repeatedly check if all are free within the timeout period.

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
        fd, path = try_lock_port_block(base)
        if fd is None:
            continue  # This block is already occupied by another task (host-level mutual exclusion)

        block = range(base, base + auto_step)
        if wait_ports_free(block, timeout=timeout, poll=poll, hold=hold):
            server  = base
            trainer = base + trainer_offset
            coach   = base + coach_offset
            debug   = base + debug_offset
            return base, server, trainer, coach, debug, fd, path

        # Ports are not free, release the lock and try the next block
        try:
            os.close(fd)
        except Exception:
            pass

    raise RuntimeError("no free port block found")

def try_lock_port_block(base: int, prefix: str = "robocup2drl") -> Tuple[Optional[int], str]:
    path = f"/tmp/{prefix}_portblock_{int(base)}.lock"
    fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o644)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return fd, path
    except BlockingIOError:
        os.close(fd)
        return None, path