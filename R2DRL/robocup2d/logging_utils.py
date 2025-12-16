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
