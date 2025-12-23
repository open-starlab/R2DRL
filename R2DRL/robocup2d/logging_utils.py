from __future__ import annotations
import sys
import logging


def get_env_logger(name: str = "robocup_env") -> logging.Logger:
    # Environment-specific logger, writes directly to stdout without prefix
    log = logging.getLogger(name)
    if not log.handlers:
        h = logging.StreamHandler(sys.stdout)
        h.setFormatter(logging.Formatter("%(message)s"))
        log.propagate = False
        log.setLevel(logging.INFO)
        log.handlers = [h]
    return log
