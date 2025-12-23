from __future__ import annotations
from pathlib import Path
from typing import Any, Dict


def load_yaml_cfg(cfg: Any) -> Dict[str, Any]:
    # 1) If already a dict, return as is
    if isinstance(cfg, dict):
        return cfg

    # 2) If string, treat as yaml file path
    if isinstance(cfg, str):
        cfg_path = Path(cfg)

        if cfg_path.is_absolute():
            # If absolute path, use directly
            path = cfg_path
        else:
            # Key: resolve relative to this file's directory (envs/robocup2d)
            here = Path(__file__).resolve().parent
            path = (here / cfg_path).resolve()

        print(f"[config] loading yaml: {path}")
        import yaml
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}

    raise TypeError("cfg must be a YAML file path or dict")


def extract_env_args(root: Dict[str, Any]) -> Dict[str, Any]:
    # Extract 'env_args' if present, otherwise return the root dict
    return root.get("env_args", root) or {}


def load_env_args(cfg: Any, overrides: Dict[str, Any]) -> Dict[str, Any]:
    root = load_yaml_cfg(cfg)
    args = dict(extract_env_args(root))
    # Allow kwargs to override yaml values
    args.update(overrides or {})
    return args
