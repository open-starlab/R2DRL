# /fsws1/h_qin/robocup/robocup/pymarl/src/envs/robocup2d/config.py
from __future__ import annotations
from pathlib import Path
from typing import Any, Dict


def load_yaml_cfg(cfg: Any) -> Dict[str, Any]:
    # 1) 已经是 dict 就直接用
    if isinstance(cfg, dict):
        return cfg

    # 2) 字符串：当成 yaml 文件名
    if isinstance(cfg, str):
        cfg_path = Path(cfg)

        if cfg_path.is_absolute():
            # 如果传进来的是绝对路径，就直接用
            path = cfg_path
        else:
            # ⭐ 关键改动：相对本文件所在目录（envs/robocup2d）
            here = Path(__file__).resolve().parent
            path = (here / cfg_path).resolve()

        print(f"[config] loading yaml: {path}")
        import yaml
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}

    raise TypeError("cfg 必须是 YAML 文件路径或 dict")


def extract_env_args(root: Dict[str, Any]) -> Dict[str, Any]:
    return root.get("env_args", root) or {}


def load_env_args(cfg: Any, overrides: Dict[str, Any]) -> Dict[str, Any]:
    root = load_yaml_cfg(cfg)
    args = dict(extract_env_args(root))
    args.update(overrides or {})  # kwargs 覆盖 yaml
    return args
