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

        import yaml
        with open(path, "r", encoding="utf-8") as f:
            root = yaml.safe_load(f) or {}
        root["_env_default_yaml_source"] = str(path)
        return root

    raise TypeError("cfg must be a YAML file path or dict")

def extract_env_args(root: Dict[str, Any]) -> Dict[str, Any]:
    # Extract 'env_args' if present, otherwise return the root dict
    return root.get("env_args", root) or {}

def _deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    merged = dict(base)
    for key, val in override.items():
        if isinstance(merged.get(key), dict) and isinstance(val, dict):
            merged[key] = _deep_merge(merged[key], val)
        else:
            merged[key] = val
    return merged

def load_env_args(cfg: Any, overrides: Dict[str, Any]) -> Dict[str, Any]:
    root = load_yaml_cfg(cfg)
    args = dict(extract_env_args(root))
    # Allow kwargs to override yaml values
    args.update(overrides or {})
    pymarl_source = args.get("_pymarl_env_config_source")

    if pymarl_source:
        print(f"[config] PyMARL env-config yaml: {pymarl_source}")

    summary_keys = [
        "n", "init_n", "opponent_level", "episode_limit", "seed",
        "benchmark_mode", "opponent_state", "scenario_start",
        "use_custom_start", "useMaxEpv", "game_logging", "text_logging",
        "terminate_on_goal", "terminate_on_possession_loss",
        "reward_on_possession_loss", "trajectory_path", "scenario_difficulty",
        "scenario_difficulty_buckets",
    ]
    summary = ", ".join(
        f"{key}={args[key]!r}" for key in summary_keys if key in args
    )
    if summary:
        print(f"[config] final env args summary: {summary}")
    return args
