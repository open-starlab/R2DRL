import numpy as np
import os
import collections
from os.path import dirname, abspath
from copy import deepcopy
import sys
import torch as th
from utils.logging import get_logger
import yaml

from run import run

logger = get_logger()
results_path = os.path.join(dirname(dirname(abspath(__file__))), "results")

def _get_config(params, arg_name, subfolder):
    config_name = None
    for _i, _v in enumerate(params):
        if _v.split("=")[0] == arg_name:
            config_name = _v.split("=")[1]
            del params[_i]
            break

    if config_name is not None:
        if subfolder == "envs":
            candidate_paths = [
                abspath(
                    os.path.join(
                        os.path.dirname(__file__),
                        "envs",
                        "robocup2d",
                        "config",
                        "{}.yaml".format(config_name),
                    )
                ),
                abspath(
                    os.path.join(
                        os.path.dirname(__file__),
                        "config",
                        subfolder,
                        "{}.yaml".format(config_name),
                    )
                ),
            ]
            config_path = next((p for p in candidate_paths if os.path.isfile(p)), None)
            if config_path is None:
                raise RuntimeError(
                    "Unknown env config '{}'. Searched: {}".format(
                        config_name, ", ".join(candidate_paths)
                    )
                )
        else:
            config_path = abspath(
                os.path.join(
                    os.path.dirname(__file__), "config", subfolder, "{}.yaml".format(config_name)
                )
            )
        with open(config_path, "r") as f:
            try:
                config_dict = yaml.safe_load(f)
            except yaml.YAMLError as exc:
                raise RuntimeError("{} .yaml error: {}".format(config_name, exc))
        if subfolder == "envs":
            config_dict.setdefault("env_args", {})["_pymarl_env_config_source"] = config_path
        return config_dict
    return {}


def recursive_dict_update(d, u):
    for k, v in u.items():
        if isinstance(v, collections.abc.Mapping):
            d[k] = recursive_dict_update(d.get(k, {}), v)
        else:
            d[k] = v
    return d


def _parse_cli_value(value):
    try:
        return yaml.safe_load(value)
    except yaml.YAMLError:
        return value


ENV_ARG_ALIASES = {
    "useMaxEpv": "env_args.useMaxEpv",
    "use_epv_reward": "env_args.useMaxEpv",
    "use-epv-reward": "env_args.useMaxEpv",
    "epv_grid_file": "env_args.epv_grid_file",
    "epv-grid-file": "env_args.epv_grid_file",
    "epv_progress_scale": "env_args.epv_progress_scale",
    "epv-progress-scale": "env_args.epv_progress_scale",
    "init_n": "env_args.init_n",
    "init-n": "env_args.init_n",
    "robocup_init_n": "env_args.init_n",
    "robocup-init-n": "env_args.init_n",
    "agent_mask_n": "env_args.init_n",
    "agent-mask-n": "env_args.init_n",
}


def _set_config_value(config, key, value):
    target = config
    parts = key.split(".")
    for part in parts[:-1]:
        if part not in target:
            target[part] = {}
        if not isinstance(target[part], dict):
            raise RuntimeError("Cannot override nested key '{}'".format(key))
        target = target[part]
    target[parts[-1]] = value


def apply_cli_overrides(config, params):
    ignored_keys = {"config", "env-config", "capture"}
    i = 1
    while i < len(params):
        token = params[i]
        if token == "with":
            i += 1
            continue

        key = None
        raw_value = None
        if token.startswith("--") and "=" in token:
            key, raw_value = token[2:].split("=", 1)
            i += 1
        elif token.startswith("--") and i + 1 < len(params):
            key = token[2:]
            raw_value = params[i + 1]
            i += 2
        elif "=" in token and not token.startswith("-"):
            key, raw_value = token.split("=", 1)
            i += 1
        else:
            i += 1
            continue

        if key in ignored_keys:
            continue
        key = ENV_ARG_ALIASES.get(key, key)
        _set_config_value(config, key, _parse_cli_value(raw_value))


def config_copy(config):
    if isinstance(config, dict):
        return {k: config_copy(v) for k, v in config.items()}
    elif isinstance(config, list):
        return [config_copy(v) for v in config]
    else:
        return deepcopy(config)

if __name__ == '__main__':
    params = deepcopy(sys.argv)

    # Get the defaults from default.yaml
    with open(os.path.join(os.path.dirname(__file__), "config", "default.yaml"), "r") as f:
        try:
            config_dict = yaml.safe_load(f)
        except yaml.YAMLError as exc:
            raise RuntimeError("default.yaml error: {}".format(exc))

    # Load algorithm and env base configs
    env_config = _get_config(params, "--env-config", "envs")
    alg_config = _get_config(params, "--config", "algs")

    config_dict = recursive_dict_update(config_dict, env_config)
    config_dict = recursive_dict_update(config_dict, alg_config)
    apply_cli_overrides(config_dict, params)

    # Keep the top-level and env seeds aligned, while allowing env-config YAML
    # to override the default top-level seed.
    env_seed = config_dict.get("env_args", {}).get("seed")
    if env_seed is not None:
        config_dict["seed"] = env_seed
    else:
        config_dict.setdefault("env_args", {})["seed"] = config_dict["seed"]

    # 设置随机种子
    np.random.seed(config_dict["seed"])
    th.manual_seed(config_dict["seed"])
    config_dict["env_args"]["seed"] = config_dict["seed"]

    # 直接运行
    logger.info("Start running with config: {}".format(config_dict))
    run(config_dict)
