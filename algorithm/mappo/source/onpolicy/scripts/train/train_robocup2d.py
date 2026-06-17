#!/usr/bin/env python
from __future__ import annotations

import os
import json
from pathlib import Path
import socket
import sys
import time

import numpy as np
import setproctitle
import torch
import wandb

from onpolicy.config import get_config
from onpolicy.envs.env_wrappers import SubprocVecEnv, DummyVecEnv
from onpolicy.envs.robocup2d import RoboCup2DEnv


REPO_ROOT = Path(__file__).resolve().parents[6]
DEFAULT_ENV_CONFIG = str(
    REPO_ROOT
    / "algorithm/qmix/source/envs/robocup2d/config"
    / "parallelr2drl_11vs11scenario_catalog_front-goal-2v1_opp-lv3_epv-on.yaml"
)
DEFAULT_PYMARL_SRC = str(REPO_ROOT / "algorithm/qmix/source")


def make_train_env(all_args):
    def get_env_fn(rank):
        def init_env():
            env = RoboCup2DEnv(all_args, rank=rank, is_eval=False)
            env.seed(all_args.seed + rank * 1000)
            return env
        return init_env

    if all_args.n_rollout_threads == 1:
        return DummyVecEnv([get_env_fn(0)])
    return SubprocVecEnv([get_env_fn(i) for i in range(all_args.n_rollout_threads)])


def make_eval_env(all_args):
    def get_env_fn(rank):
        def init_env():
            env = RoboCup2DEnv(all_args, rank=rank, is_eval=True)
            env.seed(all_args.seed * 50000 + rank * 10000)
            return env
        return init_env

    if all_args.n_eval_rollout_threads == 1:
        return DummyVecEnv([get_env_fn(0)])
    return SubprocVecEnv([get_env_fn(i) for i in range(all_args.n_eval_rollout_threads)])


def _truthy(value):
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _robocup_env_overrides(all_args):
    overrides = {}
    init_n = getattr(all_args, "robocup_init_n", None)
    if init_n is not None:
        overrides["init_n"] = int(init_n)
    use_epv_reward = getattr(all_args, "robocup_use_epv_reward", None)
    if use_epv_reward is not None:
        overrides["useMaxEpv"] = _truthy(use_epv_reward)
    use_action_mask = getattr(all_args, "robocup_use_action_mask", None)
    if use_action_mask is not None:
        overrides["use_action_mask"] = _truthy(use_action_mask)
    epv_grid_file = getattr(all_args, "robocup_epv_grid_file", None)
    if epv_grid_file:
        overrides["epv_grid_file"] = str(epv_grid_file)
    epv_progress_scale = getattr(all_args, "robocup_epv_progress_scale", None)
    if epv_progress_scale is not None:
        overrides["epv_progress_scale"] = float(epv_progress_scale)
    return overrides


def warmup_robocup_launch(all_args):
    if not _truthy(getattr(all_args, "robocup_warmup_launch", False)):
        return
    env = None
    try:
        print("[INFO] RoboCup2D warmup launch start", flush=True)
        env = RoboCup2DEnv(all_args, rank=0, is_eval=False)
    finally:
        if env is not None:
            env.close()
        pause = max(0.0, float(getattr(all_args, "robocup_warmup_pause_seconds", 0.0) or 0.0))
        if pause > 0.0:
            time.sleep(pause)
        print("[INFO] RoboCup2D warmup launch done", flush=True)


def _infer_num_agents_and_episode_length(all_args):
    pymarl_src = Path(all_args.pymarl_src).expanduser().resolve()
    if str(pymarl_src) not in sys.path:
        sys.path.insert(0, str(pymarl_src))
    from envs.robocup2d.config.loader import load_env_args
    from envs.robocup2d.config.schema import EnvConfig
    from envs.robocup2d.start_sampler import apply_catalog_launch_profile

    env_overrides = _robocup_env_overrides(all_args)
    env_args = load_env_args(all_args.robocup_env_config, env_overrides)
    apply_catalog_launch_profile(env_args)
    cfg = EnvConfig(env_args)
    all_args.robocup_env_overrides = dict(env_overrides)
    all_args.robocup_env_args = dict(env_args)
    all_args.train_players = int(getattr(cfg, "init_n", cfg.n1))
    all_args.total_players_per_side = int(getattr(cfg, "n", cfg.n1))
    all_args.opponent_players = len(getattr(cfg, "active_right_unums", []) or [])
    all_args.num_agents = int(getattr(cfg, "init_n", cfg.n1))
    all_args.episode_length = int(cfg.episode_limit)
    if not all_args.scenario_name:
        all_args.scenario_name = Path(all_args.robocup_env_config).stem
    return all_args


def parse_args(args, parser):
    parser.add_argument("--robocup_env_config", type=str, default=DEFAULT_ENV_CONFIG,
                        help="Absolute path or PyMARL env yaml name for RoboCup2D.")
    parser.add_argument("--robocup_init_n", "--robocup-init-n", "--init_n", "--init-n",
                        "--agent_mask_n", "--agent-mask-n", dest="robocup_init_n",
                        type=int, default=None,
                        help="Override env init_n, i.e. how many nearest left-side agents the agent mask controls.")
    parser.add_argument("--robocup_use_epv_reward", "--robocup-use-epv-reward", dest="robocup_use_epv_reward",
                        type=str, default=None,
                        help="Override RoboCup2D useMaxEpv. Use true/false to enable or disable max-EPV reward.")
    parser.add_argument("--robocup_use_action_mask", "--robocup-use-action-mask", "--use_action_mask", "--use-action-mask",
                        dest="robocup_use_action_mask", type=str, default=None,
                        help="Override RoboCup2D use_action_mask. Use true/false to enable or disable legal-action masks.")
    parser.add_argument("--robocup_no_action_mask", "--robocup-no-action-mask", "--no-use-action-mask", "--no_use_action_mask",
                        dest="robocup_use_action_mask", action="store_const", const="False",
                        help="Disable RoboCup2D legal-action masks regardless of the yaml.")
    parser.add_argument("--robocup_epv_grid_file", "--robocup-epv-grid-file", dest="robocup_epv_grid_file",
                        type=str, default=None,
                        help="Override RoboCup2D epv_grid_file, e.g. EPV_grid_front_goal_linear.csv.")
    parser.add_argument("--robocup_epv_progress_scale", "--robocup-epv-progress-scale", dest="robocup_epv_progress_scale",
                        type=float, default=None,
                        help="Override RoboCup2D epv_progress_scale.")
    parser.add_argument("--pymarl_src", type=str, default=DEFAULT_PYMARL_SRC,
                        help="Path to the PyMARL src directory containing envs.robocup2d.")
    parser.add_argument("--scenario_name", type=str, default="",
                        help="Name used for result grouping. Defaults to yaml stem.")
    parser.add_argument("--num_agents", type=int, default=None,
                        help="Inferred from the RoboCup2D yaml by default.")
    parser.add_argument("--robocup_obs_agent_id", "--robocup-obs-agent-id", dest="robocup_obs_agent_id",
                        type=str, default="True",
                        help="Append a one-hot agent id to each controlled agent observation. Enabled by default to match QMIX obs_agent_id.")
    parser.add_argument("--robocup_no_obs_agent_id", "--robocup-no-obs-agent-id", "--no-robocup-obs-agent-id",
                        dest="robocup_obs_agent_id", action="store_const", const="False",
                        help="Disable one-hot agent id observation augmentation for old checkpoints/ablations.")
    parser.add_argument("--share_reward", action="store_false", default=True,
                        help="RoboCup2D currently emits one shared team reward.")
    parser.add_argument("--eval_deterministic", action="store_false", default=True,
                        help="If false, sample actions during eval.")
    parser.add_argument("--evaluate_only", "--evaluate-only", action="store_true",
                        help="Load --model_dir and run evaluation only without training.")
    parser.add_argument("--evaluate_random_init", "--evaluate-random-init", action="store_true",
                        help="With --evaluate_only, evaluate the seed-initialized policy without loading --model_dir.")
    parser.add_argument("--eval_output_json", type=str, default=None,
                        help="Optional path to write evaluate-only metrics as JSON.")
    parser.add_argument("--eval_checkpoint_step", type=int, default=0,
                        help="Optional checkpoint step recorded in evaluate-only JSON output.")
    parser.add_argument("--resume_env_steps", type=int, default=0,
                        help="Existing env steps when continuing from --model_dir; used for save/eval/log step labels only.")
    parser.add_argument("--test50_during_training", "--test50-during-training", action="store_true",
                        help="Run training eval as 50-game test50 and write JSON/summary files.")
    parser.add_argument("--test50_output_dir", "--test50-output-dir", type=str, default=None,
                        help="Directory for training-time test50 outputs. Supports {step} and {step_label}.")
    parser.add_argument("--test50_episodes", "--test50-episodes", type=int, default=50,
                        help="Minimum eval episodes when --test50_during_training is enabled.")
    parser.add_argument("--game_logging", action="store_true", default=False,
                        help="Override RoboCup2D env config to write server .rcg logs.")
    parser.add_argument("--robocup_auto_port_start", type=int, default=None,
                        help="Override RoboCup2D auto_port_start for MAPPO jobs.")
    parser.add_argument("--robocup_auto_port_end", type=int, default=None,
                        help="Override RoboCup2D auto_port_end for MAPPO jobs.")
    parser.add_argument("--robocup_warmup_launch", type=str, default="False",
                        help="If true, launch and close one dummy RoboCup2D env before rollout envs.")
    parser.add_argument("--robocup_warmup_pause_seconds", type=float, default=2.0,
                        help="Sleep after closing the dummy warmup env.")
    parser.add_argument("--robocup_launch_stagger_seconds", type=float, default=0.0,
                        help="Sleep this many seconds times (rank + 1) before constructing each train env.")
    parser.add_argument("--robocup_eval_launch_stagger_seconds", type=float, default=None,
                        help="Eval launch stagger. Defaults to --robocup_launch_stagger_seconds.")
    parser.add_argument("--robocup_reset_stagger_seconds", type=float, default=0.0,
                        help="Sleep this many seconds times (rank + 1) before each train reset.")
    parser.add_argument("--robocup_eval_reset_stagger_seconds", type=float, default=None,
                        help="Eval reset stagger. Defaults to --robocup_reset_stagger_seconds.")
    parser.add_argument("--robocup_wait_ready_timeout", type=float, default=None,
                        help="Override RoboCup2D wait_ready_timeout.")
    parser.add_argument("--robocup_playon_timeout", type=float, default=None,
                        help="Override RoboCup2D playon_timeout.")
    parser.add_argument("--robocup_trainer_ready_timeout_ms", type=float, default=None,
                        help="Override trainer-ready timeout in milliseconds.")
    parser.add_argument("--robocup_ports_wait_timeout", type=float, default=None,
                        help="Override RoboCup2D port allocation wait timeout.")
    parser.add_argument("--robocup_reset_retries", type=int, default=None,
                        help="Override RoboCup2D internal reset retries.")
    parser.add_argument("--robocup_reset_perception_warmup_cycles", type=int, default=None,
                        help="Override reset perception warmup cycles.")
    parser.add_argument("--robocup_wrapper_reset_retries", type=int, default=0,
                        help="Recreate the underlying env and retry reset this many times.")
    parser.add_argument("--robocup_wrapper_recreate_pause_seconds", type=float, default=0.0,
                        help="Sleep after closing a failed env before recreating it.")
    parser.add_argument("--robocup_reuse_eval_state", type=str, default="False",
                        help="If true and eval has one thread, reuse the post-done eval state instead of resetting at every eval start.")
    parser.add_argument("--robocup_eval_on_demand", type=str, default="False",
                        help="If true, create eval envs only for each eval call and close them immediately afterwards.")
    all_args = parser.parse_known_args(args)[0]
    all_args.env_name = "RoboCup2D"
    all_args = _infer_num_agents_and_episode_length(all_args)
    return all_args


def main(args):
    parser = get_config()
    all_args = parse_args(args, parser)

    if all_args.algorithm_name == "rmappo":
        all_args.use_recurrent_policy = True
        all_args.use_naive_recurrent_policy = False
    elif all_args.algorithm_name == "mappo":
        all_args.use_recurrent_policy = False
        all_args.use_naive_recurrent_policy = False
    else:
        raise NotImplementedError("RoboCup2D adapter supports mappo/rmappo first.")

    if _truthy(getattr(all_args, "test50_during_training", False)):
        all_args.use_eval = True
        all_args.eval_episodes = max(
            int(getattr(all_args, "eval_episodes", 0) or 0),
            int(getattr(all_args, "test50_episodes", 50) or 50),
        )

    if all_args.cuda and torch.cuda.is_available():
        device = torch.device("cuda:0")
        torch.set_num_threads(all_args.n_training_threads)
        if all_args.cuda_deterministic:
            torch.backends.cudnn.benchmark = False
            torch.backends.cudnn.deterministic = True
    else:
        device = torch.device("cpu")
        torch.set_num_threads(all_args.n_training_threads)

    run_dir = Path(os.path.split(os.path.dirname(os.path.abspath(__file__)))[0] + "/results") \
        / all_args.env_name / all_args.scenario_name / all_args.algorithm_name / all_args.experiment_name
    run_dir.mkdir(parents=True, exist_ok=True)

    if all_args.use_wandb:
        run = wandb.init(config=all_args,
                         project=all_args.env_name,
                         entity=all_args.wandb_name,
                         notes=socket.gethostname(),
                         name="-".join([all_args.algorithm_name, all_args.experiment_name, "seed" + str(all_args.seed)]),
                         group=all_args.scenario_name,
                         dir=str(run_dir),
                         job_type="training",
                         reinit=True)
    else:
        existing = [int(p.name.split("run")[1]) for p in run_dir.iterdir()
                    if p.name.startswith("run") and p.name.split("run")[1].isdigit()]
        curr_run = "run1" if not existing else f"run{max(existing) + 1}"
        run_dir = run_dir / curr_run
        run_dir.mkdir(parents=True, exist_ok=True)

    setproctitle.setproctitle("-".join([
        all_args.env_name,
        all_args.scenario_name,
        all_args.algorithm_name,
        all_args.experiment_name,
    ]) + "@" + all_args.user_name)

    torch.manual_seed(all_args.seed)
    torch.cuda.manual_seed_all(all_args.seed)
    np.random.seed(all_args.seed)

    warmup_robocup_launch(all_args)

    from onpolicy.runner.shared.football_runner import FootballRunner as Runner

    if all_args.evaluate_only:
        if not all_args.model_dir and not all_args.evaluate_random_init:
            raise ValueError("--model_dir is required for --evaluate_only unless --evaluate_random_init is set")
        all_args.use_eval = True
        eval_envs = make_eval_env(all_args)
        envs = eval_envs
        config = {
            "all_args": all_args,
            "envs": envs,
            "eval_envs": eval_envs,
            "make_eval_env": make_eval_env,
            "num_agents": all_args.num_agents,
            "device": device,
            "run_dir": run_dir,
        }
        runner = Runner(config)
        try:
            metrics = runner.eval(int(all_args.eval_checkpoint_step or 0)) or {}
            result = {
                "algorithm": all_args.algorithm_name,
                "checkpoint": str(all_args.model_dir) if all_args.model_dir else "random_init",
                "checkpoint_step": int(all_args.eval_checkpoint_step or 0),
                "eval_episodes": int(all_args.eval_episodes),
            }
            result.update({k: (float(v) if isinstance(v, (np.floating, float, int)) else v) for k, v in metrics.items()})
            print(
                "EVAL_RESULT "
                + " ".join(
                    [
                        f"checkpoint_step={result['checkpoint_step']}",
                        f"episodes={result['eval_episodes']}",
                        f"win_rate={result.get('test_win_rate', result.get('eval_win_rate', 0.0)):.6f}",
                        f"mean_return={result.get('mean_return', 0.0):.6f}",
                        f"completed={result.get('completed_episodes', 0)}",
                    ]
                )
            )
            if all_args.eval_output_json:
                out_path = os.path.abspath(all_args.eval_output_json)
                os.makedirs(os.path.dirname(out_path), exist_ok=True)
                with open(out_path, "w", encoding="utf-8") as f:
                    json.dump(result, f, indent=2, sort_keys=True)
                print(f"Wrote eval JSON: {out_path}")
        finally:
            eval_envs.close()
            if all_args.use_wandb:
                run.finish()
            else:
                if hasattr(runner.writter, "export_scalars_to_json"):
                    runner.writter.export_scalars_to_json(str(runner.log_dir + "/summary.json"))
                runner.writter.close()
        return

    envs = make_train_env(all_args)
    eval_on_demand = _truthy(getattr(all_args, "robocup_eval_on_demand", False))
    eval_envs = None if (all_args.use_eval and eval_on_demand) else (
        make_eval_env(all_args) if all_args.use_eval else None
    )

    config = {
        "all_args": all_args,
        "envs": envs,
        "eval_envs": eval_envs,
        "make_eval_env": make_eval_env,
        "num_agents": all_args.num_agents,
        "device": device,
        "run_dir": run_dir,
    }

    runner = Runner(config)
    runner.run()

    envs.close()
    if eval_envs is not None and eval_envs is not envs:
        eval_envs.close()

    if all_args.use_wandb:
        run.finish()
    else:
        if hasattr(runner.writter, "export_scalars_to_json"):
            runner.writter.export_scalars_to_json(str(runner.log_dir + "/summary.json"))
        runner.writter.close()


if __name__ == "__main__":
    main(sys.argv[1:])
