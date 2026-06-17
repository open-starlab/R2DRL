import datetime
import os
import pprint
import time
import threading
import torch as th
from types import SimpleNamespace as SN
from utils.logging import Logger
from utils.timehelper import time_left, time_str
from os.path import dirname, abspath

from learners import REGISTRY as le_REGISTRY
from runners import REGISTRY as r_REGISTRY
from controllers import REGISTRY as mac_REGISTRY
from components.episode_buffer import ReplayBuffer
from components.transforms import OneHot
import logging



ROBOCUP_REWARD_KEYS = (
    "reward_on_goal_scored",
    "reward_on_goal_conceded",
    "reward_on_possession_loss",
    "reward_on_ball_out",
    "reward_on_ball_left_half",
    "reward_on_timeout",
    "reward_on_draw",
    "epv_progress_reward",
)

ROBOCUP_TERMINATION_KEYS = (
    "terminate_on_goal",
    "terminate_on_possession_loss",
    "terminate_on_ball_out",
    "terminate_on_ball_left_half",
)


def _cfg_get(cfg, key, default=None):
    if isinstance(cfg, dict):
        return cfg.get(key, default)
    return getattr(cfg, key, default)


def _as_float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _as_bool(value):
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def _rounded_interval_step(step, interval):
    try:
        step_i = int(step)
        interval_i = int(interval)
    except (TypeError, ValueError):
        return int(step)
    if interval_i <= 0:
        return step_i
    rounded = int(round(float(step_i) / float(interval_i))) * interval_i
    if rounded > 0 and abs(step_i - rounded) <= max(1, interval_i // 2):
        return rounded
    return step_i


def _active_right_count(env_args):
    active_right = _cfg_get(env_args, "active_right_unums", [])
    if active_right is None:
        return 0
    if isinstance(active_right, (list, tuple, set)):
        return len(active_right)
    return 0 if active_right == "" else 1


def _reward_mode(env_args):
    use_epv = _as_bool(_cfg_get(env_args, "useMaxEpv", False))
    epv_progress = _as_float(_cfg_get(env_args, "epv_progress_reward", 0.0))
    goal_scored = _as_float(_cfg_get(env_args, "reward_on_goal_scored", 0.0))
    non_goal_keys = [key for key in ROBOCUP_REWARD_KEYS if key not in ("reward_on_goal_scored", "epv_progress_reward")]
    non_goal_is_zero = all(abs(_as_float(_cfg_get(env_args, key, 0.0))) < 1e-12 for key in non_goal_keys)
    if (not use_epv) and abs(epv_progress) < 1e-12 and non_goal_is_zero and goal_scored > 0:
        return "goal_only"
    if use_epv or abs(epv_progress) >= 1e-12:
        return "epv_shaped"
    return "custom"


def _log_robocup_tb_metadata(logger, args, env_info):
    writer = getattr(logger, "tb_writer", None)
    if writer is None:
        return
    env_args = getattr(args, "env_args", {}) or {}
    if not env_args:
        return
    try:
        train_players = int(_cfg_get(env_args, "init_n", _cfg_get(env_args, "n", env_info.get("n_agents", 0))))
        total_players = int(_cfg_get(env_args, "n", env_info.get("n_agents", 0)))
        num_envs = int(getattr(args, "batch_size_run", 1))
        opponent_players = int(_active_right_count(env_args))
        mode = _reward_mode(env_args)
        mode_code = {"goal_only": 1.0, "epv_shaped": 2.0, "custom": 3.0}.get(mode, 0.0)

        writer.add_scalar("config/train_players", train_players, 0)
        writer.add_scalar("config/opponent_players", opponent_players, 0)
        writer.add_scalar("config/total_players_per_side", total_players, 0)
        writer.add_scalar("config/num_envs", num_envs, 0)
        writer.add_scalar("config/n_agents", int(env_info.get("n_agents", total_players)), 0)
        writer.add_scalar("config/reward/mode_code", mode_code, 0)
        writer.add_scalar("config/reward/is_goal_only", 1.0 if mode == "goal_only" else 0.0, 0)
        writer.add_scalar("config/reward/epv_progress_scale", _as_float(_cfg_get(env_args, "epv_progress_scale", 1.0)), 0)

        for key in ROBOCUP_REWARD_KEYS:
            writer.add_scalar(f"config/reward/{key}", _as_float(_cfg_get(env_args, key, 0.0)), 0)
        for key in ROBOCUP_TERMINATION_KEYS:
            writer.add_scalar(f"config/termination/{key}", 1.0 if _as_bool(_cfg_get(env_args, key, False)) else 0.0, 0)

        rows = [
            ("algorithm", getattr(args, "name", "")),
            ("env_config", _cfg_get(env_args, "_pymarl_env_config_source", "")),
            ("start_id", _cfg_get(env_args, "start_id", "")),
            ("train_players", train_players),
            ("opponent_players", opponent_players),
            ("total_players_per_side", total_players),
            ("num_envs", num_envs),
            ("reward_mode", mode),
            ("useMaxEpv", _cfg_get(env_args, "useMaxEpv", False)),
            ("epv_progress_scale", _cfg_get(env_args, "epv_progress_scale", 1.0)),
            ("epv_grid_file", _cfg_get(env_args, "epv_grid_file", "EPV_grid.csv")),
        ]
        rows.extend((key, _cfg_get(env_args, key, 0.0)) for key in ROBOCUP_REWARD_KEYS)
        markdown = "| key | value |\n| - | - |\n" + "\n".join(f"| {k} | {v} |" for k, v in rows)
        writer.add_text("config/summary", markdown, 0)
        writer.flush()
    except Exception as exc:
        logger.console_logger.warning(f"[Logger] Failed to write RoboCup TB metadata: {exc}")

def run(config):
    print("begin running")
    import sys

    console_logger = logging.getLogger("root")
    console_logger.setLevel(logging.INFO)
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(logging.Formatter("%(message)s"))
    console_logger.handlers = [handler]

    sys.stdout.reconfigure(line_buffering=True)

    logger = Logger(console_logger)

    config = args_sanity_check(config, console_logger)

    args = SN(**config)
    args.device = "cuda" if args.use_cuda else "cpu"

    logger.console_logger.info("Experiment Parameters:")
    experiment_params = pprint.pformat(config, indent=4, width=1)
    logger.console_logger.info("\n\n" + experiment_params + "\n")

    unique_token = "{}__{}".format(
        args.name, datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    )
    args.unique_token = unique_token

    if args.use_tensorboard:
        tb_logs_direc = os.path.join(args.local_results_path, "logs", "tb_logs")
        tb_exp_direc = os.path.join(tb_logs_direc, "{}").format(unique_token)
        logger.setup_tb(tb_exp_direc)

    print("run_sequential")
    run_sequential(args=args, logger=logger)

    print("Exiting Main")
    print("Stopping all threads")
    for t in threading.enumerate():
        if t.name != "MainThread":
            print("Thread {} is alive! Is daemon: {}".format(t.name, t.daemon))
            t.join(timeout=1)
            print("Thread joined")

    print("Exiting script")
    os._exit(os.EX_OK)


def evaluate_sequential(args, runner, logger=None):
    n_test_runs = max(1, args.test_nepisode // runner.batch_size)

    for _ in range(n_test_runs):
        runner.run(test_mode=True)

    if logger is not None:
        logger.print_recent_stats()

    if args.save_replay:
        runner.save_replay()

    runner.close_env()


def run_sequential(args, logger):
    runner = r_REGISTRY[args.runner](args=args, logger=logger)
    try:
        effective_batch_size = runner.batch_size

        if _as_bool(getattr(args, "test50_during_training", False)):
            args.test_nepisode = max(
                int(getattr(args, "test_nepisode", 0) or 0),
                int(getattr(args, "test50_episodes", 50) or 50),
            )

        if args.test_nepisode < effective_batch_size:
            args.test_nepisode = effective_batch_size
        else:
            args.test_nepisode = (
                args.test_nepisode // effective_batch_size
            ) * effective_batch_size

        env_info = runner.get_env_info()
        print("env_info={}".format(env_info))

        args.n_agents = env_info["n_agents"]
        args.n_actions = env_info["n_actions"]
        args.state_shape = env_info["state_shape"]

        _log_robocup_tb_metadata(logger, args, env_info)

        scheme = {
            "state": {"vshape": env_info["state_shape"]},
            "obs": {"vshape": env_info["obs_shape"], "group": "agents"},
            "actions": {"vshape": (1,), "group": "agents", "dtype": th.long},
            "avail_actions": {
                "vshape": (env_info["n_actions"],),
                "group": "agents",
                "dtype": th.int,
            },
            "reward": {"vshape": (1,)},
            "terminated": {"vshape": (1,), "dtype": th.uint8},
        }

        groups = {
            "agents": args.n_agents
        }

        preprocess = {
            "actions": ("actions_onehot", [OneHot(out_dim=args.n_actions)])
        }

        buffer = ReplayBuffer(
            scheme,
            groups,
            args.buffer_size,
            env_info["episode_limit"] + 1,
            preprocess=preprocess,
            device="cpu" if args.buffer_cpu_only else args.device,
        )

        mac = mac_REGISTRY[args.mac](buffer.scheme, groups, args)

        runner.setup(scheme=scheme, groups=groups, preprocess=preprocess, mac=mac)
        print("runner.setup")

        learner = le_REGISTRY[args.learner](mac, buffer.scheme, logger, args)

        if args.use_cuda:
            learner.cuda()

        if getattr(args, "evaluate_random_init", False):
            if args.evaluate or args.save_replay:
                evaluate_sequential(args, runner, logger)
                return

        if args.checkpoint_path != "":
            timesteps = []
            timestep_to_load = 0

            if not os.path.isdir(args.checkpoint_path):
                logger.console_logger.info(
                    "Checkpoint directiory {} doesn't exist".format(
                        args.checkpoint_path
                    )
                )
                return

            for name in os.listdir(args.checkpoint_path):
                full_name = os.path.join(args.checkpoint_path, name)
                if os.path.isdir(full_name) and name.isdigit():
                    timesteps.append(int(name))

            if len(timesteps) == 0:
                logger.console_logger.info(
                    "No checkpoints found in {}".format(args.checkpoint_path)
                )
                return

            if args.load_step == 0:
                timestep_to_load = max(timesteps)
            else:
                timestep_to_load = min(
                    timesteps, key=lambda x: abs(x - args.load_step)
                )

            model_path = os.path.join(args.checkpoint_path, str(timestep_to_load))

            logger.console_logger.info("Loading model from {}".format(model_path))
            learner.load_models(model_path)
            runner.t_env = timestep_to_load

            if args.evaluate or args.save_replay:
                evaluate_sequential(args, runner, logger)
                return

        print("start training")
        episode = 0
        last_test_T = runner.t_env
        last_log_T = runner.t_env
        model_save_time = runner.t_env
        train_updates_per_batch = max(1, int(args.train_updates_per_batch))
        buffer_warmup = max(args.batch_size, int(args.buffer_warmup))

        start_time = time.time()
        last_time = start_time
        tb_flush_every_episodes = max(1, int(args.runner_log_interval // max(1, args.batch_size_run)))

        logger.console_logger.info(
            "Beginning training for {} timesteps".format(args.t_max)
        )

        while runner.t_env <= args.t_max:
            episode_batch = runner.run(test_mode=False)

            T = episode_batch.max_t_filled()
            ep_ret = episode_batch["reward"][:, :T].sum(dim=1).mean().item()

            if getattr(logger, "tb_writer", None) is not None:
                logger.tb_writer.add_scalar(
                    "train/episode_return", float(ep_ret), int(episode)
                )
                if episode == 0 or (episode // max(1, args.batch_size_run)) % tb_flush_every_episodes == 0:
                    logger.tb_writer.flush()

            buffer.insert_episode_batch(episode_batch)

            if buffer.episodes_in_buffer >= buffer_warmup and buffer.can_sample(args.batch_size):
                # Preserve the old "episodes per learner step" pacing even when one
                # rollout now contains many more parallel episodes.
                virtual_episode_span = runner.batch_size / float(train_updates_per_batch)

                for update_idx in range(train_updates_per_batch):
                    episode_sample = buffer.sample(args.batch_size)

                    max_ep_t = episode_sample.max_t_filled()
                    episode_sample = episode_sample[:, :max_ep_t]

                    if episode_sample.device != args.device:
                        episode_sample.to(args.device)

                    learner_episode = episode + update_idx * virtual_episode_span
                    print("learner.train")
                    learner.train(episode_sample, runner.t_env, learner_episode)

            n_test_runs = max(1, args.test_nepisode // runner.batch_size)
            if (runner.t_env - last_test_T) / args.test_interval >= 1.0:
                logger.console_logger.info(
                    "t_env: {} / {}".format(runner.t_env, args.t_max)
                )
                logger.console_logger.info(
                    "Estimated time left: {}. Time passed: {}".format(
                        time_left(last_time, last_test_T, runner.t_env, args.t_max),
                        time_str(time.time() - start_time),
                    )
                )
                last_time = time.time()

                last_test_T = runner.t_env
                for _ in range(n_test_runs):
                    runner.run(test_mode=True)

            if args.save_model and (
                runner.t_env - model_save_time >= args.save_model_interval
            ):
                model_save_time = runner.t_env
                save_step = _rounded_interval_step(runner.t_env, args.save_model_interval)
                save_path = os.path.join(
                    args.local_results_path,
                    "models",
                    args.unique_token,
                    str(save_step),
                )
                os.makedirs(save_path, exist_ok=True)
                logger.console_logger.info("Saving models to {}".format(save_path))
                learner.save_models(save_path)

            episode += runner.batch_size

            if (runner.t_env - last_log_T) >= args.log_interval:
                logger.log_stat("episode", episode, runner.t_env)
                logger.print_recent_stats()
                last_log_T = runner.t_env

        if args.save_model and runner.t_env != model_save_time:
            save_step = _rounded_interval_step(runner.t_env, args.save_model_interval)
            save_path = os.path.join(
                args.local_results_path,
                "models",
                args.unique_token,
                str(save_step),
            )
            os.makedirs(save_path, exist_ok=True)
            logger.console_logger.info("Saving final models to {}".format(save_path))
            learner.save_models(save_path)

        logger.console_logger.info("Finished Training")
        print("Sequence finished!")

    finally:
        runner.close_env()


def args_sanity_check(config, _log):
    if config["use_cuda"] and not th.cuda.is_available():
        config["use_cuda"] = False
        _log.warning(
            "CUDA flag use_cuda was switched OFF automatically because no CUDA devices are available!"
        )

    config["train_updates_per_batch"] = max(
        1, int(config.get("train_updates_per_batch", 1))
    )
    config["buffer_warmup"] = max(
        int(config.get("batch_size", 1)),
        int(config.get("buffer_warmup", config.get("batch_size", 1))),
    )

    return config
