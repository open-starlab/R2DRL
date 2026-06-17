import os
import json
import torch
import random
import argparse
import numpy as np
from torch.utils.tensorboard import SummaryWriter
from datetime import datetime
from pdqn.replay_buffer import ReplayBuffer
from pdqn.agent import ParaDQNAgent
from pdqn.train import train_vec, evaluate, load_checkpoint
from environments.robocup2d_hybrid_helios_v0 import RobocupEnv_Hybrid_Helios
from environments.vec_env import SubprocVecEnv



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


def _robocup_env_overrides(args):
    overrides = {}
    init_n = getattr(args, "init_n", None)
    if init_n is not None:
        overrides["init_n"] = int(init_n)
    use_epv = getattr(args, "useMaxEpv", None)
    if use_epv is not None:
        overrides["useMaxEpv"] = _as_bool(use_epv)
    use_action_mask = getattr(args, "use_action_mask", None)
    if use_action_mask is not None:
        overrides["use_action_mask"] = _as_bool(use_action_mask)
    epv_grid_file = getattr(args, "epv_grid_file", None)
    if epv_grid_file:
        overrides["epv_grid_file"] = str(epv_grid_file)
    epv_progress_scale = getattr(args, "epv_progress_scale", None)
    if epv_progress_scale is not None:
        overrides["epv_progress_scale"] = float(epv_progress_scale)
    auto_port_start = getattr(args, "auto_port_start", None)
    if auto_port_start is not None:
        overrides["auto_port_start"] = int(auto_port_start)
    auto_port_end = getattr(args, "auto_port_end", None)
    if auto_port_end is not None:
        overrides["auto_port_end"] = int(auto_port_end)
    return overrides


def _load_robocup_env_args(env_config, overrides=None):
    try:
        from environments.robocup2d.config.loader import load_env_args
        from environments.robocup2d.start_sampler import apply_catalog_launch_profile
        env_args = dict(load_env_args(env_config, overrides or {}))
        apply_catalog_launch_profile(env_args)
        env_args["team"] = "hybrid"
        return env_args
    except Exception as exc:
        print(f"[WARN] Failed to load env config for TensorBoard metadata: {exc}")
        return {}


def _log_robocup_tb_metadata(writer, args, num_agents, env_args):
    if writer is None or not env_args:
        return
    train_players = int(_cfg_get(env_args, "init_n", _cfg_get(env_args, "n", num_agents)))
    total_players = int(_cfg_get(env_args, "n", num_agents))
    opponent_players = int(_active_right_count(env_args))
    num_envs = int(getattr(args, "num_envs", 1))
    mode = _reward_mode(env_args)
    mode_code = {"goal_only": 1.0, "epv_shaped": 2.0, "custom": 3.0}.get(mode, 0.0)

    writer.add_scalar("config/train_players", train_players, 0)
    writer.add_scalar("config/opponent_players", opponent_players, 0)
    writer.add_scalar("config/total_players_per_side", total_players, 0)
    writer.add_scalar("config/num_envs", num_envs, 0)
    writer.add_scalar("config/n_agents", int(num_agents), 0)
    writer.add_scalar("config/eval_interval_steps", int(getattr(args, "eval_interval_steps", 0) or 0), 0)
    writer.add_scalar("config/max_env_steps", int(getattr(args, "max_env_steps", 0) or 0), 0)
    writer.add_scalar("config/eval_episodes", int(getattr(args, "eval_episodes", 0) or 0), 0)
    writer.add_scalar("config/eval_on_demand", 1.0 if _as_bool(getattr(args, "eval_on_demand", True)) else 0.0, 0)
    writer.add_scalar("config/reward/mode_code", mode_code, 0)
    writer.add_scalar("config/reward/is_goal_only", 1.0 if mode == "goal_only" else 0.0, 0)
    writer.add_scalar("config/action_mask/use_action_mask", 1.0 if _as_bool(_cfg_get(env_args, "use_action_mask", True)) else 0.0, 0)
    writer.add_scalar("config/reward/epv_progress_scale", _as_float(_cfg_get(env_args, "epv_progress_scale", 1.0)), 0)
    for key in ROBOCUP_REWARD_KEYS:
        writer.add_scalar(f"config/reward/{key}", _as_float(_cfg_get(env_args, key, 0.0)), 0)
    for key in ROBOCUP_TERMINATION_KEYS:
        writer.add_scalar(f"config/termination/{key}", 1.0 if _as_bool(_cfg_get(env_args, key, False)) else 0.0, 0)

    rows = [
        ("algorithm", "paradqn"),
        ("env_config", getattr(args, "env_config", "")),
        ("start_id", _cfg_get(env_args, "start_id", "")),
        ("train_players", train_players),
        ("opponent_players", opponent_players),
        ("total_players_per_side", total_players),
        ("num_envs", num_envs),
        ("eval_interval_steps", int(getattr(args, "eval_interval_steps", 0) or 0)),
        ("max_env_steps", int(getattr(args, "max_env_steps", 0) or 0)),
        ("eval_episodes", int(getattr(args, "eval_episodes", 0) or 0)),
        ("eval_on_demand", _as_bool(getattr(args, "eval_on_demand", True))),
        ("obs_agent_id", _as_bool(getattr(args, "obs_agent_id", True))),
        ("use_action_mask", _as_bool(_cfg_get(env_args, "use_action_mask", True))),
        ("auto_port_start", _cfg_get(env_args, "auto_port_start", "")),
        ("auto_port_end", _cfg_get(env_args, "auto_port_end", "")),
        ("reward_mode", mode),
        ("useMaxEpv", _cfg_get(env_args, "useMaxEpv", False)),
        ("epv_progress_scale", _cfg_get(env_args, "epv_progress_scale", 1.0)),
        ("epv_grid_file", _cfg_get(env_args, "epv_grid_file", "EPV_grid.csv")),
    ]
    rows.extend((key, _cfg_get(env_args, key, 0.0)) for key in ROBOCUP_REWARD_KEYS)
    markdown = "| key | value |\n| - | - |\n" + "\n".join(f"| {k} | {v} |" for k, v in rows)
    writer.add_text("config/summary", markdown, 0)
    writer.flush()

def make_robocup_env(
    args: argparse.Namespace,
    rank: int = 0,
    is_eval: bool = False,
) -> RobocupEnv_Hybrid_Helios:
    return RobocupEnv_Hybrid_Helios(
        cfg=args.env_config,
        team="hybrid",
        rank=rank,
        is_eval=is_eval,
        obs_agent_id=getattr(args, "obs_agent_id", True),
        **_robocup_env_overrides(args),
    )


def make_robocup_env_fn(args: argparse.Namespace, rank: int):
    def _thunk():
        return make_robocup_env(args, rank=rank, is_eval=False).train()

    return _thunk


def train_robocup(args: argparse.Namespace):
    # seed
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    # device
    if args.device == "cuda" and torch.cuda.is_available():
        device = "cuda"
        print("Using GPU:", torch.cuda.get_device_name(0))
    else:
        device = "cpu"
        print("Using CPU")

    # environment
    if args.num_envs > 1:
        print(f"Creating {args.num_envs} parallel environments...")
        env_fns = [make_robocup_env_fn(args, rank) for rank in range(args.num_envs)]
        env = SubprocVecEnv(env_fns)
        print(f"Vectorized environment created with {len(env)} parallel processes.")
        eval_rank = args.num_envs
    else:
        print("Creating single environment...")
        env = make_robocup_env(args, rank=0, is_eval=False).train()
        eval_rank = 1

    eval_env_factory = lambda: make_robocup_env(args, rank=eval_rank, is_eval=True).eval()
    if args.eval_on_demand:
        print("Evaluation environment will be created on demand.")
        eval_env = None
    else:
        print("Creating persistent evaluation environment.")
        eval_env = eval_env_factory()
        eval_env_factory = None

    # Get environment spaces
    if args.num_envs > 1:
        # For vectorized env, spaces are already stored
        observation_space = env.observation_space_single
        action_space = env.action_space_single
        num_agents = env.num_agents
    else:
        # For single env
        observation_space = env.observation_space(env.players[0])
        action_space = env.action_space(env.players[0])
        num_agents = len(env.players)

    # buffer
    buffer = ReplayBuffer(
        capacity=args.replay_capacity,
        # Each replay slot stores the full controlled team transition.
        num_agents=num_agents,
        observation_space=observation_space,
        shared_observation_space=env.state_space,
        action_space=action_space,
        priority_alpha=args.replay_priority_alpha,
        reward_priority_scale=args.replay_reward_priority_scale,
        positive_reward_priority_bonus=args.replay_positive_reward_bonus,
        terminal_reward_priority_bonus=args.replay_terminal_reward_bonus,
    )

    # agent
    agent = ParaDQNAgent(
        observation_space=observation_space,
        action_space=action_space,
        device=device,
        gamma=args.gamma,
        lr_q=args.lr_q,
        lr_actor=args.lr_actor,
        tau_q=args.tau_q,
        tau_actor=args.tau_actor,
        double_q=args.double_q,
        q_loss_type=args.q_loss_type,
        grad_clip_norm=args.grad_clip_norm,
        policy_delay=args.policy_delay,
        actor_behavior_l2_coef=args.actor_behavior_l2_coef,
    )

    if args.test50_during_training:
        args.eval_episodes = max(int(args.eval_episodes), int(args.test50_episodes or 50))

    # logging
    env_name = os.path.splitext(os.path.basename(str(args.env_config)))[0]
    run_name = args.run_name or f"{env_name}_hybrid"
    run_dir = os.path.join(
        os.path.dirname(__file__),
        "runs",
        f"{run_name}_seed_{args.seed}_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
    )
    os.makedirs(run_dir)
    print("Run dir:", run_dir)
    writer = SummaryWriter(log_dir=os.path.join(run_dir, "logs"))
    env_args_for_tb = _load_robocup_env_args(args.env_config, _robocup_env_overrides(args))
    _log_robocup_tb_metadata(writer, args, num_agents, env_args_for_tb)
    writer.add_scalar("config/algorithm/obs_agent_id", 1.0 if _as_bool(getattr(args, "obs_agent_id", True)) else 0.0, 0)
    writer.add_scalar("config/algorithm/n_step_return", int(args.n_step_return), 0)
    writer.add_scalar("config/algorithm/double_q", 1.0 if args.double_q else 0.0, 0)
    writer.add_scalar("config/algorithm/q_loss_is_huber", 1.0 if str(args.q_loss_type).lower() in ("huber", "smooth_l1", "smoothl1") else 0.0, 0)
    writer.add_scalar("config/algorithm/grad_clip_norm", float(args.grad_clip_norm), 0)
    writer.add_scalar("config/algorithm/policy_delay", int(args.policy_delay), 0)
    writer.add_scalar("config/algorithm/eval_policy_target", 1.0 if args.eval_policy == "target" else 0.0, 0)
    writer.add_scalar("config/algorithm/updates_per_step", int(args.updates_per_step), 0)
    writer.add_scalar("config/algorithm/actor_freeze_after_steps", int(args.actor_freeze_after_steps), 0)
    writer.add_scalar("config/algorithm/actor_behavior_l2_coef", float(args.actor_behavior_l2_coef), 0)
    writer.add_scalar("config/checkpoint/save_interval_steps", int(args.save_interval_steps), 0)
    writer.add_scalar("config/replay/priority_alpha", float(args.replay_priority_alpha), 0)
    writer.add_scalar("config/replay/reward_priority_scale", float(args.replay_reward_priority_scale), 0)
    writer.add_scalar("config/replay/positive_reward_bonus", float(args.replay_positive_reward_bonus), 0)
    writer.add_scalar("config/replay/terminal_reward_bonus", float(args.replay_terminal_reward_bonus), 0)
    checkpoint_dir = os.path.join(run_dir, "checkpoints")
    os.makedirs(checkpoint_dir)

    # training
    train_vec(
        env=env,
        eval_env=eval_env,
        agent=agent,
        buffer=buffer,
        writer=writer,
        episodes=args.train_episodes,
        batch_size=args.batch_size,
        train_freq=args.train_freq,
        eval_episodes=args.eval_episodes,
        eval_interval=args.eval_interval,
        eval_interval_steps=args.eval_interval_steps,
        save_interval=args.save_interval,
        save_interval_steps=args.save_interval_steps,
        epsilon_start=args.epsilon_start,
        epsilon_end=args.epsilon_end,
        epsilon_decay_steps=args.epsilon_decay_steps,
        checkpoint_dir=checkpoint_dir,
        resume_from=args.resume_from,
        eval_env_factory=eval_env_factory,
        max_env_steps=args.max_env_steps,
        n_step_return=args.n_step_return,
        eval_policy=args.eval_policy,
        updates_per_step=args.updates_per_step,
        actor_freeze_after_steps=args.actor_freeze_after_steps,
        test50_during_training=args.test50_during_training,
        test50_output_dir=args.test50_output_dir,
        test50_episodes=args.test50_episodes,
    )

    # Close all environments at the very end
    print("Closing environments...")
    if args.num_envs > 1:
        env.close()
        if eval_env is not None:
            eval_env.close()
    else:
        env.close()
        if eval_env is not None and eval_env is not env:
            eval_env.close()

    writer.close()
    print("Training completed.")


def _resolve_checkpoint_path(path: str) -> str:
    if not path:
        raise ValueError("--resume_from is required for --evaluate_only")
    if os.path.isdir(path):
        final_path = os.path.join(path, "final.pth")
        latest_path = os.path.join(path, "latest.pth")
        if os.path.exists(final_path):
            return final_path
        if os.path.exists(latest_path):
            return latest_path
    return path


def evaluate_robocup(args: argparse.Namespace):
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    if args.device == "cuda" and torch.cuda.is_available():
        device = "cuda"
        print("Using GPU:", torch.cuda.get_device_name(0))
    else:
        device = "cpu"
        print("Using CPU")

    print(f"Creating evaluation environment for {args.eval_episodes} episodes...")
    env = make_robocup_env(args, rank=0, is_eval=True).eval()
    try:
        observation_space = env.observation_space(env.players[0])
        action_space = env.action_space(env.players[0])
        agent = ParaDQNAgent(
            observation_space=observation_space,
            action_space=action_space,
            device=device,
            gamma=args.gamma,
            lr_q=args.lr_q,
            lr_actor=args.lr_actor,
            tau_q=args.tau_q,
            tau_actor=args.tau_actor,
            double_q=args.double_q,
            q_loss_type=args.q_loss_type,
            grad_clip_norm=args.grad_clip_norm,
            policy_delay=args.policy_delay,
            actor_behavior_l2_coef=args.actor_behavior_l2_coef,
        )

        if args.evaluate_random_init:
            checkpoint_path = "random_init"
            checkpoint_step = 0
            checkpoint_episode = 0
        else:
            checkpoint_path = _resolve_checkpoint_path(args.resume_from)
            ck = load_checkpoint(checkpoint_path, agent)
            if ck is None:
                raise RuntimeError(f"Failed to load checkpoint: {checkpoint_path}")
            checkpoint_step = int(ck.get("total_env_steps", 0) or 0)
            checkpoint_episode = int(ck.get("episode", 0) or 0)

        mean_r, std_r, rates = evaluate(env, agent, episodes=args.eval_episodes, eval_policy=args.eval_policy)
        result = {
            "algorithm": "paradqn",
            "checkpoint": checkpoint_path,
            "checkpoint_step": checkpoint_step,
            "checkpoint_episode": checkpoint_episode,
            "eval_episodes": int(args.eval_episodes),
            "eval_policy": str(args.eval_policy),
            "mean_return": float(mean_r),
            "std_return": float(std_r),
        }
        result.update({k: float(v) for k, v in rates.items() if not str(k).startswith("eval_actions/")})

        print(
            "EVAL_RESULT "
            + " ".join(
                [
                    f"checkpoint_step={result['checkpoint_step']}",
                    f"episodes={result['eval_episodes']}",
                    f"win_rate={result.get('win_rate', 0.0):.6f}",
                    f"goal_rate={result.get('goal_rate', 0.0):.6f}",
                    f"mean_return={result['mean_return']:.6f}",
                    f"std_return={result['std_return']:.6f}",
                ]
            )
        )
        if args.eval_output_json:
            out_path = os.path.abspath(args.eval_output_json)
            os.makedirs(os.path.dirname(out_path), exist_ok=True)
            with open(out_path, "w", encoding="utf-8") as f:
                json.dump(result, f, indent=2, sort_keys=True)
            print(f"Wrote eval JSON: {out_path}")
        return result
    finally:
        env.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    # general settings
    parser.add_argument("--seed", type=int, default=0, help="Random seed.")
    parser.add_argument("--device", type=str, default="cuda", help="Device to use (cpu or cuda).")
    parser.add_argument("--num_envs", type=int, default=1, help="Number of parallel environments for data collection.")
    parser.add_argument(
        "--env-config",
        type=str,
        default="parallelr2drl_11vs11scenario_catalog_front-goal-1v0_opp-lv3_goal-only.yaml",
        help="Robocup2D env config yaml name/path. The team field is overridden to hybrid.",
    )
    parser.add_argument(
        "--robocup-init-n",
        "--robocup_init_n",
        "--init-n",
        "--init_n",
        "--agent-mask-n",
        "--agent_mask_n",
        dest="init_n",
        type=int,
        default=None,
        help="Override env init_n, i.e. how many nearest left-side agents the agent mask controls.",
    )
    parser.add_argument(
        "--obs-agent-id",
        "--obs_agent_id",
        dest="obs_agent_id",
        nargs="?",
        const=True,
        default=True,
        type=_as_bool,
        help="Append a one-hot agent id to each controlled player's observation. Enabled by default to match QMIX obs_agent_id.",
    )
    parser.add_argument(
        "--no-obs-agent-id",
        "--no_obs_agent_id",
        dest="obs_agent_id",
        action="store_false",
        help="Disable one-hot agent id observation augmentation for old checkpoints/ablations.",
    )
    parser.add_argument(
        "--useMaxEpv",
        "--use-epv-reward",
        dest="useMaxEpv",
        nargs="?",
        const=True,
        default=None,
        type=_as_bool,
        help="Override env useMaxEpv. Use true/false to enable or disable max-EPV reward.",
    )
    parser.add_argument(
        "--no-use-epv-reward",
        dest="useMaxEpv",
        action="store_false",
        default=None,
        help="Disable env max-EPV reward regardless of the yaml.",
    )
    parser.add_argument(
        "--use-action-mask",
        "--use_action_mask",
        "--robocup-use-action-mask",
        "--robocup_use_action_mask",
        dest="use_action_mask",
        nargs="?",
        const=True,
        default=None,
        type=_as_bool,
        help="Override env use_action_mask. Use true/false to enable or disable legal-action masks.",
    )
    parser.add_argument(
        "--no-use-action-mask",
        "--no_use_action_mask",
        "--robocup-no-use-action-mask",
        dest="use_action_mask",
        action="store_false",
        default=None,
        help="Disable env legal-action masks regardless of the yaml.",
    )
    parser.add_argument(
        "--epv-grid-file",
        "--epv_grid_file",
        dest="epv_grid_file",
        type=str,
        default=None,
        help="Override env epv_grid_file, e.g. EPV_grid_front_goal_linear.csv.",
    )
    parser.add_argument(
        "--epv-progress-scale",
        "--epv_progress_scale",
        dest="epv_progress_scale",
        type=float,
        default=None,
        help="Override env epv_progress_scale.",
    )
    parser.add_argument(
        "--robocup-auto-port-start",
        "--robocup_auto_port_start",
        "--auto-port-start",
        "--auto_port_start",
        dest="auto_port_start",
        type=int,
        default=None,
        help="Override env auto_port_start for RoboCup server allocation.",
    )
    parser.add_argument(
        "--robocup-auto-port-end",
        "--robocup_auto_port_end",
        "--auto-port-end",
        "--auto_port_end",
        dest="auto_port_end",
        type=int,
        default=None,
        help="Override env auto_port_end for RoboCup server allocation.",
    )
    parser.add_argument("--run-name", type=str, default=None, help="Optional run name prefix.")

    # agent settings
    parser.add_argument("--gamma", type=float, default=0.99, help="Discount factor.")
    parser.add_argument("--lr_q", type=float, default=1e-4, help="Learning rate for Q network (lr_q << lr_actor).")
    parser.add_argument("--lr_actor", type=float, default=1e-3, help="Learning rate for actor/param network (lr_q << lr_actor).")
    parser.add_argument("--tau_q", type=float, default=0.005, help="Soft update factor for Q target network.")
    parser.add_argument("--tau_actor", type=float, default=0.005, help="Soft update factor for actor target network.")
    parser.add_argument("--double_q", "--double-q", action="store_true", help="Use online Q/action selection with target Q evaluation for Double-DQN style targets.")
    parser.add_argument("--q-loss-type", "--q_loss_type", dest="q_loss_type", choices=("mse", "huber", "smooth_l1"), default="mse", help="TD loss for Q updates. Huber/SmoothL1 is less sensitive to rare large targets.")
    parser.add_argument("--grad-clip-norm", "--grad_clip_norm", dest="grad_clip_norm", type=float, default=0.0, help="Clip Q/actor gradient norm when > 0.")
    parser.add_argument("--policy-delay", "--policy_delay", dest="policy_delay", type=int, default=1, help="Update actor/parameter policy every N Q updates. 1 preserves vanilla ParaDQN.")
    parser.add_argument("--eval-policy", "--eval_policy", dest="eval_policy", choices=("online", "target"), default="online", help="Greedy policy used for eval/evaluate_only. target uses Polyak target networks as an EMA policy.")
    parser.add_argument("--updates-per-step", "--updates_per_step", dest="updates_per_step", type=int, default=1, help="Gradient updates per environment collection step. 1 preserves previous behavior.")
    parser.add_argument("--actor-freeze-after-steps", "--actor_freeze_after_steps", dest="actor_freeze_after_steps", type=int, default=0, help="If >0, stop actor/parameter-policy updates after this many env steps while continuing Q updates.")
    parser.add_argument("--actor-behavior-l2-coef", "--actor_behavior_l2_coef", dest="actor_behavior_l2_coef", type=float, default=0.0, help="Small behavior regularization coefficient for actor parameters. 0 preserves previous behavior.")

    # training settings
    parser.add_argument("--replay_capacity", type=int, default=20000, help="Replay buffer capacity.")
    parser.add_argument("--n_step_return", "--n-step-return", type=int, default=1, help="Number of transitions to fold into each replay target reward. 1 preserves vanilla ParaDQN.")
    parser.add_argument("--replay_priority_alpha", "--replay-priority-alpha", type=float, default=0.0, help="Priority exponent for reward-biased replay sampling. 0 preserves uniform replay.")
    parser.add_argument("--replay_reward_priority_scale", "--replay-reward-priority-scale", type=float, default=0.0, help="Priority multiplier for absolute replay reward.")
    parser.add_argument("--replay_positive_reward_bonus", "--replay-positive-reward-bonus", type=float, default=0.0, help="Additional replay priority for positive-reward transitions.")
    parser.add_argument("--replay_terminal_reward_bonus", "--replay-terminal-reward-bonus", type=float, default=0.0, help="Additional replay priority for positive terminal transitions such as goals.")
    parser.add_argument("--train_episodes", type=int, default=int(5e5), help="Number of training episodes.")
    parser.add_argument("--max_env_steps", type=int, default=0, help="Optional environment-step budget. If > 0, training stops once this many env steps are collected.")
    parser.add_argument("--batch_size", type=int, default=64, help="Batch size for training.")
    parser.add_argument("--train_freq", type=int, default=1, help="Training frequency (in steps).")
    parser.add_argument("--eval_episodes", type=int, default=20, help="Number of evaluation episodes.")
    parser.add_argument("--eval_interval", type=int, default=25, help="Legacy evaluation interval (in episodes), used only if --eval_interval_steps <= 0.")
    parser.add_argument("--eval_interval_steps", type=int, default=30000, help="Evaluation interval in environment steps for TensorBoard-aligned comparisons.")
    parser.add_argument("--eval_on_demand", type=_as_bool, default=True, help="Create and close a fresh eval env for each eval. Set false to keep one persistent eval env.")
    parser.add_argument("--evaluate_only", "--evaluate-only", action="store_true", help="Load --resume_from and run evaluation only without training.")
    parser.add_argument("--evaluate_random_init", "--evaluate-random-init", action="store_true", help="With --evaluate_only, evaluate the seed-initialized policy without loading --resume_from.")
    parser.add_argument("--eval_output_json", type=str, default=None, help="Optional path to write evaluate-only metrics as JSON.")
    parser.add_argument("--test50_during_training", "--test50-during-training", action="store_true", help="Run training eval as 50-game test50 and write JSON/summary files.")
    parser.add_argument("--test50_output_dir", "--test50-output-dir", type=str, default=None, help="Directory for training-time test50 outputs. Supports {step} and {step_label}.")
    parser.add_argument("--test50_episodes", "--test50-episodes", type=int, default=50, help="Minimum eval episodes when --test50_during_training is enabled.")
    parser.add_argument("--save_interval", type=int, default=100, help="Model saving interval (in episodes).")
    parser.add_argument("--save_interval_steps", "--save-interval-steps", type=int, default=0, help="Optional model saving interval in environment steps. If > 0, writes ck_step*.pth checkpoints.")
    parser.add_argument("--epsilon_start", type=float, default=1.0, help="Starting value of epsilon for epsilon-greedy.")
    parser.add_argument("--epsilon_end", type=float, default=0.05, help="Final value of epsilon for epsilon-greedy.")
    parser.add_argument("--epsilon_decay_steps", type=int, default=1_000_000, help="Number of steps to decay epsilon.")
    parser.add_argument("--resume_from", type=str, default=None, help="Path to resume checkpoint.")

    args = parser.parse_args()
    if args.evaluate_only:
        evaluate_robocup(args)
    else:
        train_robocup(args)
