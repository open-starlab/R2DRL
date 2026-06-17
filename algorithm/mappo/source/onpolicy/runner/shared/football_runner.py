from collections import defaultdict, deque
from itertools import chain
import json
import os
import time

try:
    import imageio
except ImportError:
    imageio = None
import numpy as np
import torch
import wandb

from onpolicy.utils.util import update_linear_schedule
from onpolicy.runner.shared.base_runner import Runner


def _t2n(x):
    return x.detach().cpu().numpy()


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


def _test50_step_label(step):
    step = int(step)
    if step == 0:
        return "0m"
    if step % 1_000_000 == 0:
        return f"{step // 1_000_000}m"
    if step % 1_000 == 0:
        return f"{step // 1_000}k"
    return str(step)


def _write_test50_files(output_dir_template, step, result, default_dir):
    template = str(output_dir_template or default_dir or "")
    if not template:
        return None
    uses_placeholder = "{" in template and "}" in template
    step_label = _test50_step_label(step)
    try:
        out_dir = template.format(step=int(step), step_label=step_label)
    except Exception as exc:
        print(f"[test50] Failed to format test50_output_dir={template!r}: {exc}", flush=True)
        return None

    result = dict(result)
    result.setdefault("step", int(step))
    result.setdefault("step_label", step_label)
    os.makedirs(out_dir, exist_ok=True)
    filename = "result.json" if uses_placeholder else f"step_{int(step)}.json"
    out_path = os.path.join(out_dir, filename)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2, sort_keys=True)
    if filename != "latest.json":
        with open(os.path.join(out_dir, "latest.json"), "w", encoding="utf-8") as f:
            json.dump(result, f, indent=2, sort_keys=True)
    with open(os.path.join(out_dir, "summary.md"), "w", encoding="utf-8") as f:
        f.write("| key | value |\n| - | - |\n")
        for key in sorted(result):
            f.write(f"| {key} | {result[key]} |\n")
    return out_path


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


def _info_float(info, key, default=0.0):
    if isinstance(info, dict):
        return _as_float(info.get(key, default), default)
    return float(default)


def _info_max_epv_improvement(info):
    if isinstance(info, dict) and "max_epv_improvement" in info:
        return _info_float(info, "max_epv_improvement", 0.0)
    return _info_float(info, "max_episode_epv", 0.0) - _info_float(info, "initial_episode_epv", 0.0)



class _DiscreteActionDiagnostics:
    """Accumulates MAPPO discrete action/mask stats without changing behavior."""

    def __init__(self, n_actions=None, default_action_idx=17, empty_action_idx=18):
        self.n_actions = int(n_actions) if n_actions is not None else None
        self.default_action_idx = int(default_action_idx)
        self.empty_action_idx = int(empty_action_idx)
        self.reset()

    def reset(self):
        self.total = 0
        self.invalid_action_count = 0
        self.action_counts = None if self.n_actions is None else np.zeros(self.n_actions, dtype=np.int64)
        self.active_total = 0
        self.inactive_total = 0
        self.active_action_counts = None if self.n_actions is None else np.zeros(self.n_actions, dtype=np.int64)
        self.inactive_action_counts = None if self.n_actions is None else np.zeros(self.n_actions, dtype=np.int64)
        self.mask_obs_count = 0
        self.mask_valid_sum = 0.0
        self.mask_selected_valid = 0
        self.mask_selected_invalid = 0

    def _ensure_n_actions(self, n_actions):
        n_actions = int(n_actions)
        if self.n_actions is None:
            self.n_actions = n_actions
            self.action_counts = np.zeros(self.n_actions, dtype=np.int64)
            self.active_action_counts = np.zeros(self.n_actions, dtype=np.int64)
            self.inactive_action_counts = np.zeros(self.n_actions, dtype=np.int64)
        elif n_actions > self.n_actions:
            extra = n_actions - self.n_actions
            self.action_counts = np.pad(self.action_counts, (0, extra))
            self.active_action_counts = np.pad(self.active_action_counts, (0, extra))
            self.inactive_action_counts = np.pad(self.inactive_action_counts, (0, extra))
            self.n_actions = n_actions

    def observe(self, actions, available_actions=None, active_masks=None):
        if actions is None:
            return
        action_arr = np.asarray(actions)
        if action_arr.ndim >= 3 and action_arr.shape[-1] == 1:
            action_arr = action_arr[..., 0]
        action_flat = action_arr.reshape(-1).astype(np.int64)

        avail_flat = None
        if available_actions is not None:
            avail = np.asarray(available_actions)
            if avail.ndim >= 3:
                self._ensure_n_actions(avail.shape[-1])
                avail_flat = avail.reshape(-1, avail.shape[-1])

        if self.n_actions is None:
            max_action = int(action_flat.max()) if action_flat.size else 0
            self._ensure_n_actions(max(max_action + 1, self.empty_action_idx + 1))

        active_flat = None
        if active_masks is not None:
            active_arr = np.asarray(active_masks)
            if active_arr.ndim >= 3 and active_arr.shape[-1] == 1:
                active_arr = active_arr[..., 0]
            active_flat = (active_arr.reshape(-1) > 0.5)
            if active_flat.size != action_flat.size:
                active_flat = None

        for idx, action_idx in enumerate(action_flat):
            self.total += 1
            valid_action = 0 <= action_idx < self.n_actions
            if valid_action:
                self.action_counts[action_idx] += 1
            else:
                self.invalid_action_count += 1

            is_active = True if active_flat is None else bool(active_flat[idx])
            if is_active:
                self.active_total += 1
                if valid_action:
                    self.active_action_counts[action_idx] += 1
            else:
                self.inactive_total += 1
                if valid_action:
                    self.inactive_action_counts[action_idx] += 1

            if avail_flat is not None and idx < avail_flat.shape[0]:
                mask = avail_flat[idx].astype(bool)
                self.mask_obs_count += 1
                self.mask_valid_sum += float(mask.sum())
                if valid_action and mask[action_idx]:
                    self.mask_selected_valid += 1
                else:
                    self.mask_selected_invalid += 1

    def to_scalars(self):
        if self.total <= 0 or self.n_actions is None:
            return {}

        denom = float(max(1, self.total))
        active_denom = float(max(1, self.active_total))
        inactive_denom = float(max(1, self.inactive_total))
        scalars = {
            "samples": float(self.total),
            "invalid_action_count": float(self.invalid_action_count),
            "invalid_action_rate": self.invalid_action_count / denom,
            "active_sample_rate": self.active_total / denom,
            "inactive_sample_rate": self.inactive_total / denom,
        }

        if self.empty_action_idx < self.n_actions:
            empty_total = int(self.action_counts[self.empty_action_idx])
            active_empty = int(self.active_action_counts[self.empty_action_idx])
            inactive_empty = int(self.inactive_action_counts[self.empty_action_idx])
            scalars["empty_action_rate"] = empty_total / denom
            scalars["active_empty_action_rate"] = active_empty / active_denom
            scalars["inactive_empty_action_rate"] = inactive_empty / inactive_denom
            scalars["inactive_nonempty_action_rate"] = (self.inactive_total - inactive_empty) / inactive_denom

        if self.default_action_idx < self.n_actions:
            scalars["default_action_rate"] = int(self.action_counts[self.default_action_idx]) / denom
            scalars["active_default_action_rate"] = int(self.active_action_counts[self.default_action_idx]) / active_denom

        for action_idx, count in enumerate(self.action_counts):
            scalars[f"action_{action_idx}_count"] = float(count)
            scalars[f"action_{action_idx}_rate"] = float(count) / denom
            scalars[f"active_action_{action_idx}_rate"] = float(self.active_action_counts[action_idx]) / active_denom
            scalars[f"inactive_action_{action_idx}_rate"] = float(self.inactive_action_counts[action_idx]) / inactive_denom

        if self.mask_obs_count > 0:
            mask_denom = float(self.mask_obs_count)
            scalars["available_actions_mean"] = self.mask_valid_sum / mask_denom
            scalars["selected_mask_valid_rate"] = self.mask_selected_valid / mask_denom
            scalars["selected_mask_invalid_rate"] = self.mask_selected_invalid / mask_denom

        return scalars

    def _top_string(self, counts, denom, limit=4):
        if self.n_actions is None or denom <= 0:
            return "none"
        order = np.argsort(counts)[::-1]
        parts = []
        for action_idx in order[:limit]:
            count = int(counts[action_idx])
            if count <= 0:
                continue
            parts.append(f"{int(action_idx)}:{count / float(denom):.3f}")
        return ",".join(parts) if parts else "none"

    def summary_string(self):
        if self.total <= 0 or self.n_actions is None:
            return "actions=no-samples"
        scalars = self.to_scalars()
        top = self._top_string(self.action_counts, self.total)
        active_top = self._top_string(self.active_action_counts, self.active_total)
        return (
            f"actions mask_valid={scalars.get('selected_mask_valid_rate', 0.0):.3f} "
            f"active_rate={scalars.get('active_sample_rate', 0.0):.3f} "
            f"empty={scalars.get('empty_action_rate', 0.0):.3f} "
            f"active_empty={scalars.get('active_empty_action_rate', 0.0):.3f} "
            f"inactive_nonempty={scalars.get('inactive_nonempty_action_rate', 0.0):.3f} "
            f"top={top} active_top={active_top}"
        )


class FootballRunner(Runner):
    def __init__(self, config):
        super(FootballRunner, self).__init__(config)
        self._make_eval_env = config.get("make_eval_env")
        self._eval_on_demand = _as_bool(getattr(self.all_args, "robocup_eval_on_demand", False))
        if self._eval_on_demand and self._make_eval_env is None:
            raise RuntimeError("robocup_eval_on_demand=True requires make_eval_env in runner config")
        if self.use_eval and self._eval_on_demand:
            print("[eval_on_demand] enabled; eval envs will be created per eval and closed afterwards", flush=True)
        self.env_infos = defaultdict(list)
        self._train_episode_rewards = np.zeros(self.n_rollout_threads, dtype=np.float32)
        self._train_episode_lengths = np.zeros(self.n_rollout_threads, dtype=np.int32)
        self._train_episode_count = 0
        self._total_env_steps = 0
        self._train_episode_returns_since_log = []
        self._train_episode_lengths_since_log = []
        self._train_action_diag = _DiscreteActionDiagnostics(self._action_dim())
        self._reuse_eval_state = (
            (not self._eval_on_demand)
            and _as_bool(getattr(self.all_args, "robocup_reuse_eval_state", False))
            and int(self.n_eval_rollout_threads) == 1
        )
        self._eval_cached_obs = None
        self._eval_cached_available_actions = None
        self._eval_cached_active_masks = None
        self._test50_during_training = _as_bool(getattr(self.all_args, "test50_during_training", False))
        self._log_robocup_metadata()

    def _action_dim(self):
        try:
            action_space = getattr(self.envs, "action_space", None)
            if isinstance(action_space, (list, tuple)) and action_space:
                return int(action_space[0].n)
            if hasattr(action_space, "n"):
                return int(action_space.n)
        except Exception:
            pass
        return None

    def _log_action_diagnostics(self, diagnostics, total_num_steps, prefix):
        if diagnostics is None:
            return
        for key, value in diagnostics.to_scalars().items():
            self._write_scalar(f"{prefix}/{key}", value, total_num_steps)

    def _write_scalar(self, key, value, step):
        value = float(value)
        if self.use_wandb:
            wandb.log({key: value}, step=step)
        else:
            self.writter.add_scalar(key, value, step)

    def _write_text(self, key, text, step):
        if self.use_wandb:
            if wandb.run is not None:
                wandb.run.summary[key] = text
        elif hasattr(self.writter, "add_text"):
            self.writter.add_text(key, text, step)

    def _write_test50_summary(self, total_num_steps, metrics):
        if not self._test50_during_training:
            return

        result = {
            "algorithm": str(self.algorithm_name),
            "scenario": str(self.scenario_name) if hasattr(self, "scenario_name") else str(getattr(self.all_args, "scenario_name", "")),
            "step": int(total_num_steps),
            "step_label": _test50_step_label(total_num_steps),
            "eval_episodes": int(getattr(self.all_args, "eval_episodes", 0) or 0),
            "requested_episodes": int(getattr(self.all_args, "eval_episodes", 0) or 0),
        }
        for key, value in (metrics or {}).items():
            if isinstance(value, (np.floating, float, int, np.integer)):
                result[key] = float(value) if not isinstance(value, (int, np.integer)) else int(value)
            else:
                result[key] = value

        default_dir = os.path.join(str(self.run_dir), "test50")
        out_path = _write_test50_files(
            getattr(self.all_args, "test50_output_dir", None),
            total_num_steps,
            result,
            default_dir,
        )
        if out_path:
            print(f"[test50] Wrote {out_path}", flush=True)

    def _log_robocup_metadata(self):
        env_args = getattr(self.all_args, "robocup_env_args", {}) or {}
        if not env_args:
            return
        train_players = int(_cfg_get(env_args, "init_n", getattr(self.all_args, "train_players", self.num_agents)))
        total_players = int(_cfg_get(env_args, "n", getattr(self.all_args, "total_players_per_side", self.num_agents)))
        opponent_players = int(_active_right_count(env_args))
        mode = _reward_mode(env_args)
        mode_code = {"goal_only": 1.0, "epv_shaped": 2.0, "custom": 3.0}.get(mode, 0.0)

        self._write_scalar("config/train_players", train_players, 0)
        self._write_scalar("config/opponent_players", opponent_players, 0)
        self._write_scalar("config/total_players_per_side", total_players, 0)
        self._write_scalar("config/num_envs", self.n_rollout_threads, 0)
        self._write_scalar("config/n_agents", self.num_agents, 0)
        self._write_scalar("config/algorithm/obs_agent_id", 1.0 if _as_bool(getattr(self.all_args, "robocup_obs_agent_id", True)) else 0.0, 0)
        self._write_scalar("config/reward/mode_code", mode_code, 0)
        self._write_scalar("config/reward/is_goal_only", 1.0 if mode == "goal_only" else 0.0, 0)
        self._write_scalar("config/reward/epv_progress_scale", _as_float(_cfg_get(env_args, "epv_progress_scale", 1.0)), 0)
        for key in ROBOCUP_REWARD_KEYS:
            self._write_scalar(f"config/reward/{key}", _as_float(_cfg_get(env_args, key, 0.0)), 0)
        for key in ROBOCUP_TERMINATION_KEYS:
            self._write_scalar(f"config/termination/{key}", 1.0 if _as_bool(_cfg_get(env_args, key, False)) else 0.0, 0)

        rows = [
            ("algorithm", self.algorithm_name),
            ("env_config", getattr(self.all_args, "robocup_env_config", "")),
            ("start_id", _cfg_get(env_args, "start_id", "")),
            ("train_players", train_players),
            ("opponent_players", opponent_players),
            ("total_players_per_side", total_players),
            ("num_envs", self.n_rollout_threads),
            ("obs_agent_id", _as_bool(getattr(self.all_args, "robocup_obs_agent_id", True))),
            ("reward_mode", mode),
            ("useMaxEpv", _cfg_get(env_args, "useMaxEpv", False)),
            ("epv_progress_scale", _cfg_get(env_args, "epv_progress_scale", 1.0)),
            ("epv_grid_file", _cfg_get(env_args, "epv_grid_file", "EPV_grid.csv")),
        ]
        rows.extend((key, _cfg_get(env_args, key, 0.0)) for key in ROBOCUP_REWARD_KEYS)
        markdown = "| key | value |\n| - | - |\n" + "\n".join(f"| {k} | {v} |" for k, v in rows)
        self._write_text("config/summary", markdown, 0)

    def run(self):
        self.warmup()

        start = time.time()
        resume_env_steps = max(0, int(getattr(self.all_args, "resume_env_steps", 0) or 0))
        remaining_env_steps = max(0, int(self.num_env_steps) - resume_env_steps)
        episodes = remaining_env_steps // self.episode_length // self.n_rollout_threads
        if resume_env_steps > 0:
            print(f"[resume] loaded model with resume_env_steps={resume_env_steps}; training remaining_env_steps={remaining_env_steps}.", flush=True)
        log_interval = max(1, int(self.log_interval))
        eval_interval = max(1, int(self.eval_interval))

        def _next_interval_step(interval):
            if resume_env_steps <= 0:
                return interval
            return ((resume_env_steps // interval) + 1) * interval

        next_log_step = _next_interval_step(log_interval)
        next_eval_step = _next_interval_step(eval_interval)
        last_save_step = 0
        save_interval = max(1, int(self.save_interval))
        next_save_step = _next_interval_step(save_interval)

        for episode in range(episodes):
            if self.use_linear_lr_decay:
                self.trainer.policy.lr_decay(episode, episodes)

            for step in range(self.episode_length):
                # Sample actions
                values, actions, action_log_probs, rnn_states, rnn_states_critic, actions_env = self.collect(step)
                current_available_actions = (
                    self.buffer.available_actions[step]
                    if self.buffer.available_actions is not None
                    else None
                )
                current_active_masks = (
                    self.buffer.active_masks[step]
                    if hasattr(self.buffer, "active_masks") and self.buffer.active_masks is not None
                    else None
                )
                self._train_action_diag.observe(actions, current_available_actions, current_active_masks)

                # Obser reward and next obs
                obs, rewards, dones, infos = self.envs.step(actions_env)
                available_actions = (
                    self.envs.get_avail_actions()
                    if self.buffer.available_actions is not None and hasattr(self.envs, "get_avail_actions")
                    else None
                )
                active_masks = (
                    self.envs.get_active_masks()
                    if hasattr(self.envs, "get_active_masks")
                    else None
                )

                data = (
                    obs, rewards, dones, infos, values, actions, action_log_probs,
                    rnn_states, rnn_states_critic, available_actions, active_masks
                )

                # insert data into buffer
                self.insert(data)

            # compute return and update network
            self.compute()
            train_infos = self.train()

            # post process
            total_num_steps = resume_env_steps + (episode + 1) * self.episode_length * self.n_rollout_threads

            # save model
            if total_num_steps >= next_save_step:
                self.save(total_num_steps)
                last_save_step = total_num_steps
                while next_save_step <= total_num_steps:
                    next_save_step += save_interval

            # log information
            if total_num_steps >= next_log_step:
                end = time.time()
                print("\n Env {} Algo {} Exp {} updates {}/{} episodes, total num timesteps {}/{}, FPS {}.\n"
                        .format(self.env_name,
                                self.algorithm_name,
                                self.experiment_name,
                                episode,
                                episodes,
                                total_num_steps,
                                self.num_env_steps,
                                int(total_num_steps / (end - start))))

                rollout_reward_per_thread = float(np.mean(self.buffer.rewards) * self.episode_length)
                if self._train_episode_returns_since_log:
                    train_episode_reward_mean = float(np.mean(self._train_episode_returns_since_log))
                    train_episode_length_mean = float(np.mean(self._train_episode_lengths_since_log))
                else:
                    train_episode_reward_mean = 0.0
                    train_episode_length_mean = 0.0
                train_infos["average_episode_rewards"] = train_episode_reward_mean
                train_infos["train/episode_reward_mean"] = train_episode_reward_mean
                train_infos["train/episode_length_mean"] = train_episode_length_mean
                train_infos["train/completed_episodes"] = float(len(self._train_episode_returns_since_log))
                train_infos["train/rollout_reward_per_thread"] = rollout_reward_per_thread
                print("train episode reward mean is {} over {} completed episodes (rollout reward/thread {})".format(train_episode_reward_mean, len(self._train_episode_returns_since_log), rollout_reward_per_thread))
                print("action diagnostics: {}".format(self._train_action_diag.summary_string()))
                self.log_train(train_infos, total_num_steps)
                self._train_episode_returns_since_log = []
                self._train_episode_lengths_since_log = []
                self._log_action_diagnostics(self._train_action_diag, total_num_steps, "diagnostics/train_actions")
                self._train_action_diag.reset()
                self.log_env(self.env_infos, total_num_steps)
                self.env_infos = defaultdict(list)
                while next_log_step <= total_num_steps:
                    next_log_step += log_interval

            # eval
            if self.use_eval and total_num_steps >= next_eval_step:
                metrics = self.eval(total_num_steps)
                self._write_test50_summary(total_num_steps, metrics or {})
                while next_eval_step <= total_num_steps:
                    next_eval_step += eval_interval

        final_num_steps = resume_env_steps + episodes * self.episode_length * self.n_rollout_threads
        if final_num_steps > 0 and last_save_step != final_num_steps:
            print(f"Saving final model at total num timesteps {final_num_steps}.")
            self.save(final_num_steps)

    def warmup(self):
        # reset env
        obs = self.envs.reset()

        # insert obs to buffer
        self.buffer.share_obs[0] = obs.copy()
        self.buffer.obs[0] = obs.copy()
        if self.buffer.available_actions is not None and hasattr(self.envs, "get_avail_actions"):
            self.buffer.available_actions[0] = self.envs.get_avail_actions().copy()
        if hasattr(self.envs, "get_active_masks"):
            self.buffer.active_masks[0] = self.envs.get_active_masks().copy()

    @torch.no_grad()
    def collect(self, step):
        self.trainer.prep_rollout()

        # [n_envs, n_agents, ...] -> [n_envs*n_agents, ...]
        available_actions = (
            np.concatenate(self.buffer.available_actions[step])
            if self.buffer.available_actions is not None
            else None
        )
        values, actions, action_log_probs, rnn_states, rnn_states_critic = self.trainer.policy.get_actions(
            np.concatenate(self.buffer.share_obs[step]),
            np.concatenate(self.buffer.obs[step]),
            np.concatenate(self.buffer.rnn_states[step]),
            np.concatenate(self.buffer.rnn_states_critic[step]),
            np.concatenate(self.buffer.masks[step]),
            available_actions=available_actions
        )

        # [n_envs*n_agents, ...] -> [n_envs, n_agents, ...]
        values = np.array(np.split(_t2n(values), self.n_rollout_threads))
        actions = np.array(np.split(_t2n(actions), self.n_rollout_threads))
        action_log_probs = np.array(np.split(_t2n(action_log_probs), self.n_rollout_threads))
        rnn_states = np.array(np.split(_t2n(rnn_states), self.n_rollout_threads))
        rnn_states_critic = np.array(np.split(_t2n(rnn_states_critic), self.n_rollout_threads))

        actions_env = [actions[idx, :, 0] for idx in range(self.n_rollout_threads)]

        return values, actions, action_log_probs, rnn_states, rnn_states_critic, actions_env

    def insert(self, data):
        obs, rewards, dones, infos, values, actions, action_log_probs, rnn_states, rnn_states_critic, available_actions, active_masks = data

        reward_per_env = np.asarray(rewards, dtype=np.float32).reshape(self.n_rollout_threads, self.num_agents, -1).mean(axis=(1, 2))
        self._train_episode_rewards += reward_per_env
        self._train_episode_lengths += 1
        self._total_env_steps += int(self.n_rollout_threads)

        # update env_infos if done
        dones_env = np.all(dones, axis=-1)
        if np.any(dones_env):
            for env_idx, (done, info) in enumerate(zip(dones_env, infos)):
                if done:
                    self._train_episode_count += 1
                    episode_step = int(self._total_env_steps)
                    episode_reward = float(self._train_episode_rewards[env_idx])
                    episode_length = float(self._train_episode_lengths[env_idx])
                    self._train_episode_returns_since_log.append(episode_reward)
                    self._train_episode_lengths_since_log.append(episode_length)
                    self._write_scalar("train/episode_reward", episode_reward, episode_step)
                    self._write_scalar("train/average_episode_return", episode_reward, episode_step)
                    self._write_scalar("train/episode_length", episode_length, episode_step)
                    self._write_scalar("train/episode_count", float(self._train_episode_count), episode_step)
                    self._write_scalar("train/possession_loss_rate", _info_float(info, "possession_loss", 0.0), episode_step)
                    self._write_scalar("train/max_epv", _info_float(info, "max_episode_epv", 0.0), episode_step)
                    self._write_scalar("train/max_epv_improvement", _info_max_epv_improvement(info), episode_step)
                    self._train_episode_rewards[env_idx] = 0.0
                    self._train_episode_lengths[env_idx] = 0
                    score_reward = _info_float(info, "score_reward", 0.0)
                    self.env_infos["goal"].append(score_reward)
                    if score_reward > 0:
                        self.env_infos["win_rate"].append(1)
                    else:
                        self.env_infos["win_rate"].append(0)
                    self.env_infos["steps"].append(_info_float(info, "max_steps", 0.0) - _info_float(info, "steps_left", 0.0))

        # reset rnn and mask args for done envs
        rnn_states[dones_env == True] = np.zeros(((dones_env == True).sum(), self.num_agents, self.recurrent_N, self.hidden_size), dtype=np.float32)
        rnn_states_critic[dones_env == True] = np.zeros(((dones_env == True).sum(), self.num_agents, self.recurrent_N, self.hidden_size), dtype=np.float32)
        masks = np.ones((self.n_rollout_threads, self.num_agents, 1), dtype=np.float32)
        masks[dones_env == True] = np.zeros(((dones_env == True).sum(), self.num_agents, 1), dtype=np.float32)

        self.buffer.insert(
            share_obs=obs,
            obs=obs,
            rnn_states_actor=rnn_states,
            rnn_states_critic=rnn_states_critic,
            actions=actions,
            action_log_probs=action_log_probs,
            value_preds=values,
            rewards=rewards,
            masks=masks,
            active_masks=active_masks,
            available_actions=available_actions
        )

    def log_env(self, env_infos, total_num_steps):
        for k, v in env_infos.items():
            if len(v) > 0:
                if self.use_wandb:
                    wandb.log({k: np.mean(v)}, step=total_num_steps)
                else:
                    self.writter.add_scalar(k, np.mean(v), total_num_steps)

    def _clear_eval_cache(self):
        self._eval_cached_obs = None
        self._eval_cached_available_actions = None
        self._eval_cached_active_masks = None

    def _create_on_demand_eval_envs(self):
        if self._make_eval_env is None:
            raise RuntimeError("No eval env factory configured for on-demand eval.")
        print("[eval_on_demand] creating eval envs", flush=True)
        return self._make_eval_env(self.all_args)

    @torch.no_grad()
    def eval(self, total_num_steps):
        previous_eval_envs = self.eval_envs
        eval_envs_to_close = None
        if self._eval_on_demand:
            self._clear_eval_cache()
            eval_envs_to_close = self._create_on_demand_eval_envs()
            self.eval_envs = eval_envs_to_close
        elif self.eval_envs is None:
            raise RuntimeError("use_eval=True but no eval_envs are configured.")

        try:
            return self._eval_impl(total_num_steps)
        finally:
            if eval_envs_to_close is not None:
                try:
                    eval_envs_to_close.close()
                finally:
                    self.eval_envs = previous_eval_envs
                    self._clear_eval_cache()
                    print("[eval_on_demand] closed eval envs", flush=True)

    def _eval_impl(self, total_num_steps):
        # DummyVecEnv already resets finished eval episodes in step_wait.
        # Reuse that post-episode state to avoid an extra reset after long idle periods.
        reuse_cached_eval_state = self._reuse_eval_state and self._eval_cached_obs is not None
        if reuse_cached_eval_state:
            eval_obs = self._eval_cached_obs.copy()
            eval_available_actions = (
                None
                if self._eval_cached_available_actions is None
                else self._eval_cached_available_actions.copy()
            )
            eval_active_masks = (
                None
                if self._eval_cached_active_masks is None
                else self._eval_cached_active_masks.copy()
            )
            print("[eval_cache] reuse cached eval env state", flush=True)
        else:
            eval_obs = self.eval_envs.reset()
            eval_available_actions = (
                self.eval_envs.get_avail_actions()
                if hasattr(self.eval_envs, "get_avail_actions")
                else None
            )
            eval_active_masks = (
                self.eval_envs.get_active_masks()
                if hasattr(self.eval_envs, "get_active_masks")
                else None
            )
            if self._reuse_eval_state:
                print("[eval_cache] initialized eval env state with reset", flush=True)
        eval_action_diag = _DiscreteActionDiagnostics(self._action_dim())
        eval_rnn_states = np.zeros((self.n_eval_rollout_threads, self.num_agents, self.recurrent_N, self.hidden_size), dtype=np.float32)
        eval_masks = np.ones((self.n_eval_rollout_threads, self.num_agents, 1), dtype=np.float32)

        # init eval goals
        num_done = 0
        eval_goals = np.zeros(self.all_args.eval_episodes)
        eval_win_rates = np.zeros(self.all_args.eval_episodes)
        eval_steps = np.zeros(self.all_args.eval_episodes)
        eval_episode_returns = np.zeros(self.all_args.eval_episodes)
        eval_possession_losses = np.zeros(self.all_args.eval_episodes)
        eval_max_epvs = np.zeros(self.all_args.eval_episodes)
        eval_max_epv_improvements = np.zeros(self.all_args.eval_episodes)
        eval_goal_diffs = np.zeros(self.all_args.eval_episodes)
        eval_score_lefts = np.zeros(self.all_args.eval_episodes)
        eval_score_rights = np.zeros(self.all_args.eval_episodes)
        eval_returns_by_thread = np.zeros(self.n_eval_rollout_threads, dtype=np.float32)
        step = 0
        quo = self.all_args.eval_episodes // self.n_eval_rollout_threads
        rem = self.all_args.eval_episodes % self.n_eval_rollout_threads
        done_episodes_per_thread = np.zeros(self.n_eval_rollout_threads, dtype=int)
        eval_episodes_per_thread = done_episodes_per_thread + quo
        eval_episodes_per_thread[:rem] += 1
        unfinished_thread = (done_episodes_per_thread != eval_episodes_per_thread)
        max_eval_steps = int(np.max(eval_episodes_per_thread)) * self.episode_length

        # loop until enough requested episodes are completed
        while num_done < self.all_args.eval_episodes and step < max_eval_steps:
            # get actions
            self.trainer.prep_rollout()

            # [n_envs, n_agents, ...] -> [n_envs*n_agents, ...]
            eval_actions, eval_rnn_states = self.trainer.policy.act(
                np.concatenate(eval_obs),
                np.concatenate(eval_rnn_states),
                np.concatenate(eval_masks),
                available_actions=(
                    np.concatenate(eval_available_actions)
                    if eval_available_actions is not None
                    else None
                ),
                deterministic=self.all_args.eval_deterministic
            )

            # [n_envs*n_agents, ...] -> [n_envs, n_agents, ...]
            eval_actions = np.array(np.split(_t2n(eval_actions), self.n_eval_rollout_threads))
            eval_rnn_states = np.array(np.split(_t2n(eval_rnn_states), self.n_eval_rollout_threads))

            eval_action_diag.observe(eval_actions, eval_available_actions, eval_active_masks)
            eval_actions_env = [eval_actions[idx, :, 0] for idx in range(self.n_eval_rollout_threads)]

            # step
            eval_obs, eval_rewards, eval_dones, eval_infos = self.eval_envs.step(eval_actions_env)
            eval_available_actions = (
                self.eval_envs.get_avail_actions()
                if hasattr(self.eval_envs, "get_avail_actions")
                else None
            )
            eval_active_masks = (
                self.eval_envs.get_active_masks()
                if hasattr(self.eval_envs, "get_active_masks")
                else None
            )
            reward_per_env = np.asarray(eval_rewards, dtype=np.float32).reshape(self.n_eval_rollout_threads, self.num_agents, -1).mean(axis=(1, 2))
            eval_returns_by_thread += reward_per_env

            # update goals if done
            eval_dones_env = np.all(eval_dones, axis=-1)
            eval_dones_unfinished_env = eval_dones_env[unfinished_thread]
            if np.any(eval_dones_unfinished_env):
                for idx_env in range(self.n_eval_rollout_threads):
                    if unfinished_thread[idx_env] and eval_dones_env[idx_env]:
                        if num_done >= self.all_args.eval_episodes:
                            break
                        info = eval_infos[idx_env]
                        score_reward = _info_float(info, "score_reward", 0.0)
                        eval_goals[num_done] = score_reward
                        eval_win_rates[num_done] = 1 if score_reward > 0 else 0
                        eval_steps[num_done] = _info_float(info, "max_steps", 0.0) - _info_float(info, "steps_left", 0.0)
                        eval_episode_returns[num_done] = eval_returns_by_thread[idx_env]
                        eval_returns_by_thread[idx_env] = 0.0
                        eval_possession_losses[num_done] = _info_float(info, "possession_loss", 0.0)
                        eval_max_epvs[num_done] = _info_float(info, "max_episode_epv", 0.0)
                        eval_max_epv_improvements[num_done] = _info_max_epv_improvement(info)
                        eval_goal_diffs[num_done] = _info_float(info, "score_diff_reward", _info_float(info, "score_diff", 0.0))
                        eval_score_lefts[num_done] = _info_float(info, "score_left", 0.0)
                        eval_score_rights[num_done] = _info_float(info, "score_right", 0.0)
                        # print("episode {:>2d} done by env {:>2d}: {}".format(num_done, idx_env, score_reward))
                        num_done += 1
                        done_episodes_per_thread[idx_env] += 1
            unfinished_thread = (done_episodes_per_thread != eval_episodes_per_thread)

            # reset rnn and masks for done envs
            eval_rnn_states[eval_dones_env == True] = np.zeros(((eval_dones_env == True).sum(), self.num_agents, self.recurrent_N, self.hidden_size), dtype=np.float32)
            eval_masks = np.ones((self.all_args.n_eval_rollout_threads, self.num_agents, 1), dtype=np.float32)
            eval_masks[eval_dones_env == True] = np.zeros(((eval_dones_env == True).sum(), self.num_agents, 1), dtype=np.float32)
            step += 1

        # get expected goal
        done_count = max(1, num_done)
        eval_goal = np.mean(eval_goals[:done_count])
        eval_win_rate = np.mean(eval_win_rates[:done_count])
        eval_step = np.mean(eval_steps[:done_count])
        eval_return = np.mean(eval_episode_returns[:done_count])
        eval_possession_loss_rate = np.mean(eval_possession_losses[:done_count])
        eval_max_epv = np.mean(eval_max_epvs[:done_count])
        eval_max_epv_improvement = np.mean(eval_max_epv_improvements[:done_count])
        eval_goal_diff = np.mean(eval_goal_diffs[:done_count])
        eval_score_left = np.mean(eval_score_lefts[:done_count])
        eval_score_right = np.mean(eval_score_rights[:done_count])

        # log and print
        print("eval expected goal is {} over {}/{} completed episodes.".format(eval_goal, num_done, self.all_args.eval_episodes))
        print("eval action diagnostics: {}".format(eval_action_diag.summary_string()))
        self._log_action_diagnostics(eval_action_diag, total_num_steps, "diagnostics/eval_actions")
        self._write_scalar("eval_goal", eval_goal, total_num_steps)
        self._write_scalar("eval_win_rate", eval_win_rate, total_num_steps)
        self._write_scalar("eval/goal", eval_goal, total_num_steps)
        self._write_scalar("eval/win_rate", eval_win_rate, total_num_steps)
        self._write_scalar("eval/average_episode_return", eval_return, total_num_steps)
        self._write_scalar("eval/episode_length", eval_step, total_num_steps)
        self._write_scalar("test_win_rate", eval_win_rate, total_num_steps)
        self._write_scalar("test/win_rate", eval_win_rate, total_num_steps)
        self._write_scalar("test/goal_rate", eval_win_rate, total_num_steps)
        self._write_scalar("test/completed_episodes", float(num_done), total_num_steps)
        self._write_scalar("test/requested_episodes", float(self.all_args.eval_episodes), total_num_steps)
        self._write_scalar("eval_step", eval_step, total_num_steps)
        self._write_scalar("test/average_episode_return", eval_return, total_num_steps)
        self._write_scalar("test/possession_loss_rate", eval_possession_loss_rate, total_num_steps)
        self._write_scalar("test/episode_length", eval_step, total_num_steps)
        self._write_scalar("test/max_epv", eval_max_epv, total_num_steps)
        self._write_scalar("test/max_epv_improvement", eval_max_epv_improvement, total_num_steps)
        self._write_scalar("eval/mean_goal_diff", eval_goal_diff, total_num_steps)
        self._write_scalar("test/mean_goal_diff", eval_goal_diff, total_num_steps)
        self._write_scalar(
            "diagnostics/eval_reused_state",
            1.0 if reuse_cached_eval_state else 0.0,
            total_num_steps,
        )

        if self._reuse_eval_state and num_done >= int(self.all_args.eval_episodes):
            self._eval_cached_obs = eval_obs.copy()
            self._eval_cached_available_actions = (
                None if eval_available_actions is None else eval_available_actions.copy()
            )
            self._eval_cached_active_masks = (
                None if eval_active_masks is None else eval_active_masks.copy()
            )
        else:
            self._eval_cached_obs = None
            self._eval_cached_available_actions = None
            self._eval_cached_active_masks = None

        return {
            "eval_goal": float(eval_goal),
            "eval_win_rate": float(eval_win_rate),
            "test_win_rate": float(eval_win_rate),
            "mean_return": float(eval_return),
            "episode_length": float(eval_step),
            "possession_loss_rate": float(eval_possession_loss_rate),
            "max_epv": float(eval_max_epv),
            "max_epv_improvement": float(eval_max_epv_improvement),
            "mean_goal_diff": float(eval_goal_diff),
            "mean_score_diff": float(eval_goal_diff),
            "mean_score_left": float(eval_score_left),
            "mean_score_right": float(eval_score_right),
            "completed_episodes": int(num_done),
            "requested_episodes": int(self.all_args.eval_episodes),
        }

    @torch.no_grad()
    def render(self):
        # reset envs and init rnn and mask
        render_env = self.envs

        # init goal
        render_goals = np.zeros(self.all_args.render_episodes)
        for i_episode in range(self.all_args.render_episodes):
            render_obs = render_env.reset()
            render_rnn_states = np.zeros((self.n_rollout_threads, self.num_agents, self.recurrent_N, self.hidden_size), dtype=np.float32)
            render_masks = np.ones((self.n_rollout_threads, self.num_agents, 1), dtype=np.float32)

            if self.all_args.save_gifs:
                frames = []
                image = self.envs.envs[0].env.unwrapped.observation()[0]["frame"]
                frames.append(image)

            render_dones = False
            while not np.any(render_dones):
                self.trainer.prep_rollout()
                render_actions, render_rnn_states = self.trainer.policy.act(
                    np.concatenate(render_obs),
                    np.concatenate(render_rnn_states),
                    np.concatenate(render_masks),
                    deterministic=True
                )

                # [n_envs*n_agents, ...] -> [n_envs, n_agents, ...]
                render_actions = np.array(np.split(_t2n(render_actions), self.n_rollout_threads))
                render_rnn_states = np.array(np.split(_t2n(render_rnn_states), self.n_rollout_threads))

                render_actions_env = [render_actions[idx, :, 0] for idx in range(self.n_rollout_threads)]

                # step
                render_obs, render_rewards, render_dones, render_infos = render_env.step(render_actions_env)

                # append frame
                if self.all_args.save_gifs:
                    image = render_infos[0]["frame"]
                    frames.append(image)

            # print goal
            render_goals[i_episode] = render_rewards[0, 0]
            print("goal in episode {}: {}".format(i_episode, render_rewards[0, 0]))

            # save gif
            if self.all_args.save_gifs:
                imageio.mimsave(
                    uri="{}/episode{}.gif".format(str(self.gif_dir), i_episode),
                    ims=frames,
                    format="GIF",
                    duration=self.all_args.ifi,
                )

        print("expected goal: {}".format(np.mean(render_goals)))
