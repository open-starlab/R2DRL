"""Train ParaDQN on a demo parametrized environment.

This script provides:
 - SimpleParamEnv: a tiny toy environment with parametrized actions for demo/training
 - train() function: runs episodes, stores transitions, samples from replay and updates agent
 - evaluate() function: runs greedy episodes to report average return

Adaptation notes:
 - Replace SimpleParamEnv with any environment that exposes:
     obs = env.reset()
     next_obs, reward, done, info = env.step((action_idx, action_param))
"""

import os
import glob
import json
import shutil
from collections import defaultdict, deque
from pathlib import Path
from typing import Callable, Optional
import numpy as np
import torch
from torch.utils.tensorboard import SummaryWriter
from .agent import ParaDQNAgent
from .replay_buffer import ReplayBuffer, Transition
from environments.robocup2d_hybrid_helios_v0 import RobocupEnv_Hybrid_Helios
from datetime import datetime


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


def _write_training_test50_result(
    output_dir_template,
    checkpoint_dir,
    step,
    episode,
    eval_episodes,
    eval_policy,
    mean_r,
    std_r,
    rates,
):
    result = {
        "algorithm": "paradqn",
        "step": int(step),
        "step_label": _test50_step_label(step),
        "episode": int(episode),
        "eval_episodes": int(eval_episodes),
        "requested_episodes": int(eval_episodes),
        "eval_policy": str(eval_policy),
        "mean_return": float(mean_r),
        "std_return": float(std_r),
    }
    for key, value in (rates or {}).items():
        if str(key).startswith("eval_actions/"):
            continue
        try:
            result[key] = float(value)
        except (TypeError, ValueError):
            result[key] = value
    if "win_rate" in result:
        result.setdefault("test_win_rate", result["win_rate"])
    if "goal_rate" in result:
        result.setdefault("test_goal_rate", result["goal_rate"])

    default_dir = os.path.join(str(checkpoint_dir), "test50") if checkpoint_dir else None
    out_path = _write_test50_files(output_dir_template, step, result, default_dir)
    if out_path:
        print(f"[test50] Wrote {out_path}", flush=True)


def save_checkpoint(path: str, agent: ParaDQNAgent, episode: int = 0, total_env_steps: int = 0):
    state = {
        "episode": int(episode),
        "total_env_steps": int(total_env_steps),
        "q_state_dict": agent.q_net.state_dict(),
        "actor_state_dict": agent.actor.state_dict(),
        "q_target_state_dict": agent.q_target.state_dict(),
        "actor_target_state_dict": agent.actor_target.state_dict(),
        "q_optimizer": agent.q_optimizer.state_dict(),
        "actor_optimizer": agent.actor_optimizer.state_dict(),
        "agent_train_updates": int(getattr(agent, "train_updates", 0)),
    }
    torch.save(state, path)


def load_checkpoint(path: str, agent: ParaDQNAgent):
    try:
        ck = torch.load(path, map_location=agent.device)
        if "q_state_dict" in ck:
            agent.q_net.load_state_dict(ck["q_state_dict"])
        if "actor_state_dict" in ck:
            agent.actor.load_state_dict(ck["actor_state_dict"])
        if "q_target_state_dict" in ck:
            agent.q_target.load_state_dict(ck["q_target_state_dict"])
        elif "q_state_dict" in ck:
            agent.q_target.load_state_dict(agent.q_net.state_dict())
        if "actor_target_state_dict" in ck:
            agent.actor_target.load_state_dict(ck["actor_target_state_dict"])
        elif "actor_state_dict" in ck:
            agent.actor_target.load_state_dict(agent.actor.state_dict())
        if "q_optimizer" in ck and getattr(agent, "q_optimizer", None) is not None and ck["q_optimizer"] is not None:
            try:
                agent.q_optimizer.load_state_dict(ck["q_optimizer"])
            except Exception:
                pass
        if "actor_optimizer" in ck and getattr(agent, "actor_optimizer", None) is not None and ck["actor_optimizer"] is not None:
            try:
                agent.actor_optimizer.load_state_dict(ck["actor_optimizer"])
            except Exception:
                pass
        if "agent_train_updates" in ck and hasattr(agent, "train_updates"):
            try:
                agent.train_updates = int(ck["agent_train_updates"])
            except Exception:
                pass
        return ck
    except Exception as e:
        print(f"Failed to load checkpoint {path}: {e}")
        return None


def cleanup():
    for file in glob.glob("*.rcg"):
        os.remove(file)
    for file in glob.glob("*.rcl"):
        os.remove(file)


def _agent_info(infos, player="l1"):
    if isinstance(infos, dict):
        item = infos.get(player)
        if isinstance(item, dict):
            return item
        return infos
    return {}


def _dict_observation_arrays(observations, players):
    obs = np.stack([observations[player]["observation"] for player in players], axis=0).astype(np.float32, copy=False)
    masks = np.stack([observations[player]["action_mask"] for player in players], axis=0).astype(np.int8, copy=False)
    return obs, masks


def _actions_dict_from_arrays(players, action_nums, action_params):
    return {
        player: (int(action_nums[i]), action_params[i])
        for i, player in enumerate(players)
    }


def _float_info(info, key, default=0.0):
    try:
        return float(info.get(key, default))
    except (AttributeError, TypeError, ValueError):
        return float(default)


def _max_epv_improvement(info):
    if isinstance(info, dict) and "max_epv_improvement" in info:
        return _float_info(info, "max_epv_improvement", 0.0)
    return _float_info(info, "max_episode_epv", 0.0) - _float_info(info, "initial_episode_epv", 0.0)


class _NStepTransitionBuilder:
    def __init__(self, n_step: int, gamma: float):
        self.n_step = max(1, int(n_step or 1))
        self.gamma = float(gamma)
        self.buffers = defaultdict(deque)

    @staticmethod
    def _done(value) -> bool:
        arr = np.asarray(value, dtype=np.float32)
        return bool(arr.size and float(np.max(arr)) > 0.0)

    def _build(self, items):
        first = items[0]
        reward = np.zeros_like(np.asarray(first.reward, dtype=np.float32), dtype=np.float32)
        last = first
        used = 0
        discount = 1.0
        for item in items[: self.n_step]:
            reward = reward + discount * np.asarray(item.reward, dtype=np.float32)
            last = item
            used += 1
            if self._done(item.done):
                break
            discount *= self.gamma
        return Transition(
            obs=first.obs,
            shared_obs=first.shared_obs,
            action_mask=first.action_mask,
            act_num=first.act_num,
            act_param=first.act_param,
            reward=reward,
            done=last.done,
            next_obs=last.next_obs,
            next_shared_obs=last.next_shared_obs,
            next_action_mask=last.next_action_mask,
            n_step=used,
            priority=None,
        )

    def append(self, key, transition: Transition):
        if self.n_step <= 1:
            return [transition]
        buf = self.buffers[key]
        buf.append(transition)
        emitted = []
        if len(buf) >= self.n_step:
            emitted.append(self._build(list(buf)))
            buf.popleft()
        if self._done(transition.done):
            emitted.extend(self.flush(key))
        return emitted

    def flush(self, key):
        if self.n_step <= 1:
            return []
        buf = self.buffers.get(key)
        if not buf:
            return []
        emitted = []
        while buf:
            emitted.append(self._build(list(buf)))
            buf.popleft()
        self.buffers.pop(key, None)
        return emitted



class _ActionDiagnostics:
    """Accumulates action/mask stats without changing ParaDQN behavior."""

    def __init__(self, actions_num, param_dim, fallback_indices=(), kick_action_idx=2):
        self.actions_num = int(actions_num)
        self.param_dim = int(param_dim)
        self.fallback_indices = tuple(int(i) for i in fallback_indices)
        self.kick_action_idx = int(kick_action_idx) if int(kick_action_idx) < self.actions_num else None
        self.reset()

    def reset(self):
        self.total = 0
        self.invalid_action_count = 0
        self.action_counts = np.zeros(self.actions_num, dtype=np.int64)
        self.fallback_count = 0
        self.kick_count = 0
        self.kick_param_count = 0
        self.kick_param_sum = np.zeros(2, dtype=np.float64)
        self.kick_param_sq_sum = np.zeros(2, dtype=np.float64)
        self.mask_obs_count = 0
        self.mask_valid_sum = 0.0
        self.mask_selected_valid = 0
        self.mask_selected_invalid = 0
        self.kick_available_count = 0
        self.fallback_available_count = 0
        self.replay_push_count = 0

    def observe(self, actions, observations=None):
        for player, action in actions.items():
            if action is None:
                continue
            a_idx = int(action[0])
            a_param = np.asarray(action[1], dtype=np.float32).reshape(-1)

            self.total += 1
            if 0 <= a_idx < self.actions_num:
                self.action_counts[a_idx] += 1
            else:
                self.invalid_action_count += 1

            if a_idx in self.fallback_indices:
                self.fallback_count += 1

            if self.kick_action_idx is not None and a_idx == self.kick_action_idx:
                self.kick_count += 1
                start = a_idx * 2
                end = start + 2
                if end <= a_param.size:
                    values = a_param[start:end].astype(np.float64)
                    self.kick_param_count += 1
                    self.kick_param_sum += values
                    self.kick_param_sq_sum += values * values

            if observations is None or player not in observations:
                continue
            obs_item = observations[player]
            if not isinstance(obs_item, dict) or "action_mask" not in obs_item:
                continue

            mask = np.asarray(obs_item["action_mask"], dtype=bool).reshape(-1)
            if mask.size < self.actions_num:
                continue

            mask = mask[:self.actions_num]
            self.mask_obs_count += 1
            self.mask_valid_sum += float(mask.sum())
            if 0 <= a_idx < self.actions_num and mask[a_idx]:
                self.mask_selected_valid += 1
            else:
                self.mask_selected_invalid += 1
            if self.kick_action_idx is not None:
                self.kick_available_count += int(mask[self.kick_action_idx])
            self.fallback_available_count += sum(int(mask[i]) for i in self.fallback_indices if i < self.actions_num)

    def observe_arrays(self, action_nums, action_params, action_masks=None):
        action_nums = np.asarray(action_nums, dtype=np.int64).reshape(-1)
        action_params = np.asarray(action_params, dtype=np.float32).reshape(action_nums.size, -1)
        if action_masks is not None:
            action_masks = np.asarray(action_masks, dtype=bool).reshape(action_nums.size, -1)

        self.total += int(action_nums.size)
        valid_actions = (action_nums >= 0) & (action_nums < self.actions_num)
        self.invalid_action_count += int((~valid_actions).sum())
        if np.any(valid_actions):
            counts = np.bincount(action_nums[valid_actions], minlength=self.actions_num)
            self.action_counts += counts[: self.actions_num]

        if self.fallback_indices:
            fallback_mask = np.isin(action_nums, np.asarray(self.fallback_indices, dtype=np.int64))
            self.fallback_count += int(fallback_mask.sum())

        if self.kick_action_idx is not None:
            kick_rows = action_nums == self.kick_action_idx
            self.kick_count += int(kick_rows.sum())
            start = self.kick_action_idx * 2
            end = start + 2
            if end <= action_params.shape[1] and np.any(kick_rows):
                values = action_params[kick_rows, start:end].astype(np.float64)
                self.kick_param_count += int(values.shape[0])
                self.kick_param_sum += values.sum(axis=0)
                self.kick_param_sq_sum += (values * values).sum(axis=0)

        if action_masks is None or action_masks.shape[1] < self.actions_num:
            return

        masks = action_masks[:, : self.actions_num]
        self.mask_obs_count += int(masks.shape[0])
        self.mask_valid_sum += float(masks.sum())
        selected_valid = np.zeros((action_nums.size,), dtype=bool)
        rows = np.flatnonzero(valid_actions)
        if rows.size:
            selected_valid[rows] = masks[rows, action_nums[rows]]
        self.mask_selected_valid += int(selected_valid.sum())
        self.mask_selected_invalid += int(action_nums.size - selected_valid.sum())
        if self.kick_action_idx is not None:
            self.kick_available_count += int(masks[:, self.kick_action_idx].sum())
        self.fallback_available_count += sum(int(masks[:, i].sum()) for i in self.fallback_indices if i < self.actions_num)

    def record_replay_push(self, n=1):
        self.replay_push_count += int(n)

    def to_scalars(self, include_replay=True):
        if self.total <= 0:
            return {}

        denom = float(max(1, self.total))
        scalars = {
            "samples": float(self.total),
            "invalid_action_count": float(self.invalid_action_count),
            "invalid_action_rate": self.invalid_action_count / denom,
            "kick_rate": self.kick_count / denom,
            "fallback_rate": self.fallback_count / denom,
        }
        if include_replay:
            scalars["replay_push_count"] = float(self.replay_push_count)
            scalars["replay_push_rate"] = self.replay_push_count / denom

        for action_idx, count in enumerate(self.action_counts):
            scalars[f"action_{action_idx}_count"] = float(count)
            scalars[f"action_{action_idx}_rate"] = float(count) / denom

        if self.mask_obs_count > 0:
            mask_denom = float(self.mask_obs_count)
            scalars["available_actions_mean"] = self.mask_valid_sum / mask_denom
            scalars["selected_mask_valid_rate"] = self.mask_selected_valid / mask_denom
            scalars["selected_mask_invalid_rate"] = self.mask_selected_invalid / mask_denom
            if self.kick_action_idx is not None:
                scalars["kick_available_rate"] = self.kick_available_count / mask_denom
            scalars["fallback_available_mean"] = self.fallback_available_count / mask_denom

        if self.kick_param_count > 0:
            count = float(self.kick_param_count)
            means = self.kick_param_sum / count
            variances = np.maximum(self.kick_param_sq_sum / count - means * means, 0.0)
            stds = np.sqrt(variances)
            scalars["kick_param_samples"] = float(self.kick_param_count)
            scalars["kick_param0_mean"] = float(means[0])
            scalars["kick_param1_mean"] = float(means[1])
            scalars["kick_param0_std"] = float(stds[0])
            scalars["kick_param1_std"] = float(stds[1])

        return scalars

    def summary_string(self):
        if self.total <= 0:
            return "actions=no-samples"
        scalars = self.to_scalars()
        return (
            f"actions kick_rate={scalars.get('kick_rate', 0.0):.3f} "
            f"fallback_rate={scalars.get('fallback_rate', 0.0):.3f} "
            f"replay_push_rate={scalars.get('replay_push_rate', 0.0):.3f} "
            f"mask_valid_rate={scalars.get('selected_mask_valid_rate', 0.0):.3f}"
        )


def _log_action_diagnostics(writer, diagnostics, step, prefix, include_replay=True):
    if writer is None or diagnostics is None:
        return
    for key, value in diagnostics.to_scalars(include_replay=include_replay).items():
        writer.add_scalar(f"{prefix}/{key}", value, step)


def _recent_mean(values, window):
    if not values:
        return 0.0
    window = max(1, min(len(values), int(window)))
    return float(np.mean(values[-window:]))


def _next_interval_step(current_step, interval):
    interval = max(1, int(interval))
    return ((int(current_step) // interval) + 1) * interval


def _log_train_episode_metrics(writer, episode_reward, episode_idx, step, info=None, episode_length=None):
    if writer is None:
        return
    info = info or {}
    writer.add_scalar("progress/episode", episode_idx, step)
    writer.add_scalar("train/episode_reward", episode_reward, step)
    writer.add_scalar("train/average_episode_return", episode_reward, step)
    writer.add_scalar("episode/reward", episode_reward, step)
    if episode_length is not None:
        writer.add_scalar("train/episode_length", float(episode_length), step)
    writer.add_scalar("train/possession_loss_rate", _float_info(info, "possession_loss", 0.0), step)
    writer.add_scalar("train/max_epv", _float_info(info, "max_episode_epv", 0.0), step)
    writer.add_scalar("train/max_epv_improvement", _max_epv_improvement(info), step)

    writer.add_scalar("episode_axis/train/episode_reward", episode_reward, episode_idx)
    writer.add_scalar("episode_axis/train/average_episode_return", episode_reward, episode_idx)
    if episode_length is not None:
        writer.add_scalar("episode_axis/train/episode_length", float(episode_length), episode_idx)


def _log_eval_metrics(writer, mean_return, std_return, rates, step, episode_idx=None):
    if writer is None:
        return
    if episode_idx is not None:
        writer.add_scalar("progress/eval_episode", episode_idx, step)
    writer.add_scalar("eval/mean_return", mean_return, step)
    writer.add_scalar("eval/std_return", std_return, step)
    writer.add_scalar("test/average_episode_return", mean_return, step)
    if rates:
        writer.add_scalar("eval/win_rate", rates["win_rate"], step)
        writer.add_scalar("test_win_rate", rates["win_rate"], step)
        writer.add_scalar("test/win_rate", rates["win_rate"], step)
        writer.add_scalar("test/goal_rate", rates.get("goal_rate", rates["win_rate"]), step)
        writer.add_scalar("eval/loss_rate", rates["loss_rate"], step)
        writer.add_scalar("eval/timeout_rate", rates["timeout_rate"], step)
        writer.add_scalar("test/possession_loss_rate", rates.get("possession_loss_rate", 0.0), step)
        writer.add_scalar("test/episode_length", rates.get("episode_length", 0.0), step)
        writer.add_scalar("test/max_epv", rates.get("max_epv", 0.0), step)
        writer.add_scalar("test/max_epv_improvement", rates.get("max_epv_improvement", 0.0), step)
        writer.add_scalar("eval/mean_goal_diff", rates.get("mean_goal_diff", 0.0), step)
        writer.add_scalar("test/mean_goal_diff", rates.get("mean_goal_diff", 0.0), step)
        for key, value in rates.items():
            if isinstance(key, str) and key.startswith("eval_actions/"):
                writer.add_scalar(f"diagnostics/{key}", value, step)

    if episode_idx is not None:
        writer.add_scalar("episode_axis/eval/mean_return", mean_return, episode_idx)
        writer.add_scalar("episode_axis/eval/std_return", std_return, episode_idx)
        writer.add_scalar("episode_axis/test/average_episode_return", mean_return, episode_idx)
        if rates:
            writer.add_scalar("episode_axis/eval/win_rate", rates["win_rate"], episode_idx)
            writer.add_scalar("episode_axis/test_win_rate", rates["win_rate"], episode_idx)
            writer.add_scalar("episode_axis/test/win_rate", rates["win_rate"], episode_idx)
            writer.add_scalar("episode_axis/test/goal_rate", rates.get("goal_rate", rates["win_rate"]), episode_idx)
            writer.add_scalar("episode_axis/eval/loss_rate", rates["loss_rate"], episode_idx)
            writer.add_scalar("episode_axis/eval/timeout_rate", rates["timeout_rate"], episode_idx)


def evaluate(
    env: RobocupEnv_Hybrid_Helios,
    agent: ParaDQNAgent,
    episodes: int,
    eval_policy: str = "online",
):
    returns = []
    win_cnt, loss_cnt, timeout_cnt, possession_loss_cnt = 0, 0, 0, 0
    episode_lengths = []
    max_epvs = []
    max_epv_improvements = []
    score_diffs = []
    score_lefts = []
    score_rights = []
    action_diag = _ActionDiagnostics(agent.actions_num, agent.param_dim, agent.fallback_action_indices)
    use_target_policy = str(eval_policy or "online").lower() == "target"
    for _ in range(episodes):
        observations, _ = env.reset()
        # shared_obs = np.concatenate([observations[player]["observation"] for player in env.players])
        ep_rewards = {player: 0.0 for player in env.players}
        ep_reward = 0.0
        # dones = {player: False for player in env.players}
        done = False
        episode_length = 0

        while not done:
            obs_arr, masks_arr = _dict_observation_arrays(observations, env.players)
            action_nums, action_params = agent.select_actions(
                obs_arr,
                masks_arr,
                epsilon=0.0,
                use_target=use_target_policy,
            )
            actions = _actions_dict_from_arrays(env.players, action_nums, action_params)

            action_diag.observe_arrays(action_nums, action_params, masks_arr)
            observations_, rewards, terminations, truncations, infos = env.step(actions)
            episode_length += 1
            # shared_obs_ = np.concatenate([observations_[player]["observation"] for player in env.players])
            ep_rewards = {player: ep_rewards[player] + rewards[player] for player in env.players}
            ep_reward = sum(ep_rewards.values())
            # dones = {player: terminations[player] or truncations[player] for player in env.players}
            done = all(terminations.values()) or all(truncations.values())

            observations = observations_
            # shared_obs = shared_obs_

            if done:
                final_info = _agent_info(infos)
                if final_info.get("win", 0):
                    win_cnt += 1
                elif final_info.get("loss", final_info.get("lose", 0)):
                    loss_cnt += 1
                elif final_info.get("timeout", 0):
                    timeout_cnt += 1
                possession_loss_cnt += int(final_info.get("possession_loss", 0))
                episode_lengths.append(episode_length)
                max_epvs.append(_float_info(final_info, "max_episode_epv", 0.0))
                max_epv_improvements.append(_max_epv_improvement(final_info))
                score_diffs.append(_float_info(final_info, "score_diff", 0.0))
                score_lefts.append(_float_info(final_info, "score_left", 0.0))
                score_rights.append(_float_info(final_info, "score_right", 0.0))
                break

        returns.append(ep_reward)
        # returns.append(total)

    total = max(1, len(returns))
    rates = {
        "win_rate": win_cnt / total if total > 0 else 0,
        "goal_rate": win_cnt / total if total > 0 else 0,
        "loss_rate": loss_cnt / total if total > 0 else 0,
        "timeout_rate": timeout_cnt / total if total > 0 else 0,
        "possession_loss_rate": possession_loss_cnt / total if total > 0 else 0,
        "episode_length": float(np.mean(episode_lengths)) if episode_lengths else 0.0,
        "max_epv": float(np.mean(max_epvs)) if max_epvs else 0.0,
        "max_epv_improvement": float(np.mean(max_epv_improvements)) if max_epv_improvements else 0.0,
        "mean_goal_diff": float(np.mean(score_diffs)) if score_diffs else 0.0,
        "mean_score_diff": float(np.mean(score_diffs)) if score_diffs else 0.0,
        "mean_score_left": float(np.mean(score_lefts)) if score_lefts else 0.0,
        "mean_score_right": float(np.mean(score_rights)) if score_rights else 0.0,
        "eval_policy_target": 1.0 if use_target_policy else 0.0,
    }

    rates.update({
        f"eval_actions/{key}": value
        for key, value in action_diag.to_scalars(include_replay=False).items()
    })

    return float(np.mean(returns)), float(np.std(returns)), rates


def train(
    env: RobocupEnv_Hybrid_Helios,
    agent: ParaDQNAgent,
    buffer: ReplayBuffer,
    writer: SummaryWriter,
    episodes: int,
    batch_size: int,
    train_freq: int,
    eval_episodes: int,
    eval_interval: int,
    save_interval: int,
    epsilon_start: float,
    epsilon_end: float,
    epsilon_decay_steps: int,
    checkpoint_dir: str,
    resume_from: Optional[str] = None,
    eval_interval_steps: Optional[int] = None,
    eval_env_factory: Optional[Callable[[], RobocupEnv_Hybrid_Helios]] = None,
    max_env_steps: Optional[int] = None,
    n_step_return: int = 1,
    eval_policy: str = "online",
    updates_per_step: int = 1,
    actor_freeze_after_steps: int = 0,
    save_interval_steps: Optional[int] = None,
    test50_during_training: bool = False,
    test50_output_dir: Optional[str] = None,
    test50_episodes: int = 50,
):
    if test50_during_training:
        eval_episodes = max(int(eval_episodes), int(test50_episodes or 50))

    total_env_steps = 0
    rewards_log = []
    start_episode = 1
    train_action_diag = _ActionDiagnostics(agent.actions_num, agent.param_dim, agent.fallback_action_indices)
    n_step_builder = _NStepTransitionBuilder(n_step_return, agent.gamma)

    def _push_emitted(transitions):
        if not transitions:
            return
        for emitted_transition in transitions:
            buffer.push(emitted_transition)
        train_action_diag.record_replay_push(len(transitions))

    def _append_transition(key, transition):
        _push_emitted(n_step_builder.append(key, transition))

    def _flush_transitions(key):
        _push_emitted(n_step_builder.flush(key))

    # resume from checkpoint if provided
    if resume_from is not None and os.path.exists(resume_from):
        resume_path = Path(resume_from)
        if resume_path.is_dir():
            resume_path = resume_path / "latest.pth"
        if resume_path.exists():
            print(f"Loading checkpoint from {str(resume_path)}")
            ck = load_checkpoint(str(resume_path), agent)
            if ck is not None:
                total_env_steps = ck.get("total_env_steps", 0)
                start_episode = ck.get("episode", 1) + 1
                print(f"Resuming from episode {start_episode}, total_env_steps={total_env_steps}")

    eval_by_steps = eval_interval_steps is not None and eval_interval_steps > 0
    next_eval_step = _next_interval_step(total_env_steps, eval_interval_steps) if eval_by_steps else None
    save_by_steps = save_interval_steps is not None and save_interval_steps > 0
    next_save_step = _next_interval_step(total_env_steps, save_interval_steps) if save_by_steps else None
    stop_by_steps = max_env_steps is not None and max_env_steps > 0

    def _save_step_checkpoint(step, episode):
        os.makedirs(checkpoint_dir, exist_ok=True)
        path = os.path.join(checkpoint_dir, f"ck_step{int(step)}.pth")
        save_checkpoint(path, agent, episode=episode, total_env_steps=int(step))
        shutil.copy(path, os.path.join(checkpoint_dir, "latest.pth"))
        print(f"Step {int(step)}  Saved checkpoint to {path}.")

    # main training loop
    for ep in range(start_episode, episodes + 1):
        if stop_by_steps and total_env_steps >= max_env_steps:
            break
        observations, _ = env.reset()
        shared_obs = np.concatenate([observations[player]["observation"] for player in env.players])
        ep_rewards = {player: 0.0 for player in env.players}
        ep_reward = 0.0
        dones = {player: False for player in env.players}
        done = False
        episode_length = 0

        while not done:
            eps = max(epsilon_end, epsilon_start - total_env_steps / epsilon_decay_steps)
            obs_arr, masks_arr = _dict_observation_arrays(observations, env.players)
            action_nums, action_params = agent.select_actions(obs_arr, masks_arr, epsilon=eps)
            actions = _actions_dict_from_arrays(env.players, action_nums, action_params)

            train_action_diag.observe_arrays(action_nums, action_params, masks_arr)
            observations_, rewards, terminations, truncations, infos = env.step(actions)
            episode_length += 1
            shared_obs_ = np.concatenate([observations_[player]["observation"] for player in env.players])
            ep_rewards = {player: ep_rewards[player] + rewards[player] for player in env.players}
            ep_reward = sum(ep_rewards.values())
            dones = {player: terminations[player] or truncations[player] for player in env.players}
            done = all(terminations.values()) or all(truncations.values())

            for player in env.players:
                transition_key = (0, player)
                if agent.is_fallback_action(actions[player][0]):
                    _flush_transitions(transition_key)
                    continue
                transition = Transition(
                    obs=observations[player]["observation"],
                    shared_obs=shared_obs,
                    action_mask=observations[player]["action_mask"],
                    act_num=actions[player][0],
                    act_param=actions[player][1],
                    reward=rewards[player],
                    done=dones[player],
                    next_obs=observations_[player]["observation"],
                    next_shared_obs=shared_obs_,
                    next_action_mask=observations_[player]["action_mask"],
                )
                _append_transition(transition_key, transition)

            observations = observations_
            shared_obs = shared_obs_
            total_env_steps += 1

            if save_by_steps and total_env_steps >= next_save_step:
                while next_save_step <= total_env_steps:
                    _save_step_checkpoint(next_save_step, ep)
                    next_save_step += save_interval_steps

            if buffer.can_sample(batch_size) and total_env_steps % train_freq == 0:
                update_count = max(1, int(updates_per_step or 1))
                for _update_idx in range(update_count):
                    batch = buffer.sample(batch_size)
                    update_actor = not (actor_freeze_after_steps and total_env_steps >= actor_freeze_after_steps)
                    info = agent.train_step(batch, update_actor=update_actor)
                    if writer is not None and isinstance(info, dict):
                        if "q_loss" in info:
                            writer.add_scalar("loss/q_loss", info["q_loss"], total_env_steps)
                        if "actor_loss" in info:
                            writer.add_scalar("loss/actor_loss", info["actor_loss"], total_env_steps)
                        if "actor_update_enabled" in info:
                            writer.add_scalar("train/actor_update_enabled", info["actor_update_enabled"], total_env_steps)
                        if "actor_behavior_l2" in info:
                            writer.add_scalar("loss/actor_behavior_l2", info["actor_behavior_l2"], total_env_steps)
                        writer.add_scalar("train/epsilon", eps, total_env_steps)
                        writer.add_scalar("train/replay_size", len(buffer), total_env_steps)

            if stop_by_steps and total_env_steps >= max_env_steps:
                final_info = _agent_info(infos)
                break

            if done:
                for player in env.players:
                    _flush_transitions((0, player))
                final_info = _agent_info(infos)
                break

        rewards_log.append(ep_reward)

        should_eval = total_env_steps >= next_eval_step if eval_by_steps else ep % eval_interval == 0
        if should_eval:
            mean_r, std_r, rates = evaluate(env, agent, episodes=eval_episodes, eval_policy=eval_policy)
            diag_summary = train_action_diag.summary_string()
            recent_reward = _recent_mean(rewards_log, eval_interval)
            print(f"Time {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} Ep {ep}/{episodes}  total_env_steps={total_env_steps}  recent_reward={recent_reward:.3f}  eval_mean={mean_r:.3f} +/- {std_r:.3f}  {diag_summary}")
            _log_eval_metrics(writer, mean_r, std_r, rates, total_env_steps, ep)
            if test50_during_training:
                _write_training_test50_result(
                    test50_output_dir,
                    checkpoint_dir,
                    total_env_steps,
                    ep,
                    eval_episodes,
                    eval_policy,
                    mean_r,
                    std_r,
                    rates,
                )
            _log_action_diagnostics(writer, train_action_diag, total_env_steps, "diagnostics/train_actions")
            train_action_diag.reset()
            if eval_by_steps:
                while next_eval_step <= total_env_steps:
                    next_eval_step += eval_interval_steps

        _log_train_episode_metrics(writer, ep_reward, ep, total_env_steps, final_info, episode_length)

        if checkpoint_dir and ep % save_interval == 0:
            os.makedirs(checkpoint_dir, exist_ok=True)
            path = os.path.join(checkpoint_dir, f"ck_ep{ep}.pth")
            save_checkpoint(path, agent, episode=ep, total_env_steps=total_env_steps)
            latest = os.path.join(checkpoint_dir, "latest.pth")
            shutil.copy(path, latest)
            print(f"Ep {ep}/{episodes}  Saved checkpoint to {path}.")

    if checkpoint_dir:
        os.makedirs(checkpoint_dir, exist_ok=True)
        path = os.path.join(checkpoint_dir, "final.pth")
        final_episode = min(ep if "ep" in locals() else start_episode - 1, episodes)
        save_checkpoint(path, agent, episode=final_episode, total_env_steps=total_env_steps)
        shutil.copy(path, os.path.join(checkpoint_dir, "latest.pth"))
        print(f"Saved final checkpoint to {path} at total_env_steps={total_env_steps}.")

    return rewards_log


def train_vec(
    env,  # Can be RobocupEnv_Hybrid_Helios or SubprocVecEnv
    eval_env,  # RobocupEnv_Hybrid_Helios
    agent: ParaDQNAgent,
    buffer: ReplayBuffer,
    writer: SummaryWriter,
    episodes: int,
    batch_size: int,
    train_freq: int,
    eval_episodes: int,
    eval_interval: int,
    save_interval: int,
    epsilon_start: float,
    epsilon_end: float,
    epsilon_decay_steps: int,
    checkpoint_dir: str,
    resume_from: Optional[str] = None,
    eval_interval_steps: Optional[int] = None,
    eval_env_factory: Optional[Callable[[], RobocupEnv_Hybrid_Helios]] = None,
    max_env_steps: Optional[int] = None,
    n_step_return: int = 1,
    eval_policy: str = "online",
    updates_per_step: int = 1,
    actor_freeze_after_steps: int = 0,
    save_interval_steps: Optional[int] = None,
    test50_during_training: bool = False,
    test50_output_dir: Optional[str] = None,
    test50_episodes: int = 50,
):
    """Train with vectorized environments for parallel data collection.

    Args:
        env: Vectorized environment (SubprocVecEnv) or single environment
        agent: ParaDQN agent
        buffer: Replay buffer
        writer: TensorBoard writer
        episodes: Total number of episodes to train
        batch_size: Batch size for training
        train_freq: Training frequency (in steps)
        eval_episodes: Number of evaluation episodes
        eval_interval: Legacy evaluation interval (in episodes); used only when eval_interval_steps is unset
        eval_interval_steps: Evaluation interval in environment steps
        max_env_steps: Optional environment-step budget. When set, training
                  stops once total_env_steps reaches this value, independent
                  of the episode count.
        save_interval: Model saving interval (in episodes)
        save_interval_steps: Optional model saving interval in environment steps
        epsilon_start: Starting epsilon for epsilon-greedy
        epsilon_end: Final epsilon for epsilon-greedy
        epsilon_decay_steps: Number of steps to decay epsilon
        checkpoint_dir: Directory to save checkpoints
        resume_from: Path to resume checkpoint
        eval_env: Optional separate environment for evaluation.
        eval_env_factory: Optional factory used to create a fresh evaluation
                  environment for each eval call. This avoids leaving a
                  synchronous RoboCup eval server idle long enough for its
                  clients to hit server_wait_seconds and exit.
    """
    from environments.vec_env import SubprocVecEnv

    if test50_during_training:
        eval_episodes = max(int(eval_episodes), int(test50_episodes or 50))

    # Check if env is vectorized
    is_vec_env = isinstance(env, SubprocVecEnv)
    num_envs = len(env) if is_vec_env else 1

    # Create evaluation environment if needed
    # eval_env_created = False
    # if is_vec_env and eval_env is None:
    #     print("Creating dedicated evaluation environment...")
    #     eval_env = RobocupEnv_Hybrid_Helios()
    #     eval_env_created = True
    # elif not is_vec_env:
    #     # For single env, use the same env for both training and evaluation
    #     eval_env = env

    total_env_steps = 0
    episode_count = 0
    start_episode = 1
    train_action_diag = _ActionDiagnostics(agent.actions_num, agent.param_dim, agent.fallback_action_indices)
    n_step_builder = _NStepTransitionBuilder(n_step_return, agent.gamma)

    def _push_emitted(transitions):
        if not transitions:
            return
        for emitted_transition in transitions:
            buffer.push(emitted_transition)
        train_action_diag.record_replay_push(len(transitions))

    def _append_transition(key, transition):
        _push_emitted(n_step_builder.append(key, transition))

    def _flush_transitions(key):
        _push_emitted(n_step_builder.flush(key))

    # resume from checkpoint if provided
    if resume_from is not None and os.path.exists(resume_from):
        resume_path = Path(resume_from)
        if resume_path.is_dir():
            resume_path = resume_path / "latest.pth"
        if resume_path.exists():
            print(f"Loading checkpoint from {str(resume_path)}")
            ck = load_checkpoint(str(resume_path), agent)
            if ck is not None:
                total_env_steps = ck.get("total_env_steps", 0)
                start_episode = ck.get("episode", 1) + 1
                episode_count = start_episode - 1
                print(f"Resuming from episode {start_episode}, total_env_steps={total_env_steps}")

    # Initialize vectorized environments
    if is_vec_env:
        observations_arr, action_masks_arr, _ = env.reset_compact()
        # Track per-environment state
        env_ep_rewards = np.zeros((num_envs, len(env.players)), dtype=np.float64)
        env_ep_lengths = [0 for _ in range(num_envs)]
    else:
        observations, _ = env.reset()
        ep_reward = 0.0
        ep_length = 0
        final_info = {}
        done = False

    rewards_log = []
    recent_rewards = []
    eval_by_steps = eval_interval_steps is not None and eval_interval_steps > 0
    next_eval_step = _next_interval_step(total_env_steps, eval_interval_steps) if eval_by_steps else None
    save_by_steps = save_interval_steps is not None and save_interval_steps > 0
    next_save_step = _next_interval_step(total_env_steps, save_interval_steps) if save_by_steps else None
    stop_by_steps = max_env_steps is not None and max_env_steps > 0

    def _save_step_checkpoint(step, episode):
        os.makedirs(checkpoint_dir, exist_ok=True)
        path = os.path.join(checkpoint_dir, f"ck_step{int(step)}.pth")
        save_checkpoint(path, agent, episode=episode, total_env_steps=int(step))
        shutil.copy(path, os.path.join(checkpoint_dir, "latest.pth"))
        print(f"Step {int(step)}  Saved checkpoint to {path}.")

    def _run_eval(step):
        target = eval_env
        close_target = False
        if target is None:
            if eval_env_factory is not None:
                target = eval_env_factory()
                close_target = True
            else:
                target = env

        try:
            mean_r, std_r, rates = evaluate(target, agent, episodes=eval_episodes, eval_policy=eval_policy)
            diag_summary = train_action_diag.summary_string()
            recent_reward = _recent_mean(recent_rewards, eval_interval)
            print(f"Time {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} Ep {episode_count}/{episodes}  total_env_steps={step}  recent_reward={recent_reward:.3f}  eval_mean={mean_r:.3f} +/- {std_r:.3f}  {diag_summary}")
            _log_eval_metrics(writer, mean_r, std_r, rates, step, episode_count)
            if test50_during_training:
                _write_training_test50_result(
                    test50_output_dir,
                    checkpoint_dir,
                    step,
                    episode_count,
                    eval_episodes,
                    eval_policy,
                    mean_r,
                    std_r,
                    rates,
                )
            _log_action_diagnostics(writer, train_action_diag, step, "diagnostics/train_actions")
            train_action_diag.reset()
        finally:
            if close_target and target is not None:
                try:
                    target.close()
                except Exception as exc:
                    print(f"[WARN] Failed to close eval env: {exc}")

    # main training loop
    while episode_count < episodes and (not stop_by_steps or total_env_steps < max_env_steps):
        eps = max(epsilon_end, epsilon_start - (epsilon_start - epsilon_end) * total_env_steps / epsilon_decay_steps)

        if is_vec_env:
            flat_obs = observations_arr.reshape(num_envs * len(env.players), -1)
            flat_masks = action_masks_arr.reshape(num_envs * len(env.players), -1)
            flat_action_nums, flat_action_params = agent.select_actions(flat_obs, flat_masks, epsilon=eps)
            action_nums = flat_action_nums.reshape(num_envs, len(env.players))
            action_params = flat_action_params.reshape(num_envs, len(env.players), -1)
            train_action_diag.observe_arrays(action_nums, action_params, action_masks_arr)

            observations_arr_, action_masks_arr_, rewards_arr, dones_arr, infos_list = env.step_compact(action_nums, action_params)

            step_after_env = total_env_steps + num_envs

            for env_idx in range(num_envs):
                env_ep_lengths[env_idx] += 1
                env_ep_rewards[env_idx] += rewards_arr[env_idx]
                transition_key = env_idx
                has_fallback = bool(agent.fallback_action_indices) and bool(
                    np.isin(action_nums[env_idx], np.asarray(agent.fallback_action_indices, dtype=np.int64)).any()
                )

                if has_fallback:
                    _flush_transitions(transition_key)
                else:
                    done_value = bool(dones_arr[env_idx])
                    transition = Transition(
                        obs=observations_arr[env_idx],
                        shared_obs=observations_arr[env_idx].reshape(-1),
                        action_mask=action_masks_arr[env_idx],
                        act_num=action_nums[env_idx],
                        act_param=action_params[env_idx],
                        reward=rewards_arr[env_idx],
                        done=np.full((len(env.players),), float(done_value), dtype=np.float32),
                        next_obs=observations_arr_[env_idx],
                        next_shared_obs=observations_arr_[env_idx].reshape(-1),
                        next_action_mask=action_masks_arr_[env_idx],
                    )
                    _append_transition(transition_key, transition)

                if bool(dones_arr[env_idx]):
                    _flush_transitions(transition_key)
                    ep_reward = float(env_ep_rewards[env_idx].sum())
                    final_info = _agent_info(infos_list[env_idx])
                    ep_length = env_ep_lengths[env_idx]
                    rewards_log.append(ep_reward)
                    recent_rewards.append(ep_reward)
                    episode_count += 1

                    reset_obs, reset_masks, _ = env.reset_at_compact(env_idx)
                    observations_arr_[env_idx] = reset_obs
                    action_masks_arr_[env_idx] = reset_masks
                    env_ep_rewards[env_idx, :] = 0.0
                    env_ep_lengths[env_idx] = 0

                    if not eval_by_steps and episode_count % eval_interval == 0:
                        _run_eval(step_after_env)

                    _log_train_episode_metrics(writer, ep_reward, episode_count, step_after_env, final_info, ep_length)

                    if checkpoint_dir and episode_count % save_interval == 0:
                        os.makedirs(checkpoint_dir, exist_ok=True)
                        path = os.path.join(checkpoint_dir, f"ck_ep{episode_count}.pth")
                        save_checkpoint(path, agent, episode=episode_count, total_env_steps=step_after_env)
                        latest = os.path.join(checkpoint_dir, "latest.pth")
                        shutil.copy(path, latest)
                        print(f"Ep {episode_count}/{episodes}  Saved checkpoint to {path}.")

                    if episode_count >= episodes:
                        break

            observations_arr = observations_arr_
            action_masks_arr = action_masks_arr_
            total_env_steps = step_after_env
            if eval_by_steps and total_env_steps >= next_eval_step:
                _run_eval(total_env_steps)
                while next_eval_step <= total_env_steps:
                    next_eval_step += eval_interval_steps

            if save_by_steps and total_env_steps >= next_save_step:
                while next_save_step <= total_env_steps:
                    _save_step_checkpoint(next_save_step, episode_count)
                    next_save_step += save_interval_steps

        else:
            # Single environment training (original logic)
            obs_arr, masks_arr = _dict_observation_arrays(observations, env.players)
            action_nums, action_params = agent.select_actions(obs_arr, masks_arr, epsilon=eps)
            actions = _actions_dict_from_arrays(env.players, action_nums, action_params)

            train_action_diag.observe_arrays(action_nums, action_params, masks_arr)
            observations_, rewards, terminations, truncations, infos = env.step(actions)
            ep_length += 1
            shared_obs = np.concatenate([observations[player]["observation"] for player in env.players])
            shared_obs_ = np.concatenate([observations_[player]["observation"] for player in env.players])

            ep_reward += sum(rewards.values())
            dones = {player: terminations[player] or truncations[player] for player in env.players}
            done = all(terminations.values()) or all(truncations.values())

            for player in env.players:
                transition_key = (0, player)
                if agent.is_fallback_action(actions[player][0]):
                    _flush_transitions(transition_key)
                    continue
                transition = Transition(
                    obs=observations[player]["observation"],
                    shared_obs=shared_obs,
                    action_mask=observations[player]["action_mask"],
                    act_num=actions[player][0],
                    act_param=actions[player][1],
                    reward=rewards[player],
                    done=dones[player],
                    next_obs=observations_[player]["observation"],
                    next_shared_obs=shared_obs_,
                    next_action_mask=observations_[player]["action_mask"],
                )
                _append_transition(transition_key, transition)

            observations = observations_
            total_env_steps += 1

            if done:
                for player in env.players:
                    _flush_transitions((0, player))
                final_info = _agent_info(infos)
                rewards_log.append(ep_reward)
                recent_rewards.append(ep_reward)
                episode_count += 1

                if not eval_by_steps and episode_count % eval_interval == 0:
                    _run_eval(total_env_steps)

                _log_train_episode_metrics(writer, ep_reward, episode_count, total_env_steps, final_info, ep_length)

                if checkpoint_dir and episode_count % save_interval == 0:
                    os.makedirs(checkpoint_dir, exist_ok=True)
                    path = os.path.join(checkpoint_dir, f"ck_ep{episode_count}.pth")
                    save_checkpoint(path, agent, episode=episode_count, total_env_steps=total_env_steps)
                    latest = os.path.join(checkpoint_dir, "latest.pth")
                    shutil.copy(path, latest)
                    print(f"Ep {episode_count}/{episodes}  Saved checkpoint to {path}.")

                if episode_count >= episodes:
                    break

                observations, _ = env.reset()
                ep_reward = 0.0
                ep_length = 0
                final_info = {}
                done = False

        # Training step
        if buffer.can_sample(batch_size) and total_env_steps % train_freq == 0:
            update_count = max(1, int(updates_per_step or 1))
            for _update_idx in range(update_count):
                batch = buffer.sample(batch_size)
                update_actor = not (actor_freeze_after_steps and total_env_steps >= actor_freeze_after_steps)
                info = agent.train_step(batch, update_actor=update_actor)
                if writer is not None and isinstance(info, dict):
                    if "q_loss" in info:
                        writer.add_scalar("loss/q_loss", info["q_loss"], total_env_steps)
                    if "actor_loss" in info:
                        writer.add_scalar("loss/actor_loss", info["actor_loss"], total_env_steps)
                    if "mean_n_step" in info:
                        writer.add_scalar("train/mean_n_step", info["mean_n_step"], total_env_steps)
                    if "actor_updated" in info:
                        writer.add_scalar("train/actor_updated", info["actor_updated"], total_env_steps)
                    if "actor_update_enabled" in info:
                        writer.add_scalar("train/actor_update_enabled", info["actor_update_enabled"], total_env_steps)
                    if "policy_delay" in info:
                        writer.add_scalar("train/policy_delay", info["policy_delay"], total_env_steps)
                    if "actor_behavior_l2" in info:
                        writer.add_scalar("loss/actor_behavior_l2", info["actor_behavior_l2"], total_env_steps)
                    if hasattr(batch, "priorities"):
                        writer.add_scalar("train/replay_sample_priority_mean", float(np.mean(batch.priorities)), total_env_steps)
                    writer.add_scalar("train/epsilon", eps, total_env_steps)
                    writer.add_scalar("train/replay_size", len(buffer), total_env_steps)

        if eval_by_steps and total_env_steps >= next_eval_step:
            _run_eval(total_env_steps)
            while next_eval_step <= total_env_steps:
                next_eval_step += eval_interval_steps

        if save_by_steps and total_env_steps >= next_save_step:
            while next_save_step <= total_env_steps:
                _save_step_checkpoint(next_save_step, episode_count)
                next_save_step += save_interval_steps

    if checkpoint_dir:
        os.makedirs(checkpoint_dir, exist_ok=True)
        path = os.path.join(checkpoint_dir, "final.pth")
        save_checkpoint(path, agent, episode=episode_count, total_env_steps=total_env_steps)
        shutil.copy(path, os.path.join(checkpoint_dir, "latest.pth"))
        print(f"Saved final checkpoint to {path} at total_env_steps={total_env_steps}.")

    return rewards_log
