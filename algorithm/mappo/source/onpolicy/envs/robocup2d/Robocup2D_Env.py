from __future__ import annotations

import random
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np
from gym import spaces


REPO_ROOT = Path(__file__).resolve().parents[6]
DEFAULT_PYMARL_SRC = REPO_ROOT / "algorithm/qmix/source"


def _add_pymarl_src(pymarl_src: Optional[str]) -> Path:
    src = Path(pymarl_src).expanduser().resolve() if pymarl_src else DEFAULT_PYMARL_SRC
    src_text = str(src)
    if src_text not in sys.path:
        sys.path.insert(0, src_text)
    return src


def _truthy(value, default=False):
    if value is None:
        return bool(default)
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


class RoboCup2DEnv(object):
    """MAPPO-compatible wrapper around the PyMARL RoboCup2D env.

    The official MAPPO football runner expects an environment with:
      - observation_space/share_observation_space/action_space as per-agent lists
      - reset() -> obs with shape [n_agents, obs_dim]
      - step(actions) -> obs, rewards, dones, info

    This adapter keeps the first version intentionally close to the official
    FootballEnv wrapper: shared-policy MAPPO receives per-agent local
    observations; the centralized critic uses the same per-agent observation
    shape because the stock football runner stores obs into share_obs.
    """

    def __init__(self, args, rank=0, is_eval=False):
        self.args = args
        self.rank = int(rank)
        self.is_eval = bool(is_eval)
        self.reset_stagger_seconds = max(
            0.0,
            float(getattr(args, "robocup_reset_stagger_seconds", 0.0) or 0.0),
        )
        eval_stagger = getattr(args, "robocup_eval_reset_stagger_seconds", None)
        if eval_stagger is None:
            eval_stagger = self.reset_stagger_seconds
        self.eval_reset_stagger_seconds = max(0.0, float(eval_stagger or 0.0))
        self.launch_stagger_seconds = max(
            0.0,
            float(
                getattr(
                    args,
                    "robocup_launch_stagger_seconds",
                    self.reset_stagger_seconds,
                )
                or 0.0
            ),
        )
        eval_launch_stagger = getattr(args, "robocup_eval_launch_stagger_seconds", None)
        if eval_launch_stagger is None:
            eval_launch_stagger = self.launch_stagger_seconds
        self.eval_launch_stagger_seconds = max(0.0, float(eval_launch_stagger or 0.0))
        self.pymarl_src = _add_pymarl_src(getattr(args, "pymarl_src", None))
        self.use_obs_agent_id = _truthy(getattr(args, "robocup_obs_agent_id", True), default=True)

        from envs.robocup2d import ParallelR2DRL
        from envs.robocup2d.start_sampler import ScenarioStartSampler

        cfg = getattr(args, "robocup_env_config", None)
        if not cfg:
            raise ValueError("--robocup_env_config must point to a RoboCup2D yaml")
        self.cfg = cfg
        self._parallel_env_cls = ParallelR2DRL
        self.wrapper_reset_retries = max(
            0,
            int(getattr(args, "robocup_wrapper_reset_retries", 0) or 0),
        )
        self.wrapper_recreate_pause_seconds = max(
            0.0,
            float(getattr(args, "robocup_wrapper_recreate_pause_seconds", 0.0) or 0.0),
        )

        env_overrides = dict(getattr(args, "robocup_env_overrides", {}) or {})
        init_n = getattr(args, "robocup_init_n", None)
        if init_n is not None:
            env_overrides["init_n"] = int(init_n)
        use_epv_reward = getattr(args, "robocup_use_epv_reward", None)
        if use_epv_reward is not None:
            env_overrides["useMaxEpv"] = str(use_epv_reward).strip().lower() in (
                "1", "true", "yes", "on"
            )
        use_action_mask = getattr(args, "robocup_use_action_mask", None)
        if use_action_mask is not None:
            env_overrides["use_action_mask"] = _truthy(use_action_mask, default=True)
        epv_grid_file = getattr(args, "robocup_epv_grid_file", None)
        if epv_grid_file:
            env_overrides["epv_grid_file"] = str(epv_grid_file)
        epv_progress_scale = getattr(args, "robocup_epv_progress_scale", None)
        if epv_progress_scale is not None:
            env_overrides["epv_progress_scale"] = float(epv_progress_scale)
        if bool(getattr(args, "game_logging", False)):
            env_overrides["game_logging"] = True
        auto_port_start = getattr(args, "robocup_auto_port_start", None)
        if auto_port_start is not None:
            env_overrides["auto_port_start"] = int(auto_port_start)
        auto_port_end = getattr(args, "robocup_auto_port_end", None)
        if auto_port_end is not None:
            env_overrides["auto_port_end"] = int(auto_port_end)
        for arg_name, cfg_key, caster in (
            ("robocup_wait_ready_timeout", "wait_ready_timeout", float),
            ("robocup_playon_timeout", "playon_timeout", float),
            ("robocup_trainer_ready_timeout_ms", "trainer_ready_timeout_ms", float),
            ("robocup_ports_wait_timeout", "ports_wait_timeout", float),
            ("robocup_reset_retries", "reset_retries", int),
            (
                "robocup_reset_perception_warmup_cycles",
                "reset_perception_warmup_cycles",
                int,
            ),
        ):
            value = getattr(args, arg_name, None)
            if value is not None:
                env_overrides[cfg_key] = caster(value)
        self.env_overrides = env_overrides
        self._stagger_before_launch()
        self.env = self._make_env()

        env_cfg = self.env.config
        self.start_sampler = None
        if bool(getattr(env_cfg, "use_custom_start", False)) and getattr(env_cfg, "trajectory_path", None):
            self.start_sampler = ScenarioStartSampler(
                init_n=int(env_cfg.init_n),
                start_window_size=int(env_cfg.start_window_size),
                progress_bucket_count=int(env_cfg.progress_bucket_count),
                current_target_window_size=int(env_cfg.current_target_window_size),
                num_selected_trajectories=env_cfg.num_selected_trajectories,
                random_sample=env_cfg.random_sample,
                trajectory_path=env_cfg.trajectory_path,
                scenario_difficulty=getattr(env_cfg, "scenario_difficulty", None),
                scenario_difficulty_buckets=getattr(env_cfg, "scenario_difficulty_buckets", None),
                start_catalog_enabled=getattr(env_cfg, "start_catalog_enabled", False),
                start_id=getattr(env_cfg, "start_id", None),
                start_index=getattr(env_cfg, "start_index", None),
            )
            env_n = int(env_cfg.n)
            traj_n = int(self.start_sampler.n_players)
            if env_n > traj_n:
                raise ValueError(
                    f"env n={env_n} but trajectory has only {traj_n} players per side. "
                    "Use n <= trajectory player count, or use a matching trajectory_path in the yaml."
                )

        info = self.env.get_env_info()
        self.num_agents = int(info["n_agents"])
        requested_agents = getattr(args, "num_agents", None)
        if requested_agents is not None and int(requested_agents) != self.num_agents:
            raise ValueError(
                f"--num_agents={requested_agents} does not match env n_agents={self.num_agents} "
                f"from {cfg}"
            )

        self.n_actions = int(info["n_actions"])
        self.base_obs_shape = int(info["obs_shape"])
        self.agent_id_dim = self.num_agents if self.use_obs_agent_id else 0
        self.obs_shape = self.base_obs_shape + self.agent_id_dim
        self.state_shape = int(info["state_shape"])
        self.episode_limit = int(info["episode_limit"])
        self.max_steps = self.episode_limit
        self.share_reward = bool(getattr(args, "share_reward", True))
        self._episode_steps = 0
        self._last_info: Dict[str, Any] = {}
        self.agent_id_eye = (
            np.eye(self.num_agents, dtype=np.float32)
            if self.use_obs_agent_id
            else None
        )

        self.action_space = [spaces.Discrete(self.n_actions) for _ in range(self.num_agents)]
        obs_box = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(self.obs_shape,),
            dtype=np.float32,
        )
        self.observation_space = [obs_box for _ in range(self.num_agents)]
        # The stock football runner writes obs into share_obs. Keep shapes equal.
        self.share_observation_space = [obs_box for _ in range(self.num_agents)]

    def reset(self):
        attempts = self.wrapper_reset_retries + 1
        for attempt in range(attempts):
            self._episode_steps = 0
            try:
                if self.start_sampler is not None:
                    start, n_control = self.start_sampler.sample_start_and_n()
                    self.env.set_start_and_n(start, n_control)
                self._stagger_before_reset()
                self.env.reset()
                return self._obs()
            except Exception as exc:
                if attempt + 1 >= attempts:
                    raise
                self._recreate_env_after_reset_failure(attempt + 1, attempts, exc)

        raise RuntimeError("RoboCup2D reset failed without raising an exception")

    def step(self, action):
        action = np.asarray(action, dtype=np.int64).reshape(self.num_agents)
        reward, done, info = self.env.step(action)
        self._episode_steps += 1
        self._last_info = dict(info)

        obs = self._obs()
        reward_value = float(reward)
        rewards = np.full((self.num_agents, 1), reward_value, dtype=np.float32)
        if not self.share_reward:
            # The underlying PyMARL env currently emits one shared team reward.
            rewards = np.full((self.num_agents, 1), reward_value, dtype=np.float32)

        dones = np.array([bool(done)] * self.num_agents, dtype=bool)
        wrapped_info = self._wrap_info(info, reward_value)
        return obs, rewards, dones, wrapped_info

    def get_avail_actions(self):
        avail = np.asarray(self.env.get_avail_actions(), dtype=np.float32)
        if avail.shape != (self.num_agents, self.n_actions):
            avail = avail.reshape(self.num_agents, self.n_actions)
        return avail

    def get_active_masks(self):
        # The PyMARL RoboCup env exposes only compact controlled agents.
        return np.ones((self.num_agents, 1), dtype=np.float32)

    def seed(self, seed=None):
        seed = 1 if seed is None else int(seed)
        random.seed(seed)
        np.random.seed(seed)
        return [seed]

    def close(self):
        try:
            self.env.close()
        except Exception:
            pass

    def render(self, mode="human"):
        return None

    def _stagger_before_reset(self):
        stagger = (
            self.eval_reset_stagger_seconds
            if self.is_eval
            else self.reset_stagger_seconds
        )
        self._rank_stagger(stagger)

    def _stagger_before_launch(self):
        stagger = (
            self.eval_launch_stagger_seconds
            if self.is_eval
            else self.launch_stagger_seconds
        )
        self._rank_stagger(stagger)

    def _rank_stagger(self, stagger):
        if stagger <= 0.0:
            return
        time.sleep(stagger * float(self.rank + 1))

    def _make_env(self):
        return self._parallel_env_cls(cfg=self.cfg, **self.env_overrides)

    def _recreate_env_after_reset_failure(self, attempt, attempts, exc):
        print(
            "[RoboCup2DEnv] reset failed "
            f"rank={self.rank} eval={self.is_eval} "
            f"attempt={attempt}/{attempts}: {exc!r}; recreating env",
            flush=True,
        )
        try:
            self.env.close()
        except Exception as close_exc:
            print(
                "[RoboCup2DEnv] close during reset recovery failed "
                f"rank={self.rank}: {close_exc!r}",
                flush=True,
            )
        if self.wrapper_recreate_pause_seconds > 0.0:
            time.sleep(self.wrapper_recreate_pause_seconds)
        self._stagger_before_launch()
        self.env = self._make_env()

    def _obs(self):
        obs = np.asarray(self.env.get_obs(), dtype=np.float32)
        if obs.shape != (self.num_agents, self.base_obs_shape):
            obs = obs.reshape(self.num_agents, self.base_obs_shape)
        if not self.use_obs_agent_id:
            return obs
        return np.concatenate([obs, self.agent_id_eye], axis=1).astype(np.float32, copy=False)

    def _wrap_info(self, info: Dict[str, Any], reward: float) -> Dict[str, Any]:
        score_diff = int(info.get("score_diff", 0))
        win = int(info.get("win", 0))
        timeout = int(info.get("timeout", 0))
        steps_left = max(0, self.episode_limit - self._episode_steps)
        wrapped = dict(info)
        # football_runner uses score_reward > 0 as the win signal.
        wrapped["score_reward"] = float(1.0 if win > 0 else 0.0)
        wrapped["score_diff_reward"] = float(score_diff)
        wrapped["env_reward"] = float(reward)
        wrapped["max_steps"] = int(self.episode_limit)
        wrapped["steps_left"] = int(steps_left)
        wrapped["win_rate"] = float(win)
        wrapped["timeout_rate"] = float(timeout)
        wrapped["terminal_reason"] = str(info.get("terminal_reason", "running"))
        return wrapped
