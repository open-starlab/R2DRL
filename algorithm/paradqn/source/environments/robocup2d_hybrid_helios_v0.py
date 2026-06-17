from .robocup2d.env import Robocup2dEnv, R2DRL, ParallelR2DRL
from .robocup2d.start_sampler import ScenarioStartSampler
try:
    from gymnasium import spaces
except ModuleNotFoundError:
    from gym import spaces
import numpy as np
# from pettingzoo import ParallelEnv
import functools
import os
import sys
import threading
import time
import traceback
from datetime import datetime


def _as_float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _as_bool(value, default=False):
    if value is None:
        return bool(default)
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def _env_float(name, default=0.0):
    return _as_float(os.environ.get(name, default), default)


def _pop_float(env_args, key, env_name, default=0.0):
    value = env_args.pop(key, None)
    if value is None:
        value = os.environ.get(env_name, default)
    return max(0.0, _as_float(value, default))


def warn_if_slow(*, warn_after_seconds=60.0, repeat_seconds=60.0):
    def decorator(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            done = threading.Event()
            start_time = time.monotonic()
            thread_id = threading.get_ident()

            def monitor():
                next_warn_after = warn_after_seconds
                while not done.wait(timeout=next_warn_after):
                    elapsed = time.monotonic() - start_time
                    frame = sys._current_frames().get(thread_id)
                    stack = "".join(traceback.format_stack(frame)) if frame is not None else "<stack unavailable>"
                    print(
                        f"[{datetime.now().isoformat(timespec='seconds')}] [WARN] {func.__qualname__} "
                        f"running for {elapsed:.1f}s\nCurrent stack:\n{stack}",
                        flush=True,
                    )
                    next_warn_after = repeat_seconds

            monitor_thread = threading.Thread(
                target=monitor,
                name=f"slow-watch-{func.__name__}",
                daemon=True,
            )
            monitor_thread.start()

            try:
                return func(*args, **kwargs)
            finally:
                done.set()

        return wrapper

    return decorator


class RobocupEnv_Hybrid_Helios:
    metadata = {
        "name": "robocup2d_hybrid_helios_v0",
    }

    def __init__(self, cfg="parallelr2drl.yaml", **env_args):
        env_args = dict(env_args)
        self.rank = int(env_args.pop("rank", 0) or 0)
        self.is_eval = bool(env_args.pop("is_eval", False))
        self.use_obs_agent_id = _as_bool(env_args.pop("obs_agent_id", True), default=True)
        self.launch_stagger_seconds = _pop_float(
            env_args,
            "launch_stagger_seconds",
            "PARADQN_ROBOCUP_LAUNCH_STAGGER_SECONDS",
            0.0,
        )
        eval_launch = env_args.pop("eval_launch_stagger_seconds", None)
        if eval_launch is None:
            eval_launch = os.environ.get(
                "PARADQN_ROBOCUP_EVAL_LAUNCH_STAGGER_SECONDS",
                self.launch_stagger_seconds,
            )
        self.eval_launch_stagger_seconds = max(0.0, _as_float(eval_launch, self.launch_stagger_seconds))
        self.reset_stagger_seconds = _pop_float(
            env_args,
            "reset_stagger_seconds",
            "PARADQN_ROBOCUP_RESET_STAGGER_SECONDS",
            0.0,
        )
        eval_reset = env_args.pop("eval_reset_stagger_seconds", None)
        if eval_reset is None:
            eval_reset = os.environ.get(
                "PARADQN_ROBOCUP_EVAL_RESET_STAGGER_SECONDS",
                self.reset_stagger_seconds,
            )
        self.eval_reset_stagger_seconds = max(0.0, _as_float(eval_reset, self.reset_stagger_seconds))

        env_args["team"] = "hybrid"
        self._stagger_before_launch()
        self.env = Robocup2dEnv(cfg=cfg, **env_args)
        self._ensure_ipc_attached()
        self.start_sampler = self._make_start_sampler()
        info = self.env.get_env_info()

        self.num_agents = int(info["n_agents"])
        self.base_obs_shape = int(info["obs_shape"])
        self.agent_id_dim = self.num_agents if self.use_obs_agent_id else 0
        self.obs_shape = self.base_obs_shape + self.agent_id_dim
        self.n_actions = int(info["n_actions"])
        self.players = [f"l{i}" for i in range(1, self.num_agents + 1)]
        self.agent_id_eye = (
            np.eye(self.num_agents, dtype=np.float32)
            if self.use_obs_agent_id
            else None
        )

        self.observation_spaces = {
            agent: spaces.Box(
                low=-float("inf"),
                high=float("inf"),
                shape=(self.obs_shape,),
                dtype=np.float32,
            )
            for agent in self.players
        }
        self.state_space = spaces.Box(
            low=-float("inf"),
            high=float("inf"),
            shape=(self.obs_shape * self.num_agents,),
            dtype=np.float32,
        )
        self.action_spaces = {
            agent: spaces.Tuple(
                (
                    spaces.Discrete(self.n_actions),
                    spaces.Box(0, 1, shape=(self.n_actions * 2,), dtype=np.float32),
                )
            )
            for agent in self.players
        }

    def _obs_with_agent_id(self, observations_arr):
        observations_arr = np.asarray(observations_arr, dtype=np.float32)
        if observations_arr.shape != (self.num_agents, self.base_obs_shape):
            observations_arr = observations_arr.reshape(self.num_agents, self.base_obs_shape)
        if not self.use_obs_agent_id:
            return observations_arr
        return np.concatenate([observations_arr, self.agent_id_eye], axis=1).astype(np.float32, copy=False)

    def _observation_arrays(self):
        observations_arr = self._obs_with_agent_id(self.env.get_obs())
        masks = np.asarray(self.env.get_avail_actions(), dtype=np.int8)
        return observations_arr, masks

    def _observations_dict(self, observations_arr, masks):
        observations = {}
        for i, agent in enumerate(self.players):
            observations[agent] = {
                "observation": observations_arr[i],
                "action_mask": masks[i],
            }
        return observations

    def _action_dict_to_arrays(self, actions):
        action_nums = np.empty((self.num_agents,), dtype=np.int64)
        action_params = np.empty((self.num_agents, self.n_actions * 2), dtype=np.float32)
        for i, agent in enumerate(self.players):
            action = actions.get(agent)
            action_nums[i] = int(action[0])
            action_params[i] = np.asarray(action[1], dtype=np.float32)
        return action_nums, action_params

    def _action_arrays_to_env_rows(self, action_nums, action_params):
        action_nums = np.asarray(action_nums, dtype=np.int64).reshape(self.num_agents)
        action_params = np.asarray(action_params, dtype=np.float32).reshape(self.num_agents, -1)
        param_indices = action_nums[:, None] * 2 + np.array([[0, 1]], dtype=np.int64)
        selected_params = np.take_along_axis(action_params, param_indices, axis=1)
        action_array = np.empty((self.num_agents, 3), dtype=np.float32)
        action_array[:, 0] = action_nums.astype(np.float32)
        action_array[:, 1:3] = selected_params
        return action_array

    def _make_start_sampler(self):
        env_cfg = self.env.config
        if not (
            bool(getattr(env_cfg, "use_custom_start", False))
            and getattr(env_cfg, "trajectory_path", None)
        ):
            return None

        sampler = ScenarioStartSampler(
            init_n=int(getattr(env_cfg, "init_n")),
            start_window_size=int(getattr(env_cfg, "start_window_size")),
            progress_bucket_count=int(getattr(env_cfg, "progress_bucket_count")),
            current_target_window_size=int(getattr(env_cfg, "current_target_window_size")),
            num_selected_trajectories=getattr(env_cfg, "num_selected_trajectories"),
            random_sample=getattr(env_cfg, "random_sample"),
            trajectory_path=getattr(env_cfg, "trajectory_path"),
            scenario_difficulty=getattr(env_cfg, "scenario_difficulty", None),
            scenario_difficulty_buckets=getattr(env_cfg, "scenario_difficulty_buckets", None),
            start_catalog_enabled=getattr(env_cfg, "start_catalog_enabled", False),
            start_id=getattr(env_cfg, "start_id", None),
            start_index=getattr(env_cfg, "start_index", None),
        )
        env_n = int(getattr(env_cfg, "n"))
        traj_n = int(sampler.n_players)
        if env_n > traj_n:
            raise ValueError(
                f"env n={env_n} but trajectory has only {traj_n} players per side. "
                "Use n <= trajectory player count, or use a matching trajectory_path in the yaml."
            )
        return sampler

    @staticmethod
    def _read_text(path):
        try:
            with open(path, "r", encoding="utf-8", errors="replace") as f:
                return f.read()
        except OSError:
            return ""

    def _ipc_log_paths(self):
        runtime = self.env.runtime
        tag = runtime.run_id
        return (
            os.path.join(runtime.log_dir, f"{tag}_trainer_{runtime.trainer_port}.log"),
            os.path.join(runtime.log_dir, f"{tag}_coach_{runtime.coach_port}.log"),
        )

    def _ipc_status(self):
        trainer_log, coach_log = self._ipc_log_paths()
        trainer_text = self._read_text(trainer_log)
        coach_text = self._read_text(coach_log)
        trainer_failed = (
            "shm_open attach" in trainer_text
            or "WARN: SHM init failed" in trainer_text
        )
        coach_failed = "shm_open(attach-only)" in coach_text
        return {
            "trainer_attached": "[trainer] shm attached:" in trainer_text,
            "trainer_failed": trainer_failed,
            "coach_failed": coach_failed,
            "coach_log_seen": bool(coach_text),
        }

    def _ensure_ipc_attached(self):
        retries = int(os.environ.get("PARADQN_IPC_VERIFY_RETRIES", "4"))
        wait_seconds = _env_float("PARADQN_IPC_VERIFY_WAIT_SECONDS", 8.0)
        poll_seconds = _env_float("PARADQN_IPC_VERIFY_POLL_SECONDS", 0.25)

        for attempt in range(retries + 1):
            deadline = time.monotonic() + max(0.0, wait_seconds)
            status = {}
            while time.monotonic() <= deadline:
                status = self._ipc_status()
                if status.get("trainer_failed") or status.get("coach_failed"):
                    break
                if status.get("trainer_attached") and status.get("coach_log_seen"):
                    return
                time.sleep(max(0.01, poll_seconds))

            if attempt >= retries:
                raise RuntimeError(f"ParaDQN RoboCup IPC attach failed after retries: {status}")

            runtime = self.env.runtime
            print(
                "[ParaDQNWrapper] IPC attach check failed; restarting env "
                f"attempt={attempt + 1}/{retries} rank={self.rank} run_id={runtime.run_id} status={status}",
                flush=True,
            )
            self.env.restart()

    def _stagger_before_launch(self):
        stagger = self.eval_launch_stagger_seconds if self.is_eval else self.launch_stagger_seconds
        self._rank_stagger(stagger)

    def _stagger_before_reset(self):
        stagger = self.eval_reset_stagger_seconds if self.is_eval else self.reset_stagger_seconds
        self._rank_stagger(stagger)

    def _rank_stagger(self, stagger):
        if stagger <= 0.0:
            return
        time.sleep(stagger * float(self.rank + 1))

    @warn_if_slow()
    def reset_compact(self, seed=None, options=None):
        """Reset and return ndarray observations/masks for vectorized training."""
        _ = seed
        _ = options

        if self.start_sampler is not None:
            start, n_control = self.start_sampler.sample_start_and_n()
            self.env.set_start_and_n(start, n_control)

        self._stagger_before_reset()
        info = self.env.reset()
        observations_arr, masks = self._observation_arrays()
        return observations_arr, masks, info or {}

    @warn_if_slow()
    def reset(self, seed=None, options=None):
        """Reset the environment to a starting point."""
        observations_arr, masks, _ = self.reset_compact(seed=seed, options=options)
        observations = self._observations_dict(observations_arr, masks)
        infos = {agent: {} for agent in self.players}
        return observations, infos

    @warn_if_slow()
    def step_compact(self, actions):
        """Step with compact ndarray actions and return ndarray transition data."""
        action_nums, action_params = actions
        action_array = self._action_arrays_to_env_rows(action_nums, action_params)
        reward, termination, info = self.env.step(action_array)
        observations_arr, masks = self._observation_arrays()
        rewards = np.full((self.num_agents,), float(reward), dtype=np.float32)
        return observations_arr, masks, rewards, bool(termination), info or {}

    @warn_if_slow()
    def step(self, actions):
        """Take a step in the environment using parametrized hybrid actions."""
        observations_arr, masks, reward_arr, termination, info = self.step_compact(
            self._action_dict_to_arrays(actions)
        )
        observations = self._observations_dict(observations_arr, masks)
        reward = float(reward_arr[0]) if reward_arr.size else 0.0
        rewards = {agent: reward for agent in self.players}
        terminations = {agent: termination for agent in self.players}
        truncations = {agent: termination for agent in self.players}
        infos = {agent: info for agent in self.players}
        return observations, rewards, terminations, truncations, infos

    def render(self):
        raise NotImplementedError("Render not implemented for RobocupEnv.")

    def close(self):
        """Close the environment and release resources."""
        if hasattr(self, "env") and self.env is not None:
            self.env.close()

    def train(self):
        self.env.test_mode = False
        return self

    def eval(self):
        self.env.test_mode = True
        return self

    @functools.lru_cache(maxsize=None)
    def observation_space(self, agent):
        return self.observation_spaces[agent]

    @functools.lru_cache(maxsize=None)
    def action_space(self, agent):
        return self.action_spaces[agent]
