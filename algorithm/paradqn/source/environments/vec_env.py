"""Vectorized environment wrapper for parallel data collection.

This module provides a SubprocVecEnv class that runs multiple environment
instances in parallel processes, allowing faster data collection during training.
"""

import os
import time
import traceback
import numpy as np
from typing import Callable, List, Dict, Tuple
from multiprocessing import Process, Pipe
from multiprocessing.connection import Connection


ERROR_TAG = "__paradqn_worker_error__"
READY_TAG = "__paradqn_worker_ready__"


class CloudpickleWrapper:
    """Use cloudpickle for env constructors, matching MAPPO/PyMARL workers."""

    def __init__(self, x):
        self.x = x

    def __getstate__(self):
        import cloudpickle
        return cloudpickle.dumps(self.x)

    def __setstate__(self, ob):
        import pickle
        self.x = pickle.loads(ob)


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return float(default)


def _error_payload(context: str, exc: BaseException):
    return (ERROR_TAG, context, repr(exc), traceback.format_exc())


def _recv_or_raise(remote: Connection, context: str):
    try:
        message = remote.recv()
    except EOFError as exc:
        raise RuntimeError(f"ParaDQN worker closed while waiting for {context}") from exc

    if isinstance(message, tuple) and len(message) == 4 and message[0] == ERROR_TAG:
        _, where, error, tb = message
        raise RuntimeError(
            f"ParaDQN worker error during {where} while waiting for {context}: {error}\n{tb}"
        )
    return message


def worker(remote: Connection, parent_remote: Connection, env_fn_wrapper: CloudpickleWrapper):
    """Worker function that runs in a subprocess to manage a single environment.

    Args:
        remote: Child process connection
        parent_remote: Parent process connection (closed in child)
        env_fn_wrapper: Wrapped function that creates an environment instance
    """
    parent_remote.close()
    env = None

    try:
        env = env_fn_wrapper.x()
        remote.send((READY_TAG, None))

        while True:
            try:
                cmd, data = remote.recv()
            except EOFError:
                break

            try:
                if cmd == "step":
                    observations, rewards, terminations, truncations, infos = env.step(data)
                    remote.send((observations, rewards, terminations, truncations, infos))

                elif cmd == "step_compact":
                    remote.send(env.step_compact(data))

                elif cmd == "reset":
                    observations, infos = env.reset()
                    remote.send((observations, infos))

                elif cmd == "reset_compact":
                    remote.send(env.reset_compact())

                elif cmd == "close":
                    break

                elif cmd == "get_spaces":
                    # Get observation space, state space and action space for one agent.
                    player = env.players[0]
                    obs_space = env.observation_space(player)
                    state_space = env.state_space
                    action_space = env.action_space(player)
                    num_agents = len(env.players)
                    players = env.players
                    remote.send((obs_space, state_space, action_space, num_agents, players))

                else:
                    raise NotImplementedError(f"Unknown command: {cmd}")
            except Exception as exc:
                remote.send(_error_payload(cmd, exc))

    except KeyboardInterrupt:
        pass
    except Exception as exc:
        try:
            remote.send(_error_payload("init", exc))
        except Exception:
            print(f"Worker init error: {exc}", flush=True)
    finally:
        try:
            if env is not None and hasattr(env, "close"):
                env.close()
        except Exception:
            pass
        try:
            remote.close()
        except Exception:
            pass


class SubprocVecEnv:
    """Vectorized environment that runs multiple environments in parallel subprocesses.

    This class manages multiple environment instances, each running in its own process,
    to enable parallel data collection and faster training.

    Args:
        env_fns: List of functions that create environment instances
    """

    def __init__(self, env_fns: List[Callable]):
        """Initialize the vectorized environment with multiple subprocesses.

        Args:
            env_fns: List of callables that create environment instances
        """
        self.waiting = False
        self.closed = False
        self.num_envs = len(env_fns)
        self.worker_start_stagger_seconds = max(
            0.0,
            _env_float(
                "PARADQN_WORKER_START_STAGGER_SECONDS",
                _env_float("PARADQN_LAUNCH_STAGGER_SECONDS", 0.0),
            ),
        )
        self.reset_stagger_seconds = max(
            0.0,
            _env_float(
                "PARADQN_VEC_RESET_STAGGER_SECONDS",
                _env_float("PARADQN_RESET_STAGGER_SECONDS", 0.0),
            ),
        )

        # Create pipes for communication with subprocesses.
        self.remotes, self.work_remotes = zip(*[Pipe() for _ in range(self.num_envs)])

        # Start subprocess for each environment.
        self.processes = []
        for idx, (work_remote, remote, env_fn) in enumerate(zip(self.work_remotes, self.remotes, env_fns)):
            args = (work_remote, remote, CloudpickleWrapper(env_fn))
            process = Process(target=worker, args=args, daemon=True)
            process.start()
            self.processes.append(process)
            work_remote.close()
            if self.worker_start_stagger_seconds > 0 and idx + 1 < self.num_envs:
                time.sleep(self.worker_start_stagger_seconds)

        for idx, remote in enumerate(self.remotes):
            status = _recv_or_raise(remote, f"worker {idx} ready")
            if not (isinstance(status, tuple) and len(status) == 2 and status[0] == READY_TAG):
                raise RuntimeError(f"Unexpected ParaDQN worker {idx} startup message: {status!r}")

        # Get environment spaces from first environment.
        self.remotes[0].send(("get_spaces", None))
        self.observation_space_single, self.state_space, self.action_space_single, self.num_agents, self.players = _recv_or_raise(
            self.remotes[0], "get_spaces"
        )

    def reset(self) -> Tuple[List[Dict], List[Dict]]:
        """Reset all environments in parallel.

        Returns:
            observations_list: List of observation dicts from each environment
            infos_list: List of info dicts from each environment
        """
        for idx, remote in enumerate(self.remotes):
            remote.send(("reset", None))
            if self.reset_stagger_seconds > 0 and idx + 1 < self.num_envs:
                time.sleep(self.reset_stagger_seconds)

        results = [_recv_or_raise(remote, "reset") for remote in self.remotes]
        observations_list, infos_list = zip(*results)

        return list(observations_list), list(infos_list)

    def reset_compact(self):
        """Reset all environments and return stacked observation/mask arrays."""
        for idx, remote in enumerate(self.remotes):
            remote.send(("reset_compact", None))
            if self.reset_stagger_seconds > 0 and idx + 1 < self.num_envs:
                time.sleep(self.reset_stagger_seconds)

        results = [_recv_or_raise(remote, "reset_compact") for remote in self.remotes]
        observations, masks, infos = zip(*results)
        return np.stack(observations, axis=0), np.stack(masks, axis=0), list(infos)

    def reset_at(self, env_idx: int):
        """Reset one environment by index and return its observation/info pair."""
        remote = self.remotes[int(env_idx)]
        remote.send(("reset", None))
        return _recv_or_raise(remote, f"reset env {env_idx}")

    def reset_at_compact(self, env_idx: int):
        """Reset one environment and return compact ndarray observation/mask data."""
        remote = self.remotes[int(env_idx)]
        remote.send(("reset_compact", None))
        return _recv_or_raise(remote, f"reset_compact env {env_idx}")

    def step_compact(self, action_nums: np.ndarray, action_params: np.ndarray):
        """Step all environments using compact ndarray actions."""
        for env_idx, remote in enumerate(self.remotes):
            remote.send(("step_compact", (action_nums[env_idx], action_params[env_idx])))

        results = [_recv_or_raise(remote, "step_compact") for remote in self.remotes]
        observations, masks, rewards, dones, infos = zip(*results)
        return (
            np.stack(observations, axis=0),
            np.stack(masks, axis=0),
            np.stack(rewards, axis=0),
            np.asarray(dones, dtype=bool),
            list(infos),
        )

    def step(self, actions_list: List[Dict]) -> Tuple[List[Dict], List[Dict], List[Dict], List[Dict], List[Dict]]:
        """Step all environments in parallel.

        Args:
            actions_list: List of action dicts, one for each environment

        Returns:
            observations_list: List of observation dicts from each environment
            rewards_list: List of reward dicts from each environment
            terminations_list: List of termination dicts from each environment
            truncations_list: List of truncation dicts from each environment
            infos_list: List of info dicts from each environment
        """
        for remote, actions in zip(self.remotes, actions_list):
            remote.send(("step", actions))

        results = [_recv_or_raise(remote, "step") for remote in self.remotes]
        observations_list, rewards_list, terminations_list, truncations_list, infos_list = zip(*results)

        return list(observations_list), list(rewards_list), list(terminations_list), list(truncations_list), list(infos_list)

    def close(self):
        """Close all subprocesses."""
        if self.closed:
            return

        if self.waiting:
            for remote in self.remotes:
                try:
                    remote.recv()
                except Exception:
                    pass

        for remote in self.remotes:
            try:
                remote.send(("close", None))
            except Exception:
                # Process may have already terminated.
                pass

        for process in self.processes:
            process.join(timeout=5)
            if process.is_alive():
                process.terminate()
                process.join(timeout=1)

        self.closed = True

    def __len__(self):
        """Return the number of parallel environments."""
        return self.num_envs

    def __del__(self):
        """Cleanup when object is deleted."""
        if not self.closed:
            self.close()
