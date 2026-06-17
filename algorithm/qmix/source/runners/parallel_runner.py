from envs import REGISTRY as env_REGISTRY
from functools import partial
from components.episode_buffer import EpisodeBatch
from multiprocessing import Pipe, Process
import json
import numpy as np
from numbers import Number
import os
import torch as th
import time
import signal
from envs.robocup2d.start_sampler import ScenarioStartSampler, apply_catalog_launch_profile


# Based (very) heavily on SubprocVecEnv from OpenAI Baselines
# https://github.com/openai/baselines/blob/master/baselines/common/vec_env/subproc_vec_env.py
class ParallelRunner:

    @staticmethod
    def _merge_info_dicts(dicts):
        merged = {}
        if not dicts:
            return merged

        for key in set().union(*(set(d) for d in dicts)):
            values = [d[key] for d in dicts if key in d]
            if not values:
                continue

            if all(isinstance(v, Number) for v in values):
                merged[key] = sum(values)
            else:
                # Non-numeric info is only used for debugging. Carrying prior merged
                # containers forward causes nested lists to grow across episodes.
                current_values = [d[key] for d in dicts[1:] if key in d]
                merged[key] = current_values[-1] if current_values else values[-1]

        return merged

    @staticmethod
    def _truthy(value):
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    @staticmethod
    def _step_label(step):
        step = int(step)
        if step == 0:
            return "0m"
        if step % 1_000_000 == 0:
            return f"{step // 1_000_000}m"
        if step % 1_000 == 0:
            return f"{step // 1_000}k"
        return str(step)

    @staticmethod
    def _numeric(value, default=0.0):
        try:
            return float(value)
        except (TypeError, ValueError):
            return float(default)

    def _label_step(self, step):
        try:
            step_i = int(step)
            interval_i = int(getattr(self.args, "test50_label_interval", getattr(self.args, "test_interval", 0)) or 0)
        except (TypeError, ValueError):
            return int(step)
        if interval_i <= 0:
            return step_i
        rounded = int(round(float(step_i) / float(interval_i))) * interval_i
        if rounded > 0 and abs(step_i - rounded) <= max(1, interval_i // 2):
            return rounded
        return step_i

    def __init__(self, args, logger):
        self.args = args
        self.logger = logger
        self.batch_size = self.args.batch_size_run
        env_args = getattr(self.args, "env_args", {})
        apply_catalog_launch_profile(env_args)
        benchmark_mode = str(env_args.get("benchmark_mode", "")).strip().lower()
        if benchmark_mode == "scenario":
            self.use_custom_start = True
        elif benchmark_mode == "full_match":
            self.use_custom_start = False
        else:
            self.use_custom_start = bool(env_args.get("use_custom_start", False))
        env_args["use_custom_start"] = self.use_custom_start
        self.start_sampler = None
        trajectory_path = env_args.get("trajectory_path")
        if self.use_custom_start:
            if trajectory_path:
                self.start_sampler = ScenarioStartSampler(
                    init_n=int(env_args["init_n"]),
                    start_window_size=int(env_args["start_window_size"]),
                    progress_bucket_count=int(env_args["progress_bucket_count"]),
                    current_target_window_size=int(env_args["current_target_window_size"]),
                    num_selected_trajectories=env_args["num_selected_trajectories"],
                    random_sample=env_args["random_sample"],
                    trajectory_path=trajectory_path,
                    scenario_difficulty=env_args.get("scenario_difficulty"),
                    scenario_difficulty_buckets=env_args.get("scenario_difficulty_buckets"),
                    start_catalog_enabled=env_args.get("start_catalog_enabled", False),
                    start_id=env_args.get("start_id"),
                    start_index=env_args.get("start_index"),
                )
                env_n = int(env_args["n"])
                traj_n = int(self.start_sampler.n_players)
                if env_n > traj_n:
                    raise ValueError(
                        f"env_args.n={env_n} but trajectory has only {traj_n} players per side. "
                        "Use n <= trajectory player count, or use a matching trajectory_path in the yaml."
                    )

        # Make subprocesses for the envs
        self.parent_conns, self.worker_conns = zip(
            *[Pipe() for _ in range(self.batch_size)]
        )
        env_fn = env_REGISTRY[self.args.env]
        self.ps = [
            Process(
                target=env_worker,
                args=(
                    worker_conn,
                    CloudpickleWrapper(partial(env_fn, **self.args.env_args)),
                ),
            )
            for worker_conn in self.worker_conns
        ]
        self._env_closed = False

        for p in self.ps:
            p.daemon = True
            p.start()
            time.sleep(1)
        for worker_conn in self.worker_conns:
            worker_conn.close()

        self.parent_conns[0].send(("get_env_info", None))
        self.env_info = self.parent_conns[0].recv()
        self.episode_limit = self.env_info["episode_limit"]

        self.t = 0
        self.t_env = 0

        self.train_returns = []
        self.test_returns = []
        self.train_stats = {}
        self.test_stats = {}
        self.train_episode_count = 0
        self.test_episode_count = 0

        self.log_train_stats_t = -100000

    @staticmethod
    def _stack_pre_transition_data(data):
        if data["state"]:
            data["state"] = np.asarray(data["state"])
        if data["avail_actions"]:
            data["avail_actions"] = np.asarray(data["avail_actions"])
        if data["obs"]:
            data["obs"] = np.asarray(data["obs"])
        return data

    @staticmethod
    def _stack_post_transition_data(data):
        if data["reward"]:
            data["reward"] = np.asarray(data["reward"], dtype=np.float32)
        if data["terminated"]:
            data["terminated"] = np.asarray(data["terminated"], dtype=np.uint8)
        return data

    def setup(self, scheme, groups, preprocess, mac):
        self.new_batch = partial(
            EpisodeBatch,
            scheme,
            groups,
            self.batch_size,
            self.episode_limit + 1,
            preprocess=preprocess,
            device=self.args.device,
        )
        self.mac = mac
        self.scheme = scheme
        self.groups = groups
        self.preprocess = preprocess

    def _safe_win_lose_ratio(self, stats):
        win_count = float(stats.get("win", 0))
        lose_count = float(stats.get("lose", 0))
        if win_count <= 0 or lose_count <= 0:
            return 0.0
        return win_count / lose_count

    def _log_episode_rewards(self, episode_returns, test_mode=False):
        prefix = "test/" if test_mode else "train/"
        episode_count_attr = (
            "test_episode_count" if test_mode else "train_episode_count"
        )
        for episode_return in episode_returns:
            setattr(
                self,
                episode_count_attr,
                getattr(self, episode_count_attr) + 1,
            )
            step = getattr(self, episode_count_attr)
            self.logger.log_stat(f"{prefix}episode_reward", float(episode_return), step)

    def get_env_info(self):
        return self.env_info

    def save_replay(self):
        pass

    def close_env(self):
        if self._env_closed:
            return
        self._env_closed = True

        for parent_conn in getattr(self, "parent_conns", ()):
            try:
                parent_conn.send(("close", None))
            except (BrokenPipeError, EOFError, OSError):
                pass

        for parent_conn in getattr(self, "parent_conns", ()):
            try:
                parent_conn.close()
            except OSError:
                pass

        for worker_conn in getattr(self, "worker_conns", ()):
            try:
                worker_conn.close()
            except OSError:
                pass

        deadline = time.time() + 2.0
        for p in getattr(self, "ps", ()):
            if not p.is_alive():
                continue
            remaining = max(0.0, deadline - time.time())
            p.join(timeout=remaining)

        alive = [p for p in getattr(self, "ps", ()) if p.is_alive()]
        for p in alive:
            p.terminate()
        for p in alive:
            p.join(timeout=1.0)

        alive = [p for p in getattr(self, "ps", ()) if p.is_alive()]
        for p in alive:
            p.kill()
        for p in alive:
            p.join(timeout=1.0)

    def reset(self, test_mode=False):
        self.batch = self.new_batch()

        for parent_conn in self.parent_conns:
            parent_conn.send(("set_test_mode", test_mode))
        for parent_conn in self.parent_conns:
            parent_conn.recv()

        if self.start_sampler is not None:
            starts = []
            n_controls = []
            for i in range(self.batch_size):
                start, n_control = self.start_sampler.sample_start_and_n()
                starts.append(start)
                n_controls.append(n_control)
            for i, parent_conn in enumerate(self.parent_conns):
                parent_conn.send(("set_start_and_n", (starts[i], n_controls[i])))
            for parent_conn in self.parent_conns:
                parent_conn.recv()

        # Reset the envs
        for parent_conn in self.parent_conns:
            parent_conn.send(("reset", None))

        pre_transition_data = {
            "state": [],
            "avail_actions": [],
            "obs": [],
        }
        # Get the obs, state and avail_actions back
        for parent_conn in self.parent_conns:
            data = parent_conn.recv()
            pre_transition_data["state"].append(data["state"])
            pre_transition_data["avail_actions"].append(data["avail_actions"])
            pre_transition_data["obs"].append(data["obs"])

        pre_transition_data = self._stack_pre_transition_data(pre_transition_data)
        self.batch.update(pre_transition_data, ts=0)

        self.t = 0
        self.env_steps_this_run = 0
    def run(self, test_mode=False):
        self.reset(test_mode=test_mode)

        episode_returns = [0 for _ in range(self.batch_size)]
        episode_lengths = [0 for _ in range(self.batch_size)]
        self.mac.init_hidden(batch_size=self.batch_size)
        terminated = [False for _ in range(self.batch_size)]
        final_env_infos = []
        while True:
            envs_not_terminated = [
                b_idx for b_idx, termed in enumerate(terminated) if not termed
            ]
            if len(envs_not_terminated) == 0:
                break

            actions = self.mac.select_actions(
                self.batch,
                t_ep=self.t,
                t_env=self.t_env,
                bs=envs_not_terminated,
                test_mode=test_mode,
            )
            cpu_actions = actions.to("cpu").numpy()

            actions_chosen = {
                "actions": actions.unsqueeze(1)
            }
            self.batch.update(
                actions_chosen,
                bs=envs_not_terminated,
                ts=self.t,
                mark_filled=False,
            )

            for action_idx, idx in enumerate(envs_not_terminated):
                self.parent_conns[idx].send(("step", cpu_actions[action_idx]))

            post_transition_data = {
                "reward": [],
                "terminated": [],
            }
            pre_transition_data = {
                "state": [],
                "avail_actions": [],
                "obs": [],
            }

            for idx, parent_conn in enumerate(self.parent_conns):
                if not terminated[idx]:
                    data = parent_conn.recv()
                    post_transition_data["reward"].append((data["reward"],))

                    episode_returns[idx] += data["reward"]
                    episode_lengths[idx] += 1
                    if not test_mode:
                        self.env_steps_this_run += 1

                    env_terminated = False
                    if data["terminated"]:
                        final_env_infos.append(data["info"])
                    if data["terminated"] and not data["info"].get(
                        "episode_limit", False
                    ):
                        env_terminated = True
                    terminated[idx] = data["terminated"]
                    post_transition_data["terminated"].append((env_terminated,))

                    pre_transition_data["state"].append(data["state"])
                    pre_transition_data["avail_actions"].append(data["avail_actions"])
                    pre_transition_data["obs"].append(data["obs"])

            post_transition_data = self._stack_post_transition_data(post_transition_data)
            self.batch.update(
                post_transition_data,
                bs=envs_not_terminated,
                ts=self.t,
                mark_filled=False,
            )

            self.t += 1
            pre_transition_data = self._stack_pre_transition_data(pre_transition_data)
            self.batch.update(
                pre_transition_data,
                bs=envs_not_terminated,
                ts=self.t,
                mark_filled=True,
            )

        if not test_mode:
            self.t_env += self.env_steps_this_run
            self._log_episode_rewards(episode_returns, test_mode=False)
        else:
            self._log_episode_rewards(episode_returns, test_mode=True)

        cur_stats = self.test_stats if test_mode else self.train_stats
        cur_returns = self.test_returns if test_mode else self.train_returns
        log_prefix = "test_" if test_mode else ""
        infos = [cur_stats] + final_env_infos
        cur_stats.update(self._merge_info_dicts(infos))
        cur_stats["n_episodes"] = self.batch_size + cur_stats.get("n_episodes", 0)
        if test_mode:
            cur_stats["draw"] = (
                cur_stats["n_episodes"]
                - cur_stats.get("win", 0)
                - cur_stats.get("lose", 0)
            )
        cur_stats["ep_length"] = sum(episode_lengths) + cur_stats.get("ep_length", 0)

        cur_returns.extend(episode_returns)
        n_test_runs = max(1, self.args.test_nepisode // self.batch_size) * self.batch_size
        if test_mode and (len(self.test_returns) == n_test_runs):
            n_episodes = max(1, cur_stats["n_episodes"])
            test_win_rate = float(cur_stats.get("win", 0) / n_episodes)
            test_loss_rate = float(cur_stats.get("lose", 0) / n_episodes)
            test_draw_rate = float(cur_stats.get("draw", 0) / n_episodes)
            test_win_minus_loss_rate = float(
                (cur_stats.get("win", 0) - cur_stats.get("lose", 0)) / n_episodes
            )
            test_win_lose_ratio = self._safe_win_lose_ratio(cur_stats)
            self.logger.log_stat("test_win_rate", test_win_rate, self.t_env)
            self.logger.log_stat("test_loss_rate", test_loss_rate, self.t_env)
            self.logger.log_stat("test_draw_rate", test_draw_rate, self.t_env)
            self.logger.log_stat("test_win_minus_loss_rate", test_win_minus_loss_rate, self.t_env)
            self.logger.log_stat("test_win_lose_ratio", test_win_lose_ratio, self.t_env)
            self._log(cur_returns, cur_stats, log_prefix)
        elif (not test_mode) and self.t_env - self.log_train_stats_t >= self.args.runner_log_interval:
            self._log(cur_returns, cur_stats, log_prefix)
            if hasattr(self.mac.action_selector, "epsilon"):
                self.logger.log_stat("epsilon", self.mac.action_selector.epsilon, self.t_env)
            self.log_train_stats_t = self.t_env

        return self.batch

    def _test50_output_dir(self, step):
        if not self._truthy(getattr(self.args, "test50_during_training", False)):
            return None, None

        template = str(getattr(self.args, "test50_output_dir", "") or "")
        if not template:
            template = os.path.join(
                str(getattr(self.args, "local_results_path", "results")),
                "test50",
                str(getattr(self.args, "unique_token", "run")),
            )

        uses_placeholder = "{" in template and "}" in template
        label_step = self._label_step(step)
        step_label = self._step_label(label_step)
        try:
            out_dir = template.format(step=label_step, step_label=step_label)
        except Exception as exc:
            self.logger.console_logger.warning(
                f"[test50] Failed to format test50_output_dir={template!r}: {exc}"
            )
            return None, None

        filename = "result.json" if uses_placeholder else f"step_{int(step)}.json"
        return out_dir, filename

    def _write_test50_summary(self, returns, stats):
        out_dir, filename = self._test50_output_dir(self.t_env)
        if not out_dir:
            return

        n_episodes = max(1, int(stats.get("n_episodes", len(returns))))
        win = self._numeric(stats.get("win", 0.0))
        lose = self._numeric(stats.get("lose", 0.0))
        draw = self._numeric(stats.get("draw", n_episodes - win - lose))
        mean_return = float(np.mean(returns)) if returns else 0.0
        std_return = float(np.std(returns)) if returns else 0.0
        mean_goal_diff = self._numeric(stats.get("score_diff", 0.0)) / n_episodes
        mean_score_left = self._numeric(stats.get("score_left", 0.0)) / n_episodes
        mean_score_right = self._numeric(stats.get("score_right", 0.0)) / n_episodes
        result = {
            "algorithm": str(getattr(self.args, "name", getattr(self.args, "alg_name", "qmix"))),
            "step": int(self.t_env),
            "label_step": int(self._label_step(self.t_env)),
            "step_label": self._step_label(self._label_step(self.t_env)),
            "requested_episodes": int(getattr(self.args, "test50_episodes", getattr(self.args, "test_nepisode", n_episodes)) or n_episodes),
            "completed_episodes": n_episodes,
            "test_nepisode": int(getattr(self.args, "test_nepisode", n_episodes) or n_episodes),
            "mean_return": mean_return,
            "return_std": std_return,
            "win_count": int(win),
            "loss_count": int(lose),
            "draw_count": int(draw),
            "test_win_rate": float(win / n_episodes),
            "test_loss_rate": float(lose / n_episodes),
            "test_draw_rate": float(draw / n_episodes),
            "test_win_minus_loss_rate": float((win - lose) / n_episodes),
            "test_win_lose_ratio": self._safe_win_lose_ratio(stats),
            "mean_goal_diff": float(mean_goal_diff),
            "mean_score_diff": float(mean_goal_diff),
            "mean_score_left": float(mean_score_left),
            "mean_score_right": float(mean_score_right),
            "episode_length": self._numeric(stats.get("ep_length", 0.0)) / n_episodes,
            "possession_loss_rate": self._numeric(stats.get("possession_loss", 0.0)) / n_episodes,
            "max_epv": self._numeric(stats.get("max_episode_epv", 0.0)) / n_episodes,
            "max_epv_improvement": self._numeric(stats.get("max_epv_improvement", 0.0)) / n_episodes,
        }

        os.makedirs(out_dir, exist_ok=True)
        out_path = os.path.join(out_dir, filename)
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(result, f, indent=2, sort_keys=True)
        if filename != "latest.json":
            with open(os.path.join(out_dir, "latest.json"), "w", encoding="utf-8") as f:
                json.dump(result, f, indent=2, sort_keys=True)
        summary_path = os.path.join(out_dir, "summary.md")
        with open(summary_path, "w", encoding="utf-8") as f:
            f.write("| key | value |\n| - | - |\n")
            for key in sorted(result):
                f.write(f"| {key} | {result[key]} |\n")
        self.logger.console_logger.info(f"[test50] Wrote {out_path}")

    def _log(self, returns, stats, prefix):
        if prefix == "test_":
            self._write_test50_summary(returns, stats)
            n_episodes = max(1, stats.get("n_episodes", len(returns)))
            goal_rate = stats.get("win", 0) / n_episodes
            self.logger.log_stat("test/goal_rate", goal_rate, self.t_env)
            self.logger.log_stat(prefix + "win_rate", goal_rate, self.t_env)
            self.logger.log_stat(prefix + "loss_rate", stats.get("lose", 0) / n_episodes, self.t_env)
            draw_count = stats.get("draw", n_episodes - stats.get("win", 0) - stats.get("lose", 0))
            self.logger.log_stat(prefix + "draw_rate", draw_count / n_episodes, self.t_env)
            self.logger.log_stat(
                prefix + "win_minus_loss_rate",
                (stats.get("win", 0) - stats.get("lose", 0)) / n_episodes,
                self.t_env,
            )
            self.logger.log_stat(
                prefix + "win_lose_ratio",
                self._safe_win_lose_ratio(stats),
                self.t_env,
            )
            self.logger.log_stat(prefix + "mean_goal_diff", stats.get("score_diff", 0.0) / n_episodes, self.t_env)
        else:
            self.logger.log_stat(prefix + "return_mean", np.mean(returns), self.t_env)
            self.logger.log_stat(prefix + "return_std", np.std(returns), self.t_env)

        n_episodes = max(1, stats.get("n_episodes", len(returns)))
        metric_prefix = "test/" if prefix == "test_" else "train/"
        if len(returns) > 0:
            self.logger.log_stat(metric_prefix + "average_episode_return", np.mean(returns), self.t_env)
        self.logger.log_stat(metric_prefix + "episode_length", stats.get("ep_length", 0) / n_episodes, self.t_env)
        self.logger.log_stat(metric_prefix + "possession_loss_rate", stats.get("possession_loss", 0) / n_episodes, self.t_env)
        self.logger.log_stat(metric_prefix + "max_epv", stats.get("max_episode_epv", 0.0) / n_episodes, self.t_env)
        self.logger.log_stat(metric_prefix + "max_epv_improvement", stats.get("max_epv_improvement", 0.0) / n_episodes, self.t_env)
        returns.clear()

        for k, v in stats.items():
            if k == "n_episodes":
                continue
            if not isinstance(v, Number):
                continue
            self.logger.log_stat(prefix + k + "_mean", v / stats["n_episodes"], self.t_env)
        stats.clear()


def env_worker(remote, env_fn):
    env = None

    def _cleanup_on_signal(signum, frame):
        if env is not None:
            try:
                env.close()
            except Exception:
                pass
        try:
            remote.close()
        except Exception:
            pass
        raise SystemExit(128 + int(signum))

    try:
        signal.signal(signal.SIGTERM, _cleanup_on_signal)
        signal.signal(signal.SIGINT, _cleanup_on_signal)
    except Exception:
        pass

    try:
        env = env_fn.x()
        while True:
            try:
                cmd, data = remote.recv()
            except EOFError:
                break

            if cmd == "step":
                actions = data
                # Take a step in the environment
                reward, terminated, env_info = env.step(actions)
                transition = env._get_transition_payload()
                remote.send({
                    # Data for the next timestep needed to pick an action
                    "state": transition["state"],
                    "avail_actions": transition["avail_actions"],
                    "obs": transition["obs"],
                    # Rest of the data for the current timestep
                    "reward": reward,
                    "terminated": terminated,
                    "info": env_info
                })
            elif cmd == "reset":
                env.reset()
                remote.send(env._get_transition_payload())
            elif cmd == "set_test_mode":
                remote.send(env.set_test_mode(data))
            elif cmd == "set_start_and_n":
                start, n_control = data
                remote.send(env.set_start_and_n(start, n_control))
            elif cmd == "close":
                break
            elif cmd == "get_env_info":
                remote.send(env.get_env_info())
            else:
                raise NotImplementedError
    finally:
        if env is not None:
            try:
                env.close()
            except Exception:
                pass
        try:
            remote.close()
        except Exception:
            pass


class CloudpickleWrapper():
    """
    Uses cloudpickle to serialize contents (otherwise multiprocessing tries to use pickle)
    """
    def __init__(self, x):
        self.x = x

    def __getstate__(self):
        import cloudpickle
        return cloudpickle.dumps(self.x)

    def __setstate__(self, ob):
        import pickle
        self.x = pickle.loads(ob)
