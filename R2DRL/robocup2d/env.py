from __future__ import annotations

import os
from time import time
import torch
import copy
import time
from collections import deque
import numpy as np

from .protocols import P
from .logging_utils import get_env_logger
from .config import load_env_args, EnvConfig
from .runtime import Runtime
from .agents import Agents
from .start_sampler import ScenarioStartSampler
from .tb_logger import TBLogger

class ParallelR2DRL:
    def __init__(self, cfg="parallelr2drl.yaml", **env_args):
        self.env = Robocup2dEnv(cfg=cfg, **env_args)
        self.test_mode = False

    def set_test_mode(self, test_mode: bool):
        self.test_mode = bool(test_mode)
        return True
    
    def reset(self, *args, **kwargs):
        self.env.test_mode = bool(self.test_mode)
        
        info = self.env.reset(*args, **kwargs)

        return info

    def set_start_and_n(self,start, n_control):
        return self.env.set_start_and_n(start, n_control)

    def step(self, actions):
        reward, done, info = self.env.step(actions)
        return reward,  done, info
    
    def get_stats(self):
        return self.env.get_stats()
    
    def close(self):
        self.env.close()

    def get_obs(self):
        return self.env.get_obs()

    def get_state(self):
        return self.env.get_state()

    def get_avail_actions(self):
        return self.env.get_avail_actions()

    def get_env_info(self):
        return self.env.get_env_info()

    def get_stats(self):
        return self.env.get_stats()

    def __getattr__(self, name):
        return getattr(self.env, name)
    
class R2DRL:
    def __init__(self, cfg="r2drl.yaml", **env_args):
        self.env = Robocup2dEnv(cfg=cfg, **env_args)

        use_tb = self.env.config.tb
        tb_log_dir = self.env.config.tb_log_dir

        self.tb = TBLogger(
            log_dir=tb_log_dir,
            enabled=use_tb,
        )
        self.start_sampler = None
        if self.env.config.trajectory_path:
            self.start_sampler = ScenarioStartSampler(
                init_n=self.env.config.init_n,
                start_window_size=self.env.config.start_window_size,
                progress_bucket_count=self.env.config.progress_bucket_count,
                current_target_window_size=self.env.config.current_target_window_size,
                num_selected_trajectories=self.env.config.num_selected_trajectories,
                random_sample=self.env.config.random_sample,
                trajectory_path=self.env.config.trajectory_path,
                scenario_difficulty=self.env.config.scenario_difficulty,
                scenario_difficulty_buckets=self.env.config.scenario_difficulty_buckets,
            )

        self.global_episode = 0
        self.test_mode = False

    def set_test_mode(self, test_mode: bool):
        self.test_mode = bool(test_mode)
        self.env.test_mode = self.test_mode
        return True

    def reset(self, *args, **kwargs):
        self.env.test_mode = bool(self.test_mode)

        if self.start_sampler is not None and self.env.config.use_custom_start:
            start, n_control = self.start_sampler.sample_start_and_n()
            self.env.set_start_and_n(start, n_control)

        info = self.env.reset(*args, **kwargs)

        return info

    def step(self, actions):
        reward, done, info = self.env.step(actions)

        if done and not self.test_mode:
            self.global_episode += 1

        return reward, done, info

    def close(self):
        self.env.close()
        self.tb.close()

    def get_obs(self):
        return self.env.get_obs()

    def get_state(self):
        return self.env.get_state()

    def get_avail_actions(self):
        return self.env.get_avail_actions()

    def get_env_info(self):
        return self.env.get_env_info()

    def get_stats(self):
        return self.env.get_stats()

    def __getattr__(self, name):
        return getattr(self.env, name)

class Robocup2dEnv:
    def __init__(self, cfg="robocup.yaml", **env_args):
        self.log = get_env_logger("robocup_env")
        self.config = EnvConfig(load_env_args(cfg, env_args))
        self._normalize_reset_mode()

        self.child_env = os.environ.copy()
        base_ld = os.environ.get("LD_LIBRARY_PATH", "")

        if self.config.lib_paths:
            merged = ":".join(map(str, self.config.lib_paths))
            if base_ld:
                merged = merged + ":" + base_ld
            self.child_env["LD_LIBRARY_PATH"] = merged
        else:
            self.child_env["LD_LIBRARY_PATH"] = base_ld

        self.runtime = Runtime(self.config, self.log, self.child_env)
        self.runtime.initialize_session()

        self.agents = Agents(
            coach_shm_id=self.runtime.coach_shm_id,
            trainer_shm_id=self.runtime.trainer_shm_id,
            player_shm_ids=self.runtime.player_shm_ids,
            config=self.config,
            log=self.log,
        )

        self.runtime.start_procs()

        self.last_state = None
        self.last_obs = None
        self.last_avail_actions = None
        self.done = 0
        self._need_restart = False
        self.episode_steps = 0
        self.turn_count = 0
        self.score = [0, 0]
        self._closed = False
        self.episode_limit = self.config.episode_limit
        self.test_mode = False
        self._reset_start = None
        self._reset_mask_n = None
        self._last_terminal_reason = "init"
        self._restart_window_seconds = 300.0
        self._restart_warn_threshold = 3
        self._restart_backoff_cap_seconds = 30.0
        self._restart_events = deque()
        self._restart_total = 0
        self._consecutive_restart_count = 0

        self._warn_if_timeouts_are_too_large()

    def set_test_mode(self, test_mode: bool):
        self.test_mode = bool(test_mode)
        return True

    def set_start_and_n(self, start, n_control):
        self._reset_start = copy.deepcopy(start)
        self._reset_mask_n = int(n_control)
        self._apply_reset_start()
        return True

    def _apply_reset_start(self):
        if self._reset_start is None:
            return

        self.agents.set_mask_n(self._reset_mask_n)
        self.agents.configure_reset_start(
            ball=self._reset_start["ball"],
            left_players=self._reset_start["left_players"],
            right_players=self._reset_start["right_players"],
            body_angles=self._reset_start["body_angles"],
        )

    def _normalize_reset_mode(self) -> None:
        benchmark_mode = str(getattr(self.config, "benchmark_mode", "")).strip().lower()
        desired_use_custom_start = None

        if benchmark_mode == "scenario":
            desired_use_custom_start = True
        elif benchmark_mode == "full_match":
            desired_use_custom_start = False

        if desired_use_custom_start is None:
            return

        current_use_custom_start = bool(self.config.use_custom_start)
        if current_use_custom_start == desired_use_custom_start:
            return

        self.log.warning(
            "[reset_mode] override use_custom_start "
            f"benchmark_mode={benchmark_mode} "
            f"old={current_use_custom_start} new={desired_use_custom_start}"
        )
        self.config.use_custom_start = desired_use_custom_start

    def _warn_if_timeouts_are_too_large(self):
        large_timeout = 600.0
        if self.config.wait_ready_timeout > large_timeout:
            self.log.warning(
                "[timeout_config] wait_ready_timeout is very large: "
                f"{self.config.wait_ready_timeout:.1f}s. "
                "Recovery after a stuck reset may be delayed."
            )
        if self.config.playon_timeout > large_timeout:
            self.log.warning(
                "[timeout_config] playon_timeout is very large: "
                f"{self.config.playon_timeout:.1f}s. "
                "Recovery after an in-episode stall may be delayed."
            )

    def _wait_all_ready_with_timeout(self, timeout: float, *, context: str) -> bool:
        ok = self.agents.wait_all_ready(timeout=float(timeout), context=context)
        if (not ok) and self.log:
            self.log.warning(
                "[wait_all_ready] failed "
                f"context={context} timeout={float(timeout):.3f}s "
                f"run_id={self.runtime.run_id}"
            )
        return ok

    def _wait_trainer_ready_with_timeout(self, timeout_ms: float, *, context: str) -> bool:
        ok = self.agents.trainer.wait_ready(timeout_ms=int(timeout_ms))
        if (not ok) and self.log:
            self.log.warning(
                "[trainer_ready] failed "
                f"context={context} timeout_ms={float(timeout_ms):.1f} "
                f"run_id={self.runtime.run_id} "
                f"phase={self.agents.trainer.phase()} "
                f"req_seq={self.agents.trainer.req_seq()} "
                f"done_seq={self.agents.trainer.done_seq()}"
            )
        return ok

    def _is_ball_out_of_bounds(self, ball_x: float, ball_y: float) -> bool:
        # Treat side-line exits and goal-line exits outside the goal mouth as
        # ball-out. Real goals continue to use the explicit coach goal flag.
        if abs(float(ball_y)) > float(self.config.half_width):
            return True
        return (
            abs(float(ball_x)) > float(self.config.half_length)
            and abs(float(ball_y)) > float(self.config.goal_y)
        )

    def _prune_restart_events(self, now: float) -> None:
        cutoff = now - self._restart_window_seconds
        while self._restart_events and self._restart_events[0] < cutoff:
            self._restart_events.popleft()

    def _recent_restart_count(self) -> int:
        self._prune_restart_events(time.monotonic())
        return len(self._restart_events)

    def _record_restart_and_get_backoff(self, reason: str) -> tuple[int, float]:
        now = time.monotonic()
        self._prune_restart_events(now)
        self._restart_events.append(now)
        self._restart_total += 1
        self._consecutive_restart_count += 1

        recent = len(self._restart_events)
        if recent < self._restart_warn_threshold:
            self.log.info(
                "[restart] trigger "
                f"reason={reason} total={self._restart_total} "
                f"recent={recent}/{int(self._restart_window_seconds)}s "
                f"consecutive={self._consecutive_restart_count}"
            )
            return recent, 0.0

        exponent = min(recent - self._restart_warn_threshold, 5)
        backoff = min(self._restart_backoff_cap_seconds, float(2 ** exponent))
        self.log.warning(
            "[restart] storm_detected "
            f"reason={reason} total={self._restart_total} "
            f"recent={recent}/{int(self._restart_window_seconds)}s "
            f"consecutive={self._consecutive_restart_count} "
            f"backoff_s={backoff:.1f}"
        )
        return recent, backoff

    def _restart_with_backoff(self, reason: str) -> None:
        _, backoff = self._record_restart_and_get_backoff(reason)
        if backoff > 0.0:
            time.sleep(backoff)
        self.restart()

    def get_avail_actions(self):
        if self.last_avail_actions is not None:
            return self.last_avail_actions

        return self._cache_transition_payload()["avail_actions"]

    def _cache_transition_payload(self):
        state = np.array(self.agents.state(norm=True), copy=True)
        obs = np.array(self.agents.get_team1_obs(norm=True, zero_inactive=True), copy=True)
        avail_actions = np.array(self.agents.get_team1_avail_actions(), copy=True)

        self.last_state = state
        self.last_obs = obs
        self.last_avail_actions = avail_actions
        return {
            "state": self.last_state,
            "avail_actions": self.last_avail_actions,
            "obs": self.last_obs,
        }

    def _get_transition_payload(self):
        if (
            self.last_state is None
            or self.last_obs is None
            or self.last_avail_actions is None
        ):
            return self._cache_transition_payload()
        return {
            "state": self.last_state,
            "avail_actions": self.last_avail_actions,
            "obs": self.last_obs,
        }

    def restart(self):
        old_agents = self.agents

        self.runtime.restart_session()

        try:
            old_agents.close()
        except Exception:
            pass

        self.agents = Agents(
            coach_shm_id=self.runtime.coach_shm_id,
            trainer_shm_id=self.runtime.trainer_shm_id,
            player_shm_ids=self.runtime.player_shm_ids,
            config=self.config,
            log=self.log,
        )
        self._apply_reset_start()
        self.runtime.start_procs()

    def reset(self):
        self.turn_count += 1

        did_restart = self._need_restart or (not self.runtime.has_live_procs())
        prev_terminal_reason = self._last_terminal_reason
        wait_ready_timeout = float(self.config.wait_ready_timeout)
        trainer_ready_timeout_ms = float(self.config.trainer_ready_timeout_ms)
        max_attempts = max(1, int(self.config.reset_retries) + 1)
        restart_reason = prev_terminal_reason if self._need_restart else "dead_procs"

        for attempt in range(max_attempts):
            if did_restart:
                self._need_restart = False
                self._restart_with_backoff(restart_reason)
                did_restart = False

            self.last_state = None
            self.last_obs = None
            self.last_avail_actions = None
            self.done = 0
            self.episode_steps = 0
            self.score = [0, 0]

            use_default_goal_restart = (
                (not self.config.use_custom_start)
                and prev_terminal_reason in ("init", "goal_scored", "goal_conceded")
            )

            if use_default_goal_restart:
                if not self._wait_all_ready_with_timeout(
                    wait_ready_timeout,
                    context=f"reset_goal_restart_precheck turn={self.turn_count} attempt={attempt + 1}/{max_attempts}",
                ):
                    if attempt + 1 < max_attempts:
                        did_restart = True
                        restart_reason = f"reset_goal_restart_timeout_attempt_{attempt + 1}"
                        continue
                    raise P.common.ShmProtocolError("Not READY Before Goal Restart Reset!!")
            else:
                if not self._wait_trainer_ready_with_timeout(
                    trainer_ready_timeout_ms,
                    context=f"reset_trainer_precheck turn={self.turn_count} attempt={attempt + 1}/{max_attempts}",
                ):
                    if attempt + 1 < max_attempts:
                        did_restart = True
                        restart_reason = f"reset_trainer_precheck_timeout_attempt_{attempt + 1}"
                        continue
                    raise P.common.ShmProtocolError("Trainer not READY Before Reset!!")

                if self.config.use_custom_start:
                    ready = self.agents.reset_custom(wait_timeout=wait_ready_timeout)
                else:
                    ready = self.agents.reset_default(wait_timeout=wait_ready_timeout)
                if not ready:
                    if attempt + 1 < max_attempts:
                        did_restart = True
                        restart_reason = f"reset_reposition_timeout_attempt_{attempt + 1}"
                        continue
                    raise P.common.ShmProtocolError("Not READY After Reset Reposition!!")

            self._last_terminal_reason = "running"
            self._consecutive_restart_count = 0

            self.agents.coach.clear_goal_flag()

            self.agents.set_agent_mask()
            self.agents.reset_episode_epv()
            self._cache_transition_payload()

            return {
                "turn_count": self.turn_count,
                "score_left": self.score[0],
                "score_right": self.score[1],
                "max_episode_epv": float(self.agents.max_episode_epv),
                "restart_total": int(self._restart_total),
                "restart_recent": int(self._recent_restart_count()),
            }

        raise P.common.ShmProtocolError("Reset failed after all restart retries")
    
    def step(self, actions):
        self.episode_steps += 1

        if isinstance(actions, torch.Tensor):
            actions = actions.detach().cpu().numpy()

        self.agents.trainer.noop()
        self.agents.write_actions(actions)
        self._need_restart = not self._wait_all_ready_with_timeout(
            float(self.config.playon_timeout),
            context=f"step turn={self.turn_count} episode_step={self.episode_steps}",
        )

        self.agents.set_agent_mask()
        epv_reward = float(self.agents.update_episode_epv())

        goal = self.agents.coach.goal()
        ball = self.agents.coach.ball(copy=False)
        ball_x = float(ball[0])
        ball_y = float(ball[1])
        ball_out_of_bounds = self._is_ball_out_of_bounds(ball_x, ball_y)
        reward = epv_reward
        self.done = 0
        goal_for = 0
        goal_against = 0
        ball_out = 0
        possession_loss = 0
        success = 0
        terminal_reason = "running"
        prev_score_left = int(self.score[0])
        prev_score_right = int(self.score[1])

        if goal == 1:
            reward += 1.0
            goal_for = 1
            success = 1
            self.score[0] += 1
            self.agents.coach.clear_goal_flag()
            if self.config.terminate_on_goal:
                self.done = 1
                terminal_reason = "goal_scored"
        elif goal == -1:
            reward -= 1.0
            goal_against = 1
            self.score[1] += 1
            self.agents.coach.clear_goal_flag()
            if self.config.terminate_on_goal:
                self.done = 1
                terminal_reason = "goal_conceded"

        timeout = (self.episode_steps >= self.episode_limit)

        if (not self.done) and self.config.terminate_on_ball_out and ball_out_of_bounds:
            self.done = 1
            ball_out = 1
            reward += float(self.config.reward_on_ball_out)
            terminal_reason = "ball_out"

        if (not self.done) and self.config.terminate_on_possession_loss:
            is_3v3 = (int(self.config.n1) == 3 and int(self.config.n2) == 3)
            opponent_has_possession = (
                self.agents.opponent_true_kickable
                if is_3v3
                else self.agents.ball_owner_team == 2
            )
            if opponent_has_possession:
                self.done = 1
                possession_loss = 1
                reward += float(self.config.reward_on_possession_loss)
                terminal_reason = "possession_loss"

        if timeout or self._need_restart:
            self.done = 1
            terminal_reason = "restart" if self._need_restart else "timeout"

        score_diff = int(self.score[0] - self.score[1])
        win = int(self.done and score_diff > 0)
        lose = int(self.done and score_diff < 0)

        info = {
            "win": win,
            "lose": lose,
            "timeout": int(timeout),
            "episode_limit": int(timeout),
            "goal_for": goal_for,
            "goal_against": goal_against,
            "score_left": int(self.score[0]),
            "score_right": int(self.score[1]),
            "score_diff": score_diff,
            "success": success,
            "ball_out": ball_out,
            "possession_loss": possession_loss,
            "max_episode_epv": float(self.agents.max_episode_epv),
            "useMaxEpv": int(self.config.useMaxEpv),
            "terminal_reason": terminal_reason,
            "restart_total": int(self._restart_total),
            "restart_recent": int(self._recent_restart_count()),
            "restart_consecutive": int(self._consecutive_restart_count),
        }
        self._last_terminal_reason = terminal_reason
        self._cache_transition_payload()
        return float(reward), bool(self.done), info

    def get_obs(self):
        if self.last_obs is not None:
            return self.last_obs

        return self._cache_transition_payload()["obs"]

    def get_state(self):
        if self.last_state is not None:
            return self.last_state

        return self._cache_transition_payload()["state"]

    def close(self):
        if self._closed:
            return
        self._closed = True
        close_error = None

        try:
            self.runtime.close()
        except Exception as exc:
            close_error = exc

        try:
            self.agents.close()
        except Exception as exc:
            if close_error is None:
                close_error = exc

        if close_error is not None:
            raise close_error

    def get_env_info(self):
        return {
            "n_agents": int(self.config.n1),
            "n_actions": int(self.agents.n_actions),
            "state_shape": int(P.coach.COACH_STATE_FLOAT),
            "obs_shape": int(P.player.STATE_NUM),
            "episode_limit": int(self.config.episode_limit),
        }

    def get_stats(self):
        return {
            "score_left": int(self.score[0]),
            "score_right": int(self.score[1]),
            "score_diff": int(self.score[0] - self.score[1]),
            "max_episode_epv": float(self.agents.max_episode_epv),
            "restart_total": int(self._restart_total),
            "restart_recent": int(self._recent_restart_count()),
            "restart_consecutive": int(self._consecutive_restart_count),
            "terminal_reason": self._last_terminal_reason,
        }
