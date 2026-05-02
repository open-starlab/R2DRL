from __future__ import annotations

import os
from time import time
import torch
import copy
import time

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
        return {}
    
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

        print("self.config.use_custom_start", self.config.use_custom_start)

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

    def get_avail_actions(self):
        if self.done and self.last_avail_actions is not None:
            return self.last_avail_actions

        self.last_avail_actions = self.agents.get_team1_avail_actions()
        return self.last_avail_actions

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

        if self._need_restart or (not self.runtime.has_live_procs()):
            self._need_restart = False
            self.restart()
            print("restart!!")

        self.last_state = None
        self.last_obs = None
        self.last_avail_actions = None
        self.done = 0
        self.episode_steps = 0
        self.score = [0, 0]

        goal = self.agents.coach.goal()
        cycle = self.agents.coach.cycle()
        if not self.agents.wait_all_ready():
            raise P.common.ShmProtocolError("Not READY Before Reset!!")
        # print(f"[RESET] turn={self.turn_count}, score=[{self.score[0]},{self.score[1]}], cycle={cycle}")
        
        if self.config.use_custom_start:
            self.agents.reset_custom()
        else:
            if self.turn_count > 1 and int(goal) == 0:
                self.agents.reset_default()

        self.agents.coach.clear_goal_flag()

        self.agents.set_agent_mask()
        self.agents.reset_episode_epv()

        return {
            "turn_count": self.turn_count,
            "score_left": self.score[0],
            "score_right": self.score[1],
            "max_episode_epv": float(self.agents.max_episode_epv),
        }
    
    def step(self, actions):
        self.episode_steps += 1

        if isinstance(actions, torch.Tensor):
            actions = actions.detach().cpu().numpy()

        self.agents.trainer.noop()
        self.agents.write_actions(actions)
        self._need_restart = not self.agents.wait_all_ready()

        self.agents.set_agent_mask()
        self.agents.update_episode_epv()

        goal = self.agents.coach.goal()
        reward = 0.0
        self.done = 0
        goal_for = 0
        goal_against = 0
        possession_loss = 0
        success = 0
        terminal_reason = "running"

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

        if (not self.done) and self.config.terminate_on_possession_loss:
            if self.agents.ball_owner_team == 2:
                self.done = 1
                possession_loss = 1
                reward += float(self.config.reward_on_possession_loss)
                terminal_reason = "possession_loss"

        if timeout or self._need_restart:
            self.done = 1
            terminal_reason = "restart" if self._need_restart else "timeout"
            if timeout and self.config.useMaxEpv:
                reward += float(self.agents.max_episode_epv)

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
            "possession_loss": possession_loss,
            "max_episode_epv": float(self.agents.max_episode_epv),
            "useMaxEpv": int(self.config.useMaxEpv),
        }
        return float(reward), bool(self.done), info

    def get_obs(self):
        if self.done and self.last_obs is not None:
            return self.last_obs

        self.last_obs = self.agents.get_team1_obs(norm=True, zero_inactive=True)
        return self.last_obs

    def get_state(self):
        if self.done and self.last_state is not None:
            return self.last_state

        state = self.agents.state(norm=True)
        self.last_state = state.copy()
        return self.last_state

    def close(self):
        if self._closed:
            return
        self._closed = True
        self.runtime.close()
        self.agents.close()

    def get_env_info(self):
        return {
            "n_agents": int(self.config.n1),
            "n_actions": int(self.agents.n_actions),
            "state_shape": int(P.coach.COACH_STATE_FLOAT),
            "obs_shape": int(P.player.STATE_NUM),
            "episode_limit": int(self.config.episode_limit),
        }

    def get_stats(self):
        return {}
