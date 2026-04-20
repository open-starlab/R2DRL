from __future__ import annotations

import os
from time import time
import torch
import numpy as np
import copy
import time

from .protocols import P
from .logging_utils import get_env_logger
from .config import load_env_args, EnvConfig
from .runtime import Runtime
from .agents import Agents
from .curriculum import CurriculumController
from .tb_logger import TBLogger

class ParallelR2DRL:
    def __init__(self, cfg="r2drl.yaml", **env_args):
        self.env = Robocup2dEnv(cfg=cfg, **env_args)
        self.test_mode = False

    def set_test_mode(self, test_mode: bool):
        self.test_mode = bool(test_mode)
        return True
    
    def reset(self, *args, **kwargs):

        if self.test_mode:
            self.env.test_mode = True
        else:
            self.env.test_mode = False
        
        info = self.env.reset(*args, **kwargs)

        return info

    def set_start_and_n(self,start, n_control):
        self.env.agents.set_mask_n(n_control)
        self.env.agents.configure_reset_start(
            ball=start["ball"],
            left_players=start["left_players"],
            right_players=start["right_players"],
            body_angles=start["body_angles"],
        )
        return True

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
        self.controller = CurriculumController(
            init_n=self.env.config.init_n,
            start_window_size=self.env.config.start_window_size,
            return_window_size=self.env.config.return_window_size,
            progress_bucket_count=self.env.config.progress_bucket_count,
            current_target_window_size=self.env.config.current_target_window_size,
            window_move_win_rate_threshold=self.env.config.window_move_win_rate_threshold,
        )

        self.global_episode = 0
        self.tb_step = 0
        self.test_mode = False
        self.episode_key = None
        self.test_returns = []

    def reset(self, *args, **kwargs):
        self.flush_test_returns_to_tb()

        # 每次 reset 记一次 step
        self.tb_step += 1

        if self.test_mode:
            self.env.test_mode = True
            self.episode_key = None
        else:
            self.env.test_mode = False
            key = self.controller.generate_key()

            self.episode_key = key
            self.controller.apply_start_and_n_by_key(
                self.env,
                key=key
            )

        info = self.env.reset(*args, **kwargs)

        return info

    def step(self, actions):
        reward, done, info = self.env.step(actions)

        if done:
            if self.test_mode:
                self.test_returns.append(float(reward))
            else:
                key = self.episode_key
                if key is not None:
                    self.controller.update_key_stats(key, reward)

                # 每 5 个 reset / episode 再写一次 TB
                if self.tb_step % 5 == 0:
                    frontier_stats = self.controller.get_frontier_stats()
                    for name, value in frontier_stats.items():
                        self.tb.add_scalar(f"curriculum/{name}", value, self.global_episode)

                    self.tb.flush()

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

    def __getattr__(self, name):
        return getattr(self.env, name)

    def flush_test_returns_to_tb(self):
        if (not self.test_mode) and len(self.test_returns) > 0:
            mean_return = float(np.mean(self.test_returns))
            self.tb.add_scalar("test/return_mean", mean_return, self.global_episode)
            self.tb.flush()
            self.test_returns.clear()

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

        print("self.config.curriculum", self.config.curriculum)

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

        goal = self.agents.coach.goal()
        cycle = self.agents.coach.cycle()
        if not self.agents.wait_all_ready():
            raise P.common.ShmProtocolError("Not READY Before Reset!!")
        # print(f"[RESET] turn={self.turn_count}, score=[{self.score[0]},{self.score[1]}], cycle={cycle}")
        
        if self.config.curriculum:
            if self.test_mode:
                self.agents.reset_default()
            else:
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

        timeout = (self.episode_steps >= self.episode_limit)
        goal = self.agents.coach.goal()
        reward = 0.0
        self.done = 0
        win = 0
        lose = 0

        if timeout or self._need_restart:
            self.done = 1
            if timeout and self.config.rewardshaping:
                reward = float(self.agents.max_episode_epv)
        elif goal == 1:
            self.done = 1
            reward = 1.0
            win = 1
            self.score[0] += 1
        elif goal == -1:
            self.done = 1
            reward = -1.0
            lose = 1
            self.score[1] += 1

        info = {
            "win": win,
            "lose": lose,
            "timeout": int(timeout),
            "max_episode_epv": float(self.agents.max_episode_epv),
            "rewardshaping": int(timeout and self.config.rewardshaping),
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
