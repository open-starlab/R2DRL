from envs import REGISTRY as env_REGISTRY
from functools import partial
from components.episode_buffer import EpisodeBatch
import numpy as np
import time

class EpisodeRunner:

    def __init__(self, args, logger):
        self.args = args
        self.logger = logger
        self.batch_size = self.args.batch_size_run
        assert self.batch_size == 1

        self.env = env_REGISTRY[self.args.env](**self.args.env_args)
        self.episode_limit = self.env.episode_limit
        self.t = 0

        self.t_env = 0

        self.train_returns = []
        self.test_returns = []
        self.train_stats = {}
        self.test_stats = {}

        # Log the first run
        self.log_train_stats_t = -1000000
        self.train_ep_idx = 0


    def setup(self, scheme, groups, preprocess, mac):
        #PyMARL 里的一个类，用来存储整个 episode 的数
        self.new_batch = partial(EpisodeBatch, scheme, groups, self.batch_size, self.episode_limit + 1,
                                 preprocess=preprocess, device=self.args.device)
        self.mac = mac #多智能体控制器

    def get_env_info(self):
        #信息
        return self.env.get_env_info()

    def save_replay(self):
        #保存
        self.env.save_replay()

    def close_env(self):
        #关闭
        self.env.close()

    def _safe_win_lose_ratio(self, stats):
        win_count = float(stats.get("win", 0))
        lose_count = float(stats.get("lose", 0))
        if win_count <= 0 or lose_count <= 0:
            return 0.0
        return win_count / lose_count

    def reset(self):
        #重启
        self.batch = self.new_batch()
        self.env.reset()
        self.t = 0

    def run(self, test_mode=False):
        # 收集数据
        self.env.test_mode = test_mode
        self.reset()

        terminated = False
        episode_return = 0
        self.mac.init_hidden(batch_size=self.batch_size)

        while (not terminated) and (self.t < self.episode_limit):

            # 1) 读环境状态

            state = self.env.get_state()
            avail_actions = self.env.get_avail_actions()
            obs = self.env.get_obs()

            # 打印一下形状/长度
            try:
                state_shape = (len(state),) if not hasattr(state, "shape") else state.shape
            except Exception:
                state_shape = "unknown"
            try:
                obs_shape = (len(obs),) if not hasattr(obs, "shape") else obs.shape
            except Exception:
                obs_shape = "unknown"

            pre_transition_data = {
                "state": [state],
                "avail_actions": [avail_actions],
                "obs": [obs],
            }  # 读取环境状态
            

            self.batch.update(pre_transition_data, ts=self.t)  # 保存 state, action mask, obs


            # 2) 选动作
            actions = self.mac.select_actions(
                self.batch, t_ep=self.t, t_env=self.t_env, test_mode=test_mode
            )  # 选择动作

            # 3) 环境 step
            reward, terminated, env_info = self.env.step(actions[0])  # 执行动作

            episode_return += reward  # 记录回报

            post_transition_data = {
                "actions": actions,
                "reward": [(reward,)],
                "terminated": [(terminated != env_info.get('episode_limit', False),)],
            }
            self.batch.update(post_transition_data, ts=self.t)  # 记录 terminated, actions, reward, time step

            self.t += 1
            
        # 保险起见，强制 clamp 一下
        if self.t > self.episode_limit:
            self.t = self.episode_limit
        print(f"[done],self.t={self.t},terminated={terminated}")
        
        # 最后一帧
        last_state = self.env.get_state()
        last_avail = self.env.get_avail_actions()
        last_obs = self.env.get_obs()
        last_data = {
            "state": [last_state],
            "avail_actions": [last_avail],
            "obs": [last_obs],
        }
        self.batch.update(last_data, ts=self.t)  # 保存最终 state, action mask, obs

        # 最后一步动作
        actions = self.mac.select_actions(self.batch, t_ep=self.t, t_env=self.t_env, test_mode=test_mode)
        self.batch.update({"actions": actions}, ts=self.t)  # 记录最终动作

        cur_stats = self.test_stats if test_mode else self.train_stats
        cur_returns = self.test_returns if test_mode else self.train_returns
        log_prefix = "test_" if test_mode else ""
        cur_stats.update({k: cur_stats.get(k, 0) + env_info.get(k, 0) for k in set(cur_stats) | set(env_info)})
        cur_stats["n_episodes"] = 1 + cur_stats.get("n_episodes", 0)
        cur_stats["ep_length"] = self.t + cur_stats.get("ep_length", 0)

        if not test_mode:
            self.t_env += self.t

        cur_returns.append(episode_return)

        if test_mode and (len(self.test_returns) == self.args.test_nepisode):
            self._log(cur_returns, cur_stats, log_prefix)
        elif self.t_env - self.log_train_stats_t >= self.args.runner_log_interval:
            self._log(cur_returns, cur_stats, log_prefix)
            if hasattr(self.mac.action_selector, "epsilon"):
                self.logger.log_stat("epsilon", self.mac.action_selector.epsilon, self.t_env)
            self.log_train_stats_t = self.t_env

        return self.batch

    def _log(self, returns, stats, prefix):
        self.logger.log_stat(prefix + "return_mean", np.mean(returns), self.t_env)
        self.logger.log_stat(prefix + "return_std", np.std(returns), self.t_env)
        if prefix == "test_":
            self.logger.log_stat(
                prefix + "win_lose_ratio",
                self._safe_win_lose_ratio(stats),
                self.t_env,
            )
        returns.clear()

        for k, v in stats.items():
            if k == "n_episodes":
                continue
            self.logger.log_stat(prefix + k + "_mean" , v/stats["n_episodes"], self.t_env)
        stats.clear()
