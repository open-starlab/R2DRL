import gym
import numpy as np
from typing import Tuple
import gym_hybrid


class MovingEnv:
    def __init__(self):
        self.env = gym.make("Moving-v0")
        self.state_dim = 10
        self.num_actions = 3
        self.param_dim_list = [1, 1, 0]
        self.param_dim_total = 2
        self.action_space = self.env.action_space
        self.observation_space = self.env.observation_space

    def reset(self):
        s = self.env.reset()
        s = np.array(s, dtype=np.float32)
        return s

    def step(self, action_tuple: Tuple[int, np.ndarray]):
        s, r, done, info = self.env.step(action_tuple)
        s = np.array(s, dtype=np.float32)
        r = float(r)
        return s, r, done, info
