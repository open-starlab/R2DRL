import numpy as np
from collections import namedtuple
try:
    from gymnasium import spaces
except ModuleNotFoundError:
    from gym import spaces
from typing import Tuple

Transition = namedtuple(
    "Transition",
    (
        "obs",
        "shared_obs",
        "action_mask",
        "act_num",
        "act_param",
        "reward",
        "done",
        "next_obs",
        "next_shared_obs",
        "next_action_mask",
        "n_step",
        "priority",
    ),
    defaults=(1, None),
)
Batch = namedtuple(
    "Batch",
    (
        "observations",
        "shared_observations",
        "action_masks",
        "action_nums",
        "action_params",
        "rewards",
        "dones",
        "next_observations",
        "next_shared_observations",
        "next_action_masks",
        "n_steps",
        "priorities",
    ),
)


class ReplayBuffer:
    """Replay buffer for parametrized actions."""

    def __init__(
        self,
        capacity: int,
        num_agents: int,
        observation_space: spaces.Box,
        shared_observation_space: spaces.Box,
        action_space: Tuple[spaces.Discrete, spaces.Box],
        priority_alpha: float = 0.0,
        reward_priority_scale: float = 0.0,
        positive_reward_priority_bonus: float = 0.0,
        terminal_reward_priority_bonus: float = 0.0,
    ):
        self.capacity = int(capacity)
        self.ptr = 0
        self.size = 0

        self.observations = np.zeros((self.capacity, num_agents, *observation_space.shape), dtype=np.float32)
        self.shared_observations = np.zeros((self.capacity, *shared_observation_space.shape), dtype=np.float32)
        self.action_masks = np.zeros((self.capacity, num_agents, action_space[0].n), dtype=np.int8)
        self.action_nums = np.zeros((self.capacity, num_agents), dtype=np.int32)
        self.action_params = np.zeros((self.capacity, num_agents, *action_space[1].shape), dtype=np.float32)
        self.rewards = np.zeros((self.capacity, num_agents), dtype=np.float32)
        self.dones = np.zeros((self.capacity, num_agents), dtype=np.float32)
        self.next_observations = np.zeros((self.capacity, num_agents, *observation_space.shape), dtype=np.float32)
        self.next_shared_observations = np.zeros((self.capacity, *shared_observation_space.shape), dtype=np.float32)
        self.next_action_masks = np.zeros((self.capacity, num_agents, action_space[0].n), dtype=np.int8)
        self.n_steps = np.ones((self.capacity, num_agents), dtype=np.float32)
        self.priorities = np.ones((self.capacity,), dtype=np.float32)
        self.priority_alpha = float(priority_alpha)
        self.reward_priority_scale = float(reward_priority_scale)
        self.positive_reward_priority_bonus = float(positive_reward_priority_bonus)
        self.terminal_reward_priority_bonus = float(terminal_reward_priority_bonus)

    def _priority_from_transition(self, transition: Transition) -> float:
        if transition.priority is not None:
            return max(float(transition.priority), 1e-6)
        reward = np.asarray(transition.reward, dtype=np.float32)
        done = np.asarray(transition.done, dtype=np.float32)
        reward_abs = float(np.max(np.abs(reward))) if reward.size else 0.0
        positive_reward = bool(float(np.max(reward)) > 0.0) if reward.size else False
        terminal_positive = bool(float(np.max(done)) > 0.0) and positive_reward if done.size else False
        priority = 1.0 + self.reward_priority_scale * reward_abs
        if positive_reward:
            priority += self.positive_reward_priority_bonus
        if terminal_positive:
            priority += self.terminal_reward_priority_bonus
        return max(float(priority), 1e-6)

    def push(self, transition: Transition):
        self.observations[self.ptr] = np.array(transition.obs, copy=True, dtype=np.float32)
        self.shared_observations[self.ptr] = np.array(transition.shared_obs, copy=True, dtype=np.float32)
        self.action_masks[self.ptr] = np.array(transition.action_mask, copy=True, dtype=np.int8)
        self.action_nums[self.ptr] = np.array(transition.act_num, copy=True, dtype=np.int32)
        self.action_params[self.ptr] = np.array(transition.act_param, copy=True, dtype=np.float32)
        self.rewards[self.ptr] = np.array(transition.reward, copy=True, dtype=np.float32)
        self.dones[self.ptr] = np.array(transition.done, copy=True, dtype=np.float32)
        self.next_observations[self.ptr] = np.array(transition.next_obs, copy=True, dtype=np.float32)
        self.next_shared_observations[self.ptr] = np.array(transition.next_shared_obs, copy=True, dtype=np.float32)
        self.next_action_masks[self.ptr] = np.array(transition.next_action_mask, copy=True, dtype=np.int8)
        self.n_steps[self.ptr] = np.array(transition.n_step, copy=True, dtype=np.float32)
        self.priorities[self.ptr] = self._priority_from_transition(transition)
        self.ptr = (self.ptr + 1) % self.capacity
        self.size = min(self.size + 1, self.capacity)

    def sample(self, batch_size: int):
        if self.priority_alpha > 0.0:
            weights = np.power(self.priorities[: self.size], self.priority_alpha, dtype=np.float64)
            total = float(weights.sum())
            if np.isfinite(total) and total > 0.0:
                probs = weights / total
                indices = np.random.choice(self.size, size=batch_size, replace=False, p=probs)
            else:
                indices = np.random.choice(self.size, size=batch_size, replace=False)
        else:
            indices = np.random.choice(self.size, size=batch_size, replace=False)
        batch = Batch(
            observations=self.observations[indices],
            shared_observations=self.shared_observations[indices],
            action_masks=self.action_masks[indices],
            action_nums=self.action_nums[indices],
            action_params=self.action_params[indices],
            rewards=self.rewards[indices],
            dones=self.dones[indices],
            next_observations=self.next_observations[indices],
            next_shared_observations=self.next_shared_observations[indices],
            next_action_masks=self.next_action_masks[indices],
            n_steps=self.n_steps[indices],
            priorities=self.priorities[indices],
        )
        return batch

    def can_sample(self, batch_size: int) -> bool:
        return self.size >= batch_size

    def __len__(self):
        return self.size
