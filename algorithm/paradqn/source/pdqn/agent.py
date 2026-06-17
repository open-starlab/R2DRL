import copy
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import numpy as np
from typing import Tuple
try:
    from gymnasium import spaces
except ModuleNotFoundError:
    from gym import spaces
from .networks import QNetwork, ParamNetwork
from .replay_buffer import Batch

class ParaDQNAgent:
    """ ParaDQN agent implementation.
    """

    def __init__(self,
                 observation_space: spaces.Box,
                 action_space: spaces.Tuple,
                 device: str,
                 gamma: float,
                 lr_q: float,
                 lr_actor: float,
                 tau_q: float,
                 tau_actor: float,
                 double_q: bool = False,
                 q_loss_type: str = "mse",
                 grad_clip_norm: float = 0.0,
                 policy_delay: int = 1,
                 actor_behavior_l2_coef: float = 0.0):
        """ Initialize ParaDQNAgent.
        
        Inputs:
            - observation_space: observation space of the environment
            - action_space: action space of the environment (discrete + continuous)
            - device: device to run the networks on
            - lr_q: learning rate for Q network
            - lr_actor: learning rate for actor/param network
            - gamma: discount factor
            - tau: soft update factor for target networks
        """
        
        self.observation_space = observation_space
        self.action_space = action_space
        self.obs_dim = observation_space.shape[0]
        self.actions_num = action_space[0].n
        self.param_dim = action_space[1].shape[0]
        self.param_space = action_space[1]
        self.device = torch.device(device)

        self.q_net = QNetwork(self.obs_dim, self.actions_num, self.param_dim).to(self.device)
        self.q_target = copy.deepcopy(self.q_net).to(self.device)
        self.actor = ParamNetwork(self.obs_dim, self.param_space).to(self.device)
        self.actor_target = copy.deepcopy(self.actor).to(self.device)

        self.q_optimizer = optim.Adam(self.q_net.parameters(), lr=lr_q)
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=lr_actor)

        self.gamma = gamma
        self.tau_q = tau_q
        self.tau_actor = tau_actor
        self.double_q = bool(double_q)
        self.q_loss_type = str(q_loss_type or "mse").lower()
        self.grad_clip_norm = float(grad_clip_norm or 0.0)
        self.policy_delay = max(1, int(policy_delay or 1))
        self.actor_behavior_l2_coef = float(actor_behavior_l2_coef or 0.0)
        self.train_updates = 0

        # Hybrid actions 3/4/5 ignore continuous parameters in the C++ player.
        self.non_param_action_indices = tuple(i for i in (3, 4, 5) if i < self.actions_num)
        self.fallback_action_indices = tuple(i for i in (4, 5) if i < self.actions_num)

    @torch.no_grad()
    def select_action(
        self,
        state: np.ndarray,
        action_mask: np.ndarray,
        epsilon: float = 0.0,
        use_target: bool = False,
    ) -> Tuple[int, np.ndarray]:
        """ Return discrete action index and continuous parameter vector for that action.

        Inputs:
            - state: (state_dim,) current state
            - action_mask: (actions_num,) boolean mask of valid actions
            - epsilon: float, probability of choosing a random action for epsilon-greedy exploration
        Returns: 
            - action_idx: int
            - action_param: np.ndarray, parameter vector for that action
        """
        action_nums, action_params = self.select_actions(
            np.asarray(state, dtype=np.float32).reshape(1, -1),
            np.asarray(action_mask).reshape(1, -1),
            epsilon=epsilon,
            use_target=use_target,
        )
        return int(action_nums[0]), action_params[0]

    def _sample_random_action_params(self, count: int) -> np.ndarray:
        count = int(count)
        if count <= 0:
            return np.empty((0, self.param_dim), dtype=np.float32)

        low = np.asarray(self.param_space.low, dtype=np.float32).reshape(1, -1)
        high = np.asarray(self.param_space.high, dtype=np.float32).reshape(1, -1)
        if (
            low.shape[1] == self.param_dim
            and high.shape[1] == self.param_dim
            and np.isfinite(low).all()
            and np.isfinite(high).all()
        ):
            samples = low + np.random.random((count, self.param_dim)).astype(np.float32) * (high - low)
            return samples.astype(np.float32, copy=False)

        return np.asarray([self.action_space[1].sample() for _ in range(count)], dtype=np.float32)

    @staticmethod
    def _sample_random_discrete_from_masks(action_masks: np.ndarray) -> np.ndarray:
        masks = np.asarray(action_masks, dtype=bool)
        if masks.ndim != 2:
            masks = masks.reshape(masks.shape[0], -1)

        masks = masks.copy()
        empty_rows = masks.sum(axis=1) <= 0
        if np.any(empty_rows):
            masks[empty_rows, :] = True

        counts = masks.sum(axis=1).astype(np.int64)
        draws = (np.random.random(masks.shape[0]) * counts).astype(np.int64)
        cdf = np.cumsum(masks, axis=1)
        return (cdf > draws[:, None]).argmax(axis=1).astype(np.int64)

    @torch.no_grad()
    def select_actions(
        self,
        states: np.ndarray,
        action_masks: np.ndarray,
        epsilon: float = 0.0,
        use_target: bool = False,
    ) -> Tuple[np.ndarray, np.ndarray]:
        states = np.asarray(states, dtype=np.float32)
        if states.ndim == 1:
            states = states.reshape(1, -1)
        action_masks = np.asarray(action_masks)
        if action_masks.ndim == 1:
            action_masks = action_masks.reshape(1, -1)

        n_rows = int(states.shape[0])
        if n_rows == 0:
            return (
                np.empty((0,), dtype=np.int64),
                np.empty((0, self.param_dim), dtype=np.float32),
            )

        action_nums = np.empty((n_rows,), dtype=np.int64)
        action_params = np.empty((n_rows, self.param_dim), dtype=np.float32)
        random_rows = np.random.random(n_rows) < float(epsilon)

        if np.any(random_rows):
            random_masks = action_masks[random_rows]
            action_nums[random_rows] = self._sample_random_discrete_from_masks(random_masks)
            action_params[random_rows] = self._sample_random_action_params(int(random_rows.sum()))

        greedy_rows = ~random_rows
        if np.any(greedy_rows):
            s = torch.as_tensor(states[greedy_rows], dtype=torch.float32, device=self.device)
            mask_np = np.asarray(action_masks[greedy_rows], dtype=bool)
            actor_net = self.actor_target if use_target else self.actor
            q_net = self.q_target if use_target else self.q_net
            params = actor_net(s)
            q_vals = q_net(s, params)
            mask = torch.as_tensor(mask_np, dtype=torch.bool, device=self.device)
            q_vals = q_vals.masked_fill(~mask, float('-inf'))
            invalid_rows = ~torch.isfinite(q_vals).any(dim=1)
            if invalid_rows.any():
                q_vals[invalid_rows] = 0.0
            action_nums[greedy_rows] = torch.argmax(q_vals, dim=1).detach().cpu().numpy().astype(np.int64)
            action_params[greedy_rows] = params.detach().cpu().numpy().astype(np.float32, copy=False)

        return action_nums, action_params

    def is_fallback_action(self, action_idx: int) -> bool:
        return int(action_idx) in self.fallback_action_indices

    def train_step(
        self,
        batch: Batch,
        q_loss_coef: float = 1.0,
        actor_loss_coef: float = 1.0,
        update_actor: bool = True,
    ):
        """ Perform one training step from a batch.

        Inputs:
            - batch: (batch_size, (states, a_idx, a_params, rewards, next_states, dones))
            - q_loss_coef: coefficient for Q loss
            - actor_loss_coef: coefficient for actor loss
        Returns:
            - info: dict with 'q_loss' and 'actor_loss' values
        """
        self.train_updates += 1
        batch_size = batch.observations.shape[0]
        num_agents = batch.observations.shape[1]
        bn = batch_size * num_agents
        observations = torch.tensor(batch.observations, dtype=torch.float32, device=self.device)  
        shared_observations = torch.tensor(batch.shared_observations, dtype=torch.float32, device=self.device)  
        action_masks = torch.tensor(batch.action_masks, dtype=torch.bool, device=self.device)
        action_nums = torch.tensor(batch.action_nums, dtype=torch.int64, device=self.device)  
        action_params = torch.tensor(batch.action_params, dtype=torch.float32, device=self.device)  
        rewards = torch.tensor(batch.rewards, dtype=torch.float32, device=self.device)  
        dones = torch.tensor(batch.dones, dtype=torch.float32, device=self.device) 
        next_observations = torch.tensor(batch.next_observations, dtype=torch.float32, device=self.device) 
        next_shared_observations = torch.tensor(batch.next_shared_observations, dtype=torch.float32, device=self.device)  
        next_action_masks = torch.tensor(batch.next_action_masks, dtype=torch.bool, device=self.device)
        n_steps = torch.tensor(batch.n_steps, dtype=torch.float32, device=self.device)

        observations = observations.view(bn, -1)
        shared_observations = shared_observations.view(bn, -1)
        action_masks = action_masks.view(bn, -1)
        action_nums = action_nums.view(bn, -1)
        # action_onehots = torch.nn.functional.one_hot(action_nums, num_classes=self.actions_num)
        action_params = action_params.view(bn, -1)
        rewards = rewards.view(bn, -1)
        dones = dones.view(bn, -1)
        next_observations = next_observations.view(bn, -1)
        next_shared_observations = next_shared_observations.view(bn, -1)
        next_action_masks = next_action_masks.view(bn, -1)
        n_steps = n_steps.view(bn, -1)

        with torch.no_grad():
            next_params_target = self.actor_target(next_observations)
            q_next_all = self.q_target(next_observations, next_params_target)
            q_next_all = q_next_all.masked_fill(~next_action_masks, float("-inf"))
            if self.double_q:
                next_params_online = self.actor(next_observations)
                q_next_online = self.q_net(next_observations, next_params_online)
                q_next_online = q_next_online.masked_fill(~next_action_masks, float("-inf"))
                next_actions = q_next_online.argmax(dim=1, keepdim=True)
                q_next_max = q_next_all.gather(1, next_actions)
            else:
                q_next_max, _ = q_next_all.max(dim=1, keepdim=True)
            q_next_max = torch.where(torch.isfinite(q_next_max), q_next_max, torch.zeros_like(q_next_max))
            discounts = torch.pow(torch.full_like(n_steps, float(self.gamma)), n_steps)
            q_target = rewards + discounts * (1.0 - dones) * q_next_max

        q_all = self.q_net(observations, action_params)
        q_pred = q_all.gather(1, action_nums)

        if self.q_loss_type in ("huber", "smooth_l1", "smoothl1"):
            q_loss = nn.SmoothL1Loss()(q_pred, q_target)
        else:
            q_loss = nn.MSELoss()(q_pred, q_target)

        self.q_optimizer.zero_grad()
        (q_loss * q_loss_coef).backward()
        if self.grad_clip_norm > 0.0:
            nn.utils.clip_grad_norm_(self.q_net.parameters(), self.grad_clip_norm)
        self.q_optimizer.step()

        should_update_actor = bool(update_actor) and actor_loss_coef > 0.0 and (self.train_updates % self.policy_delay) == 0
        actor_updated = False
        actor_loss = torch.zeros((), device=self.device)
        actor_behavior_l2 = torch.zeros((), device=self.device)
        if should_update_actor:
            self.actor_optimizer.zero_grad()
            actor_params = self.actor(observations)
            param_action_masks = action_masks.clone()
            for action_idx in self.non_param_action_indices:
                param_action_masks[:, action_idx] = False

            valid_param_rows = param_action_masks.any(dim=1)
            if valid_param_rows.any():
                q_for_actions = self.q_net(observations, actor_params)
                q_for_actions = q_for_actions.masked_fill(~param_action_masks, float("-inf"))
                q_val, _ = q_for_actions[valid_param_rows].max(dim=1)
                actor_loss = -(q_val.mean())
                if self.actor_behavior_l2_coef > 0.0:
                    action_nums_flat = action_nums.view(-1)
                    behavior_rows = valid_param_rows.clone()
                    for action_idx in self.non_param_action_indices:
                        behavior_rows = behavior_rows & (action_nums_flat != action_idx)
                    if behavior_rows.any():
                        actor_behavior_l2 = F.mse_loss(actor_params[behavior_rows], action_params[behavior_rows])
                        actor_loss = actor_loss + self.actor_behavior_l2_coef * actor_behavior_l2
                (actor_loss * actor_loss_coef).backward()
                if self.grad_clip_norm > 0.0:
                    nn.utils.clip_grad_norm_(self.actor.parameters(), self.grad_clip_norm)
                self.actor_optimizer.step()
                actor_updated = True

        self.soft_update(self.q_target, self.q_net, self.tau_q)
        if actor_updated:
            self.soft_update(self.actor_target, self.actor, self.tau_actor)

        return {
            'q_loss': q_loss.item(),
            'actor_loss': actor_loss.item(),
            'double_q': float(self.double_q),
            'mean_n_step': float(n_steps.mean().item()),
            'actor_updated': float(actor_updated),
            'actor_update_enabled': float(bool(update_actor)),
            'policy_delay': float(self.policy_delay),
            'actor_behavior_l2': float(actor_behavior_l2.item()),
        }

    def soft_update(self, target_net: nn.Module, net: nn.Module, tau: float):
        """ Soft update target network parameters.
        """
        for target_param, param in zip(target_net.parameters(), net.parameters()):
            target_param.data.mul_(1.0 - tau)
            target_param.data.add_(tau * param.data)
