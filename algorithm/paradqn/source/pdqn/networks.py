import math
import torch
import torch.nn as nn
from typing import List, Optional
try:
    from gymnasium import spaces
except ModuleNotFoundError:
    from gym import spaces


class QNetwork(nn.Module):
    """Q network for parametrized actions.
    Q(s, params), vector of Qs for each discrete action.

    Inputs:
      - state: (batch, state_dim)
      - action_param: (batch, param_dim), full concatenated parameter vector for all actions
    Output:
      - q: (batch, actions_num), Q value for each discrete action
    """

    def __init__(self, state_dim: int, actions_num: int, param_dim: int,
                 hidden_sizes: Optional[List[int]] = None):
        """ Initialize QNetwork.
        """
        super().__init__()
        if hidden_sizes is None:
            hidden_sizes = [256, 256]
        input_size = state_dim + param_dim

        layers = []
        last_size = input_size
        for hidden_size in hidden_sizes:
            layers.append(nn.Linear(last_size, hidden_size))
            layers.append(nn.ReLU())
            last_size = hidden_size

        self.net = nn.Sequential(*layers)
        self.q = nn.Linear(last_size, actions_num)
        self._initialize_weights()

    def _initialize_weights(self):
        """ Initialize Weights.
        """
        for m in self.net:
            if isinstance(m, nn.Linear):
                nn.init.kaiming_uniform_(m.weight, a=math.sqrt(5))
                if m.bias is not None:
                    nn.init.zeros_(m.bias)

        nn.init.uniform_(self.q.weight, -1e-3, 1e-3)
        if self.q.bias is not None:
            nn.init.zeros_(self.q.bias)

    def forward(self, state: torch.Tensor, action_param: torch.Tensor) -> torch.Tensor:
        """Return Q values for all discrete actions given state and full action-parameter vector.

        state: (batch, state_dim)
        action_param: (batch, param_dim)
        returns: (batch, actions_num)
        """
        x = torch.cat([state, action_param], dim=-1)
        x = self.net(x)
        x = self.q(x)
        return x


class ParamNetwork(nn.Module):
    """ Parameter Network for action parameters.

    Inputs:
        - state: (batch, state_dim)
    Output:
        - action_param: (batch, param_dim)
    """

    def __init__(self, state_dim: int, param_space: spaces.Box, 
                 hidden_sizes: Optional[List[int]] = None):
        """ Initialize ParamNetwork.
        """
        super().__init__()
        self.param_dim = param_space.shape[0]
        # self.param_low = torch.tensor(param_space.low, dtype=torch.float32)
        # self.param_high = torch.tensor(param_space.high, dtype=torch.float32)
        self.register_buffer('param_low', torch.tensor(param_space.low, dtype=torch.float32))
        self.register_buffer('param_high', torch.tensor(param_space.high, dtype=torch.float32))

        if hidden_sizes is None:
            hidden_sizes = [256, 256]
        
        last_size = state_dim
        layers = []
        for hidden_size in hidden_sizes:
            layers.append(nn.Linear(last_size, hidden_size))
            layers.append(nn.ReLU())
            last_size = hidden_size
        
        self.net = nn.Sequential(*layers)
        self.actor = nn.Linear(last_size, self.param_dim)
        self._initialize_weights()
        
    def _initialize_weights(self):
        """ Initialize Weights.
        """
        for m in self.net:
            if isinstance(m, nn.Linear):
                nn.init.kaiming_uniform_(m.weight, a=math.sqrt(5))
                if m.bias is not None:
                    nn.init.zeros_(m.bias)
        nn.init.uniform_(self.actor.weight, -3e-3, 3e-3)
        if self.actor.bias is not None:
            nn.init.zeros_(self.actor.bias)

    def forward(self, state: torch.Tensor) -> torch.Tensor:
        x = self.net(state)
        x = self.actor(x)
        x = torch.sigmoid(x)
        x = self.param_low + x * (self.param_high - self.param_low)
        return x
