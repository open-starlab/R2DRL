from modules.agents import REGISTRY as agent_REGISTRY
from components.action_selectors import REGISTRY as action_REGISTRY
import torch as th


# This multi-agent controller shares parameters between agents
class BasicMAC:
    def __init__(self, scheme, groups, args):
        self.n_agents = args.n_agents
        self.args = args
        input_shape = self._get_input_shape(scheme)
        self._build_agents(input_shape)#RNN网络
        self.agent_output_type = args.agent_output_type

        self.action_selector = action_REGISTRY[args.action_selector](args)

        self.hidden_states = None

    def select_actions(self, ep_batch, t_ep, t_env, bs=slice(None), test_mode=False):
        # Only select actions for the selected batch elements in bs
        avail_actions = ep_batch["avail_actions"][:, t_ep] # 当前时间步每个智能体可用动作mask
        agent_outputs = self.forward(ep_batch, t_ep, test_mode=test_mode) # 前向网络得到策略或Q值
        chosen_actions = self.action_selector.select_action(agent_outputs[bs], avail_actions[bs], t_env, test_mode=test_mode)# 调用动作选择器
        return chosen_actions

    def forward(self, ep_batch, t, test_mode=False):
        # 从EpisodeBatch中构建当前时间步每个智能体的输入向量
        agent_inputs = self._build_inputs(ep_batch, t) # 拼接obs/上一步动作/agent ID
        # 当前时间步每个智能体的可用动作mask
        avail_actions = ep_batch["avail_actions"][:, t]
        # 前向传播智能体网络，得到每个agent的输出（logits或Q值），并更新RNN隐藏状态
        agent_outs, self.hidden_states = self.agent(agent_inputs, self.hidden_states)

        # Softmax the agent outputs if they're policy logits
        # 如果网络输出的是策略logits而不是Q值，则需要softmax成概率分布
        if self.agent_output_type == "pi_logits":
            # 将不可用动作对应的logits设为一个极小值(-1e10)，避免softmax后得到非零概率
            if getattr(self.args, "mask_before_softmax", True):
                # Make the logits for unavailable actions very negative to minimise their affect on the softmax
                reshaped_avail_actions = avail_actions.reshape(ep_batch.batch_size * self.n_agents, -1)
                agent_outs[reshaped_avail_actions == 0] = -1e10
            # 对每个agent的动作logits做softmax，得到动作概率分布
            agent_outs = th.nn.functional.softmax(agent_outs, dim=-1)
            if not test_mode:
                # Epsilon floor # 如果是训练模式，则对策略加epsilon平滑，保证一定探索性
                epsilon_action_num = agent_outs.size(-1) # 默认可选动作总数
                if getattr(self.args, "mask_before_softmax", True):
                    # With probability epsilon, we will pick an available action uniformly
                    epsilon_action_num = reshaped_avail_actions.sum(dim=1, keepdim=True).float()
                # 用epsilon混合策略概率和均匀分布：(1-ε)*策略+ε*均匀分布
                agent_outs = ((1 - self.action_selector.epsilon) * agent_outs
                               + th.ones_like(agent_outs) * self.action_selector.epsilon/epsilon_action_num)
                # 再次确保不可用动作概率为0
                if getattr(self.args, "mask_before_softmax", True):
                    # Zero out the unavailable actions
                    agent_outs[reshaped_avail_actions == 0] = 0.0

        return agent_outs.view(ep_batch.batch_size, self.n_agents, -1)

    def init_hidden(self, batch_size):
        self.hidden_states = self.agent.init_hidden().unsqueeze(0).expand(batch_size, self.n_agents, -1)  # bav

    def parameters(self):
        return self.agent.parameters()

    def load_state(self, other_mac):
        self.agent.load_state_dict(other_mac.agent.state_dict())

    def cuda(self):
        self.agent.cuda()

    def save_models(self, path):
        th.save(self.agent.state_dict(), "{}/agent.th".format(path))

    def load_models(self, path):
        self.agent.load_state_dict(th.load("{}/agent.th".format(path), map_location=lambda storage, loc: storage))

    def _build_agents(self, input_shape):
        self.agent = agent_REGISTRY[self.args.agent](input_shape, self.args)

    def _build_inputs(self, batch, t):
        # Assumes homogenous agents with flat observations.
        # Other MACs might want to e.g. delegate building inputs to each agent
        bs = batch.batch_size
        inputs = []
        inputs.append(batch["obs"][:, t])  # b1av
        if self.args.obs_last_action:
            if t == 0:
                inputs.append(th.zeros_like(batch["actions_onehot"][:, t]))
            else:
                inputs.append(batch["actions_onehot"][:, t-1])
        if self.args.obs_agent_id:
            inputs.append(th.eye(self.n_agents, device=batch.device).unsqueeze(0).expand(bs, -1, -1))

        inputs = th.cat([x.reshape(bs*self.n_agents, -1) for x in inputs], dim=1)
        return inputs

    def _get_input_shape(self, scheme):
        input_shape = scheme["obs"]["vshape"]
        if self.args.obs_last_action:
            input_shape += scheme["actions_onehot"]["vshape"][0]
        if self.args.obs_agent_id:
            input_shape += self.n_agents

        return input_shape
