import torch as th
from torch.distributions import Categorical
from .epsilon_schedules import DecayThenFlatSchedule

REGISTRY = {}


class MultinomialActionSelector():

    def __init__(self, args):
        self.args = args

        self.schedule = DecayThenFlatSchedule(args.epsilon_start, args.epsilon_finish, args.epsilon_anneal_time,
                                              decay="linear")##计算epsilon(start,finish,time length)
        self.epsilon = self.schedule.eval(0)
        self.test_greedy = getattr(args, "test_greedy", True)#args对象的属性 "test_greedy"或者默认值 True

    def select_action(self, agent_inputs, avail_actions, t_env, test_mode=False):
        masked_policies = agent_inputs.clone()
        masked_policies[avail_actions == 0.0] = 0.0

        self.epsilon = self.schedule.eval(t_env)

        if test_mode and self.test_greedy:
            picked_actions = masked_policies.max(dim=2)[1]#贪心
        else:
            picked_actions = Categorical(masked_policies).sample().long()#概率

        return picked_actions


REGISTRY["multinomial"] = MultinomialActionSelector


class EpsilonGreedyActionSelector():

    def __init__(self, args):
        self.args = args

        self.schedule = DecayThenFlatSchedule(args.epsilon_start, args.epsilon_finish, args.epsilon_anneal_time,
                                              decay="linear")
        self.epsilon = self.schedule.eval(0)

    def select_action(self, agent_inputs, avail_actions, t_env, test_mode=False):

        # Assuming agent_inputs is a batch of Q-Values for each agent bav
        self.epsilon = self.schedule.eval(t_env)

        if test_mode:
            # Greedy action selection only
            self.epsilon = 0.0

        # mask actions that are excluded from selection
        masked_q_values = agent_inputs.clone()  # 复制一份 Q 值张量，避免直接修改原始输出
        masked_q_values[avail_actions == 0.0] = -float("inf")  # 将不可用动作的 Q 值设为 -∞，保证不会被选中

        random_numbers = th.rand_like(agent_inputs[:, :, 0])  # 生成与 [batch, agents] 同形状的随机数（0~1）
        pick_random = (random_numbers < self.epsilon).long()  # 以 ε 概率置1表示随机选，否则为0表示贪心选
        random_actions = Categorical(avail_actions.float()).sample().long()  # 在可用动作中按均匀分布随机采样一个动作

        picked_actions = pick_random * random_actions + (1 - pick_random) * masked_q_values.max(dim=2)[1]  # ε选随机，否则选Q值最大的动作
        
        return picked_actions  # 返回最终选择的动作索引（形状：[batch, agents]）


REGISTRY["epsilon_greedy"] = EpsilonGreedyActionSelector
