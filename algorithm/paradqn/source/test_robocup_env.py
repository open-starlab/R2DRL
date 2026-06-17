import os
import time
import glob
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from environments.robocup2d.robocup import Robocup2d_Python


class RNNPolicy(nn.Module):
    def __init__(self, obs_dim: int, n_actions: int, hidden_dim: int = 64):
        super().__init__()
        self.n_actions = n_actions  # 4: Hybrid, 17: Base
        self.gru = nn.GRU(input_size=obs_dim, hidden_size=hidden_dim, batch_first=True)
        self.fc_act = nn.Linear(hidden_dim, n_actions)  # 离散动作头
        self.fc_u = nn.Linear(hidden_dim, 2)  # 连续参数头 (u0,u1)
        self.hidden = None

    def reset_hidden(self, n_agents: int = 1, device: torch.device = torch.device("cpu")):
        self.hidden = torch.zeros(1, n_agents, self.gru.hidden_size, device=device)

    @torch.no_grad()
    def forward(self, obs: torch.Tensor, avail_mask: torch.Tensor):
        """
        obs:        [N, obs_dim]
        avail_mask: [N, A]  (Base: A=17; Hybrid: A=4 或 17，需要切前4位)
        return:
          - Base:   np.int32  shape [N]
          - Hybrid: np.float32 shape [N,3]  每行 = [a, u0, u1]
        """
        device = obs.device
        N, D = obs.shape
        x = obs.unsqueeze(1)  # [N,1,D]
        out, self.hidden = self.gru(x, self.hidden)  # out:[N,1,H]
        h = out.squeeze(1)  # [N,H]

        # 动作分支
        logits = self.fc_act(h)  # [N,A]
        A = logits.shape[-1]

        # 如果 avail_mask 来的是 17 维，而我们只需要 Hybrid 的 4 维，切前4位
        if A == 4 and avail_mask.shape[-1] >= 4:
            avail_use = avail_mask[:, :4].clone()
        else:
            avail_use = avail_mask.clone()

        # 屏蔽无效动作
        logits[avail_use == 0] = -1e9

        probs = F.softmax(logits, dim=-1)  # [N,A]
        # 采样（也可改为 argmax）
        a = torch.multinomial(probs, 1).squeeze(1)  # [N]

        # 连续参数分支：用 sigmoid 输出到 [0,1]
        u = torch.sigmoid(self.fc_u(h))  # [N,2]
        u0 = u[:, 0]
        u1 = u[:, 1]

        if A == 4:
            # Hybrid: 拼成 [a,u0,u1]
            out_np = torch.stack([a.float(), u0, u1], dim=1).cpu().numpy().astype(np.float32)
            # C++ 端会再次 clamp，但我们这边也做个保险：
            out_np[:, 1:] = np.clip(out_np[:, 1:], 0.0, 1.0)
            return out_np
        else:
            # Base: 只要离散动作
            return a.cpu().numpy().astype(np.int32)


def main():
    env = Robocup2d_Python(n1=11, n2=11, episode_limit=6000)
    info = env.get_env_info()
    OBS_DIM = info["obs_shape"]
    N_ACTIONS = info["n_actions"]  # Base:17  Hybrid:4

    rnn_policy = RNNPolicy(obs_dim=OBS_DIM, n_actions=N_ACTIONS, hidden_dim=64)

    try:
        env.reset()
        t = 0
        while True:
            # --- 用 shm_refs 动态获取当前 agent 数 ---
            keys = sorted(env.shm_refs)
            n = len(keys)
            if n == 0:
                # 还没连上/还没到 PlayOn，等一等
                time.sleep(0.01)
                continue

            # 第一步或 agent 数变化时，重置 RNN 隐状态
            if t == 0 or (hasattr(rnn_policy, "hidden") and rnn_policy.hidden is not None and rnn_policy.hidden.size(1) != n):
                rnn_policy.reset_hidden(n_agents=n)

            # 读观测 & 掩码
            obs = env.get_obs()  # [n, OBS_DIM]
            avail_actions = env.get_avail_actions()  # [n, 17] 或 [n, 4]
            obs_t = torch.tensor(obs, dtype=torch.float32)
            avail_t = torch.tensor(avail_actions, dtype=torch.float32)

            # 让策略出动作
            actions = rnn_policy.forward(obs_t, avail_t)
            # Hybrid: actions 形状 [n,3]（a,u0,u1）
            # Base:   actions 形状 [n]   （int32）

            # 防御性断言，避免再次传空
            if N_ACTIONS == 4:
                assert isinstance(actions, np.ndarray) and actions.shape == (n, 3), f"Hybrid 期望 {(n, 3)}，得到 {None if not isinstance(actions, np.ndarray) else actions.shape}"
            else:
                assert isinstance(actions, np.ndarray) and actions.shape == (n,), f"Base 期望 {(n,)}，得到 {None if not isinstance(actions, np.ndarray) else actions.shape}"

            # print(f"actions={actions}")

            # 真正执行一步
            reward, terminated, env_info = env.step(actions)
            # print(f"reward={reward}, terminated={terminated}, env_info={env_info}")

            if terminated:
                print(f"✅ Episode finished at step={t}")
                break

            t += 1
            # time.sleep(0.05)
    finally:
        env.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("⚠️ 测试被用户中断.")
    for file in glob.glob("*.rcg"):
        os.remove(file)
    for file in glob.glob("*.rcl"):
        os.remove(file)
    print("✅ 测试结束，清理完毕.")
