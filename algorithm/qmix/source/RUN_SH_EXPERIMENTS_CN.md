# `run.sh` 实验说明

这个文件是给第一次看 `pymarl/src/run.sh` 的人准备的。

如果你不熟悉 RoboCup、PyMARL、`env-config` 这些名字，也没关系。你可以先把 `run.sh` 里的每一行实验理解成一句话：

- 这一行是在跑 `11vs11` 还是 `3vs3`
- 是正常整场比赛，还是从某个预设场景开始
- 对手是弱、中、强
- 奖励里有没有加 `EPV`
- 我方动作空间用的是 `Base` 还是 `Hybrid`

---

## 1. `run.sh` 怎么用

`run.sh` 里的实验菜单默认只有一行真正执行，其他行都被 `#` 注释掉了。

使用方法：

1. 找到你想跑的那一行。
2. 去掉这一行前面的 `#`。
3. 保证其他实验行还是注释状态。
4. 提交作业或直接运行脚本。

一句话记忆：

- 同一时间通常只开一行实验。

---

## 2. 先看懂名字在说什么

以这条命令为例：

```bash
python -u main.py --config=qmix --env-config=parallelr2drl_3vs3scenario_base_init-3_start-hard_opp-lv3_epv-on --capture=no
```

可以拆成下面几部分：

- `3vs3scenario`
  - 表示 `3` 对 `3`
  - 不是从中场普通开球开始
  - 而是从预先采样好的场景起点开始
- `base`
  - 我方动作空间使用 `Base`
- `init-3`
  - 一开始允许控制的我方球员数是 `3`
- `start-hard`
  - 起始场景难度是 `hard`
- `opp-lv3`
  - 对手强度是 `lv3`
  - 这里对应强对手
- `epv-on`
  - 奖励里打开了 `EPV`

---

## 3. 两大类任务分别是什么

### `11vs11benchmark`

可以把它理解成：

- 标准 `11` 人对 `11` 人整场比赛
- 类似“正常踢一整场”
- 默认从普通比赛初始站位开始
- 更适合看整体战术、长期决策、整场表现

这类任务的典型特点：

- `n: 11`
- `benchmark_mode: full_match`
- `use_custom_start: false`
- `episode_limit: 3000`

外行版理解：

- 这是“完整比赛”实验。

### `3vs3scenario`

可以把它理解成：

- `3` 人对 `3` 人的小场景任务
- 不是从中场开球
- 而是从轨迹里采样出来的某个局部局面开始
- 更适合快速验证、比较开局难度、比较局部攻防能力

这类任务的典型特点：

- `n: 3`
- `benchmark_mode: scenario`
- `use_custom_start: true`
- `terminate_on_goal: true`
- `episode_limit: 300`

外行版理解：

- 这是“给你一个已经进行到一半的局面，看你能不能把这球打成”的实验。

---

## 4. `Base` 和 `Hybrid` 是什么

这两个词说的是“我方 agent 能怎么出动作”。

### `Base`

- 用离散动作空间
- 更像“从固定动作菜单里选一个”
- 更稳定，也更容易作为基线

外行版理解：

- 像按按钮选动作。

### `Hybrid`

- 动作里除了离散选择，还会带连续参数
- 表达能力更强
- 但学习难度通常也更高

外行版理解：

- 不只是“选射门”，还会进一步决定“往哪里、用多大力度”。

---

## 5. `EPV` 是什么

`EPV` 可以粗略理解成：

- 这个球现在所在的位置，未来有多大进攻价值

所以：

- `epv-off`
  - 奖励主要看进球、失球这类结果
- `epv-on`
  - 除了结果奖励，还会额外鼓励把球推进到更有威胁的位置

外行版理解：

- `epv-off` 更像“只看最后进没进球”
- `epv-on` 更像“过程里把球推进到危险区域也算做好事”

---

## 6. `lv1 / lv2 / lv3` 是什么

这是对手强度。

- `lv1`
  - 弱对手
- `lv2`
  - 中等对手
- `lv3`
  - 强对手

在当前这些配置里，对应大致是：

- `lv1 -> opponent_level: 0.1`
- `lv2 -> opponent_level: 0.5`
- `lv3 -> opponent_level: 1.0`

外行版理解：

- 就是简单、一般、困难。

---

## 7. `easy / medium / hard` 起始场景是什么意思

这个主要出现在 `3vs3scenario` 里。

- `start-easy`
  - 给你的开局通常更友好
- `start-medium`
  - 难度居中
- `start-hard`
  - 开局更难处理

注意：

- 这里说的是“开局局面难度”
- 不是“对手强度”
- 所以 `start-hard` 和 `opp-lv3` 是两件事

外行版理解：

- 一个说“这球一开始给你的局面难不难”
- 一个说“你面对的对手强不强”

---

## 8. `init-3` 是什么

这个主要出现在 `3vs3scenario` 里。

- `init-3`
  - 开局时我方 `3` 个球员都在控制范围内

外行版理解：

- 一上来就让你同时管 3 个人。

---

## 9. `run.sh` 里每组实验到底在比较什么

下面直接对应 `pymarl/src/run.sh` 里的实验菜单。

### 先说所有实验共用的 train / test 配置

只要你在 `run.sh` 里写的是：

```bash
python -u main.py --config=qmix --env-config=...
```

那么这些实验共用下面这套训练与测试主配置。

#### 训练公共配置

- 算法：`QMIX`
- runner：`parallel`
- 并行环境数：`batch_size_run: 4`
- 训练 batch size：`batch_size: 32`
- replay buffer：`buffer_size: 32`
- 学习率：`lr: 0.0005`
- 折扣因子：`gamma: 0.99`
- 最大训练步数：`t_max: 1000000000`
- target network 更新间隔：`target_update_interval: 20`
- 动作选择：`epsilon_greedy`
- epsilon 从 `1.0` 衰减到 `0.05`
- epsilon 衰减步数：`100000`

外行版理解：

- 这些参数决定“怎么训练”，而不是决定“踢什么任务”。

#### 测试公共配置

- 每隔 `30000` 个环境步做一次测试
- 每次测试跑 `8` 个 episode
- 因为 `batch_size_run = 4`，所以实际是分 `2` 轮测完，每轮并行跑 `4` 局
- 测试时 `test_greedy: True`
- 也就是测试时不用训练阶段那种探索味道更强的随机策略，而是尽量按当前最优动作来跑

外行版理解：

- 训练时会边学边试，有探索
- 测试时更像正式考试，尽量拿当前学到的最好策略去踢

#### train 和 test 共用什么，不共用什么

共用的：

- 同一个 `env-config`
- 同一个模型参数
- 同一个任务定义

不同的：

- train 会更新模型
- test 不更新模型
- train 会带探索
- test 主要用贪心策略

注意：

- 对于 `3vs3scenario`，train 和 test 都会继续使用 scenario/custom 起点
- 对于 `11vs11benchmark`，train 和 test 都会继续使用默认整场比赛开局
- `test_mode` 不会把 `3vs3` 自动改成中场开球，也不会把 `11vs11` 自动改成 scenario 起点

### A. `11vs11` 全场基准实验

这组实验在比较两件事：

- 对手强度从 `lv1` 到 `lv3` 时，我方能踢成什么样
- 打开 `EPV` 之后，整场表现有没有变化

这些命令分别表示：

- `parallelr2drl_11vs11benchmark_base_opp-lv1_epv-off`
  - `11vs11` 完整比赛
  - 我方动作空间是 `Base`
  - 对手是弱档 `lv1`
  - 奖励不加 `EPV`
- `parallelr2drl_11vs11benchmark_base_opp-lv1_epv-on`
  - 和上一条完全一样
  - 唯一差别是奖励加了 `EPV`
- `parallelr2drl_11vs11benchmark_base_opp-lv2_epv-off`
  - `11vs11` 完整比赛
  - `Base` 动作空间
  - 对手变成中档 `lv2`
  - 不加 `EPV`
- `parallelr2drl_11vs11benchmark_base_opp-lv2_epv-on`
  - 同样是 `lv2`
  - 但奖励加 `EPV`
- `parallelr2drl_11vs11benchmark_base_opp-lv3_epv-off`
  - `11vs11` 完整比赛
  - `Base` 动作空间
  - 对手是强档 `lv3`
  - 不加 `EPV`
- `parallelr2drl_11vs11benchmark_base_opp-lv3_epv-on`
  - 同样是 `lv3`
  - 但奖励加 `EPV`

这一组的 train 配置：

- 训练任务：`11vs11` 完整比赛
- 训练开局：默认比赛开局
- 训练 reset 类型：`default reset`
- 训练 episode 长度上限：`3000`
- 训练中不因进球立刻终止
- 我方动作空间：`Base`
- 差异项只在：
  - 对手强度 `lv1 / lv2 / lv3`
  - 奖励是否 `EPV on/off`

这一组的 test 配置：

- 测试任务仍然是同一个 `11vs11` 完整比赛
- 测试开局仍然是默认比赛开局
- 测试 reset 类型仍然是 `default reset`
- 每次测试总共跑 `8` 局
- 测试时使用当前模型的贪心策略
- 测试关注的是：
  - 打不同强度对手时能不能赢
  - 打开 `EPV` 后整场效果有没有提升

这一组最适合回答的问题：

- 我方在完整比赛里，打不同强度对手表现怎样？
- `EPV` 奖励对整场踢法有没有帮助？

### B. `3vs3` 场景基准实验

这组实验在比较三件事：

- 起始场景是 `easy / medium / hard`
- 对手是不是很强
- 奖励里有没有 `EPV`

`run.sh` 里这几条命令当前都是：

- `Base` 动作空间
- `init-3`
- 对手 `lv3`

它们分别表示：

- `parallelr2drl_3vs3scenario_base_init-3_start-easy_opp-lv3_epv-on`
  - `3vs3` 场景任务
  - 一开始控制 3 个我方球员
  - 开局局面对我方比较友好
  - 对手很强
  - 奖励加 `EPV`
- `parallelr2drl_3vs3scenario_base_init-3_start-medium_opp-lv3_epv-on`
  - 和上一条一样
  - 只是开局难度换成 `medium`
- `parallelr2drl_3vs3scenario_base_init-3_start-hard_opp-lv3_epv-on`
  - 和上一条一样
  - 只是开局难度换成 `hard`
- `parallelr2drl_3vs3scenario_base_init-3_start-easy_opp-lv3_epv-off`
  - `easy` 开局
  - 强对手
  - 奖励不加 `EPV`
- `parallelr2drl_3vs3scenario_base_init-3_start-medium_opp-lv3_epv-off`
  - `medium` 开局
  - 强对手
  - 奖励不加 `EPV`
- `parallelr2drl_3vs3scenario_base_init-3_start-hard_opp-lv3_epv-off`
  - `hard` 开局
  - 强对手
  - 奖励不加 `EPV`

这一组的 train 配置：

- 训练任务：`3vs3` scenario 小场景
- 训练开局：从轨迹采样的 scenario 起点开始
- 训练 reset 类型：`custom/scenario reset`
- 训练 episode 长度上限：`300`
- 训练时进球就终止这一局
- 我方动作空间：`Base`
- 一开始控制球员数：`init-3`
- 当前这组命令里对手固定为 `lv3`
- 当前这组命令里主要比较：
  - `start-easy / medium / hard`
  - `EPV on/off`

这一组的 test 配置：

- 测试任务仍然是同一个 `3vs3` scenario
- 测试开局仍然不是中场，而是继续从 scenario 采样起点开始
- 测试 reset 类型仍然是 `custom/scenario reset`
- 每次测试总共跑 `8` 局
- 测试时使用当前模型的贪心策略
- 测试关注的是：
  - 在不同开局难度下成功率怎样
  - 打开 `EPV` 后局部进攻表现是否更好

这一组最适合回答的问题：

- 在小场景攻防里，开局难度变化会不会明显影响学习效果？
- `EPV` 奖励对局部进攻推进有没有帮助？

### C. 动作空间比较实验

这组实验不是在比较对手强弱，也不是在比较开局难度。

这组实验主要比较：

- `Base` 动作空间
- `Hybrid` 动作空间

分成两类：

- `11vs11` 完整比赛下比较 `Base vs Hybrid`
- `3vs3` 场景任务下比较 `Base vs Hybrid`

具体来说：

- `parallelr2drl_11vs11benchmark_base_opp-lv1_epv-on`
  - `11vs11` 完整比赛
  - 弱对手 `lv1`
  - 奖励加 `EPV`
  - 我方用 `Base`
- `parallelr2drl_11vs11benchmark_hybrid_opp-lv1_epv-on`
  - 和上一条环境条件一样
  - 唯一差别是我方动作空间改成 `Hybrid`
- `parallelr2drl_3vs3scenario_base_init-3_start-hard_opp-lv3_epv-on`
  - `3vs3` 场景任务
  - 开局是 `hard`
  - 对手是强档 `lv3`
  - 奖励加 `EPV`
  - 我方用 `Base`
- `parallelr2drl_3vs3scenario_hybrid_init-3_start-hard_opp-lv3_epv-on`
  - 和上一条环境条件一样
  - 唯一差别是我方动作空间改成 `Hybrid`

这一组的 train 配置：

- `11vs11` 那两条：
  - 都是完整比赛
  - 都是默认比赛开局
  - 都是 `lv1 + EPV on`
  - 唯一变化是 `Base` 对比 `Hybrid`
- `3vs3` 那两条：
  - 都是 scenario 小场景
  - 都是 `init-3 + start-hard + opp-lv3 + EPV on`
  - 都是 scenario/custom 起点
  - 唯一变化是 `Base` 对比 `Hybrid`

这一组的 test 配置：

- 测试环境条件和训练任务定义保持一致
- `11vs11` 还是完整比赛、默认开局
- `3vs3` 还是 scenario 起点，不会变成中场开球
- 每次测试总共跑 `8` 局
- 测试时使用当前模型的贪心策略
- 主要看的是同样任务下 `Base` 和 `Hybrid` 谁表现更好

这一组最适合回答的问题：

- 同样的任务下，`Hybrid` 比 `Base` 学得更好吗？
- `Hybrid` 的表达能力提升，值不值得它更高的训练难度？

---

## 10. 现在 `run.sh` 正在跑的这一行是什么

当前脚本里真正没有注释的这一行是：

```bash
python -u main.py --config=qmix --env-config=parallelr2drl_3vs3scenario_base_init-3_start-hard_opp-lv3_epv-on --capture=no
```

它表示：

- `3vs3` 小场景任务
- 我方用 `Base` 动作空间
- 一开始控制 `3` 个我方球员
- 开局局面是 `hard`
- 对手是强档 `lv3`
- 奖励里打开了 `EPV`

外行版一句话：

- 这是一个“强对手 + 困难开局 + 3人局部攻防 + 奖励鼓励推进到危险区域”的实验。

---

## 11. 如果你只想快速选实验，直接看这个

### 想跑最像正常比赛的任务

选：

- `11vs11benchmark_*`

### 想跑从局部场景开始的快速任务

选：

- `3vs3scenario_*`

### 想比较奖励函数有没有帮助

看：

- `epv-off` 和 `epv-on`

### 想比较动作空间

看：

- `base` 和 `hybrid`

### 想比较对手强弱

看：

- `opp-lv1 / opp-lv2 / opp-lv3`

### 想比较开局难度

看：

- `start-easy / start-medium / start-hard`

---

## 12. 最短总结

如果只记一段话，可以记这个：

- `11vs11benchmark` 是完整比赛实验
- `3vs3scenario` 是局部场景实验
- `base/hybrid` 是动作空间类型
- `lv1/lv2/lv3` 是对手强度
- `easy/medium/hard` 是场景开局难度
- `epv-on/off` 是奖励里是否加入位置价值

看懂这几个词，`run.sh` 里大多数实验名就都能直接读懂了。
