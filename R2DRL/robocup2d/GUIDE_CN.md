# RoboCup2D 环境中文说明指南与操作手册


本文档面向这个仓库中的 `pymarl/src/envs/robocup2d` 环境实现，目标是把“这个环境是什么、能做什么、怎么配、怎么跑、怎么看结果、出了问题怎么查”讲清楚，尽量做到拿来即用。

## 1. 环境概览

这个环境是一个面向多智能体强化学习的 RoboCup 2D Soccer Simulation 封装，核心目标是让 PyMARL 可以把 `rcssserver`、球员进程、coach、trainer 当作一个统一的 RL 环境来使用。

它的特点是：

- 使用共享内存而不是普通 socket 传递观测和动作，降低训练时延迟。
- 自动管理底层进程生命周期，包括启动、重启、关闭、清理。
- 提供 PyMARL 兼容接口，包括 `reset()`、`step()`、`get_obs()`、`get_state()`、`get_avail_actions()`。
- 支持同步 step，确保仿真周期与 RL 决策周期严格对齐。
- 支持两类任务形态：
  - `11vs11` 全场比赛
  - `3vs3` scenario/custom 起点任务
- 支持两类动作空间：
  - `Base` 离散动作空间
  - `Hybrid` 参数化动作空间

## 2. 目录结构

当前目录下最重要的文件和子目录如下：

- `env.py`
  - 环境主入口，定义 `Robocup2dEnv`、`R2DRL`、`ParallelR2DRL`
- `agents.py`
  - 管理共享内存中的球员、trainer、coach 读写
  - 负责动作写入、观测读取、mask 更新、EPV 计算
- `runtime.py`
  - 管理 run_id、端口选择、日志目录、底层进程启动和关闭
- `start_sampler.py`
  - 负责 3v3 scenario/custom 起始状态采样
  - 当前从已经预处理好的轨迹帧中直接抽样
- `config/`
  - 这个环境的 YAML 配置都放在这里
- `ipc/`
  - 共享内存布局、同步握手、SHM 生命周期管理
- `protocols/`
  - player / coach / trainer 的共享内存字段定义
- `process/`
  - server、球员、coach、trainer 的启动和清理逻辑
- `LaurieOnTracking/`
  - EPV 相关资源和参考实现
- `trajectories/`
  - scenario 起始状态数据
  - 当前保留的运行轨迹文件是 `3v3trajectories_right_half_left_nearest_kickable.npz`
- `BENCHMARKS.md`
  - benchmark 预设说明

## 3. 环境类型

这个目录下主要有三种配置入口。

### 3.1 `robocup`

适合全场任务，通常用于 `11vs11`。

特点：

- 默认不走 scenario 起点采样
- 可以配置成 full-match benchmark
- 适合看进球差、整场表现、reward shaping 效果

对应配置文件：

- `config/robocup.yaml`

### 3.2 `r2drl`

适合单环境 3v3 scenario 训练或评估。

特点：

- 支持 scenario 起点采样
- 一次只维护一个环境对象
- 更接近普通单环境 RL 使用方式

对应配置文件：

- `config/r2drl.yaml`

### 3.3 `parallelr2drl`

适合 PyMARL 并行采样。

特点：

- 配合 `parallel_runner.py`
- 一次启动多个环境 worker
- 适合 3v3 scenario 的批量训练

对应配置文件：

- `config/parallelr2drl.yaml`

## 4. 配置文件现在放在哪里

现在所有和这个环境本身有关的 YAML 都统一放在：

```text
pymarl/src/envs/robocup2d/config/
```

包括：

- 基础环境配置
  - `robocup.yaml`
  - `r2drl.yaml`
  - `parallelr2drl.yaml`
- benchmark 配置
  - `robocup_benchmark_full_match_easy.yaml`
  - `robocup_benchmark_full_match_easy_epv.yaml`
  - `robocup_benchmark_full_match_medium.yaml`
  - `robocup_benchmark_full_match_medium_epv.yaml`
  - `robocup_benchmark_full_match_hard.yaml`
  - `robocup_benchmark_full_match_hard_epv.yaml`
  - `r2drl_scenario_easy.yaml`
  - `r2drl_scenario_medium.yaml`
  - `robocup_benchmark_actionspace_easy_base.yaml`
  - `robocup_benchmark_actionspace_easy_hybrid.yaml`
  - `r2drl_benchmark_actionspace_easy_base.yaml`
  - `r2drl_benchmark_actionspace_easy_hybrid.yaml`

虽然文件已经挪到环境目录里，但命令行仍然可以继续用：

```bash
python main.py --env-config=robocup_benchmark_full_match_easy
```

因为 `main.py` 已经改成会优先去 `envs/robocup2d/config/` 里找这些配置。

## 5. 核心配置项说明

下面解释最常用的环境参数。

### 5.1 基础规模

- `n`
  - 每边球员数量
  - `11` 表示 `11vs11`
  - `3` 表示 `3vs3`
- `team`
  - 我方动作空间类型
  - 可选：
    - `Base`
    - `Hybrid`

### 5.2 对手难度

- `opponent_level`
  - 通过底层球员进程的 `--level` 参数传给对手
  - 当前 benchmark 约定要按实验类型区分：
    - `11vs11 full-match`: `easy = 0.05`, `medium = 0.5`, `hard = 1.0`
    - `3vs3 scenario`: easy / middle / hard 这组 preset 目前都固定为 `0.05`

说明：

- 这个参数只作用在对手队，也就是 team2。
- benchmark 预设已经把 easy / medium / hard 写死在 YAML 里，不需要每次运行时再额外传参数覆盖。

### 5.3 回合长度

- `episode_limit`
  - RL 视角下的最大 step 数
- `half_time`
  - 传给 server 的比赛半场长度

这两个不要混淆：

- `episode_limit` 决定 RL 什么时候 timeout
- `half_time` 决定仿真 server 的内部比赛时长设定

### 5.4 日志

- `logs_dir`
  - 运行日志目录
- `text_logging`
  - 是否开启文字日志
- `game_logging`
  - 是否开启比赛日志
- `tb`
  - 是否记录 TensorBoard
- `tb_log_dir`
  - TensorBoard 输出目录

### 5.5 Scenario 起点与 reward shaping

- `rewardshaping`
  - timeout 时是否把 `max_episode_epv` 加入 reward
- `init_n`
  - scenario 初始控制球员数
- `trajectory_path`
  - 3v3 scenario 起点轨迹文件
  - 当前配置都指向 `trajectories/3v3trajectories_right_half_left_nearest_kickable.npz`
- `use_custom_start`
  - 是否让底层环境使用外层采样好的 custom/scenario 起点

说明：

- 3v3 scenario / `parallelr2drl` 会读取 `.npz` 起点文件。
- 11v11 full-match / action-space benchmark 使用普通比赛 reset，不读取 `.npz`。

### 5.6 终止条件相关

当前环境支持以下几个开关：

- `terminate_on_goal`
  - 进球后是否立刻结束 episode
- `terminate_on_possession_loss`
  - 对方拿到球权后是否立刻结束 episode
- `reward_on_possession_loss`
  - 因丢球权结束时额外给多少 reward

它们主要用于区分两类 benchmark 语义：

- `full-match`
  - 不因为进球立刻结束
  - 最后用整场 `score_diff` 评估
- `scenario`
  - 进球、丢球权、超时都可以结束
  - 主要看 `success`

## 6. 环境返回值说明

### 6.1 `obs`

- 来自 player 共享内存
- 只返回我方球员视角观测
- 如果某个球员当前不在 active mask 中，则其观测会被置零

### 6.2 `state`

- 来自 coach 共享内存
- 是全局状态

### 6.3 `avail_actions`

- 当前可执行动作 mask
- 对 inactive 的球员，会只保留默认动作可用

### 6.4 `reward`

按环境类型不同，reward 语义不同。

常见情况：

- 我方进球：`+1`
- 我方丢球：`-1`
- timeout 且开启 `rewardshaping`：加上 `max_episode_epv`
- scenario 中对手拿到球权：
  - 默认额外 reward 为 `0`

### 6.5 `info`

当前环境已经提供了比较完整的 episode 统计，常见字段包括：

- `win`
- `lose`
- `timeout`
- `episode_limit`
- `goal_for`
- `goal_against`
- `score_left`
- `score_right`
- `score_diff`
- `success`
- `possession_loss`
- `max_episode_epv`
- `rewardshaping`
- `terminal_reason`

## 7. Agent Mask 机制

这个环境支持“只控制离球最近的若干个球员”。

逻辑在 `agents.py` 中，核心思路是：

1. 读取全局 state
2. 取出球的位置
3. 计算我方每个球员到球的距离
4. 选择最近的 `n_control` 个球员设为 active

作用：

- 用于 3v3 scenario
- 用于快速验证想法
- 可以减少控制规模，降低学习难度

同时环境还会根据球与双方最近球员的距离判断：

- `self.kickable`
- `self.opponent_kickable`
- `self.ball_owner_team`

这也是 scenario benchmark 中“对方获得球权则终止”的依据。

## 8. EPV 与 Reward Shaping

环境内置了 EPV 网格：

- 文件位置：`LaurieOnTracking/EPV_grid.csv`

环境会跟踪：

- `current_epv`
- `max_episode_epv`

用途：

- 如果 `rewardshaping=true`
- 则在 timeout 时，把 `max_episode_epv` 加到 episode reward 中

这类设定主要用于 full-match benchmark 中比较：

- 纯 scoring reward
- scoring reward + MaxEPV

## 9. Scenario 起点采样机制

`3vs3` 任务使用固定的 scenario 起点采样逻辑，核心在 `start_sampler.py`。

当前仓库已经不再使用旧的动态 `curriculum.py`。起点由 `ScenarioStartSampler` 从 `.npz` 轨迹文件中采样，然后通过 `set_start_and_n(start, n_control)` 交给底层环境。

当前只保留一个实际运行用轨迹文件：

```text
pymarl/src/envs/robocup2d/trajectories/3v3trajectories_right_half_left_nearest_kickable.npz
```

这个文件已经预处理过，只包含“球在右半场、左队最近球员可踢”的帧。因此 `start_sampler.py` 读取该文件时会直接取帧，不再重复做右半场和 kickable 检查。

11v11 full-match / action-space benchmark 当前不读取 `.npz`，因为这些配置是：

```yaml
n: 11
use_custom_start: false
```

并且没有 `trajectory_path`。

大体思路：

- 从处理后的 3v3 轨迹文件里取起始状态
- 用 `scenario_difficulty` 选择 easy / middle / hard 对应的轨迹进度 bucket
- 用 `init_n` 决定初始控制最近的多少名球员
- reset 前把采样出的起点和控制人数写入底层环境

相关配置包括：

- `trajectory_path`
- `start_window_size`
- `progress_bucket_count`
- `current_target_window_size`
- `num_selected_trajectories`
- `random_sample`
- `scenario_difficulty`
- `scenario_difficulty_buckets`

其中最重要的是：

- `trajectory_path`
  - 指向起始状态轨迹文件
  - 当前应使用 `3v3trajectories_right_half_left_nearest_kickable.npz`
- `init_n`
  - 初始控制人数
- `scenario_difficulty`
  - 起点难度，可选 `easy`、`middle` / `medium`、`hard`
- `scenario_difficulty_buckets`
  - 把难度映射到轨迹进度 bucket

## 10. 动作空间说明

### 10.1 Base 动作空间

- 离散动作空间
- 当前 PyMARL 默认训练链路完全支持

适合：

- 当前仓库里的主训练流程
- benchmark 的标准对比

### 10.2 Hybrid 动作空间

- 参数化动作空间
- 环境本身已经支持写入 `action + u0 + u1`

但是要注意：

- 当前仓库默认 PyMARL 训练链路仍按“单个离散动作 index”处理
- 所以 `hybrid` 目前是“环境 ready”
- 但还不是“训练链路完全打通”

这意味着：

- 你可以把它作为环境能力研究对象
- 但如果要正式训练 hybrid policy，还需要继续改：
  - controller
  - replay buffer
  - runner
  - learner 输入输出格式

## 11. 最常用运行方式

以下命令默认在：

```bash
cd /fsws1/h_qin/robocup/robocup/pymarl/src
```

### 11.1 跑 11vs11 full-match benchmark

```bash
python main.py --config=qmix --env-config=robocup_benchmark_full_match_easy
```

Medium / Hard 只需要切换 `env-config`：

```bash
python main.py --config=qmix --env-config=robocup_benchmark_full_match_medium
python main.py --config=qmix --env-config=robocup_benchmark_full_match_hard
```

如果要打开 EPV shaping 版本：

```bash
python main.py --config=qmix --env-config=robocup_benchmark_full_match_easy_epv
```

### 11.2 跑 3vs3 scenario benchmark

```bash
python main.py --config=qmix --env-config=r2drl_scenario_easy
```

或者并行训练版本：

```bash
python main.py --config=qmix --env-config=parallelr2drl
```

如果你想明确跑 benchmark 预设，建议直接用：

```bash
python main.py --config=qmix --env-config=r2drl_scenario_medium
```

### 11.3 继续训练或做评估

常见方式是加载 checkpoint：

```bash
python main.py \
  --config=qmix \
  --env-config=robocup_benchmark_full_match_easy \
  --checkpoint_path=results/models/你的实验目录 \
  --evaluate=True
```

如果只想加载最近 checkpoint，一般保留：

- `--load_step=0`

### 11.4 13 组基准实验清单

下面这 13 组实验可以直接作为当前环境的标准 benchmark 矩阵。

#### 11vs11 全场基准实验

```bash
python main.py --config=qmix --env-config=robocup_benchmark_full_match_easy
python main.py --config=qmix --env-config=robocup_benchmark_full_match_easy_epv
python main.py --config=qmix --env-config=robocup_benchmark_full_match_medium
python main.py --config=qmix --env-config=robocup_benchmark_full_match_medium_epv
python main.py --config=qmix --env-config=robocup_benchmark_full_match_hard
python main.py --config=qmix --env-config=robocup_benchmark_full_match_hard_epv
```

对应关系：

- `robocup_benchmark_full_match_easy`
  - 简单对手 + Scoring
- `robocup_benchmark_full_match_easy_epv`
  - 简单对手 + Scoring + MaxEPV
- `robocup_benchmark_full_match_medium`
  - 中等对手 + Scoring
- `robocup_benchmark_full_match_medium_epv`
  - 中等对手 + Scoring + MaxEPV
- `robocup_benchmark_full_match_hard`
  - 困难对手 + Scoring
- `robocup_benchmark_full_match_hard_epv`
  - 困难对手 + Scoring + MaxEPV

#### 3vs3 场景基准实验

```bash
python main.py --config=qmix --env-config=r2drl_scenario_easy
python main.py --config=qmix --env-config=r2drl_scenario_medium
```

对应关系：

- `r2drl_scenario_easy`
  - easy 开局难度
- `r2drl_scenario_medium`
  - middle 开局难度

共同设定：

- 对手难度固定为 `easy`
- 动作空间固定为 `Base`
- `episode_limit: 300`

#### 动作空间比较实验

```bash
python main.py --config=qmix --env-config=robocup_benchmark_actionspace_easy_base
python main.py --config=qmix --env-config=robocup_benchmark_actionspace_easy_hybrid
python main.py --config=qmix --env-config=r2drl_benchmark_actionspace_easy_base
python main.py --config=qmix --env-config=r2drl_benchmark_actionspace_easy_hybrid
```

对应关系：

- `robocup_benchmark_actionspace_easy_base`
  - `11vs11` + `Base`
- `robocup_benchmark_actionspace_easy_hybrid`
  - `11vs11` + `Hybrid`
- `r2drl_benchmark_actionspace_easy_base`
  - `3vs3` + `Base`
- `r2drl_benchmark_actionspace_easy_hybrid`
  - `3vs3` + `Hybrid`

推荐运行方式：

- 做 `11vs11` 对手难度和奖励函数对比时，用 `robocup_benchmark_full_match_*`
- 做 `3vs3` 开局难度对比时，用 `r2drl_scenario_*`
- 做动作空间对比时，先跑 `Base`，再跑 `Hybrid`

说明：

- `Hybrid` 配置现在环境侧已经支持。
- 但如果要完整训练 `Hybrid policy`，PyMARL 训练链路仍可能需要继续适配。
- 所以如果你的目标是先把 benchmark 全部稳定跑通，建议先以 `Base` 为主。

## 12. Benchmark 说明

### 12.1 Full-match Benchmark

目标：

- 证明难度缩放有效
- 证明 reward shaping 有意义

标准设定：

- `11vs11`
- normal kickoff
- `3000` frames

比较维度：

- 对手难度
  - easy `0.05`
  - medium `0.5`
  - hard `1.0`
- reward
  - scoring reward
  - scoring reward + MaxEPV

主要指标：

- `Average Goal Difference`
- 当前日志里通常体现为：
  - `test_score_diff_mean`

### 12.2 Scenario Benchmark

目标：

- 展示环境支持从 easy 到 hard 的渐进难度
- 展示环境适合快速验证想法

标准设定：

- `3vs3`
- 从预处理轨迹文件中采样 scenario 起点
- 控制离球最近的 `1-3` 名球员

终止条件：

- 进球
- 防守方拿到球权
- 到达 `300` frames

reward 设定：

- 丢球权后不额外罚分

主要指标：

- `success rate`
- 当前日志里通常体现为：
  - `test_success_mean`

### 12.3 Action-space Comparison

目标：

- 展示环境支持动作空间研究

标准设定：

- easy 对手
- `11vs11` full match

当前现状：

- `base` 可直接走默认 PyMARL 训练链路
- `hybrid` 还需要额外改训练链路

## 13. 输出与日志位置

### 13.1 仿真日志

通常在：

```text
pymarl/src/log/<run_id>/
```

其中常见内容包括：

- server 日志
- player 日志
- coach 日志
- trainer 日志
- `rcg/` 比赛记录

### 13.2 模型输出

通常在：

```text
results/models/<实验名>/<timestep>/
```

### 13.3 TensorBoard

通常在：

```text
results/tb_logs/<unique_token>/
```

同时环境内也可能往 `tb_log_dir` 写 scenario 相关日志。

## 14. 常见问题与排障

### 14.1 进程残留

表现：

- 上一次任务停了，但 `rcssserver`、`sample_player`、`sample_trainer` 还在

建议：

- 优先 `scancel <jobid>` 停 SLURM 作业
- 然后检查是否有残留孤儿进程
- 必要时定点 kill 对应 run 的 server / player / coach / trainer

### 14.2 端口冲突

表现：

- 环境起不来
- 不同 worker 抢占同一组端口

原因：

- 平行环境会自动选端口
- 但如果旧进程没清干净，仍可能占用端口

建议：

- 检查旧进程
- 检查 `auto_port_start` / `auto_port_end` / `auto_port_step`

### 14.3 READY / REQ 卡死

表现：

- `wait_all_ready()` 超时
- reset 或 step 卡住

原因通常包括：

- 某个底层进程崩了
- SHM 状态不一致
- 上一轮进程没有清理干净

建议：

- 先看 `log/<run_id>/` 下各子进程日志
- 看是否触发 `restart`
- 查 trainer / player 是否都存活

### 14.4 `torch` 导入失败

表现：

- 运行某些脚本时提示 `ModuleNotFoundError: No module named 'torch'`

原因：

- 这个环境和 PyMARL 主流程都依赖 `torch`

建议：

- 在你的训练环境里安装对应版本的 `torch`

### 14.5 Hybrid 训练不通

表现：

- 环境能启动，但训练链路维度不匹配
- action buffer 只认离散 action

原因：

- 当前默认 PyMARL 仍假设 `actions` 是单离散 index

建议：

- 如果只是做 benchmark，先用 `Base`
- 如果要研究 `Hybrid`，需要继续修改训练链路

## 15. 推荐使用方式

如果你想快速开始，我建议按下面顺序：

1. 先用 `robocup_benchmark_full_match_easy`
   - 确认 11vs11 全场能稳定跑起来
2. 再用 `r2drl_scenario_easy`
   - 确认 scenario 起点采样能正常工作
3. 然后切到 `medium` 和 `hard`
   - 看难度提升后的指标变化
4. 最后再比较 `rewardshaping=false` 与 `true`
   - 看 MaxEPV 是否带来收益

## 16. 一句话总结

这个环境本质上是一个“为 PyMARL 服务的、高性能 RoboCup2D 多智能体训练封装”：

- `11vs11` 用来看整场能力与 reward 设计
- `3vs3` 用来看 scenario 起点、局部控制和快速验证
- `Base` 动作空间可以直接训练
- `Hybrid` 动作空间环境已支持，但训练链路还需要继续扩展

如果后面你希望，我还可以继续补两类文档：

- 面向“论文实验复现”的精简版操作清单
- 面向“开发者改代码”的源码结构与调用链说明
