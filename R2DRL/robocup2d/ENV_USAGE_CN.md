# RoboCup2D Env 使用说明

本文档说明当前 `pymarl/src/envs/robocup2d` 环境的使用方式。它以当前代码为准，重点解释 `env.py` 里的三个环境类、YAML 配置、scenario 起点、reset 逻辑、MaxEPV 开关和常见实验配置。

## 1. 三个环境类的关系

当前主要入口都在 `env.py`：

- `Robocup2dEnv`
- `R2DRL`
- `ParallelR2DRL`

### 1.1 Robocup2dEnv

`Robocup2dEnv` 是真正和 RoboCup2D 仿真交互的底层环境。

它负责：

- 读取 YAML 并构造 `EnvConfig`
- 启动和管理 `rcssserver`、player、coach、trainer
- 创建共享内存并同步 step
- 执行动作写入
- 读取 obs/state/action mask
- reset 和 restart
- 计算基础 reward、终止条件、MaxEPV reward

它本身不负责从轨迹文件里采样 scenario 起点。

它提供：

```python
set_start_and_n(start, n_control)
```

这个接口只是在底层环境里保存一组 custom 起点。真正什么时候采样起点，由外层 wrapper 或 runner 决定。

### 1.2 R2DRL

`R2DRL` 是单环境 wrapper。

它内部包了一个 `Robocup2dEnv`，如果 YAML 里配置了 `trajectory_path`，它会创建 `ScenarioStartSampler`。

训练 reset 时，它会：

```python
start, n_control = self.start_sampler.sample_start_and_n()
self.env.set_start_and_n(start, n_control)
self.env.reset()
```

所以 `R2DRL` 适合单环境 scenario 起点实验或调试。

### 1.3 ParallelR2DRL

`ParallelR2DRL` 是并行训练用的 wrapper。

它也包了一个 `Robocup2dEnv`，但它自己不创建 `ScenarioStartSampler`。并行时，起点采样在 `parallel_runner.py` 里完成，然后通过 worker 命令发送给每个子环境：

```python
("set_start_and_n", (start, n_control))
```

所以 `ParallelR2DRL` 适合 PyMARL 并行采样训练。

## 2. reset 逻辑

底层 reset 逻辑在 `Robocup2dEnv.reset()`：

```python
if self.config.use_custom_start:
    self.agents.reset_custom()
else:
    if self.turn_count > 1 and int(goal) == 0:
        self.agents.reset_default()
```

含义如下：

| use_custom_start | reset 行为 |
|---|---|
| true | 使用 `reset_custom()`，也就是 scenario/custom 起点 |
| false | 使用普通比赛 reset 逻辑，需要时调用 `reset_default()` |

注意：`use_custom_start` 只控制底层 reset 是否使用已经配置好的 custom 起点。它不会自己从轨迹文件采样。采样由 `R2DRL` 或 `parallel_runner.py` 完成。`test_mode` 不再改变 reset 类型，它只表示当前是训练还是测试/评估。

## 3. Scenario 起点采样

当前已经不再使用旧的动态课程学习类。

旧的 `CurriculumController` 已删除。现在使用：

```text
start_sampler.py
```

其中 `ScenarioStartSampler` 只负责一件事：

从 `trajectory_path` 指向的 `.npz` 轨迹文件里，按照 YAML 指定的 `scenario_difficulty` 采样一个起始状态。

### 3.1 关键 YAML 字段

scenario 配置中最重要的字段：

```yaml
use_custom_start: true
trajectory_path: /path/to/3v3trajectories_right_half_left_nearest_kickable.npz
start_window_size: 1
progress_bucket_count: 30
current_target_window_size: 1
num_selected_trajectories: 100
random_sample: false
scenario_difficulty_buckets:
  easy: 3
  middle: 15
  hard: 29
scenario_start: easy
scenario_difficulty: easy
```

其中：

- `trajectory_path`
  - 起点轨迹文件
- `scenario_difficulty`
  - 当前要采样的起点难度
  - 可选：`easy`、`middle`、`hard`
- `scenario_difficulty_buckets`
  - 难度名到 bucket index 的映射
  - bucket 是 `0..progress_bucket_count-1` 的 0-based index；如果按 30 个 bucket 对旧值取补，配置里应写 `29 - old_bucket`
- `progress_bucket_count`
  - 总 bucket 数
- `current_target_window_size`
  - 每次采样覆盖几个 bucket
- `start_window_size`
  - 在选定 frame 附近随机采样的窗口大小
- `num_selected_trajectories`
  - 只使用前多少条轨迹；为空或超过轨迹总数时使用全部
- `random_sample`
  - 是否随机选择轨迹子集

### 3.2 easy / middle / hard 的含义

当前三个 scenario benchmark 的核心区别是起点难度：

| 文件 | scenario_start | scenario_difficulty | bucket |
|---|---|---|---|
| `r2drl_scenario_easy.yaml` | easy | easy | 3 |
| `r2drl_scenario_medium.yaml` | middle | middle | 15 |
| `r2drl_scenario_hard.yaml` | hard | hard | 29 |

这三个文件的对手状态目前都固定为：

```yaml
opponent_state: easy
opponent_level: 0.3
```

所以 easy/middle/hard 不是对手难度，而是起始位置分布难度。

## 4. YAML 配置文件

所有环境 YAML 都是完整配置文件，放在：

```text
pymarl/src/envs/robocup2d/config/
```

当前不再使用 `include` 或 `components/`。

### 4.1 基础配置

- `r2drl.yaml`
  - 单环境 `R2DRL`
  - 默认 3v3
  - 如果没有 `trajectory_path`，不会自动采样 scenario 起点

- `parallelr2drl.yaml`
  - 并行 `ParallelR2DRL`
  - 通常配合 `parallel_runner.py`
  - 有 `trajectory_path`，runner 会使用 `ScenarioStartSampler`

- `robocup.yaml`
  - 直接使用 `Robocup2dEnv`
  - 不会自动从 YAML 采样 scenario 起点

### 4.2 Scenario benchmark

- `r2drl_scenario_easy.yaml`
- `r2drl_scenario_medium.yaml`
- `r2drl_scenario_hard.yaml`

共同特点：

```yaml
env: parallelr2drl
benchmark_mode: scenario
n: 3
team: Base
opponent_state: easy
opponent_level: 0.3
use_custom_start: true
terminate_on_goal: true
terminate_on_possession_loss: true
useMaxEpv: false
```

区别是 `scenario_start` / `scenario_difficulty`。

### 4.3 Full match benchmark

- `robocup_benchmark_full_match_easy.yaml`
- `robocup_benchmark_full_match_medium.yaml`
- `robocup_benchmark_full_match_hard.yaml`
- `robocup_benchmark_full_match_easy_epv.yaml`
- `robocup_benchmark_full_match_medium_epv.yaml`
- `robocup_benchmark_full_match_hard_epv.yaml`

共同特点：

```yaml
env: robocup
benchmark_mode: full_match
n: 11
team: Base
use_custom_start: false
terminate_on_goal: false
terminate_on_possession_loss: false
```

区别：

- easy/middle/hard 控制对手强度：
  - easy: `opponent_level: 0.3`
  - middle: `opponent_level: 0.6`
  - hard: `opponent_level: 0.9`
- `_epv` 文件会设置：
  - `useMaxEpv: true`

### 4.4 Action space benchmark

- `robocup_benchmark_actionspace_easy_base.yaml`
- `robocup_benchmark_actionspace_easy_hybrid.yaml`

这两个配置用于比较 action space。

区别：

```yaml
team: Base
```

或：

```yaml
team: Hybrid
```

## 5. 关键配置字段

### 5.1 env

```yaml
env: parallelr2drl
```

决定 PyMARL 使用哪个环境入口。

常见值：

- `robocup`
- `r2drl`
- `parallelr2drl`

### 5.2 benchmark_mode

```yaml
benchmark_mode: scenario
```

仅用于标记实验类型，方便日志和人读配置。当前常见值：

- `train`
- `scenario`
- `full_match`
- `actionspace`

### 5.3 n

每边球员数。

```yaml
n: 3
```

或：

```yaml
n: 11
```

### 5.4 team

控制 action space。

```yaml
team: Base
```

或：

```yaml
team: Hybrid
```

`Base` 使用基础离散动作；`Hybrid` 使用参数化动作。

### 5.5 opponent_state / opponent_level

```yaml
opponent_state: easy
opponent_level: 0.3
```

`opponent_state` 是人读标签，`opponent_level` 是实际数值。

常用约定：

- easy: `0.3`
- middle: `0.6`
- hard: `0.9`

### 5.6 use_custom_start

```yaml
use_custom_start: true
```

控制 reset 时是否使用 custom 起点。

- `true`
  - 使用 `reset_custom()`
- `false`
  - 使用普通比赛 reset 逻辑

### 5.7 useMaxEpv

```yaml
useMaxEpv: false
```

控制是否启用 MaxEPV。

当 `useMaxEpv: true` 时：

- `Agents` 会加载 EPV grid
- 每个 episode 跟踪 `max_episode_epv`
- timeout 时把 `max_episode_epv` 加入 reward
- `info` 中输出：
  - `max_episode_epv`
  - `useMaxEpv`

### 5.8 terminate_on_goal

```yaml
terminate_on_goal: true
```

进球或丢球后是否终止 episode。

scenario benchmark 通常设为 `true`。

full match benchmark 通常设为 `false`。

### 5.9 terminate_on_possession_loss

```yaml
terminate_on_possession_loss: true
```

如果右队获得球权，是否终止 episode。

scenario benchmark 通常设为 `true`。

### 5.10 trajectory_path

```yaml
trajectory_path: /fsws1/h_qin/robocup/robocup/pymarl/src/envs/robocup2d/trajectories/3v3trajectories_right_half_left_nearest_kickable.npz
```

有这个字段时，`R2DRL` 和 `parallel_runner.py` 可以创建 `ScenarioStartSampler`。

没有这个字段时，不会自动采样 scenario 起点。

## 6. 训练和测试行为

### 6.1 训练模式

当 `test_mode == false`：

- `R2DRL`
  - 如果有 `trajectory_path` 且 `use_custom_start: true`，会先采样起点
  - 然后调用 `set_start_and_n()`
  - reset 时使用 custom 起点

- `ParallelR2DRL`
  - 如果 `use_custom_start: true`，runner 采样起点
  - 发送给每个 worker
  - worker 调用 `set_start_and_n()`
  - reset 时使用 custom 起点

### 6.2 测试模式

当 `test_mode == true`：

- reset 类型仍然由 `use_custom_start` 决定
- `use_custom_start: true` 时仍然使用 scenario/custom 起点
- `use_custom_start: false` 时仍然使用普通比赛 reset 逻辑

也就是说，`test_mode` 不负责选择 default reset 或 custom reset。default reset 是 benchmark/full match 配置使用的，custom reset 是 scenario 配置使用的。

## 7. 常用命令

进入 PyMARL 源码目录：

```bash
cd /fsws1/h_qin/robocup/robocup/pymarl/src
```

激活环境：

```bash
source /home/h_qin/workspace6/anaconda3/etc/profile.d/conda.sh
conda activate marl
```

运行一个 scenario benchmark：

```bash
python -u main.py --config=qmix --env-config=r2drl_scenario_easy --capture=no
```

运行 full match benchmark：

```bash
python -u main.py --config=qmix --env-config=robocup_benchmark_full_match_easy --capture=no
```

运行 MaxEPV full match：

```bash
python -u main.py --config=qmix --env-config=robocup_benchmark_full_match_easy_epv --capture=no
```

## 8. 如何新增实验 YAML

现在每个 YAML 都是完整配置，不再使用 `include`。

新增实验时建议从最接近的文件复制一份，例如：

```bash
cp r2drl_scenario_easy.yaml r2drl_scenario_easy_maxepv.yaml
```

然后修改：

```yaml
useMaxEpv: true
tb_log_dir: ./runs/r2drl_scenario_easy_maxepv
```

如果要改 scenario 起点难度，修改：

```yaml
scenario_start: hard
scenario_difficulty: hard
```

如果要改对手强度，修改：

```yaml
opponent_state: hard
opponent_level: 0.9
```

如果要改 action space，修改：

```yaml
team: Hybrid
```

## 9. 常见问题

### 9.1 为什么 YAML 里有 scenario_difficulty，但没改变起点？

检查以下条件：

- `trajectory_path` 是否存在
- `use_custom_start` 是否为 `true`
- 使用的入口是否会采样起点

`Robocup2dEnv` 本身不会自动从 YAML 采样起点。`R2DRL` 会自动采样，`ParallelR2DRL` 由 `parallel_runner.py` 采样。

### 9.2 test 时会不会用 scenario 起点？

会。当前 reset 逻辑明确规定：

```python
if self.config.use_custom_start:
    self.agents.reset_custom()
```

所以只要 `use_custom_start: true`，test mode 和 train mode 都会使用 custom/scenario 起点。`test_mode` 只影响评估流程，不影响 reset 类型。

### 9.3 use_custom_start 和 trajectory_path 是一回事吗？

不是。

- `trajectory_path`
  - 用来采样起点
- `use_custom_start`
  - 决定 reset 时是否使用已经配置好的 custom 起点

两者通常一起用于 scenario 实验。

### 9.4 MaxEPV 和 scenario 起点有关吗？

没有直接关系。

- scenario 起点由 `ScenarioStartSampler` 控制
- MaxEPV 由 `useMaxEpv` 控制

可以单独开关。

### 9.5 robocup 能不能用 scenario 起点？

直接用 `Robocup2dEnv` 时不会自动采样起点。

但底层环境支持 `set_start_and_n()`。如果你手动调用它，并设置 `use_custom_start: true`，也可以使用 custom 起点。

## 10. 当前代码里已经不再使用的东西

当前版本已经去掉旧的动态课程学习逻辑：

- 没有 `curriculum.py`
- 没有 `CurriculumController`
- 不再维护胜率窗口
- 不再根据训练表现推进起点难度
- 不再输出 `curriculum/*` frontier stats
- YAML 中不再使用：
  - `curriculum`
  - `return_window_size`
  - `window_move_win_rate_threshold`
  - `new_key_probability`

现在保留的是固定 scenario 起点采样。
