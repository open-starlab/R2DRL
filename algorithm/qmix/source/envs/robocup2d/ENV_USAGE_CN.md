# RoboCup2D Env 使用说明

本文档按当前 `R2DRL/robocup2d` 代码编写，说明环境类、YAML、reset、scenario 起点、action/mask、EPV reward、运行命令和常见问题。

最后更新：2026-06-17。

## 1. 当前目录和定位

当前环境包在：

```text
R2DRL/robocup2d
```

这份 GitHub 包已经把三套训练代码和仿真运行时依赖放在同一个仓库内：

- `algorithm/qmix/source`：QMix/PyMARL 训练入口。
- `algorithm/mappo/source`：MAPPO 训练入口和 RoboCup2D adapter。
- `algorithm/paradqn/source`：ParaDQN 训练入口和 RoboCup2D adapter。
- `R2DRL/`：RoboCup2D 环境、HELIOS、librcsc、rcssserver 和运行时动态库。

运行时它会启动：

- `rcssserver`
- 左右两队 `sample_player`
- `sample_coach`
- `sample_trainer`

Python 侧通过共享内存和这些进程同步 obs/action/state/reset。

## 2. 安装和导入方式

从仓库根目录开始：

```bash
cd <repo-root>
python -m pip install -r requirements-demo.txt
./scripts/prepare_runtime.sh
```

`prepare_runtime.sh` 会把所有 YAML 里的运行时路径改成当前 checkout 的路径，包括 `server_path`、HELIOS 三个进程目录、`lib_paths` 和 `trajectory_path`。

只测试 import，不启动仿真：

```bash
PYTHONPATH="$PWD/R2DRL:$PWD/algorithm/qmix/source:${PYTHONPATH:-}" python - <<'PY'
from robocup2d import Robocup2dEnv
print("import ok", Robocup2dEnv)
PY
```

三种算法 demo：

```bash
MAX_ENV_STEPS=1000 ./scripts/run_qmix_demo.sh
MAX_ENV_STEPS=1000 ./scripts/run_mappo_demo.sh
MAX_ENV_STEPS=1000 ./scripts/run_paradqn_demo.sh
```

三种一起跑：

```bash
MAX_ENV_STEPS=1000 ./scripts/run_all_demos.sh
```

## 3. 三个环境类

入口都在 `env.py`。

### 3.1 `Robocup2dEnv`

底层环境，负责：

- 读取 YAML 并构造 `EnvConfig`
- 根据配置启动 server、players、coach、trainer
- 创建和管理共享内存
- reset、step、reward、terminal、restart
- 提供 `get_obs()`、`get_state()`、`get_avail_actions()`、`get_env_info()`

直接使用时：

```python
from robocup2d import Robocup2dEnv

env = Robocup2dEnv("robocup.yaml")
```

相对 YAML 名会在 `robocup2d/config/` 下查找。

### 3.2 `R2DRL`

单环境 wrapper。它内部持有一个 `Robocup2dEnv`。

如果 YAML 里有 `trajectory_path`，它会创建 `ScenarioStartSampler`。当 `use_custom_start: true` 时，每次 reset 前会：

```python
start, n_control = self.start_sampler.sample_start_and_n()
self.env.set_start_and_n(start, n_control)
self.env.reset()
```

适合单环境 scenario 调试。

### 3.3 `ParallelR2DRL`

并行训练 wrapper。它不自己采样起点，通常由 PyMARL `parallel_runner.py` 采样，然后通过 worker 命令调用：

```python
set_start_and_n(start, n_control)
```

适合 QMix/PyMARL 并行采样。

## 4. YAML 配置

配置目录：

```text
robocup2d/config/
```

当前有 69 个 YAML。

基础入口：

- `robocup.yaml`
- `r2drl.yaml`
- `parallelr2drl.yaml`

主要配置族：

- 11vs11 full-match / benchmark
- 5vs5 benchmark
- 3vs3 full-match benchmark
- 3vs3 trajectory scenario
- front-goal catalog scenario：1v0、1v1、2v1、3v2、4v3、5v4
- Base/Hybrid action-space comparison

常用命令格式：

```bash
python -u main.py --config=qmix --env-config=<不带.yaml的配置名> --capture=no
```

例子：

```bash
python -u main.py --config=qmix --env-config=parallelr2drl_5vs5benchmark_base_opp-lv3_epv-on --capture=no
python -u main.py --config=qmix --env-config=parallelr2drl_11vs11scenario_catalog_front-goal-1v1_opp-lv3_goal-only --capture=no
```

## 5. reset 语义

当前代码会根据 `benchmark_mode` 归一化 reset 方式。

| `benchmark_mode` | 代码行为 |
|---|---|
| `scenario` | 强制 `use_custom_start = true` |
| `full_match` | 强制 `use_custom_start = false` |
| `actionspace` | 不强制，按 YAML 的 `use_custom_start` |

`Robocup2dEnv.reset()` 的关键逻辑：

- 如果需要重启，先通过 `Runtime.restart_session()` 重启进程组。
- `use_custom_start: true` 时，通过 trainer 执行 `reset_custom()`，使用外层设置好的球和球员位置。
- `use_custom_start: false` 时，使用 default reset 或保留 server 自然 kickoff/goal restart。
- default reset 在 timeout 场景下会交替 kickoff side，避免总是同一侧开球。

reset 返回的信息包括：

- `turn_count`
- `score_left`
- `score_right`
- `initial_episode_epv`
- `max_episode_epv`
- `max_epv_improvement`
- `restart_total`
- `restart_recent`

## 6. step 和返回值

`step(actions)` 支持 NumPy 和 torch tensor。

Base 动作空间是一维离散动作，当前动作编号是：

| id | action |
|---:|---|
| 0 | tackle |
| 1 | shoot |
| 2 | intercept |
| 3 | advance |
| 4 | pass_direct |
| 5 | pass_lead |
| 6 | pass_through |
| 7 | hold |
| 8 | catch |
| 9 | dribble_up |
| 10 | dribble_down |
| 11 | dribble_left |
| 12 | dribble_right |
| 13 | move_up |
| 14 | move_down |
| 15 | move_left |
| 16 | move_right |
| 17 | helios |
| 18 | wait |

Hybrid 动作空间有 6 个动作：

- turn
- dash
- kick
- catch
- helios
- wait

Hybrid 还会通过共享内存传 `u0/u1` 两个连续参数。环境和 C++ player 侧已经支持，但默认 QMix/PyMARL 训练链路仍是离散 action，因此完整训练 Hybrid policy 还需要改 controller、buffer、runner、learner。

`step()` 返回：

```python
reward, done, info = env.step(actions)
```

`info` 常见字段：

- `win`, `lose`, `timeout`, `episode_limit`
- `score_left`, `score_right`, `score_diff`
- `ball_out`, `ball_left_half`, `possession_loss`
- `initial_episode_epv`, `max_episode_epv`, `max_epv_improvement`
- `epv_step_reward`, `epv_goal_completion_reward`, `epv_progress_scale`, `useMaxEpv`
- `terminal_reason`
- `restart_total`, `restart_recent`, `restart_consecutive`

注意：代码里把 RoboCup 自然 timeout 看成真实终止，不当作 PyMARL 人工 time-limit truncation，所以 `episode_limit` 字段通常为 0，`timeout` 字段才表示是否超时。

## 7. obs/state/action mask

`get_env_info()` 返回：

```python
{
    "n_agents": min(init_n, n1),
    "n_actions": 19 或 6,
    "state_shape": coach state 长度,
    "obs_shape": player obs 长度,
    "episode_limit": episode_limit,
}
```

`Agents.set_agent_mask()` 会按球的位置选择左队离球最近的 `init_n` 个球员作为 active agents。

- `get_obs()` 只返回 active 左队球员的观测。
- `get_avail_actions()` 只返回 active 左队球员的 mask。
- `use_action_mask: false` 时会使用 no-mask fallback。
- `freeze_non_controlled: true` 时，非控制球员会走 empty/default fallback 动作。

## 8. scenario 起点

当前有两套起点来源。

### 8.1 3vs3 trajectory scenario

使用：

```text
trajectories/3v3trajectories_right_half_left_nearest_kickable.npz
```

关键字段：

- `trajectory_path`
- `scenario_difficulty`: `easy`、`medium`、`hard`
- `scenario_difficulty_buckets`
- `progress_bucket_count`
- `current_target_window_size`
- `start_window_size`
- `num_selected_trajectories`
- `random_sample`

如果文件名包含 `_right_half_left_nearest_kickable`，代码认为它已经预过滤，不再重复做右半场和左队可踢过滤。

### 8.2 front-goal catalog scenario

使用：

```text
trajectories/scenarioes.npz
```

关键字段：

- `start_catalog_enabled: true`
- `start_id`: 例如 `front_goal_1v0`
- `start_index`: 用 index 选择 catalog entry
- `scenario_launch_from_catalog`: 默认 true

当 catalog profile 启用时，`apply_catalog_launch_profile()` 会根据 `.npz` 里的 metadata 覆盖：

- `n`
- `n1`
- `n2`
- `init_n`
- `active_right_unums`
- `freeze_non_controlled`

所以文件名里虽然有 `11vs11scenario_catalog`，实际发射的人数会根据 1v0、1v1、2v1 等场景动态调整。

## 9. EPV reward

EPV 网格在：

```text
LaurieOnTracking/EPV_grid.csv
LaurieOnTracking/EPV_grid_front_goal_linear.csv
```

配置字段：

- `useMaxEpv`
- `epv_grid_file`
- `epv_progress_reward`
- `epv_progress_scale`

当前逻辑：

- reset 时记录起始 EPV 为 baseline，不直接给 reward。
- `useMaxEpv: true` 时，只有我方真实 kickable 才更新 EPV progress。
- `max_episode_epv` 增加时给正奖励。
- 如果 `epv_progress_reward > 0`，使用固定进展奖励。
- 否则使用 `epv_delta * epv_progress_scale`。
- 我方进球时还会调用 `complete_episode_epv()`，把剩余 EPV 进展补到 grid 最大值。
- 进球本身仍然 `+1`，丢球仍然 `-1`。

## 10. 常用配置例子

5vs5 benchmark：

```bash
python -u main.py --config=qmix --env-config=parallelr2drl_5vs5benchmark_base_opp-lv3_epv-off --capture=no
```

11vs11 full-match：

```bash
python -u main.py --config=qmix --env-config=parallelr2drl_11vs11fullmatch_team-base_opp-lv3_epv-on --capture=no
```

front-goal 1v1：

```bash
python -u main.py --config=qmix --env-config=parallelr2drl_11vs11scenario_catalog_front-goal-1v1_opp-lv3_goal-only --capture=no
```

3vs3 trajectory scenario：

```bash
python -u main.py --config=qmix --env-config=parallelr2drl_3vs3scenario_base_init-3_start-medium_opp-lv3_epv-on --capture=no
```

Action-space comparison：

```bash
python -u main.py --config=qmix --env-config=parallelr2drl_11vs11actionspace_team-base_opp-lv1_epv-off --capture=no
python -u main.py --config=qmix --env-config=parallelr2drl_11vs11actionspace_team-hybrid_opp-lv1_epv-off --capture=no
```

## 11. 新增 YAML 时注意

新增配置建议从最接近的 YAML 复制。

必须检查：

- `benchmark_mode` 是否和 reset 语义一致。
- `init_n <= n1`。
- `opponent_level` 是否在 `[0, 1]`。
- `server_path`、`player_dir`、`coach_dir`、`trainer_dir`、`lib_paths` 是否对当前机器有效。
- 如果是 trajectory scenario，`scenario_difficulty_buckets` 是否覆盖了 `scenario_difficulty`。
- 如果是 catalog scenario，不要同时设置 `start_id` 和 `start_index`。
- 如果设置 `active_right_unums`，编号必须在 `1..n2` 内且不能重复。

## 12. 常见问题

### 12.1 import 报 `No module named torch`

当前 `env.py` 顶层导入 `torch`，所以 import 环境包也需要 torch。切到训练环境或安装 torch。

### 12.2 YAML 找不到

`Robocup2dEnv("xxx.yaml")` 的相对文件名会在 `robocup2d/config/` 下找。PyMARL 的 `--env-config` 需要训练框架那边能找到对应配置名。

### 12.3 reset 卡住

优先看：

- `logs_dir/<run_id>/server_*.log`
- `player_*_u*.log`
- `trainer_*.log`
- `coach_*.log`

然后检查：

- binary 路径是否正确
- 端口范围是否可用
- `wait_ready_timeout` / `trainer_ready_timeout_ms` 是否太短
- 是否有旧进程残留

### 12.4 front-goal 1v0 为什么右队仍有球员进程

rcssserver/trainer reset 握手需要右队存在。代码会让 `launch_n2 = max(1, effective_defenders)`，所以 1v0 会保留一个冻结/非 active 的右侧 dummy player。

### 12.5 为什么实际 `n/n1/n2` 和 YAML 表面值不同

如果 `start_catalog_enabled: true` 且 `scenario_launch_from_catalog` 没被设成 false，代码会从 catalog metadata 覆盖发射人数，这是当前 front-goal scenario 的预期行为。

