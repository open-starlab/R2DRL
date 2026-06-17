# R2DRL RoboCup2D 中文操作手册

本文档面向当前 GitHub 镜像中的 `R2DRL/robocup2d`，目标是说明“这版代码是什么、怎么安装、怎么跑、每类配置代表什么、出了问题怎么查”。

最后更新：2026-06-17。

## 1. 这版代码是什么

当前 GitHub 包是面向复现实验和直接运行 demo 的自包含镜像。它包含：

- `R2DRL/robocup2d/`：Python 强化学习环境和 YAML 配置。
- `R2DRL/helios-base/`：修改过的 HELIOS player/coach/trainer，包含运行 demo 用的二进制。
- `R2DRL/librcsc/`：修改过的 librcsc，包含运行 demo 用的 `librcsc.so`。
- `R2DRL/rcssserver/`：rcssserver 源码和运行 demo 用的 server/libs。
- `algorithm/qmix/source/`：QMix 训练代码。
- `algorithm/mappo/source/`：MAPPO 训练代码。
- `algorithm/paradqn/source/`：ParaDQN 训练代码。
- `scripts/`：路径本地化、构建辅助和三种算法 demo 脚本。

这版不是纯上游 HELIOS，也不是只给人看的源码快照；它的默认目标是让别人拿到这个 GitHub 文件夹后可以准备路径并直接跑短 demo。

## 2. 先看哪些文件

建议阅读顺序：

1. `R2DRL/README.md`
2. `robocup2d/README.md`
3. `robocup2d/BENCHMARKS.md`
4. `robocup2d/ENV_USAGE_CN.md`
5. `helios-base/README.md`
6. `librcsc/README.md`

如果只想跑实验，重点看 `robocup2d/BENCHMARKS.md` 和本文件的运行命令部分。

## 3. 安装和环境

Python 依赖可以从仓库根目录安装：

```bash
cd <repo-root>
python -m pip install -r requirements-demo.txt
```

运行前先做路径本地化和文件检查：

```bash
./scripts/prepare_runtime.sh
```

这个脚本会把 YAML 里的 `server_path`、`player_dir`、`coach_dir`、`trainer_dir`、`config_dir`、`player_config`、`lib_paths`、`trajectory_path` 改成当前 checkout 内部路径。

仿真侧运行时已经放在仓库里：

- `R2DRL/rcssserver/build/rcssserver`
- `R2DRL/helios-base/src/player/sample_player`
- `R2DRL/helios-base/src/coach/sample_coach`
- `R2DRL/helios-base/src/trainer/sample_trainer`
- `R2DRL/librcsc/rcsc/.libs/librcsc.so.19`
- `R2DRL/runtime_libs/`

如果换机器后二进制不兼容，再运行：

```bash
./scripts/build_simulators.sh
./scripts/prepare_runtime.sh
```

## 4. 最小检查

只检查 Python import：

```bash
cd <repo-root>
./scripts/prepare_runtime.sh
PYTHONPATH="$PWD/R2DRL:$PWD/algorithm/qmix/source:${PYTHONPATH:-}" python - <<'PY'
from robocup2d import Robocup2dEnv
print("import ok", Robocup2dEnv)
PY
```

直接跑三种短训练 demo：

```bash
MAX_ENV_STEPS=1000 ./scripts/run_qmix_demo.sh
MAX_ENV_STEPS=1000 ./scripts/run_mappo_demo.sh
MAX_ENV_STEPS=1000 ./scripts/run_paradqn_demo.sh
```

三种顺序跑：

```bash
MAX_ENV_STEPS=1000 ./scripts/run_all_demos.sh
```

## 5. 代码结构

### 5.1 Python 环境

- `env.py`
  - `Robocup2dEnv`
  - `R2DRL`
  - `ParallelR2DRL`
- `runtime.py`
  - run_id、端口、共享内存名、日志目录、进程启动/关闭/重启
- `agents.py`
  - action 写入、obs/state/mask 读取、agent mask、EPV reward、trainer reset payload
- `start_sampler.py`
  - 3v3 trajectory scenario 和 front-goal catalog scenario 起点采样
- `config/schema.py`
  - 配置字段校验和归一化
- `process/launcher.py`
  - 启动 `rcssserver`、player、coach、trainer

### 5.2 C++ 侧

- `helios-base/src/player/sample_player.cpp`
  - 连接 player shared memory
  - 写 obs/action mask
  - 读 Base/Hybrid action
  - 支持 `--shm-name` 和 `--level`
- `helios-base/src/coach/sample_coach.cpp`
  - 通过 `--shm-name` 写全局 state 和 goal flag
- `helios-base/src/trainer/sample_trainer.cpp`
  - 通过 `RCSC_TRAINER_SHM` 读取 reset 请求
  - 执行 default/custom reset
- `librcsc/rcsc/player/action_effector.*`
  - 新增显式方向 catch command
- `librcsc/rcsc/player/player_agent.*`
  - 新增 `doCatch(AngleDeg)` 接口

## 6. 配置族概览

当前 `robocup2d/config/` 有 69 个 YAML。

### 6.1 基础入口

- `robocup.yaml`
- `r2drl.yaml`
- `parallelr2drl.yaml`

这些更适合开发和 smoke test。正式实验建议用明确的 benchmark/scenario 配置。

### 6.2 11vs11 full-match / benchmark

常用配置：

```text
parallelr2drl_11vs11fullmatch_team-base_opp-lv1_epv-off
parallelr2drl_11vs11fullmatch_team-base_opp-lv1_epv-on
parallelr2drl_11vs11fullmatch_team-base_opp-lv2_epv-off
parallelr2drl_11vs11fullmatch_team-base_opp-lv2_epv-on
parallelr2drl_11vs11fullmatch_team-base_opp-lv3_epv-off
parallelr2drl_11vs11fullmatch_team-base_opp-lv3_epv-on
```

特点：

- `benchmark_mode: full_match`
- `use_custom_start: false`
- 不因进球立刻终止
- 主要看整场 `score_diff`

### 6.3 5vs5 benchmark

```text
parallelr2drl_5vs5benchmark_base_opp-lv3_epv-off
parallelr2drl_5vs5benchmark_base_opp-lv3_epv-on
```

特点：

- `n: 5`
- `opponent_level: 1.0`
- full-match reset 语义

### 6.4 3vs3 benchmark

有 full-match 风格的 3vs3 配置，主要比较：

- `init_n`: 1、2、3
- opponent: lv1、lv2、lv3
- EPV: on/off

这些配置不是 trajectory scenario，因为 `benchmark_mode: full_match` 会强制 `use_custom_start = false`。

### 6.5 3vs3 trajectory scenario

使用：

```text
trajectories/3v3trajectories_right_half_left_nearest_kickable.npz
```

特点：

- `benchmark_mode: scenario`
- `use_custom_start: true`
- easy/medium 起点
- 可控制最近的 1/2/3 名左队球员
- 通常因进球、丢球权、ball out、timeout 终止

### 6.6 front-goal catalog scenario

配置名形如：

```text
parallelr2drl_11vs11scenario_catalog_front-goal-1v1_opp-lv3_goal-only
parallelr2drl_11vs11scenario_catalog_front-goal-4v3_opp-lv3_epv-on
```

可用场景：

- 1v0
- 1v1
- 2v1
- 3v2
- 4v3
- 5v4

虽然文件名中保留 `11vs11scenario_catalog`，但当前代码会根据 `scenarioes.npz` 的 catalog metadata 动态覆盖实际发射人数。

### 6.7 action-space comparison

```text
parallelr2drl_11vs11actionspace_team-base_opp-lv1_epv-off
parallelr2drl_11vs11actionspace_team-hybrid_opp-lv1_epv-off
parallelr2drl_3vs3actionspace_team-base_init-1_start-easy_opp-lv1_epv-off
parallelr2drl_3vs3actionspace_team-hybrid_init-1_start-easy_opp-lv1_epv-off
```

Base 可以直接走默认 QMix 离散动作训练链路。Hybrid 环境侧和 C++ player 侧支持，但训练链路还需要额外适配参数化动作。

## 7. reset 和 terminal

`benchmark_mode` 对 reset 很关键：

- `scenario` 强制 custom start
- `full_match` 强制 normal/default reset
- `actionspace` 按 YAML 保持

terminal 条件由这些字段控制：

- `terminate_on_goal`
- `terminate_on_possession_loss`
- `terminate_on_ball_out`
- `terminate_on_ball_left_half`
- `reward_on_timeout`
- `reward_on_draw`

front-goal scenario 通常开：

```yaml
terminate_on_goal: true
terminate_on_possession_loss: true
terminate_on_ball_out: true
terminate_on_ball_left_half: true
```

full-match 通常不开 goal/possession-loss 终止。

## 8. EPV

EPV 相关文件在：

```text
robocup2d/LaurieOnTracking/
```

默认文件：

- `EPV_grid.csv`
- `EPV_grid_front_goal_linear.csv`

逻辑：

- `useMaxEpv: false` 时只记录 EPV 诊断，不给 EPV reward。
- `useMaxEpv: true` 时，只有左队真实可踢球时才更新 EPV progress。
- `max_episode_epv` 增加才给正奖励。
- 进球时可能补上到最大 EPV 的剩余进展。
- `epv_progress_scale` 控制增量缩放。

## 9. 常用运行命令

QMix demo：

```bash
MAX_ENV_STEPS=1000 ./scripts/run_qmix_demo.sh
```

MAPPO demo：

```bash
MAX_ENV_STEPS=1000 ./scripts/run_mappo_demo.sh
```

ParaDQN demo：

```bash
MAX_ENV_STEPS=1000 ./scripts/run_paradqn_demo.sh
```

全部顺序跑：

```bash
MAX_ENV_STEPS=1000 ./scripts/run_all_demos.sh
```

换前场场景时，QMix/MAPPO 使用不带 `.yaml` 后缀的配置名，ParaDQN 使用带 `.yaml` 后缀的配置名。

## 10. 结果怎么看

full-match 重点看：

- `score_diff`
- `score_left`
- `score_right`
- `timeout`
- restart counters

scenario 重点看：

- `win`
- `score_diff`
- `terminal_reason`
- `possession_loss`
- `ball_out`
- `ball_left_half`

EPV 实验额外看：

- `max_episode_epv`
- `max_epv_improvement`
- `epv_step_reward`
- `epv_goal_completion_reward`

## 11. 排障顺序

1. 确认 Python 环境有 `torch`。
2. 确认 `PYTHONPATH` 指向 `R2DRL` 根目录。
3. 确认 YAML 里的 `server_path`、`player_dir`、`coach_dir`、`trainer_dir` 有效。
4. 确认 `lib_paths` 能找到当前编译的 librcsc。
5. 看 `logs_dir/<run_id>/` 下 server/player/coach/trainer 日志。
6. 如果卡在 reset，看 `trainer_ready_timeout_ms` 和 trainer 日志。
7. 如果卡在 step，看 `playon_timeout`、player 日志和 action-debug 输出。
8. 如果频繁 restart，看 `restart_total`、`restart_recent`、`restart_consecutive`。

## 12. 当前容易误解的点

- front-goal 配置名里有 `11vs11`，但当前实际发射人数由 catalog metadata 覆盖。
- `test_mode` 不决定 default/custom reset，reset 类型主要由 `benchmark_mode/use_custom_start` 决定。
- `trajectory_path` 只是起点数据来源；真正是否使用 custom reset 还要看 `use_custom_start`。
- `useMaxEpv` 不等于使用 EPV 文件。EPV 文件会加载用于诊断，但只有 `useMaxEpv: true` 才给 EPV reward。
- Hybrid 不是不能用，而是默认 QMix 训练链路还没有完整适配参数化动作。

