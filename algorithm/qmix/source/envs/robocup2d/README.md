# RoboCup 2D RL Environment

`robocup2d` is the Python environment package used by the current R2DRL/QMix-style RoboCup2D experiments. It wraps a synchronous `rcssserver` session, HELIOS-based player processes, an online coach, and a trainer behind a PyMARL-compatible API.

Last documentation refresh: 2026-06-17.

## Entry Points

The package exposes three classes from `robocup2d/__init__.py`:

- `Robocup2dEnv`: the low-level environment that launches simulator processes and owns shared memory.
- `R2DRL`: a single-environment wrapper that can sample trajectory starts before reset.
- `ParallelR2DRL`: a PyMARL parallel-runner wrapper; the runner is expected to send sampled starts with `set_start_and_n`.

Typical import:

```python
from robocup2d import Robocup2dEnv, R2DRL, ParallelR2DRL
```

## Package Layout

- `env.py`: environment classes, reset/step logic, rewards, terminals, stats.
- `runtime.py`: run ids, port locks, process lifecycle, restart, cleanup, log directories.
- `agents.py`: shared-memory reads/writes, actions, observations, masks, EPV, custom reset payloads.
- `config/loader.py`: YAML loading. Relative config names resolve against `robocup2d/config/`.
- `config/schema.py`: validation and normalized environment fields.
- `config/*.yaml`: 69 current environment presets.
- `ipc/`: shared-memory creation and handshake helpers.
- `process/`: launch, kill, port, lock, and watchdog helpers.
- `protocols/`: binary layouts for player, coach, and trainer shared-memory buffers.
- `start_sampler.py`: trajectory and catalog scenario sampling.
- `trajectories/`: mirrored scenario datasets and front-goal catalog image/data.
- `LaurieOnTracking/`: EPV grids and original EPV helper scripts.

## Requirements

Python-side dependencies for the bundled demos are listed at the repository root:

```bash
cd <repo-root>
python -m pip install -r requirements-demo.txt
```

The simulator-side runtime is bundled under `R2DRL/`:

- `R2DRL/rcssserver/build/rcssserver`
- `R2DRL/helios-base/src/player/sample_player`
- `R2DRL/helios-base/src/coach/sample_coach`
- `R2DRL/helios-base/src/trainer/sample_trainer`
- `R2DRL/librcsc/rcsc/.libs/librcsc.so.19`
- `R2DRL/runtime_libs/`

Before running training, localize YAML paths to the current checkout:

```bash
cd <repo-root>
./scripts/prepare_runtime.sh
```

The current package layout is a source tree. Use `PYTHONPATH` rather than relying on editable install metadata:

```bash
export PYTHONPATH="$PWD/R2DRL:$PWD/algorithm/qmix/source:${PYTHONPATH:-}"
```

## Configuration Loading

`Robocup2dEnv(cfg="robocup.yaml")` resolves `robocup.yaml` relative to `robocup2d/config/`. Absolute YAML paths are used directly.

A YAML file can either store fields at the root or under `env_args`. The current configs use `env_args`.

Important fields:

- `n`, `n1`, `n2`: symmetric or asymmetric team sizes. If `n1/n2` are omitted, both default to `n`.
- `init_n`: number of controlled left-side agents exposed to PyMARL.
- `team`: `Base` or `Hybrid` action interface.
- `opponent_state`, `opponent_level`: human label and actual opponent difficulty. Team 2 receives `--level <opponent_level>`.
- `benchmark_mode`: `scenario`, `full_match`, or `actionspace`.
- `use_custom_start`: whether reset uses trainer-provided custom positions.
- `trajectory_path`: `.npz` start dataset for trajectory or catalog scenarios.
- `start_catalog_enabled`, `start_id`, `start_index`: front-goal catalog controls.
- `useMaxEpv`, `epv_grid_file`, `epv_progress_reward`, `epv_progress_scale`: EPV reward settings.
- `terminate_on_goal`, `terminate_on_possession_loss`, `terminate_on_ball_out`, `terminate_on_ball_left_half`: terminal conditions.
- `server_path`, `player_dir`, `coach_dir`, `trainer_dir`, `config_dir`, `player_config`, `lib_paths`: simulator binary/source paths.

Run `scripts/prepare_runtime.sh` from the repository root to rewrite these runtime paths for the current checkout.

## Reset Semantics

`Robocup2dEnv` normalizes reset behavior from `benchmark_mode`:

- `scenario`: forces `use_custom_start = true`.
- `full_match`: forces `use_custom_start = false`.
- `actionspace`: leaves `use_custom_start` as written in YAML.

During `reset()`:

- If a restart is needed or a process died, `Runtime` restarts the session.
- If custom starts are enabled, `Agents.reset_custom()` sends ball/player/body-angle payloads through trainer shared memory.
- If custom starts are disabled and the previous terminal was `init`, `goal_scored`, or `goal_conceded`, the code can keep the server's natural kickoff/goal restart flow.
- Otherwise `Agents.reset_default()` requests a default trainer reset and alternates kickoff side on timeout-driven default resets.

The reset return `info` includes `turn_count`, scores, initial/max EPV, and restart counters.

## Step Semantics

`step(actions)` accepts a NumPy array or torch tensor.

Base action mode expects one discrete action per controlled agent. Current action ids are:

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

Hybrid action mode exposes six action heads: `turn`, `dash`, `kick`, `catch`, `helios`, and `wait`, with two continuous parameters in shared memory. The environment side supports this, but the default PyMARL QMix stack remains discrete-action oriented.

After actions are written, the environment waits for the shared-memory ready barrier, updates the active-agent mask, reads the coach goal flag and ball state, computes reward, caches `obs/state/avail_actions`, and returns:

```python
reward: float
done: bool
info: dict
```

`info` includes:

- `win`, `lose`, `timeout`, `episode_limit`
- `score_left`, `score_right`, `score_diff`
- `ball_out`, `ball_left_half`, `possession_loss`
- `initial_episode_epv`, `max_episode_epv`, `max_epv_improvement`
- `epv_step_reward`, `epv_goal_completion_reward`, `epv_progress_scale`, `useMaxEpv`
- `terminal_reason`
- `restart_total`, `restart_recent`, `restart_consecutive`

## Observations, State, and Masks

`get_env_info()` returns:

```python
{
    "n_agents": init_n clipped to n1,
    "n_actions": 19 for Base or 6 for Hybrid,
    "state_shape": coach global-state length,
    "obs_shape": player observation length,
    "episode_limit": configured episode limit,
}
```

`get_obs()` returns only the active left-side controlled agents. `Agents.set_agent_mask()` selects the `init_n` nearest left-side players to the ball. `get_avail_actions()` returns the action mask for those active agents. If `use_action_mask: false`, all non-empty actions are exposed according to the no-mask fallback.

For front-goal catalog starts, `freeze_non_controlled: true` can be set automatically. Non-controlled players then receive empty/default fallback actions rather than normal RL-controlled actions.

## Scenario Starts

There are two supported start sources.

Trajectory scenarios use the legacy 3v3 dataset:

```text
trajectories/3v3trajectories_right_half_left_nearest_kickable.npz
```

The sampler can select by `scenario_difficulty` (`easy`, `medium`, `hard`) using `scenario_difficulty_buckets`. Difficulty windows are defined only for 3v3 trajectories.

Front-goal catalog scenarios use:

```text
trajectories/scenarioes.npz
```

With `start_catalog_enabled: true`, `start_id` or `start_index` selects catalog entries such as `front_goal_1v0`, `front_goal_1v1`, `front_goal_2v1`, `front_goal_3v2`, `front_goal_4v3`, and `front_goal_5v4`. `apply_catalog_launch_profile()` can adjust `n`, `n1`, `n2`, `init_n`, `active_right_unums`, and `freeze_non_controlled` based on catalog metadata.

## EPV Reward

`Agents` always loads an EPV grid when possible for diagnostics. Reward shaping is active only when `useMaxEpv: true`.

Behavior:

- Reset stores the starting EPV as `initial_episode_epv` and `max_episode_epv`.
- During play, if `useMaxEpv` is true, EPV progress is counted only when the left team is truly kickable.
- Reward is the positive increase in `max_episode_epv`, multiplied by `epv_progress_scale`, unless `epv_progress_reward` is positive, in which case that fixed progress reward is used.
- On a left-team goal, `complete_episode_epv()` can add the remaining progress from current max EPV to the grid maximum.
- Goal reward is still `+1`; conceded goal reward is `-1`.

Default EPV grid: `LaurieOnTracking/EPV_grid.csv`.
Front-goal linear variant: `LaurieOnTracking/EPV_grid_front_goal_linear.csv`.

## Common Commands

Import-only smoke test:

```bash
cd <repo-root>
./scripts/prepare_runtime.sh
PYTHONPATH="$PWD/R2DRL:$PWD/algorithm/qmix/source:${PYTHONPATH:-}" python - <<'PY'
from robocup2d import Robocup2dEnv
print("import ok", Robocup2dEnv)
PY
```

Three short training demos:

```bash
MAX_ENV_STEPS=1000 ./scripts/run_qmix_demo.sh
MAX_ENV_STEPS=1000 ./scripts/run_mappo_demo.sh
MAX_ENV_STEPS=1000 ./scripts/run_paradqn_demo.sh
```

Run all three in sequence:

```bash
MAX_ENV_STEPS=1000 ./scripts/run_all_demos.sh
```

QMix direct command from the bundled source tree:

```bash
cd <repo-root>/algorithm/qmix/source
python -u main.py \
  --config=qmix \
  --env-config=parallelr2drl_11vs11scenario_catalog_front-goal-1v1_opp-lv3_goal-only \
  --capture=no \
  t_max=1000 \
  batch_size_run=1 \
  use_cuda=False
```

## Debugging

Useful places to check:

- `logs_dir/<run_id>/`: server/player/coach/trainer logs and `rcg/` game logs.
- `ROBOCUP_ACTION_DEBUG=0`: disables player action-debug prints.
- `ROBOCUP_ACTION_DEBUG_LIMIT` and `ROBOCUP_ACTION_DEBUG_EVERY`: throttle action-debug output.
- `wait_ready_timeout`, `playon_timeout`, `trainer_ready_timeout_ms`: reset and step timeout controls.
- `restart_total`, `restart_recent`, `restart_consecutive`: restart storm diagnostics returned in `info` and `get_stats()`.

