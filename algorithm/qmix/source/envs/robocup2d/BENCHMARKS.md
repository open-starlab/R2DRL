# RoboCup2D Benchmark and Config Matrix

This document describes the current YAML presets in `robocup2d/config/`. It reflects the 2026-06-17 R2DRL mirror.

The config directory currently contains 69 YAML files.

## Benchmark Modes

`benchmark_mode` is read by `EnvConfig` and `Robocup2dEnv._normalize_reset_mode()`.

| mode | reset behavior | typical use |
|---|---|---|
| `scenario` | forces `use_custom_start = true` | trajectory starts and front-goal catalog starts |
| `full_match` | forces `use_custom_start = false` | normal kickoff/full-match style evaluation |
| `actionspace` | keeps YAML `use_custom_start` | Base vs Hybrid comparison |

The environment returns benchmark-friendly fields in `info`: `score_diff`, `win`, `lose`, `timeout`, `ball_out`, `ball_left_half`, `possession_loss`, `terminal_reason`, `max_episode_epv`, and restart counters.

## Opponent Levels

Current convention:

| label | `opponent_level` |
|---|---:|
| `lv1` | 0.05 |
| `lv2` | 0.5 |
| `lv3` | 1.0 |

The launcher passes this value only to right-side players as `--level`.

## 11vs11 Full-Match / Benchmark

These configs use normal match reset semantics: `benchmark_mode: full_match`, `use_custom_start: false`, `terminate_on_goal: false`, and `terminate_on_possession_loss: false`.

Primary families:

| family | variants |
|---|---|
| `parallelr2drl_11vs11fullmatch_team-base_opp-lv*_epv-{off,on}` | lv1/lv2/lv3, EPV off/on |
| `parallelr2drl_11vs11benchmark_base_opp-lv*_epv-{off,on}` | lv1/lv2/lv3, EPV off/on |
| `parallelr2drl_11vs11benchmark_hybrid_opp-lv1_epv-on` | Hybrid environment preset |

Example:

```bash
python -u main.py --config=qmix --env-config=parallelr2drl_11vs11fullmatch_team-base_opp-lv3_epv-on --capture=no
```

Recommended metrics:

- `score_diff_mean`
- `score_left_mean`
- `score_right_mean`
- `max_episode_epv_mean`
- `restart_total` and terminal-reason counts for stability checks

## 5vs5 Benchmark

Current 5vs5 presets are lv3 Base full-match variants:

| config | EPV |
|---|---|
| `parallelr2drl_5vs5benchmark_base_opp-lv3_epv-off` | off |
| `parallelr2drl_5vs5benchmark_base_opp-lv3_epv-on` | on |

Example:

```bash
python -u main.py --config=qmix --env-config=parallelr2drl_5vs5benchmark_base_opp-lv3_epv-off --capture=no
```

## 3vs3 Full-Match Benchmark

These are full-match style 3vs3 presets, not trajectory-start scenarios. They use `benchmark_mode: full_match` and `use_custom_start: false`.

Available dimensions:

- `init_n`: 1, 2, or 3 for controlled left-side agents.
- opponent: lv1, lv2, lv3 where configs exist.
- EPV: off/on where configs exist.

Examples:

```bash
python -u main.py --config=qmix --env-config=parallelr2drl_3vs3benchmark_base_init-1_opp-lv1_epv-off --capture=no
python -u main.py --config=qmix --env-config=parallelr2drl_3vs3benchmark_base_opp-lv3_epv-on --capture=no
```

## 3vs3 Trajectory Scenarios

These configs use the prefiltered 3v3 trajectory dataset:

```text
trajectories/3v3trajectories_right_half_left_nearest_kickable.npz
```

They use `benchmark_mode: scenario`, so the code forces `use_custom_start = true`. Termination usually includes goal, possession loss, and ball out.

Available dimensions:

- `init_n`: 1, 2, 3.
- `scenario_difficulty`: `easy` or `medium` in current presets.
- opponent: lv1 and selected lv3 presets.
- EPV: off/on where configs exist.
- naming style: both `..._base_...` and `..._team-base_...` exist; the actual action space comes from YAML field `team: Base`.

Examples:

```bash
python -u main.py --config=qmix --env-config=parallelr2drl_3vs3scenario_base_init-3_start-medium_opp-lv3_epv-on --capture=no
python -u main.py --config=qmix --env-config=parallelr2drl_3vs3scenario_team-base_init-1_start-easy_opp-lv1_epv-off --capture=no
```

## Front-Goal Catalog Scenarios

These configs are named `parallelr2drl_11vs11scenario_catalog_front-goal-*`, but they do not launch a full 11v11 player set in practice. `start_catalog_enabled: true` lets `apply_catalog_launch_profile()` read `scenarioes.npz` metadata and adjust the launched team sizes.

Current catalog ids:

| start id | preset variants |
|---|---|
| `front_goal_1v0` | goal-only, EPV on, EPV linear |
| `front_goal_1v1` | goal-only, EPV on |
| `front_goal_2v1` | goal-only, EPV on |
| `front_goal_3v2` | goal-only, EPV on |
| `front_goal_4v3` | goal-only, EPV on |
| `front_goal_5v4` | goal-only, EPV on |

Common settings:

- `benchmark_mode: scenario`
- `opponent_state: lv3`
- `opponent_level: 1.0`
- `terminate_on_goal: true`
- `terminate_on_possession_loss: true`
- `terminate_on_ball_out: true`
- `terminate_on_ball_left_half: true`
- `freeze_non_controlled: true` is applied by the catalog launch profile

Examples:

```bash
python -u main.py --config=qmix --env-config=parallelr2drl_11vs11scenario_catalog_front-goal-1v1_opp-lv3_goal-only --capture=no
python -u main.py --config=qmix --env-config=parallelr2drl_11vs11scenario_catalog_front-goal-4v3_opp-lv3_epv-on --capture=no
```

EPV variants:

- `goal-only`: `useMaxEpv: false`; reward mainly comes from goal/terminal logic.
- `epv-on`: `useMaxEpv: true`; uses default `EPV_grid.csv` and `epv_progress_scale: 2.0` in the current catalog presets.
- `epv-linear`: currently present for 1v0; uses `EPV_grid_front_goal_linear.csv`.

## Action-Space Comparison

These configs are used to compare environment action interfaces:

| config | scale | team | custom start |
|---|---|---|---|
| `parallelr2drl_11vs11actionspace_team-base_opp-lv1_epv-off` | 11vs11 | Base | false |
| `parallelr2drl_11vs11actionspace_team-hybrid_opp-lv1_epv-off` | 11vs11 | Hybrid | false |
| `parallelr2drl_3vs3actionspace_team-base_init-1_start-easy_opp-lv1_epv-off` | 3vs3 | Base | true |
| `parallelr2drl_3vs3actionspace_team-hybrid_init-1_start-easy_opp-lv1_epv-off` | 3vs3 | Hybrid | true |

Base is compatible with the current default QMix discrete-action pipeline. Hybrid is supported by the environment and C++ player shared-memory interface, but a full policy-training stack must also handle parameterized actions in the controller, replay buffer, runner, and learner.

## Base Entry Points

| config | purpose |
|---|---|
| `robocup.yaml` | direct `Robocup2dEnv` smoke/debug entry |
| `r2drl.yaml` | single-env wrapper entry |
| `parallelr2drl.yaml` | PyMARL parallel wrapper entry |

The base entry points are useful for development, but experiments should usually use an explicit benchmark/scenario config so the reset and terminal semantics are obvious.

## Adding a New Preset

1. Copy the nearest existing YAML.
2. Keep `benchmark_mode` consistent with reset semantics.
3. Update `tb_log_dir` and any path fields.
4. For front-goal catalog starts, set `start_catalog_enabled: true` and one of `start_id` or `start_index`.
5. For trajectory difficulty windows, set `scenario_difficulty` and provide `scenario_difficulty_buckets`.
6. Check `init_n <= n1`, `opponent_level` in `[0, 1]`, and valid `active_right_unums` if provided.

## Evaluation Notes

For goal-oriented scenarios, look at:

- `win_mean`
- `score_diff_mean`
- `terminal_reason` distribution
- `possession_loss_mean`
- `ball_out_mean`
- `ball_left_half_mean`

For full-match configs, look at:

- `score_diff_mean`
- `score_left_mean`
- `score_right_mean`
- `timeout_mean`
- restart counters

For EPV configs, also compare:

- `max_episode_epv_mean`
- `max_epv_improvement_mean`
- `epv_step_reward_mean`
- `epv_goal_completion_reward_mean`

