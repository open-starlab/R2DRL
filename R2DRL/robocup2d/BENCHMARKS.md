# RoboCup2D Benchmarks

This environment now supports two benchmark episode styles:

- `full-match`: keep playing after goals and report final `score_diff` at timeout.
- `scenario`: terminate on goal, possession loss, or timeout and report `success`.

## 1. Full-match Benchmark (11vs11)

Goal: evaluate difficulty scaling and reward shaping.

Shared setup:

- 11vs11
- normal kickoff
- `episode_limit: 3000`
- metric: `score_diff_mean`

Variants:

- Opponent difficulty:
  - easy: `--env-config=robocup_benchmark_full_match_easy` with baked-in `opponent_level=0.3`
  - medium: `--env-config=robocup_benchmark_full_match_medium` with baked-in `opponent_level=0.6`
  - hard: `--env-config=robocup_benchmark_full_match_hard` with baked-in `opponent_level=0.9`
- Reward:
  - scoring reward only: `*_full_match_{easy|medium|hard}`
  - scoring reward + MaxEPV: `*_full_match_{easy|medium|hard}_epv`

Expected logs:

- `test_score_diff_mean`
- `test_score_left_mean`
- `test_score_right_mean`
- `test_max_episode_epv_mean`

## 2. Scenario Benchmark (3vs3)

Goal: evaluate progressive difficulty and fast iteration.

Shared setup:

- curriculum starts from goal to kickoff
- control the 1 to 3 nearest players via the existing agent mask
- terminate on:
  - goal scored
  - defending team gains possession
  - `300` frames
- no extra penalty after possession loss
- metric: `success_mean`

Variants:

- easy: `--env-config=r2drl_benchmark_scenario_easy` with baked-in `opponent_level=0.3`
- medium: `--env-config=r2drl_benchmark_scenario_medium` with baked-in `opponent_level=0.6`
- hard: `--env-config=r2drl_benchmark_scenario_hard` with baked-in `opponent_level=0.9`

Expected logs:

- `test_success_mean`
- `test_possession_loss_mean`
- `test_timeout_mean`

## 3. Action-space Comparison

Goal: compare action-space choices under the same full-match benchmark.

Shared setup:

- easy opponent with baked-in `opponent_level=0.3`
- 11vs11 full match
- `episode_limit: 3000`

Variants:

- base action space: `--env-config=robocup_benchmark_actionspace_easy_base`
- hybrid action space: `--env-config=robocup_benchmark_actionspace_easy_hybrid`

Note:

- The RoboCup2D environment already exposes both action spaces.
- The default PyMARL training pipeline in this repository still stores actions as a single discrete index, so the `hybrid` benchmark preset is environment-ready but not yet end-to-end trainable without controller, buffer, and runner changes.

Suggested command pattern:

```bash
cd /fsws1/h_qin/robocup/robocup/pymarl/src
python main.py --config=qmix --env-config=robocup_benchmark_full_match_easy
```

For evaluation, use the same `--env-config` together with your normal checkpoint-loading flags. You do not need to pass an extra difficulty override each time because the benchmark presets already bake in the easy/medium/hard `opponent_level` values. The runner will now emit benchmark-friendly terminal stats directly from the environment.
