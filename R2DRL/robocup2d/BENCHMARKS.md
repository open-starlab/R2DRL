# RoboCup2D Benchmarks

This environment now supports two benchmark episode styles:

- `full-match`: keep playing after goals and report final `score_diff` at timeout.
- `scenario`: terminate on goal or timeout and report `success`.

## 1. Full-match Benchmark (11vs11)

Goal: evaluate difficulty scaling and reward shaping.

Shared setup:

- 11vs11
- normal kickoff
- `episode_limit: 3000`
- metric: `score_diff_mean`

Variants:

- Opponent difficulty:
  - lv1: `--env-config=parallelr2drl_11vs11fullmatch_team-base_opp-lv1_epv-off` with baked-in `opponent_level=0.05`
  - lv2: `--env-config=parallelr2drl_11vs11fullmatch_team-base_opp-lv2_epv-off` with baked-in `opponent_level=0.5`
  - lv3: `--env-config=parallelr2drl_11vs11fullmatch_team-base_opp-lv3_epv-off` with baked-in `opponent_level=1.0`
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

- easy: `--env-config=parallelr2drl_3vs3scenario_team-base_init-1_start-easy_opp-lv1_epv-off` with baked-in `opponent_level=0.05`
- medium: `--env-config=parallelr2drl_3vs3scenario_team-base_init-1_start-medium_opp-lv1_epv-off` with baked-in `opponent_level=0.05`

Expected logs:

- `test_success_mean`
- `test_possession_loss_mean`
- `test_timeout_mean`

## 3. Action-space Comparison

Goal: compare action-space choices under the same benchmark conditions across environment scales.

Shared setup:

- lv1 opponent with baked-in `opponent_level=0.05`
- compare `Base` vs `Hybrid`

Variants:

- 11vs11 full match:
  - base action space: `--env-config=parallelr2drl_11vs11actionspace_team-base_opp-lv1_epv-off`
  - hybrid action space: `--env-config=parallelr2drl_11vs11actionspace_team-hybrid_opp-lv1_epv-off`
- 3vs3 scenario:
  - base action space: `--env-config=parallelr2drl_3vs3actionspace_team-base_init-1_start-easy_opp-lv1_epv-off`
  - hybrid action space: `--env-config=parallelr2drl_3vs3actionspace_team-hybrid_init-1_start-easy_opp-lv1_epv-off`

Note:

- The RoboCup2D environment already exposes both action spaces.
- The 3vs3 action-space presets keep the same conditions as `parallelr2drl_3vs3scenario_team-base_init-1_start-easy_opp-lv1_epv-off`: easy start, lv1 opponent, `episode_limit: 300`.
- The default PyMARL training pipeline in this repository still stores actions as a single discrete index, so the `hybrid` benchmark preset is environment-ready but not yet end-to-end trainable without controller, buffer, and runner changes.

Suggested command pattern:

```bash
cd /fsws1/h_qin/robocup/robocup/pymarl/src
python main.py --config=qmix --env-config=parallelr2drl_11vs11fullmatch_team-base_opp-lv1_epv-off
```

For evaluation, use the same `--env-config` together with your normal checkpoint-loading flags. You do not need to pass an extra difficulty override each time because the benchmark presets already bake in the lv1/lv2/lv3 `opponent_level` values. The runner will now emit benchmark-friendly terminal stats directly from the environment.
