# R2DRL RoboCup2D Reinforcement Learning Bundle

This repository is a self-contained RoboCup2D reinforcement-learning bundle for QMix, MAPPO, and ParaDQN experiments.

Last refresh: 2026-06-17.

## What Is Included

- `algorithm/qmix/source/`: QMix/PyMARL training source.
- `algorithm/mappo/source/`: MAPPO/on-policy training source with the RoboCup2D adapter.
- `algorithm/paradqn/source/`: ParaDQN training source with the RoboCup2D adapter.
- `R2DRL/robocup2d/`: canonical RoboCup2D Python environment package and YAML config set.
- `R2DRL/helios-base/`: modified HELIOS player/coach/trainer source plus bundled runtime binaries.
- `R2DRL/librcsc/`: modified librcsc source plus bundled `librcsc.so` runtime library.
- `R2DRL/rcssserver/`: rcssserver source plus bundled runtime server binary and local `librcss*.so` libraries.
- `R2DRL/runtime_libs/`: C++ runtime libraries used by the bundled binaries.
- `scripts/`: local path preparation, simulator build helper, and three algorithm demo runners.

Large training outputs, logs, checkpoints, caches, and generated Python bytecode are excluded.

## Quick Start

From the repository root:

```bash
python -m pip install -r requirements-demo.txt
./scripts/prepare_runtime.sh
MAX_ENV_STEPS=1000 ./scripts/run_qmix_demo.sh
MAX_ENV_STEPS=1000 ./scripts/run_mappo_demo.sh
MAX_ENV_STEPS=1000 ./scripts/run_paradqn_demo.sh
```

Run all three demos sequentially:

```bash
MAX_ENV_STEPS=1000 ./scripts/run_all_demos.sh
```

The demo scripts use the bundled front-goal `1v1` goal-only task by default:

```text
parallelr2drl_11vs11scenario_catalog_front-goal-1v1_opp-lv3_goal-only
```

## Why `prepare_runtime.sh` Matters

The original experiment configs contain machine-specific paths. `scripts/prepare_runtime.sh` calls `scripts/localize_paths.py`, which rewrites every RoboCup2D YAML under this checkout so runtime fields point inside the current repository:

- `server_path`
- `player_dir`, `coach_dir`, `trainer_dir`
- `config_dir`, `player_config`
- `lib_paths`
- scenario `trajectory_path`
- TensorBoard log directories

This is what makes the same checkout movable to another path.

## Demo Environment Variables

All demo scripts accept simple overrides:

```bash
ENV_CONFIG=parallelr2drl_11vs11scenario_catalog_front-goal-2v1_opp-lv3_goal-only \
SEED=3 \
NUM_ENVS=1 \
MAX_ENV_STEPS=5000 \
./scripts/run_qmix_demo.sh
```

For ParaDQN, include the `.yaml` suffix when overriding `ENV_CONFIG`:

```bash
ENV_CONFIG=parallelr2drl_11vs11scenario_catalog_front-goal-2v1_opp-lv3_goal-only.yaml \
./scripts/run_paradqn_demo.sh
```

Port ranges are separated by default:

- QMix: `6000-8000`
- MAPPO: `6100-8100`
- ParaDQN: `6200-8200`

Override `PORT_START` and `PORT_END` if those ports are busy.

## Native Runtime

The demo scripts use bundled native binaries and libraries:

- `R2DRL/rcssserver/build/rcssserver`
- `R2DRL/rcssserver/build/rcss/{clang,conf,net,gzip}/librcss*.so*`
- `R2DRL/helios-base/src/player/sample_player`
- `R2DRL/helios-base/src/coach/sample_coach`
- `R2DRL/helios-base/src/trainer/sample_trainer`
- `R2DRL/librcsc/rcsc/.libs/librcsc.so.19`
- `R2DRL/runtime_libs/libstdc++.so.6`, `libgcc_s.so.1`, `libz.so.1`

If the bundled Linux binaries are incompatible with another machine, rebuild from source:

```bash
./scripts/build_simulators.sh
./scripts/prepare_runtime.sh
```

## Documentation

- `TRAINING_DEMOS.md`: how to run the three algorithm demos.
- `R2DRL/README.md`: runtime/environment architecture and source mapping.
- `R2DRL/robocup2d/README.md`: environment details and config families.
- `R2DRL/robocup2d/BENCHMARKS.md`: benchmark/config matrix.
- `R2DRL/robocup2d/GUIDE_CN.md`: Chinese operational guide.
- `R2DRL/robocup2d/ENV_USAGE_CN.md`: Chinese environment usage notes.
