# Training Demos

This checkout includes runnable demo entry points for QMix, MAPPO, and ParaDQN. All three use the same bundled RoboCup2D runtime under `R2DRL/`.

## One-Time Setup

```bash
cd <repo-root>
python -m pip install -r requirements-demo.txt
./scripts/prepare_runtime.sh
```

`prepare_runtime.sh` rewrites YAML paths to the current checkout and checks the bundled simulator/player libraries.

## QMix

```bash
MAX_ENV_STEPS=1000 ./scripts/run_qmix_demo.sh
```

Useful overrides:

```bash
ENV_CONFIG=parallelr2drl_11vs11scenario_catalog_front-goal-2v1_opp-lv3_goal-only \
SEED=1 \
NUM_ENVS=1 \
MAX_ENV_STEPS=5000 \
./scripts/run_qmix_demo.sh
```

## MAPPO

```bash
MAX_ENV_STEPS=1000 ./scripts/run_mappo_demo.sh
```

The MAPPO script uses `algorithm/mappo/source/onpolicy/scripts/train/train_robocup2d.py` and points `--pymarl_src` to the bundled QMix source because the MAPPO adapter reuses the canonical `envs.robocup2d` config loader.

## ParaDQN

```bash
MAX_ENV_STEPS=1000 ./scripts/run_paradqn_demo.sh
```

ParaDQN keeps its own RoboCup2D adapter copy under `algorithm/paradqn/source/environments/robocup2d`. Use the `.yaml` suffix when overriding configs:

```bash
ENV_CONFIG=parallelr2drl_11vs11scenario_catalog_front-goal-2v1_opp-lv3_goal-only.yaml \
MAX_ENV_STEPS=5000 \
./scripts/run_paradqn_demo.sh
```

## All Three

```bash
MAX_ENV_STEPS=1000 ./scripts/run_all_demos.sh
```

Default task:

```text
parallelr2drl_11vs11scenario_catalog_front-goal-1v1_opp-lv3_goal-only
```

Default port ranges:

- QMix: `6000-8000`
- MAPPO: `6100-8100`
- ParaDQN: `6200-8200`

Set `PORT_START` and `PORT_END` for an individual script, or `QMIX_PORT_START`, `MAPPO_PORT_START`, and `PARADQN_PORT_START` for `run_all_demos.sh`.

## Outputs

The scripts create local output directories inside each algorithm source tree, such as `log/`, `runs/`, `results/`, and `checkpoints/`. These are runtime outputs and should normally stay untracked.
