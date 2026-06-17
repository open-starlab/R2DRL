#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
GITHUB_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
source "$SCRIPT_DIR/prepare_runtime.sh"

ENV_CONFIG=${ENV_CONFIG:-parallelr2drl_11vs11scenario_catalog_front-goal-1v1_opp-lv3_goal-only}
QMIX_CONFIG=${QMIX_CONFIG:-qmix}
SEED=${SEED:-0}
NUM_ENVS=${NUM_ENVS:-1}
MAX_ENV_STEPS=${MAX_ENV_STEPS:-1000}
PORT_START=${PORT_START:-6000}
PORT_END=${PORT_END:-8000}
RUN_NAME=${RUN_NAME:-demo_qmix_${ENV_CONFIG}_seed${SEED}}
USE_CUDA=${USE_CUDA:-False}

cd "$GITHUB_ROOT/algorithm/qmix/source"
mkdir -p log results runs

echo "[INFO] Running QMix demo: $ENV_CONFIG"
python -u main.py \
    --config="$QMIX_CONFIG" \
    --env-config="$ENV_CONFIG" \
    --capture=no \
    seed="$SEED" \
    env_args.seed="$SEED" \
    env_args.auto_port_start="$PORT_START" \
    env_args.auto_port_end="$PORT_END" \
    env_args.game_logging=False \
    name="$RUN_NAME" \
    batch_size_run="$NUM_ENVS" \
    test_nepisode=1 \
    t_max="$MAX_ENV_STEPS" \
    test_interval="$MAX_ENV_STEPS" \
    log_interval=100 \
    runner_log_interval=100 \
    learner_log_interval=100 \
    buffer_warmup=1 \
    batch_size=1 \
    buffer_size=8 \
    save_model=False \
    use_cuda="$USE_CUDA" \
    ${QMIX_EXTRA_ARGS:-}
