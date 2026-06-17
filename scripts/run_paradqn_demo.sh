#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
GITHUB_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
source "$SCRIPT_DIR/prepare_runtime.sh"

ENV_CONFIG=${ENV_CONFIG:-parallelr2drl_11vs11scenario_catalog_front-goal-1v1_opp-lv3_goal-only.yaml}
SEED=${SEED:-0}
NUM_ENVS=${NUM_ENVS:-1}
MAX_ENV_STEPS=${MAX_ENV_STEPS:-1000}
PORT_START=${PORT_START:-6200}
PORT_END=${PORT_END:-8200}
RUN_NAME=${RUN_NAME:-demo_paradqn_${ENV_CONFIG%.yaml}_seed${SEED}}
DEVICE=${DEVICE:-cpu}

ENV_YAML="$GITHUB_ROOT/algorithm/paradqn/source/environments/robocup2d/config/$ENV_CONFIG"
cd "$GITHUB_ROOT/algorithm/paradqn/source"
mkdir -p runs checkpoints

echo "[INFO] Running ParaDQN demo: $ENV_CONFIG"
python -u train_robocup.py \
    --seed "$SEED" \
    --device "$DEVICE" \
    --num_envs "$NUM_ENVS" \
    --env-config "$ENV_YAML" \
    --auto_port_start "$PORT_START" \
    --auto_port_end "$PORT_END" \
    --use-action-mask True \
    --run-name "$RUN_NAME" \
    --max_env_steps "$MAX_ENV_STEPS" \
    --train_episodes 1000000 \
    --eval_episodes 1 \
    --eval_interval_steps "$MAX_ENV_STEPS" \
    --save_interval_steps 0 \
    ${PARADQN_EXTRA_ARGS:-}
