#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
GITHUB_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
source "$SCRIPT_DIR/prepare_runtime.sh"

ENV_CONFIG=${ENV_CONFIG:-parallelr2drl_11vs11scenario_catalog_front-goal-1v1_opp-lv3_goal-only}
SEED=${SEED:-0}
NUM_ENVS=${NUM_ENVS:-1}
MAX_ENV_STEPS=${MAX_ENV_STEPS:-1000}
PORT_START=${PORT_START:-6100}
PORT_END=${PORT_END:-8100}
RUN_NAME=${RUN_NAME:-demo_mappo_${ENV_CONFIG}_seed${SEED}}
ALGORITHM=${ALGORITHM:-rmappo}

ENV_YAML="$GITHUB_ROOT/algorithm/qmix/source/envs/robocup2d/config/${ENV_CONFIG}.yaml"
cd "$GITHUB_ROOT/algorithm/mappo/source"
mkdir -p onpolicy/scripts/results

echo "[INFO] Running MAPPO demo: $ENV_CONFIG"
python -u onpolicy/scripts/train/train_robocup2d.py \
    --algorithm_name "$ALGORITHM" \
    --experiment_name "$RUN_NAME" \
    --seed "$SEED" \
    --n_training_threads 1 \
    --n_rollout_threads "$NUM_ENVS" \
    --n_eval_rollout_threads 1 \
    --num_env_steps "$MAX_ENV_STEPS" \
    --episode_length 300 \
    --use_eval \
    --eval_interval "$MAX_ENV_STEPS" \
    --eval_episodes 1 \
    --save_interval "$MAX_ENV_STEPS" \
    --use_wandb \
    --cuda \
    --robocup_env_config "$ENV_YAML" \
    --pymarl_src "$GITHUB_ROOT/algorithm/qmix/source" \
    --robocup_auto_port_start "$PORT_START" \
    --robocup_auto_port_end "$PORT_END" \
    --robocup_use_action_mask True \
    ${MAPPO_EXTRA_ARGS:-}
