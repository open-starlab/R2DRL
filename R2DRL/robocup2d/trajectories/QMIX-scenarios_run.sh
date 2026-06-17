#!/usr/bin/env bash
# Submit front-goal QMIX runs for the scenario ladder with one shared parameter set.
# Usage:
#   bash QMIX-scenarios_run.sh
#   DRY_RUN=1 bash QMIX-scenarios_run.sh
#   SEEDS="0 1 2" SCENARIOS="1v0 2v1 3v2 4v3" bash QMIX-scenarios_run.sh

set -euo pipefail

FRONT_GOAL_LADDER_DIR=${FRONT_GOAL_LADDER_DIR:-/fsws1/h_qin/robocup/robocup/experiments/front_goal_ladder}
# shellcheck disable=SC1091
source "$FRONT_GOAL_LADDER_DIR/configs.sh"

ALGO=qmix
SCENARIOS=${SCENARIOS:-"1v0 1v1 2v1 3v2 4v3"}
REWARD_MODE=${REWARD_MODE:-goal-only}
SEEDS=${SEEDS:-0}
NUM_ENVS=${NUM_ENVS:-10}
CPUS_PER_TASK=${CPUS_PER_TASK:-16}
GPUS_PER_TASK=${GPUS_PER_TASK:-0}
MEM_PER_JOB=${MEM_PER_JOB:-64G}
TEST_NEPI=${TEST_NEPI:-$NUM_ENVS}
DRY_RUN=${DRY_RUN:-0}
GAME_LOGGING=${GAME_LOGGING:-False}

# Defaults copied from the previous QMIX run that learned early on this task.
QMIX_CONFIG=${QMIX_CONFIG:-qmix}
QMIX_T_MAX=${QMIX_T_MAX:-2500000}
QMIX_BUFFER_SIZE=${QMIX_BUFFER_SIZE:-32}
QMIX_BUFFER_WARMUP=${QMIX_BUFFER_WARMUP:-20}
QMIX_BATCH_SIZE=${QMIX_BATCH_SIZE:-10}
QMIX_TRAIN_UPDATES_PER_BATCH=${QMIX_TRAIN_UPDATES_PER_BATCH:-2}
QMIX_LR=${QMIX_LR:-0.0005}
QMIX_CRITIC_LR=${QMIX_CRITIC_LR:-$QMIX_LR}
QMIX_TARGET_UPDATE_INTERVAL=${QMIX_TARGET_UPDATE_INTERVAL:-20}
QMIX_EPSILON_ANNEAL_TIME=${QMIX_EPSILON_ANNEAL_TIME:-100000}
QMIX_SAVE_MODEL_INTERVAL=${QMIX_SAVE_MODEL_INTERVAL:-100000}

RUN_TAG=${RUN_TAG:-}

validate_reward_mode "$REWARD_MODE"

SBATCH_GPU_ARGS=()
if [ "$GPUS_PER_TASK" != "0" ]; then
    SBATCH_GPU_ARGS=(--gres "gpu:$GPUS_PER_TASK")
fi

mkdir -p "$FRONT_GOAL_LADDER_DIR/slurm"

printf '[INFO] QMIX scenario ladder submit\n'
printf '[INFO] SCENARIOS=%s\n' "$SCENARIOS"
printf '[INFO] REWARD_MODE=%s\n' "$REWARD_MODE"
printf '[INFO] SEEDS=%s\n' "$SEEDS"
printf '[INFO] NUM_ENVS=%s\n' "$NUM_ENVS"
printf '[INFO] CPUS_PER_TASK=%s\n' "$CPUS_PER_TASK"
printf '[INFO] GPUS_PER_TASK=%s\n' "$GPUS_PER_TASK"
printf '[INFO] MEM_PER_JOB=%s\n' "$MEM_PER_JOB"
printf '[INFO] TEST_NEPI=%s\n' "$TEST_NEPI"
printf '[INFO] GAME_LOGGING=%s\n' "$GAME_LOGGING"
printf '[INFO] QMIX_CONFIG=%s\n' "$QMIX_CONFIG"
printf '[INFO] QMIX_T_MAX=%s\n' "$QMIX_T_MAX"
printf '[INFO] QMIX_BUFFER_SIZE=%s\n' "$QMIX_BUFFER_SIZE"
printf '[INFO] QMIX_BUFFER_WARMUP=%s\n' "$QMIX_BUFFER_WARMUP"
printf '[INFO] QMIX_BATCH_SIZE=%s\n' "$QMIX_BATCH_SIZE"
printf '[INFO] QMIX_TRAIN_UPDATES_PER_BATCH=%s\n' "$QMIX_TRAIN_UPDATES_PER_BATCH"
printf '[INFO] QMIX_LR=%s\n' "$QMIX_LR"
printf '[INFO] QMIX_TARGET_UPDATE_INTERVAL=%s\n' "$QMIX_TARGET_UPDATE_INTERVAL"
printf '[INFO] QMIX_EPSILON_ANNEAL_TIME=%s\n' "$QMIX_EPSILON_ANNEAL_TIME"
printf '[INFO] QMIX_SAVE_MODEL_INTERVAL=%s\n' "$QMIX_SAVE_MODEL_INTERVAL"
printf '[INFO] DRY_RUN=%s\n' "$DRY_RUN"

for scenario in $SCENARIOS; do
    validate_scenario "$scenario"
    env_stem=$(front_goal_env_stem "$scenario" "$REWARD_MODE")
    env_yaml="$PYMARL_ENV_CONFIG_DIR/$env_stem.yaml"
    if [ ! -f "$env_yaml" ]; then
        echo "Missing PyMARL env yaml: $env_yaml" >&2
        exit 1
    fi

    for seed in $SEEDS; do
        if [ -n "$RUN_TAG" ]; then
            run_name=front_goal_${scenario}_${REWARD_MODE}_${ALGO}_${RUN_TAG}_seed${seed}_env${NUM_ENVS}
            job_name=fg-${ALGO}-${scenario}-${RUN_TAG}-${seed}
        else
            run_name=front_goal_${scenario}_${REWARD_MODE}_${ALGO}_seed${seed}_env${NUM_ENVS}
            job_name=fg-${ALGO}-${scenario}-${seed}
        fi

        export_arg="ALL,ALGO=$ALGO,SCENARIO=$scenario,REWARD_MODE=$REWARD_MODE,SEED=$seed,NUM_ENVS=$NUM_ENVS,CPUS_PER_TASK=$CPUS_PER_TASK,GPUS_PER_TASK=$GPUS_PER_TASK,MEM_PER_JOB=$MEM_PER_JOB,TEST_NEPI=$TEST_NEPI,RUN_NAME=$run_name,GAME_LOGGING=$GAME_LOGGING,QMIX_CONFIG=$QMIX_CONFIG,QMIX_T_MAX=$QMIX_T_MAX,QMIX_BUFFER_SIZE=$QMIX_BUFFER_SIZE,QMIX_BUFFER_WARMUP=$QMIX_BUFFER_WARMUP,QMIX_BATCH_SIZE=$QMIX_BATCH_SIZE,QMIX_TRAIN_UPDATES_PER_BATCH=$QMIX_TRAIN_UPDATES_PER_BATCH,QMIX_LR=$QMIX_LR,QMIX_CRITIC_LR=$QMIX_CRITIC_LR,QMIX_TARGET_UPDATE_INTERVAL=$QMIX_TARGET_UPDATE_INTERVAL,QMIX_EPSILON_ANNEAL_TIME=$QMIX_EPSILON_ANNEAL_TIME,QMIX_SAVE_MODEL_INTERVAL=$QMIX_SAVE_MODEL_INTERVAL"

        cmd=(
            sbatch
            --cpus-per-task "$CPUS_PER_TASK"
            --mem "$MEM_PER_JOB"
            "${SBATCH_GPU_ARGS[@]}"
            --job-name "$job_name"
            --export "$export_arg"
            "$FRONT_GOAL_LADDER_DIR/run.sh"
        )

        echo "[SUBMIT] ${cmd[*]}"
        if [ "$DRY_RUN" != "1" ]; then
            "${cmd[@]}"
        fi
    done
done
