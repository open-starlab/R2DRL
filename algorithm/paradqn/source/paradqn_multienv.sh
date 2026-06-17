#!/bin/bash
#SBATCH --partition=ubuntu
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=14
#SBATCH --job-name=paradqn_multienv
#SBATCH --output=slurm/paradqn_multienv_%j.out
#SBATCH --exclude=nevera,tesla,nsx

/work7/b_zhang/programs/miniconda3/envs/pdqn-robocup/bin/python -u train_robocup.py \
    --num_envs 8 \
    --train_episodes 500000 \
    --batch_size 64 \
    --replay_capacity 20000
    # --resume_from "/path/to/checkpoint"  # Uncomment to resume from checkpoint
