#!/bin/bash
#SBATCH --partition=ubuntu
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=8
#SBATCH --job-name=paradqn
#SBATCH --output=slurm/paradqn_%j.out
#SBATCH --exclude=nevera,tesla,nsx

/work7/b_zhang/programs/miniconda3/envs/pdqn-robocup/bin/python -u train_robocup.py # --resume_from "/work7/b_zhang/projects/para-rl/para-dqn-robocup/runs/run_seed_0_20251103_220325/checkpoints"
