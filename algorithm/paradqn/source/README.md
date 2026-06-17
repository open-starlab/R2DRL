# P-DQN: Parametrized Deep Q-Networks

This is an implementation of P-DQN, a reinforcement learning algorithm designed to handle environments with discrete-continuous hybrid action spaces.

## Quick Start

First, clone this repository and install the required packages.

```bash
git clone https://github.com/ZhBF/ParaDQN.git
cd ParaDQN
pip install -r requirements.txt
cd ..
```

Then, install the `gym-hybrid` environment.

```bash
git clone https://github.com/thomashirtz/gym-hybrid.git
cd gym-hybrid
pip install .
cd ..
```

Finally, run the training script.
```bash
cd ParaDQN
python main.py
```

## Multi-Environment Parallel Training

The training program now supports parallel data collection using multiple environment processes. This significantly speeds up training by collecting experiences from multiple environments simultaneously.

### Usage

To run training with multiple parallel environments, use the `--num_envs` argument:

```bash
python train_robocup.py --num_envs 4
```

### Example with SLURM

For single environment (original):
```bash
sbatch paradqn.sh
```

For multi-environment parallel training:
```bash
sbatch paradqn_multienv.sh
```

### Configuration

Key parameters for parallel training:
- `--num_envs`: Number of parallel environments (default: 1)
- Recommended: set `num_envs` ≤ number of CPU cores allocated
- Memory usage scales linearly with `num_envs`

### Benefits

- **Faster Data Collection**: Collects experiences from N environments simultaneously
- **Better Sample Efficiency**: More diverse experiences in the replay buffer
- **Reduced Wall-Clock Time**: Training completes faster with the same number of episodes

### Notes

- Each environment runs in a separate subprocess
- Evaluation still uses a single environment instance for consistent benchmarking
- All environments share the same agent and replay buffer
- Checkpoints and logging work the same way as single-environment training

## To Do Tasks
[x] Edit README with Quick Start instructions

[x] Add requirements.txt file

[x] Add command line arguments for training script

[x] Consider bounding and shifting action parameters

[ ] Implement more efficient storage in Replay Buffer

## Reference 
[Parametrized Deep Q-Networks Learning: Reinforcement Learning with Discrete-Continuous Hybrid Action Space](https://arxiv.org/abs/1810.06394)
