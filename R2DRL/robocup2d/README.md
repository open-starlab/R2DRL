# RoboCup 2D RL Environment

This package provides a high-performance, Python-based Reinforcement Learning environment for the RoboCup 2D Soccer Simulation. It wraps the complex interactions between the RoboCup Soccer Server (`rcssserver`), players, coach, and trainer into a unified, Gym-like interface suitable for Multi-Agent Reinforcement Learning (MARL).

## Key Features

*   **High Performance IPC**: Uses **Shared Memory (SHM)** instead of standard socket communication for observations and actions, significantly reducing latency and overhead during training.
*   **Process Management**: Automatically manages the lifecycle of all underlying processes (server, players, coach, trainer). Includes robust startup, teardown, and watchdog mechanisms to handle crashes or stalls.
*   **MARL Interface**: Provides a PyMARL-compatible interface with `reset()`, `step()`, `get_obs()`, `get_state()`, and `get_avail_actions()`.
*   **Synchronous Execution**: Ensures strict synchronization between the simulation cycle and the RL agent using memory barriers and flags.
*   **Hybrid Action Space**: Supports both discrete (Base) and parameterized continuous (Hybrid) action spaces.

## Architecture

The environment is organized into several modules:

*   **`env.py`**: The core `Robocup2dEnv` class. It orchestrates the simulation, handles the RL loop, and manages resources.
*   **`ipc/`**: Inter-Process Communication logic. Handles shared memory allocation, handshake protocols (READY/REQ flags), and synchronization barriers.
*   **`process/`**: Utilities for process lifecycle management.
    *   `launcher.py`: Starts the server and agents.
    *   `killer.py`: Handles process termination and cleanup.
    *   `ports.py`: Manages network ports to allow parallel training sessions.
    *   `watchdog.py`: Monitors child processes for unexpected exits.
*   **`protocols/`**: Definitions of data structures and memory layouts for the shared memory buffers (Player, Coach, Trainer).

## Usage

```python
from environments.robocup2d import Robocup2dEnv

# Initialize the environment
env = Robocup2dEnv(
    cfg="robocup.yaml",
    n1=11,          # Number of agents (Team 1)
    n2=11,          # Number of opponents (Team 2)
    team1="base",   # Team 1 name (determines action space type)
    team2="helios", # Team 2 name
    episode_limit=1000,
    goal_x=52.5,
    goal_y=34.0,
    HALF_LENGTH=52.5,
    HALF_WIDTH=34.0
)

env.reset()

done = False
while not done:
    # Get observations for all agents
    obs = env.get_obs()
    
    # Get available actions (masks)
    avail_actions = env.get_avail_actions()
    
    # Select actions (example: random actions)
    # actions shape: (n_agents, 1) for discrete or (n_agents, 3) for hybrid
    actions = ... 
    
    # Step the environment
    reward, done, info = env.step(actions)

env.close()
```

## Configuration

The environment is configured via a YAML file (default: `robocup.yaml`) and constructor arguments. Key configurations include:

*   **Teams**: `team1` (controlled agents) and `team2` (opponents).
*   **Ports**: Automatic port selection to support parallel execution.
*   **Paths**: Locations of the `rcssserver`, player, coach, and trainer executables.

## Requirements

*   Python 3.8+
*   `numpy`
*   `psutil`
*   `pyyaml`
*   `torch` (for tensor compatibility)
*   RoboCup 2D Server and compatible binaries (e.g., Helios, Cyrus) installed on the system.
