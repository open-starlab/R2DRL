
# R2DRL - RCSS 2D Reinforcement Learning

## Overview

R2DRL is a reinforcement learning framework for RoboCup 2D Soccer Simulation. This guide will walk you through the complete setup process, from installing dependencies to running your first training session.

## Quick Start Guide

### 1. Clone the R2DRL Repository

First, clone the R2DRL source code and navigate to the project directory:

```bash
git clone https://github.com/open-starlab/R2DRL.git
cd R2DRL
```

**Note:** All subsequent operations should be performed from this root directory unless otherwise specified.

### 2. Setup RoboCup Soccer Server (rcssserver)

The soccer server is the core simulator for RoboCup 2D Soccer Simulation.

#### 2.1 Clone the Repository

```bash
git clone https://github.com/rcsoccersim/rcssserver.git
```

#### 2.2 Modify Timeout Settings

To accommodate reinforcement learning training, which may require longer decision times, modify the server's timeout setting to an extremely large value:

```bash
sed -i.bak '2396s/.*/    const double max_msec_waited = 60 * 60 * 1000;/' rcssserver/src/stadium.cpp
```

This change prevents the server from timing out during training episodes.

#### 2.3 Build and Install

Follow the instructions in the rcssserver README to compile and install.

### 3. Setup RoboCup Soccer Monitor (rcssmonitor)

The monitor provides visualization for soccer matches.

#### 3.1 Clone the Repository

```bash
git clone https://github.com/rcsoccersim/rcssmonitor.git
```

#### 3.2 Build and Install

Follow the instructions in the rcssmonitor README to compile and install.

### 4. Setup librcsc (RoboCup Soccer Common Library)

librcsc is a fundamental library providing basic functionalities for RCSS 2D agents.

#### 4.1 Clone the Repository

```bash
git clone https://github.com/helios-base/librcsc.git
```

#### 4.2 Apply R2DRL Modifications

The R2DRL project includes modified versions of librcsc files. Copy these modifications to the cloned repository:

```bash
cp -r R2DRL/librcsc/* librcsc/
```

**What's Modified:** The R2DRL modifications include adjustments for reinforcement learning integration, custom action interfaces, and state observation enhancements.

#### 4.3 Build and Install

Follow the instructions in the librcsc README to compile and install.

### 5. Setup Helios Base

Helios Base is a sample team that serves as the foundation for R2DRL agents.

#### 5.1 Clone the Repository

```bash
git clone https://github.com/helios-base/helios-base.git
```

#### 5.2 Apply R2DRL Modifications

Copy the R2DRL-modified helios-base files:

```bash
cp -r R2DRL/helios-base/* helios-base/
```

**What's Modified:** The modifications include integration points for reinforcement learning policies, custom communication protocols with the Python training environment, and modified decision-making logic.

#### 5.3 Build and Install

Follow the instructions in the helios-base README to compile.

### 6. Configure Python Environment

The Python environment provides the reinforcement learning interface.

#### 6.1 Copy Python Environment Files

Copy the `R2DRL/robocup2d` directory to your preferred location:

```bash
cp -r R2DRL/robocup2d /path/to/your/workspace/
```

#### 6.2 Configure Paths

Before running, you must update the configuration file with your actual system paths:

1. Open `R2DRL/robocup2d/robocup.yaml`
2. Update the following fields:
   - `player_dir`
   - `player_exe`
   - `coach_dir`
   - `coach_exe`
   - `server_path`
   - `trainer_dir`
   - `trainer_exe`
   - `config_dir`
   - `player_config`

## Acknowledgments

This project builds upon:
- https://github.com/rcsoccersim/rcssserver
- https://github.com/rcsoccersim/rcssmonitor
- https://github.com/helios-base/librcsc
- https://github.com/helios-base/helios-base
