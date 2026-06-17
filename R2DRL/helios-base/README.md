# Modified HELIOS Base for R2DRL

This directory is the HELIOS base source used by the current R2DRL RoboCup2D environment. It is not a vanilla upstream copy: player, coach, and trainer code have been adapted to communicate with Python through shared memory.

Last documentation refresh: 2026-06-17.

## Role in R2DRL

The Python environment launches binaries from this tree through `robocup2d/process/launcher.py`:

- `src/player/sample_player`
- `src/coach/sample_coach`
- `src/trainer/sample_trainer`

These processes connect to `rcssserver` normally, but also attach to shared-memory buffers created by Python.

## Important Project-Specific Changes

### Player

`src/player/sample_player.cpp` supports:

- `--shm-name`: shared-memory name for this player.
- `--level`: opponent difficulty value used for right-side players.
- Base action mode: discrete actions such as shoot, pass, dribble, hold, helios, wait.
- Hybrid action mode: turn, dash, kick, catch, helios, wait with continuous parameters.
- Writing player observation and action mask to shared memory.
- Reading action requests from shared memory using `obs_seq`, `act_seq`, and `done_seq` synchronization.

Current Base action ids are maintained in `robocup2d/agents.py` and mirrored by the player-side implementation.

### Coach

`src/coach/sample_coach.cpp` supports:

- `--shm-name`: shared-memory name for global coach state.
- Writing global state and goal flags for Python reward/terminal calculation.

### Trainer

`src/trainer/sample_trainer.cpp` supports:

- `RCSC_TRAINER_SHM`: environment variable used to attach trainer shared memory.
- Default reset requests from Python.
- Custom reset requests with ball position, left/right player positions, and body angles.

## Action-Chain Note

The current `SamplePlayer::createActionGenerator()` uses `ActGen_StrictCheckPass` as the default pass generator. `ActGen_DirectPass` is present in the source but commented out in the default generator list.

Action-chain candidates are evaluated by the HELIOS planner evaluator, while the R2DRL Python side chooses high-level actions through shared memory.

## Build Notes

The GitHub bundle already includes the runtime binaries used by the demo scripts. If they are incompatible with the target machine, rebuild from the repository root:

```bash
cd <repo-root>
./scripts/build_simulators.sh
./scripts/prepare_runtime.sh
```

The Python YAML field `lib_paths` is used to build the child process `LD_LIBRARY_PATH`.

Important YAML fields are localized by `scripts/prepare_runtime.sh`:

```yaml
player_dir: /path/to/helios-base/src/player
player_exe: ./sample_player
coach_dir: /path/to/helios-base/src/coach
coach_exe: ./sample_coach
trainer_dir: /path/to/helios-base/src/trainer
trainer_exe: ./sample_trainer
config_dir: /path/to/helios-base/src/formations-dt
player_config: /path/to/helios-base/src/player.conf
```

## Runtime Logs

When launched by Python, logs are written under:

```text
logs_dir/<run_id>/
```

Typical files:

- `player_<team>_uXX_p<port>.log`
- `coach_<port>.log`
- `trainer_<port>.log`

## Upstream Reference

HELIOS base is a sample team for the RoboCup Soccer 2D Simulator.

- Upstream: https://github.com/helios-base/helios-base
- RoboCup simulator: https://github.com/rcsoccersim/

The original upstream build information is still useful, but this project-specific README is the authoritative guide for how this copy is used inside R2DRL.

