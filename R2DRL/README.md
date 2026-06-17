# R2DRL Runtime Snapshot

`R2DRL/` contains the RoboCup2D environment, simulator-side source, and bundled native runtime files used by the demo scripts in `../scripts/`.

Last documentation refresh: 2026-06-17.

## Contents

- `robocup2d/`: canonical QMix/PyMARL-compatible RoboCup2D Python environment package.
- `helios-base/`: modified HELIOS base team source plus `sample_player`, `sample_coach`, `sample_trainer`, and `player.conf`.
- `librcsc/`: modified librcsc source plus `rcsc/.libs/librcsc.so.19` for the bundled HELIOS binaries.
- `rcssserver/`: rcssserver source plus `build/rcssserver` and required local `librcss*.so` libraries.
- `runtime_libs/`: C++ runtime libraries loaded before system paths by the demo scripts.

The algorithm code lives one level up:

- `../algorithm/qmix/source/`
- `../algorithm/mappo/source/`
- `../algorithm/paradqn/source/`

## Runtime Architecture

The Python environment launches and coordinates four simulator-side process groups:

- `rcssserver`: synchronous RoboCup 2D simulator.
- `sample_player`: HELIOS-based players that read actions from shared memory and write observations/masks back.
- `sample_coach`: online coach process exposing global state and goal flags through shared memory.
- `sample_trainer`: trainer process used for default and scenario resets.

Main environment modules:

- `robocup2d/env.py`: `Robocup2dEnv`, `R2DRL`, and `ParallelR2DRL`.
- `robocup2d/runtime.py`: run id, ports, shared-memory names, logs, restart, and process lifecycle.
- `robocup2d/agents.py`: shared-memory access, action writing, observation/state/action-mask reading, EPV reward logic, and agent masks.
- `robocup2d/start_sampler.py`: trajectory and front-goal catalog scenario start sampling.
- `robocup2d/config/`: current YAML configs.

## Prepare Paths

Run from the repository root before training:

```bash
./scripts/prepare_runtime.sh
```

This rewrites all RoboCup2D YAML files so `server_path`, HELIOS directories, `lib_paths`, and scenario data paths point inside the current checkout.

## Direct Smoke Check

```bash
cd <repo-root>
./scripts/prepare_runtime.sh
PYTHONPATH="$PWD/R2DRL:$PWD/algorithm/qmix/source:${PYTHONPATH:-}" python - <<'PY'
from robocup2d import Robocup2dEnv
print("import ok:", Robocup2dEnv.__name__)
PY
```

Full simulator checks are covered by the three demo scripts in `../scripts/`.

## Task Families

The configs cover:

- 11vs11 benchmark and full-match tasks.
- 5vs5 benchmark tasks.
- front-goal catalog scenarios: 1v0, 1v1, 2v1, 3v2, 4v3, 5v4.
- legacy 3vs3 scenario/benchmark configs.
- Base discrete action space and Hybrid parameterized action space.
- EPV-on, EPV-off, EPV-linear, and goal-only reward variants where provided.
- optional legal-action masks.

See `robocup2d/BENCHMARKS.md` for the config matrix.

## Notes

The bundled binaries are Linux x86-64 builds. If they do not run on another machine, rebuild with:

```bash
./scripts/build_simulators.sh
./scripts/prepare_runtime.sh
```
