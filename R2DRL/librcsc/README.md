# Modified librcsc for R2DRL

This directory is the librcsc source used by the current R2DRL HELIOS build. It is mostly upstream-compatible librcsc, with small project-specific player-agent/action-effector changes required by the current shared-memory action interface.

Last documentation refresh: 2026-06-17.

## Role in R2DRL

`helios-base` links against this library. The Python environment does not import librcsc directly, but launched C++ player, coach, and trainer processes require it at runtime.

The Python YAML `lib_paths` field is used to build `LD_LIBRARY_PATH` for child processes. The environment prefers paths ending with:

```text
/librcsc/rcsc/.libs
```

because that is the common in-tree Autotools build output location.

## Current Project-Specific Changes

The current diff touches:

- `rcsc/player/action_effector.cpp`
- `rcsc/player/action_effector.h`
- `rcsc/player/player_agent.cpp`
- `rcsc/player/player_agent.h`

The important added capability is explicit-direction catch support:

- `ActionEffector::setCatch(const AngleDeg & dir)`
- `PlayerAgent::doCatch(const AngleDeg & dir)`

This supports the modified HELIOS/R2DRL action path where goalie catch can be issued with an explicit relative direction, including from the Hybrid action interface.

## Build Notes

The GitHub bundle already includes `rcsc/.libs/librcsc.so.19` for the demo binaries. If rebuilding is needed, use the repository-level helper:

```bash
cd <repo-root>
./scripts/build_simulators.sh
./scripts/prepare_runtime.sh
```

For local development, installing is optional if `helios-base` can find the headers/libs and runtime `LD_LIBRARY_PATH` includes the build output. YAML `lib_paths` is the main runtime hook.

## Build Order

1. Build this `librcsc`.
2. Build `helios-base` against this `librcsc`.
3. Make sure `robocup2d/config/*.yaml` points to the matching binaries and library paths.

## Upstream Reference

librcsc is the basic library for RoboCup Soccer Simulation teams and tools.

- Upstream: https://github.com/helios-base/librcsc
- RoboCup simulator: https://rcsoccersim.github.io/

The original upstream `INSTALL` file is still included for detailed Autotools options.

