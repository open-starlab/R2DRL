# Modified HELIOS Base for R2DRL

This directory is the HELIOS base source used by the current R2DRL RoboCup2D environment. It is not a vanilla upstream copy: player, coach, and trainer code have been adapted to communicate with Python through shared memory.

Last documentation refresh: 2026-10-16.

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

### Native HELIOS observation policy

Base action 17 (`ACTION_HELIOS`, also used by the HELIOS opponent) preserves the
neck action selected by the native role/behavior. In particular, ball-holder
tracking, specialized interception observation and goalie observation must not
be replaced by a generic scan after the native behavior returns. Hybrid action
4 already preserves this selection. The default `Neck_TurnToBallOrScan(0)` for
other discrete RL actions, WAIT scanning, and the no-body-command fallback are
unchanged. This is an intentional runtime behavior correction; historical
evaluations with the blanket Base override remain historical and must not be
relabeled as evaluations of the corrected client. It does not change native
intercept/tackle thresholds, formations, or shot parameters, and is not by
itself proof of improved defensive performance.

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

### Optional per-player WorldModel trace

The public client can write **read-only diagnostic snapshots** of the WorldModel
used by each player. Logging is off by default. Rebuild `sample_player` after
updating the source (see the build/regression instructions below); an older
binary does not gain this feature from a source update alone.

Create a dedicated output directory and export these variables **before**
starting your normal Python environment or demo:

```bash
mkdir -p /tmp/r2drl-wm-trace
export R2DRL_WM_TRACE_DIR=/tmp/r2drl-wm-trace
export R2DRL_WM_TRACE_SIDE=r     # optional: l, r, or all (default)
export R2DRL_WM_TRACE_EVERY=1    # optional: every N player decisions (default 1)
```

The Python launcher already passes the parent environment to both teams. Each
selected player writes its own `wm_<side>_u<unum>_pid<pid>_<timestamp>.jsonl`;
the player's ordinary log reports the actual path. No trace files are created
when `R2DRL_WM_TRACE_DIR` is unset or empty. Unset the variables to disable logging.
A missing/unwritable destination or invalid option emits one warning and
disables that player's tracing, not its soccer behavior. Logging has I/O cost;
use only for diagnostics, not latency/throughput measurements.

Each sampled decision has `pre_decision` and `post_decision` rows, joined by
file, `decision_index`, `cycle`, and `stopped`. Include `stopped`: multiple
decisions during a restart can share the same server cycle. Sampling is by
decision count, **not** by server cycle or guaranteed wall-clock frequency.

Rows include:

- Team/side/uniform number, mode, game-mode ID, process ID and source SHM name.
- Self position/velocity, body/neck/face angles, stamina, validity/age fields,
  kickability, frozen status and remaining tackle/foul freeze counts.
- Ball estimates, seen/heard positions, validity and information-age counts.
- Player estimates present in that player's WM, including their identity,
  position, velocity, facing and age fields; missing players are not filled
  from coach/server truth.
- Predicted self/teammate/opponent intercept steps and first interceptors,
  kickable teammate/opponent, and self tackle/foul/catch probabilities.
- Requested Base/Hybrid action and Hybrid parameters, plus queued body,
  turn-neck and change-view command strings in the post-decision row.

Coordinates are the unmodified **librcsc WorldModel coordinates**, not a promise
of server/RCG coordinates (the right team's normal-play WM is reversed). Ages
and validity flags must be used when interpreting stale or unknown estimates.
Invalid vectors and non-finite numbers are JSON `null`. `foul_probability` is
librcsc's estimated success probability for the foul-enabled tackle, not the
probability of receiving a foul/card.

`post_decision` is captured at the end of `communicationImpl`, after neck/view
actions but **before command composition/transmission**. It is not the next
server state and does not prove server acceptance or successful execution.
The sender can synthesize `(turn 0)` for a missing body command when not frozen;
the trace records the queued command as `null` in that case. Say messages are
not serialized by this diagnostic logger. Match with RCL/RCG to verify reception
and movement. The trace records planner inputs and commands, not the exact
internal branch/call stack or all policy caches.

This is deliberately **not** the real-data import runtime's restorable WM
snapshot protocol: no restore, fullstate injection, RNG restoration, extra
actor input or shared-memory ABI change is introduced. Existing fullstate
settings remain untouched. Use the importer runtime if a restorable state is
needed. These diagnostics cannot recover WM information absent from old runs.

## Upstream Reference

HELIOS base is a sample team for the RoboCup Soccer 2D Simulator.

- Upstream: https://github.com/helios-base/helios-base
- RoboCup simulator: https://github.com/rcsoccersim/

The original upstream build information is still useful, but this project-specific README is the authoritative guide for how this copy is used inside R2DRL.

## Goalkeeper reverse-dash compatibility

The goalkeeper Y-adjustment and backward catch-point branches support servers
with `min_dash_power = 0` (rcssserver 17+). A reverse intention is sent as
positive power with a body-relative dash direction of 180 degrees, rather than
being clipped to `dash 0`. Negative-power behavior is retained on older servers
that permit it. This client-side fix does not change server dash/catch physics,
the field-player policy, action masks, or shooting parameters. Rebuild
`sample_player` from these sources before using the fix; an existing bundled
binary is not changed merely by updating the source.

The repository's `tests/CMakeLists.txt` can build the player target and check
reverse-dash compatibility, native HELIOS neck dispatch and opt-in WorldModel
tracing against the bundled librcsc. With Boost installed:

```bash
cmake -S tests -B /tmp/r2drl-player-check -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/r2drl-player-check -j2
ctest --test-dir /tmp/r2drl-player-check --output-on-failure
```

Run these commands from the repository root. If Boost is in a nonstandard
location, pass `-DBOOST_ROOT=/path/to/boost/prefix` when configuring. The rebuilt
player is `/tmp/r2drl-player-check/bin/sample_player`; building does not install
it over the existing runtime binary. Preserve that binary before installing the
replacement at `R2DRL/helios-base/src/player/sample_player`.

CTest also checks `sample_player --help` with the **bundled** runtime libraries,
without connecting to a server. A successful build alone does not establish
runtime-library compatibility. In particular, if GCC 13 produces a
`GLIBCXX_3.4.32` dependency unavailable in the bundled `libstdc++.so.6`, configure
a fresh build directory with `-DCMAKE_CXX_COMPILER=g++-10` (when installed), and
rerun CTest before installing the binary. Do not replace bundled libraries just
to bypass this check.

