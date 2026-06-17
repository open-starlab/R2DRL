# EPV Assets for R2DRL

This directory contains the EPV data and helper scripts used by the current `robocup2d` environment.

It also keeps the original LaurieOnTracking/Metrica tutorial code that the EPV work was based on, but in this repository the important runtime files are the CSV grids loaded by `robocup2d/agents.py`.

## Runtime Use

`Agents._load_epv_grid()` looks for `config.epv_grid_file`. If the value is relative, it is resolved inside this directory.

Current defaults:

- `EPV_grid.csv`: default EPV grid.
- `EPV_grid_front_goal_linear.csv`: linear front-goal EPV grid used by the `front-goal-1v0_..._epv-linear` preset.

The environment loads EPV grids for diagnostics whenever possible. Reward shaping is active only when YAML has:

```yaml
useMaxEpv: true
```

Related reward fields:

```yaml
epv_grid_file: EPV_grid.csv
epv_progress_reward: 0.0
epv_progress_scale: 1.0
```

In the current front-goal EPV presets, `epv_progress_scale` is commonly set to `2.0`.

## Reward Behavior in Current Code

- Reset stores the starting EPV as `initial_episode_epv`.
- `max_episode_epv` tracks the best EPV reached so far.
- When `useMaxEpv` is true, EPV progress is counted only when the left team is truly kickable.
- Positive increases in `max_episode_epv` are rewarded.
- On a scored goal, `complete_episode_epv()` can add the remaining progress up to the grid maximum.
- If `useMaxEpv` is false, EPV values are still useful diagnostics but do not add shaping reward.

## Files

- `EPV_grid.csv`: default grid.
- `EPV_grid_front_goal_linear.csv`: front-goal linear reward grid.
- `*.png`: visualizations used to inspect EPV grids and front-goal behavior.
- `Metrica_EPV.py`, `Metrica_IO.py`, `Metrica_PitchControl.py`, `Metrica_Velocities.py`, `Metrica_Viz.py`: original helper/tutorial modules retained for reference.
- `Tutorial*.py`: original tutorial scripts.

## Original Reference

The original sample data and tutorial context come from Metrica Sports and Friends of Tracking:

- https://github.com/metrica-sports/sample-data
- Tutorial 1: https://www.youtube.com/watch?v=8TrleFklEsE
- Tutorial 2: https://www.youtube.com/watch?v=VX3T-4lB2o0
- Tutorial 3: https://www.youtube.com/watch?v=5X1cSehLg6s
- Tutorial 4: https://www.youtube.com/watch?v=KXSLKwADXKI

