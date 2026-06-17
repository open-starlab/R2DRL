# MAPPO RoboCup2D Adapter

This source tree is bundled with the repository-level RoboCup2D runtime.

Use the top-level scripts from `<repo-root>`:

```bash
./scripts/prepare_runtime.sh
MAX_ENV_STEPS=1000 ./scripts/run_mappo_demo.sh
```

The MAPPO training entry point is:

```text
algorithm/mappo/source/onpolicy/scripts/train/train_robocup2d.py
```

It reuses the canonical QMix/PyMARL RoboCup2D config loader through:

```text
algorithm/qmix/source/envs/robocup2d
```

The demo script passes both `--robocup_env_config` and `--pymarl_src` explicitly. The defaults are also derived from the current checkout, so they do not depend on an external PyMARL directory.
