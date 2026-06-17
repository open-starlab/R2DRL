from functools import partial
import sys
import os
from .robocup2d import Robocup2dEnv, R2DRL, ParallelR2DRL
from .multiagentenv import MultiAgentEnv
def env_fn(env, **kwargs) -> MultiAgentEnv:
    return env(**kwargs)

REGISTRY = {}
REGISTRY["r2drl"] = partial(env_fn, env=R2DRL)
REGISTRY["robocup"] = partial(env_fn, env=Robocup2dEnv)
REGISTRY["parallelr2drl"] = partial(env_fn, env=ParallelR2DRL)
