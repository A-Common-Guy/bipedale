"""The wheel drives' velocity limiter (action.wheel_speed_limit, configs/fun_fast_v2.yaml;
evaluate.py's --sim wheel_speed_limit): the torque towards a higher speed is capped at
gain * (limit - speed).

On the CPU through MJX's JAX backend.
"""

import jax.numpy as jp
import numpy as np
import pytest

from sandro_rl import config as config_mod

OVERRIDES = ["env.impl=jax", "env.graph_mode=null"]


@pytest.fixture(scope="module")
def env():
    from sandro_rl.env import SandroEnv
    return SandroEnv(config_mod.load("fun_fast_v2.yaml", OVERRIDES))


def test_the_limiter_takes_the_driving_torque_away_at_the_limit_and_brakes_beyond_it(env):
    limit, gain = env.wheel_cap
    assert (limit, gain) == (19.5, 1.0)
    speed = jp.array([0.0, 17.5, 19.5, 21.5, -19.5, -21.5])
    assert np.allclose(env._speed_cap(jp.full(6, 3.0), speed), [3.0, 2.0, 0.0, -2.0, 3.0, 3.0])
    assert np.allclose(env._speed_cap(jp.full(6, -3.0), speed), [-3.0, -3.0, -3.0, -3.0, 0.0, 2.0])


def test_only_the_wheels_are_limited_and_only_where_it_is_asked_for(env):
    from sandro_rl.env import SandroEnv
    data = type("Data", (), {})()
    data.qpos = jp.zeros(env.mj_model.nq)
    data.qvel = jp.zeros(env.mj_model.nv).at[env.dadr].set(25.0)
    frame = jp.zeros((5, 6)).at[4].set(2.0)                # 2 Nm of feed-forward on every joint
    tau = np.asarray(env._mit_torque(frame, data, None, jp.full(6, 9.0)))
    wheels = np.asarray(env.wheel_idx)
    legs = np.setdiff1d(np.arange(6), wheels)
    assert np.allclose(tau[wheels], 19.5 - 25.0) and np.allclose(tau[legs], 2.0)
    plain = SandroEnv(config_mod.load("fun_fast.yaml", OVERRIDES))
    assert plain.wheel_cap is None
    assert np.allclose(plain._mit_torque(frame, data, None, jp.full(6, 9.0)), 2.0)
