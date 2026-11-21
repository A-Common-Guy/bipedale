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


def test_the_limiter_can_vary_between_episodes_and_be_absent_in_some():
    import jax
    from sandro_rl.env import SandroEnv
    env = SandroEnv(config_mod.load("agile_terrain_v7.yaml", OVERRIDES))
    assert env.wheel_cap == (19.5, 1.25)
    keys = jax.random.split(jax.random.PRNGKey(0), 400)
    caps = jax.vmap(lambda k: env._sample_wheel_cap(k, True))(keys)
    limit, gain = np.asarray(caps["cap_limit"]), np.asarray(caps["cap_gain"])
    on = limit < 100
    assert 0.6 < on.mean() < 0.8
    assert limit[on].min() >= 17.5 and limit[on].max() <= 21.5
    assert gain.min() >= 0.5 and gain.max() <= 2.0 + 1e-5
    # Off is a limit out of reach: the torque passes whatever the speed.
    params = {"cap_limit": jp.float32(1e6), "cap_gain": jp.float32(1.0)}
    assert np.allclose(env._speed_cap(jp.full(2, 3.0), jp.array([30.0, -30.0]), params), 3.0)
    nominal = env._sample_wheel_cap(None, False)
    assert float(nominal["cap_limit"]) == 19.5
