"""The observation and action contract: sizes, mirror maps, and the deployment
interface (interface.py) against the training environment value by value."""

import math

import numpy as np
import pytest

from sandro_rl import config as config_mod, sources
from sandro_rl.interface import PolicyInterface
from sandro_rl.robot import RobotSpec


@pytest.fixture(scope="module", params=["flat_tracking.yaml", "agile_terrain.yaml"])
def cfg(request):
    return config_mod.load(request.param)


@pytest.fixture(scope="module")
def spec():
    return RobotSpec.load()


@pytest.fixture(scope="module")
def env(cfg):
    from sandro_rl.env import SandroEnv
    return SandroEnv(cfg)


def test_mirror_permutation_is_an_involution(spec):
    perm = spec.mirror_perm
    assert sorted(perm) == list(range(6))
    assert np.array_equal(perm[perm], np.arange(6))
    names = [sources.JOINTS[i] for i in perm]
    assert names[sources.JOINTS.index("right_wheel")] == "left_wheel"


def test_policy_rate_divides_control_rate(cfg, spec):
    i = PolicyInterface(cfg, spec)
    assert i.cycles_per_action * cfg["env"]["policy_rate_hz"] == pytest.approx(spec.limits.control_rate_hz)


@pytest.mark.gpu
def test_sizes_and_mirror(env):
    frame = env.actor_layout.size
    assert env.actor_obs_size == frame * env.history
    expected = 3 + 3 + (3 if env.cfg["observation"].get("accelerometer") else 0) + 4 + 4 + 2 + 6 + env.n_command
    assert frame == expected
    rng = np.random.default_rng(0)
    obs = rng.normal(size=env.actor_obs_size)
    assert np.allclose(env.mirror_actor_obs(env.mirror_actor_obs(obs)), obs)
    critic = rng.normal(size=env.critic_obs_size)
    assert np.allclose(env.mirror_critic_obs(env.mirror_critic_obs(critic)), critic)
    a = rng.normal(size=6)
    assert np.allclose(env.mirror_action(env.mirror_action(a)), a)


@pytest.mark.gpu
def test_interface_matches_environment(env, cfg, spec):
    import jax.numpy as jp

    iface = PolicyInterface(cfg, spec)
    rng = np.random.default_rng(1)
    for _ in range(5):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)
        sensed = np.concatenate([q, rng.normal(size=3), rng.normal(size=3) * 9.81,
                                 spec.stand_q + rng.normal(size=6) * 0.2, rng.normal(size=6) * 5])
        last_action = rng.uniform(-1, 1, 6)
        ref = np.array([rng.uniform(-0.5, 0.5), rng.uniform(-1.5, 1.5), rng.uniform(0.14, 0.185),
                        rng.uniform(-0.3, 0.3)])[: env.n_command]
        info = {"params": {"mount": jp.eye(3), "gyro_bias": jp.zeros(3), "vel_gain": jp.ones(6)},
                "command_ref": jp.asarray(ref), "last_action": jp.asarray(last_action)}
        want, _ = env._actor_frame(jp.asarray(sensed, dtype=jp.float32), info)
        got = iface.frame(sensed[0:4], sensed[4:7], sensed[7:10], sensed[10:16], sensed[16:22], last_action, ref)
        np.testing.assert_allclose(got, np.asarray(want), rtol=1e-5, atol=1e-5)

        action = rng.uniform(-1.2, 1.2, 6)
        params = {"gain_scale": jp.float32(1.0)}
        np.testing.assert_allclose(iface.frames(action), np.asarray(env._frames(jp.asarray(action), params)),
                                   rtol=1e-5, atol=1e-6)

    # Command shaping, one policy step.
    ref, cmd = np.array([0.1, 0.2, 0.17, 0.0])[: env.n_command], np.array([0.5, -1.0, 0.14, 0.25])[: env.n_command]
    shaped = iface.shape(ref, cmd)
    assert shaped[0] == pytest.approx(0.1 + env.accel_limit * env.dt)
    assert shaped[1] == pytest.approx(0.2 + (-1.2) * (1 - math.exp(-env.dt / env.yaw_tau)))
    assert shaped[2] == pytest.approx(0.17 - env.h_rate * env.dt)
    if env.has_roll:
        assert shaped[3] == pytest.approx(env.roll_rate * env.dt)
        for h in (0.08, 0.14, 0.19):
            for roll in (-0.3, 0.1, 0.3):
                assert iface.feasible_roll(roll, h) == pytest.approx(
                    float(env.feasible_roll(jp.float32(roll), jp.float32(h))), abs=1e-6)


def test_actor_sees_no_privileged_state(cfg, spec):
    """The actor frame is built from IMU, encoders, previous action and command only."""
    i = PolicyInterface(cfg, spec)
    rng = np.random.default_rng(2)
    args = [rng.normal(size=4), rng.normal(size=3), rng.normal(size=3), rng.normal(size=6), rng.normal(size=6),
            rng.normal(size=6), np.array([0.1, 0.2, 0.16, 0.1])]
    frame = i.frame(*args)
    assert frame.shape == (26 if i.has_roll else 25,)
    assert np.all(np.isfinite(frame))
