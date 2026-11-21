"""The one-wheel stand's honesty terms (configs/fun_one_leg_v4.yaml: commands.jump.settle_s).

On the CPU through MJX's JAX backend.
"""

import jax
import jax.numpy as jp
import pytest

from sandro_rl import config as config_mod

OVERRIDES = ["env.impl=jax", "env.graph_mode=null"]


@pytest.fixture(scope="module")
def env():
    from sandro_rl.env import SandroEnv
    return SandroEnv(config_mod.load("fun_one_leg_v4.yaml", OVERRIDES))


def _at(env, timer, lift=0.0):
    step = jax.jit(lambda s, a: env.step(env.mjx_model, s, a, jp.zeros(8)))
    state = jax.jit(lambda k: env.reset(env.mjx_model, k, jp.zeros(8)))(jax.random.key(1))
    info = {**state.info, "jump_timer": jp.float32(timer), "jump_height": jp.float32(0.05),
            "jump_wait": jp.float32(20.0)}
    if "push_wait" in info:
        info["push_wait"] = jp.float32(20.0)
    data = state.data.replace(qpos=state.data.qpos.at[2].add(lift))
    return step(state.replace(data=data, info=info), jp.zeros(6))


def test_both_wheels_down_is_a_tap_once_the_window_has_settled(env):
    """Standing on two wheels in the window: free for settle_s, then it costs per second;
    the left tyre on the floor is not a flight."""
    window, settle = env.jump["window_s"], env.jump["settle_s"]
    weights = env.cfg["rewards"]["weights"]
    early = _at(env, window - 0.5 * settle)
    assert float(early.metrics["one_leg_tap"]) == 0.0 and float(early.metrics["one_leg_flight"]) == 0.0
    late = _at(env, window - settle - 0.5)
    assert float(late.metrics["one_leg_tap"]) == pytest.approx(weights["one_leg_tap"] * env.dt)
    assert float(late.metrics["one_leg_flight"]) == 0.0
    outside = _at(env, -5.0)
    assert float(outside.metrics["one_leg_tap"]) == 0.0


def test_off_the_floor_is_a_flight_not_a_stand(env):
    """Both wheels 6 cm up: the left tyre off the floor costs, no clearance is paid, and the
    raised wheel that high is not a tap."""
    weights = env.cfg["rewards"]["weights"]
    state = _at(env, env.jump["window_s"] - env.jump["settle_s"] - 0.5, lift=0.06)
    assert float(state.metrics["one_leg_flight"]) == pytest.approx(weights["one_leg_flight"] * env.dt)
    assert float(state.metrics["one_leg_tap"]) == 0.0
    assert float(state.metrics["jump_clearance"]) == 0.0


def test_a_knee_near_the_floor_costs(env):
    """The knees stand 17 cm up: nothing. Sunk until a knee hinge is under the margin for a step (the
    simulation collides only tyres and torso, so a knee can pass through the floor): paid."""
    weights = env.cfg["rewards"]["weights"]
    assert len(env.knee_joints) == 4
    assert float(_at(env, -5.0).metrics["knee_low"]) == 0.0
    sunk = _at(env, -5.0, lift=-0.16)
    assert float(sunk.metrics["knee_low"]) < 0.1 * weights["knee_low"] * env.dt


def test_the_stand_pays_more_the_longer_it_is_unbroken(env):
    """On two wheels the streak stays 0. With a streak the same pose earns more, up to
    streak_s; nothing is counted outside a window."""
    assert float(_at(env, 4.0).info["one_leg_streak"]) == 0.0
    assert float(_at(env, -5.0).info["one_leg_streak"]) == 0.0
