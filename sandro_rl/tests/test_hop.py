"""The hop paid to a fixed clearance (configs/fun_hop_high.yaml: commands.jump.target_m).

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
    return SandroEnv(config_mod.load("fun_hop_high.yaml", OVERRIDES))


def test_a_request_is_paid_to_the_target_whatever_height_it_names(env):
    """joy_teleop asks for 5 cm: 6 cm up earns 6 / 25 of the clearance term, not all of it,
    and nothing counts as overshoot below the target."""
    target = env.jump["target_m"]
    assert target > env.jump_h_max                      # the observation keeps the lineage's scale
    step = jax.jit(lambda s, a: env.step(env.mjx_model, s, a, jp.zeros(8)))
    state = jax.jit(lambda k: env.reset(env.mjx_model, k, jp.zeros(8)))(jax.random.key(1))
    qpos = state.data.qpos.at[2].add(0.06)
    state = state.replace(data=state.data.replace(qpos=qpos),
                          info={**state.info, "jump_timer": jp.float32(0.5), "jump_height": jp.float32(0.05),
                                "jump_wait": jp.float32(5.0)})
    state = step(state, jp.zeros(6))
    clearance = float(state.metrics["clearance_min"])
    assert 0.04 < clearance < 0.07
    paid = env.cfg["rewards"]["weights"]["jump_clearance"] * env.dt      # the metrics are weighted, per step
    assert float(state.metrics["jump_clearance"]) == pytest.approx(paid * clearance / target, rel=1e-4)
    assert float(state.metrics["jump_overshoot"]) == 0.0


def test_a_flip_counts_the_turn_and_waits_with_the_fall():
    """configs/fun_flip.yaml: in the window the turn about the pitch axis is integrated and
    paid, and upside down in the air is not a fall; outside a window it is."""
    from sandro_rl.env import SandroEnv
    env = SandroEnv(config_mod.load("fun_flip.yaml", OVERRIDES))
    step = jax.jit(lambda s, a: env.step(env.mjx_model, s, a, jp.zeros(8)))
    state = jax.jit(lambda k: env.reset(env.mjx_model, k, jp.zeros(8)))(jax.random.key(1))
    assert float(state.info["flip_angle"]) == 0.0
    # 0.4 m up, turning nose up at 10 rad/s (the torso frame's z axis is the pitch axis).
    qpos = state.data.qpos.at[2].add(0.4)
    qvel = state.data.qvel.at[5].set(10.0)
    data = state.data.replace(qpos=qpos, qvel=qvel)
    quiet = {"jump_wait": jp.float32(5.0), "push_wait": jp.float32(5.0)} if "push_wait" in state.info else {
        "jump_wait": jp.float32(5.0)}
    flying = state.replace(data=data, info={**state.info, **quiet, "jump_timer": jp.float32(0.9)})
    for _ in range(12):
        flying = step(flying, jp.zeros(6))
    assert float(flying.info["flip_angle"]) == pytest.approx(1.2, abs=0.25)
    assert float(flying.metrics["flip_rate"]) > 0.0
    assert float(flying.metrics["tilt"]) > 0.8 and float(flying.terminated) == 0.0
    standing = state.replace(data=data, info={**state.info, **quiet})
    for _ in range(12):
        standing = step(standing, jp.zeros(6))
        if float(standing.done):
            break
    assert float(standing.info["flip_angle"]) == 0.0 or float(standing.done) == 1.0
    assert float(standing.done) == 1.0
