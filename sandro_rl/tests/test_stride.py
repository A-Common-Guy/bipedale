"""The stride command (configs/fun_stride.yaml): the roll channel as the legs' fore-aft split.

On the CPU through MJX's JAX backend.
"""

import jax
import jax.numpy as jp
import numpy as np
import pytest

from sandro_rl import config as config_mod
from sandro_rl import kinematics

OVERRIDES = ["env.impl=jax", "env.graph_mode=null"]


@pytest.fixture(scope="module")
def env():
    from sandro_rl.env import SandroEnv
    return SandroEnv(config_mod.load("fun_stride.yaml", OVERRIDES))


def test_the_controllers_roll_clip_never_binds(env):
    """RlPolicyController shapes the channel as a roll: over the commanded heights its
    clip must leave the largest split alone, or the stick would do less than trained."""
    for height in np.linspace(env.h_min, env.h_max, 11):
        assert float(env.feasible_roll(env.roll_max, height)) == pytest.approx(env.roll_max)
        assert float(env.feasible_roll(-env.roll_max, height)) == pytest.approx(-env.roll_max)


def test_feasible_split_is_what_the_legs_reach(env):
    g = env.geometry["right"]
    theta = np.linspace(-2.094, 2.094, 500)
    inner, outer = np.meshgrid(theta, theta)
    x, y, stretch = kinematics.forward(inner, outer, g["base_m"], g["upper_m"], g["lower_m"])
    offset = x - env.neutral
    for height in (0.10, 0.13, 0.15):
        reachable = (np.abs(y - height) < 0.002) & (stretch < env.split["stretch"])
        limit = float(env.feasible_split(1.0, height))
        assert limit == pytest.approx(2 * offset[reachable].max(), abs=0.012)
        assert float(env.feasible_split(-1.0, height)) == pytest.approx(-limit)
        assert float(env.feasible_split(0.02, height)) == pytest.approx(0.02)


def test_the_split_command_mirrors_with_a_sign(env):
    """Right wheel ahead becomes left wheel ahead in the mirror, as a roll changes side."""
    layout = env.actor_layout
    start = int(sum(layout.sizes[: list(layout.names).index("command")]))
    assert env.n_command == 4
    assert layout.perm[start + 3] == start + 3 and layout.sign[start + 3] == -1


def test_a_stride_is_commanded_shaped_and_measured(env):
    """Through steps: a wave drives the command, the reference follows at the rate limit
    and the logged error is the split's distance from the feasible reference."""
    step = jax.jit(lambda s, a: env.step(env.mjx_model, s, a, jp.zeros(8)))
    state = jax.jit(lambda k: env.reset(env.mjx_model, k, jp.zeros(8)))(jax.random.key(3))
    wave = jp.array([0.12, 2 * np.pi * 1.0, 0.0])
    state = state.replace(info={**state.info, "split_wave": wave, "command_timer": jp.float32(10.0)})
    refs, commands = [], []
    for _ in range(60):
        state = step(state, jp.zeros(6))
        refs.append(float(state.info["command_ref"][3]))
        commands.append(float(state.info["command"][3]))
    t = env.dt * np.arange(1, 61)
    np.testing.assert_allclose(commands, 0.12 * np.sin(2 * np.pi * t), atol=2e-3)
    assert np.max(np.abs(np.diff(refs))) <= env.roll_rate * env.dt + 1e-6
    assert max(refs) > 0.08
    q = np.asarray(state.data.qpos)[np.asarray(env.qadr)][np.asarray(env.leg_idx)]
    _, offsets, _ = kinematics.leg_state(q, env.geometry, env.neutral)
    want = abs(offsets[0] - offsets[1] - float(env.feasible_split(refs[-1], state.info["command_ref"][2])))
    assert float(state.metrics["err_roll"]) == pytest.approx(want, abs=1e-4)


def test_a_resumed_actor_keeps_its_joint_targets_on_a_wider_action_range(env):
    """agile_terrain_v6 acts within 0.5 rad of the stand pose, the stride within 1 rad:
    the rescaled checkpoint must command the same targets from the same situation."""
    from pathlib import Path

    from sandro_rl import checkpoint
    from sandro_rl.networks import MLP, Normalizer
    from sandro_rl.train import rescale_actions

    payload, meta = checkpoint.load(Path(__file__).resolve().parents[1] / "policies" / "agile_terrain_v6")
    old_scale = meta["config"]["env"]["action"]["legs"]["scale_rad"]
    new_scale = env.cfg["action"]["legs"]["scale_rad"]
    assert old_scale != new_scale
    scaled = rescale_actions(payload, meta["config"], env)
    net = MLP(tuple(meta["config"]["train"]["actor_hidden"]), 6, meta["config"]["train"]["activation"])
    act = lambda p, obs: np.asarray(net.apply(jax.tree.map(jp.asarray, p["params"]["actor"]),
                                              Normalizer(**{k: jp.asarray(v) for k, v in p["actor_norm"].items()})(obs)))
    rng = np.random.default_rng(0)
    obs = rng.normal(size=(16, env.actor_obs_size)) * 0.3
    layout, legs = env.actor_layout, np.asarray(env.leg_idx)
    start = int(sum(layout.sizes[: list(layout.names).index("last_action")]))
    wide = obs.copy()                       # the same last joint targets, in the new action units
    for k in range(env.history):
        wide[:, k * layout.size + start + legs] *= old_scale / new_scale
    before, after = act(payload, jp.asarray(obs)), act(scaled, jp.asarray(wide))
    np.testing.assert_allclose(after[:, legs] * new_scale, before[:, legs] * old_scale, atol=1e-4)
    wheels = np.asarray(env.wheel_idx)
    np.testing.assert_allclose(after[:, wheels], before[:, wheels], atol=1e-4)
