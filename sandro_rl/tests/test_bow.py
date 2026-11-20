"""The lean command (configs/fun_bow.yaml): the roll channel as the body's pitch.

On the CPU through MJX's JAX backend.
"""

import jax
import jax.numpy as jp
import numpy as np
import pytest

from sandro_rl import config as config_mod

OVERRIDES = ["env.impl=jax", "env.graph_mode=null"]


@pytest.fixture(scope="module")
def env():
    from sandro_rl.env import SandroEnv
    return SandroEnv(config_mod.load("fun_bow.yaml", OVERRIDES))


def test_the_controllers_roll_clip_never_binds(env):
    """RlPolicyController shapes the channel as a roll: over the commanded heights its clip
    must leave the whole channel alone, or the stick would lean less than trained."""
    assert env.has_lean and not env.has_split
    for height in np.linspace(env.h_min, env.h_max, 11):
        assert float(env.feasible_roll(env.roll_max, height)) == pytest.approx(env.roll_max)
        assert float(env.feasible_roll(-env.roll_max, height)) == pytest.approx(-env.roll_max)


def test_feasible_lean_puts_the_axle_under_the_centre_of_mass(env):
    """The limit is the lean at which the farthest axle the legs reach is under a centre of
    mass com_below_pivots_m under the pivots; the model's own is there at the stand pose."""
    import mujoco
    g, lean = env.geometry["right"], env.lean
    for height in (0.11, 0.13, 0.15):
        radius = lean["stretch"] * (g["upper_m"] + g["lower_m"])
        reach = np.sqrt(radius ** 2 - height ** 2) - 0.5 * g["base_m"]
        limit = float(env.feasible_lean(2.0, height))
        assert np.tan(limit) * (height - lean["com_below_pivots_m"]) == pytest.approx(reach, abs=1e-5)
        assert float(env.feasible_lean(-2.0, height)) == pytest.approx(-limit)
        assert float(env.feasible_lean(0.1, height)) == pytest.approx(0.1)
    assert float(env.feasible_lean(lean["gain"] * env.roll_max, 0.13)) == pytest.approx(lean["gain"] * env.roll_max)
    s = env.spec
    m, d = s.model, mujoco.MjData(s.model)
    d.qpos[:] = s.stand_qpos
    mujoco.mj_forward(m, d)
    wheels = {int(m.jnt_bodyid[j]) for j in s.wheel_joint_ids}
    bodies = [b for b in range(1, m.nbody)
              if b not in wheels and m.body_rootid[b] == m.body_rootid[s.torso_body]]
    com = sum(m.body_mass[b] * d.xipos[b] for b in bodies) / sum(m.body_mass[b] for b in bodies)
    axle = d.xanchor[s.wheel_joint_ids].mean(0)
    from sandro_rl import kinematics
    height = kinematics.leg_state(s.stand_q[np.asarray(env.leg_idx)], env.geometry, env.neutral)[0].mean()
    assert height - (com[2] - axle[2]) == pytest.approx(lean["com_below_pivots_m"], abs=0.005)


def test_the_lean_command_keeps_its_sign_in_the_mirror(env):
    """A bow is a bow in the mirror; a roll or a split would change side."""
    layout = env.actor_layout
    start = int(sum(layout.sizes[: list(layout.names).index("command")]))
    assert env.n_command == 4
    assert layout.perm[start + 3] == start + 3 and layout.sign[start + 3] == 1
    assert layout.sign[start + 1] == -1


def test_a_nod_is_commanded_shaped_and_measured(env):
    """Through steps: a wave drives the command, the reference follows at the rate limit and
    the logged error and the pitch term are about the lean asked for (gain times the channel)."""
    step = jax.jit(lambda s, a: env.step(env.mjx_model, s, a, jp.zeros(8)))
    state = jax.jit(lambda k: env.reset(env.mjx_model, k, jp.zeros(8)))(jax.random.key(3))
    wave = jp.array([0.2, 2 * np.pi * 1.0, 0.0])
    state = state.replace(info={**state.info, "split_wave": wave, "command_timer": jp.float32(10.0)})
    refs, commands = [], []
    for _ in range(40):
        state = step(state, jp.zeros(6))
        refs.append(float(state.info["command_ref"][3]))
        commands.append(float(state.info["command"][3]))
    t = env.dt * np.arange(1, 41)
    np.testing.assert_allclose(commands, 0.2 * np.sin(2 * np.pi * t), atol=2e-3)
    assert np.max(np.abs(np.diff(refs))) <= env.roll_rate * env.dt + 1e-6
    assert max(refs) > 0.1
    r = np.asarray(env._body_frame(state.data)[1])
    target = float(env.feasible_lean(env.lean["gain"] * refs[-1], state.info["command_ref"][2]))
    assert abs(target) > 0.05
    pitch = np.arctan2(-r[2, 0], np.hypot(r[0, 0], r[1, 0]))
    assert float(state.metrics["err_roll"]) == pytest.approx(abs(pitch - target), abs=1e-4)
