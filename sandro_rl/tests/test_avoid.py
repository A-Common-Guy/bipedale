"""The avoid task (configs/avoid.yaml) on the CPU, through MJX's JAX backend.

The JAX backend collides only the tyres with the floor (env.py), which is
enough here: the scanner, the clearances, the goals and their placement are
analytic and do not depend on contacts.
"""

import math

import jax
import jax.numpy as jp
import mujoco
import numpy as np
import pytest

from sandro_rl import config as config_mod
from sandro_rl.interface import GoalOdometry, PolicyInterface, goal_observation, ray_observation
from sandro_rl.robot import RobotSpec

OVERRIDES = ["env.impl=jax", "env.graph_mode=null"]


@pytest.fixture(scope="module")
def cfg():
    return config_mod.load("avoid.yaml", OVERRIDES)


@pytest.fixture(scope="module")
def env(cfg):
    from sandro_rl.env import SandroEnv
    return SandroEnv(cfg)


@pytest.fixture(scope="module")
def states(env):
    """Resets of 32 worlds on the nominal model."""
    params = jp.zeros((32, 8))
    keys = jax.random.split(jax.random.key(0), 32)
    return jax.jit(jax.vmap(lambda k, p: env.reset(env.mjx_model, k, p)))(keys, params)


def test_layout_and_mirror(env):
    frame = env.actor_layout.size
    assert frame == 25 + 3 + env.n_rays
    assert env.actor_obs_size == frame * env.history
    rng = np.random.default_rng(0)
    obs = rng.normal(size=env.actor_obs_size)
    assert np.allclose(env.mirror_actor_obs(env.mirror_actor_obs(obs)), obs)
    critic = rng.normal(size=env.critic_obs_size)
    assert np.allclose(env.mirror_critic_obs(env.mirror_critic_obs(critic)), critic)
    # Mirroring the scan reverses the rays: ray k looks where ray n-1-k looks, mirrored.
    angles = np.asarray(env.ray_angles)
    np.testing.assert_allclose(angles[::-1], -angles, atol=1e-6)


def test_starts_clear_of_pillars_with_a_free_goal(env, states):
    data = states.data
    for i in range(32):
        one = jax.tree.map(lambda x: x[i], data)
        mid = np.asarray(env._axle_mid(one))
        r_body = np.asarray(env._body_frame(one)[1])
        heading = r_body[:2, 0] / np.linalg.norm(r_body[:2, 0])
        start = float(env.terrain.clearance(jp.asarray(mid[:2]), jp.asarray(heading), env.hull_half, one))
        assert start > 0.1, f"world {i} starts {start:.3f} m from a pillar"
        goal = np.asarray(states.info["goal"][i])
        assert float(env._goal_free(jp.asarray(goal), one)) > 0.1, f"world {i} has its goal on a pillar"
        assert 1.4 < np.linalg.norm(goal - mid[:2]) < 4.1
    assert np.all((np.asarray(states.obs["actor"]) >= -10) & np.isfinite(np.asarray(states.obs["actor"])))
    rays = np.asarray(states.info["ray_obs"])
    assert np.all((rays >= 0) & (rays <= 1)) and np.mean(rays < 1) > 0.05


def test_scanner_matches_mj_ray(env, states):
    m = env.spec.model
    d = mujoco.MjData(m)
    geom = np.zeros(1, np.int32)
    groups = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
    for i in range(8):
        d.qpos[:] = np.asarray(states.data.qpos[i])
        d.mocap_pos[:], d.mocap_quat[:] = np.asarray(states.data.mocap_pos[i]), np.asarray(states.data.mocap_quat[i])
        mujoco.mj_forward(m, d)
        one = jax.tree.map(lambda x: x[i], states.data)
        got = np.asarray(env._scan(one, env._body_frame(one)[1]))
        r_imu = d.site_xmat[env.spec.imu_site].reshape(3, 3)
        r_body = r_imu @ np.asarray(env.body_from_imu)
        origin = d.site_xpos[env.spec.imu_site] + r_body @ np.asarray(env.ray_offset)
        for a, value in zip(np.asarray(env.ray_angles), got):
            direction = r_body @ np.array([math.cos(a), math.sin(a), 0.0])
            want = mujoco.mj_ray(m, d, origin, direction, groups, 1, -1, geom)
            want = env.ray_range if want < 0 else min(want, env.ray_range)
            assert value == pytest.approx(want, abs=1e-3)


def test_interface_frame_matches_environment(env, cfg, states):
    iface = PolicyInterface(cfg, RobotSpec.load())
    assert iface.has_goal and iface.n_rays == env.n_rays
    rng = np.random.default_rng(3)
    q = rng.normal(size=4)
    q /= np.linalg.norm(q)
    sensed = np.concatenate([q, rng.normal(size=3), rng.normal(size=3), np.asarray(env.stand_q) + 0.1, rng.normal(size=6)])
    goal = np.asarray(goal_observation(1.2, -0.7, iface.goal_range))
    rays = np.asarray(ray_observation(rng.uniform(0, 3, env.n_rays), env.ray_range))
    info = {"params": {"mount": jp.eye(3), "gyro_bias": jp.zeros(3), "vel_gain": jp.ones(6)},
            "command_ref": jp.asarray([0.8, 0.0, 0.16]), "last_action": jp.zeros(6),
            "goal_obs": jp.asarray(goal, jp.float32), "ray_obs": jp.asarray(rays, jp.float32)}
    want, _ = env._actor_frame(jp.asarray(sensed, jp.float32), info)
    got = iface.frame(sensed[0:4], sensed[4:7], sensed[7:10], sensed[10:16], sensed[16:22], np.zeros(6),
                      [0.8, 0.0, 0.16], goal=goal, rays=rays)
    np.testing.assert_allclose(got, np.asarray(want), rtol=1e-5, atol=1e-5)
    np.testing.assert_allclose(np.asarray(goal_observation(jp.float32(1.2), jp.float32(-0.7), 4.0, jp)),
                               goal, atol=1e-6)
    np.testing.assert_allclose(goal, [math.hypot(1.2, 0.7) / 4.0, 1.2 / math.hypot(1.2, 0.7),
                                      -0.7 / math.hypot(1.2, 0.7)], atol=1e-6)


def test_touching_a_pillar_terminates(env, states):
    one = jax.tree.map(lambda x: x[0], states.data)
    info = jax.tree.map(lambda x: x[0], states.info)
    mid = env._axle_mid(one)
    k = env.terrain.pillar_items[0]
    mocap = env.terrain.mocap_ids[k]
    for dx, expected in ((0.0, 1.0), (3.0, 0.0)):
        moved = one.replace(mocap_pos=one.mocap_pos.at[mocap].set(jp.array([mid[0] + dx, mid[1], 0.6])))
        others = moved.mocap_pos.at[np.asarray(env.terrain.mocap_ids[env.terrain.pillar_items[1:]])].set(
            jp.array([0.0, 0.0, -5.0]))
        moved = moved.replace(mocap_pos=others)
        truth = env._truth(moved, info)
        assert float(env._terminated(truth)) == expected


@pytest.mark.parametrize("layout", ["slalom", "wall"])
def test_evaluation_course_pillars_collide_where_drawn(layout):
    from sandro_rl import evaluate

    spec = RobotSpec.load()
    m, goals, pillars = evaluate.avoid_course_model(spec, layout)
    assert len(pillars) == len(evaluate.AVOID_COURSES[layout]["pillars"]) and len(goals)
    d = mujoco.MjData(m)
    stand = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "stand")
    robot = [g for g in range(m.ngeom) if m.geom_bodyid[g] != 0 and g not in pillars
             and (m.geom_contype[g] or m.geom_conaffinity[g])]
    fromto = np.zeros(6)

    def gap():
        mujoco.mj_forward(m, d)
        return min(mujoco.mj_geomDistance(m, d, g, p, 2.0, fromto) for g in robot for p in pillars)

    mujoco.mj_resetDataKeyframe(m, d, stand)
    assert gap() > 0.5
    axle = d.xanchor[spec.wheel_joint_ids].mean(axis=0)
    d.qpos[0:2] += d.geom_xpos[pillars[0]][:2] - axle[:2]     # the robot moved onto the first pillar
    assert gap() <= 0.0


@pytest.mark.gpu
def test_avoid_steps_in_warp():
    """The task on the training backend: a batch steps with finite observations
    and the contact buffers hold (pillars collide in MuJoCo Warp)."""
    from sandro_rl import train

    config = config_mod.load("avoid.yaml", ["train.num_envs=64"])
    env, agent, reset, autoreset_step, _, _ = train.build(config)
    step = jax.jit(autoreset_step)
    state = jax.jit(reset)(jax.random.key(0))
    for i in range(20):
        state = step(state, jp.full((64, 6), 0.3), jax.random.key(i))
    assert np.all(np.isfinite(np.asarray(state.obs["actor"])))
    assert np.all(np.isfinite(np.asarray(state.obs["critic"])))


def test_goal_odometry_follows_the_robot():
    odo = GoalOdometry(0.0672)
    odo.set_goal(2.0, 0.0, yaw=0.3)
    dt, w = 0.01, 0.0
    yaw = 0.3
    # Drive 1 m straight, then turn 90 degrees left on the spot.
    for _ in range(100):
        odo.update([1.0 / 0.0672, 1.0 / 0.0672], 0.0, yaw, dt)
    for _ in range(100):
        yaw += 0.5 * math.pi * dt
        odo.update([0.0, 0.0], w, yaw, dt)
    forward, left = odo.relative()
    assert forward == pytest.approx(0.0, abs=1e-6)
    assert left == pytest.approx(-1.0, abs=1e-6)
