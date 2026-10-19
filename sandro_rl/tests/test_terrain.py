"""Obstacles must collide where they are drawn, on the CPU and in MuJoCo Warp.

Static world geoms only collide where they were compiled, so moving one
through the model afterwards gives a ghost obstacle; terrain.py uses mocap
bodies instead. These tests put a plank under the wheels at run time and
check that it lifts the robot, and that the analytic ground height used by
the rewards and terminations agrees with ray casts.
"""

import jax
import jax.numpy as jp
import mujoco
import numpy as np
import pytest

from sandro_rl import evaluate, terrain
from sandro_rl.robot import RobotSpec

CFG = {"type": "obstacles",
       "planks": {"count": 4, "height_m": [0.005, 0.03], "depth_m": [0.04, 0.3], "width_m": [0.2, 1.0],
                  "radius_m": [0.5, 3.0]},
       "hills": {"count": 2, "height_m": [0.02, 0.1], "slope_deg": [5.0, 15.0], "plateau_m": [0.2, 0.8],
                 "width_m": [0.8, 1.6], "radius_m": [0.3, 3.0]}}
PLANK_TOP = 0.02


@pytest.fixture(scope="module")
def world():
    spec = RobotSpec.load()
    m = terrain.build(spec, CFG)
    t = terrain.Terrain(RobotSpec.load(spec.scene, model=m), CFG)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "stand"))
    mujoco.mj_forward(m, d)
    axle = d.xanchor[spec.wheel_joint_ids].mean(axis=0)
    return spec, m, t, d.qpos.copy(), axle


DROP_M, SETTLE_STEPS = 0.03, 200


def _settle_cpu(m, wheels, qpos, mocap_pos):
    """Axle height after dropping the robot from DROP_M onto the terrain."""
    d = mujoco.MjData(m)
    d.qpos[:] = qpos
    d.qpos[2] += DROP_M
    d.mocap_pos[:] = mocap_pos
    for _ in range(SETTLE_STEPS):
        mujoco.mj_step(m, d)
    return d.xanchor[wheels][:, 2].mean()


def _mocap_with_plank_under(m, t, axle):
    d = mujoco.MjData(m)
    pos = d.mocap_pos.copy()
    pos[t.mocap_ids[0]] = [axle[0], axle[1], PLANK_TOP]
    return d.mocap_pos.copy(), pos


def test_plank_placed_at_run_time_collides_on_cpu(world):
    spec, m, t, qpos, axle = world
    parked, under = _mocap_with_plank_under(m, t, axle)
    lift = _settle_cpu(m, spec.wheel_joint_ids, qpos, under) - _settle_cpu(m, spec.wheel_joint_ids, qpos, parked)
    assert lift == pytest.approx(PLANK_TOP, abs=2e-3), "the plank under the wheels does not carry them"


def test_obstacles_do_not_touch_each_other_or_the_floor(world):
    _, m, t, qpos, _ = world
    d = mujoco.MjData(m)
    pos, quat = t.layout(jax.random.PRNGKey(0))
    d.mocap_pos[:], d.mocap_quat[:] = np.asarray(pos), np.asarray(quat)
    d.qpos[:] = qpos
    d.qpos[0] += 50.0
    mujoco.mj_forward(m, d)
    robot_free = [c for c in d.contact[:d.ncon]
                  if m.geom_bodyid[c.geom1] in t.body_ids or m.geom_bodyid[c.geom2] in t.body_ids]
    assert not robot_free


def test_ground_height_matches_ray_casts(world):
    _, m, t, _, _ = world
    d = mujoco.MjData(m)
    pos, quat = t.layout(jax.random.PRNGKey(3))
    d.mocap_pos[:], d.mocap_quat[:] = np.asarray(pos), np.asarray(quat)
    mujoco.mj_forward(m, d)
    rng = np.random.default_rng(0)
    points = np.concatenate([np.asarray(pos)[k, :2] + rng.uniform(-1.2, 1.2, (40, 2)) for k in range(len(t.items))])

    class Data:
        mocap_pos = jp.asarray(d.mocap_pos)
        mocap_quat = jp.asarray(d.mocap_quat)

    heights = np.asarray(t.ground_height(jp.asarray(np.c_[points, np.zeros(len(points))]), Data))
    geom = np.zeros(1, np.int32)
    for p, h in zip(points, heights):
        dist = mujoco.mj_ray(m, d, np.array([p[0], p[1], 1.0]), np.array([0.0, 0.0, -1.0]), None, 1, -1, geom)
        assert abs((1.0 - dist) - h) < 1e-5
    assert heights.max() > 0.01


def test_course_obstacles_collide(world):
    spec = world[0]
    m = evaluate.course_model(spec)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "stand"))
    mujoco.mj_forward(m, d)
    axle = d.xanchor[spec.wheel_joint_ids].mean(axis=0)
    names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or "" for b in range(m.nbody)]
    for b, name in enumerate(names):
        if not name.startswith(terrain.OBSTACLE_PREFIX):
            continue
        mujoco.mj_resetDataKeyframe(m, d, 0)
        d.qpos[0:2] += m.body_pos[b][:2] - axle[:2]
        mujoco.mj_forward(m, d)
        hits = [c for c in d.contact[:d.ncon] if b in (m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2])]
        assert hits, f"{name} does not collide where it is drawn"


@pytest.mark.gpu
def test_plank_placed_at_run_time_collides_in_warp(world):
    import warp as wp
    from mujoco import mjx

    spec, m, t, qpos, axle = world
    wheels = spec.wheel_joint_ids
    parked, under = _mocap_with_plank_under(m, t, axle)
    mx = mjx.put_model(m, impl="warp", graph_mode=wp.JaxCallableGraphMode.WARP_STAGED)
    step = jax.jit(lambda x: mjx.step(mx, x))

    def settle(mocap):
        q = np.array(qpos)
        q[2] += DROP_M
        d = mjx.make_data(m, impl="warp", naconmax=64, njmax=256).replace(
            qpos=jp.asarray(q, jp.float32), mocap_pos=jp.asarray(mocap, jp.float32))
        for _ in range(SETTLE_STEPS):
            d = step(d)
        return float(np.asarray(d.xanchor)[wheels][:, 2].mean())

    gpu_lift = settle(under) - settle(parked)
    assert gpu_lift == pytest.approx(PLANK_TOP, abs=2e-3), "the plank under the wheels does not carry them in Warp"
