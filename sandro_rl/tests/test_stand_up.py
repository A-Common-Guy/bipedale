"""The stand-up task's model: leg collision boxes and the switched-off poses."""

import mujoco
import numpy as np
import pytest

from sandro_rl import collision, fallen, kinematics, terrain
from sandro_rl.robot import RobotSpec


@pytest.fixture(scope="module")
def spec():
    return RobotSpec.load()


@pytest.fixture(scope="module")
def model(spec):
    return terrain.build(spec, {}, leg_boxes=True)


def test_boxes_leave_the_dynamics_alone(spec, model):
    assert np.allclose(model.body_mass, spec.model.body_mass)
    assert np.allclose(model.body_inertia, spec.model.body_inertia)
    assert model.ngeom == spec.model.ngeom + len(collision.leg_bodies(spec))


def test_standing_robot_touches_only_with_its_tyres(spec, model):
    d = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, d, mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand"))
    mujoco.mj_forward(model, d)
    tyres = set(spec.tyre_geoms.tolist())
    for c in d.contact[: d.ncon]:
        assert {c.geom1, c.geom2} - {spec.floor_geom} <= tyres


def test_limp_legs_stop_against_their_links(spec, model):
    """Unpowered the legs fold until links meet, not through each other (> 5 rad without the boxes)."""
    d = mujoco.MjData(model)
    d.qpos[:] = fallen.tipped_qpos(spec, model, 0.3)
    for _ in range(int(3.0 / model.opt.timestep)):
        mujoco.mj_step(model, d)
    legs = np.asarray(spec.joint_qpos)[list(spec.leg_index)]
    assert np.all(np.abs(d.qpos[legs]) < 2.6)
    heights, _, _ = kinematics.leg_state(d.qpos[legs], spec.limits.leg_geometry, spec.limits.fore_aft_neutral_m, np)
    assert np.all(heights > 0.04)
    assert np.max(np.abs(d.qvel)) < 0.05


def test_switched_off_poses_match_the_model(model):
    states = fallen.load(nq=model.nq)
    assert len(states) > 500
    d = mujoco.MjData(model)
    tilts = []
    for qpos in states[::25]:
        d.qpos[:] = qpos
        mujoco.mj_kinematics(model, d)
        tilts.append(np.arccos(np.clip(d.xmat[1].reshape(3, 3)[2, 2], -1.0, 1.0)))
    assert np.all(np.isfinite(tilts))
