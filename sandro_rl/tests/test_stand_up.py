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


# The robot's outline on the floor (collision.add_floor_hulls, docs/DESIGN.md 30).

def test_a_hull_of_32_vertices_is_inside_the_cad_hull_and_within_5_mm_of_it(spec):
    whole = collision.floor_hulls(spec, 0)
    coarse = collision.floor_hulls(spec, 32)
    assert set(whole) == {f"{name}_floor_hull" for name in collision.leg_bodies(spec)} | {"torso_floor_hull"}
    for name, (body, points) in coarse.items():
        assert body == whole[name][0] and len(points) <= 32
        # Every chosen vertex is one of the hull's own, so the coarse hull lies inside it.
        gaps = np.linalg.norm(points[:, None] - whole[name][1][None], axis=2).min(axis=1)
        assert gaps.max() < 1e-9
        assert 0.0 <= collision.outline_error(whole[name][1], points) < 0.005


def test_with_hulls_the_boxes_leave_the_floor_to_them_and_the_mass_is_unchanged(spec, model):
    hulled = terrain.build(spec, {}, leg_boxes=True, leg_hulls=32)
    assert hulled.ngeom == model.ngeom + len(collision.leg_bodies(spec)) + 1
    assert np.allclose(hulled.body_mass, model.body_mass) and np.allclose(hulled.body_inertia, model.body_inertia)
    floor = mujoco.mj_name2id(hulled, mujoco.mjtObj.mjOBJ_GEOM, "floor")

    def meet(a, b):
        return bool(hulled.geom_contype[a] & hulled.geom_conaffinity[b]
                    or hulled.geom_contype[b] & hulled.geom_conaffinity[a])

    names = [hulled.geom(g).name or "" for g in range(hulled.ngeom)]
    boxes = [g for g, n in enumerate(names) if n.endswith("_link_box")]
    hulls = [g for g, n in enumerate(names) if n.endswith("_floor_hull")]
    tyres = [g for g, n in enumerate(names) if n.endswith("_tire_cyl")]
    torso = [g for g in range(hulled.ngeom) if hulled.geom_bodyid[g] == hulled.body("torso").id
             and hulled.geom_contype[g] and g not in hulls]
    assert all(meet(g, floor) for g in hulls + tyres + torso)
    assert not any(meet(g, floor) for g in boxes)
    assert all(meet(a, b) for a in boxes for b in tyres + torso)        # the boxes still stop the rest
    assert meet(boxes[0], boxes[-1])
    assert not any(meet(h, g) for h in hulls for g in boxes + tyres + torso + hulls)


def test_hulls_need_the_boxes_and_a_flat_floor(spec):
    with pytest.raises(ValueError):
        terrain.build(spec, {}, leg_boxes=False, leg_hulls=32)
    with pytest.raises(ValueError):
        terrain.build(spec, {"type": "obstacles"}, leg_boxes=True, leg_hulls=32)


def test_lying_on_the_hulls_no_box_is_on_the_floor(spec):
    m = terrain.build(spec, {}, leg_boxes=True, leg_hulls=32)
    d = mujoco.MjData(m)
    d.qpos[:] = fallen.tipped_qpos(spec, m, -1.3)
    targets = np.array([-0.2, 0.2, 0.2, -0.2])
    d.qpos[np.asarray(spec.joint_qpos)[list(spec.leg_index)]] = targets
    for _ in range(int(2.0 / m.opt.timestep)):
        fallen.hold(spec, d, targets)
        mujoco.mj_step(m, d)
    on_floor = set()
    for c in d.contact[: d.ncon]:
        if c.dist < 0.0 and 0 in (m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]):
            g = c.geom2 if m.geom_bodyid[c.geom1] == 0 else c.geom1
            on_floor.add(m.geom(int(g)).name or "torso collider")
    assert on_floor and not any(name.endswith("_link_box") for name in on_floor)
    assert any(name.endswith("_floor_hull") for name in on_floor)
