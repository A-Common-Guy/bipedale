"""Collision boxes for the leg links, fitted to the CAD collision meshes.

The description collides only the torso shell and the two tyre cylinders: the
leg links and the motor holders pass through the floor, the body and each
other, and the leg joints have no limits. Unpowered, the legs then fold
through themselves, a pose the robot cannot reach. The real legs have no hard
stop: the links come to rest against each other and the body.

This adds one box per leg body (the oriented bounding box of the body's
collision-class meshes, whose convex hulls are too large for the GPU contact
pipeline) to sandro_rl's compiled model only. Pairs of bodies that touch in
normal operation share a pin and are excluded; every other pair collides.
"""

from __future__ import annotations

import itertools

import mujoco
import numpy as np

ROBOT_CONTYPE = 2
ROBOT_CONAFFINITY = 3   # the floor and obstacles (bit 1) and the other robot geoms (bit 2)


def _quat_to_mat(q):
    m = np.zeros(9)
    mujoco.mju_quat2Mat(m, np.asarray(q, dtype=float))
    return m.reshape(3, 3)


def _mat_to_quat(r):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(r, dtype=float).reshape(-1))
    return q


def leg_bodies(spec) -> list[str]:
    """The bodies under the torso that carry collision meshes, minus the wheels."""
    m = spec.model
    wheels = {int(m.jnt_bodyid[j]) for j in spec.wheel_joint_ids}
    names = []
    for b in range(m.nbody):
        if b in wheels or b == spec.torso_body or m.body_rootid[b] != m.body_rootid[spec.torso_body]:
            continue
        if any(m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH for g in range(m.ngeom) if m.geom_bodyid[g] == b):
            names.append(m.body(b).name)
    return names


def fit_boxes(spec) -> dict[str, tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """Per body: box centre, rotation and half sizes in the body frame (PCA of the mesh vertices)."""
    m = spec.model
    boxes = {}
    for name in leg_bodies(spec):
        b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
        points = []
        for g in range(m.ngeom):
            if m.geom_bodyid[g] != b or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            mesh = m.geom_dataid[g]
            vert = m.mesh_vert[m.mesh_vertadr[mesh]: m.mesh_vertadr[mesh] + m.mesh_vertnum[mesh]]
            points.append(vert @ _quat_to_mat(m.geom_quat[g]).T + m.geom_pos[g])
        points = np.unique(np.round(np.concatenate(points), 6), axis=0)
        centre = points.mean(axis=0)
        _, _, axes = np.linalg.svd(points - centre, full_matrices=False)
        rot = axes.T
        if np.linalg.det(rot) < 0:
            rot[:, 2] = -rot[:, 2]
        local = (points - centre) @ rot
        lo, hi = local.min(axis=0), local.max(axis=0)
        boxes[name] = (centre + rot @ (0.5 * (lo + hi)), rot, np.maximum(0.5 * (hi - lo), 1e-3))
    return boxes


def add_link_boxes(mj_spec: mujoco.MjSpec, boxes, exclude=()) -> None:
    """Add the fitted boxes as massless collision geoms and the excluded body pairs."""
    for name, (pos, rot, half) in boxes.items():
        body = mj_spec.body(name)
        body.add_geom(name=f"{name}_link_box", type=mujoco.mjtGeom.mjGEOM_BOX, size=half, pos=pos,
                      quat=_mat_to_quat(rot), contype=ROBOT_CONTYPE, conaffinity=ROBOT_CONAFFINITY,
                      mass=0.0, group=3, condim=3, friction=[0.6, 0.005, 0.0001])
    for a, b in exclude:
        mj_spec.add_exclude(bodyname1=a, bodyname2=b)


def operating_contacts(spec, boxes, seconds: float = 6.0, seed: int = 0) -> set[tuple[str, str]]:
    """Body pairs whose boxes touch while the legs sweep their operating range.

    The torso is held in the air and the four leg motors follow random PD
    targets within +-0.6 rad of the stand pose: these contacts are between
    links that share a pin, not stops.
    """
    mj_spec = mujoco.MjSpec.from_file(str(spec.scene))
    add_link_boxes(mj_spec, boxes)
    m = mj_spec.compile()
    d = mujoco.MjData(m)
    d.qpos[:] = spec.stand_qpos
    d.qpos[2] += 0.3
    rng = np.random.default_rng(seed)
    legs = list(spec.leg_index)
    qadr, dadr = np.asarray(spec.joint_qpos)[legs], np.asarray(spec.joint_dof)[legs]
    acts = np.asarray(spec.joint_actuator)[legs]
    stand = np.asarray(spec.stand_q)[legs]
    box_ids = {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"{n}_link_box") for n in boxes}
    pairs = set()
    target = stand.copy()
    base = d.qpos[:7].copy()
    for step in range(int(seconds / m.opt.timestep)):
        if step % int(0.5 / m.opt.timestep) == 0:
            target = stand + rng.uniform(-0.6, 0.6, size=4)
        d.ctrl[:] = 0.0
        d.ctrl[acts] = np.clip(20.0 * (target - d.qpos[qadr]) - 0.6 * d.qvel[dadr], -10.0, 10.0)
        mujoco.mj_step(m, d)
        d.qpos[:7] = base
        d.qvel[:6] = 0.0
        for c in d.contact[: d.ncon]:
            if c.geom1 in box_ids or c.geom2 in box_ids:
                a, b = m.body(m.geom_bodyid[c.geom1]).name, m.body(m.geom_bodyid[c.geom2]).name
                if a != "world" and b != "world":
                    pairs.add(tuple(sorted((a, b))))
    return pairs


def build_spec(spec, cfg: dict | None = None) -> mujoco.MjSpec:
    """The scene with the leg collision boxes, as an MjSpec ready for terrain bodies."""
    boxes = fit_boxes(spec)
    exclude = sorted(operating_contacts(spec, boxes))
    mj_spec = mujoco.MjSpec.from_file(str(spec.scene))
    add_link_boxes(mj_spec, boxes, exclude)
    return mj_spec
