"""Symmetric stand pose + shared leg kinematics for the composed wheeled biped.

Both legs are planar four-bar linkages and every hinge on both sides shares
one axis direction (checked at build time), so the wheels are always exactly
parallel. But the two legs come from separate Onshape exports whose joint
zeros are unrelated: the left export's "inner" motor is even mounted where
the right's "outer" sits. Copying joint angles across sides therefore never
produces a symmetric pose (the closure residual can't tell -- the four-bar
closes for ANY motor pair), which is what used to tilt the spawned robot.

The fix implemented here works in physical space instead of joint space:
the left leg's motors are Newton-solved so its wheel center matches the
right wheel center in the linkage plane (in-plane = perpendicular to the
common hinge axis). Branch safety comes from continuation: each leg starts
from its own CAD-recovered assembly angles and targets move in small steps.

Run standalone to rebuild the floating model from an existing robot_full.xml
(no Onshape API access needed):

    python stand_pose.py [--stand-extension 0.7]
"""
from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).parent

LEG_JOINTS_ALL = ("motor_inner", "motor_outer",
                  "passive_biella_inner", "passive_biella_outer")
PASSIVE_JOINTS = ("passive_biella_inner", "passive_biella_outer")


def mat2quat(R):
    q = np.empty(4)
    mujoco.mju_mat2Quat(q, np.asarray(R).ravel())
    return q


def jid(model, name):
    j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if j < 0:
        raise KeyError(f"no joint '{name}'")
    return j


def detect_sides(model):
    """Leg name prefixes, reference side ('right' if present) first."""
    sides = []
    for j in range(model.njnt):
        n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        if n and n.endswith("_motor_inner"):
            sides.append(n[: -len("_motor_inner")])
    sides.sort(key=lambda s: (s != "right", s))
    return sides


def closure_pairs(model, side):
    """The connect-equality site id pairs belonging to one leg."""
    pairs = []
    for e in range(model.neq):
        if model.eq_type[e] != mujoco.mjtEq.mjEQ_CONNECT:
            continue
        s1, s2 = model.eq_obj1id[e], model.eq_obj2id[e]
        n1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, s1)
        if n1.startswith(f"{side}_"):
            pairs.append((s1, s2))
    return pairs


def leg_solve(model, data, side, unknowns=PASSIVE_JOINTS, target=None,
              n_axis=None, max_iter=60, tol=1e-10):
    """Newton-solve `unknowns` joints of one leg.

    Always enforces the four-bar loop closure; with `target` also places the
    wheel-joint anchor at `target` in the linkage plane (components along the
    hinge axis `n_axis` are projected out -- the planar mechanism cannot move
    that way). Returns the final residual norm.
    """
    pairs = closure_pairs(model, side)
    jids = [jid(model, f"{side}_{u}") for u in unknowns]
    dofs = np.array([model.jnt_dofadr[j] for j in jids])
    adrs = np.array([model.jnt_qposadr[j] for j in jids])
    wj = jid(model, f"{side}_wheel")
    wbody = int(model.jnt_bodyid[wj])
    P = None if target is None else np.eye(3) - np.outer(n_axis, n_axis)
    r = np.array([np.inf])
    for _ in range(max_iter):
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        res = [data.site_xpos[a] - data.site_xpos[b] for a, b in pairs]
        if target is not None:
            res.append(P @ (data.xanchor[wj] - target))
        r = np.concatenate(res)
        if np.linalg.norm(r) < tol:
            break
        rows = []
        for a, b in pairs:
            Ja, Jb = np.zeros((3, model.nv)), np.zeros((3, model.nv))
            mujoco.mj_jacSite(model, data, Ja, None, a)
            mujoco.mj_jacSite(model, data, Jb, None, b)
            rows.append((Ja - Jb)[:, dofs])
        if target is not None:
            Jp = np.zeros((3, model.nv))
            mujoco.mj_jac(model, data, Jp, None, data.xanchor[wj], wbody)
            rows.append((P @ Jp)[:, dofs])
        dq, *_ = np.linalg.lstsq(np.vstack(rows), -r, rcond=None)
        step = np.linalg.norm(dq)
        if step > 0.3:  # trust region so continuation never jumps branch
            dq *= 0.3 / step
        data.qpos[adrs] += dq
    return float(np.linalg.norm(r))


def replace_tire_collision(spec: mujoco.MjSpec):
    """Swap the collidable tire meshes for analytic cylinders.

    A mesh tire collides through its convex hull, whose circumference is a
    polygon: the contact point hops facet to facet while rolling, which
    makes the wheel speed oscillate at the facet-crossing frequency (~250 Hz
    at 1 m/s -- right at the control rate, so sampled wheel velocities alias
    badly). A cylinder of the same radius/width rolls smoothly."""
    model = spec.compile()
    jobs = {}
    for g in range(model.ngeom):
        if (model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH
                or not model.geom_contype[g]):
            continue
        meshname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH,
                                     model.geom_dataid[g])
        if not meshname.endswith("tire"):
            continue
        b = int(model.geom_bodyid[g])
        wj = [j for j in range(model.njnt) if model.jnt_bodyid[j] == b]
        if len(wj) != 1:
            raise SystemExit(f"tire body has {len(wj)} joints, expected the wheel")
        n = model.jnt_axis[wj[0]] / np.linalg.norm(model.jnt_axis[wj[0]])
        anchor = model.jnt_pos[wj[0]]
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, model.geom_quat[g])
        verts = (R.reshape(3, 3) @ model.mesh_vert[
            model.mesh_vertadr[model.geom_dataid[g]]:
            model.mesh_vertadr[model.geom_dataid[g]] + model.mesh_vertnum[model.geom_dataid[g]]].T).T \
            + model.geom_pos[g]
        rel = verts - anchor
        ax = rel @ n
        rad = np.linalg.norm(rel - np.outer(ax, n), axis=1)
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)
        jobs[body_name] = (meshname, n, anchor,
                           float(rad.max()),
                           float((ax.min() + ax.max()) / 2),
                           float((ax.max() - ax.min()) / 2))
    n_replaced = 0
    for body in spec.bodies:
        if body.name not in jobs:
            continue
        meshname, n, anchor, radius, mid, half = jobs[body.name]
        for g in body.geoms:
            if g.meshname == meshname and g.contype:
                g.contype = 0
                g.conaffinity = 0
        quat = np.empty(4)
        mujoco.mju_quatZ2Vec(quat, n)
        body.add_geom(name=f"{body.name}_tire_cyl",
                      type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                      size=[radius, half, 0],
                      pos=np.asarray(anchor) + mid * n, quat=quat,
                      group=3, rgba=[0.8, 0.2, 0.2, 0.4])
        n_replaced += 1
    if n_replaced:
        print(f"replaced {n_replaced} mesh tire colliders with cylinders")
    return n_replaced


def collidable_min_z(model, data):
    """Lowest world-z point of the collidable geometry (mesh + primitives)."""
    minz = np.inf
    for g in range(model.ngeom):
        if not model.geom_contype[g]:
            continue
        t = model.geom_type[g]
        if t == mujoco.mjtGeom.mjGEOM_MESH:
            mesh = model.geom_dataid[g]
            va, vn = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
            verts = model.mesh_vert[va:va + vn]
            zs = (data.geom_xmat[g].reshape(3, 3) @ verts.T)[2] + data.geom_xpos[g][2]
            minz = min(minz, float(zs.min()))
        elif t in (mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_CAPSULE):
            az = data.geom_xmat[g].reshape(3, 3)[2, 2]
            r, h = model.geom_size[g, 0], model.geom_size[g, 1]
            drop = h * abs(az) + (r * np.sqrt(max(0.0, 1 - az * az))
                                  if t == mujoco.mjtGeom.mjGEOM_CYLINDER else r)
            minz = min(minz, float(data.geom_xpos[g][2] - drop))
        elif t == mujoco.mjtGeom.mjGEOM_SPHERE:
            minz = min(minz, float(data.geom_xpos[g][2] - model.geom_size[g, 0]))
    return minz


def hinge_axis(model, data, sides):
    """Common hinge-axis direction (unit) -- verified identical for all leg
    joints of both sides, which is what makes the wheels always parallel."""
    mujoco.mj_kinematics(model, data)
    n = data.xaxis[jid(model, f"{sides[0]}_wheel")].copy()
    n /= np.linalg.norm(n)
    for s in sides:
        for j in LEG_JOINTS_ALL + ("wheel",):
            a = data.xaxis[jid(model, f"{s}_{j}")]
            if np.linalg.norm(np.cross(a, n)) > 1e-6:
                raise SystemExit(f"joint {s}_{j} axis not parallel to the common "
                                 f"hinge axis -- symmetric-stand assumptions broken")
    return n


def symmetric_stand(model, data, sides, q_by_joint):
    """Pose with the second leg's wheel center matching the first's in-plane.

    q_by_joint: branch-correct start angles per joint name (the CAD assembly
    pose recovered per side -- each leg starts on its own branch).
    """
    data.qpos[:] = 0
    data.qpos[3] = 1.0
    for name, val in q_by_joint.items():
        data.qpos[model.jnt_qposadr[jid(model, name)]] = val
    for s in sides:
        res = leg_solve(model, data, s)
        if res > 1e-8:
            raise SystemExit(f"{s} leg loop does not close at its CAD angles "
                             f"(residual {res:.2e} m) -- stale exports?")
    n = hinge_axis(model, data, sides)
    ref, other = sides
    wj = {s: jid(model, f"{s}_wheel") for s in sides}
    c_ref = data.xanchor[wj[ref]].copy()
    c0 = data.xanchor[wj[other]].copy()
    # continuation: walk the other wheel center onto the reference one
    # (in-plane; the along-axis offset is the track width and cannot move)
    res = 0.0
    for t in np.linspace(0.0, 1.0, 26)[1:]:
        res = leg_solve(model, data, other, LEG_JOINTS_ALL,
                        target=(1 - t) * c0 + t * c_ref, n_axis=n)
    if res > 1e-8:
        raise SystemExit(f"left/right symmetric IK did not converge (residual {res:.2e})")
    return n


def extend_stand(model, data, sides, n, fraction, step=0.02, max_steps=500):
    """Walk the reference leg's motors toward maximum leg extension, keeping
    the other leg's wheel center matched in-plane at every step, then back
    off to `fraction` of the reachable extension. Acts on data.qpos."""
    ref, other = sides
    wj = {s: jid(model, f"{s}_wheel") for s in sides}
    adr_i = model.jnt_qposadr[jid(model, f"{ref}_motor_inner")]
    adr_o = model.jnt_qposadr[jid(model, f"{ref}_motor_outer")]
    P = np.eye(3) - np.outer(n, n)
    mujoco.mj_kinematics(model, data)
    pivot = np.mean([data.xanchor[jid(model, f"{ref}_{m}")]
                     for m in ("motor_inner", "motor_outer")], axis=0)

    def extension():
        return float(np.linalg.norm(P @ (data.xanchor[wj[ref]] - pivot)))

    def try_step(q_base, di, do):
        data.qpos[:] = q_base
        data.qpos[adr_i] += di
        data.qpos[adr_o] += do
        if leg_solve(model, data, ref) > 1e-8:
            return None
        c_ref = data.xanchor[wj[ref]].copy()
        if leg_solve(model, data, other, LEG_JOINTS_ALL,
                     target=c_ref, n_axis=n) > 1e-8:
            return None
        return extension(), data.qpos.copy()

    path = [(extension(), data.qpos.copy())]
    dirs = [(step, 0), (-step, 0), (0, step), (0, -step),
            (step, step), (-step, -step), (step, -step), (-step, step)]
    for _ in range(max_steps):
        best = None
        for di, do in dirs:
            cand = try_step(path[-1][1], di, do)
            if cand and cand[0] > path[-1][0] + 1e-6 and (best is None or cand[0] > best[0]):
                best = cand
        if best is None:
            break
        path.append(best)
    ext0, ext_max = path[0][0], path[-1][0]
    target = ext0 + fraction * (ext_max - ext0)
    ext, q = next((e, q) for e, q in path if e >= target)
    data.qpos[:] = q
    mujoco.mj_kinematics(model, data)
    print(f"stand extension: {ext0*1000:.0f} -> {ext*1000:.0f} mm of reachable "
          f"{ext_max*1000:.0f} mm (fraction {fraction})")
    return ext


def orient_and_ground(model, data, sides):
    """Rotate the free base so the wheel axle is world-y and the COM is above
    it, then set the base height so the tires just touch the floor at z=0.
    Acts on data.qpos (base must be the first, free joint)."""
    q = data.qpos.copy()
    data.qpos[:] = q
    mujoco.mj_forward(model, data)
    wj = [jid(model, f"{s}_wheel") for s in sides]
    centers = [data.xanchor[j].copy() for j in wj]
    axis = centers[-1] - centers[0]
    axis /= np.linalg.norm(axis)
    mid = np.mean(centers, axis=0)
    tid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "torso")
    up = data.subtree_com[tid] - mid
    up -= axis * (up @ axis)
    up /= np.linalg.norm(up)
    A = np.column_stack([np.cross(axis, up), axis, up])
    q[3:7] = mat2quat(A.T)  # maps wheel axle -> y, com-up -> z
    data.qpos[:] = q
    mujoco.mj_forward(model, data)
    # exact lowest point of the collidable geometry (tires)
    minz = collidable_min_z(model, data)
    mid = np.mean([data.xanchor[j] for j in wj], axis=0)
    q[0:2] = -mid[0:2]
    q[2] += 0.001 - minz
    data.qpos[:] = q
    mujoco.mj_forward(model, data)


def hold_ctrl(model, qpos):
    """ctrl that holds qpos: leg PDs track their joint, wheels command 0 speed."""
    ctrl = np.zeros(model.nu)
    for a in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, a)
        if not name.endswith("_wheel"):
            j = jid(model, name)
            ctrl[a] = qpos[model.jnt_qposadr[j]]
    return ctrl


def report_symmetry(model, data, sides):
    """Print the physical symmetry / alignment numbers for the current pose."""
    mujoco.mj_kinematics(model, data)
    n = data.xaxis[jid(model, f"{sides[0]}_wheel")].copy()
    n /= np.linalg.norm(n)
    P = np.eye(3) - np.outer(n, n)
    wj = [jid(model, f"{s}_wheel") for s in sides]
    centers = [data.xanchor[j] for j in wj]
    inplane = np.linalg.norm(P @ (centers[1] - centers[0]))
    line = centers[1] - centers[0]
    line /= np.linalg.norm(line)
    mis = np.degrees(np.arccos(min(1.0, abs(line @ n))))
    motors = {s: [float(data.qpos[model.jnt_qposadr[jid(model, f'{s}_{m}')]])
                  for m in ("motor_inner", "motor_outer")] for s in sides}
    print(f"symmetry: wheel centers in-plane offset {inplane*1000:.3f} mm, "
          f"axle-vs-center-line misalignment {mis:.3f} deg")
    for s in sides:
        print(f"  {s}: motor_inner={motors[s][0]:+.4f}  motor_outer={motors[s][1]:+.4f}")
    return inplane, mis


def build_floating(robot_xml: Path, fraction: float, out: Path | None = None):
    """Create robot_full_floating.xml with a symmetric 'stand' keyframe from an
    existing composed robot_full.xml (uses its 'assembly' keyframe as the
    branch-correct start; no Onshape API access needed)."""
    robot_xml = Path(robot_xml)
    out = out or robot_xml.with_name("robot_full_floating.xml")

    # branch-correct start: the CAD assembly angles, per joint name
    m0 = mujoco.MjModel.from_xml_path(str(robot_xml))
    kid = mujoco.mj_name2id(m0, mujoco.mjtObj.mjOBJ_KEY, "assembly")
    if kid < 0:
        raise SystemExit(f"{robot_xml} has no 'assembly' keyframe")
    q_by_joint = {}
    for j in range(m0.njnt):
        name = mujoco.mj_id2name(m0, mujoco.mjtObj.mjOBJ_JOINT, j)
        q_by_joint[name] = float(m0.key_qpos[kid, m0.jnt_qposadr[j]])

    spec = mujoco.MjSpec.from_file(str(robot_xml))
    for k in list(spec.keys):
        spec.delete(k)
    torso = spec.bodies[1]
    if torso.name != "torso":
        raise SystemExit(f"first body is '{torso.name}', expected 'torso'")
    torso.add_freejoint()
    model = spec.compile()
    data = mujoco.MjData(model)
    sides = detect_sides(model)
    if len(sides) != 2:
        raise SystemExit(f"expected 2 legs, found {sides}")

    n = symmetric_stand(model, data, sides, q_by_joint)
    if fraction > 0:
        extend_stand(model, data, sides, n, fraction)
    orient_and_ground(model, data, sides)
    inplane, mis = report_symmetry(model, data, sides)
    if inplane > 1e-3 or mis > 0.1:
        raise SystemExit("stand pose is not symmetric -- refusing to write it")

    q = data.qpos.copy()
    key = spec.add_key(name="stand")
    key.qpos = q
    key.ctrl = hold_ctrl(model, q)
    spec.compile()
    out.write_text(spec.to_xml())
    print(f"wrote {out}")

    scene_f = robot_xml.parent / "scene_floating.xml"
    if not scene_f.exists():
        scene_f.write_text(f"""<mujoco model="scene">
    <include file="{out.name}" />
    <statistic center="0 0 0.3" extent="1.2" />
    <visual>
        <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0" />
        <global azimuth="160" elevation="-20" />
    </visual>
    <asset>
        <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0"
            width="512" height="3072" />
        <texture type="2d" name="groundplane" builtin="checker" mark="edge"
            rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8"
            width="300" height="300" />
        <material name="groundplane" texture="groundplane" texuniform="true"
            texrepeat="5 5" reflectance="0.2" />
    </asset>
    <worldbody>
        <light pos="0 0 3.5" dir="0 0 -1" directional="true" />
        <geom name="floor" size="0 0 0.05" pos="0 0 0" type="plane" material="groundplane" />
    </worldbody>
</mujoco>
""")
        print(f"wrote {scene_f}")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--robot-xml", default=HERE / "robot" / "robot_full.xml", type=Path)
    ap.add_argument("--stand-extension", type=float, default=0.7,
                    help="stand ride height as a fraction of max leg extension "
                         "(0 keeps the CAD pose, symmetrized)")
    args = ap.parse_args()
    build_floating(args.robot_xml, args.stand_extension)


if __name__ == "__main__":
    main()
