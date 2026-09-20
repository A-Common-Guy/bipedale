"""Compose an articulated robot from onshape-to-robot exports.

onshape-to-robot only reads dof_ mates from the top-level assembly, so the
full-robot export comes out as one rigid body. This script substitutes the
rigid sub-assembly instances with their articulated exports:

  1. (optional --export) re-run onshape-to-robot for every export dir
  2. query the Onshape API for the assembly definitions: every part
     occurrence carries its path (= which sub-assembly instance it belongs
     to) and its world transform, giving an exact per-instance, per-link
     pose -- including the joint angles the assembly is posed at
  3. rebuild: torso body = robot geoms that are not sub-assembly parts, with
     inertia = total robot inertia minus the sub-assemblies (parallel-axis
     subtraction at the actual link poses)
  4. mjSpec.attach() each sub-assembly model at its base-link frame (joints,
     actuators, sites, equalities are prefixed automatically with the
     Onshape instance name) and store the recovered assembly joint angles
     as a keyframe "assembly"
  5. self-check: recompile at that keyframe and verify every geom lands
     where the rigid export had it

Usage:
    python compose_robot.py [--export] [--freejoint] [--sub leg --sub leg_left]
"""
from __future__ import annotations

import argparse
import re
import shutil
import subprocess
from dataclasses import dataclass, field
from pathlib import Path

import mujoco
import numpy as np
from dotenv import load_dotenv, find_dotenv

load_dotenv(find_dotenv(usecwd=True))

from onshape_to_robot.config import Config
from onshape_to_robot.onshape_api.client import Client

import stand_pose

HERE = Path(__file__).parent
POS_TOL = 1e-3  # m, matching exported geoms <-> API occurrences

# Physical BNO085 mount derived from the roof geometry by
# SandroControl/tools/imu_frame.py. Keeping these as the compose defaults
# prevents a CAD re-export from silently moving the simulated IMU back to the
# CAD torso origin. Re-run that tool and update these values if the board moves.
IMU_MOUNT_POS = (-0.052683638, 0.1446357, -0.28703048)
IMU_MOUNT_QUAT = (0.548848255, -0.548848255, -0.445831351, 0.445831351)

# ---------------------------------------------------------------------------
# Hardware constants -- verify against the datasheets before trusting results.
# GIM6010-8 (SteadyWin, 8:1 planetary, 24 V / 252 W):
GIM6010_8 = dict(
    mass=0.388,             # kg (motor+gearbox; add driver PCB mass if stacked)
    gear=8.0,               # planetary reduction
    rotor_inertia=2.63e-6,  # kg m^2 (26.3 g cm^2, motor side)
    peak_torque=11.0,       # N m at the output (stall, short bursts only)
    rated_torque=5.0,       # N m at the output (continuous)
    max_speed=44.0,         # rad/s at the output (~420 rpm)
    radius=0.030,           # m, for the added-mass cylinder approximation
    height=0.030,           # m
)
# reflected rotor inertia seen by the joint: J_rotor * gear^2
GIM6010_8["armature"] = GIM6010_8["rotor_inertia"] * GIM6010_8["gear"] ** 2

# joints driven by a GIM6010-8 (names after the _r/_l suffix strip)
MOTOR_JOINTS = ("motor_inner", "motor_outer", "wheel")
# mesh name of the motor part in the exports
MOTOR_MESH_PREFIX = "motor_2"

# BNO085 IMU: 1-sigma noise, first-pass values -- refine from the datasheet.
# Only active when <flag sensornoise="enable"/> is set in the scene.
BNO085 = dict(
    quat_noise=0.017,   # rad (~1 deg dynamic orientation error)
    gyro_noise=0.002,   # rad/s
    accel_noise=0.02,   # m/s^2
)
# ---------------------------------------------------------------------------


def pose44(pos, quat=None, R=None):
    T = np.eye(4)
    if R is None:
        R = np.empty(9)
        mujoco.mju_quat2Mat(R, np.asarray(quat, dtype=float))
        R = R.reshape(3, 3)
    T[:3, :3], T[:3, 3] = R, pos
    return T


def mat2quat(R):
    q = np.empty(4)
    mujoco.mju_mat2Quat(q, np.asarray(R).ravel())
    return q


def authored_geoms(spec: mujoco.MjSpec, model: mujoco.MjModel):
    """(meshname, authored world pos, authored world R, body id) per mesh geom.

    Uses the spec (XML) poses, not the compiled ones: the compiler re-centers
    meshes, so compiled geom poses differ from CAD coordinates.
    """
    body_pose = {}
    out = []
    for body in spec.bodies:
        if body.name == "world":
            body_pose[body.name] = np.eye(4)
        else:
            parent = spec.body(body.name).parent
            body_pose[body.name] = body_pose[parent.name] @ pose44(body.pos, body.quat)
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                                body.name if body.name != "world" else "world")
        seen = set()
        for g in body.geoms:
            if g.type != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            T = body_pose[body.name] @ pose44(g.pos, g.quat)
            key = (g.meshname, tuple(np.round(T[:3, 3], 5)), tuple(np.round(T[:3, :3].ravel(), 4)))
            if key in seen:
                continue
            seen.add(key)
            out.append((g.meshname, T[:3, 3], T[:3, :3], bid))
    return out


def compiled_geoms(model: mujoco.MjModel, data):
    """(meshname, compiled world pos) per mesh geom -- for the self-check only."""
    out = []
    for g in range(model.ngeom):
        if model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mesh = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, model.geom_dataid[g])
        out.append((mesh, data.geom_xpos[g].copy()))
    return out


def assembly_occurrences(client: Client, cfg: Config):
    """{path tuple: world transform 4x4} for an assembly, plus its raw instances."""
    wid = cfg.version_id or cfg.workspace_id
    if not wid:
        wid = client.get_document(cfg.document_id)["defaultWorkspace"]["id"]
    data = client.get_assembly(
        cfg.document_id, wid, cfg.element_id,
        wmv="v" if cfg.version_id else "w",
        configuration=cfg.configuration,
    )
    occ = {tuple(o["path"]): np.array(o["transform"]).reshape(4, 4)
           for o in data["rootAssembly"]["occurrences"]}
    return occ, data["rootAssembly"]["instances"]


def side_prefix(instance_name: str) -> str:
    name = re.sub(r"<[^>]*>", "", instance_name).strip().lower()
    for side in ("right", "left"):
        if side in name:
            return side
    return re.sub(r"[^a-z0-9]+", "_", name).strip("_") or "sub"


def inertia_about_origin(mass, com, I_com):
    c = np.asarray(com)
    return I_com + mass * (np.dot(c, c) * np.eye(3) - np.outer(c, c))


def strip_side_suffix(spec: mujoco.MjSpec):
    """Drop trailing _r/_l on joint and actuator names (mates were named per side)."""
    ren = {}
    for j in spec.joints:
        new = re.sub(r"_[rl]$", "", j.name)
        if new != j.name:
            ren[j.name] = new
            j.name = new
    for a in spec.actuators:
        if a.target in ren:
            a.target = ren[a.target]
        a.name = re.sub(r"_[rl]$", "", a.name)


def add_motor_masses(spec: mujoco.MjSpec, motor_mass: float):
    """Fold the motor mass into each body's explicit inertial.

    The CAD motors are massless shells, so the exported inertials miss them.
    Each motor geom contributes a solid cylinder (axis = mesh frame z) of
    motor_mass at the geom pose; the body inertial is recomposed rigidly.
    Set --motor-mass 0 once the CAD carries real motor masses.
    """
    r, h = GIM6010_8["radius"], GIM6010_8["height"]
    I_local = np.diag([motor_mass * (3 * r * r + h * h) / 12,
                       motor_mass * (3 * r * r + h * h) / 12,
                       motor_mass * r * r / 2])
    n_added = 0
    for body in spec.bodies:
        if body.name == "world":
            continue
        motors, seen = [], set()
        for g in body.geoms:
            if g.meshname.startswith(MOTOR_MESH_PREFIX):
                key = tuple(np.round(np.asarray(g.pos), 5))
                if key not in seen:
                    seen.add(key)
                    motors.append(pose44(g.pos, g.quat))
        if not motors:
            continue
        # existing inertial (body frame, about its com)
        m0 = body.mass
        c0 = np.asarray(body.ipos)
        fi = np.asarray(body.fullinertia)
        I0 = np.array([[fi[0], fi[3], fi[4]],
                       [fi[3], fi[1], fi[5]],
                       [fi[4], fi[5], fi[2]]])
        parts = [(m0, c0, I0)]
        for T in motors:
            parts.append((motor_mass, T[:3, 3], T[:3, :3] @ I_local @ T[:3, :3].T))
            n_added += 1
        m = sum(p[0] for p in parts)
        c = sum(p[0] * p[1] for p in parts) / m
        I = sum(p[2] + p[0] * (np.dot(p[1] - c, p[1] - c) * np.eye(3)
                               - np.outer(p[1] - c, p[1] - c)) for p in parts)
        body.explicitinertial = True
        body.mass = m
        body.ipos = c
        body.iquat = [1, 0, 0, 0]
        body.fullinertia = [I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2]]
    return n_added


def tune_joints(spec: mujoco.MjSpec):
    """Per-joint passive dynamics, overriding the exporter's one-size default."""
    for j in spec.joints:
        if re.sub(r"_[rl]$", "", j.name) in MOTOR_JOINTS:
            j.armature = GIM6010_8["armature"]  # reflected rotor inertia
            j.frictionloss = 0.1                # gearbox dry friction (guess, tune)
            j.damping = [0.01, 0, 0]            # viscous part (guess, tune)
        else:  # passive bearing pivots: nearly free
            j.armature = 1e-5
            j.frictionloss = 0.005
            j.damping = [0.001, 0, 0]


def add_actuators(spec: mujoco.MjSpec, sides, kp: float, kv: float, wheel_kv: float):
    """Mirror the real drive modes: MIT-mode PD on the leg motors (ctrl = target
    angle, rad), velocity mode on the wheels (ctrl = target speed, rad/s)."""
    tmax = GIM6010_8["peak_torque"]
    wmax = GIM6010_8["max_speed"]
    for side in sides:
        for jn in MOTOR_JOINTS:
            name = f"{side}_{jn}"
            if jn == "wheel":
                # velocity loop: tau = wheel_kv * (ctrl - qvel), saturated
                spec.add_actuator(
                    name=name, target=name, trntype=mujoco.mjtTrn.mjTRN_JOINT,
                    gaintype=mujoco.mjtGain.mjGAIN_FIXED, gainprm=[wheel_kv] + [0] * 9,
                    biastype=mujoco.mjtBias.mjBIAS_AFFINE,
                    biasprm=[0, 0, -wheel_kv] + [0] * 7,
                    ctrlrange=[-wmax, wmax], forcerange=[-tmax, tmax])
            else:
                # onboard PD (MIT mode): tau = kp*(ctrl - q) - kv*qvel, saturated
                spec.add_actuator(
                    name=name, target=name, trntype=mujoco.mjtTrn.mjTRN_JOINT,
                    gaintype=mujoco.mjtGain.mjGAIN_FIXED, gainprm=[kp] + [0] * 9,
                    biastype=mujoco.mjtBias.mjBIAS_AFFINE,
                    biasprm=[0, -kp, -kv] + [0] * 7,
                    forcerange=[-tmax, tmax])


def add_imu(spec: mujoco.MjSpec, torso, pos, quat):
    torso.add_site(name="imu", pos=pos, quat=quat, size=[0.008, 0.008, 0.008],
                   group=3, rgba=[1, 0, 0, 1])
    S, O = mujoco.mjtSensor, mujoco.mjtObj
    for name, stype, noise in (
            ("imu_quat", S.mjSENS_FRAMEQUAT, BNO085["quat_noise"]),
            ("imu_gyro", S.mjSENS_GYRO, BNO085["gyro_noise"]),
            ("imu_accel", S.mjSENS_ACCELEROMETER, BNO085["accel_noise"])):
        spec.add_sensor(name=name, type=stype, objtype=O.mjOBJ_SITE,
                        objname="imu", noise=noise)


def add_encoders(spec: mujoco.MjSpec, sides):
    S, O = mujoco.mjtSensor, mujoco.mjtObj
    for side in sides:
        for jn in MOTOR_JOINTS:
            spec.add_sensor(name=f"{side}_{jn}_pos", type=S.mjSENS_JOINTPOS,
                            objtype=O.mjOBJ_JOINT, objname=f"{side}_{jn}")
            spec.add_sensor(name=f"{side}_{jn}_vel", type=S.mjSENS_JOINTVEL,
                            objtype=O.mjOBJ_JOINT, objname=f"{side}_{jn}")


@dataclass
class SubAssembly:
    dir: Path
    cfg: Config
    model: mujoco.MjModel
    data: mujoco.MjData
    xml: Path
    geoms: list          # authored world poses
    occ: dict            # leg-doc occurrence path -> transform
    occ_body: dict = field(default_factory=dict)  # occurrence path -> model body id
    pose0: dict = field(default_factory=dict)     # body id -> world pose at qpos0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot-dir", default=HERE / "robot", type=Path)
    ap.add_argument("--sub", action="append", type=Path,
                    help="sub-assembly export dir (repeatable; default: leg, leg_left)")
    ap.add_argument("--out", default=None, type=Path, help="output xml (default <robot-dir>/robot_full.xml)")
    ap.add_argument("--export", action="store_true", help="re-run onshape-to-robot for every dir first")
    ap.add_argument("--motor-mass", type=float, default=GIM6010_8["mass"],
                    help="mass added per motor geom (0 once CAD carries motor masses)")
    ap.add_argument("--kp", type=float, default=20.0, help="leg MIT-mode PD stiffness (N m/rad)")
    ap.add_argument("--kv", type=float, default=0.5, help="leg MIT-mode PD damping (N m s/rad)")
    ap.add_argument("--wheel-kv", type=float, default=1.0,
                    help="wheel velocity-loop gain (N m s/rad). Do not raise "
                         "above ~2 with the cylinder tire colliders: a stiff "
                         "velocity servo interacts badly with cylinder-plane "
                         "contact and the robot cannot balance")
    ap.add_argument("--stand-extension", type=float, default=0.7,
                    help="stand keyframe ride height as a fraction of max leg "
                         "extension (0 keeps the CAD assembly pose)")
    ap.add_argument("--imu-pos", type=float, nargs=3, default=IMU_MOUNT_POS,
                    help="BNO085 site position in the torso frame (default: "
                         "roof mount derived by SandroControl/tools/imu_frame.py)")
    ap.add_argument("--imu-quat", type=float, nargs=4, default=IMU_MOUNT_QUAT,
                    help="BNO085 site orientation (w x y z) in the torso frame "
                         "(default: physical roof mount)")
    args = ap.parse_args()
    out = args.out or args.robot_dir / "robot_full.xml"
    sub_dirs = args.sub or [d for d in (HERE / "leg", HERE / "leg_left") if d.is_dir()]

    if args.export:
        for d in (*sub_dirs, args.robot_dir):
            print(f"== onshape-to-robot {d.name}")
            subprocess.run(["onshape-to-robot", str(d)], cwd=HERE, check=True)

    robot_xml = args.robot_dir / "robot.xml"
    robot_cfg = Config(str(args.robot_dir))
    client = Client(logging=False, creds=robot_cfg.config_file)
    robot_occ, robot_instances = assembly_occurrences(client, robot_cfg)

    subs: dict[str, SubAssembly] = {}
    for d in sub_dirs:
        cfg = Config(str(d))
        xml = d / "robot.xml"
        spec = mujoco.MjSpec.from_file(str(xml))
        model = spec.compile()
        data = mujoco.MjData(model)
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        sub = SubAssembly(d, cfg, model, data, xml,
                          authored_geoms(spec, model),
                          assembly_occurrences(client, cfg)[0])
        sub.pose0 = {b: pose44(data.xpos[b], R=data.xmat[b].reshape(3, 3))
                     for b in range(1, model.nbody)}
        for path, L in sub.occ.items():
            for mesh, pos, R, b in sub.geoms:
                if (np.linalg.norm(L[:3, 3] - pos) < POS_TOL
                        and np.linalg.norm(L[:3, :3] - R) < 1e-2):
                    sub.occ_body[path] = b
                    break
        unmapped = len(sub.occ) - len(sub.occ_body)
        if unmapped:
            print(f"note: {d.name}: {unmapped}/{len(sub.occ)} occurrences carry no exported "
                  f"geometry (sub-sub-assemblies or ignored parts)")
        subs[cfg.element_id] = sub

    instances = []  # (sub, side, {body id: T}, {joint name: angle})
    for inst in robot_instances:
        if inst.get("type") != "Assembly" or inst.get("suppressed"):
            continue
        sub = subs.get(inst.get("elementId"))
        if sub is None:
            print(f"WARNING: sub-assembly instance '{inst['name']}' has no export dir; "
                  f"its parts stay rigid in the torso")
            continue
        T_by_body, spread = {}, 0.0
        for path, Rw in robot_occ.items():
            if len(path) < 2 or path[0] != inst["id"]:
                continue
            b = sub.occ_body.get(path[1:])
            if b is None:
                continue
            T = Rw @ np.linalg.inv(sub.occ[path[1:]])
            if b in T_by_body:
                spread = max(spread, np.linalg.norm(T[:3, 3] - T_by_body[b][:3, 3]))
            else:
                T_by_body[b] = T
        missing = [mujoco.mj_id2name(sub.model, mujoco.mjtObj.mjOBJ_BODY, b)
                   for b in range(1, sub.model.nbody) if b not in T_by_body]
        if missing:
            raise SystemExit(f"instance {inst['name']}: no occurrence found for link(s) "
                             f"{missing} -- stale exports? (try --export)")
        if spread > POS_TOL:
            print(f"WARNING: {inst['name']}: parts of one link disagree by up to "
                  f"{spread*1000:.2f} mm -- CAD moved between exports? (try --export)")

        # recover the assembly's joint angles (all joints are hinges about z)
        qs = {}
        for j in range(sub.model.njnt):
            b = sub.model.jnt_bodyid[j]
            p = sub.model.body_parentid[b]
            A0 = np.linalg.inv(sub.pose0[p]) @ sub.pose0[b]
            A = np.linalg.inv(T_by_body[p] @ sub.pose0[p]) @ T_by_body[b] @ sub.pose0[b]
            Rz = A0[:3, :3].T @ A[:3, :3]
            name = mujoco.mj_id2name(sub.model, mujoco.mjtObj.mjOBJ_JOINT, j)
            qs[re.sub(r"_[rl]$", "", name)] = float(np.arctan2(Rz[1, 0], Rz[0, 0]))
        instances.append((sub, side_prefix(inst["name"]), T_by_body, qs))

    if not instances:
        raise SystemExit("no sub-assembly instance matched any export dir")
    sides = [side for _, side, _, _ in instances]
    if len(set(sides)) != len(sides):
        raise SystemExit(f"instance prefixes not unique: {sides}")
    for sub, side, T_by_body, qs in instances:
        print(f"  {side} ({sub.dir.name}): base at {np.round(T_by_body[1][:3, 3], 4)}, angles "
              + ", ".join(f"{k}={v:+.3f}" for k, v in qs.items()))

    # robot geoms that belong to an attached instance (they get replaced)
    sub_part_pos = []
    for inst in robot_instances:
        sub = subs.get(inst.get("elementId"))
        if sub is None or inst.get("suppressed"):
            continue
        for path, Rw in robot_occ.items():
            if len(path) >= 2 and path[0] == inst["id"] and path[1:] in sub.occ_body:
                sub_part_pos.append(Rw[:3, 3])
    sub_meshes = {g[0] for s in subs.values() for g in s.geoms}

    # -- torso inertia = whole robot minus every attached link at its actual pose
    m_robot = mujoco.MjModel.from_xml_path(str(robot_xml))
    d_robot = mujoco.MjData(m_robot)
    mujoco.mj_kinematics(m_robot, d_robot)
    mujoco.mj_comPos(m_robot, d_robot)
    M_t = m_robot.body_mass[1]
    c = d_robot.xipos[1]
    R = d_robot.ximat[1].reshape(3, 3)
    S_t = M_t * c
    I_t = inertia_about_origin(M_t, c, R @ np.diag(m_robot.body_inertia[1]) @ R.T)
    for sub, _, T_by_body, _ in instances:
        for b in range(1, sub.model.nbody):
            T = T_by_body[b]
            mb = sub.model.body_mass[b]
            cb = T[:3, :3] @ sub.data.xipos[b] + T[:3, 3]
            Rb = T[:3, :3] @ sub.data.ximat[b].reshape(3, 3)
            M_t -= mb
            S_t -= mb * cb
            I_t -= inertia_about_origin(mb, cb, Rb @ np.diag(sub.model.body_inertia[b]) @ Rb.T)
    torso_inertial = None
    if M_t < 1e-4:
        print(f"WARNING: torso mass from CAD is ~zero ({M_t:.2e} kg) -- the torso parts "
              f"probably have no material assigned in Onshape. Falling back to "
              f"compiler-computed inertia from the torso geoms (density 1000 kg/m^3). "
              f"Assign materials in Onshape and re-run for correct dynamics.")
    else:
        c_t = S_t / M_t
        I_ct = I_t - M_t * (np.dot(c_t, c_t) * np.eye(3) - np.outer(c_t, c_t))
        eig = np.linalg.eigvalsh(I_ct)
        if eig.min() <= 0:
            raise SystemExit(f"torso inertia not positive definite ({eig}) -- exports inconsistent")
        torso_inertial = (M_t, c_t, I_ct)
        print(f"torso: mass={M_t:.4f} kg  com={np.round(c_t, 4)}")

    # -- build the composed spec
    spec = mujoco.MjSpec.from_file(str(robot_xml))
    # velocity/PD actuators put damping in the actuator bias, which the default
    # Euler integrator treats explicitly -- unstable for the low-inertia wheels.
    # implicitfast integrates that damping implicitly and costs almost nothing.
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    torso = spec.bodies[1]
    torso.name = "torso"
    if torso_inertial is not None:
        M_t, c_t, I_ct = torso_inertial
        torso.explicitinertial = True
        torso.mass = M_t
        torso.ipos = c_t
        torso.iquat = [1, 0, 0, 0]
        torso.fullinertia = [I_ct[0, 0], I_ct[1, 1], I_ct[2, 2],
                             I_ct[0, 1], I_ct[0, 2], I_ct[1, 2]]
    else:
        torso.explicitinertial = False
        torso.mass = 0
        torso.inertia = [0, 0, 0]

    # drop the rigid copies of the attached parts; the torso body frame is
    # identity so spec-level pos are world coordinates
    dropped = 0
    for g in list(spec.geoms):
        if g.meshname in sub_meshes and any(
                np.linalg.norm(np.asarray(g.pos) - p) < POS_TOL for p in sub_part_pos):
            spec.delete(g)
            dropped += 1
    print(f"dropped {dropped} rigid geoms from the torso")

    # attached models resolve mesh files against the robot meshdir
    for sub in subs.values():
        for f in (sub.dir / "assets").glob("*.stl"):
            tgt = args.robot_dir / "assets" / f.name
            if not tgt.exists():
                shutil.copy(f, tgt)
                print(f"copied missing asset {f.name} into {tgt.parent}")

    n_motors = 0
    for sub, side, T_by_body, _ in instances:
        child = mujoco.MjSpec.from_file(str(sub.xml))
        strip_side_suffix(child)
        if args.motor_mass > 0:
            n_motors += add_motor_masses(child, args.motor_mass)
        tune_joints(child)
        for a in list(child.actuators):  # replaced by our actuator model below
            child.delete(a)
        # only the tires need contact; the linkage's collision meshes would
        # grind against each other and fight the loop-closure constraints
        for g in child.geoms:
            if g.meshname != "tire":
                g.contype = 0
                g.conaffinity = 0
        T = T_by_body[1]
        frame = torso.add_frame(name=f"{side}_mount", pos=T[:3, 3], quat=mat2quat(T[:3, :3]))
        spec.attach(child, prefix=f"{side}_", frame=frame)
    if n_motors:
        print(f"added {args.motor_mass} kg x {n_motors} GIM6010-8 motors "
              f"({args.motor_mass * n_motors:.2f} kg total)")

    # polygonal mesh tires make the rolling contact hop facet to facet;
    # swap them for analytic cylinders of the same radius/width
    stand_pose.replace_tire_collision(spec)

    add_actuators(spec, sides, args.kp, args.kv, args.wheel_kv)
    add_imu(spec, torso, args.imu_pos, args.imu_quat)
    add_encoders(spec, sides)

    model = spec.compile()

    def fill_joint_angles(model, qpos):
        for _, side, _, qs in instances:
            for jname, q in qs.items():
                j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_{jname}")
                qpos[model.jnt_qposadr[j]] = q

    def hold_ctrl(model, qpos):
        """ctrl that holds qpos: leg PDs track their joint, wheels command 0 speed."""
        ctrl = np.zeros(model.nu)
        for a in range(model.nu):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, a)
            if not name.endswith("_wheel"):
                j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
                ctrl[a] = qpos[model.jnt_qposadr[j]]
        return ctrl

    # keyframe with the joint angles the robot assembly was posed at
    key_qpos = np.zeros(model.nq)
    fill_joint_angles(model, key_qpos)
    key = spec.add_key(name="assembly")
    key.qpos = key_qpos
    key.ctrl = hold_ctrl(model, key_qpos)
    spec.compile()
    out.write_text(spec.to_xml())
    print(f"wrote {out}")

    scene = args.robot_dir / "scene_full.xml"
    src = args.robot_dir / "scene.xml"
    if src.exists() and not scene.exists():
        scene.write_text(src.read_text().replace("robot.xml", out.name))
        print(f"wrote {scene}")

    # ---- floating-base variant with a symmetric computed spawn pose --------
    # (physical-space IK across sides; see stand_pose.py -- the two leg
    # exports have unrelated joint zeros, so angles must never be copied)
    stand_pose.build_floating(out, args.stand_extension)

    # -- self-check: recompile from disk at the assembly keyframe; geoms must
    # land where the rigid export had them (compiled poses on both sides)
    m = mujoco.MjModel.from_xml_path(str(out))
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_kinematics(m, d)
    ref = {}
    for mesh, pos in compiled_geoms(m_robot, d_robot):
        ref.setdefault(mesh, []).append(pos)
    side_re = re.compile(rf"^({'|'.join(map(re.escape, set(sides)))})_")
    worst = 0.0
    for mesh, pos in compiled_geoms(m, d):
        cands = ref.get(side_re.sub("", mesh), [])
        if cands:
            worst = max(worst, min(np.linalg.norm(pos - p) for p in cands))
    print(f"\nself-check: max geom position error vs rigid export = {worst * 1000:.3f} mm")
    print(f"model: nq={m.nq} nv={m.nv} nu={m.nu} neq={m.neq}")
    print("joints:   ", ", ".join(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(m.njnt)))
    print("actuators:", ", ".join(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a) for a in range(m.nu)))


if __name__ == "__main__":
    main()
