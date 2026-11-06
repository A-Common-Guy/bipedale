"""The BNO085 frame of the composed robot, derived from the board part in the CAD.

Shared by compose_robot.py (which writes it into the model) and
SandroControl/tools/imu_frame.py (which checks a model against it). Needs only
mujoco and numpy.

The `imu` site is the frame the controllers expect, x forward, y left, z up:
z is the board normal pointing up, y the wheel axle (right to left) in the
board plane, x = y cross z. Its origin is the centre of the chip. The chip's
own axes (the board's CAD part frame) are returned separately: the real
driver's mount rotation (sandro_hardware BNO085Sensor mount_yaw_rad) turns
them into the `imu` frame.
"""
from __future__ import annotations

import re

import mujoco
import numpy as np

# Torso meshes of the BNO085 chip (a multi-body import, one mesh per body).
IMU_BOARD_PATTERN = r"^bno085"
# The board's part y and z must lie within this of the plane normal to the axle.
IMU_BOARD_MAX_TILT_DEG = 5.0


def _pose(pos, quat):
    R = np.empty(9)
    mujoco.mju_quat2Mat(R, np.asarray(quat, dtype=float))
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R.reshape(3, 3), pos
    return T


def _quat(R):
    q = np.empty(4)
    mujoco.mju_mat2Quat(q, np.ascontiguousarray(R).ravel())
    return q


def axle_left(model: mujoco.MjModel, data: mujoco.MjData, body_id: int):
    """Wheel axle in `body` coordinates, oriented from the right wheel to the left."""
    j = {s: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{s}_wheel")
         for s in ("right", "left")}
    if min(j.values()) < 0:
        raise SystemExit("model has no right_wheel/left_wheel joints")
    R_wb = data.xmat[body_id].reshape(3, 3)
    axis = R_wb.T @ data.xaxis[j["left"]]
    span = R_wb.T @ (data.xanchor[j["left"]] - data.xanchor[j["right"]])
    if axis @ span < 0:
        axis = -axis
    return axis / np.linalg.norm(axis)


def imu_from_board(spec: mujoco.MjSpec, model: mujoco.MjModel, data: mujoco.MjData,
                   pattern: str = IMU_BOARD_PATTERN):
    """(pos, quat, chip quat, report) of the IMU frame in the torso, or None.

    `model` is `spec` compiled and `data` any pose with kinematics and
    subtree centres of mass computed. The authored geom pose is the Onshape
    part frame (the compiler re-centres meshes, so it is read from the spec);
    every matched body must share it. `pos`/`quat` place the `imu` frame in
    the torso, `chip quat` is the part frame in the `imu` frame. None when no
    torso geom matches `pattern`.
    """
    torso = spec.bodies[1]
    authored = [g for g in torso.geoms
                if g.type == mujoco.mjtGeom.mjGEOM_MESH and re.search(pattern, g.meshname, re.I)]
    meshes = sorted({g.meshname for g in authored})
    if not authored:
        return None
    T_part = _pose(authored[0].pos, authored[0].quat)
    if any(np.abs(_pose(g.pos, g.quat)[:3, :3] - T_part[:3, :3]).max() > 1e-4 for g in authored):
        raise SystemExit(f"IMU board pattern {pattern!r} matches parts with different "
                         f"frames: {meshes}")
    tid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, torso.name)
    mesh_ids = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MESH, n) for n in meshes}
    in_part = []
    for g in range(model.ngeom):
        if model.geom_bodyid[g] != tid or model.geom_dataid[g] not in mesh_ids:
            continue
        mid = model.geom_dataid[g]
        va, vn = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
        R_g = np.empty(9)
        mujoco.mju_quat2Mat(R_g, model.geom_quat[g])
        in_torso = model.mesh_vert[va:va + vn] @ R_g.reshape(3, 3).T + model.geom_pos[g]
        in_part.append((in_torso - T_part[:3, 3]) @ T_part[:3, :3])
    in_part = np.concatenate(in_part)
    lo, hi = in_part.min(axis=0), in_part.max(axis=0)
    pos = T_part[:3, :3] @ (0.5 * (lo + hi)) + T_part[:3, 3]
    label = f"{meshes[0]} (+{len(meshes) - 1})" if len(meshes) > 1 else meshes[0]

    # Up is from the axle towards the centre of mass (the stand pose puts it
    # above the axle); forward = left x up.
    left_axle = axle_left(model, data, tid)
    wheels = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{s}_wheel")
              for s in ("right", "left")]
    R_wt = data.xmat[tid].reshape(3, 3)
    axle = R_wt.T @ (0.5 * (data.xanchor[wheels[0]] + data.xanchor[wheels[1]]) - data.xpos[tid])
    com = R_wt.T @ (data.subtree_com[tid] - data.xpos[tid])
    up_ref = (com - axle) - left_axle * float((com - axle) @ left_axle)
    up_ref /= np.linalg.norm(up_ref)
    forward_ref = np.cross(left_axle, up_ref)

    part_y, normal = T_part[:3, 1], T_part[:3, 2]
    limit = np.sin(np.radians(IMU_BOARD_MAX_TILT_DEG))
    if abs(float(normal @ left_axle)) > limit or abs(float(part_y @ left_axle)) > limit:
        raise SystemExit(f"IMU board {label}: part y or z is not perpendicular to the wheel axle")
    up = normal if normal @ up_ref > 0 else -normal
    y = left_axle - up * float(up @ left_axle)
    y /= np.linalg.norm(y)
    R = np.column_stack([np.cross(y, up), y, up])
    report = (f"IMU from board '{label}': box {np.round(1e3 * (hi - lo), 1)} mm, "
              f"z = {'+' if up is normal else '-'}part z, board part y points "
              f"{'forward' if part_y @ forward_ref > 0 else 'backward'}, board tilted "
              f"{np.degrees(np.arccos(min(1.0, float(up @ up_ref)))):.2f} deg from the "
              f"axle-to-CoM direction")
    return pos, _quat(R), _quat(R.T @ T_part[:3, :3]), report
