"""The RealSense D435 frames of the composed robot, derived from the camera part in the CAD.

Used by compose_robot.py. Needs only mujoco and numpy.

The CAD camera is a plain 90 x 25 x 25 mm box on the front bracket, found by
its size rather than its Onshape name. Of its two short axes the one closer to
the body's forward direction is the optical axis, and the face it points out
of is the front glass. The returned frame is realsense2_camera's
`camera_link`: the depth origin (the left imager, 4.3 mm behind the glass and
17.5 mm left of the camera centre, at half height; realsense2_description's
_d435.urdf.xacro), x along the optical axis, y left, z up.
"""
from __future__ import annotations

import mujoco
import numpy as np

from imu_mount import _pose, _quat

D435_BOX_M = (0.090, 0.025, 0.025)
D435_BOX_TOL_M = 0.001
D435_KG = 0.072  # Intel datasheet
D435_GLASS_TO_DEPTH_M = 0.0043
D435_CENTRE_TO_DEPTH_Y_M = 0.0175
# Depth stream field of view (datasheet, 16:10); fovy is the vertical one.
D435_DEPTH_FOVY_DEG = 58.0
D435_DEPTH_RESOLUTION = (424, 240)
CAMERA_MAX_PITCH_DEG = 60.0


def _part_box(model: mujoco.MjModel, torso_id: int, g_spec, mesh_name: str):
    """(lo, hi) of a torso mesh in its authored (Onshape part) frame."""
    T_part = _pose(g_spec.pos, g_spec.quat)
    mid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MESH, mesh_name)
    for g in range(model.ngeom):
        if model.geom_bodyid[g] == torso_id and model.geom_dataid[g] == mid:
            break
    else:
        return None
    va, vn = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
    R_g = np.empty(9)
    mujoco.mju_quat2Mat(R_g, model.geom_quat[g])
    in_torso = model.mesh_vert[va:va + vn] @ R_g.reshape(3, 3).T + model.geom_pos[g]
    in_part = (in_torso - T_part[:3, 3]) @ T_part[:3, :3]
    return T_part, in_part.min(axis=0), in_part.max(axis=0)


def find_d435(spec: mujoco.MjSpec, model: mujoco.MjModel):
    """(mesh name, part pose, lo, hi) of the torso's D435 box, or None."""
    torso = spec.bodies[1]
    tid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, torso.name)
    found, seen = [], set()
    for g in torso.geoms:
        if g.type != mujoco.mjtGeom.mjGEOM_MESH or g.meshname in seen:
            continue
        seen.add(g.meshname)
        box = _part_box(model, tid, g, g.meshname)
        if box is None:
            continue
        T_part, lo, hi = box
        if np.allclose(np.sort(hi - lo), np.sort(D435_BOX_M), atol=D435_BOX_TOL_M):
            found.append((g.meshname, T_part, lo, hi))
    if len(found) > 1:
        raise SystemExit(f"more than one torso part is a D435-sized box: {[f[0] for f in found]}")
    return found[0] if found else None


def camera_from_part(spec: mujoco.MjSpec, model: mujoco.MjModel, imu_pos, imu_quat):
    """(mesh name, pos, quat, report) of `camera_link` in the torso, or None.

    `imu_pos`/`imu_quat` are the `imu` frame in the torso (x forward, y left,
    z up), which fixes forward and left.
    """
    d435 = find_d435(spec, model)
    if d435 is None:
        return None
    mesh, T_part, lo, hi = d435
    R_imu = _pose(imu_pos, imu_quat)[:3, :3]
    forward_ref, left_ref = R_imu[:, 0], R_imu[:, 1]

    size = hi - lo
    long_axis = int(np.argmax(size))
    left = T_part[:3, long_axis] * np.sign(T_part[:3, long_axis] @ left_ref)
    short = [a for a in range(3) if a != long_axis]
    optical = max(short, key=lambda a: abs(T_part[:3, a] @ forward_ref))
    sign = np.sign(T_part[:3, optical] @ forward_ref)
    forward = T_part[:3, optical] * sign
    up = np.cross(forward, left)
    R = np.column_stack([forward, left, up])

    centre_part = 0.5 * (lo + hi)
    glass_part = centre_part.copy()
    glass_part[optical] = hi[optical] if sign > 0 else lo[optical]
    glass = T_part[:3, :3] @ glass_part + T_part[:3, 3]
    pos = glass - D435_GLASS_TO_DEPTH_M * forward + D435_CENTRE_TO_DEPTH_Y_M * left

    in_imu = R_imu.T @ R
    pitch = float(np.degrees(np.arcsin(np.clip(-in_imu[2, 0], -1.0, 1.0))))
    yaw = float(np.degrees(np.arctan2(in_imu[1, 0], in_imu[0, 0])))
    roll = float(np.degrees(np.arctan2(in_imu[2, 1], in_imu[2, 2])))
    if abs(pitch) > CAMERA_MAX_PITCH_DEG:
        raise SystemExit(f"D435 '{mesh}': optical axis {pitch:.1f} deg from the body's "
                         "forward direction; is the camera part the right one?")
    offset = R_imu.T @ (pos - np.asarray(imu_pos))
    report = (f"D435 from part '{mesh}': depth origin {np.round(1e3 * offset, 1)} mm from the "
              f"IMU, pitched {pitch:.2f} deg down, roll {roll:.2f}, yaw {yaw:.2f} deg")
    return mesh, pos, _quat(R), report


def add_camera(spec: mujoco.MjSpec, torso, pos, quat):
    """The `camera_mount` frame (`camera_link`) with its sites and the depth camera.

    `camera_depth_optical` is the optical frame of the depth image (z along
    the view, x right, y down), as realsense2_camera stamps it. `d435_depth`
    is a MuJoCo camera with the depth stream's field of view (MuJoCo cameras
    look along -z with y up).
    """
    G = mujoco.mjtGeom
    frame = torso.add_frame(name="camera_mount", pos=pos, quat=quat)
    frame.add_site(name="camera_link", size=[0.004, 0.004, 0.004], group=2,
                   rgba=[0.2, 0.8, 0.9, 1])
    frame.add_site(name="camera_depth_optical", quat=[0.5, -0.5, 0.5, -0.5],
                   size=[0.003, 0.003, 0.003], group=3)
    frame.add_site(name="camera_view", type=G.mjGEOM_CYLINDER, fromto=[0, 0, 0, 0.06, 0, 0],
                   size=[0.0015, 0, 0], group=2, rgba=[0.2, 0.8, 0.9, 1])
    cam = frame.add_camera(name="d435_depth", quat=[0.5, 0.5, -0.5, -0.5],
                           fovy=D435_DEPTH_FOVY_DEG)
    cam.resolution = list(D435_DEPTH_RESOLUTION)
    return frame
