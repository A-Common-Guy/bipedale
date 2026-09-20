#!/usr/bin/env python3
"""Derive the IMU site pose from the roof mesh, and check the model against it.

The Onshape export gives the torso body a frame inherited from whatever face
the CAD assembly happened to be built on: its axes are nowhere near the
robot's own forward/left/up. A site written as pos="0 0 0" with no quat
therefore lands inside the chassis pointing sideways, which silently makes
`imu_quat`, `imu_gyro` and `imu_accel` disagree with the real BNO085 bolted
flat to the roof.

This tool recomputes the pose the site *should* have, straight from geometry,
so the numbers in the MJCF are reproducible after every re-export:

    z  outward normal of the roof plate's upper face
    y  wheel axle direction, oriented right -> left
    x  y cross z, which comes out as forward

The origin is the centre of that face, shifted by MOUNT_OFFSET to wherever the
board is actually bolted down. The shift is tangential, so the frame stays flat
on the roof however far the mount point moves.

  report  print the derived pose, the MJCF snippet and how far the model's
          current site is from it (default)
  check   exit non-zero when the model's site is off, for use in tests

Run from the SandroControl directory:
    python tools/imu_frame.py
    python tools/imu_frame.py --mode check
    python tools/imu_frame.py --offset 0 -0.035   # try a mount point
"""
from __future__ import annotations

import argparse
import pathlib
import sys

import mujoco
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
DEFAULT_MODEL = HERE.parent.parent / "onshape_export" / "robot" / "scene_floating.xml"

# Where the sensor actually sits on the roof face, in metres along the IMU
# frame's own tangent axes, measured from the centre of the face. The board is
# 35 mm right of centre, so the offset along +y (left) is negative. Keep this
# in sync with the physical build: it is what --mode check validates against,
# and moving the board without updating it makes every lever-arm term wrong.
MOUNT_OFFSET = (0.0, -0.035)

# Plate faces are found by clustering triangle normals; anything flatter than
# this against the plate normal is a chamfer or a boss wall, not the face.
_FACE_COS = 0.9
# Triangles this close to the outermost plane are treated as lying in it.
_PLANE_TOL = 3e-4

_POSITION_TOL = 1e-4      # 0.1 mm
_ANGLE_TOL_DEG = 0.05


def _mesh_vertices_in_body(model, data, geom_id, body_id):
    """Roof triangles in the parent body's frame: (verts, faces)."""
    mesh = model.geom_dataid[geom_id]
    v0, nv = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
    f0, nf = model.mesh_faceadr[mesh], model.mesh_facenum[mesh]
    verts = model.mesh_vert[v0 : v0 + nv].reshape(-1, 3).astype(float)
    faces = model.mesh_face[f0 : f0 + nf].reshape(-1, 3)

    # geom_xpos/geom_xmat are the frame mesh_vert is expressed in, so going
    # through them avoids having to replay the compiler's mesh recentering.
    world = data.geom_xpos[geom_id] + verts @ data.geom_xmat[geom_id].reshape(3, 3).T
    R_wb = data.xmat[body_id].reshape(3, 3)
    return (world - data.xpos[body_id]) @ R_wb, faces


def _triangle_normals(verts, faces):
    a, b, c = verts[faces[:, 0]], verts[faces[:, 1]], verts[faces[:, 2]]
    cross = np.cross(b - a, c - a)
    length = np.linalg.norm(cross, axis=1)
    area = 0.5 * length
    normal = cross / np.maximum(length[:, None], 1e-12)
    return (a + b + c) / 3.0, normal, area


def _geom_id(model, body_id, mesh_name, group):
    for geom in range(model.ngeom):
        if model.geom_bodyid[geom] != body_id or model.geom_group[geom] != group:
            continue
        mesh = model.geom_dataid[geom]
        if mesh < 0:
            continue
        if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, mesh) == mesh_name:
            return geom
    raise SystemExit(f"no group-{group} geom using mesh {mesh_name!r}")


def derive(model, data, body="torso", roof_mesh="robot_roof", ref_mesh="body_lower",
           offset=MOUNT_OFFSET):
    """Return (position, 3x3 rotation, diagnostics) of the IMU frame in `body`.

    `offset` is (forward, left) in metres from the centre of the roof's top
    face. It slides the origin within that face, so the frame stays flat on
    the roof and only the mount point moves.
    """
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)
    roof = _geom_id(model, body_id, roof_mesh, 2)
    verts, faces = _mesh_vertices_in_body(model, data, roof, body_id)
    centroid, normal, area = _triangle_normals(verts, faces)

    # The plate's two big faces are the largest area-weighted normal cluster.
    # Taking the pair as +/-n and picking the half that points away from the
    # chassis keeps "up" independent of however the robot is posed.
    weight = {}
    for i in range(len(normal)):
        key = tuple(np.round(normal[i], 3))
        weight[key] = weight.get(key, 0.0) + area[i]
    plate = np.array(max(weight.items(), key=lambda kv: kv[1])[0], dtype=float)
    plate /= np.linalg.norm(plate)

    # Rounding the cluster key costs enough angle to move the plate's plane by
    # more than _PLANE_TOL across a 12 cm face, which would drop part of the
    # face from the centroid. Average the members back to the mesh's own normal.
    for _ in range(3):
        member = (normal @ plate) > 0.999
        plate = (normal[member] * area[member, None]).sum(0)
        plate /= np.linalg.norm(plate)

    ref = _geom_id(model, body_id, ref_mesh, 2)
    ref_verts, ref_faces = _mesh_vertices_in_body(model, data, ref, body_id)
    ref_centroid, _, ref_area = _triangle_normals(ref_verts, ref_faces)
    chassis = (ref_centroid * ref_area[:, None]).sum(0) / ref_area.sum()
    roof_centre = (centroid * area[:, None]).sum(0) / area.sum()
    if plate @ (roof_centre - chassis) < 0:
        plate = -plate
    up = plate

    # Outermost plane along `up`, then the area centroid of the face in it.
    face_up = (normal @ up) > _FACE_COS
    if not face_up.any():
        raise SystemExit("roof plate has no face pointing away from the chassis")
    height = (verts @ up).max()
    top = face_up & ((centroid @ up) > height - _PLANE_TOL)
    centre = (centroid[top] * area[top, None]).sum(0) / area[top].sum()
    centre = centre + up * (height - centre @ up)         # snap onto the plane

    left = _axle_left(model, data, body_id)
    left = left - up * (left @ up)                        # keep the triad square
    left /= np.linalg.norm(left)
    forward = np.cross(left, up)
    forward /= np.linalg.norm(forward)

    position = centre + forward * offset[0] + left * offset[1]
    rotation = np.column_stack([forward, left, up])
    corners = np.vstack([verts[faces[top][:, i]] for i in range(3)]) - position
    diagnostics = {
        "top_area": float(area[top].sum()),
        "flatness": float(np.abs(corners @ up).max()),
        "plate_thickness": float(height - (verts @ up).min()),
        "offset": (float(offset[0]), float(offset[1])),
        "footprint_x": (float((corners @ forward).min()), float((corners @ forward).max())),
        "footprint_y": (float((corners @ left).min()), float((corners @ left).max())),
    }
    return position, rotation, diagnostics


def _axle_left(model, data, body_id):
    """Wheel axle in `body` coordinates, oriented from the right wheel to the left."""
    joints = {}
    for side in ("right", "left"):
        joint = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_wheel"
        )
        if joint < 0:
            raise SystemExit(f"model has no {side}_wheel joint")
        joints[side] = joint
    R_wb = data.xmat[body_id].reshape(3, 3)
    axis = R_wb.T @ data.xaxis[joints["left"]]
    span = R_wb.T @ (
        data.xanchor[joints["left"]] - data.xanchor[joints["right"]]
    )
    if axis @ span < 0:
        axis = -axis
    return axis / np.linalg.norm(axis)


def _quat(rotation):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.ascontiguousarray(rotation).flatten())
    return q / np.linalg.norm(q)


def _fmt(values, digits=9):
    return " ".join(f"{v:.{digits}g}" for v in values)


def _current(model, data, site="imu", body="torso"):
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site)
    if site_id < 0:
        raise SystemExit(f"model has no site named {site!r}")
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)
    if model.site_bodyid[site_id] != body_id:
        raise SystemExit(f"site {site!r} is not attached to {body!r}")
    return model.site_pos[site_id].copy(), model.site_quat[site_id].copy()


def _load(path, keyframe="stand"):
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, keyframe)
    if key >= 0:
        mujoco.mj_resetDataKeyframe(model, data, key)
    mujoco.mj_forward(model, data)
    return model, data


def _deviation(model, data, position, rotation):
    have_pos, have_quat = _current(model, data)
    want_quat = _quat(rotation)
    diff = np.zeros(4)
    mujoco.mju_mulQuat(diff, want_quat, np.array(
        [have_quat[0], -have_quat[1], -have_quat[2], -have_quat[3]]))
    angle = 2.0 * np.degrees(np.arccos(min(1.0, abs(diff[0]))))
    return float(np.linalg.norm(position - have_pos)), angle, have_pos, have_quat


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mode", choices=("report", "check"), default="report")
    parser.add_argument("--model", default=str(DEFAULT_MODEL))
    parser.add_argument(
        "--offset", type=float, nargs=2, metavar=("FORWARD", "LEFT"),
        default=list(MOUNT_OFFSET),
        help="mount point in metres from the centre of the roof's top face "
             f"(default {MOUNT_OFFSET[0]} {MOUNT_OFFSET[1]})",
    )
    args = parser.parse_args()

    model, data = _load(args.model)
    position, rotation, info = derive(model, data, offset=tuple(args.offset))
    drift, angle, have_pos, have_quat = _deviation(model, data, position, rotation)

    # A tangential offset must stay on the face, or the sensor is cantilevered
    # off the roof edge and "flat on the roof" stops being true.
    margin = min(
        info["footprint_x"][1], -info["footprint_x"][0],
        info["footprint_y"][1], -info["footprint_y"][0],
    )
    if margin < 0:
        print(f"offset {tuple(args.offset)} puts the site {-margin * 1e3:.1f} mm "
              f"outside the roof's top face", file=sys.stderr)
        return 2

    if args.mode == "check":
        if drift <= _POSITION_TOL and angle <= _ANGLE_TOL_DEG:
            print(f"imu site matches the roof top face "
                  f"({drift * 1e3:.3f} mm, {angle:.4f} deg)")
            return 0
        print(f"imu site is off the roof top face: {drift * 1e3:.2f} mm, "
              f"{angle:.2f} deg", file=sys.stderr)
        return 1

    torso = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "torso")
    R_wt = data.xmat[torso].reshape(3, 3)
    print(f"model: {args.model}")
    print(f"roof plate: {info['top_area'] * 1e4:.1f} cm^2 top face, "
          f"{info['plate_thickness'] * 1e3:.1f} mm thick, "
          f"face flat to {info['flatness'] * 1e6:.1f} um")
    print(f"mount offset from the face centre: "
          f"{info['offset'][0] * 1e3:+.1f} mm forward, "
          f"{info['offset'][1] * 1e3:+.1f} mm left")
    print(f"  face spans {info['footprint_x'][0] * 1e3:+.1f} .. "
          f"{info['footprint_x'][1] * 1e3:+.1f} mm forward, "
          f"{info['footprint_y'][0] * 1e3:+.1f} .. "
          f"{info['footprint_y'][1] * 1e3:+.1f} mm left of the site "
          f"({margin * 1e3:.1f} mm to the nearest edge)")
    print("\nderived IMU frame, in torso coordinates")
    print(f"  pos  {_fmt(position, 8)}")
    print(f"  quat {_fmt(_quat(rotation))}")
    for label, axis in zip(("x forward", "y left   ", "z up     "), rotation.T):
        print(f"  {label} {_fmt(axis, 6):<40} -> world {_fmt(R_wt @ axis, 4)}")

    print("\nat the stand keyframe")
    print(f"  site origin in world      {_fmt(data.xpos[torso] + R_wt @ position, 5)}")
    tilt = np.degrees(np.arccos(np.clip((R_wt @ rotation[:, 2]) @ [0, 0, 1], -1, 1)))
    print(f"  roof plane vs. horizontal {tilt:.3f} deg")

    print("\ncurrent site in the model")
    print(f"  pos  {_fmt(have_pos, 8)}")
    print(f"  quat {_fmt(have_quat)}")
    print(f"  off by {drift * 1e3:.2f} mm, {angle:.2f} deg")

    print("\nMJCF, replacing the <site name=\"imu\" .../> line on the torso body:")
    print(f'      <frame name="imu_mount" pos="{_fmt(position, 8)}" '
          f'quat="{_fmt(_quat(rotation))}">')
    print('        <site name="imu" size="0.004" group="2" rgba="1 0.9 0.1 1"/>')
    print("      </frame>")
    return 0


if __name__ == "__main__":
    sys.exit(main())
