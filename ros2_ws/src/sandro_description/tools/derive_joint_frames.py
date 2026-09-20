#!/usr/bin/env python3
"""Derive the logical joint frames of the six actuated joints from the export.

The Onshape export parametrises every hinge in whatever frame the CAD left
it in, so the raw joint values of the four leg motors are unrelated between
sides and the two wheel hinges point in opposite directions. The controllers
and the physical drives instead use one convention:

* leg joints: zero when the upper link hangs straight down toward the wheel,
  positive when the link opens away from the leg centre (the convention of
  ``sandro_controllers/kinematics.hpp`` and ``leg_control/leg_server.py``);
* wheel joints: positive rotation rolls the robot forward.

This tool reproduces the derivation the legacy ``SandroControl`` runtime did
at start-up and prints the ``JOINT_FRAMES`` table that ``sync_model.py``
bakes into ``robot_ros2.xml`` through ``ref`` and axis flips. It needs the
``mujoco`` Python package, which the ROS image does not ship, so run it from
any environment that has it whenever the export changes:

    python3 tools/derive_joint_frames.py [--check]

``--check`` exits non-zero when the derived table differs from the one in
``sync_model.py``.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import sync_model  # noqa: E402

SIDES = ("right", "left")
TOLERANCE_RAD = 1e-6


def _unit(vector):
    return vector / np.linalg.norm(vector)


def derive(model_path: Path, keyframe: str = "stand") -> dict[str, tuple[float, float]]:
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, keyframe)
    if key < 0:
        raise ValueError(f"keyframe {keyframe!r} missing from {model_path}")
    mujoco.mj_resetDataKeyframe(model, data, key)
    mujoco.mj_forward(model, data)

    def joint(name):
        identifier = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if identifier < 0:
            raise ValueError(f"joint {name!r} missing from {model_path}")
        return identifier

    def passive_below(motor_joint):
        motor_body = int(model.jnt_bodyid[motor_joint])
        for candidate in range(model.njnt):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, candidate)
            if not name or "passive" not in name:
                continue
            body = int(model.jnt_bodyid[candidate])
            while body and body != motor_body:
                body = int(model.body_parentid[body])
            if body == motor_body:
                return candidate
        raise RuntimeError(f"no passive joint below motor joint {motor_joint}")

    imu = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "imu")

    frames: dict[str, tuple[float, float]] = {}
    for side in SIDES:
        names = (f"{side}_motor_inner", f"{side}_motor_outer")
        joints = tuple(joint(name) for name in names)
        anchors = tuple(data.xanchor[j].copy() for j in joints)
        base_direction = _unit(anchors[1] - anchors[0])
        axis0 = _unit(data.xaxis[joints[0]].copy())
        down = _unit(np.cross(axis0, base_direction))
        wheel = joint(f"{side}_wheel")
        if float(down @ (data.xanchor[wheel] - 0.5 * (anchors[0] + anchors[1]))) < 0:
            down = -down
        for index, (name, j) in enumerate(zip(names, joints)):
            passive = passive_below(j)
            axis = _unit(data.xaxis[j].copy())
            vertical = _unit(down - axis * float(down @ axis))
            link = data.xanchor[passive] - data.xanchor[j]
            link = _unit(link - axis * float(link @ axis))
            angle = math.atan2(float(axis @ np.cross(vertical, link)), float(vertical @ link))
            raw = float(data.qpos[model.jnt_qposadr[j]])
            # Positive opens the upper link away from the leg centre: inner
            # toward the outer pivot's opposite side, outer the other way.
            desired = -base_direction if index == 0 else base_direction
            sign = 1.0 if float(np.cross(axis, vertical) @ desired) >= 0.0 else -1.0
            frames[name] = (sign, raw - angle)
        # Positive wheel rotation must roll the robot forward. Rotation about
        # the body's +y (left) axis takes the contact point backward, i.e.
        # rolls forward, so a hinge axis along -y_imu needs a flip.
        wheel_axis = _unit(data.xaxis[wheel].copy())
        left = data.site_xmat[imu].reshape(3, 3)[:, 1]
        wheel_sign = 1.0 if float(wheel_axis @ left) > 0.0 else -1.0
        frames[f"{side}_wheel"] = (wheel_sign, 0.0)
    return frames


def format_table(frames: dict[str, tuple[float, float]]) -> str:
    lines = ["JOINT_FRAMES = {"]
    for name in sync_model.STABLE_JOINTS:
        sign, zero = frames[name]
        lines.append(f'    "{name}": ({sign:+.1f}, {zero:+.8f}),')
    lines.append("}")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument(
        "--source", type=Path, default=sync_model.DEFAULT_SOURCE,
        help=f"Onshape-export model directory (default: {sync_model.DEFAULT_SOURCE})",
    )
    parser.add_argument("--check", action="store_true",
                        help="compare against sync_model.JOINT_FRAMES")
    args = parser.parse_args()
    frames = derive(args.source.resolve() / sync_model.MJCF_FILES[0])
    print(format_table(frames))
    if not args.check:
        return 0
    clean = True
    for name, (sign, zero) in frames.items():
        expected_sign, expected_zero = sync_model.JOINT_FRAMES[name]
        if sign != expected_sign or abs(zero - expected_zero) > TOLERANCE_RAD:
            print(f"mismatch for {name}: derived ({sign:+.1f}, {zero:+.8f}) "
                  f"vs sync_model ({expected_sign:+.1f}, {expected_zero:+.8f})",
                  file=sys.stderr)
            clean = False
    return 0 if clean else 1


if __name__ == "__main__":
    raise SystemExit(main())
