#!/usr/bin/env python3
"""Deterministically sync/check the canonical MJCF and its referenced meshes."""

from __future__ import annotations

import argparse
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MONOREPO_SOURCE = PACKAGE_ROOT.parents[2] / "onshape_export" / "robot"
DEFAULT_SOURCE = (
    MONOREPO_SOURCE if (MONOREPO_SOURCE / "robot_full_floating.xml").is_file()
    else PACKAGE_ROOT / "mjcf"
)
MJCF_FILES = ("robot_full_floating.xml", "scene_floating.xml")
# Identified reality applied on top of the CAD export when generating the
# ROS model (see apply_identified). Kept next to the model so a regenerated
# export never loses it.
IDENTIFIED_FILE = PACKAGE_ROOT / "config" / "sim_identified.yaml"
ROS2_MJCF_FILES = ("robot_ros2.xml", "scene_ros2.xml")
# Physics step of the ROS model. mujoco_ros2_control publishes /clock once per
# step and the controller manager sleeps on it until the next 1/300 s
# boundary, so the step must divide the control period: with MuJoCo's default
# 2 ms the controller wakes every 4 ms and runs at 250 Hz, against the
# robot's 300 Hz. Four steps per cycle also let the drive law
# (sandro_sim/MitDrivePlugin) act on fresh state every 0.83 ms.
CONTROL_RATE_HZ = 300
PHYSICS_STEPS_PER_CONTROL_CYCLE = 4
PHYSICS_TIMESTEP_S = 1.0 / (CONTROL_RATE_HZ * PHYSICS_STEPS_PER_CONTROL_CYCLE)
# Logical frames of the six actuated joints, derived from the export by
# tools/derive_joint_frames.py (needs the mujoco Python package; rerun it after
# every CAD export). Entry: (sign, zero_rad).
#   sign  +1 keeps the exported hinge axis, -1 flips it;
#   zero  exported joint value at which the logical angle is zero.
# Logical leg angles are zero with the upper link hanging straight down and
# positive when it opens away from the leg centre, matching the controllers,
# the legacy runtime, and the calibrated physical drives. Logical wheel angles
# increase when the robot rolls forward on both sides. The raw export has the
# left wheel hinge pointing the opposite way and unrelated leg zeros per side.
JOINT_FRAMES = {
    "right_motor_inner": (+1.0, -0.31663584),
    "right_motor_outer": (-1.0, +0.63591621),
    "right_wheel": (+1.0, +0.00000000),
    "left_motor_inner": (+1.0, -0.81789191),
    "left_motor_outer": (-1.0, +1.10314611),
    "left_wheel": (-1.0, +0.00000000),
}
STABLE_JOINTS = (
    "right_motor_inner",
    "right_motor_outer",
    "right_wheel",
    "left_motor_inner",
    "left_motor_outer",
    "left_wheel",
)


def inventory(source: Path) -> list[tuple[Path, Path]]:
    robot = ET.parse(source / MJCF_FILES[0]).getroot()
    mesh_names = sorted({mesh.attrib["file"] for mesh in robot.findall(".//mesh")})
    pairs = [
        (source / name, PACKAGE_ROOT / "mjcf" / name) for name in MJCF_FILES
    ]
    pairs.extend(
        (source / "assets" / name, PACKAGE_ROOT / "mjcf" / "assets" / name)
        for name in mesh_names
    )
    return pairs


def validate_source(source: Path) -> None:
    root = ET.parse(source / MJCF_FILES[0]).getroot()
    named_joints = {node.attrib.get("name") for node in root.findall(".//joint")}
    actuators = {node.attrib.get("joint") for node in root.findall("./actuator/*")}
    missing = set(STABLE_JOINTS) - named_joints
    if missing or actuators != set(STABLE_JOINTS):
        raise ValueError(
            f"stable joint contract changed: missing={sorted(missing)}, "
            f"actuators={sorted(actuators)}"
        )
    imu_sites = root.findall(".//site[@name='imu']")
    if len(imu_sites) != 1:
        raise ValueError("canonical MJCF must contain exactly one imu site")


def sync(source: Path) -> None:
    validate_source(source)
    for src, dst in inventory(source):
        if not src.is_file():
            raise FileNotFoundError(src)
        dst.parent.mkdir(parents=True, exist_ok=True)
        if not dst.exists() or src.read_bytes() != dst.read_bytes():
            temporary = dst.with_suffix(dst.suffix + ".tmp")
            shutil.copyfile(src, temporary)
            temporary.replace(dst)
    for name, content in ros2_models(source).items():
        destination = PACKAGE_ROOT / "mjcf" / name
        destination.write_text(content)


def check(source: Path) -> bool:
    validate_source(source)
    clean = True
    for src, dst in inventory(source):
        if not src.is_file():
            print(f"missing source: {src}", file=sys.stderr)
            clean = False
        elif not dst.is_file():
            print(f"missing packaged file: {dst}", file=sys.stderr)
            clean = False
        elif src.read_bytes() != dst.read_bytes():
            print(f"out of date: {dst}", file=sys.stderr)
            clean = False
    for name, content in ros2_models(source).items():
        destination = PACKAGE_ROOT / "mjcf" / name
        if not destination.is_file() or destination.read_text() != content:
            print(f"out of date: {destination}", file=sys.stderr)
            clean = False
    return clean


def _serialized(root: ET.Element) -> str:
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode") + "\n"


def logical_joint_value(name: str, raw: float) -> float:
    """Map an exported joint value to the logical convention of JOINT_FRAMES."""
    sign, zero = JOINT_FRAMES[name]
    return sign * (raw - zero)


def joint_reference(name: str) -> float:
    """`ref` attribute that makes MuJoCo report logical_joint_value()."""
    sign, zero = JOINT_FRAMES[name]
    return -zero if sign > 0 else zero


def _format(value: float) -> str:
    text = f"{value:.8g}"
    return "0" if text in ("-0", "0.0", "-0.0") else text


def _qpos_layout(robot: ET.Element) -> list[tuple[str, int]]:
    """(joint name, qpos address) for every joint in document order."""
    layout = []
    address = 0
    # <default> classes also carry <joint> elements; only worldbody ones count.
    for joint in robot.find("./worldbody").iter("joint"):
        kind = joint.attrib.get("type", "hinge")
        width = {"free": 7, "ball": 4, "hinge": 1, "slide": 1}[kind]
        layout.append((joint.attrib.get("name", ""), address))
        address += width
    return layout


def _apply_joint_frames(robot: ET.Element) -> None:
    joints = {
        joint.attrib.get("name"): joint
        for joint in robot.find("./worldbody").iter("joint")
        if joint.attrib.get("name") in JOINT_FRAMES
    }
    missing = set(JOINT_FRAMES) - set(joints)
    if missing:
        raise ValueError(f"actuated joints missing from export: {sorted(missing)}")
    for name, joint in joints.items():
        if joint.attrib.get("type", "hinge") != "hinge":
            raise ValueError(f"{name} must be a hinge to re-parametrise")
        if "ref" in joint.attrib:
            raise ValueError(f"{name} already carries a ref; export changed")
        sign, _ = JOINT_FRAMES[name]
        axis = [float(v) for v in joint.attrib.get("axis", "0 0 1").split()]
        joint.set("axis", " ".join(_format(sign * v) for v in axis))
        joint.set("ref", _format(joint_reference(name)))
    layout = dict(_qpos_layout(robot))
    for key in robot.findall("./keyframe/key"):
        qpos = [float(v) for v in key.attrib["qpos"].split()]
        for name in JOINT_FRAMES:
            qpos[layout[name]] = logical_joint_value(name, qpos[layout[name]])
        key.set("qpos", " ".join(_format(v) for v in qpos))


def load_identified() -> dict:
    """Read config/sim_identified.yaml; an absent file means CAD values only."""
    if not IDENTIFIED_FILE.is_file():
        return {}
    import yaml  # local import: only the generator needs it

    with IDENTIFIED_FILE.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def apply_identified(robot: ET.Element, identified: dict) -> None:
    """Fold identified physical parameters into the generated ROS model.

    The CAD export is the geometric truth; what it cannot know (tyre contact,
    drive friction and reflected inertia, torque constant error, payload,
    sensor noise) lives in config/sim_identified.yaml and is applied here so
    the simulation tracks the measured robot. Every key is optional.
    """
    if not identified:
        return
    worldbody = robot.find("./worldbody")

    tyres = identified.get("tyre") or {}
    if tyres:
        found = 0
        drop = 0.0
        for geom in worldbody.iter("geom"):
            name = geom.attrib.get("name", "")
            if not name.endswith("_tire_cyl"):
                continue
            found += 1
            for key in ("condim", "friction", "solref", "solimp", "margin"):
                if key in tyres:
                    geom.set(key, _format_list(tyres[key]))
            if "radius_m" in tyres:
                size = [float(v) for v in geom.attrib["size"].split()]
                drop = size[0] - float(tyres["radius_m"])
                geom.set("size", _format_list([float(tyres["radius_m"])] + size[1:]))
        if found != 2:
            raise ValueError(f"expected two *_tire_cyl geoms, found {found}")
        # Keyframes keep the wheels on the floor: the free joint's height
        # comes down by the radius lost to the squash.
        for key in robot.findall("./keyframe/key"):
            qpos = [float(v) for v in key.attrib["qpos"].split()]
            qpos[2] -= drop
            key.set("qpos", " ".join(_format(v) for v in qpos))

    joints = identified.get("joints") or {}
    for group, names in (("legs", ("right_motor_inner", "right_motor_outer",
                                   "left_motor_inner", "left_motor_outer")),
                         ("wheels", ("right_wheel", "left_wheel"))):
        values = joints.get(group) or {}
        if not values:
            continue
        for joint in worldbody.iter("joint"):
            if joint.attrib.get("name") in names:
                for key in ("damping", "frictionloss", "armature"):
                    if key in values:
                        joint.set(key, _format(float(values[key])))

    actuators = identified.get("actuators") or {}
    for group, names in (("legs", ("right_motor_inner", "right_motor_outer",
                                   "left_motor_inner", "left_motor_outer")),
                         ("wheels", ("right_wheel", "left_wheel"))):
        scale = (actuators.get(group) or {}).get("torque_scale")
        if scale is None:
            continue
        # A commanded torque becomes scale * command at the joint: the ratio
        # of real delivered torque to the drive's reported torque.
        for motor in robot.findall("./actuator/motor"):
            if motor.attrib.get("joint") in names:
                motor.set("gear", _format(float(scale)))

    payload = identified.get("payload") or {}
    if identified.get("masses"):
        apply_masses(worldbody, identified["masses"], float(payload.get("mass_kg", 0.0)))
    if payload.get("mass_kg", 0.0) > 0.0:
        torso = worldbody.find("./body[@name='torso']")
        if torso is None:
            raise ValueError("torso body not found for payload")
        # Extra mass the CAD does not carry (battery, electronics, cabling),
        # placed in the IMU frame (x forward, y left, z up) relative to the
        # IMU site. It has to be a child body with its own inertial element:
        # the torso carries an explicit <inertial>, and MuJoCo then ignores
        # geom masses on that body entirely. A jointless child body is welded
        # to its parent and its inertia is added to the assembly.
        frame = torso.find("./frame[@name='imu_mount']")
        if frame is None:
            raise ValueError("imu_mount frame not found for payload")
        mass = float(payload["mass_kg"])
        pos = payload.get("position_m", [0.0, 0.0, 0.0])
        size = [float(v) for v in payload.get("half_size_m", [0.03, 0.03, 0.02])]
        a, b, c = size
        inertia = [mass / 3.0 * (b * b + c * c), mass / 3.0 * (a * a + c * c),
                   mass / 3.0 * (a * a + b * b)]
        body = ET.SubElement(frame, "body", {
            "name": "identified_payload", "pos": _format_list(pos)})
        ET.SubElement(body, "inertial", {
            "pos": "0 0 0", "mass": _format(mass),
            "diaginertia": _format_list(inertia)})
        ET.SubElement(body, "geom", {
            "name": "identified_payload", "type": "box", "group": "3",
            "contype": "0", "conaffinity": "0", "size": _format_list(size),
            "rgba": "0.9 0.3 0.3 0.4",
        })

    mount = identified.get("imu_mount") or {}
    roll_bias = float(mount.get("roll_offset_rad", 0.0))
    if roll_bias != 0.0:
        # Rotate the IMU frame about its own x (forward) axis so a level body
        # reads this roll, like the physical mounting does. The controller's
        # tasks.roll.imu_offset_rad then applies identically on both backends.
        frame = worldbody.find("./body[@name='torso']/frame[@name='imu_mount']")
        if frame is None:
            raise ValueError("imu_mount frame not found for the IMU roll offset")
        import math

        w, x, y, z = (float(v) for v in frame.attrib["quat"].split())
        cw, sx = math.cos(0.5 * roll_bias), math.sin(0.5 * roll_bias)
        # q_frame * q_x(roll): rotation about the frame's own x axis.
        frame.set("quat", _format_list([
            w * cw - x * sx, w * sx + x * cw, y * cw + z * sx, z * cw - y * sx]))

    sensors = identified.get("imu_noise") or {}
    for sensor_name, key in (("imu_quat", "orientation"), ("imu_gyro", "gyro"),
                             ("imu_accel", "accelerometer")):
        if key in sensors:
            for sensor in robot.find("./sensor"):
                if sensor.attrib.get("name") == sensor_name:
                    sensor.set("noise", _format(float(sensors[key])))


def apply_masses(worldbody: ET.Element, masses: dict, payload_kg: float) -> None:
    """Replace the CAD body masses with the weighed components.

    Each body keeps the CAD centre of mass and principal axes; mass and
    inertia scale by one ratio. The payload's mass comes out of the hub.
    """
    leg_motors = ("right_motor_inner", "right_motor_outer",
                  "left_motor_inner", "left_motor_outer")
    joint_body: dict[str, ET.Element] = {}
    parent: dict[ET.Element, ET.Element] = {}
    for body in worldbody.iter("body"):
        for child in body.findall("body"):
            parent[child] = body
        for joint in body.findall("joint"):
            joint_body[joint.attrib.get("name", "")] = body

    def inertial(body: ET.Element) -> ET.Element:
        element = body.find("inertial")
        if element is None:
            raise ValueError(f"body {body.attrib.get('name')} has no inertial")
        return element

    def set_mass(bodies: list[ET.Element], kg: float) -> None:
        cad = sum(float(inertial(b).attrib["mass"]) for b in bodies)
        if kg <= 0.0 or cad <= 0.0:
            raise ValueError(f"cannot set {[b.attrib.get('name') for b in bodies]} to {kg} kg")
        for body in bodies:
            element = inertial(body)
            element.set("mass", _format(float(element.attrib["mass"]) * kg / cad))
            for key in ("diaginertia", "fullinertia"):
                if key in element.attrib:
                    element.set(key, _format_list(
                        [float(v) * kg / cad for v in element.attrib[key].split()]))

    torso = worldbody.find("./body[@name='torso']")
    if torso is None:
        raise ValueError("torso body not found for the measured masses")
    set_mass([torso], float(masses["central_hub_kg"]) - payload_kg)

    holders: dict[ET.Element, int] = {}
    for name in leg_motors:
        crank = joint_body[name]
        set_mass([crank], float(masses["upper_crank_kg"]))
        holders[parent[crank]] = holders.get(parent[crank], 0) + 1
    for holder, count in holders.items():
        if holder is torso or holder.find("joint") is not None:
            raise ValueError(f"leg motor holder {holder.attrib.get('name')} is not a "
                             "jointless child of the torso")
        set_mass([holder], count * float(masses["leg_motor_kg"]))

    for side in ("right", "left"):
        links = [b for n, b in joint_body.items() if n.startswith(f"{side}_passive_")]
        if len(links) != 2:
            raise ValueError(f"expected two {side} passive links, found {len(links)}")
        set_mass(links, float(masses["lower_links_per_side_kg"]))
        set_mass([joint_body[f"{side}_wheel"]], float(masses["wheel_kg"]))

    total = payload_kg + sum(float(b.find("inertial").attrib["mass"])
                             for b in worldbody.iter("body") if b.find("inertial") is not None)
    if abs(total - float(masses["total_kg"])) > 0.001:
        raise ValueError(f"measured components add up to {total:.4f} kg, "
                         f"not the weighed {masses['total_kg']} kg")


def _format_list(values) -> str:
    if isinstance(values, (int, float)):
        return _format(float(values))
    return " ".join(_format(float(v)) for v in values)


def ros2_models(source: Path) -> dict[str, str]:
    """Derive effort-native MJCF without changing the canonical export.

    The composed model intentionally contains MIT-style affine actuators for
    the legacy runtime. mujoco_ros2_control's effort interface requires a
    direct actuator, so the ROS copy replaces only the six actuator elements
    with unit-gain motors. Loop constraints, sensors, inertias and contacts are
    unchanged.

    Keyframe `ctrl` is zeroed for the same reason: those numbers are position
    setpoints for the affine actuators, and would become torques here.

    The six actuated hinges are additionally re-parametrised into the logical
    convention of JOINT_FRAMES: a negative sign flips the hinge axis and the
    zero becomes the joint's `ref`, so mujoco_ros2_control reports and
    commands the same angles the calibrated physical drives do. MuJoCo applies
    the rotation (qpos - ref) about the axis, so with the sign folded into the
    axis a keyframe value q maps to sign * (q - zero) and `ref` is -zero for
    an unchanged axis or +zero for a flipped one. Body poses are unchanged.
    """
    robot = ET.parse(source / "robot_full_floating.xml").getroot()
    actuator = robot.find("./actuator")
    if actuator is None:
        raise ValueError("canonical MJCF has no actuator block")
    for index, old in enumerate(list(actuator)):
        name = old.attrib.get("name")
        joint = old.attrib.get("joint")
        if name not in STABLE_JOINTS or joint != name:
            raise ValueError(f"unexpected actuator mapping: {old.attrib}")
        actuator.remove(old)
        actuator.insert(index, ET.Element(
            "motor", {"name": name, "joint": joint, "forcerange": "-10 10"}
        ))
    for key in robot.findall("./keyframe/key"):
        if "ctrl" in key.attrib:
            key.set("ctrl", " ".join(["0"] * len(STABLE_JOINTS)))

    option = robot.find("./option")
    if option is None:
        option = ET.Element("option")
        robot.insert(0, option)
    option.set("timestep", f"{PHYSICS_TIMESTEP_S:.12g}")

    _apply_joint_frames(robot)
    apply_identified(robot, load_identified())

    scene = ET.parse(source / "scene_floating.xml").getroot()
    include = scene.find("./include")
    if include is None:
        raise ValueError("canonical scene has no robot include")
    include.set("file", "robot_ros2.xml")
    return {
        "robot_ros2.xml": _serialized(robot),
        "scene_ros2.xml": _serialized(scene),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("sync", "check"))
    parser.add_argument(
        "--source",
        type=Path,
        default=DEFAULT_SOURCE,
        help=f"Onshape-export model directory (default: {DEFAULT_SOURCE})",
    )
    args = parser.parse_args()
    source = args.source.resolve()
    if args.command == "sync":
        sync(source)
        return 0
    return 0 if check(source) else 1


if __name__ == "__main__":
    raise SystemExit(main())
