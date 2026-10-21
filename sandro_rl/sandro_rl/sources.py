"""The robot model and the ROS stack's parameters, read from the repository.

Nothing here is copied: the generated MJCF, the controller configuration,
the per-robot overrides, the identified simulation parameters and the
latency defaults of the launch file are read from where the ROS stack reads
them, every time. `fingerprint()` hashes all of them so a checkpoint knows
which model it was trained on.
"""

from __future__ import annotations

import ast
import hashlib
import json
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[2]
SRC = REPO / "ros2_ws" / "src"
DESCRIPTION = SRC / "sandro_description"
SCENE = DESCRIPTION / "mjcf" / "scene_ros2.xml"
ROBOT = DESCRIPTION / "mjcf" / "robot_ros2.xml"
IDENTIFIED = DESCRIPTION / "config" / "sim_identified.yaml"
CONTROLLERS = SRC / "sandro_bringup" / "config" / "controllers.yaml"
ROBOT_OVERRIDES = SRC / "sandro_bringup" / "config" / "sandro_real_overrides.yaml"
LAUNCH = SRC / "sandro_bringup" / "launch" / "sandro.launch.py"
CONTROLLER = "whole_body_controller"

# ros2_control description order (sandro.ros2_control.xacro), which is also
# the whole-body controller's joint order and the policy's action order.
JOINTS = ("right_motor_inner", "right_motor_outer", "right_wheel",
          "left_motor_inner", "left_motor_outer", "left_wheel")
LEG_JOINTS = ("right_motor_inner", "right_motor_outer", "left_motor_inner", "left_motor_outer")
WHEEL_JOINTS = ("right_wheel", "left_wheel")
IMU_SENSORS = ("imu_quat", "imu_gyro", "imu_accel")
TYRE_SUFFIX = "_tire_cyl"


def _merge(base: dict, extra: dict) -> dict:
    out = dict(base)
    for key, value in extra.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _merge(out[key], value)
        else:
            out[key] = value
    return out


def controller_parameters(overrides: bool = True, controller: str = CONTROLLER) -> dict:
    """controllers.yaml, then sandro_real_overrides.yaml, as ROS layers them."""
    files = [CONTROLLERS] + ([ROBOT_OVERRIDES] if overrides and ROBOT_OVERRIDES.is_file() else [])
    values: dict = {}
    for path in files:
        document = yaml.safe_load(path.read_text()) or {}
        section = (document.get(controller) or {}).get("ros__parameters") or {}
        values = _merge(values, section)
    return values


def controller_manager_rate_hz() -> float:
    document = yaml.safe_load(CONTROLLERS.read_text())
    return float(document["controller_manager"]["ros__parameters"]["update_rate"])


def identified() -> dict:
    return yaml.safe_load(IDENTIFIED.read_text()) or {}


def simulated_imu_roll_offset() -> float:
    """Roll of the simulated IMU's mounting (sim_identified.yaml imu_mount).

    This, not the robot overrides' tasks.roll.imu_offset_rad, is the correction
    that levels the simulated body: the two differ whenever the physical IMU
    is re-identified before the simulated mount follows. The robot's value
    reaches the controller through its own rl_* overrides.
    """
    return float((identified().get("imu_mount") or {}).get("roll_offset_rad", 0.0))


def launch_defaults(path: Path = LAUNCH) -> dict[str, str]:
    """DeclareLaunchArgument(name, default_value=...) pairs of the launch file."""
    tree = ast.parse(path.read_text())
    defaults: dict[str, str] = {}
    for node in ast.walk(tree):
        if not (isinstance(node, ast.Call) and getattr(node.func, "id", None) == "DeclareLaunchArgument"):
            continue
        if not node.args or not isinstance(node.args[0], ast.Constant):
            continue
        for keyword in node.keywords:
            if keyword.arg == "default_value" and isinstance(keyword.value, ast.Constant):
                defaults[str(node.args[0].value)] = str(keyword.value.value)
    return defaults


@dataclass(frozen=True)
class Latency:
    """The latency emulation of sandro_sim (LatencySystem, DriveBus), in seconds and Hz."""

    imu_delay_s: float
    imu_rate_hz: float
    joint_delay_s: float
    joint_rate_hz: float
    joint_stagger: bool
    command_delay_s: float

    @classmethod
    def from_launch(cls) -> "Latency":
        d = launch_defaults()
        return cls(
            imu_delay_s=float(d["sim_imu_delay_s"]),
            imu_rate_hz=float(d["sim_imu_rate_hz"]),
            joint_delay_s=float(d["sim_joint_delay_s"]),
            joint_rate_hz=float(d["sim_joint_rate_hz"]),
            joint_stagger=d["sim_joint_stagger"].lower() in ("1", "true", "yes", "on"),
            command_delay_s=float(d["sim_command_delay_s"]),
        )


@dataclass(frozen=True)
class RobotLimits:
    """Limits and rates the ROS stack enforces (controllers.yaml + overrides)."""

    control_rate_hz: float
    leg_torque_nm: float
    wheel_torque_nm: float
    leg_kp_nm_rad: float
    leg_kd_nm_s_rad: float
    height_min_m: float
    height_max_m: float
    height_target_m: float
    max_forward_m_s: float
    max_yaw_rad_s: float
    max_height_rate_m_s: float
    forward_accel_limit_m_s2: float
    yaw_command_time_constant_s: float
    fall_angle_rad: float
    command_timeout_s: float
    imu_timeout_s: float
    wheel_radius_m: float
    track_width_m: float
    leg_geometry: dict = field(default_factory=dict)  # side -> base/upper/lower/forward_sign
    fore_aft_neutral_m: float = 0.045
    neutral_offset_m: float = 0.0
    leg_offset_limit_m: float = 0.025

    @classmethod
    def from_ros(cls, overrides: bool = True) -> "RobotLimits":
        p = controller_parameters(overrides)
        model, tasks, limits = p["model"], p["tasks"], p["limits"]
        drive, safety = p["drive"], p["safety"]
        geometry = {side: {k: float(model[side][k]) for k in ("base_m", "upper_m", "lower_m", "forward_sign")}
                    for side in ("right", "left")}
        return cls(
            control_rate_hz=controller_manager_rate_hz(),
            leg_torque_nm=float(limits["leg_torque_nm"]),
            wheel_torque_nm=float(limits["wheel_torque_nm"]),
            leg_kp_nm_rad=float(tasks["legs"]["drive_kp_nm_rad"]),
            leg_kd_nm_s_rad=float(tasks["legs"]["drive_kd_nm_s_rad"]),
            height_min_m=float(tasks["height"]["min_m"]),
            height_max_m=float(tasks["height"]["max_m"]),
            height_target_m=float(tasks["height"]["target_m"]),
            max_forward_m_s=float(drive["max_forward_velocity_m_s"]),
            max_yaw_rad_s=float(drive["max_yaw_rate_rad_s"]),
            max_height_rate_m_s=float(drive["max_height_rate_m_s"]),
            forward_accel_limit_m_s2=float(drive["forward_accel_limit_m_s2"]),
            yaw_command_time_constant_s=float(drive["yaw_command_time_constant_s"]),
            fall_angle_rad=float(safety["fall_angle_rad"]),
            command_timeout_s=float(safety["command_timeout_s"]),
            imu_timeout_s=float(safety["imu_timeout_s"]),
            wheel_radius_m=float(model["wheel_radius_m"]),
            track_width_m=float(model["track_width_m"]),
            leg_geometry=geometry,
            fore_aft_neutral_m=float(tasks["fore_aft"]["neutral_m"]),
            neutral_offset_m=float(tasks["fore_aft"].get("neutral_offset_m", 0.0)),
            leg_offset_limit_m=float(p["balance"]["leg_offset_limit_m"]),
        )


def mjcf_files(scene: Path = SCENE) -> list[Path]:
    """The scene, every included MJCF and every mesh they reference."""
    files: list[Path] = []
    pending = [scene]
    while pending:
        path = pending.pop()
        files.append(path)
        root = ET.parse(path).getroot()
        meshdir = path.parent
        compiler = root.find("compiler")
        if compiler is not None and "meshdir" in compiler.attrib:
            meshdir = path.parent / compiler.attrib["meshdir"]
        for include in root.iter("include"):
            pending.append(path.parent / include.attrib["file"])
        for mesh in root.iter("mesh"):
            if "file" in mesh.attrib:
                files.append(meshdir / mesh.attrib["file"])
    return sorted(set(files))


def fingerprint(scene: Path = SCENE) -> dict:
    """sha256 of every file that defines the simulated robot and its interface."""
    files = mjcf_files(scene) + [IDENTIFIED, CONTROLLERS, ROBOT_OVERRIDES, LAUNCH]
    per_file = {}
    for path in files:
        if path.is_file():
            per_file[str(path.relative_to(REPO)) if path.is_relative_to(REPO) else str(path)] = \
                hashlib.sha256(path.read_bytes()).hexdigest()
    total = hashlib.sha256(json.dumps(per_file, sort_keys=True).encode()).hexdigest()
    return {"sha256": total, "files": per_file}


def compare_fingerprints(stored: dict, current: dict | None = None) -> list[str]:
    """Files whose hash changed since `stored` was taken (empty: same model)."""
    current = current or fingerprint()
    if stored.get("sha256") == current["sha256"]:
        return []
    old, new = stored.get("files", {}), current["files"]
    return sorted(k for k in set(old) | set(new) if old.get(k) != new.get(k))
