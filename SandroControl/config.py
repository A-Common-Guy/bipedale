"""Configuration loading with paths resolved relative to the YAML file."""
from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import yaml

from kinematics import inverse


def load_config(path: str | Path) -> dict[str, Any]:
    path = Path(path).expanduser().resolve()
    with path.open(encoding="utf-8") as stream:
        cfg = yaml.safe_load(stream)
    if not isinstance(cfg, dict):
        raise ValueError(f"{path}: expected a YAML mapping")

    base = path.parent
    cfg["_path"] = path
    cfg["can"]["gim6010_path"] = _resolve(base, cfg["can"]["gim6010_path"])
    cfg["mujoco"]["model"] = _resolve(base, cfg["mujoco"]["model"])
    cfg["mujoco"]["identified_params"] = _resolve(
        base, cfg["mujoco"]["identified_params"]
    )
    cfg["sysid"]["output_dir"] = _resolve(base, cfg["sysid"]["output_dir"])

    mode = cfg["runtime"]["mode"]
    if mode not in {"simulation", "physical", "both"}:
        raise ValueError("runtime.mode must be simulation, physical, or both")
    if float(cfg["runtime"]["control_hz"]) <= 0:
        raise ValueError("runtime.control_hz must be positive")
    if int(cfg["imu"]["baudrate"]) != 3_000_000:
        raise ValueError("standard BNO085 UART-SHTP requires 3000000 baud")
    if int(cfg["imu"]["report_interval_us"]) <= 0:
        raise ValueError("imu.report_interval_us must be positive")
    if int(cfg["sysid"]["recorder_queue_size"]) < 128:
        raise ValueError("sysid.recorder_queue_size must be at least 128")

    hard = float(cfg["control"]["hard_torque_limit_nm"])
    for key in ("leg_torque_limit", "wheel_torque_limit"):
        if not 0 <= float(cfg["control"][key]) <= hard:
            raise ValueError(f"control.{key} must be between 0 and {hard} Nm")
    for key, ceiling in (("leg_kp", 100.0), ("leg_kd", 10.0), ("wheel_kd", 10.0)):
        if not 0 <= float(cfg["control"][key]) <= ceiling:
            raise ValueError(f"control.{key} must be between 0 and {ceiling}")

    validate_balance(cfg)
    validate_drive(cfg)
    return cfg


def validate_balance(cfg: dict[str, Any]) -> None:
    """Validate balance parameters for startup and live dashboard updates."""
    balance = cfg["balance"]
    positive = (
        "leg_height_m",
        "leg_height_rate_m_s",
        "imu_max_age_s",
        "ramp_s",
        "fall_angle_rad",
        "torque_rate_nm_s",
    )
    for key in positive:
        value = float(balance[key])
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"balance.{key} must be finite and positive")
    bounded = {
        "leg_kp": 100.0,
        "leg_kd": 10.0,
        "pitch_kp": 20.0,
        "pitch_kd": 5.0,
        "wheel_damping": 10.0,
        "torque_limit_nm": float(cfg["control"]["wheel_torque_limit"]),
        "settle_s": 10.0,
    }
    for key, ceiling in bounded.items():
        value = float(balance[key])
        if not math.isfinite(value) or not 0 <= value <= ceiling:
            raise ValueError(f"balance.{key} must be between 0 and {ceiling}")
    if float(balance["torque_sign"]) not in {-1.0, 1.0}:
        raise ValueError("balance.torque_sign must be -1 or 1")
    pitch_target = float(balance["pitch_target_rad"])
    if not math.isfinite(pitch_target) or abs(pitch_target) > math.pi:
        raise ValueError("balance.pitch_target_rad must be within ±pi")
    for side, geometry in cfg["robot"]["legs"].items():
        if inverse(
            float(geometry["base"]) / 2.0,
            float(balance["leg_height_m"]),
            float(geometry["base"]),
            float(geometry["upper"]),
            float(geometry["lower"]),
        ) is None:
            raise ValueError(f"balance.leg_height_m is unreachable for {side} leg")


def validate_drive(cfg: dict[str, Any]) -> None:
    """Validate cascaded driving-controller geometry, gains and limits."""
    drive = cfg["drive"]
    positive = (
        "wheel_radius_m",
        "track_width_m",
        "max_forward_velocity_m_s",
        "max_yaw_rate_rad_s",
        "forward_accel_limit_m_s2",
        "max_pitch_offset_rad",
        "position_hold_velocity_limit_m_s",
        "yaw_torque_rate_nm_s",
        "command_timeout_s",
    )
    non_negative = (
        "velocity_kp_pitch",
        "velocity_ki_pitch",
        "velocity_integral_limit_rad",
        "position_hold_kp_s",
        "command_deadband_m_s",
        "yaw_kp_torque",
        "yaw_torque_limit_nm",
    )
    for key in positive:
        value = float(drive[key])
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"drive.{key} must be finite and positive")
    for key in non_negative:
        value = float(drive[key])
        if not math.isfinite(value) or value < 0:
            raise ValueError(f"drive.{key} must be finite and non-negative")
    for key in ("velocity_to_pitch_sign", "yaw_torque_sign"):
        if float(drive[key]) not in {-1.0, 1.0}:
            raise ValueError(f"drive.{key} must be -1 or 1")
    if float(drive["max_pitch_offset_rad"]) >= float(
        cfg["balance"]["fall_angle_rad"]
    ):
        raise ValueError("drive.max_pitch_offset_rad must be below fall angle")
    if float(drive["yaw_torque_limit_nm"]) > float(
        cfg["control"]["wheel_torque_limit"]
    ):
        raise ValueError("drive.yaw_torque_limit_nm exceeds wheel torque limit")


def _resolve(base: Path, value: str | Path) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (base / path).resolve()
