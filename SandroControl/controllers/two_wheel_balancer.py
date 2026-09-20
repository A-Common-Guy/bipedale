"""Conservative torque-mode station-keeping controller."""
from __future__ import annotations

import math

from api import LegCommand, RobotCommand, RobotState, UserTarget, WheelCommand


def pitch_from_quaternion(quaternion) -> float:
    """Gravity-relative axle pitch, independent of world heading.

    The quaternion rotates body coordinates into the world frame. Using the
    world-Z components of the body X/Z axes keeps this estimate invariant to
    yaw, unlike reading the world-X component of body Z.
    """
    w, x, y, z = (float(value) for value in quaternion)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1e-9:
        raise ValueError("invalid zero-norm IMU quaternion")
    w, x, y, z = (value / norm for value in (w, x, y, z))
    forward_component = 2.0 * (w * y - x * z)
    vertical_component = 1.0 - 2.0 * (x * x + y * y)
    return math.atan2(forward_component, vertical_component)


class TwoWheelBalancer:
    """Hold equal leg height and balance pitch with equal logical wheel torque."""

    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.balance = cfg["balance"]
        self.legs_cfg = cfg["robot"]["legs"]
        self.wheel_names = tuple(
            name
            for name, item in cfg["robot"]["motors"].items()
            if item["kind"] == "wheel"
        )
        self.height = float(self.balance["leg_height_m"])
        self.elapsed = 0.0
        self.last_torque = 0.0
        self.fallen = False
        self.pitch = 0.0
        self.pitch_rate = 0.0
        self.raw_torque = 0.0
        self.ramp = 0.0
        self.inhibit_reason = "not enabled"
        self.active_pitch_target = float(self.balance["pitch_target_rad"])

    def reset(self, state: RobotState) -> None:
        measured_heights = [
            leg.y for leg in state.legs.values() if leg.reachable
        ]
        self.height = (
            sum(measured_heights) / len(measured_heights)
            if measured_heights
            else float(self.balance["leg_height_m"])
        )
        self.elapsed = 0.0
        self.last_torque = 0.0
        self.fallen = False
        self.pitch = 0.0
        self.pitch_rate = 0.0
        self.raw_torque = 0.0
        self.ramp = 0.0
        self.inhibit_reason = "settling"
        self.active_pitch_target = float(self.balance["pitch_target_rad"])

    def update(
        self, state: RobotState, target: UserTarget, dt: float
    ) -> RobotCommand:
        del target
        height_target = float(self.balance["leg_height_m"])
        height_step = float(self.balance["leg_height_rate_m_s"]) * dt
        self.height += min(max(height_target - self.height, -height_step), height_step)
        legs = {
            side: LegCommand(
                x=float(geometry["base"]) / 2.0,
                y=self.height,
                kp=float(self.balance["leg_kp"]),
                kd=float(self.balance["leg_kd"]),
            )
            for side, geometry in self.legs_cfg.items()
        }

        torque = self._balance_torque(state, dt)
        wheels = {
            name: WheelCommand(
                torque=torque,
                velocity=0.0,
                kd=float(self.balance["wheel_damping"]),
            )
            for name in self.wheel_names
        }
        return RobotCommand(legs=legs, wheels=wheels)

    def _balance_torque(self, state: RobotState, dt: float) -> float:
        imu = state.imu
        values = (*imu.quaternion, *imu.gyro)
        if (
            not imu.online
            or imu.age_s > float(self.balance["imu_max_age_s"])
            or not all(math.isfinite(float(value)) for value in values)
        ):
            self.last_torque = 0.0
            self.raw_torque = 0.0
            self.inhibit_reason = "IMU offline or stale"
            return 0.0
        pitch = pitch_from_quaternion(imu.quaternion)
        pitch_rate = float(imu.gyro[1])
        self.pitch = pitch
        self.pitch_rate = pitch_rate
        if abs(pitch) > float(self.balance["fall_angle_rad"]):
            self.fallen = True
        if self.fallen:
            self.last_torque = 0.0
            self.raw_torque = 0.0
            self.inhibit_reason = "fall latch"
            return 0.0

        self.elapsed += dt
        settle = float(self.balance["settle_s"])
        ramp_time = max(float(self.balance["ramp_s"]), 1e-9)
        ramp = min(max((self.elapsed - settle) / ramp_time, 0.0), 1.0)
        self.ramp = ramp
        raw = float(self.balance["torque_sign"]) * (
            float(self.balance["pitch_kp"])
            * (pitch - self._pitch_target())
            + float(self.balance["pitch_kd"]) * pitch_rate
        )
        self.raw_torque = raw
        limit = min(
            float(self.balance["torque_limit_nm"]),
            float(self.cfg["control"]["wheel_torque_limit"]),
        )
        target_torque = ramp * min(max(raw, -limit), limit)
        max_step = float(self.balance["torque_rate_nm_s"]) * dt
        self.last_torque += min(
            max(target_torque - self.last_torque, -max_step), max_step
        )
        self.inhibit_reason = "settling/ramping" if ramp < 1.0 else ""
        return self.last_torque

    def _pitch_target(self) -> float:
        self.active_pitch_target = float(self.balance["pitch_target_rad"])
        return self.active_pitch_target

    def status(self) -> dict:
        """Read-only diagnostics for tuning through the dashboard."""
        return {
            "pitch_rad": self.pitch,
            "pitch_rate_rad_s": self.pitch_rate,
            "pitch_target_rad": self.active_pitch_target,
            "raw_torque_nm": self.raw_torque,
            "command_torque_nm": self.last_torque,
            "ramp": self.ramp,
            "fallen": self.fallen,
            "inhibit_reason": self.inhibit_reason,
        }
