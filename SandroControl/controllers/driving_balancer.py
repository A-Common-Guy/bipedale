"""Cascaded forward-velocity and yaw controller built on pitch balance."""
from __future__ import annotations

import math

from api import RobotCommand, RobotState, UserTarget, WheelCommand
from controllers.two_wheel_balancer import TwoWheelBalancer


class DrivingBalancer(TwoWheelBalancer):
    """Balance, track forward speed, hold position at rest, and control yaw."""

    def __init__(self, cfg: dict):
        super().__init__(cfg)
        self.drive = cfg["drive"]
        self.velocity_command = 0.0
        self.velocity_target = 0.0
        self.forward_velocity = 0.0
        self.position = 0.0
        self.position_reference = 0.0
        self.velocity_integral = 0.0
        self.pitch_offset = 0.0
        self.yaw_rate = 0.0
        self.yaw_torque = 0.0
        self._dynamic_pitch_target = float(self.balance["pitch_target_rad"])

    def reset(self, state: RobotState) -> None:
        super().reset(state)
        self.velocity_command = 0.0
        self.velocity_target = 0.0
        self.velocity_integral = 0.0
        self.pitch_offset = 0.0
        self.yaw_torque = 0.0
        self._update_wheel_state(state)
        self.position_reference = self.position
        self._dynamic_pitch_target = float(self.balance["pitch_target_rad"])

    def update(
        self, state: RobotState, target: UserTarget, dt: float
    ) -> RobotCommand:
        self._update_wheel_state(state)
        requested_velocity = min(
            max(
                float(target.forward_velocity_m_s),
                -float(self.drive["max_forward_velocity_m_s"]),
            ),
            float(self.drive["max_forward_velocity_m_s"]),
        )
        velocity_step = float(self.drive["forward_accel_limit_m_s2"]) * dt
        self.velocity_command += min(
            max(requested_velocity - self.velocity_command, -velocity_step),
            velocity_step,
        )

        deadband = float(self.drive["command_deadband_m_s"])
        if abs(requested_velocity) > deadband or abs(self.velocity_command) > deadband:
            self.position_reference = self.position
            self.velocity_target = self.velocity_command
        else:
            correction = float(self.drive["position_hold_kp_s"]) * (
                self.position_reference - self.position
            )
            hold_limit = float(self.drive["position_hold_velocity_limit_m_s"])
            self.velocity_target = min(max(correction, -hold_limit), hold_limit)

        velocity_error = self.velocity_target - self.forward_velocity
        ki = float(self.drive["velocity_ki_pitch"])
        # Bound the integral by the pitch offset it is allowed to contribute, so
        # the budget stays meaningful when ki changes.
        integral_bound = (
            float(self.drive["velocity_integral_limit_rad"]) / ki if ki > 0.0 else 0.0
        )
        if state.imu.online and not self.fallen:
            self.velocity_integral += velocity_error * dt
            self.velocity_integral = min(
                max(self.velocity_integral, -integral_bound), integral_bound
            )
        pitch_offset = (
            float(self.drive["velocity_kp_pitch"]) * velocity_error
            + ki * self.velocity_integral
        )
        pitch_limit = float(self.drive["max_pitch_offset_rad"])
        self.pitch_offset = min(max(pitch_offset, -pitch_limit), pitch_limit)
        if self.pitch_offset != pitch_offset and state.imu.online and not self.fallen:
            # Back-calculate so the integral never holds authority the output
            # clamp is already discarding.
            if ki > 0.0:
                self.velocity_integral += (self.pitch_offset - pitch_offset) / ki
                self.velocity_integral = min(
                    max(self.velocity_integral, -integral_bound), integral_bound
                )
        self._dynamic_pitch_target = float(self.balance["pitch_target_rad"]) + (
            float(self.drive["velocity_to_pitch_sign"]) * self.pitch_offset
        )

        command = super().update(state, target, dt)
        requested_yaw = min(
            max(
                float(target.yaw_rate_rad_s),
                -float(self.drive["max_yaw_rate_rad_s"]),
            ),
            float(self.drive["max_yaw_rate_rad_s"]),
        )
        yaw_goal = float(self.drive["yaw_torque_sign"]) * float(
            self.drive["yaw_kp_torque"]
        ) * (requested_yaw - self.yaw_rate)
        yaw_limit = float(self.drive["yaw_torque_limit_nm"])
        yaw_goal = min(max(yaw_goal, -yaw_limit), yaw_limit) * self.ramp
        if self.fallen or self.inhibit_reason == "IMU offline or stale":
            yaw_goal = 0.0
        yaw_step = float(self.drive["yaw_torque_rate_nm_s"]) * dt
        self.yaw_torque += min(
            max(yaw_goal - self.yaw_torque, -yaw_step), yaw_step
        )

        balance_torque = self.last_torque
        total_limit = min(
            float(self.balance["torque_limit_nm"]),
            float(self.cfg["control"]["wheel_torque_limit"]),
        )
        available = max(0.0, total_limit - abs(balance_torque))
        differential = min(max(self.yaw_torque, -available), available)
        damping = float(self.balance["wheel_damping"])
        # Reference the damping to the commanded yaw only. A differential wheel
        # command is free: the two reaction torques cancel at the chassis, so
        # it disturbs pitch not at all. The common-mode term stays referenced
        # to zero, because driving a wheeled inverted pendulum forward needs
        # the wheels to roll backward first to tip the body into the turn of
        # travel; forward authority belongs to the lean, not to the wheels.
        radius = float(self.drive["wheel_radius_m"])
        track = float(self.drive["track_width_m"])
        yaw_rate = requested_yaw * track / (2.0 * radius)
        command.wheels = {
            "right_wheel": WheelCommand(
                torque=balance_torque + differential,
                velocity=yaw_rate,
                kd=damping,
            ),
            "left_wheel": WheelCommand(
                torque=balance_torque - differential,
                velocity=-yaw_rate,
                kd=damping,
            ),
        }
        return command

    def _pitch_target(self) -> float:
        self.active_pitch_target = self._dynamic_pitch_target
        return self.active_pitch_target

    def _update_wheel_state(self, state: RobotState) -> None:
        right = state.motors["right_wheel"]
        left = state.motors["left_wheel"]
        radius = float(self.drive["wheel_radius_m"])
        track = float(self.drive["track_width_m"])
        # Wheel encoders turn relative to the chassis, so ground speed needs the
        # body pitch rate added back. Without it, pitch motion aliases into the
        # velocity loop and cancels part of the inner loop's pitch damping.
        pitch_rate = float(state.imu.gyro[1])
        if not math.isfinite(pitch_rate):
            pitch_rate = 0.0
        self.position = 0.5 * (right.position + left.position) * radius
        self.forward_velocity = (
            0.5 * (right.velocity + left.velocity) + pitch_rate
        ) * radius
        self.yaw_rate = (right.velocity - left.velocity) * radius / track
        if not all(
            math.isfinite(value)
            for value in (self.position, self.forward_velocity, self.yaw_rate)
        ):
            raise ValueError("non-finite wheel odometry")

    def status(self) -> dict:
        result = super().status()
        result.update(
            forward_velocity_m_s=self.forward_velocity,
            velocity_command_m_s=self.velocity_command,
            velocity_target_m_s=self.velocity_target,
            position_m=self.position,
            position_reference_m=self.position_reference,
            pitch_offset_rad=self.pitch_offset,
            yaw_rate_rad_s=self.yaw_rate,
            yaw_torque_nm=self.yaw_torque,
        )
        return result
