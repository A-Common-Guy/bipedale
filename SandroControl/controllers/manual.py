"""Default dashboard-driven MIT controller."""
from __future__ import annotations

from typing import Dict

from api import MitCommand, RobotState, UserTarget


class ManualMitController:
    """Joint MIT impedance plus pure feed-forward wheel torque."""

    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.leg_names = tuple(
            name
            for name, item in cfg["robot"]["motors"].items()
            if item["kind"] == "leg"
        )
        self.wheel_names = tuple(
            name
            for name, item in cfg["robot"]["motors"].items()
            if item["kind"] == "wheel"
        )
        self.setpoint: Dict[str, float] = {}

    def reset(self, state: RobotState) -> None:
        self.setpoint = {
            name: state.motors[name].position if name in state.motors else 0.0
            for name in self.leg_names
        }

    def update(
        self, state: RobotState, target: UserTarget, dt: float
    ) -> Dict[str, MitCommand]:
        control = self.cfg["control"]
        step = float(control["target_rate_rad_s"]) * dt
        low = float(control["joint_min_rad"])
        high = float(control["joint_max_rad"])
        output: Dict[str, MitCommand] = {}
        for name in self.leg_names:
            measured = state.motors.get(name)
            if name not in self.setpoint:
                self.setpoint[name] = measured.position if measured else 0.0
            goal = min(
                max(target.joint_positions.get(name, self.setpoint[name]), low),
                high,
            )
            self.setpoint[name] += min(
                max(goal - self.setpoint[name], -step), step
            )
            command_position = self.setpoint[name]
            kp = float(control["leg_kp"])
            kd = float(control["leg_kd"])
            torque_limit = float(control["leg_torque_limit"])
            if measured and kp > 1e-9:
                raw = (
                    kp * (command_position - measured.position)
                    - kd * measured.velocity
                )
                bounded = min(max(raw, -torque_limit), torque_limit)
                command_position = measured.position + (
                    bounded + kd * measured.velocity
                ) / kp
            output[name] = MitCommand(
                position=command_position,
                velocity=0.0,
                kp=kp,
                kd=kd,
            )

        for name in self.wheel_names:
            limit = float(control["wheel_torque_limit"])
            output[name] = MitCommand(
                position=0.0,
                velocity=0.0,
                kp=0.0,
                kd=float(control["wheel_kd"]),
                torque=min(
                    max(target.wheel_torques.get(name, 0.0), -limit), limit
                ),
            )
        return output
