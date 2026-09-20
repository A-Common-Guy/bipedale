"""Map high-level Cartesian/wheel commands onto the six MIT actuators."""
from __future__ import annotations

import math

from kinematics import forward, inverse
from models import LegCommand, MitCommand, RobotCommand, RobotState


class CommandMapper:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.motor_cfg = cfg["robot"]["motors"]
        self.legs_cfg = cfg["robot"]["legs"]
        self._joint_setpoints: dict[str, float] = {}

    def reset(self, state: RobotState) -> None:
        self._joint_setpoints = {
            name: motor.position
            for name, motor in state.motors.items()
            if self.motor_cfg[name]["kind"] == "leg"
        }

    def map(
        self, request: RobotCommand, state: RobotState, dt: float
    ) -> tuple[dict[str, MitCommand], dict[str, LegCommand]]:
        unknown_legs = set(request.legs) - set(self.legs_cfg)
        unknown_wheels = set(request.wheels) - {
            name
            for name, item in self.motor_cfg.items()
            if item["kind"] == "wheel"
        }
        if unknown_legs or unknown_wheels:
            raise ValueError(
                f"unknown mapped commands: legs={sorted(unknown_legs)}, "
                f"wheels={sorted(unknown_wheels)}"
            )
        commands: dict[str, MitCommand] = {}
        mapped_legs: dict[str, LegCommand] = {}
        for side, geometry in self.legs_cfg.items():
            target = request.legs.get(side)
            if target is None:
                measured = state.legs.get(side)
                if measured is None or not measured.reachable:
                    raise RuntimeError(f"{side} Cartesian state is unavailable")
                target = LegCommand(x=measured.x, y=measured.y)
            self._map_leg(side, geometry, target, state, dt, commands)
            mapped_position = forward(
                commands[f"{side}_inner"].position,
                commands[f"{side}_outer"].position,
                float(geometry["base"]),
                float(geometry["upper"]),
                float(geometry["lower"]),
            )
            if mapped_position is None:
                raise RuntimeError(f"{side} post-mapper position is invalid")
            mapped_legs[side] = LegCommand(
                x=mapped_position[0],
                y=mapped_position[1],
                kp=commands[f"{side}_inner"].kp,
                kd=commands[f"{side}_inner"].kd,
            )

        control = self.cfg["control"]
        for name, item in self.motor_cfg.items():
            if item["kind"] != "wheel":
                continue
            target = request.wheels.get(name)
            torque = float(target.torque) if target else 0.0
            velocity = float(target.velocity) if target else 0.0
            kd = (
                float(target.kd)
                if target and target.kd is not None
                else float(control["wheel_kd"])
            )
            if not all(math.isfinite(value) for value in (torque, velocity, kd)):
                raise ValueError(f"{name} mapped command must be finite")
            limit = float(control["wheel_torque_limit"])
            commands[name] = MitCommand(
                position=0.0,
                velocity=velocity,
                kp=0.0,
                kd=kd,
                torque=min(max(torque, -limit), limit),
            )
        return commands, mapped_legs

    def _map_leg(
        self,
        side: str,
        geometry: dict,
        target: LegCommand,
        state: RobotState,
        dt: float,
        commands: dict[str, MitCommand],
    ) -> None:
        values = (float(target.x), float(target.y))
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"{side} Cartesian command must be finite")
        angles = inverse(
            *values,
            float(geometry["base"]),
            float(geometry["upper"]),
            float(geometry["lower"]),
        )
        if angles is None:
            raise ValueError(
                f"{side} Cartesian command ({target.x:.3f}, {target.y:.3f}) "
                "is outside the workspace"
            )
        control = self.cfg["control"]
        kp = float(target.kp) if target.kp is not None else float(control["leg_kp"])
        kd = float(target.kd) if target.kd is not None else float(control["leg_kd"])
        step = float(control["target_rate_rad_s"]) * dt
        lo, hi = float(control["joint_min_rad"]), float(control["joint_max_rad"])
        torque_limit = float(control["leg_torque_limit"])
        for suffix, goal in zip(("inner", "outer"), angles):
            name = f"{side}_{suffix}"
            measured = state.motors[name]
            current = self._joint_setpoints.get(name, measured.position)
            goal = min(max(goal, lo), hi)
            current += min(max(goal - current, -step), step)
            self._joint_setpoints[name] = current
            command_position = current
            if kp > 1e-9:
                raw = kp * (command_position - measured.position) - kd * measured.velocity
                bounded = min(max(raw, -torque_limit), torque_limit)
                command_position = measured.position + (
                    bounded + kd * measured.velocity
                ) / kp
            commands[name] = MitCommand(
                position=command_position,
                velocity=0.0,
                kp=kp,
                kd=kd,
                torque=0.0,
            )
