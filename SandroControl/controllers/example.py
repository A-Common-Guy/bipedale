"""Minimal low-level custom controller example."""
from __future__ import annotations

from api import MitCommand, RobotState, UserTarget


class ExampleController:
    """Hold the four leg joints and command zero wheel torque."""

    def __init__(self, cfg: dict):
        self.motors = cfg["robot"]["motors"]
        self.hold_positions: dict[str, float] = {}

    def reset(self, state: RobotState) -> None:
        self.hold_positions = {
            name: state.motors[name].position
            for name, item in self.motors.items()
            if item["kind"] == "leg"
        }

    def update(
        self, state: RobotState, target: UserTarget, dt: float
    ) -> dict[str, MitCommand]:
        commands = {}
        for name, item in self.motors.items():
            if item["kind"] == "leg":
                commands[name] = MitCommand(
                    position=self.hold_positions[name],
                    kp=2.0,
                    kd=0.2,
                )
            else:
                commands[name] = MitCommand(kp=0.0, kd=0.1, torque=0.0)
        return commands
