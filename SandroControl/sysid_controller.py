"""Deterministic, bounded excitation controller for lifted system ID."""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from models import MitCommand, RobotState


PRESETS = {
    "zero": {
        "label": "Zero-amplitude recorder check",
        "kinds": ("leg", "wheel"),
        "max_amplitude": 0.0,
        "unit": "",
    },
    "wheel_steps": {
        "label": "Lifted wheel torque steps",
        "kinds": ("wheel",),
        "max_amplitude": 0.2,
        "unit": "Nm",
    },
    "wheel_coast": {
        "label": "Lifted wheel pulse and coast",
        "kinds": ("wheel",),
        "max_amplitude": 0.2,
        "unit": "Nm",
    },
    "leg_position_steps": {
        "label": "Leg position multisteps",
        "kinds": ("leg",),
        "max_amplitude": 0.05,
        "unit": "rad",
    },
    "leg_torque_steps": {
        "label": "Leg torque perturbation",
        "kinds": ("leg",),
        "max_amplitude": 0.3,
        "unit": "Nm",
    },
}


@dataclass
class TrialStatus:
    state: str = "idle"
    segment: str = ""
    axis: str = ""
    preset: str = ""
    amplitude: float = 0.0
    duration_s: float = 0.0
    elapsed_s: float = 0.0
    progress: float = 0.0
    reason: str = ""
    run_id: str = ""
    hold_positions: dict[str, float] = field(default_factory=dict)


class SysIdController:
    settle_s = 1.0

    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.motor_cfg = cfg["robot"]["motors"]
        self.trial = TrialStatus()

    @staticmethod
    def presets() -> dict:
        return PRESETS

    @property
    def running(self) -> bool:
        return self.trial.state in {"settle_before", "excite", "settle_after"}

    def arm(
        self,
        axis: str,
        preset: str,
        amplitude: float,
        duration_s: float,
        lifted_confirmed: bool,
    ) -> dict:
        if self.running:
            raise RuntimeError("abort the active trial before arming another")
        if not lifted_confirmed:
            raise ValueError("confirm that the physical robot is lifted and secured")
        if axis not in self.motor_cfg:
            raise ValueError(f"unknown sys-ID axis {axis!r}")
        if preset not in PRESETS:
            raise ValueError(f"unknown sys-ID preset {preset!r}")
        kind = self.motor_cfg[axis]["kind"]
        definition = PRESETS[preset]
        if kind not in definition["kinds"]:
            raise ValueError(f"{preset} is not valid for a {kind} actuator")
        amplitude = float(amplitude)
        duration_s = float(duration_s)
        if not math.isfinite(amplitude) or amplitude < 0:
            raise ValueError("amplitude must be finite and non-negative")
        if amplitude > float(definition["max_amplitude"]):
            raise ValueError(
                f"{preset} amplitude exceeds {definition['max_amplitude']} "
                f"{definition['unit']}"
            )
        if not math.isfinite(duration_s) or not 2.0 <= duration_s <= 60.0:
            raise ValueError("duration must be between 2 and 60 seconds")
        self.trial = TrialStatus(
            state="armed",
            segment="armed",
            axis=axis,
            preset=preset,
            amplitude=amplitude,
            duration_s=duration_s,
        )
        return self.status()

    def start(self, state: RobotState, run_id: str) -> None:
        if self.trial.state != "armed":
            raise RuntimeError("arm a sys-ID trial before starting it")
        if set(state.motors) != set(self.motor_cfg):
            raise RuntimeError("complete motor state is required for sys-ID")
        self.trial.hold_positions = {
            name: motor.position for name, motor in state.motors.items()
        }
        self.trial.state = "settle_before"
        self.trial.segment = "settle_before"
        self.trial.elapsed_s = 0.0
        self.trial.progress = 0.0
        self.trial.run_id = run_id

    def abort(self, reason: str) -> None:
        if self.trial.state in {"idle", "completed", "aborted"}:
            return
        self.trial.state = "aborted"
        self.trial.segment = "aborted"
        self.trial.reason = reason

    def update(self, state: RobotState, dt: float) -> dict[str, MitCommand]:
        if not self.running:
            raise RuntimeError("sys-ID controller updated without a running trial")
        self.trial.elapsed_s += dt
        if (
            self.trial.state == "settle_before"
            and self.trial.elapsed_s >= self.settle_s
        ):
            self.trial.state = "excite"
            self.trial.segment = "excite"
            self.trial.elapsed_s = 0.0
        elif (
            self.trial.state == "excite"
            and self.trial.elapsed_s >= self.trial.duration_s
        ):
            self.trial.state = "settle_after"
            self.trial.segment = "settle_after"
            self.trial.elapsed_s = 0.0
        elif (
            self.trial.state == "settle_after"
            and self.trial.elapsed_s >= self.settle_s
        ):
            self.trial.state = "completed"
            self.trial.segment = "completed"
            self.trial.progress = 1.0
            commands = self._hold_commands()
            self._limit_leg_effort(commands, state)
            return commands

        total = 2 * self.settle_s + self.trial.duration_s
        if self.trial.state == "settle_before":
            overall = self.trial.elapsed_s
        elif self.trial.state == "excite":
            overall = self.settle_s + self.trial.elapsed_s
        else:
            overall = self.settle_s + self.trial.duration_s + self.trial.elapsed_s
        self.trial.progress = min(1.0, overall / total)
        commands = self._hold_commands()
        if self.trial.state == "excite":
            self._apply_excitation(commands)
        self._limit_leg_effort(commands, state)
        return commands

    def _hold_commands(self) -> dict[str, MitCommand]:
        commands = {}
        for name, item in self.motor_cfg.items():
            if item["kind"] == "leg":
                commands[name] = MitCommand(
                    position=self.trial.hold_positions.get(name, 0.0),
                    kp=min(4.0, float(self.cfg["control"]["leg_kp"])),
                    kd=min(0.4, float(self.cfg["control"]["leg_kd"])),
                )
            else:
                commands[name] = MitCommand(
                    position=0.0,
                    kp=0.0,
                    kd=min(0.1, float(self.cfg["control"]["wheel_kd"])),
                )
        return commands

    def _apply_excitation(self, commands: dict[str, MitCommand]) -> None:
        axis = self.trial.axis
        amplitude = self.trial.amplitude
        phase = self.trial.elapsed_s / self.trial.duration_s
        if self.trial.preset == "zero":
            self.trial.segment = "zero"
            return
        if self.trial.preset == "wheel_coast":
            commands[axis].torque = amplitude if phase < 0.25 else 0.0
            self.trial.segment = "pulse" if phase < 0.25 else "coast"
            return
        step = min(5, int(phase * 6.0))
        level = (0.0, 1.0, -1.0, 0.5, -0.5, 0.0)[step]
        self.trial.segment = f"step_{step}"
        if self.trial.preset == "wheel_steps":
            commands[axis].torque = level * amplitude
        elif self.trial.preset == "leg_position_steps":
            commands[axis].position += level * amplitude
        elif self.trial.preset == "leg_torque_steps":
            commands[axis].torque = level * amplitude

    def _limit_leg_effort(
        self, commands: dict[str, MitCommand], state: RobotState
    ) -> None:
        limit = float(self.cfg["control"]["leg_torque_limit"])
        for name, command in commands.items():
            if self.motor_cfg[name]["kind"] != "leg" or command.kp <= 1e-9:
                continue
            motor = state.motors[name]
            raw = (
                command.kp * (command.position - motor.position)
                - command.kd * motor.velocity
                + command.torque
            )
            bounded = min(max(raw, -limit), limit)
            command.position = motor.position + (
                bounded + command.kd * motor.velocity - command.torque
            ) / command.kp

    def status(self) -> dict:
        result = {
            key: value
            for key, value in vars(self.trial).items()
            if key != "hold_positions"
        }
        result["running"] = self.running
        return result
