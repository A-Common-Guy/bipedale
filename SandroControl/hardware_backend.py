"""Physical GIM6010 CAN backend. No controller policy lives here."""
from __future__ import annotations

import math
import sys
import threading
import time
from typing import Dict

from models import ImuState, MitCommand, MotorState, RobotState

TWO_PI = 2.0 * math.pi
CAN_BUS_FAILED = 0x100000


class HardwareBackend:
    name = "physical"

    def __init__(self, cfg: dict):
        self.cfg = cfg
        can_cfg = cfg["can"]
        sys.path.insert(0, str(can_cfg["gim6010_path"]))
        try:
            import can
            from gim6010_can import GIM6010Bus
        except ImportError as exc:
            raise RuntimeError(
                f"python-can and gim6010_can.py are required for physical mode"
            ) from exc

        host = can.Bus(
            channel=can_cfg["channel"], interface=can_cfg["interface"]
        )
        endpoints = (
            can_cfg["gim6010_path"].parent
            / "manual"
            / "assets_protocol_manual"
            / "endpoints_v3.json"
        )
        self.bus = GIM6010Bus(
            bus=host, endpoints=endpoints if endpoints.exists() else None
        )
        self.motor_cfg = cfg["robot"]["motors"]
        self.motors = {
            name: self.bus.motor(int(item["node"]))
            for name, item in self.motor_cfg.items()
        }
        self.enabled = False
        self._closed = False
        self._lock = threading.RLock()
        self._last_commands = {name: MitCommand() for name in self.motors}

    def start(self) -> None:
        pass

    def _logical_position(self, name: str) -> float | None:
        state = self.motors[name].state
        if state.pos is None:
            return None
        c = self.motor_cfg[name]
        return (
            float(c["sign"])
            * (state.pos - float(c["offset_rev"]))
            / float(c["ratio"])
            * TWO_PI
        )

    def _logical_velocity(self, name: str) -> float | None:
        velocity = self.motors[name].state.vel
        if velocity is None:
            return None
        c = self.motor_cfg[name]
        return float(c["sign"]) * velocity / float(c["ratio"]) * TWO_PI

    def read(self) -> RobotState:
        motors = {}
        for name, motor in self.motors.items():
            state = motor.state
            position = self._logical_position(name)
            velocity = self._logical_velocity(name)
            age = state.encoder_age()
            error = state.error_summary()
            torque = getattr(state, "mit_torque", None)
            motors[name] = MotorState(
                position=position if position is not None else 0.0,
                velocity=velocity if velocity is not None else 0.0,
                torque=(
                    float(self.motor_cfg[name]["sign"]) * torque
                    if torque is not None
                    else self._last_commands[name].torque
                ),
                iq=state.iq_measured,
                mit_position=state.mit_pos,
                mit_velocity=state.mit_vel,
                mit_torque=state.mit_torque,
                vbus=state.vbus,
                temperature=state.temperature,
                online=min(age, state.heartbeat_age()) < 1.0,
                error=error,
                age_s=age,
            )
        return RobotState(
            motors=motors,
            imu=ImuState(),
            timestamp=time.monotonic(),
            source=self.name,
        )

    def enable(self) -> None:
        with self._lock:
            silent = [
                motor.node_id
                for motor in self.motors.values()
                if min(
                    motor.state.encoder_age(), motor.state.heartbeat_age()
                )
                > 1.0
            ]
            if silent:
                raise RuntimeError(f"CAN nodes are silent: {silent}")
            self._recover_deadman_timeouts()
            failed = [
                self._detailed_error(name, motor)
                for name, motor in self.motors.items()
                if motor.state.any_error
            ]
            if failed:
                raise RuntimeError("CAN drive errors: " + " | ".join(failed))

            can_cfg = self.cfg["can"]
            for motor in self.motors.values():
                motor.set_feedback_rates(
                    encoder=int(can_cfg["encoder_period_ms"]),
                    iq=int(can_cfg["iq_period_ms"]),
                    heartbeat=int(can_cfg["heartbeat_period_ms"]),
                )
                self._load_mit_limits(motor)
                motor.set_controller_mode(3, 9)

            # Reconcile each drive at its measured position with zero gains.
            hold_positions = {}
            for name, motor in self.motors.items():
                logical = self._logical_position(name)
                if logical is None:
                    raise RuntimeError(f"node {motor.node_id}: no encoder state")
                wire_position = self._wire_position(name, logical)
                hold_positions[name] = wire_position
                motor.mit_control(
                    pos=wire_position,
                    vel=0.0,
                    kp=0.0,
                    kd=0.0,
                    torque=0.0,
                    wait_feedback=False,
                )
                motor.set_axis_state(8)

            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline:
                if all(m.state.axis_state == 8 for m in self.motors.values()):
                    break
                time.sleep(0.02)
            else:
                self.disable()
                states = [
                    (
                        f"{name} (node {motor.node_id}): "
                        f"state={motor.state.axis_state_name()}"
                        + (
                            f"; {motor.state.error_summary()}"
                            if motor.state.any_error
                            else ""
                        )
                    )
                    for name, motor in self.motors.items()
                    if motor.state.axis_state != 8
                ]
                raise RuntimeError(
                    "drives did not enter closed loop: " + " | ".join(states)
                )

            # Arm the watchdog only after configuration and closed-loop entry.
            # Keep every drive fed while the verified SDO writes are in progress.
            for motor in self.motors.values():
                motor.set_deadman(int(can_cfg["deadman_ms"]))
                for name, held_motor in self.motors.items():
                    held_motor.mit_control(
                        pos=hold_positions[name],
                        vel=0.0,
                        kp=0.0,
                        kd=0.0,
                        torque=0.0,
                        wait_feedback=False,
                    )
            self.enabled = True

    def sysid_metadata(self) -> dict:
        return {
            name: {
                "node": motor.node_id,
                "mit_limits": {
                    key: float(getattr(motor.mit_limits, key))
                    for key in (
                        "max_pos",
                        "max_vel",
                        "max_kp",
                        "max_kd",
                        "max_torque",
                    )
                },
            }
            for name, motor in self.motors.items()
        }

    def _recover_deadman_timeouts(self) -> None:
        """Clear only the expected watchdog fault from a previous stopped stream."""
        recoverable = [
            motor
            for motor in self.motors.values()
            if motor.state.axis_error == CAN_BUS_FAILED
            and not (
                motor.state.motor_err
                or motor.state.encoder_err
                or motor.state.controller_err
                or motor.state.system_err
            )
        ]
        if not recoverable:
            return
        for motor in recoverable:
            motor.set_deadman(0)
        for motor in recoverable:
            motor.clear_errors()

        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            if all(not motor.state.any_error for motor in recoverable):
                return
            time.sleep(0.01)

    @staticmethod
    def _detailed_error(name: str, motor) -> str:
        """Read and decode active error categories for one failed drive."""
        state = motor.state
        parts = [state.error_summary()]
        categories = (
            ("motor", state.motor_err or bool((state.axis_error or 0) & 0x40)),
            ("encoder", state.encoder_err or bool((state.axis_error or 0) & 0x100)),
            (
                "controller",
                state.controller_err or bool((state.axis_error or 0) & 0x200),
            ),
            ("system", state.system_err),
        )
        for category, active in categories:
            if not active:
                continue
            try:
                raw, decoded = motor.get_error(category, timeout=0.5)
                value = ",".join(decoded) if decoded else f"0x{raw:X}"
            except Exception as exc:
                value = f"unavailable ({exc})"
            parts.append(f"{category}={value}")
        details = "; ".join(part for part in parts if part)
        return f"{name} (node {motor.node_id}): {details}"

    @staticmethod
    def _load_mit_limits(motor) -> None:
        for attr, endpoint in (
            ("max_pos", "mit_max_pos"),
            ("max_vel", "mit_max_vel"),
            ("max_kp", "mit_max_kp"),
            ("max_kd", "mit_max_kd"),
            ("max_torque", "mit_max_torque"),
        ):
            value = float(motor.sdo_read(endpoint))
            if value <= 0:
                raise RuntimeError(
                    f"node {motor.node_id}: invalid {endpoint}={value}"
                )
            setattr(motor.mit_limits, attr, value)

    def _wire_position(self, name: str, logical: float) -> float:
        c = self.motor_cfg[name]
        return (
            float(c["sign"]) * logical
            + float(c["offset_rev"]) / float(c["ratio"]) * TWO_PI
        )

    def write(self, commands: Dict[str, MitCommand]) -> None:
        if not self.enabled:
            return
        with self._lock:
            for name, command in commands.items():
                motor = self.motors[name]
                sign = float(self.motor_cfg[name]["sign"])
                limit = motor.mit_limits
                motor.mit_control(
                    pos=self._wire_position(name, command.position),
                    vel=sign * command.velocity,
                    kp=min(command.kp, limit.max_kp),
                    kd=min(command.kd, limit.max_kd),
                    torque=sign
                    * min(
                        max(command.torque, -limit.max_torque),
                        limit.max_torque,
                    ),
                    wait_feedback=False,
                )
            self._last_commands = dict(commands)

    def disable(self) -> None:
        with self._lock:
            self.enabled = False
            if self._closed:
                return
            for motor in self.motors.values():
                try:
                    motor.mit_control(0, 0, 0, 0, 0, wait_feedback=False)
                except Exception:
                    pass
                try:
                    motor.idle()
                except Exception:
                    pass

    def estop(self) -> None:
        self.enabled = False
        errors = []
        for motor in self.motors.values():
            try:
                motor.estop()
            except Exception as exc:
                errors.append(f"node {motor.node_id}: {exc}")
        if errors:
            raise RuntimeError("; ".join(errors))

    def close(self) -> None:
        # Held across the whole sequence so write() cannot reach a socket that
        # is being torn down; disable() leaves enabled False for later calls.
        with self._lock:
            if self._closed:
                return
            self.disable()
            self._closed = True
            self.bus.close()
