"""Single high-rate command authority, independent from the GUI rate."""
from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import asdict
from typing import Dict

from command_mapper import CommandMapper
from config import validate_balance, validate_drive
from controller import ManualMitController
from kinematics import inverse, map_legs
from models import (
    Backend,
    Controller,
    LegCommand,
    MitCommand,
    RobotCommand,
    RobotState,
    UserTarget,
)
from recorder import RunRecorder
from sysid_controller import SysIdController

log = logging.getLogger("sandro.runtime")

JOIN_TIMEOUT_S = 2.0
LOCK_TIMEOUT_S = 1.0


class ControlRuntime(threading.Thread):
    def __init__(
        self,
        cfg: dict,
        backends: list[Backend],
        imu=None,
        controller: Controller | None = None,
    ):
        super().__init__(daemon=True, name="control-loop")
        self.cfg = cfg
        self.backends = backends
        self.imu = imu
        self.controller = controller or ManualMitController(cfg)
        if not callable(getattr(self.controller, "reset", None)) or not callable(
            getattr(self.controller, "update", None)
        ):
            raise TypeError("controller must provide reset(state) and update(state, target, dt)")
        self.sysid = SysIdController(cfg)
        self.command_mapper = CommandMapper(cfg)
        self.recorder = RunRecorder(
            cfg["sysid"]["output_dir"],
            cfg["robot"]["motors"],
            int(cfg["sysid"]["recorder_queue_size"]),
        )
        self.target = UserTarget()
        self.commands: Dict[str, MitCommand] = {}
        self.leg_commands: Dict[str, LegCommand] = {}
        self.states: Dict[str, RobotState] = {}
        self._last_drive_target = 0.0
        self.mode = "idle"
        self.fault_reason = ""
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._stopped = False
        self._closed = False
        self.metrics = {
            "frequency_hz": 0.0,
            "cycle_ms": 0.0,
            "max_cycle_ms": 0.0,
            "overruns": 0,
        }

    @property
    def state_source(self) -> str:
        requested = self.cfg["runtime"]["state_source"]
        if requested in self.states:
            return requested
        return self.backends[0].name

    def selected_state(self) -> RobotState:
        return self.states.get(self.state_source, RobotState(source=self.state_source))

    def enable(self) -> str:
        with self._lock:
            if self.mode == "active":
                return "already active"
            enabled = []
            try:
                for backend in self.backends:
                    backend.enable()
                    enabled.append(backend)
                self.states = {b.name: b.read() for b in self.backends}
                for backend_state in self.states.values():
                    self._enrich_state(backend_state)
                state = self.selected_state()
                self._validate_state(state)
                self.command_mapper.reset(state)
                self.controller.reset(state)
                self.target.joint_positions = {
                    name: motor.position
                    for name, motor in state.motors.items()
                    if self.cfg["robot"]["motors"][name]["kind"] == "leg"
                }
                self.mode = "active"
                self.fault_reason = ""
                return "enabled"
            except Exception:
                for backend in enabled:
                    backend.disable()
                raise

    def disable(self) -> str:
        with self._lock:
            self._abort_sysid_locked("operator disabled the robot")
            self.target.forward_velocity_m_s = 0.0
            self.target.yaw_rate_rad_s = 0.0
            self.mode = "idle"
            for backend in self.backends:
                backend.disable()
            self.commands = {}
            return "disabled"

    def estop(self) -> str:
        with self._lock:
            self._abort_sysid_locked("emergency stop")
            self.target.forward_velocity_m_s = 0.0
            self.target.yaw_rate_rad_s = 0.0
            self.mode = "fault"
            self.fault_reason = "emergency stop"
            failures = []
            for backend in self.backends:
                try:
                    backend.estop()
                except Exception as exc:
                    failures.append(f"{backend.name}: {exc}")
            if failures:
                raise RuntimeError("E-stop send failure: " + "; ".join(failures))
            return "e-stopped"

    def set_joint_targets(self, values: dict[str, float]) -> None:
        known = set(self.cfg["robot"]["motors"])
        with self._lock:
            self._reject_manual_during_sysid()
            for name, value in values.items():
                if name not in known or self.cfg["robot"]["motors"][name]["kind"] != "leg":
                    raise ValueError(f"unknown leg joint {name!r}")
                if not math.isfinite(float(value)):
                    raise ValueError("joint targets must be finite")
                self.target.joint_positions[name] = float(value)

    def set_cartesian_target(self, side: str, x: float, y: float) -> None:
        self._reject_manual_during_sysid()
        leg = self.cfg["robot"]["legs"].get(side)
        if leg is None:
            raise ValueError(f"unknown side {side!r}")
        result = inverse(
            float(x),
            float(y),
            float(leg["base"]),
            float(leg["upper"]),
            float(leg["lower"]),
        )
        if result is None:
            raise ValueError(f"{side} Cartesian target is outside the workspace")
        self.set_joint_targets(
            {
                f"{side}_inner": result[0],
                f"{side}_outer": result[1],
            }
        )

    def set_wheel_torques(self, values: dict[str, float]) -> None:
        with self._lock:
            self._reject_manual_during_sysid()
            for name, value in values.items():
                if name not in self.target.wheel_torques:
                    raise ValueError(f"unknown wheel {name!r}")
                if not math.isfinite(float(value)):
                    raise ValueError("wheel torque targets must be finite")
                self.target.wheel_torques[name] = float(value)

    def set_drive_target(self, forward_velocity_m_s: float, yaw_rate_rad_s: float) -> None:
        with self._lock:
            self._reject_manual_during_sysid()
            drive = self.cfg.get("drive")
            if not drive:
                raise RuntimeError("drive controller is not configured")
            forward = float(forward_velocity_m_s)
            yaw = float(yaw_rate_rad_s)
            if not math.isfinite(forward) or not math.isfinite(yaw):
                raise ValueError("drive targets must be finite")
            max_forward = float(drive["max_forward_velocity_m_s"])
            max_yaw = float(drive["max_yaw_rate_rad_s"])
            self.target.forward_velocity_m_s = min(
                max(forward, -max_forward), max_forward
            )
            self.target.yaw_rate_rad_s = min(max(yaw, -max_yaw), max_yaw)
            self._last_drive_target = time.monotonic()

    def set_gains(self, values: dict[str, float]) -> None:
        allowed = {
            "leg_kp",
            "leg_kd",
            "leg_torque_limit",
            "wheel_kd",
            "wheel_torque_limit",
        }
        hard = float(self.cfg["control"]["hard_torque_limit_nm"])
        ceilings = {
            "leg_kp": 100.0,
            "leg_kd": 10.0,
            "wheel_kd": 10.0,
            "leg_torque_limit": hard,
            "wheel_torque_limit": hard,
        }
        with self._lock:
            self._reject_manual_during_sysid()
            for name, value in values.items():
                if name not in allowed:
                    raise ValueError(f"unknown gain {name!r}")
                value = float(value)
                if not math.isfinite(value) or value < 0:
                    raise ValueError(f"{name} must be finite and non-negative")
                if value > ceilings[name]:
                    raise ValueError(f"{name} exceeds limit {ceilings[name]}")
                self.cfg["control"][name] = value

    def set_balance(self, values: dict[str, float]) -> None:
        """Atomically apply validated live tuning to the active balancer."""
        with self._lock:
            self._reject_manual_during_sysid()
            if not hasattr(self.controller, "balance"):
                raise RuntimeError("the selected controller has no balance parameters")
            unknown = set(values) - set(self.cfg["balance"])
            if unknown:
                raise ValueError(f"unknown balance parameters: {sorted(unknown)}")
            candidate = dict(self.cfg["balance"])
            for name, value in values.items():
                value = float(value)
                if not math.isfinite(value):
                    raise ValueError(f"balance.{name} must be finite")
                candidate[name] = value
            candidate_cfg = dict(self.cfg)
            candidate_cfg["balance"] = candidate
            validate_balance(candidate_cfg)
            self.cfg["balance"].update(candidate)

    def set_drive(self, values: dict[str, float]) -> None:
        """Atomically apply validated driving-controller parameters."""
        with self._lock:
            self._reject_manual_during_sysid()
            unknown = set(values) - set(self.cfg["drive"])
            if unknown:
                raise ValueError(f"unknown drive parameters: {sorted(unknown)}")
            candidate = dict(self.cfg["drive"])
            for name, value in values.items():
                value = float(value)
                if not math.isfinite(value):
                    raise ValueError(f"drive.{name} must be finite")
                candidate[name] = value
            candidate_cfg = dict(self.cfg)
            candidate_cfg["drive"] = candidate
            validate_drive(candidate_cfg)
            self.cfg["drive"].update(candidate)

    def arm_sysid(self, values: dict) -> dict:
        with self._lock:
            if self.mode != "active":
                raise RuntimeError("enable the robot before arming sys-ID")
            if self.cfg["runtime"]["mode"] != "simulation":
                if self.state_source != "physical":
                    raise RuntimeError("physical state must be authoritative for sys-ID")
                self._validate_state(self.selected_state())
            return self.sysid.arm(
                str(values["axis"]),
                str(values["preset"]),
                float(values["amplitude"]),
                float(values["duration_s"]),
                bool(values.get("lifted_confirmed", False)),
            )

    def start_sysid(self) -> str:
        with self._lock:
            if self.sysid.trial.state != "armed":
                raise RuntimeError("arm a sys-ID trial first")
            state = self.selected_state()
            self._validate_state(state)
            metadata = {
                "schema_version": 1,
                "trial": self.sysid.status(),
                "state_source": self.state_source,
                "runtime_mode": self.cfg["runtime"]["mode"],
                "control_hz": self.cfg["runtime"]["control_hz"],
                "robot": self.cfg["robot"],
                "control": self.cfg["control"],
                "mujoco_model": str(self.cfg["mujoco"]["model"]),
                "backends": {
                    backend.name: backend.sysid_metadata()
                    for backend in self.backends
                    if hasattr(backend, "sysid_metadata")
                },
            }
            run_id = self.recorder.start(metadata)
            try:
                self.sysid.start(state, run_id)
            except Exception:
                self.recorder.stop("failed", "trial failed to start")
                raise
            return run_id

    def abort_sysid(self, reason: str = "operator aborted the trial") -> str:
        with self._lock:
            if not self.sysid.running and self.sysid.trial.state != "armed":
                return "no active sys-ID trial"
            self._abort_sysid_locked(reason)
            self._resume_manual_locked()
            return "sys-ID trial aborted"

    def _abort_sysid_locked(self, reason: str) -> None:
        if self.sysid.running or self.sysid.trial.state == "armed":
            self.sysid.abort(reason)
            self.recorder.stop("aborted", reason)

    def _resume_manual_locked(self) -> None:
        state = self.selected_state()
        if state.motors:
            self._enrich_state(state)
            self.command_mapper.reset(state)
            self.controller.reset(state)
            self.target.joint_positions = {
                name: motor.position
                for name, motor in state.motors.items()
                if self.cfg["robot"]["motors"][name]["kind"] == "leg"
            }
        self.target.wheel_torques = {
            name: 0.0
            for name, item in self.cfg["robot"]["motors"].items()
            if item["kind"] == "wheel"
        }
        self.target.forward_velocity_m_s = 0.0
        self.target.yaw_rate_rad_s = 0.0

    def _reject_manual_during_sysid(self) -> None:
        if self.sysid.running:
            raise RuntimeError("manual controls are locked during sys-ID")

    def run(self) -> None:
        frequency = float(self.cfg["runtime"]["control_hz"])
        period = 1.0 / frequency
        next_tick = time.monotonic()
        count, window_start = 0, next_tick
        cycle_sum, cycle_max = 0.0, 0.0
        if self.imu:
            try:
                self.imu.start()
            except Exception as exc:
                # Losing the IMU mid-run only degrades telemetry, because
                # _validate_state guards motors alone. Startup must agree:
                # an absent sensor cannot take down command authority.
                log.warning("continuing without IMU: %s", exc)
        try:
            for backend in self.backends:
                backend.start()
        except Exception as exc:
            self._fault(f"startup failed: {type(exc).__name__}: {exc}")
            return

        while not self._stop_event.is_set():
            start = time.monotonic()
            next_tick += period
            try:
                with self._lock:
                    self.states = {b.name: b.read() for b in self.backends}
                    if self.imu:
                        imu_state = self.imu.read()
                        for state in self.states.values():
                            state.imu = imu_state
                    for backend_state in self.states.values():
                        self._enrich_state(backend_state)
                    if self.mode == "active":
                        safety_states = {self.state_source: self.selected_state()}
                        if "physical" in self.states:
                            safety_states["physical"] = self.states["physical"]
                        for state in safety_states.values():
                            self._validate_state(state)
                        state = self.selected_state()
                        if (
                            time.monotonic() - self._last_drive_target
                            > float(self.cfg["drive"]["command_timeout_s"])
                        ):
                            self.target.forward_velocity_m_s = 0.0
                            self.target.yaw_rate_rad_s = 0.0
                        if self.sysid.running:
                            self.commands = self.sysid.update(state, period)
                            self.leg_commands = {}
                        else:
                            requested = self.controller.update(
                                state, self.target, period
                            )
                            if isinstance(requested, RobotCommand):
                                self.commands, self.leg_commands = (
                                    self.command_mapper.map(requested, state, period)
                                )
                            else:
                                self.commands = requested
                                self.leg_commands = {}
                        self._validate_commands(self.commands)
                        if self.recorder.active:
                            self.recorder.record(
                                start,
                                state,
                                self.commands,
                                self.sysid.status(),
                                self.metrics,
                            )
                            if self.recorder.status()["dropped_rows"]:
                                raise RuntimeError(
                                    "sys-ID recorder overflowed; trial invalid"
                                )
                        if self.sysid.trial.state == "completed":
                            self.recorder.stop("completed")
                            self._resume_manual_locked()
                    commands = self.commands if self.mode == "active" else {}
                for backend in self.backends:
                    backend.write(commands)
            except Exception as exc:
                self._fault(f"{type(exc).__name__}: {exc}")

            elapsed = time.monotonic() - start
            count += 1
            cycle_sum += elapsed
            cycle_max = max(cycle_max, elapsed)
            now = time.monotonic()
            if now - window_start >= 1.0:
                with self._lock:
                    self.metrics.update(
                        frequency_hz=count / (now - window_start),
                        cycle_ms=1000 * cycle_sum / count,
                        max_cycle_ms=1000 * cycle_max,
                    )
                count, cycle_sum, cycle_max, window_start = 0, 0.0, 0.0, now
            delay = next_tick - time.monotonic()
            if delay > 0:
                self._stop_event.wait(delay)
            else:
                self.metrics["overruns"] += 1
                next_tick = time.monotonic()

    def _enrich_state(self, state: RobotState) -> None:
        state.legs = map_legs(state.motors, self.cfg["robot"]["legs"])
        state.motor_commands = dict(self.commands)
        state.leg_commands = dict(self.leg_commands)

    def _validate_state(self, state: RobotState) -> None:
        if not state.motors:
            raise RuntimeError(f"{state.source}: no motor state")
        velocity_limit = float(self.cfg["control"]["velocity_kill_rad_s"])
        stale_limit = float(self.cfg["control"]["stale_state_s"])
        for name, motor in state.motors.items():
            if not motor.online:
                raise RuntimeError(f"{state.source}/{name}: offline")
            if motor.age_s > stale_limit:
                raise RuntimeError(f"{state.source}/{name}: stale state")
            if motor.error:
                raise RuntimeError(f"{state.source}/{name}: {motor.error}")
            if abs(motor.velocity) > velocity_limit:
                raise RuntimeError(f"{state.source}/{name}: overspeed")

    def _validate_commands(self, commands: Dict[str, MitCommand]) -> None:
        expected = set(self.cfg["robot"]["motors"])
        if set(commands) != expected:
            missing = sorted(expected - set(commands))
            extra = sorted(set(commands) - expected)
            raise RuntimeError(
                f"controller command set mismatch (missing={missing}, extra={extra})"
            )
        hard = float(self.cfg["control"]["hard_torque_limit_nm"])
        authoritative_state = self.selected_state()
        for name, command in commands.items():
            values = asdict(command).values()
            if not all(math.isfinite(value) for value in values):
                raise RuntimeError(f"{name}: non-finite MIT command")
            if not 0 <= command.kp <= 100.0 or not 0 <= command.kd <= 10.0:
                raise RuntimeError(f"{name}: MIT gain limit exceeded")
            if abs(command.torque) > hard:
                raise RuntimeError(f"{name}: hard torque limit exceeded")
            if name in authoritative_state.motors:
                motor = authoritative_state.motors[name]
                estimated = (
                    command.kp * (command.position - motor.position)
                    + command.kd * (command.velocity - motor.velocity)
                    + command.torque
                )
                if abs(estimated) > hard + 1e-6:
                    raise RuntimeError(
                        f"{authoritative_state.source}/{name}: estimated MIT torque "
                        f"{estimated:.2f} Nm exceeds {hard:.2f} Nm"
                    )

    def _fault(self, reason: str) -> None:
        with self._lock:
            self._abort_sysid_locked(reason)
            self.mode = "fault"
            self.fault_reason = reason
            self.commands = {}
            self.target.forward_velocity_m_s = 0.0
            self.target.yaw_rate_rad_s = 0.0
        for backend in self.backends:
            try:
                backend.disable()
            except Exception:
                pass

    def snapshot(self) -> dict:
        with self._lock:
            status = getattr(self.controller, "status", None)
            data = {
                "type": "state",
                "mode": self.mode,
                "fault": self.fault_reason,
                "state_source": self.state_source,
                "states": {name: state.to_dict() for name, state in self.states.items()},
                "target": asdict(self.target),
                "commands": {
                    name: asdict(command) for name, command in self.commands.items()
                },
                "metrics": dict(self.metrics),
                "control": dict(self.cfg["control"]),
                "balance": dict(self.cfg["balance"]),
                "drive": dict(self.cfg["drive"]),
                "controller": status() if callable(status) else {},
                "sysid": self.sysid.status(),
                "recorder": self.recorder.status(),
            }
        return _json_safe(data)

    def stop(self) -> None:
        """Drop command authority and leave every drive idle.

        Every step is bounded and its failures are contained: one unreachable
        backend must not keep the others powered. Idempotent.
        """
        if self._stopped:
            return
        self._stop_event.set()
        if self.is_alive():
            self.join(timeout=JOIN_TIMEOUT_S)
            if self.is_alive():
                log.error(
                    "control loop still running %.1fs after stop; "
                    "disabling drives anyway",
                    JOIN_TIMEOUT_S,
                )
        self._stopped = True
        for backend in self.backends:
            try:
                backend.disable()
            except Exception:
                log.exception("%s: disable failed during shutdown", backend.name)
        # Bookkeeping only, so a control loop that refused to release the lock
        # delays the report rather than the drives.
        if self._lock.acquire(timeout=LOCK_TIMEOUT_S):
            try:
                self._abort_sysid_locked("runtime shutdown")
                self.mode = "idle"
                self.commands = {}
            finally:
                self._lock.release()

    def close(self) -> None:
        """Release the bus, IMU and recorder. Idempotent; safes first."""
        self.stop()
        if self._closed:
            return
        self._closed = True
        for backend in self.backends:
            try:
                backend.close()
            except Exception:
                log.exception("%s: close failed during shutdown", backend.name)
        if self.imu:
            try:
                self.imu.close()
            except Exception:
                log.exception("IMU close failed during shutdown")
        try:
            self.recorder.close()
        except Exception:
            log.exception("recorder close failed during shutdown")

    def shutdown(self) -> None:
        self.close()


def _json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value
