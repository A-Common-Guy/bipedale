from __future__ import annotations

import asyncio
import copy
import math
import json
import queue
import struct
import sys
import tempfile
import time
import unittest
from pathlib import Path

import numpy as np
import yaml

try:
    import mujoco
except ImportError:            # the IMU mount tests skip without it
    mujoco = None

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

from config import load_config
from command_mapper import CommandMapper
from controllers.driving_balancer import DrivingBalancer
from controllers.two_wheel_balancer import (
    TwoWheelBalancer,
    pitch_from_quaternion,
)

if mujoco is not None:
    from imu_frame import derive
from controller import ManualMitController
from identified import load_identified, model_sha256
from imu import (
    Bno085UartShtp,
    _ShtpFramer,
    _escape,
    _parse_report,
    _split_reports,
    _unescape,
)
from kinematics import forward, inverse, jacobian, map_legs
from models import (
    ImuState,
    LegCommand,
    MitCommand,
    MotorState,
    RobotCommand,
    RobotState,
    UserTarget,
    WheelCommand,
)
from mujoco_backend import MujocoBackend
from recorder import RunRecorder
from runtime import ControlRuntime
from sysid_controller import SysIdController


class KinematicsTests(unittest.TestCase):
    def test_inverse_forward_round_trip(self):
        geometry = (0.09, 0.08, 0.13)
        target = (0.045, 0.16)
        angles = inverse(*target, *geometry)
        self.assertIsNotNone(angles)
        result = forward(*angles, *geometry)
        self.assertAlmostEqual(result[0], target[0], places=8)
        self.assertAlmostEqual(result[1], target[1], places=8)

    def test_unreachable_target(self):
        self.assertIsNone(inverse(1.0, 1.0, 0.09, 0.08, 0.13))

    def test_cartesian_velocity_matches_finite_difference(self):
        geometry = (0.09, 0.08, 0.13)
        angles = inverse(0.045, 0.16, *geometry)
        matrix = jacobian(*angles, *geometry)
        dt = 1e-5
        velocity = (0.3, -0.2)
        start = forward(*angles, *geometry)
        finish = forward(
            angles[0] + velocity[0] * dt,
            angles[1] + velocity[1] * dt,
            *geometry,
        )
        expected = (
            (finish[0] - start[0]) / dt,
            (finish[1] - start[1]) / dt,
        )
        mapped = (
            matrix[0][0] * velocity[0] + matrix[0][1] * velocity[1],
            matrix[1][0] * velocity[0] + matrix[1][1] * velocity[1],
        )
        self.assertAlmostEqual(mapped[0], expected[0], places=5)
        self.assertAlmostEqual(mapped[1], expected[1], places=5)


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config(ROOT / "config.yaml")
        self.state = RobotState(
            motors={
                name: MotorState(position=0.0, velocity=0.0, online=True, age_s=0.0)
                for name in self.cfg["robot"]["motors"]
            },
            source="test",
        )

    def test_leg_target_is_slew_and_torque_limited(self):
        controller = ManualMitController(self.cfg)
        controller.reset(self.state)
        target = UserTarget(
            joint_positions={name: 1.0 for name in controller.leg_names}
        )
        commands = controller.update(self.state, target, 0.005)
        expected_slew = self.cfg["control"]["target_rate_rad_s"] * 0.005
        for name in controller.leg_names:
            self.assertAlmostEqual(commands[name].position, expected_slew)
            spring_torque = commands[name].kp * commands[name].position
            self.assertLessEqual(
                spring_torque, self.cfg["control"]["leg_torque_limit"]
            )

    def test_wheels_use_pure_mit_torque(self):
        controller = ManualMitController(self.cfg)
        controller.reset(self.state)
        target = UserTarget(
            wheel_torques={"right_wheel": 99.0, "left_wheel": -99.0}
        )
        commands = controller.update(self.state, target, 0.005)
        for name in controller.wheel_names:
            self.assertEqual(commands[name].kp, 0.0)
            self.assertEqual(
                abs(commands[name].torque),
                self.cfg["control"]["wheel_torque_limit"],
            )

    def test_runtime_accepts_an_injected_controller(self):
        class CustomController:
            def reset(self, state):
                pass

            def update(self, state, target, dt):
                return {}

        controller = CustomController()
        runtime = ControlRuntime(self.cfg, [], controller=controller)
        self.assertIs(runtime.controller, controller)

    def test_runtime_updates_balance_parameters_atomically(self):
        controller = TwoWheelBalancer(self.cfg)
        runtime = ControlRuntime(self.cfg, [], controller=controller)
        runtime.set_balance({"pitch_kp": 3.5, "pitch_kd": 0.3})
        self.assertEqual(controller.balance["pitch_kp"], 3.5)
        before = dict(controller.balance)
        with self.assertRaises(ValueError):
            runtime.set_balance({"torque_sign": 0.0})
        self.assertEqual(controller.balance, before)

    def test_runtime_clamps_drive_targets_and_updates_gains(self):
        controller = DrivingBalancer(self.cfg)
        runtime = ControlRuntime(self.cfg, [], controller=controller)
        runtime.set_drive_target(99.0, -99.0)
        self.assertEqual(
            runtime.target.forward_velocity_m_s,
            self.cfg["drive"]["max_forward_velocity_m_s"],
        )
        self.assertEqual(
            runtime.target.yaw_rate_rad_s,
            -self.cfg["drive"]["max_yaw_rate_rad_s"],
        )
        runtime.set_drive({"velocity_kp_pitch": 0.2})
        self.assertEqual(controller.drive["velocity_kp_pitch"], 0.2)

    def test_controller_loader_uses_module_class_syntax(self):
        from app import load_controller

        controller = load_controller(
            "example_controller:ExampleController", self.cfg
        )
        self.assertEqual(type(controller).__name__, "ExampleController")
        with self.assertRaises(ValueError):
            load_controller("missing_separator", self.cfg)

    def test_high_level_command_maps_to_bounded_mit(self):
        self.state.legs = map_legs(
            self.state.motors, self.cfg["robot"]["legs"]
        )
        mapper = CommandMapper(self.cfg)
        mapper.reset(self.state)
        request = RobotCommand(
            legs={
                side: LegCommand(x=geometry["base"] / 2.0, y=0.16)
                for side, geometry in self.cfg["robot"]["legs"].items()
            },
            wheels={
                "right_wheel": WheelCommand(torque=99.0),
                "left_wheel": WheelCommand(torque=99.0),
            },
        )
        commands, legs = mapper.map(request, self.state, 0.005)
        self.assertEqual(set(commands), set(self.cfg["robot"]["motors"]))
        geometry = self.cfg["robot"]["legs"]["right"]
        mapped_xy = forward(
            commands["right_inner"].position,
            commands["right_outer"].position,
            geometry["base"],
            geometry["upper"],
            geometry["lower"],
        )
        self.assertAlmostEqual(legs["right"].y, mapped_xy[1])
        self.assertEqual(
            commands["right_wheel"].torque,
            self.cfg["control"]["wheel_torque_limit"],
        )
        for name in ("right_inner", "right_outer", "left_inner", "left_outer"):
            command = commands[name]
            motor = self.state.motors[name]
            effort = (
                command.kp * (command.position - motor.position)
                - command.kd * motor.velocity
            )
            self.assertLessEqual(
                abs(effort), self.cfg["control"]["leg_torque_limit"] + 1e-9
            )

    def test_high_level_mapper_rejects_unreachable_leg(self):
        self.state.legs = map_legs(
            self.state.motors, self.cfg["robot"]["legs"]
        )
        mapper = CommandMapper(self.cfg)
        mapper.reset(self.state)
        with self.assertRaises(ValueError):
            mapper.map(
                RobotCommand(legs={"right": LegCommand(x=2.0, y=2.0)}),
                self.state,
                0.005,
            )


class BalancerTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config(ROOT / "config.yaml")
        geometry = self.cfg["robot"]["legs"]["right"]
        angles = inverse(
            geometry["base"] / 2.0,
            self.cfg["balance"]["leg_height_m"],
            geometry["base"],
            geometry["upper"],
            geometry["lower"],
        )
        motors = {
            name: MotorState(position=0.0, velocity=0.0, online=True, age_s=0.0)
            for name in self.cfg["robot"]["motors"]
        }
        for side in ("right", "left"):
            motors[f"{side}_inner"].position = angles[0]
            motors[f"{side}_outer"].position = angles[1]
        self.state = RobotState(
            motors=motors,
            imu=ImuState(online=True, age_s=0.0),
            source="simulation",
        )
        self.state.legs = map_legs(motors, self.cfg["robot"]["legs"])

    def test_pitch_sign_and_equal_leg_height(self):
        angle = 0.1
        quaternion = (
            math.cos(angle / 2.0),
            0.0,
            math.sin(angle / 2.0),
            0.0,
        )
        self.assertAlmostEqual(pitch_from_quaternion(quaternion), angle)
        self.state.imu.quaternion = quaternion
        self.cfg["balance"]["settle_s"] = 0.0
        self.cfg["balance"]["ramp_s"] = 0.01
        self.cfg["balance"]["torque_rate_nm_s"] = 100.0
        controller = TwoWheelBalancer(self.cfg)
        controller.reset(self.state)
        command = controller.update(self.state, UserTarget(), 0.02)
        self.assertIsInstance(command, RobotCommand)
        self.assertEqual(command.legs["right"].y, command.legs["left"].y)
        self.assertGreater(command.wheels["right_wheel"].torque, 0.0)
        self.assertEqual(
            command.wheels["right_wheel"].torque,
            command.wheels["left_wheel"].torque,
        )
        self.assertLessEqual(
            abs(command.wheels["right_wheel"].torque),
            self.cfg["balance"]["torque_limit_nm"],
        )

    def test_pitch_is_independent_of_world_heading(self):
        pitch = 0.2
        yaw = 1.3
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        # Body-to-world quaternion q_yaw * q_pitch.
        quaternion = (cy * cp, -sy * sp, cy * sp, sy * cp)
        self.assertAlmostEqual(
            pitch_from_quaternion(quaternion), pitch, places=8
        )

    def test_stale_imu_and_fall_latch_zero_torque(self):
        controller = TwoWheelBalancer(self.cfg)
        controller.reset(self.state)
        self.state.imu.age_s = 1.0
        command = controller.update(self.state, UserTarget(), 0.01)
        self.assertEqual(controller.status()["inhibit_reason"], "IMU offline or stale")
        self.assertEqual(command.wheels["right_wheel"].torque, 0.0)
        self.state.imu.age_s = 0.0
        angle = self.cfg["balance"]["fall_angle_rad"] + 0.1
        self.state.imu.quaternion = (
            math.cos(angle / 2.0),
            0.0,
            math.sin(angle / 2.0),
            0.0,
        )
        controller.update(self.state, UserTarget(), 0.01)
        self.state.imu.quaternion = (1.0, 0.0, 0.0, 0.0)
        command = controller.update(self.state, UserTarget(), 0.01)
        self.assertTrue(controller.fallen)
        self.assertEqual(controller.status()["inhibit_reason"], "fall latch")
        self.assertEqual(command.wheels["left_wheel"].torque, 0.0)

    def test_balance_torque_is_slew_limited(self):
        self.cfg["balance"]["settle_s"] = 0.0
        self.cfg["balance"]["ramp_s"] = 0.001
        self.cfg["balance"]["torque_rate_nm_s"] = 1.0
        angle = 0.2
        self.state.imu.quaternion = (
            math.cos(angle / 2.0),
            0.0,
            math.sin(angle / 2.0),
            0.0,
        )
        controller = TwoWheelBalancer(self.cfg)
        controller.reset(self.state)
        command = controller.update(self.state, UserTarget(), 0.01)
        self.assertAlmostEqual(command.wheels["right_wheel"].torque, 0.01)

    def test_driving_balancer_converts_speed_to_pitch_target(self):
        self.cfg["balance"]["settle_s"] = 0.0
        self.cfg["balance"]["ramp_s"] = 0.001
        self.cfg["drive"]["forward_accel_limit_m_s2"] = 100.0
        controller = DrivingBalancer(self.cfg)
        controller.reset(self.state)
        target = UserTarget(forward_velocity_m_s=0.2)
        controller.update(self.state, target, 0.01)
        self.assertGreater(controller.pitch_offset, 0.0)
        self.assertGreater(
            controller.active_pitch_target,
            self.cfg["balance"]["pitch_target_rad"],
        )

    def test_driving_balancer_uses_differential_yaw_torque(self):
        self.cfg["balance"]["settle_s"] = 0.0
        self.cfg["balance"]["ramp_s"] = 0.001
        controller = DrivingBalancer(self.cfg)
        controller.reset(self.state)
        command = controller.update(
            self.state, UserTarget(yaw_rate_rad_s=1.0), 0.01
        )
        self.assertGreater(
            command.wheels["right_wheel"].torque,
            command.wheels["left_wheel"].torque,
        )

    def test_driving_balancer_holds_last_position_at_zero_speed(self):
        controller = DrivingBalancer(self.cfg)
        controller.reset(self.state)
        self.state.motors["right_wheel"].position = 1.0
        self.state.motors["left_wheel"].position = 1.0
        controller.update(self.state, UserTarget(), 0.01)
        self.assertLess(controller.velocity_target, 0.0)

    def test_forward_command_gets_no_wheel_feedforward(self):
        # Forward authority comes from the lean, not from pushing the wheels.
        cfg = copy.deepcopy(self.cfg)
        cfg["balance"]["settle_s"] = 0.0
        cfg["balance"]["ramp_s"] = 0.001
        cfg["drive"]["forward_accel_limit_m_s2"] = 100.0
        controller = DrivingBalancer(cfg)
        controller.reset(self.state)
        for _ in range(50):
            command = controller.update(
                self.state, UserTarget(forward_velocity_m_s=0.4), 0.01
            )
        self.assertAlmostEqual(command.wheels["right_wheel"].velocity, 0.0)
        self.assertAlmostEqual(command.wheels["left_wheel"].velocity, 0.0)

    def test_wheel_damping_reference_is_differential_for_yaw(self):
        cfg = copy.deepcopy(self.cfg)
        cfg["balance"]["settle_s"] = 0.0
        cfg["balance"]["ramp_s"] = 0.001
        controller = DrivingBalancer(cfg)
        controller.reset(self.state)
        command = controller.update(
            self.state, UserTarget(yaw_rate_rad_s=1.0), 0.01
        )
        right = command.wheels["right_wheel"].velocity
        left = command.wheels["left_wheel"].velocity
        self.assertGreater(right, 0.0)
        self.assertLess(left, 0.0)
        self.assertAlmostEqual(right + left, 0.0)

    def test_forward_velocity_includes_chassis_pitch_rate(self):
        controller = DrivingBalancer(self.cfg)
        controller.reset(self.state)
        # Wheels planted while the body pitches: joints read the negative of the
        # pitch rate, so ground speed is zero.
        self.state.imu.gyro = (0.0, 0.8, 0.0)
        self.state.motors["right_wheel"].velocity = -0.8
        self.state.motors["left_wheel"].velocity = -0.8
        controller.update(self.state, UserTarget(), 0.01)
        self.assertAlmostEqual(controller.forward_velocity, 0.0, places=12)

    def test_integral_budget_is_in_radians_and_independent_of_gain(self):
        budget = float(self.cfg["drive"]["velocity_integral_limit_rad"])
        for ki in (0.01, 0.02, 0.1):
            cfg = copy.deepcopy(self.cfg)
            cfg["balance"]["settle_s"] = 0.0
            cfg["balance"]["ramp_s"] = 0.001
            cfg["drive"]["velocity_kp_pitch"] = 0.0  # isolate the integral
            cfg["drive"]["velocity_ki_pitch"] = ki
            controller = DrivingBalancer(cfg)
            # Reference is captured at reset, so start from zero every time.
            self.state.motors["right_wheel"].position = 0.0
            self.state.motors["left_wheel"].position = 0.0
            controller.reset(self.state)
            # Parked well behind the hold reference: the error never clears.
            offset = -2.0 / float(cfg["drive"]["wheel_radius_m"])
            self.state.motors["right_wheel"].position = offset
            self.state.motors["left_wheel"].position = offset
            for _ in range(60000):
                controller.update(self.state, UserTarget(), 0.01)
            self.assertAlmostEqual(controller.pitch_offset, budget, places=6)


class RuntimeSafetyTests(unittest.TestCase):
    def test_ignores_non_authoritative_simulation_impedance_error(self):
        cfg = load_config(ROOT / "config.yaml")
        runtime = ControlRuntime(cfg, [])
        physical = {
            name: MotorState(position=0.0, online=True, age_s=0.0)
            for name in cfg["robot"]["motors"]
        }
        simulation = {
            name: MotorState(position=2.0, online=True, age_s=0.0)
            for name in cfg["robot"]["motors"]
        }
        runtime.states = {
            "physical": RobotState(motors=physical, source="physical"),
            "simulation": RobotState(motors=simulation, source="simulation"),
        }
        commands = {name: MitCommand() for name in cfg["robot"]["motors"]}
        commands["right_outer"] = MitCommand(position=0.0, kp=8.0)

        runtime._validate_commands(commands)

    def test_unavailable_imu_does_not_stop_the_control_loop(self):
        cfg = load_config(ROOT / "config.yaml")
        cfg["runtime"]["control_hz"] = 200

        class MissingImu:
            def start(self):
                raise RuntimeError("could not open port /dev/ttyUSB0")

            def read(self):
                return ImuState()

            def close(self):
                pass

        class StubBackend:
            name = "simulation"

            def start(self):
                self.cycles = 0

            def read(self):
                self.cycles += 1
                return RobotState(
                    motors={
                        name: MotorState(position=0.0, online=True, age_s=0.0)
                        for name in cfg["robot"]["motors"]
                    },
                    source=self.name,
                )

            def write(self, commands):
                pass

            def enable(self):
                pass

            def disable(self):
                pass

            def estop(self):
                pass

            def close(self):
                pass

        backend = StubBackend()
        runtime = ControlRuntime(cfg, [backend], imu=MissingImu())
        runtime.start()
        try:
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline and runtime.metrics["frequency_hz"] == 0:
                time.sleep(0.01)
        finally:
            runtime.shutdown()

        self.assertEqual(runtime.mode, "idle")
        self.assertEqual(runtime.fault_reason, "")
        self.assertGreater(runtime.metrics["frequency_hz"], 0.0)


class SysIdTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config(ROOT / "config.yaml")
        self.state = RobotState(
            motors={
                name: MotorState(position=0.1, online=True, age_s=0.0)
                for name in self.cfg["robot"]["motors"]
            },
            source="simulation",
        )

    def test_trial_transitions_and_bounds(self):
        controller = SysIdController(self.cfg)
        with self.assertRaises(ValueError):
            controller.arm("right_wheel", "wheel_steps", 0.21, 2.0, True)
        controller.arm("right_wheel", "wheel_steps", 0.1, 2.0, True)
        controller.start(self.state, "test-run")
        saw_excitation = False
        for _ in range(10):
            commands = controller.update(self.state, 0.5)
            saw_excitation |= abs(commands["right_wheel"].torque) > 0
            for name, command in commands.items():
                if self.cfg["robot"]["motors"][name]["kind"] == "leg":
                    estimated = (
                        command.kp
                        * (command.position - self.state.motors[name].position)
                        - command.kd * self.state.motors[name].velocity
                        + command.torque
                    )
                    self.assertLessEqual(
                        abs(estimated),
                        self.cfg["control"]["leg_torque_limit"] + 1e-9,
                    )
            if controller.trial.state == "completed":
                break
        self.assertTrue(saw_excitation)
        self.assertEqual(controller.trial.state, "completed")

    def test_recorder_writes_schema_and_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = RunRecorder(Path(directory), self.state.motors, 128)
            run_id = recorder.start({"trial": {"axis": "right_wheel"}})
            recorder.record(
                time.monotonic(),
                self.state,
                {name: MitCommand() for name in self.state.motors},
                {"state": "excite", "segment": "step"},
                {"cycle_ms": 1.0, "overruns": 0},
            )
            recorder.stop("completed")
            recorder.close()
            metadata = json.loads(
                (Path(directory) / f"{run_id}.json").read_text()
            )
            header = (Path(directory) / f"{run_id}.csv").read_text().splitlines()[0]
            self.assertEqual(metadata["status"], "completed")
            self.assertEqual(metadata["rows"], 1)
            self.assertIn("right_wheel.cmd_torque", header)

    def test_recorder_overflow_is_non_blocking(self):
        recorder = RunRecorder(Path("."), self.state.motors, 1)
        recorder._queue = queue.Queue(maxsize=1)
        recorder._queue.put_nowait({})
        recorder._active = True
        recorder.record(
            time.monotonic(), self.state, {}, {}, {}
        )
        self.assertEqual(recorder.status()["dropped_rows"], 1)
        recorder._active = False

    def test_identified_model_hash_validation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            model = root / "model.xml"
            model.write_text("<mujoco/>")
            params = root / "params.yaml"
            values = {
                "enabled": True,
                "model_sha256": model_sha256(model),
                "command_delay_s": 0.0,
                "groups": {
                    kind: {
                        "damping": 0.1,
                        "frictionloss": 0.1,
                        "armature": 0.001,
                        "torque_scale": 1.0,
                    }
                    for kind in ("leg", "wheel")
                },
            }
            params.write_text(yaml.safe_dump(values))
            self.assertIsNotNone(load_identified(params, model))
            model.write_text("<mujoco model='changed'/>")
            with self.assertRaises(ValueError):
                load_identified(params, model)

    def test_simulated_mit_torque_is_limited(self):
        backend = MujocoBackend.__new__(MujocoBackend)
        backend._torque_scale = {"joint": 1.0}
        backend._torque_limit = {"joint": 2.0}
        torque = backend._mit_torque(
            "joint", MitCommand(position=10.0, kp=10.0), 0.0, 0.0
        )
        self.assertEqual(torque, 2.0)


class StubBackend:
    """Backend that records the order of its lifecycle calls."""

    def __init__(self, cfg, name="physical", disable_error=None):
        self.name = name
        self.cfg = cfg
        self.calls = []
        self.disable_error = disable_error

    def start(self):
        self.calls.append("start")

    def read(self):
        return RobotState(
            motors={
                name: MotorState(position=0.0, online=True, age_s=0.0)
                for name in self.cfg["robot"]["motors"]
            },
            source=self.name,
        )

    def write(self, commands):
        pass

    def enable(self):
        self.calls.append("enable")

    def disable(self):
        self.calls.append("disable")
        if self.disable_error is not None:
            raise self.disable_error

    def estop(self):
        self.calls.append("estop")

    def close(self):
        self.calls.append("close")


class ShutdownTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config(ROOT / "config.yaml")

    def test_stop_disables_every_backend_despite_one_failure(self):
        broken = StubBackend(
            self.cfg, "physical", disable_error=RuntimeError("CAN write failed")
        )
        healthy = StubBackend(self.cfg, "simulation")
        runtime = ControlRuntime(self.cfg, [broken, healthy])

        runtime.stop()

        self.assertEqual(broken.calls, ["disable"])
        self.assertEqual(healthy.calls, ["disable"])

    def test_stop_and_close_are_idempotent(self):
        backend = StubBackend(self.cfg)
        runtime = ControlRuntime(self.cfg, [backend])

        runtime.stop()
        runtime.stop()
        runtime.close()
        runtime.close()
        runtime.shutdown()

        self.assertEqual(backend.calls, ["disable", "close"])

    def test_close_safes_the_drives_before_releasing_the_bus(self):
        backend = StubBackend(self.cfg)
        runtime = ControlRuntime(self.cfg, [backend])

        runtime.close()

        self.assertEqual(backend.calls, ["disable", "close"])

    def test_running_control_loop_is_stopped_by_stop(self):
        cfg = load_config(ROOT / "config.yaml")
        cfg["runtime"]["control_hz"] = 200
        backend = StubBackend(cfg, "simulation")
        runtime = ControlRuntime(cfg, [backend])
        runtime.start()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and runtime.metrics["frequency_hz"] == 0:
            time.sleep(0.01)

        started = time.monotonic()
        runtime.stop()
        elapsed = time.monotonic() - started

        self.assertFalse(runtime.is_alive())
        self.assertLess(elapsed, 2.0)
        self.assertEqual(runtime.mode, "idle")


class DashboardShutdownTests(unittest.TestCase):
    """The dashboard must not delay the safe state.

    aiohttp waits out ``shutdown_timeout`` on in-flight handlers between
    on_shutdown and on_cleanup. A websocket handler never completes on its
    own, so safing the robot from on_cleanup left the drives commanded for
    that whole window after Ctrl+C.
    """

    def test_open_websocket_does_not_delay_shutdown(self):
        import app as app_module

        cfg = load_config(ROOT / "config.yaml")
        events = []

        class StubRuntime:
            def __init__(self):
                self.sysid = self
                self.running = False

            def presets(self):
                return {}

            def start(self):
                pass

            def snapshot(self):
                return {"type": "state"}

            def selected_state(self):
                return RobotState(source="physical")

            def stop(self):
                events.append("stop")

            def close(self):
                events.append("close")

        class StubVisualizer:
            hanged = False

            def render_jpeg(self):
                return b""

            def close_renderer(self):
                events.append("close_renderer")

            def close(self):
                events.append("close_visualizer")

        async def scenario():
            from aiohttp import ClientSession
            from aiohttp.test_utils import TestServer

            web_app = app_module.make_web_app(
                StubRuntime(), StubVisualizer(), False, cfg
            )
            server = TestServer(web_app)
            await server.start_server()
            async with ClientSession() as session:
                async with session.ws_connect(server.make_url("/ws")) as ws:
                    await ws.receive()  # the config frame
                    events.append("client connected")
                    started = time.monotonic()
                    await server.close()
                    return time.monotonic() - started

        elapsed = asyncio.run(scenario())

        self.assertLess(elapsed, 5.0, "shutdown waited on the open websocket")
        self.assertEqual(events[0], "client connected")
        self.assertEqual(events[1], "stop", "drives were not safed first")
        self.assertIn("close", events)


class ConfigurationTests(unittest.TestCase):
    def test_paths_are_relative_to_yaml(self):
        cfg = load_config(ROOT / "config.yaml")
        self.assertTrue(cfg["mujoco"]["model"].is_absolute())
        self.assertTrue(cfg["can"]["gim6010_path"].is_absolute())

    def test_rejects_limit_above_hard_ceiling(self):
        values = yaml.safe_load((ROOT / "config.yaml").read_text())
        values["control"]["leg_torque_limit"] = (
            float(values["control"]["hard_torque_limit_nm"]) + 1.0
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.yaml"
            path.write_text(yaml.safe_dump(values))
            with self.assertRaises(ValueError):
                load_config(path)


def _sensor_report(report_id, values, scalar, size):
    body = bytearray(size)
    body[0] = report_id
    struct.pack_into(
        "<" + "h" * len(values), body, 4, *[round(v / scalar) for v in values]
    )
    return bytes(body)


def _uart_frame(reports):
    """Wrap reports the way the BNO085 puts them on the wire."""
    data = b"".join(reports)
    packet = bytearray(4 + len(data))
    struct.pack_into("<H", packet, 0, len(packet))
    packet[2] = 3  # input sensor reports channel
    packet[4:] = data
    return bytes([0x7E, 0x01]) + _escape(packet) + bytes([0x7E])


_GAME_QUAT = _sensor_report(0x08, (0.1, 0.2, 0.3, 0.9), 2**-14, 12)
_GYRO = _sensor_report(0x02, (1.0, 2.0, 3.0), 2**-9, 10)
_ACCEL = _sensor_report(0x01, (4.0, 5.0, 6.0), 2**-8, 10)
_TIMEBASE = bytes([0xFB, 0, 0, 0, 0])


class ImuFramingTests(unittest.TestCase):
    def test_escape_round_trip(self):
        raw = bytearray([0x00, 0x7E, 0x7D, 0x01, 0x7E, 0x7D])
        self.assertEqual(_unescape(_escape(raw)), bytes(raw))

    def test_framer_reassembles_across_chunk_boundaries(self):
        wire = _uart_frame([_TIMEBASE, _ACCEL]) * 3
        for size in (1, 7, 32, len(wire)):
            framer = _ShtpFramer()
            packets = []
            for start in range(0, len(wire), size):
                packets.extend(framer.feed(wire[start : start + size]))
            self.assertEqual(len(packets), 3, f"chunk size {size}")

    def test_split_reports_walks_a_batch(self):
        packet = _unescape(_uart_frame([_TIMEBASE, _ACCEL, _GYRO, _GAME_QUAT])[1:-1])[1:]
        found = [report_id for report_id, _payload in _split_reports(packet)]
        self.assertEqual(found, [0xFB, 0x01, 0x02, 0x08])

    def test_parse_report_applies_q_point_scaling(self):
        values = _parse_report(0x02, _GYRO)
        for actual, expected in zip(values, (1.0, 2.0, 3.0)):
            self.assertAlmostEqual(actual, expected, places=2)


class ImuStateTests(unittest.TestCase):
    def _sensor(self):
        return Bno085UartShtp("/dev/null", 3000000, 0.05, 5000)

    def test_game_rotation_vector_is_published_as_wxyz(self):
        sensor = self._sensor()
        sensor._consume(_uart_frame([_TIMEBASE, _GAME_QUAT, _GYRO, _ACCEL]))
        state = sensor.read()
        for actual, expected in zip(state.quaternion, (0.9, 0.1, 0.2, 0.3)):
            self.assertAlmostEqual(actual, expected, places=3)
        for actual, expected in zip(state.gyro, (1.0, 2.0, 3.0)):
            self.assertAlmostEqual(actual, expected, places=2)
        for actual, expected in zip(state.accel, (4.0, 5.0, 6.0)):
            self.assertAlmostEqual(actual, expected, places=2)
        self.assertTrue(state.online)

    def test_age_tracks_orientation_only(self):
        """A stalled orientation must read as stale even while other reports flow."""
        sensor = self._sensor()
        sensor._consume(_uart_frame([_TIMEBASE, _ACCEL]))
        self.assertEqual(sensor.read().age_s, float("inf"))
        self.assertFalse(sensor.read().online)

        sensor._consume(_uart_frame([_TIMEBASE, _GAME_QUAT]))
        self.assertTrue(sensor.read().online)
        self.assertEqual(sensor.samples, 1)

        stale = time.monotonic() - 5.0
        with sensor._lock:
            sensor._last = stale
        sensor._consume(_uart_frame([_TIMEBASE, _ACCEL]))
        self.assertFalse(sensor.read().online)
        self.assertEqual(sensor.samples, 1)

    def test_newest_sample_in_a_chunk_wins(self):
        sensor = self._sensor()
        older = _sensor_report(0x08, (0.0, 0.0, 0.0, 1.0), 2**-14, 12)
        chunk = _uart_frame([_TIMEBASE, older]) + _uart_frame([_TIMEBASE, _GAME_QUAT])
        sensor._consume(chunk)
        self.assertAlmostEqual(sensor.read().quaternion[0], 0.9, places=3)


@unittest.skipIf(mujoco is None, "MuJoCo is not installed")
class ImuMountTests(unittest.TestCase):
    """The IMU site must sit flat on the roof, not on the exported torso frame.

    Onshape hands the torso a frame whose z runs along the wheel axle, so a
    site left at pos="0 0 0" with no quat reports orientations, rates and
    accelerations in axes that no physical BNO085 mounting can reproduce.
    """

    @classmethod
    def setUpClass(cls):
        cfg = load_config(ROOT / "config.yaml")
        cls.model = mujoco.MjModel.from_xml_path(str(cfg["mujoco"]["model"]))
        cls.data = mujoco.MjData(cls.model)
        key = mujoco.mj_name2id(cls.model, mujoco.mjtObj.mjOBJ_KEY, "stand")
        mujoco.mj_resetDataKeyframe(cls.model, cls.data, key)
        mujoco.mj_forward(cls.model, cls.data)
        cls.site = mujoco.mj_name2id(cls.model, mujoco.mjtObj.mjOBJ_SITE, "imu")
        cls.torso = mujoco.mj_name2id(cls.model, mujoco.mjtObj.mjOBJ_BODY, "torso")

    def _world_axes(self):
        """Columns are the site's x, y, z expressed in world coordinates."""
        return self.data.site_xmat[self.site].reshape(3, 3)

    def test_site_matches_the_frame_derived_from_the_roof_mesh(self):
        position, rotation, info = derive(self.model, self.data)
        self.assertLess(
            float(np.linalg.norm(position - self.model.site_pos[self.site])), 1e-4
        )
        torso_axes = self.data.xmat[self.torso].reshape(3, 3)
        np.testing.assert_allclose(
            self._world_axes(), torso_axes @ rotation, atol=1e-6
        )
        # A plate face this flat means the centroid really is on the surface.
        self.assertLess(info["flatness"], 1e-4)

    def test_y_axis_is_the_wheel_axle(self):
        joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "left_wheel")
        axle = self.data.xaxis[joint]
        self.assertAlmostEqual(
            abs(float(self._world_axes()[:, 1] @ axle)), 1.0, places=6
        )

    def test_z_axis_points_up_at_the_stand_pose(self):
        # The stand keyframe leans the roof by a few degrees, so this only
        # pins that "up" is up rather than the 113 deg the torso frame gives.
        self.assertGreater(float(self._world_axes()[2, 2]), 0.99)

    def test_triad_is_drawn_without_disturbing_the_torso(self):
        names = {
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_SITE, site)
            for site in range(self.model.nsite)
        }
        self.assertLessEqual({"imu_x", "imu_y", "imu_z"}, names)
        # Sites never carry mass; the exported <inertial> must survive intact.
        self.assertAlmostEqual(float(self.model.body_mass[self.torso]), 0.148617,
                               places=6)


@unittest.skipIf(mujoco is None, "MuJoCo is not installed")
class ImuMirroringTests(unittest.TestCase):
    """A physical IMU reading must land on the torso, not 113 deg away from it."""

    def test_mirror_state_recovers_the_pose_that_produced_the_reading(self):
        backend = MujocoBackend(load_config(ROOT / "config.yaml"))
        self.addCleanup(backend.close)
        backend.hanged = False
        backend.start()

        # Tip the torso well away from the keyframe so the round trip has to
        # undo a real rotation instead of an identity.
        expected = np.array([0.9422, 0.08, 0.24, 0.21])
        expected /= np.linalg.norm(expected)
        backend.data.qpos[3:7] = expected
        backend.mj.mj_forward(backend.model, backend.data)
        reading = backend.read().imu.quaternion

        backend.data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        backend.mj.mj_forward(backend.model, backend.data)
        backend.mirror_state(
            RobotState(motors={}, imu=ImuState(quaternion=reading, online=True))
        )

        actual = backend.data.qpos[3:7].copy()
        if float(actual @ expected) < 0:      # q and -q are the same rotation
            actual = -actual
        np.testing.assert_allclose(actual, expected, atol=2e-3)


if __name__ == "__main__":
    unittest.main()
