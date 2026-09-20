#!/usr/bin/env python3
"""Scripted drive test for a running balancer, in simulation or on hardware.

Publishes a sequence of geometry_msgs/TwistStamped commands on /cmd_vel and
prints the controller's diagnostics as a table, so a controller can be judged
from numbers without a viewer. Run with the simulation already up:

    ros2 launch sandro_bringup sandro.launch.py hardware:=mujoco \\
        controller:=whole_body_controller headless:=true rviz:=false teleop:=false
    python3 tools/drive_test.py --controller whole_body_controller

The default sequence stands, drives forward, stops, turns while driving,
spins in place, changes height, and stands again. Each phase is
"duration,forward_m_s,yaw_rad_s[,height_rate_m_s]"; pass --phase to replace
it. On hardware keep the robot tethered and the E-stop in reach; the test
only publishes commands the controller limits anyway, but it does not know
what is in front of the robot.
"""

from __future__ import annotations

import argparse
import csv
import sys
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu, JointState

COLUMNS = (
    ("t", "t_s", 6, 2),
    ("cmd_v", "cmd_v", 6, 2),
    ("cmd_w", "cmd_w", 6, 2),
    ("pitch", "pitch_rad", 7, 3),
    ("roll", "roll_rad", 7, 3),
    ("v", "forward_velocity_m_s", 7, 3),
    ("v_ref", "velocity_reference_m_s", 6, 2),
    ("x_err", "position_error_m", 7, 3),
    ("yaw", "yaw_rate_rad_s", 6, 2),
    ("h", "leg_height_m", 6, 3),
    ("dh", "height_differential_m", 7, 3),
    ("d", "leg_offset_m", 7, 3),
    ("d_cmd", "leg_offset_command_m", 7, 3),
    ("trim", "trim_m", 7, 3),
    ("tau", "balance_torque_nm", 6, 2),
    ("tau_yaw", "yaw_torque_nm", 7, 2),
)

DEFAULT_PHASES = [
    "3,0,0", "4,0.3,0", "3,0,0", "4,0.3,1.0", "3,0,0", "3,0,1.2", "2,0,0",
    "3,0,0,-0.05", "2,0,0", "3,0,0,0.05", "3,0,0",
]


class Recorder:
    """Writes every diagnostics, joint-state and IMU sample to one CSV.

    Rows share one wide header (`stream` says which message a row came from,
    unused columns stay empty) so simulation and hardware runs can be compared
    sample by sample with tools/compare_runs.py. `phase` is the index of the
    command phase active when the sample arrived.
    """

    JOINTS = ("right_motor_inner", "right_motor_outer", "right_wheel",
              "left_motor_inner", "left_motor_outer", "left_wheel")

    def __init__(self, path: str, diagnostic_keys):
        self.file = open(path, "w", newline="")
        self.columns = (
            ["t_s", "stream", "phase", "cmd_forward", "cmd_yaw", "cmd_height_rate"]
            + [f"diag_{key}" for key in diagnostic_keys]
            + [f"pos_{j}" for j in self.JOINTS] + [f"vel_{j}" for j in self.JOINTS]
            + [f"eff_{j}" for j in self.JOINTS]
            + ["imu_qx", "imu_qy", "imu_qz", "imu_qw", "imu_gx", "imu_gy", "imu_gz",
               "imu_ax", "imu_ay", "imu_az"]
        )
        self.writer = csv.DictWriter(self.file, fieldnames=self.columns, extrasaction="ignore")
        self.writer.writeheader()
        self.start = time.monotonic()
        self.phase = -1
        self.command = (0.0, 0.0, 0.0)
        self.rows = 0

    def _base(self, stream: str) -> dict:
        return {"t_s": f"{time.monotonic() - self.start:.4f}", "stream": stream,
                "phase": self.phase, "cmd_forward": self.command[0],
                "cmd_yaw": self.command[1], "cmd_height_rate": self.command[2]}

    def diagnostics(self, values: dict) -> None:
        row = self._base("diag")
        row.update({f"diag_{k}": v for k, v in values.items()})
        self.writer.writerow(row)
        self.rows += 1

    def joints(self, msg: JointState) -> None:
        row = self._base("joint")
        index = {name: i for i, name in enumerate(msg.name)}
        for joint in self.JOINTS:
            i = index.get(joint)
            if i is None:
                continue
            if i < len(msg.position):
                row[f"pos_{joint}"] = f"{msg.position[i]:.6f}"
            if i < len(msg.velocity):
                row[f"vel_{joint}"] = f"{msg.velocity[i]:.6f}"
            if i < len(msg.effort):
                row[f"eff_{joint}"] = f"{msg.effort[i]:.6f}"
        self.writer.writerow(row)
        self.rows += 1

    def imu(self, msg: Imu) -> None:
        row = self._base("imu")
        q, g, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
        row.update({"imu_qx": q.x, "imu_qy": q.y, "imu_qz": q.z, "imu_qw": q.w,
                    "imu_gx": g.x, "imu_gy": g.y, "imu_gz": g.z,
                    "imu_ax": a.x, "imu_ay": a.y, "imu_az": a.z})
        self.writer.writerow(row)
        self.rows += 1

    def close(self) -> None:
        self.file.close()


class DriveTest(Node):
    def __init__(self, controller: str, topic: str, rate_hz: float):
        super().__init__("drive_test")
        qos = QoSProfile(depth=10)
        self.publisher = self.create_publisher(TwistStamped, topic, qos)
        self.subscription = self.create_subscription(
            DiagnosticArray, f"/{controller}/diagnostics", self._on_diagnostics, qos)
        self.latest: dict[str, str] = {}
        self.message = ""
        self.level = 0
        self.samples = 0
        self.period = 1.0 / rate_hz
        self.recorder: Recorder | None = None
        self.record_path: str | None = None
        self.joint_subscription = None
        self.imu_subscription = None

    def start_recording(self, path: str) -> None:
        # The header needs the diagnostic keys, so open the file at the first
        # diagnostics message (already received when this is called).
        self.recorder = Recorder(path, sorted(self.latest))
        self.joint_subscription = self.create_subscription(
            JointState, "/joint_states", self._on_joints, 50)
        self.imu_subscription = self.create_subscription(
            Imu, "/imu_broadcaster/imu", self._on_imu, 50)

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        if not msg.status:
            return
        status = msg.status[0]
        self.latest = {item.key: item.value for item in status.values}
        self.message = status.message
        level = status.level
        self.level = int.from_bytes(level, "little") if isinstance(level, bytes) else int(level)
        self.samples += 1
        if self.recorder:
            self.recorder.diagnostics(self.latest)

    def _on_joints(self, msg: JointState) -> None:
        if self.recorder:
            self.recorder.joints(msg)

    def _on_imu(self, msg: Imu) -> None:
        if self.recorder:
            self.recorder.imu(msg)

    def publish(self, forward: float, yaw: float, height_rate: float) -> None:
        if self.recorder:
            self.recorder.command = (forward, yaw, height_rate)
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = "base_link"
        twist.twist.linear.x = forward
        twist.twist.linear.z = height_rate
        twist.twist.angular.z = yaw
        self.publisher.publish(twist)


def parse_phase(text: str) -> tuple[float, float, float, float]:
    parts = [float(v) for v in text.split(",")]
    if len(parts) == 3:
        parts.append(0.0)
    if len(parts) != 4:
        raise argparse.ArgumentTypeError(f"phase {text!r} needs 3 or 4 numbers")
    return tuple(parts)  # type: ignore[return-value]


def row(node: DriveTest, t: float, forward: float, yaw: float) -> str:
    values = dict(node.latest)
    values["t_s"] = f"{t:.2f}"
    values["cmd_v"] = f"{forward:.2f}"
    values["cmd_w"] = f"{yaw:.2f}"
    cells = []
    for _, key, width, decimals in COLUMNS:
        raw = values.get(key)
        try:
            cells.append(f"{float(raw):{width}.{decimals}f}")
        except (TypeError, ValueError):
            cells.append("-" * width)
    return " ".join(cells) + f"  {node.message}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--controller", default="whole_body_controller",
                        help="controller node name whose ~/diagnostics to read")
    parser.add_argument("--topic", default="/cmd_vel")
    parser.add_argument("--rate", type=float, default=20.0, help="command rate in Hz")
    parser.add_argument("--print-every", type=float, default=0.5,
                        help="seconds between printed rows")
    parser.add_argument("--wait", type=float, default=60.0,
                        help="seconds to wait for the first diagnostics message")
    parser.add_argument("--phase", action="append", type=parse_phase,
                        help='"duration,forward,yaw[,height_rate]"; repeatable')
    parser.add_argument("--record", metavar="CSV",
                        help="record diagnostics, /joint_states and /imu_broadcaster/imu to this file")
    parser.add_argument("--record-from-start", action="store_true",
                        help="start the recording at the first diagnostics message instead of "
                             "after the settle time, so the engagement transient is captured "
                             "(phase is -1 until phase 0 begins)")
    parser.add_argument("--settle", type=float, default=2.0,
                        help="seconds to wait after the controller engages before phase 0, so the "
                             "release or reset transient stays out of the comparison")
    args = parser.parse_args()
    phases = args.phase or [parse_phase(p) for p in DEFAULT_PHASES]

    rclpy.init()
    node = DriveTest(args.controller, args.topic, args.rate)
    # Subscriptions are serviced on their own thread so 300 Hz joint states
    # and 50 Hz diagnostics are all recorded while the command loop paces
    # itself with sleeps.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    deadline = time.monotonic() + args.wait
    while node.samples == 0 and time.monotonic() < deadline:
        time.sleep(0.05)
    if node.samples == 0:
        print(f"no diagnostics from /{args.controller}/diagnostics within {args.wait:.0f} s",
              file=sys.stderr)
        executor.shutdown()
        spinner.join(timeout=2.0)
        rclpy.shutdown()
        return 1
    if args.record and args.record_from_start:
        node.phase = -1
        node.start_recording(args.record)
    # Balancers report engaged=1 once upright and balancing; wait for it so
    # the sequence does not start while the simulation is still being reset.
    while (node.latest.get("engaged") not in (None, "1")
           and time.monotonic() < deadline):
        time.sleep(0.05)
    if node.latest.get("engaged") == "0":
        print(f"controller never engaged ({node.message})", file=sys.stderr)
        executor.shutdown()
        spinner.join(timeout=2.0)
        rclpy.shutdown()
        return 1

    settle_until = time.monotonic() + args.settle
    while time.monotonic() < settle_until:
        node.publish(0.0, 0.0, 0.0)
        time.sleep(node.period)
    if args.record and node.recorder is None:
        node.start_recording(args.record)

    print(" ".join(f"{name:>{width}}" for name, _, width, _ in COLUMNS) + "  status")
    start = time.monotonic()
    next_print = 0.0
    worst_level = 0
    fell = False
    for index, (duration, forward, yaw, height_rate) in enumerate(phases):
        if node.recorder:
            node.recorder.phase = index
        phase_end = time.monotonic() + duration
        while time.monotonic() < phase_end:
            node.publish(forward, yaw, height_rate)
            t = time.monotonic() - start
            if t >= next_print:
                print(row(node, t, forward, yaw), flush=True)
                next_print += args.print_every
            worst_level = max(worst_level, node.level)
            fell = fell or "fall_latch" in node.message
            time.sleep(node.period)
    node.publish(0.0, 0.0, 0.0)
    time.sleep(0.1)
    executor.shutdown()
    spinner.join(timeout=2.0)
    print(f"finished: {node.samples} diagnostic samples, worst level {worst_level}, "
          f"fall latch {'YES' if fell else 'no'}")
    if node.recorder:
        node.recorder.close()
        print(f"recorded {node.recorder.rows} rows to {args.record}")
    node.destroy_node()
    rclpy.shutdown()
    return 2 if fell else 0


if __name__ == "__main__":
    raise SystemExit(main())
