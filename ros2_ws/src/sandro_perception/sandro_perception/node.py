"""height_profile: the depth camera's images into the terrain profile ahead of each wheel.

Publishes `/terrain_profile` (std_msgs/Float32MultiArray: the right track's heights, the left
track's, then the same 2 x k values as known flags, 1 or 0; height_profile.py has the
definition) at `publish_hz`, and `~/diagnostics` twice a second. Nothing reads the profile yet
and this node commands nothing: it can run beside any controller.

Kept light, because on the Pi it shares the computer with the 300 Hz control loop: the two
300 Hz topics (joint states, IMU) are not given to the executor; a 50 Hz timer takes whatever
has queued and uses the newest, so they cost no wake-up per message. On the Pi 4 a depth
image costs about 8 ms, 15 times a second, and a state sample under 1 ms, 50 times.
"""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState
from std_msgs.msg import Float32MultiArray

from .height_profile import HeightProfile, Layout, Mount, Settings

LEGS = ("right_motor_inner", "right_motor_outer", "left_motor_inner", "left_motor_outer")
STATE_HZ = 50.0     # joint states and IMU into the odometry; images are placed between samples
SAMPLED = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)


def stamp_s(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def depth_image(msg: Image):
    """(array, metres per unit) of a 16UC1 (millimetres) or 32FC1 (metres) image, without
    copying it; (None, 0) for anything else."""
    if msg.encoding in ("16UC1", "mono16"):
        raw = np.frombuffer(msg.data, dtype=np.dtype(np.uint16).newbyteorder(">" if msg.is_bigendian else "<"))
        return raw.reshape(msg.height, msg.step // 2)[:, :msg.width], 1e-3
    if msg.encoding == "32FC1":
        raw = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.step // 4)[:, :msg.width]
        return np.nan_to_num(raw, nan=0.0, posinf=0.0, neginf=0.0), 1.0
    return None, 0.0


class HeightProfileNode(Node):
    def __init__(self) -> None:
        super().__init__("height_profile")
        p = lambda name, default: self.declare_parameter(name, default).value
        cad = Mount()
        mount = Mount(x_m=p("mount.x_m", cad.x_m), y_m=p("mount.y_m", cad.y_m), z_m=p("mount.z_m", cad.z_m),
                      pitch_rad=p("mount.pitch_rad", cad.pitch_rad), roll_rad=p("mount.roll_rad", cad.roll_rad),
                      yaw_rad=p("mount.yaw_rad", cad.yaw_rad))
        layout = Layout(distances_m=tuple(p("layout.distances_m", list(Layout().distances_m))),
                        half_track_m=p("layout.half_track_m", Layout().half_track_m))
        defaults = Settings()
        self.profile = HeightProfile(Settings(
            layout=layout, mount=mount, cell_m=p("map.cell_m", defaults.cell_m),
            max_range_m=p("depth.max_range_m", defaults.max_range_m),
            min_range_m=p("depth.min_range_m", defaults.min_range_m),
            stride=p("depth.stride", defaults.stride), max_age_s=p("map.max_age_s", defaults.max_age_s)))
        # The IMU's roll when the body is level, as tasks.roll.imu_offset_rad of the controllers.
        self.imu_roll_offset = p("imu.roll_offset_rad", 0.0)
        self.intrinsics = None
        self.latest = {"joints": None, "imu": None}
        self.depth_stamps: list[float] = []
        self.refreshed = self.process_ms = self.image_age_ms = 0.0
        self.clock_warned = False

        # The fast topics: on a node of their own that no executor spins.
        # (Its own name: the launch file's name remapping is for this node only.)
        self.sampler = rclpy.create_node("height_profile_sampler", context=self.context,
                                         use_global_arguments=False)
        self.sampled = {
            "joints": (self.sampler.create_subscription(JointState, p("topics.joint_states", "/joint_states"),
                                                        lambda _: None, SAMPLED, raw=True), JointState),
            "imu": (self.sampler.create_subscription(Imu, p("topics.imu", "/imu_broadcaster/imu"),
                                                     lambda _: None, SAMPLED, raw=True), Imu)}
        depth_topic = p("topics.depth", "/camera/camera/depth/image_rect_raw")
        self.create_subscription(Image, depth_topic, self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, p("topics.camera_info", "/camera/camera/depth/camera_info"),
                                 self.on_info, qos_profile_sensor_data)
        self.publisher = self.create_publisher(Float32MultiArray, p("topics.profile", "/terrain_profile"), 5)
        self.diagnostics = self.create_publisher(DiagnosticArray, "~/diagnostics", 5)
        self.publish_every = max(1, round(STATE_HZ / p("publish_hz", 25.0)))
        self.tick = 0
        self.create_timer(1.0 / STATE_HZ, self.on_timer)
        self.create_timer(0.5, self.on_diagnostics)
        self.get_logger().info(
            f"terrain profile from {depth_topic}: {len(layout.distances_m)} points per wheel track, "
            f"{layout.distances_m[0]:g}-{layout.distances_m[-1]:g} m ahead; camera {mount.x_m:g} m ahead of "
            f"the IMU, pitched {math.degrees(mount.pitch_rad):.1f} deg down")

    def take_latest(self, name: str):
        subscription, kind = self.sampled[name]
        newest = None
        for _ in range(50):
            with subscription.handle:
                taken = subscription.handle.take_message(subscription.msg_type, subscription.raw)
            if taken is None:
                break
            newest = taken[0] if isinstance(taken, tuple) else taken
        if newest is not None:
            self.latest[name] = deserialize_message(newest, kind)
        return self.latest[name]

    def on_timer(self) -> None:
        joints, imu = self.take_latest("joints"), self.take_latest("imu")
        if joints is None or imu is None:
            return
        try:
            index = {name: joints.name.index(name) for name in (*LEGS, "right_wheel", "left_wheel")}
        except ValueError:
            return
        q = imu.orientation
        if q.w == 0.0 and q.x == 0.0 and q.y == 0.0 and q.z == 0.0:
            return
        # Take the mounting roll out of the attitude: rotate by -offset about the body's x.
        half = -0.5 * self.imu_roll_offset
        c, s = math.cos(half), math.sin(half)
        attitude = (q.x * c + q.w * s, q.y * c + q.z * s, q.z * c - q.y * s, q.w * c - q.x * s)
        self.profile.update_state(
            stamp_s(joints.header.stamp), {name: joints.position[index[name]] for name in LEGS},
            (joints.velocity[index["right_wheel"]], joints.velocity[index["left_wheel"]]),
            attitude, imu.angular_velocity.y)
        self.tick += 1
        if self.tick % self.publish_every == 0:
            heights, known = self.profile.profile()
            self.publisher.publish(Float32MultiArray(
                data=[*heights.reshape(-1).tolist(), *known.reshape(-1).astype(float).tolist()]))

    def on_info(self, msg: CameraInfo) -> None:
        self.intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def on_depth(self, msg: Image) -> None:
        if self.intrinsics is None or self.profile.time is None:
            return
        depth, scale = depth_image(msg)
        if depth is None:
            self.get_logger().error(f"depth encoding {msg.encoding} is not 16UC1 or 32FC1", once=True)
            return
        stamp = stamp_s(msg.header.stamp)
        age = self.profile.time - stamp
        if not -0.2 < age < 0.5:
            # The camera's clock is not the controller manager's: assume a usual latency.
            if not self.clock_warned:
                self.clock_warned = True
                self.get_logger().warn(f"depth stamps are {age:+.2f} s from the joint states': using "
                                       "arrival time minus 50 ms")
            stamp, age = self.profile.time - 0.05, 0.05
        started = time.perf_counter()
        self.refreshed = self.profile.add_depth(stamp, depth, *self.intrinsics, scale=scale)
        self.process_ms = 1e3 * (time.perf_counter() - started)
        self.image_age_ms = 1e3 * age
        self.depth_stamps = [t for t in self.depth_stamps if t > stamp - 2.0] + [stamp]

    def on_diagnostics(self) -> None:
        status = DiagnosticStatus(name="height_profile", hardware_id="d435")
        values = {"state": 1.0 if self.profile.time is not None else 0.0,
                  "depth_hz": max(len(self.depth_stamps) - 1, 0) / 2.0,
                  "image_age_ms": self.image_age_ms, "process_ms": self.process_ms,
                  "cells_refreshed": float(self.refreshed)}
        if self.profile.time is not None:
            heights, known = self.profile.profile()
            values.update(points_known=float(known.sum()), points=float(known.size),
                          highest_m=float(heights.max()), lowest_m=float(heights.min()))
        fit = self.profile.last_fit
        if fit is not None:
            # Standing on a flat floor these are the mount's errors: add floor_pitch_deg to
            # mount.pitch_rad (in radians) to zero it.
            values.update(floor_pitch_deg=math.degrees(fit[0]), floor_roll_deg=math.degrees(fit[1]),
                          floor_offset_mm=1e3 * fit[2])
        fresh = values["depth_hz"] > 5.0
        status.level = DiagnosticStatus.OK if fresh else DiagnosticStatus.WARN
        status.message = "running" if fresh else "no depth images"
        status.values = [KeyValue(key=k, value=f"{v:.4g}") for k, v in values.items()]
        message = DiagnosticArray(status=[status])
        message.header.stamp = self.get_clock().now().to_msg()
        self.diagnostics.publish(message)


def main() -> None:
    rclpy.init()
    node = HeightProfileNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sampler.destroy_node()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
