"""Delay of the IMU behind the joints: body pitch acceleration answers the wheel torque at once."""
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import Imu, JointState

bag, t0, t1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
reader = SequentialReader()
reader.open(StorageOptions(uri=bag, storage_id="mcap"), ConverterOptions("", ""))
imu, js = [], []
while reader.has_next():
    topic, data, stamp = reader.read_next()
    if not t0 <= stamp * 1e-9 <= t1:
        continue
    if topic == "/imu_broadcaster/imu":
        m = deserialize_message(data, Imu)
        imu.append((m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, stamp * 1e-9,
                    m.angular_velocity.y))
    elif topic == "/joint_states":
        m = deserialize_message(data, JointState)
        e = dict(zip(m.name, m.effort))
        v = dict(zip(m.name, m.velocity))
        js.append((m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, stamp * 1e-9,
                   e["right_wheel"] + e["left_wheel"], v["right_wheel"] + v["left_wheel"]))
imu, js = np.array(imu), np.array(js)
print(f"header-vs-receive offset: imu {np.median(imu[:, 1] - imu[:, 0]) * 1e3:.1f} ms, "
      f"joints {np.median(js[:, 1] - js[:, 0]) * 1e3:.1f} ms")
t = np.arange(max(imu[0, 0], js[0, 0]), min(imu[-1, 0], js[-1, 0]), 0.001)
gyro = np.interp(t, imu[:, 0], imu[:, 2])
torque = np.interp(t, js[:, 0], js[:, 2])
wheel_v = np.interp(t, js[:, 0], js[:, 3])
pitch_acc = np.gradient(gyro, t)
wheel_acc = np.gradient(wheel_v, t)


def lag(a, b, label):
    a, b = a - a.mean(), b - b.mean()
    lags = np.arange(-80, 81)
    c = [np.dot(a[80 + L: len(a) - 80 + L], b[80: len(b) - 80]) for L in lags]
    c = np.array(c) / (np.std(a) * np.std(b) * (len(a) - 160))
    k = int(np.argmax(np.abs(c)))
    print(f"{label}: best |corr| {abs(c[k]):.2f} (sign {np.sign(c[k]):+.0f}) "
          f"with the first lagging the second by {lags[k]:+d} ms")


lag(pitch_acc, torque, "pitch acceleration vs wheel torque")
lag(gyro, wheel_v, "pitch rate vs wheel velocity")
lag(wheel_acc, torque, "wheel acceleration vs wheel torque (joint self-check)")
