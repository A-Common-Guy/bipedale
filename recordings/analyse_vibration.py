"""Spectra of the wheels and the policy actions during the rl_flat_level run."""
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

bag = sys.argv[1]
t0, t1 = float(sys.argv[2]), float(sys.argv[3])
reader = SequentialReader()
reader.open(StorageOptions(uri=bag, storage_id="mcap"), ConverterOptions("", ""))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}
js, diag, imu = [], [], []
while reader.has_next():
    topic, data, stamp = reader.read_next()
    t = stamp * 1e-9
    if not t0 <= t <= t1:
        continue
    msg = deserialize_message(data, get_message(types[topic]))
    if topic == "/joint_states":
        js.append((t, dict(zip(msg.name, zip(msg.position, msg.velocity, msg.effort)))))
    elif topic == "/rl_flat_level/diagnostics":
        values = {v.key: float(v.value) for s in msg.status for v in s.values
                  if v.value.replace(".", "", 1).replace("-", "", 1).replace("e", "", 1).isdigit()}
        diag.append((t, values))
    elif topic == "/imu_broadcaster/imu":
        imu.append((t, msg.angular_velocity.y, msg.angular_velocity.x))


def spectrum(t, x, label):
    t, x = np.asarray(t), np.asarray(x)
    fs = (len(t) - 1) / (t[-1] - t[0])
    x = x - x.mean()
    f = np.fft.rfftfreq(len(x), 1 / fs)
    a = np.abs(np.fft.rfft(x * np.hanning(len(x))))
    top = np.argsort(a[1:])[-3:][::-1] + 1
    print(f"{label:28s} fs {fs:6.1f} Hz  rms {x.std():8.4f}  peaks "
          + ", ".join(f"{f[i]:5.1f} Hz" for i in top))


print(f"joint_states {len(js)}, diagnostics {len(diag)}, imu {len(imu)}")
tj = [s[0] for s in js]
for j in ["right_wheel", "left_wheel", "right_motor_inner", "left_motor_inner"]:
    spectrum(tj, [s[1][j][1] for s in js], f"{j} velocity")
    spectrum(tj, [s[1][j][2] for s in js], f"{j} effort")
td = [s[0] for s in diag]
for i in range(6):
    spectrum(td, [s[1].get(f"action_{i}", 0.0) for s in diag], f"action_{i}")
spectrum([s[0] for s in imu], [s[1] for s in imu], "gyro pitch")
print("\nfirst 25 joint samples after engage: t, right/left wheel velocity, effort")
for t, s in js[:25]:
    print(f"{t - t0:6.3f}  {s['right_wheel'][1]:+7.2f} {s['left_wheel'][1]:+7.2f}  "
          f"{s['right_wheel'][2]:+6.2f} {s['left_wheel'][2]:+6.2f}")
print("\nactions (wheel 2, 5) at 50 Hz, first 20:")
for t, v in diag[:20]:
    print(f"{t - t0:6.3f}  {v.get('action_2', 0):+6.2f} {v.get('action_5', 0):+6.2f}  "
          f"pitch {v.get('pitch_rad', 0):+.3f}")
