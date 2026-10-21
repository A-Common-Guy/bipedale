"""IMU update rate, and which gyro axis matches the orientation's pitch/roll rate."""
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import Imu

bag, t0, t1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
reader = SequentialReader()
reader.open(StorageOptions(uri=bag, storage_id="mcap"), ConverterOptions("", ""))
rows = []
while reader.has_next():
    topic, data, stamp = reader.read_next()
    if topic != "/imu_broadcaster/imu" or not t0 <= stamp * 1e-9 <= t1:
        continue
    m = deserialize_message(data, Imu)
    q = m.orientation
    rows.append([stamp * 1e-9, q.w, q.x, q.y, q.z, m.angular_velocity.x, m.angular_velocity.y,
                 m.angular_velocity.z])
a = np.array(rows)
t, w, x, y, z = a[:, 0], a[:, 1], a[:, 2], a[:, 3], a[:, 4]
gyro = a[:, 5:8]
changed = np.any(np.diff(a[:, 1:], axis=0) != 0, axis=1)
print(f"{len(a)} messages, {changed.sum() / (t[-1] - t[0]):.0f} Hz distinct samples")
roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
for name, angle in (("roll", roll), ("pitch", pitch)):
    rate = np.gradient(np.unwrap(angle), t)
    corr = [np.corrcoef(rate, gyro[:, k])[0, 1] for k in range(3)]
    gain = [np.dot(rate, gyro[:, k]) / np.dot(gyro[:, k], gyro[:, k]) for k in range(3)]
    print(f"d{name}/dt vs gyro x,y,z: corr " + " ".join(f"{c:+.2f}" for c in corr)
          + "  gain " + " ".join(f"{g:+.2f}" for g in gain))
    # Lag of the orientation behind the gyro, by cross-correlation.
    k = int(np.argmax(np.abs(corr)))
    g = gyro[:, k] - gyro[:, k].mean()
    r = rate - rate.mean()
    lags = range(-30, 31)
    best = max(lags, key=lambda L: np.sign(corr[k]) * np.dot(np.roll(g, L), r))
    print(f"  orientation lags the gyro by {best / 300 * 1e3:+.1f} ms")
