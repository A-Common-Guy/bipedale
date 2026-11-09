"""IMU sample rate and the drivers' sensor ages (health_broadcaster).

    python3 buzz_timing.py BAG
"""
import sys
import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

reader = SequentialReader()
reader.open(StorageOptions(uri=sys.argv[1], storage_id="mcap"), ConverterOptions("", ""))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}
gyro, ages = [], {}
while reader.has_next():
    topic, data, _ = reader.read_next()
    if topic == "/imu_broadcaster/imu":
        m = deserialize_message(data, get_message(types[topic]))
        gyro.append((m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))
    elif topic == "/health_broadcaster/dynamic_joint_states":
        m = deserialize_message(data, get_message(types[topic]))
        for name, iv in zip(m.joint_names, m.interface_values):
            for key, value in zip(iv.interface_names, iv.values):
                if key in ("age", "feedback_age", "rate_hz", "sample_age"):
                    ages.setdefault(f"{name}/{key}", []).append(value)
g = np.array(gyro)
changes = np.count_nonzero(np.any(np.diff(g, axis=0) != 0, axis=1))
print(f"IMU: {len(g)} messages, new gyro sample in {changes / len(g) * 100:.0f} % of them")
for key, v in sorted(ages.items()):
    v = np.array(v)
    print(f"{key:36s} median {np.median(v) * 1e3:7.1f} ms  p95 {np.percentile(v, 95) * 1e3:7.1f} ms" if "age" in key
          else f"{key:36s} median {np.median(v):7.1f}")
