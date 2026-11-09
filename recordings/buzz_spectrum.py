"""Where the high-frequency drive buzz is: per joint and IMU axis, band RMS and peak.

    python3 buzz_spectrum.py BAG [START_S END_S]

Times are seconds from each stream's first message. Uses the header stamps (the control
loop's time), not the bag's receive times, whose jitter smears everything above
a few tens of Hz. With state_source:=mit the joints are sampled at 300 Hz, so
the buzz band ends at 150 Hz; a tone above it folds back into the band.
"""
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

BANDS = {"balance 5-15 Hz": (5.0, 15.0), "mid 15-60 Hz": (15.0, 60.0), "buzz 60+ Hz": (60.0, 1e9)}


def read(bag):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id="mcap"), ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    joints, imu = [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in ("/joint_states", "/imu_broadcaster/imu"):
            continue
        msg = deserialize_message(data, get_message(types[topic]))
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if topic == "/joint_states":
            joints.append((t, dict(zip(msg.name, zip(msg.velocity, msg.effort)))))
        else:
            a, g = msg.linear_acceleration, msg.angular_velocity
            imu.append((t, (a.x, a.y, a.z, g.x, g.y, g.z)))
    return joints, imu


def welch(t, x, segment=256):
    """Amplitude spectral density on a uniform grid; repeated stamps are dropped."""
    t, x = np.asarray(t), np.asarray(x, dtype=float)
    keep = np.concatenate(([True], np.diff(t) > 0))
    t, x = t[keep], x[keep]
    fs = (len(t) - 1) / (t[-1] - t[0])
    x = np.interp(np.arange(t[0], t[-1], 1 / fs), t, x)
    segment = min(segment, len(x))
    window = np.hanning(segment)
    starts = range(0, len(x) - segment + 1, segment // 2)
    power = np.mean([np.abs(np.fft.rfft((x[s:s + segment] - x[s:s + segment].mean()) * window)) ** 2
                     for s in starts], axis=0)
    power *= 2 / (fs * (window ** 2).sum())
    return np.fft.rfftfreq(segment, 1 / fs), power, fs


def row(label, t, x):
    f, p, fs = welch(t, x)
    df = f[1] - f[0]
    cells = []
    for lo, hi in BANDS.values():
        band = (f >= lo) & (f < hi)
        cells.append(np.sqrt(p[band].sum() * df) if band.any() else float("nan"))
    buzz = f >= BANDS["buzz 60+ Hz"][0]
    peak = f[buzz][np.argmax(p[buzz])] if buzz.any() else float("nan")
    print(f"{label:34s} {fs:5.0f} " + " ".join(f"{c:13.4f}" for c in cells) + f" {peak:8.0f}")


def main():
    joints, imu = read(sys.argv[1])
    lo, hi = (float(sys.argv[2]), float(sys.argv[3])) if len(sys.argv) > 3 else (0.0, 1e12)
    # Each stream from its own first stamp: the IMU's stamps need not share the joints' origin.
    joints = [s for s in joints if lo <= s[0] - joints[0][0] <= hi]
    imu = [s for s in imu if lo <= s[0] - imu[0][0] <= hi]
    print(f"{len(joints)} joint samples, {len(imu)} IMU samples, "
          f"{joints[-1][0] - joints[0][0]:.1f} s; RMS per band, peak above 60 Hz")
    print(f"{'signal':34s} {'fs':>5s} " + " ".join(f"{b:>13s}" for b in BANDS) + f" {'peak Hz':>8s}")
    t = [s[0] for s in joints]
    for name in joints[0][1]:
        row(f"{name} velocity rad/s", t, [s[1][name][0] for s in joints])
    for name in joints[0][1]:
        row(f"{name} effort Nm", t, [s[1][name][1] for s in joints])
    t = [s[0] for s in imu]
    for i, axis in enumerate(["accel x m/s2", "accel y m/s2", "accel z m/s2",
                              "gyro x rad/s", "gyro y rad/s", "gyro z rad/s"]):
        row(f"imu {axis}", t, [s[1][i] for s in imu])


if __name__ == "__main__":
    main()
