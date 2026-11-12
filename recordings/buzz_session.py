"""The yaw buzz of every controller run in one recording, split automatically.

    python3 buzz_session.py BAG [--trim 0.3] [--min 1.0] [--csv out.csv]

A run is a stretch where an RL policy is in control (its diagnostics' phase
is 1) or the whole-body controller is engaged; the stand-up move and the limp
phase after a trip are not runs. Record every controller's diagnostics with
the joints and the IMU, for example:

    ros2 bag record -s mcap -o session /joint_states /imu_broadcaster/imu \\
        /health_broadcaster/dynamic_joint_states /cmd_vel --regex '.*/diagnostics'

Per run, 5-15 Hz RMS of the gyro (yaw, pitch, roll), of the wheels opposed
(the yaw mode) and together, and of the busiest leg motor; the peak of the
opposed wheel motion; the 60+ Hz torque (mean over the joints); the
oscillation watchdog's highest values; and the largest speed and yaw-rate
references, to tell standing from driving. Reads the bag with rosbag2_py
when ROS is sourced, otherwise with the mcap_ros2 package
(pip install mcap-ros2-support numpy).
"""
import argparse
import csv
import sys
from pathlib import Path

import numpy as np

FS = 300.0
WHEELS = ("right_wheel", "left_wheel")
LEGS = ("right_motor_inner", "right_motor_outer", "left_motor_inner", "left_motor_outer")
POLICY_PHASE = 1
OSCILLATION_BIT = 1 << 6


def messages(bag):
    """(topic, stamp_s, message) for the topics used here, header stamps where set."""
    wanted = lambda topic: topic in ("/joint_states", "/imu_broadcaster/imu") or topic.endswith("/diagnostics")
    path = Path(bag)
    try:
        from rclpy.serialization import deserialize_message
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
        from rosidl_runtime_py.utilities import get_message
    except ImportError:
        from mcap_ros2.reader import read_ros2_messages
        files = sorted(path.glob("*.mcap")) if path.is_dir() else [path]
        for file in files:
            for m in read_ros2_messages(str(file)):
                if wanted(m.channel.topic):
                    yield m.channel.topic, stamp(m.ros_msg, m.log_time_ns), m.ros_msg
        return
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(path), storage_id="mcap"), ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    while reader.has_next():
        topic, data, received = reader.read_next()
        if wanted(topic):
            msg = deserialize_message(data, get_message(types[topic]))
            yield topic, stamp(msg, received), msg


def stamp(msg, received_ns):
    s = msg.header.stamp
    return s.sec + s.nanosec * 1e-9 if (s.sec or s.nanosec) else received_ns * 1e-9


def load(bag):
    joints, imu, diag = [], [], {}
    for topic, t, msg in messages(bag):
        if topic == "/joint_states":
            joints.append((t, dict(zip(msg.name, zip(msg.velocity, msg.effort)))))
        elif topic == "/imu_broadcaster/imu":
            g = msg.angular_velocity
            imu.append((t, (g.x, g.y, g.z)))
        else:
            values = {}
            for status in msg.status:
                for item in status.values:
                    try:
                        values[item.key] = float(item.value)
                    except ValueError:
                        pass
            diag.setdefault(topic.strip("/").split("/")[0], []).append((t, values))
    return joints, imu, diag


def runs(diag, gap=0.2):
    """(controller, start, end, ended) for every stretch in control."""
    found = []
    for name, rows in diag.items():
        rows.sort(key=lambda r: r[0])
        in_control = (lambda v: v.get("engaged") == 1) if name == "whole_body_controller" \
            else (lambda v: v.get("phase") == POLICY_PHASE)
        start = last = None
        for i, (t, values) in enumerate(rows):
            if in_control(values) and start is not None and t - last <= gap:
                last = t
                continue
            if start is not None:
                found.append((name, start, last, ending(rows, i, last, gap)))
                start = None
            if in_control(values):
                start = last = t
        if start is not None:
            found.append((name, start, last, "recording ended"))
    return sorted(found, key=lambda r: r[1])


def ending(rows, i, last, gap):
    if i >= len(rows) or rows[i][0] - last > gap:
        return "switched off"
    values = rows[i][1]
    if int(values.get("status_mask", 0)) & OSCILLATION_BIT:
        return "OSCILLATION trip"
    return "fallen" if values.get("phase") == 2 else "left control"


def band(x, lo, hi):
    f = np.fft.rfftfreq(len(x), 1 / FS)
    spectrum = np.fft.rfft(x - x.mean())
    spectrum[(f < lo) | (f > hi)] = 0
    return np.fft.irfft(spectrum, len(x)).std()


def peak(x, lo=3.0, hi=20.0):
    f = np.fft.rfftfreq(len(x), 1 / FS)
    a = np.abs(np.fft.rfft((x - x.mean()) * np.hanning(len(x))))
    keep = (f >= lo) & (f <= hi)
    return f[keep][np.argmax(a[keep])]


def analyse(joints, imu, diag, run, trim):
    name, start, end, ended = run
    grid = np.arange(start + trim, end, 1 / FS)
    tj = np.array([s[0] for s in joints])
    ti = np.array([s[0] for s in imu])
    vel = {n: np.interp(grid, tj, [s[1][n][0] for s in joints]) for n in WHEELS + LEGS}
    eff = {n: np.interp(grid, tj, [s[1][n][1] for s in joints]) for n in WHEELS + LEGS}
    gyro = [np.interp(grid, ti, [s[1][k] for s in imu]) for k in range(3)]
    right, left = vel["right_wheel"], vel["left_wheel"]
    rows = [v for t, v in diag[name] if start <= t <= end]
    top = lambda key: max((abs(v.get(key, 0.0)) for v in rows), default=float("nan"))
    return {
        "controller": name, "start_s": start, "duration_s": end - start, "ended": ended,
        "yaw": band(gyro[2], 5, 15), "pitch": band(gyro[1], 5, 15), "roll": band(gyro[0], 5, 15),
        # Same-sign wheel velocities drive the robot forward (2026-11-09 recordings).
        "wheels_opposed": band((right - left) / 2, 5, 15),
        "wheels_together": band((right + left) / 2, 5, 15),
        "leg_max": max(band(vel[n], 5, 15) for n in LEGS),
        "peak_hz": peak((right - left) / 2),
        "torque_60hz": float(np.mean([band(eff[n], 60, FS / 2) for n in WHEELS + LEGS])),
        "watchdog_leg": top("oscillation_leg_rad_s"), "watchdog_wheel": top("oscillation_wheel_rad_s"),
        "speed_ref": top("velocity_reference_m_s"), "yaw_ref": top("yaw_rate_reference_rad_s"),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bag")
    parser.add_argument("--trim", type=float, default=0.3, help="seconds skipped after each takeover")
    parser.add_argument("--min", type=float, default=1.0, help="shortest run analysed, seconds")
    parser.add_argument("--csv", help="also write the table here")
    args = parser.parse_args()
    joints, imu, diag = load(args.bag)
    if not joints or not imu:
        sys.exit("no /joint_states or /imu_broadcaster/imu in the bag")
    origin = joints[0][0]
    results = [analyse(joints, imu, diag, run, args.trim) for run in runs(diag)
               if run[2] - run[1] - args.trim >= args.min]
    print(f"{'at s':>6} {'controller':30s} {'dur s':>6} {'yaw':>6} {'pitch':>6} {'roll':>6} "
          f"{'opp':>6} {'tog':>6} {'leg':>6} {'peak':>5} {'60Hz':>6} {'wd leg':>6} {'wd wh':>6} "
          f"{'v ref':>5} {'w ref':>5}  ended")
    print(f"{'':6} {'(5-15 Hz RMS: gyro rad/s, wheel and leg velocity rad/s; 60+ Hz torque Nm)':78s}")
    for r in results:
        print(f"{r['start_s'] - origin:6.1f} {r['controller']:30s} {r['duration_s']:6.1f} {r['yaw']:6.3f} "
              f"{r['pitch']:6.3f} {r['roll']:6.3f} {r['wheels_opposed']:6.2f} {r['wheels_together']:6.2f} "
              f"{r['leg_max']:6.2f} {r['peak_hz']:5.1f} {r['torque_60hz']:6.3f} {r['watchdog_leg']:6.2f} "
              f"{r['watchdog_wheel']:6.2f} {r['speed_ref']:5.2f} {r['yaw_ref']:5.2f}  {r['ended']}")
    if args.csv:
        with open(args.csv, "w", newline="") as out:
            writer = csv.DictWriter(out, fieldnames=list(results[0]) if results else ["controller"])
            writer.writeheader()
            writer.writerows(results)


if __name__ == "__main__":
    main()
