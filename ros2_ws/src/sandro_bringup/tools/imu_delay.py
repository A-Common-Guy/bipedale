#!/usr/bin/env python3
"""IMU delay relative to the wheel encoders, from rocking the body by hand.

Lock both legs (spacer or zip ties), stand the robot on its wheels and block
the wheels so they cannot roll (chocks on both sides, or press them onto a
grippy mat). Rocking the body about the axle then turns each wheel encoder
by exactly minus the body pitch: a geometric identity, with no dynamics in
between, so the shift between the encoder and the IMU is the IMU delay minus
the encoder delay as the controller receives them.

    # drives unpowered or at zero torque; broadcasters only
    ros2 launch sandro_bringup sandro.launch.py hardware:=real arm_hardware:=true \\
        controller:=none teleop:=false
    python3 tools/imu_delay.py record --duration 30 --record rock.csv
    #   rock the body irregularly, +-10 to 20 deg, slow and fast (0.5 to 5 Hz)
    python3 tools/imu_delay.py analyze rock.csv

`record` writes /joint_states and the IMU like drive_test.py --record.
`analyze` needs only numpy: it takes each stream's fresh samples (the cycles
in which a new value arrived), averages the two wheels, band-passes encoder
and IMU identically (default 0.5 to 10 Hz) and finds the lag of maximum
correlation for the orientation (pitch against wheel position) and the gyro
(pitch rate against the differentiated wheel position). The sample rate is a
separate number (`imu_update_hz` in compare_runs.py, `sim_imu_rate_hz`);
`sim_imu_delay_s` is the lag plus `sim_joint_delay_s`.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))


def fresh(t, values):
    """Rows where any channel changed: the instants a new sample reached the controller."""
    values = np.asarray(values, dtype=float)
    if values.ndim == 1:
        values = values[:, None]
    ok = np.all(np.isfinite(values), axis=1) & np.isfinite(t)
    t, values = t[ok], values[ok]
    keep = np.concatenate([[True], np.any(np.diff(values, axis=0) != 0, axis=1)])
    return t[keep], values[keep]


def pitch_from_quaternion(qx, qy, qz, qw):
    return np.arcsin(np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0))


def band_pass(values, rate_hz, low_hz, high_hz):
    """Zero-phase FFT band-pass with cosine edges; the same filter on both
    signals leaves their relative lag untouched."""
    n = values.size
    spectrum = np.fft.rfft((values - values.mean()) * np.hanning(n))
    freqs = np.fft.rfftfreq(n, 1.0 / rate_hz)
    gain = np.ones_like(freqs)
    width_low, width_high = 0.5 * low_hz, 0.25 * high_hz
    gain[freqs < low_hz - width_low] = 0.0
    edge = (freqs >= low_hz - width_low) & (freqs < low_hz)
    gain[edge] = 0.5 - 0.5 * np.cos(np.pi * (freqs[edge] - low_hz + width_low) / width_low)
    gain[freqs > high_hz + width_high] = 0.0
    edge = (freqs > high_hz) & (freqs <= high_hz + width_high)
    gain[edge] = 0.5 + 0.5 * np.cos(np.pi * (freqs[edge] - high_hz) / width_high)
    return np.fft.irfft(spectrum * gain, n)


def lag_of(grid_hz, reference, response, min_lag_s, max_lag_s):
    """(lag s, signed normalised correlation) of response behind reference."""
    best_k, best_c, corr = 0, 0.0, {}
    for k in range(int(round(min_lag_s * grid_hz)), int(round(max_lag_s * grid_hz)) + 1):
        a = reference[max(0, -k):reference.size - max(0, k)]
        b = response[max(0, k):response.size - max(0, -k)]
        c = float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-12))
        corr[k] = c
        if abs(c) > abs(best_c):
            best_k, best_c = k, c
    left, centre, right = (abs(corr.get(best_k + i, 0.0)) for i in (-1, 0, 1))
    curvature = left - 2.0 * centre + right
    shift = 0.5 * (left - right) / curvature if curvature < 0 else 0.0
    return (best_k + shift) / grid_hz, best_c


def wheel_angle(joint):
    """Mean wheel position, or the single wheel that moved if only one did."""
    t, positions = fresh(joint["t_s"], np.column_stack(
        [joint["pos_right_wheel"], joint["pos_left_wheel"]]))
    right, left = positions[:, 0] - positions[0, 0], positions[:, 1] - positions[0, 1]
    spread = np.std(right), np.std(left)
    if min(spread) < 0.1 * max(spread):
        return t, right if spread[0] > spread[1] else left
    # Both wheels see the same body rotation; a sign flip in the joint frames
    # would make them cancel, so align the second to the first.
    sign = 1.0 if np.dot(right - right.mean(), left - left.mean()) >= 0 else -1.0
    return t, 0.5 * (right + sign * left)


def estimate(run: dict, band=(0.5, 10.0), grid_hz=1000.0, lag_range=(-0.05, 0.15),
             window=(None, None), segments=4) -> dict:
    imu, joint = run["imu"], run["joint"]
    t_enc, angle = wheel_angle(joint)
    t_gyro, gyro = fresh(imu["t_s"], imu["imu_gy"])
    t_quat, quat = fresh(imu["t_s"], np.column_stack(
        [imu[k] for k in ("imu_qx", "imu_qy", "imu_qz", "imu_qw")]))
    pitch = pitch_from_quaternion(*quat.T)
    # A finite difference is the velocity at the middle of its interval.
    t_vel = 0.5 * (t_enc[1:] + t_enc[:-1])
    velocity = np.diff(angle) / np.diff(t_enc)

    start = max(t_enc[0], t_gyro[0], t_quat[0]) + 0.2
    stop = min(t_enc[-1], t_gyro[-1], t_quat[-1]) - 0.2
    if window[0] is not None:
        start = max(start, window[0])
    if window[1] is not None:
        stop = min(stop, window[1])
    if stop - start < 4.0:
        raise ValueError("need at least 4 s where IMU and encoder samples overlap")
    grid = np.arange(start, stop, 1.0 / grid_hz)
    signals = {
        "orientation": (np.interp(grid, t_enc, angle), np.interp(grid, t_quat, pitch)),
        "gyro": (np.interp(grid, t_vel, velocity), np.interp(grid, t_gyro, gyro[:, 0])),
    }
    inside = (t_quat >= start) & (t_quat <= stop)
    out = {"duration_s": float(stop - start),
           "imu_hz": float(inside.sum() / (stop - start)),
           "encoder_hz": float(((t_enc >= start) & (t_enc <= stop)).sum() / (stop - start)),
           "pitch_amplitude_rad": float(np.std(pitch[inside]))}
    for name, (reference, response) in signals.items():
        reference = band_pass(reference, grid_hz, *band)
        response = band_pass(response, grid_hz, *band)
        lag, corr = lag_of(grid_hz, reference, response, *lag_range)
        pieces = [lag_of(grid_hz, reference[chunk], response[chunk], *lag_range)[0]
                  for chunk in np.array_split(np.arange(grid.size), segments)
                  if chunk.size > grid_hz]
        out[name] = {"lag_s": lag, "corr": corr,
                     "segment_sd_s": float(np.std(pieces)) if len(pieces) > 1 else math.nan}
    return out


def analyze(args) -> int:
    from compare_runs import load

    run = load(args.csv)
    if "imu" not in run or "joint" not in run:
        print("recording needs imu and joint rows (imu_delay.py record or drive_test.py --record)",
              file=sys.stderr)
        return 1
    result = estimate(run, (args.low_hz, args.high_hz), window=(args.start, args.stop))
    print(f"{args.csv}: {result['duration_s']:.1f} s, IMU {result['imu_hz']:.0f} Hz, "
          f"encoder {result['encoder_hz']:.0f} Hz, pitch spread {result['pitch_amplitude_rad']:.2f} rad")
    print(f"{'channel':13s}{'lag ms':>8s}{'+-seg':>8s}{'corr':>8s}")
    for name in ("orientation", "gyro"):
        r = result[name]
        print(f"{name:13s}{1e3 * r['lag_s']:8.1f}{1e3 * r['segment_sd_s']:8.1f}{r['corr']:8.2f}")
    if result["pitch_amplitude_rad"] < 0.05:
        print("warning: the body hardly moved; rock it +-10 to 20 deg", file=sys.stderr)
    weak = [n for n in ("orientation", "gyro") if abs(result[n]["corr"]) < 0.9]
    if weak:
        print(f"warning: weak correlation for {', '.join(weak)}: the wheels must not roll and the "
              "legs must not move relative to the body", file=sys.stderr)
    lag = result["orientation"]["lag_s"]
    print(f"\nIMU behind encoder: {1e3 * lag:.1f} ms (orientation), "
          f"{1e3 * result['gyro']['lag_s']:.1f} ms (gyro)")
    print(f"sim_imu_delay_s ~= {lag + args.sim_joint_delay_s:.3f}  (orientation lag + "
          f"sim_joint_delay_s {args.sim_joint_delay_s}); the simulation delays every IMU channel "
          "by this one value")
    return 0


def record(args) -> int:
    import time

    import rclpy
    from drive_test import Recorder
    from rclpy.node import Node
    from sensor_msgs.msg import Imu, JointState

    rclpy.init()
    node = Node("imu_delay_record")
    recorder = Recorder(args.record, [])
    recorder.phase = 0
    node.create_subscription(JointState, "/joint_states", recorder.joints, 50)
    node.create_subscription(Imu, "/imu_broadcaster/imu", recorder.imu, 50)
    print(f"recording {args.duration:.0f} s to {args.record}: rock the body now")
    try:
        end = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        recorder.close()
        node.destroy_node()
        rclpy.shutdown()
    print(f"{recorder.rows} rows; now: python3 tools/imu_delay.py analyze {args.record}")
    return 0 if recorder.rows else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    rec = sub.add_parser("record", help="record joint states and IMU (needs ROS)")
    rec.add_argument("--duration", type=float, default=30.0)
    rec.add_argument("--record", required=True, metavar="CSV")
    rec.set_defaults(func=record)
    ana = sub.add_parser("analyze", help="estimate the delay from a recording")
    ana.add_argument("csv")
    ana.add_argument("--low-hz", type=float, default=0.5)
    ana.add_argument("--high-hz", type=float, default=10.0)
    ana.add_argument("--start", type=float, help="ignore samples before this t_s")
    ana.add_argument("--stop", type=float, help="ignore samples after this t_s")
    ana.add_argument("--sim-joint-delay-s", type=float, default=0.001)
    ana.set_defaults(func=analyze)
    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
