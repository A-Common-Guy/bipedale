"""Where the body's yaw oscillation comes from, per controller run of a recording.

    python3 yaw_closure.py BAG [CONTROLLER] [--lo 5 --hi 15]

Two ways the body can yaw, as yaw rates (rad/s):

    rolling   the wheels' differential speed at the motor encoders * r / track
    legs      the axle line turning under the body: one axle forward of the other,
              from both five-bars' fore-aft (planar x) motion at the leg encoders

Band-passed to the oscillation band, the gyro's yaw rate is fitted as
a * rolling + b * legs (least squares, the IMU's delay searched over 0-30 ms).
A rigid robot gives |a| or |b| near 1 and most of the variance explained;
|b| well above 1 means the legs move more than their motor encoders show (play
or wind-up in the leg gears), and variance left unexplained points at the
tyres, the wheel gears behind their encoders or the frame.

2026-11-12, the 9 Hz mode at full strength: the gyro follows the legs' term in
phase at twice its size (b = 2.0, 91 % explained by the legs alone), and the
rotors' rolling adds almost nothing (a = -0.2). Springs in series deflect in
phase, so this splits the yaw compliance into the legs' stiffness as their
encoders see it (half) and the rest behind encoders and tyres (half), and
cannot tell the second half apart: the drive-train bench tests can.
"""
import argparse
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent)]
from buzz_session import messages, runs  # noqa: E402

BASE, UPPER, LOWER = 0.09, 0.08, 0.13          # five-bar (controllers.yaml model)
RADIUS, TRACK, FS = 0.0672, 0.304, 300.0
FORWARD = {"right": 1.0, "left": -1.0}          # body-forward sign of planar +x
LEGS = {"right": ("right_motor_inner", "right_motor_outer"),
        "left": ("left_motor_inner", "left_motor_outer")}


def axle_x(theta_inner, theta_outer):
    """Planar x of the axle (sandro_controllers kinematics.cpp forward_kinematics)."""
    p = UPPER * (np.cos(theta_outer) - np.cos(theta_inner))
    q = BASE + UPPER * (np.sin(theta_inner) + np.sin(theta_outer))
    knee = np.arccos(np.minimum(1.0, np.hypot(p, q) / (2 * LOWER))) + np.arctan2(p, q)
    return -UPPER * np.sin(theta_inner) + LOWER * np.cos(knee)


def band(x, lo, hi):
    f = np.fft.rfftfreq(len(x), 1 / FS)
    spectrum = np.fft.rfft(x - x.mean())
    spectrum[(f < lo) | (f > hi)] = 0
    return np.fft.irfft(spectrum, len(x))


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bag")
    parser.add_argument("controller", nargs="?", help="only this controller's runs")
    parser.add_argument("--lo", type=float, default=5.0)
    parser.add_argument("--hi", type=float, default=15.0)
    args = parser.parse_args()

    joints, gyro, diag = [], [], {}
    for topic, t, msg in messages(args.bag):
        if topic == "/joint_states":
            joints.append((t, dict(zip(msg.name, zip(msg.position, msg.velocity)))))
        elif topic == "/imu_broadcaster/imu":
            gyro.append((t, msg.angular_velocity.z))
        else:
            values = {}
            for status in msg.status:
                for item in status.values:
                    try:
                        values[item.key] = float(item.value)
                    except ValueError:
                        pass
            diag.setdefault(topic.strip("/").split("/")[0], []).append((t, values))
    tj = np.array([s[0] for s in joints])
    tg = np.array([s[0] for s in gyro])
    print(f"{'controller':30s} {'dur s':>6} {'gyro':>6} {'rolling':>7} {'legs':>6} "
          f"{'a':>6} {'b':>6} {'R2':>5} {'R2 roll':>7} {'R2 legs':>7} {'delay':>6}"
          f"  ({args.lo:g}-{args.hi:g} Hz RMS rad/s; delay ms)")
    for name, start, end, _ in runs(diag):
        if (args.controller and name != args.controller) or end - start < 3.0:
            continue
        grid = np.arange(start + 0.3, end, 1 / FS)
        joint = lambda n, k: np.interp(grid, tj, [s[1][n][k] for s in joints])
        rolling = band((joint("right_wheel", 1) - joint("left_wheel", 1)) * RADIUS / TRACK,
                       args.lo, args.hi)
        forward = {side: FORWARD[side] * axle_x(joint(i, 0), joint(o, 0))
                   for side, (i, o) in LEGS.items()}
        legs = band(np.gradient((forward["right"] - forward["left"]) / TRACK, 1 / FS),
                    args.lo, args.hi)
        fit = lambda z, *terms: 1 - np.var(z - np.column_stack(terms) @ np.linalg.lstsq(
            np.column_stack(terms), z, rcond=None)[0]) / np.var(z)
        best = None
        for delay in np.arange(0.0, 0.0305, 0.0005):
            z = band(np.interp(grid + delay, tg, [s[1] for s in gyro]), args.lo, args.hi)
            coef = np.linalg.lstsq(np.column_stack([rolling, legs]), z, rcond=None)[0]
            r2 = fit(z, rolling, legs)
            if best is None or r2 > best[0]:
                best = (r2, delay, z, coef)
        r2, delay, z, (a, b) = best
        print(f"{name:30s} {end - start:6.1f} {z.std():6.3f} {rolling.std():7.3f} {legs.std():6.3f} "
              f"{a:+6.2f} {b:+6.2f} {r2:5.0%} {fit(z, rolling):7.0%} {fit(z, legs):7.0%} "
              f"{delay * 1000:6.1f}")


if __name__ == "__main__":
    main()
