#!/usr/bin/env python3
"""Backlash and stiffness of one drive train, with its output blocked.

The drive's encoder sits on the motor, before the 8:1 gear, so with the wheel
(or, for a leg motor, the axle) held still a slow torque triangle moves the motor
through the gear's play
(at almost no torque) and then against the drive train's stiffness. A short
sine at a few Hz follows, for the dynamic stiffness, then a chirp (2-45 Hz):
with the output blocked the rotor rings on the gear's stiffness at
sqrt(k / J), which gives the rotor's inertia at the gear output. The
2026-11-12 yaw identification reads as a rotor of about 0.0055 kg m2 on
about 19 Nm/rad (9 Hz here); the MJCF's armature, 0.000168 kg m2, would ring
above 50 Hz. The motor position comes from /joint_states (output-side rad);
the torque is the commanded one.

Two setups, one wheel at a time:

- gear: robot lifted, the wheel's rim held against rotation (a clamp or a
  block between tyre and frame). Play and stiffness of gear, shaft and hub.
- ground: robot standing on the floor, the body pressed down and held so it
  cannot roll or turn. Adds the tyre's fore-aft compliance on the floor.
- leg: a leg motor (right_motor_inner ...) with that leg's axle clamped to the
  body or a fixed block, robot lifted. Play and stiffness of the leg's gear and
  links, which the 2026-11 yaw mode points at (recordings/yaw_closure.py).

On the robot, for each wheel and setup (the commissioning controller clamps
at 0.5 Nm and zeroes 0.1 s after the last command):

    ros2 launch sandro_bringup commissioning.launch.py target:=right_wheel activate:=true
    python3 drive_train_test.py --joint right_wheel --setup gear --output right_gear.csv

and afterwards, anywhere: python3 drive_train_test.py --analyse right_gear.csv
"""

import argparse
import csv
import math
import sys
import time

import numpy as np


CHIRP_HZ = (2.0, 45.0)


def profile(t, amplitude, slow_period, slow_cycles, sine_hz, sine_s, chirp_s=0.0, chirp_nm=0.3):
    """Commanded torque at t (s): triangle cycles, a sine at sine_hz, then a
    linear chirp over CHIRP_HZ with 0.5 s tapers."""
    slow = slow_period * slow_cycles
    if t < slow:
        phase = (t / slow_period) % 1.0
        return amplitude * (4 * phase if phase < 0.25 else 2 - 4 * phase if phase < 0.75 else 4 * phase - 4)
    if t < slow + sine_s:
        return amplitude * math.sin(2 * math.pi * sine_hz * (t - slow))
    tc = t - slow - sine_s
    if tc < chirp_s:
        f0, f1 = CHIRP_HZ
        phase = 2 * math.pi * (f0 * tc + 0.5 * (f1 - f0) / chirp_s * tc * tc)
        edge = min(max(min(tc, chirp_s - tc) / 0.5, 0.0), 1.0)
        return chirp_nm * 0.5 * (1 - math.cos(math.pi * edge)) * math.sin(phase)
    return None


def run(args):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray

    rclpy.init()
    node = Node("drive_train_test")
    topic = f"/commission_{args.joint}/commands"
    publisher = node.create_publisher(Float64MultiArray, topic, 10)
    rows, command = [], [0.0]

    def on_state(msg):
        if args.joint in msg.name:
            i = msg.name.index(args.joint)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            rows.append((stamp, msg.position[i], msg.velocity[i], msg.effort[i] if msg.effort else math.nan,
                         command[0]))

    node.create_subscription(JointState, "/joint_states", on_state, 100)
    deadline = time.monotonic() + 5.0
    while not rows and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not rows:
        sys.exit("no /joint_states: is the commissioning launch running?")
    if publisher.get_subscription_count() == 0:
        sys.exit(f"nobody subscribes {topic}: launch commissioning with target:={args.joint} activate:=true")
    print(f"{args.joint} ({args.setup}): +-{args.amplitude} Nm, {args.cycles} triangles of {args.period} s, "
          f"then {args.sine_s} s at {args.sine_hz} Hz and a {args.chirp_s} s chirp of {args.chirp_nm} Nm. "
          "The output must be blocked.")
    start = time.monotonic()
    try:
        while True:
            t = time.monotonic() - start
            tau = profile(t, args.amplitude, args.period, args.cycles, args.sine_hz, args.sine_s,
                          args.chirp_s, args.chirp_nm)
            if tau is None:
                break
            command[0] = tau
            publisher.publish(Float64MultiArray(data=[tau]))
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.005)
    finally:
        command[0] = 0.0
        for _ in range(5):
            publisher.publish(Float64MultiArray(data=[0.0]))
            time.sleep(0.01)
    rclpy.spin_once(node, timeout_sec=0.1)
    with open(args.output, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["# joint", args.joint, "setup", args.setup, "amplitude_nm", args.amplitude,
                    "period_s", args.period, "cycles", args.cycles, "sine_hz", args.sine_hz, "sine_s", args.sine_s,
                    "chirp_s", args.chirp_s, "chirp_nm", args.chirp_nm])
        w.writerow(["t_s", "position_rad", "velocity_rad_s", "effort_nm", "command_nm"])
        t0 = rows[0][0]
        w.writerows([(r[0] - t0, *r[1:]) for r in rows])
    print(f"wrote {args.output} ({len(rows)} samples)")
    node.destroy_node()
    rclpy.shutdown()
    analyse(args.output)


def analyse(path):
    with open(path) as f:
        head = next(csv.reader(f))
        meta = dict(zip((k.lstrip("# ") for k in head[0::2]), head[1::2]))
        rows = list(csv.DictReader(f))
    t = np.array([float(r["t_s"]) for r in rows])
    q = np.array([float(r["position_rad"]) for r in rows])
    tau = np.array([float(r["command_nm"]) for r in rows])
    amplitude = float(meta["amplitude_nm"])
    slow_end = float(meta["period_s"]) * float(meta["cycles"])
    slow = (t > 0.25 * float(meta["period_s"])) & (t < slow_end)
    q_s, tau_s = q[slow] - np.median(q[slow]), tau[slow]
    # Beyond the play the motor angle is linear in torque, on either side:
    # q = c_+ + tau / k (tau > 0) and q = c_- + tau / k (tau < 0). The play is
    # c_+ - c_-, the angle the motor turns through at no torque.
    loaded = 0.35 * amplitude
    pos, neg = tau_s > loaded, tau_s < -loaded
    fit = lambda m: np.polyfit(tau_s[m], q_s[m], 1)
    (s_pos, c_pos), (s_neg, c_neg) = fit(pos), fit(neg)
    play = c_pos - c_neg
    print(f"{meta['joint']} ({meta['setup']}), {slow.sum()} samples over {slow_end:.0f} s of triangles:")
    print(f"  play {play * 1e3:.1f} mrad ({math.degrees(play):.2f} deg) at the gear output")
    print(f"  stiffness beyond it {1 / s_pos:.0f} Nm/rad (positive torque), {1 / s_neg:.0f} Nm/rad (negative)")
    # Hysteresis inside each side: rising against falling torque at the same load.
    rising = np.gradient(tau_s) > 0
    for name, m in (("positive", pos), ("negative", neg)):
        up, down = m & rising, m & ~rising
        if up.sum() > 20 and down.sum() > 20:
            gap = np.polyfit(tau_s[up], q_s[up], 1)[1] - np.polyfit(tau_s[down], q_s[down], 1)[1]
            print(f"  hysteresis on the {name} side {abs(gap) * 1e3:.1f} mrad (friction in the train)")
    k_static = 2.0 / (s_pos + s_neg)
    sine_end = slow_end + float(meta["sine_s"])
    sine = (t > slow_end + 0.5) & (t < sine_end)
    chirp_s = float(meta.get("chirp_s", 0.0))
    if chirp_s > 0:
        chirp = (t > sine_end) & (t < sine_end + chirp_s)
        grid = np.arange(t[chirp][0], t[chirp][-1], 1 / 300.0)
        qc = np.interp(grid, t, q)
        tc = np.interp(grid, t, tau)
        seg, window = 256, np.hanning(256)
        suu = suq = sqq = 0
        for start in range(0, len(grid) - seg + 1, seg // 2):
            u = np.fft.rfft((tc[start:start + seg] - tc[start:start + seg].mean()) * window)
            y = np.fft.rfft((qc[start:start + seg] - qc[start:start + seg].mean()) * window)
            suu, suq, sqq = suu + abs(u) ** 2, suq + np.conj(u) * y, sqq + abs(y) ** 2
        f = np.fft.rfftfreq(seg, 1 / 300.0)
        with np.errstate(divide="ignore", invalid="ignore"):
            compliance, coherence = suq / suu, abs(suq) ** 2 / (suu * sqq)
        band = (f >= CHIRP_HZ[0] + 1) & (f <= CHIRP_HZ[1] - 2)
        static = abs(compliance[band][0])
        peak = int(np.argmax(abs(compliance[band])))
        fr, gain = f[band][peak], abs(compliance[band][peak]) / static
        print(f"  chirp {CHIRP_HZ[0]:g}-{CHIRP_HZ[1]:g} Hz: motor angle per Nm peaks at {fr:.1f} Hz, "
              f"{gain:.1f} x its {f[band][0]:.0f} Hz value (coherence {coherence[band][peak]:.2f})")
        at_edge = peak >= int(band.sum()) - 3
        if gain > 1.5 and not at_edge:
            print(f"  rotor inertia at the gear output, k / (2 pi f)^2 with the triangles' {k_static:.0f} Nm/rad: "
                  f"{k_static / (2 * math.pi * fr) ** 2:.5f} kg m2 (MJCF armature 0.000168)")
        else:
            print(f"  no resonance inside the chirp (still rising at its end): the rotor inertia is under "
                  f"{k_static / (2 * math.pi * CHIRP_HZ[1]) ** 2:.5f} kg m2")
    if sine.sum() > 100:
        hz = float(meta["sine_hz"])
        ts, qs, ts_tau = t[sine], q[sine] - q[sine].mean(), tau[sine]
        basis = np.stack([np.sin(2 * np.pi * hz * ts), np.cos(2 * np.pi * hz * ts)], 1)
        cq = np.linalg.lstsq(basis, qs, rcond=None)[0]
        ct = np.linalg.lstsq(basis, ts_tau, rcond=None)[0]
        aq, at = np.hypot(*cq), np.hypot(*ct)
        lag = math.degrees(math.atan2(cq[1], cq[0]) - math.atan2(ct[1], ct[0]))
        print(f"  at {hz:g} Hz: {at / aq:.0f} Nm/rad peak to peak ({aq * 1e3:.1f} mrad for {at:.2f} Nm), "
              f"angle {(lag + 180) % 360 - 180:+.0f} deg from torque")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--joint", choices=["right_wheel", "left_wheel", "right_motor_inner",
                                            "right_motor_outer", "left_motor_inner",
                                            "left_motor_outer"])
    parser.add_argument("--setup", choices=["gear", "ground", "leg"], default="gear")
    parser.add_argument("--amplitude", type=float, default=0.45, help="Nm, at most 0.5")
    parser.add_argument("--period", type=float, default=4.0, help="s per triangle")
    parser.add_argument("--cycles", type=int, default=4)
    parser.add_argument("--sine-hz", type=float, default=3.0)
    parser.add_argument("--sine-s", type=float, default=4.0)
    parser.add_argument("--chirp-s", type=float, default=15.0, help="s of 2-45 Hz chirp, 0 skips it")
    parser.add_argument("--chirp-nm", type=float, default=0.3)
    parser.add_argument("--output")
    parser.add_argument("--analyse", metavar="CSV", help="analyse a recording instead of running")
    args = parser.parse_args()
    if args.analyse:
        analyse(args.analyse)
        return
    if not args.joint or not args.output:
        parser.error("--joint and --output are needed for a run")
    if not 0.0 < args.amplitude <= 0.5:
        parser.error("--amplitude must be in (0, 0.5] Nm: the commissioning controller's limit")
    if not 0.0 < args.chirp_nm <= 0.5:
        parser.error("--chirp-nm must be in (0, 0.5] Nm: the commissioning controller's limit")
    run(args)


if __name__ == "__main__":
    main()
