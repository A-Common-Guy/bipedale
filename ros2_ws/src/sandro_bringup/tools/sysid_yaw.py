#!/usr/bin/env python3
"""System identification of the 9 Hz yaw mode, with the whole-body controller balancing.

    deploy/pi/pi sysid                 # on the robot, from the laptop (stack running,
                                       # WBC armed and balancing, someone watching)
    python3 sysid_yaw.py --analyse DIR # afterwards, anywhere

For each leg drive stiffness in --kp (tasks.legs.drive_kp_nm_rad, live) the WBC
taps the yaw mode with short differential wheel-torque pulses of alternating
sign and then sweeps it with a 2-25 Hz chirp (the WBC's sysid.* parameters).
Everything is recorded (ros2 bag, zstd) into DIR with plan.json, the time of
every step. Ctrl-C, a lost engage or stale diagnostics stop the excitation,
put the leg stiffness back and close the recording.

The analysis fits each pulse's ringing (frequency, damping) and the chirp's
frequency response from the measured differential wheel torque to the gyro's
yaw rate, the wheels' rolling and the legs' fore-aft yaw (recordings/
yaw_closure.py). How the frequency moves with the leg stiffness gives the
share of the yaw compliance in the legs' drive PD: all of it moves the
frequency with sqrt(kp), none of it not at all.
"""

import argparse
import json
import math
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

WBC = "whole_body_controller"
KP = "tasks.legs.drive_kp_nm_rad"
TOPICS = ["/joint_states", "/imu_broadcaster/imu", "/health_broadcaster/dynamic_joint_states",
          "/cmd_vel"]
JOINTS = ["right_motor_inner", "right_motor_outer", "right_wheel",
          "left_motor_inner", "left_motor_outer", "left_wheel"]
BASE, UPPER, LOWER, RADIUS, TRACK = 0.09, 0.08, 0.13, 0.0672, 0.304
FORWARD = {"right": 1.0, "left": -1.0}
FS = 300.0


# Running ------------------------------------------------------------------------

class Aborted(RuntimeError):
    pass


class Session:
    def __init__(self, args):
        import rclpy
        from diagnostic_msgs.msg import DiagnosticArray
        from rcl_interfaces.srv import GetParameters, SetParameters
        from rclpy.executors import SingleThreadedExecutor

        self.args = args
        rclpy.init()
        self.rclpy = rclpy
        self.node = rclpy.create_node("sysid_yaw")
        self.set_client = self.node.create_client(SetParameters, f"/{WBC}/set_parameters")
        self.get_client = self.node.create_client(GetParameters, f"/{WBC}/get_parameters")
        self.diag = {"t": 0.0, "engaged": 0.0, "sysid_active": 0.0}
        self.node.create_subscription(DiagnosticArray, f"/{WBC}/diagnostics", self._on_diag, 10)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, daemon=True).start()
        self.recorder = None
        self.restore = {}
        self.events = []
        self.out = Path(args.out)

    def _on_diag(self, msg):
        values = {"t": time.time()}
        for status in msg.status[:1]:
            for item in status.values:
                try:
                    values[item.key] = float(item.value)
                except ValueError:
                    pass
        self.diag = values

    def _wait(self, future, what, timeout=5.0):
        end = time.time() + timeout
        while not future.done():
            if time.time() > end:
                raise Aborted(f"{what}: no answer from /{WBC} within {timeout:.0f} s")
            time.sleep(0.01)
        return future.result()

    def get(self, names):
        from rcl_interfaces.srv import GetParameters
        if not self.get_client.wait_for_service(timeout_sec=3.0):
            raise Aborted(f"/{WBC} has no parameter service: is the stack running?")
        answer = self._wait(self.get_client.call_async(GetParameters.Request(names=names)),
                            "get parameters")
        return {n: v.double_value for n, v in zip(names, answer.values)}

    def set(self, values: dict):
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        from rcl_interfaces.srv import SetParameters
        request = SetParameters.Request(parameters=[
            Parameter(name=name, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                                      double_value=float(value)))
            for name, value in values.items()])
        answer = self._wait(self.set_client.call_async(request), "set parameters")
        refused = [f"{p.name}: {r.reason}" for p, r in zip(request.parameters, answer.results)
                   if not r.successful]
        if refused:
            raise Aborted("the WBC refused " + "; ".join(refused))

    def event(self, kind, **values):
        self.events.append(dict(values, kind=kind, t=time.time()))
        (self.out / "plan.json").write_text(json.dumps({"args": vars(self.args),
                                                        "events": self.events}, indent=1))

    def watch(self, seconds, label):
        """Sleep, but stop the moment the robot is no longer balancing."""
        end = time.time() + seconds
        while time.time() < end:
            d = self.diag
            if time.time() - d["t"] > 1.0:
                raise Aborted(f"{label}: no WBC diagnostics for over 1 s")
            if d.get("engaged") != 1.0:
                raise Aborted(f"{label}: the WBC is no longer balancing")
            remaining = end - time.time()
            print(f"\r  {label}: {remaining:4.1f} s  yaw rate {d.get('yaw_rate_rad_s', 0):+.2f} rad/s"
                  f"  sysid torque {d.get('sysid_torque_nm', 0):+.2f} Nm   ", end="", flush=True)
            time.sleep(0.1)
        print()

    def run(self):
        args = self.args
        print(f"waiting for /{WBC} to balance ...")
        end = time.time() + 5.0
        while self.diag.get("engaged") != 1.0:
            if time.time() > end:
                raise Aborted("the WBC is not balancing: arm and start it, let it stand, run again")
            time.sleep(0.1)
        names = [KP, "sysid.mode", "sysid.trigger"]
        current = self.get(names)
        self.restore = {KP: current[KP], "sysid.mode": 0.0}
        trigger = current["sysid.trigger"]
        per_kp = args.settle + args.pulses * args.period + 1.0 + args.chirp_s + 1.0
        print(f"leg kp {current[KP]:g} now; plan: kp {' -> '.join(f'{k:g}' for k in args.kp)}, "
              f"each {args.pulses} pulses of {args.pulse_nm} Nm and a {args.chirp_s:g} s chirp "
              f"of {args.chirp_nm} Nm ({args.f0:g}-{args.f1:g} Hz): {per_kp * len(args.kp):.0f} s")
        if not args.yes:
            input("watch the robot, hand on the E-stop; Enter starts, Ctrl-C cancels ")

        self.out.mkdir(parents=True, exist_ok=True)
        log = open(self.out / "record.log", "w")
        self.recorder = subprocess.Popen(
            ["ros2", "bag", "record", "-s", "mcap", "--storage-preset-profile", "zstd_fast",
             "-o", str(self.out / "bag"), *TOPICS, "--regex", ".*/diagnostics"],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        time.sleep(3.0)
        if self.recorder.poll() is not None:
            raise Aborted(f"ros2 bag record exited: {self.out / 'record.log'}")
        self.event("start", kp=current[KP])

        for kp in args.kp:
            print(f"leg kp {kp:g}")
            self.set({KP: kp})
            self.event("kp", kp=kp)
            self.watch(args.settle, "settle")
            trigger += 1
            self.set({"sysid.mode": 1, "sysid.amplitude_nm": args.pulse_nm,
                      "sysid.pulse_s": args.pulse_s, "sysid.period_s": args.period,
                      "sysid.duration_s": args.pulses * args.period, "sysid.trigger": trigger})
            self.event("pulses", kp=kp, amplitude_nm=args.pulse_nm, pulse_s=args.pulse_s,
                       period_s=args.period, count=args.pulses)
            self.watch(args.pulses * args.period + 1.0, "pulses")
            trigger += 1
            self.set({"sysid.mode": 2, "sysid.amplitude_nm": args.chirp_nm, "sysid.f0_hz": args.f0,
                      "sysid.f1_hz": args.f1, "sysid.duration_s": args.chirp_s,
                      "sysid.trigger": trigger})
            self.event("chirp", kp=kp, amplitude_nm=args.chirp_nm, f0_hz=args.f0, f1_hz=args.f1,
                       duration_s=args.chirp_s)
            self.watch(args.chirp_s + 1.0, "chirp")
        self.event("done")

    def close(self, reason=None):
        try:
            if self.restore:
                trigger = self.get(["sysid.trigger"])["sysid.trigger"] + 1
                self.set(dict(self.restore, **{"sysid.trigger": trigger}))
                print(f"restored leg kp {self.restore[KP]:g}, excitation off")
        except Exception as error:  # noqa: BLE001 - say it, keep closing
            print(f"COULD NOT RESTORE THE WBC ({error}): set {KP} back on the Tune page")
        if reason and self.out.is_dir():
            self.event("aborted", reason=reason)
        if self.recorder is not None and self.recorder.poll() is None:
            self.recorder.send_signal(signal.SIGINT)
            try:
                self.recorder.wait(timeout=15)
            except subprocess.TimeoutExpired:
                self.recorder.kill()
        self.executor.shutdown()
        self.node.destroy_node()
        self.rclpy.try_shutdown()


# Analysis -----------------------------------------------------------------------

def read_bag(path):
    """Joint positions, velocities, efforts; gyro z; WBC diagnostics, by header stamp."""
    import numpy as np
    joints, gyro, diag = [], [], []

    def stamp(msg, received_ns):
        s = msg.header.stamp
        return s.sec + s.nanosec * 1e-9 if (s.sec or s.nanosec) else received_ns * 1e-9

    def take(topic, msg, received_ns):
        t = stamp(msg, received_ns)
        if topic == "/joint_states":
            index = [msg.name.index(n) for n in JOINTS]
            joints.append([t] + [msg.position[i] for i in index] + [msg.velocity[i] for i in index]
                          + [msg.effort[i] for i in index])
        elif topic == "/imu_broadcaster/imu":
            gyro.append((t, msg.angular_velocity.z))
        elif topic == f"/{WBC}/diagnostics":
            values = {}
            for status in msg.status[:1]:
                for item in status.values:
                    try:
                        values[item.key] = float(item.value)
                    except ValueError:
                        pass
            diag.append((t, values.get("engaged", 0.0), values.get("sysid_torque_nm", 0.0)))

    wanted = {"/joint_states", "/imu_broadcaster/imu", f"/{WBC}/diagnostics"}
    try:
        from rclpy.serialization import deserialize_message
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
        from rosidl_runtime_py.utilities import get_message
        reader = SequentialReader()
        reader.open(StorageOptions(uri=str(path), storage_id="mcap"), ConverterOptions("", ""))
        types = {t.name: t.type for t in reader.get_all_topics_and_types()}
        while reader.has_next():
            topic, data, received = reader.read_next()
            if topic in wanted:
                take(topic, deserialize_message(data, get_message(types[topic])), received)
    except ImportError:
        from mcap_ros2.reader import read_ros2_messages
        for file in sorted(Path(path).glob("*.mcap")):
            for m in read_ros2_messages(str(file), topics=sorted(wanted)):
                take(m.channel.topic, m.ros_msg, m.log_time_ns)
    return np.array(joints), np.array(gyro), np.array(diag)


def axle_x(theta_inner, theta_outer):
    import numpy as np
    p = UPPER * (np.cos(theta_outer) - np.cos(theta_inner))
    q = BASE + UPPER * (np.sin(theta_inner) + np.sin(theta_outer))
    knee = np.arccos(np.minimum(1.0, np.hypot(p, q) / (2 * LOWER))) + np.arctan2(p, q)
    return -UPPER * np.sin(theta_inner) + LOWER * np.cos(knee)


def band(x, lo, hi):
    import numpy as np
    f = np.fft.rfftfreq(len(x), 1 / FS)
    spectrum = np.fft.rfft(x - x.mean())
    spectrum[(f < lo) | (f > hi)] = 0
    return np.fft.irfft(spectrum, len(x))


def signals(joints, gyro, t0, t1):
    """Per-wheel differential torque (input) and the yaw rates, on a 300 Hz grid."""
    import numpy as np
    grid = np.arange(t0, t1, 1 / FS)
    col = lambda name, k: np.interp(grid, joints[:, 0], joints[:, 1 + 6 * k + JOINTS.index(name)])
    torque = 0.5 * (col("right_wheel", 2) - col("left_wheel", 2))
    rolling = (col("right_wheel", 1) - col("left_wheel", 1)) * RADIUS / TRACK
    forward = {side: FORWARD[side] * axle_x(col(f"{side}_motor_inner", 0), col(f"{side}_motor_outer", 0))
               for side in ("right", "left")}
    legs = np.gradient((forward["right"] - forward["left"]) / TRACK, 1 / FS)
    yaw = np.interp(grid, gyro[:, 0], gyro[:, 1])
    return grid, torque, yaw, rolling, legs


def free_decay_mode(windows):
    """The dominant damped oscillation common to free-decay windows: f (Hz) and zeta.

    Matrix pencil: the windows' Hankel matrices stacked, their two leading
    singular vectors span the mode, and the shift between their rows gives its
    pole. Unlike a least-squares AR fit it is not dragged off by sensor noise.
    """
    import numpy as np
    span = min(len(w) for w in windows) // 2
    blocks = [np.array([w[i:i + span + 1] for i in range(len(w) - span)]) for w in windows]
    _, _, vt = np.linalg.svd(np.vstack(blocks), full_matrices=False)
    v = vt[:2].T
    poles = np.linalg.eigvals(np.linalg.pinv(v[:-1]) @ v[1:])
    z = poles[np.argmax(np.abs(poles.imag))]
    if abs(z.imag) < 1e-9:
        return math.nan, math.nan
    s = np.log(z) * FS
    return abs(s) / (2 * math.pi), -s.real / abs(s)


def yaw_model_fit(f, h, weight):
    """Fit H = k e^(-s tau) / s * wn^2 / (s^2 + 2 zeta wn s + wn^2) to a measured response.

    A rigid yaw inertia (1/s) behind one compliance resonance, as the 2026-11-12
    chirps look: (fn Hz, zeta, tau s, k, relative error).
    """
    import numpy as np
    jw = 2j * np.pi * f
    zetas = np.arange(0.05, 0.9, 0.01)[:, None]
    best = None
    for fn in np.arange(5.0, 20.01, 0.1):
        wn = 2 * np.pi * fn
        resonance = wn ** 2 / (jw ** 2 + 2 * zetas * wn * jw + wn ** 2)
        for tau in np.arange(0.0, 0.0155, 0.001):
            shape = np.exp(-jw * tau) / jw * resonance
            k = np.real(np.sum(weight * np.conj(shape) * h, axis=1)) / np.sum(weight * abs(shape) ** 2, axis=1)
            err = np.sum(weight * abs(h - k[:, None] * shape) ** 2, axis=1) / np.sum(weight * abs(h) ** 2)
            i = int(np.argmin(err))
            if best is None or err[i] < best[4]:
                best = (fn, float(zetas[i, 0]), tau, float(k[i]), float(err[i]))
    return best


def response(u, y, segment=1024):
    """H(f) = Puy / Puu and the coherence, Welch with half overlap."""
    import numpy as np
    window = np.hanning(segment)
    puu = puy = pyy = 0
    for start in range(0, len(u) - segment + 1, segment // 2):
        U = np.fft.rfft((u[start:start + segment] - u[start:start + segment].mean()) * window)
        Y = np.fft.rfft((y[start:start + segment] - y[start:start + segment].mean()) * window)
        puu, puy, pyy = puu + abs(U) ** 2, puy + np.conj(U) * Y, pyy + abs(Y) ** 2
    f = np.fft.rfftfreq(segment, 1 / FS)
    with np.errstate(divide="ignore", invalid="ignore"):  # bins the chirp never reaches
        return f, puy / puu, abs(puy) ** 2 / (puu * pyy)


def analyse(directory):
    import numpy as np
    directory = Path(directory)
    plan = json.loads((directory / "plan.json").read_text())
    joints, gyro, _ = read_bag(directory / "bag")
    rows = []
    for event in plan["events"]:
        if event["kind"] == "pulses":
            t0 = event["t"]
            _, _, yaw, _, _ = signals(joints, gyro, t0, t0 + event["count"] * event["period_s"] + 0.5)
            yaw = band(yaw, 3.0, 30.0)
            windows = []
            for k in range(event["count"]):
                start = int((k * event["period_s"] + event["pulse_s"] + 0.02) * FS)
                stop = int((k * event["period_s"] + min(event["period_s"] - 0.1, 0.5)) * FS)
                windows.append(yaw[start:stop])
            f, z = free_decay_mode(windows)
            rows.append({"kp": event["kp"], "pulse_hz": f, "pulse_zeta": z})
        elif event["kind"] == "chirp":
            t0 = event["t"]
            _, torque, yaw, rolling, legs = signals(joints, gyro, t0, t0 + event["duration_s"])
            freq, h_yaw, coherence = response(torque, yaw)
            _, h_roll, _ = response(torque, rolling)
            _, h_legs, _ = response(torque, legs)
            sel = (freq >= 2.5) & (freq <= 22.0) & np.isfinite(h_yaw)
            fn, zeta, tau, k, err = yaw_model_fit(freq[sel], h_yaw[sel], coherence[sel])
            i = int(np.argmin(abs(freq - fn)))
            lever = TRACK / (2 * RADIUS)
            inertia = 2 * lever / k            # per-wheel torque, both wheels
            row = next((r for r in reversed(rows) if r["kp"] == event["kp"] and "chirp_hz" not in r), None)
            if row is None:
                row = {"kp": event["kp"]}
                rows.append(row)
            row.update(chirp_hz=fn, chirp_zeta=zeta, delay_ms=tau * 1000, fit_error=err,
                       coherence=coherence[i], yaw_inertia_kgm2=inertia,
                       wheel_stiffness_nm_rad=inertia * (2 * math.pi * fn) ** 2 / (2 * lever ** 2),
                       legs_share=abs(h_legs[i] / h_yaw[i]), rolling_share=abs(h_roll[i] / h_yaw[i]))
    reference = next((r for r in rows if r["kp"] == 12.0), rows[0] if rows else None)
    print(f"{'kp':>5} {'ring Hz':>8} {'zeta':>5} {'mode Hz':>8} {'zeta':>5} {'coh':>4} "
          f"{'N m/rad':>8} {'legs':>5} {'roll':>5} {'f/f12':>6} {'all legs':>8} {'half':>6} {'legs PD share':>13}")
    # The chirp's fit is the precise one (the pulses scatter about 1.5 % at zeta 0.3).
    best = lambda r: r.get("chirp_hz", math.nan) if not math.isnan(r.get("chirp_hz", math.nan)) \
        else r.get("pulse_hz", math.nan)
    for r in rows:
        f = best(r)
        f12 = best(reference) if reference else math.nan
        ratio = f / f12 if f and f12 else math.nan
        all_legs = math.sqrt(r["kp"] / 12.0)
        half = math.sqrt((1 / (12.0 / r["kp"] + 1)) / 0.5)
        share = ((1 / ratio ** 2 - 1) / (12.0 / r["kp"] - 1)) if r["kp"] != 12.0 and ratio == ratio else math.nan
        print(f"{r['kp']:5g} {r.get('pulse_hz', math.nan):8.2f} {r.get('pulse_zeta', math.nan):5.2f} "
              f"{r.get('chirp_hz', math.nan):8.2f} {r.get('chirp_zeta', math.nan):5.2f} "
              f"{r.get('coherence', math.nan):4.2f} {r.get('wheel_stiffness_nm_rad', math.nan):8.0f} "
              f"{r.get('legs_share', math.nan):5.2f} {r.get('rolling_share', math.nan):5.2f} "
              f"{ratio:6.3f} {all_legs:8.3f} {half:6.3f} {share:13.2f}")
    print("ring: the pulses' free decay (under the WBC mostly its own yaw loop). mode: the chirp fitted\n"
          "as yaw inertia behind one resonance; N m/rad its stiffness per wheel at the axle. f/f12: the\n"
          "mode against kp 12; 'all legs' and 'half' what it would be if the leg PD held all or half of\n"
          "the yaw compliance; legs PD share: the share these numbers give. legs, roll: the legs'\n"
          "fore-aft yaw and the wheels' rolling against the gyro at the mode.")
    (directory / "summary.json").write_text(json.dumps(rows, indent=1, default=float))


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--kp", type=float, nargs="+", default=[12.0, 9.0, 16.0, 12.0],
                        help="leg drive stiffnesses to step through, Nm/rad")
    parser.add_argument("--pulses", type=int, default=6)
    parser.add_argument("--pulse-nm", type=float, default=0.25, help="per wheel")
    parser.add_argument("--pulse-s", type=float, default=0.03)
    parser.add_argument("--period", type=float, default=1.5, help="s between pulses")
    parser.add_argument("--chirp-s", type=float, default=20.0)
    parser.add_argument("--chirp-nm", type=float, default=0.2, help="per wheel")
    parser.add_argument("--f0", type=float, default=2.0)
    parser.add_argument("--f1", type=float, default=25.0)
    parser.add_argument("--settle", type=float, default=3.0, help="s after each kp change")
    parser.add_argument("--out", default=f"/workspaces/sandro/experiments/sysid_{time.strftime('%Y%m%d-%H%M%S')}")
    parser.add_argument("--yes", action="store_true", help="no Enter to start")
    parser.add_argument("--analyse", metavar="DIR", help="analyse a finished run instead")
    args = parser.parse_args()
    if args.analyse:
        analyse(args.analyse)
        return 0
    if max(args.pulse_nm, args.chirp_nm) > 0.5 or min(args.kp) < 6 or max(args.kp) > 25:
        sys.exit("refused: torques above 0.5 Nm per wheel or leg kp outside 6-25 Nm/rad")
    session = Session(args)
    reason = None
    try:
        session.run()
    except KeyboardInterrupt:
        reason = "Ctrl-C"
    except Aborted as error:
        reason = str(error)
    finally:
        if reason:
            print(f"\nstopped: {reason}")
        session.close(reason)
    if session.events and (Path(args.out) / "bag").exists():
        print(f"recorded: {args.out}")
        try:
            analyse(args.out)
        except Exception as error:  # noqa: BLE001 - the recording is what matters
            print(f"analysis failed ({error}); run: python3 sysid_yaw.py --analyse {args.out}")
    return 1 if reason else 0


if __name__ == "__main__":
    sys.exit(main())
