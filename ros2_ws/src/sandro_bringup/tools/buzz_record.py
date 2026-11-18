#!/usr/bin/env python3
"""Record a balancing controller's buzz and say which loop it is in.

    python3 buzz_record.py --label rl_flat_level_sysid   # on the robot, the controller balancing
    python3 buzz_record.py --analyse DIR                 # afterwards, anywhere

Three timed phases, announced on the terminal (and on the panel's Sysid page,
which runs this): stand still, drive and turn gently, stand still again.
Everything is recorded (ros2 bag, zstd) into DIR with plan.json, the time of
each phase. Nothing is commanded: the operator drives with the gamepad.

The analysis splits each phase into the yaw mode (the wheels against each
other, the gyro's yaw rate) and the pitch mode (the wheels together, the
gyro's pitch rate) and gives their RMS and strongest frequency in 5-15 Hz,
the band of the 9 Hz yaw buzz, and in 15-45 Hz, above the band the yaw
response was first measured in. The verdict names the loudest of them while
standing: which loop to measure or constrain next.
"""

import argparse
import json
import math
import signal
import subprocess
import sys
import time
from pathlib import Path

TOPICS = ["/joint_states", "/imu_broadcaster/imu", "/health_broadcaster/dynamic_joint_states", "/cmd_vel"]
WHEELS = ("right_wheel", "left_wheel")
FS = 300.0
RADIUS, TRACK = 0.0672, 0.304          # as sysid_yaw.py
BANDS = ((5.0, 15.0), (15.0, 45.0))
# Opposed wheel motion as the yaw rate it rolls, and the gyro's yaw rate, under
# which a standing robot is quiet: the WBC stays at 0.03-0.04 rad/s in 5-15 Hz.
QUIET_RAD_S = 0.06


def phases(args):
    return [("stand", args.stand, "stand still, hands off the sticks"),
            ("drive", args.drive, "drive and turn gently, stop now and then"),
            ("stand again", args.stand, "stand still again, hands off the sticks")]


def record(args):
    out = Path(args.out)
    plan = phases(args)
    total = sum(seconds for _, seconds, _ in plan)
    print(f"recording {args.label or 'the running controller'} for {total:.0f} s into {out}")
    if not args.yes:
        input("the robot balancing, room to drive; Enter starts, Ctrl-C cancels ")
    out.mkdir(parents=True, exist_ok=True)
    log = open(out / "record.log", "w")
    recorder = subprocess.Popen(
        ["ros2", "bag", "record", "-s", "mcap", "--storage-preset-profile", "zstd_fast",
         "-o", str(out / "bag"), *TOPICS, "--regex", ".*/diagnostics"],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    events = []
    reason = None

    def save():
        (out / "plan.json").write_text(json.dumps(
            {"args": vars(args), "label": args.label, "events": events}, indent=1))

    try:
        time.sleep(3.0)                     # the recorder subscribing
        if recorder.poll() is not None:
            raise RuntimeError(f"ros2 bag record exited: {out / 'record.log'}")
        for name, seconds, instruction in plan:
            print(f"NOW: {instruction} ({seconds:.0f} s)", flush=True)
            events.append({"kind": "phase", "name": name, "t": time.time(), "duration_s": seconds})
            save()
            end = time.time() + seconds
            while time.time() < end:
                if recorder.poll() is not None:
                    raise RuntimeError(f"ros2 bag record exited: {out / 'record.log'}")
                time.sleep(0.2)
        print("NOW: done, recording closed", flush=True)
    except KeyboardInterrupt:
        reason = "stopped"
    except RuntimeError as error:
        reason = str(error)
    finally:
        if reason:
            events.append({"kind": "aborted", "reason": reason, "t": time.time()})
            save()
            print(f"stopped: {reason}", flush=True)
        if recorder.poll() is None:
            recorder.send_signal(signal.SIGINT)
            try:
                recorder.wait(timeout=15)
            except subprocess.TimeoutExpired:
                recorder.kill()
    return reason


# Analysis -----------------------------------------------------------------------

def messages(path, wanted):
    """(topic, message, receive time ns) of the wanted topics of a bag, in order."""
    wanted = set(wanted)
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
                yield topic, deserialize_message(data, get_message(types[topic])), received
    except ImportError:
        from mcap_ros2.reader import read_ros2_messages
        for file in sorted(Path(path).glob("*.mcap")):
            for m in read_ros2_messages(str(file), topics=sorted(wanted)):
                yield m.channel.topic, m.ros_msg, m.log_time_ns


def stamp(msg, received_ns):
    s = msg.header.stamp
    return s.sec + s.nanosec * 1e-9 if (s.sec or s.nanosec) else received_ns * 1e-9


def read_bag(path):
    """(t, right wheel velocity, left wheel velocity) and (t, gyro x, y, z), by header stamp."""
    import numpy as np
    wheels, gyro = [], []
    for topic, msg, received_ns in messages(path, {"/joint_states", "/imu_broadcaster/imu"}):
        t = stamp(msg, received_ns)
        if topic == "/joint_states":
            wheels.append([t] + [msg.velocity[msg.name.index(n)] for n in WHEELS])
        else:
            g = msg.angular_velocity
            gyro.append((t, g.x, g.y, g.z))
    return np.array(wheels), np.array(gyro)


def band_content(x, lo, hi):
    """(RMS, strongest frequency) of x (300 Hz samples) between lo and hi Hz."""
    import numpy as np
    x = np.asarray(x, dtype=float)
    if len(x) < int(FS):
        return math.nan, math.nan
    spectrum = np.fft.rfft((x - x.mean()) * np.hanning(len(x)))
    f = np.fft.rfftfreq(len(x), 1 / FS)
    keep = (f >= lo) & (f <= hi)
    power = np.abs(spectrum[keep]) ** 2
    # Parseval with the Hann window's power gain (3/8).
    rms = math.sqrt(2.0 * power.sum() / (len(x) ** 2 * 0.375))
    return rms, float(f[keep][int(power.argmax())])


def modes(t, right, left, gyro_t, gyro, t0, t1):
    """Per band, the yaw and pitch modes of one phase: {band: {signal: (rms, hz)}}."""
    import numpy as np
    grid = np.arange(t0, t1, 1 / FS)
    wr, wl = np.interp(grid, t, right), np.interp(grid, t, left)
    signals = {
        "gyro yaw": np.interp(grid, gyro_t, gyro[:, 2]),
        "wheels opposed (as yaw)": (wr - wl) * RADIUS / TRACK,
        "gyro pitch": np.interp(grid, gyro_t, gyro[:, 1]),
        "wheels together": 0.5 * (wr + wl),
        "gyro roll": np.interp(grid, gyro_t, gyro[:, 0]),
    }
    return {f"{lo:g}-{hi:g} Hz": {name: band_content(x, lo, hi) for name, x in signals.items()}
            for lo, hi in BANDS}


def verdict(standing):
    """One line from the standing phases' modes ({band: {signal: (rms, hz)}})."""
    best = None
    for band, values in standing.items():
        for name in ("gyro yaw", "gyro pitch", "gyro roll"):
            rms, hz = values[name]
            if rms == rms and (best is None or rms > best[0]):
                best = (rms, hz, name, band)
    if best is None:
        return "no standing data"
    rms, hz, name, band = best
    if rms < QUIET_RAD_S:
        return f"quiet standing: at most {rms:.2f} rad/s ({name}, {band})"
    axis = name.split()[1]
    if axis == "yaw":
        where = ("the yaw loop inside the measured band: check the policy with recordings/yaw_loop_margin.py"
                 if hz <= 22.0 else
                 "the yaw loop above the first chirp's 22 Hz: the 45 Hz yaw response covers it")
    elif axis == "pitch":
        where = "the pitch loop (the wheels together): it has no measured response yet"
    else:
        where = "roll, the legs: no measured response yet"
    return f"buzz standing: {rms:.2f} rad/s of {name} at {hz:.1f} Hz, {where}"


def analyse(directory):
    import numpy as np
    directory = Path(directory)
    plan = json.loads((directory / "plan.json").read_text())
    wheels, gyro = read_bag(directory / "bag")
    if len(wheels) < FS or len(gyro) < FS:
        raise RuntimeError("the recording holds no joint states or no IMU: was the stack running?")
    rows = []
    standing = {}
    for event in (e for e in plan["events"] if e["kind"] == "phase"):
        t0 = event["t"] + 1.0               # the operator reacting to the prompt
        t1 = min(event["t"] + event["duration_s"], wheels[-1, 0], gyro[-1, 0])
        if t1 - t0 < 3.0:
            continue
        result = modes(wheels[:, 0], wheels[:, 1], wheels[:, 2], gyro[:, 0], gyro[:, 1:], t0, t1)
        rows.append({"phase": event["name"], "seconds": t1 - t0, "bands": result})
        if event["name"].startswith("stand"):
            for band, values in result.items():
                for name, (rms, hz) in values.items():
                    kept = standing.setdefault(band, {}).get(name)
                    if kept is None or (rms == rms and rms > kept[0]):
                        standing[band][name] = (rms, hz)
    label = plan.get("label") or "controller"
    print(f"{label}: RMS (rad/s) and strongest frequency per phase")
    names = ["gyro yaw", "wheels opposed (as yaw)", "gyro pitch", "wheels together", "gyro roll"]
    for row in rows:
        print(f"  {row['phase']} ({row['seconds']:.0f} s)")
        for band, values in row["bands"].items():
            cells = "   ".join(f"{name} {values[name][0]:.2f} @ {values[name][1]:.1f} Hz" for name in names)
            print(f"    {band:9s} {cells}")
    line = verdict(standing)
    print(line)
    (directory / "summary.json").write_text(json.dumps(
        {"label": label, "verdict": line, "phases": rows}, indent=1, default=float))
    return line


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--label", default="", help="the controller under test, for the record")
    parser.add_argument("--stand", type=float, default=30.0, help="s of each standing phase")
    parser.add_argument("--drive", type=float, default=30.0, help="s of the driving phase")
    parser.add_argument("--out", default=f"/workspaces/sandro/experiments/buzz_{time.strftime('%Y%m%d-%H%M%S')}")
    parser.add_argument("--yes", action="store_true", help="no Enter to start")
    parser.add_argument("--analyse", metavar="DIR", help="analyse a finished recording instead")
    args = parser.parse_args()
    if args.analyse:
        analyse(args.analyse)
        return 0
    reason = record(args)
    if (Path(args.out) / "bag").exists() and (Path(args.out) / "plan.json").exists():
        print(f"recorded: {args.out}")
        try:
            analyse(args.out)
        except Exception as error:  # noqa: BLE001 - the recording is what matters
            print(f"analysis failed ({error}); run: python3 buzz_record.py --analyse {args.out}")
    return 1 if reason else 0


if __name__ == "__main__":
    sys.exit(main())
