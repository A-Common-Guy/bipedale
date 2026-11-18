#!/usr/bin/env python3
"""Record a trial of the policies and which of them was running when.

    python3 policy_record.py                 # on the robot, the stack running
    python3 policy_record.py --analyse DIR   # afterwards, anywhere

Records until Ctrl-C (the panel's Stop) or --max seconds. Nothing is
commanded: the operator starts, switches and drives the controllers as usual
(Run page or gamepad). DIR holds the ros2 bag (zstd) and plan.json with:

  events       {"kind": "controller", "name": ..., "t": ...} each time the
               controller that owns the drives changes (name null: none);
               t is the wall clock, the clock of the bag's header stamps.
  controllers  the parameters of every controller that ran, read from its
               node when it first appears: policy_file, policy_fingerprint,
               the wheel filters, the gains.
  version      the software the robot ran (version/build.json).

The analysis cuts the recording at those events and, for each stretch of one
controller, splits it into 2 s windows of standing, driving and lying down,
and gives the gyro's RMS and strongest frequency in buzz_record.py's bands:
summary.json, one row per stretch, for whoever trains the next policy.
"""

import argparse
import json
import math
import signal
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import buzz_record as buzz  # noqa: E402

TOPICS = buzz.TOPICS + ["/terrain_profile", "/joy", "/controller_manager/activity"]
WINDOW_S = 2.0
# A window is driving above this forward speed or turn rate, lying down above this tilt.
DRIVING_M_S, TURNING_RAD_S, DOWN_RAD = 0.05, 0.2, 0.9
AXES = ("gyro yaw", "gyro pitch", "gyro roll")
VERSION_FILE = "/workspaces/sandro/version/build.json"


class Watcher:
    """The controller that owns the drives, and its parameters, from the controller manager."""

    def __init__(self):
        import rclpy
        from controller_manager_msgs.srv import ListControllers
        from rclpy.signals import SignalHandlerOptions
        # Ctrl-C is this tool's, to close the recording: not rclpy's.
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.rclpy = rclpy
        self.node = rclpy.create_node("policy_record")
        self.request = ListControllers.Request
        self.client = self.node.create_client(ListControllers, "/controller_manager/list_controllers")

    def _call(self, client, request, timeout=2.0):
        if not client.service_is_ready():
            return None
        future = client.call_async(request)
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        return future.result() if future.done() else None

    def owner(self):
        """The active controller with command interfaces; None: none; False: no answer."""
        answer = self._call(self.client, self.request())
        if answer is None:
            return False
        owners = [c.name for c in answer.controller if c.state == "active" and c.claimed_interfaces]
        return owners[0] if owners else None

    def parameters(self, name):
        from rcl_interfaces.srv import GetParameters, ListParameters
        from rclpy.parameter import parameter_value_to_python
        lister = self.node.create_client(ListParameters, f"/{name}/list_parameters")
        getter = self.node.create_client(GetParameters, f"/{name}/get_parameters")
        try:
            if not lister.wait_for_service(timeout_sec=2.0):
                return None
            listed = self._call(lister, ListParameters.Request(depth=0))
            names = sorted(listed.result.names) if listed else []
            answer = self._call(getter, GetParameters.Request(names=names)) if names else None
            if answer is None:
                return None
            values = {}
            for key, value in zip(names, answer.values):
                value = parameter_value_to_python(value)
                values[key] = list(value) if hasattr(value, "__iter__") and not isinstance(value, str) else value
            return values
        finally:
            self.node.destroy_client(lister)
            self.node.destroy_client(getter)

    def close(self):
        self.node.destroy_node()
        self.rclpy.try_shutdown()


def read_version(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None


def record(args, watcher=None):
    out = Path(args.out)
    print(f"recording the controllers that run, for at most {args.max:.0f} s, into {out}")
    out.mkdir(parents=True, exist_ok=True)
    log = open(out / "record.log", "w")
    recorder = subprocess.Popen(
        ["ros2", "bag", "record", "-s", "mcap", "--storage-preset-profile", "zstd_fast",
         "-o", str(out / "bag"), *TOPICS, "--regex", ".*/diagnostics"],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    events, controllers = [], {}
    plan = {"args": vars(args), "version": read_version(args.version_file),
            "events": events, "controllers": controllers}
    reason = None

    def save():
        (out / "plan.json").write_text(json.dumps(plan, indent=1, default=str))

    def stop(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        watcher = watcher or Watcher()
        time.sleep(3.0)                     # the recorder subscribing
        start = time.time()
        events.append({"kind": "start", "t": start})
        current = False                     # nothing known yet
        while time.time() - start < args.max:
            if recorder.poll() is not None:
                raise RuntimeError(f"ros2 bag record exited: {out / 'record.log'}")
            name = watcher.owner()
            if name is not False and name != current:
                current = name
                events.append({"kind": "controller", "name": name, "t": time.time()})
                if name and name not in controllers:
                    controllers[name] = watcher.parameters(name)
                save()
                elapsed = time.time() - start
                print(f"NOW: recording {name or 'no controller'} (since {elapsed:.0f} s). "
                      "Switch and drive as you like; Stop measurement ends the recording.", flush=True)
            time.sleep(0.4)
        print("NOW: the longest recording is reached, closing", flush=True)
    except KeyboardInterrupt:
        print("NOW: closing the recording", flush=True)
    except RuntimeError as error:
        reason = str(error)
    finally:
        events.append({"kind": "aborted", "reason": reason, "t": time.time()} if reason
                      else {"kind": "end", "t": time.time()})
        save()
        if reason:
            print(f"stopped: {reason}", flush=True)
        if recorder.poll() is None:
            recorder.send_signal(signal.SIGINT)
            try:
                recorder.wait(timeout=15)
            except subprocess.TimeoutExpired:
                recorder.kill()
        if watcher is not None:
            watcher.close()
    return reason


# Analysis -----------------------------------------------------------------------

def stretches(events):
    """[(controller, t0, t1)] from the plan's events: one per stay of a controller."""
    end = max((e["t"] for e in events), default=0.0)
    changes = [e for e in events if e["kind"] == "controller"]
    result = []
    for event, following in zip(changes, changes[1:] + [None]):
        t1 = following["t"] if following else end
        if event["name"] and t1 > event["t"]:
            result.append((event["name"], event["t"], t1))
    return result


def read_bag(path):
    """(t, right, left wheel velocity) and (t, gyro x, y, z, tilt from upright), by header stamp."""
    import numpy as np
    wheels, imu = [], []
    for topic, msg, received_ns in buzz.messages(path, {"/joint_states", "/imu_broadcaster/imu"}):
        t = buzz.stamp(msg, received_ns)
        if topic == "/joint_states":
            wheels.append([t] + [msg.velocity[msg.name.index(n)] for n in buzz.WHEELS])
        else:
            g, q = msg.angular_velocity, msg.orientation
            upright = max(-1.0, min(1.0, 1.0 - 2.0 * (q.x * q.x + q.y * q.y)))
            imu.append((t, g.x, g.y, g.z, math.acos(upright)))
    return np.array(wheels), np.array(imu)


def windows(wheels, imu, t0, t1):
    """The 2 s windows of one stretch: [{state, speed, bands: {band: {axis: (rms, hz)}}}]."""
    import numpy as np
    result = []
    for start in np.arange(t0, t1 - WINDOW_S + 1e-9, WINDOW_S):
        grid = np.arange(start, start + WINDOW_S, 1 / buzz.FS)
        wr, wl = np.interp(grid, wheels[:, 0], wheels[:, 1]), np.interp(grid, wheels[:, 0], wheels[:, 2])
        gyro = {"gyro roll": np.interp(grid, imu[:, 0], imu[:, 1]),
                "gyro pitch": np.interp(grid, imu[:, 0], imu[:, 2]),
                "gyro yaw": np.interp(grid, imu[:, 0], imu[:, 3])}
        tilt = np.interp(grid, imu[:, 0], imu[:, 4])
        speed = float(np.mean(0.5 * (wr + wl)) * buzz.RADIUS)
        state = ("down" if tilt.max() > DOWN_RAD else
                 "driving" if abs(speed) > DRIVING_M_S or abs(gyro["gyro yaw"].mean()) > TURNING_RAD_S
                 else "standing")
        result.append({"state": state, "speed": speed,
                       "bands": {f"{lo:g}-{hi:g} Hz": {axis: buzz.band_content(x, lo, hi)
                                                         for axis, x in gyro.items()}
                                 for lo, hi in buzz.BANDS}})
    return result


def loudness(rows):
    """Per band and axis over some windows: the typical (median window) and the worst (rms, hz)."""
    result = {}
    for band in (rows[0]["bands"] if rows else ()):
        result[band] = {}
        for axis in AXES:
            ordered = sorted(row["bands"][band][axis] for row in rows)
            result[band][axis] = {"typical": ordered[len(ordered) // 2], "worst": ordered[-1]}
    return result


def describe(row):
    """One line for a stretch."""
    parts = [f"{row['controller']} {row['seconds']:.0f} s"]
    standing = row["standing"]
    if standing["seconds"]:
        rms, hz, axis, band = max((values["typical"][0], values["typical"][1], axis, band)
                                  for band, axes in standing["bands"].items() for axis, values in axes.items())
        parts.append(f"quiet standing ({rms:.2f} rad/s)" if rms < buzz.QUIET_RAD_S else
                     f"buzz standing {rms:.2f} rad/s of {axis} at {hz:.1f} Hz")
    else:
        parts.append("never stood still")
    if row["driving"]["seconds"]:
        parts.append(f"drove {row['driving']['seconds']:.0f} s to {row['driving']['top_speed_m_s']:.1f} m/s")
    if row["down_s"]:
        parts.append(f"lying {row['down_s']:.0f} s")
    return ", ".join(parts)


def analyse(directory):
    directory = Path(directory)
    plan = json.loads((directory / "plan.json").read_text())
    wheels, imu = read_bag(directory / "bag")
    if len(wheels) < buzz.FS or len(imu) < buzz.FS:
        raise RuntimeError("the recording holds no joint states or no IMU: was the stack running?")
    start = next((e["t"] for e in plan["events"] if e["kind"] == "start"), wheels[0, 0])
    last = min(wheels[-1, 0], imu[-1, 0])
    rows = []
    for name, t0, t1 in stretches(plan["events"]):
        found = windows(wheels, imu, t0 + 1.0, min(t1, last))   # the switch itself left out
        by = {state: [w for w in found if w["state"] == state] for state in ("standing", "driving", "down")}
        parameters = (plan.get("controllers") or {}).get(name) or {}
        rows.append({
            "controller": name, "from_s": t0 - start, "seconds": t1 - t0,
            "policy_file": parameters.get("policy_file"),
            "policy_fingerprint": parameters.get("policy_fingerprint"),
            "standing": {"seconds": WINDOW_S * len(by["standing"]), "bands": loudness(by["standing"])},
            "driving": {"seconds": WINDOW_S * len(by["driving"]), "bands": loudness(by["driving"]),
                        "top_speed_m_s": max((abs(w["speed"]) for w in by["driving"]), default=0.0)},
            "down_s": WINDOW_S * len(by["down"]),
        })
    lines = [describe(row) for row in rows]
    for row, line in zip(rows, lines):
        print(f"  {row['from_s']:5.0f} s  {line}")
    verdict = "; ".join(lines) if lines else "no controller ran during the recording"
    print(verdict if not lines else f"{len(rows)} stretches of "
          f"{len({row['controller'] for row in rows})} controllers")
    (directory / "summary.json").write_text(json.dumps(
        {"verdict": verdict, "version": plan.get("version"), "stretches": rows}, indent=1, default=float))
    return verdict


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--max", type=float, default=900.0, help="s after which the recording closes by itself")
    parser.add_argument("--out", default=f"/workspaces/sandro/experiments/policies_{time.strftime('%Y%m%d-%H%M%S')}")
    parser.add_argument("--version-file", default=VERSION_FILE, help="the running software's stamp")
    parser.add_argument("--analyse", metavar="DIR", help="analyse a finished recording instead")
    args = parser.parse_args()
    if args.analyse:
        analyse(args.analyse)
        return 0
    reason = record(args)
    if (Path(args.out) / "bag").exists():
        print(f"recorded: {args.out}")
        try:
            analyse(args.out)
        except Exception as error:  # noqa: BLE001 - the recording is what matters
            print(f"analysis failed ({error}); run: python3 policy_record.py --analyse {args.out}")
    return 1 if reason else 0


if __name__ == "__main__":
    sys.exit(main())
