"""The panel: every source behind one snapshot, the pre-flight checks, and the actions."""

from __future__ import annotations

import hashlib
import json
import shlex
import threading
import time
from pathlib import Path

import yaml

from . import gamepad
from .launcher import build_command
from .logbook import ERROR, Logbook

OK, WARN, ERR, OFF = "ok", "warn", "error", "off"


def _check(id_, label, level, value, detail="", short=None):
    return {"id": id_, "label": label, "level": level, "value": value, "detail": detail,
            "short": short or value}


def _duration(seconds: float) -> str:
    seconds = int(seconds)
    if seconds < 60:
        return f"{seconds} s"
    if seconds < 3600:
        return f"{seconds // 60} min {seconds % 60:02d} s"
    return f"{seconds // 3600} h {seconds // 60 % 60:02d} min"


def launch_mode(request: dict | None) -> str:
    if not request:
        return "running"
    if request.get("hardware") == "mujoco":
        return "simulation"
    return {"imu": "IMU check", "drives_off": "drives not armed",
            "armed": "armed"}.get(request.get("mode"), "running")


def health(slow: dict, fast: dict, expect: dict, simulation: bool,
           component: str = "SandroSystem", topic_hz: dict | None = None) -> list[dict]:
    """The pre-flight list: one row per subsystem, worst first within the page order.

    `topic_hz` is each watched topic's expected rate (gui.yaml topics).
    """
    topic_hz = topic_hz or {}
    checks = []
    launch = slow.get("launch") or {}
    state = launch.get("state")
    run = launch.get("run") or {}
    if state in ("running", "stopping", "starting"):
        mode = launch_mode(run.get("request"))
        checks.append(_check("stack", "Robot stack", WARN if state == "stopping" else OK,
                             f"{state} · {mode}", f"up {_duration(run.get('uptime_s', 0))}",
                             "stopping" if state == "stopping" else mode))
    elif state == "external":
        checks.append(_check("stack", "Robot stack", WARN, "running outside the panel",
                             (launch.get("external") or [{}])[0].get("command", ""), "external"))
    else:
        last = launch.get("last_exit") or {}
        failed = last.get("code") not in (None, 0, -2)
        checks.append(_check("stack", "Robot stack", ERR if failed else OFF, "stopped",
                             f"last run exited with code {last['code']}" if failed else
                             "start it from Run"))

    can = slow.get("can_link") or {}
    if simulation:
        checks.append(_check("can", "CAN bus", OFF, "simulation"))
    elif not can.get("present"):
        checks.append(_check("can", "CAN bus", ERR, "missing", can.get("error", "")))
    elif not can.get("up"):
        checks.append(_check("can", "CAN bus", ERR, "down", "can0.service brings it up at boot"))
    else:
        bus = can.get("state") or "?"
        level = OK if bus == "ERROR-ACTIVE" else ERR if bus in ("BUS-OFF", "ERROR-PASSIVE") else WARN
        rate = can.get("rx_hz")
        bitrate = can.get("bitrate")
        notes = [f"{rate:.0f} frames/s received, {can.get('tx_hz', 0):.0f} sent"
                 if rate is not None else ""]
        if can.get("berr_tx") is not None:
            notes.append(f"bus errors tx {can['berr_tx']} rx {can.get('berr_rx', 0)}")
        if can.get("rx_errors"):
            notes.append(f"{can['rx_errors']} rx errors")
        checks.append(_check(
            "can", "CAN bus", level,
            f"{bus.lower()} · {bitrate / 1e6:g} Mbit/s" if bitrate else bus.lower(),
            " · ".join(n for n in notes if n), "ok" if level == OK else bus.lower()))

    nodes = [n for n in (slow.get("can") or {}).get("nodes", []) if n.get("joint")]
    dynamic = (slow.get("ros") or {}).get("dynamic") or {}
    if simulation:
        checks.append(_check("drives", "Drives", OFF, "simulation"))
    elif nodes:
        heard = [n for n in nodes if n.get("heartbeat_age_s") is not None
                 and n["heartbeat_age_s"] < 1.0]
        silent = [f"{n['joint']} (node {n['node']})" for n in nodes if n not in heard]
        faults = [f"{n['joint']} {n['error_text']}" for n in nodes if n.get("error_text")]
        for name, values in dynamic.items():
            if values.get("online") == 0.0 and not any(name in s for s in silent):
                silent.append(name)
        temps = [n["temperature"] for n in nodes if n.get("temperature") is not None]
        level = ERR if silent else WARN if faults else OK
        detail = "; ".join(filter(None, ["silent: " + ", ".join(silent) if silent else "",
                                         "errors: " + ", ".join(faults) if faults else "",
                                         f"max {max(temps)} °C" if temps else ""]))
        checks.append(_check("drives", "Drives", level, f"{len(heard)}/{len(nodes)} heard", detail,
                             f"{len(heard)}/{len(nodes)}" + (" error" if faults else "")))
    else:
        checks.append(_check("drives", "Drives", OFF, "no CAN monitor"))

    ros = fast.get("ros") or {}
    rates = ros.get("rates") or {}
    imu_rate = (rates.get("/imu_broadcaster/imu") or {}).get("hz", 0.0)
    # The driver's own sample age where it exists (real IMU), else the message age.
    sensor_age = ros.get("imu_sensor_age_s")
    imu_age = sensor_age if sensor_age is not None else ros.get("imu_age_s")
    max_age = expect.get("imu_max_age_ms", 25) / 1e3
    stack_up = state in ("running", "external")
    devices = (slow.get("system") or {}).get("devices") or {}
    missing = [path for path, present in devices.items() if not present and "spidev" in path]
    if ros.get("imu") is None or imu_age is None:
        if missing and not simulation:
            checks.append(_check("imu", "IMU", ERR, "no SPI device",
                                 f"{missing[0]} is missing: dtoverlay=spi6-1cs in config.txt"))
        else:
            checks.append(_check("imu", "IMU", ERR if stack_up and
                                 (slow.get("ros") or {}).get("manager") else OFF, "no data",
                                 "start the stack armed, or the IMU check"))
    elif imu_age > 0.2:
        checks.append(_check("imu", "IMU", ERR, f"stale {imu_age:.1f} s", "no new BNO085 sample"))
    else:
        wanted = 0.8 * topic_hz.get("/imu_broadcaster/imu", 0)
        level = OK if imu_age <= max_age and imu_rate >= wanted else WARN
        source = "at the driver" if sensor_age is not None else "of the message"
        checks.append(_check("imu", "IMU", level, f"{imu_rate:.0f} Hz",
                             f"sample age {imu_age * 1e3:.0f} ms {source}"))

    joints = ros.get("joints") or {}
    if joints and ros.get("joints_age_s") is not None and ros["joints_age_s"] < 0.5:
        ages = [(j["sensor_age_s"], n) for n, j in joints.items()
                if j.get("sensor_age_s") is not None]
        rate = (rates.get("/joint_states") or {}).get("hz", 0)
        level = OK if rate >= 0.8 * topic_hz.get("/joint_states", 0) else WARN
        detail = "topic rate; the simulation has no encoder age"
        if ages:
            oldest = max(ages)
            if oldest[0] > expect.get("encoder_max_age_ms", 25) / 1e3:
                level = WARN
            detail = f"oldest encoder sample {oldest[0] * 1e3:.0f} ms ({oldest[1]})"
        checks.append(_check("encoders", "Joint states", level, f"{rate:.0f} Hz", detail))
    else:
        checks.append(_check("encoders", "Joint states", ERR if stack_up and
                             (slow.get("ros") or {}).get("manager") else OFF,
                             "no data", "published once the controller manager runs"))

    ros_slow = slow.get("ros") or {}
    if ros_slow.get("manager"):
        running = [c["name"] for c in ros_slow.get("controllers", []) if c.get("owns_commands")]
        hardware = {h["name"]: h["state"] for h in ros_slow.get("hardware", [])}
        system = hardware.get(component)
        armed = system == "active" or simulation
        level = OK if running else (WARN if armed else OFF)
        checks.append(_check("controller", "Controller", level,
                             ", ".join(running) if running else "none running",
                             f"drives {'armed' if system == 'active' else system or 'n/a'}",
                             (running[0].replace("_controller", "") if running else "none")))
    elif state == "running" and launch_mode(run.get("request")) in ("armed", "simulation") \
            and run.get("uptime_s", 0) > 20:
        checks.append(_check("controller", "Controller", ERR, "controller manager down",
                             "it exited inside the running launch (see the Run log, usually a "
                             "drive that refused to arm): Stop, then Start again", "down"))
    else:
        checks.append(_check("controller", "Controller", OFF, "no controller manager",
                             "starts with the armed or simulation launch"))

    pad = slow.get("pad") or {}
    joy = (rates.get("/joy") or {})
    battery = pad.get("battery")
    if pad.get("device") or (joy.get("age_s") is not None and joy["age_s"] < 1.0):
        level = WARN if battery and battery["percent"] <= 15 else OK
        teleop = "teleop on" if joy.get("age_s") is not None and joy["age_s"] < 1.0 else \
            "teleop not running"
        checks.append(_check("pad", "Gamepad", level,
                             f"{battery['percent']}%" if battery else "connected",
                             f"{pad.get('name') or 'joystick'} · {teleop}"))
    else:
        checks.append(_check("pad", "Gamepad", WARN if not simulation else OFF, "not connected",
                             "press PS, or pair it on the Gamepad page"))

    system = slow.get("system") or {}
    temp = system.get("cpu_temp_c")
    throttle = system.get("throttled") or {}
    level = OK
    notes = []
    if throttle.get("now"):
        level = ERR
        notes.append("now: " + ", ".join(throttle["now"]))
    elif throttle.get("since_boot"):
        level = WARN
        notes.append("since boot: " + ", ".join(throttle["since_boot"]))
    if temp is not None and temp >= 80:
        level = ERR
    elif temp is not None and temp >= 70 and level == OK:
        level = WARN
    load = system.get("load")
    if load:
        notes.append(f"load {load[0]:.1f}")
    checks.append(_check("host", system.get("host") or "Computer", level,
                         f"{temp:.0f} °C" if temp is not None else "—", " · ".join(notes)))
    return checks


class Panel:
    def __init__(self, cfg: dict, paths: dict, sources: dict, simulation: bool) -> None:
        self.cfg = cfg
        self.paths = paths
        self.simulation = simulation
        self.log: Logbook = sources["log"]
        self.launcher = sources["launcher"]
        self.system = sources["system"]
        self.can = sources.get("can")
        self.pad = sources["pad"]
        self.bt = sources["bt"]
        self.ros = sources.get("ros")
        self.expect = cfg.get("expect", {})
        self.topic_hz = {t["name"]: t.get("expect_hz", 0) for t in cfg.get("topics", [])}
        teleop = yaml.safe_load(Path(paths["teleop"]).read_text())
        self.guide = gamepad.guide(teleop)
        self.teleop = teleop["joy_teleop"]["ros__parameters"]
        self._model: tuple[dict, bytes] | None = None
        self._model_error: str | None = None
        threading.Thread(target=self._build_model, name="model", daemon=True).start()
        if self.ros is not None and hasattr(self.launcher, "on_start"):
            self.launcher.on_start(self.ros.reset_baselines)

    # Model --------------------------------------------------------------

    def _build_model(self) -> None:
        from . import model
        mjcf = Path(self.paths["mjcf"])
        settings = self.cfg.get("model", {})
        cell, minimum = settings.get("cell_mm", 1.0), settings.get("min_part_mm", 10.0)
        try:
            stat = mjcf.stat()
            key = hashlib.sha1(f"{mjcf.resolve()}|{stat.st_mtime_ns}|{stat.st_size}|{cell}|"
                               f"{minimum}|2".encode()).hexdigest()[:12]
            cache = Path(self.paths["cache"])
            cache.mkdir(parents=True, exist_ok=True)
            meta, blob = cache / f"model-{key}.json", cache / f"model-{key}.bin"
            if meta.is_file() and blob.is_file():
                self._model = (json.loads(meta.read_text()), blob.read_bytes())
                return
            started = time.monotonic()
            description, data = model.build(mjcf, cell, minimum)
            for old in cache.glob("model-*"):
                old.unlink(missing_ok=True)
            meta.write_text(json.dumps(description))
            blob.write_bytes(data)
            self._model = (description, data)
            self.log.note(f"3D model: {description['triangles']} triangles in "
                          f"{time.monotonic() - started:.1f} s")
        except Exception as error:  # noqa: BLE001 - the panel works without it
            self._model_error = str(error)
            self.log.note(f"3D model unavailable: {error}", ERROR)

    def model(self):
        return self._model, self._model_error

    # Snapshots ----------------------------------------------------------

    def info(self) -> dict:
        launch = self.launcher.settings
        return {
            "simulation": self.simulation,
            "hardware": launch.hardware,
            "available_hardware": list(launch.available_hardware),
            "controllers": launch.controllers,
            "launch_args": launch.args,
            "overrides": launch.overrides_path,
            "teleop": {k: self.teleop.get(k) for k in ("policies", "policy_colors",
                                                        "speed_fraction")},
            "guide": self.guide,
            "expect": self.expect,
            "topics": self.cfg.get("topics", []),
            "ros": self.ros is not None,
            "bluetooth": getattr(self.bt, "available", False),
            "can_monitor": self.can is not None,
        }

    def fast(self) -> dict:
        pad = self.pad.snapshot()
        ros = self.ros.fast() if self.ros else None
        joy_fresh = ros and ros.get("joy") and ros.get("joy_age_s") is not None \
            and ros["joy_age_s"] < 1.0
        if joy_fresh:
            pad_state = {"source": "/joy", "state": ros["joy"], "age_s": ros["joy_age_s"]}
        elif pad.get("state"):
            pad_state = {"source": pad["device"], "state": pad["state"], "age_s": pad["age_s"]}
        else:
            pad_state = {"source": None, "state": None}
        can_joints = {}
        if self.can is not None:
            for node in self.can.snapshot()["nodes"]:
                if node.get("joint") and "position" in node and node.get("encoder_age_s") is not None \
                        and node["encoder_age_s"] < 0.5:
                    can_joints[node["joint"]] = {"position": node["position"],
                                                 "velocity": node["velocity"]}
        return {"t": time.time(), "ros": ros, "pad": pad_state, "can_joints": can_joints}

    def slow(self) -> dict:
        pad = self.pad.snapshot()
        system = self.system.snapshot()
        slow = {
            "t": time.time(),
            "launch": self.launcher.status(),
            "ros": self.ros.slow() if self.ros else None,
            "can": self.can.snapshot() if self.can else None,
            "can_link": system.pop("can", None),
            "system": system,
            "bt": self.bt.snapshot(),
            "pad": {"device": pad.get("device"), "name": pad.get("name"),
                    "error": pad.get("error"), "battery": gamepad.battery()},
            "problems": self.log.problems(),
            "log_seq": self.log.seq,
        }
        return slow

    def checks(self, slow: dict, fast: dict) -> list[dict]:
        component = self.cfg.get("robot", {}).get("hardware_component", "SandroSystem")
        return health(slow, fast, self.expect, self.simulation, component, self.topic_hz)

    # Actions ------------------------------------------------------------

    def _ros(self):
        if self.ros is None:
            raise RuntimeError("ROS is not available in this process")
        return self.ros

    def act(self, action: str, body: dict) -> dict:
        name = body.get("name", "")
        if action == "launch/preview":
            return {"command": shlex.join(build_command(body, self.launcher.settings))}
        if action == "launch/start":
            return {"status": self.launcher.start(body)}
        if action == "launch/stop":
            return {"status": self.launcher.stop()}
        if action == "controller/activate":
            return {"message": self._ros().activate(name, bool(body.get("swap")))}
        if action == "controller/deactivate":
            return {"message": self._ros().deactivate(name)}
        if action == "controller/configure":
            return {"message": self._ros().configure(name)}
        if action == "hardware/activate":
            return {"message": self._ros().set_hardware(name, True)}
        if action == "hardware/deactivate":
            return {"message": self._ros().set_hardware(name, False)}
        if action == "estop":
            if self.ros is None:
                raise RuntimeError("ROS is not available: use the power cut-off")
            return {"message": self.ros.emergency_stop()}
        if action == "clear_errors":
            return {"message": self._ros().clear_errors()}
        if action == "params/set":
            ok, reason = self._ros().set_parameter(body["node"], body["name"], body.get("value"))
            if not ok:
                raise ValueError(reason or "refused")
            return {"message": "set"}
        if action == "problems/clear":
            self.log.clear_problems()
            return {"message": "cleared"}
        if action.startswith("bt/"):
            verb = action[3:]
            if verb == "scan":
                return self.bt.scan(float(body.get("seconds", 60)))
            if verb == "stop_scan":
                return self.bt.stop_scan()
            if verb == "unblock":
                return self.bt.unblock()
            if verb in ("pair", "connect", "disconnect", "remove"):
                return getattr(self.bt, verb)(body.get("mac", ""))
        raise KeyError(action)
