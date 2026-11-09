"""The panel: every source behind one snapshot, the pre-flight checks, and the actions."""

from __future__ import annotations

import hashlib
import json
import shlex
import threading
import time
from contextlib import contextmanager
from pathlib import Path

import yaml

from . import gamepad
from .can_monitor import ESTOP_REQUESTED, drive_faults
from .control import ArmFailed, Control, condition, owning, short_type
from .errors import Busy
from .launcher import build_command
from .logbook import ERROR, Logbook

OK, WARN, ERR, OFF = "ok", "warn", "error", "off"
MANAGER = "ros2_control_node"
# A run still not ready after this long says so in the Stack check.
SLOW_START_S = 45.0


def _check(id_, label, level, value, detail="", short=None, **extra):
    return {"id": id_, "label": label, "level": level, "value": value, "detail": detail,
            "short": short or value, **extra}


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
    return "IMU check" if request.get("mode") == "imu" else "stack"


def readiness(request: dict, ros: dict | None, component: str = "SandroSystem",
              imu_component: str = "SandroImu", broadcasters: list[str] | None = None) -> list[str]:
    """What a starting launch still lacks; empty once it is ready.

    Ready: the hardware components listed in their expected states (a real
    stack: the IMU active, the drives configured, armed or not), the
    broadcasters active and the requested controller loaded.
    """
    if not ros or not ros.get("manager"):
        return ["the controller manager"]
    hardware = {h["name"]: h["state"] for h in ros.get("hardware", [])}
    controllers = {c["name"]: c["state"] for c in ros.get("controllers", [])}
    waiting = []
    imu_only = request.get("mode") == "imu" and request.get("hardware") != "mujoco"
    if request.get("hardware") == "mujoco":
        if not hardware or any(state != "active" for state in hardware.values()):
            waiting.append("the simulated hardware active")
    else:
        if hardware.get(imu_component) != "active":
            waiting.append(f"{imu_component} active")
        if not imu_only and hardware.get(component) not in ("inactive", "active"):
            waiting.append(f"{component} configured" + (f" (it is {hardware[component]})"
                                                        if component in hardware else ""))
    wanted = ["imu_broadcaster"] if imu_only else list(broadcasters or [])
    waiting += [f"{b} active" for b in wanted if controllers.get(b) != "active"]
    controller = request.get("controller") or "none"
    if not imu_only and controller != "none" and controller not in controllers:
        waiting.append(f"{controller} loaded")
    return waiting


def stack_state(launch: dict, ros: dict | None, session: dict | None, simulation: bool,
                component: str = "SandroSystem", waiting: list[str] | None = None) -> dict:
    """The Stack check, with `phase`: stopped, failed, starting, died, stopping, simulation,
    imu, armed, not_armed, estopped, unconfigured or no_manager."""
    session = session or {}
    state = launch.get("state")
    run = launch.get("run") or {}
    if state == "stopping":
        return _check("stack", "Robot stack", WARN, "stopping", "", phase="stopping")
    if state not in ("starting", "running", "external"):
        last = launch.get("last_exit") or {}
        failed = last.get("code") not in (None, 0, -2)
        return _check("stack", "Robot stack", ERR if failed else OFF, "stopped",
                      f"last run exited with code {last['code']}" if failed else "start it from Run",
                      phase="failed" if failed else "stopped")
    external = state == "external"
    mode = launch_mode(run.get("request")) if run else "running"
    prefix = "external" if external else "running"
    up = "" if external else f"up {_duration(run.get('uptime_s', 0))}"
    died = sorted(set(run.get("died") or []))
    if MANAGER in died:
        return _check("stack", "Robot stack", ERR, f"{prefix} · controller manager died",
                      "ros2_control_node exited inside the launch (the Run log says why, usually "
                      "a drive that refused to configure): Stop, then Start again", "died",
                      phase="died")
    others = f"died: {', '.join(died)} (see the Run log)" if died else ""

    def done(level, value, detail, short, phase):
        if others and level == OK:
            level = WARN
        detail = " · ".join(filter(None, [detail, others, up]))
        value = f"starting · {mode}" if phase == "starting" else f"{prefix} · {value}"
        return _check("stack", "Robot stack", level, value, detail, short, phase=phase)

    if state == "starting":
        slow = run.get("uptime_s", 0) > SLOW_START_S
        return done(WARN, "", ("still waiting for " if slow else "waiting for ")
                    + ", ".join(waiting) if waiting else "", "starting", "starting")
    if simulation or mode == "simulation":
        return done(OK, "simulation", "", "sim", "simulation")
    if mode == "IMU check":
        return done(OK, "IMU check", "", "IMU check", "imu")
    ros = ros or {}
    if not ros.get("manager"):
        return done(WARN, "no controller manager", "", "no manager", "no_manager")
    system = {h["name"]: h["state"] for h in ros.get("hardware", [])}.get(component)
    if system == "active":
        return done(OK, "armed", "", "armed", "armed")
    if session.get("estopped"):
        return done(WARN, "E-stopped", f"drives {system or 'gone'}: Re-arm on the Run page, or hold "
                    "Create + Options on the pad", "E-stopped", "estopped")
    if system == "inactive":
        return done(OK, "drives not armed", "", "not armed", "not_armed")
    return done(WARN, f"drives {system or 'not listed'}",
                "a drive did not configure: see the Drives table and the Run log", "drives off",
                "unconfigured")


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
    checks.append(stack_state(launch, slow.get("ros"), slow.get("session"), simulation, component,
                              slow.get("waiting")))

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
                                 "published once the stack (or the IMU check) runs"))
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
    died = MANAGER in (run.get("died") or [])
    if ros_slow.get("manager"):
        controllers = ros_slow.get("controllers", [])
        running = [c for c in controllers if c.get("owns_commands")]
        hardware = {h["name"]: h["state"] for h in ros_slow.get("hardware", [])}
        system = hardware.get(component)
        armed = system == "active" or simulation
        fault = next((c for c in running if c.get("fault")), None)
        if fault:
            checks.append(_check("controller", "Controller", ERR,
                                 f"{fault['name']} {fault['fault']['state']}",
                                 f"{fault['fault']['text']}: Restart it on the Run page",
                                 fault["fault"]["state"]))
        else:
            names = [c["name"] for c in running]
            drives = "armed" if system == "active" else (
                "E-stopped" if (slow.get("session") or {}).get("estopped") else system or "n/a")
            checks.append(_check("controller", "Controller", OK if names else (WARN if armed else OFF),
                                 ", ".join(names) if names else "none running", f"drives {drives}",
                                 names[0].replace("_controller", "") if names else "none"))
    elif died or (state in ("starting", "running") and run.get("uptime_s", 0) > 20):
        checks.append(_check("controller", "Controller", ERR, "controller manager down",
                             "it exited inside the running launch (see the Run log, usually a "
                             "drive that refused to configure): Stop, then Start again", "down"))
    else:
        checks.append(_check("controller", "Controller", OFF, "no controller manager",
                             "starts with the stack"))

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


def controller_catalog(controllers_yaml: str | Path) -> dict[str, dict]:
    """{name: {"type", "start_lying"}} from controllers.yaml and rl_policies.yaml beside it."""
    path = Path(controllers_yaml)
    sections: dict = {}
    for file in (path, path.with_name("rl_policies.yaml")):
        try:
            data = yaml.safe_load(file.read_text()) or {}
        except (OSError, yaml.YAMLError):
            continue
        manager = (data.pop("controller_manager", {}) or {}).get("ros__parameters", {})
        for name, entry in manager.items():
            if isinstance(entry, dict) and "type" in entry:
                sections.setdefault(name, {})["type"] = short_type(entry["type"])
        for name, section in data.items():
            params = (section or {}).get("ros__parameters", {}) if isinstance(section, dict) else {}
            lying = bool((params.get("safety") or {}).get("start_lying", False))
            sections.setdefault(name, {})["start_lying"] = lying
    return {name: {"type": s.get("type", ""), "start_lying": s.get("start_lying", False)}
            for name, s in sections.items() if "type" in s}


def _new_session(key) -> dict:
    return {"key": key, "armed_once": False, "estopped": False, "last_controller": None,
            "system": None, "disarming": False}


class Panel:
    def __init__(self, cfg: dict, paths: dict, sources: dict, simulation: bool) -> None:
        self.cfg = cfg
        self.paths = paths
        self.simulation = simulation
        self.log: Logbook = sources["log"]
        self.launcher = sources["launcher"]
        self.system = sources["system"]
        self.can = sources.get("can")
        self.estop_bus = sources.get("estop_bus")
        self.pad = sources["pad"]
        self.bt = sources["bt"]
        self.ros = sources.get("ros")
        self.expect = cfg.get("expect", {})
        robot = cfg.get("robot", {})
        self.component = robot.get("hardware_component", "SandroSystem")
        self.imu_component = robot.get("imu_component", "SandroImu")
        self.broadcasters = list(robot.get("broadcasters", ["joint_state_broadcaster"]))
        self.topic_hz = {t["name"]: t.get("expect_hz", 0) for t in cfg.get("topics", [])}
        teleop = yaml.safe_load(Path(paths["teleop"]).read_text())
        self.guide = gamepad.guide(teleop)
        self.teleop = teleop["joy_teleop"]["ros__parameters"]
        self.catalog = controller_catalog(paths["controllers"]) if paths.get("controllers") else {}
        self.control = Control(self.ros, self.log, self.component, self.broadcasters,
                               self.teleop.get("policies", [])) if self.ros is not None else None
        self.session = _new_session(None)
        self._waiting: list[str] = []
        self._action_lock = threading.Lock()
        self._busy: str | None = None
        self._model: tuple[dict, bytes] | None = None
        self._model_error: str | None = None
        threading.Thread(target=self._build_model, name="model", daemon=True).start()
        if self.ros is not None and hasattr(self.launcher, "on_start"):
            self.launcher.on_start(self.ros.new_run)

    # Model --------------------------------------------------------------

    def _build_model(self) -> None:
        from . import model
        mjcf = Path(self.paths["mjcf"])
        settings = self.cfg.get("model", {})
        cell, minimum = settings.get("cell_mm", 1.0), settings.get("min_part_mm", 10.0)
        try:
            stamps = []
            for path in model.sources(mjcf):
                try:
                    stat = path.stat()
                    stamps.append(f"{path.name}:{stat.st_mtime_ns}:{stat.st_size}")
                except OSError:
                    stamps.append(f"{path.name}:missing")
            key = hashlib.sha1(f"{mjcf.resolve()}|{'|'.join(stamps)}|{cell}|{minimum}|2".encode()
                               ).hexdigest()[:12]
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
            "controller_info": {name: self.catalog.get(name, {}) for name in launch.controllers},
            "component": self.component,
            "launch_args": launch.args,
            "overrides": launch.overrides_path,
            "teleop": {k: self.teleop.get(k) for k in ("policies", "policy_colors",
                                                        "speed_fraction")},
            "guide": self.guide,
            "expect": self.expect,
            "topics": self.cfg.get("topics", []),
            "ros": self.ros is not None,
            "estop": {"ros": self.ros is not None, "can": self.estop_bus is not None},
            "bluetooth": getattr(self.bt, "available", False),
            "can_monitor": self.can is not None,
        }

    def sample(self) -> None:
        if self.ros is not None and hasattr(self.ros, "sample"):
            self.ros.sample()

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

    def _ros_slow(self) -> dict | None:
        if self.ros is None:
            return None
        ros = self.ros.slow()
        diagnostics = ros.get("diagnostics") or {}
        ros["controllers"] = [dict(c, fault=condition(c.get("type"), diagnostics.get(c["name"])))
                              if c.get("state") == "active" else c
                              for c in ros.get("controllers", [])]
        return ros

    def _track(self, launch: dict, ros: dict | None, can: dict | None) -> None:
        """Session bookkeeping and readiness, at the slow rate whether or not a browser looks."""
        run = launch.get("run") or {}
        if run:
            key = f"run:{run['pid']}:{run['started']}"
        elif launch.get("external"):
            key = f"external:{launch['external'][0]['pid']}"
        else:
            key = None
        if key != self.session["key"]:
            self.session = _new_session(key)
            if self.control is not None:
                self.control.last_controller = None
        s = self.session
        manager = bool(ros and ros.get("manager"))
        hardware = {h["name"]: h["state"] for h in ros.get("hardware", [])} if manager else {}
        system = hardware.get(self.component)
        if system == "active":
            s["armed_once"], s["estopped"] = True, False
        elif system is not None and s["system"] == "active" and not s["disarming"]:
            # Active a moment ago and not disarmed here: the gamepad E-stop or a drive fault.
            s["estopped"] = True
        if system is not None and not s["disarming"]:
            s["system"] = system
        latched = [n for n in (can or {}).get("nodes", [])
                   if n.get("joint") and (n.get("axis_error") or 0) & ESTOP_REQUESTED]
        if key is not None and latched and system in ("inactive", "unconfigured"):
            s["estopped"] = True
        running = owning({c["name"]: c for c in (ros or {}).get("controllers", [])}) if manager \
            else []
        if running:
            s["last_controller"] = running[0]
        elif self.control is not None and self.control.last_controller:
            s["last_controller"] = self.control.last_controller
        if launch.get("state") == "starting":
            self._waiting = readiness(run.get("request") or {}, ros, self.component,
                                      self.imu_component, self.broadcasters)
            if not self._waiting and hasattr(self.launcher, "mark_ready"):
                self.launcher.mark_ready()
        else:
            self._waiting = []

    def idle(self) -> None:
        self._track(self.launcher.status(), self._ros_slow(),
                    self.can.snapshot() if self.can else None)

    def slow(self) -> dict:
        pad = self.pad.snapshot()
        system = self.system.snapshot()
        launch = self.launcher.status()
        ros = self._ros_slow()
        can = self.can.snapshot() if self.can else None
        self._track(launch, ros, can)
        run = launch.get("run") or {}
        last = self.session["last_controller"] or (run.get("request") or {}).get("controller")
        return {
            "t": time.time(),
            "launch": launch,
            "ros": ros,
            "can": can,
            "can_link": system.pop("can", None),
            "system": system,
            "bt": self.bt.snapshot(),
            "pad": {"device": pad.get("device"), "name": pad.get("name"),
                    "error": pad.get("error"), "battery": gamepad.battery()},
            "problems": self.log.problems(),
            "log_seq": self.log.seq,
            "session": {"armed_once": self.session["armed_once"],
                        "estopped": self.session["estopped"],
                        "last_controller": None if last == "none" else last},
            "waiting": list(self._waiting),
            "busy": self._busy,
        }

    def checks(self, slow: dict, fast: dict) -> list[dict]:
        return health(slow, fast, self.expect, self.simulation, self.component, self.topic_hz)

    # Actions ------------------------------------------------------------

    def _ros(self):
        if self.ros is None:
            raise RuntimeError("ROS is not available in this process")
        return self.ros

    def _control(self) -> Control:
        if self.control is None:
            raise RuntimeError("ROS is not available in this process")
        return self.control

    @contextmanager
    def _exclusive(self, what: str):
        """One controller-manager action at a time, from any tab: a second click is refused."""
        if not self._action_lock.acquire(blocking=False):
            raise Busy(f"{self._busy or 'another action'} is still in progress")
        self._busy = what
        try:
            yield
        finally:
            self._busy = None
            self._action_lock.release()

    def emergency_stop(self) -> str:
        """Never waits for another action: the CAN frame goes out first, then the rest."""
        direct = self.estop_bus.send if self.estop_bus is not None else None
        if self.control is not None:
            message = self.control.emergency_stop(direct)
        elif direct is not None:
            message = direct() + "; ROS is not available in this process"
        else:
            raise RuntimeError("ROS is not available: use the power cut-off")
        self.session["estopped"] = True
        self.log.note("E-STOP from the panel: " + message, ERROR)
        return message

    def arm_failure(self, component: str, error: Exception) -> str:
        """Why the drives did not arm, drive by drive, and what to do about it."""
        faults = []
        if self.can is not None:
            snapshot = self.can.snapshot()
            if snapshot.get("open"):
                faults = drive_faults(snapshot.get("nodes", []))
        if not faults and self.ros is not None:
            dynamic = self.ros.slow().get("dynamic") or {}
            faults = [f"{joint} reports error {int(values['error'])}: the Run log names it"
                      for joint, values in dynamic.items() if values.get("error")]
        if faults:
            return f"{error}. " + "; ".join(faults) + "."
        return (f"{error}. No drive reports an error on the bus: the Run log names the cause "
                "(a MIT frame bias above 0.2 rad means a drive zero is off).")

    def _arm(self, component: str, body: dict, controller: str | None = None) -> str:
        if not self.session["armed_once"] and not body.get("secured"):
            raise ValueError("confirm first that the robot is secured and the power cut-off is "
                             "in reach")
        control = self._control()
        try:
            message = control.arm_and_start(controller, component) if controller \
                else control.arm(component)
        except ArmFailed as error:
            raise RuntimeError(self.arm_failure(component, error)) from error
        self.session.update(armed_once=True, estopped=False, system="active")
        return message

    def _disarm(self, component: str) -> str:
        self.session["disarming"] = True
        try:
            message = self._control().disarm(component)
            self.session.update(estopped=False, system="inactive")
        finally:
            self.session["disarming"] = False
        return message

    def act(self, action: str, body: dict) -> dict:
        name = body.get("name", "")
        if action == "estop":
            return {"message": self.emergency_stop()}
        if action == "launch/preview":
            return {"command": shlex.join(build_command(body, self.launcher.settings))}
        if action == "launch/start":
            return {"status": self.launcher.start(body)}
        if action == "launch/stop":
            return {"status": self.launcher.stop()}
        manager = {
            "controller/activate": lambda c: c.activate(name, bool(body.get("swap"))),
            "controller/deactivate": lambda c: c.deactivate(name),
            "controller/restart": lambda c: c.restart(name),
            "controller/configure": lambda c: c.configure(name),
            "hardware/activate": lambda c: self._arm(name or self.component, body),
            "hardware/deactivate": lambda c: self._disarm(name or self.component),
            "arm_and_start": lambda c: self._arm(self.component, body, body.get("controller")),
            "clear_errors": lambda c: c.clear_errors(),
        }
        if action in manager:
            if action == "arm_and_start" and not body.get("controller"):
                raise ValueError("arm_and_start needs a controller")
            control = self._control()
            label = {"clear_errors": "clearing drive errors",
                     "arm_and_start": f"arming and starting {body.get('controller')}"}.get(
                action, f"{action.replace('/', ' ')} {name}".strip())
            with self._exclusive(label):
                return {"message": manager[action](control)}
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
            if verb == "watch":
                return self.bt.watch(float(body.get("seconds", 15)))
            if verb == "scan":
                return self.bt.scan(float(body.get("seconds", 60)))
            if verb == "stop_scan":
                return self.bt.stop_scan()
            if verb == "unblock":
                return self.bt.unblock()
            if verb in ("pair", "connect", "disconnect", "remove"):
                return getattr(self.bt, verb)(body.get("mac", ""))
        raise KeyError(action)
