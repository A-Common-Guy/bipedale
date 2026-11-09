"""Synthetic sources with the real interfaces, for working on the panel without a robot.

`ros2 run sandro_gui sandro_gui --demo` (or `python3 -m sandro_gui.main --demo`
from the package directory, no ROS needed) serves the full UI with a robot
that stands, sways and answers every action. It follows the real stack: the
drives come up configured but inactive with the chosen controller loaded
inactive, an E-stop latches the drives and leaves the hardware
unconfigured, and the actions run the same sequences (control.py) against a
fake controller manager. lqr_balancer falls over 15 s after it starts, to
show a fallen controller and its Restart.
"""

from __future__ import annotations

import math
import random
import threading
import time
from pathlib import Path

import yaml

from .errors import Busy
from .launcher import LaunchSettings, build_command
from .logbook import INFO, WARN, Logbook

STAND = {"right_motor_inner": 0.411, "right_motor_outer": 0.217, "right_wheel": 0.0,
         "left_motor_inner": 0.215, "left_motor_outer": 0.413, "left_wheel": 0.0}
NODES = {1: "right_motor_inner", 2: "right_motor_outer", 3: "right_wheel",
         5: "left_motor_inner", 4: "left_motor_outer", 6: "left_wheel"}
SYSTEM, IMU = "SandroSystem", "SandroImu"
# Broadcasters that read the drives, so the drives' read error stops them.
DRIVE_BROADCASTERS = ("joint_state_broadcaster", "health_broadcaster")
BROADCASTERS = ("joint_state_broadcaster", "imu_broadcaster", "health_broadcaster")
FALLS_AFTER_S = {"lqr_balancer": 15.0}


def _flatten(tree: dict, prefix: str = "") -> dict:
    out = {}
    for key, value in tree.items():
        name = f"{prefix}{key}"
        if isinstance(value, dict):
            out.update(_flatten(value, name + "."))
        else:
            out[name] = value
    return out


class Robot:
    """The shared fake state every demo source reads."""

    def __init__(self) -> None:
        self.t0 = time.monotonic()
        self.lock = threading.RLock()
        self.running = False
        self.hardware: dict[str, str] = {}
        self.controllers: dict[str, str] = {}
        self.started: dict[str, float] = {}
        self.latched = False

    def t(self) -> float:
        return time.monotonic() - self.t0

    @property
    def armed(self) -> bool:
        return self.hardware.get(SYSTEM) == "active"

    def owning(self) -> list[str]:
        return [n for n, s in self.controllers.items() if s == "active" and "broadcaster" not in n]

    def fallen(self, name: str) -> bool:
        after = FALLS_AFTER_S.get(name)
        return after is not None and name in self.started and \
            time.monotonic() - self.started[name] > after

    def latch(self) -> None:
        """What the drives' E-stop does to the stack: read error, hardware unconfigured."""
        with self.lock:
            self.latched = True
            if not self.running or SYSTEM not in self.hardware:
                return
            self.hardware[SYSTEM] = "unconfigured"
            for name in self.owning() + [b for b in DRIVE_BROADCASTERS if b in self.controllers]:
                self.controllers[name] = "inactive"

    def joints(self) -> dict:
        t = self.t()
        moving = self.armed and any(not self.fallen(n) for n in self.owning())
        sway = 0.12 * math.sin(0.5 * t) if moving else 0.0
        out = {}
        for name, base in STAND.items():
            if name.endswith("wheel"):
                out[name] = 1.2 * math.sin(0.25 * t) if moving else 0.0
            else:
                out[name] = base + sway
        return out

    def imu(self) -> dict:
        t = self.t()
        pitch = 0.03 * math.sin(1.3 * t) + 0.004 * random.random()
        roll = 0.01 * math.sin(0.7 * t)
        yaw = 0.05 * t
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        q = (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
             cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)
        gyro = (0.007 * math.cos(0.7 * t), 0.039 * math.cos(1.3 * t), 0.05)
        accel = (-9.81 * math.sin(pitch), 9.81 * math.sin(roll), 9.81 * math.cos(pitch))
        return {"q": q, "gyro": gyro, "accel": accel}


class DemoLauncher:
    def __init__(self, robot: Robot, log: Logbook, settings: LaunchSettings,
                 partners: dict[str, list[str]]) -> None:
        self.robot, self.log, self.settings = robot, log, settings
        self.partners = partners
        self.run = None
        self.state = "stopped"
        self.last_exit = None
        self._listeners = []

    def on_start(self, callback) -> None:
        self._listeners.append(callback)

    def status(self) -> dict:
        run = dict(self.run, uptime_s=time.time() - self.run["started"]) if self.run else None
        return {"state": self.state, "run": run, "external": [], "last_exit": self.last_exit}

    def mark_ready(self) -> None:
        if self.state == "starting":
            self.state = "running"
            self.log.note(f"stack ready after {time.time() - self.run['started']:.0f} s")

    def start(self, request: dict) -> dict:
        if self.run:
            raise Busy("a launch is already running; stop it first")
        command = build_command(request, self.settings)
        self.run = {"pid": 4242, "pgid": 4242, "log": time.strftime("run-%Y%m%d-%H%M%S.log"),
                    "started": time.time(), "request": request, "command": " ".join(command),
                    "died": []}
        self.state = "starting"
        self.log.note("started: " + " ".join(command))
        for callback in self._listeners:
            callback()

        def boot():
            imu_only = request.get("mode") == "imu" and request.get("hardware") != "mujoco"
            simulation = request.get("hardware") == "mujoco"
            for line in ["[INFO] [launch]: All log files can be found below /root/.ros/log",
                         "[INFO] [robot_state_publisher-1]: process started with pid [101]",
                         "[INFO] [gim6010_estop-2]: process started with pid [102]",
                         "[INFO] [ros2_control_node-3]: process started with pid [103]"]:
                self.log.add_line(line)
                time.sleep(0.4)
            robot = self.robot
            with robot.lock:
                robot.running, robot.latched = True, False
                robot.hardware = {IMU: "active"} if imu_only else \
                    {SYSTEM: "active" if simulation else "inactive", IMU: "active"}
            self.log.add_line("[ros2_control_node-3] [INFO] [1789812592.26] [resource_manager]: "
                              f"Successful 'configure' of hardware '{SYSTEM}'")
            time.sleep(0.8)
            with robot.lock:
                for name in (("imu_broadcaster", "joint_state_broadcaster") if imu_only
                             else BROADCASTERS):
                    robot.controllers[name] = "active"
            self.log.add_line("[spawner-5] [INFO] [1789812593.1] [spawner_imu_broadcaster]: "
                              "Configured and activated imu_broadcaster")
            controller = request.get("controller", "none")
            if controller != "none" and not imu_only:
                time.sleep(0.8)
                with robot.lock:
                    robot.controllers[controller] = "active" if simulation else "inactive"
                    if simulation:
                        robot.started[controller] = time.monotonic()
                    for partner in self.partners.get(controller, []):
                        robot.controllers.setdefault(partner, "inactive")
                self.log.add_line(f"[spawner-6] [INFO] [1789812594.0] [spawner_{controller}]: "
                                  f"Configured {controller}" + ("" if simulation else
                                                                 " (inactive)"))
            self.log.add_line("[ros2_control_node-3] [WARN] [1789812594.0] [GIM6010System]: "
                              "right_wheel (node 3): MIT position bias 0.031 rad corrected")

        threading.Thread(target=boot, daemon=True).start()
        return self.status()

    def stop(self) -> dict:
        if not self.run:
            return self.status()
        self.state = "stopping"
        self.log.note("stopping launch (SIGINT to process group 4242)", WARN)

        def down():
            time.sleep(1.5)
            self.log.add_line("[ros2_control_node-3] [INFO] [1789812600.0] [GIM6010System]: "
                              "drives idle, deadman off")
            with self.robot.lock:
                self.robot.running = False
                self.robot.hardware, self.robot.controllers = {}, {}
            self.last_exit = {"code": 0, "at": time.time(), "log": self.run["log"],
                              "command": self.run["command"]}
            self.run, self.state = None, "stopped"
            self.log.note("launch exited with code 0", INFO)

        threading.Thread(target=down, daemon=True).start()
        return self.status()

    def runs_list(self) -> list[dict]:
        return [{"name": "run-20261108-171203.log", "bytes": 48213, "modified": time.time() - 3600}]

    def run_path(self, name):
        return None


class DemoRos:
    """The read side of RosBridge, and the manager calls control.py makes, on the fake robot."""

    estop_service = "/gim6010_estop/trigger"
    clear_service = "/gim6010_estop/clear_errors"

    def __init__(self, robot: Robot, log: Logbook, controllers_yaml: Path) -> None:
        self.robot, self.log = robot, log
        config = yaml.safe_load(controllers_yaml.read_text())
        policies_yaml = controllers_yaml.with_name("rl_policies.yaml")
        if policies_yaml.is_file():
            policies_config = yaml.safe_load(policies_yaml.read_text())
            config["controller_manager"]["ros__parameters"].update(
                policies_config.pop("controller_manager")["ros__parameters"])
            config.update(policies_config)
        self.types = {k: v["type"] for k, v in config["controller_manager"]["ros__parameters"].items()
                      if isinstance(v, dict) and "type" in v}
        self.types.setdefault("health_broadcaster",
                              "joint_state_broadcaster/JointStateBroadcaster")
        self.params = {f"/{name}": _flatten(section["ros__parameters"])
                       for name, section in config.items() if name != "controller_manager"}
        self.params["/joy_teleop"] = {"speed_fraction": 1.0, "deadman_button": 9,
                                      "handover_height_m": 0.17}
        self.baseline = {node: dict(values) for node, values in self.params.items()}

    # Read side ----------------------------------------------------------

    def fast(self) -> dict:
        if not self.robot.running:
            return {"joints": {}, "joints_age_s": None, "imu": None, "imu_age_s": None,
                    "joy": None, "joy_age_s": None, "rates": {}, "cmd_vel": None}
        joints = {n: {"position": p, "velocity": 0.05, "effort": 0.3 * random.random(),
                      "sensor_age_s": 0.002 + 0.003 * random.random()}
                  for n, p in self.robot.joints().items()}
        rates = {"/joint_states": {"hz": 300.0, "age_s": 0.003, "count": 1},
                 "/imu_broadcaster/imu": {"hz": 300.0, "age_s": 0.003, "count": 1},
                 "/health_broadcaster/dynamic_joint_states": {"hz": 10.0, "age_s": 0.05, "count": 1},
                 "/joy": {"hz": 20.0, "age_s": 0.04, "count": 1},
                 "/cmd_vel": {"hz": 0.0, "age_s": None, "count": 0}}
        return {"joints": joints, "joints_age_s": 0.003, "imu": self.robot.imu(),
                "imu_age_s": 0.003, "imu_sensor_age_s": 0.002 + 0.004 * random.random(),
                "joy": None, "joy_age_s": None, "rates": rates, "cmd_vel": None}

    def _diagnostics(self, name: str) -> dict:
        kind = self.types[name].split("/")[-1]
        if self.robot.fallen(name):
            values = {"RlPolicyController": {"phase": 2.0, "status_mask": 2.0},
                      "WholeBodyController": {"status_mask": 4.0}}.get(
                kind, {"saturation_mask": 512.0})
            return {"level": 2, "message": "fallen" if "Rl" in kind else "fall_latch",
                    "age_s": 0.05, "values": dict(values, pitch_rad=1.31)}
        return {"level": 0, "message": "balancing", "age_s": 0.05, "values": {
            "engaged": 1.0, "pitch_rad": 0.012, "pitch_reference_rad": 0.01,
            "forward_velocity_m_s": 0.002, "velocity_reference_m_s": 0.0,
            "leg_height_m": 0.171, "height_reference_m": 0.17, "roll_rad": 0.004,
            "yaw_rate_rad_s": 0.0, "wheel_torque_nm": 0.18, "status_mask": 0.0}}

    def slow(self) -> dict:
        robot = self.robot
        if not robot.running:
            return {"manager": False, "controllers": [], "hardware": [], "dynamic": {},
                    "diagnostics": {}, "nodes": ["/gim6010_estop", "/robot_state_publisher"],
                    "teleop": {}, "estop_available": False}
        with robot.lock:
            states = dict(robot.controllers)
            hardware_states = dict(robot.hardware)
        controllers = []
        for name in dict.fromkeys([*BROADCASTERS, *self.types]):
            state = states.get(name)
            if state is None or name not in self.types:
                continue
            owns = state == "active" and "broadcaster" not in name
            controllers.append({"name": name, "type": self.types[name], "state": state,
                                "claimed": ["right_wheel/effort"] if owns else [],
                                "owns_commands": owns})
        dynamic = {n: {"online": 1.0, "temperature": 33.0 + i, "error": 0.0,
                       "feedback_age": 0.004, "saturation_count": 0.0,
                       "requested_effort": 0.1, "applied_effort": 0.1}
                   for i, n in enumerate(STAND)} if SYSTEM in hardware_states else {}
        dynamic["imu"] = {"age": 0.004, "online": 1.0}
        active = [c["name"] for c in controllers if c["owns_commands"]]
        diagnostics = {name: self._diagnostics(name) for name in active}
        components = {SYSTEM: ("system", "sandro_hardware/GIM6010System", 30),
                      IMU: ("sensor", "sandro_hardware/BNO085Sensor", 0)}
        hardware = [{"name": name, "type": components[name][0], "plugin": components[name][1],
                     "state": state, "state_id": {"active": 3, "inactive": 2}.get(state, 1),
                     "command_interfaces": components[name][2],
                     "claimed": components[name][2] if active and name == SYSTEM else 0}
                    for name, state in hardware_states.items()]
        return {"manager": True, "controllers": controllers, "hardware": hardware,
                "dynamic": dynamic, "dynamic_topic": "/health_broadcaster/dynamic_joint_states",
                "dynamic_age_s": 0.05, "diagnostics": diagnostics,
                "nodes": ["/controller_manager", "/gim6010_estop", "/joy_estop", "/joy_node",
                          "/joy_teleop", "/robot_state_publisher", *[f"/{c['name']}" for c in controllers]],
                "teleop": {"active": active[0] if active else None, "selected": None},
                "estop_available": True}

    # Manager calls ------------------------------------------------------

    def _manager(self) -> None:
        if not self.robot.running:
            raise RuntimeError("/controller_manager/list_controllers is not available")

    def controllers(self, timeout: float = 5.0) -> dict:
        self._manager()
        with self.robot.lock:
            return {name: {"name": name, "type": self.types.get(name, ""), "state": state,
                           "claimed": ["right_wheel/effort"] if state == "active"
                           and "broadcaster" not in name else []}
                    for name, state in self.robot.controllers.items()}

    def hardware_states(self, timeout: float = 5.0) -> dict:
        self._manager()
        with self.robot.lock:
            return dict(self.robot.hardware)

    def load(self, name: str) -> None:
        self._manager()
        if name not in self.types:
            raise RuntimeError(f"could not load {name}")
        time.sleep(0.4)
        with self.robot.lock:
            self.robot.controllers.setdefault(name, "unconfigured")

    def configure(self, name: str) -> None:
        self._manager()
        time.sleep(0.4)
        with self.robot.lock:
            self.robot.controllers[name] = "inactive"

    def switch(self, activate: list[str], deactivate: list[str], strict: bool = True,
               timeout: float = 5.0) -> None:
        self._manager()
        time.sleep(0.2)
        robot = self.robot
        with robot.lock:
            drives = robot.hardware.get(SYSTEM)
            refused = [n for n in activate
                       if robot.controllers.get(n) not in ("inactive", "active")
                       or ("broadcaster" not in n and drives != "active")
                       or (n in DRIVE_BROADCASTERS and drives not in ("active", "inactive"))]
            if refused and strict:
                raise RuntimeError("the controller manager refused the switch (its log says why)")
            for name in deactivate:
                if name in robot.controllers:
                    robot.controllers[name] = "inactive"
                    robot.started.pop(name, None)
            for name in activate:
                if name not in refused:
                    robot.controllers[name] = "active"
                    robot.started[name] = time.monotonic()

    def set_hardware(self, name: str, active: bool) -> str:
        self._manager()
        time.sleep(0.6)
        with self.robot.lock:
            if name not in self.robot.hardware:
                raise RuntimeError(f"{name} stays unknown")
            self.robot.hardware[name] = "active" if active else "inactive"
            if active:
                self.robot.latched = False
            else:
                for other in self.robot.owning():
                    self.robot.controllers[other] = "inactive"
            return self.robot.hardware[name]

    def trigger(self, service: str, timeout: float = 3.0, wait: float = 1.0) -> tuple[bool, str]:
        if service == self.estop_service:
            self._manager()
            self.robot.latch()
            return True, "CAN ESTOP sent to configured GIM6010 nodes"
        if service == self.clear_service:
            self._manager()
            self.robot.latched = False
            return True, "CAN Clear_Errors sent to configured GIM6010 nodes"
        controller = service.strip("/").split("/")[0]
        if service.endswith("/arm_handover") and "RlPolicy" in self.types.get(controller, "") \
                and self.robot.controllers.get(controller) in ("inactive", "active"):
            return True, "the next activation within 1 s takes over without the stand phase"
        raise RuntimeError(f"{service} is not available")

    # Parameters ---------------------------------------------------------

    def parameters(self, node: str) -> list[dict]:
        values = self.params.get(node)
        if values is None:
            raise RuntimeError(f"{node}/list_parameters is not available")
        rows = []
        for name in sorted(values):
            value = values[name]
            kind = ("bool" if isinstance(value, bool) else "int" if isinstance(value, int)
                    else "double" if isinstance(value, float) else "string" if isinstance(value, str)
                    else "double[]" if value and isinstance(value[0], float)
                    else "string[]")
            rows.append({"name": name, "type": kind, "value": value,
                         "baseline": self.baseline[node][name],
                         "read_only": name in ("joints", "imu_name", "command_topic"),
                         "description": "", "range": None})
        return rows

    def set_parameter(self, node: str, name: str, value):
        current = self.params[node][name]
        if isinstance(current, float):
            value = float(value)
        self.params[node][name] = value
        self.log.note(f"{node} {name} = {value}")
        return True, ""

    def changed_yaml(self) -> str:
        out = {}
        for node, values in self.params.items():
            changed = {k: v for k, v in values.items() if v != self.baseline[node][k]}
            if changed:
                out[node.strip("/")] = {"ros__parameters": changed}
        return yaml.safe_dump(out) if out else "# no parameter changed since the launch started\n"

    def new_run(self) -> None:
        pass


class DemoEstopBus:
    def __init__(self, robot: Robot) -> None:
        self.robot = robot

    def send(self) -> str:
        self.robot.latch()
        return "CAN E-stop sent by the panel to 6 drives on can0"


class DemoCan:
    def __init__(self, robot: Robot) -> None:
        self.robot = robot

    def snapshot(self) -> dict:
        joints = self.robot.joints()
        nodes = []
        for node, joint in sorted(NODES.items()):
            error = 0x104000 if self.robot.latched else 0
            nodes.append({
                "node": node, "joint": joint, "seen": True, "age_s": 0.002,
                "heartbeat_age_s": 0.04, "encoder_age_s": 0.003,
                "axis_state": "closed loop" if self.robot.armed else "idle", "axis_error": error,
                "fault_flags": 0,
                "error_text": "0x104000 estop_requested can_watchdog" if error else "",
                "temperature": 31 + node,
                "rates": {"heartbeat": 10.0, "encoder": 200.0},
                "position": joints[joint], "velocity": 0.0, "position_rev": joints[joint] * 8 / 6.283})
        total = sum(sum(n["rates"].values()) for n in nodes)
        return {"interface": "can0", "open": True, "error": None, "frames_hz": total, "nodes": nodes}


class DemoPad:
    def __init__(self, robot: Robot) -> None:
        self.robot = robot

    def snapshot(self) -> dict:
        t = self.robot.t()
        held = int(t) % 6 < 3
        buttons = {name: False for name in ("cross", "circle", "square", "triangle", "create", "ps",
                                            "options", "l3", "r3", "l1", "r1", "up", "down", "left",
                                            "right", "l2", "r2")}
        buttons["l1"] = held
        state = {"buttons": buttons, "lx": 0.0, "ly": 0.6 * math.sin(t) if held else 0.0,
                 "rx": 0.3 * math.sin(0.5 * t), "ry": 0.0, "l2": 0.0, "r2": 0.0}
        return {"device": "/dev/input/js0", "name": "DualSense Wireless Controller",
                "error": None, "state": state, "age_s": 0.01}


class DemoBluetooth:
    available = True

    def __init__(self) -> None:
        self.scanning_until = 0.0
        self.output = []

    def snapshot(self) -> dict:
        scanning = time.monotonic() < self.scanning_until
        devices = [{"mac": "14:3A:9A:3B:34:B7", "name": "DualSense Wireless Controller",
                    "gamepad": True, "paired": True, "trusted": True, "connected": True,
                    "battery": "0x4c (76)", "rssi": None}]
        if scanning:
            devices.append({"mac": "7C:2A:31:0E:91:44", "name": "Wireless Controller",
                            "gamepad": True, "paired": False, "trusted": False, "connected": False,
                            "battery": None, "rssi": "-61"})
        return {"available": True, "error": None, "controller": "B8:27:EB:12:34:56",
                "powered": True, "power_state": "on", "discovering": scanning,
                "rfkill": [{"path": "/sys/class/rfkill/rfkill0", "soft": False, "hard": False}],
                "scanning": scanning, "scan_left_s": max(0.0, self.scanning_until - time.monotonic()),
                "devices": devices, "busy": None, "output": self.output[-30:]}

    def watch(self, seconds=15.0):
        return {"ok": True, "message": "watching"}

    def scan(self, seconds=60.0):
        self.scanning_until = time.monotonic() + seconds
        self.output.append(f"scanning for {seconds:.0f} s: hold PS + Create until the light bar flashes")
        return {"ok": True, "message": "scanning"}

    def stop_scan(self):
        self.scanning_until = 0.0
        return {"ok": True, "message": "scan stopped"}

    def unblock(self):
        return {"ok": True, "message": "Changing power on succeeded"}

    def _say(self, verb, mac):
        self.output += [f"$ bluetoothctl {verb} {mac}", f"{verb.capitalize()} successful"]
        return {"ok": True, "message": f"{verb} successful"}

    def pair(self, mac):
        return self._say("pair", mac)

    def connect(self, mac):
        return self._say("connect", mac)

    def disconnect(self, mac):
        return self._say("disconnect", mac)

    def remove(self, mac):
        return self._say("remove", mac)


class DemoSystem:
    def snapshot(self) -> dict:
        return {"host": "sandro-pi", "time": time.time(), "cpu_temp_c": 54.2 + random.random(),
                "cpu_percent": 38.0 + 5 * random.random(), "cpu_mhz": 1800.0,
                "governor": "performance", "load": [1.21, 1.05, 0.98], "cpus": 4,
                "mem_total": 4 * 1024 ** 3, "mem_available": 2.9 * 1024 ** 3,
                "uptime_s": 5321.0, "throttled": {"raw": "0x0", "now": [], "since_boot": []},
                "devices": {"/dev/spidev6.0": True, "/dev/gpiochip0": True},
                "joysticks": ["/dev/input/js0", "/dev/input/js1"],
                "can": {"present": True, "up": True, "operstate": "UP", "state": "ERROR-ACTIVE",
                        "bitrate": 1000000, "sample_point": "0.875", "berr_tx": 0, "berr_rx": 0,
                        "rx_packets": 1234567, "tx_packets": 456789, "rx_errors": 0,
                        "tx_errors": 0, "bus_off": 0, "restarts": 0,
                        "rx_hz": 4260.0 + 20 * random.random(), "tx_hz": 1800.0}}


def sources(log: Logbook, settings: LaunchSettings, controllers_yaml: Path) -> dict:
    robot = Robot()
    log.note("demo mode: synthetic robot, nothing is commanded")
    teleop = yaml.safe_load(controllers_yaml.with_name("teleop.yaml").read_text())
    policies = teleop["joy_teleop"]["ros__parameters"].get("policies", [])
    partners = {p: [o for o in policies if o != p] for p in policies}
    return {"log": log, "launcher": DemoLauncher(robot, log, settings, partners),
            "system": DemoSystem(), "can": DemoCan(robot), "estop_bus": DemoEstopBus(robot),
            "pad": DemoPad(robot), "bt": DemoBluetooth(),
            "ros": DemoRos(robot, log, controllers_yaml)}
