"""Everything the panel reads from and asks of the ROS graph.

Read side: topic rates (raw subscriptions, so counting costs no
deserialisation), joint states, IMU, gamepad, per-drive health from
/dynamic_joint_states, the controllers' ~/diagnostics, /rosout. Write side:
only the controller manager's own services (load, configure, switch,
hardware lifecycle), parameter services, and the E-stop services; the same
calls `ros2 control` and joy_estop make. The panel never publishes commands.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
import yaml
from control_msgs.msg import DynamicJointState
from controller_manager_msgs.srv import (
    ConfigureController, ListControllers, ListHardwareComponents, LoadController,
    SetHardwareComponentState, SwitchController)
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
from lifecycle_msgs.msg import State
from rcl_interfaces.msg import Log, Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import DescribeParameters, GetParameters, ListParameters, SetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Imu, JointState, Joy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from . import gamepad
from .errors import Conflict
from .logbook import ERROR, INFO, WARN, Logbook

SENSOR_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
LATCHED_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
ROSOUT_QOS = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
ROS_LEVELS = {10: 10, 20: 20, 30: 30, 40: 40, 50: 50}
HIDDEN_NODES = ("_ros2cli", "sandro_gui", "launch_ros", "transform_listener_impl")


def parameter_to_python(value: ParameterValue):
    t = value.type
    return {
        ParameterType.PARAMETER_BOOL: lambda: value.bool_value,
        ParameterType.PARAMETER_INTEGER: lambda: value.integer_value,
        ParameterType.PARAMETER_DOUBLE: lambda: value.double_value,
        ParameterType.PARAMETER_STRING: lambda: value.string_value,
        ParameterType.PARAMETER_BYTE_ARRAY: lambda: list(value.byte_array_value),
        ParameterType.PARAMETER_BOOL_ARRAY: lambda: list(value.bool_array_value),
        ParameterType.PARAMETER_INTEGER_ARRAY: lambda: list(value.integer_array_value),
        ParameterType.PARAMETER_DOUBLE_ARRAY: lambda: list(value.double_array_value),
        ParameterType.PARAMETER_STRING_ARRAY: lambda: list(value.string_array_value),
    }.get(t, lambda: None)()


TYPE_NAMES = {0: "unset", 1: "bool", 2: "int", 3: "double", 4: "string", 5: "bytes",
              6: "bool[]", 7: "int[]", 8: "double[]", 9: "string[]"}


def python_to_parameter(kind: int, value) -> ParameterValue:
    out = ParameterValue(type=kind)
    if kind == ParameterType.PARAMETER_BOOL:
        if not isinstance(value, bool):
            raise ValueError("expected true or false")
        out.bool_value = value
    elif kind == ParameterType.PARAMETER_INTEGER:
        if isinstance(value, bool) or not float(value).is_integer():
            raise ValueError("expected an integer")
        out.integer_value = int(value)
    elif kind == ParameterType.PARAMETER_DOUBLE:
        number = float(value)
        if not math.isfinite(number):
            raise ValueError("expected a finite number")
        out.double_value = number
    elif kind == ParameterType.PARAMETER_STRING:
        out.string_value = str(value)
    elif kind == ParameterType.PARAMETER_BOOL_ARRAY:
        out.bool_array_value = [bool(v) for v in value]
    elif kind == ParameterType.PARAMETER_INTEGER_ARRAY:
        out.integer_array_value = [int(v) for v in value]
    elif kind == ParameterType.PARAMETER_DOUBLE_ARRAY:
        out.double_array_value = [float(v) for v in value]
    elif kind == ParameterType.PARAMETER_STRING_ARRAY:
        out.string_array_value = [str(v) for v in value]
    else:
        raise ValueError(f"cannot set a {TYPE_NAMES.get(kind, kind)} parameter")
    return out


def nest(flat: dict) -> dict:
    """{'a.b': 1} -> {'a': {'b': 1}}, the layout of a ros__parameters file."""
    tree: dict = {}
    for key in sorted(flat):
        node = tree
        parts = key.split(".")
        for part in parts[:-1]:
            node = node.setdefault(part, {})
        node[parts[-1]] = flat[key]
    return tree


class _Topic:
    """Message count, rate and the latest serialized message of one topic."""

    __slots__ = ("count", "last", "rate", "raw", "_count")

    def __init__(self) -> None:
        self.count = self._count = 0
        self.last = self.raw = None
        self.rate = 0.0

    def update(self, dt: float) -> None:
        self.rate = (self.count - self._count) / dt
        self._count = self.count


class RosBridge:
    def __init__(self, cfg: dict, logbook: Logbook, use_rosout=lambda: True,
                 teleop_policies: list[str] | None = None) -> None:
        self.cfg = cfg
        self.log = logbook
        self.use_rosout = use_rosout
        self.teleop_policies = teleop_policies or []
        robot = cfg.get("robot", {})
        self.hardware_component = robot.get("hardware_component", "SandroSystem")
        self.broadcasters = robot.get("broadcasters", ["joint_state_broadcaster"])
        if not rclpy.ok():
            rclpy.init()
        self.node = Node("sandro_gui")
        self.group = ReentrantCallbackGroup()
        manager = "/controller_manager"
        c = self.node.create_client
        self.cm = {
            "list": c(ListControllers, f"{manager}/list_controllers", callback_group=self.group),
            "hardware": c(ListHardwareComponents, f"{manager}/list_hardware_components",
                          callback_group=self.group),
            "switch": c(SwitchController, f"{manager}/switch_controller", callback_group=self.group),
            "load": c(LoadController, f"{manager}/load_controller", callback_group=self.group),
            "configure": c(ConfigureController, f"{manager}/configure_controller",
                           callback_group=self.group),
            "set_hardware": c(SetHardwareComponentState,
                              f"{manager}/set_hardware_component_state", callback_group=self.group),
        }
        self.estop = c(Trigger, robot.get("estop_service", "/gim6010_estop/trigger"),
                       callback_group=self.group)
        self.clear = c(Trigger, robot.get("clear_errors_service", "/gim6010_estop/clear_errors"),
                       callback_group=self.group)
        self._param_clients: dict[tuple[str, str], object] = {}
        self.baseline: dict[str, dict] = {}
        self.param_types: dict[str, dict] = {}

        self.topics = {topic["name"]: topic for topic in cfg.get("topics", [])}
        self.streams: dict[str, _Topic] = {name: _Topic() for name in self.topics}
        self._dynamic_cache: tuple[float, dict] = (0.0, {})
        self.controllers: list[dict] = []
        self.controllers_time = None
        self.hardware: list[dict] = []
        self.active_controller = self.selected_policy = None
        self._diag_raw: dict[str, tuple[bytes, float]] = {}
        self._diag_subs = {}
        self.nodes: list[str] = []
        self._listing = self._listing_hw = 0.0

        self._subscribe()
        self.node.create_timer(0.5, self._update_rates, callback_group=self.group)
        self.node.create_timer(1.0, self._poll_manager, callback_group=self.group)
        self.node.create_timer(2.0, self._poll_graph, callback_group=self.group)
        self._last_rates = time.monotonic()
        # Single-threaded on purpose: every callback is short and service calls
        # are made from the HTTP threads; the multi-threaded executor spun a
        # whole Pi core while callbacks were in flight.
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self._spin, name="ros", daemon=True)
        self._thread.start()

    def _spin(self) -> None:
        try:
            self.executor.spin()
        except Exception as error:  # noqa: BLE001 - shutdown races
            if rclpy.ok():
                self.log.note(f"ROS executor stopped: {error}", ERROR)

    def shutdown(self) -> None:
        # The spin thread must be out of rcl before the node and context go:
        # tearing them down under it aborts the process (and dumps core).
        self.executor.shutdown(timeout_sec=1.0)
        self._thread.join(timeout=2.0)
        self.node.destroy_node()
        rclpy.try_shutdown()

    # Subscriptions ------------------------------------------------------

    def _subscribe(self) -> None:
        # Raw: a callback only counts and keeps the bytes; the snapshots
        # deserialize the latest message, 20 times a second instead of 900.
        sub = self.node.create_subscription
        for name, topic in self.topics.items():
            qos = LATCHED_QOS if topic.get("latched") else SENSOR_QOS
            sub(get_message(topic["type"]), name,
                lambda raw, stream=self.streams[name]: self._on_raw(stream, raw), qos,
                callback_group=self.group, raw=True)
        sub(String, "/teleop/active_controller",
            lambda m: setattr(self, "active_controller", m.data or None), LATCHED_QOS,
            callback_group=self.group)
        sub(String, "/teleop/selected_policy",
            lambda m: setattr(self, "selected_policy", m.data or None), LATCHED_QOS,
            callback_group=self.group)
        sub(Log, "/rosout", self._on_rosout, ROSOUT_QOS, callback_group=self.group)

    @staticmethod
    def _on_raw(stream: _Topic, raw: bytes) -> None:
        stream.count += 1
        stream.last = time.monotonic()
        stream.raw = raw

    def _latest(self, name: str, kind, max_age: float = 1.0):
        """(message, age) of a topic's latest message, or (None, age) when absent or old."""
        stream = self.streams.get(name)
        if stream is None or stream.raw is None:
            return None, None
        age = time.monotonic() - stream.last
        if age > max_age:
            return None, age
        try:
            return deserialize_message(stream.raw, kind), age
        except Exception:  # noqa: BLE001 - a malformed message is no data
            return None, age

    def dynamic(self) -> dict:
        """/dynamic_joint_states as {joint or sensor: {interface: value}}, parsed at most at 5 Hz."""
        stamp, parsed = self._dynamic_cache
        if time.monotonic() - stamp < 0.2:
            return parsed
        msg, _ = self._latest("/dynamic_joint_states", DynamicJointState, 2.0)
        parsed = {}
        if msg is not None:
            for i, name in enumerate(msg.joint_names):
                values = msg.interface_values[i]
                parsed[name] = dict(zip(values.interface_names, values.values))
        self._dynamic_cache = (time.monotonic(), parsed)
        return parsed

    def _on_rosout(self, msg: Log) -> None:
        if msg.name == self.node.get_name() or not self.use_rosout():
            return
        stamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        self.log.add(ROS_LEVELS.get(msg.level, INFO), msg.name, msg.msg, origin="rosout",
                     stamp=stamp or None)

    def _on_diag(self, name: str, raw: bytes) -> None:
        self._diag_raw[name] = (raw, time.monotonic())

    def _update_rates(self) -> None:
        now = time.monotonic()
        dt = now - self._last_rates
        self._last_rates = now
        if dt <= 0:
            return
        for stream in self.streams.values():
            stream.update(dt)

    # Polling ------------------------------------------------------------

    def _poll_manager(self) -> None:
        if not self.cm["list"].service_is_ready():
            self.controllers, self.hardware = [], []
            return
        now = time.monotonic()
        # A call the manager never answers (it went away) must not stop the polling.
        if now - self._listing > 3.0:
            self._listing = now
            self.cm["list"].call_async(ListControllers.Request()).add_done_callback(self._on_listed)
        if now - self._listing_hw > 3.0 and self.cm["hardware"].service_is_ready():
            self._listing_hw = now
            self.cm["hardware"].call_async(ListHardwareComponents.Request()).add_done_callback(
                self._on_hardware)

    def _on_listed(self, future) -> None:
        self._listing = 0.0
        try:
            listed = future.result().controller
        except Exception:  # noqa: BLE001 - manager going away
            return
        self.controllers = [{
            "name": c.name, "type": c.type, "state": c.state,
            "claimed": list(c.claimed_interfaces),
            "owns_commands": c.state == "active" and len(c.claimed_interfaces) > 0,
        } for c in listed]
        self.controllers_time = time.monotonic()
        for c in listed:
            if c.name not in self._diag_subs:
                self._diag_subs[c.name] = self.node.create_subscription(
                    DiagnosticArray, f"/{c.name}/diagnostics",
                    lambda raw, name=c.name: self._on_diag(name, raw), SENSOR_QOS,
                    callback_group=self.group, raw=True)

    def _on_hardware(self, future) -> None:
        self._listing_hw = 0.0
        try:
            components = future.result().component
        except Exception:  # noqa: BLE001
            return
        self.hardware = [{
            "name": h.name, "type": h.type, "plugin": h.plugin_name,
            "state": h.state.label, "state_id": h.state.id,
            "command_interfaces": len(h.command_interfaces),
            "claimed": sum(1 for i in h.command_interfaces if i.is_claimed),
        } for h in components]

    def _poll_graph(self) -> None:
        try:
            names = self.node.get_node_names_and_namespaces()
        except Exception:  # noqa: BLE001
            return
        self.nodes = sorted({(ns.rstrip("/") + "/" + n) for n, ns in names
                             if not any(h in n for h in HIDDEN_NODES)})

    # Snapshots ----------------------------------------------------------

    def fast(self) -> dict:
        now = time.monotonic()
        rates = {name: {"hz": round(s.rate, 1), "age_s": None if s.last is None else now - s.last,
                        "count": s.count} for name, s in self.streams.items()}
        # How old the newest sample is at the driver (sandro_hardware's feedback_age and
        # imu/age); absent in simulation, where the message age is all there is.
        dynamic = self.dynamic()
        joints = {}
        msg, joints_age = self._latest("/joint_states", JointState)
        if msg is not None:
            for i, name in enumerate(msg.name):
                joints[name] = {
                    "position": msg.position[i] if i < len(msg.position) else math.nan,
                    "velocity": msg.velocity[i] if i < len(msg.velocity) else math.nan,
                    "effort": msg.effort[i] if i < len(msg.effort) else math.nan,
                    "sensor_age_s": dynamic.get(name, {}).get("feedback_age")}
        imu = None
        msg, imu_age = self._latest("/imu_broadcaster/imu", Imu)
        if msg is not None:
            o, g, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
            imu = {"q": (o.x, o.y, o.z, o.w), "gyro": (g.x, g.y, g.z), "accel": (a.x, a.y, a.z)}
        msg, joy_age = self._latest("/joy", Joy)
        joy = gamepad.from_joy(msg.axes, msg.buttons) if msg is not None else None
        msg, _ = self._latest("/cmd_vel", TwistStamped)
        command = None
        if msg is not None:
            command = {"forward": msg.twist.linear.x, "yaw": msg.twist.angular.z,
                       "height_rate": msg.twist.linear.z, "roll": msg.twist.angular.x}
        return {"joints": joints, "joints_age_s": joints_age if joints else None,
                "imu": imu, "imu_age_s": imu_age if imu else None,
                "imu_sensor_age_s": dynamic.get("imu", {}).get("age") if imu else None,
                "joy": joy, "joy_age_s": joy_age if joy else None, "rates": rates,
                "cmd_vel": command}

    def slow(self) -> dict:
        now = time.monotonic()
        dynamic = self.dynamic()
        dynamic_stream = self.streams.get("/dynamic_joint_states")
        diagnostics = {}
        for name, (raw, stamp) in list(self._diag_raw.items()):
            if now - stamp > 2.0:
                continue
            try:
                msg = deserialize_message(raw, DiagnosticArray)
            except Exception:  # noqa: BLE001
                continue
            for status in msg.status[:1]:
                values = {}
                for item in status.values:
                    try:
                        values[item.key] = float(item.value)
                    except ValueError:
                        values[item.key] = item.value
                diagnostics[name] = {"level": int.from_bytes(status.level, "little")
                                     if isinstance(status.level, bytes) else int(status.level),
                                     "message": status.message, "values": values,
                                     "age_s": now - stamp}
        manager = self.cm["list"].service_is_ready()
        return {"manager": manager, "controllers": self.controllers if manager else [],
                "hardware": self.hardware if manager else [], "dynamic": dynamic,
                "dynamic_age_s": None if dynamic_stream is None or dynamic_stream.last is None
                else now - dynamic_stream.last,
                "diagnostics": diagnostics, "nodes": self.nodes,
                "teleop": {"active": self.active_controller, "selected": self.selected_policy},
                "estop_available": self.estop.service_is_ready()}

    # Service calls (from HTTP threads) ----------------------------------

    def _call(self, client, request, timeout: float = 5.0):
        if not client.wait_for_service(timeout_sec=min(2.0, timeout)):
            raise RuntimeError(f"{client.srv_name} is not available")
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            client.remove_pending_request(future)
            raise TimeoutError(f"{client.srv_name} did not answer within {timeout:.0f} s")
        return future.result()

    def _list(self) -> dict:
        return {c.name: c for c in self._call(self.cm["list"], ListControllers.Request()).controller}

    def _switch(self, activate: list[str], deactivate: list[str], strict: bool = True) -> None:
        request = SwitchController.Request()
        request.activate_controllers = activate
        request.deactivate_controllers = deactivate
        request.strictness = (SwitchController.Request.STRICT if strict
                              else SwitchController.Request.BEST_EFFORT)
        request.timeout = Duration(seconds=2.0).to_msg()
        if not self._call(self.cm["switch"], request).ok:
            raise RuntimeError("the controller manager refused the switch (its log says why)")

    def _load(self, name: str) -> None:
        if not self._call(self.cm["load"], LoadController.Request(name=name), 15.0).ok:
            raise RuntimeError(f"could not load {name}")

    def _configure(self, name: str) -> None:
        if not self._call(self.cm["configure"], ConfigureController.Request(name=name), 15.0).ok:
            raise RuntimeError(f"could not configure {name}")

    def _ready(self, name: str, listed: dict) -> None:
        if name not in listed:
            self._load(name)
            self._configure(name)
        elif listed[name].state == "unconfigured":
            self._configure(name)

    def activate(self, name: str, swap: bool = False) -> str:
        listed = self._list()
        running = [c.name for c in listed.values()
                   if c.state == "active" and c.claimed_interfaces and c.name != name]
        if running and not swap:
            raise Conflict(running)
        self._ready(name, listed)
        if name in self.teleop_policies:
            for other in self.teleop_policies:
                if other != name and other not in listed:
                    try:
                        self._load(other)
                        self._configure(other)
                    except Exception as error:  # noqa: BLE001 - the switch is optional
                        self.log.note(f"could not preload {other} for the gamepad switch: {error}",
                                      WARN)
        self._switch([name], running if swap else [])
        message = f"activated {name}" + (f" (deactivated {', '.join(running)})" if running else "")
        self.log.note(message, WARN)
        return message

    def deactivate(self, name: str) -> str:
        self._switch([], [name])
        self.log.note(f"deactivated {name}", WARN)
        return f"deactivated {name}"

    def configure(self, name: str) -> str:
        listed = self._list()
        if name not in listed:
            self._load(name)
        self._configure(name)
        self.log.note(f"configured {name}")
        return f"configured {name}"

    def set_hardware(self, name: str, active: bool) -> str:
        request = SetHardwareComponentState.Request()
        request.name = name
        request.target_state = (State(id=State.PRIMARY_STATE_ACTIVE, label="active") if active
                                else State(id=State.PRIMARY_STATE_INACTIVE, label="inactive"))
        response = self._call(self.cm["set_hardware"], request, 10.0)
        if not response.ok:
            raise RuntimeError(f"{name} stays {response.state.label}: the launch log names the "
                               "drive fault")
        self.log.note(f"{name} is {response.state.label}", WARN)
        if active:
            listed = self._list()
            stopped = [b for b in self.broadcasters if b in listed and listed[b].state == "inactive"]
            if stopped:
                self._switch(stopped, [], strict=False)
        return f"{name} {response.state.label}"

    def emergency_stop(self) -> str:
        notes = []
        try:
            listed = self._list()
            running = [c.name for c in listed.values() if c.state == "active" and c.claimed_interfaces]
            if running:
                self._switch([], running, strict=False)
                notes.append(f"deactivated {', '.join(running)}")
            else:
                notes.append("no controller was running")
        except Exception as error:  # noqa: BLE001 - still fire the CAN E-stop
            notes.append(f"controllers: {error}")
        if self.estop.service_is_ready():
            response = self._call(self.estop, Trigger.Request(), 3.0)
            notes.append(f"CAN E-stop: {response.message}")
        else:
            notes.append("no CAN E-stop service (simulation, or the stack is not running)")
        message = "; ".join(notes)
        self.log.note("E-STOP from the panel: " + message, ERROR)
        return message

    def clear_errors(self) -> str:
        response = self._call(self.clear, Trigger.Request(), 3.0)
        self.log.note(response.message, WARN)
        return response.message

    # Parameters ---------------------------------------------------------

    def _param_client(self, node: str, kind, service: str):
        key = (node, service)
        if key not in self._param_clients:
            self._param_clients[key] = self.node.create_client(
                kind, f"{node}/{service}", callback_group=self.group)
        return self._param_clients[key]

    def parameters(self, node: str) -> list[dict]:
        names = list(self._call(self._param_client(node, ListParameters, "list_parameters"),
                                ListParameters.Request(depth=0)).result.names)
        names = sorted(n for n in names if n not in ("use_sim_time", "qos_overrides")
                       and not n.startswith("qos_overrides."))
        values = self._call(self._param_client(node, GetParameters, "get_parameters"),
                            GetParameters.Request(names=names)).values
        described = self._call(self._param_client(node, DescribeParameters, "describe_parameters"),
                               DescribeParameters.Request(names=names)).descriptors
        rows, types = [], {}
        baseline = self.baseline.setdefault(node, {})
        for name, value, descriptor in zip(names, values, described):
            python = parameter_to_python(value)
            types[name] = value.type
            baseline.setdefault(name, python)
            ranges = list(descriptor.floating_point_range) or list(descriptor.integer_range)
            rows.append({
                "name": name, "type": TYPE_NAMES.get(value.type, str(value.type)),
                "value": python, "baseline": baseline[name], "read_only": descriptor.read_only,
                "description": descriptor.description,
                "range": [ranges[0].from_value, ranges[0].to_value] if ranges else None,
            })
        self.param_types[node] = types
        return rows

    def set_parameter(self, node: str, name: str, value) -> tuple[bool, str]:
        kind = self.param_types.get(node, {}).get(name)
        if kind is None:
            self.parameters(node)
            kind = self.param_types.get(node, {}).get(name)
        if kind is None:
            return False, f"{node} has no parameter {name}"
        try:
            parameter = Parameter(name=name, value=python_to_parameter(kind, value))
        except (TypeError, ValueError) as error:
            return False, str(error)
        result = self._call(self._param_client(node, SetParameters, "set_parameters"),
                            SetParameters.Request(parameters=[parameter])).results[0]
        if result.successful:
            self.log.note(f"{node} {name} = {value}")
        else:
            self.log.note(f"{node} refused {name} = {value}: {result.reason}", WARN)
        return result.successful, result.reason

    def changed_yaml(self) -> str:
        """The parameters changed in this run, as a file for extra_params or the overrides."""
        out = {}
        for node, baseline in self.baseline.items():
            changed = {}
            try:
                rows = self.parameters(node)
            except Exception:  # noqa: BLE001 - node gone
                continue
            for row in rows:
                if row["value"] != baseline.get(row["name"]):
                    changed[row["name"]] = row["value"]
            if changed:
                out[node.strip("/").split("/")[-1]] = {"ros__parameters": nest(changed)}
        if not out:
            return "# no parameter changed since the launch started\n"
        return ("# Parameters changed from the panel; merge into "
                "sandro_bringup/config/sandro_real_overrides.yaml to keep them.\n"
                + yaml.safe_dump(out, sort_keys=False, default_flow_style=None))

    def reset_baselines(self) -> None:
        self.baseline.clear()
        self.param_types.clear()
