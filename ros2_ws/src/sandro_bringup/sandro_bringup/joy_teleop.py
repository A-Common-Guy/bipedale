"""Gamepad driving, soft policy switching and policy indication for Sandro.

Drive (hold L1, the deadman):
    left stick   up/down forward speed, left/right yaw rate
    right stick  up/down body height rate, left/right body roll (policies
                 with a roll command; the others ignore it)
    R3           height back to the active policy's initial height

Policies:
    L1 + triangle  switch to the next policy in `policies`, softly: the robot
                   is brought to a standstill at the handover height with zero
                   roll and still joints, the incoming controller is armed for
                   a handover (~/arm_handover) and one controller switch swaps
                   the policies in a single control cycle; the incoming policy
                   engages at once from the measured pose. Release L1 or
                   squeeze the E-stop chord to abort.
    circle         tell which policy runs

The lightbar shows the running policy in its `policy_colors` entry (red when no
controller runs, white for a controller outside `policies`) and the player
LEDs light up to its position in `policies`; during a switch the lightbar
blinks between the outgoing and incoming colours. Both are written through
the kernel's LED class devices of the pad (`led_root`), so they work whichever
way SDL opened it. The gamepad also rumbles once per position (one pulse for
the first, two for the second, ...) whenever the running controller changes
and on circle, where the pad offers rumble; one long rumble means a switch
was aborted. The running
controller is also published on /teleop/active_controller and the selected
policy (which the E-stop re-arm restarts) on /teleop/selected_policy.

Stick ranges follow the running controller's own command limits, read from
its parameters, so full deflection means the same on every policy.
"""

from __future__ import annotations

import glob
import os
import time
from collections import deque
from dataclasses import dataclass

import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
from rcl_interfaces.srv import GetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState, Joy, JoyFeedback
from std_msgs.msg import String
from std_srvs.srv import Trigger


@dataclass
class Limits:
    forward_m_s: float
    yaw_rad_s: float
    height_rate_m_s: float
    roll_rad: float
    initial_height_m: float | None = None
    engage_tilt_rad: float = 0.15
    engage_joint_velocity_rad_s: float = 2.0


def axis(axes, index: int) -> float:
    try:
        return float(axes[index])
    except IndexError:
        return 0.0


def pressed(buttons, index: int) -> bool:
    try:
        return index >= 0 and buttons[index] > 0
    except IndexError:
        return False


def stick_command(axes, mapping: dict, limits: Limits, fraction: float):
    """(forward m/s, yaw rad/s, height rate m/s, roll rad) from the sticks."""
    return (
        mapping["forward_sign"] * axis(axes, mapping["forward_axis"]) * limits.forward_m_s * fraction,
        mapping["yaw_sign"] * axis(axes, mapping["yaw_axis"]) * limits.yaw_rad_s * fraction,
        mapping["height_sign"] * axis(axes, mapping["height_axis"]) * limits.height_rate_m_s,
        mapping["roll_sign"] * axis(axes, mapping["roll_axis"]) * limits.roll_rad,
    )


def height_rate_towards(reference_m: float, target_m: float, limit_m_s: float,
                        gain_per_s: float) -> float:
    return max(-limit_m_s, min(limit_m_s, gain_per_s * (target_m - reference_m)))


def next_policy(policies: list[str], current: str | None) -> str | None:
    if current not in policies or len(policies) < 2:
        return None
    return policies[(policies.index(current) + 1) % len(policies)]


def handover_ready(diag: dict, velocity, target: Limits, handover_height_m: float,
                   margin: float) -> tuple[bool, str]:
    """Whether the running policy holds the robot still enough to hand over, and why not."""
    if diag.get("message") != "policy":
        return False, f"running controller is '{diag.get('message', 'silent')}'"
    if abs(diag.get("velocity_reference_m_s", 1.0)) > 0.03:
        return False, "speed reference not zero"
    if abs(diag.get("height_reference_m", 0.0) - handover_height_m) > 0.004:
        return False, "height not at the handover height"
    if abs(diag.get("roll_reference_rad", 0.0)) > 0.02:
        return False, "roll not zero"
    if diag.get("tilt_rad", 1.0) > 0.5 * target.engage_tilt_rad:
        return False, "body tilted"
    if diag.get("rate_rad_s", 1.0) > 0.5:
        return False, "body turning"
    if velocity is None:
        return False, "no joint state"
    if any(abs(v) > margin * target.engage_joint_velocity_rad_s for v in velocity):
        return False, "joints moving"
    return True, ""


NO_CONTROLLER_RGB = (255, 0, 0)
OTHER_CONTROLLER_RGB = (255, 255, 255)


def led_state(policies: list[str], colors: list[int], active: str | None,
              switching_to: str | None, blink_on: bool) -> tuple[tuple[int, int, int], int]:
    """(lightbar rgb, player LEDs lit) showing the running policy or the switch underway."""
    def color(name):
        if name is None:
            return NO_CONTROLLER_RGB
        if name not in policies:
            return OTHER_CONTROLLER_RGB
        k = 3 * policies.index(name)
        return tuple(colors[k:k + 3]) if k + 3 <= len(colors) else OTHER_CONTROLLER_RGB

    shown = switching_to if switching_to is not None and blink_on else active
    count = policies.index(shown) + 1 if shown in policies else 0
    return color(shown), count


class PadLeds:
    """The gamepad's lightbar and player LEDs through the kernel LED class devices."""

    def __init__(self, root: str) -> None:
        self.root = root
        self.prefix: str | None = None
        self.shown = None
        self.next_search = 0.0

    def show(self, rgb, count: int) -> None:
        if (rgb, count) == self.shown:
            return
        now = time.monotonic()
        if self.prefix is None:
            if now < self.next_search:
                return
            self.next_search = now + 2.0
            found = sorted(glob.glob(os.path.join(self.root, "input*:rgb:indicator")))
            if not found:
                return
            self.prefix = found[-1][: -len(":rgb:indicator")]
        try:
            self._write("rgb:indicator/multi_intensity", " ".join(str(int(c)) for c in rgb))
            self._write("rgb:indicator/brightness", "255")
            for k in range(1, 6):
                self._write(f"white:player-{k}/brightness", "1" if k <= count else "0")
            self.shown = (rgb, count)
        except OSError:
            self.prefix = None

    def _write(self, leaf: str, value: str) -> None:
        with open(f"{self.prefix}:{leaf}", "w") as f:
            f.write(value)


class JoyTeleop(Node):
    def __init__(self) -> None:
        super().__init__("joy_teleop")
        p = self.declare_parameter
        p("joy_topic", "/joy")
        p("command_topic", "/cmd_vel")
        p("frame", "base_link")
        p("controller_manager", "/controller_manager")
        p("deadman_button", 9)
        p("forward_axis", 1)
        p("yaw_axis", 0)
        p("height_axis", 3)
        p("roll_axis", 2)
        p("forward_sign", 1.0)
        p("yaw_sign", 1.0)
        p("height_sign", 1.0)
        p("roll_sign", -1.0)
        p("speed_fraction", 1.0)
        p("switch_button", 3)
        p("announce_button", 1)
        p("recentre_button", 8)
        p("estop_axes", [4, 5])
        p("estop_pressed_below", -0.5)
        p("policies", ["rl_flat_level", "rl_agile_terrain"])
        p("handover_height_m", 0.17)
        p("handover_margin", 0.6)
        p("handover_hold_s", 0.3)
        p("handover_timeout_s", 8.0)
        p("height_gain_per_s", 3.0)
        p("joy_timeout_s", 0.5)
        p("default_limits", [0.5, 1.5, 0.05, 0.0])
        p("rate_hz", 50.0)
        p("policy_colors", [0, 255, 0, 255, 100, 0])
        p("led_root", "/sys/class/leds")
        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self.mapping = {k: g(k) for k in (
            "forward_axis", "yaw_axis", "height_axis", "roll_axis",
            "forward_sign", "yaw_sign", "height_sign", "roll_sign")}
        self.deadman = int(g("deadman_button"))
        self.switch_button = int(g("switch_button"))
        self.announce_button = int(g("announce_button"))
        self.recentre_button = int(g("recentre_button"))
        self.estop_axes = [int(v) for v in g("estop_axes")]
        self.estop_below = float(g("estop_pressed_below"))
        self.fraction = float(g("speed_fraction"))
        self.policies = list(g("policies"))
        self.handover_height = float(g("handover_height_m"))
        self.margin = float(g("handover_margin"))
        self.hold_s = float(g("handover_hold_s"))
        self.timeout_s = float(g("handover_timeout_s"))
        self.gain = float(g("height_gain_per_s"))
        self.joy_timeout_s = float(g("joy_timeout_s"))
        self.frame = g("frame")
        d = [float(v) for v in g("default_limits")]
        self.default_limits = Limits(*d)
        self.policy_colors = [int(v) for v in g("policy_colors")]
        self.leds = PadLeds(g("led_root"))

        group = ReentrantCallbackGroup()
        manager = g("controller_manager")
        self.list_client = self.create_client(
            ListControllers, f"{manager}/list_controllers", callback_group=group)
        self.switch_client = self.create_client(
            SwitchController, f"{manager}/switch_controller", callback_group=group)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.command_pub = self.create_publisher(TwistStamped, g("command_topic"), 10)
        self.active_pub = self.create_publisher(String, "/teleop/active_controller", latched)
        self.selected_pub = self.create_publisher(String, "/teleop/selected_policy", latched)
        self.feedback_pub = self.create_publisher(JoyFeedback, "/joy/set_feedback", 10)
        self.create_subscription(Joy, g("joy_topic"), self._on_joy, 10, callback_group=group)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10,
                                 callback_group=group)
        self.diag: dict[str, dict] = {}
        for name in self.policies:
            self.create_subscription(
                DiagnosticArray, f"/{name}/diagnostics",
                lambda msg, name=name: self._on_diag(name, msg), 10, callback_group=group)

        self.joy: Joy | None = None
        self.joy_time = 0.0
        self.previous_buttons: list[int] = []
        self.joint_names: list[str] = []
        self.joint_velocity: dict[str, float] = {}
        self.controller_joints: dict[str, list[str]] = {}
        self.limits: dict[str, Limits] = {}
        self.active: str | None = None
        self.selected: str | None = None
        self.loaded: dict[str, str] = {}
        self.parameter_clients: dict[str, object] = {}
        self.handover_clients: dict[str, object] = {}
        self.listing = False
        self.publishing = False
        self.recentre = False
        self.switch: dict | None = None
        self.rumble: deque = deque()

        self.create_timer(1.0 / float(g("rate_hz")), self._tick, callback_group=group)
        self.create_timer(1.0, self._poll_controllers, callback_group=group)
        self.get_logger().info(
            f"gamepad teleop: hold button {self.deadman} to drive; switch policy with "
            f"{self.deadman}+{self.switch_button} among {self.policies}; "
            f"button {self.announce_button} tells the running one")

    # Inputs -------------------------------------------------------------

    def _on_joy(self, msg: Joy) -> None:
        now = time.monotonic()
        edges = [i for i, b in enumerate(msg.buttons)
                 if b and (i >= len(self.previous_buttons) or not self.previous_buttons[i])]
        self.previous_buttons = list(msg.buttons)
        self.joy, self.joy_time = msg, now
        held = pressed(msg.buttons, self.deadman)
        if self.announce_button in edges:
            self.announce()
        if held and self.switch_button in edges:
            self.request_switch()
        if held and self.recentre_button in edges:
            self.recentre = True

    def _on_joints(self, msg: JointState) -> None:
        for i, name in enumerate(msg.name):
            if i < len(msg.velocity):
                self.joint_velocity[name] = msg.velocity[i]

    def _on_diag(self, name: str, msg: DiagnosticArray) -> None:
        for status in msg.status:
            values = {"message": status.message, "time": time.monotonic()}
            for item in status.values:
                try:
                    values[item.key] = float(item.value)
                except ValueError:
                    pass
            self.diag[name] = values

    def _joints_of(self, controller: str, table: dict[str, float]):
        names = self.controller_joints.get(controller)
        if not names or any(n not in table for n in names):
            return None
        return [table[n] for n in names]

    # Controllers --------------------------------------------------------

    def _poll_controllers(self) -> None:
        if self.listing or self.switch is not None or not self.list_client.service_is_ready():
            return
        self.listing = True
        self.list_client.call_async(ListControllers.Request()).add_done_callback(self._on_listed)

    def _on_listed(self, future) -> None:
        self.listing = False
        try:
            controllers = future.result().controller
        except Exception as error:  # noqa: BLE001
            self.get_logger().warn(f"list_controllers failed: {error}")
            return
        running = [c.name for c in controllers
                   if c.state == "active" and len(c.claimed_interfaces) > 0]
        self.loaded = {c.name: c.state for c in controllers}
        self.set_active(running[0] if running else None)

    def set_active(self, name: str | None) -> None:
        if name == self.active:
            return
        self.active = name
        self.recentre = False
        self.active_pub.publish(String(data=name or ""))
        if name is not None:
            self.fetch_limits(name)
        if name in self.policies:
            self.selected = name
            self.selected_pub.publish(String(data=name))
        self.get_logger().warn(f"running controller: {name or 'none'}")
        self.announce()

    def fetch_limits(self, name: str) -> None:
        if name in self.limits:
            return
        if name not in self.parameter_clients:
            self.parameter_clients[name] = self.create_client(
                GetParameters, f"/{name}/get_parameters")
        client = self.parameter_clients[name]
        if not client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(f"{name} parameters unreachable; default stick ranges")
            return
        rl = ["commands.forward_m_s", "commands.yaw_rad_s", "shaping.height_rate_m_s",
              "commands.roll_rad", "commands.initial_height_m", "safety.engage_tilt_rad",
              "safety.engage_joint_velocity_rad_s", "joints"]
        wbc = ["drive.max_forward_velocity_m_s", "drive.max_yaw_rate_rad_s",
               "drive.max_height_rate_m_s", "joints"]
        request = GetParameters.Request(names=rl)
        client.call_async(request).add_done_callback(
            lambda f, name=name, client=client: self._on_rl_parameters(name, client, f, wbc))

    def _on_rl_parameters(self, name, client, future, wbc) -> None:
        try:
            values = future.result().values
        except Exception as error:  # noqa: BLE001
            self.get_logger().warn(f"{name} parameters: {error}")
            return
        if values and values[0].type != 0:
            v = values
            self.limits[name] = Limits(
                v[0].double_value, v[1].double_value, v[2].double_value, v[3].double_value,
                v[4].double_value, v[5].double_value or 0.15, v[6].double_value or 2.0)
            self.controller_joints[name] = list(v[7].string_array_value)
            self.get_logger().info(
                f"{name}: full stick {self.limits[name].forward_m_s:.2f} m/s, "
                f"{self.limits[name].yaw_rad_s:.2f} rad/s, "
                f"{self.limits[name].height_rate_m_s:.2f} m/s height, "
                f"{self.limits[name].roll_rad:.2f} rad roll")
            return
        client.call_async(GetParameters.Request(names=wbc)).add_done_callback(
            lambda f, name=name: self._on_wbc_parameters(name, f))

    def _on_wbc_parameters(self, name, future) -> None:
        try:
            v = future.result().values
        except Exception as error:  # noqa: BLE001
            self.get_logger().warn(f"{name} parameters: {error}")
            return
        if v and v[0].type != 0:
            self.limits[name] = Limits(v[0].double_value, v[1].double_value,
                                       v[2].double_value, 0.0)
            self.controller_joints[name] = list(v[3].string_array_value)

    def limits_of(self, name: str | None) -> Limits:
        return self.limits.get(name, self.default_limits) if name else self.default_limits

    # Switching ----------------------------------------------------------

    def request_switch(self) -> None:
        if self.switch is not None:
            return
        target = next_policy(self.policies, self.active)
        if target is None:
            self.get_logger().warn(
                f"policy switch: running controller {self.active} is not one of {self.policies}")
            self.buzz()
            return
        if self.loaded.get(target) != "inactive":
            self.get_logger().error(
                f"policy switch: {target} is not loaded and configured "
                f"(state {self.loaded.get(target, 'absent')})")
            self.buzz()
            return
        self.fetch_limits(target)
        self.switch = {"from": self.active, "to": target, "start": time.monotonic(),
                       "ready_since": None, "called": False, "why": ""}
        self.get_logger().warn(
            f"policy switch {self.active} -> {target}: settling at "
            f"{self.handover_height:.3f} m, zero speed and roll")

    def abort_switch(self, why: str) -> None:
        self.get_logger().error(f"policy switch aborted: {why}")
        self.switch = None
        self.buzz()

    def _switch_step(self, now: float):
        s = self.switch
        if s["called"]:
            return (0.0, 0.0, 0.0, 0.0)
        if now - s["start"] > self.timeout_s:
            self.abort_switch(f"not ready after {self.timeout_s:.0f} s ({s['why']})")
            return (0.0, 0.0, 0.0, 0.0)
        diag = self.diag.get(s["from"], {})
        source = self.limits_of(s["from"])
        reference = diag.get("height_reference_m", self.handover_height)
        command = (0.0, 0.0, height_rate_towards(
            reference, self.handover_height, source.height_rate_m_s, self.gain), 0.0)
        target = self.limits.get(s["to"])
        if target is None:
            s["why"] = "incoming policy limits unknown"
            return command
        velocity = self._joints_of(s["to"], self.joint_velocity)
        fresh = now - diag.get("time", 0.0) < 0.5
        ready, s["why"] = handover_ready(diag, velocity, target, self.handover_height,
                                         self.margin)
        ready = ready and fresh
        if not ready:
            s["ready_since"] = None
            return command
        if s["ready_since"] is None:
            s["ready_since"] = now
        if now - s["ready_since"] >= self.hold_s:
            s["called"] = True
            if s["to"] not in self.handover_clients:
                self.handover_clients[s["to"]] = self.create_client(
                    Trigger, f"/{s['to']}/arm_handover")
            client = self.handover_clients[s["to"]]
            if not client.service_is_ready():
                self.abort_switch(f"{s['to']} offers no arm_handover service")
                return command
            client.call_async(Trigger.Request()).add_done_callback(self._on_armed)
        return command

    def _on_armed(self, future) -> None:
        s = self.switch
        try:
            ok = future.result().success
        except Exception as error:  # noqa: BLE001
            ok = False
            self.get_logger().error(f"arm_handover failed: {error}")
        if s is None:
            return
        if not ok:
            self.abort_switch(f"{s['to']} refused the handover")
            return
        request = SwitchController.Request()
        request.activate_controllers = [s["to"]]
        request.deactivate_controllers = [s["from"]]
        request.strictness = SwitchController.Request.STRICT
        request.timeout = rclpy.duration.Duration(seconds=0.5).to_msg()
        self.get_logger().warn(f"policy switch: handing over to {s['to']}")
        self.switch_client.call_async(request).add_done_callback(self._on_switched)

    def _on_switched(self, future) -> None:
        s = self.switch
        try:
            ok = future.result().ok
        except Exception as error:  # noqa: BLE001
            ok = False
            self.get_logger().error(f"switch_controller failed: {error}")
        self.switch = None
        if not ok:
            self.abort_switch(f"controller manager refused {s['from']} -> {s['to']}")
            return
        self.set_active(s["to"])

    # Output -------------------------------------------------------------

    def _tick(self) -> None:
        now = time.monotonic()
        self._play_rumble(now)
        switching_to = self.switch["to"] if self.switch is not None else None
        self.leds.show(*led_state(self.policies, self.policy_colors, self.active,
                                  switching_to, int(now * 4) % 2 == 1))
        joy_fresh = self.joy is not None and now - self.joy_time < self.joy_timeout_s
        held = joy_fresh and pressed(self.joy.buttons, self.deadman)
        estop = joy_fresh and self.estop_axes and all(
            axis(self.joy.axes, i) <= self.estop_below for i in self.estop_axes)
        if self.switch is not None and not self.switch["called"] and (not held or estop):
            self.abort_switch("E-stop chord" if estop else "deadman released")
        if self.switch is not None:
            self.publish(*self._switch_step(now))
            return
        if not held:
            if self.publishing:
                self.publish(0.0, 0.0, 0.0, 0.0)
                self.publishing = False
            self.recentre = False
            return
        limits = self.limits_of(self.active)
        forward, yaw, height_rate, roll = stick_command(
            self.joy.axes, self.mapping, limits, self.fraction)
        if self.recentre:
            if abs(height_rate) > 0.1 * limits.height_rate_m_s or limits.initial_height_m is None:
                self.recentre = False
            else:
                reference = self.diag.get(self.active, {}).get("height_reference_m")
                if reference is None or abs(reference - limits.initial_height_m) < 0.002:
                    self.recentre = False
                else:
                    height_rate = height_rate_towards(
                        reference, limits.initial_height_m, limits.height_rate_m_s, self.gain)
        self.publish(forward, yaw, height_rate, roll)
        self.publishing = True

    def publish(self, forward: float, yaw: float, height_rate: float, roll: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.twist.linear.x = forward
        msg.twist.angular.z = yaw
        msg.twist.linear.z = height_rate
        msg.twist.angular.x = roll
        self.command_pub.publish(msg)

    # Feedback -----------------------------------------------------------

    def announce(self) -> None:
        name = self.active if self.active in self.policies else None
        if name is None:
            self.get_logger().warn(
                f"running: {self.active or 'nothing'} (selected policy {self.selected})")
            return
        count = self.policies.index(name) + 1
        self.get_logger().warn(f"running policy {count}: {name}")
        self.pulses(count)

    def pulses(self, count: int) -> None:
        start = time.monotonic()
        for k in range(count):
            self.rumble.append((start + 0.4 * k, 0.8))
            self.rumble.append((start + 0.4 * k + 0.2, 0.0))

    def buzz(self) -> None:
        start = time.monotonic()
        self.rumble.append((start, 1.0))
        self.rumble.append((start + 0.8, 0.0))

    def _play_rumble(self, now: float) -> None:
        while self.rumble and self.rumble[0][0] <= now:
            _, intensity = self.rumble.popleft()
            self.feedback_pub.publish(
                JoyFeedback(type=JoyFeedback.TYPE_RUMBLE, id=0, intensity=intensity))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JoyTeleop()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
