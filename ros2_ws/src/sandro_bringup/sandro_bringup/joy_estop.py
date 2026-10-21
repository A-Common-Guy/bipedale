"""Gamepad emergency stop and two-step re-arm for Sandro.

Squeezing the configured trigger chord (PS5 L2 + R2 by default) deactivates
every active ros2_control controller that owns command interfaces, which zeros
all six efforts in simulation and on hardware, and additionally triggers the
CAN E-stop service when it exists (physical robot only). The chord is
edge-triggered and latched until released.

Re-arming is deliberate and takes two separate holds of the re-arm buttons
(PS5 Create + Options for 2 s by default), so the motors never jump straight
from a stop into a balancer:

1. With the drives disarmed (after the CAN E-stop the drives latch
   ESTOP_REQUESTED and the hardware component leaves `active`), the first hold
   activates the hardware component. Activation clears that bit and the CAN
   watchdog code only, any other fault still refuses, and the drives hold zero
   torque with no controller.
2. With the drives armed and no balancer running, the next hold activates the
   configured controller (loading it first if needed). Hold the robot upright:
   the legs stand at once and the wheels engage when it is upright and still.

The same can be done by hand:

    ros2 control set_hardware_component_state SandroSystem active
    ros2 control switch_controllers --activate whole_body_controller

This is software over DDS and CAN. It is not a substitute for the physical
power cut-off.
"""

from __future__ import annotations

import time

import rclpy
from controller_manager_msgs.srv import (
    ConfigureController, ListControllers, ListHardwareComponents, LoadController,
    SetHardwareComponentState, SwitchController)
from lifecycle_msgs.msg import State
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import Trigger


def chord_pressed(axes, chord, pressed_below) -> bool:
    """True when every axis index in `chord` is present and at or below `pressed_below`.

    SDL reports the PS5 triggers as axes, +1 released and -1 fully pressed; a
    joystick that has not sent an event yet reads 0, which never triggers.
    """
    if not chord:
        return False
    try:
        return all(axes[index] <= pressed_below for index in chord)
    except IndexError:
        return False


def buttons_held(buttons, chord) -> bool:
    """True when every button index in `chord` is present and pressed."""
    if not chord:
        return False
    try:
        return all(buttons[index] > 0 for index in chord)
    except IndexError:
        return False


def rearm_step(hardware_active, running, controller_loaded) -> str:
    """What one re-arm hold does: arm_hardware, load_and_activate, activate or nothing.

    `hardware_active` is None when the hardware component does not exist (it
    then cannot be the reason for refusing), `running` lists the active
    command-owning controllers.
    """
    if hardware_active is False:
        return "arm_hardware"
    if running:
        return "nothing"
    return "activate" if controller_loaded else "load_and_activate"


def controllers_to_stop(states) -> list[str]:
    """Names of active controllers that own command interfaces.

    Broadcasters claim no command interface and keep publishing state, which
    is exactly what you want to keep after a stop.
    """
    return [
        state.name
        for state in states
        if state.state == "active" and len(state.claimed_interfaces) > 0
    ]


class JoyEstop(Node):
    def __init__(self) -> None:
        super().__init__("joy_estop")
        self.declare_parameter("axes", [4, 5])
        self.declare_parameter("axis_pressed_below", -0.5)
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("controller_manager", "/controller_manager")
        self.declare_parameter("hardware_estop_service", "/gim6010_estop/trigger")
        self.declare_parameter("service_timeout_s", 2.0)
        self.declare_parameter("rearm_buttons", [4, 6])
        self.declare_parameter("rearm_hold_s", 2.0)
        self.declare_parameter("rearm_controller", "whole_body_controller")
        self.declare_parameter("hardware_component", "SandroSystem")
        self.declare_parameter("rearm_broadcasters", ["joint_state_broadcaster"])
        self.chord = [int(v) for v in self.get_parameter("axes").value]
        self.pressed_below = float(self.get_parameter("axis_pressed_below").value)
        self.rearm_chord = [int(v) for v in self.get_parameter("rearm_buttons").value]
        self.rearm_hold_s = float(self.get_parameter("rearm_hold_s").value)
        self.rearm_controller = self.get_parameter("rearm_controller").value
        self.hardware_component = self.get_parameter("hardware_component").value
        self.rearm_broadcasters = list(self.get_parameter("rearm_broadcasters").value)
        manager = self.get_parameter("controller_manager").value
        self.timeout = float(self.get_parameter("service_timeout_s").value)
        group = ReentrantCallbackGroup()
        self.list_client = self.create_client(
            ListControllers, f"{manager}/list_controllers", callback_group=group)
        self.switch_client = self.create_client(
            SwitchController, f"{manager}/switch_controller", callback_group=group)
        self.list_hardware_client = self.create_client(
            ListHardwareComponents, f"{manager}/list_hardware_components",
            callback_group=group)
        self.set_hardware_client = self.create_client(
            SetHardwareComponentState, f"{manager}/set_hardware_component_state",
            callback_group=group)
        self.load_client = self.create_client(
            LoadController, f"{manager}/load_controller", callback_group=group)
        self.configure_client = self.create_client(
            ConfigureController, f"{manager}/configure_controller", callback_group=group)
        self.hardware_client = self.create_client(
            Trigger, self.get_parameter("hardware_estop_service").value,
            callback_group=group)
        self.subscription = self.create_subscription(
            Joy, self.get_parameter("joy_topic").value, self._on_joy, 10,
            callback_group=group)
        # The gamepad policy switch (joy_teleop) changes which policy runs;
        # re-arm restarts that one rather than the launch's.
        self.create_subscription(
            String, "/teleop/selected_policy", self._on_selected,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=group)
        self.latched = False
        self.rearm_since = None
        self.rearm_fired = False
        self.rearm_busy = False
        self.hardware_active = None
        self.get_logger().info(
            f"gamepad E-stop armed on axes {self.chord} (press together); re-arm by "
            f"holding buttons {self.rearm_chord} for {self.rearm_hold_s:.1f} s, twice")

    def _on_selected(self, msg: String) -> None:
        if msg.data and msg.data != self.rearm_controller:
            self.rearm_controller = msg.data
            self.get_logger().info(f"re-arm now starts {self.rearm_controller}")

    def _on_joy(self, msg: Joy) -> None:
        pressed = chord_pressed(msg.axes, self.chord, self.pressed_below)
        if pressed and not self.latched:
            self.latched = True
            self.trigger()
        elif not pressed and self.latched:
            self.latched = False
            self.get_logger().warn(
                f"E-stop chord released. To re-arm, hold buttons {self.rearm_chord} for "
                f"{self.rearm_hold_s:.1f} s (drives at zero torque), then again "
                f"({self.rearm_controller}).")
        if pressed or not buttons_held(msg.buttons, self.rearm_chord):
            self.rearm_since = None
            self.rearm_fired = False
            return
        now = time.monotonic()
        if self.rearm_since is None:
            self.rearm_since = now
        elif not self.rearm_fired and now - self.rearm_since >= self.rearm_hold_s:
            self.rearm_fired = True
            self.rearm()

    def rearm(self) -> None:
        if self.rearm_busy:
            self.get_logger().warn("re-arm already in progress")
            return
        if not (self.list_client.service_is_ready() and
                self.list_hardware_client.service_is_ready()):
            self.get_logger().error("controller manager not reachable; cannot re-arm")
            return
        self.rearm_busy = True
        self.list_hardware_client.call_async(
            ListHardwareComponents.Request()).add_done_callback(self._on_hardware_listed)

    def _rearm_failed(self, what) -> None:
        self.rearm_busy = False
        self.get_logger().error(f"re-arm: {what}")

    def _on_hardware_listed(self, future) -> None:
        try:
            components = future.result().component
        except Exception as error:  # noqa: BLE001
            self._rearm_failed(f"list_hardware_components failed: {error}")
            return
        states = [c.state.id for c in components if c.name == self.hardware_component]
        self.hardware_active = states[0] == State.PRIMARY_STATE_ACTIVE if states else None
        self.list_client.call_async(
            ListControllers.Request()).add_done_callback(self._on_rearm_listed)

    def _on_rearm_listed(self, future) -> None:
        try:
            controllers = future.result().controller
        except Exception as error:  # noqa: BLE001
            self._rearm_failed(f"list_controllers failed: {error}")
            return
        running = controllers_to_stop(controllers)
        loaded = any(c.name == self.rearm_controller for c in controllers)
        step = rearm_step(self.hardware_active, running, loaded)
        if step == "arm_hardware":
            self.get_logger().warn(
                f"RE-ARM 1/2: activating {self.hardware_component} (drives at zero torque)")
            request = SetHardwareComponentState.Request()
            request.name = self.hardware_component
            request.target_state = State(id=State.PRIMARY_STATE_ACTIVE, label="active")
            self.set_hardware_client.call_async(request).add_done_callback(
                self._on_hardware_armed)
        elif step == "nothing":
            self.rearm_busy = False
            self.get_logger().warn(f"re-arm: {running} already running; nothing to do")
        elif step == "activate":
            self._activate()
        else:
            self.get_logger().info(f"re-arm: loading {self.rearm_controller}")
            request = LoadController.Request()
            request.name = self.rearm_controller
            self.load_client.call_async(request).add_done_callback(self._on_loaded)

    def _on_hardware_armed(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001
            self._rearm_failed(f"set_hardware_component_state failed: {error}")
            return
        self.rearm_busy = False
        if response.ok:
            self.get_logger().warn(
                f"RE-ARM 1/2 done: drives armed at zero torque. Hold the robot upright and "
                f"hold the re-arm buttons again to start {self.rearm_controller}.")
            self.list_client.call_async(
                ListControllers.Request()).add_done_callback(self._restart_broadcasters)
        else:
            self.get_logger().error(
                f"RE-ARM refused: {self.hardware_component} is '{response.state.label}'; "
                "the controller manager log names the drive fault")

    def _restart_broadcasters(self, future) -> None:
        """The hardware read error also deactivated the broadcasters reading it."""
        try:
            controllers = future.result().controller
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"list_controllers failed: {error}")
            return
        names = [c.name for c in controllers
                 if c.name in self.rearm_broadcasters and c.state == "inactive"]
        if not names:
            return
        request = SwitchController.Request()
        request.activate_controllers = names
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.timeout = rclpy.duration.Duration(seconds=self.timeout).to_msg()
        self.switch_client.call_async(request).add_done_callback(
            lambda done, names=names: self.get_logger().info(
                f"re-arm: restarted {names}: "
                f"{'ok' if not done.exception() and done.result().ok else 'FAILED'}"))

    def _on_loaded(self, future) -> None:
        try:
            ok = future.result().ok
        except Exception as error:  # noqa: BLE001
            ok = False
            self.get_logger().error(f"load_controller failed: {error}")
        if not ok:
            self._rearm_failed(f"could not load {self.rearm_controller}")
            return
        request = ConfigureController.Request()
        request.name = self.rearm_controller
        self.configure_client.call_async(request).add_done_callback(self._on_configured)

    def _on_configured(self, future) -> None:
        try:
            ok = future.result().ok
        except Exception as error:  # noqa: BLE001
            ok = False
            self.get_logger().error(f"configure_controller failed: {error}")
        if not ok:
            self._rearm_failed(f"could not configure {self.rearm_controller}")
            return
        self._activate()

    def _activate(self) -> None:
        self.get_logger().warn(f"RE-ARM 2/2: activating {self.rearm_controller}")
        request = SwitchController.Request()
        request.activate_controllers = [self.rearm_controller]
        request.strictness = SwitchController.Request.STRICT
        request.timeout = rclpy.duration.Duration(seconds=self.timeout).to_msg()
        self.switch_client.call_async(request).add_done_callback(self._on_activated)

    def _on_activated(self, future) -> None:
        try:
            ok = future.result().ok
        except Exception as error:  # noqa: BLE001
            ok = False
            self.get_logger().error(f"switch_controller failed: {error}")
        self.rearm_busy = False
        if ok:
            self.get_logger().warn(f"RE-ARM 2/2 done: {self.rearm_controller} active")
        else:
            self.get_logger().error(f"RE-ARM: {self.rearm_controller} did not activate")

    def trigger(self) -> None:
        self.get_logger().error("GAMEPAD E-STOP: deactivating controllers")
        if self.list_client.service_is_ready():
            future = self.list_client.call_async(ListControllers.Request())
            future.add_done_callback(self._on_listed)
        else:
            self.get_logger().error("controller manager not reachable; nothing to deactivate")
        self._trigger_hardware()

    def _on_listed(self, future) -> None:
        try:
            names = controllers_to_stop(future.result().controller)
        except Exception as error:  # noqa: BLE001 - report any failure of the stop path
            self.get_logger().error(f"list_controllers failed: {error}")
            return
        if not names:
            self.get_logger().warn("no active command-owning controller to deactivate")
            return
        request = SwitchController.Request()
        request.deactivate_controllers = names
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.timeout = rclpy.duration.Duration(seconds=self.timeout).to_msg()
        switch = self.switch_client.call_async(request)
        switch.add_done_callback(
            lambda done, names=names: self._on_switched(done, names))

    def _on_switched(self, future, names) -> None:
        try:
            ok = future.result().ok
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"switch_controller failed: {error}")
            return
        level = self.get_logger().error if ok else self.get_logger().fatal
        level(f"deactivated {names}: {'ok' if ok else 'FAILED'}")

    def _trigger_hardware(self) -> None:
        if not self.hardware_client.service_is_ready():
            self.get_logger().info("no CAN E-stop service (simulation); controllers only")
            return
        future = self.hardware_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_hardware)

    def _on_hardware(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"CAN E-stop call failed: {error}")
            return
        (self.get_logger().error if response.success else self.get_logger().fatal)(
            f"CAN E-stop: {response.message}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JoyEstop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
