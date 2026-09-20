"""Gamepad emergency stop for Sandro.

Pressing the configured button chord (PS5 L2 + R2 by default) deactivates
every active ros2_control controller that owns command interfaces, which zeros
all six efforts in simulation and on hardware, and additionally triggers the
CAN E-stop service when it exists (physical robot only). The chord is
edge-triggered and latched until released; nothing on the gamepad re-arms the
robot. Re-activate deliberately, with the robot held upright. In simulation:

    ros2 control switch_controllers --activate whole_body_controller

On hardware the drives latch ESTOP_REQUESTED after the CAN E-stop. Arming the
hardware component again clears that bit (and only that bit and the CAN
watchdog code): relaunch with `arm_hardware:=true`, or
`ros2 control set_hardware_component_state SandroSystem active` followed by the
controller activation above.

This is software over DDS and CAN. It is not a substitute for the physical
power cut-off.
"""

from __future__ import annotations

import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger


def chord_pressed(buttons, chord) -> bool:
    """True when every button index in `chord` is present and pressed."""
    if not chord:
        return False
    try:
        return all(buttons[index] > 0 for index in chord)
    except IndexError:
        return False


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
        self.declare_parameter("buttons", [6, 7])
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("controller_manager", "/controller_manager")
        self.declare_parameter("hardware_estop_service", "/gim6010_estop/trigger")
        self.declare_parameter("service_timeout_s", 2.0)
        self.chord = [int(v) for v in self.get_parameter("buttons").value]
        manager = self.get_parameter("controller_manager").value
        self.timeout = float(self.get_parameter("service_timeout_s").value)
        group = ReentrantCallbackGroup()
        self.list_client = self.create_client(
            ListControllers, f"{manager}/list_controllers", callback_group=group)
        self.switch_client = self.create_client(
            SwitchController, f"{manager}/switch_controller", callback_group=group)
        self.hardware_client = self.create_client(
            Trigger, self.get_parameter("hardware_estop_service").value,
            callback_group=group)
        self.subscription = self.create_subscription(
            Joy, self.get_parameter("joy_topic").value, self._on_joy, 10,
            callback_group=group)
        self.latched = False
        self.get_logger().info(
            f"gamepad E-stop armed on buttons {self.chord} (press together)")

    def _on_joy(self, msg: Joy) -> None:
        pressed = chord_pressed(msg.buttons, self.chord)
        if pressed and not self.latched:
            self.latched = True
            self.trigger()
        elif not pressed and self.latched:
            self.latched = False
            self.get_logger().warn(
                "E-stop chord released; nothing re-arms from the gamepad. Simulation: "
                "`ros2 control switch_controllers --activate <name>`. Hardware: the "
                "drives latch ESTOP_REQUESTED; relaunch with arm_hardware:=true (activation "
                "clears that bit) or `ros2 control set_hardware_component_state "
                "SandroSystem active`, then activate the controller.")

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
