"""Put the simulated robot back on its stand keyframe with the balancer inactive.

Physics runs from the moment the controller manager starts, about a second
before the balancer is active, and the unpowered legs collapse in that time.
Resetting the world while the balancer is already running is not enough: it
may have engaged on the collapsed robot and anchored its position hold and
posture trim there, so the teleport to the keyframe starts the run with a
large position error and a wrong trim. This script therefore deactivates the
balancer, resets the world and activates the balancer again, all from one
node so the robot spends only tens of milliseconds unpowered on the keyframe.

    ros2 run sandro_bringup sim_reset --ros-args -p controller:=whole_body_controller
"""

from __future__ import annotations

import sys

import rclpy
from controller_manager_msgs.srv import SwitchController
from mujoco_ros2_control_msgs.srv import ResetWorld
from rclpy.node import Node


class SimReset(Node):
    def __init__(self) -> None:
        super().__init__("sim_reset")
        self.declare_parameter("controller", "whole_body_controller")
        self.declare_parameter("controller_manager", "/controller_manager")
        self.declare_parameter("reset_service", "/mujoco_ros2_control_node/reset_world")
        self.declare_parameter("service_timeout_s", 30.0)
        self.controller = self.get_parameter("controller").value
        manager = self.get_parameter("controller_manager").value
        self.timeout = float(self.get_parameter("service_timeout_s").value)
        self.switch = self.create_client(SwitchController, f"{manager}/switch_controller")
        self.reset = self.create_client(ResetWorld, self.get_parameter("reset_service").value)

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=self.timeout):
            raise RuntimeError(f"{client.srv_name} not available")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout)
        if future.result() is None:
            raise RuntimeError(f"{client.srv_name} did not answer")
        return future.result()

    def switch_controller(self, activate: bool) -> None:
        request = SwitchController.Request()
        if activate:
            request.activate_controllers = [self.controller]
        else:
            request.deactivate_controllers = [self.controller]
        request.strictness = SwitchController.Request.STRICT
        request.timeout.sec = 5
        response = self.call(self.switch, request)
        if not response.ok:
            raise RuntimeError(f"switching {self.controller} failed")

    def run(self) -> None:
        if self.controller:
            self.switch_controller(activate=False)
        response = self.call(self.reset, ResetWorld.Request())
        if not response.success:
            raise RuntimeError(f"reset_world failed: {response.message}")
        if self.controller:
            self.switch_controller(activate=True)
        self.get_logger().info(
            f"world reset to the stand keyframe; {self.controller or 'no controller'} "
            "re-activated")


def main(args=None) -> int:
    rclpy.init(args=args)
    node = SimReset()
    try:
        node.run()
    except RuntimeError as error:
        node.get_logger().error(str(error))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
