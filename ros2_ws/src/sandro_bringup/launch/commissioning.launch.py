"""Explicit, timeout-protected single-motor or single-leg commissioning."""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


MOTORS = {
    "right_inner": "commission_right_motor_inner",
    "right_outer": "commission_right_motor_outer",
    "right_wheel": "commission_right_wheel",
    "left_inner": "commission_left_motor_inner",
    "left_outer": "commission_left_motor_outer",
    "left_wheel": "commission_left_wheel",
}
LEGS = {"right": "commission_right_leg", "left": "commission_left_leg"}


def _setup(context):
    target_type = LaunchConfiguration("target_type").perform(context)
    target = LaunchConfiguration("target").perform(context)
    activate = LaunchConfiguration("activate").perform(context).lower() == "true"
    choices = MOTORS if target_type == "motor" else LEGS if target_type == "leg" else {}
    if target not in choices:
        return [LogInfo(msg=(
            f"Invalid {target_type} target '{target}'. Motors: {sorted(MOTORS)}; "
            f"legs: {sorted(LEGS)}."
        ))]
    if not activate:
        return [LogInfo(msg=(
            f"SAFE PREVIEW: {target_type}={target}. Nothing was started. "
            "Re-run with activate:=true only with the robot lifted, physical E-stop "
            "reachable, zero/sign checked, and the 0.5 Nm test boundary understood."
        ))]

    description = get_package_share_directory("sandro_description")
    bringup = get_package_share_directory("sandro_bringup")
    robot_description = subprocess.check_output(
        [
            "xacro", os.path.join(description, "urdf", "sandro.urdf.xacro"),
            "control_backend:=real",
            f"can_interface:={LaunchConfiguration('can_interface').perform(context)}",
            f"imu_port:={LaunchConfiguration('imu_port').perform(context)}",
        ],
        text=True,
    )
    manager = "/controller_manager"
    return [
        LogInfo(msg=(
            f"ARMING COMMISSIONING MODE for {target_type}={target}. "
            "Commands time out after 0.1 s and clamp at 0.5 Nm."
        )),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="sandro_hardware",
            executable="gim6010_estop",
            parameters=[{
                "can_interface": LaunchConfiguration(
                    "can_interface").perform(context),
                "node_ids": [1, 2, 3, 4, 5, 6],
            }],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description},
                os.path.join(bringup, "config", "controllers.yaml"),
            ],
            output="screen",
        ),
        Node(
            package="controller_manager", executable="spawner",
            arguments=["joint_state_broadcaster", "-c", manager,
                       "--controller-manager-timeout", "30"],
            output="screen",
        ),
        Node(
            package="controller_manager", executable="spawner",
            arguments=["imu_broadcaster", "-c", manager,
                       "--controller-manager-timeout", "30"],
            output="screen",
        ),
        Node(
            package="controller_manager", executable="spawner",
            arguments=[choices[target], "-c", manager,
                       "--controller-manager-timeout", "30"],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("target_type", default_value="motor",
                              description="motor or leg"),
        DeclareLaunchArgument("target", default_value="right_inner",
                              description="Exactly one motor name or leg side"),
        DeclareLaunchArgument("activate", default_value="false",
                              description="Explicit hardware-arm opt-in; defaults safe"),
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument("imu_port", default_value="/dev/ttyUSB0"),
        OpaqueFunction(function=_setup),
    ])
