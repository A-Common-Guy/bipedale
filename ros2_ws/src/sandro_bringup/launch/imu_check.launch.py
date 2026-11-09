"""The real BNO085 alone, with no drives in the picture (tools/imu_only.sh as a launch).

The URDF is rendered for hardware:=real and every ros2_control block except
SandroImu is stripped, so the GIM6010 plugin is never loaded and nothing can
be armed. /imu_broadcaster/imu then carries the IMU exactly as in a full run:

    ros2 launch sandro_bringup imu_check.launch.py
"""

import os
import re
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

IMU_ARGUMENTS = ("imu_transport", "imu_spi_device", "imu_port", "imu_mount_yaw_rad")


def imu_only(urdf: str, component: str = "SandroImu") -> str:
    """The URDF with every <ros2_control> block but `component` removed."""
    for block in re.findall(r"<ros2_control\b.*?</ros2_control>", urdf, re.S):
        if component not in block.split(">", 1)[0]:
            urdf = urdf.replace(block, "")
    return urdf


def _setup(context):
    description = get_package_share_directory("sandro_description")
    bringup = get_package_share_directory("sandro_bringup")
    arguments = [f"{name}:={LaunchConfiguration(name).perform(context)}"
                 for name in IMU_ARGUMENTS if LaunchConfiguration(name).perform(context)]
    try:
        urdf = subprocess.check_output(
            ["xacro", os.path.join(description, "urdf", "sandro.urdf.xacro"),
             "control_backend:=real", *arguments], text=True)
    except (OSError, subprocess.CalledProcessError) as error:
        return [LogInfo(msg=f"Could not expand robot xacro: {error}")]
    urdf = imu_only(urdf)
    if urdf.count("<ros2_control") != 1:
        return [LogInfo(msg="IMU check: expected exactly the SandroImu ros2_control block")]
    return [
        LogInfo(msg="IMU CHECK: BNO085 only, no drive plugin loaded"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf}], output="screen"),
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[{"robot_description": urdf},
                         os.path.join(bringup, "config", "controllers.yaml")],
             output="screen"),
        # imu_broadcaster for the standard Imu message; health_broadcaster for
        # /health_broadcaster/dynamic_joint_states, which carries the driver's
        # sample age and online flag.
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster", "health_broadcaster", "imu_broadcaster",
                        "--controller-manager", "/controller_manager",
                        "--controller-manager-timeout", "30"],
             output="screen"),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("imu_transport", default_value="spi",
                              description="BNO085 link: spi (Raspberry Pi) or uart (FTDI)"),
        DeclareLaunchArgument("imu_spi_device", default_value="/dev/spidev6.0"),
        DeclareLaunchArgument("imu_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("imu_mount_yaw_rad", default_value="",
                              description="Chip yaw in the holder; empty keeps the URDF default"),
        OpaqueFunction(function=_setup),
    ])
