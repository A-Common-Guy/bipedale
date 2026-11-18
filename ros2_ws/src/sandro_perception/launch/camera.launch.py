"""The depth camera and the terrain profile, beside whatever stack is running.

    ros2 launch sandro_perception camera.launch.py [driver:=false] [extra_params:=FILE]

driver:=false starts only the height_profile node (a bag or a simulated camera supplies the
depth). extra_params: a parameter file applied on top, e.g. the robot overrides.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    params = [os.path.join(get_package_share_directory("sandro_perception"), "config", "perception.yaml")]
    extra = LaunchConfiguration("extra_params").perform(context)
    if extra:
        params.append(extra)
    nodes = [Node(package="sandro_perception", executable="height_profile", name="height_profile",
                  parameters=params, output="screen")]
    if LaunchConfiguration("driver").perform(context).lower() in ("true", "1", "yes"):
        nodes.append(Node(package="realsense2_camera", executable="realsense2_camera_node",
                          namespace="camera", name="camera", parameters=params, output="screen"))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("driver", default_value="true",
                              description="start realsense2_camera (false: depth comes from elsewhere)"),
        DeclareLaunchArgument("extra_params", default_value=""),
        OpaqueFunction(function=_setup),
    ])
