"""Top-level Sandro launch with safe, dependency-aware fallbacks."""

import os
import subprocess

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _truth(value):
    return value.lower() in ("1", "true", "yes", "on")


def _package(name):
    try:
        return get_package_share_directory(name)
    except PackageNotFoundError:
        return None


def _setup(context):
    hardware = LaunchConfiguration("hardware").perform(context)
    controller = LaunchConfiguration("controller").perform(context)
    teleop = _truth(LaunchConfiguration("teleop").perform(context))
    rviz = _truth(LaunchConfiguration("rviz").perform(context))
    headless = _truth(LaunchConfiguration("headless").perform(context))
    arm_hardware = _truth(LaunchConfiguration("arm_hardware").perform(context))
    sim_reset = _truth(LaunchConfiguration("sim_reset").perform(context))
    sim_latency = _truth(LaunchConfiguration("sim_latency").perform(context))
    extra_params = LaunchConfiguration("extra_params").perform(context)
    bringup = _package("sandro_bringup")
    actions = []

    if hardware == "sim":
        hardware = "mujoco"  # Backward-compatible shorthand.
    if hardware not in ("mujoco", "real"):
        return [LogInfo(msg=f"Invalid hardware='{hardware}'; expected mujoco or real.")]

    description_share = _package("sandro_description")
    robot_description = None
    if description_share:
        candidates = [
            os.path.join(description_share, "urdf", "sandro.urdf.xacro"),
            os.path.join(description_share, "urdf", "sandro.xacro"),
        ]
        xacro_file = next((path for path in candidates if os.path.isfile(path)), None)
        if xacro_file:
            try:
                robot_description = subprocess.check_output(
                    [
                        "xacro", xacro_file,
                        f"control_backend:={hardware}",
                        f"can_interface:={LaunchConfiguration('can_interface').perform(context)}",
                        *[
                            f"{name}:={LaunchConfiguration(name).perform(context)}"
                            for name in ("imu_transport", "imu_spi_device", "imu_port")
                        ],
                        f"state_source:={LaunchConfiguration('state_source').perform(context)}",
                        f"encoder_period_ms:={LaunchConfiguration('encoder_period_ms').perform(context)}",
                        f"simulation_headless:={'true' if headless else 'false'}",
                        f"simulation_latency:={'true' if sim_latency else 'false'}",
                        *[
                            f"{name}:={LaunchConfiguration(name).perform(context)}"
                            for name in ("sim_imu_delay_s", "sim_imu_rate_hz", "sim_joint_delay_s",
                                         "sim_joint_rate_hz", "sim_joint_stagger",
                                         "sim_command_delay_s")
                        ],
                    ],
                    text=True,
                )
            except (OSError, subprocess.CalledProcessError) as exc:
                actions.append(LogInfo(msg=f"Could not expand robot xacro: {exc}"))
        else:
            actions.append(LogInfo(msg="sandro_description found, but no standard xacro found."))
    else:
        actions.append(LogInfo(
            msg="sandro_description is not installed; skipping robot state/control nodes."
        ))

    if robot_description:
        actions.append(Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ))
        if hardware == "real":
            actions.append(Node(
                package="sandro_hardware",
                executable="gim6010_estop",
                parameters=[{
                    "can_interface": LaunchConfiguration(
                        "can_interface").perform(context),
                    "node_ids": [1, 2, 3, 4, 5, 6],
                }],
                output="screen",
            ))
        control_allowed = hardware == "mujoco" or arm_hardware
        if hardware == "real" and not arm_hardware:
            actions.append(LogInfo(
                msg="REAL SAFE STATE: controller manager not started and drives are not armed. "
                    "Use arm_hardware:=true only after commissioning checks."
            ))
        elif hardware == "mujoco" and not _package("mujoco_ros2_control"):
            actions.append(LogInfo(
                msg="mujoco_ros2_control is absent; skipping simulation controller manager."
            ))
            control_allowed = False

        sim_share = _package("sandro_sim") if hardware == "mujoco" else None
        if control_allowed:
            actions.append(Node(
                package=(
                    "mujoco_ros2_control" if hardware == "mujoco"
                    else "controller_manager"
                ),
                executable="ros2_control_node",
                parameters=[
                    {"robot_description": robot_description},
                    os.path.join(bringup, "config", "controllers.yaml"),
                    # The MuJoCo backend's plugins: the per-step MIT drive law.
                    *([os.path.join(sim_share, "config", "mujoco_plugins.yaml")]
                      if sim_share else []),
                    # Optional overrides (tuning experiments, per-robot values)
                    # applied after the shipped configuration.
                    *([extra_params] if extra_params else []),
                    {"use_sim_time": hardware == "mujoco"},
                ],
                output="screen",
            ))
            for broadcaster in ("joint_state_broadcaster", "imu_broadcaster"):
                actions.append(Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        broadcaster, "--controller-manager", "/controller_manager",
                        "--controller-manager-timeout", "30",
                    ],
                    output="screen",
                ))
            if controller != "none":
                controller_spawner = Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        controller, "--controller-manager", "/controller_manager",
                        "--controller-manager-timeout", "30",
                    ],
                    output="screen",
                )
                actions.append(controller_spawner)
                if hardware == "mujoco" and sim_reset:
                    # Physics runs from the moment the controller manager starts,
                    # about a second before the balancer is active, and unpowered
                    # legs collapse in that time. Once the controller is up, put
                    # the robot back on the stand keyframe with the balancer
                    # deactivated around the reset (sandro_bringup/sim_reset), so
                    # it engages on the standing robot with fresh references,
                    # exactly as it would when an operator lifts the real one.
                    actions.append(RegisterEventHandler(OnProcessExit(
                        target_action=controller_spawner,
                        on_exit=[Node(
                            package="sandro_bringup",
                            executable="sim_reset",
                            name="sim_reset",
                            output="screen",
                            parameters=[{"controller": controller}],
                        )],
                    )))

    if teleop and hardware == "real" and not arm_hardware:
        actions.append(LogInfo(
            msg="REAL SAFE STATE: teleop publisher suppressed until arm_hardware:=true."
        ))
    elif teleop:
        if _package("joy") and _package("teleop_twist_joy"):
            teleop_config = os.path.join(bringup, "config", "teleop.yaml")
            actions.extend([
                Node(package="joy", executable="joy_node",
                     parameters=[teleop_config], output="screen"),
                Node(package="teleop_twist_joy", executable="teleop_node",
                     name="teleop_twist_joy_node",
                     parameters=[teleop_config], output="screen"),
                # Gamepad E-stop chord (L2 + R2): deactivates the balancer and,
                # on hardware, fires the CAN E-stop. Software only; the
                # physical cut-off stays mandatory.
                Node(package="sandro_bringup", executable="joy_estop",
                     name="joy_estop",
                     parameters=[teleop_config], output="screen"),
            ])
        else:
            actions.append(LogInfo(msg="joy/teleop_twist_joy unavailable; teleop skipped."))

    if rviz and not headless:
        if _package("rviz2"):
            # rviz2 routinely ignores SIGINT for the default 5 + 10 s, which
            # is what tempts a second Ctrl-C that then kills the controller
            # manager mid-shutdown (exit code -2, drives left to the deadman).
            actions.append(Node(package="rviz2", executable="rviz2", output="screen",
                                sigterm_timeout="2", sigkill_timeout="3"))
        else:
            actions.append(LogInfo(msg="rviz2 unavailable; visualization skipped."))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("hardware", default_value="mujoco",
                              description="mujoco (alias: sim) or real"),
        DeclareLaunchArgument(
            "controller", default_value="whole_body_controller",
            description=(
                "Controller to spawn, or none: whole_body_controller, "
                "driving_balancer (cascade), lqr_balancer (full state feedback)"
            ),
        ),
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument("imu_transport", default_value="spi",
                              description="BNO085 link: spi (Raspberry Pi) or uart (FTDI)"),
        DeclareLaunchArgument("imu_spi_device", default_value="/dev/spidev6.0"),
        DeclareLaunchArgument("imu_port", default_value="/dev/ttyUSB0",
                              description="UART device, used with imu_transport:=uart"),
        DeclareLaunchArgument(
            "state_source", default_value="encoder",
            description="real only: joint state from the 0x09 'encoder' broadcast or the synchronous 'mit' reply",
        ),
        DeclareLaunchArgument(
            "encoder_period_ms", default_value="5",
            description="real only: period of the drives' encoder broadcast; 20 is enough with state_source:=mit",
        ),
        DeclareLaunchArgument("teleop", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument(
            "arm_hardware", default_value="false",
            description="Explicit real-hardware controller-manager opt-in; never defaults true",
        ),
        DeclareLaunchArgument(
            "sim_reset", default_value="true",
            description="MuJoCo only: reset to the stand keyframe once the controller is active",
        ),
        DeclareLaunchArgument(
            "sim_latency", default_value="true",
            description="MuJoCo only: emulate sensor and command latency (sandro_sim/LatencySystem)",
        ),
        DeclareLaunchArgument("sim_imu_delay_s", default_value="0.010",
                              description="MuJoCo only: IMU latency from sample to host, seconds"),
        DeclareLaunchArgument("sim_imu_rate_hz", default_value="200",
                              description="MuJoCo only: IMU sample rate in Hz (0 = every cycle)"),
        DeclareLaunchArgument("sim_joint_delay_s", default_value="0.001",
                              description="MuJoCo only: encoder latency from sample to host "
                                          "(CAN frame plus USB adapter), seconds"),
        DeclareLaunchArgument("sim_joint_rate_hz", default_value="200",
                              description="MuJoCo only: encoder sample rate in Hz (0 = every cycle)"),
        DeclareLaunchArgument("sim_joint_stagger", default_value="true",
                              description="MuJoCo only: drives sample at spread phases (broadcast); "
                                          "false for state_source:=mit replies"),
        DeclareLaunchArgument("sim_command_delay_s", default_value="0.001",
                              description="MuJoCo only: latency from write() to the drive "
                                          "(USB adapter plus CAN frames), seconds"),
        DeclareLaunchArgument(
            "extra_params", default_value="",
            description="Additional controller-manager YAML applied after controllers.yaml",
        ),
        OpaqueFunction(function=_setup),
    ])
