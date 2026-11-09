"""Top-level Sandro launch with safe, dependency-aware fallbacks."""

import os
import subprocess

import yaml

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DRIVES = "SandroSystem"
# Spawned active in one spawner call, which stops at the first failure: the
# IMU goes last so a missing IMU cannot keep the joint state off.
BROADCASTERS = ("joint_state_broadcaster", "health_broadcaster", "imu_broadcaster")
# ros2_control_node on the Raspberry Pi 4 (cores 0-3): SCHED_FIFO priority and
# core of the control-loop thread, and mlockall of the whole process.
REALTIME = {"thread_priority": 50, "cpu_affinity": 3, "lock_memory": True}
# What the "auto" launch arguments resolve to for each state_source: the
# encoder broadcast period on the robot, and the joint sampling MuJoCo
# emulates. 'encoder': free-running 200 Hz broadcasts at spread phases; 'mit':
# every drive replies to each 300 Hz command, ~2.3 ms after it is sent
# (docs/SIM_TO_REAL.md). sandro_rl/sources.py reads this table.
JOINT_TIMING = {
    "encoder": {"encoder_period_ms": "5", "sim_joint_delay_s": "0.001",
                "sim_joint_rate_hz": "200", "sim_joint_stagger": "true"},
    "mit": {"encoder_period_ms": "20", "sim_joint_delay_s": "0.0023",
            "sim_joint_rate_hz": "300", "sim_joint_stagger": "false"},
}


def _truth(value):
    return value.lower() in ("1", "true", "yes", "on")


def _joint_timing(context, name):
    """A JOINT_TIMING launch argument, 'auto' resolved from state_source."""
    value = LaunchConfiguration(name).perform(context)
    if value != "auto":
        return value
    source = LaunchConfiguration("state_source").perform(context)
    if source not in JOINT_TIMING:
        raise ValueError(f"state_source must be one of {sorted(JOINT_TIMING)}, not {source!r}")
    return JOINT_TIMING[source][name]


def _package(name):
    try:
        return get_package_share_directory(name)
    except PackageNotFoundError:
        return None


def _switchable(bringup, controller):
    """The gamepad switch's other policies when the launched controller is one of them."""
    with open(os.path.join(bringup, "config", "teleop.yaml"), encoding="utf-8") as stream:
        policies = yaml.safe_load(stream)["joy_teleop"]["ros__parameters"]["policies"]
    return [p for p in policies if p != controller] if controller in policies else []


def _spawner(controllers, *flags):
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            *controllers, *flags, "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        output="screen",
    )


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
                        f"simulation_headless:={'true' if headless else 'false'}",
                        f"simulation_latency:={'true' if sim_latency else 'false'}",
                        *[
                            f"{name}:={LaunchConfiguration(name).perform(context)}"
                            for name in ("sim_imu_delay_s", "sim_imu_rate_hz", "sim_command_delay_s")
                        ],
                        *[f"{name}:={_joint_timing(context, name)}" for name in JOINT_TIMING["encoder"]],
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
        control_allowed = True
        # One launch per session on the robot: the controller manager runs from
        # the start with the IMU active and the drives configured but inactive
        # (no torque, write() is never called), the broadcasters publish, and
        # the requested controller waits inactive. Arming is
        # set_hardware_component_state SandroSystem active, then activating the
        # controller (control panel, joy_estop or ros2 control).
        session = hardware == "real" and not arm_hardware
        if session:
            actions.append(LogInfo(
                msg=f"REAL SAFE STATE: drives are not armed; {DRIVES} starts inactive (no "
                    "torque)" + (f" and {controller} waits inactive" if controller != "none"
                                 else "") +
                    f". Arm from the control panel, or: ros2 control "
                    f"set_hardware_component_state {DRIVES} active, then ros2 control "
                    "switch_controllers --activate <controller>. arm_hardware:=true arms at "
                    "launch; use it only after commissioning checks."
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
                    # The sandro_rl policies (generated by export_ros.py).
                    os.path.join(bringup, "config", "rl_policies.yaml"),
                    # The MuJoCo backend's plugins: the per-step MIT drive law.
                    *([os.path.join(sim_share, "config", "mujoco_plugins.yaml")]
                      if sim_share else []),
                    *([REALTIME] if hardware == "real" else []),
                    # Optional overrides (tuning experiments, per-robot values)
                    # applied after the shipped configuration.
                    *([extra_params] if extra_params else []),
                    {"use_sim_time": hardware == "mujoco"},
                    # After extra_params, so no override file can arm the drives.
                    # A drive that fails to configure (CAN down) leaves the
                    # IMU and the panel running instead of the whole manager.
                    *([{
                        "hardware_components_initial_state.inactive": [DRIVES],
                        "hardware_components_initial_state.shutdown_on_initial_state_failure":
                            False,
                    }] if session else []),
                ],
                output="screen",
            ))
            actions.append(_spawner(BROADCASTERS))
            if controller != "none":
                # The other policies of the gamepad switch, loaded and
                # configured but inactive, so a switch is a single activation.
                partners = _switchable(bringup, controller)
                if session:
                    actions.append(_spawner([controller, *partners], "--inactive"))
                else:
                    # Its own call: sim_reset waits for its exit, the activation.
                    controller_spawner = _spawner([controller])
                    actions.append(controller_spawner)
                    if partners:
                        actions.append(_spawner(partners, "--inactive"))
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

    # Teleop runs in both real modes: nothing moves without an active
    # controller on active drives, and joy_estop never arms the drives first.
    if teleop:
        if _package("joy"):
            teleop_config = os.path.join(bringup, "config", "teleop.yaml")
            actions.extend([
                Node(package="joy", executable="joy_node",
                     parameters=[teleop_config],
                     arguments=["--ros-args", "--log-level", "warn"], output="screen"),
                # Sticks to /cmd_vel scaled to the running controller, the
                # soft policy switch and the rumble that tells the policy.
                Node(package="sandro_bringup", executable="joy_teleop",
                     name="joy_teleop",
                     parameters=[teleop_config], output="screen"),
                # Gamepad E-stop chord (L2 + R2): deactivates the balancer and,
                # on hardware, fires the CAN E-stop. Software only; the
                # physical cut-off stays mandatory.
                # Its re-arm restarts the controller that ran last, this
                # launch's until another one has run.
                Node(package="sandro_bringup", executable="joy_estop",
                     name="joy_estop",
                     parameters=[teleop_config,
                                 *([{"rearm_controller": controller}]
                                   if controller != "none" else [])],
                     output="screen"),
            ])
        else:
            actions.append(LogInfo(msg="joy unavailable; teleop skipped."))

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
                "driving_balancer (cascade), lqr_balancer (full state feedback), "
                "rl_flat_level, rl_flat_level_fast (300 Hz) or rl_agile_terrain (sandro_rl policies), "
                "rl_stand_up (engages lying down, gets up, then balances)"
            ),
        ),
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument("imu_transport", default_value="spi",
                              description="BNO085 link: spi (Raspberry Pi) or uart (FTDI)"),
        DeclareLaunchArgument("imu_spi_device", default_value="/dev/spidev6.0"),
        DeclareLaunchArgument("imu_port", default_value="/dev/ttyUSB0",
                              description="UART device, used with imu_transport:=uart"),
        DeclareLaunchArgument(
            "state_source", default_value="mit",
            description="joint state from the synchronous 'mit' reply (default since 2026-11-09) or "
                        "the 0x09 'encoder' broadcast; in MuJoCo it selects the matching emulated "
                        "joint timing",
        ),
        DeclareLaunchArgument(
            "encoder_period_ms", default_value="auto",
            description="real only: period of the drives' encoder broadcast; auto is 20, or 5 with "
                        "state_source:=encoder",
        ),
        DeclareLaunchArgument("teleop", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument(
            "arm_hardware", default_value="false",
            description="real only: arm the drives and start the controller at launch; "
                        "false starts them inactive for arming later; never defaults true",
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
        DeclareLaunchArgument("sim_joint_delay_s", default_value="auto",
                              description="MuJoCo only: joint state latency from sample to host "
                                          "(CAN frame plus USB adapter), seconds; auto from state_source"),
        DeclareLaunchArgument("sim_joint_rate_hz", default_value="auto",
                              description="MuJoCo only: joint sample rate in Hz (0 = every cycle); "
                                          "auto from state_source"),
        DeclareLaunchArgument("sim_joint_stagger", default_value="auto",
                              description="MuJoCo only: drives sample at spread phases (broadcast) "
                                          "or together (MIT replies); auto from state_source"),
        DeclareLaunchArgument("sim_command_delay_s", default_value="0.001",
                              description="MuJoCo only: latency from write() to the drive "
                                          "(USB adapter plus CAN frames), seconds"),
        DeclareLaunchArgument(
            "extra_params", default_value="",
            description="Additional controller-manager YAML applied after controllers.yaml",
        ),
        OpaqueFunction(function=_setup),
    ])
