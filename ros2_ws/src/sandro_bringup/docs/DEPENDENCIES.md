# Dependency decisions

- Base: `ros:jazzy-ros-base-noble` fixes ROS 2 Jazzy and Ubuntu 24.04.
- DDS: `rmw_cyclonedds_cpp` is selected in the image and Compose. The checked-in
  Cyclone DDS file uses automatic interface selection so it works on different
  hosts; production deployments should pin the intended NIC.
- Control: ROS 2 control, controller manager, standard controllers, joint state
  broadcaster, diagnostics, xacro, robot state publisher, Joy,
  teleop_twist_joy, and RViz use Jazzy binary packages.
- MuJoCo: the maintained `ros-controls/mujoco_ros2_control` project is pinned
  by commit in the Dockerfile because the configured Jazzy apt repository has
  no binary. The existing composed MJCF remains the physics source.
- CAN: `ros2_socketcan` is maintained, but its topic bridge would put DDS in the
  300 Hz actuator path and does not implement GIM6010 lifecycle/SDO/MIT
  semantics. `sandro_hardware` therefore uses Linux SocketCAN directly.
- IMU: available BNO08x ROS projects do not provide a clearly maintained
  Jazzy SHTP implementation over both UART and SPI. The small sensor plugin
  implements both (spidev and the GPIO character device directly, no extra
  library) behind standard IMU state interfaces.
- Host networking is required for straightforward DDS discovery and access to
  host SocketCAN. Serial/gamepad access is separate and must use narrowly
  scoped Compose `devices:` plus host udev/group permissions.
