# Sandro ROS 2 bringup

Deployment scaffolding targets ROS 2 Jazzy on Ubuntu 24.04, using Cyclone DDS.
The existing `SandroControl/` application remains separate and unchanged.

## Quickstart

```bash
make build
make sim         # simulation with the whole-body controller (launch default)
make sim-cascade # simulation with the PID cascade (driving_balancer)
make shell     # development shell
make test      # builds and runs colcon tests
```

The image builds five local packages: `sandro_description`, `sandro_hardware`,
`sandro_controllers`, `sandro_sim` (latency and drive emulation for MuJoCo) and
`sandro_bringup`. It pins the maintained
`ros-controls/mujoco_ros2_control` source because a Jazzy binary is not
available in the configured apt repository.

For physical setup:

```bash
# Configure can0 on the host and expose only required serial/input devices in compose.yaml.
make real
```

`make real` **does not arm hardware**. The ROS IMU plugin reads the BNO085
over SPI on the Raspberry Pi by default (`imu_transport:=spi`,
`/dev/spidev6.0` with `dtoverlay=spi6-1cs`, INT on GPIO25, RST on GPIO24; the
same wiring as `raspi_tests/bno085_spi.py`). Set `SANDRO_IMU_DEVICE` and
`SANDRO_GPIO_CHIP` if the devices differ. The old FTDI UART link, which
delivered only about 30 samples per second on every recorded run
(`SIM_TO_REAL.md`), is still available:
`SANDRO_IMU_DEVICE=/dev/ttyUSB0 SANDRO_LAUNCH_ARGS=imu_transport:=uart make real`.
The launch default
`arm_hardware:=false` prevents the controller manager from starting. Read
`ros2_ws/src/sandro_bringup/docs/HARDWARE_COMMISSIONING.md` before overriding
it. Container networking is host mode for DDS and SocketCAN; serial and
gamepad nodes still need explicit `devices:` entries and host udev/group
permissions. X11 is mounted for RViz and the MuJoCo viewer. The X server also requires a
cookie, so `make sim`, `make shell`, and `make hardware-shell` first run
`make xauth`, which writes a container-readable copy to `/tmp/.sandro.xauth`.
Starting a container by hand needs `make xauth` first; otherwise GLFW and Qt
abort with "Authorization required, but no authorization protocol specified".
Without a display, use `headless:=true rviz:=false`.

After completing the commissioning checklist, deliberate full activation is:

```bash
ros2 launch sandro_bringup sandro.launch.py \
  hardware:=real arm_hardware:=true controller:=driving_balancer
```

Top-level launch arguments:

```bash
ros2 launch sandro_bringup sandro.launch.py \
  hardware:=mujoco controller:=whole_body_controller \
  teleop:=true rviz:=true headless:=false
```

PS5 teleoperation uses L1 as a deadman and publishes standard
`geometry_msgs/msg/TwistStamped` on `/cmd_vel`: left stick vertical is forward
speed, right stick horizontal is yaw rate, and right stick vertical is a body
height rate (`linear.z`, used by the whole-body controller only). L2 and R2
pressed together are a software E-stop that deactivates the active controller
(and fires the CAN E-stop on hardware); re-arm from the CLI. Verify
axis/button indices with `ros2 topic echo /joy` before use. Limits and units are explicit in
`sandro_bringup/config`; their source is `SandroControl/config.yaml`.

Tune any policy value with normal ROS parameters, for example:

```bash
ros2 param set /whole_body_controller balance.effort_weights.wheel_torque 5.0
ros2 param dump /whole_body_controller --output-dir tuned_profiles
ros2 control load_controller --set-state inactive my_controller
ros2 control switch_controllers --deactivate whole_body_controller --activate my_controller
```

All torque, slew, velocity, angle, and timeout boundaries are named. Controller
diagnostics report requested/effective values and saturation causes; the
hardware also exports requested/applied effort and saturation counters on
`/dynamic_joint_states`.

See package docs for
[controller extension](ros2_ws/src/sandro_bringup/docs/CONTROLLERS.md),
[the whole-body controller](ros2_ws/src/sandro_bringup/docs/WHOLE_BODY_CONTROLLER.md),
[modelling and control design](ros2_ws/src/sandro_bringup/docs/MODELLING_AND_CONTROL.md),
[closing the sim-to-real gap](ros2_ws/src/sandro_bringup/docs/SIM_TO_REAL.md),
[hardware commissioning](ros2_ws/src/sandro_bringup/docs/HARDWARE_COMMISSIONING.md),
and [dependency decisions](ros2_ws/src/sandro_bringup/docs/DEPENDENCIES.md).
