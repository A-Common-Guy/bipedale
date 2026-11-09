# sandro_gui

The robot's control panel, served by the Raspberry Pi on port 8080: start and
stop the stack, arm the drives, start controllers, tune parameters, watch
every sensor's rate and freshness, see the robot in 3D, pair the gamepad.
How to drive it, step by step and button by button: [docs/USER_GUIDE.md](docs/USER_GUIDE.md).

| Page | What it does |
|---|---|
| Overview | The CAD robot posed from the joint angles and tilted by the IMU, with the IMU, BNO085 chip, `base_link` and world frames; pre-flight checks (stack, CAN, drives, IMU, encoders, controller, gamepad, Pi); every drive's state, error, temperature and frame rates; the warnings and errors of the run, repeats folded. |
| Run | Start form (IMU check, drives off, armed; controller; teleop; robot overrides) with the exact `ros2 launch` command; Stop; hardware arm and disarm; controllers start, stop, re-configure; the running controller's `~/diagnostics`; the launch log and past runs. |
| Tune | Any node's parameters, set live through the parameter services, changes marked against the launch values, revert, and an export of the changes as a `ros__parameters` file. |
| IMU | Attitude, rates, acceleration, topic rate against new-sample rate, sample age, strip charts. |
| Gamepad | Bluetooth scan, pair, connect, forget, rfkill unblock; the pad's live input as the robot sees it and the `/cmd_vel` teleop sends. |
| Guide | The start-up procedure and every gamepad combination, generated from `teleop.yaml`. |

## Running it

On the robot nothing needs starting: the container's main process runs the
panel and restarts it if it exits (`deploy/pi/pi`, see below). By hand:

```bash
ros2 run sandro_gui sandro_gui                     # http://<host>:8080
ros2 run sandro_gui sandro_gui --hardware mujoco   # Start launches the simulation
ros2 run sandro_gui sandro_gui --demo              # synthetic robot, no ROS needed
```

Without ROS at all (for work on the page itself), from this directory:
`python3 -m sandro_gui.main --demo --port 8090`.

The panel is a single Python process, standard library plus rclpy, numpy and
PyYAML, which the robot image already has. The browser side is plain ES
modules and a vendored three.js r170 (MIT, `web/vendor/`); nothing is fetched
from the internet, since the robot's network often has none.

## On the Pi

`deploy/pi/pi setup` (once, or after pulling this) builds the image with
BlueZ's `bluetoothctl`, and `deploy/pi/pi reset` recreates the container
with the host's D-Bus socket (pairing) and the panel as its main process.
Then:

```bash
deploy/pi/pi build        # sync and build, as before
deploy/pi/pi gui          # address and state of the panel
deploy/pi/pi gui restart  # after changing the panel's Python
deploy/pi/pi gui log      # the panel's own output
```

`deploy/pi/pi launch` still works from a terminal; the panel then shows that
launch as *external* and can stop it, but its output stays in the terminal.

## How it works

- **Launch.** Start runs `ros2 launch sandro_bringup sandro.launch.py ...` (or
  `imu_check.launch.py`) in its own session with the output going to
  `gui_runs/run-*.log`, never to a pipe: restarting the panel does not take
  the robot down, and the panel re-attaches to the run on start. Stop is one
  SIGINT to the process group, as Ctrl-C: the controller manager deactivates
  the drives before it exits; SIGTERM follows only after `stop_timeout_s`.
- **Before arming.** Without a controller manager there are no ROS topics, so
  a raw CAN socket listens to the bus (read only): heartbeat (axis state,
  error, temperature), encoder broadcast (turned into logical joint angles
  with the node, sign, ratio and offset of `sandro.ros2_control.xacro`) and
  the host's MIT commands, flagged by the kernel. The 3D view uses those
  angles until `/joint_states` exists.
- **Rates.** Raw subscriptions count messages without deserialising them;
  `/joint_states` and the IMU are compared sample to sample, so the panel
  shows both the topic rate (the 300 Hz control loop) and the rate of samples
  that actually changed (the encoders' 200 Hz, the BNO085's 240 Hz).
- **Controllers.** Only the controller manager's own services (load,
  configure, switch, hardware lifecycle) and the E-stop services, the calls
  `ros2 control` and `joy_estop` make. Starting a policy of the gamepad switch
  also loads the others inactive, as the launch does, so L1 + Triangle works.
  The panel never publishes a command.
- **3D model.** Built once from `sandro_description/mjcf/robot_full_floating.xml`
  and cached in `gui_runs/cache/`: visual meshes grouped into the rigid bodies
  of each five-bar (two cranks, two lower links, the wheel), clustered on a
  1 mm grid, quantised, about 0.6 MB gzipped. The browser poses each group
  with the controllers' forward kinematics, so a wrong joint reading looks
  wrong. Yaw is removed from the attitude, so the view does not drift with
  the gyro.
- **Errors.** Launch output is parsed into level, source and text (the drive
  and IMU plugins log through their own loggers, which never reach
  `/rosout`); `/rosout` is used for launches started elsewhere. Warnings and
  errors with the same text up to the numbers become one problem with a count.

The settings are in [config/gui.yaml](config/gui.yaml): port, launch arguments
(keep them in step with `deploy/pi/pi`'s `LAUNCH_DEFAULTS`), watched topics and
their expected rates, the controllers the Run page offers.

The panel has no login: anyone on the robot's network can drive it, exactly
as anyone there can reach DDS. Keep the Pi on a private network.
