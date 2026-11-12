# sandro_gui

The robot's control panel, served by the Raspberry Pi on port 8080: start and
stop the stack, arm the drives, start controllers, tune parameters, watch
every sensor's rate and freshness, see the robot in 3D, pair the gamepad.
How to drive it, step by step and button by button: [docs/USER_GUIDE.md](docs/USER_GUIDE.md).

| Page | What it does |
|---|---|
| Overview | The CAD robot posed from the joint angles and tilted by the IMU, with the IMU, BNO085 chip, `base_link` and world frames; pre-flight checks (stack, CAN, drives, IMU, encoders, controller, gamepad, Pi); every drive's state, error, temperature and frame rates; the warnings and errors of the run, repeats folded. |
| Run | Start stack (one launch, drives not armed, the chosen controller preloaded; IMU check as a diagnostic) with the exact `ros2 launch` command and its starting → ready state; Arm, Start, Arm and start, Re-arm and restart after an E-stop, Restart of a fallen controller; Stop; the hardware and controller tables; the running controller's `~/diagnostics`; the launch log and past runs. |
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

Updates: the Software page deploys a commit from `~/sandro/repo.git`
(`deploy/pi/pi push` from the laptop, or Fetch from GitHub), builds what it
changed and restarts the panel; see `deploy/pi/README.md`.

## How it works

- **Software** (`software.py`). `version/source.json` is what `ros2_ws/src`
  holds and `version/build.json` what `install/` was built from; `pi sync`,
  `pi build` and a deploy write them. A deploy exports one commit of
  `repo.git` with `git archive`, writes only the files that differ (with the
  current time, so make rebuilds exactly those; as root, handed back to the
  owner of `ros2_ws/src` so `pi sync` can still overwrite them), builds the
  packages `pi build`'s change_scope would, and exits so `panel.sh` starts the
  panel on the new build; the browser reloads when the panel's start time
  changes. Refused while a launch runs.
- **Launch.** Start runs `ros2 launch sandro_bringup sandro.launch.py ...
  arm_hardware:=false controller:=<name>` (or `imu_check.launch.py`) in its
  own session with the output going to `gui_runs/run-*.log` (the last
  `keep_runs`, at most `keep_mb`), never to a pipe: restarting the panel does
  not take the robot down, and the panel re-attaches to the run on start. The
  run is *starting* until the hardware components are in their expected
  states, the broadcasters are active and the controller is loaded; a
  `process has died` line marks a crash at once. Arming
  (`set_hardware_component_state` active) asks once per launch that the robot
  is secured. Stop is one SIGINT to the process group, as Ctrl-C: the
  controller manager deactivates the drives before it exits; SIGTERM follows
  only after `stop_timeout_s`.
- **Before a launch.** Without a controller manager there are no ROS topics, so
  a raw CAN socket listens to the bus (read only): heartbeat (axis state,
  error, temperature), encoder broadcast (turned into logical joint angles
  with the node, sign, ratio and offset of `sandro.ros2_control.xacro`) and
  the host's MIT commands, flagged by the kernel. The 3D view uses those
  angles until `/joint_states` exists.
- **Rates.** Raw subscriptions count messages without deserialising them.
  The 300 Hz topics (`sampled` in gui.yaml) sit on a node no executor spins:
  the ticker takes their queued messages 20 times a second, counting each, so
  they cost no executor wake-up per message. Drive health comes from
  `health_broadcaster` at 10 Hz (`/dynamic_joint_states` of older stacks is
  read too). `/rosout` is subscribed only while the launch runs elsewhere.
  With no browser connected no snapshot is built; the stream is gzipped.
- **Controllers.** Only the controller manager's own services (load,
  configure, switch, hardware lifecycle), the E-stop services and a policy's
  `~/arm_handover`, the calls `ros2 control`, `joy_teleop` and `joy_estop`
  make; the sequences are in `control.py`, shared with the demo and the
  tests. One such action runs at a time (a second gets HTTP 409). Starting a
  policy of the gamepad switch also loads the others inactive, as the launch
  does, so L1 + Triangle works. A controller whose `~/diagnostics` level is
  ERROR shows *fallen* (or its error) with a Restart. The panel never
  publishes a command.
- **E-stop.** The header button first sends the GIM6010 E-stop frame on the
  panel's own CAN socket (the only frame the panel ever transmits), then calls
  the E-stop node's service and deactivates the controllers in parallel, and
  aborts any action still in flight.
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
