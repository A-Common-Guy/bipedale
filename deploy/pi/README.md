# Running Sandro on the Raspberry Pi

The Pi runs the real-hardware stack (`hardware:=real`) in an arm64 Jazzy container;
the development machine edits code, runs the simulator and RViz. Both are on the same
DDS network (host networking), so `ros2 topic echo` and RViz work from the laptop.

```bash
deploy/pi/pi setup     # once: docker, can0 at boot, performance governor, robot image
deploy/pi/pi launch    # every time: sync + incremental build + launch (hardware:=real)
```

Also available as `make pi-launch`, `make pi-build`, `make pi-shell`, `make pi-can`.

## Control panel

The container's main process is the control panel (`ros2_ws/src/sandro_gui`), so
after a build the robot is driven from a browser on **http://10.42.0.169:8080**:
check the bus, drives, IMU and gamepad, start the stack (IMU check, drives off,
armed), start controllers, tune parameters, pair the pad. It comes up with the Pi.
The procedure and every gamepad combination are in
[the user guide](../../ros2_ws/src/sandro_gui/docs/USER_GUIDE.md).

```bash
deploy/pi/pi setup && deploy/pi/pi reset   # once: image with bluetoothctl, panel as main process
deploy/pi/pi build                          # builds sandro_gui with the rest
deploy/pi/pi gui                            # address and state; `gui restart`, `gui log`
```

A launch the panel started keeps running when the panel restarts, and its output is
kept in `~/sandro/gui_runs/run-*.log`. `deploy/pi/pi launch` still works; the panel
shows that run as external and can stop it.

## Versions and updates

The panel's **Software** page (and the SW chip in its header) shows which commit is
on disk, built, running in the panel and in the stack, and updates the robot with one
button: *Deploy* writes that commit's `ros2_ws/src` (only the files that differ), builds
the packages they need with the same rules as `pi build`, and restarts the panel on
the new build. The robot stack must be stopped. Every launch log starts with the
version it ran.

The commits come from the robot's own repository, `~/sandro/repo.git`:

```bash
deploy/pi/pi push      # the laptop's current commit, over SSH; no internet on the robot needed
deploy/pi/pi deploy    # push, then the panel deploys it (progress on the Software page)
```

or *Fetch from GitHub* on the page, when the robot has internet (the repository is
public, no key needed). `pi sync` and `pi build` keep working for uncommitted work and
stamp the version too: the commit plus "uncommitted changes", synced from which host.

## Why it is fast

| Change                                     | Cost on the next `launch`                          |
|--------------------------------------------|----------------------------------------------------|
| Edit to a Python, YAML, URDF, launch file  | rsync only (~1 s), live through `--symlink-install` |
| New `.py` in a package's module directory  | rsync only (the directory is linked as a whole)     |
| Other new or deleted file                  | re-runs that package only (`--packages-select`)    |
| One C++ file                               | rebuilds that package: recompiles the file, relinks (ccache, Release) |
| Header or `CMakeLists.txt`                 | that package and the ones depending on it          |
| Dockerfile / system packages / `panel.sh`  | `pi setup` then `pi reset` (minutes, rare)         |

Sources are bind-mounted into the container; `build/`, `install/` and the ccache live on
the Pi, so nothing is rebuilt from scratch. The first `pi build` compiles everything once;
afterwards `pi build` builds only what the sync changed (`pi build --all` the whole
workspace, `pi build --packages-select <pkg>` exactly that). Builds on the Pi are
`CMAKE_BUILD_TYPE=Release` with `BUILD_TESTING=OFF`: run the tests on the development
machine (`make test`).

`launch` defaults to `hardware:=real imu_transport:=spi imu_spi_device:=/dev/spidev6.0 rviz:=false`
plus the per-robot `config/sandro_real_overrides.yaml`, and the controller defaults to
`whole_body_controller`; extra arguments are appended, e.g.
`deploy/pi/pi launch arm_hardware:=true` or `deploy/pi/pi launch controller:=lqr_balancer`.

## One launch per session

Without `arm_hardware:=true` the launch is the whole session: the controller manager starts
with the IMU active and the drives (`SandroSystem`) configured but inactive, at zero torque
and with nothing written to them, the broadcasters publish, and the controller (with the
other gamepad-switch policy, for an RL policy) is loaded inactive. Arming is
`set_hardware_component_state SandroSystem active` and then activating the controller, from
the panel or by hand (`ros2_ws/src/sandro_bringup/docs/HARDWARE_COMMISSIONING.md`). Teleop
runs in both modes. The gamepad re-arm (Create + Options) re-activates the drives only after
they have been armed once in the running launch; the first arming is the operator's.
`arm_hardware:=true` arms the drives and starts the controller at launch, as before.

The panel reads drive and IMU health at 10 Hz on `/health_broadcaster/dynamic_joint_states`;
`/joint_states` stays at 300 Hz, and the 300 Hz `/dynamic_joint_states` is no longer
published.

## Gamepad (PS5 DualSense over Bluetooth)

Paired and trusted once; afterwards pressing PS reconnects it and `/dev/input/js0` appears
inside the container (it bind-mounts `/dev`). To pair a new pad, hold PS + Create until the
light bar flashes fast, then Scan and Pair on the panel's Gamepad page, or by hand on the Pi
(BlueZ forgets a scanned device once the scan stops, so pair while scanning):

```bash
bluetoothctl --timeout 60 scan on &   # wait until the pad shows up in `bluetoothctl devices`
bluetoothctl pair <MAC> && bluetoothctl trust <MAC> && bluetoothctl connect <MAC>
```

If `bluetoothctl show` reports `PowerState: off-blocked`, the radio is rfkill soft-blocked:
`sudo rfkill unblock bluetooth && bluetoothctl power on` (the state persists across reboots).
The button and axis indices in `config/teleop.yaml` are SDL's gamepad layout, which is what
`joy` reports for the DualSense here: L1 = button 9 is the deadman, L2 + R2 (axes 4 and 5)
the software E-stop.
As always, arming (`arm_hardware:=true`, or arming from the panel) is a deliberate extra
step; see `ros2_ws/src/sandro_bringup/docs/HARDWARE_COMMISSIONING.md`.

## Pi hardware wiring (Raspberry Pi 4)

| Device   | Bus          | Pins                                                    |
|----------|--------------|---------------------------------------------------------|
| BNO085   | SPI6, mode 3 | CS GPIO18, MISO 19, MOSI 20, SCLK 21, INT 25, RST 24; P0 and P1 tied to 3V3 |
| MCP2515  | SPI0, mode 0 | CS GPIO8, MISO 9, MOSI 10, SCLK 11, INT 23; 16 MHz crystal |

* The Pi 4's *auxiliary* SPI1 cannot do SPI mode 3, which the BNO085 requires. SPI6 uses
  the same pins (GPIO18-21) and is a full controller.
* With P0 (the WAKE pin) tied high the BNO085 only accepts a host write while INT is
  asserted. The driver therefore resets the chip at activation and writes its
  configuration before it starts draining packets (`bno085_sensor.cpp`).
* An 8 MHz MCP2515 crystal cannot reach 1 Mbit/s with the kernel driver (it needs
  16 MHz, which is why the module carries one).

`/boot/firmware/config.txt` (written by `pi setup`):

```
dtparam=spi=on
dtoverlay=spi6-1cs
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=23
```

## Tools

* `deploy/pi/tools/bno085_spi_probe.py` is a dependency-light stand-alone BNO085 test
  (run it on the Pi, outside ROS) for wiring and bring-up problems.
* `deploy/pi/pi status` shows container, CAN, SPI and CPU state;
  `deploy/pi/pi can` runs `candump`.
* `deploy/pi/pi sysid` identifies the 9 Hz yaw mode with the WBC balancing: it steps the
  legs' drive stiffness, taps and sweeps the yaw with the WBC's `sysid.*` excitation,
  records everything into `~/sandro/experiments/sysid_<time>` and prints the fit
  (`ros2_ws/src/sandro_bringup/tools/sysid_yaw.py`, `--help` for the knobs). It asks for
  Enter before it starts and stops on Ctrl-C or the first sign the robot is not balancing.

## Latency notes

* The CPU governor is pinned to `performance` (`sandro-perf.service`); `ondemand` adds
  frequency-scaling jitter to a 300 Hz loop.
* The container is privileged with `rtprio=99` and `memlock=-1`, so the controller manager
  can request `SCHED_FIFO` and lock its memory. Check `deploy/pi/pi status` for throttling
  (`get_throttled` should be 0x0).
* On the robot the launch sets the controller manager's `thread_priority: 50`,
  `cpu_affinity: 3` and `lock_memory: true` (`REALTIME` in `sandro.launch.py`): its control
  loop thread runs `SCHED_FIFO` at priority 50 pinned to core 3, and the whole process is
  locked in RAM. The kernel is the stock one; nothing is isolated, so other processes can
  still be scheduled on core 3, below the FIFO thread. Verify on a running launch:

  ```bash
  deploy/pi/pi shell "ps -eLo tid,cls,rtprio,psr,comm | awk 'NR == 1 || /ros2_control/'"
  # the control loop thread: CLS FF, RTPRIO 50, PSR 3; the executor and DDS
  # threads of the same process stay TS (normal) on any core
  deploy/pi/pi shell "grep VmLck /proc/\$(pgrep -f -o ros2_control_node)/status"   # non-zero
  ```

  The launch log must show `Successful set up FIFO RT scheduling policy with priority 50.`
  `Could not enable FIFO RT scheduling policy`, `Unable to set the CPU affinity` or
  `Unable to lock the memory` mean the container lacks the privileges above
  (`deploy/pi/pi reset` recreates it with them).

## Logs and stopping

* ROS logs (`ROS_LOG_DIR`) go to a 64 MB tmpfs at `/var/log/ros` in the container, so they
  are lost at a container restart; files nothing wrote for a day are pruned. `joy_node` logs
  warnings only.
* `gui_runs/panel.log` is truncated into `panel.log.1` once it passes 5 MB (checked every
  minute).
* `docker stop` (and a Pi shutdown) sends the panel a SIGTERM; it Ctrl-Cs the launch and waits
  up to 25 s for it to exit, so the controller manager idles the drives. The container is
  created with `--stop-timeout 30`, so docker does not SIGKILL it sooner.
* These come from the image and the `docker run` flags: after updating, run
  `deploy/pi/pi setup && deploy/pi/pi reset` once.
