# SandroControl

SandroControl is one small control stack for the six GIM6010 actuators. The
same `MitCommand` objects are sent to MuJoCo, physical CAN, or both. A browser
dashboard controls targets and displays telemetry/3D state, but it is never in
the control timing path.

## Architecture

The control thread is the only command authority:

```text
controller.update(state, target, dt)
             |
             v
      Dict[name, MitCommand]
         /              \
  MuJoCo backend      CAN backend
         \              /
          RobotState (SI units)
                   |
          downsampled WebSocket
                   |
              dashboard
```

- Control runs at 300 Hz by default.
- Browser telemetry runs at 30 Hz.
- MuJoCo images render at 30 Hz by default.
- Slow/disconnected browsers do not block the controller.
- Leg drives use MIT position impedance.
- Wheels use the same MIT frame with `kp=0` and feed-forward torque, so the
  bounded MIT position field is irrelevant during continuous rotation.
- All public command and state values are output-shaft SI units: radians,
  radians/second, and Nm.

Leg coordinates are identical in simulation, Cartesian visualization, and
hardware: both upper links are `0 rad` when they point vertically downward;
positive angles open each link away from the leg centre. MuJoCo's exported
joint offsets and directions are derived from its geometry at startup and stay
internal. On hardware, `sign` performs the direction conversion and
`offset_rev` must contain the raw rotor position measured in the zero-down pose.

`runtime.py` owns timing, safety checks, state selection and command routing.
`controller.py` contains the deliberately small replaceable policy.
`hardware_backend.py` contains CAN conversion only. `mujoco_backend.py`
implements the MIT equation directly as joint torque. `dashboard.html` is a
remote target/telemetry client.

## Install and run

Use Python 3.10 or newer:

```bash
cd SandroControl
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

# Safe default: MuJoCo only
python app.py
```

Open <http://127.0.0.1:8322>.

The model path defaults to the existing
`../onshape_export/robot/scene_floating.xml`. The dashboard provides:

- enable, disable and emergency stop;
- actuator-space sliders for all four leg motors;
- per-actuator zero buttons;
- Cartesian `(x, y)` targets for each five-bar leg, including direct dragging
  of either end effector in the linkage view;
- wheel MIT feed-forward torque;
- live MIT gains/limits;
- measured motor position, velocity, torque, age and online state;
- IMU quaternion, gyro and acceleration;
- actual controller frequency, mean/max cycle time and overrun count;
- measured/target five-bar drawings and a rendered MuJoCo 3D view.
- a hanged mode that pins the floating torso above the floor while leaving the
  leg and wheel joints dynamic.

Simulation starts disabled and hanged by default. Press **Enable** before
targets produce torque.

Each leg slider reports three different positions: `goal` is the requested
user target, `MIT` is the slew/torque-limited position field sent this cycle,
and `measured` is the actual encoder/MuJoCo joint position shown by the 3D
model. These are expected to differ while moving or when the torque limit is
active.

Navigate the 3D view with left-drag to orbit, right-drag or Shift-drag to pan,
and the mouse wheel to zoom. **Reset view** restores MuJoCo's default camera.

## Configuration

Edit `config.yaml`; paths are resolved relative to that file.

Important settings:

- `runtime.mode`: `simulation`, `physical`, or `both`.
- `runtime.state_source`: controller feedback source in `both` mode.
- `runtime.control_hz`: command loop frequency.
- `runtime.render_hz`: independent dashboard 3D refresh frequency.
- `MUJOCO_GL` (environment): offscreen GL backend. Left unset, startup prefers
  EGL so frames render on the discrete GPU and falls back to GLFW; the chosen
  backend is reported as `gl=` in the startup log. A software fallback such as
  `osmesa` renders roughly twenty times slower and starves the 3D view.
- `can.channel` and `can.interface`: normally `can0` and `socketcan`.
- `mujoco.hanged` and `hang_height_m`: start with the torso suspended.
- `robot.motors`: node, sign, ratio, zero offset and leg/wheel role.
- `control.*`: gains, slew rates and safety ceilings.
- `imu.port`: normally `/dev/ttyUSB0`.

The supplied CAN mapping follows the requested wiring:

- right inner/front `2`, right outer/back `1`, right wheel `3`;
- left inner/front `5`, left outer/back `4`, left wheel `6`.

The names `inner/front` and `outer/back`, signs, and all zero offsets must be
checked against the assembled robot. CAD joint naming alone cannot establish
physical encoder polarity.

CLI mode overrides the YAML:

```bash
python app.py --mode simulation
python app.py --mode physical
python app.py --mode both
```

In `both` mode each MIT command is routed to both backends and each backend
keeps its own state. Hardware safety requires `runtime.state_source: physical`
in this mode; simulation-state feedback is rejected because it could generate
unsafe torque on a physical pose that has diverged. The two states remain
available for side-by-side telemetry, but this is not a physics-identical
digital-twin guarantee.

## Physical bring-up

Do not begin with the robot supporting its own weight.

1. Lift and mechanically secure the robot. Keep a hardware power cut-off in
   reach.
2. Bring up CAN and verify all six nodes independently.
3. With drives idle, move every joint by hand and check dashboard signs,
   node/name mapping, units and zeros.
4. Store measured rotor-revolution zeros in each `offset_rev`.
5. Confirm the drive-side `mit_max_*` values are valid. They are read at every
   enable and used for command quantisation limits.
6. Start with low `leg_kp`, `leg_torque_limit`, and `wheel_torque_limit`.
7. Test `Disable`, E-stop, CAN removal, stale feedback and process termination
   before ground tests.

The physical backend enables the drive dead-man, reconciles every axis at its
current position with a zero-gain MIT frame, then enters closed loop. Runtime
faults disable all backends on stale/offline state, drive errors, overspeed,
non-finite commands, backend exceptions, or hard torque-limit violations.
Drive firmware limits and a physical E-stop remain required; Python is not a
certified safety controller.

Ctrl+C safes the drives first and releases resources second. The control loop
is stopped and every backend disabled from aiohttp's `on_shutdown`, ahead of
the connection drain, so an open dashboard cannot keep the drives commanded;
the bus, renderer, IMU and recorder are then released from `on_cleanup`. Each
step is bounded and its failures are contained, a second Ctrl+C reaches the
same safe state through `main()`, and a watchdog forces the process to exit
once the drives are idle. Shutdown should never need `kill -9`.

### BNO085

The included reader uses the BNO085 standard **UART-SHTP** transport, not
UART-RVC. The sensor must be strapped for UART-SHTP (PS1 high, PS0 low) and
operates at 3,000,000 baud. Set `imu.enabled: true` after `/dev/ttyUSB0`
permissions are configured.

If the sensor cannot be opened at startup, the runtime logs `continuing
without IMU` and keeps commanding: the IMU is telemetry only, so its absence
shows as an offline IMU panel instead of stopping the control loop. Mid-run
loss behaves the same way.

`imu.report_interval_us` controls the requested orientation, calibrated gyro,
and calibrated acceleration report interval. The default `5000` requests
200 Hz. These three signals are read continuously on an IMU thread and exposed
to the controller in SI units in simulation, physical, and both modes; the
quaternion is normalized to the stack's `(w, x, y, z)` convention.

Orientation comes from the **game rotation vector**, which fuses gyro and
accelerometer without the magnetometer. It has no magnetic settling delay and
never jumps when heading is corrected, at the cost of slow yaw drift. That
trade is the right one for balance control, where latency matters and absolute
heading does not.

The SHTP framing is decoded from bulk reads rather than through a
CircuitPython transport. Reading the port one byte at a time tops out near
19 kB/s, and three reports at 200 Hz produce about 17 kB/s, so a per-byte
reader cannot drain the port and published samples fall seconds behind. The
reader instead decodes every byte currently queued and publishes only the
newest sample from it, so the controller always sees the freshest orientation
rather than a replay of a backlog. `imu.age_s` advances only when a genuinely
new orientation report arrives, so a stalled sensor reads as stale instead of
silently repeating its last value.

Startup enables exactly the three wanted reports, disables every other one the
sensor might still be streaming from a previous run, and then verifies that
each requested report actually reaches the host. The feature response only
acknowledges the command, so without that check a silently ignored report
surfaces much later as a field that never updates. If verification fails the
driver escalates to a sensor reset and retries; if it still fails, `start()`
raises rather than running half-configured. Host-to-sensor bytes are paced at
1 ms because the sensor hub drops a back-to-back burst at 3 Mbps; only
configuration is transmitted, so this does not affect streaming.

On FTDI adapters the driver tries to lower `latency_timer` from its 16 ms
default to 1 ms. Without that, the adapter holds a burst of samples before the
kernel sees them and orientation freshness is capped near 16-33 ms regardless
of baud rate. The write needs privileges a normal session lacks, so make it
stick with a udev rule:

```
# /etc/udev/rules.d/99-ftdi-latency.rules
ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
```

`tools/imu_diag.py` reports what the sensor is really doing. `--mode sniff`
passively decodes the port and prints per-report rates, which distinguishes
"the sensor never sent it" from "the app never read it"; `--mode app` drives
the real reader and prints the published orientation rate and sample age. Both
need exclusive access to the port, so stop `app.py` first.

### IMU mounting frame

`imu.quaternion`, `imu.gyro` and `imu.accel` are all expressed in one frame:
**x forward, y left, z up**, with the origin on the roof plate's top face,
35 mm right of its centre. Mount the physical BNO085 to match, flat on the roof
with its own axes pointing that way; any other mounting needs a fixed rotation
applied to all three signals before they reach the controller.

That frame is what `TwoWheelBalancer` assumes: `pitch_from_quaternion` reads
pitch off the body z axis and takes the pitch rate straight from `gyro[1]`,
which is only the pitch rate when y is the left-pointing axle direction.

The simulated IMU uses the same frame because the `imu` site in
`robot_full.xml` / `robot_full_floating.xml` is placed there explicitly. That
placement is not optional bookkeeping: the Onshape export gives the torso body
a frame inherited from the CAD assembly, whose z axis runs along the wheel
axle, 113 degrees away from anything physical. A site left at `pos="0 0 0"`
with no `quat` therefore sits inside the chassis pointing sideways, and the
simulated IMU cannot be compared to the real one.

Because the site is deliberately not the torso body frame, code that converts
between them has to say so. `MujocoBackend.mirror_state` composes the inverse
site rotation before writing a physical orientation onto the free joint, and
`rl_env.py` derives its forward/axle/up reference axes from `site_xmat` rather
than from the keyframe's free-joint quaternion.

`tools/imu_frame.py` recomputes the pose straight from the roof mesh — z is the
outward normal of the plate's upper face, y is the wheel axle oriented right to
left, x is `y × z` — so the numbers in the MJCF are reproducible rather than
magic:

```bash
python tools/imu_frame.py                     # pose, MJCF snippet, deviation
python tools/imu_frame.py --mode check        # non-zero exit when it drifts
python tools/imu_frame.py --offset 0 -0.035   # try another mount point
```

The mount point lives in the tool's `MOUNT_OFFSET`, as `(forward, left)` metres
from the centre of the top face; `--mode check` validates the model against it,
so move the board in that constant rather than by editing coordinates. The
shift is tangential, so the frame stays flat on the roof wherever it goes, and
the tool refuses an offset that would hang the site off the plate edge. Note
that only `imu.accel` is sensitive to the offset — a sensor off the rotation
centre picks up centrifugal and tangential terms — while the quaternion and
gyro depend purely on the orientation.

Run it after every Onshape re-export; the export overwrites the MJCF and will
drop the site back onto the torso origin. The model also draws the frame as an
RGB triad (`imu_x`, `imu_y`, `imu_z`: red x, green y, blue z), visible in the
dashboard's 3D view and in `mujoco.viewer`. They are sites, so they add no mass
and no collisions; hide them with the site-group toggle in the viewer.

## System identification

The dashboard contains a lifted-robot system-identification panel. These
experiments are intentionally limited to actuator and five-bar dynamics; wheel
contact, body mass/COM, and ground tests should only follow after lifted-model
validation.

Before every physical trial:

1. Mechanically lift and secure the robot with an independent power cut-off in
   reach.
2. Enable the robot and verify that every drive is online and error-free.
3. Select one axis and a bounded preset.
4. Check the lifted confirmation, press **Arm**, review the displayed limit,
   then press **Start**.
5. Use **Abort trial**, Disable, or E-stop at any time. Losing the dashboard
   connection also aborts an active trial.

Manual sliders and gain fields are locked while a trial runs. Excitation is
applied to one axis while leg joints are held at their measured start position
with low impedance and wheels receive damping only. The built-in bounds are:

- wheel torque steps/coast-down: at most `0.2 Nm`;
- leg position multisteps: at most `0.05 rad`;
- leg torque perturbations: at most `0.3 Nm`;
- duration: `2–60 s`.

Start with this sequence:

1. A zero-amplitude recording.
2. Each lifted wheel at `0.1 Nm`, then `0.2 Nm`, including coast-down.
3. Each leg actuator at `0.03 rad`, then `0.05 rad`, while its paired actuator
   is held.
4. Repeat at an unused amplitude as a holdout trial.

The control thread places samples into a bounded non-blocking queue; a separate
writer stores `sysid_runs/sysid_<timestamp>.csv` and matching JSON metadata.
The CSV contains control-rate MIT commands, encoder state, available MIT/Iq
feedback, IMU samples, feedback age, loop timing, and experiment segments. A
nonzero `dropped_rows` invalidates a fit.

Fit one actuator kind at a time:

```bash
python fit_sysid.py sysid_runs/wheel_run_1.csv \
  sysid_runs/wheel_run_2.csv \
  --holdout sysid_runs/wheel_holdout.csv

python fit_sysid.py sysid_runs/leg_run_1.csv \
  --holdout sysid_runs/leg_holdout.csv
```

The fitter replays recorded MIT commands in the hanged MuJoCo model and fits a
small bounded parameter group: damping, Coulomb friction, reflected inertia,
effective torque scale, and command delay. It reports baseline, fitted, and
holdout normalized RMSE.

Results are written to `identified_params.yaml` **disabled by default**. Review
the metrics and parameters before applying:

```bash
python fit_sysid.py sysid_runs/wheel_run_1.csv \
  --holdout sysid_runs/wheel_holdout.csv --enable
```

The override includes a hash of the MJCF and its included XML files.
SandroControl refuses to apply stale parameters after the model changes.
Set `enabled: false` to roll back instantly; generated CAD/MJCF files are never
modified. Simulation also clamps MIT torque to the same leg/wheel limits as
hardware.

## Attaching a controller

A user controller is a normal Python class with two methods. Built-in
controllers live in `controllers/`; controller-facing types come from the
stable `api.py` module. A controller never opens CAN, calls MuJoCo, manages
threads, or communicates with the dashboard.

```python
from api import MitCommand, RobotState, UserTarget

class MyController:
    def __init__(self, cfg: dict):
        self.names = tuple(cfg["robot"]["motors"])
        self.q_hold = {}

    def reset(self, state: RobotState) -> None:
        self.q_hold = {
            name: motor.position for name, motor in state.motors.items()
        }

    def update(self, state: RobotState, target: UserTarget, dt: float):
        return {
            name: MitCommand(position=self.q_hold[name], kp=2.0, kd=0.2)
            if "wheel" not in name
            else MitCommand(kp=0.0, kd=0.1, torque=0.0)
            for name in self.names
        }
```

Save custom classes in `controllers/` (or any importable module) and select one
without changing SandroControl:

```bash
python app.py --mode simulation \
  --controller controllers.example:ExampleController

python app.py --mode both --controller controllers.my_controller:MyController
```

The part before `:` is an importable module; the part after it is the class
name. SandroControl constructs the class as `MyController(cfg)`. For embedding
the runtime in another Python program, injection is also explicit:

```python
controller = MyController(cfg)
runtime = ControlRuntime(cfg, backends, imu, controller=controller)
```

See `controllers/example.py` for a complete low-level controller.

### Controller lifecycle

`reset(state)` is called after every successful operator **Enable**, once all
backends are active and their state has passed safety validation. Initialize
integrators, filters, reference positions, recurrent network state, and timers
there.

`update(state, target, dt)` is called on the single control thread while the
runtime is active:

- `state` is the authoritative backend selected by `runtime.state_source`;
- `target` contains the latest optional dashboard targets;
- `dt` is the configured nominal control period, currently `1 / 300 s`;
- the return value may be a high-level `RobotCommand`, or a low-level mapping
  containing exactly the six configured motor names;
- every value must be finite and the method must not block.

Treat the supplied state as read-only for that cycle. Store persistent
controller data on `self`. Do not sleep, perform file/network I/O, query CAN,
or render from `update()`. Slow work causes control overruns and may eventually
trigger the drive dead-man.

The sys-ID trial controller temporarily takes command authority during an
armed experiment. After completion or abort, the selected user controller is
reset from the current measured state before it resumes.

### State interface

All state values use the logical output-shaft frame, independent of CAN node
sign, gear ratio, or MuJoCo joint direction:

```text
RobotState
  motors[name].position       rad
  motors[name].velocity       rad/s
  motors[name].torque         Nm
  motors[name].iq             measured phase-current feedback, optional
  motors[name].mit_position   decoded MIT feedback, optional
  motors[name].mit_velocity   decoded MIT feedback, optional
  motors[name].mit_torque     decoded MIT feedback, optional
  motors[name].vbus           bus voltage, optional
  motors[name].temperature    degrees C, optional
  motors[name].online         feedback/heartbeat available
  motors[name].error          decoded drive error, empty when healthy
  motors[name].age_s          age of latest encoder sample
  legs["right"|"left"].x/y    measured endpoint position, metres (+y down)
  legs[side].vx/vy            measured endpoint velocity, m/s
  legs[side].reachable        five-bar mapping is valid
  motor_commands[name]        previous cycle's post-mapper MIT command
  leg_commands[side]          previous cycle's post-mapper Cartesian command
  imu.quaternion              (w, x, y, z)
  imu.gyro                    rad/s
  imu.accel                   m/s^2
  imu.online / imu.age_s
  timestamp                   monotonic local time
  source                      "simulation" or "physical"
```

Optional hardware fields are `None` when that feedback stream is unavailable.
In `both` mode the controller always receives physical state; both physical and
simulation states remain available in `runtime.snapshot()` for telemetry and
analysis.

`UserTarget.joint_positions` and `UserTarget.wheel_torques` are merely
dashboard requests. `UserTarget.forward_velocity_m_s` and
`UserTarget.yaw_rate_rad_s` carry backend-independent driving commands for the
driving balancer and future gamepad adapters. A custom autonomous controller
may ignore any of them.

### Command interface

Controllers that operate in Cartesian space should return `RobotCommand`:

```python
from api import LegCommand, RobotCommand, WheelCommand

return RobotCommand(
    legs={
        "right": LegCommand(x=0.045, y=0.16, kp=8.0, kd=0.5),
        "left":  LegCommand(x=0.045, y=0.16, kp=8.0, kd=0.5),
    },
    wheels={
        "right_wheel": WheelCommand(torque=tau, kd=0.05),
        "left_wheel":  WheelCommand(torque=tau, kd=0.05),
    },
)
```

The runtime performs five-bar IK, joint target slew, configured joint/torque
limits, and then normal MIT safety validation. Omitted wheels receive zero
torque; omitted legs hold their measured Cartesian position. An unreachable
endpoint faults safely instead of sending a partial command. Cartesian
coordinates use metres, `+x` from inner to outer pivot, and `+y` downward.

`state.leg_commands` and `state.motor_commands` expose the previous cycle's
post-slew Cartesian position and actual post-mapper MIT fields, respectively.
This lets a controller observe saturation and slew without creating a second
command path.

The low-level escape hatch remains available. Returning
`dict[str, MitCommand]` bypasses Cartesian mapping but not runtime safety.

Each returned `MitCommand` is the direct backend-independent MIT tuple:

```text
torque = kp * (position - measured_position)
       + kd * (velocity - measured_velocity)
       + feed_forward_torque
```

- `position`: output-shaft rad;
- `velocity`: output-shaft rad/s;
- `kp`: Nm/rad;
- `kd`: Nm/(rad/s);
- `torque`: feed-forward Nm.

For a continuous wheel, use `kp=0`; its position field is then irrelevant.
For a leg, position impedance is normally safer than unrestricted
feed-forward torque. The controller is responsible for its own integral
anti-windup, filtering, target slew limiting, and policy-specific bounds.

The runtime still rejects missing/extra motors, NaN/Inf, gains above runtime
ceilings, feed-forward torque above the hard ceiling, estimated physical MIT
torque above the hard ceiling, stale/offline feedback, overspeed, and drive
errors. Any controller exception faults the runtime and disables every
backend. These checks are a final boundary, not a substitute for conservative
limits inside the controller.

### Two-wheel balancer

The built-in station-keeping controller commands both feet to the same
Cartesian height and sends equal logical torque to both wheels:

```bash
# First verify mapping and IMU signs with the base suspended
python app.py --mode simulation \
  --controller controllers.two_wheel_balancer:TwoWheelBalancer

# Ground simulation only after the suspended sign check:
# set mujoco.hanged: false, restart, then Enable.
```

Its initial balance law is deliberately small:

```text
pitch_error = pitch - pitch_target
wheel_torque = pitch_kp * pitch_error + pitch_kd * pitch_rate
```

Pitch comes from the `(w,x,y,z)` IMU quaternion; positive pitch means falling
in logical forward. The same positive logical torque is sent to both wheels so
they drive under the falling body. `torque_sign` exists only for controlled
sign validation—do not change individual wheel signs to tune balance.

The `balance:` section in `config.yaml` controls equal leg height and slew,
leg impedance, pitch target/gains, wheel damping, torque/rate limits, settle
and ramp times, IMU freshness, fall angle, and torque sign. These values can
also be changed live in the dashboard's **Two-wheel balance tuning** panel.
The panel shows measured pitch/rate, unclipped raw torque, final commanded
torque, gain ramp and the active inhibit reason, so saturation and safety
cutoffs are visible rather than inferred. Live changes are not written back to
YAML. The fall latch resets only after Disable/Enable calls `reset()`.

Before physical balancing:

1. Lift the robot and confirm both logical positive wheel torques spin forward.
2. Tilt it by hand and confirm positive pitch produces positive equal torque.
3. Verify stale/disconnected IMU and excessive tilt produce zero wheel torque.
4. Test on the floor with a support tether, power cut-off, and low gains.

This first controller intentionally has no driving, yaw, wheel-position outer
loop, or COM estimator. Add those only after torque-mode station keeping is
stable and system identification has improved the ground simulation.

### Driving balancer

Once `TwoWheelBalancer` is stable, launch the cascaded controller:

```bash
python app.py --mode both \
  --controller controllers.driving_balancer:DrivingBalancer
```

The dashboard then commands forward velocity in m/s and yaw rate in rad/s.
The controller keeps the proven pitch loop as the fast inner loop and adds:

1. a bounded forward-speed PI loop that produces a small pitch target offset;
2. slow wheel-position hold when the requested forward speed returns to zero;
3. a differential yaw-rate torque loop that preserves common balance torque;
4. command acceleration, pitch-offset, yaw-torque and total-torque limits.

Nonzero dashboard commands are refreshed at 10 Hz. The runtime resets them to
zero if updates stop for `drive.command_timeout_s`, the dashboard disconnects,
or the robot is disabled/faulted. This same
`UserTarget.forward_velocity_m_s` / `yaw_rate_rad_s` boundary is intended for a
future PS5 input adapter, so gamepad support will not modify the controller.

Tune the inner pitch controller first and leave it fixed. Then tune
`velocity_kp_pitch`, add only enough `velocity_ki_pitch` to remove persistent
speed bias, enable position hold, and tune yaw last. Drive-loop gains can be
changed live in the dashboard but are not persisted to YAML.

### PS5 controller input

`operator_input.ps5` is a standalone Linux DualSense interface. It is
deliberately **not** imported or started by `app.py`, and it never sends robot
commands. Connecting and polling are explicit, so another component can decide
whether controller data should drive the robot, a recorder, a UI, or something
else.

Connect the PS5 controller over USB or Bluetooth, then verify raw input:

```bash
python tools/ps5_input.py --list
python tools/ps5_input.py
# Select a specific input node when auto-detection finds more than one:
python tools/ps5_input.py --device /dev/input/event12
```

The process needs read access to the selected `/dev/input/event*` node. On
Linux this normally means adding the user to the distribution's `input` group
or installing an appropriate udev rule, then logging in again. Do not run the
whole robot application as root just to gain controller access.

Over both USB and Bluetooth the kernel's `hid_playstation` driver publishes
three nodes for one pad: the gamepad itself, `Motion Sensors`, and `Touchpad`.
Discovery returns only the first, identified by `BTN_SOUTH`, so auto-detection
cannot pick a node that reports no sticks. Confirm the driver bound the pad
with `grep -A5 DualSense /proc/bus/input/devices`; a `Handlers=` line naming an
`event*` node means the pairing itself is fine.

When `--list` finds nothing it also prints the input devices this user can
read, plus any DualSense node the kernel exposes but the user cannot open. That
distinguishes a pad that is not connected from one that is connected but
unreadable, which is the more common case on a fresh machine.

The reusable API is caller-polled and has no hidden thread:

```python
from operator_input import Ps5Controller

pad = Ps5Controller()             # or Ps5Controller("/dev/input/event12")
pad.connect()
try:
    while running:
        state = pad.poll(0.05)
        consume(state)            # your code; SandroControl is not called
finally:
    pad.close()
```

`Ps5Snapshot` reports both sticks normalized to `[-1, 1]`, triggers in
`[0, 1]`, pressed buttons by stable names, a monotonic timestamp, and connection
state. Stick `y` is positive upward. Device removal raises
`ControllerDisconnected`; callers should treat that as a stop request.

For the existing driving balancer, the optional pure mapper uses the left
stick for forward velocity and the right stick for yaw:

```python
from operator_input import snapshot_to_drive_command

request = snapshot_to_drive_command(
    state,
    max_forward_velocity_m_s=cfg["drive"]["max_forward_velocity_m_s"],
    max_yaw_rate_rad_s=cfg["drive"]["max_yaw_rate_rad_s"],
    deadzone=0.08,
)
```

This still does not apply the request. A future adapter can explicitly route
the two fields either to `ControlRuntime.set_drive_target(...)` when embedded,
or to the existing `/ws` `drive_target` message from a separate process.
Refresh nonzero requests faster than `drive.command_timeout_s`; send zero on
shutdown and disconnection. Do not poll HID from a controller's `update()`
method, because device I/O can block the 300 Hz control thread.

### What is intentionally not exposed

There is no independent synchronous `robot.read()` / `robot.write()` API while
the runtime is active. Allowing arbitrary threads to send motor commands would
create two command authorities and break timing and dead-man guarantees.
Controllers read state and write commands only through `update()`. Read-only
external clients may use the dashboard WebSocket snapshots, which are
downsampled and must not be used for feedback control.

For a conventional PID, compute errors from `state.motors[name]` and return
MIT commands. For learned control, construct observations from `RobotState`,
run bounded inference in `update()`, and return the same six-command mapping.

## Tests

```bash
python -m unittest discover -s tests -v
python app.py --mode simulation
```

The first command tests kinematics, config validation and default controller
semantics without opening CAN or MuJoCo.
