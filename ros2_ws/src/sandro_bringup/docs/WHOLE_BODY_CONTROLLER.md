# Whole-body controller

`sandro_controllers/WholeBodyController` is the launch's default controller
and the leg-exploiting alternative to the wheel-only balancers. The modelling record and the reasoning behind each
control choice are in `MODELLING_AND_CONTROL.md`; this page is the operating
manual. It claims the same six effort commands, joint states
and IMU states, and listens to the same `/cmd_vel` `TwistStamped`, so it is a
drop-in swap:

```bash
make sim                          # MuJoCo with viewer
ros2 launch sandro_bringup sandro.launch.py \
  hardware:=mujoco controller:=whole_body_controller headless:=true rviz:=false
ros2 control load_controller --set-state inactive driving_balancer
ros2 control switch_controllers \
  --deactivate whole_body_controller --activate driving_balancer
```

`linear.x` is forward speed, `angular.z` yaw rate, and additionally `linear.z`
is a body height rate in m/s. With the PS5 teleop that is the right stick
vertical (up stands taller, 0.05 m/s at full deflection, L1 held); the
wheel-only balancers ignore it. Pressing L2 and R2 together is the gamepad
E-stop: every command-owning controller is deactivated (all efforts zero, the
robot drops) and on hardware the CAN E-stop fires too. Re-arm while holding the
robot upright: in simulation `ros2 control switch_controllers --activate
whole_body_controller`; on hardware relaunch with `arm_hardware:=true` (the
drives latch `ESTOP_REQUESTED`, which arming clears; see
`HARDWARE_COMMISSIONING.md`). Only one balancer may be active.

## What the legs buy

The wheel-only cascade holds the legs at a fixed height and balances with wheel
torque alone. This controller uses the legs for three things:

1. **Balance.** The fore-aft position of the wheels under the body is a second
   balance actuator. Shifting the wheels 10 mm under a falling body corrects the
   gravity lever as much as leaning the body by 10 mm / CoM height, and it does
   it at leg speed without first accelerating 3.4 kg with the wheels. One LQR
   coordinates both actuators (see below); in simulation the legs shift the
   wheels about 12 mm forward while braking from 0.3 m/s.
2. **Speed and posture.** With the legs able to place the axle under the centre
   of mass, the body no longer has to lean to compensate a CoM that is not
   where the CAD says, or a slope. A slow posture trim measures the gravity
   lever the body is balancing (`CoM height * pitch + leg offset`), filters it
   while the robot is at rest with no steady wheel torque (a lean held
   against wheel stiction is friction, not mass), and moves it into the leg
   offset reference so the robot stands level. It starts from zero at activation; an identified
   centre of mass belongs in `tasks.fore_aft.neutral_offset_m`.
3. **Turns.** Lateral acceleration `v * yaw_rate` tips the robot outward. The
   roll task leans the body into the turn by a differential leg height
   (`track * tan(atan(v * yaw_rate / g))`), and the same task levels the body
   on laterally uneven ground from the IMU roll. Height itself is a task too,
   so the robot can crouch for speed or stand up for reach while balancing.

## Structure

```
sandro_controllers/
  lqr.{hpp,cpp}              matrix exponential, ZOH discretisation, DARE (SDA)
  whole_body_model.{hpp,cpp} reduced planar model, linearisation, attitude, leg FK
  whole_body_core.{hpp,cpp}  estimate -> references -> LQR -> tasks -> safety
  whole_body_controller.*    ros2_control plugin: parameters, RT buffers, diagnostics
  test/test_whole_body.cpp   solver, model and closed-loop tests on a planar plant
```

The core is free of ROS so every task and safety path is unit-tested against a
planar wheeled-inverted-pendulum plant with leg lag and an unmodelled CoM
offset.

### Model

The robot reduces to a body of mass `M` (everything the legs carry) on wheels
of radius `r` (mass `m_w` and axle inertia `I_w` each), with the body's centre of mass `L = k * leg_height` above the
axle and shifted forward by `d` (the leg offset, positive when the wheels are
placed backward). With total wheel torque `tau` acting between wheels and body:

```
a x_dd + b pitch_dd = tau / r                 a = M + 2 m_w + 2 I_w / r^2
b x_dd + c pitch_dd = M g (L pitch + d) - tau  b = M L,  c = I + M L^2
```

`d` enters exactly like `pitch` scaled by `1/L`; that identity is the whole
reason wheel placement works as a balance actuator. The legs are not
instantaneous: the achieved `d` follows the commanded `d_c` with the lag
`model.leg_lag_s` (70 ms on the robot). State is
`[x, x_dot, pitch, pitch_rate, d_c, d]`, inputs are `[tau, d_c_dot]`, and the
Riccati equation is solved at several leg heights (`balance.gain_table`)
because `L` and the pendulum's unstable pole (about 12 rad/s here) change with
height. The
gains are interpolated on the measured leg height at run time and re-solved in
the parameter callback whenever a model or weight changes, never in the
realtime loop.

Model defaults are the CAD values from the MJCF (`sandro_description` README
lists the joint convention and the derivation tools): `M = 3.284 kg`, CoM
`0.654 * leg height` above the axle, pitch inertia `0.0295 kg m^2` at
`0.1855 m`. The five-bar planar model reproduces the MJCF axle position to
0.1 mm once the joint convention is applied.

### Tasks and allocation

- **Balance LQR** gives total wheel torque and the leg offset rate. The rate is
  integrated into a leg offset command, bounded by `balance.leg_offset_limit_m`
  and the workspace check. The achieved offset it feeds back is the leg-lag
  model's prediction by default; `balance.measured_offset_blend` moves it
  toward the measured axle position (see *Tuning*). Every parameter set is checked for its
  input-delay margin at configure time (`balance.required_delay_margin_s`) and
  the margin is printed with the gains.
- **Legs** hold a Cartesian reference per side (fore-aft placement from the
  balance LQR, height from the height task). By default
  (`tasks.legs.pd_in_drive` 1) the PD runs inside the GIM6010 drives: each leg
  joint receives its inverse-kinematics reference with the joint gains
  `tasks.legs.drive_kp_nm_rad` / `drive_kd_nm_s_rad` in the MIT frame and the
  controller only sends the gravity support (`J^T [0, M g / 2]`, plus the
  optional reaction feed-forward) as torque. The drive closes the loop in its
  own cycle, synchronous with the frame and free of the bus and sensor
  latency, which is what lets the legs be stiff (kp 12 Nm/rad is about
  1800 N/m at the axle). The policy limit is kept by pulling the reference
  toward the measured position when the torque the drive will compute from
  the latest state would exceed `limits.leg_torque_nm`. With
  `pd_in_drive` 0 the controller computes a Cartesian impedance itself,
  `F = Kp (p_ref - p) - Kd p_dot_filtered`, on a low-passed velocity
  (`tasks.leg_velocity_filter_hz`); gravity support is protected from
  saturation either way. `tasks.leg_reaction_{ground,stator}_gain` optionally
  feed forward the wheel torque's reaction on the leg (off by default).
- **Yaw** is model based: `I_z psi_dd = (track / 2 r) dtau - scrub`. A
  proportional term on the wheel-differential yaw rate (`tasks.yaw.
  wheel_rate_blend` 1; 0 uses the IMU) damps the loop at about 7 rad/s with
  `kp_nm_s_rad` 0.3, an integral on the IMU yaw rate makes the body turn at the
  commanded rate against tyre scrub and drive friction, and the Coulomb-like
  scrub (`scrub_ff_nm`) and the yaw inertia (`model.yaw_inertia_kgm2`) are fed
  forward from the yaw command, which follows the operator through a
  first-order lag (`drive.yaw_command_time_constant_s`). The differential
  torque is allocated after balance inside the per-wheel torque budget.
- **Roll** commands the left-minus-right leg height difference from the
  lean-into-turn feedforward (speed times the lagged yaw command) plus PI on
  IMU roll.
- **Height** slews `tasks.height.target_m` plus the integrated `linear.z`
  command between `min_m` and `max_m`.

### Safety

- Command timeout zeros the motion command; balance continues.
- IMU stale (non-finite values from the hardware plugin) stops the wheels and
  keeps the legs holding the stance; a stale IMU never launches the robot.
- Balancing engages only when the body is upright and still
  (`balance.engage_*`) after `safety.settle_s`. Before that the legs hold the
  stance and the wheels are off, so on hardware you activate, lift the robot
  upright, and it starts by itself. References re-anchor at engagement so the
  impedance never steps.
- A fall while balancing (`|pitch| > safety.fall_angle_rad`) latches every
  actuator off until the controller is re-activated. Not being upright before
  engagement is the normal starting condition and does not latch.
- Torque, torque-rate, workspace, leg-offset, height, roll and position-error
  limits are enforced and reported in `~/diagnostics` (`status` message and
  `status_mask`).

## Simulation

The simulation emulates the physical sensor and command latency by default
(`sandro_sim/LatencySystem`; `sim_latency:=false` for the ideal backend, and
`sim_imu_delay_s`, `sim_joint_delay_s`, `sim_command_delay_s`, the rates and
`sim_joint_stagger` as launch arguments) and runs the drives' MIT law on every
physics step (`sandro_sim/MitDrivePlugin`). Parameters that only balance the
ideal simulation will not balance the robot; see `SIM_TO_REAL.md`. For
sweeps, `sandro_sim/tools/closed_loop.py` runs the same controller and
emulation without ROS, about 50 times faster than real time.

Physics runs from the moment the controller manager starts, about a second
before the balancer is active, and unpowered legs collapse in that time. The
launch therefore runs `sandro_bringup/sim_reset` once the controller spawner
has finished (`sim_reset:=true`, MuJoCo only): it deactivates the balancer,
resets the world to the stand keyframe and activates the balancer again from
one node, so it engages on a standing robot with fresh references instead of
the ones it anchored on the collapsed pose. A scripted drive that prints the
controller's diagnostics as a table lives in `sandro_bringup/tools/drive_test.py`
(from the repository root inside the container):

```bash
python3 ros2_ws/src/sandro_bringup/tools/drive_test.py --controller whole_body_controller
```

Its default sequence stands, drives, stops, turns while driving, spins in
place, crouches and stands up; the run must end with `fall latch no`. Recorded
behaviour with the shipped parameters and the per-robot override in the
latency-emulating identified simulation
(`recordings/sim_wbc2_latency_2026-09-20.csv`, sequence of `SIM_TO_REAL.md`
plus a 1 rad/s spin and a 0.4 m/s drive): standing pitch RMS 0.015–0.020 rad,
0.19–0.20 m/s for a 0.2 m/s command and 0.40 for 0.4 m/s without overshoot,
0.45 rad/s of a 0.6 rad/s turn while driving (the simulated tyre scrub is
about twice the robot's), 0.67 rad/s of a 1 rad/s spin, height within about
1 cm at the extremes (joint friction and the lower-leg mass that the axle
force does not carry account for the residual). The same set balances the
ideal simulation (`sim_wbc2_ideal_2026-09-20.csv`).

## Tuning

Everything is a live ROS parameter, validated as a complete set; a rejected set
reports why. In order:

1. `model.*`: weigh the robot, set `body_mass_kg`. Check `trim_m` and
   `pitch_reference_rad` in diagnostics while standing: a converged trim is how
   far the real centre of mass sits behind the model's neutral axle position.
   Move the neutral there with `tasks.fore_aft.neutral_offset_m` (negative for
   backward) so the legs keep their full offset range; a rearward shift needs
   `tasks.height.max_m` lowered (0.185 m for the robot's 8 mm) to stay clear
   of the front pivot's stretched singularity. Per-robot values live in
   `config/sandro_real_overrides.yaml` (`SIM_TO_REAL.md`).
2. `balance.weights` / `balance.effort_weights`: Bryson's rule in SI units. The
   log line at configure prints the resulting gains and the input-delay
   margin; a set below `balance.required_delay_margin_s` (12 ms) is rejected.
   Keep the per-wheel velocity gain (`gain_wheel_velocity_nm_s_m / 2 *
   wheel_radius`) well below `wheel_inertia_kgm2 * update_rate` (about 0.1 Nm
   per rad/s): above it the wheels chatter against tyre or gearbox compliance
   at the sampling rate, which is what the first weight set did in simulation.
   `effort_weights.leg_offset_rate` decides how much the legs help; lower it
   for more leg action. Position and velocity 30 / 4 track speed against the
   drive friction without overshoot; 10 / 1 (the first set) is calmer at the
   wheels if the robot's stick-slip at standstill is a nuisance, 60 / 8 hunted
   against Coulomb friction in simulation.
3. `model.leg_lag_s`: the time constant with which the measured axle follows
   its command. Read it from a recording as the lag at which
   `leg_offset_command_m` and `leg_offset_m` correlate best (70 ms on the
   robot, run 4); re-identify after changing the leg impedance, since the
   overdamped leg responds in about `kd / kp`. With it in the model the LQR
   knows what the legs can deliver.
   `balance.measured_offset_blend`: 0 (shipped) feeds the LQR the lag
   model's prediction of the achieved offset, 1 the measured axle position.
   On the robot the measured path, through the lag-blind design, made both
   legs hunt in phase at 1.9 Hz and 0 stopped it outright (`SIM_TO_REAL.md`,
   sixth comparison). With the lag model the gain on the measurement is a
   third of before and in simulation 1 stands calmer (pitch RMS 0.025
   against 0.030); try 1 on the robot once standing is clean with 0, and go
   back if the legs start swinging in phase.
4. `tasks.legs.drive_kp_nm_rad` / `drive_kd_nm_s_rad`: the joint gains the
   drives apply (MIT frame, kp up to 500, kd up to 5). 12 / 0.5 are
   SandroControl's leg gains, which balance the robot with the PID cascade,
   and give the axle about 1800 N/m; the drive's response time is about
   `kd / kp` plus stiction, 72 ms in simulation and 70 ms on the robot, which
   is `model.leg_lag_s`. Faster legs need less damping per stiffness: 19 / 0.5
   gives 48 ms and 20 / 0.3 gives 24 ms in simulation, both stable in the
   latency emulation (`recordings/sim_wbc4_drivepd_*`); set `model.leg_lag_s`
   to the identified value after any change. Watch the leg efforts in a
   recording for sample-rate chatter when lowering `kd`. The controller-side
   impedance (`tasks.height.kp_n_m` etc., `pd_in_drive` 0) is bounded by the
   bus and sensor delay to about 400 N/m and is kept for comparison only.
   `tasks.height.ki_n_m_s` learns the weight share the feed-forward misses
   (`height_support_n` in diagnostics) in both modes.
5. `tasks.leg_reaction_ground_gain` / `stator_gain` (off): feed forward the
   wheel torque's reaction on the leg, `J^T [-tau / r, 0] + tau d(link)/d(q)`,
   through `leg_reaction_filter_hz`. The ground term assumes the torque reaches
   the ground; below the drive friction (0.2 Nm per wheel) it does not, yet the
   term still shifts the wheels under the body and tips it the way the torque
   did, which fell in the latency simulation with 400 N/m legs. Only for
   experiments on the robot with stiff legs and an identified wheel friction.
6. `tasks.yaw.*` and `tasks.roll.*` once driving is clean. `kp_nm_s_rad` follows
   from the wanted bandwidth, `I_z * w / (track / 2 r)`; do not raise it to get
   the commanded rate, that is what `scrub_ff_nm` and `ki_nm_rad` are for. The
   1.0–1.5 of the first runs rang at 3.6 Hz with ±0.8 Nm while the robot stood
   still.

## Hardware

Do not activate on the physical robot before:

- the joint zero is established, either as the drive's persistent user zero
  or as `offset_rev`, with each upper link hanging straight down, the standing
  pose reads about 0.41 / 0.22 rad per leg like the simulation, and both wheels
  read forward-positive (`HARDWARE_COMMISSIONING.md` step 5);
- the IMU frame is x forward, y left, z up;
- `balance.wheel_torque_sign`, `tasks.yaw.torque_sign` and
  `model.{right,left}.forward_sign` have been verified with the robot lifted
  (positive planar leg x is body-forward on the right and body-backward on the
  left because the inner/outer motor names are mirrored); with a yaw command
  the wheel-differential rate in diagnostics (`wheel_yaw_rate_rad_s`) and the
  IMU rate (`yaw_rate_rad_s`) must carry the same sign;
- `model.body_mass_kg` is the weighed value;
- the drives report `mit_max_kp` >= 12 and `mit_max_kd` >= 0.5 at activation
  (the plugin logs the limits); the controller's drive gains are clipped to
  them.

Then, with the robot tethered: activate, hold it upright, and it engages when
`|pitch| < balance.engage_pitch_rad`, the pitch rate is below
`balance.engage_pitch_rate_rad_s`, and both leg readings evaluate inside the
five-bar workspace (a wrapped or mis-zeroed encoder keeps the wheels off and
reports `leg_kinematics` in the diagnostics). The legs droop by about `M g / (2 kp)`
(2.7 cm at 600 N/m) when the robot is lifted because the gravity support is fed
forward; that is expected.

## Path to constrained inverse dynamics

The plugin, parameters, estimator and task definitions stay; a full rigid-body
QP would replace the LQR and `J^T` allocation in `whole_body_core.cpp` with a
constrained solve over generalised accelerations, six torques and two contact
wrenches. It needs identified inertias and a closed-chain model of the
five-bars (the MJCF loop constraints are the reference), plus a QP solver in
the image; the MJCF-derived parameters above are the starting point for that
identification.
