# Closing the sim-to-real gap

The physical robot balances and drives on the whole-body controller but shows
more oscillation and slower turning than the simulation did. This page is the
workflow and the tooling for making the simulation predict the robot, so that
tuning done in MuJoCo transfers. Model background is in
`MODELLING_AND_CONTROL.md`.

## 1. What the simulation was missing

| Effect | Reality | Simulation before | Now |
| --- | --- | --- | --- |
| Sensor latency | BNO085 reports at 200 Hz plus fusion lag; drive encoder broadcasts every 5 ms, the six drives unsynchronised | fresh every 3.3 ms, zero delay | `sandro_sim/LatencySystem` decorator: each interface is sampled on its own 200 Hz clock (joint clocks staggered) and delivered after a transport delay |
| Command latency | a frame is on the wire about 1 ms after the write | applied at the next physics step | `sandro_sim/MitDrivePlugin` holds each frame for `sim_command_delay_s` at physics-step resolution |
| Drive-side MIT loop | the drive computes `kp (q_ref - q) + kd (v_ref - v) + tau` in its own cycle from its own encoder | backend takes torque only | `sandro_sim/MitDrivePlugin` runs the law before every physics step on the current joint state |
| Control rate | 300 Hz | 250 Hz: the controller manager sleeps on `/clock`, which only ticks every 2 ms physics step | physics step 1/1200 s (`sync_model.py`), four steps per control cycle |
| Tyre scrub | rubber tyre resists twisting about the contact normal | condim 3, no torsional friction | condim 4, torsional friction 0.004 m, 10 ms contact |
| CoM / mass | robot not weighed; CAD only | CAD | `payload` in `config/sim_identified.yaml` |
| Drive friction, torque constant | unmeasured | legacy fit / CAD | `joints`, `actuators` in the same file |

The two latencies turned out to be the whole story of the oscillation. With
the first emulation (until 2026-10-16: IMU 10 ms at 200 Hz, encoders 5 ms at
200 Hz, one cycle of command delay, a 250 Hz controller), the parameters that balanced the ideal simulation fell within
two seconds: the leg Cartesian impedance (fore-aft 1000/40, height 600/40)
oscillated at the sample rate, and the balance LQR had a loop-delay margin of
exactly 3 samples (10 ms). A sweep isolated the encoder-plus-command path as
the critical one; 10 ms on the IMU alone was tolerated.

The parameters changed accordingly (fore-aft 300/12, later 400/20, height
400/20 with a support integral, wheel torque weight 2.0 giving 17 ms of
margin); the latency-emulating simulation balanced, drove, turned and spun
with them. The leg PD has since moved into the drives (seventh comparison).

### Timing of the emulation (2026-10-16)

The first emulation was harsher than the robot in four ways, all of which
the numbers above were obtained with:

- The controller ran at 250 Hz. mujoco_ros2_control publishes `/clock` once
  per physics step and the controller manager sleeps until `now + 1/300 s`,
  so with MuJoCo's default 2 ms step it woke every 4 ms. The LQR, discretised
  for 3.33 ms, was running on a slower plant than it was designed for.
- The MIT law ran once per control cycle on 3.3 ms old state and its torque
  was held for the whole cycle: a sampled PD at 300 Hz, against the drive's
  continuous one. It now runs in a mujoco_ros2_control plugin before every
  physics step.
- "Delay" and "rate" compounded. A 200 Hz channel read by a 300 Hz loop
  refreshed only every other cycle (150 Hz), and on top of the hold the value
  was the 5 ms delay old, so the encoders were 5 to 13 ms old instead of the
  robot's 1 to 6 ms. The rate now sets a sample clock and the delay is the
  transport latency from the sample to the controller; with the default
  1 ms the encoder data is 1 to 6 ms old, as on the robot.
- The command delay was rounded up to whole control cycles (3.3 ms) and is
  now applied per physics step (1 ms by default, about one 3-frame CAN
  burst plus the gs_usb round trip).

The IMU stays at 10 ms / 200 Hz until the SPI path on the Raspberry Pi
(`imu_transport:=spi`, the launch default) is measured with
`compare_runs.py imu_update_hz`; the UART path delivered only 30 Hz (`sim_imu_rate_hz:=30`
reproduces it, eighth comparison). Recheck the shipped gains against these settings before relying on
the margins quoted above: the loop is less delayed than the one they were
tuned for. `sim_joint_stagger:=false` puts all drives on one clock, as
`state_source:=mit` does.

## 2. Measuring the gap

Run the same scripted drive on both backends and record everything:

```bash
# simulation (latency emulation on by default)
ros2 launch sandro_bringup sandro.launch.py hardware:=mujoco controller:=whole_body_controller \
  headless:=true rviz:=false teleop:=false
python3 ros2_ws/src/sandro_bringup/tools/drive_test.py --record sim.csv

# hardware, robot tethered, gamepad E-stop in reach (L2+R2)
ros2 launch sandro_bringup sandro.launch.py hardware:=real arm_hardware:=true \
  controller:=whole_body_controller teleop:=true
python3 ros2_ws/src/sandro_bringup/tools/drive_test.py --record real.csv

python3 ros2_ws/src/sandro_bringup/tools/compare_runs.py sim.csv real.csv --label sim --label real
```

`drive_test.py --settle 3` waits three seconds after engagement before phase
0 so the operator's release (or the simulation reset) stays out of the
comparison. `compare_runs.py` prints per command phase: pitch RMS and peak-to-peak,
dominant pitch frequency, speed and yaw-rate tracking error, yaw rise time,
wheel torque RMS, wheel velocity sign-flip fraction (0.5 is chatter at the
sample rate), leg effort RMS and height error. Two runs agree when these are
within about 20 % of each other in every phase. Use a short, gentle sequence
first on hardware, for example `--phase 5,0,0 --phase 4,0.2,0 --phase 4,0,0
--phase 4,0.2,0.6 --phase 3,0,0`.

### First comparison (2026-09-20)

`recordings/real_2026-09-20.csv` (robot, first WBC parameter set) against the
same sequence in simulation gave, before any identification:

| Finding | Evidence | Applied |
| --- | --- | --- |
| Legs carry about 1.2 kg more than CAD | learnt support 16.4 N per leg vs 10.4 N in simulation | `model.body_mass_kg: 4.5` (override); 1.2 kg payload body in `sim_identified.yaml` |
| Centre of mass about 28 mm behind the model's neutral axle (superseded: this was the lean held against wheel stiction, see the fourth comparison; the mass is 7 to 8 mm behind) | `trim_m` toward +0.02, leg offset pinned at its +0.025 limit, body leaning +0.02..0.045 rad; speed reached 0.06 of 0.2 m/s with no leg range left | `tasks.fore_aft.neutral_offset_m: -0.028` and `height.max_m: 0.175` (override); payload 10.7 cm behind the CoM |
| Yaw torque-limited, tyres do not skid | 0.37 Nm average, hitting the 1 Nm limit, for 0.22 rad/s of 0.6; wheel-differential yaw 0.225 equals gyro 0.215 | `tasks.yaw.torque_limit_nm: 2.0` (override); tyre torsional friction 0.07 in simulation |
| 2 Hz pitch oscillation while standing | pitch RMS 0.058 rad, 1 Nm torque RMS in the first phase, decaying as the trim converged | mass and CoM above; trim rate 1/s |

With the payload and the override applied to the simulation, the standing,
driving and stopping phases agree with the robot within about 20 % (pitch RMS
0.027 vs 0.027 rad standing, speed error 0.21 vs 0.21 m/s driving, leg efforts
0.75–0.90 vs 0.75–0.82 Nm, wheel torque 0.30 vs 0.35 Nm) and the turn costs
1.9 vs 1.7 Nm per rad/s of yaw. The remaining differences (the robot's release
transient in phase 0, its saturated legs while driving, its yaw drift after
the turn) are the effects the override removes, so the next robot recording
must use it:

```bash
ros2 launch sandro_bringup sandro.launch.py hardware:=real arm_hardware:=true \
  controller:=whole_body_controller teleop:=true \
  extra_params:=$(ros2 pkg prefix sandro_bringup)/share/sandro_bringup/config/sandro_real_overrides.yaml
python3 ros2_ws/src/sandro_bringup/tools/drive_test.py --record real2.csv \
  --phase 5,0,0 --phase 4,0.2,0 --phase 4,0,0 --phase 4,0.2,0.6 --phase 3,0,0
```

Expect `trim_m` to settle near +0.01 rather than 0: shifting the axle also
drags the leg masses, about 0.7 mm of effective lever per mm of shift. Add
the settled trim to `neutral_offset_m` once more (`neutral_offset_m -= trim`),
weigh the robot, and put the real mass into both files.

A payload in `sim_identified.yaml` is generated as a child body with its own
inertial element; a geom mass on the torso would be ignored, because the
torso carries an explicit inertial.

### Second comparison (2026-09-20, with the override)

`recordings/real2_2026-09-20.csv`. The turn now matches the simulation (yaw
error 0.21 vs 0.54 before), the first standing phase is calmer (pitch RMS
0.029 vs 0.058), and three new observations were explained by the data:

| Observation | Evidence | Applied |
| --- | --- | --- |
| Creeps forward after a drive command | reached 0.056 of 0.2 m/s, so the position reference ran ahead and clamped at 0.3 m; afterwards station keeping pulled the robot toward it (+0.4 Nm mean torque, position error pinned) | the position hold now anchors where the stick is released (shipped; the 0.3 m limit stays as the speed loop's integral action while driving); wheel Coulomb friction feed-forward `wheel_friction_ff_nm: 0.12` in the direction of the commanded torque (override) since 0.43 Nm of torque produced almost no acceleration, about 0.2 Nm of friction per drive train; simulated wheel `frictionloss` 0.2. The creep itself is a stalemate reproduced in `test_whole_body.cpp`: legs pinned by the centre-of-mass offset, rolling resistance, a forward lean and a clamped position reference, so the leg range (neutral offset) is the real fix |
| Right leg lower | leg kinematics report the right leg 14 mm shorter in both runs while the IMU and its gravity vector read the body level within 0.3°: on a flat floor that is a 3° IMU roll mounting bias that the roll integral levelled | `tasks.roll.imu_offset_rad: -0.049` (override) and the same bias on the simulated IMU (`imu_mount` in `sim_identified.yaml`) |
| Less stable later in the run | position loop saturated and the trim wandered from -0.021 to +0.023 while creeping; wheel torque RMS doubled in phases 2 to 4 | follows from the creep fix; the trim now learns only while the robot is at rest (command and measured speed inside the deadband) |

The hardware plugin gained `state_source:=mit` (with `encoder_period_ms:=20`):
joint velocity and leg position from the MIT reply each drive sends to every
command, one cycle old at most and synchronous with the loop, instead of the
5 ms encoder broadcast that the loop beats against. It is untested on the
robot as of this writing; try it once the baseline above is repeatable, and
if it helps set `sim_joint_delay_s:=0.0023 sim_joint_rate_hz:=300
sim_joint_stagger:=false` for the simulation to match (the reply is sampled
when the previous cycle's command arrives and read one cycle later).

### Third comparison (2026-09-20, identified simulation with the override)

Running the identified simulation with the second override exposed a
controller defect rather than a model one: the robot ran away backward at
0.9 m/s within a second of engaging and fell. Recording from the first
diagnostics message (`drive_test.py --record-from-start`) showed why. The
trim had been seeded at engagement from the measured gravity lever
`CoM height * pitch + leg offset`, and at engagement the legs stand wherever
the impedance or the reset pose left them (26 mm from the shifted neutral on
the stand keyframe), not under the centre of mass. That placement was read as
a 30 mm mass offset, a 1.3 Nm gravity lever the wheels cannot hold. Fixed by
starting the trim from zero (the identified mass belongs in the neutral
offset) and letting it learn only while the robot is at rest, so the lean
used to accelerate or brake is never learnt as a mass offset. Two more
findings from the same recordings:

- The controller engaged on the collapsed robot half a second before the
  world was reset, so its position hold and trim were anchored to a pose that
  no longer existed. The launch now runs `sandro_bringup/sim_reset`, which
  deactivates the balancer, resets the world and activates it again from one
  node (tens of milliseconds unpowered on the keyframe).
- A faster trim (2 /s) followed the transients and sustained a slow
  position oscillation between the leg offset limits; 1 /s stays.

The neutral was moved to -0.040 m at this point because the trim had
settled at +0.02 again with -0.028. That reading was wrong (the trim had
learnt a friction lean, see the fourth comparison); the value now in the
override is -0.008.

`recordings/sim_identified_2026-09-20.csv` against `real2`:

| Phase | Metric | real2 | identified sim |
| --- | --- | --- | --- |
| standing (0, 2, 4) | pitch RMS, rad | 0.029 / 0.032 / 0.024 | 0.031 / 0.030 / 0.037 |
| driving 0.2 m/s (1) | pitch RMS, rad | 0.022 | 0.011 |
| turn 0.6 rad/s (3) | yaw-rate error RMS | 0.21 | 0.19 |
| all | wheel torque RMS, Nm | 0.32 to 0.62 | 0.23 to 0.39 |
| all | wheel velocity sign flips | 0.02 to 0.08 | 0.00 to 0.01 |

Standing and turning agree within the 20 % target. The robot still spends
about 1.7 times the wheel torque and reverses its wheels far more often: more
drive-train friction and backlash than `frictionloss` 0.2 models, which the
next identification step (2. Latency, 4. Wheel drive) should settle. The
simulation's own first two seconds are not comparable: the stand keyframe is
not balanced for the identified mass, so the controller drives 0.3 m before
settling; `--settle 3` keeps that out of phase 0, and a release from an
operator's hands does not have it.

Next robot recording, identical to the second but with this build:

```bash
ros2 launch sandro_bringup sandro.launch.py hardware:=real arm_hardware:=true \
  controller:=whole_body_controller teleop:=true \
  extra_params:=$(ros2 pkg prefix sandro_bringup)/share/sandro_bringup/config/sandro_real_overrides.yaml
python3 ros2_ws/src/sandro_bringup/tools/drive_test.py --record real3.csv --settle 3 \
  --phase 5,0,0 --phase 4,0.2,0 --phase 4,0,0 --phase 4,0.2,0.6 --phase 3,0,0
```

Expect: both legs the same height, `trim_m` within about 0.01 of zero and the
leg offset away from its limits while standing, speed reaching 0.2 m/s in
phase 1, and no motion after the phase ends.

### Fourth comparison (2026-09-20, third hardware run)

`recordings/real3_2026-09-20.csv`. Standing was the calmest so far (pitch
RMS 0.010 rad, wheel torque RMS 0.12 Nm, no creep), but as soon as the robot
drove the whole balance loop hunted at 2 Hz: the leg offset command swung
between its +-25 mm limits, the right leg followed with a 7 cm peak-to-peak
fore-aft swing (std 20 mm against 5 to 9 mm on the left), pitch swung +-8
degrees and the yaw rate +-2 rad/s, with the yaw PI following at +-1.4 Nm.
The operator saw it as the right leg oscillating.

The standing data explains it. With `neutral_offset_m: -0.040` the leg
offset command sat pinned at -0.025 for the whole standing phase, the trim
pinned at its -0.030 limit, and the excess became a pitch reference of
-0.045 rad (body leaning back 2.6 degrees). The balance loop had leg
authority on one side only and was operating with both legs 40 mm off the
pivot centre, where the front links approach their stretched singularity
(the reason `height.max_m` had to drop to 0.17). The neutral itself was
wrong: in the calm standing phase of all three runs the axle stood at
body-forward 0.033 to 0.040 m with the body within a degree or two of level,
which puts the centre of mass at 0.037 to 0.038, i.e. 7 to 8 mm behind the
CAD neutral, not 28 or 40. The earlier values came from `trim_m`, which had
learnt the forward lean the robot held against wheel stiction while it crept
(0.4 Nm of wheel torque and no motion looks like a mass offset to a trim
that only checks speed). Applied:

- `tasks.fore_aft.neutral_offset_m: -0.008`, `height.max_m: 0.185`
  (override); the payload in `sim_identified.yaml` moved to 31 mm behind the
  IMU so the simulated mass is also 8 mm behind the neutral.
- New `balance.trim_torque_deadband_nm: 0.15` (shipped): the trim adapts only
  while the balance torque is inside it as well; balanced at rest the wheel
  torque averages near zero, a lean against stiction does not.
- Roll `kp_m_rad 0.1, ki_m_rad_s 0.2` (shipped, was 0.05/0.1): the roll loop
  had not levelled the 3 degree IMU bias within the standing phase.

One more asymmetry in the data: the left inner drive's reported velocity
was 0.46 of its own position derivative in this run, against 0.84 to 0.87
for the other five drives (and 0.87 for it in the second run). Its lag was
normal. Check that drive's velocity estimator or encoder configuration
before the next run; the left leg's damping depends on it, and it may be
part of why the two legs responded so differently.

`recordings/sim_identified2_2026-09-20.csv` (corrected override) against
`real2`, the last hardware run without the hunting:

| Phase | Metric | real2 | identified sim |
| --- | --- | --- | --- |
| standing (0, 2, 4) | pitch RMS, rad | 0.029 / 0.032 / 0.024 | 0.024 / 0.023 / 0.037 |
| driving 0.2 m/s (1) | pitch RMS, rad | 0.022 | 0.014 |
| turn 0.6 rad/s (3) | yaw-rate error RMS | 0.21 | 0.20 |
| standing | trim / leg offset, m | | +0.003 / +0.004, command never at its limit |
| standing | leg heights, m | | 0.1716 / 0.1719 |

The next robot recording is the same command as above. Expect the leg offset
command off its limits and `trim_m` below 0.01 while standing, the body
level in pitch (no -0.045 pitch reference), both legs within a few mm of the
same height within about two seconds, and no 2 Hz hunting when driving. If
the right leg still swings more than the left with the legs centred, the
asymmetry is in the hardware (that drive's velocity estimate, backlash or
friction on one side), not in the controller's geometry.

### Fifth comparison (2026-09-20, analysis of the third run and the controller redesign)

`recordings/real3_2026-09-20.csv` was analysed signal by signal before
changing the controller. Three mechanisms were in the data:

| Finding | Evidence | Applied |
| --- | --- | --- |
| The yaw loop rang on its own | standing still (phase 0) the yaw torque oscillated at 3.6 Hz with ±0.8 Nm and the IMU yaw rate with ±0.9 rad/s; both legs' fore-aft positions followed it (correlation 0.85–0.89, yaw torque leading by 70 ms); `real2`, with `kp` 1.0, already rang at 4 Hz with ±0.4 Nm | model-based yaw loop: `kp` 0.3 Nm per rad/s (7 rad/s on `I_z` 0.09 kg m²) on the wheel-differential rate, integral 1.0 on the IMU rate, scrub 0.35 Nm and inertia fed forward from a lagged command (`MODELLING_AND_CONTROL.md` 3.5) |
| The wheel torque moved the axles through the leg compliance | driving (phases 1–3) everything hunted at 1.9 Hz; the right leg's fore-aft position was anti-correlated with the balance torque (−0.82 to −0.93) and swung 7 cm peak to peak; in the identified simulation the same correlation is ±0.73 on both legs. Rolling without slip a wheel torque works on the leg through the ground force `tau / r` and the stator moment on the wheel link, about 15 N-equivalent per Nm per wheel, i.e. 50 mm per Nm at the 300 N/m fore-aft stiffness | fore-aft impedance 400/20 like the height (the most the latency emulation accepts); damping on a 30 Hz low-passed velocity; gravity support protected from saturation; `balance.measured_offset_blend` to take the deflection out of the LQR if needed; the leg stiffness itself is a hardware tuning step (below) |
| One side much softer than the other | the right leg's fore-aft standard deviation was 20 mm against 9 mm on the left while the *left* wheel carried the larger torque (0.65 against 0.54 Nm RMS): the asymmetry is not the leg geometry (the identified simulation is symmetric to 0.1 mm) but the hardware, most likely backlash or friction on the right leg's drives, with both legs standing 40 mm off centre near the front pivot's singularity where the linkage is softest; the left inner drive's reported velocity was again 0.31 of its position derivative in phase 3 | neutral offset −0.008 (already in the override); check the right leg's drives for backlash and the left inner drive's velocity estimate before the next run |

The `2 Hz` itself is the pendulum's own frequency (unstable pole 12.8 rad/s)
with the 0.4 Nm friction dead band: inside it the wheels do not move, the
body falls, the wheels catch up.

The redesign was then validated in the identified latency-emulating
simulation (`recordings/sim_wbc2_latency_2026-09-20.csv`, sequence above
plus a 1 rad/s spin and a 0.4 m/s drive) against the previous controller
in the same simulation (`sim_identified2_2026-09-20.csv`):

| Phase | Metric | previous | redesign |
| --- | --- | --- | --- |
| standing (0, 2) | pitch RMS, rad | 0.024 / 0.023 | 0.015 / 0.020 |
| driving 0.2 m/s (1) | speed reached, m/s | 0.236 | 0.194 |
| turn 0.6 rad/s (3) | yaw-rate error RMS | 0.204 | 0.176 |
| standing (0, 2) | wheel torque RMS, Nm | 0.20 / 0.22 | 0.44 / 0.53 |
| standing (0, 2) | wheel velocity sign flips | 0.025 / 0.026 | 0.045 / 0.035 |
| driving 0.4 m/s | speed reached, m/s | – | 0.403 |

The extra wheel activity at standstill is the firmer slow loop (position and
velocity weights 30 / 4 against 10 / 1) against the simulated 0.2 Nm of
Coulomb friction per wheel; the robot stood calmer than the simulation with
the first set (pitch RMS 0.010, torque RMS 0.12 in `real3` phase 0), so
expect less of it on hardware. The same set balances the ideal simulation
(`sim_wbc2_ideal_2026-09-20.csv`).

Three results from the same campaign bound what the simulation can decide:

- **Leg stiffness.** Every fore-aft/height stiffness above about 400 N/m
  fell in the latency emulation at 12 Hz within half a second, at `kd / kp`
  of 0.008 and 0.02 s alike, while the ideal simulation balanced with 1500/30.
  The PID cascade's leg gains (12 Nm/rad and 0.5 Nm s/rad, about 1850 N/m and
  77 N s/m at the axle) also fell when closed in the controller through the
  emulated delays. This was resolved, not tuned around: the physical robot
  has always run these gains **inside the drives** (SandroControl commands
  them in the MIT frame), where the loop sees no bus or sensor latency, and
  the ROS plugin had been sending kp = kd = 0. The seventh comparison below
  records the port. The two hypotheses on the emulation's fidelity (filtered
  drive velocity, joint `armature` reflected by the gear ratio once instead
  of squared) remain worth checking for the wheel and torque paths.
- **Reaction feed-forward.** Feeding the wheel torque's reaction forward to
  the legs (`tasks.leg_reaction_ground_gain`) fell in the emulation even
  through a 4 Hz low-pass and at half gain, while the stator term alone was
  harmless: below the drive friction the torque does not reach the ground,
  yet the term still shifts the wheels back under the body, which tips the
  body the way the torque did. It ships off.
- **Offset state.** Feeding the LQR the commanded leg offset instead of the
  measured axle position removes the compliance path entirely but leaves the
  legs' sway undamped: standing pitch RMS 0.036 against 0.015 in the
  emulation. The measured position shipped (`measured_offset_blend` 1)
  until the fourth hardware run; the sixth comparison set it back to 0.

Next robot recording, with this build and the override:

```bash
ros2 launch sandro_bringup sandro.launch.py hardware:=real arm_hardware:=true \
  controller:=whole_body_controller teleop:=true \
  extra_params:=$(ros2 pkg prefix sandro_bringup)/share/sandro_bringup/config/sandro_real_overrides.yaml
python3 ros2_ws/src/sandro_bringup/tools/drive_test.py --record real4.csv --settle 3 \
  --phase 5,0,0 --phase 4,0.2,0 --phase 4,0,0 --phase 4,0.2,0.6 --phase 3,0,0
```

Expect: no yaw torque oscillation while standing (`yaw_torque_nm` within
±0.1 Nm), the wheel-differential and IMU yaw rates agreeing in sign and
within 10 % in the turn, speed reaching 0.2 m/s in phase 1, the leg offset
command off its limits, and both legs' fore-aft positions moving together
(`right_leg_x_m` and `left_leg_x_m` standard deviations within a factor of
1.5). Then raise the leg stiffness in steps as described in
`WHOLE_BODY_CONTROLLER.md`.

### Sixth comparison (2026-09-20, fourth hardware run: the leg lag)

`recordings/real4_2026-09-20.csv`, this build with the override, robot on a
short tether. It balanced, drove and turned (0.55 of 0.6 rad/s with a
0.13 s rise, yaw torque within ±0.05 Nm while standing: the yaw ringing is
gone), but standing still both legs swung fore-aft in phase at 1.9 Hz with
the leg offset command between its ±25 mm limits, ±1.5 Nm of wheel torque
and ±0.06 rad of pitch. The data isolates the loop: the measured axle
follows the leg offset command with 70 ms of lag in every phase
(cross-correlation 0.97–0.99), and the LQR's leg-offset mode has a 72 ms
time constant designed for an instantaneous leg. The 400/20 impedance on the
light unloaded wheel is overdamped, so its response time is `kd / kp` =
50 ms, plus stiction. The calm standing of the third run was the same
channel pinned at its −25 mm limit by the wrong neutral, i.e. switched off.

Setting `balance.measured_offset_blend` to 0 live stopped the oscillation
outright, so it is now the shipped default. It removes the lagging
measurement from the loop.

The durable form is the leg lag in the LQR model: the state gained the
commanded offset, the achieved offset follows it with `model.leg_lag_s`
(70 ms), and the regulator now takes 11.3 (m/s)/m from the command it knows
and 4.7 from the achieved offset, against 13.8 from a single measured offset
before (`MODELLING_AND_CONTROL.md` 3.2). In the identified latency simulation
(`recordings/sim_wbc3_leglag_2026-09-20.csv`, blend 0 as shipped, against
`sim_wbc2_latency` with blend 0 and no lag model):

| Phase | Metric | no lag model | lag model |
| --- | --- | --- | --- |
| standing (0, 2) | pitch RMS, rad | 0.036 / 0.037 | 0.033 / 0.030 |
| driving 0.2 m/s (1) | pitch RMS, rad | 0.028 | 0.012 |
| driving 0.2 m/s (1) | speed reached, m/s | 0.170 | 0.191 |
| turn 0.6 rad/s (3) | pitch RMS, rad | 0.013 | 0.015 |
| standing (0, 2) | leg offset command std, mm | 12.4 / 11.5 | 11.1 / 9.5 |

With the measured offset fed back (`sim_wbc3_leglag_measured`, blend 1) the
same design stands at 0.025 rad and drives at 0.012, so on the robot blend 1
is worth one more try once standing is clean with 0.

### Seventh comparison (2026-09-20, the leg PD moved into the drives)

The GIM6010 drives run in MIT mode: one frame per cycle with position and
velocity references, kp, kd and a torque, and the drive computes
`kp (q_ref - q) + kd (v_ref - v) + tau` in its own loop. SandroControl always
used this for the legs; the ROS hardware plugin packed kp = kd = 0, so every
leg loop closed over the bus and the sensor delay, which is what capped the
Cartesian impedance at 400 N/m and produced the 70 ms leg lag. The plugin now
exports `position`, `velocity`, `kp` and `kd` command interfaces next to
`effort`, the simulation wrapper emulates the same loop (fields delayed like
the CAN frame, law on the undelayed joint state), and the whole-body
controller sends the legs their joint references with SandroControl's gains
and only the gravity support as torque (`tasks.legs.pd_in_drive`).

Identified latency simulation, `recordings/sim_wbc4_drivepd_*` against
`sim_wbc3_leglag` (controller-side 400/20):

| | 400/20 in the controller | kp 12 / kd 0.5 in the drive | 19 / 0.5 | 20 / 0.3 |
| --- | --- | --- | --- | --- |
| leg response time (command to measured offset) | 144 ms | 72 ms | 48 ms | 24 ms |
| offset command minus measured, std | 3.8 mm | 1.7 mm | 1.9 mm | 1.9 mm |
| standing pitch RMS (phases 0, 2) | 0.033 / 0.030 | 0.025 / 0.027 | 0.022 / 0.025 | 0.022 / 0.023 |
| driving 0.2 m/s pitch RMS | 0.012 | 0.011 | 0.011 | 0.011 |
| standing wheel torque std | 0.31 / 0.26 | 0.23 / 0.25 | 0.21 / 0.22 | 0.20 / 0.23 |
| fall | no | no | no | no |

The 72 ms of the shipped gains matches the 70 ms the robot showed in the
fourth run with the same joint stiffness, so `model.leg_lag_s` stays 0.07;
after changing the drive gains re-identify it as the lag at which
`leg_offset_command_m` and `leg_offset_m` correlate best.

First hardware run with the drive-side PD (fifth run, not recorded): the
robot balanced well, but the stance was not symmetric (the right front link
stood at the angle of the left rear link, i.e. the two axles were displaced
in opposite body directions) and only the right leg followed height
commands. The plugin converts positions to the wire frame exactly as
SandroControl does (`sign * q + offset_rev / ratio * 2 pi`, all offsets 0),
so the reference is not the difference; what neither project checked is
whether each drive's own MIT position frame agrees with the encoder
broadcast. A left drive whose MIT zero differs would hold its joint against
the torque limit at a wrong angle while the controller, comparing the
broadcast, sees a small error and never moves it: a stiff, wrong, unmoving
leg, which is what was seen. The plugin now measures this bias per drive at
activation (zero gains, MIT reply against broadcast), logs it, corrects a
small one and refuses to arm beyond `mit_frame_bias_tolerance_rad`.

Next robot recording, this build. At activation read the six `MIT position
... bias ... rad` lines: the biases should be within a few hundredths of a
radian; a large one on the left drives confirms the diagnosis and is
corrected automatically below the tolerance. Also confirm every drive
reports `mit_max_kp` >= 12 and `mit_max_kd` >= 0.5. Then record the usual
sequence with a height phase (`--phase 3,0,0,-0.05 --phase 3,0,0,0.05`) and
check in the diagnostics that `position_command_<joint>_rad` and the joint
states agree within a few degrees on all four leg joints, that the right and
left legs move together in height, and that standing pitch stays within
about ±0.03 rad without the 1.9 Hz leg swing. The legs should follow their
command with about 70 ms of lag (if it differs, set `model.leg_lag_s`).

### Eighth comparison (2026-10-16, closed-loop simulation without ROS)

`sandro_sim/tools/closed_loop.py` runs the whole-body controller against
MuJoCo in one process: the unchanged `WholeBodyCore`, the latency emulation's
own `DelayLine`, `DriveBus` and MIT law, the controller manager's wake-up
rule on the 1/1200 s physics clock, and the IMU noise mujoco_ros2_control
adds. It writes `drive_test.py` recordings, a 23 s run takes under a second,
and `check` runs the gates below in a few seconds. Against the ROS
simulation's `sim_wbc4_drivepd_kp12` recording it reproduces standing pitch
RMS 0.024 vs 0.025 rad, speed error 0.072 vs 0.070 m/s, yaw rise 1.02 vs
0.96 s and identical leg efforts, so sweeps can be run there and only the
conclusions confirmed through the launch.

`compare_runs.py` now also reports the timing the controller saw. On the
hardware recordings and the simulations (phase 0):

| | `real4` | ROS simulation until now | simulation now |
| --- | --- | --- | --- |
| IMU orientation updates | 29.9 Hz | 125 Hz | 200 Hz (`sim_imu_rate_hz`) |
| encoder updates per joint | 199.9 Hz | 125 Hz | 199.9 Hz |
| joints updating together | 0.66 | 1.00 | 0.60 |

The encoders now match. **The IMU does not: every hardware run so far
received a new BNO085 sample only every 30 to 37 ms**, although the plugin
asks for 200 Hz, consistent with the FTDI adapter's USB latency timer
batching the UART stream. The emulation assumed 200 Hz. With the shipped
parameters and the robot overrides:

| IMU rate | 5 ms | 10 ms | 20 ms | 30 ms IMU delay |
| --- | --- | --- | --- | --- |
| 200 Hz | ok | ok | ok | falls |
| 100 Hz | ok | ok | ok | falls |
| 50 Hz | ok | ok | falls | falls |
| 30 Hz | ok | ok (falls at 15 ms) | falls | falls |

At 30 Hz a command delay of 6 ms also falls (3.3 ms does not), while the
encoder delay hardly matters any more (1 to 10 ms change nothing) because
the leg loop runs in the drives. The falls are a 6 Hz oscillation of the
balance loop growing within a second. At 30 Hz the hardware sequence still
passes, but `drive_test.py`'s default sequence (0.3 m/s, 1 rad/s turn,
spin, height changes) falls while driving at 0.3 m/s. Raising
`balance.effort_weights.wheel_torque` from 2 to 5 survives 30 Hz with 20 ms
at the cost of a softer stance (pitch RMS 0.041 against 0.024). The SPI
path on the Raspberry Pi should remove the batching; measure its rate and
delay with `compare_runs.py` before choosing between the two.

Two more results:

- **The 0.3 to 0.4 Hz standing sway is friction, not latency.** It is the
  same with every delay at zero; compensating the full 0.2 Nm of simulated
  wheel Coulomb friction (`balance.wheel_friction_ff_nm`) calms it to
  0.009 rad but makes the wheels chatter (10 % sign flips), and the posture
  trim adapting during it makes it larger. `real3` stood with the same
  0.38 Hz.
- **The robot's controller-side legs were faster than the simulated ones.**
  With the 400/20 Cartesian impedance the measured axle followed its command
  by 62 to 84 ms on the robot (`real4`) against 120 to 230 ms in simulation;
  with the drive-side PD the simulation's 54 to 66 ms matches
  `model.leg_lag_s` 0.07. The leg drives' simulated friction or armature is
  the suspect (identification step 4 below). The simulation does not
  reproduce `real4`'s 1.9 Hz leg hunting with that configuration at any
  latency it survives.

The gates (`closed_loop.py check`, controllers.yaml plus the robot
overrides): the hardware sequence at nominal latency standing within
0.035 rad RMS and tracking 0.2 m/s; the same with a 30 Hz IMU; the same
with the drive law once per control cycle; 8 N shoves; the default
drive_test sequence at nominal latency and with a 30 Hz IMU. The last fails
today, for the reason above. Run them after every parameter or model change.

## 3. Identify, in this order

Each step changes one thing in `sandro_description/config/sim_identified.yaml`
or the controller model, then `tools/sync_model.py sync` regenerates the ROS
model and the comparison is repeated.

1. **Mass and CoM.** Lock both legs at a known pose (spacer or zip ties,
   drives unpowered) and weigh the robot. Then stand it on its wheels, tilt
   the body forward and hold it on a kitchen scale at a measured horizontal
   distance from the axle; note the IMU pitch and the reading. Repeat
   backward (+-15 to 30 deg, support 0.2 m or more from the axle).
   `sandro_description/tools/com_from_weighing.py` turns the readings into
   the CoM height above the axle and its forward offset (about +-2 mm and
   +-1 mm with a 2 g scale), evaluates the MuJoCo model at the same leg pose,
   and prints the payload (`--write` puts it into `sim_identified.yaml`),
   `model.body_mass_kg`, `tasks.fore_aft.neutral_offset_m` and, with
   `--leg-height`, `model.com_height_offset_m`. It needs the `mujoco` Python
   package. The CoM height is the one number the balance model cannot get
   from the trim; the trim check below still applies.
   `payload.mass_kg` is the total minus the
   CAD's 3.40 kg. Stand on the controller and read `trim_m` from
   `~/diagnostics` once it settles: positive means the real centre of mass is
   behind the model's neutral axle by that lever. Move the controller's
   neutral there (`tasks.fore_aft.neutral_offset_m`, negative for backward)
   and place the payload so the simulated CoM moves the same way; set
   `model.body_mass_kg` to the weighed value minus the two wheels. Both go in
   `config/sandro_real_overrides.yaml`, applied with `extra_params:=` on both
   backends.
2. **Latency.** The rates are measured: `compare_runs.py` reports
   `imu_update_hz`, `encoder_update_hz` and `encoder_sync_frac` from any
   recording, and `sim_imu_rate_hz` / `sim_joint_rate_hz` must reproduce
   them (encoders 200 Hz unsynchronised; IMU 30 Hz over UART, the SPI path
   still to be measured). Measure the IMU delay with
   `tools/imu_delay.py`: legs locked, wheels blocked so they cannot roll,
   broadcasters running (`controller:=none`), rock the body by hand for
   30 s. Each wheel encoder then reads minus the body pitch, so the shift of
   maximum correlation is the IMU delay minus the encoder delay, as the
   controller receives them (synthetic recordings: within 0.1 ms).
   `sim_imu_delay_s` is that lag plus `sim_joint_delay_s`. Measure the
   encoder age from the `feedback_age` interface on `/dynamic_joint_states`.
   The IMU delay is the one that matters (eighth comparison).
3. **Tyre scrub.** Compare `yaw_rise_s` and `yaw_err_rms` in the turning
   phases. Raise `tyre.friction[1]` (torsional) until the simulated yaw
   response is as slow as the robot's, then retune `tasks.yaw.*` in
   simulation.
4. **Drive friction and inertia.** The lifted-robot excitation in
   `SandroControl/fit_sysid.py` fits damping, Coulomb friction, reflected
   inertia and torque scale per group; copy the accepted fit into `joints`
   and `actuators`.
5. **IMU noise.** From a standing recording, the standard deviation of the
   real pitch minus its low-pass goes into `imu_noise.orientation`.

Only then tune the controller, in the simulation, and confirm on the robot
with the same recording and comparison.

## 4. Loop rate and the 500 Hz question

The controller manager and the CAN traffic run at 300 Hz. Raising them to
500 Hz would not close the gap, for two reasons.

- The bus is nearly full already. Each cycle sends six MIT frames and receives
  six replies, on top of the drives' 200 Hz encoder and current broadcasts and
  heartbeats: about 6 000 frames/s, roughly 75 % of a 1 Mbit/s bus. At 500 Hz
  that is about 8 500 frames/s, over 100 %. The queue then adds far more delay
  than the faster cycle removes.
- The delay that matters is the sensor path, not the cycle. With the leg PD
  in the drives the IMU path decides the balance loop's margin (eighth
  comparison); going from 300 to 500 Hz shaves 1.3 ms while the IMU holds
  each sample for up to 33 ms over UART.

What does help: get the BNO085 samples to the controller at their report
rate (the SPI path; the game rotation vector runs to 400 Hz) and take joint
state from the MIT reply frames that already arrive every cycle instead of
the separate 5 ms encoder broadcast (`state_source:=mit`), which removes
about 20 % of the bus load.

## 5. Tooling summary

| Tool | Purpose |
| --- | --- |
| `sandro_bringup/tools/drive_test.py --record` | same scripted drive on any backend, CSV of diagnostics, joint states and IMU; `--record-from-start` also captures the engagement transient (phase -1) |
| `sandro_bringup/tools/compare_runs.py` | per-phase gap metrics for two or more recordings, including the sensor update rates and the leg lag |
| `sandro_bringup/tools/imu_delay.py` | `record` joint states and IMU while the body is rocked with the wheels blocked; `analyze` the IMU delay relative to the encoders (orientation and gyro) |
| `sandro_description/tools/com_from_weighing.py` | CoM height and forward offset from tilted weighings, and the matching sim payload and controller parameters |
| `sandro_sim/tools/closed_loop.py` | the whole-body controller against MuJoCo without ROS, about 50 times real time: `run` (one recording), `sweep` (grid over latencies and parameters), `check` (the gates of the eighth comparison) |
| `sandro_description/config/sim_identified.yaml` | identified reality applied to the ROS MJCF by `sync_model.py` |
| `sandro_sim/LatencySystem` | sensor and command latency in simulation; `sim_latency:=false` and the `sim_*` launch arguments control it |
| `extra_params:=file.yaml` launch argument | controller parameter overrides for tuning experiments without editing the shipped config |
| `~/diagnostics` of the whole-body controller | now also `wheel_yaw_rate_rad_s`, `yaw_feedforward_nm`, `leg_offset_state_m` and the per-joint `*_leg_reaction_*_nm`; the configure log prints the LQR gains with the input-delay margin |
| `sandro_bringup/config/sandro_real_overrides.yaml` | the identified per-robot values (mass, neutral offset, yaw limit, friction, IMU roll bias); apply on both backends |
| `state_source:=mit encoder_period_ms:=20` launch arguments | hardware joint state from the synchronous MIT reply instead of the 5 ms broadcast |
| `position`/`velocity`/`kp`/`kd` command interfaces | the full MIT frame per joint on both backends; a controller that leaves them unset drives a torque source |
