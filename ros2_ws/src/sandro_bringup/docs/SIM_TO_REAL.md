# Closing the sim-to-real gap

The physical robot balances and drives on the whole-body controller but shows
more oscillation and slower turning than the simulation did. This page is the
workflow and the tooling for making the simulation predict the robot, so that
tuning done in MuJoCo transfers. Model background is in
`MODELLING_AND_CONTROL.md`.

## 1. What the simulation was missing

| Effect | Reality | Simulation before | Now |
| --- | --- | --- | --- |
| Sensor latency | BNO085 reports at 200 Hz plus fusion lag; drive encoder broadcasts every 5 ms | fresh every 3.3 ms, zero delay | `sandro_sim/LatencySystem` decorator: delay + sample-and-hold per interface |
| Command latency | a command reaches the drive one control cycle later | applied at the next physics step | same decorator, command path |
| Tyre scrub | rubber tyre resists twisting about the contact normal | condim 3, no torsional friction | condim 4, torsional friction 0.004 m, 10 ms contact |
| CoM / mass | robot not weighed; CAD only | CAD | `payload` in `config/sim_identified.yaml` |
| Drive friction, torque constant | unmeasured | legacy fit / CAD | `joints`, `actuators` in the same file |

The two latencies turned out to be the whole story of the oscillation. With
the emulation on (IMU 10 ms at 200 Hz, encoders 5 ms at 200 Hz, one cycle of
command delay), the parameters that balanced the ideal simulation fell within
two seconds: the leg Cartesian impedance (fore-aft 1000/40, height 600/40)
oscillated at the sample rate, and the balance LQR had a loop-delay margin of
exactly 3 samples (10 ms). A sweep isolated the encoder-plus-command path as
the critical one; 10 ms on the IMU alone was tolerated.

The shipped parameters changed accordingly (fore-aft 300/12, height 400/20
with a support integral, wheel torque weight 2.0 giving 17 ms of margin); the
latency-emulating simulation now balances, drives, turns and spins with them.
Expect the robot to be calmer with the same set.

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
if it helps set `sim_joint_delay_s:=0.0033 sim_joint_rate_hz:=300` for the
simulation to match.

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
  The PID cascade (`driving_balancer`, joint gains 12 Nm/rad and 0.5 Nm s/rad,
  about 1850 N/m and 77 N s/m at the axle) **also falls in the latency
  emulation**, and it balances the physical robot. The emulated leg loop is
  therefore pessimistic. Two candidate reasons, both checkable: the encoder
  velocity the drives report is filtered (the reported velocity is 0.85 of
  the position derivative in every recording, part of which may be
  attenuation bias of the regression), whereas the emulation samples and
  holds MuJoCo's raw velocity; and the joint `armature` of 0.000168 kg m² in
  the MJCF is a rotor inertia reflected by the gear ratio once rather than
  squared (a 60 mm frameless rotor is about 2e-5 kg m², times 64 is
  1.3e-3), which would make the simulated leg and wheel eight times lighter
  on the motor side than the real ones. Either makes the real leg loop far
  more tolerant of damping than the emulation. Until one is confirmed, tune
  the leg stiffness on the robot as described in `WHOLE_BODY_CONTROLLER.md`.
- **Reaction feed-forward.** Feeding the wheel torque's reaction forward to
  the legs (`tasks.leg_reaction_ground_gain`) fell in the emulation even
  through a 4 Hz low-pass and at half gain, while the stator term alone was
  harmless: below the drive friction the torque does not reach the ground,
  yet the term still shifts the wheels back under the body, which tips the
  body the way the torque did. It ships off.
- **Offset state.** Feeding the LQR the commanded leg offset instead of the
  measured axle position removes the compliance path entirely but leaves the
  legs' sway undamped: standing pitch RMS 0.036 against 0.015 in the
  emulation. The measured position ships (`measured_offset_blend` 1).

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

Next robot recording: the same command as above with this build. Expect
standing pitch within about ±0.03 rad without the 1.9 Hz leg swing, the leg
offset command moving by a few millimetres rather than between its limits,
and the diagnostics' `leg_offset_model_m` tracking `leg_offset_m` within a
few millimetres (if it does not, the identified lag is off: re-read it from
the recording).

## 3. Identify, in this order

Each step changes one thing in `sandro_description/config/sim_identified.yaml`
or the controller model, then `tools/sync_model.py sync` regenerates the ROS
model and the comparison is repeated.

1. **Mass and CoM.** Weigh the robot. `payload.mass_kg` is the total minus the
   CAD's 3.40 kg. Stand on the controller and read `trim_m` from
   `~/diagnostics` once it settles: positive means the real centre of mass is
   behind the model's neutral axle by that lever. Move the controller's
   neutral there (`tasks.fore_aft.neutral_offset_m`, negative for backward)
   and place the payload so the simulated CoM moves the same way; set
   `model.body_mass_kg` to the weighed value minus the two wheels. Both go in
   `config/sandro_real_overrides.yaml`, applied with `extra_params:=` on both
   backends.
2. **Latency.** The defaults are estimates. Measure the IMU path by tapping
   the body while recording `/imu_broadcaster/imu` and `/joint_states` at
   full rate (the recorder does this) and comparing the pitch-rate onset with
   the wheel encoder reaction; measure the encoder age from the `feedback_age`
   interface on `/dynamic_joint_states`. Set `sim_imu_delay_s`,
   `sim_joint_delay_s` and the rates as launch arguments until the standing
   phase's `pitch_peak_hz` and `wheel_flip_frac` match the robot.
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
- The delay that matters is the sensor path, not the cycle. The emulation
  shows the encoder hold (5 ms) plus one command cycle is what limits the leg
  and wheel loops; the IMU at 10 ms is tolerated. Going from 300 to 500 Hz
  shaves 1.3 ms of the roughly 15 ms loop while the encoder broadcast stays at
  200 Hz.

What does help: raise the BNO085 report rate (the game rotation vector runs
to 400 Hz) and take joint state from the MIT reply frames that already arrive
every cycle instead of the separate 5 ms encoder broadcast, which would remove
about 20 % of the bus load and give 300 Hz fresh joint data. Both are
hardware-plugin changes, worth doing after the identification above shows how
much latency remains.

## 5. Tooling summary

| Tool | Purpose |
| --- | --- |
| `sandro_bringup/tools/drive_test.py --record` | same scripted drive on any backend, CSV of diagnostics, joint states and IMU; `--record-from-start` also captures the engagement transient (phase -1) |
| `sandro_bringup/tools/compare_runs.py` | per-phase gap metrics for two or more recordings |
| `sandro_description/config/sim_identified.yaml` | identified reality applied to the ROS MJCF by `sync_model.py` |
| `sandro_sim/LatencySystem` | sensor and command latency in simulation; `sim_latency:=false` and the `sim_*` launch arguments control it |
| `extra_params:=file.yaml` launch argument | controller parameter overrides for tuning experiments without editing the shipped config |
| `~/diagnostics` of the whole-body controller | now also `wheel_yaw_rate_rad_s`, `yaw_feedforward_nm`, `leg_offset_state_m` and the per-joint `*_leg_reaction_*_nm`; the configure log prints the LQR gains with the input-delay margin |
| `sandro_bringup/config/sandro_real_overrides.yaml` | the identified per-robot values (mass, neutral offset, yaw limit, friction, IMU roll bias); apply on both backends |
| `state_source:=mit encoder_period_ms:=20` launch arguments | hardware joint state from the synchronous MIT reply instead of the 5 ms broadcast |
