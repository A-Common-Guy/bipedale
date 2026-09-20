# Sandro: modelling and control design

This is the record of how the robot was modelled and why the whole-body
controller is built the way it is. Procedures live elsewhere: simulation and
tuning in `WHOLE_BODY_CONTROLLER.md`, hardware bring-up in
`HARDWARE_COMMISSIONING.md`, joint convention and model regeneration in the
`sandro_description` README.

## 1. The robot

Two planar five-bar legs carry a rigid body on two wheels. Each leg has two
motors on the body (inner and outer pivot, 90 mm apart), two 80 mm upper links,
two 130 mm lower links meeting at the wheel axle, and a wheel motor at the
axle. All six actuators are GIM6010 drives (8:1) in torque mode over CAN at
300 Hz; a BNO085 IMU on the roof gives orientation and rates. The controllers
see six effort commands, six joint positions and velocities, and the ten
standard IMU interfaces, identically in MuJoCo and on hardware.

Frames: body x forward, y left, z up (the IMU frame). Planar leg coordinates
are +x from the inner toward the outer pivot and +y down toward the wheel. The
inner/outer motor names are mirrored between the sides (right inner is the
rear motor, left inner the front one), so planar +x is body-forward on the
right leg and body-backward on the left leg.

## 2. Modelling phase

### 2.1 Sources of truth

| Quantity | Source | Trust |
| --- | --- | --- |
| Link geometry, loop closure, contacts | Onshape MJCF export (`robot_full_floating.xml`) | authoritative |
| Masses, CoM, inertias | same export (CAD densities) | CAD values, robot not yet weighed |
| Joint zeros and signs | derived from export geometry, calibrated on hardware | authoritative once commissioned |
| URDF | schematic tree for RViz only | no inertias, not a physics model |

### 2.2 Joint convention

The raw export parametrises hinges in arbitrary CAD frames: leg zeros differ
per side and the left wheel hinge points the opposite way, so equal wheel
efforts spun the robot in place. The controllers, the legacy runtime and the
calibrated drives use one convention: leg joints zero when the upper link
hangs straight down, positive opening away from the leg centre; wheels
positive when rolling forward. A per-joint `(sign, zero)` table is derived
from the export geometry (`derive_joint_frames.py`, the same construction the
legacy runtime did at start-up) and baked into the ROS MJCF as a flipped axis
plus a joint `ref` (`sync_model.py`). Checks: body poses unchanged to 1e-9,
the `stand` keyframe converts to a near-symmetric leg pose, equal efforts drive
both wheels forward with the expected backward pitch reaction, opposite efforts
yield pure yaw.

### 2.3 Five-bar kinematics

Closed-form planar FK, IK and a finite-difference Jacobian
(`sandro_controllers/kinematics.*`) with base 0.090, upper 0.080, lower
0.130 m. With the convention applied, FK reproduces the MJCF axle position to
0.1 mm at the keyframe. Workspace: reach is 0.21 m from each pivot, so at
0.20 m height the fore-aft range collapses to ±19 mm and the linkage nears its
stretched singularity; the usable height range was set to 0.14–0.19 m with a
3 % margin from full stretch and the ±120° joint range enforced by validation.
Leg velocities come from the Jacobian, leg forces map to motor torques by
`tau = J^T F`.

### 2.4 Mass properties (CAD)

| Quantity | Value | Note |
| --- | --- | --- |
| Total mass | 3.40 kg | |
| Body mass `M` (carried by the legs) | 3.284 kg | total minus two 58 g wheels |
| Wheel radius `r` | 0.0742 m | tyre cylinder |
| Wheel inertia `I_w` (per wheel) | 0.00033 kg m² | disc 0.000161 + reflected rotor 0.000168 |
| CoM height above axle `L` | 0.654 × leg height | 0.121 m at the 0.1855 m keyframe |
| Body pitch inertia `I` about CoM | 0.0295 + 0.15 (h − 0.1855) kg m² | |
| CoM fore-aft | over the axle at planar x = 0.045 | keyframe axle sits 11 mm behind it |
| Track | 0.304 m (physical), 0.273 m tyre spacing in CAD | yaw kinematics only |
| Joint friction (sim) | 0.1 Nm Coulomb, 0.01 Nm s/rad | all six hinges |
| IMU noise (sim) | 0.017 on quaternion, 0.002 rad/s gyro | about 1° attitude noise |

The affine fits were obtained by posing the MJCF at several leg heights with
the loop constraints solved, then reading the assembled CoM and inertia.

### 2.5 Reduced dynamic model

The legs are position-controlled Cartesian actuators, fast compared with the
body, so the balance dynamics reduce to a wheeled inverted pendulum whose
centre of mass the legs can shift fore-aft. With `x` the axle position,
`pitch` positive leaning forward, `d` the forward CoM offset produced by the
legs (positive when the wheels are placed backward) and `tau` the total wheel
torque acting between wheels and body, the Lagrangian gives, linearised about
upright:

```
a x_dd + b pitch_dd = tau / r                    a = M + 2 m_w + 2 I_w / r^2
b x_dd + c pitch_dd = M g (L pitch + d) - tau     b = M L,   c = I + M L^2
```

The leg offset `d` enters exactly like `pitch` scaled by `1/L`: a 10 mm wheel
shift equals a 10 mm / 0.11 m ≈ 5° lean, obtained at leg speed instead of by
accelerating the whole body. The open-loop pendulum pole is about 12.8 rad/s
(78 ms), which is why the cascade in the wheel-only controller has little loop
separation to work with, and why the legs help. The state is
`[x, x_dot, pitch, pitch_rate, d]`, the inputs `[tau, d_dot]`.

The same model is written nonlinearly in `test_whole_body.cpp` (with leg lag
and an unmodelled CoM offset) as the plant the controller is unit-tested on.

### 2.6 What the model omits, and what covers it

| Omitted | Consequence seen | Covered by |
| --- | --- | --- |
| Tyre contact compliance, gearbox backlash | wheels chatter at the sampling rate when the velocity gain is high | gain limit on wheel velocity feedback, 15 Hz velocity filter |
| Leg compliance under wheel torque: the ground force `tau / r` at the axle and the stator moment `-tau` on the lower link that carries the wheel motor deflect the leg impedance (about 50 mm per Nm per wheel at the first 300 N/m, 15 mm at 400 N/m) | the axle moved centimetres with every torque swing on the robot; the yaw differential twisted the two legs in opposite directions | stiffer legs, tuned on the robot (the delay emulation forbids it in simulation, section 4); optional reaction feed-forward (off); `measured_offset_blend` to take the deflection out of the LQR if it hunts |
| Yaw dynamics: body twisting on the legs and on the tyre scrub | the 1.0–1.5 Nm per rad/s yaw gain on the IMU rate rang at 3.6 Hz with ±0.8 Nm while the robot stood still | model-based yaw loop: proportional on the wheel-differential rate at about 7 rad/s, integral on the IMU rate, scrub and inertia feed-forward (3.5) |
| Sensor and command latency (IMU 200 Hz plus fusion, encoders 200 Hz, one cycle of command delay) | oscillation on hardware; the ideal-sim tuning falls in the latency-emulating simulation | `sandro_sim/LatencySystem` in simulation, gains chosen for 17–30 ms of delay margin (`SIM_TO_REAL.md`) |
| Leg tracking lag (~40 rad/s loaded) | LQR assumes ideal placement | leg-offset weights keep that mode below the leg bandwidth |
| CoM position error, slopes, rolling drag | steady lean | posture trim moves the axle under the CoM |
| Lateral dynamics in turns | outward tipping | lean-into-turn roll task |
| Lower-leg mass not carried by the axle force, joint friction | ~1 cm height error at the extremes | acceptable; a height integral is the next step if needed |

### 2.7 Identification path for the physical robot

Weigh the robot (`model.body_mass_kg`). Stand it on the controller and read
`trim_m`: it converges to the lever between the real centre of mass and the
model's neutral axle position; move the neutral there with
`tasks.fore_aft.neutral_offset_m` so the legs keep their range, and give the
simulation the same mass and CoM through the `payload` body in
`sandro_description/config/sim_identified.yaml`. Shifting the axle also moves
the leg masses, so about 0.7 mm of lever is gained per mm of shift and the
trim iterates once. CoM height and inertia only enter through the gains and
are tolerant of 10–20 % error; the earlier `SandroControl/fit_sysid.py` route
remains available for actuator friction and inertia if higher fidelity is
wanted. The first robot recording and its comparison are in `SIM_TO_REAL.md`:
about 1.2 kg more than CAD, CoM 28 mm behind the CAD neutral, tyres that do
not skid, and a yaw loop that was torque-limited.

## 3. Control choices

### 3.1 Architecture

Controllers are ordinary `ros2_control` plugins that all claim the same six
efforts and read the same states and `/cmd_vel`; the controller is a launch
argument and can be switched live. Three balancers exist: the PID cascade
(`driving_balancer`), a wheel-only full-state LQR (`lqr_balancer`), and the
whole-body controller described here. Everything below is in the ROS-free core
so it is testable without a robot.

### 3.2 One LQR over wheels and leg placement

Rather than a pitch loop feeding the wheels plus a separate leg loop, both
balance actuators are inputs of one regulator, so the trade-off between wheel
torque and leg motion is made by the cost function, not by hand. The
discrete-time Riccati equation is solved with the structure-preserving doubling
algorithm at seven leg heights between 0.14 and 0.19 m, and the gains are
interpolated on the measured height because `L`, `I` and the unstable pole all
change with it. Solving happens at configure and in the parameter callback,
never in the realtime loop; a set that does not yield a stabilising gain is
rejected with a reason.

Weights follow Bryson's rule, 1/(acceptable deviation)² in SI units, with
three deliberate departures: `Q = diag(30, 4, 150, 1, 1500)`, `R = diag(2, 20)`
(position and velocity were 10 and 1 in the first set; 30 / 4 tracks speed
against the drive friction without overshoot, 60 / 8 hunted against Coulomb
friction in simulation).

- The velocity and torque weights keep the per-wheel velocity feedback well
  below `I_w / control period ≈ 0.1 Nm per rad/s`. That feedback also closes
  a loop on the wheel's own inertia through the tyre or gearbox compliance, a
  mode far above the pendulum's; the first weight set (0.18) chattered at
  300 Hz in simulation and fell.
- The torque weight also sets the loop-delay margin of the balance loop
  (largest pure input delay the closed loop tolerates, computed by augmenting
  the discrete plant with delayed inputs): 0.6 gave 3 samples (10 ms), which
  is about the physical sensor-plus-command latency and is why the first
  hardware runs oscillated; 2.0 gives 5 samples (17 ms); 5.0 gives 7 samples
  but is sluggish (leg offset and position limits hit). 2.0 is shipped.
- The leg offset rate is cheap so the legs carry a real share of the balance.

Resulting gains at 0.17 m and 4.5 kg (total wheel torque, leg offset rate):

| State | Wheel gain | Leg gain |
| --- | --- | --- |
| position | −0.70 Nm/m | 1.16 (m/s)/m |
| velocity | −0.66 Nm/(m/s) | 0.88 (m/s)/(m/s) |
| pitch | −10.4 Nm/rad | 0.89 (m/s)/rad |
| pitch rate | −0.91 Nm/(rad/s) | 0.15 (m/s)/(rad/s) |
| leg offset | −21.9 Nm/m | 13.8 (m/s)/m |

Per-wheel velocity feedback is 0.024 Nm per rad/s. The fastest closed-loop
pole (pitch-rate damping on a low-inertia body) sits near −60 rad/s; with
0.6 it was −86 rad/s, which the measured latency could not support. The
input-delay margin is now computed at configure time for every table height
(the discrete closed loop augmented with delayed inputs) and a set below
`balance.required_delay_margin_s` is rejected; the shipped set has 5 samples
(16.7 ms) at every height.

The offset state the regulator feeds back is the measured axle position by
default (`balance.measured_offset_blend` 1). It is the physical gravity
lever, and in the latency-emulating simulation the legs' deflection under
wheel torque is damping information: pitch RMS while standing is 0.015 rad
against 0.036 with the commanded offset fed back (blend 0). The robot's 2 Hz
hunting ran through this path, but it was driven by the yaw loop and by legs
pinned near their singularity (`SIM_TO_REAL.md`, fifth comparison); the blend
exists so that the path can be opened if hunting persists once those are
fixed.

### 3.3 Legs as Cartesian impedances

Per leg, in the planar frame, `F = Kp (p_ref − p) − Kd p_dot_f + [0, F_support]`,
mapped through `J^T`, with `p_dot_f` the Cartesian velocity low-passed at
30 Hz. Gravity support `M g / 2` is fed forward so the impedance only has to
correct errors, and a slow integral on the mean height error (150 N/(m s),
bounded to ±15 N) learns the share the feed-forward misses: the lower links
rest partly on the wheels, so the axle force is less than half the non-wheel
weight, which showed up as a 1.4 cm standing sag. Under saturation the
support is protected: only the corrective part of a leg's torque is scaled,
as a whole so the force keeps its direction, and the body is never dropped
to make room for a correction. Every commanded target is clamped to the
validated workspace.

The legs are the balance loop's inner actuator and their stiffness is the
open tuning question of this design. Rolling without slip, a wheel torque
`tau` does work on the leg through the ground force `tau / r` at the axle and
through the stator moment `-tau` on the lower link that carries the wheel
motor (virtual work: `Q = tau (J_x / r - d(phi)/dq)`), about 15 N-equivalent
per newton-metre per wheel at the standing pose. At the first 300 N/m that
was 50 mm of axle travel per newton-metre, and the robot's legs swung 7 cm
peak to peak with the 2 Hz torque swings; at 400 N/m it is still 15 mm.
Stiffness is bounded by loop delay in a specific way: with about 13 ms of
sensor-plus-command delay the stiffness term is negative damping,
`k sin(wT)/w` about `k T`, so the damping must grow with the stiffness
(`kd / kp` about 0.04 s) while `kd / m_leg` times the delay stays small.
The PID cascade's joint gains (12 Nm/rad, 0.5 Nm s/rad, about 1850 N/m and
77 N s/m at the axle through `J^-1`) satisfy this and balance the robot. The
latency-emulating simulation, however, falls above about 400 N/m at either
ratio, and it also falls with the cascade's gains; the emulated leg loop is
therefore pessimistic (section 4), and the stiffness is tuned on the robot
(`WHOLE_BODY_CONTROLLER.md`). Height and fore-aft ship at 400/20, the values
the emulation accepts.

The reaction can also be fed forward (`tasks.leg_reaction_{ground,stator}_
gain`, `J^T [-tau / r, 0] + tau d(phi)/dq` through a low-pass). It is off:
the ground term assumes the torque reaches the ground, and below the drive's
0.2 Nm of Coulomb friction it does not, yet the term still shifts the wheels
back under the body and tips it the way the torque did, a positive feedback
that fell in the latency simulation with 400 N/m legs; the stator term alone
is harmless there but changes little. It remains an experiment for stiff
legs and an identified wheel friction.

### 3.4 Posture trim

At rest the body balances when `L pitch + d + d_unmodelled = 0`, so the
measured gravity lever `L pitch + d` is the unmodelled offset. It is low-pass
filtered (1 /s) while the robot is at rest, that is while both the drive
command and the measured speed are inside a 0.05 m/s deadband and the balance
torque is inside 0.15 Nm, and moved into the leg offset reference; what
exceeds the leg range becomes a pitch reference. The torque test is what
separates a mass offset from a lean held against wheel stiction: balanced at
rest the wheels hold no steady torque, pushing against 0.4 Nm of friction
they do, and the third hardware run showed the trim learning that lean and
moving the neutral 30 mm off.
The robot therefore stands level with the wheels under its centre of mass
instead of leaning, on flat ground and on slopes up to what 25 mm of wheel
placement can absorb. The trim starts from zero at activation. Seeding it
from the lever measured at engagement was tried and removed: at that moment
the legs stand where the impedance or a reset pose left them, and reading a
26 mm leg placement as a mass offset sent the identified simulation into a
runaway. An identified centre of mass belongs in the neutral offset
(`tasks.fore_aft.neutral_offset_m`), which is also what gives the legs their
range back; with the legs pinned against their limit the robot can neither
track a speed against rolling resistance nor stop creeping (see
`test_whole_body.cpp`, `ReleasingTheStickHoldsHereInsteadOfCatchingUp` and
`LegRangeRestoresSpeedTrackingAgainstRollingResistance`).

### 3.5 Turns and roll

Lateral acceleration `v yaw_rate` tips the robot outward; the body is rolled
into the turn by `atan(v yaw_rate / g)` through a differential leg height
(`track × tan(roll)`), plus a PI on IMU roll that also levels the body on
laterally uneven ground. The yaw rate in that lean is the commanded one after
a first-order lag (`drive.yaw_command_time_constant_s`, 0.15 s), so the lean
and the feed-forwards below are smooth.

Yaw is designed on its own model: `I_z psi_dd = (track / 2 r) dtau − scrub`,
with `I_z` about 0.09 kg m² from the identified MJCF (wheel spin inertia
reflected through the track included) and `track / 2 r` = 2.05. The
proportional gain follows from the wanted bandwidth, `kp = I_z w / 2.05`:
0.3 Nm per rad/s for 7 rad/s. It acts on the wheel-differential yaw rate,
which is synchronous with the loop and free of the body's twist on the legs
and on the tyre scrub (`tasks.yaw.wheel_rate_blend`; the first hardware runs
showed the wheel-differential and gyro rates agreeing to 5 %). The integral
(1.0 Nm/rad, 4.8 rad/s with damping ratio 0.7 together with `kp`) acts on the
IMU rate, so the body itself reaches the commanded rate against tyre scrub and
drive friction. The Coulomb-like scrub is fed forward with the sign of the
command (`scrub_ff_nm`, 0.35 Nm on the robot from the second recording) and
the yaw inertia with the command's acceleration. The first design, a PI of 1.0
to 1.5 Nm per rad/s on the IMU rate, closed a 30-plus rad/s loop through the
IMU, the scrub and the soft legs and rang at 3.6 Hz with ±0.8 Nm while the
robot stood still; while driving, the ±1.4 Nm differential twisted the two
legs in opposite directions and was a large part of the 2 Hz hunting.
Differential torque is allocated after balance inside the per-wheel torque
budget, so a turn can never take authority from balancing.

### 3.6 Speed, position and height

The position state integrates the commanded speed, so speed tracking and
station keeping are the same regulator; the reference is bounded within 0.3 m
of the robot so pushing against an obstacle cannot wind up, and while driving
that bound is the integral action that makes the speed track against
friction. Releasing the stick anchors the hold at the current position, so a
speed the robot could not follow never becomes a catch-up creep afterwards. Height is a slewed
reference between 0.14 and 0.19 m that the `linear.z` field of the same
`TwistStamped` moves as a rate, mapped to the right stick on the gamepad.

### 3.7 Safety and start-up

- Balancing engages only when the body is upright (|pitch| < 0.15 rad) and
  still (< 0.5 rad/s) after `settle_s`; until then the legs hold their stance
  and the wheels are off. References re-anchor at engagement, so the
  impedance never steps when the state has jumped (operator lifting the robot,
  simulation reset).
- A fall while balancing latches every actuator off until the controller is
  re-activated. Not being upright before engagement is the normal starting
  condition and does not latch; the earlier design latched here and could
  never start in simulation.
- A stale command zeros motion but keeps balancing. A stale IMU stops the
  wheels and keeps the legs holding, since a collapse is worse than a stance.
- Torque, torque-rate, workspace, leg-offset, height, roll and position limits
  are enforced and reported in `~/diagnostics`; all tunables are validated as a
  complete set.
- Simulation: physics runs about a second before the controller is active and
  unpowered legs collapse, so the launch resets the world to the stand keyframe
  once the controller is up. A start-paused sequence was rejected because the
  simulator's control loop is paced by simulated time and activation would
  deadlock.
- Gamepad: L1 deadman; L2 + R2 together deactivate every command-owning
  controller and fire the CAN E-stop on hardware. Software only, never re-arms.

### 3.8 Alternatives considered

- Full rigid-body inverse-dynamics QP: the right long-term backend, but it
  needs identified inertias, a closed-chain model and a QP solver in the
  image; the URDF has no inertias and the MJCF values are unverified. The
  plugin, estimator and task definitions are written so only the allocation
  core changes when it arrives.
- Sharing pitch between wheels and legs with two hand-tuned gains (the first
  whole-body draft): replaced by the single LQR so the split is principled and
  height-dependent.
- Anchoring the fore-aft task to the pose at activation (first draft): removed
  once the joint convention made absolute planar positions trustworthy.
- Wheel damping referenced to zero speed (cascade): not used; the LQR velocity
  term is referenced to the command and is the stabilising feedback.

## 4. Validation record

Unit tests (`test_whole_body.cpp`) cover the solver against the Riccati
residual and closed-loop stability, the linearisation's physical signs,
parameter rejection, and closed-loop behaviour on the planar plant: recovery
from 0.15 rad, speed tracking then position hold, CoM-offset absorption by the
legs, lean into turns, height command bounds, stale command, IMU loss, fall
latch and engage logic, torque scaling and gravity feed-forward.

MuJoCo (`tools/drive_test.py`): with the ideal simulation the first parameter
set stood after the launch reset, drove at 0.3 m/s within about 0.1 m/s, held
position to a few cm, turned at the commanded 1.0 rad/s while driving with
8–10 mm of lean, spun at 1.15–1.19 rad/s and changed height between 0.150 and
0.197 m while balancing, with wheel efforts around 0.2 Nm and leg efforts under
0.8 Nm. During braking the legs shifted the wheels about 12 mm forward under
the body. The physical robot balanced and followed commands with that set, with
more oscillation and slower turning than the simulation; the latency emulation
reproduced the oscillation as an outright fall and led to a softer set, which
balances in both the ideal and the latency-emulating simulation.

The redesign of the yaw loop, the leg torque budget, the LQR weights and the
diagnostics (this document's current state) was validated the same way in
the identified latency-emulating simulation with the per-robot override
(`recordings/sim_wbc2_latency_2026-09-20.csv` against
`sim_identified2_2026-09-20.csv`): standing pitch RMS 0.015–0.020 rad against
0.023, 0.19–0.20 m/s for a 0.2 m/s command against a 0.24 overshoot, 0.40 for
0.4 m/s, yaw error while turning 0.18 against 0.20 rad/s, at the price of more
wheel activity at standstill (torque RMS 0.44 against 0.20 Nm, the firmer
slow loop against the simulated Coulomb friction). Two negative results from
the same campaign are recorded in `SIM_TO_REAL.md`: leg stiffness above about
400 N/m and the reaction feed-forward both fall in the emulation, and so does
the PID cascade that balances the robot, which is why the leg stiffness is a
hardware tuning step.

Not yet done: robot weighing, identification of the real CoM position,
latencies, tyre scrub and actuator friction, the leg stiffness on the robot,
and the hardware comparison run with this build.
