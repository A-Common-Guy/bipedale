# sandro_controllers

ROS 2 Jazzy `ament_cmake` controllers for Sandro.

## Driving balancer

`sandro_controllers/DrivingBalancerController` claims effort commands for six
joints, position and velocity state for those joints, and the ten standard
ros2_control IMU interfaces. Joint order is:

1. right inner, right outer, right wheel
2. left inner, left outer, left wheel

It receives `geometry_msgs/msg/TwistStamped` on `~/cmd_vel`; only `linear.x`
and `angular.z` are used. Receipt time, rather than the message header stamp,
drives the mandatory command watchdog. A stale command requests zero motion
while balance and position hold remain active.

Standard ros2_control IMU interfaces have no sample timestamp. The hardware
component must therefore publish non-finite IMU interface values when its own
sample watchdog expires. The controller immediately zeros both wheel efforts
for non-finite IMU data and for the latched fall state. This contract avoids
mistaking an unchanged, stationary quaternion for a stale sample.

Requested and effective motion, pitch, torque, ramp, age, and saturation mask
are published as `diagnostic_msgs/msg/DiagnosticArray` on `~/diagnostics`.
Parameters in `config/controllers.yaml` use SI units and can be changed
atomically while active; invalid signs, limits, geometry, or unreachable leg
height are rejected.

## Whole-body controller

`sandro_controllers/WholeBodyController` preserves the same six effort, joint
state, IMU, and `TwistStamped` interfaces and additionally reads `linear.z` as
a body height rate. One LQR over `[x, x_dot, pitch, pitch_rate,
leg_offset_command, leg_offset]` commands total wheel torque and the rate of
the commanded fore-aft wheel placement, with the legs modelled as a
first-order lag (`model.leg_lag_s`); its gains are solved from the reduced
planar model at several leg heights, checked for their input-delay margin,
and interpolated on the measured height. The legs run a Cartesian impedance with
gravity support through `J^T` (support protected from saturation, damping on
low-passed velocities, optional feed-forward of the wheel torque's reaction),
a model-based yaw loop on the wheel-differential and IMU rates turns the
robot, the body leans into turns by a differential leg height, and a slow
posture trim puts the axle under the centre of mass. Balancing engages only
when the body is upright and still; a fall while balancing latches every
actuator off.

The core (`whole_body_core.*`, `whole_body_model.*`, `lqr.*`) is ROS-free and
tested against a planar plant in `test/test_whole_body.cpp`. It depends on
Eigen. See `sandro_bringup/docs/WHOLE_BODY_CONTROLLER.md`.

## Commissioning controller

`sandro_controllers/TimedEffortController` accepts a
`std_msgs/msg/Float64MultiArray` on `~/commands`. The array follows its `joints`
parameter. `command_timeout_s` is mandatory and strictly positive. Missing,
stale, malformed, and non-finite commands result in zero effort; deactivation
also writes zero.

The balancers all claim the same six effort interfaces. Activate only one at
a time; `sandro_bringup/config/controllers.yaml` configures each of them.
