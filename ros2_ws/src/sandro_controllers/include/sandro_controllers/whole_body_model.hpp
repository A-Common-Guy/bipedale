#pragma once

// Reduced whole-body model of Sandro: two planar five-bar legs carrying a
// rigid body on two wheels. The legs are treated as position-controlled
// Cartesian actuators, so the fast balance dynamics reduce to a wheeled
// inverted pendulum whose centre of mass can be shifted fore-aft by the legs
// and whose height, and therefore pendulum length, follows the leg height.
//
// Coordinates (body frame: x forward, y left, z up; IMU frame):
//   x           forward position of the axle midpoint on the ground
//   pitch       body pitch, positive when the body leans forward
//   leg_offset  forward distance from the axle midpoint to where the body's
//               centre of mass would be with the legs at their neutral
//               fore-aft position; positive when the wheels are shifted
//               backward under the body. The legs follow their commanded
//               offset with a first-order lag (leg_lag_s), so the command
//               and the achieved offset are separate states.
// Planar five-bar coordinates per leg (kinematics.hpp): +x from the inner
// toward the outer pivot, +y down toward the wheel. The inner/outer motor
// names are mirrored between the sides, so planar +x is body-forward on the
// right leg and body-backward on the left leg (forward_sign).

#include "sandro_controllers/kinematics.hpp"
#include "sandro_controllers/lqr.hpp"

namespace sandro_controllers {

struct WholeBodyModel {
  LegGeometry right_geometry{0.09, 0.08, 0.13};
  LegGeometry left_geometry{0.09, 0.08, 0.13};
  // Body-x direction of planar leg +x, exactly +1 or -1.
  double right_forward_sign{1.0};
  double left_forward_sign{-1.0};
  // Which lower link carries the wheel motor's stator (exactly 0 or 1 for
  // the inner or the outer link of that leg's planar five-bar). The stator
  // reaction of the wheel torque acts on that link. On Sandro both wheels
  // hang from the front lower link, which is the outer link on the right leg
  // and the inner link on the left leg because the motor names are mirrored.
  double right_wheel_on_outer_link{1.0};
  double left_wheel_on_outer_link{0.0};
  // Everything the legs carry, i.e. total mass minus the two wheels.
  double body_mass_kg{3.284};
  // Per wheel. The inertia is about the axle and includes the reflected
  // rotor inertia of the geared drive.
  double wheel_mass_kg{0.0584};
  double wheel_inertia_kgm2{0.00033};
  double wheel_radius_m{0.0742};
  // Spacing of the two tyre contact patches.
  double track_width_m{0.304};
  // Yaw inertia of the whole robot about the vertical through its centre of
  // mass, including the wheels' spin inertia reflected through the track
  // (0.086 + 0.002 kg m^2 from the identified MJCF at the stand keyframe).
  double yaw_inertia_kgm2{0.09};
  // Time constant with which the measured axle follows its commanded
  // fore-aft offset: the overdamped leg impedance on the light unloaded wheel
  // responds in about kd / kp, plus stiction. 70 ms on the robot (fourth
  // hardware run, cross-correlation of leg_offset_command_m and
  // leg_offset_m); about 30 ms in the MuJoCo model. The regulator designed
  // without it asked the legs for a 72 ms mode and hunted at 1.9 Hz.
  double leg_lag_s{0.07};
  // Height of the body centre of mass above the axle as an affine function
  // of the planar leg height (pivot to axle): com = k * height + offset.
  double com_height_per_leg_height{0.654};
  double com_height_offset_m{0.0};
  // Body pitch inertia about its own centre of mass, affine in leg height.
  double pitch_inertia_kgm2{0.0295};
  double pitch_inertia_reference_height_m{0.1855};
  double pitch_inertia_per_m{0.15};
  double gravity_m_s2{9.81};
};

bool valid_model(const WholeBodyModel &model);

double com_height(const WholeBodyModel &model, double leg_height_m);
double pitch_inertia(const WholeBodyModel &model, double leg_height_m);

struct LinearModel {
  lqr::StateMatrix A{lqr::StateMatrix::Zero()};
  lqr::InputMatrix B{lqr::InputMatrix::Zero()};
};

// Continuous-time linearisation about the upright equilibrium at a given leg
// height. State [x, x_dot, pitch, pitch_rate, leg_offset_command,
// leg_offset], input [total wheel torque, leg_offset_command_rate]; the
// achieved offset follows the command with the model's leg_lag_s.
LinearModel linearize(const WholeBodyModel &model, double leg_height_m);

struct Attitude {
  double pitch_rad{};
  double roll_rad{};
  bool valid{};
};

// Gravity-referenced pitch and roll of a body whose frame is x forward, y
// left, z up. Both are independent of heading. Positive pitch leans forward
// (the x axis dips); positive roll drops the right side.
Attitude attitude_from_quaternion(double w, double x, double y, double z);

// Cartesian state of one five-bar leg.
struct LegState {
  Point2 position{};
  Point2 velocity{};
  Jacobian2 jacobian{};
  // d(angle of the lower link that carries the wheel motor) / d(joints).
  JointPair wheel_link_jacobian{};
  bool valid{};
};

LegState leg_state(double theta_inner, double theta_outer, double omega_inner,
                   double omega_outer, const LegGeometry &geometry,
                   bool wheel_on_outer_link = true);

// Whether (x, y) is safely inside the leg workspace: reachable, away from the
// stretched singularity, and within the joint range on both motors.
bool inside_leg_workspace(double x, double y, const LegGeometry &geometry,
                          double joint_limit_rad, double stretch_margin = 0.97);

} // namespace sandro_controllers
