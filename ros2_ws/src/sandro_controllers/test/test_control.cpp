#include <cmath>
#include <limits>

#include "gtest/gtest.h"

#include "sandro_controllers/control_core.hpp"
#include "sandro_controllers/kinematics.hpp"
#include "sandro_controllers/lqr_core.hpp"

namespace sandro_controllers {
namespace {
constexpr double kPi = 3.14159265358979323846;

RobotState standing_state(const ControlParameters &parameters) {
  RobotState state;
  const auto right =
      inverse_kinematics(parameters.right_geometry.base / 2.0,
                         parameters.leg_height_m, parameters.right_geometry);
  const auto left =
      inverse_kinematics(parameters.left_geometry.base / 2.0,
                         parameters.leg_height_m, parameters.left_geometry);
  state.position[0] = (*right)[0];
  state.position[1] = (*right)[1];
  state.position[3] = (*left)[0];
  state.position[4] = (*left)[1];
  state.quaternion_w = 1.0;
  state.imu_age_s = 0.0;
  return state;
}

// The planar wheeled inverted pendulum that tools/lqr_design.py reduces the
// MJCF to, so the gains under test face the plant they were solved against.
struct PlanarWip {
  static constexpr double kRadius = 0.0742;
  static constexpr double kM11 = 3.43188;   // wheels plus body, seen at the axle
  static constexpr double kM12 = 0.397364;  // body mass times CoM height
  static constexpr double kM22 = 0.0776823; // body pitch inertia at the axle
  static constexpr double kGravity = 3.89816; // m * g * CoM height

  double position{};
  double velocity{};
  double pitch{};
  double pitch_rate{};

  void step(const double total_torque, const double dt) {
    const double determinant = kM11 * kM22 - kM12 * kM12;
    const double wheel_row = total_torque / kRadius;
    const double body_row = kGravity * pitch - total_torque;
    const double accel = (kM22 * wheel_row - kM12 * body_row) / determinant;
    const double pitch_accel =
        (kM11 * body_row - kM12 * wheel_row) / determinant;
    velocity += accel * dt;
    position += velocity * dt;
    pitch_rate += pitch_accel * dt;
    pitch += pitch_rate * dt;
  }

  void write_into(RobotState &state) const {
    // Encoders turn relative to the chassis, so the body pitch comes back out.
    const double wheel_angle = position / kRadius - pitch;
    const double wheel_rate = velocity / kRadius - pitch_rate;
    state.position[2] = wheel_angle;
    state.position[5] = wheel_angle;
    state.velocity[2] = wheel_rate;
    state.velocity[5] = wheel_rate;
    state.quaternion_w = std::cos(0.5 * pitch);
    state.quaternion_x = 0.0;
    state.quaternion_y = std::sin(0.5 * pitch);
    state.quaternion_z = 0.0;
    state.pitch_rate_rad_s = pitch_rate;
    state.imu_age_s = 0.0;
  }
};

ControlParameters lqr_control_parameters() {
  ControlParameters parameters;
  parameters.settle_s = 0.0;
  parameters.ramp_s = 0.0;
  parameters.wheel_radius_m = PlanarWip::kRadius;
  return parameters;
}

void set_pitch(RobotState &state, const double pitch) {
  state.quaternion_w = std::cos(pitch / 2.0);
  state.quaternion_x = 0.0;
  state.quaternion_y = std::sin(pitch / 2.0);
  state.quaternion_z = 0.0;
}

} // namespace

TEST(Kinematics, InverseForwardRoundTrip) {
  const LegGeometry geometry{0.09, 0.08, 0.13};
  const auto angles = inverse_kinematics(0.045, 0.16, geometry);
  ASSERT_TRUE(angles.has_value());
  const auto point = forward_kinematics((*angles)[0], (*angles)[1], geometry);
  ASSERT_TRUE(point.has_value());
  EXPECT_NEAR(point->x, 0.045, 1e-8);
  EXPECT_NEAR(point->y, 0.16, 1e-8);
}

TEST(Kinematics, RejectsUnreachableAndInvalidGeometry) {
  EXPECT_FALSE(inverse_kinematics(1.0, 1.0, {0.09, 0.08, 0.13}));
  EXPECT_FALSE(inverse_kinematics(0.0, 0.1, {0.09, -0.08, 0.13}));
}

TEST(Kinematics, JacobianMatchesFiniteDifference) {
  const LegGeometry geometry{0.09, 0.08, 0.13};
  const auto angles = inverse_kinematics(0.045, 0.16, geometry);
  ASSERT_TRUE(angles);
  const auto matrix = jacobian((*angles)[0], (*angles)[1], geometry);
  ASSERT_TRUE(matrix);
  constexpr double dt = 1e-5;
  constexpr double v1 = 0.3;
  constexpr double v2 = -0.2;
  const auto start = forward_kinematics((*angles)[0], (*angles)[1], geometry);
  const auto finish = forward_kinematics((*angles)[0] + v1 * dt,
                                         (*angles)[1] + v2 * dt, geometry);
  ASSERT_TRUE(start && finish);
  EXPECT_NEAR((*matrix)[0][0] * v1 + (*matrix)[0][1] * v2,
              (finish->x - start->x) / dt, 1e-5);
  EXPECT_NEAR((*matrix)[1][0] * v1 + (*matrix)[1][1] * v2,
              (finish->y - start->y) / dt, 1e-5);
}

TEST(Pitch, HasExpectedSignAndIgnoresWorldYaw) {
  const double pitch = 0.2;
  const double yaw = 1.3;
  const double cp = std::cos(pitch / 2.0);
  const double sp = std::sin(pitch / 2.0);
  const double cy = std::cos(yaw / 2.0);
  const double sy = std::sin(yaw / 2.0);
  EXPECT_NEAR(pitch_from_quaternion(cy * cp, -sy * sp, cy * sp, sy * cp), pitch,
              1e-8);
  EXPECT_TRUE(std::isnan(pitch_from_quaternion(0.0, 0.0, 0.0, 0.0)));
}

TEST(ControlCore, UsesMeasuredPeriodForBalanceTorqueSlew) {
  ControlParameters parameters;
  parameters.settle_s = 0.0;
  parameters.ramp_s = 0.0;
  parameters.velocity_kp_pitch_s_m = 0.0;
  parameters.velocity_ki_pitch_per_m = 0.0;
  parameters.yaw_kp_nm_s_rad = 0.0;
  parameters.balance_torque_rate_nm_s = 1.0;
  RobotState state = standing_state(parameters);
  set_pitch(state, 0.2);
  DrivingBalancerCore core;
  core.reset(state, parameters);
  const auto output = core.update(state, MotionCommand{}, parameters, 0.01);
  EXPECT_NEAR(output.effort_nm[2], 0.01, 1e-12);
  EXPECT_NEAR(output.effort_nm[5], 0.01, 1e-12);
}

TEST(ControlCore, StaleCommandBecomesZeroButBalanceRemainsActive) {
  ControlParameters parameters;
  parameters.settle_s = 0.0;
  parameters.ramp_s = 0.0;
  parameters.forward_accel_limit_m_s2 = 100.0;
  RobotState state = standing_state(parameters);
  DrivingBalancerCore core;
  core.reset(state, parameters);
  MotionCommand command{0.3, 0.5, parameters.command_timeout_s + 0.01};
  const auto output = core.update(state, command, parameters, 0.01);
  EXPECT_EQ(output.saturation_mask & SAT_COMMAND_STALE, SAT_COMMAND_STALE);
  EXPECT_DOUBLE_EQ(output.requested_forward_velocity_m_s, 0.3);
  EXPECT_DOUBLE_EQ(output.effective_forward_velocity_m_s, 0.0);
  EXPECT_DOUBLE_EQ(output.effective_yaw_rate_rad_s, 0.0);
}

TEST(ControlCore, StaleImuAndFallLatchZeroBothWheels) {
  ControlParameters parameters;
  parameters.settle_s = 0.0;
  parameters.ramp_s = 0.0;
  RobotState state = standing_state(parameters);
  DrivingBalancerCore core;
  core.reset(state, parameters);

  state.imu_age_s = parameters.imu_timeout_s + 0.01;
  state.velocity[2] = 5.0;
  state.velocity[5] = -5.0;
  auto output = core.update(state, MotionCommand{}, parameters, 0.01);
  EXPECT_DOUBLE_EQ(output.effort_nm[2], 0.0);
  EXPECT_DOUBLE_EQ(output.effort_nm[5], 0.0);
  EXPECT_EQ(output.saturation_mask & SAT_IMU_STALE, SAT_IMU_STALE);

  core.reset(state, parameters);
  state.imu_age_s = 0.0;
  set_pitch(state, parameters.fall_angle_rad + 0.1);
  core.update(state, MotionCommand{}, parameters, 0.01);
  set_pitch(state, 0.0);
  output = core.update(state, MotionCommand{}, parameters, 0.01);
  EXPECT_TRUE(output.fallen);
  EXPECT_DOUBLE_EQ(output.effort_nm[2], 0.0);
  EXPECT_DOUBLE_EQ(output.effort_nm[5], 0.0);
}

TEST(ControlCore, YawIsDifferentialAndTotalWheelEffortIsBounded) {
  ControlParameters parameters;
  parameters.settle_s = 0.0;
  parameters.ramp_s = 0.0;
  RobotState state = standing_state(parameters);
  DrivingBalancerCore core;
  core.reset(state, parameters);
  const auto output =
      core.update(state, MotionCommand{0.0, 1.0, 0.0}, parameters, 0.1);
  EXPECT_GT(output.effort_nm[2], output.effort_nm[5]);
  EXPECT_LE(std::abs(output.effort_nm[2]), parameters.wheel_torque_limit_nm);
  EXPECT_LE(std::abs(output.effort_nm[5]), parameters.wheel_torque_limit_nm);
}

TEST(ControlCore, AppliesExplicitWheelDamping) {
  ControlParameters parameters;
  parameters.settle_s = 0.0;
  parameters.ramp_s = 0.0;
  parameters.velocity_kp_pitch_s_m = 0.0;
  parameters.velocity_ki_pitch_per_m = 0.0;
  parameters.yaw_kp_nm_s_rad = 0.0;
  RobotState state = standing_state(parameters);
  state.velocity[2] = 1.0;
  state.velocity[5] = -1.0;
  DrivingBalancerCore core;
  core.reset(state, parameters);
  const auto output = core.update(state, MotionCommand{}, parameters, 0.01);
  EXPECT_NEAR(output.effort_nm[2], -parameters.wheel_damping_nm_s_rad, 1e-12);
  EXPECT_NEAR(output.effort_nm[5], parameters.wheel_damping_nm_s_rad, 1e-12);
}

TEST(ControlCore, LegImpedanceEffortIsLimited) {
  ControlParameters parameters;
  RobotState state = standing_state(parameters);
  state.position[0] -= 1.0;
  DrivingBalancerCore core;
  core.reset(standing_state(parameters), parameters);
  const auto output = core.update(state, MotionCommand{}, parameters, 0.1);
  EXPECT_LE(std::abs(output.effort_nm[0]), parameters.leg_torque_limit_nm);
  EXPECT_EQ(output.saturation_mask & SAT_LEG_TORQUE, SAT_LEG_TORQUE);
}

TEST(ControlCore, ForwardCommandGetsNoWheelFeedforward) {
  // Forward authority must come from the lean, not from pushing the wheels,
  // so the common-mode damping reference stays at zero even at full stick.
  ControlParameters parameters;
  parameters.settle_s = 0.0;
  parameters.ramp_s = 0.0;
  parameters.forward_accel_limit_m_s2 = 100.0;
  const double speed = 0.4;

  RobotState state = standing_state(parameters);
  DrivingBalancerCore core;
  core.reset(state, parameters);
  ControlOutput out;
  for (int i = 0; i < 50; ++i) {
    out = core.update(state, MotionCommand{speed, 0.0, 0.0}, parameters, 0.001);
  }
  ASSERT_NEAR(out.effective_forward_velocity_m_s, speed, 1e-9);
  EXPECT_NEAR(out.reference_right_wheel_rad_s, 0.0, 1e-12);
  EXPECT_NEAR(out.reference_left_wheel_rad_s, 0.0, 1e-12);
  // The lean is what carries the command through.
  EXPECT_GT(out.pitch_target_rad, 0.0);
}

TEST(ControlCore, WheelDampingReferenceIsDifferentialForYaw) {
  ControlParameters parameters;
  parameters.settle_s = 0.0;
  parameters.ramp_s = 0.0;
  RobotState state = standing_state(parameters);
  DrivingBalancerCore core;
  core.reset(state, parameters);
  const auto output =
      core.update(state, MotionCommand{0.0, 1.0, 0.0}, parameters, 0.001);
  EXPECT_GT(output.reference_right_wheel_rad_s, 0.0);
  EXPECT_LT(output.reference_left_wheel_rad_s, 0.0);
  EXPECT_NEAR(output.reference_right_wheel_rad_s +
                  output.reference_left_wheel_rad_s,
              0.0, 1e-12);
}

TEST(OuterLoop, ForwardVelocityIncludesChassisPitchRate) {
  ControlParameters parameters;
  parameters.settle_s = 0.0;
  parameters.ramp_s = 0.0;
  RobotState state = standing_state(parameters);
  DrivingBalancerCore core;
  core.reset(state, parameters);

  // Wheels planted on the ground while the body pitches: the joints read the
  // negative of the pitch rate, so ground speed is zero.
  state.pitch_rate_rad_s = 0.8;
  state.velocity[2] = -0.8;
  state.velocity[5] = -0.8;
  const auto output = core.update(state, MotionCommand{}, parameters, 0.001);
  EXPECT_NEAR(output.measured_forward_velocity_m_s, 0.0, 1e-12);
}

TEST(OuterLoop, PitchRateDoesNotErodeInnerLoopDamping) {
  ControlParameters parameters;
  parameters.settle_s = 0.0;
  parameters.ramp_s = 0.0;
  const double rate = 0.5;

  const auto damping = [&](const double kp) {
    ControlParameters p = parameters;
    p.velocity_kp_pitch_s_m = kp;
    RobotState state = standing_state(p);
    DrivingBalancerCore core;
    core.reset(state, p);
    state.pitch_rate_rad_s = rate;
    state.velocity[2] = -rate;
    state.velocity[5] = -rate;
    return core.update(state, MotionCommand{}, p, 0.001).raw_balance_torque_nm /
           rate;
  };

  // Balance damping must be the configured value whatever the outer-loop gain.
  EXPECT_NEAR(damping(0.0), parameters.pitch_kd_nm_s_rad, 1e-9);
  EXPECT_NEAR(damping(0.12), parameters.pitch_kd_nm_s_rad, 1e-9);
  EXPECT_NEAR(damping(0.5), parameters.pitch_kd_nm_s_rad, 1e-9);
}

TEST(OuterLoop, SpeedErrorLeansTowardTheRequestedDirection) {
  ControlParameters parameters;
  parameters.settle_s = 0.0;
  parameters.ramp_s = 0.0;
  parameters.forward_accel_limit_m_s2 = 100.0;
  RobotState state = standing_state(parameters);
  DrivingBalancerCore core;
  core.reset(state, parameters);

  const auto forward = core.update(state, MotionCommand{0.3, 0.0, 0.0},
                                   parameters, 0.001);
  EXPECT_GT(forward.pitch_target_rad, 0.0);

  core.reset(state, parameters);
  const auto reverse = core.update(state, MotionCommand{-0.3, 0.0, 0.0},
                                   parameters, 0.001);
  EXPECT_LT(reverse.pitch_target_rad, 0.0);
}

TEST(OuterLoop, IntegralBudgetIsInRadiansAndIndependentOfGain) {
  const auto settled_integral_offset = [](const double ki) {
    ControlParameters p;
    p.settle_s = 0.0;
    p.ramp_s = 0.0;
    p.velocity_kp_pitch_s_m = 0.0;  // isolate the integral contribution
    p.velocity_ki_pitch_per_m = ki;
    RobotState state = standing_state(p);
    DrivingBalancerCore core;
    core.reset(state, p);
    // Parked well behind the hold reference, so the error never goes away.
    state.position[2] = -2.0 / p.wheel_radius_m;
    state.position[5] = -2.0 / p.wheel_radius_m;
    ControlOutput out;
    for (int i = 0; i < 300 * 600; ++i) {
      out = core.update(state, MotionCommand{0.0, 0.0, 0.0}, p, 1.0 / 300.0);
    }
    return out.pitch_target_rad;
  };

  ControlParameters defaults;
  for (const double ki : {0.01, 0.02, 0.1}) {
    EXPECT_NEAR(settled_integral_offset(ki), defaults.velocity_integral_limit_rad,
                1e-6)
        << "ki=" << ki;
  }
}

TEST(OuterLoop, IntegralDoesNotWindBeyondThePitchOffsetClamp) {
  ControlParameters parameters;
  parameters.settle_s = 0.0;
  parameters.ramp_s = 0.0;
  parameters.velocity_integral_limit_rad = 1.0;  // deliberately over-generous
  RobotState state = standing_state(parameters);
  DrivingBalancerCore core;
  core.reset(state, parameters);
  state.position[2] = -5.0 / parameters.wheel_radius_m;
  state.position[5] = -5.0 / parameters.wheel_radius_m;
  for (int i = 0; i < 300 * 120; ++i) {
    core.update(state, MotionCommand{0.0, 0.0, 0.0}, parameters, 1.0 / 300.0);
  }

  // Once the error clears, the stored integral must not overshoot the clamp.
  state.position[2] = 0.0;
  state.position[5] = 0.0;
  const auto out =
      core.update(state, MotionCommand{0.0, 0.0, 0.0}, parameters, 1.0 / 300.0);
  EXPECT_LE(std::abs(out.pitch_target_rad),
            parameters.max_pitch_offset_rad + 1e-9);
}

TEST(Parameters, RejectsHiddenInvalidLimitsAndSigns) {
  ControlParameters parameters;
  EXPECT_TRUE(valid_parameters(parameters));
  parameters.balance_torque_sign = 0.0;
  EXPECT_FALSE(valid_parameters(parameters));
  parameters = ControlParameters{};
  parameters.command_timeout_s = 0.0;
  EXPECT_FALSE(valid_parameters(parameters));
  parameters = ControlParameters{};
  parameters.velocity_kp_pitch_s_m = -0.1;
  EXPECT_FALSE(valid_parameters(parameters));
  parameters = ControlParameters{};
  parameters.velocity_ki_pitch_per_m = -0.1;
  EXPECT_FALSE(valid_parameters(parameters));
  parameters = ControlParameters{};
  parameters.leg_height_m = 10.0;
  EXPECT_FALSE(valid_parameters(parameters));
}

TEST(LqrBalancer, RecoversFromAPitchDisturbance) {
  const ControlParameters parameters = lqr_control_parameters();
  LqrParameters gains;
  gains.velocity_integral_nm_s_m = 0.0;
  RobotState state = standing_state(parameters);
  PlanarWip plant;
  plant.pitch = 0.12;
  plant.write_into(state);

  LqrBalancerCore core;
  core.reset(state, parameters);
  const double dt = 0.001;
  for (int i = 0; i < 8000; ++i) {
    const auto out = core.update(state, MotionCommand{}, parameters, gains, dt);
    ASSERT_FALSE(out.fallen);
    plant.step(out.effort_nm[2] + out.effort_nm[5], dt);
    plant.write_into(state);
  }
  EXPECT_NEAR(plant.pitch, 0.0, 1e-3);
  EXPECT_NEAR(plant.velocity, 0.0, 1e-3);
  // The station-keeping gain brings it back to where it started, rather than
  // leaving the offset the recovery needed.
  EXPECT_NEAR(plant.position, 0.0, 0.02);
}

TEST(LqrBalancer, ReachesTheCommandedSpeed) {
  const ControlParameters parameters = lqr_control_parameters();
  LqrParameters gains;
  gains.velocity_integral_nm_s_m = 0.0;
  RobotState state = standing_state(parameters);
  PlanarWip plant;
  plant.write_into(state);

  LqrBalancerCore core;
  core.reset(state, parameters);
  const double dt = 0.001;
  const double speed = 0.3;
  double minimum_position = 0.0;
  for (int i = 0; i < 12000; ++i) {
    const auto out = core.update(state, MotionCommand{speed, 0.0, 0.0},
                                 parameters, gains, dt);
    ASSERT_FALSE(out.fallen);
    plant.step(out.effort_nm[2] + out.effort_nm[5], dt);
    plant.write_into(state);
    minimum_position = std::min(minimum_position, plant.position);
  }
  EXPECT_NEAR(plant.velocity, speed, 0.01);
  EXPECT_GT(plant.position, 1.0);
  // Non-minimum phase: it rolls backward first to tip into the direction of
  // travel. Nobody coded that; the coordinated gains produce it.
  EXPECT_LT(minimum_position, -1e-4);
}

TEST(LqrBalancer, CommandedSpeedCostsNoStandingTorque) {
  // The velocity gain is referenced to the command, so unlike the cascade's
  // wheel damping it applies no drag once the robot is at speed.
  const ControlParameters parameters = lqr_control_parameters();
  LqrParameters gains;
  gains.velocity_integral_nm_s_m = 0.0;
  const double speed = 0.3;

  RobotState state = standing_state(parameters);
  PlanarWip plant;
  plant.velocity = speed;
  plant.write_into(state);

  LqrBalancerCore core;
  core.reset(state, parameters);
  ControlOutput out;
  for (int i = 0; i < 2000; ++i) {
    out = core.update(state, MotionCommand{speed, 0.0, 0.0}, parameters, gains,
                      0.001);
  }
  ASSERT_NEAR(out.effective_forward_velocity_m_s, speed, 1e-9);
  EXPECT_NEAR(out.velocity_error_m_s, 0.0, 1e-9);
  EXPECT_NEAR(out.effort_nm[2], 0.0, 1e-9);
  EXPECT_NEAR(out.effort_nm[5], 0.0, 1e-9);
}

TEST(LqrBalancer, TrimIsBoundedAndReleasedOnFall) {
  ControlParameters parameters = lqr_control_parameters();
  LqrParameters gains;
  gains.velocity_integral_nm_s_m = 2.0;
  gains.velocity_integral_limit_nm = 0.4;
  RobotState state = standing_state(parameters);
  LqrBalancerCore core;
  core.reset(state, parameters);

  // A speed error the robot never acts on winds the trim to its bound, and no
  // further.
  ControlOutput out;
  for (int i = 0; i < 5000; ++i) {
    out = core.update(state, MotionCommand{0.4, 0.0, 0.0}, parameters, gains,
                      0.001);
  }
  EXPECT_LE(std::abs(out.velocity_integral_nm), gains.velocity_integral_limit_nm
                                                    + 1e-9);
  EXPECT_GT(std::abs(out.velocity_integral_nm), 0.0);

  set_pitch(state, 1.2);
  out = core.update(state, MotionCommand{}, parameters, gains, 0.001);
  EXPECT_TRUE(out.fallen);
  EXPECT_EQ(out.velocity_integral_nm, 0.0);
  EXPECT_EQ(out.effort_nm[2], 0.0);
  EXPECT_EQ(out.effort_nm[5], 0.0);
}

TEST(LqrBalancer, WaitsForUprightBeforeTakingTheWheels) {
  const ControlParameters parameters = lqr_control_parameters();
  const LqrParameters gains;
  RobotState state = standing_state(parameters);
  // Lying over, as it is after the legs collapse on startup. Note this is past
  // the engage angle but still short of fall_angle_rad, so the old behaviour
  // would have driven the wheels at full torque under a downed robot.
  set_pitch(state, 0.6);

  LqrBalancerCore core;
  core.reset(state, parameters);
  // Push a leg off its target so the impedance has something to pull against.
  state.position[0] += 0.1;
  ControlOutput out;
  for (int i = 0; i < 500; ++i) {
    out = core.update(state, MotionCommand{}, parameters, gains, 0.001);
  }
  EXPECT_FALSE(out.engaged);
  EXPECT_FALSE(out.fallen);
  EXPECT_EQ(out.effort_nm[2], 0.0);
  EXPECT_EQ(out.effort_nm[5], 0.0);
  // The legs are still held while it waits.
  EXPECT_GT(std::abs(out.effort_nm[0]), 0.0);

  // Stood back up, as the world reset does in simulation or an operator does
  // on the bench.
  set_pitch(state, 0.05);
  for (int i = 0; i < 500; ++i) {
    out = core.update(state, MotionCommand{}, parameters, gains, 0.001);
  }
  EXPECT_TRUE(out.engaged);
  EXPECT_GT(std::abs(out.effort_nm[2]), 0.0);
}

TEST(LqrBalancer, DoesNotLatchAFallItNeverEngagedFor) {
  const ControlParameters parameters = lqr_control_parameters();
  const LqrParameters gains;
  RobotState state = standing_state(parameters);
  set_pitch(state, 1.4); // well past fall_angle_rad, but never engaged
  LqrBalancerCore core;
  core.reset(state, parameters);
  ControlOutput out;
  for (int i = 0; i < 200; ++i) {
    out = core.update(state, MotionCommand{}, parameters, gains, 0.001);
  }
  EXPECT_FALSE(out.fallen);
  EXPECT_FALSE(out.engaged);

  // Stand it up and it still engages, rather than being latched out forever.
  set_pitch(state, 0.02);
  out = core.update(state, MotionCommand{}, parameters, gains, 0.001);
  EXPECT_TRUE(out.engaged);
  EXPECT_FALSE(out.fallen);
}

TEST(LqrBalancer, RejectsDestabilisingGains) {
  LqrParameters gains;
  EXPECT_TRUE(valid_lqr_parameters(gains));
  gains.pitch_nm_rad = -8.0;
  EXPECT_FALSE(valid_lqr_parameters(gains));
  gains = LqrParameters{};
  gains.pitch_rate_nm_s_rad = 0.0;
  EXPECT_FALSE(valid_lqr_parameters(gains));
  gains = LqrParameters{};
  gains.velocity_integral_limit_nm = -1.0;
  EXPECT_FALSE(valid_lqr_parameters(gains));
}

} // namespace sandro_controllers
