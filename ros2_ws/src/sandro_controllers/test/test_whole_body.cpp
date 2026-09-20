#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"

#include "sandro_controllers/kinematics.hpp"
#include "sandro_controllers/lqr.hpp"
#include "sandro_controllers/whole_body_core.hpp"
#include "sandro_controllers/whole_body_model.hpp"

namespace sandro_controllers {
namespace {

constexpr double kPeriod = 1.0 / 300.0;

WholeBodyParameters test_parameters() {
  WholeBodyParameters p;
  p.settle_s = 0.0;
  p.ramp_s = 0.2;
  p.gain_table_points = 4;
  return p;
}

WholeBodySettings test_settings(const WholeBodyParameters &p = test_parameters()) {
  WholeBodySettings settings;
  std::string reason;
  const bool ok = build_whole_body_settings(p, kPeriod, settings, &reason);
  EXPECT_TRUE(ok) << reason;
  return settings;
}

// Planar plant the core is designed against, plus the unmodelled effects the
// design must tolerate: a centre of mass that is not where the legs think it
// is, and legs that follow their Cartesian references with a lag.
struct PlanarPlant {
  WholeBodyModel model{};
  double com_offset_m{};
  double x{};
  double v{};
  double pitch{};
  double pitch_rate{};
  double leg_offset{};
  double leg_height{0.17};
  double leg_lag_s{0.03};
  double rolling_resistance_nm{}; // Coulomb drag at the contact, as wheel torque
  // Axle deflection per newton-metre of total wheel torque, the way soft
  // legs let the ground force and the stator reaction move the wheels
  // under the body (0.03 m/Nm was the shipped 300 N/m impedance).
  double leg_compliance_m_per_nm{};
  double yaw_rate{};
  std::array<double, 4> previous_angles{};
  bool have_previous{};

  void step(const double total_torque, const double leg_offset_target,
            const double height_target, const double dt) {
    // A forward torque pushes the axle forward under the body, i.e. the
    // wheels move ahead of where the legs placed them: a negative offset.
    const double target =
        leg_offset_target - leg_compliance_m_per_nm * total_torque;
    leg_offset += (target - leg_offset) * std::min(1.0, dt / leg_lag_s);
    leg_height += (height_target - leg_height) * std::min(1.0, dt / leg_lag_s);
    const double M = model.body_mass_kg;
    const double L = com_height(model, leg_height);
    const double I = pitch_inertia(model, leg_height);
    const double r = model.wheel_radius_m;
    const double g = model.gravity_m_s2;
    const double d = com_offset_m + leg_offset;
    const double a = M + 2.0 * model.wheel_mass_kg +
                     2.0 * model.wheel_inertia_kgm2 / (r * r);
    const double c1 = M * (L * std::cos(pitch) - d * std::sin(pitch));
    const double c2 = I + M * (L * L + d * d);
    const double lever = L * std::sin(pitch) + d * std::cos(pitch);
    // Rolling resistance acts at the contact, so it slows the robot without
    // reacting on the body the way motor torque does.
    const double drag = rolling_resistance_nm * std::tanh(v / 0.02);
    const double rhs1 = (total_torque - drag) / r + M * pitch_rate * pitch_rate * lever;
    const double rhs2 = M * g * lever - total_torque;
    const double det = a * c2 - c1 * c1;
    const double accel = (c2 * rhs1 - c1 * rhs2) / det;
    const double pitch_accel = (a * rhs2 - c1 * rhs1) / det;
    v += accel * dt;
    x += v * dt;
    pitch_rate += pitch_accel * dt;
    pitch += pitch_rate * dt;
  }

  WholeBodyState observe(const WholeBodyParameters &p, const double dt) {
    WholeBodyState s;
    const double r = model.wheel_radius_m;
    // Encoders turn relative to the chassis.
    for (const std::size_t wheel : {2U, 5U}) {
      s.position[wheel] = x / r - pitch;
      s.velocity[wheel] = v / r - pitch_rate;
    }
    s.quaternion_w = std::cos(0.5 * pitch);
    s.quaternion_y = std::sin(0.5 * pitch);
    s.angular_velocity_rad_s = {0.0, pitch_rate, yaw_rate};
    s.imu_age_s = 0.0;
    const std::array<const LegGeometry *, 2> geometry{&model.right_geometry,
                                                      &model.left_geometry};
    const std::array<double, 2> forward_sign{model.right_forward_sign,
                                             model.left_forward_sign};
    std::array<double, 4> angles{};
    for (std::size_t side = 0; side < 2; ++side) {
      const double x_planar = p.fore_aft_neutral_m - forward_sign[side] * leg_offset;
      const auto ik = inverse_kinematics(x_planar, leg_height, *geometry[side]);
      if (!ik) {
        throw std::runtime_error("plant left the leg workspace");
      }
      angles[2 * side] = (*ik)[0];
      angles[2 * side + 1] = (*ik)[1];
    }
    const std::array<std::size_t, 4> index{0, 1, 3, 4};
    for (std::size_t k = 0; k < 4; ++k) {
      s.position[index[k]] = angles[k];
      s.velocity[index[k]] =
          have_previous ? (angles[k] - previous_angles[k]) / dt : 0.0;
    }
    previous_angles = angles;
    have_previous = true;
    return s;
  }
};

template <typename CommandFn>
WholeBodyOutput run(PlanarPlant &plant, WholeBodyCore &core,
                    const WholeBodySettings &settings, const double seconds,
                    CommandFn command_at) {
  WholeBodyOutput out;
  const int steps = static_cast<int>(std::lround(seconds / kPeriod));
  for (int i = 0; i < steps; ++i) {
    const double t = i * kPeriod;
    const WholeBodyState state = plant.observe(settings.parameters, kPeriod);
    out = core.update(state, command_at(t), settings, kPeriod);
    const double total = out.effort_nm[2] + out.effort_nm[5];
    for (int sub = 0; sub < 4; ++sub) {
      plant.step(total, out.leg_offset_command_m, out.height_reference_m,
                 kPeriod / 4.0);
    }
    if (std::abs(plant.pitch) > 1.0) {
      break;
    }
  }
  return out;
}

WholeBodyCommand still(double) { return WholeBodyCommand{0.0, 0.0, 0.0, 0.0}; }

} // namespace

TEST(Lqr, MatrixExponentialMatchesRotation) {
  Eigen::Matrix<double, 2, 2> generator;
  const double angle = 0.7;
  generator << 0.0, -angle, angle, 0.0;
  const auto result = lqr::expm<2>(generator);
  EXPECT_NEAR(result(0, 0), std::cos(angle), 1e-12);
  EXPECT_NEAR(result(0, 1), -std::sin(angle), 1e-12);
  EXPECT_NEAR(result(1, 0), std::sin(angle), 1e-12);
  EXPECT_NEAR(result(1, 1), std::cos(angle), 1e-12);
}

TEST(Lqr, DiscretisationReducesToEulerForSmallSteps) {
  const LinearModel model = linearize(WholeBodyModel{}, 0.17);
  const double dt = 1e-4;
  const auto discrete = lqr::discretize(model.A, model.B, dt);
  const lqr::StateMatrix euler_a = lqr::StateMatrix::Identity() + model.A * dt;
  const lqr::InputMatrix euler_b = model.B * dt;
  EXPECT_LT((discrete.A - euler_a).cwiseAbs().maxCoeff(), 1e-5);
  EXPECT_LT((discrete.B - euler_b).cwiseAbs().maxCoeff(), 1e-5);
}

TEST(Lqr, RiccatiSolutionIsStabilisingAndSelfConsistent) {
  const WholeBodyParameters p;
  const LinearModel model = linearize(p.model, 0.17);
  const auto discrete = lqr::discretize(model.A, model.B, kPeriod);
  lqr::StateMatrix Q = lqr::StateMatrix::Zero();
  lqr::InputWeight R = lqr::InputWeight::Zero();
  for (int i = 0; i < 5; ++i) {
    Q(i, i) = p.state_weights[static_cast<std::size_t>(i)];
  }
  for (int i = 0; i < 2; ++i) {
    R(i, i) = p.effort_weights[static_cast<std::size_t>(i)];
  }
  const auto solution = lqr::solve_dare(discrete.A, discrete.B, Q, R);
  ASSERT_TRUE(solution.converged);
  EXPECT_LT(solution.iterations, 100);
  EXPECT_LT(solution.spectral_radius, 1.0);
  // Riccati residual.
  const lqr::InputWeight S = R + discrete.B.transpose() * solution.P * discrete.B;
  const lqr::StateMatrix residual =
      Q + discrete.A.transpose() * solution.P * discrete.A -
      discrete.A.transpose() * solution.P * discrete.B * S.inverse() *
          discrete.B.transpose() * solution.P * discrete.A -
      solution.P;
  EXPECT_LT(residual.cwiseAbs().maxCoeff(),
            1e-8 * solution.P.cwiseAbs().maxCoeff());
  // Physical sign of the gains with u = -K x: leaning forward needs forward
  // wheel torque, and the legs move the axle forward under the body.
  EXPECT_LT(solution.K(0, 2), 0.0);
  EXPECT_GT(solution.K(1, 2), 0.0);
  // The open loop is unstable; the pendulum pole sits near 12 rad/s.
  EXPECT_GT(lqr::spectral_radius(discrete.A), 1.0);
}

TEST(WholeBodyModel, LinearisationMatchesInvertedPendulumIntuition) {
  const WholeBodyModel m;
  const LinearModel lin = linearize(m, 0.1855);
  // Gravity destabilises pitch and a leg offset acts like pitch scaled by the
  // centre-of-mass height.
  EXPECT_GT(lin.A(3, 2), 100.0);
  EXPECT_NEAR(lin.A(3, 4) * com_height(m, 0.1855), lin.A(3, 2), 1e-9);
  // Forward torque accelerates the wheels forward and pitches the body back.
  EXPECT_GT(lin.B(1, 0), 0.0);
  EXPECT_LT(lin.B(3, 0), 0.0);
  EXPECT_NEAR(com_height(m, 0.1855), 0.121, 0.003);
}

TEST(WholeBodyModel, AttitudeSignsAndHeadingInvariance) {
  const double pitch = 0.2;
  const double roll = -0.1;
  const double yaw = 1.3;
  // ZYX composition: q = qz(yaw) * qy(pitch) * qx(roll).
  const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
  const double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
  const double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
  const double w = cy * cp * cr + sy * sp * sr;
  const double x = cy * cp * sr - sy * sp * cr;
  const double y = cy * sp * cr + sy * cp * sr;
  const double z = sy * cp * cr - cy * sp * sr;
  const Attitude attitude = attitude_from_quaternion(w, x, y, z);
  ASSERT_TRUE(attitude.valid);
  EXPECT_NEAR(attitude.pitch_rad, pitch, 1e-9);
  EXPECT_NEAR(attitude.roll_rad, roll, 1e-9);
  EXPECT_FALSE(attitude_from_quaternion(0.0, 0.0, 0.0, 0.0).valid);
}

TEST(WholeBodySettings, BuildsScheduleAndRejectsInvalidSets) {
  const WholeBodySettings settings = test_settings();
  ASSERT_TRUE(settings.valid);
  ASSERT_EQ(settings.gains.count, 4);
  // Gains change smoothly with height and interpolate between table points.
  const auto low = settings.gains.at(settings.parameters.gain_table_min_height_m);
  const auto high = settings.gains.at(settings.parameters.gain_table_max_height_m);
  const auto mid = settings.gains.at(
      0.5 * (settings.parameters.gain_table_min_height_m +
             settings.parameters.gain_table_max_height_m));
  EXPECT_NE(low[0][2], high[0][2]);
  EXPECT_GT(mid[0][2], std::min(low[0][2], high[0][2]) - 1e-9);
  EXPECT_LT(mid[0][2], std::max(low[0][2], high[0][2]) + 1e-9);

  WholeBodyParameters bad = test_parameters();
  bad.wheel_torque_sign = 0.5;
  std::string reason;
  EXPECT_FALSE(valid_whole_body_parameters(bad, &reason));
  EXPECT_FALSE(reason.empty());
  bad = test_parameters();
  bad.height_max_m = 0.205; // stretched singularity
  EXPECT_FALSE(valid_whole_body_parameters(bad));
  bad = test_parameters();
  bad.state_weights[2] = 0.0;
  EXPECT_FALSE(valid_whole_body_parameters(bad));
  bad = test_parameters();
  bad.model.body_mass_kg = -1.0;
  EXPECT_FALSE(valid_whole_body_parameters(bad));
  bad = test_parameters();
  bad.command_timeout_s = 0.0;
  EXPECT_FALSE(valid_whole_body_parameters(bad));
  bad = test_parameters();
  bad.leg_offset_limit_m = 0.06;
  EXPECT_FALSE(valid_whole_body_parameters(bad));
}

TEST(WholeBodyCore, RecoversFromPitchDisturbanceWithoutFalling) {
  const WholeBodySettings settings = test_settings();
  PlanarPlant plant;
  plant.pitch = 0.15;
  WholeBodyCore core;
  core.reset(plant.observe(settings.parameters, kPeriod), settings);
  const auto out = run(plant, core, settings, 3.0, still);
  EXPECT_FALSE(out.fallen);
  EXPECT_TRUE(out.engaged);
  EXPECT_NEAR(plant.pitch, 0.0, 0.02);
  EXPECT_NEAR(plant.v, 0.0, 0.05);
  EXPECT_LT(std::abs(plant.x), 0.5);
}

TEST(WholeBodyCore, TracksSpeedThenHoldsPosition) {
  const WholeBodySettings settings = test_settings();
  PlanarPlant plant;
  WholeBodyCore core;
  core.reset(plant.observe(settings.parameters, kPeriod), settings);
  const double speed = 0.3;
  auto drive = run(plant, core, settings, 4.0, [speed](double) {
    return WholeBodyCommand{speed, 0.0, 0.0, 0.0};
  });
  EXPECT_FALSE(drive.fallen);
  EXPECT_NEAR(plant.v, speed, 0.05);
  EXPECT_GT(plant.x, 0.6);
  const double x_at_release = plant.x;
  const auto hold = run(plant, core, settings, 4.0, still);
  EXPECT_FALSE(hold.fallen);
  EXPECT_NEAR(plant.v, 0.0, 0.05);
  // Releasing the stick anchors the hold where the robot is once the speed
  // command has slewed to zero: about the stopping distance past the
  // release point, never a catch-up toward where the command would have
  // taken it.
  EXPECT_LT(std::abs(plant.x - x_at_release), 0.25);
  EXPECT_NEAR(hold.position_m - hold.position_reference_m, 0.0, 0.05);
}

TEST(WholeBodyCore, ReleasingTheStickHoldsHereInsteadOfCatchingUp) {
  // The physical failure mode: friction holds the robot back and the legs
  // have no fore-aft range left to lean against it, so the speed loop
  // saturates its position error.
  WholeBodyParameters p = test_parameters();
  p.position_error_limit_m = 0.3;
  p.leg_offset_limit_m = 0.005;
  const WholeBodySettings settings = test_settings(p);
  PlanarPlant plant;
  plant.rolling_resistance_nm = 0.4; // about 0.2 Nm per wheel, as measured
  WholeBodyCore core;
  core.reset(plant.observe(settings.parameters, kPeriod), settings);
  const auto drive = run(plant, core, settings, 4.0, [](double) {
    return WholeBodyCommand{0.4, 0.0, 0.0, 0.0};
  });
  EXPECT_FALSE(drive.fallen);
  EXPECT_LT(plant.v, 0.35);
  EXPECT_NE(drive.status & WBC_POSITION_ERROR_LIMIT, 0U);
  EXPECT_GT(drive.position_reference_m - drive.position_m, 0.25);
  const double x_at_release = plant.x;
  const auto hold = run(plant, core, settings, 3.0, still);
  EXPECT_FALSE(hold.fallen);
  EXPECT_NEAR(plant.v, 0.0, 0.05);
  EXPECT_LT(std::abs(plant.x - x_at_release), 0.2);
  EXPECT_EQ(hold.status & WBC_POSITION_ERROR_LIMIT, 0U);
  EXPECT_NEAR(hold.position_m - hold.position_reference_m, 0.0, 0.05);
}

TEST(WholeBodyCore, LegRangeRestoresSpeedTrackingAgainstRollingResistance) {
  // Same drag, legs free: the LQR's leg channel puts the centre of mass
  // ahead of the axle and gravity supplies the steady push, so the speed
  // tracks without a body lean. This is why the identified centre of mass
  // goes into the neutral offset: pinned legs are what made the robot creep.
  WholeBodyParameters p = test_parameters();
  p.position_error_limit_m = 0.3;
  const WholeBodySettings settings = test_settings(p);
  PlanarPlant plant;
  plant.rolling_resistance_nm = 0.4;
  WholeBodyCore core;
  core.reset(plant.observe(settings.parameters, kPeriod), settings);
  const auto drive = run(plant, core, settings, 4.0, [](double) {
    return WholeBodyCommand{0.4, 0.0, 0.0, 0.0};
  });
  EXPECT_FALSE(drive.fallen);
  EXPECT_NEAR(plant.v, 0.4, 0.08);
  EXPECT_EQ(drive.status & WBC_POSITION_ERROR_LIMIT, 0U);
  EXPECT_LT(std::abs(plant.pitch), 0.05); // 0.067 rad with the legs pinned
  EXPECT_GT(plant.leg_offset, 0.005);
}

TEST(WholeBodyCore, TrimIgnoresALeanHeldAgainstWheelStiction) {
  // The robot is still and its body leans 0.03 rad forward while the wheels
  // hold about 0.3 Nm against stiction (the third hardware run). That lean is
  // friction, not mass: the trim must not move. The same lean with the wheel
  // torque near zero is a mass offset and is learnt.
  const WholeBodySettings settings = test_settings();
  PlanarPlant plant;
  WholeBodyCore core;
  core.reset(plant.observe(settings.parameters, kPeriod), settings);
  run(plant, core, settings, 1.0, still); // engage on the level plant
  plant.pitch = 0.03;
  WholeBodyOutput out;
  for (int i = 0; i < 600; ++i) { // 2 s frozen: wheels stuck, body leaning
    out = core.update(plant.observe(settings.parameters, kPeriod), still(0.0),
                      settings, kPeriod);
  }
  EXPECT_GT(std::abs(out.balance_torque_nm), settings.parameters.trim_torque_deadband_nm);
  EXPECT_NEAR(out.trim_m, 0.0, 1e-3);
}

TEST(WholeBodyCore, LegsAbsorbAnUnmodelledCentreOfMassOffset) {
  const WholeBodySettings settings = test_settings();
  PlanarPlant plant;
  plant.com_offset_m = 0.01;
  WholeBodyCore core;
  core.reset(plant.observe(settings.parameters, kPeriod), settings);
  const auto out = run(plant, core, settings, 15.0, still);
  EXPECT_FALSE(out.fallen);
  // The body ends level and the wheels have moved forward under the CoM.
  EXPECT_NEAR(plant.pitch, 0.0, 0.01);
  EXPECT_NEAR(out.leg_offset_command_m, -plant.com_offset_m, 0.004);
  EXPECT_NEAR(out.trim_m, -plant.com_offset_m, 0.004);
}

TEST(WholeBodyCore, LeansIntoTurnsAndSplitsWheelTorque) {
  const WholeBodySettings settings = test_settings();
  PlanarPlant plant;
  WholeBodyCore core;
  core.reset(plant.observe(settings.parameters, kPeriod), settings);
  const auto out = run(plant, core, settings, 3.0, [](double) {
    return WholeBodyCommand{0.5, 1.0, 0.0, 0.0};
  });
  EXPECT_FALSE(out.fallen);
  // Left turn at speed: lean left, i.e. left leg shorter than the right.
  EXPECT_LT(out.roll_reference_rad, -0.03);
  EXPECT_LT(out.height_differential_m, -0.005);
  EXPECT_LT(out.leg_reference_m[1].y, out.leg_reference_m[0].y);
  // The plant reports no yaw rate, so the yaw task pushes the right wheel.
  EXPECT_GT(out.yaw_torque_nm, 0.0);
  EXPECT_GT(out.effort_nm[2] - out.effort_nm[5], 0.0);
}

TEST(WholeBodyCore, HeightCommandIsBoundedAndSlewed) {
  const WholeBodySettings settings = test_settings();
  const auto &p = settings.parameters;
  PlanarPlant plant;
  WholeBodyCore core;
  core.reset(plant.observe(p, kPeriod), settings);
  const auto up = run(plant, core, settings, 3.0, [](double) {
    return WholeBodyCommand{0.0, 0.0, 1.0, 0.0}; // clipped to max rate
  });
  EXPECT_FALSE(up.fallen);
  EXPECT_NEAR(up.height_reference_m, p.height_max_m, 1e-6);
  EXPECT_NE(up.status & WBC_HEIGHT_LIMIT, 0U);
  EXPECT_NEAR(plant.leg_height, p.height_max_m, 0.005);
  const auto down = run(plant, core, settings, 0.5, [](double) {
    return WholeBodyCommand{0.0, 0.0, -1.0, 0.0};
  });
  // 0.5 s at the height slew limit.
  EXPECT_NEAR(down.height_reference_m, p.height_max_m - 0.5 * p.height_rate_m_s,
              0.003);
  EXPECT_FALSE(down.fallen);
}

TEST(WholeBodyCore, StaleCommandStopsDrivingButKeepsBalancing) {
  const WholeBodySettings settings = test_settings();
  PlanarPlant plant;
  WholeBodyCore core;
  core.reset(plant.observe(settings.parameters, kPeriod), settings);
  const auto out = run(plant, core, settings, 3.0, [&settings](double) {
    return WholeBodyCommand{0.4, 0.5, 0.0,
                            settings.parameters.command_timeout_s + 0.1};
  });
  EXPECT_NE(out.status & WBC_COMMAND_STALE, 0U);
  EXPECT_DOUBLE_EQ(out.velocity_reference_m_s, 0.0);
  EXPECT_DOUBLE_EQ(out.yaw_rate_reference_rad_s, 0.0);
  EXPECT_FALSE(out.fallen);
  EXPECT_NEAR(plant.v, 0.0, 0.05);
}

TEST(WholeBodyCore, ImuLossStopsWheelsButKeepsStance) {
  const WholeBodySettings settings = test_settings();
  PlanarPlant plant;
  WholeBodyCore core;
  core.reset(plant.observe(settings.parameters, kPeriod), settings);
  run(plant, core, settings, 1.0, still);
  WholeBodyState state = plant.observe(settings.parameters, kPeriod);
  state.quaternion_w = std::numeric_limits<double>::quiet_NaN();
  const auto out = core.update(state, WholeBodyCommand{}, settings, kPeriod);
  EXPECT_NE(out.status & WBC_IMU_STALE, 0U);
  EXPECT_DOUBLE_EQ(out.effort_nm[2], 0.0);
  EXPECT_DOUBLE_EQ(out.effort_nm[5], 0.0);
  EXPECT_FALSE(out.engaged);
  // Legs still carry the body (gravity support points down = positive y).
  EXPECT_TRUE(out.effort_nm[0] != 0.0 || out.effort_nm[1] != 0.0);
  EXPECT_TRUE(out.effort_nm[3] != 0.0 || out.effort_nm[4] != 0.0);
}

TEST(WholeBodyCore, FallWhileBalancingLatchesEveryActuatorUntilReset) {
  const WholeBodySettings settings = test_settings();
  PlanarPlant plant;
  WholeBodyCore core;
  core.reset(plant.observe(settings.parameters, kPeriod), settings);
  const auto balancing = run(plant, core, settings, 1.0, still);
  ASSERT_TRUE(balancing.engaged);
  WholeBodyState state = plant.observe(settings.parameters, kPeriod);
  state.quaternion_w = std::cos(0.5 * 1.2);
  state.quaternion_y = std::sin(0.5 * 1.2);
  core.update(state, WholeBodyCommand{}, settings, kPeriod);
  const WholeBodyState upright = plant.observe(settings.parameters, kPeriod);
  const auto out = core.update(upright, WholeBodyCommand{}, settings, kPeriod);
  EXPECT_TRUE(out.fallen);
  EXPECT_NE(out.status & WBC_FALL_LATCH, 0U);
  for (const double effort : out.effort_nm) {
    EXPECT_DOUBLE_EQ(effort, 0.0);
  }
  core.reset(upright, settings);
  const auto after = core.update(upright, WholeBodyCommand{}, settings, kPeriod);
  EXPECT_FALSE(after.fallen);
}

TEST(WholeBodyCore, LyingDownAtActivationWaitsInsteadOfLatching) {
  // The operator activates, then lifts the robot upright; in simulation the
  // world is reset after activation. Both leave the body far from upright at
  // first, which must hold the stance and then engage, never latch.
  const WholeBodySettings settings = test_settings();
  PlanarPlant plant;
  WholeBodyCore core;
  WholeBodyState state = plant.observe(settings.parameters, kPeriod);
  state.quaternion_w = std::cos(0.5 * 1.3);
  state.quaternion_y = std::sin(0.5 * 1.3);
  core.reset(state, settings);
  WholeBodyOutput out;
  for (int i = 0; i < 300; ++i) {
    out = core.update(state, WholeBodyCommand{}, settings, kPeriod);
  }
  EXPECT_FALSE(out.fallen);
  EXPECT_FALSE(out.engaged);
  EXPECT_DOUBLE_EQ(out.effort_nm[2], 0.0);
  EXPECT_DOUBLE_EQ(out.effort_nm[5], 0.0);
  EXPECT_NE(out.effort_nm[0], 0.0);
  // The state jumps to upright with a different leg pose and wheel angle.
  plant.leg_height = 0.185;
  plant.x = 0.4;
  WholeBodyState upright = plant.observe(settings.parameters, kPeriod);
  upright.velocity = {};
  out = core.update(upright, WholeBodyCommand{}, settings, kPeriod);
  EXPECT_TRUE(out.engaged);
  // References re-anchor to the measured state so nothing steps (the height
  // reference then moves one slew step toward the target).
  EXPECT_NEAR(out.height_reference_m, 0.185,
              settings.parameters.height_rate_m_s * kPeriod + 1e-9);
  EXPECT_NEAR(out.position_reference_m, out.position_m, 1e-9);
  const auto settled = run(plant, core, settings, 3.0, still);
  EXPECT_FALSE(settled.fallen);
  EXPECT_NEAR(plant.pitch, 0.0, 0.02);
}

TEST(WholeBodyCore, EngagesOnlyWhenUprightAndStill) {
  const WholeBodySettings settings = test_settings();
  PlanarPlant plant;
  plant.pitch = 0.5; // held by the operator, not yet upright
  WholeBodyCore core;
  core.reset(plant.observe(settings.parameters, kPeriod), settings);
  WholeBodyState state = plant.observe(settings.parameters, kPeriod);
  auto out = core.update(state, WholeBodyCommand{}, settings, kPeriod);
  EXPECT_FALSE(out.engaged);
  EXPECT_NE(out.status & WBC_SETTLING, 0U);
  EXPECT_DOUBLE_EQ(out.effort_nm[2], 0.0);
  EXPECT_DOUBLE_EQ(out.effort_nm[5], 0.0);
  EXPECT_NE(out.effort_nm[0], 0.0); // legs hold the stance meanwhile
  plant.pitch = 0.02;
  plant.pitch_rate = 0.0;
  state = plant.observe(settings.parameters, kPeriod);
  out = core.update(state, WholeBodyCommand{}, settings, kPeriod);
  EXPECT_TRUE(out.engaged);
}

TEST(WholeBodyCore, DoesNotEngageWithALegOutsideItsWorkspace) {
  // A wrong encoder zero or a multi-turn wrap at power-on puts a leg reading
  // outside the five-bar workspace; the wheels must stay off rather than
  // balance on a leg the controller cannot see.
  const WholeBodySettings settings = test_settings();
  PlanarPlant plant;
  WholeBodyCore core;
  WholeBodyState state = plant.observe(settings.parameters, kPeriod);
  state.position[0] += 2.0 * 3.14159265358979323846; // one wrapped output turn
  core.reset(state, settings);
  WholeBodyOutput out;
  for (int i = 0; i < 300; ++i) {
    out = core.update(state, WholeBodyCommand{}, settings, kPeriod);
  }
  EXPECT_FALSE(out.engaged);
  EXPECT_NE(out.status & WBC_LEG_KINEMATICS, 0U);
  EXPECT_DOUBLE_EQ(out.effort_nm[2], 0.0);
  EXPECT_DOUBLE_EQ(out.effort_nm[5], 0.0);
  // The readable leg still holds its stance; the unreadable one is damped only.
  EXPECT_NE(out.effort_nm[3], 0.0);
  EXPECT_DOUBLE_EQ(out.effort_nm[0], 0.0);
  // Once the reading is sane again, engagement proceeds normally.
  const WholeBodyState sane = plant.observe(settings.parameters, kPeriod);
  out = core.update(sane, WholeBodyCommand{}, settings, kPeriod);
  EXPECT_TRUE(out.engaged);
}

TEST(WholeBodyCore, NeutralOffsetMovesBothAxlesTheSameWayInTheBody) {
  // A body-frame offset must move the right axle and the left axle in the
  // same body direction, which is opposite planar directions because the
  // inner/outer naming is mirrored.
  WholeBodyParameters p = test_parameters();
  p.fore_aft_neutral_offset_m = -0.02;  // real CoM 20 mm behind the neutral
  p.height_max_m = 0.175;  // the shifted front pivot nears full stretch above
  const WholeBodySettings settings = test_settings(p);
  PlanarPlant plant;
  WholeBodyCore core;
  // Standing at the unshifted CAD neutral: the wheels are 20 mm ahead of the
  // new neutral, and activation anchors the references to the measured pose
  // so nothing steps.
  WholeBodyState state = plant.observe(p, kPeriod);
  core.reset(state, settings);
  auto out = core.update(state, WholeBodyCommand{}, settings, kPeriod);
  EXPECT_NEAR(out.leg_offset_m, -0.02, 1e-6);
  EXPECT_NEAR(out.leg_reference_m[0].x, 0.045, 1e-6);
  EXPECT_NEAR(out.leg_reference_m[1].x, 0.045, 1e-6);
  // Standing exactly at the shifted neutral (plant offset 20 mm backward):
  // right planar x smaller (body-forward axis), left planar x larger.
  plant.leg_offset = 0.02;
  state = plant.observe(p, kPeriod);
  state.velocity = {};
  core.reset(state, settings);
  out = core.update(state, WholeBodyCommand{}, settings, kPeriod);
  EXPECT_NEAR(out.leg_offset_m, 0.0, 1e-6);
  EXPECT_NEAR(out.leg_reference_m[0].x, 0.045 - 0.02, 1e-6);
  EXPECT_NEAR(out.leg_reference_m[1].x, 0.045 + 0.02, 1e-6);
  WholeBodyParameters bad = p;
  bad.fore_aft_neutral_offset_m = -0.06;  // would leave the workspace
  EXPECT_FALSE(valid_whole_body_parameters(bad));
  bad = p;
  bad.height_max_m = 0.19;  // stretched singularity margin at the front pivot
  EXPECT_FALSE(valid_whole_body_parameters(bad));
}

TEST(WholeBodyCore, FrictionFeedForwardFollowsTheCommandedTorqueOnlyWhileBalancing) {
  WholeBodyParameters p = test_parameters();
  p.wheel_friction_ff_nm = 0.1;
  p.wheel_friction_ff_smoothing_nm = 0.01;
  const WholeBodySettings settings = test_settings(p);
  PlanarPlant plant;
  WholeBodyCore core;
  core.reset(plant.observe(p, kPeriod), settings);
  run(plant, core, settings, 1.0, still);  // engaged and past the ramp
  // Lean the body forward: the balance torque is positive, so each wheel gets
  // its half plus the whole feed-forward (tanh saturated).
  WholeBodyState state = plant.observe(p, kPeriod);
  state.quaternion_w = std::cos(0.5 * 0.05);
  state.quaternion_y = std::sin(0.5 * 0.05);
  WholeBodyOutput out;
  for (int i = 0; i < 30; ++i) {  // let the torque slew settle
    out = core.update(state, WholeBodyCommand{}, settings, kPeriod);
  }
  ASSERT_GT(out.balance_torque_nm, 0.05);
  EXPECT_NEAR(out.effort_nm[2], 0.5 * out.balance_torque_nm + 0.1, 2e-3);
  EXPECT_NEAR(out.effort_nm[5], 0.5 * out.balance_torque_nm + 0.1, 2e-3);
  // Wheel speed alone changes nothing (no velocity feedback in the term).
  state.velocity[2] = state.velocity[5] = 5.0;
  const auto rolling = core.update(state, WholeBodyCommand{}, settings, kPeriod);
  EXPECT_NEAR(rolling.effort_nm[2] - 0.5 * rolling.balance_torque_nm, 0.1, 2e-3);
  // Not engaged (held, tilted): never.
  WholeBodyCore idle;
  WholeBodyState tilted = plant.observe(p, kPeriod);
  tilted.quaternion_w = std::cos(0.5 * 0.6);
  tilted.quaternion_y = std::sin(0.5 * 0.6);
  idle.reset(tilted, settings);
  const auto held = idle.update(tilted, WholeBodyCommand{}, settings, kPeriod);
  EXPECT_FALSE(held.engaged);
  EXPECT_DOUBLE_EQ(held.effort_nm[2], 0.0);
  p.wheel_friction_ff_nm = 6.0;  // at the wheel limit
  EXPECT_FALSE(valid_whole_body_parameters(p));
}

TEST(WholeBodyCore, ImuRollOffsetIsRemovedBeforeTheRollTask) {
  WholeBodyParameters p = test_parameters();
  p.roll_imu_offset_rad = -0.05;  // IMU reads -0.05 when the body is level
  const WholeBodySettings settings = test_settings(p);
  PlanarPlant plant;
  WholeBodyCore core;
  WholeBodyState state = plant.observe(p, kPeriod);
  // Body level, IMU biased: quaternion with roll -0.05.
  state.quaternion_w = std::cos(-0.025);
  state.quaternion_x = std::sin(-0.025);
  core.reset(state, settings);
  const auto out = core.update(state, WholeBodyCommand{}, settings, kPeriod);
  EXPECT_NEAR(out.roll_rad, 0.0, 1e-9);
}

TEST(WholeBodyCore, LegTorquesRespectTheLimitAndKeepDirection) {
  WholeBodyParameters p = test_parameters();
  p.height_kp_n_m = 50000.0;
  const WholeBodySettings settings = test_settings(p);
  PlanarPlant plant;
  plant.leg_height = 0.15; // far from the 0.17 target: huge vertical force
  WholeBodyCore core;
  WholeBodyState state = plant.observe(p, kPeriod);
  core.reset(state, settings);
  // Force the reference away from the measured height.
  for (int i = 0; i < 300; ++i) {
    core.update(state, WholeBodyCommand{}, settings, kPeriod);
  }
  const auto out = core.update(state, WholeBodyCommand{}, settings, kPeriod);
  EXPECT_NE(out.status & WBC_LEG_TORQUE_LIMIT, 0U);
  for (const std::size_t index : {0U, 1U, 3U, 4U}) {
    EXPECT_LE(std::abs(out.effort_nm[index]), p.leg_torque_limit_nm + 1e-9);
  }
  EXPECT_NEAR(std::max(std::abs(out.effort_nm[0]), std::abs(out.effort_nm[1])),
              p.leg_torque_limit_nm, 1e-9);
}

TEST(WholeBodyCore, WheelTorqueReactionIsFedForwardToTheLegs) {
  // Rolling without slip, a wheel torque tau works on the leg through the
  // ground force tau / r at the axle and the stator moment -tau on the link
  // that carries the wheel motor; the leg motors must hold against both so
  // the axle stays where the balance loop put it.
  WholeBodyParameters p = test_parameters();
  p.wheel_friction_ff_nm = 0.0;
  p.leg_reaction_ground_gain = 1.0;
  p.leg_reaction_stator_gain = 1.0;
  p.leg_reaction_filter_hz = 1e6;  // instantaneous for the algebra below
  const WholeBodySettings settings = test_settings(p);
  PlanarPlant plant;
  WholeBodyCore core;
  core.reset(plant.observe(p, kPeriod), settings);
  run(plant, core, settings, 1.0, still);
  // Lean forward: a positive balance torque on both wheels.
  WholeBodyState state = plant.observe(p, kPeriod);
  state.velocity = {};
  state.quaternion_w = std::cos(0.5 * 0.05);
  state.quaternion_y = std::sin(0.5 * 0.05);
  for (int i = 0; i < 29; ++i) {
    core.update(state, WholeBodyCommand{}, settings, kPeriod);
  }
  // The core is stateful (torque slew, filters): compare two updates of the
  // same state, with and without compensation, from copies.
  WholeBodyCore core_off = core;
  const WholeBodyOutput out = core.update(state, WholeBodyCommand{}, settings, kPeriod);
  ASSERT_GT(out.balance_torque_nm, 0.05);
  ASSERT_TRUE(out.leg_valid[0] && out.leg_valid[1]);
  const double r = p.model.wheel_radius_m;
  const std::array<double, 2> forward_sign{p.model.right_forward_sign,
                                           p.model.left_forward_sign};
  const std::array<const LegGeometry *, 2> geometry{&p.model.right_geometry,
                                                    &p.model.left_geometry};
  const std::array<bool, 2> outer_link{p.model.right_wheel_on_outer_link != 0.0,
                                       p.model.left_wheel_on_outer_link != 0.0};
  for (std::size_t side = 0; side < 2; ++side) {
    const std::size_t inner = side == 0 ? 0 : 3;
    const double wheel_torque = out.effort_nm[inner + 2];  // no friction ff
    EXPECT_NEAR(wheel_torque, 0.5 * out.balance_torque_nm, 1e-9);
    const auto j = jacobian(state.position[inner], state.position[inner + 1],
                            *geometry[side]);
    const auto link = lower_link_angle_jacobian(
        state.position[inner], state.position[inner + 1], *geometry[side],
        outer_link[side]);
    ASSERT_TRUE(j && link);
    const double tau_planar = forward_sign[side] * wheel_torque;
    const double ground_x = -tau_planar / r;
    const std::array<double, 2> expected{
        (*j)[0][0] * ground_x + tau_planar * (*link)[0],
        (*j)[0][1] * ground_x + tau_planar * (*link)[1]};
    EXPECT_NEAR(out.leg_reaction_torque_nm[side][0], expected[0], 1e-9);
    EXPECT_NEAR(out.leg_reaction_torque_nm[side][1], expected[1], 1e-9);
    // Both terms push the same way: the stator moment is not negligible.
    EXPECT_GT(std::abs(tau_planar * (*link)[0]) + std::abs(tau_planar * (*link)[1]),
              0.1 * std::abs(wheel_torque));
  }
  // The right leg's planar +x is body-forward: a forward torque pushes its
  // axle toward +x, so the compensation force points to -x, which on this
  // linkage is a positive inner-joint torque.
  EXPECT_GT(out.leg_reaction_torque_nm[0][0], 0.0);
  // Mirrored on the left (planar +x is body-backward).
  EXPECT_GT(out.leg_reaction_torque_nm[1][1], 0.0);
  // The compensation is part of the leg efforts and switches off with the
  // parameter.
  WholeBodyParameters off = p;
  off.leg_reaction_ground_gain = 0.0;
  off.leg_reaction_stator_gain = 0.0;
  const WholeBodySettings settings_off = test_settings(off);
  const auto without =
      core_off.update(state, WholeBodyCommand{}, settings_off, kPeriod);
  EXPECT_NEAR(without.balance_torque_nm, out.balance_torque_nm, 1e-9);
  EXPECT_NEAR(out.effort_nm[0] - without.effort_nm[0],
              out.leg_reaction_torque_nm[0][0], 1e-6);
  EXPECT_NEAR(out.effort_nm[4] - without.effort_nm[4],
              out.leg_reaction_torque_nm[1][1], 1e-6);
  EXPECT_DOUBLE_EQ(without.leg_reaction_torque_nm[0][0], 0.0);
}

TEST(WholeBodyCore, LqrFeedsBackTheCommandedLegOffsetNotTheDeflectedOne) {
  // On the robot every wheel torque swing deflected the soft legs, the LQR
  // read the deflection as an offset error and answered with more wheel
  // torque: the 2 Hz hunting. With measured_offset_blend 0 the regulator
  // feeds back the offset it commanded, so an axle displacement the legs did
  // not ask for is left to the pitch loop and the trim.
  WholeBodyParameters p = test_parameters();
  p.measured_offset_blend = 0.0;
  const WholeBodySettings commanded = test_settings(p);
  WholeBodyParameters q = p;
  q.measured_offset_blend = 1.0;
  const WholeBodySettings measured = test_settings(q);
  PlanarPlant plant;
  WholeBodyCore core;
  core.reset(plant.observe(p, kPeriod), commanded);
  const auto settled = run(plant, core, commanded, 2.0, still);
  ASSERT_TRUE(settled.engaged);
  EXPECT_DOUBLE_EQ(settled.leg_offset_state_m, settled.leg_offset_command_m);
  // The axle is pushed 10 mm forward under the body (wheels ahead of where
  // the legs placed them) while nothing else changes.
  plant.leg_offset = settled.leg_offset_command_m - 0.01;
  WholeBodyState deflected = plant.observe(p, kPeriod);
  deflected.velocity = {};
  WholeBodyCore with_command = core;
  WholeBodyCore with_measurement = core;
  WholeBodyOutput out_command;
  WholeBodyOutput out_measured;
  for (int i = 0; i < 30; ++i) {  // past the torque slew
    out_command = with_command.update(deflected, still(0.0), commanded, kPeriod);
    out_measured = with_measurement.update(deflected, still(0.0), measured, kPeriod);
  }
  // The state is the command at the start of the cycle, the report the
  // command after this cycle's integration.
  EXPECT_NEAR(out_command.leg_offset_state_m, out_command.leg_offset_command_m,
              p.leg_offset_rate_limit_m_s * kPeriod);
  EXPECT_NEAR(out_measured.leg_offset_state_m, out_measured.leg_offset_m, 1e-9);
  EXPECT_NEAR(out_measured.leg_offset_m, settled.leg_offset_command_m - 0.01, 1e-6);
  // Fed the measurement, the LQR fights the deflection with wheel torque
  // (about 22 Nm/m of offset gain); fed the command it does not.
  EXPECT_GT(std::abs(out_measured.balance_torque_nm), 0.15);
  EXPECT_LT(std::abs(out_command.balance_torque_nm), 0.03);
  // Soft legs modelled as compliance do not destabilise the commanded-state
  // loop either.
  PlanarPlant soft;
  soft.leg_compliance_m_per_nm = 0.03;
  soft.pitch = 0.1;
  WholeBodyCore soft_core;
  soft_core.reset(soft.observe(p, kPeriod), commanded);
  const auto recovered = run(soft, soft_core, commanded, 3.0, still);
  EXPECT_FALSE(recovered.fallen);
  EXPECT_NEAR(soft.pitch, 0.0, 0.02);
}

TEST(WholeBodyCore, YawDampsOnTheWheelsAndIntegratesOnTheImu) {
  const WholeBodySettings settings = test_settings();
  const auto &p = settings.parameters;
  PlanarPlant plant;
  WholeBodyCore core;
  core.reset(plant.observe(p, kPeriod), settings);
  run(plant, core, settings, 1.0, still);
  // The wheels report a left turn the IMU does not see: only the
  // proportional term reacts, braking the right wheel.
  WholeBodyState state = plant.observe(p, kPeriod);
  state.velocity = {};
  const double rate = 0.5;
  const double wheel_rate = rate * p.model.track_width_m / (2.0 * p.model.wheel_radius_m);
  state.velocity[2] = wheel_rate;
  state.velocity[5] = -wheel_rate;
  WholeBodyOutput out;
  for (int i = 0; i < 200; ++i) {  // past the 15 Hz filter and torque slew
    out = core.update(state, WholeBodyCommand{}, settings, kPeriod);
  }
  EXPECT_NEAR(out.wheel_yaw_rate_rad_s, rate, 0.01);
  EXPECT_NEAR(out.yaw_torque_nm, -p.yaw_kp_nm_s_rad * rate, 0.03);
  // A steady commanded rate the wheels follow but the body does not (tyre
  // scrub): the integral on the IMU error keeps growing the torque.
  WholeBodyCore turning;
  PlanarPlant plant2;
  turning.reset(plant2.observe(p, kPeriod), settings);
  run(plant2, turning, settings, 1.0, still);
  WholeBodyState scrub = plant2.observe(p, kPeriod);
  scrub.velocity = {};
  scrub.velocity[2] = wheel_rate;
  scrub.velocity[5] = -wheel_rate;
  scrub.angular_velocity_rad_s[2] = 0.0;
  WholeBodyOutput early;
  WholeBodyOutput late;
  for (int i = 0; i < 600; ++i) {
    const auto o = turning.update(scrub, WholeBodyCommand{0.0, rate, 0.0, 0.0},
                                  settings, kPeriod);
    if (i == 300) {
      early = o;
    }
    late = o;
  }
  EXPECT_NEAR(late.yaw_rate_reference_rad_s, rate, 1e-3);
  EXPECT_GT(late.yaw_torque_nm, early.yaw_torque_nm + 0.1);
  // Scrub feed-forward carries the sign of the command.
  EXPECT_GT(late.yaw_feedforward_nm, 0.5 * p.yaw_scrub_ff_nm);
}

TEST(WholeBodyCore, YawCommandIsLaggedAndInertiaFedForward) {
  const WholeBodySettings settings = test_settings();
  const auto &p = settings.parameters;
  PlanarPlant plant;
  WholeBodyCore core;
  core.reset(plant.observe(p, kPeriod), settings);
  run(plant, core, settings, 1.0, still);
  const WholeBodyState state = plant.observe(p, kPeriod);
  const auto first = core.update(state, WholeBodyCommand{0.0, 1.0, 0.0, 0.0},
                                 settings, kPeriod);
  // One step into a first-order lag: reference = dt / T of the command.
  EXPECT_NEAR(first.yaw_rate_reference_rad_s,
              kPeriod / p.yaw_command_time_constant_s, 1e-6);
  // Inertia feed-forward: I_z * accel * 2 r / track, plus the scrub term.
  const double accel = (1.0 / p.yaw_command_time_constant_s) *
                       (1.0 - kPeriod / p.yaw_command_time_constant_s);
  const double inertia_ff = p.model.yaw_inertia_kgm2 * accel * 2.0 *
                            p.model.wheel_radius_m / p.model.track_width_m;
  WholeBodyOutput out = first;
  for (int i = 0; i < 2; ++i) {
    out = core.update(state, WholeBodyCommand{0.0, 1.0, 0.0, 0.0}, settings, kPeriod);
  }
  EXPECT_GT(out.yaw_feedforward_nm, 0.5 * inertia_ff);
  EXPECT_LT(out.yaw_feedforward_nm, inertia_ff + p.yaw_scrub_ff_nm + 1e-6);
}

TEST(WholeBodyCore, LegTorqueBudgetKeepsGravitySupportUnderSaturation) {
  WholeBodyParameters p = test_parameters();
  p.fore_aft_kp_n_m = 50000.0;
  const WholeBodySettings settings = test_settings(p);
  PlanarPlant plant;
  plant.leg_offset = 0.02;  // 20 mm from where the command starts: huge force
  WholeBodyCore core;
  WholeBodyState state = plant.observe(p, kPeriod);
  state.velocity = {};
  core.reset(state, settings);
  // Anchor, then move the command away from the measured position.
  core.update(state, WholeBodyCommand{}, settings, kPeriod);
  plant.leg_offset = 0.0;
  state = plant.observe(p, kPeriod);
  state.velocity = {};
  const auto out = core.update(state, WholeBodyCommand{}, settings, kPeriod);
  EXPECT_NE(out.status & WBC_LEG_TORQUE_LIMIT, 0U);
  const auto j = jacobian(state.position[0], state.position[1], p.model.right_geometry);
  ASSERT_TRUE(j);
  const double support = 0.5 * p.model.body_mass_kg * p.model.gravity_m_s2;
  const std::array<double, 2> support_torque{(*j)[1][0] * support,
                                             (*j)[1][1] * support};
  // The saturated corrective torque was added on top of the untouched
  // support, and one joint sits exactly at the limit.
  for (const std::size_t index : {0U, 1U}) {
    EXPECT_LE(std::abs(out.effort_nm[index]), p.leg_torque_limit_nm + 1e-9);
  }
  EXPECT_NEAR(std::max(std::abs(out.effort_nm[0]), std::abs(out.effort_nm[1])),
              p.leg_torque_limit_nm, 1e-9);
  const std::array<double, 2> corrective{out.effort_nm[0] - support_torque[0],
                                         out.effort_nm[1] - support_torque[1]};
  // Direction kept: the corrective torques are a positive scaling of the
  // unsaturated ones, whose ratio a zero-velocity impedance fixes.
  WholeBodyParameters soft = p;
  soft.fore_aft_kp_n_m = 1.0;
  WholeBodyCore reference_core;
  reference_core.reset(plant.observe(soft, kPeriod), test_settings(soft));
  EXPECT_GT(corrective[0] * corrective[0] + corrective[1] * corrective[1], 0.1);
}

TEST(WholeBodySettings, RejectsWeightsWithoutDelayMargin) {
  WholeBodyParameters p = test_parameters();
  WholeBodySettings settings;
  std::string reason;
  ASSERT_TRUE(build_whole_body_settings(p, kPeriod, settings, &reason)) << reason;
  EXPECT_GE(settings.delay_margin_s, p.required_delay_margin_s);
  EXPECT_GE(settings.delay_margin_s, 4.0 * kPeriod);
  // The first hardware set: wheel torque weight 0.6 left about 3 samples.
  p.effort_weights[0] = 0.6;
  EXPECT_FALSE(build_whole_body_settings(p, kPeriod, settings, &reason));
  EXPECT_NE(reason.find("delay margin"), std::string::npos);
  p.required_delay_margin_s = 0.0;
  EXPECT_TRUE(build_whole_body_settings(p, kPeriod, settings, &reason)) << reason;
  EXPECT_LT(settings.delay_margin_s, 4.0 * kPeriod);
}

TEST(Kinematics, LowerLinkAnglesFollowTheAxle) {
  const LegGeometry g;
  // Symmetric stance: both lower links lean inward toward the axle under the
  // centre, mirrored about x = base / 2.
  const auto ik = inverse_kinematics(0.045, 0.17, g);
  ASSERT_TRUE(ik);
  const auto angles = lower_link_angles((*ik)[0], (*ik)[1], g);
  ASSERT_TRUE(angles);
  const double half_pi = 1.57079632679489661923;
  EXPECT_NEAR(angles->inner - half_pi, half_pi - angles->outer, 1e-9);
  EXPECT_LT(angles->inner, half_pi);  // inner link points toward +x and down
  EXPECT_GT(angles->outer, half_pi);  // outer link points toward -x and down
  // Moving the axle toward +x rotates the inner link toward +x (angle down).
  const auto shifted = inverse_kinematics(0.055, 0.17, g);
  ASSERT_TRUE(shifted);
  const auto shifted_angles = lower_link_angles((*shifted)[0], (*shifted)[1], g);
  ASSERT_TRUE(shifted_angles);
  EXPECT_LT(shifted_angles->inner, angles->inner);
  EXPECT_LT(shifted_angles->outer, angles->outer);
  // The Jacobian matches finite differences of the angles.
  const auto jac = lower_link_angle_jacobian((*ik)[0], (*ik)[1], g, true);
  ASSERT_TRUE(jac);
  const double eps = 1e-5;
  const auto plus = lower_link_angles((*ik)[0] + eps, (*ik)[1], g);
  const auto minus = lower_link_angles((*ik)[0] - eps, (*ik)[1], g);
  ASSERT_TRUE(plus && minus);
  EXPECT_NEAR((*jac)[0], (plus->outer - minus->outer) / (2.0 * eps), 1e-5);
}

TEST(WholeBodyCore, GravitySupportIsFedForwardThroughTheJacobian) {
  const WholeBodySettings settings = test_settings();
  const auto &p = settings.parameters;
  PlanarPlant plant;
  plant.leg_height = p.height_target_m;
  WholeBodyCore core;
  const WholeBodyState state = plant.observe(p, kPeriod);
  core.reset(state, settings);
  const auto out = core.update(state, WholeBodyCommand{}, settings, kPeriod);
  // Upright, at target, at rest: leg torques equal J^T [0, m g / 2].
  const auto j = jacobian(state.position[0], state.position[1], p.model.right_geometry);
  ASSERT_TRUE(j);
  const double support = 0.5 * p.model.body_mass_kg * p.model.gravity_m_s2;
  EXPECT_NEAR(out.effort_nm[0], (*j)[1][0] * support, 1e-6);
  EXPECT_NEAR(out.effort_nm[1], (*j)[1][1] * support, 1e-6);
  EXPECT_GT(std::abs(out.effort_nm[0]) + std::abs(out.effort_nm[1]), 0.5);
  EXPECT_LT(std::max(std::abs(out.effort_nm[0]), std::abs(out.effort_nm[1])),
            p.leg_torque_limit_nm);
}

} // namespace sandro_controllers
