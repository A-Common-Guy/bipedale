#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <string>

#include "sandro_controllers/rl_policy_core.hpp"

namespace sc = sandro_controllers;

namespace {

const std::string kSource = SANDRO_CONTROLLERS_SOURCE_DIR;

sc::RlPolicySettings flat_settings() {
  sc::RlPolicySettings s;
  s.policy_file = kSource + "/policies/flat_level.bin";
  s.stand_q = {0.41133484, 0.21714621, 0.0, 0.21523191, 0.41288111, 0.0};
  return s;
}

sc::RlPolicySettings agile_settings() {
  sc::RlPolicySettings s = flat_settings();
  s.policy_file = kSource + "/policies/agile_terrain.bin";
  s.command_size = 4;
  s.forward_m_s = 1.0;
  s.yaw_rad_s = 3.0;
  s.height_min_m = 0.09;
  s.height_max_m = 0.19;
  s.roll_rad = 0.3;
  s.forward_accel_m_s2 = 2.0;
  s.height_rate_m_s = 0.15;
  s.roll_rate_rad_s = 0.6;
  s.leg_height_min_m = 0.07;
  s.leg_height_max_m = 0.195;
  s.wheel_scale_rad_s = 25.0;
  return s;
}

sc::RlPolicyState standing(const sc::RlPolicySettings &s) {
  sc::RlPolicyState state;
  state.position = s.stand_q;
  return state;
}

}  // namespace

// The C++ actor reproduces onnxruntime on the exported policy.onnx.
TEST(RlPolicy, ActorMatchesOnnxReference) {
  for (const std::string name : {"flat_level", "agile_terrain"}) {
    sc::MlpPolicy policy;
    std::string error;
    ASSERT_TRUE(policy.load(kSource + "/policies/" + name + ".bin", &error)) << error;
    std::ifstream in(kSource + "/test/data/" + name + "_reference.txt");
    ASSERT_TRUE(in) << name;
    std::size_t count{}, inputs{}, outputs{};
    in >> count >> inputs >> outputs;
    ASSERT_EQ(inputs, policy.input_size());
    ASSERT_EQ(outputs, policy.output_size());
    Eigen::VectorXf obs(static_cast<Eigen::Index>(inputs));
    for (std::size_t n = 0; n < count; ++n) {
      for (std::size_t i = 0; i < inputs; ++i) {
        in >> obs[static_cast<Eigen::Index>(i)];
      }
      const Eigen::VectorXf action = policy.run(obs);
      for (std::size_t i = 0; i < outputs; ++i) {
        float expected{};
        in >> expected;
        EXPECT_NEAR(action[static_cast<Eigen::Index>(i)], expected, 1e-4F) << name << " case " << n;
      }
    }
  }
}

TEST(RlPolicy, RejectsMismatchedSettings) {
  sc::RlPolicyCore core;
  std::string error;
  auto s = flat_settings();
  s.command_size = 4;
  s.roll_rad = 0.3;
  EXPECT_FALSE(core.configure(s, 300, &error));
  EXPECT_FALSE(core.configure(flat_settings(), 250, &error));
  EXPECT_TRUE(core.configure(flat_settings(), 300, &error)) << error;
  EXPECT_TRUE(core.configure(agile_settings(), 300, &error)) << error;
}

TEST(RlPolicy, FrameLayout) {
  sc::RlPolicyCore core;
  std::string error;
  auto s = agile_settings();
  s.imu_offset_rad = 0.1;
  ASSERT_TRUE(core.configure(s, 300, &error)) << error;
  auto state = standing(s);
  // IMU rolled by exactly its mounting offset: the body is level.
  state.orientation = {std::sin(0.05), 0.0, 0.0, std::cos(0.05)};
  state.angular_velocity = {0.4, 0.0, 0.0};
  state.position[0] += 0.1;
  state.velocity[2] = 10.0;
  Eigen::VectorXf frame(core.frame_size());
  core.build_frame(state, frame);
  ASSERT_EQ(frame.size(), 26);
  EXPECT_NEAR(frame[0], 0.0F, 1e-6F);
  EXPECT_NEAR(frame[1], 0.0F, 1e-6F);
  EXPECT_NEAR(frame[2], -1.0F, 1e-6F);
  EXPECT_NEAR(frame[3], 0.1F, 1e-6F);    // gyro * 0.25
  EXPECT_NEAR(frame[6], 0.1F, 1e-6F);    // right inner - stand
  EXPECT_NEAR(frame[14], 0.5F, 1e-6F);   // right wheel * 0.05
}

// build_frame reproduces interface.PolicyInterface.frame on random signals
// (zero last action and command reference, as before the first policy step).
TEST(RlPolicy, FrameMatchesThePythonInterface) {
  for (const std::string name : {"flat_level", "agile_terrain"}) {
    std::ifstream in(kSource + "/test/data/" + name + "_frames.txt");
    ASSERT_TRUE(in) << name;
    auto s = name == "flat_level" ? flat_settings() : agile_settings();
    std::size_t count{};
    in >> count >> s.imu_offset_rad;
    for (auto &q : s.stand_q) {
      in >> q;
    }
    sc::RlPolicyCore core;
    std::string error;
    ASSERT_TRUE(core.configure(s, 300, &error)) << error;
    Eigen::VectorXf frame(core.frame_size());
    for (std::size_t n = 0; n < count; ++n) {
      sc::RlPolicyState state;
      for (auto &v : state.orientation) in >> v;
      for (auto &v : state.angular_velocity) in >> v;
      for (auto &v : state.position) in >> v;
      for (auto &v : state.velocity) in >> v;
      core.build_frame(state, frame);
      for (Eigen::Index i = 0; i < frame.size(); ++i) {
        float expected{};
        in >> expected;
        EXPECT_NEAR(frame[i], expected, 1e-5F) << name << " case " << n << " element " << i;
      }
    }
  }
}

TEST(RlPolicy, ShapingFollowsTheRateLimits) {
  sc::RlPolicyCore core;
  std::string error;
  ASSERT_TRUE(core.configure(agile_settings(), 300, &error)) << error;
  core.reset(standing(agile_settings()));
  // Upright and still for engage_hold_s, then the first policy step.
  for (int i = 0; i < 40 && core.phase() != sc::RlPhase::kPolicy; ++i) {
    core.update(standing(agile_settings()), sc::RlPolicyCommand{}, 1.0 / 300.0);
  }
  ASSERT_EQ(core.phase(), sc::RlPhase::kPolicy);
  const double h0 = core.reference()[2];
  sc::RlPolicyCommand command{5.0, 10.0, 1.0, 1.0, 0.0};
  core.shape(command);
  EXPECT_NEAR(core.reference()[0], 0.02, 1e-9);                 // 2 m/s^2 * 10 ms
  EXPECT_NEAR(core.reference()[1], 3.0 * (1.0 - std::exp(-0.01 / 0.15)), 1e-9);
  // The height command starts at initial_height_m (0.17) and rises 1.5 mm
  // per step; the reference follows it at the same rate from the measured height.
  EXPECT_NEAR(core.reference()[2], h0 + std::clamp(0.17 + 2 * 0.0015 - h0, -0.0015, 0.0015), 1e-9);
  EXPECT_NEAR(core.reference()[3], 0.006, 1e-9);                // 0.6 rad/s * 10 ms
}

TEST(RlPolicy, MapsActionsToMitFrames) {
  sc::RlPolicyCore core;
  std::string error;
  const auto s = flat_settings();
  ASSERT_TRUE(core.configure(s, 300, &error)) << error;
  const sc::RlPolicyFrame f = core.frames_from_action({1.0F, 0.0F, 1.0F, 0.0F, -8.0F, -1.0F});
  EXPECT_NEAR(f.position[0], s.stand_q[0] + 0.5, 1e-9);
  EXPECT_NEAR(f.position[4], s.joint_min_rad, 1e-9);
  EXPECT_EQ(f.kp[0], 12.0);
  EXPECT_EQ(f.kd[0], 0.5);
  EXPECT_NEAR(f.velocity[2], 20.0, 1e-9);
  EXPECT_NEAR(f.velocity[5], -20.0, 1e-9);
  EXPECT_EQ(f.kp[2], 0.0);
  EXPECT_EQ(f.kd[2], 0.4);
}

TEST(RlPolicy, StandsUpThenEngagesAndLatchesAFall) {
  sc::RlPolicyCore core;
  std::string error;
  auto s = flat_settings();
  s.engage_hold_s = 0.1;
  ASSERT_TRUE(core.configure(s, 300, &error)) << error;
  auto state = standing(s);
  state.position[0] -= 0.6;
  core.reset(state);
  auto tipped = standing(s);
  tipped.orientation = {0.0, std::sin(0.6), 0.0, std::cos(0.6)};  // 1.2 rad of pitch
  // Tipped before engaging (held, or settling after a reset) is not a fall.
  tipped.position = state.position;
  auto out = core.update(tipped, sc::RlPolicyCommand{}, 1.0 / 300.0);
  EXPECT_EQ(out.phase, sc::RlPhase::kStand);
  out = core.update(state, sc::RlPolicyCommand{}, 1.0 / 300.0);
  EXPECT_EQ(out.phase, sc::RlPhase::kStand);
  EXPECT_NEAR(out.frame.position[0], state.position[0] + 2.0 * 0.6 / (1.5 * 300.0), 1e-6);
  // A body still turning does not engage, even upright at the stand pose.
  auto turning = standing(s);
  turning.angular_velocity = {0.0, 2.0, 0.0};
  for (int i = 0; i < 60; ++i) {
    out = core.update(turning, sc::RlPolicyCommand{}, 1.0 / 300.0);
  }
  EXPECT_EQ(out.phase, sc::RlPhase::kStand);
  EXPECT_TRUE(out.status & sc::RL_WAITING_UPRIGHT);
  // Nor does a spinning wheel.
  auto spinning = standing(s);
  spinning.velocity[5] = 5.0;
  for (int i = 0; i < 60; ++i) {
    out = core.update(spinning, sc::RlPolicyCommand{}, 1.0 / 300.0);
  }
  EXPECT_EQ(out.phase, sc::RlPhase::kStand);
  int cycles = 0;
  for (; cycles < 460 && core.phase() == sc::RlPhase::kStand; ++cycles) {
    out = core.update(standing(s), sc::RlPolicyCommand{}, 1.0 / 300.0);
  }
  EXPECT_EQ(out.phase, sc::RlPhase::kPolicy);
  EXPECT_TRUE(out.inferred);
  EXPECT_EQ(cycles, 30);  // engage_hold_s at 300 Hz

  tipped.position = s.stand_q;
  out = core.update(tipped, sc::RlPolicyCommand{}, 1.0 / 300.0);
  EXPECT_EQ(out.phase, sc::RlPhase::kFallen);
  out = core.update(standing(s), sc::RlPolicyCommand{}, 1.0 / 300.0);
  EXPECT_EQ(out.phase, sc::RlPhase::kFallen);
  EXPECT_EQ(out.frame.kp[0], 0.0);
  EXPECT_EQ(out.frame.kd[2], 0.5);
}

TEST(RlPolicy, LegsAtTheStandPoseHoldItWhateverTheStartPose) {
  sc::RlPolicyCore core;
  std::string error;
  const auto s = flat_settings();
  ASSERT_TRUE(core.configure(s, 300, &error)) << error;
  auto collapsed = standing(s);
  for (const int j : {0, 1, 3, 4}) {
    collapsed.position[j] -= 1.0;
  }
  core.reset(collapsed);
  auto tipped = standing(s);
  tipped.orientation = {0.0, std::sin(0.3), 0.0, std::cos(0.3)};
  const auto out = core.update(tipped, sc::RlPolicyCommand{}, 1.0 / 300.0);
  EXPECT_EQ(out.phase, sc::RlPhase::kStand);
  for (const int j : {0, 1, 3, 4}) {
    EXPECT_NEAR(out.frame.position[j], s.stand_q[j], 1e-9);
  }
}

TEST(RlPolicy, StaleImuBeyondTheTimeoutIsAFall) {
  sc::RlPolicyCore core;
  std::string error;
  const auto s = flat_settings();
  ASSERT_TRUE(core.configure(s, 300, &error)) << error;
  core.reset(standing(s));
  auto stale = standing(s);
  stale.orientation[0] = std::nan("");
  sc::RlPolicyOutput out;
  for (int i = 0; i < 30; ++i) {
    out = core.update(stale, sc::RlPolicyCommand{}, 1.0 / 300.0);
  }
  EXPECT_NE(out.phase, sc::RlPhase::kFallen);
  for (int i = 0; i < 30; ++i) {
    out = core.update(stale, sc::RlPolicyCommand{}, 1.0 / 300.0);
  }
  EXPECT_EQ(out.phase, sc::RlPhase::kFallen);
}
