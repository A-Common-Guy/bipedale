#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "sandro_controllers/kinematics.hpp"

namespace sandro_controllers {

// The actor exported by sandro_rl/sandro_rl/export_ros.py: observation
// normaliser, dense layers with one hidden activation, action clip. Loading
// allocates; run() does not.
class MlpPolicy {
public:
  bool load(const std::string &path, std::string *error);
  std::size_t input_size() const { return static_cast<std::size_t>(mean_.size()); }
  std::size_t output_size() const {
    return weights_.empty() ? 0U : static_cast<std::size_t>(weights_.back().rows());
  }
  const Eigen::VectorXf &run(const Eigen::VectorXf &observation);

private:
  std::uint32_t activation_{};
  float observation_clip_{};
  float action_clip_{};
  Eigen::VectorXf mean_;
  Eigen::VectorXf inverse_std_;
  // Each stored as (out x in), so a layer is y = W x + b.
  std::vector<Eigen::MatrixXf> weights_;
  std::vector<Eigen::VectorXf> biases_;
  std::vector<Eigen::VectorXf> activations_;
};

// Everything the controller needs from a policy's policy.json, as ROS
// parameters (sandro_bringup/config/rl_policies.yaml). Joint arrays are in
// right-inner, right-outer, right-wheel, left-inner, left-outer, left-wheel
// order, the training's JOINTS.
struct RlPolicySettings {
  std::string policy_file;
  double policy_rate_hz{100.0};
  int history{4};
  int command_size{3};
  double gyro_scale{0.25};
  double leg_velocity_scale{0.1};
  double wheel_velocity_scale{0.05};
  double imu_offset_rad{0.0};

  double forward_m_s{0.5};
  double yaw_rad_s{1.5};
  double height_min_m{0.14};
  double height_max_m{0.185};
  double roll_rad{0.0};
  double initial_height_m{0.17};

  double forward_accel_m_s2{1.0};
  double yaw_time_constant_s{0.15};
  double height_rate_m_s{0.05};
  double roll_rate_rad_s{0.5};
  double leg_height_min_m{0.14};
  double leg_height_max_m{0.185};
  double track_width_m{0.304};

  double leg_scale_rad{0.5};
  double leg_kp_nm_rad{12.0};
  double leg_kd_nm_s_rad{0.5};
  double joint_min_rad{-2.0943951024};
  double joint_max_rad{2.0943951024};
  bool wheel_velocity_mode{true};
  double wheel_scale_rad_s{20.0};
  double wheel_kd_nm_s_rad{0.4};
  double wheel_scale_nm{3.0};
  // First-order low-pass on the wheel commands sent to the drives, Hz; 0 off.
  // The observation keeps the policy's own action, as in training.
  double wheel_filter_hz{0.0};
  std::array<double, 6> stand_q{};

  LegGeometry leg_geometry{};

  // Once engaged, a tilt beyond this latches the drives limp.
  double fall_angle_rad{1.0};
  double imu_timeout_s{0.15};
  double command_timeout_s{0.3};
  // Before the policy takes over, legs farther than stand_tolerance_rad from
  // stand_q are moved there over stand_s, since the policy only ever started
  // near that pose. It then engages once the body has been within
  // engage_tilt_rad, turning slower than engage_rate_rad_s and every joint
  // slower than engage_joint_velocity_rad_s for engage_hold_s: the first
  // frame fills the whole observation history. Training episodes start
  // within 0.05 rad of the stand pose and nearly still; the old flat_level
  // engaged at 0.12 rad with the legs still moving at 0.9 rad/s and kicked
  // straight into its 7.5 Hz ringing (recordings/rl_flat_vibration_2026-10-21).
  double stand_s{1.5};
  double stand_tolerance_rad{0.15};
  double engage_tilt_rad{0.08};
  double engage_rate_rad_s{1.0};
  double engage_joint_velocity_rad_s{0.3};
  double engage_hold_s{0.2};
  // A hand-over from another balancer engages up to this tilt: that robot is
  // already balancing, possibly leaning into an acceleration, and refusing
  // leaves it unbalanced.
  double hand_over_tilt_rad{0.15};
  double fallen_kd_nm_s_rad{0.5};
  // Oscillation watchdog, while the policy runs: per joint, the joint
  // velocity band-passed to 5-15 Hz (two first-order high-passes and two
  // low-passes), RMS over oscillation_window_s. Beyond the leg or the wheel
  // limit the drives go limp as on a fall. The robot's 7.5 Hz ringing reached
  // 1.8 (legs) and 3-3.9 rad/s (wheels); the deployed policies in simulation
  // stay below 0.61 and 3.3 rad/s, pushes and 80 ms of wheel lag included.
  // 0 disables a limit.
  double oscillation_leg_rad_s{1.0};
  double oscillation_wheel_rad_s{5.0};
  double oscillation_window_s{0.3};
};

struct RlPolicyState {
  std::array<double, 6> position{};
  std::array<double, 6> velocity{};
  // ROS order x, y, z, w; non-finite when the IMU is stale.
  std::array<double, 4> orientation{0.0, 0.0, 0.0, 1.0};
  std::array<double, 3> angular_velocity{};
};

// /cmd_vel: linear.x, angular.z, linear.z (height rate), angular.x (body roll
// target, rad; used by policies trained with a roll command).
struct RlPolicyCommand {
  double forward_m_s{};
  double yaw_rate_rad_s{};
  double height_rate_m_s{};
  double roll_rad{};
  double age_s{};
};

// One MIT frame per joint: the drive applies
// kp (position - q) + kd (velocity - v) + effort.
struct RlPolicyFrame {
  std::array<double, 6> effort{};
  std::array<double, 6> position{};
  std::array<double, 6> velocity{};
  std::array<double, 6> kp{};
  std::array<double, 6> kd{};
};

enum class RlPhase : std::uint8_t { kStand = 0, kPolicy = 1, kFallen = 2 };

enum RlStatus : std::uint32_t {
  RL_OK = 0U,
  RL_IMU_STALE = 1U << 0,
  RL_FALLEN = 1U << 1,
  RL_COMMAND_TIMEOUT = 1U << 2,
  RL_WAITING_UPRIGHT = 1U << 4,
  RL_NON_FINITE_ACTION = 1U << 5,
  RL_OSCILLATION = 1U << 6,
};

struct RlPolicyOutput {
  RlPolicyFrame frame;
  RlPhase phase{RlPhase::kStand};
  std::uint32_t status{RL_OK};
  double tilt_rad{};
  double pitch_rad{};
  double roll_rad{};
  double rate_rad_s{};
  std::array<double, 4> reference{};  // forward, yaw rate, height, roll
  std::array<double, 2> leg_height_m{};
  std::array<float, 6> action{};
  // The oscillation watchdog's 5-15 Hz velocity RMS, largest leg and wheel.
  std::array<double, 2> oscillation_rad_s{};
  double inference_s{};
  bool inferred{false};
};

class RlPolicyCore {
public:
  // Loads the policy and sizes every buffer; checks it against the settings.
  bool configure(const RlPolicySettings &settings, int update_rate_hz,
                 std::string *error);
  void reset(const RlPolicyState &state);
  // Takes over from another balancer holding the robot upright: engages at
  // once from the measured pose, without the stand phase. False (and the
  // stand phase stays) unless the IMU is valid and the body within
  // hand_over_tilt_rad.
  bool hand_over(const RlPolicyState &state);
  RlPolicyOutput update(const RlPolicyState &state,
                        const RlPolicyCommand &command, double period_s);

  int frame_size() const { return frame_size_; }
  RlPhase phase() const { return phase_; }
  // The observation frame of one cycle, exposed for tests.
  void build_frame(const RlPolicyState &state, Eigen::Ref<Eigen::VectorXf> frame) const;
  RlPolicyFrame frames_from_action(const std::array<float, 6> &action) const;
  void shape(const RlPolicyCommand &command);
  const std::array<double, 4> &reference() const { return reference_; }
  std::array<double, 2> leg_heights(const RlPolicyState &state) const;

private:
  Eigen::Matrix3d body_rotation(const RlPolicyState &state) const;
  void engage(const RlPolicyState &state);
  double feasible_roll(double roll, double height) const;
  // Advances the watchdog filters; the largest leg and wheel band RMS.
  std::array<double, 2> watch_oscillation(const RlPolicyState &state, double period_s);

  RlPolicySettings settings_{};
  MlpPolicy policy_;
  int frame_size_{};
  int cycles_per_action_{3};
  double policy_dt_s_{0.01};
  Eigen::Matrix3d body_from_imu_{Eigen::Matrix3d::Identity()};

  RlPhase phase_{RlPhase::kStand};
  double stand_elapsed_s_{};
  double still_s_{};
  std::array<double, 6> stand_start_{};
  double imu_stale_s_{};
  long cycle_{};
  std::array<double, 4> reference_{};
  double height_command_m_{};
  std::array<float, 6> last_action_{};
  std::array<float, 6> applied_action_{};
  std::array<std::array<double, 6>, 2> oscillation_high_pass_{};  // the high-passes' low-pass states
  std::array<std::array<double, 6>, 2> oscillation_low_pass_{};
  std::array<double, 6> oscillation_mean_square_{};
  bool oscillation_tripped_{false};
  Eigen::VectorXf observation_;
  Eigen::VectorXf frame_;
  RlPolicyFrame policy_frame_{};
};

} // namespace sandro_controllers
