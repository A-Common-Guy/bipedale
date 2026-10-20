#include "sandro_controllers/rl_policy_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>

namespace sandro_controllers {
namespace {

constexpr std::array<std::size_t, 4> kLegs{0U, 1U, 3U, 4U};
constexpr std::array<std::size_t, 2> kWheels{2U, 5U};
// gravity, gyro, leg position, leg velocity, wheel velocity, last action
constexpr int kFrameWithoutCommand = 3 + 3 + 4 + 4 + 2 + 6;

template <typename T> bool read(std::ifstream &in, T &value) {
  return static_cast<bool>(in.read(reinterpret_cast<char *>(&value), sizeof(T)));
}

bool read_floats(std::ifstream &in, float *data, std::size_t count) {
  return static_cast<bool>(in.read(reinterpret_cast<char *>(data),
                                   static_cast<std::streamsize>(count * sizeof(float))));
}

bool finite(const RlPolicyState &state) {
  for (std::size_t i = 0; i < 6; ++i) {
    if (!std::isfinite(state.position[i]) || !std::isfinite(state.velocity[i])) {
      return false;
    }
  }
  return true;
}

bool imu_fresh(const RlPolicyState &state) {
  return std::all_of(state.orientation.begin(), state.orientation.end(),
                     [](double v) { return std::isfinite(v); }) &&
         std::all_of(state.angular_velocity.begin(), state.angular_velocity.end(),
                     [](double v) { return std::isfinite(v); }) &&
         (state.orientation[0] != 0.0 || state.orientation[1] != 0.0 ||
          state.orientation[2] != 0.0 || state.orientation[3] != 0.0);
}

} // namespace

bool MlpPolicy::load(const std::string &path, std::string *error) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    *error = "cannot open policy file '" + path + "'";
    return false;
  }
  char magic[8]{};
  in.read(magic, sizeof(magic));
  if (!in || std::memcmp(magic, "SNDRMLP1", sizeof(magic)) != 0) {
    *error = "'" + path + "' is not a SNDRMLP1 policy file";
    return false;
  }
  std::uint32_t layers{};
  std::uint32_t inputs{};
  if (!read(in, activation_) || !read(in, layers) || !read(in, inputs) ||
      !read(in, observation_clip_) || !read(in, action_clip_) || activation_ > 2U ||
      layers == 0U || layers > 16U || inputs == 0U || inputs > 4096U) {
    *error = "'" + path + "': invalid header";
    return false;
  }
  mean_.resize(inputs);
  Eigen::VectorXf std(inputs);
  if (!read_floats(in, mean_.data(), inputs) || !read_floats(in, std.data(), inputs) ||
      (std.array() <= 0.0F).any()) {
    *error = "'" + path + "': invalid normaliser";
    return false;
  }
  inverse_std_ = std.cwiseInverse();
  weights_.clear();
  biases_.clear();
  activations_.clear();
  std::uint32_t previous = inputs;
  for (std::uint32_t i = 0; i < layers; ++i) {
    std::uint32_t in_size{};
    std::uint32_t out_size{};
    if (!read(in, in_size) || !read(in, out_size) || in_size != previous ||
        out_size == 0U || out_size > 4096U) {
      *error = "'" + path + "': layer " + std::to_string(i) + " has the wrong shape";
      return false;
    }
    // The file holds W row-major as (in x out); the same bytes read
    // column-major are W transposed, (out x in).
    Eigen::MatrixXf weight(out_size, in_size);
    Eigen::VectorXf bias(out_size);
    if (!read_floats(in, weight.data(), static_cast<std::size_t>(in_size) * out_size) ||
        !read_floats(in, bias.data(), out_size)) {
      *error = "'" + path + "': truncated at layer " + std::to_string(i);
      return false;
    }
    weights_.push_back(std::move(weight));
    biases_.push_back(std::move(bias));
    activations_.emplace_back(out_size);
    previous = out_size;
  }
  activations_.emplace_back(inputs);
  char extra{};
  if (in.read(&extra, 1)) {
    *error = "'" + path + "': trailing bytes";
    return false;
  }
  return true;
}

const Eigen::VectorXf &MlpPolicy::run(const Eigen::VectorXf &observation) {
  Eigen::VectorXf &input = activations_.back();
  input = ((observation - mean_).cwiseProduct(inverse_std_))
              .cwiseMax(-observation_clip_)
              .cwiseMin(observation_clip_);
  const Eigen::VectorXf *x = &input;
  for (std::size_t i = 0; i < weights_.size(); ++i) {
    Eigen::VectorXf &y = activations_[i];
    y.noalias() = weights_[i] * *x;
    y += biases_[i];
    if (i + 1 < weights_.size()) {
      switch (activation_) {
      case 0U:
        y = (y.array() > 0.0F).select(y, y.array().exp() - 1.0F);
        break;
      case 1U:
        y = y.array().tanh();
        break;
      default:
        y = y.cwiseMax(0.0F);
        break;
      }
    }
    x = &y;
  }
  Eigen::VectorXf &out = activations_[weights_.size() - 1];
  out = out.cwiseMax(-action_clip_).cwiseMin(action_clip_);
  return out;
}

bool RlPolicyCore::configure(const RlPolicySettings &settings, const int update_rate_hz,
                             std::string *error) {
  const auto &s = settings;
  if (s.history < 1 || (s.command_size != 3 && s.command_size != 4) ||
      !(s.policy_rate_hz > 0.0) || update_rate_hz <= 0 || !(s.forward_m_s > 0.0) ||
      !(s.yaw_rad_s > 0.0) || !(s.height_max_m > s.height_min_m) ||
      (s.command_size == 4 && !(s.roll_rad > 0.0)) || !(s.leg_kp_nm_rad > 0.0) ||
      !(s.wheel_kd_nm_s_rad > 0.0) || !(s.stand_s > 0.0)) {
    *error = "inconsistent policy settings";
    return false;
  }
  const double ratio = static_cast<double>(update_rate_hz) / s.policy_rate_hz;
  cycles_per_action_ = static_cast<int>(std::lround(ratio));
  if (cycles_per_action_ < 1 || std::abs(ratio - cycles_per_action_) > 1e-6) {
    *error = "the update rate (" + std::to_string(update_rate_hz) +
             " Hz) must be a whole multiple of the policy rate";
    return false;
  }
  if (!policy_.load(s.policy_file, error)) {
    return false;
  }
  frame_size_ = kFrameWithoutCommand + s.command_size;
  if (policy_.input_size() != static_cast<std::size_t>(frame_size_ * s.history) ||
      policy_.output_size() != 6U) {
    *error = "the policy takes " + std::to_string(policy_.input_size()) + " inputs and " +
             std::to_string(policy_.output_size()) + " outputs; the settings describe " +
             std::to_string(frame_size_ * s.history) + " and 6";
    return false;
  }
  settings_ = s;
  policy_dt_s_ = cycles_per_action_ / static_cast<double>(update_rate_hz);
  const double c = std::cos(-s.imu_offset_rad);
  const double n = std::sin(-s.imu_offset_rad);
  body_from_imu_ << 1.0, 0.0, 0.0, 0.0, c, -n, 0.0, n, c;
  observation_ = Eigen::VectorXf::Zero(frame_size_ * s.history);
  frame_ = Eigen::VectorXf::Zero(frame_size_);
  return true;
}

Eigen::Matrix3d RlPolicyCore::body_rotation(const RlPolicyState &state) const {
  const Eigen::Quaterniond q(state.orientation[3], state.orientation[0],
                             state.orientation[1], state.orientation[2]);
  return q.normalized().toRotationMatrix() * body_from_imu_;
}

std::array<double, 2> RlPolicyCore::leg_heights(const RlPolicyState &state) const {
  std::array<double, 2> heights{};
  for (std::size_t side = 0; side < 2; ++side) {
    const auto axle = forward_kinematics(state.position[kLegs[2 * side]],
                                         state.position[kLegs[2 * side + 1]],
                                         settings_.leg_geometry);
    heights[side] = axle ? axle->y : 0.5 * (settings_.height_min_m + settings_.height_max_m);
  }
  return heights;
}

void RlPolicyCore::build_frame(const RlPolicyState &state,
                               Eigen::Ref<Eigen::VectorXf> frame) const {
  const auto &s = settings_;
  const Eigen::Matrix3d r = body_rotation(state);
  const Eigen::Vector3d gravity = r.transpose() * Eigen::Vector3d(0.0, 0.0, -1.0);
  int k = 0;
  for (int i = 0; i < 3; ++i) {
    frame[k++] = static_cast<float>(gravity[i]);
  }
  for (int i = 0; i < 3; ++i) {
    frame[k++] = static_cast<float>(state.angular_velocity[i] * s.gyro_scale);
  }
  for (const auto j : kLegs) {
    frame[k++] = static_cast<float>(state.position[j] - s.stand_q[j]);
  }
  for (const auto j : kLegs) {
    frame[k++] = static_cast<float>(state.velocity[j] * s.leg_velocity_scale);
  }
  for (const auto j : kWheels) {
    frame[k++] = static_cast<float>(state.velocity[j] * s.wheel_velocity_scale);
  }
  for (const float a : last_action_) {
    frame[k++] = a;
  }
  frame[k++] = static_cast<float>(reference_[0] / s.forward_m_s);
  frame[k++] = static_cast<float>(reference_[1] / s.yaw_rad_s);
  frame[k++] = static_cast<float>((reference_[2] - 0.5 * (s.height_min_m + s.height_max_m)) /
                                  (0.5 * (s.height_max_m - s.height_min_m)));
  if (s.command_size == 4) {
    frame[k++] = static_cast<float>(reference_[3] / s.roll_rad);
  }
}

double RlPolicyCore::feasible_roll(const double roll, const double height) const {
  const double room = std::max(std::min(height - settings_.leg_height_min_m,
                                        settings_.leg_height_max_m - height),
                               0.0);
  const double limit = std::atan(2.0 * room / settings_.track_width_m);
  return std::clamp(roll, -limit, limit);
}

void RlPolicyCore::shape(const RlPolicyCommand &command) {
  const auto &s = settings_;
  const double dt = policy_dt_s_;
  height_command_m_ = std::clamp(
      height_command_m_ + std::clamp(command.height_rate_m_s, -s.height_rate_m_s, s.height_rate_m_s) * dt,
      s.height_min_m, s.height_max_m);
  const double forward = std::clamp(command.forward_m_s, -s.forward_m_s, s.forward_m_s);
  const double yaw = std::clamp(command.yaw_rate_rad_s, -s.yaw_rad_s, s.yaw_rad_s);
  auto &ref = reference_;
  const double step = s.forward_accel_m_s2 * dt;
  ref[0] += std::clamp(forward - ref[0], -step, step);
  ref[1] += (yaw - ref[1]) * (1.0 - std::exp(-dt / s.yaw_time_constant_s));
  const double height_step = s.height_rate_m_s * dt;
  ref[2] += std::clamp(height_command_m_ - ref[2], -height_step, height_step);
  if (s.command_size == 4) {
    const double target = feasible_roll(std::clamp(command.roll_rad, -s.roll_rad, s.roll_rad), ref[2]);
    const double roll_step = s.roll_rate_rad_s * dt;
    ref[3] += std::clamp(target - ref[3], -roll_step, roll_step);
  } else {
    ref[3] = 0.0;
  }
}

RlPolicyFrame RlPolicyCore::frames_from_action(const std::array<float, 6> &action) const {
  const auto &s = settings_;
  RlPolicyFrame f;
  for (const auto j : kLegs) {
    f.position[j] = std::clamp(s.stand_q[j] + s.leg_scale_rad * action[j], s.joint_min_rad,
                               s.joint_max_rad);
    f.kp[j] = s.leg_kp_nm_rad;
    f.kd[j] = s.leg_kd_nm_s_rad;
  }
  for (const auto j : kWheels) {
    if (s.wheel_velocity_mode) {
      f.velocity[j] = s.wheel_scale_rad_s * action[j];
      f.kd[j] = s.wheel_kd_nm_s_rad;
    } else {
      f.effort[j] = s.wheel_scale_nm * action[j];
    }
  }
  return f;
}

void RlPolicyCore::reset(const RlPolicyState &state) {
  imu_stale_s_ = 0.0;
  stand_elapsed_s_ = 0.0;
  still_s_ = 0.0;
  stand_start_ = state.position;
  last_action_.fill(0.0F);
  bool near_stand = true;
  for (const auto j : kLegs) {
    near_stand = near_stand &&
                 std::abs(state.position[j] - settings_.stand_q[j]) <= settings_.stand_tolerance_rad;
  }
  phase_ = RlPhase::kStand;
  if (near_stand) {
    stand_elapsed_s_ = settings_.stand_s;
  }
}

void RlPolicyCore::engage(const RlPolicyState &state) {
  const auto heights = leg_heights(state);
  const double height = std::clamp(0.5 * (heights[0] + heights[1]), settings_.height_min_m,
                                   settings_.height_max_m);
  reference_ = {0.0, 0.0, height, 0.0};
  height_command_m_ = std::clamp(settings_.initial_height_m, settings_.height_min_m,
                                 settings_.height_max_m);
  last_action_.fill(0.0F);
  build_frame(state, frame_);
  for (int k = 0; k < settings_.history; ++k) {
    observation_.segment(k * frame_size_, frame_size_) = frame_;
  }
  cycle_ = 0;
  phase_ = RlPhase::kPolicy;
}

RlPolicyOutput RlPolicyCore::update(const RlPolicyState &state, const RlPolicyCommand &raw,
                                    const double period_s) {
  const auto &s = settings_;
  RlPolicyOutput out;
  RlPolicyCommand command = raw;
  if (!(command.age_s <= s.command_timeout_s)) {
    command = RlPolicyCommand{};
    out.status |= RL_COMMAND_TIMEOUT;
  }
  const bool fresh = imu_fresh(state);
  imu_stale_s_ = fresh ? 0.0 : imu_stale_s_ + period_s;
  if (!fresh) {
    out.status |= RL_IMU_STALE;
  }
  if (fresh) {
    const Eigen::Matrix3d r = body_rotation(state);
    out.tilt_rad = std::acos(std::clamp(r(2, 2), -1.0, 1.0));
    out.pitch_rad = std::atan2(-r(2, 0), r(2, 2));
    out.roll_rad = std::atan2(r(2, 1), r(2, 2));
    out.rate_rad_s = std::hypot(state.angular_velocity[0], state.angular_velocity[1],
                                state.angular_velocity[2]);
    // Before engaging the body is held, or still settling from a reset; the
    // stand phase only waits for it to be upright.
    if (phase_ == RlPhase::kPolicy && out.tilt_rad > s.fall_angle_rad) {
      phase_ = RlPhase::kFallen;
    }
  }
  if (imu_stale_s_ > s.imu_timeout_s || !finite(state)) {
    phase_ = RlPhase::kFallen;
  }
  out.leg_height_m = leg_heights(state);

  if (phase_ == RlPhase::kFallen) {
    // Latched until re-activation: limp with damping, as the WBC on a fall.
    out.frame = RlPolicyFrame{};
    out.frame.kd.fill(s.fallen_kd_nm_s_rad);
    out.phase = phase_;
    out.status |= RL_FALLEN;
    out.reference = reference_;
    return out;
  }

  if (phase_ == RlPhase::kStand) {
    bool near_stand = true;
    for (const auto j : kLegs) {
      near_stand = near_stand && std::abs(state.position[j] - s.stand_q[j]) <= s.stand_tolerance_rad;
    }
    // Legs already at the stand pose hold it: the start pose may be a stale
    // sample from before a sim reset, and interpolating from it collapses them.
    stand_elapsed_s_ = near_stand ? s.stand_s : std::min(stand_elapsed_s_ + period_s, s.stand_s);
    const double alpha = stand_elapsed_s_ / s.stand_s;
    RlPolicyFrame f;
    for (const auto j : kLegs) {
      f.position[j] = stand_start_[j] + alpha * (s.stand_q[j] - stand_start_[j]);
      f.kp[j] = s.leg_kp_nm_rad;
      f.kd[j] = s.leg_kd_nm_s_rad;
    }
    for (const auto j : kWheels) {
      f.kd[j] = s.wheel_kd_nm_s_rad;
    }
    policy_frame_ = f;
    if (alpha >= 1.0) {
      bool joints_still = true;
      for (const double v : state.velocity) {
        joints_still = joints_still && std::abs(v) <= s.engage_joint_velocity_rad_s;
      }
      const bool still = fresh && joints_still && out.tilt_rad <= s.engage_tilt_rad &&
                         out.rate_rad_s <= s.engage_rate_rad_s;
      still_s_ = still ? still_s_ + period_s : 0.0;
      if (still && still_s_ >= s.engage_hold_s - 1e-9) {
        engage(state);
      } else {
        out.status |= RL_WAITING_UPRIGHT;
      }
    } else {
      still_s_ = 0.0;
    }
  }

  if (phase_ == RlPhase::kPolicy && fresh && cycle_ % cycles_per_action_ == 0) {
    shape(command);
    build_frame(state, frame_);
    const int tail = frame_size_ * (s.history - 1);
    for (int k = tail - 1; k >= 0; --k) {
      observation_[k + frame_size_] = observation_[k];
    }
    observation_.head(frame_size_) = frame_;
    const auto start = std::chrono::steady_clock::now();
    const Eigen::VectorXf &action = policy_.run(observation_);
    out.inference_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    out.inferred = true;
    if (!action.allFinite()) {
      phase_ = RlPhase::kFallen;
      out.status |= RL_NON_FINITE_ACTION | RL_FALLEN;
      out.frame = RlPolicyFrame{};
      out.frame.kd.fill(s.fallen_kd_nm_s_rad);
      out.phase = phase_;
      return out;
    }
    for (std::size_t j = 0; j < 6; ++j) {
      last_action_[j] = action[static_cast<Eigen::Index>(j)];
    }
    policy_frame_ = frames_from_action(last_action_);
  }
  if (phase_ == RlPhase::kPolicy) {
    ++cycle_;
  }

  // Not bounded to the torque the policy trained with: that clip acted on the
  // live joint state inside the drive, and pulling the references toward the
  // delayed measured state instead weakens a saturated wheel enough to fall.
  // The drive's own torque limit is the ceiling.
  out.frame = policy_frame_;
  out.phase = phase_;
  out.reference = reference_;
  out.action = last_action_;
  return out;
}

} // namespace sandro_controllers
