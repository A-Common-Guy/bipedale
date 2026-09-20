#include "sandro_controllers/whole_body_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sandro_controllers {
namespace {

constexpr std::array<std::size_t, 2> kInnerIndex{0, 3};
constexpr std::array<std::size_t, 2> kWheelIndex{2, 5};
constexpr double kPi = 3.14159265358979323846;
// Below this slewed speed command the operator has released the stick.
constexpr double kDrivingThresholdMS = 0.01;

double slew(const double current, const double target, const double rate,
            const double dt) {
  const double step = std::abs(rate) * dt;
  return current + std::clamp(target - current, -step, step);
}

// First-order low-pass coefficient for a cutoff in Hz at a period dt.
double lowpass_alpha(const double cutoff_hz, const double dt) {
  return std::clamp(2.0 * kPi * cutoff_hz * dt, 0.0, 1.0);
}

bool unit_sign(const double value) {
  return std::abs(std::abs(value) - 1.0) < 1e-12;
}

bool finite_all(const WholeBodyParameters &p) {
  const double values[] = {
      p.gain_table_min_height_m,  p.gain_table_max_height_m,
      p.required_delay_margin_s,  p.measured_offset_blend,
      p.leg_offset_limit_m,       p.leg_offset_rate_limit_m_s,
      p.position_error_limit_m,   p.wheel_friction_ff_nm,
      p.wheel_friction_ff_smoothing_nm, p.roll_imu_offset_rad,
      p.trim_rate_per_s,
      p.trim_limit_m,             p.trim_velocity_deadband_m_s,
      p.trim_torque_deadband_nm,
      p.wheel_torque_sign,        p.velocity_filter_hz,
      p.wheel_torque_rate_nm_s,   p.engage_pitch_rad,
      p.engage_pitch_rate_rad_s,  p.height_target_m,
      p.height_min_m,             p.height_max_m,
      p.height_rate_m_s,          p.height_kp_n_m,
      p.height_kd_n_s_m,          p.height_ki_n_m_s,
      p.height_integral_limit_n,  p.fore_aft_neutral_m,
      p.fore_aft_neutral_offset_m,
      p.fore_aft_kp_n_m,          p.fore_aft_kd_n_s_m,
      p.leg_velocity_filter_hz,   p.leg_reaction_ground_gain,
      p.leg_reaction_stator_gain, p.leg_reaction_filter_hz,
      p.roll_lean_gain,           p.roll_kp_m_rad,
      p.roll_ki_m_rad_s,          p.roll_differential_limit_m,
      p.yaw_kp_nm_s_rad,          p.yaw_ki_nm_rad,
      p.yaw_wheel_rate_blend,     p.yaw_scrub_ff_nm,
      p.yaw_scrub_ff_smoothing_rad_s,
      p.yaw_torque_limit_nm,
      p.yaw_torque_rate_nm_s,     p.yaw_torque_sign,
      p.leg_fallback_damping_nm_s_rad,
      p.leg_torque_limit_nm,      p.wheel_torque_limit_nm,
      p.max_forward_velocity_m_s, p.max_yaw_rate_rad_s,
      p.forward_accel_limit_m_s2, p.yaw_command_time_constant_s,
      p.max_height_rate_m_s,
      p.command_timeout_s,        p.imu_timeout_s,
      p.settle_s,                 p.ramp_s,
      p.fall_angle_rad};
  const bool scalars = std::all_of(std::begin(values), std::end(values),
                                   [](const double v) { return std::isfinite(v); });
  const bool weights =
      std::all_of(p.state_weights.begin(), p.state_weights.end(),
                  [](const double v) { return std::isfinite(v); }) &&
      std::all_of(p.effort_weights.begin(), p.effort_weights.end(),
                  [](const double v) { return std::isfinite(v); });
  return scalars && weights;
}

double lowest_offset_bound(const WholeBodyParameters &p) {
  return p.height_min_m - p.height_target_m;
}

double highest_offset_bound(const WholeBodyParameters &p) {
  return p.height_max_m - p.height_target_m;
}

bool fail(std::string *reason, const char *text) {
  if (reason) {
    *reason = text;
  }
  return false;
}

// Largest s in [0, 1] such that |fixed + s * scalable| <= limit on both
// joints; fixed is assumed to be within the limit already.
double impedance_scale(const std::array<double, 2> &fixed,
                       const std::array<double, 2> &scalable,
                       const double limit) {
  double scale = 1.0;
  for (std::size_t i = 0; i < 2; ++i) {
    if (std::abs(scalable[i]) < 1e-12) {
      continue;
    }
    const double room = scalable[i] > 0.0 ? limit - fixed[i] : limit + fixed[i];
    scale = std::min(scale, std::max(0.0, room / std::abs(scalable[i])));
  }
  return scale;
}

} // namespace

std::string whole_body_status_text(const std::uint32_t status) {
  if (status == WBC_OK) {
    return "ok";
  }
  std::string text;
  const auto add = [&text, status](const std::uint32_t bit, const char *name) {
    if ((status & bit) != 0U) {
      if (!text.empty()) {
        text += ",";
      }
      text += name;
    }
  };
  add(WBC_COMMAND_STALE, "command_stale");
  add(WBC_IMU_STALE, "imu_stale");
  add(WBC_FALL_LATCH, "fall_latch");
  add(WBC_SETTLING, "settling_or_ramping");
  add(WBC_LEG_KINEMATICS, "leg_kinematics");
  add(WBC_LEG_TORQUE_LIMIT, "leg_torque_limit");
  add(WBC_WHEEL_TORQUE_LIMIT, "wheel_torque_limit");
  add(WBC_LEG_OFFSET_LIMIT, "leg_offset_limit");
  add(WBC_POSITION_ERROR_LIMIT, "position_error_limit");
  add(WBC_HEIGHT_LIMIT, "height_limit");
  add(WBC_ROLL_LIMIT, "roll_limit");
  add(WBC_YAW_TORQUE_LIMIT, "yaw_torque_limit");
  add(WBC_INVALID_SETTINGS, "invalid_settings");
  return text;
}

WholeBodyGains GainSchedule::at(const double leg_height_m) const {
  if (count <= 0) {
    return WholeBodyGains{};
  }
  if (count == 1 || !std::isfinite(leg_height_m) ||
      leg_height_m <= points[0].leg_height_m) {
    return points[0].gains;
  }
  const int last = count - 1;
  if (leg_height_m >= points[last].leg_height_m) {
    return points[last].gains;
  }
  int upper = 1;
  while (upper < last && points[upper].leg_height_m < leg_height_m) {
    ++upper;
  }
  const auto &lo = points[upper - 1];
  const auto &hi = points[upper];
  const double span = hi.leg_height_m - lo.leg_height_m;
  const double t = span > 0.0 ? (leg_height_m - lo.leg_height_m) / span : 0.0;
  WholeBodyGains result{};
  for (int i = 0; i < kWholeBodyInputs; ++i) {
    for (int j = 0; j < kWholeBodyStates; ++j) {
      result[i][j] = lo.gains[i][j] + t * (hi.gains[i][j] - lo.gains[i][j]);
    }
  }
  return result;
}

bool valid_whole_body_parameters(const WholeBodyParameters &p,
                                 std::string *reason) {
  if (!finite_all(p)) {
    return fail(reason, "a parameter is not finite");
  }
  if (!valid_model(p.model)) {
    return fail(reason, "model parameters are invalid");
  }
  if (std::any_of(p.state_weights.begin(), p.state_weights.end(),
                  [](const double v) { return v < 0.0; }) ||
      p.state_weights[2] <= 0.0) {
    return fail(reason, "state weights must be >= 0 with a positive pitch weight");
  }
  if (std::any_of(p.effort_weights.begin(), p.effort_weights.end(),
                  [](const double v) { return v <= 0.0; })) {
    return fail(reason, "effort weights must be > 0");
  }
  if (p.gain_table_points < 2 || p.gain_table_points > kMaxGainPoints) {
    return fail(reason, "gain table needs between 2 and 32 points");
  }
  if (!(p.gain_table_min_height_m > 0.0 &&
        p.gain_table_min_height_m < p.gain_table_max_height_m)) {
    return fail(reason, "gain table height range is invalid");
  }
  if (p.required_delay_margin_s < 0.0 || p.measured_offset_blend < 0.0 ||
      p.measured_offset_blend > 1.0) {
    return fail(reason, "delay margin must be >= 0 and the measured offset blend within [0, 1]");
  }
  if (p.leg_offset_limit_m < 0.0 || p.leg_offset_rate_limit_m_s <= 0.0 ||
      p.position_error_limit_m <= 0.0 || p.trim_rate_per_s < 0.0 ||
      p.trim_limit_m < 0.0 || p.trim_velocity_deadband_m_s < 0.0 ||
      p.wheel_friction_ff_nm < 0.0 || p.wheel_friction_ff_smoothing_nm <= 0.0) {
    return fail(reason, "balance limits are invalid");
  }
  if (p.wheel_friction_ff_nm >= p.wheel_torque_limit_nm) {
    return fail(reason, "wheel friction feed-forward must stay below the wheel torque limit");
  }
  if (!unit_sign(p.wheel_torque_sign) || !unit_sign(p.yaw_torque_sign)) {
    return fail(reason, "signs must be exactly +1 or -1");
  }
  if (p.trim_torque_deadband_nm <= 0.0) {
    return fail(reason, "trim torque deadband must be > 0");
  }
  if (p.wheel_torque_rate_nm_s <= 0.0 || p.engage_pitch_rad <= 0.0 ||
      p.engage_pitch_rate_rad_s <= 0.0 || p.velocity_filter_hz <= 0.0) {
    return fail(reason, "wheel torque rate, velocity filter and engage thresholds must be > 0");
  }
  if (!(p.height_min_m > 0.0 && p.height_min_m <= p.height_target_m &&
        p.height_target_m <= p.height_max_m) ||
      p.height_rate_m_s <= 0.0 || p.height_kp_n_m < 0.0 ||
      p.height_kd_n_s_m < 0.0 || p.height_ki_n_m_s < 0.0 ||
      p.height_integral_limit_n < 0.0) {
    return fail(reason, "height task parameters are invalid");
  }
  if (p.fore_aft_kp_n_m < 0.0 || p.fore_aft_kd_n_s_m < 0.0 ||
      p.leg_velocity_filter_hz <= 0.0 || p.leg_reaction_ground_gain < 0.0 ||
      p.leg_reaction_ground_gain > 1.0 || p.leg_reaction_stator_gain < 0.0 ||
      p.leg_reaction_stator_gain > 1.0 || p.leg_reaction_filter_hz <= 0.0) {
    return fail(reason, "fore-aft gains must be >= 0, the leg filters > 0 and the reaction gains within [0, 1]");
  }
  if (p.roll_lean_gain < 0.0 || p.roll_kp_m_rad < 0.0 ||
      p.roll_ki_m_rad_s < 0.0 || p.roll_differential_limit_m < 0.0) {
    return fail(reason, "roll task parameters must be >= 0");
  }
  if (p.yaw_kp_nm_s_rad < 0.0 || p.yaw_ki_nm_rad < 0.0 ||
      p.yaw_wheel_rate_blend < 0.0 || p.yaw_wheel_rate_blend > 1.0 ||
      p.yaw_scrub_ff_nm < 0.0 || p.yaw_scrub_ff_smoothing_rad_s <= 0.0 ||
      p.yaw_torque_limit_nm < 0.0 ||
      p.yaw_torque_rate_nm_s <= 0.0 || p.leg_fallback_damping_nm_s_rad < 0.0) {
    return fail(reason, "yaw task parameters are invalid");
  }
  if (p.yaw_scrub_ff_nm > p.yaw_torque_limit_nm) {
    return fail(reason, "yaw scrub feed-forward must stay within the yaw torque limit");
  }
  if (p.leg_torque_limit_nm <= 0.0 || p.wheel_torque_limit_nm <= 0.0) {
    return fail(reason, "torque limits must be > 0");
  }
  if (p.max_forward_velocity_m_s < 0.0 || p.max_yaw_rate_rad_s < 0.0 ||
      p.forward_accel_limit_m_s2 <= 0.0 ||
      p.yaw_command_time_constant_s <= 0.0 || p.max_height_rate_m_s < 0.0) {
    return fail(reason, "drive limits are invalid");
  }
  if (p.command_timeout_s <= 0.0 || p.imu_timeout_s <= 0.0 ||
      p.settle_s < 0.0 || p.ramp_s < 0.0 || p.fall_angle_rad <= 0.0) {
    return fail(reason, "safety parameters are invalid");
  }
  if (com_height(p.model, p.height_min_m) <= 0.02 ||
      com_height(p.model, p.gain_table_min_height_m) <= 0.02 ||
      pitch_inertia(p.model, p.height_min_m) <= 0.0 ||
      pitch_inertia(p.model, p.gain_table_min_height_m) <= 0.0) {
    return fail(reason, "centre-of-mass height or inertia is not positive at the lowest height");
  }
  // Every commanded Cartesian target must stay inside both leg workspaces
  // with a margin from the stretched singularity.
  const std::array<const LegGeometry *, 2> geometry{&p.model.right_geometry,
                                                    &p.model.left_geometry};
  const std::array<double, 2> forward_sign{p.model.right_forward_sign,
                                           p.model.left_forward_sign};
  for (std::size_t side = 0; side < 2; ++side) {
    const double neutral =
        p.fore_aft_neutral_m + forward_sign[side] * p.fore_aft_neutral_offset_m;
    for (const double y : {p.height_min_m, p.height_max_m}) {
      for (const double x : {neutral - p.leg_offset_limit_m, neutral,
                             neutral + p.leg_offset_limit_m}) {
        if (!inside_leg_workspace(x, y, *geometry[side], kLegJointLimitRad)) {
          return fail(reason, "height range, neutral offset and leg offset limit leave the leg workspace");
        }
      }
    }
  }
  return true;
}

bool build_whole_body_settings(const WholeBodyParameters &p,
                               const double control_period_s,
                               WholeBodySettings &settings,
                               std::string *reason) {
  settings = WholeBodySettings{};
  settings.parameters = p;
  settings.control_period_s = control_period_s;
  if (!valid_whole_body_parameters(p, reason)) {
    return false;
  }
  if (!std::isfinite(control_period_s) || control_period_s <= 0.0) {
    return fail(reason, "control period must be > 0");
  }
  lqr::StateMatrix Q = lqr::StateMatrix::Zero();
  for (int i = 0; i < kWholeBodyStates; ++i) {
    Q(i, i) = p.state_weights[static_cast<std::size_t>(i)];
  }
  lqr::InputWeight R = lqr::InputWeight::Zero();
  for (int i = 0; i < kWholeBodyInputs; ++i) {
    R(i, i) = p.effort_weights[static_cast<std::size_t>(i)];
  }
  const int n = p.gain_table_points;
  int delay_margin_samples = kMaxDelayMarginSamples;
  for (int i = 0; i < n; ++i) {
    const double height =
        p.gain_table_min_height_m +
        (p.gain_table_max_height_m - p.gain_table_min_height_m) *
            static_cast<double>(i) / static_cast<double>(n - 1);
    const LinearModel continuous = linearize(p.model, height);
    const lqr::DiscreteModel discrete =
        lqr::discretize(continuous.A, continuous.B, control_period_s);
    const lqr::DareSolution solution =
        lqr::solve_dare(discrete.A, discrete.B, Q, R);
    if (!solution.converged) {
      return fail(reason, "Riccati equation did not yield a stabilising gain");
    }
    delay_margin_samples = std::min(
        delay_margin_samples,
        lqr::delay_margin_samples(discrete.A, discrete.B, solution.K,
                                  kMaxDelayMarginSamples));
    GainPoint &point = settings.gains.points[static_cast<std::size_t>(i)];
    point.leg_height_m = height;
    for (int r = 0; r < kWholeBodyInputs; ++r) {
      for (int c = 0; c < kWholeBodyStates; ++c) {
        point.gains[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)] =
            solution.K(r, c);
      }
    }
  }
  settings.gains.count = n;
  settings.delay_margin_s =
      std::max(0, delay_margin_samples) * control_period_s;
  if (settings.delay_margin_s < p.required_delay_margin_s) {
    return fail(reason, "balance weights leave less input-delay margin than required (raise effort_weights.wheel_torque or lower required_delay_margin_s)");
  }
  settings.valid = true;
  return true;
}

void WholeBodyCore::reset(const WholeBodyState &state,
                          const WholeBodySettings &settings) {
  const auto &p = settings.parameters;
  elapsed_s_ = 0.0;
  ramp_elapsed_s_ = 0.0;
  engaged_ = false;
  fallen_ = false;
  velocity_command_m_s_ = 0.0;
  yaw_rate_command_rad_s_ = 0.0;
  driving_ = false;
  height_offset_command_m_ = 0.0;
  height_differential_m_ = 0.0;
  height_integral_n_ = 0.0;
  roll_integral_m_ = 0.0;
  trim_m_ = 0.0;
  balance_torque_nm_ = 0.0;
  yaw_torque_nm_ = 0.0;
  yaw_integral_nm_ = 0.0;
  have_filtered_velocity_ = false;
  filtered_velocity_m_s_ = 0.0;
  filtered_wheel_yaw_rate_rad_s_ = 0.0;
  filtered_leg_velocity_m_s_ = {};
  have_filtered_leg_velocity_ = {};
  filtered_wheel_torque_nm_ = {};
  position_reference_m_ =
      0.5 * (state.position[kWheelIndex[0]] + state.position[kWheelIndex[1]]) *
      p.model.wheel_radius_m;

  // Start the height and fore-aft references where the legs actually are so
  // activation does not step the leg impedance; both then slew to target.
  const std::array<const LegGeometry *, 2> geometry{&p.model.right_geometry,
                                                    &p.model.left_geometry};
  const std::array<double, 2> forward_sign{p.model.right_forward_sign,
                                           p.model.left_forward_sign};
  double height_sum = 0.0;
  double offset_sum = 0.0;
  int count = 0;
  for (std::size_t side = 0; side < 2; ++side) {
    const auto point =
        forward_kinematics(state.position[kInnerIndex[side]],
                           state.position[kInnerIndex[side] + 1], *geometry[side]);
    if (point) {
      const double neutral =
          p.fore_aft_neutral_m + forward_sign[side] * p.fore_aft_neutral_offset_m;
      height_sum += point->y;
      offset_sum += -forward_sign[side] * (point->x - neutral);
      ++count;
    }
  }
  height_reference_m_ =
      count ? std::clamp(height_sum / count, p.height_min_m, p.height_max_m)
            : p.height_target_m;
  leg_offset_command_m_ =
      count ? std::clamp(offset_sum / count, -p.leg_offset_limit_m,
                         p.leg_offset_limit_m)
            : 0.0;
}

WholeBodyOutput WholeBodyCore::update(const WholeBodyState &state,
                                      const WholeBodyCommand &command,
                                      const WholeBodySettings &settings,
                                      const double period_s) {
  WholeBodyOutput out;
  if (!settings.valid || !std::isfinite(period_s) || period_s <= 0.0) {
    out.status = WBC_INVALID_SETTINGS;
    return out;
  }
  const auto &p = settings.parameters;
  const auto &m = p.model;
  const double dt = period_s;

  // ---- commands -----------------------------------------------------------
  double forward = command.forward_velocity_m_s;
  double yaw = command.yaw_rate_rad_s;
  double height_rate = command.height_rate_m_s;
  if (!std::isfinite(forward) || !std::isfinite(yaw) ||
      !std::isfinite(height_rate) || !std::isfinite(command.age_s) ||
      command.age_s > p.command_timeout_s) {
    forward = 0.0;
    yaw = 0.0;
    height_rate = 0.0;
    out.status |= WBC_COMMAND_STALE;
  }
  forward = std::clamp(forward, -p.max_forward_velocity_m_s,
                       p.max_forward_velocity_m_s);
  yaw = std::clamp(yaw, -p.max_yaw_rate_rad_s, p.max_yaw_rate_rad_s);
  height_rate =
      std::clamp(height_rate, -p.max_height_rate_m_s, p.max_height_rate_m_s);
  velocity_command_m_s_ =
      slew(velocity_command_m_s_, forward, p.forward_accel_limit_m_s2, dt);
  // The yaw command follows the operator through a first-order lag; its
  // derivative is the acceleration the inertia feed-forward uses.
  const double yaw_lag = std::min(1.0, dt / p.yaw_command_time_constant_s);
  const double yaw_accel =
      (yaw - yaw_rate_command_rad_s_) * yaw_lag / std::max(dt, 1e-9);
  yaw_rate_command_rad_s_ += (yaw - yaw_rate_command_rad_s_) * yaw_lag;
  out.velocity_reference_m_s = velocity_command_m_s_;
  out.yaw_rate_reference_rad_s = yaw_rate_command_rad_s_;

  // ---- estimate -----------------------------------------------------------
  const Attitude attitude =
      attitude_from_quaternion(state.quaternion_w, state.quaternion_x,
                               state.quaternion_y, state.quaternion_z);
  const auto &rates = state.angular_velocity_rad_s;
  const bool imu_valid =
      attitude.valid && std::isfinite(rates[0]) && std::isfinite(rates[1]) &&
      std::isfinite(rates[2]) && std::isfinite(state.imu_age_s) &&
      state.imu_age_s >= 0.0 && state.imu_age_s <= p.imu_timeout_s;
  const double pitch = imu_valid ? attitude.pitch_rad : 0.0;
  const double roll = imu_valid ? attitude.roll_rad - p.roll_imu_offset_rad : 0.0;
  const double pitch_rate = imu_valid ? rates[1] : 0.0;
  const double yaw_rate = imu_valid ? rates[2] : 0.0;
  out.pitch_rad = pitch;
  out.roll_rad = roll;
  out.pitch_rate_rad_s = pitch_rate;
  out.yaw_rate_rad_s = yaw_rate;

  const std::array<const LegGeometry *, 2> geometry{&m.right_geometry,
                                                    &m.left_geometry};
  const std::array<double, 2> forward_sign{m.right_forward_sign,
                                           m.left_forward_sign};
  const std::array<bool, 2> wheel_on_outer{m.right_wheel_on_outer_link != 0.0,
                                           m.left_wheel_on_outer_link != 0.0};
  std::array<LegState, 2> legs{};
  double height_sum = 0.0;
  double offset_sum = 0.0;
  int leg_count = 0;
  const double leg_alpha = lowpass_alpha(p.leg_velocity_filter_hz, dt);
  for (std::size_t side = 0; side < 2; ++side) {
    const std::size_t inner = kInnerIndex[side];
    legs[side] = leg_state(state.position[inner], state.position[inner + 1],
                           state.velocity[inner], state.velocity[inner + 1],
                           *geometry[side], wheel_on_outer[side]);
    // The five-bar is periodic in its joint angles, so a multi-turn encoder
    // wrap or a wrong zero would pass FK unnoticed; the joint range catches it.
    if (std::abs(state.position[inner]) > kLegJointLimitRad ||
        std::abs(state.position[inner + 1]) > kLegJointLimitRad) {
      legs[side].valid = false;
    }
    out.leg_valid[side] = legs[side].valid;
    if (legs[side].valid) {
      out.leg_position_m[side] = legs[side].position;
      height_sum += legs[side].position.y;
      const double neutral =
          p.fore_aft_neutral_m + forward_sign[side] * p.fore_aft_neutral_offset_m;
      offset_sum += -forward_sign[side] * (legs[side].position.x - neutral);
      ++leg_count;
      // Low-pass the Cartesian velocity the damping terms act on: the
      // encoder estimate arrives held and delayed, and unfiltered damping on
      // the light leg chattered at the sample rate.
      Point2 &filtered = filtered_leg_velocity_m_s_[side];
      if (have_filtered_leg_velocity_[side]) {
        filtered.x += leg_alpha * (legs[side].velocity.x - filtered.x);
        filtered.y += leg_alpha * (legs[side].velocity.y - filtered.y);
      } else {
        filtered = legs[side].velocity;
        have_filtered_leg_velocity_[side] = true;
      }
    } else {
      out.status |= WBC_LEG_KINEMATICS;
      have_filtered_leg_velocity_[side] = false;
    }
  }
  const double leg_height =
      leg_count ? height_sum / leg_count : height_reference_m_;
  const double leg_offset =
      leg_count ? offset_sum / leg_count : leg_offset_command_m_;
  out.leg_height_m = leg_height;
  out.leg_offset_m = leg_offset;

  // Encoders turn relative to the chassis; add the body pitch back to get
  // the absolute wheel rotation, hence ground position and speed. The
  // wheel-differential rate is the yaw rate of the axle line.
  const double mean_wheel_angle =
      0.5 * (state.position[kWheelIndex[0]] + state.position[kWheelIndex[1]]);
  const double mean_wheel_rate =
      0.5 * (state.velocity[kWheelIndex[0]] + state.velocity[kWheelIndex[1]]);
  const double position = (mean_wheel_angle + pitch) * m.wheel_radius_m;
  const double raw_velocity = (mean_wheel_rate + pitch_rate) * m.wheel_radius_m;
  const double raw_wheel_yaw_rate =
      (state.velocity[kWheelIndex[0]] - state.velocity[kWheelIndex[1]]) *
      m.wheel_radius_m / m.track_width_m;
  const double alpha = lowpass_alpha(p.velocity_filter_hz, dt);
  if (have_filtered_velocity_) {
    filtered_velocity_m_s_ += alpha * (raw_velocity - filtered_velocity_m_s_);
    filtered_wheel_yaw_rate_rad_s_ +=
        alpha * (raw_wheel_yaw_rate - filtered_wheel_yaw_rate_rad_s_);
  } else {
    filtered_velocity_m_s_ = raw_velocity;
    filtered_wheel_yaw_rate_rad_s_ = raw_wheel_yaw_rate;
    have_filtered_velocity_ = true;
  }
  const double velocity = filtered_velocity_m_s_;
  out.position_m = position;
  out.forward_velocity_m_s = velocity;
  out.wheel_yaw_rate_rad_s = filtered_wheel_yaw_rate_rad_s_;

  // ---- fall latch / IMU gate ---------------------------------------------
  // A fall while balancing latches every actuator off until the controller
  // is re-activated. Not being upright before balancing has engaged is the
  // normal starting condition (the operator is still lifting the robot, or
  // the simulation is about to be reset), so it only defers engagement.
  if (imu_valid && engaged_ && std::abs(pitch) > p.fall_angle_rad) {
    fallen_ = true;
  }
  out.fallen = fallen_;
  const bool balance_active = imu_valid && !fallen_;
  if (!balance_active) {
    out.status |= fallen_ ? WBC_FALL_LATCH : WBC_IMU_STALE;
    balance_torque_nm_ = 0.0;
    yaw_torque_nm_ = 0.0;
    yaw_integral_nm_ = 0.0;
    yaw_rate_command_rad_s_ = 0.0;
    roll_integral_m_ = 0.0;
    engaged_ = false;
    ramp_elapsed_s_ = 0.0;
    elapsed_s_ = 0.0;
  }
  if (fallen_) {
    // On the ground: every actuator off until the controller is re-activated.
    return out;
  }

  // ---- engage / ramp ------------------------------------------------------
  const double com = std::max(0.02, com_height(m, leg_height));
  // The trim is the centre-of-mass correction the posture needs; the legs
  // provide what they can and the body leans for the remainder.
  const double offset_reference =
      std::clamp(trim_m_, -p.leg_offset_limit_m, p.leg_offset_limit_m);
  const double pitch_reference = (trim_m_ - offset_reference) / com;
  out.trim_m = trim_m_;
  out.leg_offset_reference_m = offset_reference;
  out.pitch_reference_rad = pitch_reference;

  double ramp = 0.0;
  if (balance_active) {
    elapsed_s_ += dt;
    if (!engaged_) {
      // Both legs must evaluate: a joint reading outside the five-bar
      // workspace at this point is almost certainly a wrong encoder zero or a
      // multi-turn wrap at power-on, and balancing on it would be blind.
      if (elapsed_s_ >= p.settle_s && leg_count == 2 &&
          std::abs(pitch - pitch_reference) <= p.engage_pitch_rad &&
          std::abs(pitch_rate) <= p.engage_pitch_rate_rad_s) {
        engaged_ = true;
        ramp_elapsed_s_ = 0.0;
        // Anchor every reference to where the robot actually is: the state
        // may have jumped since activation (operator lifting the robot, or a
        // simulation reset) and the impedance must not step.
        position_reference_m_ = position;
        // The trim is deliberately not seeded from the lever measured here.
        // At engagement the legs stand wherever the impedance or a reset
        // pose left them, not under the centre of mass, and reading that
        // placement as a mass offset sent the robot into a runaway (a 26 mm
        // leg offset became a 1.3 Nm gravity lever the wheels cannot hold).
        // The neutral offset carries the identified mass; the trim learns
        // the rest at rest, at trim_rate_per_s.
        if (leg_count) {
          height_reference_m_ =
              std::clamp(leg_height, p.height_min_m, p.height_max_m);
          height_offset_command_m_ =
              std::clamp(height_offset_command_m_, lowest_offset_bound(p),
                         highest_offset_bound(p));
          leg_offset_command_m_ = std::clamp(leg_offset, -p.leg_offset_limit_m,
                                             p.leg_offset_limit_m);
        }
      }
    } else {
      ramp_elapsed_s_ += dt;
    }
    ramp = engaged_
               ? (p.ramp_s <= 0.0
                      ? 1.0
                      : std::clamp(ramp_elapsed_s_ / p.ramp_s, 0.0, 1.0))
               : 0.0;
  }
  out.engaged = engaged_;
  out.ramp = ramp;
  if (ramp < 1.0) {
    out.status |= WBC_SETTLING;
  }
  const bool full_authority = ramp >= 1.0;

  // ---- height reference ---------------------------------------------------
  const double lowest_offset = lowest_offset_bound(p);
  const double highest_offset = highest_offset_bound(p);
  const double unclamped_offset =
      height_offset_command_m_ + ramp * height_rate * dt;
  height_offset_command_m_ =
      std::clamp(unclamped_offset, lowest_offset, highest_offset);
  if (height_offset_command_m_ != unclamped_offset) {
    out.status |= WBC_HEIGHT_LIMIT;
  }
  height_reference_m_ =
      std::clamp(slew(height_reference_m_,
                      p.height_target_m + height_offset_command_m_,
                      p.height_rate_m_s, dt),
                 p.height_min_m, p.height_max_m);
  out.height_reference_m = height_reference_m_;

  // ---- roll: lean into turns, level on uneven ground ----------------------
  // Lateral acceleration v * yaw_rate must be balanced by the body leaning
  // toward the inside of the turn. With the wheels on the ground the body
  // rolls by (left height - right height) / track, positive right-side-down.
  const double lean = std::atan(velocity_command_m_s_ * yaw_rate_command_rad_s_ /
                                m.gravity_m_s2);
  const double roll_reference = -p.roll_lean_gain * lean;
  out.roll_reference_rad = roll_reference;
  const double roll_error = roll_reference - roll;
  if (balance_active && full_authority) {
    roll_integral_m_ = std::clamp(roll_integral_m_ + p.roll_ki_m_rad_s * roll_error * dt,
                                  -p.roll_differential_limit_m,
                                  p.roll_differential_limit_m);
  }
  const double raw_differential =
      balance_active ? m.track_width_m * std::tan(roll_reference) +
                           p.roll_kp_m_rad * roll_error + roll_integral_m_
                     : 0.0;
  const double clamped_differential =
      std::clamp(raw_differential, -p.roll_differential_limit_m,
                 p.roll_differential_limit_m);
  if (clamped_differential != raw_differential) {
    out.status |= WBC_ROLL_LIMIT;
  }
  height_differential_m_ = ramp * clamped_differential;
  out.height_differential_m = height_differential_m_;
  const std::array<double, 2> height_reference{
      std::clamp(height_reference_m_ - 0.5 * height_differential_m_,
                 p.height_min_m, p.height_max_m),
      std::clamp(height_reference_m_ + 0.5 * height_differential_m_,
                 p.height_min_m, p.height_max_m)};

  // ---- balance: LQR over wheels and leg placement ------------------------
  double leg_offset_rate = 0.0;
  // The offset state the regulator sees: what it commanded the legs to do,
  // blended with the measured axle position if asked. The legs are a stiff
  // inner loop; their residual error (stiction, torque reaction) then acts
  // on the pendulum like any small centre-of-mass error, which the pitch
  // loop and the trim absorb, instead of being re-injected as balance action.
  const double leg_offset_state =
      p.measured_offset_blend * leg_offset +
      (1.0 - p.measured_offset_blend) * leg_offset_command_m_;
  out.leg_offset_state_m = leg_offset_state;
  if (balance_active) {
    // The reference integrates the commanded speed while driving; the
    // error limit below then acts as the integral action that makes the
    // speed track against friction. Releasing the stick means "stop here":
    // the hold is anchored at the current position, so a speed the robot
    // could not follow never turns into a catch-up creep afterwards.
    const bool driving = std::abs(velocity_command_m_s_) > kDrivingThresholdMS;
    if (!engaged_) {
      position_reference_m_ = position;
    } else if (driving) {
      position_reference_m_ += velocity_command_m_s_ * dt;
    } else if (driving_) {
      position_reference_m_ = position;
    }
    driving_ = driving;
    const double lower = position - p.position_error_limit_m;
    const double upper = position + p.position_error_limit_m;
    const double clamped_reference =
        std::clamp(position_reference_m_, lower, upper);
    if (clamped_reference != position_reference_m_) {
      out.status |= WBC_POSITION_ERROR_LIMIT;
      position_reference_m_ = clamped_reference;
    }
    // Posture trim: at rest the gravity lever com * pitch + leg_offset is
    // whatever unmodelled offset the body is balancing; filter it and move
    // the axle there instead of holding the lean. Only at rest: while the
    // robot accelerates or brakes the same lever is the balancing lean, and
    // learning it as a mass offset shifts the equilibrium under the robot.
    // A lean held against wheel stiction shows as a steady balance torque
    // with no motion; that is friction, not mass, so it is excluded too. The
    // lever uses the measured axle position: it is the physical one.
    if (full_authority &&
        std::abs(velocity_command_m_s_) <= p.trim_velocity_deadband_m_s &&
        std::abs(velocity) <= p.trim_velocity_deadband_m_s &&
        std::abs(balance_torque_nm_) <= p.trim_torque_deadband_nm) {
      const double lever = com * pitch + leg_offset;
      trim_m_ = std::clamp(
          trim_m_ + p.trim_rate_per_s * (lever - trim_m_) * dt,
          -p.trim_limit_m, p.trim_limit_m);
    }

    out.gains = settings.gains.at(leg_height);
    const std::array<double, kWholeBodyStates> error{
        position - position_reference_m_, velocity - velocity_command_m_s_,
        pitch - pitch_reference, pitch_rate,
        leg_offset_state - offset_reference};
    std::array<double, kWholeBodyInputs> input{};
    for (int i = 0; i < kWholeBodyInputs; ++i) {
      double sum = 0.0;
      for (int j = 0; j < kWholeBodyStates; ++j) {
        sum -= out.gains[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] *
               error[static_cast<std::size_t>(j)];
      }
      input[static_cast<std::size_t>(i)] = sum;
    }
    const double total_limit = 2.0 * p.wheel_torque_limit_nm;
    const double clipped_torque =
        std::clamp(input[0], -total_limit, total_limit);
    if (clipped_torque != input[0]) {
      out.status |= WBC_WHEEL_TORQUE_LIMIT;
    }
    balance_torque_nm_ = slew(balance_torque_nm_, ramp * clipped_torque,
                              2.0 * p.wheel_torque_rate_nm_s, dt);
    leg_offset_rate =
        ramp * std::clamp(input[1], -p.leg_offset_rate_limit_m_s,
                          p.leg_offset_rate_limit_m_s);
  }
  out.balance_torque_nm = balance_torque_nm_;
  out.leg_offset_rate_m_s = leg_offset_rate;
  out.position_reference_m = position_reference_m_;

  if (engaged_ && balance_active) {
    const double unclamped = leg_offset_command_m_ + leg_offset_rate * dt;
    leg_offset_command_m_ =
        std::clamp(unclamped, -p.leg_offset_limit_m, p.leg_offset_limit_m);
    if (leg_offset_command_m_ != unclamped) {
      out.status |= WBC_LEG_OFFSET_LIMIT;
    }
  } else {
    // Relax toward neutral while the operator holds the robot.
    leg_offset_command_m_ =
        slew(leg_offset_command_m_, 0.0, 0.2 * p.leg_offset_rate_limit_m_s, dt);
  }
  out.leg_offset_command_m = leg_offset_command_m_;

  // ---- yaw: differential wheel torque ------------------------------------
  // I_z psi_dd = (track / 2r) dtau - scrub. Proportional damping on the
  // wheel-differential rate (synchronous, free of body twist on the legs),
  // integral on the IMU rate (the body's true rate), Coulomb-like scrub and
  // inertia fed forward from the lagged command.
  const double moment_per_torque = m.track_width_m / (2.0 * m.wheel_radius_m);
  if (balance_active) {
    const double feedback_rate =
        p.yaw_wheel_rate_blend * filtered_wheel_yaw_rate_rad_s_ +
        (1.0 - p.yaw_wheel_rate_blend) * yaw_rate;
    const double proportional_error = yaw_rate_command_rad_s_ - feedback_rate;
    const double integral_error = yaw_rate_command_rad_s_ - yaw_rate;
    if (full_authority) {
      yaw_integral_nm_ = std::clamp(
          yaw_integral_nm_ + p.yaw_ki_nm_rad * integral_error * dt,
          -p.yaw_torque_limit_nm, p.yaw_torque_limit_nm);
    }
    const double feedforward =
        ramp * (m.yaw_inertia_kgm2 * yaw_accel / moment_per_torque +
                p.yaw_scrub_ff_nm *
                    std::tanh(yaw_rate_command_rad_s_ /
                              p.yaw_scrub_ff_smoothing_rad_s));
    out.yaw_feedforward_nm = feedforward;
    const double raw_yaw =
        p.yaw_torque_sign *
        (p.yaw_kp_nm_s_rad * proportional_error + yaw_integral_nm_ + feedforward);
    const double clipped_yaw =
        std::clamp(raw_yaw, -p.yaw_torque_limit_nm, p.yaw_torque_limit_nm);
    if (clipped_yaw != raw_yaw) {
      out.status |= WBC_YAW_TORQUE_LIMIT;
      // Do not let the integral hold authority the clamp discards.
      yaw_integral_nm_ = std::clamp(
          yaw_integral_nm_ + p.yaw_torque_sign * (clipped_yaw - raw_yaw),
          -p.yaw_torque_limit_nm, p.yaw_torque_limit_nm);
    }
    yaw_torque_nm_ =
        slew(yaw_torque_nm_, ramp * clipped_yaw, p.yaw_torque_rate_nm_s, dt);
  }
  // Balance keeps priority over yaw inside the per-wheel budget.
  const double available =
      std::max(0.0, p.wheel_torque_limit_nm - 0.5 * std::abs(balance_torque_nm_));
  const double differential =
      std::clamp(yaw_torque_nm_, -2.0 * available, 2.0 * available);
  if (differential != yaw_torque_nm_) {
    out.status |= WBC_WHEEL_TORQUE_LIMIT;
  }
  out.yaw_torque_nm = differential;
  // Coulomb friction feed-forward per wheel in the direction of the torque
  // being commanded (a smoothed dead-zone inverse), so friction does not eat
  // the command before the wheel moves. Only while balancing: it must never
  // move a robot that is being held.
  const auto with_friction_ff = [&](const double command) {
    if (!balance_active || !engaged_ || p.wheel_friction_ff_nm <= 0.0) {
      return command;
    }
    return command + ramp * p.wheel_friction_ff_nm *
                         std::tanh(command / p.wheel_friction_ff_smoothing_nm);
  };
  // Torque each wheel delivers to the ground, forward-positive, in the
  // logical joint convention; the friction feed-forward is dissipated in the
  // drive train and does not reach the ground or the stator.
  const std::array<double, 2> wheel_torque{0.5 * (balance_torque_nm_ + differential),
                                           0.5 * (balance_torque_nm_ - differential)};
  const double reaction_alpha = lowpass_alpha(p.leg_reaction_filter_hz, dt);
  for (std::size_t side = 0; side < 2; ++side) {
    filtered_wheel_torque_nm_[side] +=
        reaction_alpha * (wheel_torque[side] - filtered_wheel_torque_nm_[side]);
  }
  const double raw_right =
      p.wheel_torque_sign * with_friction_ff(wheel_torque[0]);
  const double raw_left =
      p.wheel_torque_sign * with_friction_ff(wheel_torque[1]);
  out.effort_nm[kWheelIndex[0]] =
      std::clamp(raw_right, -p.wheel_torque_limit_nm, p.wheel_torque_limit_nm);
  out.effort_nm[kWheelIndex[1]] =
      std::clamp(raw_left, -p.wheel_torque_limit_nm, p.wheel_torque_limit_nm);
  if (out.effort_nm[kWheelIndex[0]] != raw_right ||
      out.effort_nm[kWheelIndex[1]] != raw_left) {
    out.status |= WBC_WHEEL_TORQUE_LIMIT;
  }

  // ---- legs: Cartesian impedance through J^T -----------------------------
  // Slow integral on the mean height error: whatever share of the weight the
  // feed-forward misses is learnt while balancing, per leg.
  if (balance_active && engaged_ && leg_count == 2) {
    const double height_error = height_reference_m_ - leg_height;
    height_integral_n_ = std::clamp(
        height_integral_n_ + p.height_ki_n_m_s * height_error * dt,
        -p.height_integral_limit_n, p.height_integral_limit_n);
  }
  const double support_share =
      0.5 * m.body_mass_kg * m.gravity_m_s2 + height_integral_n_;
  out.height_support_n = support_share;
  for (std::size_t side = 0; side < 2; ++side) {
    const std::size_t inner = kInnerIndex[side];
    const std::size_t outer = inner + 1;
    const double neutral =
        p.fore_aft_neutral_m + forward_sign[side] * p.fore_aft_neutral_offset_m;
    const double x_reference =
        std::clamp(neutral - forward_sign[side] * leg_offset_command_m_,
                   neutral - p.leg_offset_limit_m, neutral + p.leg_offset_limit_m);
    out.leg_reference_m[side] = Point2{x_reference, height_reference[side]};
    const LegState &leg = legs[side];
    if (!leg.valid) {
      // Cannot evaluate the linkage: damp the joints rather than push blind.
      out.effort_nm[inner] = std::clamp(
          -p.leg_fallback_damping_nm_s_rad * state.velocity[inner],
          -p.leg_torque_limit_nm, p.leg_torque_limit_nm);
      out.effort_nm[outer] = std::clamp(
          -p.leg_fallback_damping_nm_s_rad * state.velocity[outer],
          -p.leg_torque_limit_nm, p.leg_torque_limit_nm);
      continue;
    }
    const auto &j = leg.jacobian;
    const auto map = [&j](const double fx, const double fy) {
      return std::array<double, 2>{j[0][0] * fx + j[1][0] * fy,
                                   j[0][1] * fx + j[1][1] * fy};
    };
    // Gravity support is fed forward so the impedance only has to correct
    // errors, not carry the body; it is never scaled away by saturation.
    std::array<double, 2> support_torque = map(0.0, support_share);
    // Reaction of this wheel's torque on the leg, rolling without slip: the
    // ground pushes the axle forward with tau / r and the stator pushes the
    // wheel link with -tau. Forward in the body is planar +x times
    // forward_sign; forward rolling is a positive planar rotation on a leg
    // whose planar +x is body-forward, negative otherwise. The torque is
    // the low-passed one (see leg_reaction_filter_hz).
    const double tau_slow = balance_active ? filtered_wheel_torque_nm_[side] : 0.0;
    const double tau_planar = forward_sign[side] * tau_slow;
    std::array<double, 2> reaction_torque =
        map(-p.leg_reaction_ground_gain * forward_sign[side] * tau_slow /
                m.wheel_radius_m,
            0.0);
    reaction_torque[0] +=
        p.leg_reaction_stator_gain * tau_planar * leg.wheel_link_jacobian[0];
    reaction_torque[1] +=
        p.leg_reaction_stator_gain * tau_planar * leg.wheel_link_jacobian[1];
    out.leg_reaction_torque_nm[side] = reaction_torque;
    // Impedance on the axle, planar (+x toward the outer pivot, +y down),
    // damping on the filtered Cartesian velocity.
    const Point2 &velocity_filtered = filtered_leg_velocity_m_s_[side];
    const double force_x =
        p.fore_aft_kp_n_m * (x_reference - leg.position.x) -
        p.fore_aft_kd_n_s_m * velocity_filtered.x;
    const double force_y =
        p.height_kp_n_m * (height_reference[side] - leg.position.y) -
        p.height_kd_n_s_m * velocity_filtered.y;
    std::array<double, 2> corrective = map(force_x, force_y);
    corrective[0] += reaction_torque[0];
    corrective[1] += reaction_torque[1];
    // Torque budget: the support keeps its full value unless it alone
    // exceeds the limit; the corrective part (impedance plus reaction
    // compensation) is scaled as a whole so its direction is kept.
    const double support_peak =
        std::max(std::abs(support_torque[0]), std::abs(support_torque[1]));
    if (support_peak > p.leg_torque_limit_nm) {
      const double scale = p.leg_torque_limit_nm / support_peak;
      support_torque[0] *= scale;
      support_torque[1] *= scale;
      corrective = {0.0, 0.0};
      out.status |= WBC_LEG_TORQUE_LIMIT;
    }
    const double scale =
        impedance_scale(support_torque, corrective, p.leg_torque_limit_nm);
    if (scale < 1.0) {
      out.status |= WBC_LEG_TORQUE_LIMIT;
    }
    out.effort_nm[inner] = std::clamp(support_torque[0] + scale * corrective[0],
                                      -p.leg_torque_limit_nm,
                                      p.leg_torque_limit_nm);
    out.effort_nm[outer] = std::clamp(support_torque[1] + scale * corrective[1],
                                      -p.leg_torque_limit_nm,
                                      p.leg_torque_limit_nm);
  }
  return out;
}

} // namespace sandro_controllers
