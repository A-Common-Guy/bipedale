#include "sandro_controllers/control_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sandro_controllers {
namespace {
bool finite(const ControlParameters &p) {
  const double values[] = {p.right_geometry.base,
                           p.right_geometry.upper,
                           p.right_geometry.lower,
                           p.left_geometry.base,
                           p.left_geometry.upper,
                           p.left_geometry.lower,
                           p.leg_height_m,
                           p.leg_height_rate_m_s,
                           p.leg_kp_nm_rad,
                           p.leg_kd_nm_s_rad,
                           p.leg_torque_limit_nm,
                           p.leg_target_rate_rad_s,
                           p.leg_joint_min_rad,
                           p.leg_joint_max_rad,
                           p.pitch_target_rad,
                           p.pitch_kp_nm_rad,
                           p.pitch_kd_nm_s_rad,
                           p.wheel_damping_nm_s_rad,
                           p.wheel_torque_limit_nm,
                           p.balance_torque_limit_nm,
                           p.balance_torque_rate_nm_s,
                           p.balance_torque_sign,
                           p.imu_timeout_s,
                           p.settle_s,
                           p.ramp_s,
                           p.fall_angle_rad,
                           p.wheel_radius_m,
                           p.track_width_m,
                           p.max_forward_velocity_m_s,
                           p.max_yaw_rate_rad_s,
                           p.forward_accel_limit_m_s2,
                           p.velocity_kp_pitch_s_m,
                           p.velocity_ki_pitch_per_m,
                           p.velocity_integral_limit_rad,
                           p.max_pitch_offset_rad,
                           p.velocity_to_pitch_sign,
                           p.position_hold_kp_s,
                           p.position_hold_velocity_limit_m_s,
                           p.command_deadband_m_s,
                           p.yaw_kp_nm_s_rad,
                           p.yaw_torque_limit_nm,
                           p.yaw_torque_rate_nm_s,
                           p.yaw_torque_sign,
                           p.command_timeout_s};
  return std::all_of(std::begin(values), std::end(values),
                     [](const double value) { return std::isfinite(value); });
}

bool positive_geometry(const LegGeometry &g) {
  return g.base > 0.0 && g.upper > 0.0 && g.lower > 0.0;
}
} // namespace

double slew_toward(const double current, const double target, const double rate,
                   const double dt) {
  const double step = rate * dt;
  return current + std::clamp(target - current, -step, step);
}

double pitch_from_quaternion(double w, double x, double y, double z) {
  const double norm = std::sqrt(w * w + x * x + y * y + z * z);
  if (!std::isfinite(norm) || norm < 1e-9) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  w /= norm;
  x /= norm;
  y /= norm;
  z /= norm;
  const double forward_component = 2.0 * (w * y - x * z);
  const double vertical_component = 1.0 - 2.0 * (x * x + y * y);
  return std::atan2(forward_component, vertical_component);
}

bool valid_parameters(const ControlParameters &p) {
  if (!finite(p) || !positive_geometry(p.right_geometry) ||
      !positive_geometry(p.left_geometry)) {
    return false;
  }
  const bool nonnegative =
      p.leg_height_rate_m_s >= 0.0 && p.leg_kp_nm_rad >= 0.0 &&
      p.leg_kd_nm_s_rad >= 0.0 && p.leg_torque_limit_nm > 0.0 &&
      p.leg_target_rate_rad_s >= 0.0 && p.pitch_kp_nm_rad >= 0.0 &&
      p.pitch_kd_nm_s_rad >= 0.0 && p.wheel_torque_limit_nm > 0.0 &&
      p.wheel_damping_nm_s_rad >= 0.0 && p.balance_torque_limit_nm > 0.0 &&
      p.balance_torque_rate_nm_s >= 0.0 && p.imu_timeout_s > 0.0 &&
      p.settle_s >= 0.0 && p.ramp_s >= 0.0 && p.fall_angle_rad > 0.0 &&
      p.wheel_radius_m > 0.0 && p.track_width_m > 0.0 &&
      p.max_forward_velocity_m_s >= 0.0 && p.max_yaw_rate_rad_s >= 0.0 &&
      p.forward_accel_limit_m_s2 >= 0.0 &&
      p.velocity_kp_pitch_s_m >= 0.0 &&
      p.velocity_ki_pitch_per_m >= 0.0 &&
      p.velocity_integral_limit_rad >= 0.0 &&
      p.max_pitch_offset_rad >= 0.0 && p.position_hold_kp_s >= 0.0 &&
      p.position_hold_velocity_limit_m_s >= 0.0 &&
      p.command_deadband_m_s >= 0.0 && p.yaw_kp_nm_s_rad >= 0.0 &&
      p.yaw_torque_limit_nm >= 0.0 && p.yaw_torque_rate_nm_s >= 0.0 &&
      p.command_timeout_s > 0.0;
  const bool signs =
      std::abs(std::abs(p.balance_torque_sign) - 1.0) < 1e-12 &&
      std::abs(std::abs(p.velocity_to_pitch_sign) - 1.0) < 1e-12 &&
      std::abs(std::abs(p.yaw_torque_sign) - 1.0) < 1e-12;
  return nonnegative && signs && p.leg_joint_min_rad < p.leg_joint_max_rad &&
         inverse_kinematics(p.right_geometry.base / 2.0, p.leg_height_m,
                            p.right_geometry)
             .has_value() &&
         inverse_kinematics(p.left_geometry.base / 2.0, p.leg_height_m,
                            p.left_geometry)
             .has_value();
}

void DrivingBalancerCore::reset(const RobotState &state,
                                const ControlParameters &p) {
  double height_sum = 0.0;
  unsigned int count = 0;
  if (const auto point = forward_kinematics(
          state.position[0], state.position[1], p.right_geometry)) {
    height_sum += point->y;
    ++count;
  }
  if (const auto point = forward_kinematics(
          state.position[3], state.position[4], p.left_geometry)) {
    height_sum += point->y;
    ++count;
  }
  leg_height_m_ =
      count ? height_sum / static_cast<double>(count) : p.leg_height_m;
  leg_setpoint_ = {state.position[0], state.position[1], state.position[3],
                   state.position[4]};
  elapsed_s_ = 0.0;
  last_balance_torque_nm_ = 0.0;
  velocity_command_m_s_ = 0.0;
  position_reference_m_ =
      0.5 * (state.position[2] + state.position[5]) * p.wheel_radius_m;
  velocity_integral_m_ = 0.0;
  yaw_torque_nm_ = 0.0;
  fallen_ = false;
}

ControlOutput DrivingBalancerCore::update(const RobotState &state,
                                          const MotionCommand &command,
                                          const ControlParameters &p,
                                          const double period_s) {
  ControlOutput out;
  if (!std::isfinite(period_s) || period_s <= 0.0 || !valid_parameters(p)) {
    out.saturation_mask = SAT_IMU_STALE;
    return out;
  }

  out.requested_forward_velocity_m_s = command.forward_velocity_m_s;
  out.requested_yaw_rate_rad_s = command.yaw_rate_rad_s;
  double requested_forward = command.forward_velocity_m_s;
  double requested_yaw = command.yaw_rate_rad_s;
  if (!std::isfinite(requested_forward) || !std::isfinite(requested_yaw) ||
      !std::isfinite(command.age_s) || command.age_s > p.command_timeout_s) {
    requested_forward = 0.0;
    requested_yaw = 0.0;
    out.saturation_mask |= SAT_COMMAND_STALE;
  }
  const double limited_forward =
      std::clamp(requested_forward, -p.max_forward_velocity_m_s,
                 p.max_forward_velocity_m_s);
  const double limited_yaw =
      std::clamp(requested_yaw, -p.max_yaw_rate_rad_s, p.max_yaw_rate_rad_s);
  if (limited_forward != requested_forward) {
    out.saturation_mask |= SAT_FORWARD_LIMIT;
  }
  if (limited_yaw != requested_yaw) {
    out.saturation_mask |= SAT_YAW_RATE_LIMIT;
  }

  const double position =
      0.5 * (state.position[2] + state.position[5]) * p.wheel_radius_m;
  // Wheel encoders turn relative to the chassis, so ground speed needs the body
  // pitch rate added back. Without it, pitch motion aliases into the velocity
  // loop and subtracts pitch_kp * velocity_kp * wheel_radius from the inner
  // loop's pitch damping.
  const double chassis_pitch_rate = std::isfinite(state.pitch_rate_rad_s)
                                        ? state.pitch_rate_rad_s
                                        : 0.0;
  out.measured_forward_velocity_m_s =
      (0.5 * (state.velocity[2] + state.velocity[5]) + chassis_pitch_rate) *
      p.wheel_radius_m;
  out.measured_yaw_rate_rad_s = (state.velocity[2] - state.velocity[5]) *
                                p.wheel_radius_m / p.track_width_m;
  velocity_command_m_s_ = slew_toward(velocity_command_m_s_, limited_forward,
                               p.forward_accel_limit_m_s2, period_s);
  out.effective_forward_velocity_m_s = velocity_command_m_s_;
  out.effective_yaw_rate_rad_s = limited_yaw;

  double velocity_target = 0.0;
  if (std::abs(limited_forward) > p.command_deadband_m_s ||
      std::abs(velocity_command_m_s_) > p.command_deadband_m_s) {
    position_reference_m_ = position;
    velocity_target = velocity_command_m_s_;
  } else {
    velocity_target =
        std::clamp(p.position_hold_kp_s * (position_reference_m_ - position),
                   -p.position_hold_velocity_limit_m_s,
                   p.position_hold_velocity_limit_m_s);
  }

  const double pitch =
      pitch_from_quaternion(state.quaternion_w, state.quaternion_x,
                            state.quaternion_y, state.quaternion_z);
  out.pitch_rad = pitch;
  const bool imu_valid =
      std::isfinite(pitch) && std::isfinite(state.pitch_rate_rad_s) &&
      std::isfinite(state.imu_age_s) && state.imu_age_s >= 0.0 &&
      state.imu_age_s <= p.imu_timeout_s;
  if (imu_valid && std::abs(pitch) > p.fall_angle_rad) {
    fallen_ = true;
  }
  out.fallen = fallen_;

  const double velocity_error =
      velocity_target - out.measured_forward_velocity_m_s;
  const double integral_bound =
      p.velocity_ki_pitch_per_m > 0.0
          ? p.velocity_integral_limit_rad / p.velocity_ki_pitch_per_m
          : 0.0;
  if (imu_valid && !fallen_) {
    velocity_integral_m_ =
        std::clamp(velocity_integral_m_ + velocity_error * period_s,
                   -integral_bound, integral_bound);
  }
  const double unclamped_offset =
      p.velocity_kp_pitch_s_m * velocity_error +
      p.velocity_ki_pitch_per_m * velocity_integral_m_;
  const double pitch_offset = std::clamp(
      unclamped_offset, -p.max_pitch_offset_rad, p.max_pitch_offset_rad);
  if (pitch_offset != unclamped_offset) {
    out.saturation_mask |= SAT_PITCH_OFFSET_LIMIT;
    // Back-calculate so the integral never holds authority the output clamp is
    // already discarding.
    if (imu_valid && !fallen_ && p.velocity_ki_pitch_per_m > 0.0) {
      velocity_integral_m_ =
          std::clamp(velocity_integral_m_ + (pitch_offset - unclamped_offset) /
                                                p.velocity_ki_pitch_per_m,
                     -integral_bound, integral_bound);
    }
  }
  out.pitch_target_rad =
      p.pitch_target_rad + p.velocity_to_pitch_sign * pitch_offset;

  if (!imu_valid || fallen_) {
    last_balance_torque_nm_ = 0.0;
    yaw_torque_nm_ = 0.0;
    out.saturation_mask |= imu_valid ? SAT_FALL_LATCH : SAT_IMU_STALE;
  } else {
    elapsed_s_ += period_s;
    out.ramp = p.ramp_s == 0.0
                   ? (elapsed_s_ >= p.settle_s ? 1.0 : 0.0)
                   : std::clamp((elapsed_s_ - p.settle_s) / p.ramp_s, 0.0, 1.0);
    if (out.ramp < 1.0) {
      out.saturation_mask |= SAT_SETTLING;
    }
    out.raw_balance_torque_nm =
        p.balance_torque_sign *
        (p.pitch_kp_nm_rad * (pitch - out.pitch_target_rad) +
         p.pitch_kd_nm_s_rad * state.pitch_rate_rad_s);
    const double balance_limit =
        std::min(p.balance_torque_limit_nm, p.wheel_torque_limit_nm);
    const double clipped_balance =
        std::clamp(out.raw_balance_torque_nm, -balance_limit, balance_limit);
    if (clipped_balance != out.raw_balance_torque_nm) {
      out.saturation_mask |= SAT_BALANCE_TORQUE_LIMIT;
    }
    last_balance_torque_nm_ =
        slew_toward(last_balance_torque_nm_, out.ramp * clipped_balance,
             p.balance_torque_rate_nm_s, period_s);

    const double raw_yaw = p.yaw_torque_sign * p.yaw_kp_nm_s_rad *
                           (limited_yaw - out.measured_yaw_rate_rad_s);
    const double yaw_goal =
        out.ramp *
        std::clamp(raw_yaw, -p.yaw_torque_limit_nm, p.yaw_torque_limit_nm);
    if (std::abs(raw_yaw) > p.yaw_torque_limit_nm) {
      out.saturation_mask |= SAT_YAW_TORQUE_LIMIT;
    }
    yaw_torque_nm_ =
        slew_toward(yaw_torque_nm_, yaw_goal, p.yaw_torque_rate_nm_s, period_s);
  }

  out.balance_torque_nm = last_balance_torque_nm_;
  const double available = std::max(0.0, p.wheel_torque_limit_nm -
                                             std::abs(last_balance_torque_nm_));
  const double differential = std::clamp(yaw_torque_nm_, -available, available);
  if (differential != yaw_torque_nm_) {
    out.saturation_mask |= SAT_TOTAL_WHEEL_TORQUE;
  }
  out.yaw_torque_nm = differential;
  // Reference the damping to the commanded *yaw* only. A differential wheel
  // command is free: the two reaction torques cancel at the chassis, so it
  // disturbs pitch not at all, and referencing it here stops the damping
  // fighting our own turn. The common-mode term deliberately stays referenced
  // to zero. Driving a wheeled inverted pendulum forward requires first
  // rolling *backward* to tip the body into the direction of travel, so a
  // forward feedforward pushes against the plant's own dynamics and pitches
  // the chassis away from the lean the outer loop is asking for. Forward
  // authority belongs to that lean, not to the wheels directly.
  const double reference_yaw_rate =
      limited_yaw * p.track_width_m / (2.0 * p.wheel_radius_m);
  out.reference_right_wheel_rad_s = reference_yaw_rate;
  out.reference_left_wheel_rad_s = -reference_yaw_rate;
  const double raw_right_effort =
      imu_valid && !fallen_
          ? last_balance_torque_nm_ + differential -
                p.wheel_damping_nm_s_rad *
                    (state.velocity[2] - out.reference_right_wheel_rad_s)
          : 0.0;
  const double raw_left_effort =
      imu_valid && !fallen_
          ? last_balance_torque_nm_ - differential -
                p.wheel_damping_nm_s_rad *
                    (state.velocity[5] - out.reference_left_wheel_rad_s)
          : 0.0;
  out.effort_nm[2] = std::clamp(raw_right_effort, -p.wheel_torque_limit_nm,
                                p.wheel_torque_limit_nm);
  out.effort_nm[5] = std::clamp(raw_left_effort, -p.wheel_torque_limit_nm,
                                p.wheel_torque_limit_nm);
  if (out.effort_nm[2] != raw_right_effort ||
      out.effort_nm[5] != raw_left_effort) {
    out.saturation_mask |= SAT_TOTAL_WHEEL_TORQUE;
  }

  apply_leg_impedance(state, p, period_s, leg_height_m_, leg_setpoint_, out);
  return out;
}

void apply_leg_impedance(const RobotState &state, const ControlParameters &p,
                         const double period_s, double &leg_height_m,
                         std::array<double, 4> &leg_setpoint,
                         ControlOutput &out) {
  leg_height_m = slew_toward(leg_height_m, p.leg_height_m,
                             p.leg_height_rate_m_s, period_s);
  const std::array<LegGeometry, 2> geometries{p.right_geometry,
                                              p.left_geometry};
  const std::array<std::size_t, 4> state_indices{0, 1, 3, 4};
  for (std::size_t side = 0; side < 2; ++side) {
    const auto target = inverse_kinematics(geometries[side].base / 2.0,
                                           leg_height_m, geometries[side]);
    if (!target) {
      continue;
    }
    for (std::size_t joint = 0; joint < 2; ++joint) {
      const std::size_t local = side * 2 + joint;
      const std::size_t index = state_indices[local];
      const double goal = std::clamp((*target)[joint], p.leg_joint_min_rad,
                                     p.leg_joint_max_rad);
      leg_setpoint[local] = slew_toward(leg_setpoint[local], goal,
                                        p.leg_target_rate_rad_s, period_s);
      const double raw_effort =
          p.leg_kp_nm_rad * (leg_setpoint[local] - state.position[index]) -
          p.leg_kd_nm_s_rad * state.velocity[index];
      out.effort_nm[index] =
          std::clamp(raw_effort, -p.leg_torque_limit_nm, p.leg_torque_limit_nm);
      if (out.effort_nm[index] != raw_effort) {
        out.saturation_mask |= SAT_LEG_TORQUE;
      }
    }
  }
}

} // namespace sandro_controllers
