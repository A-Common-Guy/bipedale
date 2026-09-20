#include "sandro_controllers/lqr_core.hpp"

#include <algorithm>
#include <cmath>

namespace sandro_controllers {

bool valid_lqr_parameters(const LqrParameters &g) {
  const double values[] = {g.position_nm_m,
                           g.velocity_nm_s_m,
                           g.pitch_nm_rad,
                           g.pitch_rate_nm_s_rad,
                           g.velocity_integral_nm_s_m,
                           g.velocity_integral_limit_nm,
                           g.engage_angle_rad};
  if (!std::all_of(std::begin(values), std::end(values),
                   [](const double value) { return std::isfinite(value); })) {
    return false;
  }
  // The pitch gains must stabilise the pendulum, so they carry a fixed sign.
  // The position and velocity gains are left free: their sign depends on the
  // Riccati solution and is not obvious by inspection.
  return g.pitch_nm_rad > 0.0 && g.pitch_rate_nm_s_rad > 0.0 &&
         g.velocity_integral_nm_s_m >= 0.0 &&
         g.velocity_integral_limit_nm >= 0.0 && g.engage_angle_rad > 0.0;
}

void LqrBalancerCore::reset(const RobotState &state,
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
  balance_torque_nm_ = 0.0;
  velocity_command_m_s_ = 0.0;
  position_reference_m_ =
      0.5 * (state.position[2] + state.position[5]) * p.wheel_radius_m;
  velocity_integral_nm_ = 0.0;
  yaw_torque_nm_ = 0.0;
  engaged_ = false;
  fallen_ = false;
}

ControlOutput LqrBalancerCore::update(const RobotState &state,
                                      const MotionCommand &command,
                                      const ControlParameters &p,
                                      const LqrParameters &g,
                                      const double period_s) {
  ControlOutput out;
  if (!std::isfinite(period_s) || period_s <= 0.0 || !valid_parameters(p) ||
      !valid_lqr_parameters(g)) {
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
  // pitch rate added back, exactly as in the cascade.
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

  // While driving, the position state tracks the robot so only the velocity
  // error acts. Released, it pins where the robot should stand and the
  // position gain holds station.
  if (std::abs(limited_forward) > p.command_deadband_m_s ||
      std::abs(velocity_command_m_s_) > p.command_deadband_m_s) {
    position_reference_m_ = position;
  }

  const double pitch =
      pitch_from_quaternion(state.quaternion_w, state.quaternion_x,
                            state.quaternion_y, state.quaternion_z);
  out.pitch_rad = pitch;
  out.pitch_target_rad = p.pitch_target_rad;
  const bool imu_valid =
      std::isfinite(pitch) && std::isfinite(state.pitch_rate_rad_s) &&
      std::isfinite(state.imu_age_s) && state.imu_age_s >= 0.0 &&
      state.imu_age_s <= p.imu_timeout_s;
  // Wait for upright before taking the wheels. On startup the legs are
  // unpowered until this controller activates, so the robot is usually already
  // down by then; driving immediately would spin the wheels out from under it
  // and push it past the fall angle instead of catching it. Waiting also gives
  // the natural hardware bringup: lift the robot upright and it engages.
  if (!engaged_ && imu_valid && !fallen_ &&
      std::abs(pitch) <= g.engage_angle_rad) {
    engaged_ = true;
    elapsed_s_ = 0.0;
    position_reference_m_ = position;
    velocity_integral_nm_ = 0.0;
  }
  // The fall latch arms only after engaging, so the collapse that happens
  // before takeover is not mistaken for a fall the robot cannot recover from.
  if (engaged_ && imu_valid && std::abs(pitch) > p.fall_angle_rad) {
    fallen_ = true;
  }
  out.engaged = engaged_;
  out.fallen = fallen_;

  out.position_error_m = position - position_reference_m_;
  out.velocity_error_m_s =
      out.measured_forward_velocity_m_s - velocity_command_m_s_;

  if (!imu_valid || fallen_ || !engaged_) {
    balance_torque_nm_ = 0.0;
    yaw_torque_nm_ = 0.0;
    velocity_integral_nm_ = 0.0;
    if (!imu_valid) {
      out.saturation_mask |= SAT_IMU_STALE;
    } else {
      out.saturation_mask |= fallen_ ? SAT_FALL_LATCH : SAT_SETTLING;
    }
  } else {
    elapsed_s_ += period_s;
    out.ramp = p.ramp_s == 0.0
                   ? (elapsed_s_ >= p.settle_s ? 1.0 : 0.0)
                   : std::clamp((elapsed_s_ - p.settle_s) / p.ramp_s, 0.0, 1.0);
    if (out.ramp < 1.0) {
      out.saturation_mask |= SAT_SETTLING;
    }

    velocity_integral_nm_ = std::clamp(
        velocity_integral_nm_ +
            g.velocity_integral_nm_s_m * out.velocity_error_m_s * period_s,
        -g.velocity_integral_limit_nm, g.velocity_integral_limit_nm);

    // One coordinated gain vector over the whole state, rather than a pitch
    // loop wrapped in a velocity loop.
    const double total_torque =
        g.pitch_nm_rad * (pitch - out.pitch_target_rad) +
        g.pitch_rate_nm_s_rad * state.pitch_rate_rad_s +
        g.position_nm_m * out.position_error_m +
        g.velocity_nm_s_m * out.velocity_error_m_s + velocity_integral_nm_;
    out.raw_balance_torque_nm = p.balance_torque_sign * 0.5 * total_torque;

    const double balance_limit =
        std::min(p.balance_torque_limit_nm, p.wheel_torque_limit_nm);
    const double clipped =
        std::clamp(out.raw_balance_torque_nm, -balance_limit, balance_limit);
    if (clipped != out.raw_balance_torque_nm) {
      out.saturation_mask |= SAT_BALANCE_TORQUE_LIMIT;
      // Do not let the trim hold authority the clamp is already discarding.
      velocity_integral_nm_ = std::clamp(
          velocity_integral_nm_ +
              2.0 * p.balance_torque_sign *
                  (clipped - out.raw_balance_torque_nm),
          -g.velocity_integral_limit_nm, g.velocity_integral_limit_nm);
    }
    balance_torque_nm_ = slew_toward(balance_torque_nm_, out.ramp * clipped,
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

  out.balance_torque_nm = balance_torque_nm_;
  out.velocity_integral_nm = velocity_integral_nm_;
  const double available =
      std::max(0.0, p.wheel_torque_limit_nm - std::abs(balance_torque_nm_));
  const double differential = std::clamp(yaw_torque_nm_, -available, available);
  if (differential != yaw_torque_nm_) {
    out.saturation_mask |= SAT_TOTAL_WHEEL_TORQUE;
  }
  out.yaw_torque_nm = differential;

  // No wheel damping term here. The velocity gain already is the wheel
  // velocity feedback, and it is referenced to the command, so it stabilises
  // without dragging against our own motion.
  const double reference_yaw_rate =
      limited_yaw * p.track_width_m / (2.0 * p.wheel_radius_m);
  out.reference_right_wheel_rad_s = reference_yaw_rate;
  out.reference_left_wheel_rad_s = -reference_yaw_rate;

  const bool driving = imu_valid && engaged_ && !fallen_;
  const double raw_right = driving ? balance_torque_nm_ + differential : 0.0;
  const double raw_left = driving ? balance_torque_nm_ - differential : 0.0;
  out.effort_nm[2] =
      std::clamp(raw_right, -p.wheel_torque_limit_nm, p.wheel_torque_limit_nm);
  out.effort_nm[5] =
      std::clamp(raw_left, -p.wheel_torque_limit_nm, p.wheel_torque_limit_nm);
  if (out.effort_nm[2] != raw_right || out.effort_nm[5] != raw_left) {
    out.saturation_mask |= SAT_TOTAL_WHEEL_TORQUE;
  }

  apply_leg_impedance(state, p, period_s, leg_height_m_, leg_setpoint_, out);
  return out;
}

} // namespace sandro_controllers
