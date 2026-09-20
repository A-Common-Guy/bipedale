#pragma once

#include <array>
#include <cstdint>

#include "sandro_controllers/kinematics.hpp"

namespace sandro_controllers {

struct ControlParameters {
  LegGeometry right_geometry{};
  LegGeometry left_geometry{};
  double leg_height_m{0.17};
  double leg_height_rate_m_s{0.03};
  double leg_kp_nm_rad{12.0};
  double leg_kd_nm_s_rad{0.5};
  double leg_torque_limit_nm{3.0};
  double leg_target_rate_rad_s{6.0};
  double leg_joint_min_rad{-2.0943951024};
  double leg_joint_max_rad{2.0943951024};
  double pitch_target_rad{0.0};
  double pitch_kp_nm_rad{15.0};
  double pitch_kd_nm_s_rad{0.65};
  double wheel_damping_nm_s_rad{0.1};
  double wheel_torque_limit_nm{6.0};
  double balance_torque_limit_nm{6.0};
  double balance_torque_rate_nm_s{100.0};
  double balance_torque_sign{1.0};
  double imu_timeout_s{0.15};
  double settle_s{1.0};
  double ramp_s{0.5};
  double fall_angle_rad{1.0};
  double wheel_radius_m{0.0742};
  double track_width_m{0.304};
  double max_forward_velocity_m_s{0.5};
  double max_yaw_rate_rad_s{1.5};
  double forward_accel_limit_m_s2{1.0};
  double velocity_kp_pitch_s_m{0.12};
  double velocity_ki_pitch_per_m{0.02};
  // Budget for the integral's own pitch contribution, so it stays a real
  // trim authority when the gain changes.
  double velocity_integral_limit_rad{0.05};
  double max_pitch_offset_rad{0.15};
  double velocity_to_pitch_sign{1.0};
  double position_hold_kp_s{1.0};
  double position_hold_velocity_limit_m_s{0.15};
  double command_deadband_m_s{0.01};
  double yaw_kp_nm_s_rad{0.15};
  double yaw_torque_limit_nm{0.4};
  double yaw_torque_rate_nm_s{3.0};
  double yaw_torque_sign{1.0};
  double command_timeout_s{0.3};
};

struct RobotState {
  // right inner, right outer, right wheel, left inner, left outer, left wheel
  std::array<double, 6> position{};
  std::array<double, 6> velocity{};
  double quaternion_w{1.0};
  double quaternion_x{};
  double quaternion_y{};
  double quaternion_z{};
  double pitch_rate_rad_s{};
  double imu_age_s{};
};

struct MotionCommand {
  double forward_velocity_m_s{};
  double yaw_rate_rad_s{};
  double age_s{};
};

enum Saturation : std::uint32_t {
  SAT_NONE = 0,
  SAT_COMMAND_STALE = 1U << 0,
  SAT_FORWARD_LIMIT = 1U << 1,
  SAT_YAW_RATE_LIMIT = 1U << 2,
  SAT_PITCH_OFFSET_LIMIT = 1U << 3,
  SAT_BALANCE_TORQUE_LIMIT = 1U << 4,
  SAT_YAW_TORQUE_LIMIT = 1U << 5,
  SAT_TOTAL_WHEEL_TORQUE = 1U << 6,
  SAT_LEG_TORQUE = 1U << 7,
  SAT_IMU_STALE = 1U << 8,
  SAT_FALL_LATCH = 1U << 9,
  SAT_SETTLING = 1U << 10
};

struct ControlOutput {
  std::array<double, 6> effort_nm{};
  double requested_forward_velocity_m_s{};
  double requested_yaw_rate_rad_s{};
  double effective_forward_velocity_m_s{};
  double effective_yaw_rate_rad_s{};
  double measured_forward_velocity_m_s{};
  double measured_yaw_rate_rad_s{};
  double pitch_rad{};
  double pitch_target_rad{};
  double raw_balance_torque_nm{};
  double balance_torque_nm{};
  double yaw_torque_nm{};
  double reference_right_wheel_rad_s{};
  double reference_left_wheel_rad_s{};
  // Populated by the LQR balancer; the cascade leaves these at zero.
  double position_error_m{};
  double velocity_error_m_s{};
  double velocity_integral_nm{};
  double ramp{};
  std::uint32_t saturation_mask{SAT_NONE};
  // True once the LQR balancer has seen the chassis upright and taken over the
  // wheels. The cascade engages unconditionally and leaves this false.
  bool engaged{};
  bool fallen{};
};

double pitch_from_quaternion(double w, double x, double y, double z);
bool valid_parameters(const ControlParameters &parameters);

// Move current toward target at no more than rate per second.
double slew_toward(double current, double target, double rate, double dt);

// Hold both five-bar legs at the configured height. Shared by the cascade and
// the LQR balancer so leg impedance, joint limits and torque clamping stay
// identical between them.
void apply_leg_impedance(const RobotState &state,
                         const ControlParameters &parameters, double period_s,
                         double &leg_height_m,
                         std::array<double, 4> &leg_setpoint,
                         ControlOutput &out);

class DrivingBalancerCore {
public:
  void reset(const RobotState &state, const ControlParameters &parameters);
  ControlOutput update(const RobotState &state, const MotionCommand &command,
                       const ControlParameters &parameters, double period_s);

private:
  double leg_height_m_{0.17};
  std::array<double, 4> leg_setpoint_{};
  double elapsed_s_{};
  double last_balance_torque_nm_{};
  double velocity_command_m_s_{};
  double position_reference_m_{};
  double velocity_integral_m_{};
  double yaw_torque_nm_{};
  bool fallen_{};
};

} // namespace sandro_controllers
