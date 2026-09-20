#include "sandro_controllers/whole_body_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace sandro_controllers {
namespace {

using Parameter = rclcpp::Parameter;

rcl_interfaces::msg::ParameterDescriptor
descriptor(const std::string &text, const bool read_only = false) {
  rcl_interfaces::msg::ParameterDescriptor result;
  result.description = text;
  result.read_only = read_only;
  return result;
}

struct DoubleParameter {
  const char *name;
  double &(*field)(WholeBodyParameters &);
  const char *description;
};

#define SANDRO_WBC_FIELD(member)                                               \
  [](WholeBodyParameters &p) -> double & { return p.member; }

// One table drives declaration, reading and live updates so a parameter can
// never be declared without being read or vice versa.
const std::vector<DoubleParameter> &double_parameters() {
  static const std::vector<DoubleParameter> table = {
      {"model.right.base_m", SANDRO_WBC_FIELD(model.right_geometry.base),
       "Right five-bar pivot spacing in metres."},
      {"model.right.upper_m", SANDRO_WBC_FIELD(model.right_geometry.upper),
       "Right five-bar upper-link length in metres."},
      {"model.right.lower_m", SANDRO_WBC_FIELD(model.right_geometry.lower),
       "Right five-bar lower-link length in metres."},
      {"model.right.forward_sign", SANDRO_WBC_FIELD(model.right_forward_sign),
       "Body-x direction of right planar leg +x; exactly +1 or -1."},
      {"model.right.wheel_on_outer_link",
       SANDRO_WBC_FIELD(model.right_wheel_on_outer_link),
       "1 if the right wheel motor's stator sits on the outer lower link, 0 for the inner."},
      {"model.left.base_m", SANDRO_WBC_FIELD(model.left_geometry.base),
       "Left five-bar pivot spacing in metres."},
      {"model.left.upper_m", SANDRO_WBC_FIELD(model.left_geometry.upper),
       "Left five-bar upper-link length in metres."},
      {"model.left.lower_m", SANDRO_WBC_FIELD(model.left_geometry.lower),
       "Left five-bar lower-link length in metres."},
      {"model.left.forward_sign", SANDRO_WBC_FIELD(model.left_forward_sign),
       "Body-x direction of left planar leg +x; exactly +1 or -1."},
      {"model.left.wheel_on_outer_link",
       SANDRO_WBC_FIELD(model.left_wheel_on_outer_link),
       "1 if the left wheel motor's stator sits on the outer lower link, 0 for the inner."},
      {"model.body_mass_kg", SANDRO_WBC_FIELD(model.body_mass_kg),
       "Mass carried by the legs (total minus wheels) in kg."},
      {"model.wheel_mass_kg", SANDRO_WBC_FIELD(model.wheel_mass_kg),
       "Mass of one wheel in kg."},
      {"model.wheel_inertia_kgm2", SANDRO_WBC_FIELD(model.wheel_inertia_kgm2),
       "Inertia of one wheel about the axle incl. reflected rotor, kg m^2."},
      {"model.wheel_radius_m", SANDRO_WBC_FIELD(model.wheel_radius_m),
       "Effective rolling radius in metres."},
      {"model.track_width_m", SANDRO_WBC_FIELD(model.track_width_m),
       "Tyre contact spacing in metres."},
      {"model.yaw_inertia_kgm2", SANDRO_WBC_FIELD(model.yaw_inertia_kgm2),
       "Yaw inertia of the robot about the vertical through its CoM, kg m^2."},
      {"model.leg_lag_s", SANDRO_WBC_FIELD(model.leg_lag_s),
       "Time constant with which the measured leg offset follows its command, s."},
      {"model.com_height_per_leg_height",
       SANDRO_WBC_FIELD(model.com_height_per_leg_height),
       "Centre-of-mass height above the axle per metre of leg height."},
      {"model.com_height_offset_m", SANDRO_WBC_FIELD(model.com_height_offset_m),
       "Centre-of-mass height offset in metres."},
      {"model.pitch_inertia_kgm2", SANDRO_WBC_FIELD(model.pitch_inertia_kgm2),
       "Body pitch inertia about its CoM at the reference leg height."},
      {"model.pitch_inertia_reference_height_m",
       SANDRO_WBC_FIELD(model.pitch_inertia_reference_height_m),
       "Leg height at which pitch_inertia_kgm2 was identified."},
      {"model.pitch_inertia_per_m", SANDRO_WBC_FIELD(model.pitch_inertia_per_m),
       "Change of body pitch inertia per metre of leg height."},
      {"model.gravity_m_s2", SANDRO_WBC_FIELD(model.gravity_m_s2),
       "Gravitational acceleration."},
      {"balance.weights.position", SANDRO_WBC_FIELD(state_weights[0]),
       "LQR weight on ground position error (1/m^2)."},
      {"balance.weights.velocity", SANDRO_WBC_FIELD(state_weights[1]),
       "LQR weight on forward velocity error (s^2/m^2)."},
      {"balance.weights.pitch", SANDRO_WBC_FIELD(state_weights[2]),
       "LQR weight on pitch error (1/rad^2)."},
      {"balance.weights.pitch_rate", SANDRO_WBC_FIELD(state_weights[3]),
       "LQR weight on pitch rate (s^2/rad^2)."},
      {"balance.weights.leg_offset_command", SANDRO_WBC_FIELD(state_weights[4]),
       "LQR weight on the commanded fore-aft leg offset from its reference (1/m^2)."},
      {"balance.weights.leg_offset", SANDRO_WBC_FIELD(state_weights[5]),
       "LQR weight on the achieved fore-aft leg offset from its reference (1/m^2)."},
      {"balance.effort_weights.wheel_torque",
       SANDRO_WBC_FIELD(effort_weights[0]),
       "LQR weight on total wheel torque (1/Nm^2)."},
      {"balance.effort_weights.leg_offset_rate",
       SANDRO_WBC_FIELD(effort_weights[1]),
       "LQR weight on fore-aft leg offset rate (s^2/m^2)."},
      {"balance.gain_table.min_height_m",
       SANDRO_WBC_FIELD(gain_table_min_height_m),
       "Lowest leg height with solved gains."},
      {"balance.gain_table.max_height_m",
       SANDRO_WBC_FIELD(gain_table_max_height_m),
       "Highest leg height with solved gains."},
      {"balance.required_delay_margin_s",
       SANDRO_WBC_FIELD(required_delay_margin_s),
       "Smallest input-delay margin the balance loop must keep at every table height, in seconds."},
      {"balance.measured_offset_blend", SANDRO_WBC_FIELD(measured_offset_blend),
       "Leg offset the LQR feeds back: 0 the commanded, 1 the measured axle position."},
      {"balance.leg_offset_limit_m", SANDRO_WBC_FIELD(leg_offset_limit_m),
       "Fore-aft wheel placement the legs may use for balance, in metres."},
      {"balance.leg_offset_rate_limit_m_s",
       SANDRO_WBC_FIELD(leg_offset_rate_limit_m_s),
       "Fore-aft leg offset rate limit in m/s."},
      {"balance.position_error_limit_m",
       SANDRO_WBC_FIELD(position_error_limit_m),
       "Anti-windup bound on the position reference in metres."},
      {"balance.wheel_friction_ff_nm", SANDRO_WBC_FIELD(wheel_friction_ff_nm),
       "Coulomb friction compensated per wheel in Nm, in the direction of the commanded torque."},
      {"balance.wheel_friction_ff_smoothing_nm",
       SANDRO_WBC_FIELD(wheel_friction_ff_smoothing_nm),
       "Commanded torque over which the friction feed-forward ramps in (tanh)."},
      {"balance.trim_rate_per_s", SANDRO_WBC_FIELD(trim_rate_per_s),
       "Fraction per second of the standing CoM error moved into the legs."},
      {"balance.trim_limit_m", SANDRO_WBC_FIELD(trim_limit_m),
       "Maximum posture trim in metres; excess becomes a pitch reference."},
      {"balance.trim_velocity_deadband_m_s",
       SANDRO_WBC_FIELD(trim_velocity_deadband_m_s),
       "Drive command and measured speed below which the posture trim adapts."},
      {"balance.trim_torque_deadband_nm", SANDRO_WBC_FIELD(trim_torque_deadband_nm),
       "Balance torque above which the trim does not adapt (a lean held against stiction)."},
      {"balance.wheel_torque_sign", SANDRO_WBC_FIELD(wheel_torque_sign),
       "Wheel effort polarity; exactly +1 or -1."},
      {"balance.wheel_torque_rate_nm_s",
       SANDRO_WBC_FIELD(wheel_torque_rate_nm_s),
       "Per-wheel balance torque slew in Nm/s."},
      {"balance.velocity_filter_hz", SANDRO_WBC_FIELD(velocity_filter_hz),
       "Low-pass cutoff on the encoder-derived forward velocity in Hz."},
      {"balance.engage_pitch_rad", SANDRO_WBC_FIELD(engage_pitch_rad),
       "Pitch error below which balancing engages after settling."},
      {"balance.engage_pitch_rate_rad_s",
       SANDRO_WBC_FIELD(engage_pitch_rate_rad_s),
       "Pitch rate below which balancing engages after settling."},
      {"tasks.height.target_m", SANDRO_WBC_FIELD(height_target_m),
       "Nominal planar leg height (pivot to axle) in metres."},
      {"tasks.height.min_m", SANDRO_WBC_FIELD(height_min_m),
       "Lowest commandable leg height in metres."},
      {"tasks.height.max_m", SANDRO_WBC_FIELD(height_max_m),
       "Highest commandable leg height in metres."},
      {"tasks.height.rate_m_s", SANDRO_WBC_FIELD(height_rate_m_s),
       "Height reference slew in m/s."},
      {"tasks.height.kp_n_m", SANDRO_WBC_FIELD(height_kp_n_m),
       "Vertical leg stiffness in N/m."},
      {"tasks.height.kd_n_s_m", SANDRO_WBC_FIELD(height_kd_n_s_m),
       "Vertical leg damping in N s/m."},
      {"tasks.height.ki_n_m_s", SANDRO_WBC_FIELD(height_ki_n_m_s),
       "Vertical support integral gain in N/(m s), learns the weight share."},
      {"tasks.height.integral_limit_n", SANDRO_WBC_FIELD(height_integral_limit_n),
       "Bound on the learnt support correction per leg in N."},
      {"tasks.fore_aft.neutral_m", SANDRO_WBC_FIELD(fore_aft_neutral_m),
       "Planar x of the axle with the wheels centred under the pivots."},
      {"tasks.fore_aft.neutral_offset_m",
       SANDRO_WBC_FIELD(fore_aft_neutral_offset_m),
       "Body-frame shift of the neutral axle position, positive forward; set to "
       "minus the converged trim_m so the legs keep their full range."},
      {"tasks.fore_aft.kp_n_m", SANDRO_WBC_FIELD(fore_aft_kp_n_m),
       "Fore-aft leg stiffness in N/m."},
      {"tasks.fore_aft.kd_n_s_m", SANDRO_WBC_FIELD(fore_aft_kd_n_s_m),
       "Fore-aft leg damping in N s/m."},
      {"tasks.leg_velocity_filter_hz", SANDRO_WBC_FIELD(leg_velocity_filter_hz),
       "Low-pass cutoff on the Cartesian leg velocities used for damping, Hz."},
      {"tasks.legs.pd_in_drive", SANDRO_WBC_FIELD(leg_pd_in_drive),
       "1: the leg PD runs in the drives' MIT loop (joint references + drive_kp/kd, feed-forward torque only); 0: in the controller."},
      {"tasks.legs.drive_kp_nm_rad", SANDRO_WBC_FIELD(leg_drive_kp_nm_rad),
       "Joint stiffness the drives apply to the leg references, Nm/rad (MIT kp)."},
      {"tasks.legs.drive_kd_nm_s_rad", SANDRO_WBC_FIELD(leg_drive_kd_nm_s_rad),
       "Joint damping the drives apply to the leg references, Nm s/rad (MIT kd)."},
      {"tasks.leg_reaction_ground_gain",
       SANDRO_WBC_FIELD(leg_reaction_ground_gain),
       "Fraction of the ground force tau / r at the axle fed forward to the leg motors (0..1)."},
      {"tasks.leg_reaction_stator_gain",
       SANDRO_WBC_FIELD(leg_reaction_stator_gain),
       "Fraction of the wheel motor's stator moment on the wheel link fed forward (0..1)."},
      {"tasks.leg_reaction_filter_hz", SANDRO_WBC_FIELD(leg_reaction_filter_hz),
       "Low-pass cutoff on the wheel torque the leg reaction feed-forward follows, Hz."},
      {"tasks.roll.lean_gain", SANDRO_WBC_FIELD(roll_lean_gain),
       "Fraction of the ideal lean-into-turn roll to command."},
      {"tasks.roll.kp_m_rad", SANDRO_WBC_FIELD(roll_kp_m_rad),
       "Height differential per radian of roll error."},
      {"tasks.roll.ki_m_rad_s", SANDRO_WBC_FIELD(roll_ki_m_rad_s),
       "Height differential integral gain in m/(rad s)."},
      {"tasks.roll.differential_limit_m",
       SANDRO_WBC_FIELD(roll_differential_limit_m),
       "Maximum left-minus-right leg height difference in metres."},
      {"tasks.roll.imu_offset_rad", SANDRO_WBC_FIELD(roll_imu_offset_rad),
       "Roll the IMU reports when the body is level; subtracted before the roll task."},
      {"tasks.yaw.kp_nm_s_rad", SANDRO_WBC_FIELD(yaw_kp_nm_s_rad),
       "Differential wheel torque per rad/s of yaw-rate error."},
      {"tasks.yaw.ki_nm_rad", SANDRO_WBC_FIELD(yaw_ki_nm_rad),
       "Differential wheel torque per rad of integrated IMU yaw-rate error."},
      {"tasks.yaw.wheel_rate_blend", SANDRO_WBC_FIELD(yaw_wheel_rate_blend),
       "Yaw rate the proportional term uses: 1 wheel differential, 0 IMU."},
      {"tasks.yaw.scrub_ff_nm", SANDRO_WBC_FIELD(yaw_scrub_ff_nm),
       "Coulomb-like tyre scrub fed forward with the sign of the commanded yaw rate, Nm."},
      {"tasks.yaw.scrub_ff_smoothing_rad_s",
       SANDRO_WBC_FIELD(yaw_scrub_ff_smoothing_rad_s),
       "Commanded yaw rate over which the scrub feed-forward ramps in (tanh)."},
      {"tasks.yaw.torque_limit_nm", SANDRO_WBC_FIELD(yaw_torque_limit_nm),
       "Differential yaw torque limit (right minus left) in Nm."},
      {"tasks.yaw.torque_rate_nm_s", SANDRO_WBC_FIELD(yaw_torque_rate_nm_s),
       "Differential yaw torque slew in Nm/s."},
      {"tasks.yaw.torque_sign", SANDRO_WBC_FIELD(yaw_torque_sign),
       "Yaw torque polarity; exactly +1 or -1."},
      {"tasks.leg_fallback_damping_nm_s_rad",
       SANDRO_WBC_FIELD(leg_fallback_damping_nm_s_rad),
       "Joint damping for a leg whose kinematics cannot be evaluated."},
      {"limits.leg_torque_nm", SANDRO_WBC_FIELD(leg_torque_limit_nm),
       "Policy effort limit per leg joint in Nm."},
      {"limits.wheel_torque_nm", SANDRO_WBC_FIELD(wheel_torque_limit_nm),
       "Policy effort limit per wheel in Nm."},
      {"drive.max_forward_velocity_m_s",
       SANDRO_WBC_FIELD(max_forward_velocity_m_s),
       "Forward command limit in m/s."},
      {"drive.max_yaw_rate_rad_s", SANDRO_WBC_FIELD(max_yaw_rate_rad_s),
       "Yaw command limit in rad/s."},
      {"drive.forward_accel_limit_m_s2",
       SANDRO_WBC_FIELD(forward_accel_limit_m_s2),
       "Forward command slew in m/s^2."},
      {"drive.yaw_command_time_constant_s",
       SANDRO_WBC_FIELD(yaw_command_time_constant_s),
       "First-order lag of the yaw rate command in seconds."},
      {"drive.max_height_rate_m_s", SANDRO_WBC_FIELD(max_height_rate_m_s),
       "Height rate command limit (TwistStamped linear.z) in m/s."},
      {"safety.command_timeout_s", SANDRO_WBC_FIELD(command_timeout_s),
       "Twist command timeout in seconds."},
      {"safety.imu_timeout_s", SANDRO_WBC_FIELD(imu_timeout_s),
       "Maximum IMU sample age in seconds."},
      {"safety.settle_s", SANDRO_WBC_FIELD(settle_s),
       "Minimum time after activation before balancing may engage."},
      {"safety.ramp_s", SANDRO_WBC_FIELD(ramp_s),
       "Balance effort ramp duration after engaging, in seconds."},
      {"safety.fall_angle_rad", SANDRO_WBC_FIELD(fall_angle_rad),
       "Pitch magnitude that latches a fall, in rad."},
  };
  return table;
}
#undef SANDRO_WBC_FIELD

constexpr const char *kGainPointsName = "balance.gain_table.points";

bool apply_parameter(WholeBodyParameters &p, const Parameter &value) {
  const std::string &name = value.get_name();
  if (name == kGainPointsName) {
    p.gain_table_points = static_cast<int>(value.as_int());
    return true;
  }
  for (const auto &entry : double_parameters()) {
    if (name == entry.name) {
      entry.field(p) = value.as_double();
      return true;
    }
  }
  return false;
}

} // namespace

controller_interface::CallbackReturn WholeBodyController::on_init() {
  try {
    declare_parameters();
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter declaration failed: %s",
                 error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

void WholeBodyController::declare_parameters() {
  auto node = get_node();
  node->declare_parameter(
      "joints",
      std::vector<std::string>{"right_motor_inner", "right_motor_outer",
                               "right_wheel", "left_motor_inner",
                               "left_motor_outer", "left_wheel"},
      descriptor("Six joints in right-inner, right-outer, right-wheel, "
                 "left-inner, left-outer, left-wheel order.",
                 true));
  node->declare_parameter("imu_name", "imu",
                          descriptor("IMU state-interface prefix.", true));
  node->declare_parameter(
      "command_topic", "~/cmd_vel",
      descriptor("geometry_msgs/TwistStamped: linear.x forward speed, "
                 "angular.z yaw rate, linear.z height rate.",
                 true));
  node->declare_parameter(
      "diagnostics.publish_rate_hz", 10.0,
      descriptor("Diagnostic publication rate in Hz.", true));
  WholeBodyParameters defaults;
  for (const auto &entry : double_parameters()) {
    node->declare_parameter(entry.name, entry.field(defaults),
                            descriptor(entry.description));
  }
  node->declare_parameter(
      kGainPointsName, defaults.gain_table_points,
      descriptor("Number of leg heights at which LQR gains are solved."));
}

WholeBodyParameters WholeBodyController::read_parameters() const {
  WholeBodyParameters result;
  for (const auto &entry : double_parameters()) {
    apply_parameter(result, get_node()->get_parameter(entry.name));
  }
  apply_parameter(result, get_node()->get_parameter(kGainPointsName));
  return result;
}

controller_interface::InterfaceConfiguration
WholeBodyController::command_interface_configuration() const {
  // The full MIT frame of every joint: effort first (indices 0..5, as
  // before), then position, velocity, kp and kd blocks of six.
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  const auto joints = get_node()->get_parameter("joints").as_string_array();
  for (const char *interface :
       {hardware_interface::HW_IF_EFFORT, hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, "kp", "kd"}) {
    for (const auto &joint : joints) {
      config.names.push_back(joint + "/" + interface);
    }
  }
  return config;
}

controller_interface::InterfaceConfiguration
WholeBodyController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  const auto joints = get_node()->get_parameter("joints").as_string_array();
  for (const auto &joint : joints) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  for (const auto &joint : joints) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  const auto imu = get_node()->get_parameter("imu_name").as_string();
  for (const auto *name :
       {"orientation.x", "orientation.y", "orientation.z", "orientation.w",
        "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
        "linear_acceleration.x", "linear_acceleration.y",
        "linear_acceleration.z"}) {
    config.names.push_back(imu + "/" + name);
  }
  return config;
}

controller_interface::CallbackReturn
WholeBodyController::on_configure(const rclcpp_lifecycle::State &) {
  const auto joints = get_node()->get_parameter("joints").as_string_array();
  if (joints.size() != joint_names_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' must contain six names");
    return controller_interface::CallbackReturn::ERROR;
  }
  std::copy(joints.begin(), joints.end(), joint_names_.begin());
  imu_name_ = get_node()->get_parameter("imu_name").as_string();
  command_topic_ = get_node()->get_parameter("command_topic").as_string();
  diagnostics_rate_hz_ =
      get_node()->get_parameter("diagnostics.publish_rate_hz").as_double();
  if (imu_name_.empty() || command_topic_.empty() ||
      !std::isfinite(diagnostics_rate_hz_) || diagnostics_rate_hz_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "imu_name, command_topic and diagnostics rate must be set");
    return controller_interface::CallbackReturn::ERROR;
  }
  const auto update_rate = get_update_rate();
  control_period_s_ = update_rate > 0 ? 1.0 / static_cast<double>(update_rate)
                                      : 1.0 / 300.0;

  WholeBodySettings settings;
  std::string reason;
  accepted_parameters_ = read_parameters();
  if (!build_whole_body_settings(accepted_parameters_, control_period_s_,
                                 settings, &reason)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Invalid whole-body parameters: %s", reason.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  const WholeBodyGains gains =
      settings.gains.at(accepted_parameters_.height_target_m);
  RCLCPP_INFO(get_node()->get_logger(),
              "Whole-body LQR at %.3f m leg height (leg lag %.0f ms): wheel "
              "gains [x %.2f, v %.2f, pitch %.2f, rate %.2f, offset cmd %.2f, "
              "offset %.2f] Nm; leg gains [x %.2f, v %.2f, pitch %.2f, rate "
              "%.2f, offset cmd %.2f, offset %.2f] m/s; input-delay margin "
              "%.1f ms over the gain table",
              accepted_parameters_.height_target_m,
              1000.0 * accepted_parameters_.model.leg_lag_s, gains[0][0],
              gains[0][1], gains[0][2], gains[0][3], gains[0][4], gains[0][5],
              gains[1][0], gains[1][1], gains[1][2], gains[1][3], gains[1][4],
              gains[1][5], 1000.0 * settings.delay_margin_s);
  settings_.writeFromNonRT(settings);
  command_.writeFromNonRT(StampedCommand{});

  command_subscription_ =
      get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
          command_topic_, rclcpp::SystemDefaultsQoS(),
          [this](const geometry_msgs::msg::TwistStamped::SharedPtr message) {
            command_.writeFromNonRT(StampedCommand{
                message->twist.linear.x, message->twist.angular.z,
                message->twist.linear.z, get_node()->now().seconds()});
          });
  diagnostics_ros_publisher_ =
      get_node()->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
          "~/diagnostics", rclcpp::SystemDefaultsQoS());
  diagnostics_publisher_ = std::make_shared<
      realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>(
      diagnostics_ros_publisher_);

  parameter_callback_ = get_node()->add_on_set_parameters_callback(
      [this](const std::vector<Parameter> &changes) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        WholeBodyParameters proposed = accepted_parameters_;
        bool changed = false;
        try {
          for (const auto &change : changes) {
            changed = apply_parameter(proposed, change) || changed;
          }
        } catch (const std::exception &error) {
          result.successful = false;
          result.reason = error.what();
          return result;
        }
        if (!changed) {
          return result;
        }
        WholeBodySettings rebuilt;
        std::string reason;
        if (!build_whole_body_settings(proposed, control_period_s_, rebuilt,
                                       &reason)) {
          result.successful = false;
          result.reason = "Rejected whole-body parameter set: " + reason;
          return result;
        }
        accepted_parameters_ = proposed;
        settings_.writeFromNonRT(rebuilt);
        return result;
      });
  return controller_interface::CallbackReturn::SUCCESS;
}

bool WholeBodyController::read_state(WholeBodyState &state) const {
  for (std::size_t i = 0; i < 6; ++i) {
    const auto position = state_interfaces_[i].get_optional<double>();
    const auto velocity = state_interfaces_[6 + i].get_optional<double>();
    if (!position || !velocity || !std::isfinite(*position) ||
        !std::isfinite(*velocity)) {
      return false;
    }
    state.position[i] = *position;
    state.velocity[i] = *velocity;
  }
  std::array<double, 10> imu{};
  for (std::size_t i = 0; i < imu.size(); ++i) {
    const auto value = state_interfaces_[12 + i].get_optional<double>();
    if (!value) {
      return false;
    }
    imu[i] = *value;
  }
  state.quaternion_x = imu[0];
  state.quaternion_y = imu[1];
  state.quaternion_z = imu[2];
  state.quaternion_w = imu[3];
  state.angular_velocity_rad_s = {imu[4], imu[5], imu[6]};
  // The standard IMU semantic component carries no timestamp. The BNO085
  // plugin exports non-finite values when stale and MuJoCo samples every
  // cycle, so finiteness is the freshness signal.
  const bool fresh = std::isfinite(imu[0]) && std::isfinite(imu[1]) &&
                     std::isfinite(imu[2]) && std::isfinite(imu[3]) &&
                     std::isfinite(imu[4]) && std::isfinite(imu[5]) &&
                     std::isfinite(imu[6]);
  state.imu_age_s = fresh ? 0.0 : std::numeric_limits<double>::infinity();
  return true;
}

controller_interface::CallbackReturn
WholeBodyController::on_activate(const rclcpp_lifecycle::State &) {
  if (command_interfaces_.size() != 30 || state_interfaces_.size() != 22) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected 30 command (effort, position, velocity, kp, kd per "
                 "joint) and 22 state interfaces");
    return controller_interface::CallbackReturn::ERROR;
  }
  WholeBodyState state;
  if (!read_state(state)) {
    RCLCPP_ERROR(get_node()->get_logger(), "State interfaces unavailable");
    return controller_interface::CallbackReturn::ERROR;
  }
  core_.reset(state, *settings_.readFromRT());
  command_.writeFromNonRT(StampedCommand{});
  last_diagnostics_time_s_ = 0.0;
  last_reported_status_ = 0U;
  zero_commands();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
WholeBodyController::on_deactivate(const rclcpp_lifecycle::State &) {
  zero_commands();
  command_.writeFromNonRT(StampedCommand{});
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
WholeBodyController::update(const rclcpp::Time &time,
                            const rclcpp::Duration &period) {
  WholeBodyState state;
  if (!read_state(state)) {
    zero_commands();
    return controller_interface::return_type::ERROR;
  }
  const StampedCommand received = *command_.readFromRT();
  WholeBodyCommand command;
  command.forward_velocity_m_s = received.forward_velocity_m_s;
  command.yaw_rate_rad_s = received.yaw_rate_rad_s;
  command.height_rate_m_s = received.height_rate_m_s;
  command.age_s = received.receipt_time_s < 0.0
                      ? std::numeric_limits<double>::infinity()
                      : std::max(0.0, time.seconds() - received.receipt_time_s);
  const WholeBodyOutput output =
      core_.update(state, command, *settings_.readFromRT(), period.seconds());
  const std::array<const std::array<double, 6> *, 5> frame{
      &output.effort_nm, &output.position_command_rad,
      &output.velocity_command_rad_s, &output.kp_nm_rad, &output.kd_nm_s_rad};
  for (std::size_t block = 0; block < frame.size(); ++block) {
    for (std::size_t i = 0; i < 6; ++i) {
      const double value = (*frame[block])[i];
      if (!std::isfinite(value) ||
          !command_interfaces_[6 * block + i].set_value(value)) {
        zero_commands();
        return controller_interface::return_type::ERROR;
      }
    }
  }
  // Say once, in words, when something that stops the robot changes:
  // which sensor or leg went bad, and when it came back.
  constexpr std::uint32_t kReported =
      WBC_IMU_STALE | WBC_FALL_LATCH | WBC_LEG_KINEMATICS | WBC_INVALID_SETTINGS;
  const std::uint32_t reported = output.status & kReported;
  if (reported != last_reported_status_) {
    if (reported != 0U) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "whole-body controller: %s (legs valid right=%d left=%d, "
                  "IMU age %.3f s, pitch %.2f rad)",
                  whole_body_status_text(reported).c_str(),
                  output.leg_valid[0] ? 1 : 0, output.leg_valid[1] ? 1 : 0,
                  state.imu_age_s, output.pitch_rad);
    } else {
      RCLCPP_INFO(get_node()->get_logger(),
                  "whole-body controller: sensors and legs healthy again");
    }
    last_reported_status_ = reported;
  }
  publish_diagnostics(time, output, command.age_s, state.imu_age_s);
  return controller_interface::return_type::OK;
}

void WholeBodyController::zero_commands() {
  for (auto &command : command_interfaces_) {
    (void)command.set_value(0.0);
  }
}

void WholeBodyController::publish_diagnostics(const rclcpp::Time &time,
                                              const WholeBodyOutput &output,
                                              const double command_age_s,
                                              const double imu_age_s) {
  if (time.seconds() - last_diagnostics_time_s_ < 1.0 / diagnostics_rate_hz_ ||
      !diagnostics_publisher_ || !diagnostics_publisher_->trylock()) {
    return;
  }
  last_diagnostics_time_s_ = time.seconds();
  auto &message = diagnostics_publisher_->msg_;
  message.header.stamp = time;
  message.status.resize(1);
  auto &status = message.status.front();
  status.name = get_node()->get_name() + std::string("/whole_body");
  status.hardware_id = "sandro";
  const bool error = output.fallen ||
                     (output.status & (WBC_IMU_STALE | WBC_INVALID_SETTINGS));
  status.level = error ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                       : (output.status == WBC_OK
                              ? diagnostic_msgs::msg::DiagnosticStatus::OK
                              : diagnostic_msgs::msg::DiagnosticStatus::WARN);
  status.message = whole_body_status_text(output.status);
  status.values.clear();
  const auto add = [&status](const std::string &key, const auto value) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = std::to_string(value);
    status.values.push_back(std::move(item));
  };
  add("engaged", output.engaged ? 1 : 0);
  add("ramp", output.ramp);
  add("pitch_rad", output.pitch_rad);
  add("pitch_reference_rad", output.pitch_reference_rad);
  add("pitch_rate_rad_s", output.pitch_rate_rad_s);
  add("roll_rad", output.roll_rad);
  add("roll_reference_rad", output.roll_reference_rad);
  add("forward_velocity_m_s", output.forward_velocity_m_s);
  add("velocity_reference_m_s", output.velocity_reference_m_s);
  add("position_error_m", output.position_m - output.position_reference_m);
  add("yaw_rate_rad_s", output.yaw_rate_rad_s);
  add("wheel_yaw_rate_rad_s", output.wheel_yaw_rate_rad_s);
  add("yaw_rate_reference_rad_s", output.yaw_rate_reference_rad_s);
  add("leg_height_m", output.leg_height_m);
  add("height_reference_m", output.height_reference_m);
  add("height_differential_m", output.height_differential_m);
  add("height_support_n", output.height_support_n);
  add("leg_offset_m", output.leg_offset_m);
  add("leg_offset_reference_m", output.leg_offset_reference_m);
  add("leg_offset_command_m", output.leg_offset_command_m);
  add("leg_offset_model_m", output.leg_offset_model_m);
  add("leg_pd_in_drive", output.kp_nm_rad[0] > 0.0 || output.kp_nm_rad[3] > 0.0 ? 1 : 0);
  for (const std::size_t i : {0U, 1U, 3U, 4U}) {
    add("position_command_" + joint_names_[i] + "_rad", output.position_command_rad[i]);
  }
  add("leg_offset_state_m", output.leg_offset_state_m);
  add("leg_offset_rate_m_s", output.leg_offset_rate_m_s);
  add("trim_m", output.trim_m);
  add("balance_torque_nm", output.balance_torque_nm);
  add("yaw_torque_nm", output.yaw_torque_nm);
  add("yaw_feedforward_nm", output.yaw_feedforward_nm);
  add("right_leg_reaction_inner_nm", output.leg_reaction_torque_nm[0][0]);
  add("right_leg_reaction_outer_nm", output.leg_reaction_torque_nm[0][1]);
  add("left_leg_reaction_inner_nm", output.leg_reaction_torque_nm[1][0]);
  add("left_leg_reaction_outer_nm", output.leg_reaction_torque_nm[1][1]);
  add("right_leg_x_m", output.leg_position_m[0].x);
  add("right_leg_y_m", output.leg_position_m[0].y);
  add("left_leg_x_m", output.leg_position_m[1].x);
  add("left_leg_y_m", output.leg_position_m[1].y);
  add("gain_wheel_pitch_nm_rad", output.gains[0][2]);
  add("gain_wheel_velocity_nm_s_m", output.gains[0][1]);
  add("gain_leg_pitch_m_s_rad", output.gains[1][2]);
  add("gain_leg_offset_command_per_s", output.gains[1][4]);
  add("gain_leg_offset_per_s", output.gains[1][5]);
  for (std::size_t i = 0; i < output.effort_nm.size(); ++i) {
    add("effort_" + joint_names_[i] + "_nm", output.effort_nm[i]);
  }
  add("command_age_s", command_age_s);
  add("imu_age_s", imu_age_s);
  add("status_mask", output.status);
  diagnostics_publisher_->unlockAndPublish();
}

} // namespace sandro_controllers

PLUGINLIB_EXPORT_CLASS(sandro_controllers::WholeBodyController,
                       controller_interface::ControllerInterface)
