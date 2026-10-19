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
#include "sandro_controllers/whole_body_parameters.hpp"

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

bool apply_parameter(WholeBodyParameters &p, const Parameter &value) {
  if (value.get_name() == kWholeBodyGainPointsName) {
    return set_whole_body_parameter(p, value.get_name(),
                                    static_cast<double>(value.as_int()));
  }
  for (const auto &entry : whole_body_double_parameters()) {
    if (value.get_name() == entry.name) {
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
  for (const auto &entry : whole_body_double_parameters()) {
    node->declare_parameter(entry.name, entry.field(defaults),
                            descriptor(entry.description));
  }
  node->declare_parameter(
      kWholeBodyGainPointsName, defaults.gain_table_points,
      descriptor("Number of leg heights at which LQR gains are solved."));
}

WholeBodyParameters WholeBodyController::read_parameters() const {
  WholeBodyParameters result;
  for (const auto &entry : whole_body_double_parameters()) {
    apply_parameter(result, get_node()->get_parameter(entry.name));
  }
  apply_parameter(result, get_node()->get_parameter(kWholeBodyGainPointsName));
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
