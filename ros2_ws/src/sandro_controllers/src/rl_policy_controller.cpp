#include "sandro_controllers/rl_policy_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace sandro_controllers {
namespace {

rcl_interfaces::msg::ParameterDescriptor read_only(const std::string &text) {
  rcl_interfaces::msg::ParameterDescriptor result;
  result.description = text;
  result.read_only = true;
  return result;
}

struct DoubleParameter {
  const char *name;
  double RlPolicySettings::*field;
  const char *description;
};

const std::vector<DoubleParameter> &double_parameters() {
  static const std::vector<DoubleParameter> table{
      {"policy_rate_hz", &RlPolicySettings::policy_rate_hz, "Policy step rate (Hz)."},
      {"observation.gyro_scale", &RlPolicySettings::gyro_scale, "Gyro observation scale."},
      {"observation.leg_velocity_scale", &RlPolicySettings::leg_velocity_scale,
       "Leg joint velocity observation scale."},
      {"observation.wheel_velocity_scale", &RlPolicySettings::wheel_velocity_scale,
       "Wheel velocity observation scale."},
      {"observation.imu_offset_rad", &RlPolicySettings::imu_offset_rad,
       "IMU roll when the body is level (mounting), removed before the gravity observation."},
      {"commands.forward_m_s", &RlPolicySettings::forward_m_s, "Forward speed command limit."},
      {"commands.yaw_rad_s", &RlPolicySettings::yaw_rad_s, "Yaw rate command limit."},
      {"commands.height_min_m", &RlPolicySettings::height_min_m, "Lowest body height command."},
      {"commands.height_max_m", &RlPolicySettings::height_max_m, "Highest body height command."},
      {"commands.roll_rad", &RlPolicySettings::roll_rad,
       "Body roll command limit; 0 for policies without a roll command."},
      {"commands.initial_height_m", &RlPolicySettings::initial_height_m,
       "Height command on engaging; the reference moves there from the measured height."},
      {"shaping.forward_accel_m_s2", &RlPolicySettings::forward_accel_m_s2,
       "Forward reference acceleration limit."},
      {"shaping.yaw_time_constant_s", &RlPolicySettings::yaw_time_constant_s,
       "Yaw rate reference time constant."},
      {"shaping.height_rate_m_s", &RlPolicySettings::height_rate_m_s,
       "Height reference rate limit."},
      {"shaping.roll_rate_rad_s", &RlPolicySettings::roll_rate_rad_s, "Roll reference rate limit."},
      {"shaping.leg_height_min_m", &RlPolicySettings::leg_height_min_m,
       "Lowest reachable leg height, bounds the roll command."},
      {"shaping.leg_height_max_m", &RlPolicySettings::leg_height_max_m,
       "Highest reachable leg height, bounds the roll command."},
      {"shaping.track_width_m", &RlPolicySettings::track_width_m, "Wheel track width."},
      {"action.leg_scale_rad", &RlPolicySettings::leg_scale_rad, "Leg target per unit action."},
      {"action.leg_kp_nm_rad", &RlPolicySettings::leg_kp_nm_rad, "Leg drive stiffness."},
      {"action.leg_kd_nm_s_rad", &RlPolicySettings::leg_kd_nm_s_rad, "Leg drive damping."},
      {"action.joint_min_rad", &RlPolicySettings::joint_min_rad, "Leg target lower bound."},
      {"action.joint_max_rad", &RlPolicySettings::joint_max_rad, "Leg target upper bound."},
      {"action.wheel_scale_rad_s", &RlPolicySettings::wheel_scale_rad_s,
       "Wheel velocity target per unit action (velocity mode)."},
      {"action.wheel_kd_nm_s_rad", &RlPolicySettings::wheel_kd_nm_s_rad,
       "Wheel drive damping (velocity mode)."},
      {"action.wheel_scale_nm", &RlPolicySettings::wheel_scale_nm,
       "Wheel torque per unit action (torque mode)."},
      {"safety.fall_angle_rad", &RlPolicySettings::fall_angle_rad,
       "Tilt beyond which the drives go limp until re-activation."},
      {"safety.imu_timeout_s", &RlPolicySettings::imu_timeout_s,
       "IMU staleness that counts as a fall."},
      {"safety.command_timeout_s", &RlPolicySettings::command_timeout_s,
       "Command age beyond which the command is zero."},
      {"safety.stand_s", &RlPolicySettings::stand_s,
       "Time to move the legs to the stand pose before the policy engages."},
      {"safety.stand_tolerance_rad", &RlPolicySettings::stand_tolerance_rad,
       "Leg joints closer than this to the stand pose skip the stand-up move."},
      {"safety.engage_tilt_rad", &RlPolicySettings::engage_tilt_rad,
       "The policy engages only with the body tilted less than this."},
      {"safety.engage_rate_rad_s", &RlPolicySettings::engage_rate_rad_s,
       "The policy engages only with the body turning slower than this."},
      {"safety.engage_joint_velocity_rad_s", &RlPolicySettings::engage_joint_velocity_rad_s,
       "The policy engages only with every joint turning slower than this."},
      {"safety.engage_hold_s", &RlPolicySettings::engage_hold_s,
       "How long the body must be upright and still before the policy engages."},
      {"safety.fallen_kd_nm_s_rad", &RlPolicySettings::fallen_kd_nm_s_rad,
       "Damping on every joint after a fall."},
  };
  return table;
}

std::string resolve(const std::string &path) {
  const std::string prefix = "package://";
  if (path.rfind(prefix, 0) != 0) {
    return path;
  }
  const auto slash = path.find('/', prefix.size());
  const std::string package = path.substr(prefix.size(), slash - prefix.size());
  return ament_index_cpp::get_package_share_directory(package) +
         (slash == std::string::npos ? "" : path.substr(slash));
}

const char *phase_name(const RlPhase phase) {
  switch (phase) {
  case RlPhase::kStand:
    return "standing up";
  case RlPhase::kPolicy:
    return "policy";
  default:
    return "fallen";
  }
}

} // namespace

controller_interface::CallbackReturn RlPolicyController::on_init() {
  try {
    declare_parameters();
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter declaration failed: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

void RlPolicyController::declare_parameters() {
  auto node = get_node();
  node->declare_parameter(
      "joints",
      std::vector<std::string>{"right_motor_inner", "right_motor_outer", "right_wheel",
                               "left_motor_inner", "left_motor_outer", "left_wheel"},
      read_only("Six joints in the policy's order."));
  node->declare_parameter("imu_name", "imu", read_only("IMU state-interface prefix."));
  node->declare_parameter("command_topic", "~/cmd_vel",
                          read_only("geometry_msgs/TwistStamped: linear.x forward speed, "
                                    "angular.z yaw rate, linear.z height rate, angular.x roll."));
  node->declare_parameter("diagnostics.publish_rate_hz", 50.0,
                          read_only("Diagnostic publication rate in Hz."));
  node->declare_parameter("policy_file", "",
                          read_only("Actor written by export_ros.py; package:// paths allowed."));
  node->declare_parameter("policy_fingerprint", "",
                          read_only("Model fingerprint the policy was trained on."));
  node->declare_parameter("observation.history", 4, read_only("Frames stacked, newest first."));
  node->declare_parameter("observation.command_size", 3,
                          read_only("Command terms: 3, or 4 with a roll command."));
  node->declare_parameter("action.wheel_mode", "velocity", read_only("velocity or torque."));
  node->declare_parameter("action.stand_q", std::vector<double>(6, 0.0),
                          read_only("Joint pose the leg actions are relative to."));
  const LegGeometry geometry;
  node->declare_parameter("legs.base_m", geometry.base, read_only("Five-bar pivot spacing."));
  node->declare_parameter("legs.upper_m", geometry.upper, read_only("Five-bar upper link."));
  node->declare_parameter("legs.lower_m", geometry.lower, read_only("Five-bar lower link."));
  const RlPolicySettings defaults;
  for (const auto &entry : double_parameters()) {
    node->declare_parameter(entry.name, defaults.*entry.field, read_only(entry.description));
  }
}

bool RlPolicyController::read_settings(RlPolicySettings &s, std::string *error) const {
  const auto node = get_node();
  for (const auto &entry : double_parameters()) {
    s.*entry.field = node->get_parameter(entry.name).as_double();
  }
  s.history = static_cast<int>(node->get_parameter("observation.history").as_int());
  s.command_size = static_cast<int>(node->get_parameter("observation.command_size").as_int());
  const auto mode = node->get_parameter("action.wheel_mode").as_string();
  if (mode != "velocity" && mode != "torque") {
    *error = "action.wheel_mode must be velocity or torque";
    return false;
  }
  s.wheel_velocity_mode = mode == "velocity";
  const auto stand = node->get_parameter("action.stand_q").as_double_array();
  if (stand.size() != 6) {
    *error = "action.stand_q needs six values";
    return false;
  }
  std::copy(stand.begin(), stand.end(), s.stand_q.begin());
  s.leg_geometry.base = node->get_parameter("legs.base_m").as_double();
  s.leg_geometry.upper = node->get_parameter("legs.upper_m").as_double();
  s.leg_geometry.lower = node->get_parameter("legs.lower_m").as_double();
  const auto file = node->get_parameter("policy_file").as_string();
  if (file.empty()) {
    *error = "policy_file is not set";
    return false;
  }
  try {
    s.policy_file = resolve(file);
  } catch (const std::exception &e) {
    *error = "cannot resolve '" + file + "': " + e.what();
    return false;
  }
  return true;
}

controller_interface::InterfaceConfiguration
RlPolicyController::command_interface_configuration() const {
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
RlPolicyController::state_interface_configuration() const {
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
  for (const auto *name : {"orientation.x", "orientation.y", "orientation.z", "orientation.w",
                           "angular_velocity.x", "angular_velocity.y", "angular_velocity.z"}) {
    config.names.push_back(imu + "/" + name);
  }
  return config;
}

controller_interface::CallbackReturn
RlPolicyController::on_configure(const rclcpp_lifecycle::State &) {
  const auto node = get_node();
  if (node->get_parameter("joints").as_string_array().size() != 6) {
    RCLCPP_ERROR(node->get_logger(), "'joints' must contain six names");
    return controller_interface::CallbackReturn::ERROR;
  }
  diagnostics_rate_hz_ = node->get_parameter("diagnostics.publish_rate_hz").as_double();
  RlPolicySettings settings;
  std::string error;
  const int update_rate = get_update_rate() > 0 ? static_cast<int>(get_update_rate()) : 300;
  if (!read_settings(settings, &error) || !core_.configure(settings, update_rate, &error)) {
    RCLCPP_ERROR(node->get_logger(), "Invalid policy configuration: %s", error.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(node->get_logger(),
              "Policy %s: %d x %d observations, command limits %.2f m/s, %.2f rad/s, "
              "height %.3f-%.3f m, roll %.2f rad; IMU offset %.3f rad; trained on model %.12s",
              settings.policy_file.c_str(), settings.history, core_.frame_size(),
              settings.forward_m_s, settings.yaw_rad_s, settings.height_min_m,
              settings.height_max_m, settings.roll_rad, settings.imu_offset_rad,
              node->get_parameter("policy_fingerprint").as_string().c_str());

  command_.writeFromNonRT(StampedCommand{});
  command_subscription_ = node->create_subscription<geometry_msgs::msg::TwistStamped>(
      node->get_parameter("command_topic").as_string(), rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr message) {
        command_.writeFromNonRT(StampedCommand{message->twist.linear.x, message->twist.angular.z,
                                               message->twist.linear.z, message->twist.angular.x,
                                               get_node()->now().seconds()});
      });
  diagnostics_ros_publisher_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "~/diagnostics", rclcpp::SystemDefaultsQoS());
  diagnostics_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>(
          diagnostics_ros_publisher_);
  return controller_interface::CallbackReturn::SUCCESS;
}

bool RlPolicyController::read_state(RlPolicyState &state) const {
  for (std::size_t i = 0; i < 6; ++i) {
    const auto position = state_interfaces_[i].get_optional<double>();
    const auto velocity = state_interfaces_[6 + i].get_optional<double>();
    if (!position || !velocity) {
      return false;
    }
    state.position[i] = *position;
    state.velocity[i] = *velocity;
  }
  for (std::size_t i = 0; i < 7; ++i) {
    const auto value = state_interfaces_[12 + i].get_optional<double>();
    if (!value) {
      return false;
    }
    if (i < 4) {
      state.orientation[i] = *value;
    } else {
      state.angular_velocity[i - 4] = *value;
    }
  }
  return true;
}

controller_interface::CallbackReturn
RlPolicyController::on_activate(const rclcpp_lifecycle::State &) {
  if (command_interfaces_.size() != 30 || state_interfaces_.size() != 19) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected 30 command (effort, position, velocity, kp, kd per joint) and 19 "
                 "state interfaces");
    return controller_interface::CallbackReturn::ERROR;
  }
  RlPolicyState state;
  if (!read_state(state)) {
    RCLCPP_ERROR(get_node()->get_logger(), "State interfaces unavailable");
    return controller_interface::CallbackReturn::ERROR;
  }
  core_.reset(state);
  command_.writeFromNonRT(StampedCommand{});
  last_diagnostics_time_s_ = 0.0;
  inference_max_s_ = 0.0;
  last_phase_ = RlPhase::kStand;
  last_reported_status_ = 0U;
  zero_commands();
  RCLCPP_INFO(get_node()->get_logger(), "%s",
              core_.phase() == RlPhase::kStand
                  ? "Activated: legs move to the stand pose, the policy engages once upright"
                  : "Activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
RlPolicyController::on_deactivate(const rclcpp_lifecycle::State &) {
  zero_commands();
  command_.writeFromNonRT(StampedCommand{});
  RCLCPP_INFO(get_node()->get_logger(), "Deactivated; longest inference %.0f us",
              1e6 * inference_max_s_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type RlPolicyController::update(const rclcpp::Time &time,
                                                             const rclcpp::Duration &period) {
  RlPolicyState state;
  if (!read_state(state)) {
    zero_commands();
    return controller_interface::return_type::ERROR;
  }
  const StampedCommand received = *command_.readFromRT();
  RlPolicyCommand command{received.forward_m_s, received.yaw_rate_rad_s,
                          received.height_rate_m_s, received.roll_rad,
                          received.receipt_time_s < 0.0
                              ? std::numeric_limits<double>::infinity()
                              : std::max(0.0, time.seconds() - received.receipt_time_s)};
  const RlPolicyOutput output = core_.update(state, command, period.seconds());
  if (output.inferred) {
    inference_last_s_ = output.inference_s;
    inference_max_s_ = std::max(inference_max_s_, output.inference_s);
  }
  const std::array<const std::array<double, 6> *, 5> frame{
      &output.frame.effort, &output.frame.position, &output.frame.velocity, &output.frame.kp,
      &output.frame.kd};
  for (std::size_t block = 0; block < frame.size(); ++block) {
    for (std::size_t i = 0; i < 6; ++i) {
      const double value = (*frame[block])[i];
      if (!std::isfinite(value) || !command_interfaces_[6 * block + i].set_value(value)) {
        zero_commands();
        return controller_interface::return_type::ERROR;
      }
    }
  }
  report(state, output);
  publish_diagnostics(time, output, command.age_s);
  return controller_interface::return_type::OK;
}

void RlPolicyController::report(const RlPolicyState &state, const RlPolicyOutput &output) {
  if (output.phase != last_phase_) {
    if (output.phase == RlPhase::kFallen) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Fallen (tilt %.2f rad, status 0x%x): drives limp until re-activation",
                  output.tilt_rad, output.status);
    } else if (output.phase == RlPhase::kPolicy) {
      const auto &w = state.angular_velocity;
      const auto &v = state.velocity;
      RCLCPP_INFO(get_node()->get_logger(),
                  "Phase: policy (pitch %+.3f rad, gyro %+.2f %+.2f %+.2f rad/s, leg velocity "
                  "%+.2f %+.2f %+.2f %+.2f, wheel velocity %+.2f %+.2f rad/s, action %+.2f %+.2f "
                  "%+.2f %+.2f %+.2f %+.2f)",
                  output.pitch_rad, w[0], w[1], w[2], v[0], v[1], v[3], v[4], v[2], v[5],
                  output.action[0], output.action[1], output.action[2], output.action[3],
                  output.action[4], output.action[5]);
    } else {
      RCLCPP_INFO(get_node()->get_logger(), "Phase: %s", phase_name(output.phase));
    }
    last_phase_ = output.phase;
  }
  constexpr std::uint32_t kReported = RL_IMU_STALE | RL_WAITING_UPRIGHT | RL_NON_FINITE_ACTION;
  const std::uint32_t reported = output.status & kReported;
  if (reported != last_reported_status_) {
    if (reported & RL_WAITING_UPRIGHT) {
      RCLCPP_INFO(get_node()->get_logger(),
                  "Legs at the stand pose; waiting for the body to be upright (tilt %.2f rad)",
                  output.tilt_rad);
    }
    if (reported & RL_IMU_STALE) {
      RCLCPP_WARN(get_node()->get_logger(), "IMU stale");
    }
    if (reported & RL_NON_FINITE_ACTION) {
      RCLCPP_ERROR(get_node()->get_logger(), "Policy returned a non-finite action");
    }
    last_reported_status_ = reported;
  }
}

void RlPolicyController::zero_commands() {
  for (auto &command : command_interfaces_) {
    (void)command.set_value(0.0);
  }
}

void RlPolicyController::publish_diagnostics(const rclcpp::Time &time,
                                             const RlPolicyOutput &output,
                                             const double command_age_s) {
  if (time.seconds() - last_diagnostics_time_s_ < 1.0 / diagnostics_rate_hz_ ||
      !diagnostics_publisher_ || !diagnostics_publisher_->trylock()) {
    return;
  }
  last_diagnostics_time_s_ = time.seconds();
  auto &message = diagnostics_publisher_->msg_;
  message.header.stamp = time;
  message.status.resize(1);
  auto &status = message.status.front();
  status.name = get_node()->get_name() + std::string("/rl_policy");
  status.hardware_id = "sandro";
  status.level = output.phase == RlPhase::kFallen ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                 : output.phase == RlPhase::kStand ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                                                   : diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = phase_name(output.phase);
  status.values.clear();
  const auto add = [&status](const std::string &key, const auto value) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = std::to_string(value);
    status.values.push_back(std::move(item));
  };
  add("phase", static_cast<int>(output.phase));
  add("status_mask", output.status);
  add("tilt_rad", output.tilt_rad);
  add("rate_rad_s", output.rate_rad_s);
  add("pitch_rad", output.pitch_rad);
  add("roll_rad", output.roll_rad);
  add("velocity_reference_m_s", output.reference[0]);
  add("yaw_rate_reference_rad_s", output.reference[1]);
  add("height_reference_m", output.reference[2]);
  add("roll_reference_rad", output.reference[3]);
  add("right_leg_height_m", output.leg_height_m[0]);
  add("left_leg_height_m", output.leg_height_m[1]);
  for (std::size_t i = 0; i < 6; ++i) {
    add("action_" + std::to_string(i), output.action[i]);
  }
  add("inference_us", 1e6 * inference_last_s_);
  add("inference_max_us", 1e6 * inference_max_s_);
  add("command_age_s", command_age_s);
  diagnostics_publisher_->unlockAndPublish();
}

} // namespace sandro_controllers

PLUGINLIB_EXPORT_CLASS(sandro_controllers::RlPolicyController,
                       controller_interface::ControllerInterface)
