#include "sandro_controllers/lqr_balancer_controller.hpp"

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
using Settings = LqrBalancerController::Settings;

rcl_interfaces::msg::ParameterDescriptor
descriptor(const std::string &text, const bool read_only = false) {
  rcl_interfaces::msg::ParameterDescriptor result;
  result.description = text;
  result.read_only = read_only;
  return result;
}

// Only the parameters the LQR core actually reads are declared. The cascade's
// speed-to-pitch and wheel damping knobs have no meaning here: that feedback
// lives in the gain vector instead.
bool apply_parameter(Settings &s, const Parameter &value) {
  const std::string &n = value.get_name();
#define SANDRO_CONTROL_PARAMETER(name, member)                                 \
  if (n == name) {                                                             \
    s.control.member = value.as_double();                                      \
    return true;                                                               \
  }
#define SANDRO_GAIN_PARAMETER(name, member)                                    \
  if (n == name) {                                                             \
    s.gains.member = value.as_double();                                        \
    return true;                                                               \
  }
  SANDRO_CONTROL_PARAMETER("legs.right.base_m", right_geometry.base)
  SANDRO_CONTROL_PARAMETER("legs.right.upper_m", right_geometry.upper)
  SANDRO_CONTROL_PARAMETER("legs.right.lower_m", right_geometry.lower)
  SANDRO_CONTROL_PARAMETER("legs.left.base_m", left_geometry.base)
  SANDRO_CONTROL_PARAMETER("legs.left.upper_m", left_geometry.upper)
  SANDRO_CONTROL_PARAMETER("legs.left.lower_m", left_geometry.lower)
  SANDRO_CONTROL_PARAMETER("legs.height_m", leg_height_m)
  SANDRO_CONTROL_PARAMETER("legs.height_rate_m_s", leg_height_rate_m_s)
  SANDRO_CONTROL_PARAMETER("legs.kp_nm_rad", leg_kp_nm_rad)
  SANDRO_CONTROL_PARAMETER("legs.kd_nm_s_rad", leg_kd_nm_s_rad)
  SANDRO_CONTROL_PARAMETER("legs.torque_limit_nm", leg_torque_limit_nm)
  SANDRO_CONTROL_PARAMETER("legs.target_rate_rad_s", leg_target_rate_rad_s)
  SANDRO_CONTROL_PARAMETER("legs.joint_min_rad", leg_joint_min_rad)
  SANDRO_CONTROL_PARAMETER("legs.joint_max_rad", leg_joint_max_rad)
  SANDRO_CONTROL_PARAMETER("balance.pitch_target_rad", pitch_target_rad)
  SANDRO_CONTROL_PARAMETER("balance.torque_limit_nm", balance_torque_limit_nm)
  SANDRO_CONTROL_PARAMETER("balance.torque_rate_nm_s", balance_torque_rate_nm_s)
  SANDRO_CONTROL_PARAMETER("balance.torque_sign", balance_torque_sign)
  SANDRO_CONTROL_PARAMETER("balance.imu_timeout_s", imu_timeout_s)
  SANDRO_CONTROL_PARAMETER("balance.settle_s", settle_s)
  SANDRO_CONTROL_PARAMETER("balance.ramp_s", ramp_s)
  SANDRO_CONTROL_PARAMETER("balance.fall_angle_rad", fall_angle_rad)
  SANDRO_CONTROL_PARAMETER("wheels.torque_limit_nm", wheel_torque_limit_nm)
  SANDRO_CONTROL_PARAMETER("drive.wheel_radius_m", wheel_radius_m)
  SANDRO_CONTROL_PARAMETER("drive.track_width_m", track_width_m)
  SANDRO_CONTROL_PARAMETER("drive.max_forward_velocity_m_s",
                           max_forward_velocity_m_s)
  SANDRO_CONTROL_PARAMETER("drive.max_yaw_rate_rad_s", max_yaw_rate_rad_s)
  SANDRO_CONTROL_PARAMETER("drive.forward_accel_limit_m_s2",
                           forward_accel_limit_m_s2)
  SANDRO_CONTROL_PARAMETER("drive.command_deadband_m_s", command_deadband_m_s)
  SANDRO_CONTROL_PARAMETER("drive.yaw_kp_nm_s_rad", yaw_kp_nm_s_rad)
  SANDRO_CONTROL_PARAMETER("drive.yaw_torque_limit_nm", yaw_torque_limit_nm)
  SANDRO_CONTROL_PARAMETER("drive.yaw_torque_rate_nm_s", yaw_torque_rate_nm_s)
  SANDRO_CONTROL_PARAMETER("drive.yaw_torque_sign", yaw_torque_sign)
  SANDRO_CONTROL_PARAMETER("drive.command_timeout_s", command_timeout_s)
  SANDRO_GAIN_PARAMETER("lqr.position_nm_m", position_nm_m)
  SANDRO_GAIN_PARAMETER("lqr.velocity_nm_s_m", velocity_nm_s_m)
  SANDRO_GAIN_PARAMETER("lqr.pitch_nm_rad", pitch_nm_rad)
  SANDRO_GAIN_PARAMETER("lqr.pitch_rate_nm_s_rad", pitch_rate_nm_s_rad)
  SANDRO_GAIN_PARAMETER("lqr.velocity_integral_nm_s_m",
                        velocity_integral_nm_s_m)
  SANDRO_GAIN_PARAMETER("lqr.velocity_integral_limit_nm",
                        velocity_integral_limit_nm)
  SANDRO_GAIN_PARAMETER("lqr.engage_angle_rad", engage_angle_rad)
#undef SANDRO_GAIN_PARAMETER
#undef SANDRO_CONTROL_PARAMETER
  return false;
}

const std::vector<std::string> &parameter_names() {
  static const std::vector<std::string> names = {
      "legs.right.base_m",
      "legs.right.upper_m",
      "legs.right.lower_m",
      "legs.left.base_m",
      "legs.left.upper_m",
      "legs.left.lower_m",
      "legs.height_m",
      "legs.height_rate_m_s",
      "legs.kp_nm_rad",
      "legs.kd_nm_s_rad",
      "legs.torque_limit_nm",
      "legs.target_rate_rad_s",
      "legs.joint_min_rad",
      "legs.joint_max_rad",
      "balance.pitch_target_rad",
      "balance.torque_limit_nm",
      "balance.torque_rate_nm_s",
      "balance.torque_sign",
      "balance.imu_timeout_s",
      "balance.settle_s",
      "balance.ramp_s",
      "balance.fall_angle_rad",
      "wheels.torque_limit_nm",
      "drive.wheel_radius_m",
      "drive.track_width_m",
      "drive.max_forward_velocity_m_s",
      "drive.max_yaw_rate_rad_s",
      "drive.forward_accel_limit_m_s2",
      "drive.command_deadband_m_s",
      "drive.yaw_kp_nm_s_rad",
      "drive.yaw_torque_limit_nm",
      "drive.yaw_torque_rate_nm_s",
      "drive.yaw_torque_sign",
      "drive.command_timeout_s",
      "lqr.position_nm_m",
      "lqr.velocity_nm_s_m",
      "lqr.pitch_nm_rad",
      "lqr.pitch_rate_nm_s_rad",
      "lqr.velocity_integral_nm_s_m",
      "lqr.velocity_integral_limit_nm",
      "lqr.engage_angle_rad"};
  return names;
}

std::string saturation_text(const std::uint32_t mask) {
  if (mask == SAT_NONE) {
    return "none";
  }
  std::string text;
  const auto add = [&text, mask](const std::uint32_t bit, const char *name) {
    if ((mask & bit) != 0U) {
      if (!text.empty()) {
        text += ",";
      }
      text += name;
    }
  };
  add(SAT_COMMAND_STALE, "command_stale");
  add(SAT_FORWARD_LIMIT, "forward_limit");
  add(SAT_YAW_RATE_LIMIT, "yaw_rate_limit");
  add(SAT_BALANCE_TORQUE_LIMIT, "balance_torque_limit");
  add(SAT_YAW_TORQUE_LIMIT, "yaw_torque_limit");
  add(SAT_TOTAL_WHEEL_TORQUE, "total_wheel_torque");
  add(SAT_LEG_TORQUE, "leg_torque");
  add(SAT_IMU_STALE, "imu_stale");
  add(SAT_FALL_LATCH, "fall_latch");
  add(SAT_SETTLING, "waiting_for_upright_or_ramping");
  return text;
}
} // namespace

controller_interface::CallbackReturn LqrBalancerController::on_init() {
  try {
    declare_parameters();
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter declaration failed: %s",
                 error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

void LqrBalancerController::declare_parameters() {
  auto node = get_node();
  node->declare_parameter(
      "joints",
      std::vector<std::string>{"right_motor_inner", "right_motor_outer",
                               "right_wheel", "left_motor_inner",
                               "left_motor_outer", "left_wheel"},
      descriptor(
          "Six joints in right-inner, right-outer, right-wheel, left-inner, "
          "left-outer, left-wheel order.",
          true));
  node->declare_parameter(
      "imu_name", "imu",
      descriptor("Prefix of the IMU state interfaces.", true));
  node->declare_parameter(
      "command_topic", "~/cmd_vel",
      descriptor("geometry_msgs/TwistStamped motion command topic.", true));
  node->declare_parameter(
      "diagnostics.publish_rate_hz", 10.0,
      descriptor(
          "Requested/effective/saturation diagnostic publication rate in Hz.",
          true));

  const auto declare = [&node](const std::string &name, const double initial,
                               const std::string &description) {
    node->declare_parameter(name, initial, descriptor(description));
  };
  declare("legs.right.base_m", 0.09, "Right five-bar pivot spacing in metres.");
  declare("legs.right.upper_m", 0.08,
          "Right five-bar upper-link length in metres.");
  declare("legs.right.lower_m", 0.13,
          "Right five-bar lower-link length in metres.");
  declare("legs.left.base_m", 0.09, "Left five-bar pivot spacing in metres.");
  declare("legs.left.upper_m", 0.08,
          "Left five-bar upper-link length in metres.");
  declare("legs.left.lower_m", 0.13,
          "Left five-bar lower-link length in metres.");
  declare("legs.height_m", 0.17, "Cartesian foot height (+down) in metres.");
  declare("legs.height_rate_m_s", 0.03,
          "Maximum height-target slew in metres/second.");
  declare("legs.kp_nm_rad", 12.0, "Leg joint proportional gain in Nm/rad.");
  declare("legs.kd_nm_s_rad", 0.5, "Leg joint damping gain in Nm/(rad/s).");
  declare("legs.torque_limit_nm", 3.0,
          "Absolute effort limit for each leg joint in Nm.");
  declare("legs.target_rate_rad_s", 6.0,
          "Maximum IK joint-target slew in rad/s.");
  declare("legs.joint_min_rad", -2.0943951024,
          "Minimum commanded leg angle in rad.");
  declare("legs.joint_max_rad", 2.0943951024,
          "Maximum commanded leg angle in rad.");
  declare("balance.pitch_target_rad", 0.0,
          "Upright pitch trim in rad; the LQR regulates pitch to this.");
  declare("balance.torque_limit_nm", 6.0,
          "Absolute balance torque per wheel in Nm.");
  declare("balance.torque_rate_nm_s", 100.0, "Balance torque slew in Nm/s.");
  declare("balance.torque_sign", 1.0, "Balance polarity; exactly +1 or -1.");
  declare("balance.imu_timeout_s", 0.15, "Maximum IMU sample age in seconds.");
  declare("balance.settle_s", 1.0,
          "Zero-balance settling duration after activation in s.");
  declare("balance.ramp_s", 0.5,
          "Balance gain ramp duration after settling in s.");
  declare("balance.fall_angle_rad", 1.0,
          "Pitch magnitude that latches a fall in rad.");
  declare("wheels.torque_limit_nm", 6.0,
          "Absolute total effort limit per wheel in Nm.");
  declare("drive.wheel_radius_m", 0.0742, "Effective wheel radius in metres.");
  declare("drive.track_width_m", 0.304, "Wheel-centre spacing in metres.");
  declare("drive.max_forward_velocity_m_s", 0.5,
          "Forward command limit in m/s.");
  declare("drive.max_yaw_rate_rad_s", 1.5, "Yaw command limit in rad/s.");
  declare("drive.forward_accel_limit_m_s2", 1.0,
          "Forward command slew in m/s^2.");
  declare("drive.command_deadband_m_s", 0.01,
          "Below this the position gain holds station instead.");
  declare("drive.yaw_kp_nm_s_rad", 0.15,
          "Yaw-rate proportional gain in Nm/(rad/s).");
  declare("drive.yaw_torque_limit_nm", 0.4,
          "Differential yaw torque limit in Nm.");
  declare("drive.yaw_torque_rate_nm_s", 3.0,
          "Differential torque slew in Nm/s.");
  declare("drive.yaw_torque_sign", 1.0,
          "Yaw torque polarity; exactly +1 or -1.");
  declare("drive.command_timeout_s", 0.3,
          "Mandatory Twist command timeout in seconds.");
  declare("lqr.position_nm_m", 0.8165,
          "Total wheel torque per metre of position error.");
  declare("lqr.velocity_nm_s_m", 1.7642,
          "Total wheel torque per m/s of speed error.");
  declare("lqr.pitch_nm_rad", 8.0343,
          "Total wheel torque per radian of pitch error.");
  declare("lqr.pitch_rate_nm_s_rad", 1.0413,
          "Total wheel torque per rad/s of pitch rate.");
  declare("lqr.velocity_integral_nm_s_m", 0.8,
          "Speed-error trim rate in Nm per (m/s) per second.");
  declare("lqr.velocity_integral_limit_nm", 1.5,
          "Absolute speed-error trim authority in Nm.");
  declare("lqr.engage_angle_rad", 0.15,
          "Take the wheels only once pitch is within this much of upright.");
}

Settings LqrBalancerController::read_parameters() const {
  Settings result;
  for (const auto &name : parameter_names()) {
    apply_parameter(result, get_node()->get_parameter(name));
  }
  return result;
}

controller_interface::InterfaceConfiguration
LqrBalancerController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  const auto joints = get_node()->get_parameter("joints").as_string_array();
  for (const auto &joint : joints) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

controller_interface::InterfaceConfiguration
LqrBalancerController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  const auto joints = get_node()->get_parameter("joints").as_string_array();
  const auto imu_name = get_node()->get_parameter("imu_name").as_string();
  for (const auto &joint : joints) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  for (const auto &joint : joints) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  for (const auto *name :
       {"orientation.x", "orientation.y", "orientation.z", "orientation.w",
        "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
        "linear_acceleration.x", "linear_acceleration.y",
        "linear_acceleration.z"}) {
    config.names.push_back(imu_name + "/" + name);
  }
  return config;
}

controller_interface::CallbackReturn
LqrBalancerController::on_configure(const rclcpp_lifecycle::State &) {
  const auto joints = get_node()->get_parameter("joints").as_string_array();
  if (joints.size() != joint_names_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "'joints' must contain exactly six names");
    return controller_interface::CallbackReturn::ERROR;
  }
  std::copy(joints.begin(), joints.end(), joint_names_.begin());
  imu_name_ = get_node()->get_parameter("imu_name").as_string();
  command_topic_ = get_node()->get_parameter("command_topic").as_string();
  diagnostics_rate_hz_ =
      get_node()->get_parameter("diagnostics.publish_rate_hz").as_double();
  const Settings initial = read_parameters();
  if (!valid_parameters(initial.control) ||
      !valid_lqr_parameters(initial.gains) || imu_name_.empty() ||
      command_topic_.empty() || !std::isfinite(diagnostics_rate_hz_) ||
      diagnostics_rate_hz_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid controller parameters");
    return controller_interface::CallbackReturn::ERROR;
  }
  settings_.writeFromNonRT(initial);
  command_.writeFromNonRT(StampedCommand{});

  command_subscription_ =
      get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
          command_topic_, rclcpp::SystemDefaultsQoS(),
          [this](const geometry_msgs::msg::TwistStamped::SharedPtr message) {
            command_.writeFromNonRT(StampedCommand{
                message->twist.linear.x, message->twist.angular.z,
                get_node()->now().seconds()});
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
        Settings proposed = *settings_.readFromNonRT();
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
        if (changed && (!valid_parameters(proposed.control) ||
                        !valid_lqr_parameters(proposed.gains))) {
          result.successful = false;
          result.reason =
              "Control parameter set is non-finite, unsafe, or unreachable";
          return result;
        }
        if (changed) {
          settings_.writeFromNonRT(proposed);
        }
        return result;
      });
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LqrBalancerController::on_activate(const rclcpp_lifecycle::State &) {
  if (command_interfaces_.size() != 6 || state_interfaces_.size() != 22) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected 6 command and 22 state interfaces");
    return controller_interface::CallbackReturn::ERROR;
  }
  RobotState state;
  for (std::size_t i = 0; i < 6; ++i) {
    const auto position = state_interfaces_[i].get_optional<double>();
    const auto velocity = state_interfaces_[6 + i].get_optional<double>();
    if (!position || !velocity) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Joint state interface unavailable");
      return controller_interface::CallbackReturn::ERROR;
    }
    state.position[i] = *position;
    state.velocity[i] = *velocity;
  }
  core_.reset(state, settings_.readFromRT()->control);
  command_.writeFromNonRT(StampedCommand{});
  last_diagnostics_time_s_ = 0.0;
  zero_commands();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LqrBalancerController::on_deactivate(const rclcpp_lifecycle::State &) {
  zero_commands();
  command_.writeFromNonRT(StampedCommand{});
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
LqrBalancerController::update(const rclcpp::Time &time,
                              const rclcpp::Duration &period) {
  RobotState state;
  for (std::size_t i = 0; i < 6; ++i) {
    const auto position = state_interfaces_[i].get_optional<double>();
    const auto velocity = state_interfaces_[6 + i].get_optional<double>();
    if (!position || !velocity) {
      zero_commands();
      return controller_interface::return_type::ERROR;
    }
    state.position[i] = *position;
    state.velocity[i] = *velocity;
  }
  std::array<double, 10> imu{};
  for (std::size_t i = 0; i < imu.size(); ++i) {
    const auto value = state_interfaces_[12 + i].get_optional<double>();
    if (!value) {
      zero_commands();
      return controller_interface::return_type::ERROR;
    }
    imu[i] = *value;
  }
  state.quaternion_x = imu[0];
  state.quaternion_y = imu[1];
  state.quaternion_z = imu[2];
  state.quaternion_w = imu[3];
  state.pitch_rate_rad_s = imu[5];
  state.imu_age_s = std::isfinite(state.quaternion_w) &&
                            std::isfinite(state.quaternion_x) &&
                            std::isfinite(state.quaternion_y) &&
                            std::isfinite(state.quaternion_z) &&
                            std::isfinite(state.pitch_rate_rad_s)
                        ? 0.0
                        : std::numeric_limits<double>::infinity();

  const StampedCommand received = *command_.readFromRT();
  MotionCommand target;
  target.forward_velocity_m_s = received.forward_velocity_m_s;
  target.yaw_rate_rad_s = received.yaw_rate_rad_s;
  target.age_s = received.receipt_time_s < 0.0
                     ? std::numeric_limits<double>::infinity()
                     : std::max(0.0, time.seconds() - received.receipt_time_s);
  const Settings *settings = settings_.readFromRT();
  const ControlOutput output = core_.update(
      state, target, settings->control, settings->gains, period.seconds());
  for (std::size_t i = 0; i < 6; ++i) {
    if (!command_interfaces_[i].set_value(output.effort_nm[i])) {
      zero_commands();
      return controller_interface::return_type::ERROR;
    }
  }
  publish_diagnostics(time, output, target.age_s, state.imu_age_s);
  return controller_interface::return_type::OK;
}

void LqrBalancerController::zero_commands() {
  for (auto &command : command_interfaces_) {
    (void)command.set_value(0.0);
  }
}

void LqrBalancerController::publish_diagnostics(const rclcpp::Time &time,
                                                const ControlOutput &output,
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
  status.name = get_node()->get_name() + std::string("/lqr_balancer");
  status.hardware_id = "sandro";
  status.level = output.fallen || (output.saturation_mask & SAT_IMU_STALE)
                     ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                     : (output.saturation_mask == SAT_NONE
                            ? diagnostic_msgs::msg::DiagnosticStatus::OK
                            : diagnostic_msgs::msg::DiagnosticStatus::WARN);
  status.message = saturation_text(output.saturation_mask);
  status.values.clear();
  const auto add_value = [&status](const std::string &key, const auto value) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = std::to_string(value);
    status.values.push_back(std::move(item));
  };
  add_value("requested_forward_velocity_m_s",
            output.requested_forward_velocity_m_s);
  add_value("effective_forward_velocity_m_s",
            output.effective_forward_velocity_m_s);
  add_value("measured_forward_velocity_m_s",
            output.measured_forward_velocity_m_s);
  add_value("requested_yaw_rate_rad_s", output.requested_yaw_rate_rad_s);
  add_value("effective_yaw_rate_rad_s", output.effective_yaw_rate_rad_s);
  add_value("measured_yaw_rate_rad_s", output.measured_yaw_rate_rad_s);
  add_value("position_error_m", output.position_error_m);
  add_value("velocity_error_m_s", output.velocity_error_m_s);
  add_value("velocity_integral_nm", output.velocity_integral_nm);
  add_value("pitch_rad", output.pitch_rad);
  add_value("pitch_target_rad", output.pitch_target_rad);
  add_value("raw_balance_torque_nm", output.raw_balance_torque_nm);
  add_value("balance_torque_nm", output.balance_torque_nm);
  add_value("yaw_torque_nm", output.yaw_torque_nm);
  add_value("engaged", output.engaged ? 1 : 0);
  add_value("ramp", output.ramp);
  add_value("command_age_s", command_age_s);
  add_value("imu_age_s", imu_age_s);
  add_value("saturation_mask", output.saturation_mask);
  diagnostics_publisher_->unlockAndPublish();
}

} // namespace sandro_controllers

PLUGINLIB_EXPORT_CLASS(sandro_controllers::LqrBalancerController,
                       controller_interface::ControllerInterface)
