#include "sandro_controllers/timed_effort_controller.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace sandro_controllers {

controller_interface::CallbackReturn TimedEffortController::on_init() {
  try {
    rcl_interfaces::msg::ParameterDescriptor joints_descriptor;
    joints_descriptor.description =
        "Joints receiving effort commands, in Float64MultiArray order.";
    joints_descriptor.read_only = true;
    get_node()->declare_parameter("joints", std::vector<std::string>{},
                                  joints_descriptor);
    rcl_interfaces::msg::ParameterDescriptor topic_descriptor;
    topic_descriptor.description = "Float64MultiArray effort command topic.";
    topic_descriptor.read_only = true;
    get_node()->declare_parameter("command_topic", "~/commands",
                                  topic_descriptor);
    rcl_interfaces::msg::ParameterDescriptor timeout_descriptor;
    timeout_descriptor.description =
        "Mandatory maximum command age in seconds; stale commands become zero.";
    get_node()->declare_parameter("command_timeout_s", 0.1, timeout_descriptor);
    rcl_interfaces::msg::ParameterDescriptor limit_descriptor;
    limit_descriptor.description = "Absolute commissioning effort boundary in "
                                   "Nm; commands are visibly clamped.";
    get_node()->declare_parameter("effort_limit_nm", 0.5, limit_descriptor);
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter declaration failed: %s",
                 error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
TimedEffortController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  const auto joints = get_node()->get_parameter("joints").as_string_array();
  for (const auto &joint : joints) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

controller_interface::InterfaceConfiguration
TimedEffortController::state_interface_configuration() const {
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::CallbackReturn
TimedEffortController::on_configure(const rclcpp_lifecycle::State &) {
  joints_ = get_node()->get_parameter("joints").as_string_array();
  command_topic_ = get_node()->get_parameter("command_topic").as_string();
  const double command_timeout_s =
      get_node()->get_parameter("command_timeout_s").as_double();
  const double effort_limit_nm =
      get_node()->get_parameter("effort_limit_nm").as_double();
  if (joints_.empty() || command_topic_.empty() ||
      !std::isfinite(command_timeout_s) || command_timeout_s <= 0.0 ||
      !std::isfinite(effort_limit_nm) || effort_limit_nm <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "joints/topic must be nonempty; timeout and effort_limit_nm "
                 "must be > 0");
    return controller_interface::CallbackReturn::ERROR;
  }
  command_timeout_s_.store(command_timeout_s);
  effort_limit_nm_.store(effort_limit_nm);
  command_.writeFromNonRT(
      Command{std::vector<double>(joints_.size(), 0.0), -1.0});
  subscription_ =
      get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
          command_topic_, rclcpp::SystemDefaultsQoS(),
          [this](const std_msgs::msg::Float64MultiArray::SharedPtr message) {
            if (message->data.size() != joints_.size()) {
              RCLCPP_WARN_THROTTLE(
                  get_node()->get_logger(), *get_node()->get_clock(), 2000,
                  "Ignoring effort command with %zu values; expected %zu",
                  message->data.size(), joints_.size());
              return;
            }
            if (!std::all_of(
                    message->data.begin(), message->data.end(),
                    [](const double value) { return std::isfinite(value); })) {
              RCLCPP_WARN_THROTTLE(get_node()->get_logger(),
                                   *get_node()->get_clock(), 2000,
                                   "Ignoring non-finite effort command");
              return;
            }
            command_.writeFromNonRT(
                Command{message->data, get_node()->now().seconds()});
          });
  parameter_callback_ = get_node()->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &changes) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        double timeout = command_timeout_s_.load();
        double limit = effort_limit_nm_.load();
        try {
          for (const auto &change : changes) {
            if (change.get_name() == "command_timeout_s") {
              timeout = change.as_double();
            } else if (change.get_name() == "effort_limit_nm") {
              limit = change.as_double();
            }
          }
        } catch (const std::exception &error) {
          result.successful = false;
          result.reason = error.what();
          return result;
        }
        if (!std::isfinite(timeout) || timeout <= 0.0 ||
            !std::isfinite(limit) || limit <= 0.0) {
          result.successful = false;
          result.reason =
              "command_timeout_s and effort_limit_nm must be finite and > 0";
          return result;
        }
        command_timeout_s_.store(timeout);
        effort_limit_nm_.store(limit);
        return result;
      });
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
TimedEffortController::on_activate(const rclcpp_lifecycle::State &) {
  command_.writeFromNonRT(
      Command{std::vector<double>(joints_.size(), 0.0), -1.0});
  zero_commands();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
TimedEffortController::on_deactivate(const rclcpp_lifecycle::State &) {
  zero_commands();
  command_.writeFromNonRT(
      Command{std::vector<double>(joints_.size(), 0.0), -1.0});
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
TimedEffortController::update(const rclcpp::Time &time,
                              const rclcpp::Duration &) {
  const Command *command = command_.readFromRT();
  const bool fresh =
      command->receipt_time_s >= 0.0 &&
      time.seconds() - command->receipt_time_s <= command_timeout_s_.load();
  const double effort_limit_nm = effort_limit_nm_.load();
  for (std::size_t i = 0; i < command_interfaces_.size(); ++i) {
    const double requested = fresh ? command->efforts[i] : 0.0;
    const double applied =
        std::clamp(requested, -effort_limit_nm, effort_limit_nm);
    if (!command_interfaces_[i].set_value(applied)) {
      zero_commands();
      return controller_interface::return_type::ERROR;
    }
  }
  return controller_interface::return_type::OK;
}

void TimedEffortController::zero_commands() {
  for (auto &command : command_interfaces_) {
    (void)command.set_value(0.0);
  }
}

} // namespace sandro_controllers

PLUGINLIB_EXPORT_CLASS(sandro_controllers::TimedEffortController,
                       controller_interface::ControllerInterface)
