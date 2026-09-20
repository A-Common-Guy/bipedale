#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace sandro_controllers {

class TimedEffortController : public controller_interface::ControllerInterface {
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  struct Command {
    std::vector<double> efforts;
    double receipt_time_s{-1.0};
  };

  void zero_commands();

  std::vector<std::string> joints_;
  std::string command_topic_;
  std::atomic<double> command_timeout_s_{0.1};
  std::atomic<double> effort_limit_nm_{0.5};
  realtime_tools::RealtimeBuffer<Command> command_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
      subscription_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_;
};

} // namespace sandro_controllers
