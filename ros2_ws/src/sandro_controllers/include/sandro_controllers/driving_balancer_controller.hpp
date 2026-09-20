#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/subscription.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "sandro_controllers/control_core.hpp"

namespace sandro_controllers {

class DrivingBalancerController
    : public controller_interface::ControllerInterface {
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
  struct StampedCommand {
    double forward_velocity_m_s{};
    double yaw_rate_rad_s{};
    double receipt_time_s{-1.0};
  };

  void declare_parameters();
  ControlParameters read_parameters() const;
  void publish_diagnostics(const rclcpp::Time &time,
                           const ControlOutput &output, double command_age_s,
                           double imu_age_s);
  void zero_commands();

  std::array<std::string, 6> joint_names_{};
  std::string imu_name_;
  std::string command_topic_;
  double diagnostics_rate_hz_{10.0};
  double last_diagnostics_time_s_{};
  realtime_tools::RealtimeBuffer<ControlParameters> parameters_;
  realtime_tools::RealtimeBuffer<StampedCommand> command_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
      command_subscription_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_ros_publisher_;
  std::shared_ptr<
      realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>
      diagnostics_publisher_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_;
  DrivingBalancerCore core_;
};

} // namespace sandro_controllers
