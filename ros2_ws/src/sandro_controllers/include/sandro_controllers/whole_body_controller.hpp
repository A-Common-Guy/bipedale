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

#include "sandro_controllers/whole_body_core.hpp"

namespace sandro_controllers {

// Whole-body controller plugin. It claims the same six effort commands, joint
// states and IMU states as the other balancers and listens to the same
// geometry_msgs/TwistStamped: linear.x forward speed, angular.z yaw rate, and
// additionally linear.z as a body height rate. Activate exactly one balancer.
//
// All tunables are live ROS parameters. A change is validated as a complete
// set and the height-scheduled LQR gains are re-solved off the realtime path
// before the new settings are swapped in.
class WholeBodyController : public controller_interface::ControllerInterface {
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
    double height_rate_m_s{};
    double receipt_time_s{-1.0};
  };

  void declare_parameters();
  WholeBodyParameters read_parameters() const;
  bool read_state(WholeBodyState &state) const;
  void publish_diagnostics(const rclcpp::Time &time,
                           const WholeBodyOutput &output, double command_age_s,
                           double imu_age_s);
  void zero_commands();

  std::array<std::string, 6> joint_names_{};
  std::string imu_name_;
  std::string command_topic_;
  double diagnostics_rate_hz_{10.0};
  double last_diagnostics_time_s_{};
  double control_period_s_{1.0 / 300.0};
  // Non-realtime copy of the accepted parameter set.
  WholeBodyParameters accepted_parameters_{};
  realtime_tools::RealtimeBuffer<WholeBodySettings> settings_;
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
  WholeBodyCore core_;
};

} // namespace sandro_controllers
