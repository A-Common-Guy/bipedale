#pragma once

#include <atomic>
#include <memory>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/subscription.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "sandro_controllers/rl_policy_core.hpp"

namespace sandro_controllers {

// Runs a locomotion policy trained in sandro_rl. It claims the same MIT
// frames, joint states and IMU states as the whole-body controller and
// listens to the same geometry_msgs/TwistStamped: linear.x forward speed,
// angular.z yaw rate, linear.z body height rate, and angular.x a body roll
// target for policies trained with one. Activate exactly one balancer.
//
// One instance per policy; sandro_bringup/config/rl_policies.yaml is
// generated from each policy's policy.json by sandro_rl's export_ros.py.
class RlPolicyController : public controller_interface::ControllerInterface {
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
    double forward_m_s{};
    double yaw_rate_rad_s{};
    double height_rate_m_s{};
    double roll_rad{};
    double receipt_time_s{-1.0};
  };

  void declare_parameters();
  bool read_settings(RlPolicySettings &settings, std::string *error) const;
  bool read_state(RlPolicyState &state) const;
  void publish_diagnostics(const rclcpp::Time &time, const RlPolicyOutput &output,
                           double command_age_s);
  void report(const RlPolicyState &state, const RlPolicyOutput &output);
  void zero_commands();

  double diagnostics_rate_hz_{50.0};
  double last_diagnostics_time_s_{};
  double inference_max_s_{};
  double inference_last_s_{};
  RlPhase last_phase_{RlPhase::kStand};
  std::uint32_t last_reported_status_{};
  realtime_tools::RealtimeBuffer<StampedCommand> command_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_subscription_;
  // ~/arm_handover: the next activation within kHandoverWindowS takes over
  // from the running balancer without the stand phase (the gamepad switch).
  static constexpr double kHandoverWindowS = 1.0;
  std::atomic<double> handover_armed_s_{-1e9};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handover_service_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_ros_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>
      diagnostics_publisher_;
  RlPolicyCore core_;
};

} // namespace sandro_controllers
