#pragma once

// Decorator hardware interface: owns the interfaces the controllers see and
// drives an inner system (the MuJoCo backend) privately, emulating the
// GIM6010 drives between the two. Everything the inner component does
// (physics, its own node, services, plugins) is untouched.
//
//   controllers <-> [LatencySystem: delay lines + MIT drive law] <-> MujocoSystemInterface
//
// Two things the physical path has that the bare backend does not:
// - latency and sample rates of the sensor and command paths (delay lines on
//   every state and command value crossing the controller boundary);
// - the drive-side MIT loop: a joint's position/velocity/kp/kd/effort
//   commands are delayed like the CAN frame, then the torque
//   kp (q_ref - q) + kd (v_ref - v) + effort is computed from the inner
//   system's undelayed joint state, as the drive does from its own encoder,
//   and only that torque reaches the backend's effort interface. The inner
//   system therefore only ever sees an effort command per joint.
//
// Hardware parameters (all optional, seconds and hertz; 0 = pass-through):
//   inner_plugin        pluginlib name of the wrapped system (required)
//   imu_delay_s         latency of every sensor state interface
//   imu_rate_hz         refresh rate of the sensor state interfaces
//   joint_delay_s       latency of every joint state interface
//   joint_rate_hz       refresh rate of the joint state interfaces
//   command_delay_s     latency of every command interface
//   mit_torque_limit_nm clip of the emulated drive's torque (default 10)

#include <array>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_loader.hpp"

#include "sandro_sim/delay_line.hpp"

namespace sandro_sim {

class LatencySystem final : public hardware_interface::SystemInterface {
public:
  ~LatencySystem() override;

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareComponentInterfaceParams &params) override;
  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::CallbackReturn
  on_error(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::return_type
  read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
  hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  struct StateChannel {
    std::string name;
    hardware_interface::StateInterface::ConstSharedPtr inner;
    DelayLine line;
  };
  struct CommandChannel {
    std::string name;
    hardware_interface::CommandInterface::SharedPtr inner;
    DelayLine line;
  };
  // One joint's MIT command frame: the five fields the controller writes
  // (each with its own delay line; missing ones stay NaN = unset), the inner
  // effort command the computed torque goes to, and the inner undelayed
  // position and velocity the drive law reads.
  struct MitChannel {
    std::string joint;
    std::array<std::string, 5> names{};  // position, velocity, kp, kd, effort
    std::array<bool, 5> present{};
    std::array<DelayLine, 5> lines{};
    hardware_interface::CommandInterface::SharedPtr inner_effort;
    hardware_interface::StateInterface::ConstSharedPtr inner_position;
    hardware_interface::StateInterface::ConstSharedPtr inner_velocity;
  };

  double parameter(const std::string &key, double fallback) const;
  void mirror_inner_state(std::uint8_t primary_state, const char *label);

  std::unique_ptr<pluginlib::ClassLoader<hardware_interface::SystemInterface>>
      loader_;
  pluginlib::UniquePtr<hardware_interface::SystemInterface> inner_;
  std::vector<StateChannel> states_;
  std::vector<CommandChannel> commands_;
  std::vector<MitChannel> mit_channels_;
  double mit_torque_limit_nm_{10.0};
  double imu_delay_s_{};
  double imu_rate_hz_{};
  double joint_delay_s_{};
  double joint_rate_hz_{};
  double command_delay_s_{};
};

} // namespace sandro_sim
