#pragma once

#include <hardware_interface/system_interface.hpp>
#include <memory>
#include <string>
#include <vector>

namespace sandro_hardware
{
class GIM6010System final : public hardware_interface::SystemInterface
{
public:
  ~GIM6010System() override;
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State &) override;
  hardware_interface::return_type read(
    const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(
    const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace sandro_hardware
