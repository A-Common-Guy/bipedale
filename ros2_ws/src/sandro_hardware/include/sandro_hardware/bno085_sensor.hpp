#pragma once

#include <hardware_interface/sensor_interface.hpp>
#include <memory>
#include <vector>

namespace sandro_hardware
{
class BNO085Sensor final : public hardware_interface::SensorInterface
{
public:
  ~BNO085Sensor() override;
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
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

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace sandro_hardware
