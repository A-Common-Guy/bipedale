#include "sandro_sim/latency_system.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "hardware_interface/types/hardware_component_params.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace sandro_sim {

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

LatencySystem::~LatencySystem() {
  // The inner instance must be destroyed before the loader that created it.
  states_.clear();
  commands_.clear();
  inner_.reset();
  loader_.reset();
}

double LatencySystem::parameter(const std::string &key,
                                const double fallback) const {
  const auto &parameters = get_hardware_info().hardware_parameters;
  const auto it = parameters.find(key);
  if (it == parameters.end() || it->second.empty()) {
    return fallback;
  }
  std::size_t used = 0;
  const double value = std::stod(it->second, &used);
  if (used != it->second.size() || !std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument("hardware parameter '" + key +
                                "' must be a finite non-negative number");
  }
  return value;
}

CallbackReturn LatencySystem::on_init(
    const hardware_interface::HardwareComponentInterfaceParams &params) {
  if (hardware_interface::SystemInterface::on_init(params) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  try {
    const auto &parameters = get_hardware_info().hardware_parameters;
    const auto plugin = parameters.find("inner_plugin");
    if (plugin == parameters.end() || plugin->second.empty()) {
      throw std::invalid_argument("hardware parameter 'inner_plugin' is required");
    }
    imu_delay_s_ = parameter("imu_delay_s", 0.0);
    imu_rate_hz_ = parameter("imu_rate_hz", 0.0);
    joint_delay_s_ = parameter("joint_delay_s", 0.0);
    joint_rate_hz_ = parameter("joint_rate_hz", 0.0);
    command_delay_s_ = parameter("command_delay_s", 0.0);

    loader_ = std::make_unique<
        pluginlib::ClassLoader<hardware_interface::SystemInterface>>(
        "hardware_interface", "hardware_interface::SystemInterface");
    inner_ = loader_->createUniqueInstance(plugin->second);

    // The inner component gets the same description under a distinct name,
    // so its framework node cannot clash with ours, and is told which plugin
    // it is. Parameters it does not know (ours) are ignored by it.
    hardware_interface::HardwareComponentParams inner_params;
    inner_params.hardware_info = get_hardware_info();
    inner_params.hardware_info.name += "_inner";
    inner_params.hardware_info.hardware_plugin_name = plugin->second;
    inner_params.logger = get_logger().get_child("inner");
    inner_params.clock = get_clock();
    inner_params.executor = params.executor;
    if (inner_->init(inner_params) != CallbackReturn::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "inner system '%s' failed to initialise",
                   plugin->second.c_str());
      return CallbackReturn::ERROR;
    }

    // Our interfaces are created by the framework from the description; the
    // inner one's are private handles we read and write ourselves. A component
    // written against the legacy API exports handles that point into its own
    // members (the MuJoCo backend does); the framework-created ones would
    // never be written by it, so prefer the legacy export when it is non-empty.
    std::unordered_map<std::string,
                       hardware_interface::StateInterface::ConstSharedPtr>
        inner_states;
    std::unordered_map<std::string,
                       hardware_interface::CommandInterface::SharedPtr>
        inner_commands;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    auto legacy_states = inner_->export_state_interfaces();
    auto legacy_commands = inner_->export_command_interfaces();
#pragma GCC diagnostic pop
    if (!legacy_states.empty()) {
      for (auto &interface : legacy_states) {
        auto handle = std::make_shared<hardware_interface::StateInterface>(
            std::move(interface));
        inner_states[handle->get_name()] = handle;
      }
    } else {
      for (const auto &handle : inner_->on_export_state_interfaces()) {
        inner_states[handle->get_name()] = handle;
      }
    }
    if (!legacy_commands.empty()) {
      for (auto &interface : legacy_commands) {
        auto handle = std::make_shared<hardware_interface::CommandInterface>(
            std::move(interface));
        inner_commands[handle->get_name()] = handle;
      }
    } else {
      for (const auto &handle : inner_->on_export_command_interfaces()) {
        inner_commands[handle->get_name()] = handle;
      }
    }
    const auto &info = get_hardware_info();
    for (const auto &joint : info.joints) {
      for (const auto &state : joint.state_interfaces) {
        const std::string name = joint.name + "/" + state.name;
        const auto it = inner_states.find(name);
        if (it == inner_states.end()) {
          throw std::invalid_argument("inner system does not export state '" + name + "'");
        }
        StateChannel channel{name, it->second, DelayLine{}};
        channel.line.configure(joint_delay_s_, joint_rate_hz_);
        states_.push_back(std::move(channel));
      }
      for (const auto &command : joint.command_interfaces) {
        const std::string name = joint.name + "/" + command.name;
        const auto it = inner_commands.find(name);
        if (it == inner_commands.end()) {
          throw std::invalid_argument("inner system does not export command '" + name + "'");
        }
        CommandChannel channel{name, it->second, DelayLine{}};
        channel.line.configure(command_delay_s_, 0.0);
        commands_.push_back(std::move(channel));
      }
    }
    for (const auto &sensor : info.sensors) {
      for (const auto &state : sensor.state_interfaces) {
        const std::string name = sensor.name + "/" + state.name;
        const auto it = inner_states.find(name);
        if (it == inner_states.end()) {
          throw std::invalid_argument("inner system does not export state '" + name + "'");
        }
        StateChannel channel{name, it->second, DelayLine{}};
        channel.line.configure(imu_delay_s_, imu_rate_hz_);
        states_.push_back(std::move(channel));
      }
    }
    RCLCPP_INFO(get_logger(),
                "wrapping '%s': %zu state and %zu command interfaces; IMU %.1f ms "
                "at %.0f Hz, joints %.1f ms at %.0f Hz, commands %.1f ms",
                plugin->second.c_str(), states_.size(), commands_.size(),
                1e3 * imu_delay_s_, imu_rate_hz_, 1e3 * joint_delay_s_,
                joint_rate_hz_, 1e3 * command_delay_s_);
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_logger(), "configuration invalid: %s", error.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

void LatencySystem::mirror_inner_state(const std::uint8_t primary_state,
                                       const char *label) {
  inner_->set_lifecycle_state(rclcpp_lifecycle::State(primary_state, label));
}

CallbackReturn LatencySystem::on_configure(const rclcpp_lifecycle::State &previous) {
  const auto result = inner_->on_configure(previous);
  if (result == CallbackReturn::SUCCESS) {
    mirror_inner_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  }
  return result;
}

CallbackReturn LatencySystem::on_cleanup(const rclcpp_lifecycle::State &previous) {
  const auto result = inner_->on_cleanup(previous);
  if (result == CallbackReturn::SUCCESS) {
    mirror_inner_state(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                       "unconfigured");
  }
  return result;
}

CallbackReturn LatencySystem::on_activate(const rclcpp_lifecycle::State &previous) {
  for (auto &channel : states_) {
    channel.line.reset();
  }
  for (auto &channel : commands_) {
    channel.line.reset();
  }
  const auto result = inner_->on_activate(previous);
  if (result == CallbackReturn::SUCCESS) {
    mirror_inner_state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  }
  return result;
}

CallbackReturn LatencySystem::on_deactivate(const rclcpp_lifecycle::State &previous) {
  const auto result = inner_->on_deactivate(previous);
  if (result == CallbackReturn::SUCCESS) {
    mirror_inner_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  }
  return result;
}

CallbackReturn LatencySystem::on_shutdown(const rclcpp_lifecycle::State &previous) {
  const auto result = inner_->on_shutdown(previous);
  mirror_inner_state(lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED, "finalized");
  return result;
}

CallbackReturn LatencySystem::on_error(const rclcpp_lifecycle::State &previous) {
  return inner_->on_error(previous);
}

return_type LatencySystem::read(const rclcpp::Time &time,
                                const rclcpp::Duration &period) {
  const auto result = inner_->read(time, period);
  if (result != return_type::OK) {
    return result;
  }
  const double now = time.seconds();
  for (auto &channel : states_) {
    const auto value = channel.inner->get_optional<double>();
    if (!value) {
      continue;
    }
    set_state(channel.name, channel.line.update(now, *value));
  }
  return return_type::OK;
}

return_type LatencySystem::write(const rclcpp::Time &time,
                                 const rclcpp::Duration &period) {
  const double now = time.seconds();
  for (auto &channel : commands_) {
    const double value = get_command<double>(channel.name);
    if (!channel.inner->set_value(channel.line.update(now, value))) {
      return return_type::ERROR;
    }
  }
  return inner_->write(time, period);
}

} // namespace sandro_sim

PLUGINLIB_EXPORT_CLASS(sandro_sim::LatencySystem, hardware_interface::SystemInterface)
