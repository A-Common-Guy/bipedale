#include "sandro_sim/latency_system.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "hardware_interface/types/hardware_component_params.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sandro_sim/drive_bus.hpp"
#include "sandro_sim/mit_drive.hpp"

namespace sandro_sim {

namespace {
constexpr std::array<const char *, 5> kMitFields{
    hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
    "kp", "kd", hardware_interface::HW_IF_EFFORT};
constexpr std::size_t kEffortField = 4;
} // namespace

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

bool LatencySystem::flag(const std::string &key, const bool fallback) const {
  const auto &parameters = get_hardware_info().hardware_parameters;
  const auto it = parameters.find(key);
  if (it == parameters.end() || it->second.empty()) {
    return fallback;
  }
  if (it->second == "true" || it->second == "1") {
    return true;
  }
  if (it->second == "false" || it->second == "0") {
    return false;
  }
  throw std::invalid_argument("hardware parameter '" + key + "' must be true or false");
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
    joint_stagger_ = flag("joint_stagger", true);
    command_delay_s_ = parameter("command_delay_s", 0.0);
    mit_torque_limit_nm_ = parameter("mit_torque_limit_nm", 10.0);

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
    // The backend is a torque source: it only gets the effort command of
    // each joint. Position, velocity, kp and kd are the emulated drive's
    // business (see write()).
    for (auto &joint : inner_params.hardware_info.joints) {
      std::vector<hardware_interface::InterfaceInfo> effort_only;
      for (const auto &command : joint.command_interfaces) {
        if (command.name == hardware_interface::HW_IF_EFFORT) {
          effort_only.push_back(command);
        }
      }
      joint.command_interfaces = effort_only;
    }
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
    const double joint_period_s = joint_rate_hz_ > 0.0 ? 1.0 / joint_rate_hz_ : 0.0;
    for (std::size_t index = 0; index < info.joints.size(); ++index) {
      const auto &joint = info.joints[index];
      // One broadcast carries all of a joint's states; free-running drives
      // broadcast at unrelated phases.
      const double phase_s =
          joint_stagger_ ? joint_period_s * static_cast<double>(index) /
                               static_cast<double>(info.joints.size())
                         : 0.0;
      for (const auto &state : joint.state_interfaces) {
        const std::string name = joint.name + "/" + state.name;
        const auto it = inner_states.find(name);
        if (it == inner_states.end()) {
          throw std::invalid_argument("inner system does not export state '" + name + "'");
        }
        StateChannel channel{name, it->second, DelayLine{}};
        channel.line.configure(joint_delay_s_, joint_rate_hz_, phase_s, true);
        states_.push_back(std::move(channel));
      }
      // A joint whose description declares any of position, velocity, kp
      // or kd is driven through the emulated MIT loop; an effort-only joint
      // is a plain delayed torque command.
      MitChannel mit;
      mit.joint = joint.name;
      bool mit_mode = false;
      for (const auto &command : joint.command_interfaces) {
        std::size_t field = kMitFields.size();
        for (std::size_t i = 0; i < kMitFields.size(); ++i) {
          if (command.name == kMitFields[i]) {
            field = i;
          }
        }
        if (field == kMitFields.size()) {
          throw std::invalid_argument(joint.name + ": unsupported command interface '" +
                                      command.name + "' (position, velocity, kp, kd, effort)");
        }
        mit.names[field] = joint.name + "/" + command.name;
        mit.present[field] = true;
        mit.lines[field].configure(command_delay_s_, 0.0);
        mit_mode = mit_mode || field != kEffortField;
      }
      if (!mit.present[kEffortField]) {
        throw std::invalid_argument(joint.name + " must declare an effort command interface");
      }
      const std::string effort_name = mit.names[kEffortField];
      const auto effort_it = inner_commands.find(effort_name);
      if (effort_it == inner_commands.end()) {
        throw std::invalid_argument("inner system does not export command '" + effort_name + "'");
      }
      if (!mit_mode) {
        CommandChannel channel{effort_name, effort_it->second, DelayLine{}};
        channel.line.configure(command_delay_s_, 0.0);
        commands_.push_back(std::move(channel));
        continue;
      }
      mit.inner_effort = effort_it->second;
      const auto position_it = inner_states.find(joint.name + "/" + hardware_interface::HW_IF_POSITION);
      const auto velocity_it = inner_states.find(joint.name + "/" + hardware_interface::HW_IF_VELOCITY);
      if (position_it == inner_states.end() || velocity_it == inner_states.end()) {
        throw std::invalid_argument(joint.name + ": the emulated drive needs the inner position and velocity states");
      }
      mit.inner_position = position_it->second;
      mit.inner_velocity = velocity_it->second;
      mit_channels_.push_back(std::move(mit));
    }
    for (const auto &sensor : info.sensors) {
      for (const auto &state : sensor.state_interfaces) {
        const std::string name = sensor.name + "/" + state.name;
        const auto it = inner_states.find(name);
        if (it == inner_states.end()) {
          throw std::invalid_argument("inner system does not export state '" + name + "'");
        }
        StateChannel channel{name, it->second, DelayLine{}};
        channel.line.configure(imu_delay_s_, imu_rate_hz_, 0.0, true);
        states_.push_back(std::move(channel));
      }
    }
    std::vector<std::string> mit_joints;
    for (const auto &channel : mit_channels_) {
      mit_joints.push_back(channel.joint);
    }
    drive_bus().configure(mit_joints, command_delay_s_, mit_torque_limit_nm_);
    RCLCPP_INFO(get_logger(),
                "wrapping '%s': %zu state interfaces, %zu torque-only and %zu "
                "MIT-mode joints; IMU %.1f ms at %.0f Hz, joints %.1f ms at "
                "%.0f Hz (%s), commands %.1f ms, emulated drive torque limit %.1f Nm",
                plugin->second.c_str(), states_.size(), commands_.size(),
                mit_channels_.size(), 1e3 * imu_delay_s_, imu_rate_hz_,
                1e3 * joint_delay_s_, joint_rate_hz_,
                joint_stagger_ ? "staggered" : "simultaneous", 1e3 * command_delay_s_,
                mit_torque_limit_nm_);
    if (!mit_channels_.empty() && !drive_bus().attached()) {
      RCLCPP_WARN(get_logger(),
                  "sandro_sim/MitDrivePlugin is not loaded: the MIT drive law runs once "
                  "per control cycle from the cycle's state instead of on every physics "
                  "step (load sandro_sim/config/mujoco_plugins.yaml into ros2_control_node)");
    }
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
  for (auto &channel : mit_channels_) {
    for (auto &line : channel.lines) {
      line.reset();
    }
  }
  drive_bus().clear();
  const auto result = inner_->on_activate(previous);
  if (result == CallbackReturn::SUCCESS) {
    mirror_inner_state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  }
  return result;
}

CallbackReturn LatencySystem::on_deactivate(const rclcpp_lifecycle::State &previous) {
  drive_bus().clear();
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
  const double nan = std::numeric_limits<double>::quiet_NaN();
  auto &bus = drive_bus();
  for (std::size_t joint = 0; joint < mit_channels_.size(); ++joint) {
    auto &channel = mit_channels_[joint];
    // The frame the drive receives, unset fields NaN. The physics-side
    // plugin delays it by the command latency and runs the drive law on
    // every step; the torque computed below from the same frame delayed
    // here is what the backend applies when that plugin is absent.
    std::array<double, 5> raw{};
    std::array<double, 5> fields{};
    for (std::size_t i = 0; i < fields.size(); ++i) {
      raw[i] = channel.present[i] ? get_command<double>(channel.names[i]) : nan;
      fields[i] = channel.present[i] ? channel.lines[i].update(now, raw[i]) : nan;
    }
    bus.push(joint, now, MitCommand{raw[0], raw[1], raw[2], raw[3], raw[4]});
    const auto position = channel.inner_position->get_optional<double>();
    const auto velocity = channel.inner_velocity->get_optional<double>();
    if (!position || !velocity) {
      return return_type::ERROR;
    }
    const MitCommand command{fields[0], fields[1], fields[2], fields[3], fields[4]};
    if (!channel.inner_effort->set_value(
            mit_torque(command, *position, *velocity, mit_torque_limit_nm_))) {
      return return_type::ERROR;
    }
  }
  return inner_->write(time, period);
}

} // namespace sandro_sim

PLUGINLIB_EXPORT_CLASS(sandro_sim::LatencySystem, hardware_interface::SystemInterface)
