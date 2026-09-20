#include "sandro_hardware/gim6010_system.hpp"

#include "sandro_hardware/protocol.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <mutex>
#include <net/if.h>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <thread>
#include <type_traits>
#include <unistd.h>
#include <unordered_map>

namespace sandro_hardware
{
namespace
{
using CallbackReturn = hardware_interface::CallbackReturn;
using protocol::CanCommand;

template<typename T>
T required_number(const std::unordered_map<std::string, std::string> & parameters, const std::string & key)
{
  const auto it = parameters.find(key);
  if (it == parameters.end()) {
    throw std::invalid_argument("missing required parameter '" + key + "'");
  }
  std::size_t used = 0;
  if constexpr (std::is_integral_v<T>) {
    const auto value = std::stoll(it->second, &used, 0);
    if (used != it->second.size() ||
      value < static_cast<long long>(std::numeric_limits<T>::lowest()) ||
      static_cast<unsigned long long>(value) >
      static_cast<unsigned long long>(std::numeric_limits<T>::max()))
    {
      throw std::invalid_argument("parameter '" + key + "' is outside its integer range");
    }
    return static_cast<T>(value);
  } else {
    const auto value = std::stod(it->second, &used);
    if (used != it->second.size() || !std::isfinite(value)) {
      throw std::invalid_argument("parameter '" + key + "' is not finite");
    }
    return static_cast<T>(value);
  }
}

std::string required_string(
  const std::unordered_map<std::string, std::string> & parameters, const std::string & key)
{
  const auto it = parameters.find(key);
  if (it == parameters.end() || it->second.empty()) {
    throw std::invalid_argument("missing required parameter '" + key + "'");
  }
  return it->second;
}

void put_u32(can_frame & frame, std::size_t at, uint32_t value)
{
  std::memcpy(frame.data + at, &value, sizeof(value));
}

}  // namespace

struct GIM6010System::Impl
{
  struct Joint
  {
    std::string name;
    uint8_t node{};
    double sign{};
    double ratio{};
    double offset{};
    protocol::MitLimits limits;
    double position{};
    double velocity{};
    double raw_position_rev{};
    double raw_velocity_rev_s{};
    double effort{};
    // MIT command fields. Only effort is mandatory; the others are exported
    // when the description declares them and read as "unset" while NaN, so
    // a controller that claims effort alone runs the drive as a torque
    // source exactly as before.
    double command{};
    double command_position{std::numeric_limits<double>::quiet_NaN()};
    double command_velocity{std::numeric_limits<double>::quiet_NaN()};
    double command_kp{std::numeric_limits<double>::quiet_NaN()};
    double command_kd{std::numeric_limits<double>::quiet_NaN()};
    bool has_position_command{false};
    bool has_velocity_command{false};
    bool has_kp_command{false};
    bool has_kd_command{false};
    // Position the drive reports in its MIT reply (its own frame, output
    // radians) and the bias of that frame against the encoder broadcast,
    // measured at activation with zero gains and added to every position
    // reference so the drive-side PD acts on the logical reference.
    double mit_reply_position{std::numeric_limits<double>::quiet_NaN()};
    std::chrono::steady_clock::time_point mit_reply_stamp{};
    double mit_frame_bias{};
    double age{std::numeric_limits<double>::infinity()};
    double online{};
    double iq{};
    double temperature{};
    double error{};
    double requested_effort{};
    double applied_effort{};
    double saturation_count{};
    uint32_t axis_error{};
    uint8_t fault_flags{};
    uint8_t axis_state{};
    std::chrono::steady_clock::time_point encoder_stamp{};
    std::chrono::steady_clock::time_point heartbeat_stamp{};
  };

  std::vector<Joint> joints;
  std::unordered_map<uint8_t, std::size_t> by_node;
  std::string interface;
  int encoder_period_ms{};
  // Largest MIT-frame bias accepted at activation; beyond it arming fails.
  double mit_frame_bias_tolerance_rad{0.2};
  rclcpp::Clock steady_clock{RCL_STEADY_TIME};
  bool fault_reported{false};
  int iq_period_ms{};
  int heartbeat_period_ms{};
  int deadman_ms{};
  double stale_state_s{};
  double heartbeat_stale_s{};
  double activation_timeout_s{};
  uint16_t encoder_rate_endpoint{};
  uint16_t iq_rate_endpoint{};
  uint16_t heartbeat_rate_endpoint{};
  uint16_t deadman_endpoint{};
  uint16_t mit_max_position_endpoint{};
  uint16_t mit_max_velocity_endpoint{};
  uint16_t mit_max_torque_endpoint{};
  uint16_t mit_max_kp_endpoint{};
  uint16_t mit_max_kd_endpoint{};
  double hard_effort_limit_nm{};
  // "encoder": position and velocity from the drives' periodic 0x09
  // broadcast (encoder_period_ms). "mit": velocity, and position while it is
  // inside the MIT range, from the reply each drive sends to every MIT
  // command, i.e. synchronous with the control cycle and one cycle old at
  // most. The broadcast then only keeps the wheels' multi-turn position.
  bool mit_state{false};
  int socket{-1};
  std::string interface_name;
  std::atomic_bool stop{false};
  std::atomic_bool active{false};
  // Every CAN frame seen on the socket, ours or not: zero after a second of
  // listening means nobody is transmitting, i.e. the drives are off or the
  // cable is out, as opposed to one drive missing.
  std::atomic<unsigned long> frames_received{0};
  std::thread receiver;
  std::mutex mutex;
  std::condition_variable reply_cv;
  std::vector<uint8_t> sdo_reply;
  uint16_t awaited_endpoint{std::numeric_limits<uint16_t>::max()};

  ~Impl()
  {
    // Best effort if the component is torn down while still active (a
    // shutdown that skipped deactivation): zero torque and idle every drive
    // before the socket goes; the CAN deadman covers whatever this misses.
    if (active && socket >= 0) {
      active = false;
      for (auto & joint : joints) {
        try {
          mit(joint, 0.0);
          send_u32(joint.node, CanCommand::set_axis_state, 1);
        } catch (...) {
        }
      }
    }
    close_socket();
  }

  void send(uint8_t node, CanCommand command, const uint8_t * data, uint8_t length)
  {
    can_frame frame{};
    frame.can_id = protocol::can_id(node, command);
    frame.can_dlc = length;
    if (length) {
      std::memcpy(frame.data, data, length);
    }
    if (::write(socket, &frame, sizeof(frame)) != sizeof(frame)) {
      throw std::runtime_error("SocketCAN write failed: " + std::string(std::strerror(errno)));
    }
  }

  void send_u32(uint8_t node, CanCommand command, uint32_t value)
  {
    can_frame frame{};
    frame.can_id = protocol::can_id(node, command);
    frame.can_dlc = 4;
    put_u32(frame, 0, value);
    if (::write(socket, &frame, sizeof(frame)) != sizeof(frame)) {
      throw std::runtime_error("SocketCAN write failed: " + std::string(std::strerror(errno)));
    }
  }

  void open_socket()
  {
    socket = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket < 0) {
      throw std::runtime_error("cannot create SocketCAN socket: " + std::string(std::strerror(errno)));
    }
    ifreq request{};
    std::strncpy(request.ifr_name, interface.c_str(), IFNAMSIZ - 1);
    interface_name = interface;
    if (::ioctl(socket, SIOCGIFINDEX, &request) < 0) {
      const auto message = std::string(std::strerror(errno));
      close_socket();
      throw std::runtime_error("unknown CAN interface '" + interface + "': " + message);
    }
    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = request.ifr_ifindex;
    if (::bind(socket, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
      const auto message = std::string(std::strerror(errno));
      close_socket();
      throw std::runtime_error("cannot bind '" + interface + "': " + message);
    }
    stop = false;
    receiver = std::thread([this] {receive_loop();});
  }

  void close_socket()
  {
    active = false;
    stop = true;
    if (receiver.joinable()) {
      receiver.join();
    }
    if (socket >= 0) {
      ::close(socket);
      socket = -1;
    }
  }

  void receive_loop()
  {
    pollfd descriptor{socket, POLLIN, 0};
    while (!stop) {
      const auto ready = ::poll(&descriptor, 1, 100);
      if (ready <= 0) {
        continue;
      }
      can_frame frame{};
      if (::read(socket, &frame, sizeof(frame)) != sizeof(frame) ||
        (frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)))
      {
        continue;
      }
      ++frames_received;
      const auto node = static_cast<uint8_t>((frame.can_id >> 5) & 0x3f);
      const auto command = static_cast<uint8_t>(frame.can_id & 0x1f);
      const auto found = by_node.find(node);
      if (found == by_node.end()) {
        continue;
      }
      const auto now = std::chrono::steady_clock::now();
      std::lock_guard<std::mutex> lock(mutex);
      auto & joint = joints[found->second];
      if (command == static_cast<uint8_t>(CanCommand::heartbeat) && frame.can_dlc >= 8) {
        std::memcpy(&joint.axis_error, frame.data, 4);
        joint.axis_state = frame.data[4];
        joint.fault_flags = frame.data[5] & 0x0f;
        joint.temperature = static_cast<int8_t>(frame.data[6]);
        joint.error = joint.axis_error || joint.fault_flags ? 1.0 : 0.0;
        joint.heartbeat_stamp = now;
      } else if (command == static_cast<uint8_t>(CanCommand::encoder) && frame.can_dlc >= 8) {
        float position = 0.0F, velocity = 0.0F;
        std::memcpy(&position, frame.data, 4);
        std::memcpy(&velocity, frame.data + 4, 4);
        joint.raw_position_rev = position;
        joint.raw_velocity_rev_s = velocity;
        joint.position = protocol::logical_position(position, joint.sign, joint.ratio, joint.offset);
        joint.velocity = protocol::logical_velocity(velocity, joint.sign, joint.ratio);
        joint.encoder_stamp = now;
      } else if (command == static_cast<uint8_t>(CanCommand::iq) && frame.can_dlc >= 8) {
        float measured = 0.0F;
        std::memcpy(&measured, frame.data + 4, 4);
        joint.iq = measured;
      } else if (command == static_cast<uint8_t>(CanCommand::mit_control) && frame.can_dlc >= 6) {
        const auto [reply_node, position, velocity, torque] =
          protocol::unpack_mit(frame.data, frame.can_dlc, joint.limits);
        (void)reply_node;
        joint.effort = joint.sign * torque;
        joint.mit_reply_position = position;
        joint.mit_reply_stamp = now;
        if (mit_state) {
          // Output-shaft radians and rad/s straight from the reply. The
          // position field saturates at +-mit_max_position, so a wheel that
          // has turned further keeps the broadcast's multi-turn position.
          joint.velocity = joint.sign * velocity;
          if (std::abs(position) < 0.9 * joint.limits.position) {
            joint.position = joint.sign * position;
          }
          joint.encoder_stamp = now;
        }
      } else if (command == static_cast<uint8_t>(CanCommand::tx_sdo) && frame.can_dlc >= 4) {
        uint16_t endpoint{};
        std::memcpy(&endpoint, frame.data + 1, 2);
        if (endpoint == awaited_endpoint) {
          sdo_reply.assign(frame.data, frame.data + frame.can_dlc);
          reply_cv.notify_all();
        }
      }
    }
  }

  void sdo_write_verified(uint8_t node, uint16_t endpoint, uint32_t value)
  {
    uint8_t request[8]{1, static_cast<uint8_t>(endpoint), static_cast<uint8_t>(endpoint >> 8), 0};
    std::memcpy(request + 4, &value, 4);
    send(node, CanCommand::rx_sdo, request, 8);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    request[0] = 0;
    std::memset(request + 4, 0, 4);
    std::unique_lock<std::mutex> lock(mutex);
    awaited_endpoint = endpoint;
    sdo_reply.clear();
    lock.unlock();
    send(node, CanCommand::rx_sdo, request, 8);
    lock.lock();
    const auto received = reply_cv.wait_for(
      lock, std::chrono::milliseconds(500), [this] {return !sdo_reply.empty();});
    awaited_endpoint = std::numeric_limits<uint16_t>::max();
    if (!received || sdo_reply.size() < 8) {
      throw std::runtime_error("node " + std::to_string(node) + ": SDO verification timed out");
    }
    uint32_t actual{};
    std::memcpy(&actual, sdo_reply.data() + 4, 4);
    if (actual != value) {
      throw std::runtime_error(
              "node " + std::to_string(node) + ": SDO verification mismatch at endpoint " +
              std::to_string(endpoint));
    }
  }

  std::vector<uint8_t> sdo_read(uint8_t node, uint16_t endpoint)
  {
    uint8_t request[8]{0, static_cast<uint8_t>(endpoint), static_cast<uint8_t>(endpoint >> 8), 0};
    std::unique_lock<std::mutex> lock(mutex);
    awaited_endpoint = endpoint;
    sdo_reply.clear();
    lock.unlock();
    send(node, CanCommand::rx_sdo, request, 8);
    lock.lock();
    const auto received = reply_cv.wait_for(
      lock, std::chrono::milliseconds(500), [this] {return !sdo_reply.empty();});
    awaited_endpoint = std::numeric_limits<uint16_t>::max();
    if (!received || sdo_reply.size() < 8) {
      throw std::runtime_error(
              "node " + std::to_string(node) + ": SDO read timed out at endpoint " +
              std::to_string(endpoint));
    }
    return sdo_reply;
  }

  float sdo_read_float(uint8_t node, uint16_t endpoint)
  {
    const auto reply = sdo_read(node, endpoint);
    float value{};
    std::memcpy(&value, reply.data() + 4, sizeof(value));
    if (!std::isfinite(value) || value <= 0.0F) {
      throw std::runtime_error(
              "node " + std::to_string(node) + ": invalid positive float at endpoint " +
              std::to_string(endpoint));
    }
    return value;
  }

  // One MIT frame: the drive computes kp (q_ref - q) + kd (v_ref - v) + tau
  // in its own loop, synchronous and free of bus latency. The feed-forward
  // torque is clipped to the limit, and when a stiffness is commanded the
  // position reference is pulled toward the measured position so that the
  // torque the drive will compute from the latest known state also stays
  // within the limit (the same bound SandroControl applied); the drive's own
  // MIT torque limit is the last line.
  void mit(Joint & joint, double torque)
  {
    joint.requested_effort = torque;
    const double limit = std::min(hard_effort_limit_nm, joint.limits.torque);
    joint.applied_effort = std::clamp(torque, -limit, limit);
    if (joint.applied_effort != torque) {
      joint.saturation_count += 1.0;
    }
    const auto gain = [](const double value, const double ceiling) {
      return std::isfinite(value) && value > 0.0 ? std::min(value, ceiling) : 0.0;
    };
    const double kp = gain(joint.command_kp, joint.limits.kp);
    const double kd = gain(joint.command_kd, joint.limits.kd);
    const double velocity_reference =
      std::isfinite(joint.command_velocity) ? joint.command_velocity : 0.0;
    double position_reference =
      kp > 0.0 && std::isfinite(joint.command_position) ? joint.command_position : joint.position;
    if (kp > 0.0) {
      const double damping = kd * (velocity_reference - joint.velocity);
      const double predicted =
        kp * (position_reference - joint.position) + damping + joint.applied_effort;
      const double bounded = std::clamp(predicted, -limit, limit);
      if (bounded != predicted) {
        position_reference =
          joint.position + (bounded - damping - joint.applied_effort) / kp;
        joint.saturation_count += 1.0;
      }
    }
    const auto bytes = protocol::pack_mit(
      protocol::wire_position(position_reference, joint.sign, joint.ratio, joint.offset) +
      joint.mit_frame_bias,
      joint.sign * velocity_reference, kp, kd, joint.sign * joint.applied_effort,
      joint.limits);
    send(joint.node, CanCommand::mit_control, bytes.data(), bytes.size());
  }

  // One line naming every drive that is not healthy, at most once a second,
  // so a disconnected motor is visible in the log among everything else.
  void report_faults(const std::string & offenders)
  {
    if (offenders.empty()) {
      if (fault_reported) {
        RCLCPP_INFO(rclcpp::get_logger("GIM6010System"), "all drives healthy again");
        fault_reported = false;
      }
      return;
    }
    fault_reported = true;
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger("GIM6010System"), steady_clock, 1000,
      "DRIVE FAULT: %s", offenders.c_str());
  }
};

GIM6010System::~GIM6010System() = default;

CallbackReturn GIM6010System::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  try {
    if (info.joints.size() != 6) {
      throw std::invalid_argument("GIM6010System requires exactly six joints");
    }
    impl_ = std::make_unique<Impl>();
    impl_->interface = required_string(info.hardware_parameters, "can_interface");
    impl_->encoder_period_ms = required_number<int>(info.hardware_parameters, "encoder_period_ms");
    impl_->iq_period_ms = required_number<int>(info.hardware_parameters, "iq_period_ms");
    impl_->heartbeat_period_ms = required_number<int>(info.hardware_parameters, "heartbeat_period_ms");
    impl_->deadman_ms = required_number<int>(info.hardware_parameters, "deadman_ms");
    impl_->stale_state_s = required_number<double>(info.hardware_parameters, "stale_state_s");
    impl_->heartbeat_stale_s = required_number<double>(
      info.hardware_parameters, "heartbeat_stale_s");
    impl_->activation_timeout_s = required_number<double>(info.hardware_parameters, "activation_timeout_s");
    impl_->encoder_rate_endpoint = required_number<uint16_t>(info.hardware_parameters, "encoder_rate_endpoint");
    impl_->iq_rate_endpoint = required_number<uint16_t>(info.hardware_parameters, "iq_rate_endpoint");
    impl_->heartbeat_rate_endpoint = required_number<uint16_t>(info.hardware_parameters, "heartbeat_rate_endpoint");
    impl_->deadman_endpoint = required_number<uint16_t>(info.hardware_parameters, "deadman_endpoint");
    impl_->mit_max_position_endpoint = required_number<uint16_t>(
      info.hardware_parameters, "mit_max_position_endpoint");
    impl_->mit_max_velocity_endpoint = required_number<uint16_t>(
      info.hardware_parameters, "mit_max_velocity_endpoint");
    impl_->mit_max_torque_endpoint = required_number<uint16_t>(
      info.hardware_parameters, "mit_max_torque_endpoint");
    impl_->mit_max_kp_endpoint = required_number<uint16_t>(
      info.hardware_parameters, "mit_max_kp_endpoint");
    impl_->mit_max_kd_endpoint = required_number<uint16_t>(
      info.hardware_parameters, "mit_max_kd_endpoint");
    impl_->hard_effort_limit_nm = required_number<double>(
      info.hardware_parameters, "hard_effort_limit_nm");
    const auto bias_tolerance = info.hardware_parameters.find("mit_frame_bias_tolerance_rad");
    if (bias_tolerance != info.hardware_parameters.end()) {
      impl_->mit_frame_bias_tolerance_rad = std::stod(bias_tolerance->second);
      if (!(impl_->mit_frame_bias_tolerance_rad >= 0.0)) {
        throw std::invalid_argument("mit_frame_bias_tolerance_rad must be >= 0");
      }
    }
    const auto state_source = info.hardware_parameters.find("state_source");
    if (state_source != info.hardware_parameters.end()) {
      if (state_source->second == "mit") {
        impl_->mit_state = true;
      } else if (state_source->second != "encoder") {
        throw std::invalid_argument("state_source must be 'encoder' or 'mit'");
      }
    }
    if (impl_->encoder_period_ms <= 0 || impl_->iq_period_ms <= 0 ||
      impl_->heartbeat_period_ms <= 0 || impl_->deadman_ms <= 0 ||
      impl_->stale_state_s <= 0.0 || impl_->heartbeat_stale_s <= 0.0 ||
      impl_->activation_timeout_s <= 0.0 ||
      impl_->hard_effort_limit_nm <= 0.0)
    {
      throw std::invalid_argument("period, deadman, stale_state and activation_timeout parameters must be > 0");
    }
    for (const auto & description : info.joints) {
      Impl::Joint joint;
      bool has_effort = false;
      for (const auto & command : description.command_interfaces) {
        if (command.name == hardware_interface::HW_IF_EFFORT) {
          has_effort = true;
        } else if (command.name == hardware_interface::HW_IF_POSITION) {
          joint.has_position_command = true;
        } else if (command.name == hardware_interface::HW_IF_VELOCITY) {
          joint.has_velocity_command = true;
        } else if (command.name == "kp") {
          joint.has_kp_command = true;
        } else if (command.name == "kd") {
          joint.has_kd_command = true;
        } else {
          throw std::invalid_argument(
                  description.name + ": unsupported command interface '" + command.name +
                  "' (effort, position, velocity, kp, kd)");
        }
      }
      if (!has_effort) {
        throw std::invalid_argument(description.name + " must expose an effort command interface");
      }
      joint.name = description.name;
      joint.node = required_number<uint8_t>(description.parameters, "node_id");
      joint.sign = required_number<double>(description.parameters, "sign");
      joint.ratio = required_number<double>(description.parameters, "ratio");
      joint.offset = required_number<double>(description.parameters, "offset_rev");
      if ((joint.sign != 1.0 && joint.sign != -1.0) || joint.ratio <= 0.0 ||
        joint.node > 63)
      {
        throw std::invalid_argument(description.name + ": invalid node, sign, or ratio parameter");
      }
      if (impl_->by_node.count(joint.node)) {
        throw std::invalid_argument("duplicate node_id " + std::to_string(joint.node));
      }
      impl_->by_node[joint.node] = impl_->joints.size();
      impl_->joints.push_back(std::move(joint));
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("GIM6010System"), "configuration invalid: %s", error.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> GIM6010System::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (auto & joint : impl_->joints) {
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &joint.position);
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &joint.velocity);
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &joint.effort);
    interfaces.emplace_back(joint.name, "feedback_age", &joint.age);
    interfaces.emplace_back(joint.name, "raw_position_rev", &joint.raw_position_rev);
    interfaces.emplace_back(joint.name, "raw_velocity_rev_s", &joint.raw_velocity_rev_s);
    interfaces.emplace_back(joint.name, "online", &joint.online);
    interfaces.emplace_back(joint.name, "iq", &joint.iq);
    interfaces.emplace_back(joint.name, "temperature", &joint.temperature);
    interfaces.emplace_back(joint.name, "error", &joint.error);
    interfaces.emplace_back(joint.name, "requested_effort", &joint.requested_effort);
    interfaces.emplace_back(joint.name, "applied_effort", &joint.applied_effort);
    interfaces.emplace_back(joint.name, "saturation_count", &joint.saturation_count);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> GIM6010System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (auto & joint : impl_->joints) {
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &joint.command);
    if (joint.has_position_command) {
      interfaces.emplace_back(
        joint.name, hardware_interface::HW_IF_POSITION, &joint.command_position);
    }
    if (joint.has_velocity_command) {
      interfaces.emplace_back(
        joint.name, hardware_interface::HW_IF_VELOCITY, &joint.command_velocity);
    }
    if (joint.has_kp_command) {
      interfaces.emplace_back(joint.name, "kp", &joint.command_kp);
    }
    if (joint.has_kd_command) {
      interfaces.emplace_back(joint.name, "kd", &joint.command_kd);
    }
  }
  return interfaces;
}

CallbackReturn GIM6010System::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    impl_->open_socket();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("GIM6010System"), "%s", error.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn GIM6010System::on_activate(const rclcpp_lifecycle::State &)
{
  try {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(impl_->activation_timeout_s);
    for (;;) {
      bool ready = true;
      {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        const auto now = std::chrono::steady_clock::now();
        for (const auto & joint : impl_->joints) {
          ready &= joint.encoder_stamp.time_since_epoch().count() != 0 &&
            joint.heartbeat_stamp.time_since_epoch().count() != 0 &&
            now - joint.encoder_stamp <= std::chrono::duration<double>(impl_->stale_state_s);
        }
      }
      if (ready) {break;}
      if (std::chrono::steady_clock::now() >= deadline) {
        // Name the culprits: which drives never spoke, and whether the bus
        // carried anything at all (a silent bus is power or wiring, not a
        // drive).
        std::string silent;
        {
          std::lock_guard<std::mutex> lock(impl_->mutex);
          const auto now = std::chrono::steady_clock::now();
          for (const auto & joint : impl_->joints) {
            const bool heartbeat = joint.heartbeat_stamp.time_since_epoch().count() != 0;
            const bool encoder = joint.encoder_stamp.time_since_epoch().count() != 0 &&
              now - joint.encoder_stamp <= std::chrono::duration<double>(impl_->stale_state_s);
            if (!heartbeat || !encoder) {
              silent += (silent.empty() ? "" : ", ") + joint.name + " (node " +
                std::to_string(joint.node) + (!heartbeat ? ": no heartbeat" : ": no encoder broadcast") + ")";
            }
          }
        }
        const unsigned long frames = impl_->frames_received.load();
        throw std::runtime_error(
                frames == 0 ?
                "no CAN frame at all in " + std::to_string(impl_->activation_timeout_s) +
                " s on '" + impl_->interface_name + "': the bus is silent (drives unpowered, "
                "cut-off engaged, or the adapter-to-drive cable is off); silent: " + silent :
                "drives without fresh feedback: " + silent + " (" + std::to_string(frames) +
                " frames seen on the bus)");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    for (auto & joint : impl_->joints) {
      uint32_t axis_error{};
      uint8_t fault_flags{};
      {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        axis_error = joint.axis_error;
        fault_flags = joint.fault_flags;
      }
      if (axis_error && !protocol::operator_clearable_error(axis_error, fault_flags)) {
        throw std::runtime_error(
                joint.name + " node " + std::to_string(joint.node) +
                ": axis_error=" + std::to_string(axis_error) +
                " fault_flags=" + std::to_string(fault_flags) +
                " (not an operator-induced error; inspect before re-arming)");
      }
      if (axis_error) {
        // Only the CAN watchdog and a requested E-stop reach here. Arming
        // again is the deliberate operator action that clears them; the
        // watchdog is disabled first so it cannot re-trip during the clear.
        RCLCPP_WARN(
          rclcpp::get_logger("GIM6010System"),
          "%s node %u: clearing operator-induced axis_error=0x%X (%s%s)",
          joint.name.c_str(), joint.node, axis_error,
          (axis_error & protocol::kAxisErrorEstopRequested) ? "estop_requested " : "",
          (axis_error & protocol::kAxisErrorCanBusFailed) ? "can_watchdog" : "");
        impl_->sdo_write_verified(joint.node, impl_->deadman_endpoint, 0);
        const uint8_t zeros[8]{};
        impl_->send(joint.node, CanCommand::clear_errors, zeros, 8);
      }
      impl_->sdo_write_verified(
        joint.node, impl_->encoder_rate_endpoint, impl_->encoder_period_ms);
      impl_->sdo_write_verified(joint.node, impl_->iq_rate_endpoint, impl_->iq_period_ms);
      impl_->sdo_write_verified(
        joint.node, impl_->heartbeat_rate_endpoint, impl_->heartbeat_period_ms);
      joint.limits.position = impl_->sdo_read_float(
        joint.node, impl_->mit_max_position_endpoint);
      joint.limits.velocity = impl_->sdo_read_float(
        joint.node, impl_->mit_max_velocity_endpoint);
      joint.limits.torque = impl_->sdo_read_float(
        joint.node, impl_->mit_max_torque_endpoint);
      joint.limits.kp = impl_->sdo_read_float(joint.node, impl_->mit_max_kp_endpoint);
      joint.limits.kd = impl_->sdo_read_float(joint.node, impl_->mit_max_kd_endpoint);
      RCLCPP_INFO(
        rclcpp::get_logger("GIM6010System"),
        "%s node %u drive limits: position=%.3f rad velocity=%.3f rad/s "
        "torque=%.3f Nm kp=%.3f kd=%.3f; hardware effort boundary=%.3f Nm",
        joint.name.c_str(), joint.node, joint.limits.position, joint.limits.velocity,
        joint.limits.torque, joint.limits.kp, joint.limits.kd, impl_->hard_effort_limit_nm);
      uint8_t mode[8]{};
      const uint32_t control = 3, input = 9;
      std::memcpy(mode, &control, 4);
      std::memcpy(mode + 4, &input, 4);
      impl_->send(joint.node, CanCommand::set_controller_mode, mode, 8);
      impl_->mit(joint, 0.0);
      impl_->send_u32(joint.node, CanCommand::set_axis_state, 8);
    }
    const auto closed_loop_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(impl_->activation_timeout_s);
    while (std::chrono::steady_clock::now() < closed_loop_deadline) {
      {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (std::all_of(
            impl_->joints.begin(), impl_->joints.end(),
            [](const auto & joint) {return joint.axis_state == 8;}))
        {
          break;
        }
      }
      for (auto & joint : impl_->joints) {impl_->mit(joint, 0.0);}
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!std::all_of(
        impl_->joints.begin(), impl_->joints.end(),
        [](const auto & joint) {return joint.axis_state == 8;}))
    {
      throw std::runtime_error("not all drives entered closed-loop control");
    }
    // The drive-side PD acts in the drive's own MIT position frame. The
    // zero-gain frames above were answered with that position; compare it
    // with the encoder broadcast for every joint a controller may stiffen,
    // correct a small bias, refuse a large one (a wrong zero would pin the
    // joint against the torque limit while the controller sees a small
    // error). Saturated replies (multi-turn wheels) are skipped.
    for (auto & joint : impl_->joints) {
      joint.mit_frame_bias = 0.0;
      if (!joint.has_kp_command) {
        continue;
      }
      double reply{}, logical{};
      {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        reply = joint.mit_reply_position;
        logical = joint.position;
      }
      if (!std::isfinite(reply) || std::abs(reply) >= 0.9 * joint.limits.position) {
        RCLCPP_WARN(
          rclcpp::get_logger("GIM6010System"),
          "%s node %u: no usable MIT position reply, stiffness will act in the drive's own frame",
          joint.name.c_str(), joint.node);
        continue;
      }
      const double expected = protocol::wire_position(logical, joint.sign, joint.ratio, joint.offset);
      const double bias = reply - expected;
      RCLCPP_INFO(
        rclcpp::get_logger("GIM6010System"),
        "%s node %u: MIT position %.4f rad, encoder frame %.4f rad, bias %.4f rad",
        joint.name.c_str(), joint.node, reply, expected, bias);
      if (std::abs(bias) > impl_->mit_frame_bias_tolerance_rad) {
        throw std::runtime_error(
                joint.name + " node " + std::to_string(joint.node) +
                ": MIT position frame differs from the encoder broadcast by " +
                std::to_string(bias) + " rad (tolerance " +
                std::to_string(impl_->mit_frame_bias_tolerance_rad) +
                "); a drive-side stiffness would pull the joint to the wrong place. Check the "
                "drive's zero (zero_load_encoder.py) and sign before arming");
      }
      joint.mit_frame_bias = bias;
    }
    // The communication watchdog is intentionally the final activation write.
    for (auto & joint : impl_->joints) {
      impl_->sdo_write_verified(joint.node, impl_->deadman_endpoint, impl_->deadman_ms);
      for (auto & keepalive : impl_->joints) {impl_->mit(keepalive, 0.0);}
    }
    impl_->active = true;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("GIM6010System"), "activation failed: %s", error.what());
    on_deactivate(rclcpp_lifecycle::State{});
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn GIM6010System::on_deactivate(const rclcpp_lifecycle::State &)
{
  impl_->active = false;
  bool ok = true;
  // Zero torque and idle first, every drive, whatever happens to the rest.
  for (auto & joint : impl_->joints) {
    try {
      impl_->mit(joint, 0.0);
      impl_->send_u32(joint.node, CanCommand::set_axis_state, 1);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        rclcpp::get_logger("GIM6010System"), "%s node %u: idle failed: %s",
        joint.name.c_str(), joint.node, error.what());
      ok = false;
    }
  }
  // Then disable the deadman, per drive, without letting one unreachable
  // node (a dead bus during a failed activation) stall the others.
  for (auto & joint : impl_->joints) {
    try {
      impl_->sdo_write_verified(joint.node, impl_->deadman_endpoint, 0);
      for (auto & keepalive : impl_->joints) {impl_->mit(keepalive, 0.0);}
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        rclcpp::get_logger("GIM6010System"), "%s node %u: deadman not disabled (%s); "
        "it will trip by itself without frames", joint.name.c_str(), joint.node, error.what());
      ok = false;
    }
  }
  return ok ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

CallbackReturn GIM6010System::on_cleanup(const rclcpp_lifecycle::State &)
{
  impl_->close_socket();
  return CallbackReturn::SUCCESS;
}

CallbackReturn GIM6010System::on_shutdown(const rclcpp_lifecycle::State & previous)
{
  // A shutdown from the active state (Ctrl-C on the controller manager)
  // must leave the drives idle, not holding their last command.
  if (impl_->active) {
    on_deactivate(previous);
  }
  impl_->close_socket();
  return CallbackReturn::SUCCESS;
}

CallbackReturn GIM6010System::on_error(const rclcpp_lifecycle::State & previous)
{
  if (impl_->active) {
    on_deactivate(previous);
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type GIM6010System::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(impl_->mutex);
  std::string offenders;
  for (auto & joint : impl_->joints) {
    joint.age = joint.encoder_stamp.time_since_epoch().count() ?
      std::chrono::duration<double>(now - joint.encoder_stamp).count() :
      std::numeric_limits<double>::infinity();
    const auto heartbeat_age = joint.heartbeat_stamp.time_since_epoch().count() ?
      std::chrono::duration<double>(now - joint.heartbeat_stamp).count() :
      std::numeric_limits<double>::infinity();
    joint.online =
      joint.age < impl_->stale_state_s && heartbeat_age < impl_->heartbeat_stale_s ? 1.0 : 0.0;
    if (impl_->active && (
        joint.age >= impl_->stale_state_s ||
        heartbeat_age >= impl_->heartbeat_stale_s || joint.error != 0.0))
    {
      char detail[192];
      std::snprintf(
        detail, sizeof(detail), "%s (node %u): %s%s%s",
        joint.name.c_str(), joint.node,
        joint.age >= impl_->stale_state_s ? "no encoder feedback " : "",
        heartbeat_age >= impl_->heartbeat_stale_s ? "no heartbeat " : "",
        joint.error != 0.0 ? "axis error " : "");
      if (joint.error != 0.0) {
        char code[64];
        std::snprintf(
          code, sizeof(code), "0x%X%s%s flags=%u", joint.axis_error,
          (joint.axis_error & protocol::kAxisErrorEstopRequested) ? " estop_requested" : "",
          (joint.axis_error & protocol::kAxisErrorCanBusFailed) ? " can_watchdog" : "",
          joint.fault_flags);
        std::strncat(detail, code, sizeof(detail) - std::strlen(detail) - 1);
      }
      offenders += (offenders.empty() ? "" : " | ") + std::string(detail);
    }
  }
  impl_->report_faults(offenders);
  return offenders.empty() ? hardware_interface::return_type::OK :
         hardware_interface::return_type::ERROR;
}

hardware_interface::return_type GIM6010System::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!impl_->active) {
    return hardware_interface::return_type::OK;
  }
  try {
    for (auto & joint : impl_->joints) {
      if (!std::isfinite(joint.command)) {
        throw std::runtime_error(joint.name + ": effort command is not finite");
      }
      impl_->mit(joint, joint.command);
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("GIM6010System"), "command failed: %s", error.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}
}  // namespace sandro_hardware

PLUGINLIB_EXPORT_CLASS(sandro_hardware::GIM6010System, hardware_interface::SystemInterface)
