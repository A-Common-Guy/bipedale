#include "sandro_hardware/bno085_sensor.hpp"

#include "sandro_hardware/protocol.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <mutex>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <thread>
#include <type_traits>
#include <unistd.h>
#include <unordered_map>

namespace sandro_hardware
{
namespace
{
using CallbackReturn = hardware_interface::CallbackReturn;

template<typename T>
T required_number(const std::unordered_map<std::string, std::string> & parameters, const std::string & key)
{
  const auto found = parameters.find(key);
  if (found == parameters.end()) {
    throw std::invalid_argument("missing required parameter '" + key + "'");
  }
  std::size_t used{};
  if constexpr (std::is_integral_v<T>) {
    const auto value = std::stoll(found->second, &used, 0);
    if (used != found->second.size() ||
      value < static_cast<long long>(std::numeric_limits<T>::lowest()) ||
      static_cast<unsigned long long>(value) >
      static_cast<unsigned long long>(std::numeric_limits<T>::max()))
    {
      throw std::invalid_argument("parameter '" + key + "' is outside its integer range");
    }
    return static_cast<T>(value);
  } else {
    const auto value = std::stod(found->second, &used);
    if (used != found->second.size() || !std::isfinite(value)) {
      throw std::invalid_argument("parameter '" + key + "' is not finite");
    }
    return static_cast<T>(value);
  }
}

std::string required_string(
  const std::unordered_map<std::string, std::string> & parameters, const std::string & key)
{
  const auto found = parameters.find(key);
  if (found == parameters.end() || found->second.empty()) {
    throw std::invalid_argument("missing required parameter '" + key + "'");
  }
  return found->second;
}

speed_t baud_constant(int baud)
{
  switch (baud) {
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 2000000: return B2000000;
    case 3000000: return B3000000;
    default: throw std::invalid_argument("unsupported UART baudrate " + std::to_string(baud));
  }
}
}  // namespace

struct BNO085Sensor::Impl
{
  std::string name;
  std::string port;
  int baudrate{};
  int read_timeout_ms{};
  uint32_t report_interval_us{};
  double online_timeout_s{};
  bool fault_reported{false};
  int tx_byte_gap_us{};
  int max_backlog_bytes{};
  int descriptor{-1};
  std::atomic_bool stop{false};
  std::thread receiver;
  std::mutex mutex;
  protocol::ShtpFramer framer;
  std::unordered_map<uint8_t, uint8_t> sequence;
  std::array<double, 4> orientation{0.0, 0.0, 0.0, 1.0};
  std::array<double, 3> angular_velocity{};
  std::array<double, 3> linear_acceleration{};
  std::array<std::chrono::steady_clock::time_point, 3> report_stamps{};
  std::array<double, 12> exported{0.0, 0.0, 0.0, 1.0};

  ~Impl() {close_port();}

  void open_port()
  {
    descriptor = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (descriptor < 0) {
      throw std::runtime_error("cannot open " + port + ": " + std::strerror(errno));
    }
    termios settings{};
    if (::tcgetattr(descriptor, &settings) != 0) {
      throw std::runtime_error("tcgetattr failed: " + std::string(std::strerror(errno)));
    }
    ::cfmakeraw(&settings);
    const auto speed = baud_constant(baudrate);
    ::cfsetispeed(&settings, speed);
    ::cfsetospeed(&settings, speed);
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = static_cast<cc_t>(std::clamp(read_timeout_ms / 100, 0, 255));
    if (::tcsetattr(descriptor, TCSANOW, &settings) != 0) {
      throw std::runtime_error("tcsetattr failed: " + std::string(std::strerror(errno)));
    }
    ::tcflush(descriptor, TCIOFLUSH);
  }

  void close_port()
  {
    stop = true;
    if (receiver.joinable()) {
      receiver.join();
    }
    if (descriptor >= 0) {
      ::close(descriptor);
      descriptor = -1;
    }
  }

  void send_report(uint8_t report, uint32_t interval)
  {
    std::vector<uint8_t> command(17);
    command[0] = 0xfd;
    command[1] = report;
    std::memcpy(command.data() + 5, &interval, 4);
    const auto frame = protocol::uart_frame(2, sequence[2]++, command);
    for (const auto byte : frame) {
      if (::write(descriptor, &byte, 1) != 1) {
        throw std::runtime_error("BNO085 UART write failed: " + std::string(std::strerror(errno)));
      }
      if (tx_byte_gap_us > 0) {
        std::this_thread::sleep_for(std::chrono::microseconds(tx_byte_gap_us));
      }
    }
    ::tcdrain(descriptor);
  }

  void consume(const uint8_t * data, std::size_t size)
  {
    auto packets = framer.feed(data, size);
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & packet : packets) {
      for (const auto & [id, report] : protocol::split_reports(packet)) {
        if (id == 0x08) {
          const auto value = protocol::parse_sensor_report(report, 4, std::ldexp(1.0, -14));
          orientation = {value[0], value[1], value[2], value[3]};
          report_stamps[0] = now;
        } else if (id == 0x02) {
          const auto value = protocol::parse_sensor_report(report, 3, std::ldexp(1.0, -9));
          angular_velocity = {value[0], value[1], value[2]};
          report_stamps[1] = now;
        } else if (id == 0x01) {
          const auto value = protocol::parse_sensor_report(report, 3, std::ldexp(1.0, -8));
          linear_acceleration = {value[0], value[1], value[2]};
          report_stamps[2] = now;
        }
      }
    }
  }

  void receive_loop()
  {
    std::vector<uint8_t> bytes(static_cast<std::size_t>(max_backlog_bytes));
    while (!stop) {
      const auto count = ::read(descriptor, bytes.data(), bytes.size());
      if (count > 0) {
        consume(bytes.data(), static_cast<std::size_t>(count));
      } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        RCLCPP_ERROR(
          rclcpp::get_logger("BNO085Sensor"), "UART read failed: %s", std::strerror(errno));
        return;
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(read_timeout_ms));
      }
    }
  }
};

BNO085Sensor::~BNO085Sensor() = default;

CallbackReturn BNO085Sensor::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SensorInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  try {
    if (info.sensors.size() != 1) {
      throw std::invalid_argument("BNO085Sensor requires exactly one sensor");
    }
    impl_ = std::make_unique<Impl>();
    impl_->name = info.sensors.front().name;
    impl_->port = required_string(info.hardware_parameters, "port");
    impl_->baudrate = required_number<int>(info.hardware_parameters, "baudrate");
    impl_->read_timeout_ms = required_number<int>(info.hardware_parameters, "read_timeout_ms");
    impl_->report_interval_us = required_number<uint32_t>(
      info.hardware_parameters, "report_interval_us");
    impl_->online_timeout_s = required_number<double>(
      info.hardware_parameters, "online_timeout_s");
    impl_->tx_byte_gap_us = required_number<int>(info.hardware_parameters, "tx_byte_gap_us");
    impl_->max_backlog_bytes = required_number<int>(
      info.hardware_parameters, "max_backlog_bytes");
    if (impl_->read_timeout_ms <= 0 || impl_->report_interval_us == 0 ||
      impl_->online_timeout_s <= 0.0 || impl_->tx_byte_gap_us < 0 ||
      impl_->max_backlog_bytes < 64)
    {
      throw std::invalid_argument(
              "read_timeout, report_interval, online_timeout and backlog must be positive");
    }
    (void)baud_constant(impl_->baudrate);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("BNO085Sensor"), "configuration invalid: %s", error.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> BNO085Sensor::export_state_interfaces()
{
  static constexpr std::array<const char *, 12> names{
    "orientation.x", "orientation.y", "orientation.z", "orientation.w",
    "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
    "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z",
    "age", "online"};
  std::vector<hardware_interface::StateInterface> interfaces;
  for (std::size_t i = 0; i < names.size(); ++i) {
    interfaces.emplace_back(impl_->name, names[i], &impl_->exported[i]);
  }
  return interfaces;
}

CallbackReturn BNO085Sensor::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    impl_->open_port();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("BNO085Sensor"), "%s", error.what());
    impl_->close_port();
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn BNO085Sensor::on_activate(const rclcpp_lifecycle::State &)
{
  try {
    ::tcflush(impl_->descriptor, TCIFLUSH);
    impl_->framer.reset();
    {
      std::lock_guard<std::mutex> lock(impl_->mutex);
      impl_->report_stamps = {};
    }
    for (const auto report : {0x03, 0x04, 0x05, 0x06, 0x09}) {
      impl_->send_report(static_cast<uint8_t>(report), 0);
    }
    for (const auto report : {0x08, 0x02, 0x01}) {
      impl_->send_report(static_cast<uint8_t>(report), impl_->report_interval_us);
    }
    impl_->stop = false;
    impl_->receiver = std::thread([this] {impl_->receive_loop();});
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(impl_->online_timeout_s);
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (std::all_of(
            impl_->report_stamps.begin(), impl_->report_stamps.end(),
            [](const auto & stamp) {return stamp.time_since_epoch().count() != 0;}))
        {
          return CallbackReturn::SUCCESS;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    throw std::runtime_error("BNO085 did not stream orientation, gyro, and acceleration");
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("BNO085Sensor"), "activation failed: %s", error.what());
    on_deactivate(rclcpp_lifecycle::State{});
    return CallbackReturn::ERROR;
  }
}

CallbackReturn BNO085Sensor::on_deactivate(const rclcpp_lifecycle::State &)
{
  impl_->stop = true;
  if (impl_->receiver.joinable()) {
    impl_->receiver.join();
  }
  try {
    for (const auto report : {0x08, 0x02, 0x01}) {
      impl_->send_report(static_cast<uint8_t>(report), 0);
    }
  } catch (const std::exception & error) {
    RCLCPP_WARN(rclcpp::get_logger("BNO085Sensor"), "could not quiesce BNO085: %s", error.what());
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn BNO085Sensor::on_cleanup(const rclcpp_lifecycle::State &)
{
  impl_->close_port();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type BNO085Sensor::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(impl_->mutex);
  std::copy(impl_->orientation.begin(), impl_->orientation.end(), impl_->exported.begin());
  std::copy(
    impl_->angular_velocity.begin(), impl_->angular_velocity.end(), impl_->exported.begin() + 4);
  std::copy(
    impl_->linear_acceleration.begin(), impl_->linear_acceleration.end(), impl_->exported.begin() + 7);
  impl_->exported[10] = impl_->report_stamps[0].time_since_epoch().count() ?
    std::chrono::duration<double>(now - impl_->report_stamps[0]).count() :
    std::numeric_limits<double>::infinity();
  impl_->exported[11] = impl_->exported[10] < impl_->online_timeout_s ? 1.0 : 0.0;
  if (impl_->exported[11] == 0.0) {
    // Standard IMU interfaces have no age field. A NaN orientation makes
    // stale data visible to every standard consumer while the hardware
    // component remains alive and can recover when samples resume.
    std::fill_n(
      impl_->exported.begin(), 4, std::numeric_limits<double>::quiet_NaN());
    static rclcpp::Clock steady_clock{RCL_STEADY_TIME};
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger("BNO085Sensor"), steady_clock, 1000,
      "IMU FAULT: no BNO085 report for %.0f ms (balancers stop the wheels on NaN attitude)",
      1e3 * impl_->exported[10]);
    impl_->fault_reported = true;
  } else if (impl_->fault_reported) {
    RCLCPP_INFO(rclcpp::get_logger("BNO085Sensor"), "BNO085 reports again");
    impl_->fault_reported = false;
  }
  return hardware_interface::return_type::OK;
}
}  // namespace sandro_hardware

PLUGINLIB_EXPORT_CLASS(sandro_hardware::BNO085Sensor, hardware_interface::SensorInterface)
