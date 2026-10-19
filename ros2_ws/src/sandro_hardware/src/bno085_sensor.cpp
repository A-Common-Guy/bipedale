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
#include <linux/gpio.h>
#include <linux/spi/spidev.h>
#include <mutex>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
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
using Clock = std::chrono::steady_clock;

// spidev's default transfer buffer (module parameter bufsiz).
constexpr std::size_t kSpiMaxTransfer = 4096;

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

std::runtime_error system_error(const std::string & what)
{
  return std::runtime_error(what + ": " + std::strerror(errno));
}
}  // namespace

struct BNO085Sensor::Impl
{
  enum class Transport { uart, spi };

  std::string name;
  Transport transport{Transport::uart};
  int read_timeout_ms{};
  uint32_t report_interval_us{};
  double online_timeout_s{};
  bool fault_reported{false};
  std::atomic_bool stop{false};
  std::thread receiver;
  std::mutex mutex;
  std::unordered_map<uint8_t, uint8_t> sequence;
  std::array<double, 4> orientation{0.0, 0.0, 0.0, 1.0};
  std::array<double, 3> angular_velocity{};
  std::array<double, 3> linear_acceleration{};
  std::array<Clock::time_point, 3> report_stamps{};
  std::array<double, 12> exported{0.0, 0.0, 0.0, 1.0};

  // UART over the FTDI adapter.
  std::string port;
  int baudrate{};
  int tx_byte_gap_us{};
  int max_backlog_bytes{};
  int descriptor{-1};
  protocol::ShtpFramer framer;

  // SPI on the Raspberry Pi: spidev plus the INT (active low) and RST lines
  // through the GPIO character device. WAKE is tied high, so the chip only
  // accepts a write while it asserts INT.
  std::string spi_device;
  uint32_t spi_speed_hz{};
  std::string gpio_chip;
  uint32_t int_gpio{};
  uint32_t reset_gpio{};
  int spi{-1};
  int lines{-1};

  ~Impl() {close_port();}

  void open_port()
  {
    if (transport == Transport::uart) {
      open_uart();
    } else {
      open_spi();
    }
  }

  void open_uart()
  {
    descriptor = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (descriptor < 0) {
      throw system_error("cannot open " + port);
    }
    termios settings{};
    if (::tcgetattr(descriptor, &settings) != 0) {
      throw system_error("tcgetattr failed");
    }
    ::cfmakeraw(&settings);
    const auto speed = baud_constant(baudrate);
    ::cfsetispeed(&settings, speed);
    ::cfsetospeed(&settings, speed);
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = static_cast<cc_t>(std::clamp(read_timeout_ms / 100, 0, 255));
    if (::tcsetattr(descriptor, TCSANOW, &settings) != 0) {
      throw system_error("tcsetattr failed");
    }
    ::tcflush(descriptor, TCIOFLUSH);
  }

  void open_spi()
  {
    spi = ::open(spi_device.c_str(), O_RDWR);
    if (spi < 0) {
      throw system_error("cannot open " + spi_device);
    }
    const uint8_t mode = SPI_MODE_3;
    const uint8_t bits = 8;
    if (::ioctl(spi, SPI_IOC_WR_MODE, &mode) < 0 ||
      ::ioctl(spi, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
      ::ioctl(spi, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed_hz) < 0)
    {
      throw system_error("cannot configure " + spi_device + " for SPI mode 3");
    }

    const int chip = ::open(gpio_chip.c_str(), O_RDWR | O_CLOEXEC);
    if (chip < 0) {
      throw system_error("cannot open " + gpio_chip);
    }
    gpio_v2_line_request request{};
    request.offsets[0] = int_gpio;
    request.offsets[1] = reset_gpio;
    request.num_lines = 2;
    std::strncpy(request.consumer, "sandro_bno085", GPIO_MAX_NAME_SIZE - 1);
    request.config.flags =
      GPIO_V2_LINE_FLAG_INPUT | GPIO_V2_LINE_FLAG_BIAS_PULL_UP | GPIO_V2_LINE_FLAG_EDGE_FALLING;
    request.config.num_attrs = 2;
    request.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_FLAGS;
    request.config.attrs[0].attr.flags = GPIO_V2_LINE_FLAG_OUTPUT;
    request.config.attrs[0].mask = 0b10;
    request.config.attrs[1].attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
    request.config.attrs[1].attr.values = 0b10;  // RST released
    request.config.attrs[1].mask = 0b10;
    const int result = ::ioctl(chip, GPIO_V2_GET_LINE_IOCTL, &request);
    const int request_errno = errno;
    ::close(chip);
    if (result < 0) {
      errno = request_errno;
      throw system_error(
              "cannot request GPIO " + std::to_string(int_gpio) + " (INT) and " +
              std::to_string(reset_gpio) + " (RST) on " + gpio_chip);
    }
    lines = request.fd;
  }

  void close_port()
  {
    stop = true;
    if (receiver.joinable()) {
      receiver.join();
    }
    for (int * fd : {&descriptor, &spi, &lines}) {
      if (*fd >= 0) {
        ::close(*fd);
        *fd = -1;
      }
    }
  }

  bool int_asserted() const
  {
    gpio_v2_line_values values{};
    values.mask = 0b01;
    if (::ioctl(lines, GPIO_V2_LINE_GET_VALUES_IOCTL, &values) < 0) {
      throw system_error("cannot read the BNO085 INT line");
    }
    return (values.bits & 0b01) == 0;
  }

  void set_reset(bool released) const
  {
    gpio_v2_line_values values{};
    values.mask = 0b10;
    values.bits = released ? 0b10 : 0;
    if (::ioctl(lines, GPIO_V2_LINE_SET_VALUES_IOCTL, &values) < 0) {
      throw system_error("cannot drive the BNO085 RST line");
    }
  }

  bool wait_int(std::chrono::milliseconds timeout) const
  {
    const auto deadline = Clock::now() + timeout;
    for (;;) {
      if (int_asserted()) {
        return true;
      }
      const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - Clock::now()).count();
      if (left <= 0) {
        return false;
      }
      // The level is re-read after every wake-up, so a missed edge only
      // costs one poll period.
      pollfd descriptor_poll{lines, POLLIN, 0};
      if (::poll(&descriptor_poll, 1, static_cast<int>(std::min<long>(left, 2))) > 0) {
        std::array<gpio_v2_line_event, 16> events{};
        if (::read(lines, events.data(), sizeof(events)) < 0 && errno != EINTR) {
          throw system_error("cannot read BNO085 INT events");
        }
      }
    }
  }

  void spi_transfer(const uint8_t * tx, uint8_t * rx, std::size_t size) const
  {
    spi_ioc_transfer transfer{};
    transfer.tx_buf = reinterpret_cast<uintptr_t>(tx);
    transfer.rx_buf = reinterpret_cast<uintptr_t>(rx);
    transfer.len = static_cast<uint32_t>(size);
    transfer.speed_hz = spi_speed_hz;
    transfer.bits_per_word = 8;
    if (::ioctl(spi, SPI_IOC_MESSAGE(1), &transfer) < 0) {
      throw system_error("BNO085 SPI transfer failed");
    }
  }

  // Header first, then the whole packet in one transfer while INT is still
  // asserted (the chip re-sends it from the start), as raspi_tests/bno085_spi.py.
  std::vector<uint8_t> read_spi_packet(std::chrono::milliseconds timeout) const
  {
    if (!wait_int(timeout)) {
      return {};
    }
    static const std::vector<uint8_t> zeros(kSpiMaxTransfer, 0);
    std::array<uint8_t, 4> header{};
    spi_transfer(zeros.data(), header.data(), header.size());
    const auto length = protocol::shtp_length(header.data());
    if (length <= 4 || length > kSpiMaxTransfer || !wait_int(std::chrono::milliseconds(100))) {
      return {};
    }
    std::vector<uint8_t> packet(length);
    spi_transfer(zeros.data(), packet.data(), packet.size());
    return packet;
  }

  void send_report(uint8_t report, uint32_t interval)
  {
    const auto command = protocol::set_feature_command(report, interval);
    const uint8_t channel = protocol::kShtpControlChannel;
    if (transport == Transport::spi) {
      if (!wait_int(std::chrono::milliseconds(1000))) {
        RCLCPP_WARN(
          rclcpp::get_logger("BNO085Sensor"),
          "INT not asserted before writing report 0x%02x; the chip may ignore it", report);
      }
      const auto packet = protocol::shtp_packet(channel, sequence[channel]++, command);
      spi_transfer(packet.data(), nullptr, packet.size());
      return;
    }
    const auto frame = protocol::uart_frame(channel, sequence[channel]++, command);
    for (const auto byte : frame) {
      if (::write(descriptor, &byte, 1) != 1) {
        throw system_error("BNO085 UART write failed");
      }
      if (tx_byte_gap_us > 0) {
        std::this_thread::sleep_for(std::chrono::microseconds(tx_byte_gap_us));
      }
    }
    ::tcdrain(descriptor);
  }

  void consume_packet(const std::vector<uint8_t> & packet, Clock::time_point now)
  {
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

  void consume(const uint8_t * data, std::size_t size)
  {
    auto packets = framer.feed(data, size);
    const auto now = Clock::now();
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & packet : packets) {
      consume_packet(packet, now);
    }
  }

  void receive_loop()
  {
    if (transport == Transport::spi) {
      receive_spi();
    } else {
      receive_uart();
    }
  }

  void receive_uart()
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

  void receive_spi()
  {
    try {
      while (!stop) {
        const auto packet = read_spi_packet(std::chrono::milliseconds(read_timeout_ms));
        if (packet.size() > 4 && packet[2] == protocol::kShtpReportChannel) {
          const auto now = Clock::now();
          std::lock_guard<std::mutex> lock(mutex);
          consume_packet(packet, now);
        }
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(rclcpp::get_logger("BNO085Sensor"), "%s", error.what());
    }
  }

  // After a reset the chip announces itself and accepts writes only while it
  // still signals with INT: read one boot packet, then write. Draining every
  // boot packet first lets it go quiet and it then ignores writes.
  void boot_spi()
  {
    set_reset(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    set_reset(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    if (!wait_int(std::chrono::milliseconds(2000))) {
      throw std::runtime_error("no BNO085 INT after reset: check INT/RST wiring and power");
    }
    (void)read_spi_packet(std::chrono::milliseconds(1000));
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
    const auto & parameters = info.hardware_parameters;
    impl_ = std::make_unique<Impl>();
    impl_->name = info.sensors.front().name;
    const auto transport = parameters.count("transport") ?
      parameters.at("transport") : std::string("uart");
    if (transport == "spi") {
      impl_->transport = Impl::Transport::spi;
    } else if (transport != "uart") {
      throw std::invalid_argument("transport must be uart or spi, not '" + transport + "'");
    }
    impl_->read_timeout_ms = required_number<int>(parameters, "read_timeout_ms");
    impl_->report_interval_us = required_number<uint32_t>(parameters, "report_interval_us");
    impl_->online_timeout_s = required_number<double>(parameters, "online_timeout_s");
    if (impl_->read_timeout_ms <= 0 || impl_->report_interval_us == 0 ||
      impl_->online_timeout_s <= 0.0)
    {
      throw std::invalid_argument(
              "read_timeout_ms, report_interval_us and online_timeout_s must be positive");
    }
    if (impl_->transport == Impl::Transport::spi) {
      impl_->spi_device = required_string(parameters, "spi_device");
      impl_->spi_speed_hz = required_number<uint32_t>(parameters, "spi_speed_hz");
      impl_->gpio_chip = required_string(parameters, "gpio_chip");
      impl_->int_gpio = required_number<uint32_t>(parameters, "int_gpio");
      impl_->reset_gpio = required_number<uint32_t>(parameters, "reset_gpio");
      if (impl_->spi_speed_hz == 0 || impl_->int_gpio == impl_->reset_gpio) {
        throw std::invalid_argument("spi_speed_hz must be positive and INT and RST distinct");
      }
    } else {
      impl_->port = required_string(parameters, "port");
      impl_->baudrate = required_number<int>(parameters, "baudrate");
      impl_->tx_byte_gap_us = required_number<int>(parameters, "tx_byte_gap_us");
      impl_->max_backlog_bytes = required_number<int>(parameters, "max_backlog_bytes");
      if (impl_->tx_byte_gap_us < 0 || impl_->max_backlog_bytes < 64) {
        throw std::invalid_argument("tx_byte_gap_us must be >= 0 and max_backlog_bytes >= 64");
      }
      (void)baud_constant(impl_->baudrate);
    }
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
    {
      std::lock_guard<std::mutex> lock(impl_->mutex);
      impl_->report_stamps = {};
    }
    if (impl_->transport == Impl::Transport::spi) {
      // The reset leaves every report disabled.
      impl_->sequence.clear();
      impl_->boot_spi();
    } else {
      ::tcflush(impl_->descriptor, TCIFLUSH);
      impl_->framer.reset();
      for (const auto report : {0x03, 0x04, 0x05, 0x06, 0x09}) {
        impl_->send_report(static_cast<uint8_t>(report), 0);
      }
    }
    for (const auto report : {0x08, 0x02, 0x01}) {
      impl_->send_report(static_cast<uint8_t>(report), impl_->report_interval_us);
    }
    impl_->stop = false;
    impl_->receiver = std::thread([this] {impl_->receive_loop();});
    const auto deadline = Clock::now() + std::chrono::duration<double>(impl_->online_timeout_s);
    while (Clock::now() < deadline) {
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
    if (impl_->transport == Impl::Transport::spi) {
      // Activation resets the chip anyway; holding it in reset is the one
      // write that needs no INT handshake.
      impl_->set_reset(false);
    } else {
      for (const auto report : {0x08, 0x02, 0x01}) {
        impl_->send_report(static_cast<uint8_t>(report), 0);
      }
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
  const auto now = Clock::now();
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
