#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

namespace sandro_hardware::protocol
{
constexpr double kTwoPi = 6.28318530717958647692;
constexpr uint8_t kDelimiter = 0x7e;
constexpr uint8_t kEscape = 0x7d;

enum class CanCommand : uint8_t
{
  heartbeat = 0x01, estop = 0x02, rx_sdo = 0x04, tx_sdo = 0x05,
  set_axis_state = 0x07, mit_control = 0x08, encoder = 0x09,
  set_controller_mode = 0x0b, iq = 0x14, clear_errors = 0x18
};

// Axis error bits the operator can cause and re-arming may clear. The CAN
// watchdog (deadman) trips when the host stops sending, and the E-stop command
// latches ESTOP_REQUESTED. Every other bit is a genuine fault and blocks
// activation until it is investigated.
constexpr uint32_t kAxisErrorCanBusFailed = 0x100000;
constexpr uint32_t kAxisErrorEstopRequested = 0x4000;

inline bool operator_clearable_error(uint32_t axis_error, uint8_t fault_flags)
{
  constexpr uint32_t clearable = kAxisErrorCanBusFailed | kAxisErrorEstopRequested;
  return axis_error != 0 && (axis_error & ~clearable) == 0 && fault_flags == 0;
}

inline uint16_t can_id(uint8_t node, CanCommand command)
{
  if (node > 63) {
    throw std::out_of_range("GIM6010 node_id must be in [0, 63]");
  }
  return static_cast<uint16_t>((node << 5) | static_cast<uint8_t>(command));
}

struct MitLimits
{
  double position{12.5};
  double velocity{65.0};
  double kp{500.0};
  double kd{5.0};
  double torque{50.0};
};

inline uint32_t to_uint(double value, double low, double high, unsigned bits)
{
  value = std::clamp(value, low, high);
  const auto span = (uint32_t{1} << bits) - 1;
  return static_cast<uint32_t>(std::llround((value - low) * span / (high - low)));
}

inline double from_uint(uint32_t value, double low, double high, unsigned bits)
{
  const auto span = (uint32_t{1} << bits) - 1;
  return static_cast<double>(value) * (high - low) / span + low;
}

inline std::array<uint8_t, 8> pack_mit(
  double position, double velocity, double kp, double kd, double torque,
  const MitLimits & limit)
{
  const auto p = to_uint(position, -limit.position, limit.position, 16);
  const auto v = to_uint(velocity, -limit.velocity, limit.velocity, 12);
  const auto k_p = to_uint(kp, 0.0, limit.kp, 12);
  const auto k_d = to_uint(kd, 0.0, limit.kd, 12);
  const auto t = to_uint(torque, -limit.torque, limit.torque, 12);
  return {
    static_cast<uint8_t>(p >> 8), static_cast<uint8_t>(p),
    static_cast<uint8_t>(v >> 4), static_cast<uint8_t>((v << 4) | (k_p >> 8)),
    static_cast<uint8_t>(k_p), static_cast<uint8_t>(k_d >> 4),
    static_cast<uint8_t>((k_d << 4) | (t >> 8)), static_cast<uint8_t>(t)};
}

inline std::tuple<uint8_t, double, double, double> unpack_mit(
  const uint8_t * data, std::size_t size, const MitLimits & limit)
{
  if (size < 6) {
    throw std::invalid_argument("MIT feedback must contain at least 6 bytes");
  }
  const uint32_t p = (data[1] << 8) | data[2];
  const uint32_t v = (data[3] << 4) | (data[4] >> 4);
  const uint32_t t = ((data[4] & 0x0f) << 8) | data[5];
  return {
    data[0], from_uint(p, -limit.position, limit.position, 16),
    from_uint(v, -limit.velocity, limit.velocity, 12),
    from_uint(t, -limit.torque, limit.torque, 12)};
}

inline double logical_position(double rotor_revolutions, double sign, double ratio, double offset)
{
  return sign * (rotor_revolutions - offset) / ratio * kTwoPi;
}

inline double logical_velocity(double rotor_revolutions_s, double sign, double ratio)
{
  return sign * rotor_revolutions_s / ratio * kTwoPi;
}

inline double wire_position(double logical_radians, double sign, double ratio, double offset)
{
  return sign * logical_radians + offset / ratio * kTwoPi;
}

inline std::vector<uint8_t> escape(const std::vector<uint8_t> & input)
{
  std::vector<uint8_t> output;
  output.reserve(input.size());
  for (const auto byte : input) {
    if (byte == kDelimiter || byte == kEscape) {
      output.push_back(kEscape);
      output.push_back(byte ^ 0x20);
    } else {
      output.push_back(byte);
    }
  }
  return output;
}

inline std::vector<uint8_t> uart_frame(
  uint8_t channel, uint8_t sequence, const std::vector<uint8_t> & payload)
{
  const uint16_t length = static_cast<uint16_t>(4 + payload.size());
  std::vector<uint8_t> packet{
    static_cast<uint8_t>(length), static_cast<uint8_t>(length >> 8), channel, sequence};
  packet.insert(packet.end(), payload.begin(), payload.end());
  packet = escape(packet);
  packet.insert(packet.begin(), {kDelimiter, 0x01});
  packet.push_back(kDelimiter);
  return packet;
}

class ShtpFramer
{
public:
  std::vector<std::vector<uint8_t>> feed(const uint8_t * data, std::size_t size)
  {
    buffer_.insert(buffer_.end(), data, data + size);
    std::vector<std::vector<uint8_t>> packets;
    for (;;) {
      const auto first = std::find(buffer_.begin(), buffer_.end(), kDelimiter);
      if (first == buffer_.end()) {
        buffer_.clear();
        break;
      }
      const auto second = std::find(first + 1, buffer_.end(), kDelimiter);
      if (second == buffer_.end()) {
        buffer_.erase(buffer_.begin(), first);
        break;
      }
      std::vector<uint8_t> frame(first + 1, second);
      buffer_.erase(buffer_.begin(), second);
      std::vector<uint8_t> decoded;
      bool escaped = false;
      for (const auto byte : frame) {
        if (escaped) {
          decoded.push_back(byte ^ 0x20);
          escaped = false;
        } else if (byte == kEscape) {
          escaped = true;
        } else {
          decoded.push_back(byte);
        }
      }
      if (decoded.size() > 5 && decoded.front() == 0x01) {
        decoded.erase(decoded.begin());
        const auto length = static_cast<std::size_t>(
          (decoded[0] | (decoded[1] << 8)) & 0x7fff);
        if (length >= 4 && length <= decoded.size()) {
          decoded.resize(length);
          packets.push_back(std::move(decoded));
        }
      }
    }
    return packets;
  }
  void reset() {buffer_.clear();}

private:
  std::vector<uint8_t> buffer_;
};

inline std::vector<std::pair<uint8_t, std::vector<uint8_t>>> split_reports(
  const std::vector<uint8_t> & packet)
{
  std::vector<std::pair<uint8_t, std::vector<uint8_t>>> reports;
  if (packet.size() < 4) {
    return reports;
  }
  const std::array<std::size_t, 256> sizes = [] {
      std::array<std::size_t, 256> out{};
      out[0x01] = 10; out[0x02] = 10; out[0x03] = 10; out[0x04] = 10;
      out[0x05] = 14; out[0x06] = 10; out[0x08] = 12; out[0x09] = 14;
      out[0xf1] = 16; out[0xf8] = 16; out[0xfa] = 5; out[0xfb] = 5; out[0xfc] = 17;
      return out;
    }();
  for (std::size_t at = 4; at < packet.size();) {
    const auto report_size = sizes[packet[at]];
    if (report_size == 0 || at + report_size > packet.size()) {
      break;
    }
    reports.emplace_back(packet[at], std::vector<uint8_t>(
        packet.begin() + at, packet.begin() + at + report_size));
    at += report_size;
  }
  return reports;
}

inline std::vector<double> parse_sensor_report(
  const std::vector<uint8_t> & report, unsigned count, double scale)
{
  if (report.size() < 4 + count * 2) {
    throw std::invalid_argument("short BNO085 sensor report");
  }
  std::vector<double> values;
  values.reserve(count);
  for (unsigned i = 0; i < count; ++i) {
    const auto raw = static_cast<int16_t>(
      static_cast<uint16_t>(report[4 + 2 * i]) |
      static_cast<uint16_t>(report[5 + 2 * i]) << 8);
    values.push_back(raw * scale);
  }
  return values;
}
}  // namespace sandro_hardware::protocol
