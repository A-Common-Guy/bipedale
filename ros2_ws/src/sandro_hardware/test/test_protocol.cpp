#include "sandro_hardware/protocol.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <vector>

namespace p = sandro_hardware::protocol;

TEST(CanProtocol, FrameIdentifiers)
{
  EXPECT_EQ(p::can_id(1, p::CanCommand::heartbeat), 0x21);
  EXPECT_EQ(p::can_id(6, p::CanCommand::mit_control), 0xc8);
  EXPECT_THROW(p::can_id(64, p::CanCommand::encoder), std::out_of_range);
}

TEST(CanProtocol, MitPackingMatchesReferenceLayout)
{
  const p::MitLimits limits{};
  const auto packed = p::pack_mit(0.0, 0.0, 0.0, 0.0, 0.0, limits);
  const std::array<uint8_t, 8> expected{0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x08, 0x00};
  EXPECT_EQ(packed, expected);

  const uint8_t feedback[]{3, 0x80, 0x00, 0x80, 0x08, 0x00};
  const auto [node, position, velocity, torque] = p::unpack_mit(feedback, 6, limits);
  EXPECT_EQ(node, 3);
  EXPECT_NEAR(position, 0.0, 2.0 * limits.position / 65535.0);
  EXPECT_NEAR(velocity, 0.0, 2.0 * limits.velocity / 4095.0);
  EXPECT_NEAR(torque, 0.0, 2.0 * limits.torque / 4095.0);
}

TEST(CanProtocol, MitGainsOccupyTheirTwelveBitFields)
{
  // kp is bits 4-15 of bytes 3-4, kd bits 4-15 of bytes 5-6 (each 12 bits,
  // 0..limit). Full-scale gains set every bit of their field and nothing
  // else; the zero position/velocity/torque fields stay at mid-scale.
  const p::MitLimits limits{};
  const auto packed = p::pack_mit(0.0, 0.0, limits.kp, limits.kd, 0.0, limits);
  const std::array<uint8_t, 8> expected{0x80, 0x00, 0x80, 0x0f, 0xff, 0xff, 0xf8, 0x00};
  EXPECT_EQ(packed, expected);
  // Half-scale stiffness, no damping: kp field 0x800.
  const auto half = p::pack_mit(0.0, 0.0, 0.5 * limits.kp, 0.0, 0.0, limits);
  EXPECT_EQ(half[3], 0x08);
  EXPECT_EQ(half[4], 0x00);
  EXPECT_EQ(half[5], 0x00);
  EXPECT_EQ(half[6], 0x08);
  // Gains above the limit are clipped, never wrapped.
  EXPECT_EQ(p::pack_mit(0.0, 0.0, 2.0 * limits.kp, 2.0 * limits.kd, 0.0, limits), expected);
}

TEST(CanProtocol, JointConversionsPreserveMapping)
{
  EXPECT_NEAR(p::logical_position(4.0, -1.0, 8.0, 0.0), -M_PI, 1e-12);
  EXPECT_NEAR(p::logical_velocity(4.0, -1.0, 8.0), -M_PI, 1e-12);
  const double logical = 0.37;
  const double rotor = 1.25;
  const double offset = rotor - logical * 8.0 / p::kTwoPi;
  EXPECT_NEAR(
    p::wire_position(p::logical_position(rotor, 1.0, 8.0, offset), 1.0, 8.0, offset),
    logical + offset / 8.0 * p::kTwoPi, 1e-12);
}

TEST(CanProtocol, MitReplyPositionMatchesTheEncoderConvention)
{
  const double limit = 12.5;
  for (const double sign : {1.0, -1.0}) {
    for (const double offset : {0.0, 0.37, -2.6}) {
      for (const double bias : {0.0, 0.05, -0.12}) {
        const double rotor = 1.9;  // revolutions, as broadcast
        const double logical = p::logical_position(rotor, sign, 8.0, offset);
        // What the drive reports for that pose: the frame commands are sent in.
        const double reply = p::wire_position(logical, sign, 8.0, offset) + bias;
        EXPECT_NEAR(
          p::mit_logical_position(reply, bias, sign, 8.0, offset, limit, logical + 0.3),
          logical, 1e-12) << sign << " " << offset << " " << bias;
        // Without the bias and offset the reply is not the logical position.
        if (bias != 0.0 || offset != 0.0) {
          EXPECT_GT(std::abs(sign * reply - logical), 1e-3);
        }
      }
    }
  }
}

TEST(CanProtocol, MitReplyPositionTakesTheSpanNearestTheBroadcast)
{
  // A wheel 30 rad out on the wire wraps to 30 - 25 in a +-12.5 rad field.
  const double limit = 12.5;
  const double rotor = 30.0 * 8.0 / p::kTwoPi;
  const double logical = p::logical_position(rotor, -1.0, 8.0, 0.0);
  const double reply = 30.0 - 2.0 * limit;
  EXPECT_NEAR(
    p::mit_logical_position(reply, 0.0, -1.0, 8.0, 0.0, limit, logical - 0.5), logical, 1e-12);
  EXPECT_NEAR(
    p::mit_logical_position(reply, 0.0, -1.0, 8.0, 0.0, limit, 0.0), -reply, 1e-12);
}

TEST(CanProtocol, SdoRequestLayout)
{
  const std::array<uint8_t, 8> write{1, 73, 0, 0, 0x2c, 0x01, 0, 0};
  EXPECT_EQ(p::sdo_request(1, 73, 300), write);
  const std::array<uint8_t, 8> read{0, 0x4f, 0x01, 0, 0, 0, 0, 0};
  EXPECT_EQ(p::sdo_request(0, 335), read);
}

TEST(CanProtocol, OnlyQueueFullSendErrorsAreTransient)
{
  EXPECT_TRUE(p::transient_send_error(EAGAIN));
  EXPECT_TRUE(p::transient_send_error(EWOULDBLOCK));
  EXPECT_TRUE(p::transient_send_error(ENOBUFS));
  EXPECT_FALSE(p::transient_send_error(ENETDOWN));
  EXPECT_FALSE(p::transient_send_error(EBADF));
  EXPECT_FALSE(p::transient_send_error(EIO));
}

TEST(CanProtocol, DropBudgetToleratesAShortRunOfLossyCycles)
{
  p::DropBudget budget(3);
  EXPECT_TRUE(budget.end_cycle(true));
  EXPECT_TRUE(budget.end_cycle(true));
  EXPECT_TRUE(budget.end_cycle(false));  // a clean cycle ends the run
  EXPECT_EQ(budget.run(), 0u);
  for (int cycle = 0; cycle < 3; ++cycle) {
    EXPECT_TRUE(budget.end_cycle(true));
  }
  EXPECT_FALSE(budget.end_cycle(true));
  EXPECT_EQ(budget.run(), 4u);
  budget.reset();
  EXPECT_TRUE(budget.end_cycle(true));
}

TEST(ShtpProtocol, EscapingAndChunkedFraming)
{
  const std::vector<uint8_t> payload{0xfd, 0x08, 0x7e, 0x7d};
  const auto frame = p::uart_frame(2, 7, payload);
  EXPECT_EQ(frame.front(), p::kDelimiter);
  EXPECT_EQ(frame[1], 0x01);
  EXPECT_EQ(frame.back(), p::kDelimiter);

  p::ShtpFramer framer;
  EXPECT_TRUE(framer.feed(frame.data(), 4).empty());
  const auto packets = framer.feed(frame.data() + 4, frame.size() - 4);
  ASSERT_EQ(packets.size(), 1u);
  ASSERT_EQ(packets[0].size(), 8u);
  EXPECT_EQ(packets[0][2], 2);
  EXPECT_EQ(packets[0][3], 7);
  EXPECT_EQ(std::vector<uint8_t>(packets[0].begin() + 4, packets[0].end()), payload);
}

TEST(ShtpProtocol, SpiPacketAndSetFeatureLayout)
{
  // Matches deploy/pi/tools/bno085_spi_probe.py: struct.pack("<BBBHIII", 0xFD, id, 0, 0, period, 0, 0).
  const auto command = p::set_feature_command(0x08, 5000);
  const std::vector<uint8_t> expected{
    0xfd, 0x08, 0, 0, 0, 0x88, 0x13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  EXPECT_EQ(command, expected);

  const auto packet = p::shtp_packet(p::kShtpControlChannel, 5, command);
  ASSERT_EQ(packet.size(), 21u);
  EXPECT_EQ(packet[0], 21);
  EXPECT_EQ(packet[1], 0);
  EXPECT_EQ(packet[2], p::kShtpControlChannel);
  EXPECT_EQ(packet[3], 5);
  EXPECT_EQ(p::shtp_length(packet.data()), 21u);

  const uint8_t continued[]{0x15, 0x80, 3, 0};
  EXPECT_EQ(p::shtp_length(continued), 21u);
}

TEST(Protocol, OnlyOperatorInducedAxisErrorsAreClearable)
{
  // 0x104000 is what the drives report after the gamepad E-stop: the
  // requested-E-stop bit plus the CAN watchdog code.
  EXPECT_TRUE(p::operator_clearable_error(0x104000, 0));
  EXPECT_TRUE(p::operator_clearable_error(p::kAxisErrorEstopRequested, 0));
  EXPECT_TRUE(p::operator_clearable_error(p::kAxisErrorCanBusFailed, 0));
  EXPECT_FALSE(p::operator_clearable_error(0, 0));
  EXPECT_FALSE(p::operator_clearable_error(0x104000, 1));  // fault flags set
  EXPECT_FALSE(p::operator_clearable_error(0x104000 | 0x40000, 0));  // over temp
  EXPECT_FALSE(p::operator_clearable_error(0x100, 0));  // encoder failed
}

TEST(ShtpProtocol, SplitsTimestampAndParsesQuaternionInXyzwOrder)
{
  std::vector<uint8_t> packet{21, 0, 3, 0};
  packet.insert(packet.end(), {0xfb, 0, 0, 0, 0});
  packet.insert(packet.end(), {0x08, 0, 0, 0, 0x00, 0x10, 0x00, 0xe0, 0x00, 0x20, 0x00, 0x40});
  const auto reports = p::split_reports(packet);
  ASSERT_EQ(reports.size(), 2u);
  ASSERT_EQ(reports[1].first, 0x08);
  const auto xyzw = p::parse_sensor_report(reports[1].second, 4, std::ldexp(1.0, -14));
  EXPECT_DOUBLE_EQ(xyzw[0], 0.25);
  EXPECT_DOUBLE_EQ(xyzw[1], -0.5);
  EXPECT_DOUBLE_EQ(xyzw[2], 0.5);
  EXPECT_DOUBLE_EQ(xyzw[3], 1.0);
}

TEST(ShtpProtocol, DecodesTheThreeEnabledReports)
{
  std::vector<uint8_t> packet{0, 0, 3, 0};
  packet.insert(packet.end(), {0xfb, 0, 0, 0, 0});
  packet.insert(packet.end(), {0x08, 0, 0, 0, 0x00, 0x10, 0x00, 0xe0, 0x00, 0x20, 0x00, 0x40});
  packet.insert(packet.end(), {0x02, 0, 0, 0, 0x00, 0x02, 0x00, 0xfe, 0x00, 0x04});
  const auto first = p::decode_imu_reports(packet);
  EXPECT_EQ(first.present, (std::array<bool, 3>{true, true, false}));
  EXPECT_DOUBLE_EQ(first.orientation[1], -0.5);
  EXPECT_DOUBLE_EQ(first.angular_velocity[0], 1.0);
  EXPECT_DOUBLE_EQ(first.angular_velocity[1], -1.0);
  EXPECT_DOUBLE_EQ(first.angular_velocity[2], 2.0);

  std::vector<uint8_t> acceleration{0, 0, 3, 0};
  acceleration.insert(acceleration.end(), {0x01, 0, 0, 0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09});
  const auto second = p::decode_imu_reports(acceleration);
  EXPECT_EQ(second.present, (std::array<bool, 3>{false, false, true}));
  EXPECT_DOUBLE_EQ(second.linear_acceleration[2], 9.0);
}

TEST(ShtpProtocol, AgeIsThatOfTheOldestReport)
{
  using Clock = std::chrono::steady_clock;
  const Clock::time_point now{std::chrono::seconds(100)};
  std::array<Clock::time_point, 3> stamps{
    now - std::chrono::milliseconds(2), now - std::chrono::milliseconds(40),
    now - std::chrono::milliseconds(5)};
  EXPECT_NEAR(p::oldest_age(stamps, now), 0.040, 1e-9);
  stamps[2] = Clock::time_point{};
  EXPECT_TRUE(std::isinf(p::oldest_age(stamps, now)));
}

namespace
{
std::array<std::array<double, 3>, 3> rotation(const std::array<double, 4> & q)
{
  const auto [x, y, z, w] = q;
  return {{
    {1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
    {2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
    {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)}}};
}
}  // namespace

TEST(ShtpProtocol, ChipToBodyUndoesAYawedMounting)
{
  // Chip x along body left (the 2026-11 holder): a roll rate about the
  // chip's x is a body pitch rate, gravity on chip -y is body +x.
  const double yaw = M_PI / 2;
  const double n = std::sqrt(0.1 * 0.1 + 0.2 * 0.2 + 0.3 * 0.3 + 0.9 * 0.9);
  std::array<double, 4> q{0.1 / n, -0.2 / n, 0.3 / n, 0.9 / n};
  const auto chip_in_world = rotation(q);
  std::array<double, 3> gyro{1.0, 0.0, 0.5};
  std::array<double, 3> accel{0.0, -9.81, 0.0};
  p::chip_to_body(yaw, q, gyro, accel);
  EXPECT_NEAR(gyro[0], 0.0, 1e-12);
  EXPECT_NEAR(gyro[1], 1.0, 1e-12);
  EXPECT_NEAR(gyro[2], 0.5, 1e-12);
  EXPECT_NEAR(accel[0], 9.81, 1e-12);
  EXPECT_NEAR(accel[1], 0.0, 1e-12);
  // chip in world = body in world * chip in body
  const auto body_in_world = rotation(q);
  const auto chip_in_body = rotation({0.0, 0.0, std::sin(yaw / 2), std::cos(yaw / 2)});
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      double sum = 0.0;
      for (int k = 0; k < 3; ++k) {
        sum += body_in_world[i][k] * chip_in_body[k][j];
      }
      EXPECT_NEAR(sum, chip_in_world[i][j], 1e-12) << i << "," << j;
    }
  }

  std::array<double, 4> same{0.1, 0.2, 0.3, 0.9};
  std::array<double, 3> unchanged{1.0, 2.0, 3.0};
  std::array<double, 3> also{4.0, 5.0, 6.0};
  p::chip_to_body(0.0, same, unchanged, also);
  EXPECT_EQ(same, (std::array<double, 4>{0.1, 0.2, 0.3, 0.9}));
  EXPECT_EQ(unchanged, (std::array<double, 3>{1.0, 2.0, 3.0}));
}
