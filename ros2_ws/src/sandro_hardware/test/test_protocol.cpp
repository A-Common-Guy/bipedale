#include "sandro_hardware/protocol.hpp"

#include <gtest/gtest.h>

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
  // Matches raspi_tests/bno085_spi.py: struct.pack("<BBBHIII", 0xFD, id, 0, 0, period, 0, 0).
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
