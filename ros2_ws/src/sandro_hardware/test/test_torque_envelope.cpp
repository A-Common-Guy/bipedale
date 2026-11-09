#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "sandro_hardware/torque_envelope.hpp"

using sandro_hardware::TorqueEnvelope;
using sandro_hardware::TorqueEnvelopeSettings;

namespace
{
constexpr double kCycle = 1.0 / 300.0;
constexpr double kCool = 20.0;

// Seconds at the full limit before it falls below `below` Nm.
double burst_s(TorqueEnvelope & envelope, const double below)
{
  double t = 0.0;
  while (envelope.limit(kCool) >= below && t < 60.0) {
    envelope.account(envelope.limit(kCool), kCycle);
    t += kCycle;
  }
  return t;
}
}  // namespace

TEST(TorqueEnvelope, ColdDriveGivesThePeak)
{
  TorqueEnvelope envelope;
  EXPECT_DOUBLE_EQ(envelope.limit(kCool), 9.0);
  EXPECT_DOUBLE_EQ(envelope.limit(std::numeric_limits<double>::quiet_NaN()), 9.0);
}

// From cold the peak lasts 0.24 time constants (heat 0.7 * 25 of 81), then
// folds back and settles at the rated torque, where heat is rated^2.
TEST(TorqueEnvelope, SustainedLoadFoldsBackToTheRatedTorque)
{
  TorqueEnvelope envelope;
  EXPECT_NEAR(burst_s(envelope, 9.0 - 1e-9), -10.0 * std::log(1.0 - 17.5 / 81.0), 0.02);
  for (int n = 0; n < 300 * 120; ++n) {
    envelope.account(envelope.limit(kCool), kCycle);
  }
  EXPECT_NEAR(envelope.limit(kCool), 5.0, 1e-3);
  EXPECT_NEAR(envelope.heat(), 25.0, 0.01);
}

TEST(TorqueEnvelope, RecoversWhileIdleAndAtTheRatedTorque)
{
  TorqueEnvelope envelope;
  burst_s(envelope, 5.5);
  for (int n = 0; n < 300 * 30; ++n) {
    envelope.account(0.0, kCycle);
  }
  EXPECT_DOUBLE_EQ(envelope.limit(kCool), 9.0);
  // Holding the rated torque forever never drops the limit below it.
  TorqueEnvelope held;
  for (int n = 0; n < 300 * 120; ++n) {
    held.account(5.0, kCycle);
  }
  EXPECT_GE(held.limit(kCool), 5.0);
}

TEST(TorqueEnvelope, HotDriveIsDeratedToTheRatedTorque)
{
  TorqueEnvelope envelope;
  EXPECT_DOUBLE_EQ(envelope.limit(65.0), 9.0);
  EXPECT_DOUBLE_EQ(envelope.limit(72.5), 7.0);
  EXPECT_DOUBLE_EQ(envelope.limit(80.0), 5.0);
  EXPECT_DOUBLE_EQ(envelope.limit(95.0), 5.0);
}

TEST(TorqueEnvelope, RejectsInconsistentSettings)
{
  TorqueEnvelopeSettings s;
  EXPECT_TRUE(s.valid());
  s.rated_nm = 10.0;
  EXPECT_FALSE(s.valid());
  s = {};
  s.knee = 1.0;
  EXPECT_FALSE(s.valid());
  s = {};
  s.derate_end_c = s.derate_start_c;
  EXPECT_FALSE(s.valid());
}
