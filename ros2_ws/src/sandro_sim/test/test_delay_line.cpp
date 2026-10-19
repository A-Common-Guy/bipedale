#include <cmath>

#include "gtest/gtest.h"

#include "sandro_sim/delay_line.hpp"

namespace sandro_sim {

TEST(DelayLine, PassThroughWhenUnconfigured) {
  DelayLine line;
  line.configure(0.0, 0.0);
  EXPECT_TRUE(line.passthrough());
  EXPECT_DOUBLE_EQ(line.update(0.0, 1.5), 1.5);
  EXPECT_DOUBLE_EQ(line.update(0.01, -2.0), -2.0);
}

TEST(DelayLine, DelaysARampByTheConfiguredTime) {
  DelayLine line;
  line.configure(0.010, 0.0);
  const double dt = 1.0 / 300.0;
  double out = 0.0;
  for (int i = 0; i <= 90; ++i) {
    const double t = i * dt;
    out = line.update(t, t);  // value equals its own timestamp
  }
  // Output is the newest sample at least 10 ms old: within one step of t-0.01.
  EXPECT_LE(out, 90 * dt - 0.010 + 1e-12);
  EXPECT_GE(out, 90 * dt - 0.010 - dt - 1e-12);
}

TEST(DelayLine, HoldsBetweenRefreshesAtTheSampleRate) {
  DelayLine line;
  line.configure(0.0, 100.0);  // refresh every 10 ms, no delay
  const double dt = 1.0 / 300.0;
  int changes = 0;
  double previous = line.update(0.0, 0.0);
  for (int i = 1; i <= 300; ++i) {
    const double t = i * dt;
    const double out = line.update(t, t);
    if (out != previous) {
      ++changes;
    }
    previous = out;
  }
  // 1 s at 100 Hz refresh: about 100 changes, not 300.
  EXPECT_GE(changes, 95);
  EXPECT_LE(changes, 101);
}

TEST(DelayLine, HoldsFirstSampleUntilOldEnough) {
  DelayLine line;
  line.configure(0.020, 0.0);
  EXPECT_DOUBLE_EQ(line.update(0.0, 7.0), 7.0);
  EXPECT_DOUBLE_EQ(line.update(0.005, 8.0), 7.0);
  EXPECT_DOUBLE_EQ(line.update(0.010, 9.0), 7.0);
  EXPECT_DOUBLE_EQ(line.update(0.021, 10.0), 7.0);
  EXPECT_DOUBLE_EQ(line.update(0.026, 11.0), 8.0);
}

TEST(DelayLine, RefreshesOnTheSensorClockNotTheConsumers) {
  // A 200 Hz sensor read by a 300 Hz consumer is fresh in two cycles out of
  // three; refreshing on the first read a period after the previous refresh
  // would give 150 Hz.
  DelayLine line;
  line.configure(0.0, 200.0);
  const double dt = 1.0 / 300.0;
  int changes = 0;
  double previous = line.update(0.0, 0.0);
  for (int i = 1; i <= 300; ++i) {
    const double t = i * dt;
    const double out = line.update(t, t);
    if (out != previous) {
      ++changes;
    }
    previous = out;
  }
  EXPECT_GE(changes, 198);
  EXPECT_LE(changes, 201);
}

TEST(DelayLine, DeliversTheInterpolatedSampleAfterTheTransportDelay) {
  // 200 Hz samples at k * 5 ms, each arriving 1 ms later: a consumer sees
  // the exact value at the newest sample instant that has arrived, so the
  // data is between 1 and 6 ms old, never older.
  DelayLine line;
  line.configure(0.001, 200.0, 0.0, true);
  const double dt = 1.0 / 300.0;
  for (int i = 0; i <= 300; ++i) {
    const double t = i * dt;
    const double out = line.update(t, t);  // value equals its own timestamp
    if (t < 0.011) {
      continue;  // history still filling
    }
    const double expected = std::floor((t - 0.001) / 0.005 + 1e-9) * 0.005;
    EXPECT_NEAR(out, expected, 1e-9) << "t=" << t;
    EXPECT_GE(t - out, 0.001 - 1e-9);
    EXPECT_LE(t - out, 0.006 + 1e-9);
  }
}

TEST(DelayLine, PhaseShiftsTheSampleGrid) {
  DelayLine line;
  line.configure(0.0, 200.0, 0.002, true);
  const double dt = 1.0 / 300.0;
  for (int i = 0; i <= 120; ++i) {
    const double t = i * dt;
    const double out = line.update(t, t);
    if (t < 0.01) {
      continue;
    }
    const double expected = 0.002 + std::floor((t - 0.002) / 0.005 + 1e-9) * 0.005;
    EXPECT_NEAR(out, expected, 1e-9) << "t=" << t;
  }
}

TEST(DelayLine, ZeroOrderHoldKeepsCommandsPiecewiseConstant) {
  // Without interpolation a sample instant between two updates reads the
  // older value, as a command frame on the bus does.
  DelayLine line;
  line.configure(0.0, 200.0, 0.0025);
  line.update(0.0, 1.0);
  EXPECT_DOUBLE_EQ(line.update(0.004, 2.0), 1.0);  // sample at 2.5 ms
}

TEST(DelayLine, ResetForgetsHistory) {
  DelayLine line;
  line.configure(0.010, 0.0);
  line.update(0.0, 1.0);
  line.update(0.02, 2.0);
  line.reset();
  EXPECT_DOUBLE_EQ(line.update(1.0, 5.0), 5.0);
}

} // namespace sandro_sim
