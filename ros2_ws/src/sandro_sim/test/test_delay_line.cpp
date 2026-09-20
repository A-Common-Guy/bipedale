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

TEST(DelayLine, ResetForgetsHistory) {
  DelayLine line;
  line.configure(0.010, 0.0);
  line.update(0.0, 1.0);
  line.update(0.02, 2.0);
  line.reset();
  EXPECT_DOUBLE_EQ(line.update(1.0, 5.0), 5.0);
}

} // namespace sandro_sim
