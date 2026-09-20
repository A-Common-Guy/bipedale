#include <cmath>
#include <limits>

#include "gtest/gtest.h"

#include "sandro_sim/mit_drive.hpp"

namespace sandro_sim {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

TEST(MitDrive, ComputesTheDriveLaw) {
  const MitCommand command{1.0, 0.5, 12.0, 0.5, 0.3};
  // 12 * (1.0 - 0.8) + 0.5 * (0.5 - 2.0) + 0.3
  EXPECT_NEAR(mit_torque(command, 0.8, 2.0, 10.0), 2.4 - 0.75 + 0.3, 1e-12);
}

TEST(MitDrive, UnsetFieldsMakeATorqueSource) {
  // A controller that only claims effort leaves the other fields NaN.
  const MitCommand effort_only{kNaN, kNaN, kNaN, kNaN, 0.7};
  EXPECT_DOUBLE_EQ(mit_torque(effort_only, 3.0, -4.0, 10.0), 0.7);
  // Unset position with a stiffness: hold the measured position (no step).
  const MitCommand hold{kNaN, kNaN, 20.0, 0.5, 0.0};
  EXPECT_NEAR(mit_torque(hold, 0.4, 1.0, 10.0), -0.5, 1e-12);
  // Negative or NaN gains count as zero; NaN effort as zero.
  const MitCommand junk{1.0, 1.0, -5.0, kNaN, kNaN};
  EXPECT_DOUBLE_EQ(mit_torque(junk, 0.0, 0.0, 10.0), 0.0);
}

TEST(MitDrive, ClipsToTheTorqueLimit) {
  const MitCommand stiff{1.0, 0.0, 500.0, 0.0, 0.0};
  EXPECT_DOUBLE_EQ(mit_torque(stiff, 0.0, 0.0, 10.0), 10.0);
  EXPECT_DOUBLE_EQ(mit_torque(stiff, 2.0, 0.0, 10.0), -10.0);
  // A non-positive or NaN limit means no clipping.
  EXPECT_DOUBLE_EQ(mit_torque(stiff, 0.0, 0.0, 0.0), 500.0);
}

} // namespace sandro_sim
