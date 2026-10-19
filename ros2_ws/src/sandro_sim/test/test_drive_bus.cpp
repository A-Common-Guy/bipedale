#include <optional>
#include <vector>

#include "gtest/gtest.h"

#include "sandro_sim/drive_bus.hpp"

namespace sandro_sim {

namespace {
MitCommand torque(const double effort) { return MitCommand{0.0, 0.0, 0.0, 0.0, effort}; }
} // namespace

TEST(DriveBus, NothingIsDueBeforeTheFirstFrame) {
  DriveBus bus;
  bus.configure({"a", "b"}, 0.001, 10.0);
  std::vector<std::optional<MitCommand>> frames;
  bus.due(1.0, frames);
  ASSERT_EQ(frames.size(), 2U);
  EXPECT_FALSE(frames[0]);
  EXPECT_FALSE(frames[1]);
}

TEST(DriveBus, AFrameBecomesDueAfterTheCommandDelayAndIsHeld) {
  DriveBus bus;
  bus.configure({"a"}, 0.001, 10.0);
  std::vector<std::optional<MitCommand>> frames;
  bus.push(0, 0.010, torque(1.0));
  bus.due(0.0105, frames);
  EXPECT_FALSE(frames[0]);  // still on the wire
  bus.due(0.011, frames);
  ASSERT_TRUE(frames[0]);
  EXPECT_DOUBLE_EQ(frames[0]->effort, 1.0);
  bus.due(0.020, frames);  // no newer frame: the drive keeps executing it
  ASSERT_TRUE(frames[0]);
  EXPECT_DOUBLE_EQ(frames[0]->effort, 1.0);
}

TEST(DriveBus, StepsThroughQueuedFramesInOrder) {
  // Two control cycles in flight within one delay: each takes over at its
  // own arrival time, the physics step sees the newest one that has arrived.
  DriveBus bus;
  bus.configure({"a"}, 0.004, 10.0);
  std::vector<std::optional<MitCommand>> frames;
  bus.push(0, 0.0, torque(1.0));
  bus.push(0, 1.0 / 300.0, torque(2.0));
  bus.due(0.005, frames);
  EXPECT_DOUBLE_EQ(frames[0]->effort, 1.0);
  bus.due(0.008, frames);
  EXPECT_DOUBLE_EQ(frames[0]->effort, 2.0);
}

TEST(DriveBus, ClearAndReconfigureForgetFrames) {
  DriveBus bus;
  bus.configure({"a"}, 0.0, 10.0);
  const auto generation = bus.generation();
  std::vector<std::optional<MitCommand>> frames;
  bus.push(0, 0.0, torque(1.0));
  bus.clear();
  bus.due(1.0, frames);
  EXPECT_FALSE(frames[0]);
  bus.push(0, 0.0, torque(1.0));
  bus.configure({"a", "b"}, 0.0, 5.0);
  EXPECT_GT(bus.generation(), generation);
  EXPECT_DOUBLE_EQ(bus.torque_limit_nm(), 5.0);
  bus.due(1.0, frames);
  ASSERT_EQ(frames.size(), 2U);
  EXPECT_FALSE(frames[0]);
}

TEST(DriveBus, IgnoresUnknownJointsAndBoundsTheQueue) {
  DriveBus bus;
  bus.configure({"a"}, 1.0, 10.0);
  bus.push(7, 0.0, torque(1.0));
  for (std::size_t i = 0; i < 2 * DriveBus::kQueueCapacity; ++i) {
    bus.push(0, static_cast<double>(i) * 1e-3, torque(static_cast<double>(i)));
  }
  std::vector<std::optional<MitCommand>> frames;
  bus.due(10.0, frames);
  ASSERT_TRUE(frames[0]);
  EXPECT_DOUBLE_EQ(frames[0]->effort, 2.0 * DriveBus::kQueueCapacity - 1.0);
}

TEST(DriveBus, ReportsTheAttachedPlugin) {
  DriveBus bus;
  EXPECT_FALSE(bus.attached());
  bus.attach();
  bus.configure({"a"}, 0.0, 10.0);
  EXPECT_TRUE(bus.attached());
}

} // namespace sandro_sim
