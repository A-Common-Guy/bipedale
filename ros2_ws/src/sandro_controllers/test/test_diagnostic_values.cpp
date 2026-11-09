#include <gtest/gtest.h>

#include <cstdlib>
#include <new>
#include <string>
#include <vector>

#include "sandro_controllers/diagnostic_values.hpp"

namespace sc = sandro_controllers;

namespace {

std::size_t g_allocations = 0;

struct KeyValue {
  std::string key;
  std::string value;
};

struct Status {
  std::vector<KeyValue> values;
};

void fill(Status &status, const double x) {
  sc::DiagnosticValues<Status> values(status);
  values.add("pitch_reference_rad_long_key", x);
  values.add("status_mask", 5U);
  values.add("engaged", true);
  values.add("action_0", 0.25F);
  values.finish();
}

}  // namespace

void *operator new(std::size_t size) {
  ++g_allocations;
  if (void *p = std::malloc(size)) {
    return p;
  }
  throw std::bad_alloc();
}
void operator delete(void *p) noexcept { std::free(p); }
void operator delete(void *p, std::size_t) noexcept { std::free(p); }

TEST(DiagnosticValues, FormatsCompactlyAndAllocatesOnlyForTheFirstMessage) {
  Status status;
  fill(status, -2.5e-7);
  ASSERT_EQ(status.values.size(), 4U);
  EXPECT_EQ(status.values[0].key, "pitch_reference_rad_long_key");
  EXPECT_EQ(status.values[0].value, "-2.5e-07");  // std::to_string gave -0.000000
  EXPECT_EQ(status.values[1].value, "5");
  EXPECT_EQ(status.values[2].value, "1");
  EXPECT_EQ(status.values[3].value, "0.25");
  g_allocations = 0;
  for (int i = 0; i < 100; ++i) {
    fill(status, 0.1 * i);
  }
  EXPECT_EQ(g_allocations, 0U);
  EXPECT_EQ(status.values[0].value, "9.9");
}

TEST(DiagnosticValues, AShorterMessageDropsTheStaleSlots) {
  Status status;
  fill(status, 1.0);
  {
    sc::DiagnosticValues<Status> values(status);
    values.add("only", 2.0);
    values.finish();
  }
  ASSERT_EQ(status.values.size(), 1U);
  EXPECT_EQ(status.values[0].key, "only");
}

TEST(RateGate, KeepsTheAverageRateOnACoarseLoop) {
  sc::RateGate gate;
  int published = 0;
  for (int cycle = 0; cycle < 300 * 10; ++cycle) {  // 10 s at 300 Hz
    const double now = cycle / 300.0;
    if (gate.due(now)) {
      gate.commit(now, 50.0);
      ++published;
    }
  }
  EXPECT_NEAR(published, 500, 1);  // the old gate gave 43 Hz
  // After a long gap it does not burst to catch up.
  sc::RateGate late;
  late.commit(0.0, 10.0);
  EXPECT_FALSE(late.due(0.05));
  ASSERT_TRUE(late.due(5.0));
  late.commit(5.0, 10.0);
  EXPECT_FALSE(late.due(5.0 + 1.0 / 300.0));
}
