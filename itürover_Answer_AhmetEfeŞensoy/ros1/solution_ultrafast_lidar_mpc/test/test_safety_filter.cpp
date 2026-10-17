#include <gtest/gtest.h>

#include <cmath>

#include "ultrafast_lidar_mpc/safety_filter.hpp"

namespace ultrafast_lidar_mpc {
namespace {

ScanFrame openFrame() {
  ScanFrame frame;
  frame.conservative_ranges.assign(360U, 10.0);
  frame.collision_points.reserve(360U);
  for (std::size_t bin = 0U; bin < 360U; ++bin) {
    const double bearing = -M_PI + 2.0 * M_PI * static_cast<double>(bin) / 360.0;
    frame.collision_points.push_back(
        ScanPoint{Vec2{10.0 * std::cos(bearing), 10.0 * std::sin(bearing)},
                  10.0, bearing, true});
  }
  return frame;
}

ScanFrame singleHitFrame(double x, double y) {
  ScanFrame frame = openFrame();
  const double range = std::hypot(x, y);
  const double bearing = std::atan2(y, x);
  frame.collision_points.push_back(ScanPoint{Vec2{x, y}, range, bearing, true});
  return frame;
}

TEST(SafetyFilter, NeverAcceleratesOnStaleScan) {
  SafetyFilter filter(ControllerConfig{});

  const SafetyDecision decision = filter.filter(
      Command{6.0, 0.0}, MotionState{2.0, 0.0, 0.9}, openFrame(),
      1.0, 0.70, 6.0);

  EXPECT_TRUE(decision.vetoed);
  EXPECT_EQ(decision.reason, SafetyReason::STALE_SCAN);
  EXPECT_LE(decision.command.linear, 2.0);
}

TEST(SafetyFilter, BrakesWhenOneMissedFrameMakesStateUnstoppable) {
  const ScanFrame scan = singleHitFrame(2.0, 0.0);

  const SafetyDecision decision = SafetyFilter(ControllerConfig{}).filter(
      Command{6.0, 0.0}, MotionState{2.1, 0.0, 0.9}, scan,
      1.0, 0.90, 6.0);

  EXPECT_DOUBLE_EQ(decision.command.linear, 0.0);
  EXPECT_TRUE(decision.vetoed);
  EXPECT_EQ(decision.reason, SafetyReason::UNSTOPPABLE);
}

TEST(SafetyFilter, InvalidScanCommandsMaximumBraking) {
  const SafetyDecision decision = SafetyFilter(ControllerConfig{}).filter(
      Command{3.0, 1.0}, MotionState{1.0, 0.0, 0.8}, ScanFrame{},
      1.0, 1.0, 6.0);

  EXPECT_EQ(decision.reason, SafetyReason::INVALID_SCAN);
  EXPECT_DOUBLE_EQ(decision.command.linear, 0.0);
  EXPECT_DOUBLE_EQ(decision.command.angular, 0.0);
}

TEST(SafetyFilter, ClampsToCertifiedAndObstacleCaps) {
  const ScanFrame scan = singleHitFrame(4.0, 0.0);
  const SafetyDecision decision = SafetyFilter(ControllerConfig{}).filter(
      Command{6.0, 0.2}, MotionState{1.0, 0.0, 0.9}, scan,
      1.0, 0.95, 3.0, 0.02);

  EXPECT_TRUE(decision.vetoed);
  EXPECT_EQ(decision.reason, SafetyReason::CAP_EXCEEDED);
  EXPECT_LE(decision.command.linear, 3.0);
  EXPECT_GT(decision.command.linear, 0.0);
}

TEST(SafetyFilter, FreshOpenScanPreservesPlannerCommandExactly) {
  const Command planned{3.0, -0.4};

  const SafetyDecision decision = SafetyFilter(ControllerConfig{}).filter(
      planned, MotionState{2.0, -0.2, 0.95}, openFrame(),
      1.0, 0.95, 4.0);

  EXPECT_FALSE(decision.vetoed);
  EXPECT_EQ(decision.reason, SafetyReason::NONE);
  EXPECT_DOUBLE_EQ(decision.command.linear, planned.linear);
  EXPECT_DOUBLE_EQ(decision.command.angular, planned.angular);
}

}  // namespace
}  // namespace ultrafast_lidar_mpc

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
