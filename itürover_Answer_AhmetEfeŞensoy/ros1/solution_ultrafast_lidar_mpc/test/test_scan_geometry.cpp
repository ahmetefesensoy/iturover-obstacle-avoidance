#include <algorithm>
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "synthetic_scan.hpp"
#include "ultrafast_lidar_mpc/scan_geometry.hpp"

namespace ultrafast_lidar_mpc {
namespace {

TEST(ScanGeometry, PreservesCloseSingleRayForCollision) {
  auto msg = test::SyntheticScan::constant(5.0);
  msg.ranges[180] = 0.40F;
  ScanGeometry g(ControllerConfig{});
  const ScanFrame f = g.condition(msg);
  EXPECT_NEAR(f.conservative_ranges[180], 0.40, 1e-7);
  EXPECT_TRUE(std::any_of(f.collision_points.begin(), f.collision_points.end(),
                          [](const ScanPoint& point) {
                            return point.range < 0.41;
                          }));
}

TEST(ScanGeometry, MedianOnlyAffectsFitStream) {
  auto msg = test::SyntheticScan::constant(2.0);
  msg.ranges[100] = 0.30F;
  const ScanFrame f = ScanGeometry(ControllerConfig{}).condition(msg);
  EXPECT_NEAR(f.conservative_ranges[100], 0.30, 1e-7);
  ASSERT_GT(f.fit_points.size(), 100U);
  EXPECT_GT(f.fit_points[100].range, 1.9);
}

TEST(ScanGeometry, FitsNoisyWall) {
  const auto points = test::SyntheticScan::linePoints(-1.0, 0.01, 80);
  const LineModel line = ScanGeometry::fitLine(points);
  EXPECT_GT(line.confidence, 0.9);
  EXPECT_NEAR(std::abs(line.normal.x), 1.0, 0.02);
  EXPECT_NEAR(std::abs(line.offset), 1.0, 0.02);
}

TEST(ScanGeometry, DetectsWraparoundRangeDiscontinuity) {
  auto msg = test::SyntheticScan::constant(3.0);
  msg.ranges.front() = 0.5F;
  const ScanFrame frame = ScanGeometry(ControllerConfig{}).condition(msg);
  const auto discontinuities = ScanGeometry::rangeDiscontinuities(frame, 1.0);
  EXPECT_NE(std::find(discontinuities.begin(), discontinuities.end(), 359U),
            discontinuities.end());
}

TEST(ScanGeometry, OmitsInvalidRaysFromBothPointStreams) {
  auto msg = test::SyntheticScan::constant(2.0);
  msg.ranges[10] = std::numeric_limits<float>::infinity();
  const ScanFrame frame = ScanGeometry(ControllerConfig{}).condition(msg);
  EXPECT_TRUE(std::isinf(frame.conservative_ranges[10]));
  EXPECT_EQ(frame.collision_points.size(), 359U);
  EXPECT_EQ(frame.fit_points.size(), 359U);
}

}  // namespace
}  // namespace ultrafast_lidar_mpc

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
