#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>
#include <utility>

#include <gtest/gtest.h>

#include "synthetic_scan.hpp"
#include "ultrafast_lidar_mpc/scan_geometry.hpp"

namespace ultrafast_lidar_mpc {
namespace {

template <typename T, typename = void>
struct HasSingleArgumentCondition : std::false_type {};

template <typename T>
struct HasSingleArgumentCondition<
    T, decltype(void(std::declval<const T&>().condition(
           std::declval<const sensor_msgs::LaserScan&>())))> : std::true_type {};

static_assert(!HasSingleArgumentCondition<ScanGeometry>::value,
              "Scan conditioning must use caller-owned ScanFrame storage");

TEST(ScanGeometry, PreservesCloseSingleRayForCollision) {
  auto msg = test::SyntheticScan::constant(5.0);
  msg.ranges[180] = 0.40F;
  ScanGeometry g(ControllerConfig{});
  ScanFrame f;
  g.condition(msg, f);
  EXPECT_NEAR(f.conservative_ranges[180], 0.40, 1e-7);
  EXPECT_TRUE(std::any_of(f.collision_points.begin(), f.collision_points.end(),
                          [](const ScanPoint& point) {
                            return point.range < 0.41;
                          }));
}

TEST(ScanGeometry, MedianOnlyAffectsFitStream) {
  auto msg = test::SyntheticScan::constant(2.0);
  msg.ranges[100] = 0.30F;
  ScanFrame f;
  ScanGeometry(ControllerConfig{}).condition(msg, f);
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
  ScanFrame frame;
  ScanGeometry(ControllerConfig{}).condition(msg, frame);
  const auto discontinuities = ScanGeometry::rangeDiscontinuities(frame, 1.0);
  EXPECT_NE(std::find(discontinuities.begin(), discontinuities.end(), 359U),
            discontinuities.end());
}

TEST(ScanGeometry, OmitsInvalidRaysFromBothPointStreams) {
  auto msg = test::SyntheticScan::constant(2.0);
  msg.ranges[10] = std::numeric_limits<float>::infinity();
  ScanFrame frame;
  ScanGeometry(ControllerConfig{}).condition(msg, frame);
  EXPECT_TRUE(std::isinf(frame.conservative_ranges[10]));
  EXPECT_EQ(frame.collision_points.size(), 359U);
  EXPECT_EQ(frame.fit_points.size(), 359U);
}

TEST(ScanGeometry, ExcludesOneInvalidNeighborFromMedian) {
  auto msg = test::SyntheticScan::constant(9.0);
  msg.ranges[99] = 1.0F;
  msg.ranges[100] = 4.0F;
  msg.ranges[101] = std::numeric_limits<float>::infinity();
  ScanFrame frame;
  ScanGeometry(ControllerConfig{}).condition(msg, frame);
  EXPECT_NEAR(frame.fit_points[100].range, 2.5, 1e-7);
  EXPECT_NEAR(frame.collision_points[100].range, 2.5, 1e-7);
}

TEST(ScanGeometry, ExcludesBothInvalidNeighborsFromMedian) {
  auto msg = test::SyntheticScan::constant(9.0);
  msg.ranges[99] = std::numeric_limits<float>::infinity();
  msg.ranges[100] = 4.0F;
  msg.ranges[101] = std::numeric_limits<float>::infinity();
  ScanFrame frame;
  ScanGeometry(ControllerConfig{}).condition(msg, frame);
  ASSERT_EQ(frame.fit_points.size(), 358U);
  ASSERT_EQ(frame.collision_points.size(), 358U);
  EXPECT_NEAR(frame.fit_points[99].range, 4.0, 1e-7);
  EXPECT_NEAR(frame.collision_points[99].range, 4.0, 1e-7);
}

TEST(ScanGeometry, ReusesPreallocatedFrameStorageAcrossScans) {
  const auto msg = test::SyntheticScan::constant(2.0);
  ScanGeometry geometry(ControllerConfig{});
  ScanFrame reusable_frame;
  geometry.condition(msg, reusable_frame);
  const std::size_t range_capacity = reusable_frame.conservative_ranges.capacity();
  const std::size_t collision_capacity = reusable_frame.collision_points.capacity();
  const std::size_t fit_capacity = reusable_frame.fit_points.capacity();

  for (int iteration = 0; iteration < 10; ++iteration) {
    geometry.condition(msg, reusable_frame);
  }

  EXPECT_EQ(reusable_frame.conservative_ranges.capacity(), range_capacity);
  EXPECT_EQ(reusable_frame.collision_points.capacity(), collision_capacity);
  EXPECT_EQ(reusable_frame.fit_points.capacity(), fit_capacity);
}

}  // namespace
}  // namespace ultrafast_lidar_mpc

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
