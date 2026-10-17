#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <random>
#include <vector>

#include "ultrafast_lidar_mpc/scan_matcher.hpp"

namespace ultrafast_lidar_mpc {
namespace {

constexpr std::size_t kRayCount = 360U;
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

ScanFrame rasterize(const std::vector<Vec2>& world_points, double robot_x,
                    double robot_y, double robot_yaw, double noise_std,
                    std::uint32_t seed) {
  ScanFrame frame;
  frame.conservative_ranges.assign(kRayCount,
                                   std::numeric_limits<double>::infinity());
  const double cosine = std::cos(robot_yaw);
  const double sine = std::sin(robot_yaw);
  for (const Vec2& world : world_points) {
    const double wx = world.x - robot_x;
    const double wy = world.y - robot_y;
    const double x = cosine * wx + sine * wy;
    const double y = -sine * wx + cosine * wy;
    const double range = std::hypot(x, y);
    if (range < 0.08 || range > 10.0) {
      continue;
    }
    const double bearing = std::atan2(y, x);
    const std::size_t bin = static_cast<std::size_t>(std::llround(
        (bearing + kPi) * static_cast<double>(kRayCount) / kTwoPi)) %
                            kRayCount;
    frame.conservative_ranges[bin] =
        std::min(frame.conservative_ranges[bin], range);
  }

  std::mt19937 generator(seed);
  std::normal_distribution<double> noise(0.0, noise_std);
  frame.fit_points.reserve(kRayCount);
  frame.collision_points.reserve(kRayCount);
  for (std::size_t bin = 0; bin < kRayCount; ++bin) {
    double& range = frame.conservative_ranges[bin];
    if (!std::isfinite(range)) {
      continue;
    }
    range = std::max(0.08, range + noise(generator));
    const double bearing = -kPi + kTwoPi * static_cast<double>(bin) /
                                      static_cast<double>(kRayCount);
    const ScanPoint point{Vec2{range * std::cos(bearing),
                               range * std::sin(bearing)},
                          range, bearing, true};
    frame.fit_points.push_back(point);
    frame.collision_points.push_back(point);
  }
  return frame;
}

std::vector<Vec2> corridorWorld() {
  std::vector<Vec2> points;
  points.reserve(9000U);
  for (double x = -5.0; x <= 8.0; x += 0.006) {
    points.push_back(Vec2{x, -1.4});
    points.push_back(Vec2{x, 1.4});
  }
  for (double y = -1.4; y <= 1.4; y += 0.004) {
    points.push_back(Vec2{8.0, y});
  }
  for (double y = -0.85; y <= -0.30; y += 0.004) {
    points.push_back(Vec2{3.1 + 0.35 * (y + 0.85), y});
  }
  for (double angle = 0.0; angle < kTwoPi; angle += 0.004) {
    points.push_back(Vec2{5.0 + 0.28 * std::cos(angle),
                          0.65 + 0.28 * std::sin(angle)});
  }
  return points;
}

ScanFrame constantFrame(double range) {
  ScanFrame frame;
  frame.conservative_ranges.assign(kRayCount, range);
  frame.fit_points.reserve(kRayCount);
  frame.collision_points.reserve(kRayCount);
  for (std::size_t bin = 0; bin < kRayCount; ++bin) {
    const double bearing = -kPi + kTwoPi * static_cast<double>(bin) /
                                      static_cast<double>(kRayCount);
    const ScanPoint point{Vec2{range * std::cos(bearing),
                               range * std::sin(bearing)},
                          range, bearing, true};
    frame.fit_points.push_back(point);
    frame.collision_points.push_back(point);
  }
  return frame;
}

ScanFrame radialPatternFrame(double residual, std::size_t exact_rays = 0U) {
  ScanFrame frame;
  frame.conservative_ranges.reserve(kRayCount);
  frame.fit_points.reserve(kRayCount);
  frame.collision_points.reserve(kRayCount);
  for (std::size_t bin = 0; bin < kRayCount; ++bin) {
    const double bearing = -kPi + kTwoPi * static_cast<double>(bin) /
                                      static_cast<double>(kRayCount);
    const double base = 3.0 + 0.45 * std::sin(5.0 * bearing) +
                        0.22 * std::cos(11.0 * bearing);
    const double range = base + (bin < exact_rays ? 0.0 : residual);
    frame.conservative_ranges.push_back(range);
    const ScanPoint point{Vec2{range * std::cos(bearing),
                               range * std::sin(bearing)},
                          range, bearing, true};
    frame.fit_points.push_back(point);
    frame.collision_points.push_back(point);
  }
  return frame;
}

ScanFrame periodicFrame() {
  ScanFrame frame;
  frame.conservative_ranges.reserve(kRayCount);
  frame.fit_points.reserve(kRayCount);
  frame.collision_points.reserve(kRayCount);
  for (std::size_t bin = 0; bin < kRayCount; ++bin) {
    const double bearing = -kPi + kTwoPi * static_cast<double>(bin) /
                                      static_cast<double>(kRayCount);
    const double range = 3.0 + 0.5 * std::cos(24.0 * bearing);
    frame.conservative_ranges.push_back(range);
    const ScanPoint point{Vec2{range * std::cos(bearing),
                               range * std::sin(bearing)},
                          range, bearing, true};
    frame.fit_points.push_back(point);
    frame.collision_points.push_back(point);
  }
  return frame;
}

TEST(ScanMatcher, RecoversNoisyForwardMotion) {
  const std::vector<Vec2> world = corridorWorld();
  const ScanFrame previous = rasterize(world, 0.0, 0.0, 0.0, 0.004, 11U);
  const ScanFrame current = rasterize(world, 0.42, -0.01, 0.03, 0.004, 12U);
  ScanMatcher matcher(ControllerConfig{});

  const PoseDelta delta = matcher.match(previous, current, Command{4.2, 0.3}, 0.1);

  EXPECT_NEAR(delta.dx, 0.42, 0.04);
  EXPECT_NEAR(delta.dy, -0.01, 0.04);
  EXPECT_NEAR(delta.dyaw, 0.03, 0.025);
  EXPECT_GT(delta.confidence, 0.65) << "pose=" << delta.dx << "," << delta.dy
                                    << "," << delta.dyaw;
}

TEST(ScanMatcher, RecoversRotationAndLateralMotion) {
  const std::vector<Vec2> world = corridorWorld();
  const ScanFrame previous = rasterize(world, 0.0, 0.0, 0.0, 0.002, 21U);
  const ScanFrame current = rasterize(world, 0.10, 0.03, -0.14, 0.002, 22U);
  ScanMatcher matcher(ControllerConfig{});

  const PoseDelta delta = matcher.match(previous, current, Command{1.0, -1.4}, 0.1);

  EXPECT_NEAR(delta.dx, 0.10, 0.04);
  EXPECT_NEAR(delta.dy, 0.03, 0.04);
  EXPECT_NEAR(delta.dyaw, -0.14, 0.025);
  EXPECT_GT(delta.confidence, 0.65);
}

TEST(ScanMatcher, LowersConfidenceInFeaturelessCircle) {
  const ScanFrame previous = constantFrame(3.0);
  const ScanFrame current = constantFrame(3.0);
  ScanMatcher matcher(ControllerConfig{});

  const PoseDelta delta = matcher.match(previous, current, Command{}, 0.1);

  EXPECT_LT(delta.confidence, 0.4);
}

TEST(ScanMatcher, RejectsFewerThanSixtyInliers) {
  ScanFrame previous = constantFrame(3.0);
  ScanFrame current = constantFrame(3.0);
  for (std::size_t index = 59U; index < kRayCount; ++index) {
    previous.conservative_ranges[index] = std::numeric_limits<double>::infinity();
    current.conservative_ranges[index] = std::numeric_limits<double>::infinity();
  }
  previous.fit_points.resize(59U);
  previous.collision_points.resize(59U);
  current.fit_points.resize(59U);
  current.collision_points.resize(59U);
  ScanMatcher matcher(ControllerConfig{});

  const PoseDelta delta = matcher.match(previous, current, Command{}, 0.1);

  EXPECT_DOUBLE_EQ(delta.confidence, 0.0);
}

TEST(ScanMatcher, ResetPreservesStatelessEstimate) {
  const std::vector<Vec2> world = corridorWorld();
  const ScanFrame previous = rasterize(world, 0.0, 0.0, 0.0, 0.0, 31U);
  const ScanFrame moved = rasterize(world, 0.32, 0.06, 0.08, 0.0, 32U);
  ScanMatcher matcher(ControllerConfig{});
  const PoseDelta before_reset =
      matcher.match(previous, moved, Command{3.2, 0.8}, 0.1);
  ASSERT_GT(before_reset.confidence, 0.6);

  matcher.reset();
  const PoseDelta after_reset =
      matcher.match(previous, moved, Command{3.2, 0.8}, 0.1);

  EXPECT_DOUBLE_EQ(before_reset.dx, after_reset.dx);
  EXPECT_DOUBLE_EQ(before_reset.dy, after_reset.dy);
  EXPECT_DOUBLE_EQ(before_reset.dyaw, after_reset.dyaw);
  EXPECT_DOUBLE_EQ(before_reset.confidence, after_reset.confidence);
}

TEST(ScanMatcher, IgnoresPointsProjectedBeyondSensorRange) {
  const ScanFrame previous = constantFrame(9.9);
  ScanFrame current;
  current.conservative_ranges.assign(kRayCount,
                                     std::numeric_limits<double>::infinity());
  for (std::size_t index = 0; index < 60U; ++index) {
    const double bearing = -0.05 + 0.10 * static_cast<double>(index) / 59.0;
    constexpr double range = 9.7;
    current.fit_points.push_back(
        ScanPoint{Vec2{range * std::cos(bearing), range * std::sin(bearing)},
                  range, bearing, true});
  }
  ScanMatcher matcher(ControllerConfig{});

  const PoseDelta delta = matcher.match(previous, current, Command{6.0, 0.0}, 0.1);

  EXPECT_DOUBLE_EQ(delta.confidence, 0.0);
}

TEST(ScanMatcher, MediumResidualsContributeToConfidenceRms) {
  const ScanFrame previous = radialPatternFrame(0.0);
  const ScanFrame current = radialPatternFrame(0.12, 24U);
  ScanMatcher matcher(ControllerConfig{});

  const PoseDelta delta = matcher.match(previous, current, Command{}, 0.1);

  EXPECT_LT(delta.confidence, 0.55);
}

TEST(ScanMatcher, RepeatedAngularStructureKeepsAmbiguityPenalty) {
  const ScanFrame repeated = periodicFrame();
  ScanMatcher matcher(ControllerConfig{});

  const PoseDelta delta = matcher.match(repeated, repeated, Command{}, 0.1);

  // With perfect residuals and all rays inliers, the first two confidence
  // terms sum to 0.80. A repeated remote mode must contribute no separation
  // bonus (the remaining 0.20).
  EXPECT_LE(delta.confidence, 0.81);
}

TEST(ScanMatcher, RejectsImplausiblePerScanJump) {
  const std::vector<Vec2> world = corridorWorld();
  const ScanFrame previous = rasterize(world, 0.0, 0.0, 0.0, 0.0, 41U);
  const ScanFrame current = rasterize(world, 1.20, 0.0, 0.0, 0.0, 42U);
  ScanMatcher matcher(ControllerConfig{});

  const PoseDelta delta = matcher.match(previous, current, Command{12.0, 0.0}, 0.1);

  EXPECT_DOUBLE_EQ(delta.dx, 0.0);
  EXPECT_DOUBLE_EQ(delta.dy, 0.0);
  EXPECT_DOUBLE_EQ(delta.dyaw, 0.0);
  EXPECT_DOUBLE_EQ(delta.confidence, 0.0);
}

TEST(ScanMatcher, ScanEvidenceOverridesCommandWithinSearchWindow) {
  const std::vector<Vec2> world = corridorWorld();
  const ScanFrame previous = rasterize(world, 0.0, 0.0, 0.0, 0.001, 51U);
  const ScanFrame current = rasterize(world, 0.22, -0.04, 0.06, 0.001, 52U);
  ScanMatcher matcher(ControllerConfig{});

  const PoseDelta delta = matcher.match(previous, current, Command{4.0, -0.5}, 0.1);

  EXPECT_NEAR(delta.dx, 0.22, 0.04);
  EXPECT_NEAR(delta.dy, -0.04, 0.04);
  EXPECT_NEAR(delta.dyaw, 0.06, 0.025);
}

TEST(ScanMatcher, ProducesDeterministicEstimate) {
  const std::vector<Vec2> world = corridorWorld();
  const ScanFrame previous = rasterize(world, 0.0, 0.0, 0.0, 0.003, 61U);
  const ScanFrame current = rasterize(world, 0.28, 0.02, -0.05, 0.003, 62U);
  ScanMatcher matcher(ControllerConfig{});

  const PoseDelta first = matcher.match(previous, current, Command{2.8, -0.5}, 0.1);
  const PoseDelta second = matcher.match(previous, current, Command{2.8, -0.5}, 0.1);

  EXPECT_DOUBLE_EQ(first.dx, second.dx);
  EXPECT_DOUBLE_EQ(first.dy, second.dy);
  EXPECT_DOUBLE_EQ(first.dyaw, second.dyaw);
  EXPECT_DOUBLE_EQ(first.confidence, second.confidence);
}

}  // namespace
}  // namespace ultrafast_lidar_mpc

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
