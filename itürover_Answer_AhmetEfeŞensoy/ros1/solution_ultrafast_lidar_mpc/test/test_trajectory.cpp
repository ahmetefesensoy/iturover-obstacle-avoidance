#include <gtest/gtest.h>

#include <cmath>

#include "ultrafast_lidar_mpc/rolling_grid.hpp"
#include "ultrafast_lidar_mpc/trajectory.hpp"

namespace ultrafast_lidar_mpc {
namespace {

ScanFrame pointFrame(const Vec2& point) {
  ScanFrame frame;
  const double range = std::hypot(point.x, point.y);
  frame.collision_points.push_back(
      ScanPoint{point, range, std::atan2(point.y, point.x), true});
  return frame;
}

Trajectory twoSampleTrajectory(const Vec2& start, const Vec2& finish,
                               double lateral_offset = 0.0) {
  Trajectory trajectory;
  trajectory.samples.push_back(
      TrajectorySample{0.0, start.x, start.y + lateral_offset, 0.0, 1.0, 0.0});
  trajectory.samples.push_back(
      TrajectorySample{1.0, finish.x, finish.y + lateral_offset, 0.0, 1.0, 0.0});
  return trajectory;
}

RollingGrid narrowGateGrid(const ControllerConfig& config) {
  RollingGrid grid(config);
  ScanFrame frame;
  constexpr double half_aperture = 0.365 / 2.0;
  for (double x = -0.25; x <= 1.25; x += config.grid_resolution) {
    for (double y = half_aperture; y <= 0.8; y += config.grid_resolution) {
      frame.collision_points.push_back(
          ScanPoint{Vec2{x, y}, std::hypot(x, y), std::atan2(y, x), true});
      frame.collision_points.push_back(
          ScanPoint{Vec2{x, -y}, std::hypot(x, y), std::atan2(-y, x), true});
    }
  }
  grid.update(frame, PoseDelta{}, 0.0);
  return grid;
}

TEST(Trajectory, HonorsOneMeterPerSecondSquaredAcceleration) {
  TrajectoryModel model(ControllerConfig{});
  const Trajectory trajectory = model.propagate(
      MotionState{0.0, 0.0, 1.0}, ControlProfile{6.0, 0.0, 0.0}, 2.0);

  ASSERT_FALSE(trajectory.samples.empty());
  EXPECT_NEAR(trajectory.samples.back().v, 2.0, 0.03);
  EXPECT_NEAR(trajectory.samples.back().x, 2.0, 0.06);
}

TEST(Trajectory, ClampsAngularAcceleration) {
  ControllerConfig config;
  config.max_angular_accel = 1.0;
  TrajectoryModel model(config);

  const Trajectory trajectory = model.propagate(
      MotionState{2.0, 0.0, 1.0}, ControlProfile{2.0, 2.0, 2.0}, 0.5);

  ASSERT_FALSE(trajectory.samples.empty());
  EXPECT_LE(std::abs(trajectory.samples.back().w), 0.5 + 1e-9);
  EXPECT_GT(std::abs(trajectory.samples.back().yaw), 0.10);
}

TEST(Trajectory, DetectsObstacleBetweenIntegrationSamples) {
  const ControllerConfig config;
  RollingGrid grid(config);
  grid.update(pointFrame(Vec2{0.50, 0.0}), PoseDelta{}, 0.0);
  const Trajectory trajectory =
      twoSampleTrajectory(Vec2{0.0, 0.0}, Vec2{1.0, 0.0});

  const CollisionResult result =
      SweptCollisionChecker(config).check(trajectory, grid, 0.0);

  EXPECT_FALSE(result.collision_free);
  // The oriented 0.12 s latency capsule reaches the point before the center.
  EXPECT_NEAR(result.first_collision_time, 0.28, 0.04);
}

TEST(Trajectory, ComputesLatencyAwareStoppingDistance) {
  EXPECT_NEAR(stoppingDistance(4.0, 1.0, 0.12), 8.48, 1e-6);
}

TEST(Trajectory, CenteredPathFitsNarrowGateButShiftedPathDoesNot) {
  const ControllerConfig config;
  const RollingGrid grid = narrowGateGrid(config);
  const Trajectory centered =
      twoSampleTrajectory(Vec2{0.0, 0.0}, Vec2{1.0, 0.0});
  const Trajectory shifted =
      twoSampleTrajectory(Vec2{0.0, 0.0}, Vec2{1.0, 0.0}, 0.09);
  const SweptCollisionChecker checker(config);

  EXPECT_TRUE(checker.check(centered, grid, 0.005).collision_free);
  EXPECT_FALSE(checker.check(shifted, grid, 0.005).collision_free);
}

TEST(Trajectory, TreatsOutsideGridAsCollision) {
  const ControllerConfig config;
  RollingGrid grid(config);
  grid.update(ScanFrame{}, PoseDelta{}, 0.0);
  const Trajectory outside =
      twoSampleTrajectory(Vec2{3.8, 0.0}, Vec2{4.2, 0.0});

  EXPECT_FALSE(SweptCollisionChecker(config).check(outside, grid, 0.0)
                   .collision_free);
}

}  // namespace
}  // namespace ultrafast_lidar_mpc

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
