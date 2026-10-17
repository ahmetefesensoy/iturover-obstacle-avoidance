#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "ultrafast_lidar_mpc/planner.hpp"

namespace ultrafast_lidar_mpc {
namespace {

RollingGrid gridWithPoints(const ControllerConfig& config,
                           const std::vector<Vec2>& points) {
  RollingGrid grid(config);
  ScanFrame frame;
  for (const Vec2& point : points) {
    frame.collision_points.push_back(
        ScanPoint{point, std::hypot(point.x, point.y),
                  std::atan2(point.y, point.x), true});
  }
  grid.update(frame, PoseDelta{}, 0.0);
  return grid;
}

NavigationStructure straightStructure() {
  NavigationStructure structure;
  structure.course_heading = 0.0;
  structure.course_confidence = 1.0;
  return structure;
}

TEST(Planner, RejectsFasterCollidingCandidate) {
  ControllerConfig config;
  config.grid_size = 20.0;
  std::vector<Vec2> blocking_wall;
  for (double y = -1.0; y <= 1.0; y += config.grid_resolution) {
    blocking_wall.push_back(Vec2{1.15, y});
  }
  const RollingGrid grid = gridWithPoints(config, blocking_wall);
  const PlannerInput input{MotionState{1.0, 0.0, 1.0}, grid,
                           straightStructure(), 0.01, 6.0, RaceState::RACE};

  const PlanResult result = MinimumTimePlanner(config).plan(input);

  ASSERT_TRUE(result.valid);
  EXPECT_LT(result.command.linear, 2.0);
  EXPECT_GT(result.minimum_clearance,
            config.robot_radius + config.static_margin + input.localization_sigma);
  EXPECT_GT(result.diagnostics.collision_rejections, 0U);
}

TEST(Planner, AcceleratesAtPhysicalLimitInOpenCorridor) {
  ControllerConfig config;
  config.grid_size = 20.0;
  RollingGrid grid(config);
  grid.update(ScanFrame{}, PoseDelta{}, 0.0);
  const PlannerInput input{MotionState{1.0, 0.0, 1.0}, grid,
                           straightStructure(), 0.01, 6.0, RaceState::RACE};

  const PlanResult result = MinimumTimePlanner(config).plan(input);

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.command.linear, 6.0, 1e-9);
  ASSERT_FALSE(result.trajectory.samples.empty());
  EXPECT_GT(result.trajectory.samples.back().v, 2.9);
  EXPECT_NEAR(result.trajectory.samples.back().y, 0.0, 0.02);
}

TEST(Planner, AimsThroughLockedGateCenterAndNormal) {
  ControllerConfig config;
  config.grid_size = 20.0;
  RollingGrid grid(config);
  grid.update(ScanFrame{}, PoseDelta{}, 0.0);
  NavigationStructure structure = straightStructure();
  Gate gate;
  gate.id = 1U;
  gate.left_post = Vec2{1.5, 0.31};
  gate.right_post = Vec2{1.5, -0.09};
  gate.center = Vec2{1.5, 0.11};
  gate.normal = Vec2{1.0, 0.0};
  gate.width = 0.40;
  gate.confidence = 0.95;
  structure.active_gate = gate;
  const PlannerInput input{MotionState{1.0, 0.0, 1.0}, grid, structure,
                           0.005, 4.0, RaceState::RACE};

  const PlanResult result = MinimumTimePlanner(config).plan(input);

  ASSERT_TRUE(result.valid);
  double crossing_error = std::numeric_limits<double>::infinity();
  for (const TrajectorySample& sample : result.trajectory.samples) {
    if (sample.x >= gate.center.x) {
      crossing_error = std::abs(sample.y - gate.center.y);
      break;
    }
  }
  EXPECT_LT(crossing_error, 0.07);
}

TEST(Planner, FullyBlockedStartReturnsInvalidZeroCommand) {
  ControllerConfig config;
  std::vector<Vec2> ring;
  for (double angle = -M_PI; angle < M_PI; angle += 0.04) {
    ring.push_back(Vec2{0.08 * std::cos(angle), 0.08 * std::sin(angle)});
  }
  const RollingGrid grid = gridWithPoints(config, ring);
  const PlannerInput input{MotionState{0.0, 0.0, 1.0}, grid,
                           straightStructure(), 0.0, 6.0, RaceState::RACE};

  const PlanResult result = MinimumTimePlanner(config).plan(input);

  EXPECT_FALSE(result.valid);
  EXPECT_DOUBLE_EQ(result.command.linear, 0.0);
  EXPECT_DOUBLE_EQ(result.command.angular, 0.0);
}

TEST(Planner, DeterministicallyPrefersSmallestCommandChangeOnTie) {
  ControllerConfig config;
  config.grid_size = 12.0;
  RollingGrid grid(config);
  grid.update(ScanFrame{}, PoseDelta{}, 0.0);
  const PlannerInput input{MotionState{1.0, 0.0, 1.0}, grid,
                           straightStructure(), 0.0, 1.0, RaceState::RACE};
  MinimumTimePlanner planner(config);

  const PlanResult first = planner.plan(input);
  const PlanResult second = planner.plan(input);

  ASSERT_TRUE(first.valid && second.valid);
  EXPECT_DOUBLE_EQ(first.command.linear, second.command.linear);
  EXPECT_DOUBLE_EQ(first.command.angular, second.command.angular);
  EXPECT_NEAR(first.command.angular, 0.0, 1e-9);
}

}  // namespace
}  // namespace ultrafast_lidar_mpc

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
