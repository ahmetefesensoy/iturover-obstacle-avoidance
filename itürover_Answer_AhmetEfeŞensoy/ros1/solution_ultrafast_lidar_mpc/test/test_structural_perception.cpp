#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "synthetic_scan.hpp"
#include "ultrafast_lidar_mpc/structural_perception.hpp"

namespace ultrafast_lidar_mpc {
namespace {

using test::SyntheticScan;

TEST(StructuralPerception, FindsNarrowGateCenterNotDeepestRay) {
  const ScanFrame frame =
      SyntheticScan::wallWithGap(1.20, -0.18, 0.20, 6.0, -0.10);
  StructuralPerception perception(ControllerConfig{});

  const NavigationStructure structure = perception.update(frame, PoseDelta{});

  ASSERT_TRUE(static_cast<bool>(structure.active_gate));
  EXPECT_NEAR(structure.active_gate->width, 0.38, 0.025);
  EXPECT_NEAR(structure.active_gate->center.y, 0.01, 0.02);
  EXPECT_NEAR(structure.active_gate->normal.y, 0.0, 0.08);
  EXPECT_GT(structure.active_gate->confidence, 0.65);
}

TEST(StructuralPerception, KeepsGateIdentityThroughPartialOcclusion) {
  StructuralPerception perception(ControllerConfig{});
  const NavigationStructure first = perception.update(
      SyntheticScan::wallWithGap(1.2, -0.2, 0.2, 5.0, 0.0), PoseDelta{});
  const NavigationStructure second = perception.update(
      SyntheticScan::partiallyOccludedGate(1.0, -0.2, 0.2),
      PoseDelta{0.2, 0.0, 0.0, 0.9});

  ASSERT_TRUE(first.active_gate && second.active_gate);
  EXPECT_EQ(first.active_gate->id, second.active_gate->id);
  EXPECT_NEAR(second.active_gate->center.x, 1.0, 0.08);
}

TEST(StructuralPerception, SingleDeepRayDoesNotClearLockedGate) {
  StructuralPerception perception(ControllerConfig{});
  const NavigationStructure locked = perception.update(
      SyntheticScan::wallWithGap(1.2, -0.2, 0.2, 5.0, 0.0), PoseDelta{});
  ASSERT_TRUE(locked.active_gate);
  ScanFrame single_ray;
  single_ray.conservative_ranges.assign(
      360U, std::numeric_limits<double>::infinity());
  single_ray.conservative_ranges[180U] = 8.0;
  const ScanPoint point{Vec2{8.0, 0.0}, 8.0, 0.0, true};
  single_ray.fit_points.push_back(point);
  single_ray.collision_points.push_back(point);

  const NavigationStructure retained =
      perception.update(single_ray, PoseDelta{});

  ASSERT_TRUE(retained.active_gate);
  EXPECT_EQ(retained.active_gate->id, locked.active_gate->id);
}

TEST(StructuralPerception, RejectsRearOpeningAsCourseDirection) {
  StructuralPerception perception(ControllerConfig{});

  const NavigationStructure structure =
      perception.update(SyntheticScan::frontAndRearCorridors(), PoseDelta{});

  EXPECT_LT(std::abs(wrapAngle(structure.course_heading)), 0.35);
  EXPECT_GT(structure.course_confidence, 0.70);
}

TEST(StructuralPerception, RequiresTwoRearScansToConfirmGateCrossing) {
  StructuralPerception perception(ControllerConfig{});
  const ScanFrame gate =
      SyntheticScan::wallWithGap(0.30, -0.2, 0.2, 4.0, 0.0);
  ASSERT_TRUE(perception.update(gate, PoseDelta{}).active_gate);

  const NavigationStructure first_rear =
      perception.update(ScanFrame{}, PoseDelta{0.45, 0.0, 0.0, 0.9});
  const NavigationStructure second_rear =
      perception.update(ScanFrame{}, PoseDelta{0.05, 0.0, 0.0, 0.9});

  EXPECT_FALSE(first_rear.gate_crossed);
  EXPECT_TRUE(second_rear.gate_crossed);
}

TEST(StructuralPerception, RequiresThreeConsecutiveTerminalWallScans) {
  StructuralPerception perception(ControllerConfig{});
  const ScanFrame wall = SyntheticScan::solidWall(1.3);

  EXPECT_FALSE(perception.update(wall, PoseDelta{}).terminal_wall);
  EXPECT_FALSE(perception.update(wall, PoseDelta{}).terminal_wall);
  EXPECT_TRUE(perception.update(wall, PoseDelta{}).terminal_wall);
}

TEST(StructuralPerception, PassableOpeningResetsTerminalWallPersistence) {
  StructuralPerception perception(ControllerConfig{});
  const ScanFrame wall = SyntheticScan::solidWall(1.3);
  const ScanFrame opening =
      SyntheticScan::wallWithGap(1.3, -0.25, 0.25, 5.0, 0.0);
  EXPECT_FALSE(perception.update(wall, PoseDelta{}).terminal_wall);
  EXPECT_FALSE(perception.update(wall, PoseDelta{}).terminal_wall);
  EXPECT_FALSE(perception.update(opening, PoseDelta{}).terminal_wall);
  EXPECT_FALSE(perception.update(wall, PoseDelta{}).terminal_wall);
  EXPECT_FALSE(perception.update(wall, PoseDelta{}).terminal_wall);
  EXPECT_TRUE(perception.update(wall, PoseDelta{}).terminal_wall);
}

}  // namespace
}  // namespace ultrafast_lidar_mpc

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
