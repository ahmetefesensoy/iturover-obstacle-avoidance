#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>

#include "ultrafast_lidar_mpc/rolling_grid.hpp"

namespace ultrafast_lidar_mpc {
namespace {

ScanFrame emptyFrame() {
  return ScanFrame{};
}

ScanFrame singleHitFrame(double x, double y) {
  ScanFrame frame;
  const double range = std::hypot(x, y);
  const double bearing = std::atan2(y, x);
  const ScanPoint point{Vec2{x, y}, range, bearing, true};
  frame.collision_points.push_back(point);
  frame.fit_points.push_back(point);
  return frame;
}

TEST(RollingGrid, WarpsOldHitsIntoCurrentRobotFrame) {
  RollingGrid grid(ControllerConfig{});
  grid.update(singleHitFrame(2.0, 0.0), PoseDelta{}, 0.0);
  grid.update(emptyFrame(), PoseDelta{0.5, 0.0, 0.0, 1.0}, 0.1);

  EXPECT_TRUE(grid.occupied(Vec2{1.5, 0.0}));
  EXPECT_FALSE(grid.occupied(Vec2{2.0, 0.0}));
}

TEST(RollingGrid, InverseRotatesOldHitsIntoCurrentRobotFrame) {
  RollingGrid grid(ControllerConfig{});
  grid.update(singleHitFrame(1.0, 0.0), PoseDelta{}, 0.0);
  grid.update(emptyFrame(), PoseDelta{0.0, 0.0, M_PI_2, 1.0}, 0.1);

  EXPECT_LT(grid.clearance(Vec2{0.0, -1.0}), 0.04);
  EXPECT_FALSE(grid.occupied(Vec2{1.0, 0.0}));
}

TEST(RollingGrid, ExpiresHistoryAfterConfiguredLifetime) {
  RollingGrid grid(ControllerConfig{});
  grid.update(singleHitFrame(1.0, 0.0), PoseDelta{}, 0.0);
  grid.update(emptyFrame(), PoseDelta{}, 0.81);

  EXPECT_FALSE(grid.occupied(Vec2{1.0, 0.0}));
}

TEST(RollingGrid, ClearanceMatchesNearestOccupiedCell) {
  RollingGrid grid(ControllerConfig{});
  grid.update(singleHitFrame(1.0, 0.0), PoseDelta{}, 0.0);

  EXPECT_NEAR(grid.clearance(Vec2{0.5, 0.0}), 0.5, 0.04);
  EXPECT_DOUBLE_EQ(grid.clearance(Vec2{4.1, 0.0}), 0.0);
}

TEST(RollingGrid, ResetClearsOccupancy) {
  RollingGrid grid(ControllerConfig{});
  grid.update(singleHitFrame(1.0, 0.0), PoseDelta{}, 0.0);
  ASSERT_TRUE(grid.occupied(Vec2{1.0, 0.0}));

  grid.reset();

  EXPECT_FALSE(grid.occupied(Vec2{1.0, 0.0}));
}

TEST(RollingGrid, ReusesAllGridStorageAcrossUpdates) {
  RollingGrid grid(ControllerConfig{});
  const auto initial_capacity = grid.capacitySignature();
  for (std::size_t index = 0U; index < 100U; ++index) {
    const double x = 1.0 + 0.001 * static_cast<double>(index);
    grid.update(singleHitFrame(x, 0.2),
                PoseDelta{0.01, 0.0, 0.001, 1.0},
                0.01 * static_cast<double>(index));
  }

  EXPECT_EQ(grid.capacitySignature(), initial_capacity);
}

}  // namespace
}  // namespace ultrafast_lidar_mpc

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
