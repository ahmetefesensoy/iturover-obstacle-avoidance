#include <gtest/gtest.h>

#include <stdexcept>

#include "ultrafast_lidar_mpc/config.hpp"
#include "ultrafast_lidar_mpc/types.hpp"

namespace ultrafast_lidar_mpc {
namespace {

TEST(Config, AcceptsRaceDefaults) {
  ControllerConfig c;
  EXPECT_NO_THROW(c.validate());
  EXPECT_DOUBLE_EQ(c.robot_radius, 0.105);
  EXPECT_DOUBLE_EQ(c.max_linear_accel, 1.0);
  EXPECT_DOUBLE_EQ(c.open_speed_command, 6.0);
}

TEST(Config, RejectsUnsafeOrImpossibleRanges) {
  ControllerConfig c;
  c.robot_radius = 0.0;
  EXPECT_THROW(c.validate(), std::invalid_argument);
  c = ControllerConfig{};
  c.control_rate_hz = 5.0;
  EXPECT_THROW(c.validate(), std::invalid_argument);
  c = ControllerConfig{};
  c.max_linear_decel = -1.0;
  EXPECT_THROW(c.validate(), std::invalid_argument);
}

TEST(Config, RejectsInvalidSafetyTimingAndGridValues) {
  ControllerConfig c;
  c.scan_timeout = 0.14;
  EXPECT_THROW(c.validate(), std::invalid_argument);
  c = ControllerConfig{};
  c.grid_resolution = 0.051;
  EXPECT_THROW(c.validate(), std::invalid_argument);
  c = ControllerConfig{};
  c.integration_dt = 0.051;
  EXPECT_THROW(c.validate(), std::invalid_argument);
  c = ControllerConfig{};
  c.gate_width_min = 0.61;
  EXPECT_THROW(c.validate(), std::invalid_argument);
  c = ControllerConfig{};
  c.open_speed_command = 0.99;
  EXPECT_THROW(c.validate(), std::invalid_argument);
}

TEST(Types, WrapsAnglesToPrincipalRange) {
  EXPECT_DOUBLE_EQ(wrapAngle(0.0), 0.0);
  EXPECT_NEAR(wrapAngle(3.0 * M_PI), -M_PI, 1e-12);
  EXPECT_NEAR(wrapAngle(-3.0 * M_PI), M_PI, 1e-12);
}

}  // namespace
}  // namespace ultrafast_lidar_mpc

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
