#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

#include <sensor_msgs/LaserScan.h>

#include "ultrafast_lidar_mpc/types.hpp"

namespace ultrafast_lidar_mpc {
namespace test {

class SyntheticScan {
 public:
  static sensor_msgs::LaserScan constant(double range) {
    sensor_msgs::LaserScan scan;
    scan.angle_min = -static_cast<float>(M_PI);
    scan.angle_increment = static_cast<float>(2.0 * M_PI / 360.0);
    scan.angle_max = scan.angle_min + 359.0F * scan.angle_increment;
    scan.range_min = 0.05F;
    scan.range_max = 10.0F;
    scan.ranges.assign(360, static_cast<float>(range));
    return scan;
  }

  static std::vector<Vec2> linePoints(double x_offset, double noise,
                                      std::size_t count) {
    std::vector<Vec2> points;
    points.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
      const double y = -1.0 + 2.0 * static_cast<double>(index) /
                                  static_cast<double>(count - 1U);
      const double x = x_offset + noise * std::sin(static_cast<double>(index));
      points.push_back(Vec2{x, y});
    }
    return points;
  }
};

}  // namespace test
}  // namespace ultrafast_lidar_mpc
