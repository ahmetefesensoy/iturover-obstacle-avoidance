#pragma once

#include <cstddef>
#include <vector>

#include <sensor_msgs/LaserScan.h>

#include "ultrafast_lidar_mpc/config.hpp"
#include "ultrafast_lidar_mpc/types.hpp"

namespace ultrafast_lidar_mpc {

struct LineModel {
  Vec2 normal;
  double offset{0.0};
  double confidence{0.0};
};

class ScanGeometry {
 public:
  explicit ScanGeometry(const ControllerConfig& config);

  ScanFrame condition(const sensor_msgs::LaserScan& scan) const;
  void condition(const sensor_msgs::LaserScan& scan, ScanFrame& frame) const;
  static LineModel fitLine(const std::vector<Vec2>& points);
  static std::vector<std::size_t> rangeDiscontinuities(const ScanFrame& frame,
                                                        double threshold);

 private:
  ControllerConfig config_;
};

}  // namespace ultrafast_lidar_mpc
