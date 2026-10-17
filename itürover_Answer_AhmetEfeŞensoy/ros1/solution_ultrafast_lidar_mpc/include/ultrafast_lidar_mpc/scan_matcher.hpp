#pragma once

#include <vector>

#include "ultrafast_lidar_mpc/config.hpp"
#include "ultrafast_lidar_mpc/types.hpp"

namespace ultrafast_lidar_mpc {

class ScanMatcher {
 public:
  explicit ScanMatcher(const ControllerConfig& config);

  PoseDelta match(const ScanFrame& previous, const ScanFrame& current,
                  const Command& last_command, double dt);
  void reset();

 private:
  std::vector<double> point_weights_;
};

}  // namespace ultrafast_lidar_mpc
