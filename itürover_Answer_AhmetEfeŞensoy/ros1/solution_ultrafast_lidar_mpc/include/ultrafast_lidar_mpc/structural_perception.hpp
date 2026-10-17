#pragma once

#include <cstdint>

#include <boost/optional.hpp>

#include "ultrafast_lidar_mpc/config.hpp"
#include "ultrafast_lidar_mpc/types.hpp"

namespace ultrafast_lidar_mpc {

struct NavigationStructure {
  double course_heading{0.0};
  double course_confidence{0.0};
  boost::optional<Gate> active_gate;
  bool gate_crossed{false};
  bool terminal_wall{false};
};

class StructuralPerception {
 public:
  explicit StructuralPerception(const ControllerConfig& config);

  NavigationStructure update(const ScanFrame& frame, const PoseDelta& motion,
                             bool allow_reverse_escape = false);
  void reset();

 private:
  ControllerConfig config_;
  double course_heading_{0.0};
  double course_confidence_{0.0};
  boost::optional<Gate> tracked_gate_;
  std::uint64_t next_gate_id_{1U};
  int rear_gate_streak_{0};
  int terminal_wall_streak_{0};
  bool gate_was_ahead_{false};
  bool crossing_reported_{false};
};

}  // namespace ultrafast_lidar_mpc
