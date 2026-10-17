#pragma once

#include <limits>

#include <sensor_msgs/LaserScan.h>

#include "ultrafast_lidar_mpc/config.hpp"
#include "ultrafast_lidar_mpc/planner.hpp"
#include "ultrafast_lidar_mpc/rolling_grid.hpp"
#include "ultrafast_lidar_mpc/safety_filter.hpp"
#include "ultrafast_lidar_mpc/scan_geometry.hpp"
#include "ultrafast_lidar_mpc/scan_matcher.hpp"
#include "ultrafast_lidar_mpc/structural_perception.hpp"
#include "ultrafast_lidar_mpc/types.hpp"

namespace ultrafast_lidar_mpc {

struct ControllerOutput {
  Command command;
  RaceState state{RaceState::LAUNCH};
  SafetyReason safety_reason{SafetyReason::NONE};
  double certified_speed_cap{0.0};
  double localization_sigma{0.0};
  bool plan_valid{false};
  bool gate_locked{false};
  double estimated_velocity{0.0};
  double match_confidence{0.0};
  double gate_id{0.0};
  double gate_confidence{0.0};
  double minimum_clearance{0.0};
  double candidate_count{0.0};
  double planner_microseconds{0.0};
};

class RaceController {
 public:
  explicit RaceController(const ControllerConfig& config);

  ControllerOutput onScan(const sensor_msgs::LaserScan& scan);
  ControllerOutput onControlTick(double now);
  void reset();

  RaceState state() const { return state_; }

 private:
  ControllerConfig config_;
  ScanGeometry geometry_;
  ScanMatcher matcher_;
  RollingGrid grid_;
  StructuralPerception perception_;
  MinimumTimePlanner planner_;
  SafetyFilter safety_filter_;
  ScanFrame previous_frame_;
  ScanFrame current_frame_;
  bool have_previous_scan_{false};
  double last_scan_time_{std::numeric_limits<double>::quiet_NaN()};
  Command last_command_;
  MotionState motion_;
  RaceState state_{RaceState::LAUNCH};
  int low_match_streak_{0};
  int no_plan_streak_{0};
  int recovery_streak_{0};
  int gate_lock_scans_{0};
  PlanResult cached_plan_;
  ControllerOutput cached_output_;
};

}  // namespace ultrafast_lidar_mpc
