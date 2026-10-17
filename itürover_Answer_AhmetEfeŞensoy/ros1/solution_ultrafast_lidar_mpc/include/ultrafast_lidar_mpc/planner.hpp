#pragma once

#include <cstddef>

#include "ultrafast_lidar_mpc/config.hpp"
#include "ultrafast_lidar_mpc/rolling_grid.hpp"
#include "ultrafast_lidar_mpc/structural_perception.hpp"
#include "ultrafast_lidar_mpc/trajectory.hpp"
#include "ultrafast_lidar_mpc/types.hpp"

namespace ultrafast_lidar_mpc {

struct PlannerInput {
  MotionState motion;
  const RollingGrid& grid;
  NavigationStructure structure;
  double localization_sigma{0.0};
  double certified_speed_cap{0.0};
  RaceState state{RaceState::RACE};
};

struct PlannerDiagnostics {
  std::size_t generated{0U};
  std::size_t collision_rejections{0U};
  std::size_t stopping_rejections{0U};
  std::size_t progress_rejections{0U};
  std::size_t gate_rejections{0U};
  std::size_t dominated_pruned{0U};
};

struct PlanResult {
  Command command;
  Trajectory trajectory;
  double certified_speed_cap{0.0};
  double minimum_clearance{0.0};
  bool valid{false};
  PlannerDiagnostics diagnostics;
};

class MinimumTimePlanner {
 public:
  explicit MinimumTimePlanner(const ControllerConfig& config);

  PlanResult plan(const PlannerInput& input) const;

 private:
  ControllerConfig config_;
  TrajectoryModel trajectory_model_;
  SweptCollisionChecker collision_checker_;
};

}  // namespace ultrafast_lidar_mpc
