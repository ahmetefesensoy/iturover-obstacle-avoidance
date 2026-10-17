#pragma once

#include <cstddef>
#include <limits>

#include <boost/container/static_vector.hpp>

#include "ultrafast_lidar_mpc/config.hpp"
#include "ultrafast_lidar_mpc/rolling_grid.hpp"
#include "ultrafast_lidar_mpc/types.hpp"

namespace ultrafast_lidar_mpc {

struct ControlProfile {
  double target_v{0.0};
  double start_curvature{0.0};
  double end_curvature{0.0};
};

struct TrajectorySample {
  double t{0.0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double v{0.0};
  double w{0.0};
};

struct Trajectory {
  static constexpr std::size_t kMaximumSamples = 256U;
  boost::container::static_vector<TrajectorySample, kMaximumSamples> samples;
};

struct CollisionResult {
  bool collision_free{true};
  double first_collision_time{std::numeric_limits<double>::infinity()};
  double minimum_clearance{std::numeric_limits<double>::infinity()};
};

double stoppingDistance(double speed, double deceleration,
                        double forward_latency_seconds);

class TrajectoryModel {
 public:
  explicit TrajectoryModel(const ControllerConfig& config);

  Trajectory propagate(const MotionState& initial,
                       const ControlProfile& profile, double horizon) const;

 private:
  ControllerConfig config_;
};

class SweptCollisionChecker {
 public:
  explicit SweptCollisionChecker(const ControllerConfig& config);

  CollisionResult check(const Trajectory& trajectory, const RollingGrid& grid,
                        double localization_sigma) const;

 private:
  double base_radius_{0.0};
  double forward_latency_seconds_{0.0};
};

}  // namespace ultrafast_lidar_mpc
