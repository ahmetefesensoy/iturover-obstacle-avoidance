#include "ultrafast_lidar_mpc/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ultrafast_lidar_mpc {
namespace {

double moveToward(double value, double target, double positive_step,
                  double negative_step) {
  if (target > value) {
    return std::min(target, value + positive_step);
  }
  return std::max(target, value - negative_step);
}

double smoothStep(double ratio) {
  const double clamped = std::max(0.0, std::min(1.0, ratio));
  return clamped * clamped * (3.0 - 2.0 * clamped);
}

}  // namespace

double stoppingDistance(double speed, double deceleration,
                        double forward_latency_seconds) {
  if (!std::isfinite(speed) || !std::isfinite(deceleration) ||
      !std::isfinite(forward_latency_seconds) || deceleration <= 0.0 ||
      forward_latency_seconds < 0.0) {
    throw std::invalid_argument("invalid stopping-distance input");
  }
  const double forward_speed = std::max(0.0, speed);
  return forward_speed * forward_latency_seconds +
         forward_speed * forward_speed / (2.0 * deceleration);
}

TrajectoryModel::TrajectoryModel(const ControllerConfig& config) : config_(config) {
  config_.validate();
}

Trajectory TrajectoryModel::propagate(const MotionState& initial,
                                      const ControlProfile& profile,
                                      double horizon) const {
  if (!std::isfinite(horizon) || horizon < 0.0 ||
      !std::isfinite(initial.v) || !std::isfinite(initial.w) ||
      !std::isfinite(profile.target_v) ||
      !std::isfinite(profile.start_curvature) ||
      !std::isfinite(profile.end_curvature)) {
    throw std::invalid_argument("invalid trajectory input");
  }
  const std::size_t steps = static_cast<std::size_t>(
      std::ceil(horizon / config_.integration_dt));
  if (steps + 1U > Trajectory::kMaximumSamples) {
    throw std::length_error("trajectory horizon exceeds fixed capacity");
  }

  Trajectory trajectory;
  TrajectorySample current{0.0, 0.0, 0.0, 0.0, initial.v, initial.w};
  trajectory.samples.push_back(current);
  while (current.t + 1e-12 < horizon) {
    const double dt = std::min(config_.integration_dt, horizon - current.t);
    const double next_t = current.t + dt;
    const double next_v = moveToward(
        current.v, profile.target_v, config_.max_linear_accel * dt,
        config_.max_linear_decel * dt);
    const double blend = horizon > 0.0 ? smoothStep(next_t / horizon) : 1.0;
    const double curvature = profile.start_curvature +
                             blend * (profile.end_curvature -
                                      profile.start_curvature);
    const double desired_w = curvature * next_v;
    const double next_w = moveToward(
        current.w, desired_w, config_.max_angular_accel * dt,
        config_.max_angular_accel * dt);

    const double integration_v = 0.5 * (current.v + next_v);
    const double integration_w = 0.5 * (current.w + next_w);
    double local_x = integration_v * dt;
    double local_y = 0.0;
    if (std::abs(integration_w) > 1e-6) {
      const double delta_yaw = integration_w * dt;
      local_x = integration_v / integration_w * std::sin(delta_yaw);
      local_y = integration_v / integration_w * (1.0 - std::cos(delta_yaw));
    }
    const double cosine = std::cos(current.yaw);
    const double sine = std::sin(current.yaw);
    TrajectorySample next;
    next.t = next_t;
    next.x = current.x + cosine * local_x - sine * local_y;
    next.y = current.y + sine * local_x + cosine * local_y;
    next.yaw = wrapAngle(current.yaw + integration_w * dt);
    next.v = next_v;
    next.w = next_w;
    trajectory.samples.push_back(next);
    current = next;
  }
  return trajectory;
}

SweptCollisionChecker::SweptCollisionChecker(const ControllerConfig& config)
    : base_radius_(config.robot_radius + config.static_margin),
      forward_latency_seconds_(config.max_forward_latency) {
  config.validate();
}

CollisionResult SweptCollisionChecker::check(
    const Trajectory& trajectory, const RollingGrid& grid,
    double localization_sigma) const {
  CollisionResult result;
  if (!std::isfinite(localization_sigma) || localization_sigma < 0.0 ||
      trajectory.samples.empty()) {
    result.collision_free = false;
    result.first_collision_time = 0.0;
    result.minimum_clearance = 0.0;
    return result;
  }
  const double required_radius = base_radius_ + localization_sigma;
  const double sample_spacing = 0.5 * grid.resolution();

  const auto inspect_pose = [&](double x, double y, double yaw, double v,
                                double time, CollisionResult* output) {
    const double extension = std::max(0.0, v) * forward_latency_seconds_;
    const std::size_t capsule_steps = std::max<std::size_t>(
        1U, static_cast<std::size_t>(std::ceil(extension / sample_spacing)));
    for (std::size_t step = 0U; step <= capsule_steps; ++step) {
      const double forward = extension * static_cast<double>(step) /
                             static_cast<double>(capsule_steps);
      const Vec2 point{x + forward * std::cos(yaw),
                       y + forward * std::sin(yaw)};
      const double available = grid.clearance(point);
      output->minimum_clearance =
          std::min(output->minimum_clearance, available);
      if (available <= required_radius && output->collision_free) {
        output->collision_free = false;
        output->first_collision_time = time;
      }
    }
  };

  if (trajectory.samples.size() == 1U) {
    const TrajectorySample& sample = trajectory.samples.front();
    inspect_pose(sample.x, sample.y, sample.yaw, sample.v, sample.t, &result);
    return result;
  }

  for (std::size_t index = 1U; index < trajectory.samples.size(); ++index) {
    const TrajectorySample& a = trajectory.samples[index - 1U];
    const TrajectorySample& b = trajectory.samples[index];
    const double segment_length = std::hypot(b.x - a.x, b.y - a.y);
    const std::size_t segment_steps = std::max<std::size_t>(
        1U, static_cast<std::size_t>(std::ceil(segment_length / sample_spacing)));
    const double yaw_delta = wrapAngle(b.yaw - a.yaw);
    for (std::size_t step = index == 1U ? 0U : 1U; step <= segment_steps;
         ++step) {
      const double ratio = static_cast<double>(step) /
                           static_cast<double>(segment_steps);
      inspect_pose(a.x + ratio * (b.x - a.x),
                   a.y + ratio * (b.y - a.y),
                   wrapAngle(a.yaw + ratio * yaw_delta),
                   a.v + ratio * (b.v - a.v),
                   a.t + ratio * (b.t - a.t), &result);
    }
  }
  return result;
}

}  // namespace ultrafast_lidar_mpc
