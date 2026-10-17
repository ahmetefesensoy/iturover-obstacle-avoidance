#include "ultrafast_lidar_mpc/safety_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ultrafast_lidar_mpc {
namespace {

constexpr double kPi = 3.14159265358979323846;

SafetyDecision brakingDecision(SafetyReason reason) {
  SafetyDecision decision;
  decision.command = Command{};
  decision.vetoed = true;
  decision.reason = reason;
  return decision;
}

bool validScan(const ScanFrame& scan) {
  if (scan.conservative_ranges.size() < 180U) {
    return false;
  }
  std::size_t valid_points = 0U;
  for (const ScanPoint& point : scan.collision_points) {
    if (point.raw_valid && std::isfinite(point.range) && point.range > 0.0 &&
        std::isfinite(point.bearing) && std::isfinite(point.p.x) &&
        std::isfinite(point.p.y)) {
      ++valid_points;
    }
  }
  return valid_points >= 30U;
}

bool insideSweptSector(double relative_bearing, double turn, double half_width) {
  if (turn >= 0.0) {
    if (relative_bearing < -half_width &&
        relative_bearing + 2.0 * kPi <= turn + half_width) {
      relative_bearing += 2.0 * kPi;
    }
    return relative_bearing >= -half_width &&
           relative_bearing <= turn + half_width;
  }
  if (relative_bearing > half_width &&
      relative_bearing - 2.0 * kPi >= turn - half_width) {
    relative_bearing -= 2.0 * kPi;
  }
  return relative_bearing <= half_width &&
         relative_bearing >= turn - half_width;
}

double minimumSweptDistance(const Command& command, const MotionState& motion,
                            const ScanFrame& scan,
                            const ControllerConfig& config,
                            double localization_sigma) {
  const double inflated_radius = config.robot_radius + config.static_margin +
                                 localization_sigma;
  const double speed = std::abs(motion.v);
  const double horizon = config.scan_timeout + speed / config.max_linear_decel;
  const double turn = std::max(-kPi, std::min(kPi, command.angular * horizon));
  const double base_heading = command.linear < 0.0 ? kPi : 0.0;
  double minimum = 10.0 - inflated_radius;
  for (const ScanPoint& point : scan.collision_points) {
    if (!point.raw_valid || !std::isfinite(point.range) || point.range <= 0.0 ||
        !std::isfinite(point.bearing)) {
      continue;
    }
    const double half_width =
        std::asin(std::min(1.0, inflated_radius / point.range));
    const double relative = wrapAngle(point.bearing - base_heading);
    if (insideSweptSector(relative, turn, half_width)) {
      minimum = std::min(minimum, point.range - inflated_radius);
    }
  }
  return std::max(0.0, minimum);
}

}  // namespace

SafetyFilter::SafetyFilter(const ControllerConfig& config) : config_(config) {
  config_.validate();
}

SafetyDecision SafetyFilter::filter(const Command& planned,
                                    const MotionState& motion,
                                    const ScanFrame& scan, double now,
                                    double last_scan_time,
                                    double certified_cap,
                                    double localization_sigma) const {
  if (!std::isfinite(now) || !std::isfinite(last_scan_time) ||
      now < last_scan_time || now - last_scan_time > config_.scan_timeout) {
    return brakingDecision(SafetyReason::STALE_SCAN);
  }
  if (!validScan(scan) || !std::isfinite(planned.linear) ||
      !std::isfinite(planned.angular) || !std::isfinite(motion.v) ||
      !std::isfinite(motion.w) || !std::isfinite(motion.confidence) ||
      !std::isfinite(localization_sigma) || localization_sigma < 0.0) {
    return brakingDecision(SafetyReason::INVALID_SCAN);
  }
  if (!std::isfinite(certified_cap) || certified_cap < 0.0) {
    return brakingDecision(SafetyReason::CAP_EXCEEDED);
  }

  const double remaining = minimumSweptDistance(
      planned, motion, scan, config_, localization_sigma);
  const double speed = std::abs(motion.v);
  const double required_to_stop =
      speed * config_.scan_timeout +
      speed * speed / (2.0 * config_.max_linear_decel);
  if (required_to_stop > remaining + 1e-9) {
    return brakingDecision(SafetyReason::UNSTOPPABLE);
  }

  // Solve v*latency + v^2/(2a) <= remaining. The simpler sqrt(2*a*d)
  // bound ignores the missed-frame distance and is unsafe at racing speed.
  const double latency_speed =
      config_.max_linear_decel * config_.scan_timeout;
  const double obstacle_cap =
      -latency_speed +
      std::sqrt(latency_speed * latency_speed +
                2.0 * config_.max_linear_decel * remaining);
  const double final_cap = std::min(certified_cap, obstacle_cap);
  const double requested_magnitude = std::abs(planned.linear);
  if (requested_magnitude > final_cap + 1e-12) {
    SafetyDecision decision;
    decision.command.linear = std::copysign(final_cap, planned.linear);
    decision.command.angular = planned.angular;
    decision.vetoed = true;
    decision.reason = SafetyReason::CAP_EXCEEDED;
    return decision;
  }

  SafetyDecision decision;
  decision.command = planned;
  return decision;
}

}  // namespace ultrafast_lidar_mpc
