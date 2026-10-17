#include "ultrafast_lidar_mpc/scan_matcher.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace ultrafast_lidar_mpc {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kHuberDelta = 0.08;
constexpr double kInlierResidual = 0.20;
constexpr std::size_t kMinimumInliers = 60U;

struct AngularGrid {
  double angle_min{-kPi};
  double increment{kTwoPi};
  std::size_t count{0U};
  bool valid{false};
};

struct Score {
  double cost{std::numeric_limits<double>::infinity()};
  double rms{std::numeric_limits<double>::infinity()};
  double inlier_fraction{0.0};
  std::size_t inliers{0U};
};

struct Candidate {
  double dx{0.0};
  double dy{0.0};
  double dyaw{0.0};
  Score score;
};

double clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

double positiveModulo(double angle) {
  const double wrapped = std::fmod(angle, kTwoPi);
  return wrapped < 0.0 ? wrapped + kTwoPi : wrapped;
}

AngularGrid inferAngularGrid(const ScanFrame& frame) {
  AngularGrid grid;
  grid.count = frame.conservative_ranges.size();
  if (grid.count == 0U) {
    return grid;
  }
  grid.increment = kTwoPi / static_cast<double>(grid.count);

  std::size_t point_index = 0U;
  for (std::size_t bin = 0U; bin < grid.count && point_index < frame.fit_points.size();
       ++bin) {
    if (!std::isfinite(frame.conservative_ranges[bin])) {
      continue;
    }
    const ScanPoint& point = frame.fit_points[point_index++];
    if (std::isfinite(point.bearing)) {
      grid.angle_min = point.bearing - static_cast<double>(bin) * grid.increment;
      grid.valid = true;
      break;
    }
  }
  return grid;
}

std::size_t angularBin(double bearing, const AngularGrid& grid) {
  const double offset = positiveModulo(bearing - grid.angle_min);
  return static_cast<std::size_t>(std::llround(offset / grid.increment)) % grid.count;
}

double huberLoss(double residual) {
  const double magnitude = std::abs(residual);
  if (magnitude <= kHuberDelta) {
    return 0.5 * residual * residual;
  }
  return kHuberDelta * (magnitude - 0.5 * kHuberDelta);
}

double localLineConfidence(const std::vector<ScanPoint>& points, std::size_t center) {
  if (center < 3U || center + 3U >= points.size()) {
    return 0.0;
  }

  double mean_x = 0.0;
  double mean_y = 0.0;
  for (std::size_t index = center - 3U; index <= center + 3U; ++index) {
    if (index > center - 3U &&
        std::hypot(points[index].p.x - points[index - 1U].p.x,
                   points[index].p.y - points[index - 1U].p.y) > 0.35) {
      return 0.0;
    }
    mean_x += points[index].p.x;
    mean_y += points[index].p.y;
  }
  mean_x /= 7.0;
  mean_y /= 7.0;

  double xx = 0.0;
  double xy = 0.0;
  double yy = 0.0;
  for (std::size_t index = center - 3U; index <= center + 3U; ++index) {
    const double x = points[index].p.x - mean_x;
    const double y = points[index].p.y - mean_y;
    xx += x * x;
    xy += x * y;
    yy += y * y;
  }
  const double trace = xx + yy;
  const double determinant = xx * yy - xy * xy;
  const double discriminant = std::sqrt(std::max(0.0, trace * trace - 4.0 * determinant));
  const double lambda_min = 0.5 * (trace - discriminant);
  const double lambda_max = 0.5 * (trace + discriminant);
  return 1.0 - lambda_min / std::max(lambda_max, 1e-12);
}

Score evaluate(const ScanFrame& previous, const ScanFrame& current,
               const AngularGrid& previous_grid,
               const std::vector<double>& point_weights, double dx, double dy,
               double dyaw, std::size_t point_stride) {
  Score score;
  if (!previous_grid.valid || current.fit_points.empty()) {
    return score;
  }

  const double cosine = std::cos(dyaw);
  const double sine = std::sin(dyaw);
  double weighted_loss = 0.0;
  double squared_residual = 0.0;
  double rms_weight = 0.0;
  double inlier_weight = 0.0;
  double possible_weight = 0.0;
  for (std::size_t index = 0U; index < current.fit_points.size();
       index += point_stride) {
    const ScanPoint& point = current.fit_points[index];
    const double x = cosine * point.p.x - sine * point.p.y + dx;
    const double y = sine * point.p.x + cosine * point.p.y + dy;
    const double predicted_range = std::hypot(x, y);
    if (!std::isfinite(predicted_range) || predicted_range <= 0.0) {
      continue;
    }
    const std::size_t bin = angularBin(std::atan2(y, x), previous_grid);
    const double observed_range = previous.conservative_ranges[bin];
    const double weight = point_weights[index];
    possible_weight += weight;
    if (!std::isfinite(observed_range)) {
      continue;
    }

    const double residual = predicted_range - observed_range;
    weighted_loss += weight * huberLoss(residual);
    if (std::abs(residual) <= kInlierResidual) {
      ++score.inliers;
      inlier_weight += weight;
      if (std::abs(residual) <= kHuberDelta) {
        squared_residual += weight * residual * residual;
        rms_weight += weight;
      }
    }
  }

  const std::size_t sampled_minimum =
      (kMinimumInliers + point_stride - 1U) / point_stride;
  if (score.inliers < sampled_minimum || inlier_weight <= 0.0 || possible_weight <= 0.0) {
    return score;
  }
  const double missed_fraction = 1.0 - inlier_weight / possible_weight;
  score.cost = weighted_loss / possible_weight + 0.5 * kHuberDelta * kHuberDelta *
                                                    missed_fraction;
  score.rms = rms_weight > 0.0 ? std::sqrt(squared_residual / rms_weight)
                               : std::numeric_limits<double>::infinity();
  score.inlier_fraction = inlier_weight / possible_weight;
  return score;
}

bool betterCandidate(const Candidate& lhs, const Candidate& rhs) {
  constexpr double kTieTolerance = 1e-15;
  if (lhs.score.cost < rhs.score.cost - kTieTolerance) {
    return true;
  }
  if (std::abs(lhs.score.cost - rhs.score.cost) > kTieTolerance) {
    return false;
  }
  const double lhs_norm = lhs.dx * lhs.dx + lhs.dy * lhs.dy + lhs.dyaw * lhs.dyaw;
  const double rhs_norm = rhs.dx * rhs.dx + rhs.dy * rhs.dy + rhs.dyaw * rhs.dyaw;
  if (lhs_norm != rhs_norm) {
    return lhs_norm < rhs_norm;
  }
  if (lhs.dx != rhs.dx) {
    return lhs.dx < rhs.dx;
  }
  if (lhs.dy != rhs.dy) {
    return lhs.dy < rhs.dy;
  }
  return lhs.dyaw < rhs.dyaw;
}

Candidate searchBox(const ScanFrame& previous, const ScanFrame& current,
                    const AngularGrid& grid,
                    const std::vector<double>& point_weights, double center_dx,
                    double center_dy, double center_yaw, double half_dx,
                    double half_dy, double half_yaw, double step_dx,
                    double step_dy, double step_yaw, std::size_t point_stride,
                    double* second_cost) {
  Candidate best;
  Candidate second;
  const int dx_steps = static_cast<int>(std::ceil(half_dx / step_dx));
  const int dy_steps = static_cast<int>(std::ceil(half_dy / step_dy));
  const int yaw_steps = static_cast<int>(std::ceil(half_yaw / step_yaw));
  for (int ix = -dx_steps; ix <= dx_steps; ++ix) {
    const double dx = center_dx + static_cast<double>(ix) * step_dx;
    for (int iy = -dy_steps; iy <= dy_steps; ++iy) {
      const double dy = center_dy + static_cast<double>(iy) * step_dy;
      for (int it = -yaw_steps; it <= yaw_steps; ++it) {
        const double yaw = center_yaw + static_cast<double>(it) * step_yaw;
        Candidate candidate{dx, dy, yaw,
                            evaluate(previous, current, grid, point_weights, dx,
                                     dy, yaw, point_stride)};
        if (betterCandidate(candidate, best)) {
          second = best;
          best = candidate;
        } else if (betterCandidate(candidate, second)) {
          second = candidate;
        }
      }
    }
  }
  if (second_cost != nullptr) {
    *second_cost = second.score.cost;
  }
  return best;
}

double radialObservability(const ScanFrame& frame) {
  double mean = 0.0;
  double squared = 0.0;
  std::size_t count = 0U;
  for (const double range : frame.conservative_ranges) {
    if (!std::isfinite(range)) {
      continue;
    }
    mean += range;
    squared += range * range;
    ++count;
  }
  if (count < kMinimumInliers) {
    return 0.0;
  }
  mean /= static_cast<double>(count);
  const double variance = std::max(0.0, squared / static_cast<double>(count) - mean * mean);
  const double coefficient = std::sqrt(variance) / std::max(mean, 1e-6);
  return clamp(coefficient / 0.03, 0.0, 1.0);
}

}  // namespace

ScanMatcher::ScanMatcher(const ControllerConfig& config) : config_(config) {
  point_weights_.reserve(720U);
}

PoseDelta ScanMatcher::match(const ScanFrame& previous, const ScanFrame& current,
                             const Command& last_command, double dt) {
  if (!std::isfinite(dt) || dt <= 0.0 || previous.conservative_ranges.empty() ||
      current.fit_points.empty()) {
    return PoseDelta{};
  }

  point_weights_.resize(current.fit_points.size());
  for (std::size_t index = 0U; index < current.fit_points.size(); ++index) {
    point_weights_[index] = localLineConfidence(current.fit_points, index) > 0.8 ? 1.5 : 1.0;
  }

  const AngularGrid grid = inferAngularGrid(previous);
  if (!grid.valid) {
    return PoseDelta{};
  }
  const double prior_dx = std::isfinite(last_command.linear) ? last_command.linear * dt : 0.0;
  const double prior_yaw = std::isfinite(last_command.angular) ? last_command.angular * dt : 0.0;
  const double initial_dx_half = std::max(0.20, 0.45 * std::abs(prior_dx));

  double second_cost = std::numeric_limits<double>::infinity();
  const std::size_t coarse_stride =
      current.fit_points.size() >= 300U
          ? 4U
          : (current.fit_points.size() >= 210U
                 ? 3U
                 : (current.fit_points.size() >= 120U ? 2U : 1U));
  Candidate best = searchBox(previous, current, grid, point_weights_, prior_dx, 0.0,
                             prior_yaw, initial_dx_half, 0.20, 0.30, 0.04, 0.04,
                             0.035, coarse_stride, &second_cost);
  double dx_step = 0.04;
  double dy_step = 0.04;
  double yaw_step = 0.035;
  for (int refinement = 0; refinement < 2; ++refinement) {
    dx_step /= 4.0;
    dy_step /= 4.0;
    yaw_step /= 4.0;
    best = searchBox(previous, current, grid, point_weights_, best.dx, best.dy,
                     best.dyaw, dx_step * 4.0, dy_step * 4.0, yaw_step * 4.0,
                     dx_step, dy_step, yaw_step,
                     coarse_stride > static_cast<std::size_t>(refinement + 1)
                         ? coarse_stride - static_cast<std::size_t>(refinement + 1)
                         : 1U,
                     &second_cost);
  }

  // Search on deterministic subsets, then compute the reported quality from
  // every available fit point. This keeps the bounded search fast without
  // overstating confidence from a subsample.
  best.score = evaluate(previous, current, grid, point_weights_, best.dx, best.dy,
                        best.dyaw, 1U);

  if (!std::isfinite(best.score.cost) || std::hypot(best.dx, best.dy) > 0.9 ||
      std::abs(best.dyaw) > 0.7) {
    return PoseDelta{};
  }

  const double separation =
      std::isfinite(second_cost)
          ? clamp((second_cost - best.score.cost) / std::max(second_cost, 1e-6), 0.0, 1.0)
          : 0.0;
  double confidence = clamp(0.45 * best.score.inlier_fraction +
                                0.35 * std::exp(-best.score.rms / 0.06) +
                                0.20 * separation,
                            0.0, 1.0);
  // A rotationally symmetric scan can have a tiny residual at every yaw. The
  // radial-variation factor makes that unobservable geometry explicit instead
  // of allowing the command-centered search to manufacture confidence.
  confidence *= 0.35 + 0.65 * radialObservability(previous);
  return PoseDelta{best.dx, best.dy, wrapAngle(best.dyaw), confidence};
}

void ScanMatcher::reset() {
  point_weights_.clear();
}

}  // namespace ultrafast_lidar_mpc
