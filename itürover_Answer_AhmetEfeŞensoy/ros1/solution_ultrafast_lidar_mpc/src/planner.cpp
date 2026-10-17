#include "ultrafast_lidar_mpc/planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace ultrafast_lidar_mpc {
namespace {

constexpr double kPi = 3.14159265358979323846;

double clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

double dot(const Vec2& a, const Vec2& b) {
  return a.x * b.x + a.y * b.y;
}

double trajectoryLengthFrom(const Trajectory& trajectory, std::size_t begin) {
  double length = 0.0;
  for (std::size_t index = begin + 1U; index < trajectory.samples.size(); ++index) {
    const TrajectorySample& a = trajectory.samples[index - 1U];
    const TrajectorySample& b = trajectory.samples[index];
    length += std::hypot(b.x - a.x, b.y - a.y);
  }
  return length;
}

double terminalForwardClearance(const Trajectory& trajectory,
                                const RollingGrid& grid,
                                double required_radius) {
  const TrajectorySample& terminal = trajectory.samples.back();
  const double spacing = 0.5 * grid.resolution();
  constexpr double maximum_probe = 10.0;
  for (double distance = 0.0; distance <= maximum_probe; distance += spacing) {
    const Vec2 point{terminal.x + distance * std::cos(terminal.yaw),
                     terminal.y + distance * std::sin(terminal.yaw)};
    if (grid.clearance(point) <= required_radius) {
      return std::max(0.0, distance - spacing);
    }
  }
  return maximum_probe;
}

bool canStopEverywhere(const Trajectory& trajectory, const RollingGrid& grid,
                       const ControllerConfig& config,
                       double localization_sigma) {
  if (trajectory.samples.empty()) {
    return false;
  }
  const double required_radius = config.robot_radius + config.static_margin +
                                 localization_sigma;
  const double extra = terminalForwardClearance(trajectory, grid, required_radius);
  for (std::size_t index = 0U; index < trajectory.samples.size(); ++index) {
    const double available = trajectoryLengthFrom(trajectory, index) + extra;
    const double required = stoppingDistance(trajectory.samples[index].v,
                                             config.max_linear_decel,
                                             config.max_forward_latency);
    if (required > available + 1e-9) {
      return false;
    }
  }
  return true;
}

struct GateCheck {
  bool valid{true};
  bool crossed{false};
  double center_error{std::numeric_limits<double>::infinity()};
};

GateCheck checkGate(const Trajectory& trajectory, const Gate& gate,
                    const ControllerConfig& config,
                    double localization_sigma) {
  GateCheck result;
  const double normal_length = std::hypot(gate.normal.x, gate.normal.y);
  if (normal_length < 1e-9) {
    result.valid = false;
    return result;
  }
  const Vec2 normal{gate.normal.x / normal_length, gate.normal.y / normal_length};
  const Vec2 tangent{-normal.y, normal.x};
  const double allowed = 0.5 * gate.width - config.robot_radius -
                         config.static_margin - localization_sigma;
  if (allowed <= 0.0) {
    result.valid = false;
    return result;
  }
  const auto signed_plane = [&](const TrajectorySample& sample) {
    return (sample.x - gate.center.x) * normal.x +
           (sample.y - gate.center.y) * normal.y;
  };
  double previous_side = signed_plane(trajectory.samples.front());
  for (std::size_t index = 1U; index < trajectory.samples.size(); ++index) {
    const TrajectorySample& previous = trajectory.samples[index - 1U];
    const TrajectorySample& current = trajectory.samples[index];
    const double current_side = signed_plane(current);
    if (previous_side <= 0.0 && current_side >= 0.0) {
      const double denominator = current_side - previous_side;
      const double ratio = denominator > 1e-12 ? -previous_side / denominator : 0.0;
      const Vec2 crossing{previous.x + ratio * (current.x - previous.x),
                          previous.y + ratio * (current.y - previous.y)};
      result.center_error =
          std::abs(dot(Vec2{crossing.x - gate.center.x,
                            crossing.y - gate.center.y},
                       tangent));
      const double crossing_yaw = wrapAngle(
          previous.yaw + ratio * wrapAngle(current.yaw - previous.yaw));
      const double crossing_speed = previous.v + ratio * (current.v - previous.v);
      const double normal_heading = std::atan2(normal.y, normal.x);
      if (result.center_error > allowed ||
          (crossing_speed > 1.5 &&
           std::abs(wrapAngle(crossing_yaw - normal_heading)) >
               18.0 * kPi / 180.0)) {
        result.valid = false;
      }
      result.crossed = true;
      return result;
    }
    previous_side = current_side;
  }
  const TrajectorySample& terminal = trajectory.samples.back();
  result.center_error =
      std::hypot(terminal.x - gate.center.x, terminal.y - gate.center.y);
  return result;
}

struct RankedCandidate {
  bool valid{false};
  double target_v{0.0};
  double terminal_curvature{0.0};
  int shape{0};
  Trajectory trajectory;
  double progress_rate{-std::numeric_limits<double>::infinity()};
  double gate_error{std::numeric_limits<double>::infinity()};
  double minimum_clearance{0.0};
  double command_change{std::numeric_limits<double>::infinity()};
};

bool better(const RankedCandidate& candidate, const RankedCandidate& incumbent) {
  if (!candidate.valid) {
    return false;
  }
  if (!incumbent.valid) {
    return true;
  }
  constexpr double epsilon = 1e-9;
  if (std::abs(candidate.progress_rate - incumbent.progress_rate) > epsilon) {
    return candidate.progress_rate > incumbent.progress_rate;
  }
  if (std::abs(candidate.gate_error - incumbent.gate_error) > epsilon) {
    return candidate.gate_error < incumbent.gate_error;
  }
  if (candidate.minimum_clearance != incumbent.minimum_clearance) {
    return candidate.minimum_clearance > incumbent.minimum_clearance;
  }
  if (std::abs(candidate.command_change - incumbent.command_change) > epsilon) {
    return candidate.command_change < incumbent.command_change;
  }
  if (std::abs(candidate.terminal_curvature) !=
      std::abs(incumbent.terminal_curvature)) {
    return std::abs(candidate.terminal_curvature) <
           std::abs(incumbent.terminal_curvature);
  }
  if (candidate.target_v != incumbent.target_v) {
    return candidate.target_v < incumbent.target_v;
  }
  return candidate.terminal_curvature < incumbent.terminal_curvature;
}

double directCurvature(const NavigationStructure& structure) {
  if (structure.active_gate) {
    const Vec2& target = structure.active_gate->center;
    const double squared_distance = target.x * target.x + target.y * target.y;
    return squared_distance > 1e-6 ? 2.0 * target.y / squared_distance : 0.0;
  }
  return std::sin(structure.course_heading);
}

}  // namespace

MinimumTimePlanner::MinimumTimePlanner(const ControllerConfig& config)
    : config_(config),
      trajectory_model_(config),
      collision_checker_(config) {
  config_.validate();
}

PlanResult MinimumTimePlanner::plan(const PlannerInput& input) const {
  PlanResult output;
  if (!std::isfinite(input.localization_sigma) || input.localization_sigma < 0.0 ||
      !std::isfinite(input.certified_speed_cap) ||
      input.certified_speed_cap < 0.0) {
    return output;
  }
  const double speed_cap =
      std::min(config_.open_speed_command, input.certified_speed_cap);
  const double current_curvature =
      std::abs(input.motion.v) > 0.10 ? input.motion.w / input.motion.v : 0.0;
  const Vec2 course{std::cos(input.structure.course_heading),
                    std::sin(input.structure.course_heading)};

  RankedCandidate winner;
  const auto evaluate_candidate = [&](double target_v, double terminal_curvature,
                                      int shape, PlannerDiagnostics* diagnostics) {
    RankedCandidate candidate;
    candidate.target_v = clamp(target_v, 0.0, speed_cap);
    candidate.terminal_curvature = terminal_curvature;
    candidate.shape = shape;
    double start_curvature = current_curvature;
    if (shape == 0) {
      start_curvature = terminal_curvature;
    } else if (shape == 2) {
      start_curvature = 0.5 * (current_curvature + terminal_curvature);
    }
    const ControlProfile profile{candidate.target_v, start_curvature,
                                 terminal_curvature};
    candidate.trajectory = trajectory_model_.propagate(
        input.motion, profile, config_.trajectory_horizon);
    ++diagnostics->generated;

    const TrajectorySample& terminal = candidate.trajectory.samples.back();
    const double progress = terminal.x * course.x + terminal.y * course.y;
    candidate.progress_rate = progress / config_.trajectory_horizon;
    if (input.state != RaceState::ESCAPE && progress < -0.02) {
      ++diagnostics->progress_rejections;
      return candidate;
    }
    // Progress is the first lexicographic objective. Once a valid candidate
    // exists, a strictly slower trajectory cannot win regardless of its later
    // tie-break fields, so avoid thousands of clearance queries for it.
    if (winner.valid &&
        candidate.progress_rate < winner.progress_rate - 1e-9) {
      ++diagnostics->dominated_pruned;
      return candidate;
    }

    const CollisionResult collision = collision_checker_.check(
        candidate.trajectory, input.grid, input.localization_sigma);
    if (!collision.collision_free) {
      ++diagnostics->collision_rejections;
      return candidate;
    }
    if (!canStopEverywhere(candidate.trajectory, input.grid, config_,
                           input.localization_sigma)) {
      ++diagnostics->stopping_rejections;
      return candidate;
    }
    if (input.structure.active_gate) {
      const GateCheck gate = checkGate(candidate.trajectory,
                                       *input.structure.active_gate, config_,
                                       input.localization_sigma);
      if (!gate.valid) {
        ++diagnostics->gate_rejections;
        return candidate;
      }
      candidate.gate_error = gate.center_error;
    } else {
      candidate.gate_error = 0.0;
    }
    candidate.minimum_clearance = collision.minimum_clearance;
    const double target_w = candidate.target_v * terminal_curvature;
    candidate.command_change =
        std::hypot(candidate.target_v - speed_cap,
                   target_w - input.motion.w);
    candidate.valid = true;
    return candidate;
  };

  std::array<double, 3U> speed_targets{{speed_cap,
                                        clamp(input.motion.v, 0.0, speed_cap),
                                        0.0}};
  const double direct = directCurvature(input.structure);
  double winner_curvature_step = 0.1;
  double winner_speed_step = std::max(0.05, 0.25 * speed_cap);
  for (double target_v : speed_targets) {
    const double terminal_speed =
        std::min(target_v, input.motion.v +
                               config_.max_linear_accel * config_.trajectory_horizon);
    const double divisor = std::max(0.25, std::abs(terminal_speed));
    const double min_curvature = clamp(
        (input.motion.w - config_.max_angular_accel * config_.trajectory_horizon) /
            divisor,
        -4.0, 4.0);
    const double max_curvature = clamp(
        (input.motion.w + config_.max_angular_accel * config_.trajectory_horizon) /
            divisor,
        -4.0, 4.0);
    const double curvature_step = (max_curvature - min_curvature) / 30.0;
    const auto evaluate_shapes = [&](double curvature) {
      std::array<double, 3U> seen_starts{};
      std::size_t seen_count = 0U;
      for (int shape = 0; shape < 3; ++shape) {
        double start = current_curvature;
        if (shape == 0) {
          start = curvature;
        } else if (shape == 2) {
          start = 0.5 * (current_curvature + curvature);
        }
        bool duplicate = false;
        for (std::size_t index = 0U; index < seen_count; ++index) {
          duplicate = duplicate || std::abs(start - seen_starts[index]) < 1e-12;
        }
        if (duplicate) {
          continue;
        }
        seen_starts[seen_count++] = start;
        const RankedCandidate candidate = evaluate_candidate(
            target_v, curvature, shape, &output.diagnostics);
        if (better(candidate, winner)) {
          winner = candidate;
          winner_curvature_step = std::max(0.01, curvature_step);
        }
      }
    };
    // Direct course/gate solutions are statistically the useful modes and
    // establish a strong bound before scanning the full reachable interval.
    for (int center = -2; center <= 2; ++center) {
      const double curvature = clamp(
          direct + static_cast<double>(center) * std::max(0.02, curvature_step),
          min_curvature, max_curvature);
      evaluate_shapes(curvature);
    }
    const int middle = 15;
    for (int radius = 0; radius <= 15; ++radius) {
      const int lower = middle - radius;
      const int upper = middle + radius;
      if (lower >= 0) {
        evaluate_shapes(min_curvature + static_cast<double>(lower) * curvature_step);
      }
      if (upper <= 30 && upper != lower) {
        evaluate_shapes(min_curvature + static_cast<double>(upper) * curvature_step);
      }
    }
  }

  if (winner.valid) {
    if (winner.target_v <= 1e-9) {
      winner_speed_step = std::max(0.05, 0.25 * speed_cap);
    } else {
      winner_speed_step = std::max(0.05, 0.25 *
          std::max(winner.target_v, speed_cap - winner.target_v));
    }
    const double speed_step = 0.25 * winner_speed_step;
    const double curvature_step = 0.25 * winner_curvature_step;
    const RankedCandidate coarse_winner = winner;
    const std::array<int, 11U> offsets{{0, 1, -1, 2, -2, 3, -3, 4, -4, 5, -5}};
    double previous_speed = -1.0;
    for (const int speed_index : offsets) {
      const double speed = clamp(coarse_winner.target_v +
                                     static_cast<double>(speed_index) * speed_step,
                                 0.0, speed_cap);
      if (std::abs(speed - previous_speed) < 1e-12) {
        continue;
      }
      previous_speed = speed;
      for (const int curvature_index : offsets) {
        const double curvature = coarse_winner.terminal_curvature +
                                 static_cast<double>(curvature_index) *
                                     curvature_step;
        const RankedCandidate candidate = evaluate_candidate(
            speed, curvature, coarse_winner.shape, &output.diagnostics);
        if (better(candidate, winner)) {
          winner = candidate;
        }
      }
    }
  }

  if (!winner.valid) {
    output.command = Command{};
    return output;
  }
  output.valid = true;
  output.command.linear = winner.target_v;
  output.command.angular = winner.target_v * winner.terminal_curvature;
  output.trajectory = winner.trajectory;
  output.certified_speed_cap = winner.target_v;
  output.minimum_clearance = winner.minimum_clearance;
  return output;
}

}  // namespace ultrafast_lidar_mpc
