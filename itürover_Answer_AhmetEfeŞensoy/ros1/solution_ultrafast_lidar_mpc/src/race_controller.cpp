#include "ultrafast_lidar_mpc/race_controller.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace ultrafast_lidar_mpc {
namespace {

double dot(const Vec2& a, const Vec2& b) {
  return a.x * b.x + a.y * b.y;
}

bool validConditionedScan(const ScanFrame& frame) {
  return frame.conservative_ranges.size() >= 180U &&
         frame.collision_points.size() >= 30U;
}

double certifiedCapFromScan(const ScanFrame& frame,
                            const ControllerConfig& config,
                            double localization_sigma) {
  double forward_range = 10.0;
  for (const ScanPoint& point : frame.collision_points) {
    if (!std::isfinite(point.range) || std::abs(point.bearing) > 0.35) {
      continue;
    }
    forward_range = std::min(forward_range, point.range);
  }
  const double remaining = std::max(
      0.0, forward_range - config.robot_radius - config.static_margin -
               localization_sigma);
  const double latency_speed = config.max_linear_decel * config.scan_timeout;
  const double braking_cap =
      -latency_speed +
      std::sqrt(latency_speed * latency_speed +
                2.0 * config.max_linear_decel * remaining);
  return std::max(0.0, std::min(config.open_speed_command, braking_cap));
}

bool plannedGateCrossingIsCentered(const PlanResult& plan, const Gate& gate) {
  if (!plan.valid || plan.trajectory.samples.size() < 2U) {
    return false;
  }
  const double normal_length = std::hypot(gate.normal.x, gate.normal.y);
  if (normal_length < 1e-9) {
    return false;
  }
  const Vec2 normal{gate.normal.x / normal_length, gate.normal.y / normal_length};
  const Vec2 tangent{-normal.y, normal.x};
  double previous_side =
      -gate.center.x * normal.x - gate.center.y * normal.y;
  for (std::size_t index = 1U; index < plan.trajectory.samples.size(); ++index) {
    const TrajectorySample& a = plan.trajectory.samples[index - 1U];
    const TrajectorySample& b = plan.trajectory.samples[index];
    const double current_side =
        (b.x - gate.center.x) * normal.x +
        (b.y - gate.center.y) * normal.y;
    if (previous_side <= 0.0 && current_side >= 0.0) {
      const double denominator = current_side - previous_side;
      const double ratio = denominator > 1e-12 ? -previous_side / denominator : 0.0;
      const Vec2 crossing{a.x + ratio * (b.x - a.x),
                          a.y + ratio * (b.y - a.y)};
      return std::abs(dot(Vec2{crossing.x - gate.center.x,
                               crossing.y - gate.center.y},
                          tangent)) < 0.04;
    }
    previous_side = current_side;
  }
  return false;
}

}  // namespace

RaceController::RaceController(const ControllerConfig& config)
    : config_(config),
      geometry_(config),
      matcher_(config),
      grid_(config),
      perception_(config),
      planner_(config),
      safety_filter_(config) {
  config_.validate();
  cached_output_.state = RaceState::LAUNCH;
}

ControllerOutput RaceController::onScan(const sensor_msgs::LaserScan& scan) {
  geometry_.condition(scan, current_frame_);
  const double now = current_frame_.stamp;
  if (!std::isfinite(now) ||
      (have_previous_scan_ && now <= last_scan_time_) ||
      !validConditionedScan(current_frame_)) {
    cached_output_.command = Command{};
    cached_output_.state = state_;
    cached_output_.safety_reason = SafetyReason::INVALID_SCAN;
    cached_output_.plan_valid = false;
    return cached_output_;
  }

  PoseDelta delta;
  double dt = 1.0 / 10.0;
  if (have_previous_scan_) {
    dt = now - last_scan_time_;
    delta = matcher_.match(previous_frame_, current_frame_, last_command_, dt);
    if (delta.confidence < 0.35) {
      ++low_match_streak_;
    } else {
      low_match_streak_ = 0;
    }
  } else {
    delta.confidence = 1.0;
  }

  if (low_match_streak_ >= 2) {
    grid_.reset();
    perception_.reset();
    matcher_.reset();
    low_match_streak_ = 0;
    gate_lock_scans_ = 0;
    if (state_ == RaceState::GATE_APPROACH || state_ == RaceState::GATE_CROSS) {
      state_ = RaceState::RACE;
    }
  }

  motion_.v = dt > 0.0 ? delta.dx / dt : 0.0;
  motion_.w = dt > 0.0 ? delta.dyaw / dt : 0.0;
  motion_.confidence = delta.confidence;
  const double localization_sigma =
      0.005 + 0.075 * (1.0 - std::max(0.0, std::min(1.0, delta.confidence)));

  grid_.update(current_frame_, delta, now);
  const NavigationStructure structure = perception_.update(
      current_frame_, delta, state_ == RaceState::ESCAPE);

  if (state_ == RaceState::LAUNCH) {
    state_ = RaceState::RACE;
  }
  if (structure.terminal_wall) {
    state_ = RaceState::BRAKE;
  } else if (state_ == RaceState::GATE_CROSS && structure.gate_crossed) {
    state_ = RaceState::RACE;
    gate_lock_scans_ = 0;
  } else if (state_ == RaceState::RACE && structure.active_gate &&
             structure.active_gate->confidence >= config_.gate_min_confidence) {
    state_ = RaceState::GATE_APPROACH;
    gate_lock_scans_ = 1;
  } else if (state_ == RaceState::GATE_APPROACH) {
    ++gate_lock_scans_;
  }

  const double certified_cap =
      certifiedCapFromScan(current_frame_, config_, localization_sigma);
  PlanResult plan;
  if (state_ != RaceState::BRAKE) {
    const PlannerInput input{motion_, grid_, structure, localization_sigma,
                             certified_cap, state_};
    plan = planner_.plan(input);
  }

  if (state_ == RaceState::GATE_APPROACH && gate_lock_scans_ >= 2 &&
      structure.active_gate &&
      plannedGateCrossingIsCentered(plan, *structure.active_gate)) {
    state_ = RaceState::GATE_CROSS;
  }

  if (plan.valid && plan.command.linear > 0.0) {
    no_plan_streak_ = 0;
    ++recovery_streak_;
    if (state_ == RaceState::ESCAPE && recovery_streak_ >= 2) {
      state_ = RaceState::RACE;
    }
  } else {
    recovery_streak_ = 0;
    ++no_plan_streak_;
    if (state_ == RaceState::RACE && no_plan_streak_ >= 3) {
      state_ = RaceState::ESCAPE;
    }
  }

  Command planned = plan.valid ? plan.command : Command{};
  double plan_cap = plan.valid ? plan.certified_speed_cap : 0.0;
  SafetyDecision safety = safety_filter_.filter(
      planned, motion_, current_frame_, now, now, plan_cap,
      localization_sigma);
  if (safety.reason == SafetyReason::UNSTOPPABLE) {
    state_ = RaceState::BRAKE;
  }
  if (state_ == RaceState::BRAKE) {
    safety.command = Command{};
  }

  cached_plan_ = plan;
  cached_output_.command = safety.command;
  cached_output_.state = state_;
  cached_output_.safety_reason = safety.reason;
  cached_output_.certified_speed_cap = plan_cap;
  cached_output_.localization_sigma = localization_sigma;
  cached_output_.plan_valid = plan.valid;
  cached_output_.gate_locked = static_cast<bool>(structure.active_gate);
  last_command_ = safety.command;
  last_scan_time_ = now;
  have_previous_scan_ = true;
  std::swap(previous_frame_, current_frame_);
  return cached_output_;
}

ControllerOutput RaceController::onControlTick(double now) {
  ControllerOutput output = cached_output_;
  output.state = state_;
  if (!have_previous_scan_ || !std::isfinite(now) || now < last_scan_time_ ||
      now - last_scan_time_ > config_.scan_timeout) {
    output.command = Command{};
    output.safety_reason = SafetyReason::STALE_SCAN;
    return output;
  }

  const double age = now - last_scan_time_;
  if (age <= 0.10 && cached_plan_.valid &&
      !cached_plan_.trajectory.samples.empty()) {
    const TrajectorySample* selected = &cached_plan_.trajectory.samples.back();
    for (const TrajectorySample& sample : cached_plan_.trajectory.samples) {
      if (sample.t >= age) {
        selected = &sample;
        break;
      }
    }
    output.command.linear = std::copysign(
        std::min(std::abs(output.command.linear), std::abs(selected->v)),
        output.command.linear);
    output.command.angular = selected->w;
  }
  output.command.linear = std::copysign(
      std::min(std::abs(output.command.linear), output.certified_speed_cap),
      output.command.linear);
  return output;
}

void RaceController::reset() {
  matcher_.reset();
  grid_.reset();
  perception_.reset();
  previous_frame_ = ScanFrame{};
  current_frame_ = ScanFrame{};
  have_previous_scan_ = false;
  last_scan_time_ = std::numeric_limits<double>::quiet_NaN();
  last_command_ = Command{};
  motion_ = MotionState{};
  state_ = RaceState::LAUNCH;
  low_match_streak_ = 0;
  no_plan_streak_ = 0;
  recovery_streak_ = 0;
  gate_lock_scans_ = 0;
  cached_plan_ = PlanResult{};
  cached_output_ = ControllerOutput{};
}

}  // namespace ultrafast_lidar_mpc
