#include "ultrafast_lidar_mpc/structural_perception.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace ultrafast_lidar_mpc {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr std::size_t kMaximumLines = 64U;

struct LineFeature {
  Vec2 center;
  Vec2 tangent;
  double length{0.0};
  double confidence{0.0};
};

struct LineSet {
  std::array<LineFeature, kMaximumLines> values{};
  std::size_t size{0U};
};

double clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

double dot(const Vec2& a, const Vec2& b) {
  return a.x * b.x + a.y * b.y;
}

double distance(const Vec2& a, const Vec2& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

Vec2 rotate(const Vec2& point, double yaw) {
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  return Vec2{cosine * point.x - sine * point.y,
              sine * point.x + cosine * point.y};
}

Vec2 inverseTransform(const Vec2& point, const PoseDelta& motion) {
  return rotate(Vec2{point.x - motion.dx, point.y - motion.dy}, -motion.dyaw);
}

double nearestModuloPi(double heading, double reference) {
  while (heading - reference > 0.5 * kPi) {
    heading -= kPi;
  }
  while (heading - reference < -0.5 * kPi) {
    heading += kPi;
  }
  return heading;
}

bool fitLine(const std::vector<ScanPoint>& points, std::size_t begin,
             std::size_t end, LineFeature* line) {
  if (line == nullptr || end <= begin || end - begin < 12U) {
    return false;
  }
  double mean_x = 0.0;
  double mean_y = 0.0;
  for (std::size_t index = begin; index < end; ++index) {
    mean_x += points[index].p.x;
    mean_y += points[index].p.y;
  }
  const double count = static_cast<double>(end - begin);
  mean_x /= count;
  mean_y /= count;

  double xx = 0.0;
  double xy = 0.0;
  double yy = 0.0;
  for (std::size_t index = begin; index < end; ++index) {
    const double x = points[index].p.x - mean_x;
    const double y = points[index].p.y - mean_y;
    xx += x * x;
    xy += x * y;
    yy += y * y;
  }
  const double trace = xx + yy;
  if (trace <= 1e-12) {
    return false;
  }
  const double determinant = xx * yy - xy * xy;
  const double discriminant =
      std::sqrt(std::max(0.0, trace * trace - 4.0 * determinant));
  const double lambda_min = 0.5 * (trace - discriminant);
  const double lambda_max = 0.5 * (trace + discriminant);
  Vec2 tangent;
  if (std::abs(xy) > 1e-12) {
    tangent = Vec2{lambda_max - yy, xy};
  } else {
    tangent = xx >= yy ? Vec2{1.0, 0.0} : Vec2{0.0, 1.0};
  }
  const double tangent_norm = std::hypot(tangent.x, tangent.y);
  tangent.x /= tangent_norm;
  tangent.y /= tangent_norm;

  double min_projection = std::numeric_limits<double>::infinity();
  double max_projection = -std::numeric_limits<double>::infinity();
  for (std::size_t index = begin; index < end; ++index) {
    const Vec2 centered{points[index].p.x - mean_x, points[index].p.y - mean_y};
    const double projection = dot(centered, tangent);
    min_projection = std::min(min_projection, projection);
    max_projection = std::max(max_projection, projection);
  }
  line->center = Vec2{mean_x, mean_y};
  line->tangent = tangent;
  line->length = max_projection - min_projection;
  line->confidence = 1.0 - lambda_min / std::max(lambda_max, 1e-12);
  return line->length >= 0.5 && line->confidence >= 0.75;
}

LineSet extractLines(const ScanFrame& frame) {
  LineSet lines;
  const std::vector<ScanPoint>& points = frame.fit_points;
  std::size_t begin = 0U;
  while (begin < points.size() && lines.size < lines.values.size()) {
    std::size_t end = begin + 1U;
    while (end < points.size()) {
      const double bearing_gap =
          std::abs(wrapAngle(points[end].bearing - points[end - 1U].bearing));
      if (bearing_gap > 0.08 ||
          distance(points[end].p, points[end - 1U].p) > 0.30) {
        break;
      }
      ++end;
    }
    LineFeature line;
    if (fitLine(points, begin, end, &line)) {
      lines.values[lines.size++] = line;
    }
    begin = end;
  }
  return lines;
}

Vec2 rayPoint(std::size_t bin, std::size_t count, double range) {
  const double bearing = -kPi + kTwoPi * static_cast<double>(bin) /
                                    static_cast<double>(count);
  return Vec2{range * std::cos(bearing), range * std::sin(bearing)};
}

boost::optional<Gate> detectGate(const ScanFrame& frame,
                                 const ControllerConfig& config,
                                 double course_heading,
                                 const boost::optional<Gate>& previous) {
  const std::vector<double>& ranges = frame.conservative_ranges;
  if (ranges.size() < 3U) {
    return boost::none;
  }
  const Vec2 course{std::cos(course_heading), std::sin(course_heading)};
  boost::optional<Gate> best;
  double best_score = -1.0;
  for (std::size_t rise = 0U; rise + 2U < ranges.size(); ++rise) {
    if (!std::isfinite(ranges[rise]) || !std::isfinite(ranges[rise + 1U]) ||
        ranges[rise + 1U] - ranges[rise] < 0.70) {
      continue;
    }
    for (std::size_t fall = rise + 1U; fall + 1U < ranges.size(); ++fall) {
      if (!std::isfinite(ranges[fall]) || !std::isfinite(ranges[fall + 1U]) ||
          ranges[fall] - ranges[fall + 1U] < 0.70) {
        continue;
      }
      Vec2 low = rayPoint(rise, ranges.size(), ranges[rise]);
      Vec2 high = rayPoint(fall + 1U, ranges.size(), ranges[fall + 1U]);
      const double width = distance(low, high);
      if (width < config.gate_width_min || width > config.gate_width_max ||
          std::abs(low.x - high.x) > 0.12) {
        continue;
      }
      double minimum_depth = std::numeric_limits<double>::infinity();
      for (std::size_t inside = rise + 1U; inside <= fall; ++inside) {
        if (!std::isfinite(ranges[inside])) {
          continue;
        }
        const Vec2 deep = rayPoint(inside, ranges.size(), ranges[inside]);
        minimum_depth = std::min(minimum_depth,
                                 dot(deep, course) -
                                     0.5 * (dot(low, course) + dot(high, course)));
      }
      if (!std::isfinite(minimum_depth) || minimum_depth < 0.70) {
        continue;
      }
      Vec2 tangent{high.x - low.x, high.y - low.y};
      tangent.x /= width;
      tangent.y /= width;
      Vec2 normal{tangent.y, -tangent.x};
      if (dot(normal, course) < 0.0) {
        normal.x = -normal.x;
        normal.y = -normal.y;
      }
      const double alignment = dot(normal, course);
      if (alignment < std::cos(70.0 * kPi / 180.0)) {
        continue;
      }
      if (low.y > high.y) {
        std::swap(low, high);
      }
      Gate candidate;
      candidate.left_post = high;
      candidate.right_post = low;
      candidate.center = Vec2{0.5 * (low.x + high.x),
                              0.5 * (low.y + high.y)};
      candidate.normal = normal;
      candidate.width = width;
      const double plane_score = 1.0 - clamp(std::abs(low.x - high.x) / 0.12, 0.0, 1.0);
      const double depth_score = clamp((minimum_depth - 0.70) / 1.5, 0.0, 1.0);
      const double width_center = 0.5 * (config.gate_width_min + config.gate_width_max);
      const double width_score =
          1.0 - clamp(std::abs(width - width_center) /
                          (0.5 * (config.gate_width_max - config.gate_width_min)),
                      0.0, 1.0);
      double temporal_score = 0.0;
      if (previous) {
        temporal_score = clamp(1.0 - distance(candidate.center, previous->center) / 0.25,
                               0.0, 1.0);
      }
      candidate.confidence = clamp(0.30 * alignment + 0.25 * plane_score +
                                       0.20 * depth_score + 0.15 * width_score +
                                       0.10 * temporal_score,
                                   0.0, 1.0);
      const double score = candidate.confidence + 0.15 * temporal_score;
      if (candidate.confidence >= config.gate_min_confidence && score > best_score) {
        best_score = score;
        best = candidate;
      }
    }
  }
  return best;
}

double maximumPassableOpening(const ScanFrame& frame, double course_heading) {
  const std::vector<double>& ranges = frame.conservative_ranges;
  const Vec2 course{std::cos(course_heading), std::sin(course_heading)};
  double maximum_width = 0.0;
  for (std::size_t rise = 0U; rise + 2U < ranges.size(); ++rise) {
    if (!std::isfinite(ranges[rise]) || !std::isfinite(ranges[rise + 1U]) ||
        ranges[rise + 1U] - ranges[rise] < 0.70) {
      continue;
    }
    for (std::size_t fall = rise + 1U; fall + 1U < ranges.size(); ++fall) {
      if (!std::isfinite(ranges[fall]) || !std::isfinite(ranges[fall + 1U]) ||
          ranges[fall] - ranges[fall + 1U] < 0.70) {
        continue;
      }
      const Vec2 a = rayPoint(rise, ranges.size(), ranges[rise]);
      const Vec2 b = rayPoint(fall + 1U, ranges.size(), ranges[fall + 1U]);
      double depth = std::numeric_limits<double>::infinity();
      for (std::size_t inside = rise + 1U; inside <= fall; ++inside) {
        if (std::isfinite(ranges[inside])) {
          const Vec2 point = rayPoint(inside, ranges.size(), ranges[inside]);
          depth = std::min(depth, dot(point, course) -
                                     0.5 * (dot(a, course) + dot(b, course)));
        }
      }
      if (depth >= 0.70) {
        maximum_width = std::max(maximum_width, distance(a, b));
      }
    }
  }
  return maximum_width;
}

bool terminalWallVisible(const LineSet& lines, double course_heading) {
  const Vec2 course{std::cos(course_heading), std::sin(course_heading)};
  const Vec2 lateral{-course.y, course.x};
  for (std::size_t index = 0U; index < lines.size; ++index) {
    const LineFeature& line = lines.values[index];
    const double forward = dot(line.center, course);
    const double side = std::abs(dot(line.center, lateral));
    if (line.length >= 1.8 && forward > 0.0 && forward < 1.8 && side < 0.35 &&
        std::abs(dot(line.tangent, course)) < std::cos(70.0 * kPi / 180.0)) {
      return true;
    }
  }
  return false;
}

bool associated(const Gate& a, const Gate& b) {
  return distance(a.center, b.center) < 0.25 &&
         std::acos(clamp(dot(a.normal, b.normal), -1.0, 1.0)) <
             20.0 * kPi / 180.0 &&
         std::abs(a.width - b.width) < 0.10;
}

Gate smoothGate(const Gate& previous, const Gate& detected) {
  constexpr double alpha = 0.55;
  const auto blend = [](const Vec2& old_value, const Vec2& new_value) {
    return Vec2{(1.0 - alpha) * old_value.x + alpha * new_value.x,
                (1.0 - alpha) * old_value.y + alpha * new_value.y};
  };
  Gate result = detected;
  result.id = previous.id;
  result.left_post = blend(previous.left_post, detected.left_post);
  result.right_post = blend(previous.right_post, detected.right_post);
  result.center = blend(previous.center, detected.center);
  result.normal = blend(previous.normal, detected.normal);
  const double normal_length = std::hypot(result.normal.x, result.normal.y);
  result.normal.x /= normal_length;
  result.normal.y /= normal_length;
  result.width = distance(result.left_post, result.right_post);
  result.confidence = (1.0 - alpha) * previous.confidence +
                      alpha * detected.confidence;
  result.age = 0.0;
  return result;
}

}  // namespace

StructuralPerception::StructuralPerception(const ControllerConfig& config)
    : config_(config) {
  config_.validate();
}

NavigationStructure StructuralPerception::update(const ScanFrame& frame,
                                                 const PoseDelta& motion,
                                                 bool allow_reverse_escape) {
  const bool motion_valid = std::isfinite(motion.dx) && std::isfinite(motion.dy) &&
                            std::isfinite(motion.dyaw);
  if (motion_valid) {
    course_heading_ = wrapAngle(course_heading_ - motion.dyaw);
    if (tracked_gate_) {
      tracked_gate_->left_post = inverseTransform(tracked_gate_->left_post, motion);
      tracked_gate_->right_post = inverseTransform(tracked_gate_->right_post, motion);
      tracked_gate_->center = inverseTransform(tracked_gate_->center, motion);
      tracked_gate_->normal = rotate(tracked_gate_->normal, -motion.dyaw);
      tracked_gate_->age += 1.0;
    }
  }

  const LineSet lines = extractLines(frame);
  double heading_x = 0.0;
  double heading_y = 0.0;
  double heading_weight = 0.0;
  for (std::size_t index = 0U; index < lines.size; ++index) {
    double heading = std::atan2(lines.values[index].tangent.y,
                                lines.values[index].tangent.x);
    heading = nearestModuloPi(heading, course_heading_);
    const double error = std::abs(wrapAngle(heading - course_heading_));
    if (!allow_reverse_escape && error > 70.0 * kPi / 180.0) {
      continue;
    }
    const double weight = lines.values[index].confidence * lines.values[index].length;
    heading_x += weight * std::cos(heading);
    heading_y += weight * std::sin(heading);
    heading_weight += weight;
  }
  if (heading_weight > 0.0) {
    const double measured = std::atan2(heading_y, heading_x);
    const double measurement_confidence =
        clamp(std::hypot(heading_x, heading_y) / heading_weight, 0.0, 1.0);
    const double alpha = 0.35 + 0.45 * measurement_confidence;
    const double delta = wrapAngle(measured - course_heading_);
    if (allow_reverse_escape || std::abs(delta) <= 0.5 * kPi) {
      course_heading_ = wrapAngle(course_heading_ + alpha * delta);
      course_confidence_ = clamp(0.25 * course_confidence_ +
                                     0.75 * measurement_confidence,
                                 0.0, 1.0);
    }
  } else {
    course_confidence_ *= 0.92;
  }

  boost::optional<Gate> detected =
      detectGate(frame, config_, course_heading_, tracked_gate_);
  if (detected) {
    if (tracked_gate_ && associated(*tracked_gate_, *detected)) {
      tracked_gate_ = smoothGate(*tracked_gate_, *detected);
    } else {
      detected->id = next_gate_id_++;
      detected->age = 0.0;
      tracked_gate_ = detected;
      rear_gate_streak_ = 0;
      gate_was_ahead_ = false;
      crossing_reported_ = false;
    }
  } else if (tracked_gate_ && tracked_gate_->age > 8.0 &&
             (!gate_was_ahead_ || crossing_reported_)) {
    tracked_gate_ = boost::none;
  }

  bool crossed = false;
  if (tracked_gate_) {
    const double forward_plane = dot(tracked_gate_->center, tracked_gate_->normal);
    if (forward_plane > 0.02) {
      gate_was_ahead_ = true;
    }
    const bool both_rear = tracked_gate_->left_post.x < 0.0 &&
                           tracked_gate_->right_post.x < 0.0 &&
                           forward_plane < 0.0;
    if (gate_was_ahead_ && both_rear) {
      ++rear_gate_streak_;
    } else if (!both_rear) {
      rear_gate_streak_ = 0;
    }
    if (rear_gate_streak_ >= 2 && !crossing_reported_) {
      crossed = true;
      crossing_reported_ = true;
    }
  }

  const double passable_width = maximumPassableOpening(frame, course_heading_);
  const double required_width =
      2.0 * config_.robot_radius + 2.0 * config_.static_margin;
  if (terminalWallVisible(lines, course_heading_) &&
      passable_width <= required_width) {
    ++terminal_wall_streak_;
  } else {
    terminal_wall_streak_ = 0;
  }

  NavigationStructure result;
  result.course_heading = course_heading_;
  result.course_confidence = course_confidence_;
  result.active_gate = tracked_gate_;
  result.gate_crossed = crossed;
  result.terminal_wall =
      terminal_wall_streak_ >= config_.end_wall_persistence_scans;
  return result;
}

void StructuralPerception::reset() {
  course_heading_ = 0.0;
  course_confidence_ = 0.0;
  tracked_gate_ = boost::none;
  rear_gate_streak_ = 0;
  terminal_wall_streak_ = 0;
  gate_was_ahead_ = false;
  crossing_reported_ = false;
  next_gate_id_ = 1U;
}

}  // namespace ultrafast_lidar_mpc
