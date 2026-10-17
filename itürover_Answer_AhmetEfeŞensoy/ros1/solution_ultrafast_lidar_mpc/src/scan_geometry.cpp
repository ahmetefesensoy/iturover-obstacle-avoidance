#include "ultrafast_lidar_mpc/scan_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ultrafast_lidar_mpc {
namespace {

bool isValidRange(float range, const sensor_msgs::LaserScan& scan) {
  return std::isfinite(range) && range >= scan.range_min && range <= scan.range_max;
}

double medianOfValidNeighbors(const sensor_msgs::LaserScan& scan,
                              std::size_t previous_index, std::size_t index,
                              std::size_t next_index) {
  double values[3];
  std::size_t count = 0U;
  for (const std::size_t neighbor : {previous_index, index, next_index}) {
    const float candidate = scan.ranges[neighbor];
    if (isValidRange(candidate, scan)) {
      values[count++] = static_cast<double>(candidate);
    }
  }
  if (count == 1U) {
    return values[0];
  }
  if (count == 2U) {
    return (values[0] + values[1]) / 2.0;
  }
  const double minimum = std::min(values[0], std::min(values[1], values[2]));
  const double maximum = std::max(values[0], std::max(values[1], values[2]));
  return values[0] + values[1] + values[2] - minimum - maximum;
}

ScanPoint pointForRange(double range, double bearing) {
  return ScanPoint{Vec2{range * std::cos(bearing), range * std::sin(bearing)},
                   range, bearing, true};
}

}  // namespace

ScanGeometry::ScanGeometry(const ControllerConfig& config) : config_(config) {}

ScanFrame ScanGeometry::condition(const sensor_msgs::LaserScan& scan) const {
  ScanFrame frame;
  condition(scan, frame);
  return frame;
}

void ScanGeometry::condition(const sensor_msgs::LaserScan& scan,
                             ScanFrame& frame) const {
  frame.stamp = scan.header.stamp.toSec();
  const std::size_t count = scan.ranges.size();
  frame.conservative_ranges.resize(count);
  std::fill(frame.conservative_ranges.begin(), frame.conservative_ranges.end(),
            std::numeric_limits<double>::infinity());
  frame.collision_points.clear();
  frame.fit_points.clear();
  frame.collision_points.reserve(count);
  frame.fit_points.reserve(count);

  for (std::size_t index = 0; index < count; ++index) {
    const float raw = scan.ranges[index];
    if (!isValidRange(raw, scan)) {
      continue;
    }

    const double raw_range = static_cast<double>(raw);
    frame.conservative_ranges[index] = raw_range;

    const double fit_range = medianOfValidNeighbors(
        scan, (index + count - 1U) % count, index, (index + 1U) % count);
    const double collision_range = std::min(raw_range, fit_range);
    const double bearing = static_cast<double>(scan.angle_min) +
                           static_cast<double>(index) * scan.angle_increment;
    frame.collision_points.push_back(pointForRange(collision_range, bearing));
    frame.fit_points.push_back(pointForRange(fit_range, bearing));
  }
}

LineModel ScanGeometry::fitLine(const std::vector<Vec2>& points) {
  std::vector<Vec2> finite_points;
  finite_points.reserve(points.size());
  for (const Vec2& point : points) {
    if (std::isfinite(point.x) && std::isfinite(point.y)) {
      finite_points.push_back(point);
    }
  }
  if (finite_points.empty()) {
    return LineModel{};
  }

  Vec2 centroid;
  for (const Vec2& point : finite_points) {
    centroid.x += point.x;
    centroid.y += point.y;
  }
  const double point_count = static_cast<double>(finite_points.size());
  centroid.x /= point_count;
  centroid.y /= point_count;

  double covariance_xx = 0.0;
  double covariance_xy = 0.0;
  double covariance_yy = 0.0;
  for (const Vec2& point : finite_points) {
    const double dx = point.x - centroid.x;
    const double dy = point.y - centroid.y;
    covariance_xx += dx * dx;
    covariance_xy += dx * dy;
    covariance_yy += dy * dy;
  }
  covariance_xx /= point_count;
  covariance_xy /= point_count;
  covariance_yy /= point_count;

  const double trace = covariance_xx + covariance_yy;
  const double determinant = covariance_xx * covariance_yy - covariance_xy * covariance_xy;
  const double discriminant = std::sqrt(std::max(0.0, trace * trace - 4.0 * determinant));
  const double lambda_min = (trace - discriminant) / 2.0;
  const double lambda_max = (trace + discriminant) / 2.0;

  Vec2 normal;
  if (std::abs(covariance_xy) > 1e-12) {
    normal = Vec2{covariance_xy, lambda_min - covariance_xx};
  } else if (covariance_xx <= covariance_yy) {
    normal = Vec2{1.0, 0.0};
  } else {
    normal = Vec2{0.0, 1.0};
  }
  const double normal_length = std::hypot(normal.x, normal.y);
  normal.x /= normal_length;
  normal.y /= normal_length;

  return LineModel{normal, normal.x * centroid.x + normal.y * centroid.y,
                   1.0 - lambda_min / std::max(lambda_max, 1e-9)};
}

std::vector<std::size_t> ScanGeometry::rangeDiscontinuities(
    const ScanFrame& frame, double threshold) {
  std::vector<std::size_t> discontinuities;
  const std::size_t count = frame.conservative_ranges.size();
  if (count < 2U) {
    return discontinuities;
  }
  for (std::size_t index = 0; index < count; ++index) {
    const double current = frame.conservative_ranges[index];
    const double next = frame.conservative_ranges[(index + 1U) % count];
    if (std::isfinite(current) && std::isfinite(next) &&
        std::abs(current - next) > threshold) {
      discontinuities.push_back(index);
    }
  }
  return discontinuities;
}

}  // namespace ultrafast_lidar_mpc
