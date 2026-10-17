#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <sensor_msgs/LaserScan.h>

#include "ultrafast_lidar_mpc/types.hpp"

namespace ultrafast_lidar_mpc {
namespace test {

class SyntheticScan {
 public:
  static sensor_msgs::LaserScan constant(double range) {
    sensor_msgs::LaserScan scan;
    scan.angle_min = -static_cast<float>(M_PI);
    scan.angle_increment = static_cast<float>(2.0 * M_PI / 360.0);
    scan.angle_max = scan.angle_min + 359.0F * scan.angle_increment;
    scan.range_min = 0.05F;
    scan.range_max = 10.0F;
    scan.ranges.assign(360, static_cast<float>(range));
    return scan;
  }

  static std::vector<Vec2> linePoints(double x_offset, double noise,
                                      std::size_t count) {
    std::vector<Vec2> points;
    points.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
      const double y = -1.0 + 2.0 * static_cast<double>(index) /
                                  static_cast<double>(count - 1U);
      const double x = x_offset + noise * std::sin(static_cast<double>(index));
      points.push_back(Vec2{x, y});
    }
    return points;
  }

  static ScanFrame wallWithGap(double plane_x, double gap_low, double gap_high,
                               double back_x, double side_ray_bias) {
    std::vector<double> ranges(360U, std::numeric_limits<double>::infinity());
    for (std::size_t bin = 0U; bin < ranges.size(); ++bin) {
      const double bearing = -M_PI + 2.0 * M_PI * static_cast<double>(bin) /
                                         static_cast<double>(ranges.size());
      const double cosine = std::cos(bearing);
      if (cosine <= 0.0) {
        continue;
      }
      const double wall_y = plane_x * std::tan(bearing);
      const bool in_gap = wall_y > gap_low && wall_y < gap_high;
      const double hit_x = in_gap ? back_x : plane_x;
      const double hit_y = hit_x * std::tan(bearing);
      if (in_gap || std::abs(hit_y) <= 2.2) {
        ranges[bin] = std::hypot(hit_x, hit_y);
      }
    }
    if (std::isfinite(side_ray_bias)) {
      const double bearing = std::atan2(side_ray_bias, plane_x);
      const long bin = std::lround((bearing + M_PI) * 360.0 / (2.0 * M_PI));
      ranges[static_cast<std::size_t>((bin % 360L + 360L) % 360L)] = 9.5;
    }
    return frameFromRanges(ranges);
  }

  static ScanFrame partiallyOccludedGate(double plane_x, double gap_low,
                                          double gap_high) {
    ScanFrame frame = wallWithGap(plane_x, gap_low, gap_high, 5.0,
                                  std::numeric_limits<double>::quiet_NaN());
    for (std::size_t bin = 0U; bin < frame.conservative_ranges.size(); ++bin) {
      const double bearing = -M_PI + 2.0 * M_PI * static_cast<double>(bin) /
                                         static_cast<double>(360U);
      const double y = plane_x * std::tan(bearing);
      if (y > gap_high - 0.08 && y < gap_high + 0.12) {
        frame.conservative_ranges[bin] =
            std::numeric_limits<double>::infinity();
      }
    }
    rebuildPoints(&frame);
    return frame;
  }

  static ScanFrame frontAndRearCorridors() {
    std::vector<double> ranges(360U, std::numeric_limits<double>::infinity());
    for (std::size_t bin = 0U; bin < ranges.size(); ++bin) {
      const double bearing = -M_PI + 2.0 * M_PI * static_cast<double>(bin) /
                                         static_cast<double>(ranges.size());
      const double sine = std::sin(bearing);
      if (std::abs(sine) < 1e-6) {
        continue;
      }
      const double range = 1.0 / std::abs(sine);
      const double x = range * std::cos(bearing);
      if (std::abs(x) <= 5.0) {
        ranges[bin] = range;
      }
    }
    return frameFromRanges(ranges);
  }

  static ScanFrame solidWall(double plane_x, double half_span = 1.2) {
    std::vector<double> ranges(360U, std::numeric_limits<double>::infinity());
    for (std::size_t bin = 0U; bin < ranges.size(); ++bin) {
      const double bearing = -M_PI + 2.0 * M_PI * static_cast<double>(bin) /
                                         static_cast<double>(ranges.size());
      const double cosine = std::cos(bearing);
      if (cosine <= 0.0) {
        continue;
      }
      const double y = plane_x * std::tan(bearing);
      if (std::abs(y) <= half_span) {
        ranges[bin] = std::hypot(plane_x, y);
      }
    }
    return frameFromRanges(ranges);
  }

 private:
  static ScanFrame frameFromRanges(const std::vector<double>& ranges) {
    ScanFrame frame;
    frame.conservative_ranges = ranges;
    rebuildPoints(&frame);
    return frame;
  }

  static void rebuildPoints(ScanFrame* frame) {
    frame->fit_points.clear();
    frame->collision_points.clear();
    frame->fit_points.reserve(frame->conservative_ranges.size());
    frame->collision_points.reserve(frame->conservative_ranges.size());
    for (std::size_t bin = 0U; bin < frame->conservative_ranges.size(); ++bin) {
      const double range = frame->conservative_ranges[bin];
      if (!std::isfinite(range)) {
        continue;
      }
      const double bearing = -M_PI + 2.0 * M_PI * static_cast<double>(bin) /
                                         static_cast<double>(frame->conservative_ranges.size());
      const ScanPoint point{Vec2{range * std::cos(bearing),
                                 range * std::sin(bearing)},
                            range, bearing, true};
      frame->fit_points.push_back(point);
      frame->collision_points.push_back(point);
    }
  }
};

}  // namespace test
}  // namespace ultrafast_lidar_mpc
