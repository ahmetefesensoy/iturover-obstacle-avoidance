#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

namespace ultrafast_lidar_mpc {

struct Vec2 {
  double x{0.0};
  double y{0.0};
};

struct PoseDelta {
  double dx{0.0};
  double dy{0.0};
  double dyaw{0.0};
  double confidence{0.0};
};

struct MotionState {
  double v{0.0};
  double w{0.0};
  double confidence{0.0};
};

struct Command {
  double linear{0.0};
  double angular{0.0};
};

struct ScanPoint {
  Vec2 p;
  double range{0.0};
  double bearing{0.0};
  bool raw_valid{false};
};

struct ScanFrame {
  double stamp{0.0};
  std::vector<double> conservative_ranges;
  std::vector<ScanPoint> collision_points;
  std::vector<ScanPoint> fit_points;
};

struct Gate {
  std::uint64_t id{0};
  Vec2 left_post;
  Vec2 right_post;
  Vec2 center;
  Vec2 normal;
  double width{0.0};
  double confidence{0.0};
  double age{0.0};
};

enum class RaceState { LAUNCH, RACE, GATE_APPROACH, GATE_CROSS, BRAKE, ESCAPE };

inline double wrapAngle(double angle) {
  return std::remainder(angle, 2.0 * M_PI);
}

}  // namespace ultrafast_lidar_mpc
