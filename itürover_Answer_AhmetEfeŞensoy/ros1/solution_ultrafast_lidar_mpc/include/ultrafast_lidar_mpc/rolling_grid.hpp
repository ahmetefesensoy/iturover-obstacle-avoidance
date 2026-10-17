#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "ultrafast_lidar_mpc/config.hpp"
#include "ultrafast_lidar_mpc/types.hpp"

namespace ultrafast_lidar_mpc {

class RollingGrid {
 public:
  explicit RollingGrid(const ControllerConfig& config);

  void update(const ScanFrame& frame, const PoseDelta& motion, double now);
  bool occupied(const Vec2& point) const;
  double clearance(const Vec2& point) const;
  void reset();

  std::size_t width() const { return width_; }
  std::size_t height() const { return width_; }
  double resolution() const { return resolution_; }
  std::array<std::size_t, 5U> capacitySignature() const;

 private:
  bool cellFor(const Vec2& point, std::size_t* cell) const;
  Vec2 cellCenter(std::size_t cell) const;
  void insertPoint(const Vec2& point, double stamp);
  void rebuildClearance();

  double resolution_{0.0};
  double half_extent_{0.0};
  double history_seconds_{0.0};
  std::size_t width_{0U};
  std::vector<std::uint8_t> occupied_cells_;
  std::vector<std::uint8_t> spare_occupied_cells_;
  std::vector<double> hit_stamps_;
  std::vector<double> spare_hit_stamps_;
  std::vector<float> clearance_cells_;
};

}  // namespace ultrafast_lidar_mpc
