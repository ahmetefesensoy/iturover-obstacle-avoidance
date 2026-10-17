#include "ultrafast_lidar_mpc/rolling_grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace ultrafast_lidar_mpc {
namespace {

constexpr float kDiagonalCost = 1.41421356237F;

bool finiteMotion(const PoseDelta& motion) {
  return std::isfinite(motion.dx) && std::isfinite(motion.dy) &&
         std::isfinite(motion.dyaw);
}

}  // namespace

RollingGrid::RollingGrid(const ControllerConfig& config)
    : resolution_(config.grid_resolution),
      history_seconds_(config.grid_history_seconds) {
  config.validate();
  width_ = static_cast<std::size_t>(
      std::ceil(config.grid_size / config.grid_resolution));
  if (width_ == 0U || width_ > std::numeric_limits<std::size_t>::max() / width_) {
    throw std::invalid_argument("rolling grid dimensions overflow");
  }
  half_extent_ = 0.5 * static_cast<double>(width_) * resolution_;
  const std::size_t cell_count = width_ * width_;
  occupied_cells_.assign(cell_count, 0U);
  spare_occupied_cells_.assign(cell_count, 0U);
  hit_stamps_.assign(cell_count, -std::numeric_limits<double>::infinity());
  spare_hit_stamps_.assign(cell_count,
                           -std::numeric_limits<double>::infinity());
  clearance_cells_.assign(cell_count, std::numeric_limits<float>::infinity());
}

bool RollingGrid::cellFor(const Vec2& point, std::size_t* cell) const {
  if (cell == nullptr || !std::isfinite(point.x) || !std::isfinite(point.y) ||
      point.x < -half_extent_ || point.x >= half_extent_ ||
      point.y < -half_extent_ || point.y >= half_extent_) {
    return false;
  }
  const std::size_t column = static_cast<std::size_t>(
      std::floor((point.x + half_extent_) / resolution_));
  const std::size_t row = static_cast<std::size_t>(
      std::floor((point.y + half_extent_) / resolution_));
  if (column >= width_ || row >= width_) {
    return false;
  }
  *cell = row * width_ + column;
  return true;
}

Vec2 RollingGrid::cellCenter(std::size_t cell) const {
  const std::size_t row = cell / width_;
  const std::size_t column = cell - row * width_;
  return Vec2{(static_cast<double>(column) + 0.5) * resolution_ - half_extent_,
              (static_cast<double>(row) + 0.5) * resolution_ - half_extent_};
}

void RollingGrid::insertPoint(const Vec2& point, double stamp) {
  std::size_t cell = 0U;
  if (!cellFor(point, &cell)) {
    return;
  }
  occupied_cells_[cell] = 1U;
  hit_stamps_[cell] = std::max(hit_stamps_[cell], stamp);
}

void RollingGrid::update(const ScanFrame& frame, const PoseDelta& motion,
                         double now) {
  if (!std::isfinite(now)) {
    return;
  }

  std::fill(spare_occupied_cells_.begin(), spare_occupied_cells_.end(), 0U);
  std::fill(spare_hit_stamps_.begin(), spare_hit_stamps_.end(),
            -std::numeric_limits<double>::infinity());

  if (finiteMotion(motion)) {
    const double cosine = std::cos(motion.dyaw);
    const double sine = std::sin(motion.dyaw);
    for (std::size_t cell = 0U; cell < occupied_cells_.size(); ++cell) {
      if (occupied_cells_[cell] == 0U ||
          now - hit_stamps_[cell] > history_seconds_) {
        continue;
      }
      const Vec2 old_point = cellCenter(cell);
      const double shifted_x = old_point.x - motion.dx;
      const double shifted_y = old_point.y - motion.dy;
      const Vec2 current_point{cosine * shifted_x + sine * shifted_y,
                               -sine * shifted_x + cosine * shifted_y};
      std::size_t current_cell = 0U;
      if (!cellFor(current_point, &current_cell)) {
        continue;
      }
      spare_occupied_cells_[current_cell] = 1U;
      spare_hit_stamps_[current_cell] =
          std::max(spare_hit_stamps_[current_cell], hit_stamps_[cell]);
    }
  }

  occupied_cells_.swap(spare_occupied_cells_);
  hit_stamps_.swap(spare_hit_stamps_);
  for (const ScanPoint& point : frame.collision_points) {
    insertPoint(point.p, now);
  }
  rebuildClearance();
}

void RollingGrid::rebuildClearance() {
  const float infinity = std::numeric_limits<float>::infinity();
  for (std::size_t cell = 0U; cell < occupied_cells_.size(); ++cell) {
    clearance_cells_[cell] = occupied_cells_[cell] != 0U ? 0.0F : infinity;
  }

  const auto relax = [this](std::size_t cell, std::size_t neighbor, float cost) {
    clearance_cells_[cell] =
        std::min(clearance_cells_[cell], clearance_cells_[neighbor] + cost);
  };
  for (std::size_t row = 0U; row < width_; ++row) {
    for (std::size_t column = 0U; column < width_; ++column) {
      const std::size_t cell = row * width_ + column;
      if (column > 0U) {
        relax(cell, cell - 1U, 1.0F);
      }
      if (row > 0U) {
        relax(cell, cell - width_, 1.0F);
        if (column > 0U) {
          relax(cell, cell - width_ - 1U, kDiagonalCost);
        }
        if (column + 1U < width_) {
          relax(cell, cell - width_ + 1U, kDiagonalCost);
        }
      }
    }
  }
  for (std::size_t row = width_; row-- > 0U;) {
    for (std::size_t column = width_; column-- > 0U;) {
      const std::size_t cell = row * width_ + column;
      if (column + 1U < width_) {
        relax(cell, cell + 1U, 1.0F);
      }
      if (row + 1U < width_) {
        relax(cell, cell + width_, 1.0F);
        if (column > 0U) {
          relax(cell, cell + width_ - 1U, kDiagonalCost);
        }
        if (column + 1U < width_) {
          relax(cell, cell + width_ + 1U, kDiagonalCost);
        }
      }
    }
  }
  for (float& distance : clearance_cells_) {
    distance *= static_cast<float>(resolution_);
  }
}

bool RollingGrid::occupied(const Vec2& point) const {
  std::size_t cell = 0U;
  return cellFor(point, &cell) && occupied_cells_[cell] != 0U;
}

double RollingGrid::clearance(const Vec2& point) const {
  std::size_t cell = 0U;
  if (!cellFor(point, &cell)) {
    return 0.0;
  }
  return static_cast<double>(clearance_cells_[cell]);
}

void RollingGrid::reset() {
  std::fill(occupied_cells_.begin(), occupied_cells_.end(), 0U);
  std::fill(spare_occupied_cells_.begin(), spare_occupied_cells_.end(), 0U);
  std::fill(hit_stamps_.begin(), hit_stamps_.end(),
            -std::numeric_limits<double>::infinity());
  std::fill(spare_hit_stamps_.begin(), spare_hit_stamps_.end(),
            -std::numeric_limits<double>::infinity());
  std::fill(clearance_cells_.begin(), clearance_cells_.end(),
            std::numeric_limits<float>::infinity());
}

std::array<std::size_t, 5U> RollingGrid::capacitySignature() const {
  return {{occupied_cells_.capacity(), spare_occupied_cells_.capacity(),
           hit_stamps_.capacity(), spare_hit_stamps_.capacity(),
           clearance_cells_.capacity()}};
}

}  // namespace ultrafast_lidar_mpc
