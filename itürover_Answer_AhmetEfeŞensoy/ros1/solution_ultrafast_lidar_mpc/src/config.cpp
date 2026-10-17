#include "ultrafast_lidar_mpc/config.hpp"

#include <stdexcept>

namespace ultrafast_lidar_mpc {
namespace {

void requirePositive(double value, const char* name) {
  if (value <= 0.0) {
    throw std::invalid_argument(name);
  }
}

}  // namespace

void ControllerConfig::validate() const {
  requirePositive(robot_radius, "robot_radius must be positive");
  requirePositive(static_margin, "static_margin must be positive");
  requirePositive(max_forward_latency, "max_forward_latency must be positive");
  requirePositive(max_linear_accel, "max_linear_accel must be positive");
  requirePositive(max_linear_decel, "max_linear_decel must be positive");
  requirePositive(max_angular_accel, "max_angular_accel must be positive");
  requirePositive(control_rate_hz, "control_rate_hz must be positive");
  requirePositive(scan_timeout, "scan_timeout must be positive");
  requirePositive(grid_size, "grid_size must be positive");
  requirePositive(grid_resolution, "grid_resolution must be positive");
  requirePositive(grid_history_seconds, "grid_history_seconds must be positive");
  requirePositive(trajectory_horizon, "trajectory_horizon must be positive");
  requirePositive(integration_dt, "integration_dt must be positive");
  requirePositive(gate_width_min, "gate_width_min must be positive");
  requirePositive(gate_width_max, "gate_width_max must be positive");

  if (control_rate_hz < 20.0) {
    throw std::invalid_argument("control_rate_hz must be at least 20 Hz");
  }
  if (scan_timeout < 0.15) {
    throw std::invalid_argument("scan_timeout must be at least 0.15 s");
  }
  if (grid_resolution > 0.05) {
    throw std::invalid_argument("grid_resolution must not exceed 0.05 m");
  }
  if (integration_dt > 0.05) {
    throw std::invalid_argument("integration_dt must not exceed 0.05 s");
  }
  if (gate_width_min > gate_width_max) {
    throw std::invalid_argument("gate width interval is inverted");
  }
  if (open_speed_command < 1.0) {
    throw std::invalid_argument("open_speed_command must be at least 1 m/s");
  }
  if (end_wall_persistence_scans <= 0) {
    throw std::invalid_argument("end_wall_persistence_scans must be positive");
  }
}

}  // namespace ultrafast_lidar_mpc
