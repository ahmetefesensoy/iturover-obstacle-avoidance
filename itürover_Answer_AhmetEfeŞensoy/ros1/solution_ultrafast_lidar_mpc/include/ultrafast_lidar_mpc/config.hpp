#pragma once

namespace ultrafast_lidar_mpc {

struct ControllerConfig {
  double robot_radius{0.105};
  double static_margin{0.018};
  double max_forward_latency{0.12};
  double max_linear_accel{1.0};
  double max_linear_decel{1.0};
  double max_angular_accel{8.0};
  double open_speed_command{6.0};
  double control_rate_hz{30.0};
  double scan_timeout{0.22};
  double grid_size{8.0};
  double grid_resolution{0.025};
  double grid_history_seconds{0.8};
  double trajectory_horizon{2.0};
  double integration_dt{0.025};
  double gate_width_min{0.30};
  double gate_width_max{0.60};
  double gate_min_confidence{0.65};
  int end_wall_persistence_scans{3};

  void validate() const;
};

}  // namespace ultrafast_lidar_mpc
