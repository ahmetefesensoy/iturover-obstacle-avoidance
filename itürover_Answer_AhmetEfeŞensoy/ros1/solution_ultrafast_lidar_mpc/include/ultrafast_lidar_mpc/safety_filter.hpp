#pragma once

#include "ultrafast_lidar_mpc/config.hpp"
#include "ultrafast_lidar_mpc/types.hpp"

namespace ultrafast_lidar_mpc {

enum class SafetyReason {
  NONE,
  STALE_SCAN,
  INVALID_SCAN,
  UNSTOPPABLE,
  CAP_EXCEEDED
};

struct SafetyDecision {
  Command command;
  bool vetoed{false};
  SafetyReason reason{SafetyReason::NONE};
};

class SafetyFilter {
 public:
  explicit SafetyFilter(const ControllerConfig& config);

  SafetyDecision filter(const Command& planned, const MotionState& motion,
                        const ScanFrame& scan, double now,
                        double last_scan_time, double certified_cap,
                        double localization_sigma = 0.0) const;

 private:
  ControllerConfig config_;
};

}  // namespace ultrafast_lidar_mpc
