#include <mutex>
#include <stdexcept>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64MultiArray.h>

#include "ultrafast_lidar_mpc/race_controller.hpp"

namespace ultrafast_lidar_mpc {
namespace {

ControllerConfig loadConfig(ros::NodeHandle& private_node) {
  ControllerConfig config;
  private_node.param("robot_radius", config.robot_radius, config.robot_radius);
  private_node.param("static_margin", config.static_margin, config.static_margin);
  private_node.param("max_forward_latency", config.max_forward_latency,
                     config.max_forward_latency);
  private_node.param("max_linear_accel", config.max_linear_accel,
                     config.max_linear_accel);
  private_node.param("max_linear_decel", config.max_linear_decel,
                     config.max_linear_decel);
  private_node.param("max_angular_accel", config.max_angular_accel,
                     config.max_angular_accel);
  private_node.param("open_speed_command", config.open_speed_command,
                     config.open_speed_command);
  private_node.param("control_rate_hz", config.control_rate_hz,
                     config.control_rate_hz);
  private_node.param("scan_timeout", config.scan_timeout, config.scan_timeout);
  private_node.param("grid_size", config.grid_size, config.grid_size);
  private_node.param("grid_resolution", config.grid_resolution,
                     config.grid_resolution);
  private_node.param("grid_history_seconds", config.grid_history_seconds,
                     config.grid_history_seconds);
  private_node.param("trajectory_horizon", config.trajectory_horizon,
                     config.trajectory_horizon);
  private_node.param("integration_dt", config.integration_dt,
                     config.integration_dt);
  private_node.param("gate_width_min", config.gate_width_min,
                     config.gate_width_min);
  private_node.param("gate_width_max", config.gate_width_max,
                     config.gate_width_max);
  private_node.param("gate_min_confidence", config.gate_min_confidence,
                     config.gate_min_confidence);
  private_node.param("end_wall_persistence_scans",
                     config.end_wall_persistence_scans,
                     config.end_wall_persistence_scans);
  config.validate();
  return config;
}

geometry_msgs::Twist twistFor(const Command& command) {
  geometry_msgs::Twist message;
  message.linear.x = command.linear;
  message.angular.z = command.angular;
  return message;
}

const char* stateName(RaceState state) {
  switch (state) {
    case RaceState::LAUNCH: return "LAUNCH";
    case RaceState::RACE: return "RACE";
    case RaceState::GATE_APPROACH: return "GATE_APPROACH";
    case RaceState::GATE_CROSS: return "GATE_CROSS";
    case RaceState::BRAKE: return "BRAKE";
    case RaceState::ESCAPE: return "ESCAPE";
  }
  return "UNKNOWN";
}

class RaceControllerNode {
 public:
  RaceControllerNode(ros::NodeHandle& node, ros::NodeHandle& private_node,
                     const ControllerConfig& config)
      : controller_(config) {
    command_publisher_ = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    telemetry_publisher_ = private_node.advertise<std_msgs::Float64MultiArray>(
        "telemetry", 1);
    scan_subscriber_ = node.subscribe<sensor_msgs::LaserScan>(
        "/scan", 1, &RaceControllerNode::scanCallback, this,
        ros::TransportHints().tcpNoDelay());
    control_timer_ = node.createTimer(
        ros::Duration(1.0 / config.control_rate_hz),
        &RaceControllerNode::controlCallback, this);
    telemetry_timer_ = node.createTimer(
        ros::Duration(0.5), &RaceControllerNode::telemetryCallback, this);
  }

  ~RaceControllerNode() { stop(); }

  void stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    command_publisher_.publish(geometry_msgs::Twist{});
  }

 private:
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = controller_.onScan(*scan);
    command_publisher_.publish(twistFor(latest_.command));
  }

  void controlCallback(const ros::TimerEvent&) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = controller_.onControlTick(ros::Time::now().toSec());
    command_publisher_.publish(twistFor(latest_.command));
  }

  void telemetryCallback(const ros::TimerEvent&) {
    std::lock_guard<std::mutex> lock(mutex_);
    std_msgs::Float64MultiArray message;
    message.data.reserve(14U);
    message.data = {
        ros::Time::now().toSec(), static_cast<double>(latest_.state),
        latest_.estimated_velocity, latest_.match_confidence, latest_.gate_id,
        latest_.gate_confidence, latest_.command.linear, latest_.command.angular,
        latest_.certified_speed_cap, latest_.minimum_clearance,
        latest_.candidate_count, latest_.planner_microseconds,
        static_cast<double>(latest_.safety_reason), latest_.localization_sigma};
    telemetry_publisher_.publish(message);
    ROS_INFO("state=%s v=%.3f conf=%.2f gate=%.0f/%.2f cmd=(%.3f,%.3f) "
             "cap=%.3f clear=%.3f candidates=%.0f planner_us=%.0f safety=%d",
             stateName(latest_.state), latest_.estimated_velocity,
             latest_.match_confidence, latest_.gate_id, latest_.gate_confidence,
             latest_.command.linear, latest_.command.angular,
             latest_.certified_speed_cap, latest_.minimum_clearance,
             latest_.candidate_count, latest_.planner_microseconds,
             static_cast<int>(latest_.safety_reason));
  }

  RaceController controller_;
  std::mutex mutex_;
  ros::Publisher command_publisher_;
  ros::Publisher telemetry_publisher_;
  ros::Subscriber scan_subscriber_;
  ros::Timer control_timer_;
  ros::Timer telemetry_timer_;
  ControllerOutput latest_;
};

}  // namespace
}  // namespace ultrafast_lidar_mpc

int main(int argc, char** argv) {
  ros::init(argc, argv, "ultrafast_lidar_mpc");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");
  try {
    const ultrafast_lidar_mpc::ControllerConfig config =
        ultrafast_lidar_mpc::loadConfig(private_node);
    ultrafast_lidar_mpc::RaceControllerNode controller_node(node, private_node,
                                                            config);
    ros::AsyncSpinner spinner(2U);
    spinner.start();
    ros::waitForShutdown();
    controller_node.stop();
  } catch (const std::exception& error) {
    ROS_FATAL("invalid ultrafast controller configuration: %s", error.what());
    return 2;
  }
  return 0;
}
