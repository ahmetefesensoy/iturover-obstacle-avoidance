# Ultrafast LiDAR MPC

ROS 1 LiDAR-only minimum-time controller for the supplied TurtleBot3 Burger
obstacle course. Production control subscribes only to `/scan`, publishes
`/cmd_vel`, and does not consume odometry, IMU, camera, map, ground-truth, or
simulator state.

Run simulator and controller together:

```bash
roslaunch ultrafast_lidar_mpc reactive_robot.launch gui:=true rviz:=false
```

`/ultrafast_lidar_mpc/telemetry` is published at 2 Hz as a fixed-order
`Float64MultiArray`:

1. timestamp
2. race-state enum
3. estimated forward velocity
4. scan-match confidence
5. active gate ID (zero when absent)
6. gate confidence
7. commanded linear velocity
8. commanded angular velocity
9. certified speed cap
10. minimum planned clearance
11. generated candidate count
12. planner time in microseconds
13. safety-reason enum
14. localization sigma

The configured 6 m/s value is a requested open-space target. The planner and
independent safety filter reduce it whenever the visible stopping envelope,
grid boundary, gate aperture, localization uncertainty, or dynamics require.
