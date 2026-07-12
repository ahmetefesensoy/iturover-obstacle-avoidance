#!/bin/bash
# WSL içinde Gazebo'da obstacle_maze world'ünü açar
# Kullanım: wsl -d Ubuntu-20.04 bash ituroveranswer/solution_ml/run_maze_gazebo.sh

# Source ROS
source /opt/ros/noetic/setup.bash
source /root/rover_ws/devel/setup.bash

# Launch file'a obstacle_maze için spawn_args ekle
LAUNCH_FILE="/root/rover_ws/src/iturover_obstacle_avoidance_assignment/launch/reactive_robot_sim.launch"

# obstacle_maze için spawn_args ekle (eğer yoksa)
if ! grep -q 'obstacle_maze' "$LAUNCH_FILE"; then
    sed -i '/obs_corridor_world/a\    <arg if="$(eval world == '"'"'obstacle_maze'"'"')" name="spawn_args" default="-x -4.1 -y 4.3 -z 0.0 -R 0 -P 0 -Y -1.08"/>' "$LAUNCH_FILE"
    echo "spawn_args eklendi"
fi

# Gazebo'yu başlat
roslaunch reactive_robot_sim reactive_robot_sim.launch world:=obstacle_maze gui:=true rviz:=true
