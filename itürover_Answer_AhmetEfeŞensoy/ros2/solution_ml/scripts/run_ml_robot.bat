@echo off
echo WSL kapatiliyor (Gazebo blank bug icin)...
wsl --shutdown
timeout /t 3 /nobreak >nul

echo ML Robot baslatiliyor...
wsl -d Ubuntu-20.04 bash -c "^
  source /opt/ros/noetic/setup.bash && ^
  rm -rf ~/rover_ws/src/reactive_robot_sim && ^
  cp -r '/mnt/c/Users/Gelir \304\260daresi/Desktop/iturover_obstacle_avoidance_assignment-main/main - Kopya/' ~/rover_ws/src/reactive_robot_sim && ^
  chmod +x ~/rover_ws/src/reactive_robot_sim/scripts/ml_robot.py && ^
  cd ~/rover_ws && catkin_make -DCMAKE_BUILD_TYPE=Release && ^
  source devel/setup.bash && ^
  export TURTLEBOT3_MODEL=burger_rgbd && ^
  roslaunch reactive_robot_sim ml_robot.launch weights_path:='/mnt/c/Users/Gelir \304\260daresi/Desktop/iturover_obstacle_avoidance_assignment-main/ituroveranswer/solution_ml/weights/bc_weights_v7.npz'^
"
pause
