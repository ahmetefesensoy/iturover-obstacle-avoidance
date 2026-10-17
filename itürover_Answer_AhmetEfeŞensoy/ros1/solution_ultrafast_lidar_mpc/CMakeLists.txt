cmake_minimum_required(VERSION 3.0.2)
project(ultrafast_lidar_mpc)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic -Werror=return-type)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ultrafast_lidar_core
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs std_msgs
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(ultrafast_lidar_core
  src/config.cpp
  src/planner.cpp
  src/race_controller.cpp
  src/rolling_grid.cpp
  src/safety_filter.cpp
  src/scan_geometry.cpp
  src/scan_matcher.cpp
  src/structural_perception.cpp
  src/trajectory.cpp
)
target_link_libraries(ultrafast_lidar_core ${catkin_LIBRARIES})

add_executable(ultrafast_lidar_mpc_node src/race_controller_node.cpp)
target_link_libraries(ultrafast_lidar_mpc_node ultrafast_lidar_core ${catkin_LIBRARIES})

install(TARGETS ultrafast_lidar_core ultrafast_lidar_mpc_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_config test/test_config.cpp)
  if(TARGET test_config)
    target_link_libraries(test_config ultrafast_lidar_core)
  endif()

  catkin_add_gtest(test_scan_geometry test/test_scan_geometry.cpp)
  if(TARGET test_scan_geometry)
    target_link_libraries(test_scan_geometry ultrafast_lidar_core)
  endif()

  catkin_add_gtest(test_scan_matcher test/test_scan_matcher.cpp)
  if(TARGET test_scan_matcher)
    target_link_libraries(test_scan_matcher ultrafast_lidar_core)
  endif()

  catkin_add_gtest(test_rolling_grid test/test_rolling_grid.cpp)
  if(TARGET test_rolling_grid)
    target_link_libraries(test_rolling_grid ultrafast_lidar_core)
  endif()

  catkin_add_gtest(test_structural_perception test/test_structural_perception.cpp)
  if(TARGET test_structural_perception)
    target_link_libraries(test_structural_perception ultrafast_lidar_core)
  endif()

  catkin_add_gtest(test_trajectory test/test_trajectory.cpp)
  if(TARGET test_trajectory)
    target_link_libraries(test_trajectory ultrafast_lidar_core)
  endif()

  catkin_add_gtest(test_planner test/test_planner.cpp)
  if(TARGET test_planner)
    target_link_libraries(test_planner ultrafast_lidar_core)
  endif()

  catkin_add_gtest(test_safety_filter test/test_safety_filter.cpp)
  if(TARGET test_safety_filter)
    target_link_libraries(test_safety_filter ultrafast_lidar_core)
  endif()

  catkin_add_gtest(test_race_controller test/test_race_controller.cpp)
  if(TARGET test_race_controller)
    target_link_libraries(test_race_controller ultrafast_lidar_core)
  endif()

  add_rostest(test/topic_contract.test)
endif()

catkin_install_python(PROGRAMS test/test_topic_contract.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
