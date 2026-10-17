cmake_minimum_required(VERSION 3.0.2)
project(ultrafast_lidar_mpc)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic -Werror=return-type)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ultrafast_lidar_core
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(ultrafast_lidar_core
  src/config.cpp
  src/rolling_grid.cpp
  src/scan_geometry.cpp
  src/scan_matcher.cpp
  src/structural_perception.cpp
)
target_link_libraries(ultrafast_lidar_core ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
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
endif()
