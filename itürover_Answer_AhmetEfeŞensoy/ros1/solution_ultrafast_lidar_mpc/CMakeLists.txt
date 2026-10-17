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

add_library(ultrafast_lidar_core src/config.cpp)
target_link_libraries(ultrafast_lidar_core ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_config test/test_config.cpp)
  if(TARGET test_config)
    target_link_libraries(test_config ultrafast_lidar_core)
  endif()
endif()
