cmake_minimum_required(VERSION 3.8)
project(reactive_robot_sim)

find_package(ament_cmake REQUIRED)

# Simülasyon varlıklarını (launch, urdf, worlds, models, rviz) paylaşılan
# install dizinine kur; böylece $(find reactive_robot_sim) ve
# get_package_share_directory('reactive_robot_sim') çalışır.
install(
  DIRECTORY launch urdf worlds models rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
