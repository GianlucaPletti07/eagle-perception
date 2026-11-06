cmake_minimum_required(VERSION 3.8)
project(lidar_odometry)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# The task requires a build without warnings, so make the compiler strict.
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(lidar_odometry_node
  src/main.cpp
  src/lidar_odometry_node.cpp
)
target_include_directories(lidar_odometry_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(lidar_odometry_node
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_eigen
)
target_link_libraries(lidar_odometry_node Eigen3::Eigen)

install(TARGETS lidar_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()