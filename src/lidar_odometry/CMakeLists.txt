cmake_minimum_required(VERSION 3.8)
project(lidar_odometry)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# The task requires a build without warnings, so make the compiler strict.
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

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
)

install(TARGETS lidar_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()