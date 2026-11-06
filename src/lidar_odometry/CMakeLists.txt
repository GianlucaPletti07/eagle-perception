cmake_minimum_required(VERSION 3.12)
project(lidar_odometry)

# Timings are only meaningful with optimizations: default to Release unless told otherwise.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# The task requires a build without warnings, so make the compiler strict.
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters registration)
find_package(lidar_odometry_msgs REQUIRED)

add_executable(lidar_odometry_node
  src/main.cpp
  src/lidar_odometry_node.cpp
  src/local_map.cpp
  src/preprocessing.cpp
  src/scan_to_map_odometry.cpp
  src/visualization.cpp
)
target_include_directories(lidar_odometry_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
# PCL headers emit warnings we cannot fix, so they are treated as system headers.
target_include_directories(lidar_odometry_node SYSTEM PRIVATE
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(lidar_odometry_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_eigen
  pcl_conversions
  lidar_odometry_msgs
)
target_link_libraries(lidar_odometry_node
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

install(TARGETS lidar_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()