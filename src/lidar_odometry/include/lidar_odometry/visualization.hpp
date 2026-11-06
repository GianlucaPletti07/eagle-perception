#pragma once

#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace lidar_odometry
{

/// Line through every position estimated so far.
visualization_msgs::msg::Marker makeTrajectoryMarker(
  const std_msgs::msg::Header & header, const std::vector<geometry_msgs::msg::Point> & positions,
  double line_width);

/// Arrow at the current vehicle pose, pointing where the car is heading.
visualization_msgs::msg::Marker makeVehicleMarker(
  const std_msgs::msg::Header & header, const Eigen::Isometry3d & odom_from_base,
  double arrow_length);

}  // namespace lidar_odometry