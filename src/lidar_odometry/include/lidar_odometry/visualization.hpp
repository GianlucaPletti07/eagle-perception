#pragma once

#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace lidar_odometry
{

/// Sizes in metres and colours of the markers drawn in RViz.
struct MarkerStyle
{
  double trajectory_width;
  std_msgs::msg::ColorRGBA trajectory_color;
  double vehicle_length;
  double vehicle_width;
  std_msgs::msg::ColorRGBA vehicle_color;
};

/// A line through every position estimated so far and an arrow at the current vehicle pose.
visualization_msgs::msg::MarkerArray makeMarkers(
  const std_msgs::msg::Header & header, const std::vector<geometry_msgs::msg::Point> & trajectory,
  const Eigen::Isometry3d & odom_from_base, const MarkerStyle & style);

}  // namespace lidar_odometry
