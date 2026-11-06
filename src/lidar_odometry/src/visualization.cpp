#include "lidar_odometry/visualization.hpp"

#include <std_msgs/msg/color_rgba.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace lidar_odometry
{
namespace
{

/// Arrow shaft and head width relative to its length.
constexpr double kArrowWidthRatio = 0.25;

std_msgs::msg::ColorRGBA makeColor(float red, float green, float blue)
{
  std_msgs::msg::ColorRGBA color;
  color.r = red;
  color.g = green;
  color.b = blue;
  color.a = 1.0F;
  return color;
}

}  // namespace

visualization_msgs::msg::Marker makeTrajectoryMarker(
  const std_msgs::msg::Header & header, const std::vector<geometry_msgs::msg::Point> & positions,
  double line_width)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "trajectory";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = line_width;
  marker.color = makeColor(0.1F, 0.6F, 1.0F);
  marker.points = positions;
  return marker;
}

visualization_msgs::msg::Marker makeVehicleMarker(
  const std_msgs::msg::Header & header, const Eigen::Isometry3d & odom_from_base,
  double arrow_length)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "vehicle";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = tf2::toMsg(odom_from_base);
  marker.scale.x = arrow_length;
  marker.scale.y = arrow_length * kArrowWidthRatio;
  marker.scale.z = arrow_length * kArrowWidthRatio;
  marker.color = makeColor(1.0F, 0.5F, 0.0F);
  return marker;
}

}  // namespace lidar_odometry