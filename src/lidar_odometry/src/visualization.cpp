#include "lidar_odometry/visualization.hpp"

#include <cstdint>
#include <string>

#include "tf2_eigen/tf2_eigen.hpp"

namespace lidar_odometry
{
namespace
{

using visualization_msgs::msg::Marker;

/// Republishing the same namespace and id makes RViz update a marker instead of adding one.
Marker makeMarker(const std_msgs::msg::Header & header, const std::string & name, int32_t type)
{
  Marker marker;
  marker.header = header;
  marker.ns = name;
  marker.id = 0;
  marker.type = type;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  return marker;
}

}  // namespace

visualization_msgs::msg::MarkerArray makeMarkers(
  const std_msgs::msg::Header & header, const std::vector<geometry_msgs::msg::Point> & trajectory,
  const Eigen::Isometry3d & odom_from_base, const MarkerStyle & style)
{
  Marker trajectory_marker = makeMarker(header, "trajectory", Marker::LINE_STRIP);
  trajectory_marker.scale.x = style.trajectory_width;
  trajectory_marker.color = style.trajectory_color;
  trajectory_marker.points = trajectory;

  Marker vehicle_marker = makeMarker(header, "vehicle", Marker::ARROW);
  vehicle_marker.pose = tf2::toMsg(odom_from_base);
  vehicle_marker.scale.x = style.vehicle_length;
  vehicle_marker.scale.y = style.vehicle_width;
  vehicle_marker.scale.z = style.vehicle_width;
  vehicle_marker.color = style.vehicle_color;

  visualization_msgs::msg::MarkerArray markers;
  markers.markers = {trajectory_marker, vehicle_marker};
  return markers;
}

}  // namespace lidar_odometry
