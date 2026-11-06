#include "lidar_odometry/lidar_odometry_node.hpp"

namespace lidar_odometry
{

LidarOdometryNode::LidarOdometryNode(const rclcpp::NodeOptions & options)
: Node("lidar_odometry", options)
{
  // Scans are large: with a best-effort subscription most of them would be silently lost.
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  scan_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/lidar/points", qos,
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & scan) {onScan(scan);});
}

void LidarOdometryNode::onScan(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & scan)
{
  RCLCPP_INFO(
    get_logger(), "Received scan with %u points in frame '%s'",
    scan->width * scan->height, scan->header.frame_id.c_str());
}

}  // namespace lidar_odometry