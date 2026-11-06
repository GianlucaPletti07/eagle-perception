#include "lidar_odometry/lidar_odometry_node.hpp"

namespace lidar_odometry
{

LidarOdometryNode::LidarOdometryNode(const rclcpp::NodeOptions & options)
: Node("lidar_odometry", options)
{
  const auto input_topic = declare_parameter<std::string>("input_topic");
  const auto scan_queue_depth = declare_parameter<int>("scan_queue_depth");

  // Scans are large: with a best-effort subscription most of them would be silently lost.
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(scan_queue_depth))).reliable();
  scan_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic,
    qos,
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & scan) {onScan(scan);});
}

void LidarOdometryNode::onScan(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & scan)
{
  RCLCPP_INFO(
    get_logger(), "Received scan with %u points in frame '%s'",
    scan->width * scan->height, scan->header.frame_id.c_str());
}

}  // namespace lidar_odometry