#include "lidar_odometry/lidar_odometry_node.hpp"

#include <tf2_eigen/tf2_eigen.hpp>

namespace lidar_odometry
{

LidarOdometryNode::LidarOdometryNode(const rclcpp::NodeOptions & options)
: Node("lidar_odometry", options)
{
  const auto input_topic = declare_parameter<std::string>("input_topic");
  const auto scan_queue_depth = declare_parameter<int>("scan_queue_depth");
  base_frame_ = declare_parameter<std::string>("base_frame");

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Scans are large: with a best-effort subscription most of them would be silently lost.
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(scan_queue_depth))).reliable();
  scan_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic, qos,
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & scan) {onScan(scan);});
}

void LidarOdometryNode::onScan(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & scan)
{
  if (!ensureSensorExtrinsics(scan->header.frame_id)) {
    return;
  }

  RCLCPP_INFO(
    get_logger(), "Received scan with %u points in frame '%s'",
    scan->width * scan->height, scan->header.frame_id.c_str());
}

bool LidarOdometryNode::ensureSensorExtrinsics(const std::string & sensor_frame)
{
  if (base_from_sensor_) {
    return true;
  }

  try {
    const auto transform = tf_buffer_->lookupTransform(base_frame_, sensor_frame, tf2::TimePointZero);
    base_from_sensor_ = tf2::transformToEigen(transform);
  } catch (const tf2::TransformException & error) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "Waiting for transform %s -> %s: %s",
      base_frame_.c_str(), sensor_frame.c_str(), error.what());
    return false;
  }

  const Eigen::Vector3d & offset = base_from_sensor_->translation();
  RCLCPP_INFO(
    get_logger(), "Sensor mounted at (%.3f, %.3f, %.3f) m in '%s'",
    offset.x(), offset.y(), offset.z(), base_frame_.c_str());
  return true;
}

}  // namespace lidar_odometry