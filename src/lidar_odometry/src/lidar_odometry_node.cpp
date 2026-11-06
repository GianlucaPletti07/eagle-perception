#include "lidar_odometry/lidar_odometry_node.hpp"

#include <chrono>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace lidar_odometry
{
namespace
{

using Clock = std::chrono::steady_clock;

double millisecondsBetween(Clock::time_point from, Clock::time_point to)
{
  return std::chrono::duration<double, std::milli>(to - from).count();
}

}  // namespace

LidarOdometryNode::LidarOdometryNode(const rclcpp::NodeOptions & options)
: Node("lidar_odometry", options)
{
  const auto input_topic = declare_parameter<std::string>("input_topic");
  const auto scan_queue_depth = declare_parameter<int>("scan_queue_depth");
  base_frame_ = declare_parameter<std::string>("base_frame");
  crop_params_ = loadCropParameters();
  voxel_leaf_size_ = declare_parameter<double>("preprocessing.voxel_leaf_size");
  publish_filtered_cloud_ = declare_parameter<bool>("publish_filtered_cloud");

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Debug output: only the most recent filtered cloud is worth showing.
  filtered_cloud_publisher_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("~/filtered_points", 1);
  processing_time_publisher_ =
    create_publisher<lidar_odometry_msgs::msg::ProcessingTime>("~/processing_time", 10);

  // Scans are large: with a best-effort subscription most of them would be silently lost.
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(scan_queue_depth))).reliable();
  scan_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic, qos,
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & scan) {onScan(scan);});
}

void LidarOdometryNode::onScan(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & scan)
{
  const auto start = Clock::now();
  if (!ensureSensorExtrinsics(scan->header.frame_id)) {
    return;
  }

  Cloud raw_scan;
  pcl::fromROSMsg(*scan, raw_scan);
  const Cloud::Ptr filtered_scan =
    downsample(cropToVehicleFrame(raw_scan, *base_from_sensor_, crop_params_), voxel_leaf_size_);
  const auto preprocessing_end = Clock::now();

  if (publish_filtered_cloud_) {
    publishFilteredCloud(*filtered_scan, scan->header);
  }

  lidar_odometry_msgs::msg::ProcessingTime timing;
  timing.header.stamp = scan->header.stamp;
  timing.header.frame_id = base_frame_;
  timing.preprocessing_ms = millisecondsBetween(start, preprocessing_end);
  timing.input_points = static_cast<uint32_t>(raw_scan.size());
  timing.filtered_points = static_cast<uint32_t>(filtered_scan->size());
  timing.total_ms = millisecondsBetween(start, Clock::now());
  processing_time_publisher_->publish(timing);
}

bool LidarOdometryNode::ensureSensorExtrinsics(const std::string & sensor_frame)
{
  if (base_from_sensor_) {
    return true;
  }

  try {
    const auto transform =
      tf_buffer_->lookupTransform(base_frame_, sensor_frame, tf2::TimePointZero);
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

CropParameters LidarOdometryNode::loadCropParameters()
{
  CropParameters params;
  params.max_range = declare_parameter<double>("preprocessing.max_range");
  params.ground_max_height = declare_parameter<double>("preprocessing.ground_max_height");
  return params;
}

void LidarOdometryNode::publishFilteredCloud(
  const Cloud & cloud, const std_msgs::msg::Header & scan_header)
{
  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  // Stamped with the scan it came from, as every output of this node.
  message.header.stamp = scan_header.stamp;
  message.header.frame_id = base_frame_;
  filtered_cloud_publisher_->publish(message);
}

}  // namespace lidar_odometry