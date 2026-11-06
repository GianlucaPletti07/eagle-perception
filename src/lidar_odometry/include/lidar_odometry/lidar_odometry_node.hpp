#pragma once

#include <memory>
#include <optional>
#include <string>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace lidar_odometry
{

/// Estimates the vehicle motion over time by aligning each LiDAR scan with the previous ones.
class LidarOdometryNode : public rclcpp::Node
{
public:
  explicit LidarOdometryNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onScan(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & scan);

  /// Caches the fixed sensor mounting from /tf_static. Returns false until it is available.
  bool ensureSensorExtrinsics(const std::string & sensor_frame);

  std::string base_frame_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::optional<Eigen::Isometry3d> base_from_sensor_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_subscription_;
};

}  // namespace lidar_odometry 