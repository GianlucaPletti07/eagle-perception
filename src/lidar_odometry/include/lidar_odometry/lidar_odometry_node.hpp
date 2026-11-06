#pragma once

#include <memory>
#include <optional>
#include <string>

#include <Eigen/Geometry>
#include <lidar_odometry_msgs/msg/processing_time.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "lidar_odometry/preprocessing.hpp"
#include "lidar_odometry/scan_to_map_odometry.hpp"

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

  CropParameters loadCropParameters();
  RegistrationParameters loadRegistrationParameters();
  LocalMapParameters loadLocalMapParameters();

  void publishOdometry(const std_msgs::msg::Header & scan_header);
  void publishFilteredCloud(const Cloud & cloud, const std_msgs::msg::Header & scan_header);

  std::string base_frame_;
  std::string odom_frame_;
  CropParameters crop_params_;
  double voxel_leaf_size_;
  bool publish_filtered_cloud_;

  std::unique_ptr<ScanToMapOdometry> odometry_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::optional<Eigen::Isometry3d> base_from_sensor_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_publisher_;
  rclcpp::Publisher<lidar_odometry_msgs::msg::ProcessingTime>::SharedPtr processing_time_publisher_;
};

}  // namespace lidar_odometry