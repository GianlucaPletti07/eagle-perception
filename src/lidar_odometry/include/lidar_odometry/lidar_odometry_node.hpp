#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace lidar_odometry
{

/// Estimates the vehicle motion over time by aligning each LiDAR scan with the previous ones.
class LidarOdometryNode : public rclcpp::Node
{
public:
  explicit LidarOdometryNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onScan(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & scan);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_subscription_;
};

}  // namespace lidar_odometry