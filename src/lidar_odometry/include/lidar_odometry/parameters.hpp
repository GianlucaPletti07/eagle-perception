#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

#include "lidar_odometry/local_map.hpp"
#include "lidar_odometry/preprocessing.hpp"
#include "lidar_odometry/scan_to_map_odometry.hpp"

namespace lidar_odometry
{

/// Settings fixed at startup: changing them would need new subscriptions, frames or maps.
struct StartupParameters
{
  std::string input_topic;
  int scan_queue_depth;
  std::string base_frame;
  std::string odom_frame;
  bool track_map_enabled;
  double track_map_voxel_leaf_size;
};

/// Settings that can also be changed while the node runs, e.g. from rqt_reconfigure.
struct TunableParameters
{
  CropParameters crop;
  double voxel_leaf_size;
  RegistrationParameters registration;
  LocalMapParameters local_map;
  bool publish_filtered_cloud;
  double trajectory_line_width;
  double vehicle_arrow_length;
};

/// Declares the startup parameters as read-only and returns their values.
StartupParameters declareStartupParameters(rclcpp::Node & node);

/// Declares the tunable parameters together with their valid ranges.
void declareTunableParameters(rclcpp::Node & node);

/// Reads the current values of the tunable parameters.
TunableParameters readTunableParameters(const rclcpp::Node & node);

}  // namespace lidar_odometry