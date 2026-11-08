#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "lidar_odometry/local_map.hpp"
#include "lidar_odometry/preprocessing.hpp"
#include "lidar_odometry/scan_to_map_odometry.hpp"
#include "lidar_odometry/visualization.hpp"

namespace lidar_odometry
{

/// Settings fixed at startup: changing them would need new subscriptions, frames or maps.
struct StartupParameters
{
  std::string input_topic;
  std::string base_frame;
  std::string odom_frame;
  int scan_queue_depth;
  int output_queue_depth;
  int track_map_queue_depth;
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
  MarkerStyle markers;
};

/// Declares the startup parameters as read-only and returns their values.
StartupParameters loadStartupParameters(rclcpp::Node & node);

/// Declares the tunable parameters with their valid ranges on the first call, and returns
/// their current values on every call.
TunableParameters loadTunableParameters(rclcpp::Node & node);

}  // namespace lidar_odometry
