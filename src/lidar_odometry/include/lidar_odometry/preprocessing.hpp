#pragma once

#include "Eigen/Geometry"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace lidar_odometry
{

using PointT = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointT>;

/// Part of the scan kept for registration. Distances in metres, in the vehicle frame.
struct CropParameters
{
  /// Horizontal distance from base_link beyond which points are dropped.
  double max_range;
  /// Points at or below this height are treated as ground.
  double ground_max_height;
};

/// Moves the scan into the vehicle frame and keeps only the points that help alignment:
/// no ground and nothing too far away to be reliable.
Cloud::Ptr cropToVehicleFrame(
  const Cloud & scan, const Eigen::Isometry3d & base_from_sensor, const CropParameters & params);

/// Keeps one point per voxel, so registration cost no longer depends on the point density.
Cloud::Ptr downsample(const Cloud::ConstPtr & cloud, double leaf_size);

/// Expresses a cloud in another frame, e.g. a scan seen from a pose moved into odom.
Cloud transformCloud(const Cloud & cloud, const Eigen::Isometry3d & target_from_source);

}  // namespace lidar_odometry
