#pragma once

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_odometry
{

using PointT = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointT>;

/// Part of the scan kept for registration. Distances in metres, in the vehicle frame.
struct CropParameters
{
  double max_range;          ///< Horizontal distance from base_link beyond which points are dropped.
  double ground_max_height;  ///< Points at or below this height are treated as ground.
};

/// Moves the scan into the vehicle frame and keeps only the points that help alignment:
/// no ground and nothing too far away to be reliable.
Cloud::Ptr cropToVehicleFrame(
  const Cloud & scan, const Eigen::Isometry3d & base_from_sensor, const CropParameters & params);

/// Moves the scan into the vehicle frame and keeps only the points that help alignment:
/// no ground, no returns from the car itself, nothing too far away to be reliable.
Cloud::Ptr cropToVehicleFrame(
  const Cloud & scan, const Eigen::Isometry3d & base_from_sensor, const CropParameters & params);

/// Keeps one point per voxel, so registration cost no longer depends on the point density.
Cloud::Ptr downsample(const Cloud::ConstPtr & cloud, double leaf_size);

}  // namespace lidar_odometry