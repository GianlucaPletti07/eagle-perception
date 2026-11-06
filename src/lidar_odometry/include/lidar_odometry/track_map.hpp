#pragma once

#include <Eigen/Geometry>

#include "lidar_odometry/preprocessing.hpp"

namespace lidar_odometry
{

/// Point cloud of the whole track, built from every keyframe and its estimated pose.
/// Unlike the local map it never forgets: it is meant for inspection, not for registration.
class TrackMap
{
public:
  explicit TrackMap(double voxel_leaf_size);

  /// Adds a scan, given in the vehicle frame, as seen from the given pose.
  void add(const Cloud & scan_in_base, const Eigen::Isometry3d & odom_from_base);

  /// The whole map in the odom frame.
  Cloud::ConstPtr cloud() const;

private:
  double voxel_leaf_size_;
  Cloud::Ptr cloud_;
};

}  // namespace lidar_odometry