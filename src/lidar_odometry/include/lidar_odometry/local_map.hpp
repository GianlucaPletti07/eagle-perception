#pragma once

#include <cstddef>
#include <deque>

#include "Eigen/Geometry"

#include "lidar_odometry/preprocessing.hpp"

namespace lidar_odometry
{

struct LocalMapParameters
{
  /// Travel in metres after which the current scan becomes a keyframe.
  double keyframe_distance;
  /// Rotation in radians after which the current scan becomes a keyframe.
  double keyframe_angle;
  /// Older keyframes are dropped, bounding cost and memory.
  std::size_t max_keyframes;
  /// Leaf used to thin the merged keyframes.
  double voxel_leaf_size;
};

/// Sliding window of the most recent keyframes, merged into one cloud in the odom frame.
/// Aligning each scan against it, instead of against the previous scan only, reduces drift.
class LocalMap
{
public:
  explicit LocalMap(const LocalMapParameters & params);

  /// New settings apply from the next keyframe on.
  void setParameters(const LocalMapParameters & params);

  bool empty() const;

  /// True when the vehicle moved or turned enough since the last keyframe.
  bool isNewKeyframe(const Eigen::Isometry3d & odom_from_base) const;

  /// Adds a scan, given in the vehicle frame, as seen from the given pose.
  void addKeyframe(const Cloud & scan_in_base, const Eigen::Isometry3d & odom_from_base);

  /// Merged keyframes in the odom frame.
  Cloud::ConstPtr cloud() const;

private:
  void rebuildCloud();

  LocalMapParameters params_;
  std::deque<Cloud::Ptr> keyframes_;
  Eigen::Isometry3d last_keyframe_pose_;
  Cloud::Ptr cloud_;
};

}  // namespace lidar_odometry
