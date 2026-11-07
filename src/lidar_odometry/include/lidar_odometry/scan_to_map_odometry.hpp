#pragma once

#include <Eigen/Geometry>
#include <pcl/registration/icp.h>

#include "lidar_odometry/local_map.hpp"
#include "lidar_odometry/preprocessing.hpp"

namespace lidar_odometry
{

struct RegistrationParameters
{
  double max_correspondence_distance;  ///< Point pairs farther apart than this (m) are ignored.
  int max_iterations;                  ///< Upper bound on ICP iterations per scan.
  double transformation_epsilon;       ///< ICP stops once an iteration changes the pose less than this.
};

struct RegistrationResult
{
  bool converged;
  bool keyframe_added;  ///< The scan became a keyframe of the local map.
};

/// Tracks the vehicle pose by aligning each scan against a local map of recent keyframes.
/// Motion is estimated in the ground plane only (x, y, yaw), since the ground is flat.
class ScanToMapOdometry
{
public:
  ScanToMapOdometry(
    const RegistrationParameters & registration, const LocalMapParameters & local_map);

  /// Applies new settings from the next scan on, keeping the current pose and map.
  void setParameters(
    const RegistrationParameters & registration, const LocalMapParameters & local_map);

  /// Updates the pose with a preprocessed scan expressed in the vehicle frame.
  RegistrationResult update(const Cloud::ConstPtr & scan_in_base);

  /// Current pose of base_link in the odom frame.
  const Eigen::Isometry3d & pose() const {return odom_from_base_;}

private:
  void configureIcp(const RegistrationParameters & registration);
  void addKeyframe(const Cloud & scan_in_base);

  pcl::IterativeClosestPoint<PointT, PointT> icp_;
  LocalMap local_map_;
  Eigen::Isometry3d odom_from_base_;
  Eigen::Isometry3d last_motion_;  ///< Motion over the previous scan interval, used as prediction.
};

}  // namespace lidar_odometry
