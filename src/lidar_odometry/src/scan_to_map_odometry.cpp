#include "lidar_odometry/scan_to_map_odometry.hpp"

#include <cmath>
#include <memory>

#include <pcl/registration/transformation_estimation_2D.h>

namespace lidar_odometry
{

namespace
{

/// Rebuilds a pose from its planar components only. Composing float ICP results scan after
/// scan slowly breaks the orthogonality of the rotation; this keeps it exact and z at zero.
Eigen::Isometry3d planarPose(const Eigen::Matrix4f & transform)
{
  const double yaw = std::atan2(transform(1, 0), transform(0, 0));
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(transform(0, 3), transform(1, 3), 0.0);
  pose.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return pose;
}

}  // namespace

ScanToMapOdometry::ScanToMapOdometry(
  const RegistrationParameters & registration, const LocalMapParameters & local_map)
: local_map_(local_map),
  odom_from_base_(Eigen::Isometry3d::Identity()),
  last_motion_(Eigen::Isometry3d::Identity())
{
  // The ground is flat, so only x, y and yaw are estimated: z, roll and pitch cannot drift.
  icp_.setTransformationEstimation(
    std::make_shared<pcl::registration::TransformationEstimation2D<PointT, PointT>>());
  icp_.setMaxCorrespondenceDistance(registration.max_correspondence_distance);
  icp_.setMaximumIterations(registration.max_iterations);
  icp_.setTransformationEpsilon(registration.transformation_epsilon);
}

RegistrationResult ScanToMapOdometry::update(const Cloud::ConstPtr & scan_in_base)
{
  if (local_map_.empty()) {
    // The first scan defines the odom frame: the car starts at its origin, facing +x.
    addKeyframe(*scan_in_base);
    return {true, 0.0};
  }

  // Constant velocity: the car is assumed to repeat the motion of the previous interval.
  const Eigen::Isometry3d prediction = odom_from_base_ * last_motion_;

  icp_.setInputSource(scan_in_base);
  Cloud aligned_scan;
  icp_.align(aligned_scan, prediction.matrix().cast<float>());
  const RegistrationResult result{icp_.hasConverged(), icp_.getFitnessScore()};

  if (!result.converged) {
    // Keep the prediction, and leave the map untouched so a bad scan cannot corrupt it.
    odom_from_base_ = prediction;
    return result;
  }

  const Eigen::Isometry3d previous_pose = odom_from_base_;
  odom_from_base_ = planarPose(icp_.getFinalTransformation());
  last_motion_ = previous_pose.inverse() * odom_from_base_;

  if (local_map_.isNewKeyframe(odom_from_base_)) {
    addKeyframe(*scan_in_base);
  }
  return result;
}

void ScanToMapOdometry::addKeyframe(const Cloud & scan_in_base)
{
  local_map_.addKeyframe(scan_in_base, odom_from_base_);
  // ICP rebuilds its search tree only when the target changes, i.e. once per keyframe.
  icp_.setInputTarget(local_map_.cloud());
}

}  // namespace lidar_odometry