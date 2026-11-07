#include "lidar_odometry/local_map.hpp"

#include <memory>

namespace lidar_odometry
{

LocalMap::LocalMap(const LocalMapParameters & params)
: params_(params),
  last_keyframe_pose_(Eigen::Isometry3d::Identity()),
  cloud_(std::make_shared<Cloud>())
{
}

void LocalMap::setParameters(const LocalMapParameters & params)
{
  params_ = params;
}

bool LocalMap::empty() const
{
  return keyframes_.empty();
}

bool LocalMap::isNewKeyframe(const Eigen::Isometry3d & odom_from_base) const
{
  if (empty()) {
    return true;
  }
  const Eigen::Isometry3d motion = last_keyframe_pose_.inverse() * odom_from_base;
  const double distance = motion.translation().norm();
  const double angle = Eigen::AngleAxisd(motion.rotation()).angle();
  return distance >= params_.keyframe_distance || angle >= params_.keyframe_angle;
}

void LocalMap::addKeyframe(const Cloud & scan_in_base, const Eigen::Isometry3d & odom_from_base)
{
  keyframes_.push_back(std::make_shared<Cloud>(transformCloud(scan_in_base, odom_from_base)));
  while (keyframes_.size() > params_.max_keyframes) {
    keyframes_.pop_front();
  }
  last_keyframe_pose_ = odom_from_base;
  rebuildCloud();
}

Cloud::ConstPtr LocalMap::cloud() const
{
  return cloud_;
}

void LocalMap::rebuildCloud()
{
  auto merged = std::make_shared<Cloud>();
  for (const auto & keyframe : keyframes_) {
    *merged += *keyframe;
  }
  // Overlapping keyframes repeat the same surfaces: thinning keeps the map light to search.
  cloud_ = downsample(merged, params_.voxel_leaf_size);
}

}  // namespace lidar_odometry
