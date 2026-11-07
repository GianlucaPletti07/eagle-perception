#include "lidar_odometry/track_map.hpp"

#include <memory>

namespace lidar_odometry
{

TrackMap::TrackMap(double voxel_leaf_size)
: voxel_leaf_size_(voxel_leaf_size),
  cloud_(std::make_shared<Cloud>())
{
}

void TrackMap::add(const Cloud & scan_in_base, const Eigen::Isometry3d & odom_from_base)
{
  *cloud_ += transformCloud(scan_in_base, odom_from_base);
  // Where the lap overlaps itself the same surfaces are seen twice: thinning merges them.
  cloud_ = downsample(cloud_, voxel_leaf_size_);
}

Cloud::ConstPtr TrackMap::cloud() const
{
  return cloud_;
}

}  // namespace lidar_odometry
