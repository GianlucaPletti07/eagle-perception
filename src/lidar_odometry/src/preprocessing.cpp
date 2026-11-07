#include "lidar_odometry/preprocessing.hpp"

#include <memory>

#include <pcl/common/point_tests.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

namespace lidar_odometry
{

Cloud::Ptr cropToVehicleFrame(
  const Cloud & scan, const Eigen::Isometry3d & base_from_sensor, const CropParameters & params)
{
  const Eigen::Isometry3f transform = base_from_sensor.cast<float>();
  const double max_range_squared = params.max_range * params.max_range;

  auto cropped = std::make_shared<Cloud>();
  cropped->reserve(scan.size());

  // A single pass: each point is transformed once and either kept or dropped.
  for (const auto & sensor_point : scan) {
    if (!pcl::isFinite(sensor_point)) {
      continue;
    }
    const Eigen::Vector3f point = transform * sensor_point.getVector3fMap();
    const bool is_ground = point.z() <= params.ground_max_height;
    const bool is_too_far = point.head<2>().squaredNorm() > max_range_squared;
    if (is_ground || is_too_far) {
      continue;
    }
    cropped->push_back(PointT(point.x(), point.y(), point.z()));
  }
  return cropped;
}

Cloud::Ptr downsample(const Cloud::ConstPtr & cloud, double leaf_size)
{
  const auto leaf = static_cast<float>(leaf_size);
  pcl::VoxelGrid<PointT> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(leaf, leaf, leaf);

  auto downsampled = std::make_shared<Cloud>();
  voxel_grid.filter(*downsampled);
  return downsampled;
}

Cloud transformCloud(const Cloud & cloud, const Eigen::Isometry3d & target_from_source)
{
  const Eigen::Matrix4f matrix = target_from_source.matrix().cast<float>();
  Cloud transformed;
  pcl::transformPointCloud(cloud, transformed, matrix);
  return transformed;
}

}  // namespace lidar_odometry
