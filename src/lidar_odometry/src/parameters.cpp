#include "lidar_odometry/parameters.hpp"

#include <cstddef>
#include <cstdint>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace lidar_odometry
{
namespace
{

using rcl_interfaces::msg::ParameterDescriptor;

// Names of the tunable parameters, shared by declaration and reading.
constexpr char kMaxRange[] = "preprocessing.max_range";
constexpr char kGroundMaxHeight[] = "preprocessing.ground_max_height";
constexpr char kScanVoxelLeafSize[] = "preprocessing.voxel_leaf_size";
constexpr char kMaxCorrespondenceDistance[] = "registration.max_correspondence_distance";
constexpr char kMaxIterations[] = "registration.max_iterations";
constexpr char kTransformationEpsilon[] = "registration.transformation_epsilon";
constexpr char kKeyframeDistance[] = "local_map.keyframe_distance";
constexpr char kKeyframeAngle[] = "local_map.keyframe_angle";
constexpr char kMaxKeyframes[] = "local_map.max_keyframes";
constexpr char kMapVoxelLeafSize[] = "local_map.voxel_leaf_size";
constexpr char kPublishFilteredCloud[] = "publish_filtered_cloud";
constexpr char kTrajectoryLineWidth[] = "visualization.trajectory_line_width";
constexpr char kVehicleArrowLength[] = "visualization.vehicle_arrow_length";

ParameterDescriptor described(const std::string & description)
{
  ParameterDescriptor descriptor;
  descriptor.description = description;
  return descriptor;
}

ParameterDescriptor readOnly(const std::string & description)
{
  ParameterDescriptor descriptor = described(description);
  descriptor.read_only = true;
  return descriptor;
}

ParameterDescriptor floatingRange(const std::string & description, double from, double to)
{
  ParameterDescriptor descriptor = described(description);
  descriptor.floating_point_range.resize(1);
  descriptor.floating_point_range[0].from_value = from;
  descriptor.floating_point_range[0].to_value = to;
  return descriptor;
}

ParameterDescriptor integerRange(const std::string & description, int64_t from, int64_t to)
{
  ParameterDescriptor descriptor = described(description);
  descriptor.integer_range.resize(1);
  descriptor.integer_range[0].from_value = from;
  descriptor.integer_range[0].to_value = to;
  descriptor.integer_range[0].step = 1;
  return descriptor;
}

}  // namespace

StartupParameters declareStartupParameters(rclcpp::Node & node)
{
  StartupParameters params;
  params.input_topic = node.declare_parameter<std::string>(
    "input_topic", readOnly("PointCloud2 topic with the LiDAR scans"));
  params.scan_queue_depth = node.declare_parameter<int>(
    "scan_queue_depth", readOnly("Scans kept in the subscription queue if processing falls behind"));
  params.base_frame = node.declare_parameter<std::string>(
    "base_frame", readOnly("Vehicle frame, on the ground at the rear axle"));
  params.odom_frame = node.declare_parameter<std::string>(
    "odom_frame", readOnly("Fixed frame where the vehicle starts"));
  params.track_map_enabled = node.declare_parameter<bool>(
    "track_map.enabled", readOnly("Accumulate keyframes into a map of the whole track"));
  params.track_map_voxel_leaf_size = node.declare_parameter<double>(
    "track_map.voxel_leaf_size", readOnly("Voxel edge (m) used to thin the track map"));
  return params;
}

void declareTunableParameters(rclcpp::Node & node)
{
  node.declare_parameter<double>(
    kMaxRange, floatingRange("Horizontal distance (m) beyond which points are dropped", 1.0, 200.0));
  node.declare_parameter<double>(
    kGroundMaxHeight, floatingRange("Points at or below this height (m) are ground", -0.5, 1.0));
  node.declare_parameter<double>(
    kScanVoxelLeafSize, floatingRange("Voxel edge (m) used to thin each scan", 0.01, 2.0));
  node.declare_parameter<double>(
    kMaxCorrespondenceDistance,
    floatingRange("Point pairs farther apart (m) are not matched by ICP", 0.05, 10.0));
  node.declare_parameter<int>(
    kMaxIterations, integerRange("Upper bound on ICP iterations per scan", 1, 200));
  node.declare_parameter<double>(
    kTransformationEpsilon,
    floatingRange("ICP stops when an iteration changes the pose less than this", 1e-12, 1.0));
  node.declare_parameter<double>(
    kKeyframeDistance, floatingRange("Travel (m) after which a scan becomes a keyframe", 0.1, 50.0));
  node.declare_parameter<double>(
    kKeyframeAngle, floatingRange("Rotation (rad) after which a scan becomes a keyframe", 0.01, 3.14));
  node.declare_parameter<int>(
    kMaxKeyframes, integerRange("Recent keyframes the scans are aligned against", 1, 100));
  node.declare_parameter<double>(
    kMapVoxelLeafSize, floatingRange("Voxel edge (m) used to thin the local map", 0.01, 2.0));
  node.declare_parameter<bool>(
    kPublishFilteredCloud, described("Publish the preprocessed scan on ~/filtered_points"));
  node.declare_parameter<double>(
    kTrajectoryLineWidth, floatingRange("Width (m) of the trajectory marker", 0.01, 2.0));
  node.declare_parameter<double>(
    kVehicleArrowLength, floatingRange("Length (m) of the vehicle arrow marker", 0.1, 10.0));
}

TunableParameters readTunableParameters(const rclcpp::Node & node)
{
  TunableParameters params;
  params.crop.max_range = node.get_parameter(kMaxRange).as_double();
  params.crop.ground_max_height = node.get_parameter(kGroundMaxHeight).as_double();
  params.voxel_leaf_size = node.get_parameter(kScanVoxelLeafSize).as_double();
  params.registration.max_correspondence_distance =
    node.get_parameter(kMaxCorrespondenceDistance).as_double();
  params.registration.max_iterations =
    static_cast<int>(node.get_parameter(kMaxIterations).as_int());
  params.registration.transformation_epsilon =
    node.get_parameter(kTransformationEpsilon).as_double();
  params.local_map.keyframe_distance = node.get_parameter(kKeyframeDistance).as_double();
  params.local_map.keyframe_angle = node.get_parameter(kKeyframeAngle).as_double();
  params.local_map.max_keyframes =
    static_cast<std::size_t>(node.get_parameter(kMaxKeyframes).as_int());
  params.local_map.voxel_leaf_size = node.get_parameter(kMapVoxelLeafSize).as_double();
  params.publish_filtered_cloud = node.get_parameter(kPublishFilteredCloud).as_bool();
  params.trajectory_line_width = node.get_parameter(kTrajectoryLineWidth).as_double();
  params.vehicle_arrow_length = node.get_parameter(kVehicleArrowLength).as_double();
  return params;
}

}  // namespace lidar_odometry