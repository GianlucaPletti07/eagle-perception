#include "lidar_odometry/parameters.hpp"

#include <cstddef>
#include <cstdint>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "std_msgs/msg/color_rgba.hpp"

namespace lidar_odometry
{
namespace
{

using rcl_interfaces::msg::ParameterDescriptor;

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

/// Declares the parameter on the first call and reads its current value afterwards, so the
/// same code sets the parameters up at startup and picks up changes made at runtime.
template<typename T>
T declareOrGet(
  rclcpp::Node & node, const std::string & name,
  const ParameterDescriptor & descriptor)
{
  if (!node.has_parameter(name)) {
    return node.declare_parameter<T>(name, descriptor);
  }
  return node.get_parameter(name).get_value<T>();
}

double loadDouble(
  rclcpp::Node & node, const std::string & name, const std::string & description,
  double from, double to)
{
  return declareOrGet<double>(node, name, floatingRange(description, from, to));
}

int64_t loadInteger(
  rclcpp::Node & node, const std::string & name, const std::string & description,
  int64_t from, int64_t to)
{
  return declareOrGet<int64_t>(node, name, integerRange(description, from, to));
}

float loadColorChannel(
  rclcpp::Node & node, const std::string & name, const std::string & description)
{
  return static_cast<float>(loadDouble(node, name, description, 0.0, 1.0));
}

/// A colour is four parameters, one per channel, each between 0 and 1.
std_msgs::msg::ColorRGBA loadColor(
  rclcpp::Node & node, const std::string & name, const std::string & description)
{
  std_msgs::msg::ColorRGBA color;
  color.r = loadColorChannel(node, name + ".r", description + ", red channel");
  color.g = loadColorChannel(node, name + ".g", description + ", green channel");
  color.b = loadColorChannel(node, name + ".b", description + ", blue channel");
  color.a = loadColorChannel(node, name + ".a", description + ", opacity");
  return color;
}

}  // namespace

StartupParameters loadStartupParameters(rclcpp::Node & node)
{
  StartupParameters params;
  params.input_topic = node.declare_parameter<std::string>(
    "input_topic", readOnly("PointCloud2 topic with the LiDAR scans"));
  params.base_frame = node.declare_parameter<std::string>(
    "base_frame", readOnly("Vehicle frame, on the ground at the rear axle"));
  params.odom_frame = node.declare_parameter<std::string>(
    "odom_frame", readOnly("Fixed frame where the vehicle starts"));
  params.scan_queue_depth = node.declare_parameter<int>(
    "queue_depth.scans",
    readOnly("Scans kept in the subscription queue if processing falls behind"));
  params.output_queue_depth = node.declare_parameter<int>(
    "queue_depth.outputs", readOnly("Messages kept in the queue of each output topic"));
  params.track_map_queue_depth = node.declare_parameter<int>(
    "queue_depth.track_map", readOnly("Track maps kept for subscribers that join late"));
  params.track_map_enabled = node.declare_parameter<bool>(
    "track_map.enabled", readOnly("Accumulate keyframes into a map of the whole track"));
  params.track_map_voxel_leaf_size = node.declare_parameter<double>(
    "track_map.voxel_leaf_size", readOnly("Voxel edge (m) used to thin the track map"));
  return params;
}

TunableParameters loadTunableParameters(rclcpp::Node & node)
{
  TunableParameters params;

  params.crop.max_range = loadDouble(
    node, "preprocessing.max_range", "Horizontal distance (m) beyond which points are dropped",
    1.0, 200.0);
  params.crop.ground_max_height = loadDouble(
    node, "preprocessing.ground_max_height", "Points at or below this height (m) are ground",
    -0.5, 1.0);
  params.voxel_leaf_size = loadDouble(
    node, "preprocessing.voxel_leaf_size", "Voxel edge (m) used to thin each scan", 0.01, 2.0);

  params.registration.max_correspondence_distance = loadDouble(
    node, "registration.max_correspondence_distance",
    "Point pairs farther apart (m) are not matched by ICP", 0.05, 10.0);
  params.registration.max_iterations = static_cast<int>(loadInteger(
      node, "registration.max_iterations", "Upper bound on ICP iterations per scan", 1, 200));
  params.registration.transformation_epsilon = loadDouble(
    node, "registration.transformation_epsilon",
    "ICP stops when an iteration changes the pose less than this", 1e-12, 1.0);

  params.local_map.keyframe_distance = loadDouble(
    node, "local_map.keyframe_distance", "Travel (m) after which a scan becomes a keyframe",
    0.1, 50.0);
  params.local_map.keyframe_angle = loadDouble(
    node, "local_map.keyframe_angle", "Rotation (rad) after which a scan becomes a keyframe",
    0.01, 3.14);
  params.local_map.max_keyframes = static_cast<std::size_t>(loadInteger(
      node, "local_map.max_keyframes", "Recent keyframes the scans are aligned against", 1, 100));
  params.local_map.voxel_leaf_size = loadDouble(
    node, "local_map.voxel_leaf_size", "Voxel edge (m) used to thin the local map", 0.01, 2.0);

  params.publish_filtered_cloud = declareOrGet<bool>(
    node, "publish_filtered_cloud",
    described("Publish the preprocessed scan on ~/filtered_points"));

  params.markers.trajectory_width = loadDouble(
    node, "visualization.trajectory_width", "Width (m) of the trajectory line", 0.01, 2.0);
  params.markers.trajectory_color = loadColor(
    node, "visualization.trajectory_color", "Colour of the trajectory line");
  params.markers.vehicle_length = loadDouble(
    node, "visualization.vehicle_length", "Length (m) of the vehicle arrow", 0.1, 10.0);
  params.markers.vehicle_width = loadDouble(
    node, "visualization.vehicle_width", "Width (m) of the vehicle arrow", 0.01, 5.0);
  params.markers.vehicle_color = loadColor(
    node, "visualization.vehicle_color", "Colour of the vehicle arrow");

  return params;
}

}  // namespace lidar_odometry
