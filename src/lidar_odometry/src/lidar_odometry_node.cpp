#include "lidar_odometry/lidar_odometry_node.hpp"

#include <chrono>
#include <cstdint>

#include "pcl_conversions/pcl_conversions.h"
#include "tf2_eigen/tf2_eigen.hpp"

#include "lidar_odometry/visualization.hpp"

namespace lidar_odometry
{
namespace
{

using Clock = std::chrono::steady_clock;

double millisecondsBetween(Clock::time_point from, Clock::time_point to)
{
  return std::chrono::duration<double, std::milli>(to - from).count();
}

/// Every output carries the time of the scan it came from, not the time it was computed.
std_msgs::msg::Header stampedLike(
  const std_msgs::msg::Header & scan_header,
  const std::string & frame)
{
  std_msgs::msg::Header header;
  header.stamp = scan_header.stamp;
  header.frame_id = frame;
  return header;
}

void publishCloud(
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2> & publisher, const Cloud & cloud,
  const std_msgs::msg::Header & header)
{
  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  message.header = header;
  publisher.publish(message);
}

}  // namespace

LidarOdometryNode::LidarOdometryNode(const rclcpp::NodeOptions & options)
: Node("lidar_odometry", options),
  startup_params_(loadStartupParameters(*this)),
  params_(loadTunableParameters(*this))
{
  odometry_ = std::make_unique<ScanToMapOdometry>(params_.registration, params_.local_map);
  if (startup_params_.track_map_enabled) {
    track_map_ = std::make_unique<TrackMap>(startup_params_.track_map_voxel_leaf_size);
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  createPublishers();

  // Scans are large: with a best-effort subscription most of them would be silently lost.
  const auto scan_qos =
    rclcpp::QoS(static_cast<size_t>(startup_params_.scan_queue_depth)).reliable();
  scan_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    startup_params_.input_topic, scan_qos,
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & scan) {onScan(scan);});

  // Humble calls this before storing the new values, so changes are only flagged here and
  // read at the next scan. The node is single-threaded: this never runs during onScan.
  parameter_callback_handle_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> &) {
      tunable_parameters_changed_ = true;
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      return result;
    });
}

void LidarOdometryNode::createPublishers()
{
  const auto output_qos = rclcpp::QoS(static_cast<size_t>(startup_params_.output_queue_depth));
  odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odometry", output_qos);
  marker_publisher_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/markers", output_qos);
  filtered_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/filtered_points",
    output_qos);
  processing_time_publisher_ =
    create_publisher<lidar_odometry_msgs::msg::ProcessingTime>("~/processing_time", output_qos);

  // Transient local: RViz still receives the latest map if it subscribes after it was published.
  const auto track_map_qos =
    rclcpp::QoS(static_cast<size_t>(startup_params_.track_map_queue_depth)).transient_local();
  track_map_publisher_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("~/track_map", track_map_qos);
}

void LidarOdometryNode::onScan(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & scan)
{
  const auto start = Clock::now();
  if (tunable_parameters_changed_) {
    applyTunableParameters();
  }
  if (!ensureSensorExtrinsics(scan->header.frame_id)) {
    return;
  }

  Cloud raw_scan;
  pcl::fromROSMsg(*scan, raw_scan);
  const Cloud::Ptr filtered_scan = downsample(
    cropToVehicleFrame(raw_scan, *base_from_sensor_, params_.crop), params_.voxel_leaf_size);
  const auto preprocessing_end = Clock::now();

  const RegistrationResult registration = odometry_->update(filtered_scan);
  const auto registration_end = Clock::now();
  if (!registration.converged) {
    RCLCPP_WARN(get_logger(), "ICP did not converge, keeping the predicted pose");
  }

  publishResults(scan->header, *filtered_scan, registration.keyframe_added);

  lidar_odometry_msgs::msg::ProcessingTime timing;
  timing.header = stampedLike(scan->header, startup_params_.base_frame);
  timing.preprocessing_ms = millisecondsBetween(start, preprocessing_end);
  timing.registration_ms = millisecondsBetween(preprocessing_end, registration_end);
  timing.input_points = static_cast<uint32_t>(raw_scan.size());
  timing.filtered_points = static_cast<uint32_t>(filtered_scan->size());
  timing.total_ms = millisecondsBetween(start, Clock::now());
  processing_time_publisher_->publish(timing);
}

bool LidarOdometryNode::ensureSensorExtrinsics(const std::string & sensor_frame)
{
  if (base_from_sensor_) {
    return true;
  }

  const std::string & base_frame = startup_params_.base_frame;
  try {
    const auto transform =
      tf_buffer_->lookupTransform(base_frame, sensor_frame, tf2::TimePointZero);
    base_from_sensor_ = tf2::transformToEigen(transform);
  } catch (const tf2::TransformException & error) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "Waiting for transform %s -> %s: %s",
      base_frame.c_str(), sensor_frame.c_str(), error.what());
    return false;
  }

  const Eigen::Vector3d & offset = base_from_sensor_->translation();
  RCLCPP_INFO(
    get_logger(), "Sensor mounted at (%.3f, %.3f, %.3f) m in '%s'",
    offset.x(), offset.y(), offset.z(), base_frame.c_str());
  return true;
}

void LidarOdometryNode::applyTunableParameters()
{
  params_ = loadTunableParameters(*this);
  odometry_->setParameters(params_.registration, params_.local_map);
  tunable_parameters_changed_ = false;
  RCLCPP_INFO(get_logger(), "Tunable parameters updated");
}

void LidarOdometryNode::publishResults(
  const std_msgs::msg::Header & scan_header, const Cloud & filtered_scan, bool keyframe_added)
{
  const Eigen::Isometry3d & odom_from_base = odometry_->pose();
  const std_msgs::msg::Header odom_header = stampedLike(scan_header, startup_params_.odom_frame);

  publishPose(odom_header, odom_from_base);

  trajectory_.push_back(tf2::toMsg(Eigen::Vector3d(odom_from_base.translation())));
  marker_publisher_->publish(
    makeMarkers(odom_header, trajectory_, odom_from_base, params_.markers));

  if (track_map_ && keyframe_added) {
    track_map_->add(filtered_scan, odom_from_base);
    publishCloud(*track_map_publisher_, *track_map_->cloud(), odom_header);
  }
  if (params_.publish_filtered_cloud) {
    publishCloud(
      *filtered_cloud_publisher_, filtered_scan,
      stampedLike(scan_header, startup_params_.base_frame));
  }
}

void LidarOdometryNode::publishPose(
  const std_msgs::msg::Header & odom_header, const Eigen::Isometry3d & odom_from_base)
{
  nav_msgs::msg::Odometry odometry;
  odometry.header = odom_header;
  odometry.child_frame_id = startup_params_.base_frame;
  odometry.pose.pose = tf2::toMsg(odom_from_base);
  odometry_publisher_->publish(odometry);

  geometry_msgs::msg::TransformStamped transform = tf2::eigenToTransform(odom_from_base);
  transform.header = odom_header;
  transform.child_frame_id = startup_params_.base_frame;
  tf_broadcaster_->sendTransform(transform);
}

}  // namespace lidar_odometry
