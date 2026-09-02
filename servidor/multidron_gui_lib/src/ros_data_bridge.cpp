#include "multidron_gui_lib/ros_data_bridge.hpp"

#include "multidron_gui_lib/fiducial_config_loader.hpp"

#include "sensor_msgs/msg/point_field.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <QMatrix4x4>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace multidron_gui_lib
{
namespace
{

struct FieldInfo
{
  std::uint32_t offset = 0;
  std::uint8_t datatype = 0;
};

std::optional<FieldInfo> FindField(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const std::string & name)
{
  for (const auto & field : cloud.fields) {
    if (field.name == name) {
      return FieldInfo{field.offset, field.datatype};
    }
  }
  return std::nullopt;
}

template<typename T>
bool ReadScalar(
  const sensor_msgs::msg::PointCloud2 & cloud,
  std::size_t base,
  const std::optional<FieldInfo> & field,
  std::uint8_t expected_datatype,
  T * value)
{
  if (!field.has_value() || value == nullptr || field->datatype != expected_datatype) {
    return false;
  }
  const std::size_t offset = base + field->offset;
  if (offset + sizeof(T) > cloud.data.size()) {
    return false;
  }
  std::memcpy(value, cloud.data.data() + offset, sizeof(T));
  return true;
}

QQuaternion ToQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  QQuaternion result(
    static_cast<float>(q.w), static_cast<float>(q.x),
    static_cast<float>(q.y), static_cast<float>(q.z));
  if (result.lengthSquared() < 1e-12F) {
    return QQuaternion();
  }
  return result.normalized();
}

QVector3D ToVector(const geometry_msgs::msg::Point & point)
{
  return QVector3D(
    static_cast<float>(point.x),
    static_cast<float>(point.y),
    static_cast<float>(point.z));
}

QMatrix4x4 PoseMatrix(const geometry_msgs::msg::Pose & pose)
{
  QMatrix4x4 matrix;
  matrix.translate(ToVector(pose.position));
  matrix.rotate(ToQuaternion(pose.orientation));
  return matrix;
}

double QuaternionYaw(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

QColor MarkerColor(const std_msgs::msg::ColorRGBA & color)
{
  QColor result;
  result.setRgbF(
    std::clamp(static_cast<double>(color.r), 0.0, 1.0),
    std::clamp(static_cast<double>(color.g), 0.0, 1.0),
    std::clamp(static_cast<double>(color.b), 0.0, 1.0),
    std::clamp(static_cast<double>(color.a), 0.0, 1.0));
  return result;
}

bool IsTrackingUsable(const orbslam3_msgs::msg::NavigationState & state)
{
  using Message = orbslam3_msgs::msg::NavigationState;
  return state.tracking_state == Message::TRACKING_OK ||
         state.tracking_state == Message::TRACKING_OK_KLT;
}

bool IsNonGroundTruthPose(const orbslam3_msgs::msg::NavigationState & state)
{
  using Message = orbslam3_msgs::msg::NavigationState;
  return state.pose_source == Message::POSE_SOURCE_ORB ||
         state.pose_source == Message::POSE_SOURCE_GLOBAL;
}

}  // namespace

RosDataBridge::RosDataBridge(
  std::shared_ptr<GuiDataModel> model,
  const rclcpp::NodeOptions & options)
: Node("multidron_gui_bridge", options), model_(std::move(model))
{
  if (!model_) {
    throw std::invalid_argument("GuiDataModel no puede ser null");
  }

  sparse_topic_ = declare_parameter<std::string>("sparse_topic", "/global_sparse_cloud");
  keyframes_topic_ = declare_parameter<std::string>("keyframes_topic", "/global_keyframes");
  const std::int64_t drone_count = declare_parameter<std::int64_t>("drone_count", 2);
  const std::string namespace_base =
    declare_parameter<std::string>("drone_namespace_base", "dron");
  const std::string navigation_suffix =
    declare_parameter<std::string>("navigation_topic_suffix", "orbslam/navigation_state");
  const std::string fiducial_config_path =
    declare_parameter<std::string>("fiducial_config_path", "");
  const double stale_timeout_sec = declare_parameter<double>("drone_stale_timeout_sec", 1.0);

  if (drone_count <= 0) {
    throw std::invalid_argument("drone_count debe ser positivo");
  }
  if (!std::isfinite(stale_timeout_sec) || stale_timeout_sec <= 0.0) {
    throw std::invalid_argument("drone_stale_timeout_sec debe ser positivo y finito");
  }
  stale_timeout_ns_ = static_cast<std::int64_t>(stale_timeout_sec * 1e9);

  rclcpp::QoS map_qos(rclcpp::KeepLast(1));
  map_qos.reliable().transient_local();
  sparse_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    sparse_topic_, map_qos,
    std::bind(&RosDataBridge::OnSparseCloud, this, std::placeholders::_1));
  keyframe_subscription_ = create_subscription<visualization_msgs::msg::MarkerArray>(
    keyframes_topic_, map_qos,
    std::bind(&RosDataBridge::OnKeyframes, this, std::placeholders::_1));
  mission_subscription_ = create_subscription<mission_msgs::msg::MissionGeometry>(
    "/mission/geometry", map_qos,
    std::bind(&RosDataBridge::OnMissionGeometry, this, std::placeholders::_1));

  for (std::int64_t drone_id = 1; drone_id <= drone_count; ++drone_id) {
    std::string suffix = navigation_suffix;
    while (!suffix.empty() && suffix.front() == '/') {
      suffix.erase(suffix.begin());
    }
    const std::string topic = "/" + namespace_base + "_" +
      std::to_string(drone_id) + "/" + suffix;
    navigation_subscriptions_.push_back(
      create_subscription<orbslam3_msgs::msg::NavigationState>(
        topic, rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
        [this, drone_id](orbslam3_msgs::msg::NavigationState::ConstSharedPtr state) {
          OnNavigationState(static_cast<std::uint32_t>(drone_id), std::move(state));
        }));
    RCLCPP_INFO(
      get_logger(), "[GUI-ROS-SUB] kind=navigation drone_id=%ld topic=%s",
      drone_id, topic.c_str());
  }

  if (!fiducial_config_path.empty()) {
    LoadFiducialsFromConfig(fiducial_config_path);
  } else {
    RCLCPP_WARN(
      get_logger(),
      "[GUI-FIDUCIALS] fiducial_config_path vacio; FiducialLayer queda vacia");
  }

  stale_timer_ = create_wall_timer(
    std::chrono::milliseconds(200), std::bind(&RosDataBridge::CheckStaleDrones, this));

  RCLCPP_INFO(
    get_logger(),
    "[GUI-ROS-READY] sparse=%s keyframes=%s mission=/mission/geometry drones=%ld",
    sparse_topic_.c_str(), keyframes_topic_.c_str(), drone_count);
}

void RosDataBridge::OnMissionGeometry(
  mission_msgs::msg::MissionGeometry::ConstSharedPtr geometry)
{
  if (!geometry) {
    return;
  }
  MissionRegionVector regions;
  regions.reserve(geometry->regions.size());
  for (const auto & source : geometry->regions) {
    MissionRegionVisual region;
    region.region_id = source.region_id;
    region.level_index = source.level_index;
    switch (source.side) {
      case mission_msgs::msg::BaseSubRoi::SIDE_AB: region.side = "AB"; break;
      case mission_msgs::msg::BaseSubRoi::SIDE_BC: region.side = "BC"; break;
      case mission_msgs::msg::BaseSubRoi::SIDE_CD: region.side = "CD"; break;
      case mission_msgs::msg::BaseSubRoi::SIDE_DA: region.side = "DA"; break;
      default: region.side = "?"; break;
    }
    region.min_world = ToVector(source.bounds.min);
    region.max_world = ToVector(source.bounds.max);
    regions.push_back(std::move(region));
  }
  const auto count = regions.size();
  model_->SetMissionRegions(std::move(regions));
  RCLCPP_INFO(get_logger(),
    "[GUI-MISSION-GEOMETRY] mission=%s revision=%lu regions=%zu assigned=false",
    geometry->mission_id.c_str(), geometry->config_revision, count);
}

void RosDataBridge::OnSparseCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  if (!cloud) {
    return;
  }
  if (cloud->is_bigendian) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "[GUI-SPARSE-REJECT] PointCloud2 big-endian no soportado");
    return;
  }

  const auto x_field = FindField(*cloud, "x");
  const auto y_field = FindField(*cloud, "y");
  const auto z_field = FindField(*cloud, "z");
  const auto score_field = FindField(*cloud, "score");
  const auto drone_field = FindField(*cloud, "drone_id");
  const auto epoch_low_field = FindField(*cloud, "map_epoch_low");
  const auto epoch_high_field = FindField(*cloud, "map_epoch_high");
  const auto mp_id_low_field = FindField(*cloud, "local_mp_id_low");
  const auto mp_id_high_field = FindField(*cloud, "local_mp_id_high");

  if (!x_field || !y_field || !z_field || cloud->point_step == 0U) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "[GUI-SPARSE-REJECT] faltan x/y/z o point_step=0");
    return;
  }
  if (!score_field) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "[GUI-SPARSE-SCORE-MISSING] no existe field score; se usa 1.0 solo para visualizacion");
  }

  SparsePointVector parsed;
  parsed.reserve(static_cast<std::size_t>(cloud->width) * cloud->height);
  std::uint64_t source_index = 0;

  for (std::uint32_t row = 0; row < cloud->height; ++row) {
    for (std::uint32_t column = 0; column < cloud->width; ++column, ++source_index) {
      const std::size_t base = static_cast<std::size_t>(row) * cloud->row_step +
        static_cast<std::size_t>(column) * cloud->point_step;
      if (base + cloud->point_step > cloud->data.size()) {
        continue;
      }

      float x = 0.0F;
      float y = 0.0F;
      float z = 0.0F;
      if (!ReadScalar(
          *cloud, base, x_field, sensor_msgs::msg::PointField::FLOAT32, &x) ||
        !ReadScalar(
          *cloud, base, y_field, sensor_msgs::msg::PointField::FLOAT32, &y) ||
        !ReadScalar(
          *cloud, base, z_field, sensor_msgs::msg::PointField::FLOAT32, &z))
      {
        continue;
      }
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      SparsePoint point;
      point.position = QVector3D(x, y, z);
      point.source_index = source_index;
      float score = 1.0F;
      if (score_field) {
        (void)ReadScalar(
          *cloud, base, score_field, sensor_msgs::msg::PointField::FLOAT32, &score);
      }
      point.score = std::isfinite(score) ? score : 0.0F;

      std::uint32_t drone_id = 0;
      (void)ReadScalar(
        *cloud, base, drone_field, sensor_msgs::msg::PointField::UINT32, &drone_id);
      point.drone_id = drone_id;

      std::uint32_t epoch_low = 0;
      std::uint32_t epoch_high = 0;
      (void)ReadScalar(
        *cloud, base, epoch_low_field, sensor_msgs::msg::PointField::UINT32, &epoch_low);
      (void)ReadScalar(
        *cloud, base, epoch_high_field, sensor_msgs::msg::PointField::UINT32, &epoch_high);
      point.map_epoch = static_cast<std::uint64_t>(epoch_low) |
        (static_cast<std::uint64_t>(epoch_high) << 32U);
      std::uint32_t mp_id_low = 0;
      std::uint32_t mp_id_high = 0;
      if (ReadScalar(
          *cloud, base, mp_id_low_field, sensor_msgs::msg::PointField::UINT32,
          &mp_id_low) &&
        ReadScalar(
          *cloud, base, mp_id_high_field, sensor_msgs::msg::PointField::UINT32,
          &mp_id_high))
      {
        point.source_index = static_cast<std::uint64_t>(mp_id_low) |
          (static_cast<std::uint64_t>(mp_id_high) << 32U);
      }
      parsed.push_back(std::move(point));
    }
  }

  float score_min = 1.0F;
  float score_max = 0.0F;
  for (const auto & point : parsed) {
    score_min = std::min(score_min, point.score);
    score_max = std::max(score_max, point.score);
  }
  const std::size_t count = parsed.size();
  model_->SetSparsePoints(std::move(parsed));
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "[GUI-SPARSE-UPDATE] points=%zu score_min=%.4f score_max=%.4f "
    "color_source=score identity=drone_epoch_local_mp",
    count, score_min, score_max);
}

void RosDataBridge::OnKeyframes(
  visualization_msgs::msg::MarkerArray::ConstSharedPtr markers)
{
  if (!markers) {
    return;
  }

  KeyframeVector snapshot;
  {
    std::lock_guard<std::mutex> lock(keyframe_mutex_);
    for (const auto & marker : markers->markers) {
      if (marker.action == visualization_msgs::msg::Marker::DELETEALL) {
        keyframe_cache_.clear();
        continue;
      }
      if (marker.action == visualization_msgs::msg::Marker::DELETE) {
        keyframe_cache_.erase(marker.id);
        continue;
      }
      if (marker.action != visualization_msgs::msg::Marker::ADD) {
        continue;
      }

      KeyframeVisual visual;
      visual.marker_id = marker.id;
      visual.marker_namespace = marker.ns;
      visual.position = ToVector(marker.pose.position);
      visual.orientation = ToQuaternion(marker.pose.orientation);
      visual.color = MarkerColor(marker.color);

      const QMatrix4x4 pose = PoseMatrix(marker.pose);
      visual.line_points_world.reserve(marker.points.size());
      for (const auto & point : marker.points) {
        visual.line_points_world.push_back(pose * ToVector(point));
      }
      keyframe_cache_[marker.id] = std::move(visual);
    }

    snapshot.reserve(keyframe_cache_.size());
    for (const auto & item : keyframe_cache_) {
      snapshot.push_back(item.second);
    }
  }
  const std::size_t count = snapshot.size();
  model_->SetKeyframes(std::move(snapshot));
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "[GUI-KF-UPDATE] markers=%zu frame=world", count);
}

void RosDataBridge::OnNavigationState(
  std::uint32_t configured_drone_id,
  orbslam3_msgs::msg::NavigationState::ConstSharedPtr state)
{
  if (!state) {
    return;
  }

  const std::uint32_t drone_id = state->drone_id != 0U ? state->drone_id : configured_drone_id;
  DroneState output;
  {
    std::lock_guard<std::mutex> lock(drone_mutex_);
    const auto previous = drone_cache_.find(drone_id);
    if (previous != drone_cache_.end()) {
      output = previous->second;
    }

    output.drone_id = drone_id;
    output.sample_sequence = state->sample_sequence;
    output.map_epoch = state->map_epoch;
    output.pose_revision = state->pose_revision;
    output.reference_keyframe_valid = state->reference_keyframe_valid;
    output.reference_keyframe_id = state->reference_keyframe_id;
    output.tracking_state = state->tracking_state;
    output.pose_source = state->pose_source;
    output.global_status = state->global_status;
    output.received_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();

    // Regla F7: la escena está en world y nunca usa GT como fuente funcional.
    // Si el mensaje vigente procede del fallback GT temporal de Fase 5, se conserva
    // la última pose visual no-GT y se marca como perdida/no disponible.
    const bool usable = state->global_valid && IsTrackingUsable(*state) &&
      IsNonGroundTruthPose(*state);
    if (usable) {
      output.position = ToVector(state->w_t_body.position);
      output.orientation = ToQuaternion(state->w_t_body.orientation);
      output.yaw_rad = QuaternionYaw(state->w_t_body.orientation);
      output.has_world_pose = true;
      output.lost_or_unavailable = false;
    } else {
      output.lost_or_unavailable = true;
    }

    const auto current = drone_cache_.find(drone_id);
    const bool newer = current == drone_cache_.end() ||
      output.map_epoch > current->second.map_epoch ||
      (output.map_epoch == current->second.map_epoch &&
      output.sample_sequence > current->second.sample_sequence) ||
      (output.map_epoch == current->second.map_epoch &&
      output.sample_sequence == current->second.sample_sequence &&
      output.pose_revision >= current->second.pose_revision);
    if (!newer) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[GUI-DATA-STALE] kind=navigation drone_id=%u incoming_sequence=%lu current_sequence=%lu",
        drone_id, state->sample_sequence, current->second.sample_sequence);
      return;
    }
    drone_cache_[drone_id] = output;
  }
  (void)model_->UpdateDrone(output);
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "[GUI-DRONE-POSE] drone_id=%u epoch=%lu pose_revision=%lu available=%s stale=%s",
    output.drone_id, output.map_epoch, output.pose_revision,
    output.has_world_pose ? "true" : "false",
    output.lost_or_unavailable ? "true" : "false");
}

void RosDataBridge::CheckStaleDrones()
{
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  const std::size_t changed = model_->MarkStaleDrones(now_ns, stale_timeout_ns_);
  if (changed > 0U) {
    RCLCPP_WARN(
      get_logger(), "[GUI-DATA-STALE] kind=navigation changed=%zu timeout_ms=%ld",
      changed, static_cast<long>(stale_timeout_ns_ / 1000000LL));
  }
}

void RosDataBridge::LoadFiducialsFromConfig(const std::string & path)
{
  FiducialVector objects;
  std::string error;
  if (!FiducialConfigLoader::Load(path, &objects, &error)) {
    RCLCPP_ERROR(
      get_logger(), "[GUI-FIDUCIALS-ERROR] path=%s error=%s",
      path.c_str(), error.c_str());
    return;
  }
  const std::size_t count = objects.size();
  model_->SetFiducials(std::move(objects));
  RCLCPP_INFO(
    get_logger(), "[GUI-FIDUCIAL-UPDATE] path=%s objects=%zu frame=world",
    path.c_str(), count);
}

}  // namespace multidron_gui_lib
