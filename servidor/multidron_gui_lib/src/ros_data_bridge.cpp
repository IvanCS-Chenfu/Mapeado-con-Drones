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
#include <tuple>
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

bool HasCanonicalWorldPose(const orbslam3_msgs::msg::NavigationState & state)
{
  using Message = orbslam3_msgs::msg::NavigationState;
  if (state.pose_source == Message::POSE_SOURCE_GT_FORCED) {
    return state.local_valid && state.local_continuity_valid && state.velocity_valid;
  }
  return state.global_valid && (state.pose_source == Message::POSE_SOURCE_ORB ||
         state.pose_source == Message::POSE_SOURCE_GLOBAL);
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
  task_subscription_ = create_subscription<mission_msgs::msg::TaskStateArray>(
    "/mission/task_states", map_qos,
    std::bind(&RosDataBridge::OnTaskStates, this, std::placeholders::_1));
  voxel_subscription_ = create_subscription<mission_msgs::msg::VoxelMap>(
    "/mission/voxel_map", map_qos,
    std::bind(&RosDataBridge::OnVoxelMap, this, std::placeholders::_1));
  planned_route_subscription_ = create_subscription<mission_msgs::msg::TrajectoryPlan>(
    "/mission/planned_routes", map_qos,
    std::bind(&RosDataBridge::OnPlannedRoute, this, std::placeholders::_1));

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
  RCLCPP_INFO(
    get_logger(),
    "[GUI-MISSION-GEOMETRY] mission=%s revision=%lu regions=%zu assigned=false",
    geometry->mission_id.c_str(), geometry->config_revision, count);
}

void RosDataBridge::OnTaskStates(mission_msgs::msg::TaskStateArray::ConstSharedPtr tasks)
{
  if (!tasks) {
    return;
  }
  for (const auto & task : tasks->tasks) {
    if (task.assigned_drone_id == 0U) {
      continue;
    }
    TaskVisual visual;
    visual.drone_id = task.assigned_drone_id;
    visual.task_id = task.task_id;
    visual.task_type = task.task_type;
    visual.region_id = task.region_id;
    visual.state_revision = task.state_revision;
    visual.progress = task.progress;
    visual.progress_known = task.progress_known;
    visual.detail = task.detail;
    for (const auto & interval : task.coverage_intervals) {
      visual.coverage_intervals.push_back(
        {QVector3D(
            interval.start_world.x, interval.start_world.y, interval.start_world.z),
          QVector3D(
            interval.end_world.x, interval.end_world.y, interval.end_world.z)});
    }
    switch (task.state) {
      case mission_msgs::msg::TaskState::PENDING: visual.state = "PENDING"; break;
      case mission_msgs::msg::TaskState::ASSIGNED: visual.state = "ASSIGNED"; break;
      case mission_msgs::msg::TaskState::RUNNING: visual.state = "RUNNING"; break;
      case mission_msgs::msg::TaskState::PAUSED: visual.state = "PAUSED"; break;
      case mission_msgs::msg::TaskState::COMPLETED: visual.state = "COMPLETED"; break;
      case mission_msgs::msg::TaskState::FAILED: visual.state = "FAILED"; break;
      case mission_msgs::msg::TaskState::BLOCKED: visual.state = "BLOCKED"; break;
      case mission_msgs::msg::TaskState::WAITING: visual.state = "WAITING"; break;
      case mission_msgs::msg::TaskState::BLOCKED_BRANCH:
        visual.state = "BLOCKED_BRANCH";
        break;
      case mission_msgs::msg::TaskState::SUPERSEDED: visual.state = "SUPERSEDED"; break;
      case mission_msgs::msg::TaskState::TO_FINISH: visual.state = "TO_FINISH"; break;
      default: visual.state = "OTHER"; break;
    }
    model_->UpdateTask(visual);
  }
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "[GUI-TASKS] mission=%s tasks=%zu", tasks->mission_id.c_str(), tasks->tasks.size());
}

void RosDataBridge::OnVoxelMap(mission_msgs::msg::VoxelMap::ConstSharedPtr voxels)
{
  if (!voxels || voxels->voxel_size <= 0.0) {
    return;
  }
  VoxelVector visual;
  visual.reserve(voxels->voxels.size() + voxels->reserved_voxels.size());
  const float size = static_cast<float>(voxels->voxel_size);
  std::map<std::tuple<std::int64_t, std::int64_t, std::int64_t>, std::size_t> indices;
  for (const auto & voxel : voxels->voxels) {
    VoxelVisual output;
    output.ix = voxel.ix;
    output.iy = voxel.iy;
    output.iz = voxel.iz;
    output.size_m = size;
    output.center_world = QVector3D(
      (static_cast<float>(voxel.ix) + 0.5F) * size,
      (static_cast<float>(voxel.iy) + 0.5F) * size,
      (static_cast<float>(voxel.iz) + 0.5F) * size);
    output.score = voxel.score;
    output.state = voxel.state == mission_msgs::msg::VoxelCell::OCCUPIED ?
      VoxelState::Occupied : (voxel.state == mission_msgs::msg::VoxelCell::FREE ?
      VoxelState::Free : VoxelState::Unknown);
    indices.emplace(std::make_tuple(voxel.ix, voxel.iy, voxel.iz), visual.size());
    visual.push_back(std::move(output));
  }
  for (const auto & voxel : voxels->reserved_voxels) {
    const auto key = std::make_tuple(voxel.ix, voxel.iy, voxel.iz);
    const auto existing = indices.find(key);
    if (existing != indices.end() && visual[existing->second].state == VoxelState::Occupied) {
      continue;
    }
    VoxelVisual output;
    output.ix = voxel.ix;
    output.iy = voxel.iy;
    output.iz = voxel.iz;
    output.size_m = size;
    output.center_world = QVector3D(
      (static_cast<float>(voxel.ix) + 0.5F) * size,
      (static_cast<float>(voxel.iy) + 0.5F) * size,
      (static_cast<float>(voxel.iz) + 0.5F) * size);
    output.state = VoxelState::Reserved;
    if (existing == indices.end()) {
      indices.emplace(key, visual.size());
      visual.push_back(std::move(output));
    } else {
      visual[existing->second] = std::move(output);
    }
  }
  const auto count = visual.size();
  model_->SetVoxels(std::move(visual));
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "[GUI-VOXELS] revision=%lu cells=%zu reserved=%zu", voxels->map_revision, count,
    voxels->reserved_voxels.size());
}

void RosDataBridge::OnPlannedRoute(mission_msgs::msg::TrajectoryPlan::ConstSharedPtr route)
{
  if (!route || route->drone_id == 0U || route->trajectory_id.empty()) {
    return;
  }
  if (route->execution_state == mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_COMPLETED ||
    route->execution_state == mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_CANCELED ||
    route->execution_state == mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_REJECTED)
  {
    const bool cleared = model_->ClearTrajectory(route->drone_id, route->trajectory_id);
    RCLCPP_INFO(
      get_logger(), "[GUI-TRAJECTORY-CLEAR] drone_id=%u trajectory_id=%s state=%u cleared=%s detail=%s",
      route->drone_id, route->trajectory_id.c_str(), route->execution_state,
      cleared ? "true" : "false",
      route->execution_detail.c_str());
    return;
  }
  if (route->execution_state != mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_ACTIVE) {
    RCLCPP_INFO(
      get_logger(), "[GUI-TRAJECTORY-IGNORE] drone_id=%u trajectory_id=%s state=%u detail=%s",
      route->drone_id, route->trajectory_id.c_str(), route->execution_state,
      route->execution_detail.c_str());
    return;
  }
  if (route->waypoints.size() < 2U) {
    return;
  }
  TrajectoryVisual visual;
  visual.drone_id = route->drone_id;
  visual.task_id = route->task_id;
  visual.trajectory_id = route->trajectory_id;
  visual.plan_revision = route->plan_revision;
  visual.map_revision = route->map_revision;
  visual.alignment_revision = route->alignment_revision;
  visual.samples_world.reserve(route->waypoints.size());
  for (const auto & waypoint : route->waypoints) {
    visual.samples_world.push_back(ToVector(waypoint.position_world));
  }
  const auto count = visual.samples_world.size();
  model_->ReplaceTrajectory(visual);
  RCLCPP_INFO(
    get_logger(),
    "[GUI-TRAJECTORY-UPDATE] drone_id=%u task_id=%s trajectory_id=%s state=%u count=%zu map_revision=%lu",
    visual.drone_id, visual.task_id.c_str(), visual.trajectory_id.c_str(),
    route->execution_state, count, visual.map_revision);
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

    // La GUI solo consume la pose canónica del mux, también en la rama GT simulada.
    const bool usable = HasCanonicalWorldPose(*state) &&
      (state->pose_source == orbslam3_msgs::msg::NavigationState::POSE_SOURCE_GT_FORCED ||
      IsTrackingUsable(*state));
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
