#include "task_server/drone_registry.hpp"
#include "task_server/evidence_pipeline.hpp"
#include "task_server/workflow_scheduler.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <mission_msgs/msg/base_sub_roi.hpp>
#include <mission_msgs/msg/drone_registry.hpp>
#include <mission_msgs/msg/dense_kf_observation.hpp>
#include <mission_msgs/msg/fiducial_primary_observation.hpp>
#include <mission_msgs/msg/safety_event.hpp>
#include <mission_msgs/msg/global_sparse_map_delta.hpp>
#include <mission_msgs/msg/keyframe_sparse_evidence_delta.hpp>
#include <mission_msgs/msg/mission_geometry.hpp>
#include <mission_msgs/msg/task_state_array.hpp>
#include <mission_msgs/msg/task_report.hpp>
#include <mission_msgs/msg/trajectory_plan.hpp>
#include <mission_msgs/msg/visual_risk_event.hpp>
#include <mission_msgs/msg/voxel_map.hpp>
#include <mission_msgs/action/execute_trajectory.hpp>
#include <mission_msgs/srv/inspect_facade.hpp>
#include <mission_msgs/srv/plan_route.hpp>
#include <mission_msgs/srv/register_drone.hpp>
#include <mission_msgs/srv/report_autonomous_result.hpp>
#include <mission_msgs/srv/submit_autonomous_command.hpp>
#include <orbslam3_msgs/msg/global_key_frame_pose.hpp>
#include <orbslam3_msgs/msg/navigation_state.hpp>
#include <orbslam3_msgs/srv/get_global_key_frame_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <task_lib/facade_coverage.hpp>
#include <task_lib/mission_config.hpp>
#include <task_lib/dstar_lite.hpp>
#include <task_lib/reservation_overlay.hpp>
#include <task_lib/voxel_map.hpp>

#include <chrono>
#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <map>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace
{

geometry_msgs::msg::Point ToPoint(const task_lib::Vec3 & value)
{
  geometry_msgs::msg::Point point;
  point.x = value.x;
  point.y = value.y;
  point.z = value.z;
  return point;
}

mission_msgs::msg::AxisAlignedBox ToBox(const task_lib::AxisAlignedBox & value)
{
  mission_msgs::msg::AxisAlignedBox box;
  box.min = ToPoint(value.min);
  box.max = ToPoint(value.max);
  return box;
}

template<typename T>
bool ReadCloudField(
  const sensor_msgs::msg::PointCloud2 & cloud, std::size_t offset,
  const std::string & field_name, std::uint8_t datatype, T * value)
{
  for (const auto & field : cloud.fields) {
    if (field.name != field_name || field.datatype != datatype) {
      continue;
    }
    const auto position = offset + field.offset;
    if (position + sizeof(T) > cloud.data.size()) {
      return false;
    }
    std::memcpy(value, cloud.data.data() + position, sizeof(T));
    return true;
  }
  return false;
}

bool IsEligiblePose(
  const orbslam3_msgs::msg::NavigationState & state,
  const std::string & source)
{
  using State = orbslam3_msgs::msg::NavigationState;
  const bool globally_anchored = state.global_valid &&
    state.global_status == State::GLOBAL_STATUS_AUTHORITATIVE;
  if (source == "gt") {
    return state.pose_source == State::POSE_SOURCE_GT_FORCED && globally_anchored;
  }
  return state.pose_source == State::POSE_SOURCE_ORB &&
         globally_anchored;
}

double AngularDistance(double first_rad, double second_rad)
{
  return std::abs(std::remainder(first_rad - second_rad, 2.0 * M_PI));
}

void FillSingleCellDepthGaps(std::set<task_lib::VoxelKey> * cells)
{
  if (cells == nullptr || cells->empty()) {
    return;
  }
  const auto evidence = *cells;
  constexpr std::array<task_lib::VoxelKey, 3> kAxes{{
    {1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
  for (const auto & cell : evidence) {
    for (const auto & axis : kAxes) {
      const task_lib::VoxelKey far{
        cell.ix + 2 * axis.ix, cell.iy + 2 * axis.iy, cell.iz + 2 * axis.iz};
      if (evidence.count(far) == 0U) {
        continue;
      }
      cells->insert(
        task_lib::VoxelKey{
          cell.ix + axis.ix, cell.iy + axis.iy, cell.iz + axis.iz});
    }
  }
}

void LimitDepthPoints(
  std::vector<geometry_msgs::msg::Point32> * points, std::size_t maximum_points)
{
  if (points == nullptr || points->size() <= maximum_points || maximum_points == 0U) {
    return;
  }
  std::vector<geometry_msgs::msg::Point32> sampled;
  sampled.reserve(maximum_points);
  for (std::size_t index = 0U; index < maximum_points; ++index) {
    const auto source_index = std::min(
      points->size() - 1U,
      static_cast<std::size_t>(std::floor(
        (static_cast<double>(index) + 0.5) * static_cast<double>(points->size()) /
        static_cast<double>(maximum_points))));
    sampled.push_back((*points)[source_index]);
  }
  *points = std::move(sampled);
}

struct DronePose
{
  task_lib::Vec3 position;
  double yaw_rad = 0.0;
  std::uint64_t map_epoch = 0U;
  std::uint64_t pose_revision = 0U;
};

struct NavigationOrientation
{
  double yaw_rad = 0.0;
  double camera_pitch_rad = 0.0;
  std::uint64_t map_epoch = 0U;
  bool initialized = false;
};

struct VisualCaution
{
  task_lib::Vec3 position;
  std::uint64_t map_epoch = 0U;
  std::uint64_t pose_revision = 0U;
  std::uint64_t frame_id = 0U;
  std::uint8_t poor_sector = 0U;
  double applied_yaw_rad = 0.0;
  double applied_camera_pitch_rad = 0.0;
  std::string source_trajectory_id;
};

enum class FacadeInspectionStage : std::uint8_t
{
  IDLE = 0U,
  INSPECTING = 1U,
  WAITING_DEPTH_INTEGRATION = 2U,
  RECHECKING_SAME_TARGET = 3U,
};

const char * FacadeInspectionStageName(FacadeInspectionStage stage)
{
  switch (stage) {
    case FacadeInspectionStage::INSPECTING:
      return "INSPECTING";
    case FacadeInspectionStage::WAITING_DEPTH_INTEGRATION:
      return "WAITING_DEPTH_INTEGRATION";
    case FacadeInspectionStage::RECHECKING_SAME_TARGET:
      return "RECHECKING_SAME_TARGET";
    case FacadeInspectionStage::IDLE:
    default:
      return "IDLE";
  }
}

struct FacadeTaskRuntime
{
  task_lib::FacadeLine facade;
  std::vector<task_lib::FacadeCoverageInterval> covered;
  task_lib::FacadeCoveragePlan coverage_plan;
  // Cache derivada de las claims; nunca se activa de forma irreversible.
  std::vector<bool> active_coverage_sections;
  std::map<std::string, std::set<std::size_t>> coverage_claims_by_source;
  std::vector<double> rejected_inspection_target_ratios;
  std::optional<double> segment_start_ratio;
  std::optional<double> segment_target_ratio;
  std::optional<task_lib::Vec3> inspected_target;
  double movement_yaw_rad = 0.0;
  double movement_pitch_rad = 0.0;
  mission_msgs::msg::DenseKFObservation facade_observation;
  mission_msgs::msg::DenseKFObservation target_observation;
  std::uint32_t inspection_failures = 0U;
  bool initialized = false;
  bool inspection_pending = false;
  bool awaiting_free_prefix_completion = false;
  bool awaiting_depth = false;
  FacadeInspectionStage inspection_stage = FacadeInspectionStage::IDLE;
};

struct PendingExecutionPlan
{
  std::string task_id;
  std::uint32_t drone_id = 0U;
  task_lib::VoxelKey target;
  std::uint64_t selected_revision = 0U;
  bool require_known_free = false;
};

struct PendingAutonomousTrajectoryMonitor
{
  task_server::WorkflowWorkItem source;
  std::uint8_t command_type = 0U;
  geometry_msgs::msg::Point visual_target;
  mission_msgs::msg::TrajectoryPlan plan;
  bool capture_after_command = false;
  task_server::WorkflowQueue failure_queue = task_server::WorkflowQueue::TRAJECTORY_PLANNING;
};

struct ExecutionRuntime
{
  std::string task_id;
  std::string trajectory_id;
  std::string reservation_id;
  mission_msgs::msg::TrajectoryPlan active_plan;
  std::optional<task_lib::VoxelKey> target;
  std::set<task_lib::VoxelKey> corridor;
  std::optional<task_lib::Vec3> stop_hold_position;
  bool in_flight = false;
  bool stop_requested = false;
  bool local_visual_reorientation_active = false;
  bool corridor_affected = false;
  bool require_known_free = false;
  bool autonomous_workflow = false;
  std::string autonomous_command_id;
  std::string autonomous_workflow_id;
};

struct KeyframeIdentity
{
  std::uint32_t drone_id = 0U;
  std::uint64_t map_epoch = 0U;
  std::uint64_t keyframe_id = 0U;

  bool operator<(const KeyframeIdentity & other) const
  {
    if (drone_id != other.drone_id) {
      return drone_id < other.drone_id;
    }
    if (map_epoch != other.map_epoch) {
      return map_epoch < other.map_epoch;
    }
    return keyframe_id < other.keyframe_id;
  }
};

struct KeyframeFreeEvidence
{
  bool pose_available = false;
  bool query_pending = false;
  std::uint64_t pose_revision = 0U;
  geometry_msgs::msg::Pose w_t_keyframe;
  std::map<std::string, task_lib::Vec3> local_voxel_centers;
};

struct DepthKeyframeEvidence
{
  std::map<std::uint64_t, mission_msgs::msg::DenseKFObservation> observations;
  bool pose_available = false;
  bool query_pending = false;
  std::uint64_t pose_revision = 0U;
  geometry_msgs::msg::Pose w_t_keyframe;
  std::map<std::uint64_t, std::uint64_t> applied_pose_revisions;
  std::map<std::uint64_t, std::uint64_t> applied_source_revisions;
  std::map<std::uint64_t, std::string> source_ids;
};

struct PendingDepthIntegration
{
  std::string task_id;
  std::uint32_t drone_id = 0U;
  mission_msgs::msg::DenseKFObservation facade_observation;
  mission_msgs::msg::DenseKFObservation target_observation;
};

struct AutonomousDepthResult
{
  task_server::WorkflowWorkItem work_item;
  std::uint8_t command_type = 0U;
  std::uint8_t result_status = 0U;
  std::vector<mission_msgs::msg::DenseKFObservation> observations;
};

struct AutonomousWorkflowContext
{
  task_lib::FacadeCandidate candidate;
  double segment_start_ratio = 0.0;
  bool has_candidate = false;
  bool move_to_wall = false;
  bool fallback_free_advance = false;
  bool view_advance_capture = false;
};

struct PendingDepthContinuation
{
  task_server::WorkflowWorkItem work_item;
  std::set<std::string> pending_source_ids;
  bool recheck_unknown_target = false;
};

struct CoverageSourceContext
{
  std::string task_id;
  std::uint32_t drone_id = 0U;
  std::size_t candidate_section_index = 0U;
  bool direct_depth_occupied = false;
  bool synthetic_no_facade = false;
};

bool IsFinite(const task_lib::Vec3 & value)
{
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

task_lib::Vec3 Add(const task_lib::Vec3 & lhs, const task_lib::Vec3 & rhs)
{
  return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

task_lib::Vec3 Subtract(const task_lib::Vec3 & lhs, const task_lib::Vec3 & rhs)
{
  return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

task_lib::Vec3 Scale(const task_lib::Vec3 & value, double factor)
{
  return {value.x * factor, value.y * factor, value.z * factor};
}

double Dot(const task_lib::Vec3 & lhs, const task_lib::Vec3 & rhs)
{
  return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

double Norm(const task_lib::Vec3 & value)
{
  return std::sqrt(Dot(value, value));
}

std::optional<task_lib::Vec3> Normalize(const task_lib::Vec3 & value)
{
  const double norm = Norm(value);
  if (!std::isfinite(norm) || norm <= 1e-9) {
    return std::nullopt;
  }
  return Scale(value, 1.0 / norm);
}

double YawFromQuaternion(const geometry_msgs::msg::Quaternion & orientation)
{
  const double norm = std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);
  if (norm <= 1e-9) {
    return 0.0;
  }
  const double x = orientation.x / norm;
  const double y = orientation.y / norm;
  const double z = orientation.z / norm;
  const double w = orientation.w / norm;
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

task_lib::Vec3 Rotate(
  const geometry_msgs::msg::Quaternion & orientation,
  const task_lib::Vec3 & value)
{
  const double norm = std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);
  if (norm <= 1e-9) {
    return value;
  }
  const double x = orientation.x / norm;
  const double y = orientation.y / norm;
  const double z = orientation.z / norm;
  const double w = orientation.w / norm;
  const task_lib::Vec3 cross{y * value.z - z * value.y, z * value.x - x * value.z,
    x * value.y - y * value.x};
  const task_lib::Vec3 twice_cross{2.0 * cross.x, 2.0 * cross.y, 2.0 * cross.z};
  const task_lib::Vec3 second{y * twice_cross.z - z * twice_cross.y,
    z * twice_cross.x - x * twice_cross.z, x * twice_cross.y - y * twice_cross.x};
  return {value.x + w * twice_cross.x + second.x, value.y + w * twice_cross.y + second.y,
    value.z + w * twice_cross.z + second.z};
}

task_lib::Vec3 TransformPoint(
  const geometry_msgs::msg::Pose & transform, const task_lib::Vec3 & local)
{
  const auto rotated = Rotate(transform.orientation, local);
  return {transform.position.x + rotated.x, transform.position.y + rotated.y,
    transform.position.z + rotated.z};
}

task_lib::Vec3 InverseTransformPoint(
  const geometry_msgs::msg::Pose & transform, const task_lib::Vec3 & world)
{
  geometry_msgs::msg::Quaternion inverse;
  inverse.x = -transform.orientation.x;
  inverse.y = -transform.orientation.y;
  inverse.z = -transform.orientation.z;
  inverse.w = transform.orientation.w;
  return Rotate(
    inverse, {world.x - transform.position.x, world.y - transform.position.y,
      world.z - transform.position.z});
}

std::string KeyframeToken(const KeyframeIdentity & identity)
{
  return std::to_string(identity.drone_id) + ":" + std::to_string(identity.map_epoch) + ":" +
         std::to_string(identity.keyframe_id);
}

std::string VoxelToken(const task_lib::Vec3 & position, double voxel_size)
{
  return std::to_string(static_cast<std::int64_t>(std::floor(position.x / voxel_size))) + ":" +
         std::to_string(static_cast<std::int64_t>(std::floor(position.y / voxel_size))) + ":" +
         std::to_string(static_cast<std::int64_t>(std::floor(position.z / voxel_size)));
}

task_lib::Vec3 VoxelCenter(const task_lib::Vec3 & position, double voxel_size)
{
  return {
    (std::floor(position.x / voxel_size) + 0.5) * voxel_size,
    (std::floor(position.y / voxel_size) + 0.5) * voxel_size,
    (std::floor(position.z / voxel_size) + 0.5) * voxel_size};
}

task_lib::Vec3 VoxelCenter(const task_lib::VoxelKey & key, double voxel_size)
{
  return {
    (static_cast<double>(key.ix) + 0.5) * voxel_size,
    (static_cast<double>(key.iy) + 0.5) * voxel_size,
    (static_cast<double>(key.iz) + 0.5) * voxel_size};
}

task_lib::VoxelKey VoxelKeyFromWorld(const geometry_msgs::msg::Point & point, double voxel_size)
{
  return {
    static_cast<std::int64_t>(std::floor(point.x / voxel_size)),
    static_cast<std::int64_t>(std::floor(point.y / voxel_size)),
    static_cast<std::int64_t>(std::floor(point.z / voxel_size))};
}

std::set<task_lib::VoxelKey> VoxelRaySupercover(
  const task_lib::Vec3 & start, const task_lib::Vec3 & end, double voxel_size,
  bool include_endpoint)
{
  std::set<task_lib::VoxelKey> cells;
  if (voxel_size <= 0.0 || !IsFinite(start) || !IsFinite(end)) {
    return cells;
  }
  auto current = VoxelKeyFromWorld(ToPoint(start), voxel_size);
  const auto finish = VoxelKeyFromWorld(ToPoint(end), voxel_size);
  cells.insert(current);
  const double delta[3] = {end.x - start.x, end.y - start.y, end.z - start.z};
  const double origin[3] = {start.x, start.y, start.z};
  std::int64_t coordinate[3] = {current.ix, current.iy, current.iz};
  const std::int64_t target[3] = {finish.ix, finish.iy, finish.iz};
  int step[3] = {0, 0, 0};
  double t_max[3] = {
    std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity()};
  double t_delta[3] = {
    std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity()};
  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(delta[axis]) <= 1e-12) {
      continue;
    }
    step[axis] = delta[axis] > 0.0 ? 1 : -1;
    const double boundary = static_cast<double>(
      coordinate[axis] + (step[axis] > 0 ? 1 : 0)) * voxel_size;
    t_max[axis] = (boundary - origin[axis]) / delta[axis];
    t_delta[axis] = voxel_size / std::abs(delta[axis]);
  }
  const std::size_t guard_limit = static_cast<std::size_t>(
    std::abs(target[0] - coordinate[0]) + std::abs(target[1] - coordinate[1]) +
    std::abs(target[2] - coordinate[2]) + 8) * 8U;
  for (std::size_t guard = 0U; guard < guard_limit && !(current == finish); ++guard) {
    const double next_t = std::min({t_max[0], t_max[1], t_max[2]});
    std::vector<int> crossed;
    for (int axis = 0; axis < 3; ++axis) {
      if (std::abs(t_max[axis] - next_t) <= 1e-10) {
        crossed.push_back(axis);
      }
    }
    const auto before = current;
    for (std::size_t mask = 1U; mask < (1U << crossed.size()); ++mask) {
      auto neighbor = before;
      for (std::size_t bit = 0U; bit < crossed.size(); ++bit) {
        if ((mask & (1U << bit)) == 0U) {
          continue;
        }
        if (crossed[bit] == 0) {
          neighbor.ix += step[0];
        } else if (crossed[bit] == 1) {
          neighbor.iy += step[1];
        } else {
          neighbor.iz += step[2];
        }
      }
      if (include_endpoint || !(neighbor == finish)) {
        cells.insert(neighbor);
      }
    }
    for (const int axis : crossed) {
      coordinate[axis] += step[axis];
      t_max[axis] += t_delta[axis];
    }
    current = {coordinate[0], coordinate[1], coordinate[2]};
  }
  if (!include_endpoint) {
    cells.erase(finish);
  }
  return cells;
}

double SquaredDistance(const task_lib::Vec3 & left, const task_lib::Vec3 & right)
{
  const double dx = left.x - right.x;
  const double dy = left.y - right.y;
  const double dz = left.z - right.z;
  return dx * dx + dy * dy + dz * dz;
}

double SquaredDistanceToBox(const task_lib::Vec3 & point, const task_lib::AxisAlignedBox & box)
{
  const auto axis_distance = [](double value, double minimum, double maximum) {
      if (value < minimum) {
        return minimum - value;
      }
      if (value > maximum) {
        return value - maximum;
      }
      return 0.0;
    };
  const double dx = axis_distance(point.x, box.min.x, box.max.x);
  const double dy = axis_distance(point.y, box.min.y, box.max.y);
  const double dz = axis_distance(point.z, box.min.z, box.max.z);
  return dx * dx + dy * dy + dz * dz;
}

std::set<task_lib::VoxelKey> SegmentCorridor(
  const mission_msgs::msg::TrajectoryPlan & plan, double voxel_size,
  const task_lib::VoxelKey & body_half_extent_cells, double sample_step_voxels)
{
  std::set<task_lib::VoxelKey> corridor;
  if (plan.waypoints.empty()) {
    return corridor;
  }
  const auto add_footprint =
    [&corridor, &body_half_extent_cells](const task_lib::VoxelKey & center) {
      for (std::int64_t ix = -body_half_extent_cells.ix;
        ix <= body_half_extent_cells.ix; ++ix)
      {
        for (std::int64_t iy = -body_half_extent_cells.iy;
          iy <= body_half_extent_cells.iy; ++iy)
        {
          for (std::int64_t iz = -body_half_extent_cells.iz;
            iz <= body_half_extent_cells.iz; ++iz)
          {
            corridor.insert({center.ix + ix, center.iy + iy, center.iz + iz});
          }
        }
      }
    };
  add_footprint(VoxelKeyFromWorld(plan.waypoints.front().position_world, voxel_size));
  for (std::size_t index = 1U; index < plan.waypoints.size(); ++index) {
    const auto & start = plan.waypoints[index - 1U].position_world;
    const auto & end = plan.waypoints[index].position_world;
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double dz = end.z - start.z;
    const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
    const std::size_t samples = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(
        length / (voxel_size * sample_step_voxels))));
    for (std::size_t sample = 1U; sample <= samples; ++sample) {
      const double ratio = static_cast<double>(sample) / static_cast<double>(samples);
      geometry_msgs::msg::Point point;
      point.x = start.x + ratio * dx;
      point.y = start.y + ratio * dy;
      point.z = start.z + ratio * dz;
      add_footprint(VoxelKeyFromWorld(point, voxel_size));
    }
  }
  return corridor;
}

std::set<task_lib::VoxelKey> FootprintAt(
  const task_lib::Vec3 & position, double voxel_size,
  const task_lib::VoxelKey & body_half_extent_cells)
{
  std::set<task_lib::VoxelKey> footprint;
  const auto center = VoxelKeyFromWorld(ToPoint(position), voxel_size);
  for (std::int64_t ix = -body_half_extent_cells.ix; ix <= body_half_extent_cells.ix; ++ix) {
    for (std::int64_t iy = -body_half_extent_cells.iy; iy <= body_half_extent_cells.iy; ++iy) {
      for (std::int64_t iz = -body_half_extent_cells.iz; iz <= body_half_extent_cells.iz; ++iz) {
        footprint.insert({center.ix + ix, center.iy + iy, center.iz + iz});
      }
    }
  }
  return footprint;
}

const char * VoxelStateName(task_lib::VoxelState state)
{
  switch (state) {
    case task_lib::VoxelState::Free:
      return "free";
    case task_lib::VoxelState::Occupied:
      return "occupied";
    case task_lib::VoxelState::Unknown:
    default:
      return "unknown";
  }
}

double DistanceToSegment(
  const task_lib::Vec3 & point, const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & end)
{
  const task_lib::Vec3 first{start.x, start.y, start.z};
  const task_lib::Vec3 second{end.x, end.y, end.z};
  const task_lib::Vec3 delta{second.x - first.x, second.y - first.y, second.z - first.z};
  const double length_squared = SquaredDistance(delta, task_lib::Vec3{});
  if (length_squared <= 1e-12) {
    return std::sqrt(SquaredDistance(point, first));
  }
  const task_lib::Vec3 relative{point.x - first.x, point.y - first.y, point.z - first.z};
  const double projection = std::clamp(
    (relative.x * delta.x + relative.y * delta.y + relative.z * delta.z) / length_squared,
    0.0, 1.0);
  return std::sqrt(
    SquaredDistance(
      point, task_lib::Vec3{first.x + projection * delta.x, first.y + projection * delta.y,
        first.z + projection * delta.z}));
}

std::pair<std::size_t, double> ClosestPlanSegment(
  const mission_msgs::msg::TrajectoryPlan & plan, const task_lib::Vec3 & point)
{
  if (plan.waypoints.size() < 2U) {
    return {0U, std::numeric_limits<double>::infinity()};
  }
  std::pair<std::size_t, double> closest{0U, std::numeric_limits<double>::infinity()};
  for (std::size_t index = 1U; index < plan.waypoints.size(); ++index) {
    const double distance = DistanceToSegment(
      point, plan.waypoints[index - 1U].position_world, plan.waypoints[index].position_world);
    if (distance < closest.second) {
      closest = {index - 1U, distance};
    }
  }
  return closest;
}

std::string VoxelKeyToken(const task_lib::VoxelKey & key)
{
  return std::to_string(key.ix) + ":" + std::to_string(key.iy) + ":" +
         std::to_string(key.iz);
}

std::string NavigationProfileId(const mission_msgs::msg::DroneRegistration & registration)
{
  std::ostringstream stream;
  stream << registration.vehicle_profile << ':' << std::fixed << std::setprecision(3) <<
    registration.dimensions_m.x << ':' << registration.dimensions_m.y << ':' <<
    registration.dimensions_m.z;
  return stream.str();
}

mission_msgs::msg::TrajectoryPlan ToTrajectoryPlan(
  const task_lib::DStarLiteResult & route, const std::string & mission_id,
  const std::string & task_id, std::uint32_t drone_id, std::uint64_t map_epoch,
  std::uint64_t plan_revision)
{
  mission_msgs::msg::TrajectoryPlan plan;
  plan.mission_id = mission_id;
  plan.task_id = task_id;
  plan.drone_id = drone_id;
  plan.plan_id = "dstar_" + std::to_string(drone_id) + "_" + std::to_string(plan_revision);
  plan.trajectory_id = plan.plan_id;
  plan.plan_revision = plan_revision;
  plan.map_epoch = map_epoch;
  plan.map_revision = route.map_revision;
  plan.alignment_revision = 0U;
  plan.generator_id = "dstar_lite_3d";
  plan.generator_version = 1U;
  plan.execution_state = mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_PLANNED;
  plan.execution_detail = "Candidato D* pendiente de ejecucion";
  for (const auto & point : route.waypoints_world) {
    mission_msgs::msg::TrajectoryWaypoint waypoint;
    waypoint.position_world = ToPoint(point);
    plan.waypoints.push_back(std::move(waypoint));
  }
  return plan;
}

mission_msgs::msg::TrajectoryPlan TimeExecutableRoute(
  mission_msgs::msg::TrajectoryPlan plan, double nominal_velocity_mps, double timing_factor,
  double min_segment_duration_sec)
{
  if (plan.waypoints.size() < 2U) {
    return plan;
  }
  double duration = 0.0;
  const double velocity = std::max(0.05, nominal_velocity_mps);
  const double effective_speed = velocity / std::max(1.0, timing_factor);
  plan.waypoints.front().time_from_start.sec = 0;
  plan.waypoints.front().time_from_start.nanosec = 0U;
  for (std::size_t index = 1; index < plan.waypoints.size(); ++index) {
    const auto & previous = plan.waypoints[index - 1U].position_world;
    const auto & candidate = plan.waypoints[index].position_world;
    const double dx = candidate.x - previous.x;
    const double dy = candidate.y - previous.y;
    const double dz = candidate.z - previous.z;
    const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (length <= 1e-9) {
      continue;
    }
    duration += std::max(length / effective_speed, min_segment_duration_sec);
    auto & waypoint = plan.waypoints[index];
    waypoint.time_from_start.sec = static_cast<std::int32_t>(duration);
    waypoint.time_from_start.nanosec = static_cast<std::uint32_t>(
      (duration - std::floor(duration)) * 1e9);
  }
  plan.max_velocity_mps = velocity;
  plan.execution_detail = "Ruta D* completa pendiente de aceptacion";
  return plan;
}

std::map<task_lib::VoxelKey, bool> NavigationTraversability(
  const task_lib::NavigationSnapshot & snapshot)
{
  std::map<task_lib::VoxelKey, bool> traversability;
  for (const auto & cell : snapshot.cells) {
    traversability[cell.key] = cell.traversable;
  }
  return traversability;
}

bool IsSegmentTraversable(
  const geometry_msgs::msg::Point & start, const geometry_msgs::msg::Point & end,
  double voxel_size, const std::map<task_lib::VoxelKey, bool> & traversability)
{
  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double dz = end.z - start.z;
  const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
  const std::size_t samples = std::max<std::size_t>(
    1U, static_cast<std::size_t>(std::ceil(length / (voxel_size * 0.5))));
  for (std::size_t sample = 0U; sample <= samples; ++sample) {
    const double ratio = static_cast<double>(sample) / static_cast<double>(samples);
    geometry_msgs::msg::Point point;
    point.x = start.x + ratio * dx;
    point.y = start.y + ratio * dy;
    point.z = start.z + ratio * dz;
    const auto state = traversability.find(VoxelKeyFromWorld(point, voxel_size));
    if (state != traversability.end() && !state->second) {
      return false;
    }
  }
  return true;
}

mission_msgs::msg::TrajectoryPlan SimplifyExecutableRoute(
  mission_msgs::msg::TrajectoryPlan plan, const geometry_msgs::msg::Point & start,
  double voxel_size, double min_separation_m,
  const task_lib::NavigationSnapshot & navigation_snapshot)
{
  if (plan.waypoints.size() < 2U) {
    return plan;
  }
  plan.waypoints.front().position_world = start;
  const auto traversability = NavigationTraversability(navigation_snapshot);
  std::vector<mission_msgs::msg::TrajectoryWaypoint> simplified;
  simplified.reserve(plan.waypoints.size());
  simplified.push_back(plan.waypoints.front());
  for (std::size_t index = 1U; index < plan.waypoints.size(); ++index) {
    const auto & candidate = plan.waypoints[index];
    const auto & retained = simplified.back().position_world;
    const double dx = candidate.position_world.x - retained.x;
    const double dy = candidate.position_world.y - retained.y;
    const double dz = candidate.position_world.z - retained.z;
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (distance <= 1e-9) {
      continue;
    }
    const bool has_successor = index + 1U < plan.waypoints.size();
    if (distance < min_separation_m && has_successor &&
      IsSegmentTraversable(
        retained, plan.waypoints[index + 1U].position_world, voxel_size, traversability))
    {
      continue;
    }
    simplified.push_back(candidate);
  }
  plan.waypoints = std::move(simplified);
  return plan;
}

}  // namespace

class TaskServerNode final : public rclcpp::Node
{
public:
  using ExecuteTrajectory = mission_msgs::action::ExecuteTrajectory;
  using ReportAutonomousResult = mission_msgs::srv::ReportAutonomousResult;
  using SubmitAutonomousCommand = mission_msgs::srv::SubmitAutonomousCommand;

  TaskServerNode()
  : Node("task_server")
  {
    const auto default_config =
      ament_index_cpp::get_package_share_directory("task_server") + "/config/mission_house.yaml";
    auto mission_config_path = declare_parameter("mission_config", std::string{});
    if (mission_config_path.empty()) {
      mission_config_path = default_config;
    }
    voxel_size_ = declare_parameter("voxel_size", 0.25);
    voxel_occupied_score_threshold_ = static_cast<float>(
      declare_parameter("voxel_occupied_score_threshold", 0.4));
    visual_target_min_score_ = static_cast<float>(
      declare_parameter("visual_target_min_score", 0.2));
    visual_target_max_score_ = static_cast<float>(
      declare_parameter("visual_target_max_score", 0.6));
    sparse_plane_min_map_point_score_ = static_cast<float>(
      declare_parameter("sparse_plane_min_map_point_score", 0.2));
    if (!std::isfinite(voxel_occupied_score_threshold_) ||
      voxel_occupied_score_threshold_ < 0.0F || voxel_occupied_score_threshold_ > 1.0F ||
      !std::isfinite(visual_target_min_score_) || !std::isfinite(visual_target_max_score_) ||
      visual_target_min_score_ < 0.0F || visual_target_max_score_ > 1.0F ||
      visual_target_min_score_ > visual_target_max_score_ ||
      !std::isfinite(sparse_plane_min_map_point_score_) ||
      sparse_plane_min_map_point_score_ < 0.0F || sparse_plane_min_map_point_score_ > 1.0F)
    {
      throw std::invalid_argument("umbrales de score voxel invalidos");
    }
    depth_evidence_enabled_ = declare_parameter("depth_evidence_enabled", false);
    depth_min_confidence_ = declare_parameter("depth_min_confidence", 0.25);
    depth_min_support_points_ = static_cast<std::size_t>(std::max<std::int64_t>(
        1, declare_parameter("depth_min_support_points", 20)));
    depth_max_points_per_observation_ = static_cast<std::size_t>(std::max<std::int64_t>(
        1, declare_parameter("depth_max_points_per_observation", 256)));
    depth_far_free_distance_m_ = declare_parameter("depth_far_free_distance_m", 5.0);
    depth_direct_surface_max_incidence_deg_ = declare_parameter(
      "depth_direct_surface_max_incidence_deg", 30.0);
    sparse_plane_min_inliers_ = static_cast<std::size_t>(std::max<std::int64_t>(
        3, declare_parameter("sparse_plane_min_inliers", 20)));
    sparse_plane_residual_voxels_ = declare_parameter("sparse_plane_residual_voxels", 0.5);
    sparse_plane_max_samples_ = static_cast<std::size_t>(std::max<std::int64_t>(
        1, declare_parameter("sparse_plane_max_samples", 1024)));
    sparse_ray_free_min_score_ = static_cast<float>(
      declare_parameter("sparse_ray_free_min_score", 0.5));
    if (!std::isfinite(depth_min_confidence_) || depth_min_confidence_ < 0.0 ||
      depth_min_confidence_ > 1.0 || !std::isfinite(depth_far_free_distance_m_) ||
      depth_far_free_distance_m_ <= 0.0 ||
      !std::isfinite(depth_direct_surface_max_incidence_deg_) ||
      depth_direct_surface_max_incidence_deg_ <= 0.0 ||
      depth_direct_surface_max_incidence_deg_ > 90.0 ||
      !std::isfinite(sparse_plane_residual_voxels_) ||
      sparse_plane_residual_voxels_ <= 0.0 || !std::isfinite(sparse_ray_free_min_score_) ||
      sparse_ray_free_min_score_ < sparse_plane_min_map_point_score_ ||
      sparse_ray_free_min_score_ > 1.0F)
    {
      throw std::invalid_argument("parametros de evidencia depth invalidos");
    }
    free_body_half_extent_ = task_lib::Vec3{
      declare_parameter("free_body_half_extent_x_m", 0.25),
      declare_parameter("free_body_half_extent_y_m", 0.25),
      declare_parameter("free_body_half_extent_z_m", 0.08)};
    if (!IsFinite(free_body_half_extent_) || free_body_half_extent_.x < 0.0 ||
      free_body_half_extent_.y < 0.0 || free_body_half_extent_.z < 0.0)
    {
      throw std::invalid_argument("semiextensiones FREE del dron invalidas");
    }
    planner_parameters_.unknown_cost_multiplier =
      declare_parameter("planning_unknown_cost_multiplier", 3.0);
    planner_parameters_.search_margin_m =
      declare_parameter("planning_search_margin_m", 2.0);
    planner_parameters_.max_expansions = static_cast<std::size_t>(
      declare_parameter("planning_max_expansions", 250000));
    planner_parameters_.heuristic_weight =
      declare_parameter("planning_heuristic_weight", 1.2);
    planner_parameters_.initial_corridor_half_width_cells =
      declare_parameter("planning_initial_corridor_half_width_cells", 8);
    planner_parameters_.corridor_expand_step_cells =
      declare_parameter("planning_corridor_expand_step_cells", 4);
    planning_coarse_voxel_factor_ =
      declare_parameter("planning_coarse_voxel_factor", 4);
    execute_facade_sweeps_ = declare_parameter("execute_facade_sweeps", false);
    execution_enabled_.store(declare_parameter("execution_enabled_on_start", true));
    execution_nominal_velocity_mps_ = declare_parameter("execution_nominal_velocity_mps", 0.8);
    execution_timing_factor_ = declare_parameter("execution_timing_factor", 2.0);
    trajectory_waypoint_min_separation_m_ =
      declare_parameter("trajectory_waypoint_min_separation_m", 1.0);
    trajectory_min_segment_duration_sec_ =
      declare_parameter("trajectory_min_segment_duration_sec", 8.0);
    extra_obstacle_clearance_voxels_ = declare_parameter("extra_obstacle_clearance_voxels", 1);
    start_escape_radius_voxels_ = declare_parameter("start_escape_radius_voxels", 4);
    unknown_fallback_free_radius_voxels_ =
      declare_parameter("unknown_fallback_free_radius_voxels", 8);
    facade_coverage_offset_voxels_ =
      declare_parameter("facade_coverage_offset_voxels", 2);
    depth_coverage_neighbor_sections_ =
      declare_parameter("depth_coverage_neighbor_sections", 2);
    voxel_worker_coalesce_ms_ = declare_parameter("voxel_worker_coalesce_ms", 100);
    facade_preferences_.preferred_wall_distance_m =
      declare_parameter("facade_preferred_wall_distance_m", 4.0);
    facade_preferences_.preferred_displacement_m =
      declare_parameter("facade_preferred_displacement_m", 2.0);
    facade_preferences_.wall_distance_weight =
      declare_parameter("facade_wall_distance_weight", 1.0);
    facade_preferences_.displacement_weight =
      declare_parameter("facade_displacement_weight", 1.0);
    facade_preferences_.height_weight = declare_parameter("facade_height_weight", 1.0);
    facade_preferences_.completion_ratio =
      declare_parameter("facade_completion_ratio", 0.99);
    facade_candidate_step_m_ = declare_parameter("facade_candidate_step_m", voxel_size_);
    facade_orientation_tolerance_deg_ =
      declare_parameter("facade_orientation_tolerance_deg", 25.0);
    facade_max_inspection_failures_ = static_cast<std::uint32_t>(
      declare_parameter("facade_max_inspection_failures", 3));
    facade_worker_period_ms_ = declare_parameter("facade_worker_period_ms", 250);
    reservation_sweep_sample_step_voxels_ =
      declare_parameter("reservation_sweep_sample_step_voxels", 0.5);
    debug_trajectory_diagnostics_ =
      declare_parameter("debug_trajectory_diagnostics", false);
    debug_facade_dstar_failure_ =
      declare_parameter("debug_facade_dstar_failure", false);
    if (execution_nominal_velocity_mps_ <= 0.0 || execution_timing_factor_ < 1.0 ||
      trajectory_waypoint_min_separation_m_ <= 0.0 ||
      trajectory_min_segment_duration_sec_ <= 0.0 ||
      extra_obstacle_clearance_voxels_ < 0 || start_escape_radius_voxels_ < 0 ||
      unknown_fallback_free_radius_voxels_ <= 0 || facade_coverage_offset_voxels_ < 0 ||
      depth_coverage_neighbor_sections_ < 0 ||
      voxel_worker_coalesce_ms_ <= 0 ||
      facade_preferences_.preferred_wall_distance_m <= 0.0 ||
      facade_preferences_.preferred_displacement_m <= 0.0 ||
      facade_preferences_.wall_distance_weight < 0.0 ||
      facade_preferences_.displacement_weight < 0.0 ||
      facade_preferences_.height_weight < 0.0 ||
      facade_preferences_.completion_ratio <= 0.0 ||
      facade_preferences_.completion_ratio > 1.0 || facade_candidate_step_m_ <= 0.0 ||
      facade_orientation_tolerance_deg_ <= 0.0 ||
      facade_max_inspection_failures_ == 0U || facade_worker_period_ms_ <= 0 ||
      reservation_sweep_sample_step_voxels_ <= 0.0)
    {
      throw std::invalid_argument("parametros de exploracion y ejecucion F6H/F6I invalidos");
    }
    pose_source_ = declare_parameter("phase5_navigation_source", "orb");
    std::transform(
      pose_source_.begin(), pose_source_.end(), pose_source_.begin(),
      [](unsigned char value) {return static_cast<char>(std::tolower(value));});
    if (pose_source_ != "gt" && pose_source_ != "orb") {
      throw std::invalid_argument("phase5_navigation_source debe ser gt u orb");
    }
    flow_enabled_ = declare_parameter("mission_flow_events_enabled", false);
    const auto architecture_enabled =
      declare_parameter("system_architecture_events_enabled", false);

    const auto config = task_lib::LoadMissionConfig(mission_config_path);
    geometry_data_ = task_lib::BuildMissionGeometry(config);
    mission_id_ = config.mission_id;
    mission_frame_ = config.frame_id;
    registry_ = std::make_unique<task_server::DroneRegistry>(
      config.drones, kProtocolVersion, kGeneratorId, kGeneratorVersion,
      mission_msgs::msg::DroneRegistration::CAPABILITY_SPARSE_MAPPING |
      mission_msgs::msg::DroneRegistration::CAPABILITY_STEREO_DEPTH);
    voxel_map_ = std::make_unique<task_lib::ReversibleVoxelMap>(voxel_size_);
    task_server::KeyframeEvidenceParameters evidence_parameters;
    evidence_parameters.plane_min_score = sparse_plane_min_map_point_score_;
    evidence_parameters.direct_ray_min_score = sparse_ray_free_min_score_;
    evidence_parameters.plane_min_inliers = sparse_plane_min_inliers_;
    evidence_parameters.plane_residual_m = sparse_plane_residual_voxels_ * voxel_size_;
    evidence_parameters.voxel_size_m = voxel_size_;
    evidence_parameters.plane_max_samples = sparse_plane_max_samples_;
    keyframe_evidence_worker_ = std::make_unique<task_server::KeyframeEvidenceWorker>(
      evidence_parameters);
    voxel_map_builder_ = std::make_unique<task_server::VoxelMapBuilder>(voxel_size_);
    task_server::DepthIntegrationParameters depth_parameters;
    depth_parameters.voxel_size_m = voxel_size_;
    depth_parameters.far_free_distance_m = depth_far_free_distance_m_;
    depth_parameters.direct_surface_max_incidence_deg = depth_direct_surface_max_incidence_deg_;
    depth_parameters.min_support_points = depth_min_support_points_;
    depth_parameters.min_confidence = depth_min_confidence_;
    depth_integration_worker_ = std::make_unique<task_server::DepthIntegrationWorker>(
      depth_parameters);
    map_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    execution_control_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    workflow_ingress_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    workflow_dispatch_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    voxel_materialization_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    trajectory_monitor_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    global_pose_client_ = create_client<orbslam3_msgs::srv::GetGlobalKeyFramePose>(
      "/global_mapping/get_global_keyframe_pose", rclcpp::ServicesQoS(), map_callback_group_);
    for (const auto & region : geometry_data_.regions) {
      mission_msgs::msg::TaskState task;
      task.task_id = "map_section_" + region.region_id;
      task.task_type = "MAP_SECTION";
      task.region_id = region.region_id;
      task.base_owner_task_id = task.task_id;
      task.branch_owner_task_id = task.task_id;
      task.lineage_id = task.task_id;
      task.resolution_task_id.clear();
      task.state = mission_msgs::msg::TaskState::PENDING;
      task.state_revision = 1U;
      task.progress = 0.0F;
      task.progress_known = false;
      task.detail = "Pendiente: espera dron registrado con pose autorizada";
      tasks_.push_back(std::move(task));
    }

    const auto snapshot_qos = rclcpp::QoS(1).reliable().transient_local();
    geometry_publisher_ = create_publisher<mission_msgs::msg::MissionGeometry>(
      "/mission/geometry", snapshot_qos);
    registry_publisher_ = create_publisher<mission_msgs::msg::DroneRegistry>(
      "/mission/registry", snapshot_qos);
    task_state_publisher_ = create_publisher<mission_msgs::msg::TaskStateArray>(
      "/mission/task_states", snapshot_qos);
    voxel_map_publisher_ = create_publisher<mission_msgs::msg::VoxelMap>(
      "/mission/voxel_map", snapshot_qos);
    planned_route_publisher_ = create_publisher<mission_msgs::msg::TrajectoryPlan>(
      "/mission/planned_routes", snapshot_qos);
    for (const auto drone_id : config.drones) {
      const auto action_name = "/dron_" + std::to_string(drone_id) +
        "/execute_trajectory";
      execute_trajectory_clients_.emplace(
        drone_id,
        rclcpp_action::create_client<ExecuteTrajectory>(this, action_name, map_callback_group_));
      inspect_facade_clients_.emplace(
        drone_id, create_client<mission_msgs::srv::InspectFacade>(
          "/dron_" + std::to_string(drone_id) + "/inspect_facade",
          rclcpp::ServicesQoS(), map_callback_group_));
      autonomous_command_clients_.emplace(
        drone_id, create_client<SubmitAutonomousCommand>(
          "/dron_" + std::to_string(drone_id) + "/autonomous_command",
          rclcpp::ServicesQoS(), workflow_dispatch_callback_group_));
    }
    execution_toggle_service_ = create_service<std_srvs::srv::SetBool>(
      "/mission/set_coverage_execution_enabled",
      std::bind(
        &TaskServerNode::HandleExecutionToggle, this, std::placeholders::_1,
        std::placeholders::_2), rmw_qos_profile_services_default,
      execution_control_callback_group_);
    rclcpp::SubscriptionOptions map_subscription_options;
    map_subscription_options.callback_group = map_callback_group_;
    task_report_subscription_ = create_subscription<mission_msgs::msg::TaskReport>(
      "/mission/task_reports", rclcpp::QoS(50).reliable(),
      std::bind(&TaskServerNode::HandleTaskReport, this, std::placeholders::_1),
      map_subscription_options);
    visual_risk_subscription_ = create_subscription<mission_msgs::msg::VisualRiskEvent>(
      "/mission/visual_risk_events", rclcpp::QoS(20).reliable(),
      std::bind(&TaskServerNode::HandleVisualRiskEvent, this, std::placeholders::_1),
      map_subscription_options);
    safety_event_subscription_ = create_subscription<mission_msgs::msg::SafetyEvent>(
      "/mission/safety_events", rclcpp::QoS(20).reliable(),
      std::bind(&TaskServerNode::HandleSafetyEvent, this, std::placeholders::_1),
      map_subscription_options);
    fiducial_primary_subscription_ =
      create_subscription<mission_msgs::msg::FiducialPrimaryObservation>(
      "/mission/fiducial_primary_observations", rclcpp::QoS(64).reliable(),
      std::bind(
        &TaskServerNode::HandleFiducialPrimaryObservation, this, std::placeholders::_1),
      map_subscription_options);
    sparse_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/global_sparse_cloud", snapshot_qos,
      std::bind(&TaskServerNode::HandleSparseCloud, this, std::placeholders::_1),
      map_subscription_options);
    sparse_delta_subscription_ = create_subscription<mission_msgs::msg::GlobalSparseMapDelta>(
      "/global_sparse_map_delta", snapshot_qos,
      std::bind(&TaskServerNode::HandleSparseDelta, this, std::placeholders::_1),
      map_subscription_options);
    keyframe_sparse_evidence_subscription_ =
      create_subscription<mission_msgs::msg::KeyframeSparseEvidenceDelta>(
      "/global_keyframe_sparse_evidence_delta", snapshot_qos,
      std::bind(&TaskServerNode::HandleKeyframeSparseEvidenceDelta, this, std::placeholders::_1),
      map_subscription_options);
    for (const auto drone_id : config.drones) {
      const auto topic = "/dron_" + std::to_string(drone_id) +
        "/orbslam/navigation_state";
      navigation_subscriptions_.push_back(
        create_subscription<orbslam3_msgs::msg::NavigationState>(
          topic, rclcpp::QoS(20).reliable(),
          [this, drone_id](orbslam3_msgs::msg::NavigationState::ConstSharedPtr state) {
            HandleNavigation(drone_id, state);
          }, map_subscription_options));
      const auto keyframe_pose_topic = "/dron_" + std::to_string(drone_id) +
        "/orbslam/global_keyframe_pose";
      global_keyframe_pose_subscriptions_.push_back(
        create_subscription<orbslam3_msgs::msg::GlobalKeyFramePose>(
          keyframe_pose_topic, rclcpp::QoS(rclcpp::KeepLast(8)).reliable(),
          [this](orbslam3_msgs::msg::GlobalKeyFramePose::ConstSharedPtr pose) {
            HandleGlobalKeyframePose(pose);
          }, map_subscription_options));
    }
    if (flow_enabled_) {
      flow_publisher_ = create_publisher<std_msgs::msg::String>(
        "/mission/flow_events", rclcpp::QoS(100).reliable());
    }
    if (architecture_enabled) {
      architecture_publisher_ = create_publisher<std_msgs::msg::String>(
        "/system_architecture/activity", rclcpp::QoS(100).reliable());
    }
    register_service_ = create_service<mission_msgs::srv::RegisterDrone>(
      "/mission/register_drone",
      [this](const std::shared_ptr<mission_msgs::srv::RegisterDrone::Request> request,
      std::shared_ptr<mission_msgs::srv::RegisterDrone::Response> response)
      {
        HandleRegistration(request, response);
      }, rmw_qos_profile_services_default, map_callback_group_);
    plan_route_service_ = create_service<mission_msgs::srv::PlanRoute>(
      "/mission/plan_route",
      [this](const std::shared_ptr<mission_msgs::srv::PlanRoute::Request> request,
      std::shared_ptr<mission_msgs::srv::PlanRoute::Response> response)
      {
        HandlePlanRoute(request, response);
      }, rmw_qos_profile_services_default, map_callback_group_);
    autonomous_result_service_ = create_service<ReportAutonomousResult>(
      "/mission/report_autonomous_result",
      [this](const std::shared_ptr<ReportAutonomousResult::Request> request,
      std::shared_ptr<ReportAutonomousResult::Response> response)
      {
        HandleAutonomousResult(request, response);
      }, rclcpp::ServicesQoS(), workflow_ingress_callback_group_);

    PublishGeometry();
    PublishRegistry();
    PublishTaskStates();
    PublishVoxelMap();
    PublishGeometryFlow();
    PublishArchitecture("task_server_to_gui_geometry", "MISSION_GEOMETRY_READY");
    if (flow_enabled_) {
      PublishArchitecture("task_server_to_sim_mission_flow", "MISSION_FLOW_EVENT");
      flow_republish_timer_ = create_timer(
        std::chrono::seconds(2), [this]() {
          PublishGeometryFlow();
          flow_republish_timer_->cancel();
        }, map_callback_group_);
    }
    free_pose_requery_timer_ = rclcpp::create_wall_timer(
      std::chrono::seconds(2), [this]() {RequestKnownKeyframePoses();}, map_callback_group_,
      get_node_base_interface().get(), get_node_timers_interface().get());
    facade_worker_timer_ = rclcpp::create_wall_timer(
      std::chrono::milliseconds(facade_worker_period_ms_),
      [this]() {RunFacadeWorker();}, map_callback_group_, get_node_base_interface().get(),
      get_node_timers_interface().get());
    execution_gate_apply_timer_ = rclcpp::create_wall_timer(
      std::chrono::milliseconds(50), [this]() {ApplyExecutionGateRequest();},
      map_callback_group_, get_node_base_interface().get(), get_node_timers_interface().get());
    voxel_worker_timer_ = rclcpp::create_wall_timer(
      std::chrono::milliseconds(voxel_worker_coalesce_ms_),
      [this]() {RunVoxelMapWorker();}, map_callback_group_, get_node_base_interface().get(),
      get_node_timers_interface().get());
    depth_integration_worker_timer_ = rclcpp::create_wall_timer(
      std::chrono::milliseconds(50), [this]() {RunDepthIntegrationWorker();},
      workflow_dispatch_callback_group_, get_node_base_interface().get(),
      get_node_timers_interface().get());
    point_selection_worker_timer_ = rclcpp::create_wall_timer(
      std::chrono::milliseconds(50), [this]() {RunPointSelectionWorker();},
      workflow_dispatch_callback_group_, get_node_base_interface().get(),
      get_node_timers_interface().get());
    trajectory_planning_worker_timer_ = rclcpp::create_wall_timer(
      std::chrono::milliseconds(50), [this]() {RunTrajectoryPlanningWorker();},
      workflow_dispatch_callback_group_, get_node_base_interface().get(),
      get_node_timers_interface().get());
    active_trajectory_monitor_worker_timer_ = rclcpp::create_wall_timer(
      std::chrono::milliseconds(50), [this]() {RunActiveTrajectoryMonitorWorker();},
      workflow_dispatch_callback_group_, get_node_base_interface().get(),
      get_node_timers_interface().get());
    RCLCPP_INFO(
      get_logger(),
      "[F6A-MISSION-CONFIG] mission=%s revision=%lu frame=%s hard_volume=derived",
      mission_id_.c_str(), geometry_data_.config_revision, mission_frame_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "[F6A-WORKFLOW-READY] queues=assignment,selection,planning,depth,monitor callbacks=separated legacy_dispatch=unchanged");
    RCLCPP_INFO(
      get_logger(), "[F6B-GEOMETRY] levels=%zu regions=%zu unassigned=true",
      geometry_data_.levels.size(), geometry_data_.regions.size());
    RCLCPP_INFO(
      get_logger(), "[F6D-VOXEL] size=%.3f source=%s sparse_occupied=%.2f visual_target=[%.2f,%.2f] free_live=true body_half=(%.2f,%.2f,%.2f)",
      voxel_size_, pose_source_.c_str(), voxel_occupied_score_threshold_,
      visual_target_min_score_, visual_target_max_score_, free_body_half_extent_.x,
      free_body_half_extent_.y, free_body_half_extent_.z);
    RCLCPP_INFO(
      get_logger(),
      "[F6H-FACADE-CONFIG] wall_distance_m=%.2f displacement_m=%.2f completion_ratio=%.3f unknown_transitable=true candidate_step_m=%.2f max_inspection_failures=%u",
      facade_preferences_.preferred_wall_distance_m,
      facade_preferences_.preferred_displacement_m,
      facade_preferences_.completion_ratio, facade_candidate_step_m_,
      facade_max_inspection_failures_);
    RCLCPP_INFO(
      get_logger(),
      "[F6G-PLANNER-READY] unknown_cost=%.2f extra_clearance_voxels=%ld margin=%.2f max_expansions=%zu epsilon=%.2f coarse_factor=%ld corridor_initial=%ld corridor_step=%ld",
      planner_parameters_.unknown_cost_multiplier, extra_obstacle_clearance_voxels_,
      planner_parameters_.search_margin_m, planner_parameters_.max_expansions,
      planner_parameters_.heuristic_weight, planning_coarse_voxel_factor_,
      planner_parameters_.initial_corridor_half_width_cells,
      planner_parameters_.corridor_expand_step_cells);
  }

private:
  void PublishGeometry()
  {
    mission_msgs::msg::MissionGeometry message;
    message.header.stamp = now();
    message.header.frame_id = mission_frame_;
    message.mission_id = mission_id_;
    message.config_revision = geometry_data_.config_revision;
    message.mapping_roi = ToBox(geometry_data_.mapping_roi);
    message.hard_flight_volume = ToBox(geometry_data_.hard_flight_volume);
    message.mapping_hysteresis = {geometry_data_.mapping_hysteresis.x,
      geometry_data_.mapping_hysteresis.y, geometry_data_.mapping_hysteresis.z};
    message.level_height = geometry_data_.level_height;
    for (const auto & source : geometry_data_.levels) {
      mission_msgs::msg::MappingLevel level;
      level.level_index = source.level_index;
      level.z_min = source.z_min;
      level.z_max = source.z_max;
      message.levels.push_back(level);
    }
    for (const auto & source : geometry_data_.regions) {
      mission_msgs::msg::BaseSubRoi region;
      region.region_id = source.region_id;
      region.level_index = source.level_index;
      region.side = static_cast<std::uint8_t>(source.side);
      region.bounds = ToBox(source.bounds);
      region.ownership_revision = 0U;
      message.regions.push_back(region);
    }
    geometry_publisher_->publish(message);
  }

  void PublishRegistry()
  {
    mission_msgs::msg::DroneRegistry message;
    message.header.stamp = now();
    message.header.frame_id = mission_frame_;
    message.mission_id = mission_id_;
    message.config_revision = geometry_data_.config_revision;
    message.drones = registry_->Snapshot();
    registry_publisher_->publish(message);
  }

  void PublishTaskStates()
  {
    mission_msgs::msg::TaskStateArray message;
    message.header.stamp = now();
    message.header.frame_id = mission_frame_;
    message.mission_id = mission_id_;
    message.config_revision = geometry_data_.config_revision;
    message.tasks = tasks_;
    task_state_publisher_->publish(message);
  }

  void PublishVoxelMap()
  {
    mission_msgs::msg::VoxelMap message;
    message.header.stamp = now();
    message.header.frame_id = mission_frame_;
    message.mission_id = mission_id_;
    message.config_revision = geometry_data_.config_revision;
    message.map_revision = voxel_map_->revision();
    message.voxel_size = voxel_size_;
    for (const auto & cell : voxel_map_->Snapshot()) {
      mission_msgs::msg::VoxelCell voxel;
      voxel.ix = cell.key.ix;
      voxel.iy = cell.key.iy;
      voxel.iz = cell.key.iz;
      voxel.state = static_cast<std::uint8_t>(cell.state);
      voxel.score = cell.score;
      message.voxels.push_back(voxel);
    }
    for (const auto & reservation : reservation_overlay_.Snapshot()) {
      for (const auto & cell : reservation.cells) {
        mission_msgs::msg::VoxelCell voxel;
        voxel.ix = cell.ix;
        voxel.iy = cell.iy;
        voxel.iz = cell.iz;
        voxel.state = mission_msgs::msg::VoxelCell::RESERVED;
        message.reserved_voxels.push_back(voxel);
      }
    }
    voxel_map_publisher_->publish(message);
  }

  void MarkActiveCorridorChanges(
    const std::vector<task_lib::VoxelChange> & raw_changes,
    const task_lib::NavigationUpdate & update)
  {
    for (const auto & change : update.changes) {
      if (!change.before.traversable || change.after.traversable) {
        continue;
      }
      for (auto & item : execution_runtime_by_drone_) {
        auto & runtime = item.second;
        const auto & profiles = runtime.require_known_free ? drone_facade_navigation_profiles_ :
          drone_navigation_profiles_;
        const auto profile = profiles.find(item.first);
        if (profile == profiles.end() || profile->second != update.profile_id) {
          continue;
        }
        if (!runtime.in_flight || runtime.stop_requested ||
          runtime.corridor.count(change.key) == 0U)
        {
          continue;
        }
        const auto inflation = drone_inflation_cells_.find(item.first);
        const task_lib::VoxelKey obstacle_inflation = inflation == drone_inflation_cells_.end() ?
          task_lib::VoxelKey{} : inflation->second;
        const task_lib::VoxelChange * cause = nullptr;
        double cause_distance_squared = std::numeric_limits<double>::infinity();
        for (const auto & raw : raw_changes) {
          if (raw.after != task_lib::VoxelState::Occupied) {
            continue;
          }
          const auto dx = static_cast<double>(change.key.ix - raw.key.ix);
          const auto dy = static_cast<double>(change.key.iy - raw.key.iy);
          const auto dz = static_cast<double>(change.key.iz - raw.key.iz);
          if (std::abs(dx) > obstacle_inflation.ix || std::abs(dy) > obstacle_inflation.iy ||
            std::abs(dz) > obstacle_inflation.iz)
          {
            continue;
          }
          const double distance_squared = dx * dx + dy * dy + dz * dz;
          if (distance_squared < cause_distance_squared) {
            cause = &raw;
            cause_distance_squared = distance_squared;
          }
        }
        runtime.corridor_affected = true;
        RCLCPP_INFO(
          get_logger(),
          "[F6I-CORRIDOR-CHANGE] drone=%u trajectory_id=%s voxel=%s revision=%lu severity=occupied_or_inflated",
          item.first, runtime.trajectory_id.c_str(), VoxelKeyToken(change.key).c_str(),
          update.raw_revision);
        if (debug_trajectory_diagnostics_) {
          const auto plan = execution_plans_.find(runtime.trajectory_id);
          const auto closest = plan == execution_plans_.end() ?
            std::pair<std::size_t, double>{0U, std::numeric_limits<double>::infinity()} :
          ClosestPlanSegment(plan->second, VoxelCenter(change.key, voxel_size_));
          const bool direct = cause != nullptr && cause->key == change.key;
          RCLCPP_INFO(
            get_logger(),
            "[F6I-STOP-CAUSAL] drone=%u trajectory_id=%s nav_voxel=%s raw_voxel=%s raw_before=%s raw_after=%s cause=%s segment=%zu distance_m=%.3f revision=%lu",
            item.first, runtime.trajectory_id.c_str(), VoxelKeyToken(change.key).c_str(),
            cause == nullptr ? "unknown" : VoxelKeyToken(cause->key).c_str(),
            cause == nullptr ? "unknown" : VoxelStateName(cause->before),
            cause == nullptr ? "unknown" : VoxelStateName(cause->after),
            cause == nullptr ? "unresolved_navigation_change" :
            (direct ? "raw_occupied_direct" : "raw_occupied_static_clearance"),
            closest.first, closest.second, update.raw_revision);
        }
        RequestActiveExecutionStop(item.first, "occupied_or_inflated_corridor");
      }
    }
  }

  double ExecutionVelocityForTarget(const task_lib::VoxelKey & target) const
  {
    (void)target;
    return execution_nominal_velocity_mps_;
  }

  void RequestActiveExecutionStop(std::uint32_t drone_id, const char * reason)
  {
    auto runtime_it = execution_runtime_by_drone_.find(drone_id);
    if (runtime_it == execution_runtime_by_drone_.end() || !runtime_it->second.in_flight ||
      runtime_it->second.stop_requested)
    {
      return;
    }
    auto & runtime = runtime_it->second;
    const auto plan_it = execution_plans_.find(runtime.trajectory_id);
    if (plan_it == execution_plans_.end()) {
      RCLCPP_ERROR(
        get_logger(), "[F6I-EXECUTION-STOP-REJECTED] trajectory_id=%s reason=missing_plan",
        runtime.trajectory_id.c_str());
      return;
    }
    runtime.stop_requested = true;
    plan_it->second.execution_state = mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_CANCELED;
    plan_it->second.execution_detail = "STOP solicitado; frenando en pose actual";
    planned_route_publisher_->publish(plan_it->second);
    RCLCPP_WARN(
      get_logger(), "[F6I-EXECUTION-STOP-REQUEST] task=%s trajectory_id=%s reason=%s",
      runtime.task_id.c_str(), runtime.trajectory_id.c_str(), reason);

    ExecuteTrajectory::Goal stop_goal;
    stop_goal.plan = plan_it->second;
    stop_goal.plan.waypoints.clear();
    stop_goal.stop_at_current_pose = true;
    const auto trajectory_id = runtime.trajectory_id;
    const auto current_pose = navigation_poses_.find(drone_id);
    runtime.stop_hold_position = current_pose == navigation_poses_.end() ?
      std::nullopt : std::optional<task_lib::Vec3>{current_pose->second.position};
    rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions options;
    options.goal_response_callback = [this, drone_id, trajectory_id](
      std::shared_ptr<rclcpp_action::ClientGoalHandle<ExecuteTrajectory>> handle) {
        if (handle) {
          RCLCPP_INFO(
            get_logger(), "[F6I-EXECUTION-STOP-ACTIVE] trajectory_id=%s",
            trajectory_id.c_str());
          return;
        }
        RCLCPP_ERROR(
          get_logger(), "[F6I-EXECUTION-STOP-REJECTED] trajectory_id=%s reason=task_manager_rejected",
          trajectory_id.c_str());
        auto & runtime = execution_runtime_by_drone_[drone_id];
        if (runtime.trajectory_id == trajectory_id) {
          runtime.stop_requested = false;
          const auto plan_it = execution_plans_.find(trajectory_id);
          if (plan_it != execution_plans_.end()) {
            plan_it->second.execution_state =
              mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_ACTIVE;
            plan_it->second.execution_detail = "STOP rechazado; se conserva la trayectoria activa";
            planned_route_publisher_->publish(plan_it->second);
          }
        }
      };
    options.result_callback = [this, drone_id, trajectory_id](
      const rclcpp_action::ClientGoalHandle<ExecuteTrajectory>::WrappedResult & result) {
        const bool completed = result.code == rclcpp_action::ResultCode::SUCCEEDED &&
          result.result && result.result->success;
        RCLCPP_INFO(
          get_logger(), "[F6I-EXECUTION-STOP-FINAL] trajectory_id=%s success=%s detail=%s",
          trajectory_id.c_str(), completed ? "true" : "false",
          result.result ? result.result->reason.c_str() : "missing_result");
        auto & runtime = execution_runtime_by_drone_[drone_id];
        if (runtime.trajectory_id != trajectory_id) {
          return;
        }
        if (runtime.autonomous_workflow) {
          const auto body = drone_reservation_body_cells_.find(drone_id);
          const task_lib::VoxelKey body_half_extent = body == drone_reservation_body_cells_.end() ?
            task_lib::VoxelKey{} : body->second;
          const auto current_pose = navigation_poses_.find(drone_id);
          const auto hold_position = current_pose == navigation_poses_.end() ?
            runtime.stop_hold_position : std::optional<task_lib::Vec3>{current_pose->second.position};
          if (completed && hold_position.has_value() && !runtime.reservation_id.empty()) {
            const auto hold_cells = FootprintAt(*hold_position, voxel_size_, body_half_extent);
            const std::string hold_reservation_id = runtime.reservation_id + "_hold";
            if (reservation_overlay_.Replace(
                runtime.reservation_id, {hold_reservation_id, drone_id,
                  task_lib::ReservationMode::Hold, hold_cells, 0U}))
            {
              runtime.reservation_id = hold_reservation_id;
              runtime.corridor = hold_cells;
            }
          }
          runtime.in_flight = false;
          runtime.stop_requested = false;
          runtime.corridor_affected = false;
          runtime.stop_hold_position.reset();
          PublishVoxelMap();
          RCLCPP_INFO(
            get_logger(),
            "[F6J-AUTONOMOUS-STOP-HOLD] drone=%u command=%s trajectory_id=%s success=%s",
            drone_id, runtime.autonomous_command_id.c_str(), trajectory_id.c_str(),
            completed ? "true" : "false");
          return;
        }
        auto terminal_plan = runtime.active_plan;
        terminal_plan.execution_state = mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_CANCELED;
        terminal_plan.execution_detail = completed ?
          "STOP completado: dron estabilizado en pose actual" :
          "STOP no completado: requiere intervencion";
        planned_route_publisher_->publish(terminal_plan);
        const auto body = drone_reservation_body_cells_.find(drone_id);
        const task_lib::VoxelKey body_half_extent = body == drone_reservation_body_cells_.end() ?
          task_lib::VoxelKey{} : body->second;
        const auto current_pose = navigation_poses_.find(drone_id);
        const auto hold_position = current_pose == navigation_poses_.end() ?
          runtime.stop_hold_position : std::optional<task_lib::Vec3>{current_pose->second.position};
        if (!hold_position.has_value()) {
          RCLCPP_ERROR(
            get_logger(),
            "[F6J-RESERVATION-HOLD-REJECTED] drone=%u trajectory_id=%s reason=missing_pose",
            drone_id, trajectory_id.c_str());
          runtime.stop_requested = false;
          return;
        }
        const auto hold_cells = FootprintAt(*hold_position, voxel_size_, body_half_extent);
        const std::string hold_reservation_id = runtime.reservation_id + "_hold";
        if (!reservation_overlay_.Replace(
            runtime.reservation_id, {hold_reservation_id, drone_id,
              task_lib::ReservationMode::Hold, hold_cells, 0U}))
        {
          RCLCPP_ERROR(
            get_logger(),
            "[F6J-RESERVATION-HOLD-REJECTED] drone=%u trajectory_id=%s",
            drone_id, trajectory_id.c_str());
          runtime.stop_requested = false;
          return;
        }
        runtime.reservation_id = hold_reservation_id;
        runtime.in_flight = false;
        runtime.stop_requested = false;
        runtime.trajectory_id.clear();
        runtime.active_plan = mission_msgs::msg::TrajectoryPlan{};
        runtime.target.reset();
        runtime.corridor = hold_cells;
        runtime.stop_hold_position.reset();
        runtime.corridor_affected = false;
        PublishVoxelMap();
        RCLCPP_INFO(
          get_logger(),
          "[F6J-RESERVATION-HOLD] drone=%u trajectory_id=%s cells=%zu pose=(%.3f,%.3f,%.3f)",
          drone_id, trajectory_id.c_str(), hold_cells.size(), hold_position->x,
          hold_position->y, hold_position->z);
        if (completed) {
          if (fiducial_interrupt_task_by_drone_.count(drone_id) != 0U) {
            FinalizeFiducialInterrupt(drone_id, "fiducial_stop_completed");
            return;
          }
          EnqueueReadyDrone(drone_id, "stop_completed_hold_owner", true);
          DispatchNextPendingPlan();
        }
      };
    const auto client = execute_trajectory_clients_.find(drone_id);
    if (client == execute_trajectory_clients_.end() || !client->second) {
      RCLCPP_ERROR(
        get_logger(), "[F6I-EXECUTION-STOP-REJECTED] trajectory_id=%s reason=missing_drone_client",
        trajectory_id.c_str());
      return;
    }
    client->second->async_send_goal(stop_goal, options);
  }

  void HandleSafetyEvent(mission_msgs::msg::SafetyEvent::ConstSharedPtr event)
  {
    if (!event || event->drone_id == 0U) {
      return;
    }
    const auto runtime_it = execution_runtime_by_drone_.find(event->drone_id);
    if (runtime_it == execution_runtime_by_drone_.end() ||
      runtime_it->second.trajectory_id.empty())
    {
      return;
    }
    auto & runtime = runtime_it->second;
    if (event->event_type == mission_msgs::msg::SafetyEvent::DEPTH_EMERGENCY) {
      if (runtime.stop_requested) {
        return;
      }
      runtime.stop_requested = true;
      const auto plan_it = execution_plans_.find(runtime.trajectory_id);
      if (plan_it != execution_plans_.end()) {
        plan_it->second.execution_state =
          mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_CANCELED;
        plan_it->second.execution_detail = "DEPTH_EMERGENCY: STOP local en curso";
        planned_route_publisher_->publish(plan_it->second);
      }
      if (!runtime.reservation_id.empty()) {
        reservation_overlay_.Release(runtime.reservation_id);
        runtime.reservation_id.clear();
        runtime.corridor.clear();
        PublishVoxelMap();
      }
      RCLCPP_WARN(
        get_logger(), "[F6N-DEPTH-STOP-STARTED] drone=%u trajectory_id=%s detail=%s",
        event->drone_id, runtime.trajectory_id.c_str(), event->detail.c_str());
      return;
    }
    if (event->event_type == mission_msgs::msg::SafetyEvent::STOPPED && runtime.stop_requested) {
      RCLCPP_INFO(
        get_logger(), "[F6N-DEPTH-STOP-FINAL] drone=%u trajectory_id=%s detail=%s",
        event->drone_id, runtime.trajectory_id.c_str(), event->detail.c_str());
      ReleaseExecutionRuntime(event->drone_id, "depth_local_stop_completed");
    }
  }

  void HandleFiducialPrimaryObservation(
    mission_msgs::msg::FiducialPrimaryObservation::ConstSharedPtr event)
  {
    if (!event || event->drone_id == 0U || event->object_id < 0) {
      return;
    }
    const auto identity = std::make_tuple(event->drone_id, event->map_epoch, event->object_id);
    if (!seen_fiducials_.insert(identity).second) {
      return;
    }
    const auto task = std::find_if(
      tasks_.begin(), tasks_.end(), [event](const auto & candidate) {
        return candidate.task_type == "MAP_SECTION" &&
        candidate.assigned_drone_id == event->drone_id &&
        (candidate.state == mission_msgs::msg::TaskState::ASSIGNED ||
        candidate.state == mission_msgs::msg::TaskState::RUNNING ||
        candidate.state == mission_msgs::msg::TaskState::PAUSED);
      });
    RCLCPP_WARN(
      get_logger(),
      "[F6H-FIDUCIAL-PRIMARY] drone=%u epoch=%lu object=%d visit=%lu task=%s",
      event->drone_id, event->map_epoch, event->object_id, event->visit_id,
      task == tasks_.end() ? "none" : task->task_id.c_str());
    if (task == tasks_.end()) {
      return;
    }
    auto * facade = FacadeRuntimeFor(*task);
    if (facade != nullptr) {
      facade->segment_start_ratio.reset();
      facade->segment_target_ratio.reset();
      facade->inspected_target.reset();
      facade->awaiting_free_prefix_completion = false;
      facade->awaiting_depth = false;
      facade->inspection_stage = FacadeInspectionStage::IDLE;
    }
    task->state = mission_msgs::msg::TaskState::TO_FINISH;
    fiducial_interrupt_task_by_drone_[event->drone_id] = task->task_id;
    if (facade != nullptr) {
      SyncFacadeTaskState(
        *task, *facade,
        "A terminar: barrido interrumpido por fiducial primario no observado");
    } else {
      ++task->state_revision;
      task->detail = "A terminar: barrido interrumpido por fiducial primario no observado";
      PublishTaskStates();
    }
    const auto execution = execution_runtime_by_drone_.find(event->drone_id);
    if (execution != execution_runtime_by_drone_.end() && execution->second.in_flight) {
      RequestActiveExecutionStop(event->drone_id, "new_primary_fiducial");
      return;
    }
    if (facade != nullptr && facade->inspection_pending) {
      RCLCPP_INFO(
        get_logger(),
        "[F6H-FIDUCIAL-INTERRUPT-WAIT] drone=%u task=%s reason=inspection_in_progress",
        event->drone_id, task->task_id.c_str());
      return;
    }
    FinalizeFiducialInterrupt(event->drone_id, "fiducial_idle_interrupt");
  }

  void HandleVisualRiskEvent(mission_msgs::msg::VisualRiskEvent::ConstSharedPtr event)
  {
    if (!event || event->drone_id == 0U || event->trajectory_id.empty()) {
      return;
    }
    auto runtime_it = execution_runtime_by_drone_.find(event->drone_id);
    if (runtime_it == execution_runtime_by_drone_.end() ||
      runtime_it->second.trajectory_id != event->trajectory_id)
    {
      return;
    }
    auto & runtime = runtime_it->second;
    const bool autonomous_workflow = runtime.autonomous_workflow;
    if (event->event_type == mission_msgs::msg::VisualRiskEvent::EVENT_STOP_STARTED) {
      if (runtime.stop_requested) {
        return;
      }
      runtime.stop_requested = true;
      const auto plan_it = execution_plans_.find(event->trajectory_id);
      if (plan_it != execution_plans_.end()) {
        plan_it->second.execution_state =
          mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_CANCELED;
        plan_it->second.execution_detail = "TRACKING_RISK: STOP local en curso";
        planned_route_publisher_->publish(plan_it->second);
      }
      if (!runtime.reservation_id.empty()) {
        reservation_overlay_.Release(runtime.reservation_id);
        runtime.reservation_id.clear();
        runtime.corridor.clear();
        PublishVoxelMap();
      }
      visual_risk_events_[event->drone_id] = *event;
      const auto pose = navigation_poses_.find(event->drone_id);
      if (pose != navigation_poses_.end()) {
        visual_cautions_[event->drone_id] = VisualCaution{
          pose->second.position, pose->second.map_epoch, pose->second.pose_revision,
          event->frame_id, event->risk_mask, event->recommended_yaw_rad,
          event->recommended_camera_pitch_rad, event->trajectory_id};
        RCLCPP_WARN(
          get_logger(),
          "[F6L-VISUAL-CAUTION] drone=%u trajectory_id=%s epoch=%lu pose=(%.3f,%.3f,%.3f) "
          "sector=%u",
          event->drone_id, event->trajectory_id.c_str(), pose->second.map_epoch,
          pose->second.position.x, pose->second.position.y, pose->second.position.z,
          event->risk_mask);
      }
      RCLCPP_WARN(
        get_logger(), "[F6L-VISUAL-STOP-STARTED] drone=%u trajectory_id=%s frame=%lu mask=%u",
        event->drone_id, event->trajectory_id.c_str(), event->frame_id, event->risk_mask);
      return;
    }
    if (event->event_type == mission_msgs::msg::VisualRiskEvent::EVENT_STOP_COMPLETED) {
      if (!runtime.stop_requested || !event->success) {
        if (autonomous_workflow) {
          RCLCPP_WARN(
            get_logger(),
            "[F6L-AUTONOMOUS-VISUAL-TERMINAL-WAIT] drone=%u trajectory_id=%s reason=stop_not_completed",
            event->drone_id, event->trajectory_id.c_str());
          return;
        }
        ReleaseExecutionRuntime(event->drone_id, "visual_stop_not_completed");
        return;
      }
      runtime.stop_requested = false;
      runtime.local_visual_reorientation_active = true;
      RCLCPP_INFO(
        get_logger(),
        "[F6L-LOCAL-REORIENT-WAIT] drone=%u source=%s yaw_deg=%.3f pitch_deg=%.3f",
        event->drone_id, event->trajectory_id.c_str(), event->recommended_yaw_rad * 180.0 / M_PI,
        event->recommended_camera_pitch_rad * 180.0 / M_PI);
      return;
    }
    if (event->event_type == mission_msgs::msg::VisualRiskEvent::EVENT_REORIENTATION_STARTED) {
      if (runtime.local_visual_reorientation_active) {
        RCLCPP_INFO(
          get_logger(), "[F6L-LOCAL-REORIENT-STARTED] drone=%u source=%s",
          event->drone_id, event->trajectory_id.c_str());
      }
      return;
    }
    if (event->event_type != mission_msgs::msg::VisualRiskEvent::EVENT_REORIENTATION_COMPLETED ||
      !runtime.local_visual_reorientation_active)
    {
      return;
    }
    runtime.local_visual_reorientation_active = false;
    if (event->success) {
      const auto pose = navigation_poses_.find(event->drone_id);
      const std::uint64_t epoch = pose == navigation_poses_.end() ? 0U : pose->second.map_epoch;
      navigation_orientations_[event->drone_id] = NavigationOrientation{
        event->recommended_yaw_rad, event->recommended_camera_pitch_rad, epoch, true};
      RCLCPP_INFO(
        get_logger(),
        "[F6M-ORIENTATION-UPDATED] drone=%u source=%s yaw_deg=%.3f pitch_deg=%.3f local=true",
        event->drone_id, event->trajectory_id.c_str(), event->recommended_yaw_rad * 180.0 / M_PI,
        event->recommended_camera_pitch_rad * 180.0 / M_PI);
    }
    if (autonomous_workflow) {
      RCLCPP_INFO(
        get_logger(),
        "[F6L-AUTONOMOUS-VISUAL-TERMINAL-WAIT] drone=%u trajectory_id=%s reason=reorientation_terminal",
        event->drone_id, event->trajectory_id.c_str());
      return;
    }
    ReleaseExecutionRuntime(
      event->drone_id, event->success ? "visual_local_reorientation_completed" :
      "visual_local_reorientation_failed");
  }

  void FlushVoxelChanges(const char * event)
  {
    const auto raw_changes = voxel_map_->TakeChanges();
    if (raw_changes.empty()) {
      return;
    }
    std::map<std::string, task_lib::NavigationUpdate> updates;
    for (const auto & profile : navigation_profiles_) {
      auto update = voxel_map_->RefreshNavigation(profile, raw_changes);
      RCLCPP_INFO(
        get_logger(),
        "[F6D-NAV-UPDATE] profile=%s raw_revision=%lu raw_changes=%zu recomputed=%zu edge_changes=%zu coarse_changes=%zu elapsed_ms=%.3f",
        profile.c_str(), update.raw_revision, raw_changes.size(), update.recomputed_cells,
        update.changes.size(), update.coarse_changes.size(), update.elapsed_ms);
      updates.emplace(profile, std::move(update));
    }
    for (auto & planner : planners_) {
      const auto profile = drone_navigation_profiles_.find(planner.first);
      if (profile == drone_navigation_profiles_.end()) {
        continue;
      }
      const auto update = updates.find(profile->second);
      if (update != updates.end()) {
        planner.second.ApplyNavigationChanges(
          update->second.changes, update->second.coarse_changes, voxel_map_->revision());
      }
    }
    for (const auto & update : updates) {
      MarkActiveCorridorChanges(raw_changes, update.second);
    }
    PublishVoxelMap();
    PublishFlow("voxel_to_gui", "VOXEL_MAP_UPDATED");
    PublishFlow("voxel_to_planning", "MAP_CHANGE_EVENT");
    PublishArchitecture("task_server_to_gui_voxels", event);
  }

  void HandleRegistration(
    const std::shared_ptr<mission_msgs::srv::RegisterDrone::Request> request,
    const std::shared_ptr<mission_msgs::srv::RegisterDrone::Response> response)
  {
    PublishFlow("manager_to_registration", "REGISTER_DRONE_REQUEST");
    const auto result = registry_->Register(request->registration);
    response->accepted = result.accepted;
    response->reason = result.reason;
    response->mission_id = mission_id_;
    response->mission_frame = mission_frame_;
    response->config_revision = geometry_data_.config_revision;
    response->voxel_size = voxel_size_;
    response->protocol_version = kProtocolVersion;
    if (result.changed) {
      const auto profile_id = NavigationProfileId(request->registration);
      const auto clearance_cells = [this](double dimension_m) {
          return static_cast<std::int64_t>(std::ceil(dimension_m * 0.5 / voxel_size_)) +
                 extra_obstacle_clearance_voxels_;
        };
      const auto body_cells = [this](double dimension_m) {
          return static_cast<std::int64_t>(std::ceil(dimension_m * 0.5 / voxel_size_));
        };
      const auto profile = task_lib::NavigationProfile{
        profile_id,
        task_lib::VoxelKey{
          clearance_cells(request->registration.dimensions_m.x),
          clearance_cells(request->registration.dimensions_m.y),
          clearance_cells(request->registration.dimensions_m.z)},
        planner_parameters_.unknown_cost_multiplier, planning_coarse_voxel_factor_, false};
      const auto facade_profile = task_lib::NavigationProfile{
        profile_id + ":facade_free", profile.obstacle_inflation_cells,
        profile.unknown_cost_multiplier, profile.coarse_voxel_factor, true};
      if (!voxel_map_->RegisterNavigationProfile(profile) ||
        !voxel_map_->RegisterNavigationProfile(facade_profile))
      {
        response->accepted = false;
        response->reason = "perfil navegable incompatible";
        return;
      }
      navigation_profiles_.insert(profile_id);
      navigation_profiles_.insert(facade_profile.profile_id);
      drone_navigation_profiles_[request->registration.drone_id] = profile_id;
      drone_facade_navigation_profiles_[request->registration.drone_id] =
        facade_profile.profile_id;
      drone_inflation_cells_[request->registration.drone_id] = profile.obstacle_inflation_cells;
      drone_reservation_body_cells_[request->registration.drone_id] = {
        body_cells(request->registration.dimensions_m.x),
        body_cells(request->registration.dimensions_m.y),
        body_cells(request->registration.dimensions_m.z)};
      PublishRegistry();
      PublishFlow("registration_to_task_worker", "DRONE_REGISTERED");
      PublishFlow("task_worker_registry", "DRONE_REGISTRY_SNAPSHOT");
      PublishArchitecture("task_manager_to_task_server", "REGISTER_DRONE");
      RCLCPP_INFO(
        get_logger(),
        "[F6D-NAV-PROFILE] drone=%u profile=%s facade_profile=%s obstacle_inflation=(%ld,%ld,%ld) body=(%ld,%ld,%ld) coarse_factor=%ld",
        request->registration.drone_id, profile_id.c_str(),
        facade_profile.profile_id.c_str(),
        profile.obstacle_inflation_cells.ix, profile.obstacle_inflation_cells.iy,
        profile.obstacle_inflation_cells.iz,
        drone_reservation_body_cells_[request->registration.drone_id].ix,
        drone_reservation_body_cells_[request->registration.drone_id].iy,
        drone_reservation_body_cells_[request->registration.drone_id].iz,
        profile.coarse_voxel_factor);
    }
    RCLCPP_INFO(
      get_logger(), "[F6C-REGISTRY] drone=%u accepted=%s changed=%s reason=%s",
      request->registration.drone_id, result.accepted ? "true" : "false",
      result.changed ? "true" : "false", result.reason.c_str());
    if (result.accepted && eligible_drones_.count(request->registration.drone_id) != 0U) {
      EnqueueTaskAvailableDrone(request->registration.drone_id, "registered_after_anchor");
    }
    AssignAvailableTasks();
  }

  void HandleAutonomousResult(
    const std::shared_ptr<ReportAutonomousResult::Request> request,
    std::shared_ptr<ReportAutonomousResult::Response> response)
  {
    if (!request || request->mission_id != mission_id_ || request->drone_id == 0U ||
      request->task_id.empty() || request->workflow_id.empty() || request->command_id.empty())
    {
      response->reason = "invalid_autonomous_result";
      return;
    }
    if (workflow_scheduler_.HasAcceptedResult(request->command_id)) {
      response->accepted = true;
      response->duplicate = true;
      response->reason = "duplicate_accepted";
      return;
    }
    workflow_scheduler_.CompleteNormalCommand(request->drone_id, request->command_id);
    const auto terminal_state = request->result_status ==
      ReportAutonomousResult::Request::RESULT_COMPLETED ?
      mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_COMPLETED :
      mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_CANCELED;
    ReleaseAutonomousTrajectoryMonitor(
      request->drone_id, request->command_id, request->reason, terminal_state);
    task_server::WorkflowWorkItem continuation;
    continuation.queue =
      request->command_type == SubmitAutonomousCommand::Request::COMMAND_MOVE_AND_WATCH_FIDUCIAL ?
      task_server::WorkflowQueue::POINT_SELECTION :
      task_server::WorkflowQueue::DEPTH_INTEGRATION;
    continuation.identity.drone_id = request->drone_id;
    continuation.identity.task_id = request->task_id;
    continuation.identity.workflow_id = request->workflow_id;
    continuation.identity.command_id = request->command_id;
    continuation.identity.map_epoch = request->map_epoch;
    continuation.identity.map_revision = request->map_revision;
    if (continuation.queue == task_server::WorkflowQueue::DEPTH_INTEGRATION) {
      AutonomousDepthResult result;
      result.work_item = continuation;
      result.command_type = request->command_type;
      result.result_status = request->result_status;
      result.observations = request->depth_observations;
      for (auto & observation : result.observations) {
        LimitDepthObservation(&observation);
      }
      autonomous_depth_results_[request->command_id] = std::move(result);
    }
    if (!workflow_scheduler_.Enqueue(continuation)) {
      autonomous_depth_results_.erase(request->command_id);
      response->reason = "workflow_continuation_already_pending";
      return;
    }
    if (!workflow_scheduler_.MarkResultAccepted(request->command_id)) {
      autonomous_depth_results_.erase(request->command_id);
      response->reason = "result_deduplication_race";
      return;
    }
    response->accepted = true;
    response->reason = "accepted_queued";
    RCLCPP_INFO(
      get_logger(),
      "[F6D-AUTONOMOUS-RESULT-ACCEPTED] drone=%u command=%s workflow=%s queue=%u depth=%zu status=%u fiducial_seen=%s",
      request->drone_id, request->command_id.c_str(), request->workflow_id.c_str(),
      static_cast<unsigned int>(continuation.queue), request->depth_observations.size(),
      request->result_status, request->fiducial_seen ? "true" : "false");
  }

  void HandleSparseCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
  {
    if (!cloud || cloud->is_bigendian || cloud->point_step == 0U || sparse_bootstrap_received_) {
      return;
    }
    std::vector<task_lib::SparseEvidence> evidence;
    const auto count = cloud->data.size() / cloud->point_step;
    evidence.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
      const std::size_t offset = index * cloud->point_step;
      float x = 0.0F;
      float y = 0.0F;
      float z = 0.0F;
      float score = 1.0F;
      std::uint32_t drone_id = 0U;
      std::uint32_t epoch_low = 0U;
      std::uint32_t epoch_high = 0U;
      std::uint32_t mp_low = 0U;
      std::uint32_t mp_high = 0U;
      if (!ReadCloudField(*cloud, offset, "x", sensor_msgs::msg::PointField::FLOAT32, &x) ||
        !ReadCloudField(*cloud, offset, "y", sensor_msgs::msg::PointField::FLOAT32, &y) ||
        !ReadCloudField(*cloud, offset, "z", sensor_msgs::msg::PointField::FLOAT32, &z))
      {
        return;
      }
      ReadCloudField(*cloud, offset, "score", sensor_msgs::msg::PointField::FLOAT32, &score);
      ReadCloudField(*cloud, offset, "drone_id", sensor_msgs::msg::PointField::UINT32, &drone_id);
      ReadCloudField(
        *cloud, offset, "map_epoch_low", sensor_msgs::msg::PointField::UINT32,
        &epoch_low);
      ReadCloudField(
        *cloud, offset, "map_epoch_high", sensor_msgs::msg::PointField::UINT32,
        &epoch_high);
      ReadCloudField(
        *cloud, offset, "local_mp_id_low", sensor_msgs::msg::PointField::UINT32,
        &mp_low);
      ReadCloudField(
        *cloud, offset, "local_mp_id_high", sensor_msgs::msg::PointField::UINT32,
        &mp_high);
      const auto epoch = (static_cast<std::uint64_t>(epoch_high) << 32U) | epoch_low;
      const auto map_point = (static_cast<std::uint64_t>(mp_high) << 32U) | mp_low;
      evidence.push_back(
        {std::to_string(drone_id) + ":" + std::to_string(epoch) + ":" +
          std::to_string(map_point), {x, y, z}, score});
    }
    pending_sparse_snapshot_ = std::move(evidence);
    sparse_bootstrap_received_ = true;
  }

  void HandleSparseDelta(mission_msgs::msg::GlobalSparseMapDelta::ConstSharedPtr delta)
  {
    if (!delta) {
      return;
    }
    for (const auto & point : delta->upserts) {
      pending_sparse_upserts_[std::to_string(point.drone_id) + ":" +
        std::to_string(point.map_epoch) + ":" + std::to_string(point.local_mappoint_id)] =
        task_lib::SparseEvidence{
        std::to_string(point.drone_id) + ":" + std::to_string(point.map_epoch) + ":" +
        std::to_string(point.local_mappoint_id), {point.x, point.y, point.z}, point.score};
      pending_sparse_deletes_.erase(
        std::to_string(point.drone_id) + ":" +
        std::to_string(point.map_epoch) + ":" + std::to_string(point.local_mappoint_id));
    }
    for (const auto & point : delta->deletes) {
      const auto source = std::to_string(point.drone_id) + ":" +
        std::to_string(point.map_epoch) + ":" + std::to_string(point.local_mappoint_id);
      pending_sparse_upserts_.erase(source);
      pending_sparse_deletes_.insert(source);
    }
  }

  void HandleKeyframeSparseEvidenceDelta(
    mission_msgs::msg::KeyframeSparseEvidenceDelta::ConstSharedPtr delta)
  {
    if (!delta || !keyframe_evidence_worker_) {
      return;
    }
    const auto stats = keyframe_evidence_worker_->Consume(*delta, &evidence_database_);
    if (stats.accepted_keyframes != 0U || stats.tombstones != 0U) {
      RCLCPP_INFO(
        get_logger(),
        "[F6E-KF-EVIDENCE-WRITTEN] upserts=%zu tombstones=%zu direct=%zu planes=%zu sources=%zu",
        stats.accepted_keyframes, stats.tombstones, stats.direct_sources,
        stats.plane_sources, evidence_database_.SourceCount());
    }
  }

  std::string DepthObservationFailure(
    const mission_msgs::msg::DenseKFObservation & observation) const
  {
    if (!depth_evidence_enabled_) {
      return "depth_evidence_disabled";
    }
    if (!observation.valid || observation.drone_id == 0U) {
      return observation.failure_reason.empty() ? "invalid_observation" : observation.failure_reason;
    }
    const std::size_t total_points =
      observation.points_k.size() + observation.far_free_points_k.size();
    if (total_points == 0U) {
      return "empty_points";
    }
    if (total_points < depth_min_support_points_) {
      return "insufficient_depth_support";
    }
    if (!std::isfinite(observation.confidence) || observation.confidence < depth_min_confidence_) {
      return "confidence_below_minimum";
    }
    if (!std::isfinite(observation.min_depth_m) || !std::isfinite(observation.max_depth_m) ||
      observation.min_depth_m <= 0.0F || observation.max_depth_m < observation.min_depth_m)
    {
      return "invalid_depth_range";
    }
    const KeyframeIdentity identity{
      observation.drone_id, observation.map_epoch, observation.local_keyframe_id};
    const auto evidence = depth_keyframe_evidence_.find(identity);
    if (evidence != depth_keyframe_evidence_.end()) {
      const auto stored = evidence->second.observations.find(observation.tracking_frame_id);
      if (stored != evidence->second.observations.end() &&
        stored->second.source_revision > observation.source_revision)
      {
        return "stale_source_revision";
      }
    }
    return {};
  }

  void LimitDepthObservation(mission_msgs::msg::DenseKFObservation * observation)
  {
    if (observation == nullptr) {
      return;
    }
    const auto near_before = observation->points_k.size();
    const auto far_before = observation->far_free_points_k.size();
    LimitDepthPoints(&observation->points_k, depth_max_points_per_observation_);
    LimitDepthPoints(&observation->far_free_points_k, depth_max_points_per_observation_);
    if (near_before != observation->points_k.size() ||
      far_before != observation->far_free_points_k.size())
    {
      RCLCPP_INFO(
        get_logger(),
        "[F6N-DEPTH-SAMPLED] frame=%lu near=%zu->%zu far_free=%zu->%zu max_per_product=%zu",
        observation->tracking_frame_id, near_before, observation->points_k.size(), far_before,
        observation->far_free_points_k.size(), depth_max_points_per_observation_);
    }
  }

  void QueueDepthObservation(const mission_msgs::msg::DenseKFObservation & observation)
  {
    const KeyframeIdentity identity{
      observation.drone_id, observation.map_epoch, observation.local_keyframe_id};
    auto [evidence, inserted] = depth_keyframe_evidence_.try_emplace(identity);
    (void)inserted;
    auto & stored = evidence->second.observations[observation.tracking_frame_id];
    stored = observation;
    RCLCPP_INFO(
      get_logger(),
      "[F6N-DEPTH-QUEUED] kf=%s frame=%lu revision=%lu near=%zu far_free=%zu confidence=%.3f pose_available=%s",
      KeyframeToken(identity).c_str(), observation.tracking_frame_id,
      observation.source_revision, observation.points_k.size(),
      observation.far_free_points_k.size(),
      observation.confidence,
      evidence->second.pose_available ? "true" : "false");
  }

  static KeyframeIdentity DepthObservationIdentity(
    const mission_msgs::msg::DenseKFObservation & observation)
  {
    return KeyframeIdentity{
      observation.drone_id, observation.map_epoch, observation.local_keyframe_id};
  }

  void EnqueueDepthIntegration(
    const std::string & task_id, std::uint32_t drone_id,
    const mission_msgs::msg::DenseKFObservation & facade_observation,
    const mission_msgs::msg::DenseKFObservation & target_observation)
  {
    depth_integrations_.erase(
      std::remove_if(
        depth_integrations_.begin(), depth_integrations_.end(),
        [&task_id](const PendingDepthIntegration & pending) {
          return pending.task_id == task_id;
        }),
      depth_integrations_.end());
    depth_integrations_.push_back(
      PendingDepthIntegration{task_id, drone_id, facade_observation, target_observation});
    RCLCPP_INFO(
      get_logger(),
      "[F6N-DEPTH-TASK-QUEUED] task=%s drone=%u facade=%s:%lu target=%s:%lu pending=%zu",
      task_id.c_str(), drone_id,
      KeyframeToken(DepthObservationIdentity(facade_observation)).c_str(),
      facade_observation.tracking_frame_id,
      KeyframeToken(DepthObservationIdentity(target_observation)).c_str(),
      target_observation.tracking_frame_id, depth_integrations_.size());
    RequestPendingDepthIntegrationPoses();
  }

  void CompleteDepthIntegration(const std::string & task_id, const char * reason)
  {
    const auto before = depth_integrations_.size();
    depth_integrations_.erase(
      std::remove_if(
        depth_integrations_.begin(), depth_integrations_.end(),
        [&task_id](const PendingDepthIntegration & pending) {
          return pending.task_id == task_id;
        }),
      depth_integrations_.end());
    if (depth_integrations_.size() != before) {
      RCLCPP_INFO(
        get_logger(), "[F6N-DEPTH-TASK-COMPLETE] task=%s reason=%s pending=%zu",
        task_id.c_str(), reason, depth_integrations_.size());
    }
  }

  bool ApplyDepthEvidence(const KeyframeIdentity & identity, DepthKeyframeEvidence * evidence)
  {
    if (!evidence || !evidence->pose_available || !depth_evidence_enabled_) {
      return false;
    }
    bool changed = false;
    for (const auto & observation_entry : evidence->observations) {
      const auto frame_id = observation_entry.first;
      const auto & observation = observation_entry.second;
      if (evidence->applied_pose_revisions[frame_id] == evidence->pose_revision &&
        evidence->applied_source_revisions[frame_id] == observation.source_revision)
      {
        continue;
      }
      const auto camera_k = task_lib::Vec3{
        observation.k_t_camera.position.x, observation.k_t_camera.position.y,
        observation.k_t_camera.position.z};
      const auto camera_world = TransformPoint(evidence->w_t_keyframe, camera_k);
      std::set<task_lib::VoxelKey> free_cells;
      std::set<task_lib::VoxelKey> direct_free_cells;
      std::set<task_lib::VoxelKey> occupied_cells;
      std::size_t accepted_rays = 0U;
      std::size_t direct_surface_rays = 0U;
      const auto normal_k = Normalize(
        task_lib::Vec3{
        observation.normal_k.x, observation.normal_k.y, observation.normal_k.z});
      const double max_incidence_cos = std::cos(
        depth_direct_surface_max_incidence_deg_ * M_PI / 180.0);
      const auto append_free_ray = [
        &free_cells, &camera_k, &camera_world, &observation, evidence, this](
        const geometry_msgs::msg::Point32 & point, bool truncate_far) -> bool
        {
          const task_lib::Vec3 endpoint_k{point.x, point.y, point.z};
          if (!IsFinite(endpoint_k)) {
            return false;
          }
          const auto delta_k = Subtract(endpoint_k, camera_k);
          const double distance = Norm(delta_k);
          if (distance < observation.min_depth_m || distance > observation.max_depth_m) {
            return false;
          }
          const task_lib::Vec3 bounded_endpoint_k = truncate_far ?
            Add(camera_k, Scale(delta_k, depth_far_free_distance_m_ / distance)) : endpoint_k;
          const auto endpoint_world = TransformPoint(evidence->w_t_keyframe, bounded_endpoint_k);
          const auto ray = VoxelRaySupercover(camera_world, endpoint_world, voxel_size_, false);
          free_cells.insert(ray.begin(), ray.end());
          return true;
        };
      for (const auto & point : observation.points_k) {
        const task_lib::Vec3 endpoint_k{point.x, point.y, point.z};
        const double distance = Norm(Subtract(endpoint_k, camera_k));
        if (!IsFinite(endpoint_k) || distance < observation.min_depth_m ||
          distance > depth_far_free_distance_m_)
        {
          continue;
        }
        const auto ray_direction = Normalize(Subtract(endpoint_k, camera_k));
        const bool direct_surface = observation.normal_valid && normal_k.has_value() &&
          ray_direction.has_value() &&
          std::abs(Dot(*normal_k, *ray_direction)) >= max_incidence_cos;
        if (direct_surface) {
          const auto endpoint_world = TransformPoint(evidence->w_t_keyframe, endpoint_k);
          const auto ray = VoxelRaySupercover(camera_world, endpoint_world, voxel_size_, false);
          direct_free_cells.insert(ray.begin(), ray.end());
          occupied_cells.insert(VoxelKeyFromWorld(ToPoint(endpoint_world), voxel_size_));
          ++direct_surface_rays;
        }
        if (append_free_ray(point, false)) {
          ++accepted_rays;
        }
      }
      for (const auto & point : observation.far_free_points_k) {
        const task_lib::Vec3 endpoint_k{point.x, point.y, point.z};
        const double distance = Norm(Subtract(endpoint_k, camera_k));
        if (!IsFinite(endpoint_k) || distance <= depth_far_free_distance_m_ ||
          distance > observation.max_depth_m)
        {
          continue;
        }
        if (append_free_ray(point, true)) {
          ++accepted_rays;
        }
      }
      FillSingleCellDepthGaps(&free_cells);
      FillSingleCellDepthGaps(&direct_free_cells);
      const std::string source_id = "depth_free:" + KeyframeToken(identity) + ":" +
        std::to_string(frame_id);
      const std::string direct_free_source_id = "depth_direct_free:" +
        KeyframeToken(identity) + ":" + std::to_string(frame_id);
      const std::string occupied_source_id = "depth_occupied:" +
        KeyframeToken(identity) + ":" + std::to_string(frame_id);
      changed = voxel_map_->ReplaceDepthFreeCells(source_id, free_cells) || changed;
      if (direct_free_cells.empty()) {
        changed = voxel_map_->RemoveEvidence(direct_free_source_id) || changed;
      } else {
        changed = voxel_map_->ReplaceDirectDepthFreeCells(
          direct_free_source_id, direct_free_cells) || changed;
      }
      if (occupied_cells.empty()) {
        changed = voxel_map_->RemoveEvidence(occupied_source_id) || changed;
      } else {
        changed = voxel_map_->ReplaceDepthOccupiedCells(
          occupied_source_id, occupied_cells) || changed;
      }
      evidence->source_ids[frame_id] = source_id;
      evidence->applied_pose_revisions[frame_id] = evidence->pose_revision;
      evidence->applied_source_revisions[frame_id] = observation.source_revision;
      RCLCPP_INFO(
        get_logger(),
        "[F6N-DEPTH-INTEGRATED] kf=%s frame=%lu pose_revision=%lu rays=%zu free_cells=%zu direct_rays=%zu direct_free=%zu occupied=%zu",
        KeyframeToken(identity).c_str(), frame_id, evidence->pose_revision, accepted_rays,
        free_cells.size(), direct_surface_rays, direct_free_cells.size(), occupied_cells.size());
    }
    return changed;
  }

  void RequeueDepthFallback(
    const task_server::WorkflowWorkItem & item, const char * reason)
  {
    autonomous_workflow_contexts_.erase(item.identity.workflow_id);
    auto continuation = item;
    continuation.queue = task_server::WorkflowQueue::POINT_SELECTION;
    if (workflow_scheduler_.Enqueue(continuation)) {
      RCLCPP_INFO(
        get_logger(),
        "[F6F-DEPTH-FALLBACK-QUEUED] drone=%u command=%s workflow=%s reason=%s",
        item.identity.drone_id, item.identity.command_id.c_str(),
        item.identity.workflow_id.c_str(), reason);
    }
  }

  void RequeueAutonomousWorkflowFailure(
    const task_server::WorkflowWorkItem & source, task_server::WorkflowQueue failure_queue)
  {
    auto retry = source;
    retry.queue = failure_queue;
    workflow_scheduler_.Requeue(retry);
  }

  bool DispatchAcceptedAutonomousWorkflowCommand(
    const task_server::WorkflowWorkItem & source, std::uint8_t command_type,
    const geometry_msgs::msg::Point & visual_target,
    const mission_msgs::msg::TrajectoryPlan & trajectory_plan, bool capture_after_command,
    task_server::WorkflowQueue failure_queue, const std::string & command_id)
  {
    const auto client = autonomous_command_clients_.find(source.identity.drone_id);
    if (client == autonomous_command_clients_.end() || !client->second ||
      !client->second->service_is_ready())
    {
      workflow_scheduler_.CompleteNormalCommand(source.identity.drone_id, command_id);
      ReleaseAutonomousTrajectoryMonitor(
        source.identity.drone_id, command_id, "autonomous_service_unavailable",
        mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_REJECTED);
      RequeueAutonomousWorkflowFailure(source, failure_queue);
      return false;
    }
    auto request = std::make_shared<SubmitAutonomousCommand::Request>();
    request->mission_id = mission_id_;
    request->drone_id = source.identity.drone_id;
    request->task_id = source.identity.task_id;
    request->workflow_id = source.identity.workflow_id;
    request->command_id = command_id;
    request->map_epoch = source.identity.map_epoch;
    request->map_revision = voxel_map_->revision();
    request->command_type = command_type;
    request->visual_target = visual_target;
    request->trajectory_plan = trajectory_plan;
    request->capture_after_command = capture_after_command;
    client->second->async_send_request(
      request,
      [this, source, command_id, command_type, trajectory_id = trajectory_plan.trajectory_id,
      failure_queue](
        rclcpp::Client<SubmitAutonomousCommand>::SharedFuture future) {
        try {
          const auto response = future.get();
          if (response && response->accepted) {
            const auto runtime = execution_runtime_by_drone_.find(source.identity.drone_id);
            if (!trajectory_id.empty() && runtime != execution_runtime_by_drone_.end() &&
            runtime->second.autonomous_workflow &&
            runtime->second.autonomous_command_id == command_id)
            {
              const auto plan = execution_plans_.find(trajectory_id);
              if (plan != execution_plans_.end()) {
                plan->second.execution_state =
                mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_ACTIVE;
                plan->second.execution_detail = "Trayectoria autonoma aceptada por task_manager";
                planned_route_publisher_->publish(plan->second);
              }
            }
            RCLCPP_INFO(
              get_logger(),
              "[F6H-I-AUTONOMOUS-DISPATCHED] drone=%u task=%s workflow=%s command=%s type=%u",
              source.identity.drone_id, source.identity.task_id.c_str(),
              source.identity.workflow_id.c_str(), command_id.c_str(),
              static_cast<unsigned int>(command_type));
            return;
          }
        } catch (const std::exception &) {
        }
        workflow_scheduler_.CompleteNormalCommand(source.identity.drone_id, command_id);
        ReleaseAutonomousTrajectoryMonitor(
          source.identity.drone_id, command_id, "autonomous_command_rejected",
          mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_REJECTED);
        RequeueAutonomousWorkflowFailure(source, failure_queue);
      });
    return true;
  }

  bool QueueAutonomousTrajectoryMonitor(
    const task_server::WorkflowWorkItem & source, std::uint8_t command_type,
    const geometry_msgs::msg::Point & visual_target,
    const mission_msgs::msg::TrajectoryPlan & trajectory_plan, bool capture_after_command,
    task_server::WorkflowQueue failure_queue)
  {
    if (trajectory_plan.waypoints.size() < 2U) {
      RequeueAutonomousWorkflowFailure(source, failure_queue);
      return false;
    }
    PendingAutonomousTrajectoryMonitor pending;
    pending.source = source;
    pending.command_type = command_type;
    pending.visual_target = visual_target;
    pending.plan = trajectory_plan;
    pending.capture_after_command = capture_after_command;
    pending.failure_queue = failure_queue;
    const auto inserted = pending_autonomous_trajectory_monitors_.emplace(
      source.identity.workflow_id, std::move(pending));
    if (!inserted.second) {
      return false;
    }
    auto monitor = source;
    monitor.queue = task_server::WorkflowQueue::ACTIVE_TRAJECTORY_MONITOR;
    if (!workflow_scheduler_.Enqueue(monitor)) {
      pending_autonomous_trajectory_monitors_.erase(source.identity.workflow_id);
      RequeueAutonomousWorkflowFailure(source, failure_queue);
      return false;
    }
    RCLCPP_INFO(
      get_logger(),
      "[F6J-AUTONOMOUS-MONITOR-QUEUED] drone=%u task=%s workflow=%s trajectory_id=%s",
      source.identity.drone_id, source.identity.task_id.c_str(),
      source.identity.workflow_id.c_str(), trajectory_plan.trajectory_id.c_str());
    return true;
  }

  bool ActivateAutonomousTrajectoryMonitor(
    const PendingAutonomousTrajectoryMonitor & pending, const std::string & command_id)
  {
    const auto & plan = pending.plan;
    const auto body = drone_reservation_body_cells_.find(plan.drone_id);
    const auto corridor = SegmentCorridor(
      plan, voxel_size_, body == drone_reservation_body_cells_.end() ? task_lib::VoxelKey{} :
      body->second, reservation_sweep_sample_step_voxels_);
    const std::string reservation_id = "reservation_" + plan.trajectory_id;
    auto & runtime = execution_runtime_by_drone_[plan.drone_id];
    const bool committed = runtime.reservation_id.empty() ?
      reservation_overlay_.Commit(
      {reservation_id, plan.drone_id, task_lib::ReservationMode::Moving, corridor, 0U}) :
      reservation_overlay_.Replace(
      runtime.reservation_id,
      {reservation_id, plan.drone_id, task_lib::ReservationMode::Moving, corridor, 0U});
    if (!committed) {
      return false;
    }
    runtime.task_id = plan.task_id;
    runtime.trajectory_id = plan.trajectory_id;
    runtime.reservation_id = reservation_id;
    runtime.active_plan = plan;
    runtime.target = VoxelKeyFromWorld(plan.waypoints.back().position_world, voxel_size_);
    runtime.corridor = corridor;
    runtime.in_flight = true;
    runtime.stop_requested = false;
    runtime.corridor_affected = false;
    runtime.require_known_free = true;
    runtime.autonomous_workflow = true;
    runtime.autonomous_command_id = command_id;
    runtime.autonomous_workflow_id = pending.source.identity.workflow_id;
    auto published_plan = plan;
    published_plan.execution_state = mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_PLANNED;
    published_plan.execution_detail = "Trayectoria autonoma reservada; esperando aceptacion";
    execution_plans_[plan.trajectory_id] = published_plan;
    PublishVoxelMap();
    planned_route_publisher_->publish(published_plan);
    RCLCPP_INFO(
      get_logger(),
      "[F6J-AUTONOMOUS-RESERVATION-COMMIT] drone=%u command=%s trajectory_id=%s cells=%zu",
      plan.drone_id, command_id.c_str(), plan.trajectory_id.c_str(), corridor.size());
    return true;
  }

  void RunActiveTrajectoryMonitorWorker()
  {
    const auto item = workflow_scheduler_.Dequeue(
      task_server::WorkflowQueue::ACTIVE_TRAJECTORY_MONITOR);
    if (!item.has_value()) {
      return;
    }
    const auto pending = pending_autonomous_trajectory_monitors_.find(item->identity.workflow_id);
    if (pending == pending_autonomous_trajectory_monitors_.end()) {
      return;
    }
    const auto client = autonomous_command_clients_.find(item->identity.drone_id);
    if (client == autonomous_command_clients_.end() || !client->second ||
      !client->second->service_is_ready())
    {
      workflow_scheduler_.Requeue(*item);
      return;
    }
    const std::string command_id =
      "autonomous:" + item->identity.workflow_id + ":" +
      std::to_string(++autonomous_command_revision_);
    if (!workflow_scheduler_.AcceptNormalCommand(item->identity.drone_id, command_id).has_value()) {
      workflow_scheduler_.Requeue(*item);
      return;
    }
    const auto monitor = pending->second;
    if (!ActivateAutonomousTrajectoryMonitor(monitor, command_id)) {
      workflow_scheduler_.CompleteNormalCommand(item->identity.drone_id, command_id);
      pending_autonomous_trajectory_monitors_.erase(pending);
      RequeueAutonomousWorkflowFailure(monitor.source, monitor.failure_queue);
      return;
    }
    pending_autonomous_trajectory_monitors_.erase(pending);
    DispatchAcceptedAutonomousWorkflowCommand(
      monitor.source, monitor.command_type, monitor.visual_target, monitor.plan,
      monitor.capture_after_command, monitor.failure_queue, command_id);
  }

  bool DispatchAutonomousWorkflowCommand(
    const task_server::WorkflowWorkItem & source, std::uint8_t command_type,
    const geometry_msgs::msg::Point & visual_target,
    const mission_msgs::msg::TrajectoryPlan & trajectory_plan, bool capture_after_command,
    task_server::WorkflowQueue failure_queue)
  {
    if (command_type == SubmitAutonomousCommand::Request::COMMAND_MOVE_AND_CAPTURE) {
      return QueueAutonomousTrajectoryMonitor(
        source, command_type, visual_target, trajectory_plan, capture_after_command,
        failure_queue);
    }
    const auto client = autonomous_command_clients_.find(source.identity.drone_id);
    if (client == autonomous_command_clients_.end() || !client->second ||
      !client->second->service_is_ready())
    {
      RequeueAutonomousWorkflowFailure(source, failure_queue);
      return false;
    }
    const std::string command_id =
      "autonomous:" + source.identity.workflow_id + ":" +
      std::to_string(++autonomous_command_revision_);
    if (!workflow_scheduler_.AcceptNormalCommand(
        source.identity.drone_id, command_id).has_value())
    {
      RequeueAutonomousWorkflowFailure(source, failure_queue);
      return false;
    }
    return DispatchAcceptedAutonomousWorkflowCommand(
      source, command_type, visual_target, trajectory_plan, capture_after_command,
      failure_queue, command_id);
  }

  void RunPointSelectionWorker()
  {
    if (!execution_enabled_.load(std::memory_order_acquire)) {
      return;
    }
    const auto item = workflow_scheduler_.Dequeue(task_server::WorkflowQueue::POINT_SELECTION);
    if (!item.has_value()) {
      return;
    }
    const auto task = std::find_if(
      tasks_.begin(), tasks_.end(), [&item](const auto & candidate) {
        return candidate.task_id == item->identity.task_id &&
        candidate.assigned_drone_id == item->identity.drone_id;
      });
    const auto pose = navigation_poses_.find(item->identity.drone_id);
    if (task == tasks_.end() || pose == navigation_poses_.end()) {
      return;
    }
    auto * runtime = FacadeRuntimeFor(*task);
    if (runtime == nullptr) {
      return;
    }
    const bool coverage_complete = !runtime->active_coverage_sections.empty() &&
      std::all_of(
      runtime->active_coverage_sections.begin(), runtime->active_coverage_sections.end(),
      [](bool active) {return active;});
    if (coverage_complete) {
      const auto drone_id = task->assigned_drone_id;
      task->state = mission_msgs::msg::TaskState::COMPLETED;
      task->assigned_drone_id = 0U;
      SyncFacadeTaskState(*task, *runtime, "Coverage U de fachada completado");
      autonomous_workflow_contexts_.erase(item->identity.workflow_id);
      EnqueueTaskAvailableDrone(drone_id, "coverage_completed");
      AssignAvailableTasks();
      return;
    }
    const auto open_side = runtime->coverage_plan.open_side;
    const double open_coordinate =
      open_side == task_lib::FacadeCoverageSide::MinX ? runtime->coverage_plan.bounds.min.x :
      open_side == task_lib::FacadeCoverageSide::MaxX ? runtime->coverage_plan.bounds.max.x :
      open_side == task_lib::FacadeCoverageSide::MinY ? runtime->coverage_plan.bounds.min.y :
      runtime->coverage_plan.bounds.max.y;
    const double pose_coordinate =
      open_side == task_lib::FacadeCoverageSide::MinX ||
      open_side ==
      task_lib::FacadeCoverageSide::MaxX ? pose->second.position.x : pose->second.position.y;
    if (std::abs(pose_coordinate - open_coordinate) <= voxel_size_) {
      const auto drone_id = task->assigned_drone_id;
      task->state = mission_msgs::msg::TaskState::TO_FINISH;
      task->assigned_drone_id = 0U;
      SyncFacadeTaskState(
        *task, *runtime,
        "A terminar: cara abierta alcanzada con secciones U pendientes");
      autonomous_workflow_contexts_.erase(item->identity.workflow_id);
      EnqueueTaskAvailableDrone(drone_id, "coverage_open_side_reached");
      AssignAvailableTasks();
      return;
    }
    const auto candidate = SelectFacadeTaskCandidate(*task, *runtime, pose->second);
    if (!candidate.has_value()) {
      const auto drone_id = task->assigned_drone_id;
      task->state = mission_msgs::msg::TaskState::TO_FINISH;
      task->assigned_drone_id = 0U;
      SyncFacadeTaskState(*task, *runtime, "A terminar: sin seccion U inspeccionable");
      autonomous_workflow_contexts_.erase(item->identity.workflow_id);
      EnqueueTaskAvailableDrone(drone_id, "no_visual_candidate");
      AssignAvailableTasks();
      return;
    }
    AutonomousWorkflowContext next;
    next.candidate = *candidate;
    next.segment_start_ratio =
      task_lib::ProjectToFacadeRatio(runtime->facade, pose->second.position);
    next.has_candidate = true;
    autonomous_workflow_contexts_[item->identity.workflow_id] = next;
    const auto target_key = VoxelKeyFromWorld(ToPoint(candidate->target), voxel_size_);
    const auto target_state = voxel_map_->StateAt(target_key);
    const auto profile = drone_facade_navigation_profiles_.find(item->identity.drone_id);
    const auto body = drone_reservation_body_cells_.find(item->identity.drone_id);
    const auto navigation_snapshot = profile == drone_facade_navigation_profiles_.end() ?
      task_lib::NavigationSnapshot{} : NavigationSnapshotFor(
      item->identity.drone_id, profile->second,
      body == drone_reservation_body_cells_.end() ? task_lib::VoxelKey{} : body->second,
      true, &pose->second.position);
    const auto target_navigation = std::find_if(
      navigation_snapshot.cells.begin(), navigation_snapshot.cells.end(),
      [&target_key](const auto & cell) {return cell.key == target_key;});
    const bool target_traversable = target_navigation != navigation_snapshot.cells.end() &&
      target_navigation->traversable;
    if (target_state == task_lib::VoxelState::Occupied ||
      (target_state == task_lib::VoxelState::Free && !target_traversable))
    {
      runtime->rejected_inspection_target_ratios.push_back(candidate->target_ratio);
      autonomous_workflow_contexts_.erase(item->identity.workflow_id);
      workflow_scheduler_.Requeue(*item);
      return;
    }
    task->state = mission_msgs::msg::TaskState::RUNNING;
    SyncFacadeTaskState(
      *task, *runtime, target_state == task_lib::VoxelState::Free ?
      "Pose de inspeccion FREE; encola D*" : "Pose de inspeccion UNKNOWN; solicita mirada depth");
    if (target_state == task_lib::VoxelState::Free) {
      auto planning = *item;
      planning.queue = task_server::WorkflowQueue::TRAJECTORY_PLANNING;
      workflow_scheduler_.Enqueue(planning);
      return;
    }
    mission_msgs::msg::TrajectoryPlan empty_plan;
    DispatchAutonomousWorkflowCommand(
      *item, SubmitAutonomousCommand::Request::COMMAND_LOOK_AND_CAPTURE,
      ToPoint(candidate->visual_target), empty_plan, true,
      task_server::WorkflowQueue::POINT_SELECTION);
  }

  void RunTrajectoryPlanningWorker()
  {
    if (!execution_enabled_.load(std::memory_order_acquire)) {
      return;
    }
    const auto item = workflow_scheduler_.Dequeue(task_server::WorkflowQueue::TRAJECTORY_PLANNING);
    if (!item.has_value()) {
      return;
    }
    const auto context = autonomous_workflow_contexts_.find(item->identity.workflow_id);
    const auto task = std::find_if(
      tasks_.begin(), tasks_.end(), [&item](const auto & candidate) {
        return candidate.task_id == item->identity.task_id &&
        candidate.assigned_drone_id == item->identity.drone_id;
      });
    const auto pose = navigation_poses_.find(item->identity.drone_id);
    if (context == autonomous_workflow_contexts_.end() || task == tasks_.end() ||
      pose == navigation_poses_.end())
    {
      return;
    }
    auto * runtime = FacadeRuntimeFor(*task);
    std::string reason;
    auto * planner = PlannerFor(item->identity.drone_id, &reason, true, &pose->second.position);
    if (runtime == nullptr || planner == nullptr) {
      workflow_scheduler_.Requeue(*item);
      return;
    }
    auto route = planner->Plan(pose->second.position, context->second.candidate.target);
    bool free_prefix = false;
    if (!route.success) {
      LogFacadeStrictRouteFailure(
        item->identity.drone_id, pose->second.position, context->second.candidate.target, route);
      std::string exploratory_reason;
      auto * exploratory = PlannerFor(
        item->identity.drone_id, &exploratory_reason, false, &pose->second.position);
      const auto exploratory_route = exploratory == nullptr ? task_lib::DStarLiteResult{} :
      exploratory->Plan(pose->second.position, context->second.candidate.target);
      std::optional<task_lib::DStarLiteResult> prefix;
      for (auto voxel = exploratory_route.corridor.rbegin();
        exploratory_route.success && voxel != exploratory_route.corridor.rend(); ++voxel)
      {
        if (voxel_map_->StateAt(*voxel) != task_lib::VoxelState::Free) {
          continue;
        }
        const auto prefix_target = VoxelCenter(*voxel, voxel_size_);
        if (Norm(Subtract(prefix_target, pose->second.position)) < voxel_size_) {
          continue;
        }
        const auto candidate_route = planner->Plan(pose->second.position, prefix_target);
        if (candidate_route.success) {
          prefix = candidate_route;
          break;
        }
      }
      if (!prefix.has_value()) {
        autonomous_workflow_contexts_.erase(item->identity.workflow_id);
        auto selection = *item;
        selection.queue = task_server::WorkflowQueue::POINT_SELECTION;
        workflow_scheduler_.Enqueue(selection);
        return;
      }
      route = std::move(*prefix);
      free_prefix = true;
    }
    navigation_orientations_[item->identity.drone_id] = NavigationOrientation{
      context->second.candidate.observation_yaw_rad, 0.0, pose->second.map_epoch, true};
    const auto plan = BuildExecutablePlan(
      route, item->identity.task_id, item->identity.drone_id, pose->second,
      execution_nominal_velocity_mps_, true, &pose->second.position);
    const bool view_advance = free_prefix || context->second.fallback_free_advance;
    context->second.move_to_wall = !view_advance;
    context->second.view_advance_capture = view_advance;
    DispatchAutonomousWorkflowCommand(
      *item, SubmitAutonomousCommand::Request::COMMAND_MOVE_AND_CAPTURE,
      ToPoint(context->second.candidate.visual_target), plan,
      true,
      task_server::WorkflowQueue::TRAJECTORY_PLANNING);
    RCLCPP_INFO(
      get_logger(),
      "[F6I-WORKFLOW-PLAN] drone=%u task=%s workflow=%s free_prefix=%s waypoints=%zu",
      item->identity.drone_id, item->identity.task_id.c_str(),
      item->identity.workflow_id.c_str(), free_prefix ? "true" : "false", plan.waypoints.size());
  }

  void RunDepthIntegrationWorker()
  {
    const auto item = workflow_scheduler_.Dequeue(task_server::WorkflowQueue::DEPTH_INTEGRATION);
    if (!item.has_value()) {
      return;
    }
    const auto pending = autonomous_depth_results_.find(item->identity.command_id);
    if (pending == autonomous_depth_results_.end()) {
      RCLCPP_WARN(
        get_logger(), "[F6F-DEPTH-JOB-MISSING] drone=%u command=%s",
        item->identity.drone_id, item->identity.command_id.c_str());
      return;
    }
    const AutonomousDepthResult result = std::move(pending->second);
    autonomous_depth_results_.erase(pending);
    const bool terminal_allows_depth =
      result.result_status == ReportAutonomousResult::Request::RESULT_COMPLETED ||
      result.result_status == ReportAutonomousResult::Request::RESULT_ABORTED ||
      !result.observations.empty();
    if (!terminal_allows_depth) {
      RequeueDepthFallback(*item, "result_without_usable_depth");
      return;
    }
    if (result.result_status != ReportAutonomousResult::Request::RESULT_COMPLETED &&
      result.result_status != ReportAutonomousResult::Request::RESULT_ABORTED)
    {
      RCLCPP_INFO(
        get_logger(),
        "[F6F-DEPTH-TERMINAL-ACCEPTED] drone=%u command=%s status=%u observations=%zu",
        item->identity.drone_id, item->identity.command_id.c_str(), result.result_status,
        result.observations.size());
    }
    const auto context = autonomous_workflow_contexts_.find(item->identity.workflow_id);
    const bool view_advance = context != autonomous_workflow_contexts_.end() &&
      context->second.view_advance_capture;
    task_server::DepthIntegrationJob job;
    job.command_id = item->identity.command_id;
    if (result.command_type == SubmitAutonomousCommand::Request::COMMAND_MOVE_AND_CAPTURE) {
      job.inspection_kind = view_advance ? task_server::DepthInspectionKind::VIEW_ADVANCE :
        task_server::DepthInspectionKind::VIEW_WALL;
    } else {
      job.inspection_kind = task_server::DepthInspectionKind::VIEW_UNKNOWN;
    }
    job.observations = result.observations;
    const auto integrated = depth_integration_worker_->Consume(job, &evidence_database_);
    if (!integrated.accepted || integrated.sources.empty()) {
      RequeueDepthFallback(
        *item, integrated.reason.empty() ? "depth_sources_not_written" : integrated.reason.c_str());
      return;
    }
    PendingDepthContinuation continuation;
    continuation.work_item = *item;
    continuation.work_item.queue = job.inspection_kind ==
      task_server::DepthInspectionKind::VIEW_UNKNOWN ?
      task_server::WorkflowQueue::TRAJECTORY_PLANNING : task_server::WorkflowQueue::POINT_SELECTION;
    continuation.recheck_unknown_target =
      job.inspection_kind == task_server::DepthInspectionKind::VIEW_UNKNOWN;
    for (const auto & source : integrated.sources) {
      const auto source_id = task_server::VoxelMapBuilder::SourceId(
        source.identity, source.source_name);
      continuation.pending_source_ids.insert(source_id);
      if ((job.inspection_kind == task_server::DepthInspectionKind::VIEW_WALL ||
        job.inspection_kind == task_server::DepthInspectionKind::VIEW_ADVANCE) &&
        context != autonomous_workflow_contexts_.end() && context->second.has_candidate)
      {
        const bool depth_occupied = source.source_name.rfind("depth_occupied:", 0U) == 0U;
        coverage_source_contexts_[source_id] = CoverageSourceContext{
          item->identity.task_id, item->identity.drone_id, context->second.candidate.section_index,
          depth_occupied,
          job.inspection_kind == task_server::DepthInspectionKind::VIEW_WALL &&
          context->second.candidate.synthetic_visual_target};
        if (depth_occupied || context->second.candidate.synthetic_visual_target) {
          RCLCPP_INFO(
            get_logger(),
            "[F6H-COVERAGE-SOURCE-PENDING] drone=%u command=%s section=%zu kind=%s source=%s",
            item->identity.drone_id, item->identity.command_id.c_str(),
            context->second.candidate.section_index,
            context->second.candidate.synthetic_visual_target ? "SIN_FACHADA" :
            (job.inspection_kind == task_server::DepthInspectionKind::VIEW_ADVANCE ?
            "view_advance" : "vista_pared"), source_id.c_str());
        }
      }
      RequestKeyframePose(
        KeyframeIdentity{source.identity.drone_id, source.identity.map_epoch,
          source.identity.keyframe_id});
    }
    pending_depth_continuations_[item->identity.command_id] = std::move(continuation);
    RCLCPP_INFO(
      get_logger(),
      "[F6F-DEPTH-SOURCES-WRITTEN] drone=%u command=%s kind=%s sources=%zu",
      item->identity.drone_id, item->identity.command_id.c_str(),
      job.inspection_kind == task_server::DepthInspectionKind::VIEW_WALL ? "vista_pared" :
      job.inspection_kind == task_server::DepthInspectionKind::VIEW_ADVANCE ? "view_advance" :
      "vista_unknown", integrated.sources.size());
  }

  void ProcessAppliedDepthContinuations(const std::vector<std::string> & source_ids)
  {
    if (source_ids.empty()) {
      return;
    }
    const std::set<std::string> applied(source_ids.begin(), source_ids.end());
    for (auto pending = pending_depth_continuations_.begin();
      pending != pending_depth_continuations_.end(); )
    {
      for (const auto & source_id : applied) {
        pending->second.pending_source_ids.erase(source_id);
      }
      if (!pending->second.pending_source_ids.empty()) {
        ++pending;
        continue;
      }
      auto continuation = pending->second.work_item;
      const bool recheck_unknown_target = pending->second.recheck_unknown_target;
      const auto command_id = pending->first;
      pending = pending_depth_continuations_.erase(pending);
      if (recheck_unknown_target && !RequeueAppliedUnknownDepthContinuation(continuation)) {
        continue;
      }
      if (workflow_scheduler_.Enqueue(continuation)) {
        RCLCPP_INFO(
          get_logger(),
          "[F6F-DEPTH-SOURCES-APPLIED] drone=%u command=%s workflow=%s queue=%u",
          continuation.identity.drone_id, command_id.c_str(),
          continuation.identity.workflow_id.c_str(), static_cast<unsigned int>(continuation.queue));
      }
    }
  }

  std::set<std::size_t> CoverageNeighborClaims(
    const FacadeTaskRuntime & runtime, std::size_t section_index) const
  {
    std::set<std::size_t> claims;
    if (section_index >= runtime.coverage_plan.sections.size()) {
      return claims;
    }
    claims.insert(section_index);
    for (std::int64_t offset = 1; offset <= depth_coverage_neighbor_sections_; ++offset) {
      if (section_index >= static_cast<std::size_t>(offset)) {
        claims.insert(section_index - static_cast<std::size_t>(offset));
      }
      const auto after = section_index + static_cast<std::size_t>(offset);
      if (after < runtime.coverage_plan.sections.size()) {
        claims.insert(after);
      }
    }
    return claims;
  }

  bool RebuildDerivedCoverageSections(FacadeTaskRuntime * runtime) const
  {
    if (runtime == nullptr) {
      return false;
    }
    std::vector<bool> derived(runtime->coverage_plan.sections.size(), false);
    for (const auto & source_claims : runtime->coverage_claims_by_source) {
      for (const auto section : source_claims.second) {
        if (section < derived.size()) {
          derived[section] = true;
        }
      }
    }
    if (derived == runtime->active_coverage_sections) {
      return false;
    }
    runtime->active_coverage_sections = std::move(derived);
    return true;
  }

  void UpdateCoverageClaims(
    const std::string & source_id, const CoverageSourceContext & context,
    const std::set<std::size_t> & claims, const char * kind)
  {
    const auto task = std::find_if(
      tasks_.begin(), tasks_.end(), [&context](const auto & candidate) {
        return candidate.task_id == context.task_id;
      });
    if (task == tasks_.end()) {
      return;
    }
    auto * runtime = FacadeRuntimeFor(*task);
    if (runtime == nullptr) {
      return;
    }
    const auto previous = runtime->coverage_claims_by_source.find(source_id);
    if (claims.empty()) {
      if (previous == runtime->coverage_claims_by_source.end()) {
        return;
      }
      runtime->coverage_claims_by_source.erase(previous);
    } else if (previous != runtime->coverage_claims_by_source.end() &&
      previous->second == claims)
    {
      return;
    } else {
      runtime->coverage_claims_by_source[source_id] = claims;
    }
    const bool changed = RebuildDerivedCoverageSections(runtime);
    RCLCPP_INFO(
      get_logger(),
      "[F6H-COVERAGE-CLAIMS] task=%s drone=%u source=%s kind=%s claims=%zu active=%zu changed=%s",
      task->task_id.c_str(), task->assigned_drone_id, source_id.c_str(), kind, claims.size(),
      static_cast<std::size_t>(std::count(
        runtime->active_coverage_sections.begin(), runtime->active_coverage_sections.end(), true)),
      changed ? "true" : "false");
    if (changed) {
      SyncFacadeTaskState(*task, *runtime, "Coverage U derivado de evidencia depth materializada");
    }
  }

  void ApplyCoverageClaimsFromMap(const task_server::VoxelMaterializationStats & materialized)
  {
    const std::set<std::string> reapplied(
      materialized.applied_source_ids.begin(), materialized.applied_source_ids.end());
    for (const auto & source_id : materialized.removed_source_ids) {
      if (reapplied.count(source_id) != 0U) {
        continue;
      }
      const auto context = coverage_source_contexts_.find(source_id);
      if (context == coverage_source_contexts_.end()) {
        continue;
      }
      UpdateCoverageClaims(source_id, context->second, {}, "source_removed");
      coverage_source_contexts_.erase(context);
    }
    for (const auto & source_id : materialized.applied_source_ids) {
      const auto context = coverage_source_contexts_.find(source_id);
      if (context == coverage_source_contexts_.end()) {
        continue;
      }
      const auto task = std::find_if(
        tasks_.begin(), tasks_.end(), [&context](const auto & candidate) {
          return candidate.task_id == context->second.task_id;
        });
      if (task == tasks_.end()) {
        continue;
      }
      auto * runtime = FacadeRuntimeFor(*task);
      if (runtime == nullptr) {
        continue;
      }
      std::set<std::size_t> claims;
      const auto occupied = materialized.occupied_cells_by_source.find(source_id);
      if (context->second.synthetic_no_facade) {
        claims.insert(context->second.candidate_section_index);
      } else if (context->second.direct_depth_occupied &&
        occupied != materialized.occupied_cells_by_source.end())
      {
        const auto direct = CoverageNeighborClaims(
          *runtime, context->second.candidate_section_index);
        claims.insert(direct.begin(), direct.end());
        for (const auto & cell : occupied->second) {
          const auto section = task_lib::FacadeCoverageSectionForPoint(
            runtime->coverage_plan, VoxelCenter(cell, voxel_size_));
          if (section.has_value()) {
            claims.insert(*section);
          }
        }
      }
      UpdateCoverageClaims(
        source_id, context->second, claims,
        context->second.synthetic_no_facade ? "SIN_FACHADA" : "DEPTH_OCCUPIED");
    }
  }

  void RunVoxelMapWorker()
  {
    bool changed = false;
    if (pending_sparse_snapshot_.has_value()) {
      changed = voxel_map_->ApplySparseSnapshot(
        *pending_sparse_snapshot_, voxel_occupied_score_threshold_);
      pending_sparse_snapshot_.reset();
      PublishArchitecture("orbslam3_multi_to_task_server_sparse", "SPARSE_BOOTSTRAP_APPLIED");
    }
    if (!pending_sparse_upserts_.empty() || !pending_sparse_deletes_.empty()) {
      std::vector<task_lib::SparseEvidence> upserts;
      upserts.reserve(pending_sparse_upserts_.size());
      for (const auto & item : pending_sparse_upserts_) {
        upserts.push_back(item.second);
      }
      std::vector<std::string> deletes(
        pending_sparse_deletes_.begin(), pending_sparse_deletes_.end());
      changed =
        voxel_map_->ApplySparseDelta(
        upserts, deletes, voxel_occupied_score_threshold_) || changed;
      pending_sparse_upserts_.clear();
      pending_sparse_deletes_.clear();
    }
    if (voxel_map_builder_) {
      const auto materialized = voxel_map_builder_->Apply(&evidence_database_, voxel_map_.get());
      changed = materialized.changed || changed;
      ApplyCoverageClaimsFromMap(materialized);
      ProcessAppliedDepthContinuations(materialized.applied_source_ids);
      if (materialized.transactions != 0U) {
        RCLCPP_INFO(
          get_logger(),
          "[F6G-VOXEL-DELTAS-APPLIED] transactions=%zu upserts=%zu deletes=%zu free_cells=%zu changed=%s",
          materialized.transactions, materialized.upserts, materialized.deletes,
          materialized.free_cells, materialized.changed ? "true" : "false");
      }
    }
    if (depth_evidence_enabled_) {
      for (auto & item : depth_keyframe_evidence_) {
        changed = ApplyDepthEvidence(item.first, &item.second) || changed;
      }
    }
    (void)changed;
    FlushVoxelChanges("VOXEL_MAP_WORKER_COMMIT");
  }

  void RequestKeyframePose(const KeyframeIdentity & identity)
  {
    auto free_evidence = keyframe_free_evidence_.find(identity);
    auto depth_evidence = depth_keyframe_evidence_.find(identity);
    const bool database_has_sources = evidence_database_.HasSources(
      task_server::KeyframeEvidenceIdentity{
      identity.drone_id, identity.map_epoch, identity.keyframe_id});
    if ((!database_has_sources && free_evidence == keyframe_free_evidence_.end() &&
      depth_evidence == depth_keyframe_evidence_.end()) ||
      (free_evidence != keyframe_free_evidence_.end() && free_evidence->second.query_pending) ||
      (depth_evidence != depth_keyframe_evidence_.end() && depth_evidence->second.query_pending) ||
      !global_pose_client_->service_is_ready())
    {
      return;
    }
    if (free_evidence != keyframe_free_evidence_.end()) {
      free_evidence->second.query_pending = true;
    }
    if (depth_evidence != depth_keyframe_evidence_.end()) {
      depth_evidence->second.query_pending = true;
    }
    auto request = std::make_shared<orbslam3_msgs::srv::GetGlobalKeyFramePose::Request>();
    request->drone_id = identity.drone_id;
    request->map_epoch = identity.map_epoch;
    request->keyframe_id = identity.keyframe_id;
    global_pose_client_->async_send_request(
      request,
      [this, identity](
        rclcpp::Client<orbslam3_msgs::srv::GetGlobalKeyFramePose>::SharedFuture future)
      {
        const auto free_current = keyframe_free_evidence_.find(identity);
        if (free_current != keyframe_free_evidence_.end()) {
          free_current->second.query_pending = false;
        }
        const auto depth_current = depth_keyframe_evidence_.find(identity);
        if (depth_current != depth_keyframe_evidence_.end()) {
          depth_current->second.query_pending = false;
        }
        try {
          ApplyGlobalKeyframePose(future.get()->result, "service");
        } catch (const std::exception & error) {
          RCLCPP_WARN(
            get_logger(), "[F6I-FREE-KF-QUERY] kf=%s result=error detail=%s",
            KeyframeToken(identity).c_str(), error.what());
        }
      });
  }

  void RequestKnownKeyframePoses()
  {
    if (RequestPendingDepthIntegrationPoses()) {
      return;
    }
    if (!keyframe_free_evidence_.empty()) {
      auto next = free_requery_cursor_valid_ ?
        keyframe_free_evidence_.upper_bound(free_requery_cursor_) : keyframe_free_evidence_.begin();
      if (next == keyframe_free_evidence_.end()) {
        next = keyframe_free_evidence_.begin();
      }
      free_requery_cursor_ = next->first;
      free_requery_cursor_valid_ = true;
      RequestKeyframePose(next->first);
    }
    if (!depth_keyframe_evidence_.empty()) {
      auto next = depth_requery_cursor_valid_ ?
        depth_keyframe_evidence_.upper_bound(depth_requery_cursor_) :
        depth_keyframe_evidence_.begin();
      if (next == depth_keyframe_evidence_.end()) {
        next = depth_keyframe_evidence_.begin();
      }
      depth_requery_cursor_ = next->first;
      depth_requery_cursor_valid_ = true;
      RequestKeyframePose(next->first);
    }
  }

  bool RequestPendingDepthIntegrationPoses()
  {
    while (!depth_integrations_.empty()) {
      const auto & pending = depth_integrations_.front();
      const bool facade_applied = DepthObservationApplied(pending.facade_observation);
      const bool target_applied = DepthObservationApplied(pending.target_observation);
      if (facade_applied && target_applied) {
        const auto completed_task_id = pending.task_id;
        CompleteDepthIntegration(completed_task_id, "both_observations_applied");
        continue;
      }
      if (!facade_applied) {
        const auto identity = DepthObservationIdentity(pending.facade_observation);
        RCLCPP_INFO(
          get_logger(),
          "[F6N-DEPTH-TASK-POSE-REQUEST] task=%s drone=%u role=facade kf=%s frame=%lu",
          pending.task_id.c_str(), pending.drone_id, KeyframeToken(identity).c_str(),
          pending.facade_observation.tracking_frame_id);
        RequestKeyframePose(identity);
      }
      if (!target_applied) {
        const auto identity = DepthObservationIdentity(pending.target_observation);
        RCLCPP_INFO(
          get_logger(),
          "[F6N-DEPTH-TASK-POSE-REQUEST] task=%s drone=%u role=target kf=%s frame=%lu",
          pending.task_id.c_str(), pending.drone_id, KeyframeToken(identity).c_str(),
          pending.target_observation.tracking_frame_id);
        RequestKeyframePose(identity);
      }
      return true;
    }
    return false;
  }

  bool AddLocalFreeVoxel(
    const KeyframeIdentity & identity, KeyframeFreeEvidence * evidence,
    const task_lib::Vec3 & local_body)
  {
    const auto local_voxel_center = VoxelCenter(local_body, voxel_size_);
    const std::string source_id = "free_sweep:" + KeyframeToken(identity) + ":" +
      VoxelToken(local_voxel_center, voxel_size_);
    const auto [sample, inserted] = evidence->local_voxel_centers.emplace(
      source_id, local_voxel_center);
    if (!inserted) {
      return false;
    }
    const auto world_body = TransformPoint(evidence->w_t_keyframe, sample->second);
    voxel_map_->ReplaceFreeVolume(source_id, world_body, free_body_half_extent_);
    RCLCPP_INFO(
      get_logger(), "[F6I-FREE-VOXEL] kf=%s local_voxel=%s",
      KeyframeToken(identity).c_str(), VoxelToken(local_voxel_center, voxel_size_).c_str());
    return true;
  }

  void ApplyGlobalKeyframePose(
    const orbslam3_msgs::msg::GlobalKeyFramePose & pose, const char * source)
  {
    if (pose.status != orbslam3_msgs::msg::GlobalKeyFramePose::STATUS_AVAILABLE) {
      return;
    }
    const KeyframeIdentity identity{pose.drone_id, pose.map_epoch, pose.keyframe_id};
    const bool evidence_pose_updated = evidence_database_.UpdateKeyframePose(
      task_server::KeyframeEvidenceIdentity{
      pose.drone_id, pose.map_epoch, pose.keyframe_id}, pose.pose_revision, pose.w_t_keyframe);
    if (evidence_pose_updated) {
      RCLCPP_INFO(
        get_logger(), "[F6F-DEPTH-KF-POSE-WRITTEN] kf=%s revision=%lu source=%s",
        KeyframeToken(identity).c_str(), pose.pose_revision, source);
    }
    const auto free_evidence = keyframe_free_evidence_.find(identity);
    const auto depth_evidence = depth_keyframe_evidence_.find(identity);
    if (free_evidence == keyframe_free_evidence_.end() &&
      depth_evidence == depth_keyframe_evidence_.end() && !evidence_pose_updated)
    {
      return;
    }
    if (free_evidence != keyframe_free_evidence_.end() &&
      (!free_evidence->second.pose_available ||
      pose.pose_revision > free_evidence->second.pose_revision))
    {
      free_evidence->second.pose_available = true;
      free_evidence->second.pose_revision = pose.pose_revision;
      free_evidence->second.w_t_keyframe = pose.w_t_keyframe;
      const std::string keyframe_source = "free_kf:" + KeyframeToken(identity);
      voxel_map_->ReplaceFreeVolume(
        keyframe_source,
        VoxelCenter(
          task_lib::Vec3{pose.w_t_keyframe.position.x, pose.w_t_keyframe.position.y,
            pose.w_t_keyframe.position.z},
          voxel_size_),
        free_body_half_extent_);
      for (const auto & sample : free_evidence->second.local_voxel_centers) {
        voxel_map_->ReplaceFreeVolume(
          sample.first, TransformPoint(pose.w_t_keyframe, sample.second), free_body_half_extent_);
      }
      RCLCPP_INFO(
        get_logger(), "[F6I-FREE-KF] kf=%s revision=%lu source=%s traversed_voxels=%zu",
        KeyframeToken(identity).c_str(), pose.pose_revision, source,
        free_evidence->second.local_voxel_centers.size());
    }
    if (depth_evidence != depth_keyframe_evidence_.end() &&
      (!depth_evidence->second.pose_available ||
      pose.pose_revision > depth_evidence->second.pose_revision))
    {
      depth_evidence->second.pose_available = true;
      depth_evidence->second.pose_revision = pose.pose_revision;
      depth_evidence->second.w_t_keyframe = pose.w_t_keyframe;
      RCLCPP_INFO(
        get_logger(), "[F6N-DEPTH-POSE] kf=%s revision=%lu source=%s",
        KeyframeToken(identity).c_str(), pose.pose_revision, source);
    }
  }

  void HandleGlobalKeyframePose(orbslam3_msgs::msg::GlobalKeyFramePose::ConstSharedPtr pose)
  {
    if (pose) {
      ApplyGlobalKeyframePose(*pose, "push");
    }
  }

  void RememberTraversedFreeVolume(
    std::uint32_t drone_id, const orbslam3_msgs::msg::NavigationState & state)
  {
    if (!state.reference_keyframe_valid) {
      return;
    }
    const task_lib::Vec3 body_world{
      state.w_t_body.position.x, state.w_t_body.position.y, state.w_t_body.position.z};
    if (!IsFinite(body_world)) {
      return;
    }
    const KeyframeIdentity identity{drone_id, state.map_epoch, state.reference_keyframe_id};
    auto [evidence, inserted] = keyframe_free_evidence_.try_emplace(identity);
    (void)inserted;
    if (!evidence->second.pose_available) {
      RequestKeyframePose(identity);
      return;
    }
    if (!AddLocalFreeVoxel(
        identity, &evidence->second,
        InverseTransformPoint(evidence->second.w_t_keyframe, body_world)))
    {
      return;
    }
  }

  void HandleNavigation(
    std::uint32_t drone_id, orbslam3_msgs::msg::NavigationState::ConstSharedPtr state)
  {
    if (!state || !IsEligiblePose(*state, pose_source_)) {
      eligible_drones_.erase(drone_id);
      navigation_poses_.erase(drone_id);
      RemoveTaskAvailableDrone(drone_id);
      return;
    }
    const bool newly_eligible = eligible_drones_.insert(drone_id).second;
    const double yaw_rad = YawFromQuaternion(state->w_t_body.orientation);
    navigation_poses_[drone_id] = DronePose{
      task_lib::Vec3{state->w_t_body.position.x, state->w_t_body.position.y,
        state->w_t_body.position.z}, yaw_rad, state->map_epoch, state->pose_revision};
    auto & orientation = navigation_orientations_[drone_id];
    if (!orientation.initialized || orientation.map_epoch != state->map_epoch) {
      orientation.yaw_rad = yaw_rad;
      orientation.camera_pitch_rad = 0.0;
      orientation.map_epoch = state->map_epoch;
      orientation.initialized = true;
      RCLCPP_INFO(
        get_logger(),
        "[F6M-ORIENTATION-INITIALIZED] drone=%u epoch=%lu yaw_deg=%.3f pitch_deg=0.000",
        drone_id, state->map_epoch, yaw_rad * 180.0 / M_PI);
    }
    RememberTraversedFreeVolume(drone_id, *state);
    if (newly_eligible) {
      EnqueueTaskAvailableDrone(drone_id, "anchor_authoritative");
    }
    AssignAvailableTasks();
  }

  void HandlePlanRoute(
    const std::shared_ptr<mission_msgs::srv::PlanRoute::Request> request,
    std::shared_ptr<mission_msgs::srv::PlanRoute::Response> response)
  {
    if (!request || !response) {
      return;
    }
    const auto pose = navigation_poses_.find(request->drone_id);
    if (pose == navigation_poses_.end() || eligible_drones_.count(request->drone_id) == 0U) {
      response->reason = "dron sin pose canonica autorizada";
      return;
    }
    const task_lib::Vec3 target{
      request->target_world.x, request->target_world.y, request->target_world.z};
    if (!std::isfinite(target.x) || !std::isfinite(target.y) || !std::isfinite(target.z)) {
      response->reason = "objetivo XYZ no finito";
      return;
    }
    std::string planner_reason;
    auto * planner = PlannerFor(request->drone_id, &planner_reason);
    if (planner == nullptr) {
      response->reason = planner_reason;
      return;
    }
    PublishFlow("task_worker_to_planning", "PLAN_ROUTE_REQUEST");
    RCLCPP_INFO(
      get_logger(),
      "[F6G-PLAN-START] drone=%u task=%s map_revision=%lu target=(%.3f,%.3f,%.3f)",
      request->drone_id, request->task_id.c_str(), voxel_map_->revision(), target.x, target.y,
      target.z);
    const auto planning_started = std::chrono::steady_clock::now();
    const auto route = planner->Plan(pose->second.position, target);
    const auto planning_latency_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - planning_started).count();
    if (!route.success) {
      response->reason = "D* no encontro ruta segura: " + route.failure_reason;
      PublishPlanningFlow(
        "DSTAR_REJECT", request->drone_id, route.failure_reason, voxel_map_->revision(),
        planning_latency_ms, route.expanded, "");
      RCLCPP_WARN(
        get_logger(), "[F6G-PLAN-REJECT] drone=%u map_revision=%lu reason=%s expanded=%lu queue_pops=%lu stale_queue_pops=%lu coarse_factor=%ld corridor_width=%ld corridor_widenings=%lu latency_ms=%.3f",
        request->drone_id, voxel_map_->revision(), route.failure_reason.c_str(), route.expanded,
        route.queue_pops, route.stale_queue_pops, route.coarse_voxel_factor,
        route.corridor_half_width_cells, route.corridor_widenings, planning_latency_ms);
      return;
    }
    const std::string task_id = request->task_id.empty() ? "explicit_plan" : request->task_id;
    response->accepted = true;
    response->reason = route.incremental_repair ? "replan incremental" : "plan inicial";
    response->plan = BuildExecutablePlan(
      route, task_id, request->drone_id, pose->second,
      ExecutionVelocityForTarget(VoxelKeyFromWorld(request->target_world, voxel_size_)));
    const auto target_key = VoxelKeyFromWorld(request->target_world, voxel_size_);
    if (request->dispatch_execution) {
      if (!DispatchCoveragePlan(response->plan, target_key)) {
        response->accepted = false;
        response->reason = "D* encontro ruta, pero el despacho fue rechazado";
        return;
      }
      response->reason += "; despacho solicitado";
    } else {
      planned_route_publisher_->publish(response->plan);
    }
    PublishPlanningFlow(
      route.incremental_repair ? "DSTAR_REPLAN" : "DSTAR_PLAN", request->drone_id,
      route.incremental_repair ? "map_repair" : "initial", route.map_revision,
      planning_latency_ms, route.expanded, response->plan.trajectory_id);
    PublishArchitecture("task_server_to_gui_planned_route", "PLANNED_ROUTE_UPDATED");
    RCLCPP_INFO(
      get_logger(),
      "[F6G-PLAN] drone=%u task=%s trajectory_id=%s map_revision=%lu repair=%s expanded=%lu queue_pops=%lu stale_queue_pops=%lu coarse_factor=%ld corridor_width=%ld corridor_widenings=%lu waypoints=%zu corridor=%zu latency_ms=%.3f",
      request->drone_id, task_id.c_str(), response->plan.trajectory_id.c_str(),
      route.map_revision, route.incremental_repair ? "true" : "false", route.expanded,
      route.queue_pops, route.stale_queue_pops, route.coarse_voxel_factor,
      route.corridor_half_width_cells, route.corridor_widenings, response->plan.waypoints.size(),
      route.corridor.size(), planning_latency_ms);
  }

  task_lib::DStarLitePlanner * PlannerFor(
    std::uint32_t drone_id, std::string * reason, bool require_known_free = false,
    const task_lib::Vec3 * start_escape_position = nullptr)
  {
    const auto body = drone_reservation_body_cells_.find(drone_id);
    const task_lib::VoxelKey body_half_extent = body == drone_reservation_body_cells_.end() ?
      task_lib::VoxelKey{} : body->second;
    auto planner = planners_.find(drone_id);
    if (planner != planners_.end()) {
      const auto & profiles = require_known_free ? drone_facade_navigation_profiles_ :
        drone_navigation_profiles_;
      const auto profile = profiles.find(drone_id);
      if (profile != profiles.end()) {
        planner->second.SetNavigationSnapshot(
          NavigationSnapshotFor(
            drone_id, profile->second, body_half_extent, require_known_free,
            start_escape_position));
      }
      return &planner->second;
    }
    const auto & profiles = require_known_free ? drone_facade_navigation_profiles_ :
      drone_navigation_profiles_;
    const auto profile = profiles.find(drone_id);
    if (profile == profiles.end()) {
      if (reason != nullptr) {
        *reason = "dron sin perfil navegable registrado";
      }
      return nullptr;
    }
    task_lib::DStarLitePlanner instance;
    auto planner_parameters = planner_parameters_;
    planner_parameters.occupied_inflation_cells = 0;
    instance.Configure(geometry_data_.hard_flight_volume, voxel_size_, planner_parameters);
    instance.SetNavigationSnapshot(
      NavigationSnapshotFor(
        drone_id, profile->second, body_half_extent, require_known_free, start_escape_position));
    return &planners_.emplace(drone_id, std::move(instance)).first->second;
  }

  task_lib::NavigationSnapshot NavigationSnapshotFor(
    std::uint32_t drone_id, const std::string & profile_id,
    const task_lib::VoxelKey & body_half_extent, bool require_known_free,
    const task_lib::Vec3 * start_escape_position) const
  {
    auto snapshot = voxel_map_->NavigationSnapshotFor(profile_id);
    if (require_known_free && start_escape_position != nullptr &&
      start_escape_radius_voxels_ > 0)
    {
      const auto start = VoxelKeyFromWorld(ToPoint(*start_escape_position), voxel_size_);
      const auto radius_squared = start_escape_radius_voxels_ * start_escape_radius_voxels_;
      std::map<task_lib::VoxelKey, task_lib::NavigationCell> cells;
      for (const auto & cell : snapshot.cells) {
        cells[cell.key] = cell;
      }
      for (const auto & cell : voxel_map_->Snapshot()) {
        if (cell.state != task_lib::VoxelState::Free) {
          continue;
        }
        const auto dx = cell.key.ix - start.ix;
        const auto dy = cell.key.iy - start.iy;
        const auto dz = cell.key.iz - start.iz;
        if (dx * dx + dy * dy + dz * dz > radius_squared) {
          continue;
        }
        cells[cell.key] = task_lib::NavigationCell{
          cell.key, task_lib::VoxelState::Free, true,
          task_lib::kNavigationAllNeighbors, 1.0};
      }
      snapshot.cells.clear();
      snapshot.cells.reserve(cells.size());
      for (const auto & cell : cells) {
        snapshot.cells.push_back(cell.second);
      }
    }
    return reservation_overlay_.ApplyTo(snapshot, drone_id, body_half_extent);
  }

  std::optional<task_lib::Vec3> SelectUnknownFallbackFreeTarget(
    std::uint32_t drone_id, const task_lib::FacadeCandidate & candidate,
    const DronePose & pose, const task_lib::BaseSubRoi & region) const
  {
    const auto target_key = VoxelKeyFromWorld(ToPoint(candidate.target), voxel_size_);
    const auto radius_squared =
      unknown_fallback_free_radius_voxels_ * unknown_fallback_free_radius_voxels_;
    const double middle_height = 0.5 * (region.bounds.min.z + region.bounds.max.z);
    const auto profile = drone_facade_navigation_profiles_.find(drone_id);
    const auto body = drone_reservation_body_cells_.find(drone_id);
    if (profile == drone_facade_navigation_profiles_.end()) {
      return std::nullopt;
    }
    const auto navigation_snapshot = NavigationSnapshotFor(
      drone_id, profile->second,
      body == drone_reservation_body_cells_.end() ? task_lib::VoxelKey{} : body->second,
      true, &pose.position);
    std::optional<task_lib::Vec3> best;
    std::optional<task_lib::VoxelKey> best_key;
    double best_cost = std::numeric_limits<double>::infinity();
    for (const auto & cell : voxel_map_->Snapshot()) {
      if (cell.state != task_lib::VoxelState::Free) {
        continue;
      }
      const auto dx = cell.key.ix - target_key.ix;
      const auto dy = cell.key.iy - target_key.iy;
      const auto dz = cell.key.iz - target_key.iz;
      if (dx * dx + dy * dy + dz * dz > radius_squared) {
        continue;
      }
      const auto navigation_cell = std::find_if(
        navigation_snapshot.cells.begin(), navigation_snapshot.cells.end(),
        [&cell](const auto & candidate_cell) {return candidate_cell.key == cell.key;});
      if (navigation_cell == navigation_snapshot.cells.end() ||
        !navigation_cell->traversable)
      {
        continue;
      }
      const auto fallback = VoxelCenter(cell.key, voxel_size_);
      const double wall_cost = std::abs(
        Norm(Subtract(fallback, candidate.visual_target)) -
        facade_preferences_.preferred_wall_distance_m);
      const double displacement_cost = std::abs(
        Norm(Subtract(fallback, pose.position)) - facade_preferences_.preferred_displacement_m);
      const double height_cost = std::abs(fallback.z - middle_height);
      const double cost = facade_preferences_.wall_distance_weight * wall_cost +
        facade_preferences_.displacement_weight * displacement_cost +
        facade_preferences_.height_weight * height_cost;
      if (!best.has_value() || cost < best_cost - 1e-9 ||
        (std::abs(cost - best_cost) <= 1e-9 && cell.key < *best_key))
      {
        best = fallback;
        best_key = cell.key;
        best_cost = cost;
      }
    }
    return best;
  }

  bool RequeueAppliedUnknownDepthContinuation(task_server::WorkflowWorkItem & continuation)
  {
    const auto context = autonomous_workflow_contexts_.find(continuation.identity.workflow_id);
    const auto task = std::find_if(
      tasks_.begin(), tasks_.end(), [&continuation](const auto & candidate) {
        return candidate.task_id == continuation.identity.task_id &&
        candidate.assigned_drone_id == continuation.identity.drone_id;
      });
    const auto pose = navigation_poses_.find(continuation.identity.drone_id);
    if (context == autonomous_workflow_contexts_.end() || task == tasks_.end() ||
      pose == navigation_poses_.end())
    {
      return false;
    }
    const auto target_key =
      VoxelKeyFromWorld(ToPoint(context->second.candidate.target), voxel_size_);
    const auto target_state = voxel_map_->StateAt(target_key);
    const auto profile = drone_facade_navigation_profiles_.find(continuation.identity.drone_id);
    const auto body = drone_reservation_body_cells_.find(continuation.identity.drone_id);
    const auto navigation_snapshot = profile == drone_facade_navigation_profiles_.end() ?
      task_lib::NavigationSnapshot{} : NavigationSnapshotFor(
      continuation.identity.drone_id, profile->second,
      body == drone_reservation_body_cells_.end() ? task_lib::VoxelKey{} : body->second,
      true, &pose->second.position);
    const auto target_navigation = std::find_if(
      navigation_snapshot.cells.begin(), navigation_snapshot.cells.end(),
      [&target_key](const auto & cell) {return cell.key == target_key;});
    const bool target_traversable = target_navigation != navigation_snapshot.cells.end() &&
      target_navigation->traversable;
    if (target_state == task_lib::VoxelState::Free && target_traversable) {
      context->second.fallback_free_advance = false;
      RCLCPP_INFO(
        get_logger(),
        "[F6H-DEPTH-TARGET-RECHECK] drone=%u workflow=%s target=%s state=free action=plan",
        continuation.identity.drone_id, continuation.identity.workflow_id.c_str(),
        VoxelKeyToken(target_key).c_str());
      return true;
    }
    if (target_state == task_lib::VoxelState::Unknown) {
      const auto * region = RegionForTask(*task);
      const auto fallback = region == nullptr ? std::optional<task_lib::Vec3>{} :
      SelectUnknownFallbackFreeTarget(
        continuation.identity.drone_id, context->second.candidate, pose->second, *region);
      if (fallback.has_value()) {
        context->second.candidate.target = *fallback;
        context->second.fallback_free_advance = true;
        RCLCPP_INFO(
          get_logger(),
          "[F6H-DEPTH-TARGET-RECHECK] drone=%u workflow=%s target=%s state=unknown action=fallback_free fallback=(%.2f,%.2f,%.2f) radius_voxels=%ld",
          continuation.identity.drone_id, continuation.identity.workflow_id.c_str(),
          VoxelKeyToken(target_key).c_str(), fallback->x, fallback->y, fallback->z,
          unknown_fallback_free_radius_voxels_);
        return true;
      }
    }
    auto * runtime = FacadeRuntimeFor(*task);
    if (runtime != nullptr) {
      runtime->rejected_inspection_target_ratios.push_back(context->second.candidate.target_ratio);
    }
    autonomous_workflow_contexts_.erase(context);
    continuation.queue = task_server::WorkflowQueue::POINT_SELECTION;
    RCLCPP_INFO(
      get_logger(),
      "[F6H-DEPTH-TARGET-RECHECK] drone=%u workflow=%s target=%s state=%s action=reselect",
      continuation.identity.drone_id, continuation.identity.workflow_id.c_str(),
      VoxelKeyToken(target_key).c_str(), VoxelStateName(target_state));
    return true;
  }

  void LogFacadeStrictRouteFailure(
    std::uint32_t drone_id, const task_lib::Vec3 & start_world,
    const task_lib::Vec3 & goal_world, const task_lib::DStarLiteResult & route) const
  {
    if (!debug_facade_dstar_failure_) {
      return;
    }
    const auto profile = drone_facade_navigation_profiles_.find(drone_id);
    if (profile == drone_facade_navigation_profiles_.end()) {
      return;
    }
    const auto body = drone_reservation_body_cells_.find(drone_id);
    const auto base = voxel_map_->NavigationSnapshotFor(profile->second);
    const auto snapshot = reservation_overlay_.ApplyTo(
      base, drone_id,
      body == drone_reservation_body_cells_.end() ? task_lib::VoxelKey{} : body->second);
    const auto make_cells = [](const task_lib::NavigationSnapshot & value) {
        std::map<task_lib::VoxelKey, task_lib::NavigationCell> cells;
        for (const auto & cell : value.cells) {
          cells.emplace(cell.key, cell);
        }
        return cells;
      };
    const auto base_cells = make_cells(base);
    const auto overlay_cells = make_cells(snapshot);
    const auto cell_for = [](const auto & cells, const task_lib::NavigationSnapshot & source,
        const task_lib::VoxelKey & key) {
        const auto found = cells.find(key);
        if (found != cells.end()) {
          return found->second;
        }
        return task_lib::NavigationCell{
        key, task_lib::VoxelState::Unknown, !source.require_known_free,
        source.require_known_free ? 0U : task_lib::kNavigationAllNeighbors,
        source.unknown_cost_multiplier};
      };
    const auto inflation = drone_inflation_cells_.find(drone_id);
    const task_lib::VoxelKey inflation_cells = inflation == drone_inflation_cells_.end() ?
      task_lib::VoxelKey{} : inflation->second;
    std::vector<task_lib::VoxelKey> occupied_keys;
    for (const auto & cell : voxel_map_->Snapshot()) {
      if (cell.state == task_lib::VoxelState::Occupied) {
        occupied_keys.push_back(cell.key);
      }
    }
    const auto has_nearby_occupied = [&occupied_keys, inflation_cells](
      const task_lib::VoxelKey & key) {
        for (const auto & occupied_key : occupied_keys) {
          const auto delta_x = std::llabs(occupied_key.ix - key.ix);
          const auto delta_y = std::llabs(occupied_key.iy - key.iy);
          const auto delta_z = std::llabs(occupied_key.iz - key.iz);
          if (delta_x <= inflation_cells.ix && delta_y <= inflation_cells.iy &&
            delta_z <= inflation_cells.iz)
          {
            return true;
          }
        }
        return false;
      };
    const auto classify = [this, &base, &snapshot, &base_cells, &overlay_cells, &cell_for,
        &has_nearby_occupied](const task_lib::VoxelKey & key) {
        const auto raw = voxel_map_->StateAt(key);
        const auto base_cell = cell_for(base_cells, base, key);
        const auto overlay_cell = cell_for(overlay_cells, snapshot, key);
        if (overlay_cell.traversable) {
          return std::string{"free"};
        }
        if (raw == task_lib::VoxelState::Occupied) {
          return std::string{"occupied_real"};
        }
        if (!base_cell.traversable) {
          if (raw == task_lib::VoxelState::Unknown) {
            return std::string{"unknown"};
          }
          return has_nearby_occupied(key) ? std::string{"occupied_inflated"} :
                 std::string{"blocked_navigation"};
        }
        return std::string{"reserved"};
      };
    const auto start = VoxelKeyFromWorld(ToPoint(start_world), voxel_size_);
    const auto goal = VoxelKeyFromWorld(ToPoint(goal_world), voxel_size_);
    RCLCPP_WARN(
      get_logger(),
      "[F6I-STRICT-ROUTE-FAIL] drone=%u revision=%lu reason=%s used_safe_escape=%s start=%s raw=%s nav=%s goal=%s raw=%s nav=%s",
      drone_id, route.map_revision, route.failure_reason.c_str(),
      route.used_safe_escape ? "true" : "false", VoxelKeyToken(start).c_str(),
      VoxelStateName(voxel_map_->StateAt(start)), classify(start).c_str(),
      VoxelKeyToken(goal).c_str(), VoxelStateName(voxel_map_->StateAt(goal)),
      classify(goal).c_str());
    const std::int64_t margin_cells = static_cast<std::int64_t>(std::ceil(
        planner_parameters_.search_margin_m / voxel_size_)) + 2;
    const task_lib::VoxelKey lower{
      std::min(start.ix, goal.ix) - margin_cells, std::min(start.iy, goal.iy) - margin_cells,
      std::min(start.iz, goal.iz) - margin_cells};
    const task_lib::VoxelKey upper{
      std::max(start.ix, goal.ix) + margin_cells, std::max(start.iy, goal.iy) + margin_cells,
      std::max(start.iz, goal.iz) + margin_cells};
    const auto inside = [&lower, &upper](const task_lib::VoxelKey & key) {
        return key.ix >= lower.ix && key.ix <= upper.ix && key.iy >= lower.iy &&
               key.iy <= upper.iy && key.iz >= lower.iz && key.iz <= upper.iz;
      };
    std::deque<task_lib::VoxelKey> pending;
    std::set<task_lib::VoxelKey> reachable;
    const auto start_cell = cell_for(overlay_cells, snapshot, start);
    if (start_cell.traversable) {
      pending.push_back(start);
      reachable.insert(start);
    }
    std::map<std::string, std::size_t> frontier;
    constexpr std::size_t kMaxDiagnosticCells = 100000U;
    while (!pending.empty() && reachable.size() < kMaxDiagnosticCells) {
      const auto current = pending.front();
      pending.pop_front();
      const auto current_cell = cell_for(overlay_cells, snapshot, current);
      for (std::int64_t dx = -1; dx <= 1; ++dx) {
        for (std::int64_t dy = -1; dy <= 1; ++dy) {
          for (std::int64_t dz = -1; dz <= 1; ++dz) {
            if (dx == 0 && dy == 0 && dz == 0) {
              continue;
            }
            const task_lib::VoxelKey neighbor{current.ix + dx, current.iy + dy, current.iz + dz};
            if (!inside(neighbor)) {
              continue;
            }
            const auto neighbor_cell = cell_for(overlay_cells, snapshot, neighbor);
            const bool edge_allowed =
              (current_cell.neighbor_mask & task_lib::NavigationNeighborBit(dx, dy, dz)) != 0U;
            if (!neighbor_cell.traversable || !edge_allowed) {
              ++frontier[classify(neighbor)];
              continue;
            }
            if (reachable.insert(neighbor).second) {
              pending.push_back(neighbor);
            }
          }
        }
      }
    }
    RCLCPP_WARN(
      get_logger(),
      "[F6I-STRICT-FREE-FRONTIER] drone=%u reachable=%zu goal_reachable=%s capped=%s frontier_occupied_real=%zu frontier_inflated=%zu frontier_reserved=%zu frontier_unknown=%zu",
      drone_id, reachable.size(), reachable.count(goal) != 0U ? "true" : "false",
      !pending.empty() ? "true" : "false", frontier["occupied_real"],
      frontier["occupied_inflated"], frontier["reserved"], frontier["unknown"]);
  }

  mission_msgs::msg::TrajectoryPlan BuildExecutablePlan(
    const task_lib::DStarLiteResult & route, const std::string & task_id,
    std::uint32_t drone_id, const DronePose & pose, double velocity_mps,
    bool require_known_free = false, const task_lib::Vec3 * start_escape_position = nullptr)
  {
    auto plan = ToTrajectoryPlan(
      route, mission_id_, task_id, drone_id, pose.map_epoch, ++plan_revision_);
    const auto orientation = navigation_orientations_.find(drone_id);
    const double yaw_rad = orientation == navigation_orientations_.end() ? pose.yaw_rad :
      orientation->second.yaw_rad;
    const double camera_pitch_rad = orientation == navigation_orientations_.end() ? 0.0 :
      orientation->second.camera_pitch_rad;
    for (auto & waypoint : plan.waypoints) {
      waypoint.yaw_rad = yaw_rad;
      waypoint.camera_pitch_rad = camera_pitch_rad;
    }
    RCLCPP_INFO(
      get_logger(),
      "[F6M-ORIENTATION-HOLD] drone=%u trajectory_id=%s yaw_deg=%.3f pitch_deg=%.3f",
      drone_id, plan.trajectory_id.c_str(), yaw_rad * 180.0 / M_PI,
      camera_pitch_rad * 180.0 / M_PI);
    task_lib::NavigationSnapshot navigation_snapshot;
    const auto & profiles = require_known_free ? drone_facade_navigation_profiles_ :
      drone_navigation_profiles_;
    const auto profile = profiles.find(drone_id);
    if (profile != profiles.end()) {
      const auto body = drone_reservation_body_cells_.find(drone_id);
      navigation_snapshot = NavigationSnapshotFor(
        drone_id, profile->second,
        body == drone_reservation_body_cells_.end() ? task_lib::VoxelKey{} : body->second,
        require_known_free, start_escape_position);
    }
    const auto raw_waypoints = plan.waypoints.size();
    plan = SimplifyExecutableRoute(
      std::move(plan), ToPoint(pose.position), voxel_size_,
      trajectory_waypoint_min_separation_m_, navigation_snapshot);
    if (plan.waypoints.size() != raw_waypoints) {
      RCLCPP_INFO(
        get_logger(),
        "[F6I-ROUTE-SIMPLIFIED] trajectory_id=%s raw_waypoints=%zu final_waypoints=%zu min_separation_m=%.3f",
        plan.trajectory_id.c_str(), raw_waypoints, plan.waypoints.size(),
        trajectory_waypoint_min_separation_m_);
    }
    plan = TimeExecutableRoute(
      std::move(plan), velocity_mps, execution_timing_factor_,
      trajectory_min_segment_duration_sec_);
    return plan;
  }

  const task_lib::BaseSubRoi * RegionForTask(
    const mission_msgs::msg::TaskState & task) const
  {
    const auto region = std::find_if(
      geometry_data_.regions.begin(), geometry_data_.regions.end(), [&task](const auto & value) {
        return value.region_id == task.region_id;
      });
    return region == geometry_data_.regions.end() ? nullptr : &(*region);
  }

  std::vector<task_lib::Vec3> OccupiedPointsIn(
    const task_lib::AxisAlignedBox & bounds) const
  {
    std::vector<task_lib::Vec3> points;
    for (const auto & cell : voxel_map_->Snapshot()) {
      if (cell.state != task_lib::VoxelState::Occupied) {
        continue;
      }
      const auto center = VoxelCenter(cell.key, voxel_size_);
      if (center.x >= bounds.min.x && center.x <= bounds.max.x &&
        center.y >= bounds.min.y && center.y <= bounds.max.y &&
        center.z >= bounds.min.z && center.z <= bounds.max.z)
      {
        points.push_back(center);
      }
    }
    return points;
  }

  FacadeTaskRuntime * FacadeRuntimeFor(mission_msgs::msg::TaskState & task)
  {
    const auto * region = RegionForTask(task);
    if (region == nullptr) {
      return nullptr;
    }
    auto & runtime = facade_runtime_by_task_[task.task_id];
    if (!runtime.initialized) {
      runtime.facade = task_lib::EstimateFacadeLine(
        *region, geometry_data_.mapping_roi, OccupiedPointsIn(region->bounds),
        facade_preferences_.preferred_wall_distance_m);
      runtime.coverage_plan = task_lib::BuildFacadeCoveragePlan(
        region->bounds, geometry_data_.mapping_roi, voxel_size_, facade_coverage_offset_voxels_);
      runtime.active_coverage_sections.assign(runtime.coverage_plan.sections.size(), false);
      runtime.initialized = runtime.facade.length_m > 1e-6 &&
        !runtime.coverage_plan.sections.empty();
      RCLCPP_INFO(
        get_logger(),
        "[F6H-FACADE-INITIALIZED] task=%s side=%s u_sections=%zu open_side=%u legacy_length_m=%.3f",
        task.task_id.c_str(), task.region_id.c_str(), runtime.coverage_plan.sections.size(),
        static_cast<unsigned int>(runtime.coverage_plan.open_side), runtime.facade.length_m);
    }
    return runtime.initialized ? &runtime : nullptr;
  }

  void SyncFacadeTaskState(
    mission_msgs::msg::TaskState & task, const FacadeTaskRuntime & runtime,
    const std::string & detail)
  {
    task.progress = static_cast<float>(
      task_lib::FacadeCoverageRatio(runtime.coverage_plan, runtime.active_coverage_sections));
    task.progress_known = true;
    task.coverage_intervals.clear();
    for (std::size_t index = 0U; index < runtime.coverage_plan.sections.size(); ++index) {
      if (index >= runtime.active_coverage_sections.size() ||
        !runtime.active_coverage_sections[index])
      {
        continue;
      }
      const auto & section = runtime.coverage_plan.sections[index];
      mission_msgs::msg::FacadeCoverageInterval message;
      message.region_id = task.region_id;
      message.start_ratio = static_cast<double>(index) /
        static_cast<double>(runtime.coverage_plan.sections.size());
      message.end_ratio = static_cast<double>(index + 1U) /
        static_cast<double>(runtime.coverage_plan.sections.size());
      message.start_world = ToPoint(section.start);
      message.end_world = ToPoint(section.end);
      task.coverage_intervals.push_back(std::move(message));
    }
    task.detail = detail;
    ++task.state_revision;
    PublishTaskStates();
  }

  bool DepthObservationApplied(
    const mission_msgs::msg::DenseKFObservation & observation) const
  {
    const KeyframeIdentity identity{
      observation.drone_id, observation.map_epoch, observation.local_keyframe_id};
    const auto evidence = depth_keyframe_evidence_.find(identity);
    if (evidence == depth_keyframe_evidence_.end()) {
      return false;
    }
    const auto pose_revision = evidence->second.applied_pose_revisions.find(
      observation.tracking_frame_id);
    const auto source_revision = evidence->second.applied_source_revisions.find(
      observation.tracking_frame_id);
    return pose_revision != evidence->second.applied_pose_revisions.end() &&
           source_revision != evidence->second.applied_source_revisions.end() &&
           pose_revision->second == evidence->second.pose_revision &&
           source_revision->second == observation.source_revision;
  }

  void MarkFacadeInspectionFailure(
    mission_msgs::msg::TaskState & task, FacadeTaskRuntime & runtime,
    const std::string & reason)
  {
    ++runtime.inspection_failures;
    runtime.inspection_pending = false;
    runtime.awaiting_depth = false;
    runtime.awaiting_free_prefix_completion = false;
    runtime.inspection_stage = FacadeInspectionStage::IDLE;
    RCLCPP_WARN(
      get_logger(), "[F6N-FACADE-INSPECTION-FAILED] task=%s drone=%u attempt=%u/%u reason=%s",
      task.task_id.c_str(), task.assigned_drone_id, runtime.inspection_failures,
      facade_max_inspection_failures_, reason.c_str());
    if (runtime.inspection_failures < facade_max_inspection_failures_) {
      SyncFacadeTaskState(task, runtime, "Inspeccion pendiente de reintento: " + reason);
      return;
    }
    const auto drone_id = task.assigned_drone_id;
    task.state = mission_msgs::msg::TaskState::TO_FINISH;
    task.assigned_drone_id = 0U;
    runtime.inspection_failures = 0U;
    SyncFacadeTaskState(task, runtime, "A terminar: tres inspecciones sin corredor FREE");
    EnqueueTaskAvailableDrone(drone_id, "facade_inspection_failed");
    AssignAvailableTasks();
  }

  void HandleFacadeInspectionResult(
    const std::string & task_id, const task_lib::FacadeCandidate & candidate,
    const mission_msgs::srv::InspectFacade::Response & response)
  {
    const auto task_it = std::find_if(
      tasks_.begin(), tasks_.end(), [&task_id](const auto & task) {
        return task.task_id == task_id;
      });
    if (task_it == tasks_.end()) {
      return;
    }
    auto runtime_it = facade_runtime_by_task_.find(task_id);
    if (runtime_it == facade_runtime_by_task_.end()) {
      return;
    }
    auto & runtime = runtime_it->second;
    runtime.inspection_pending = false;
    const auto interrupt = fiducial_interrupt_task_by_drone_.find(task_it->assigned_drone_id);
    const bool fiducial_interrupt = interrupt != fiducial_interrupt_task_by_drone_.end() &&
      interrupt->second == task_id;
    if (!response.success && response.reason == "drone_busy") {
      if (fiducial_interrupt) {
        FinalizeFiducialInterrupt(task_it->assigned_drone_id, "fiducial_inspection_completed");
        return;
      }
      runtime.inspection_stage = FacadeInspectionStage::IDLE;
      SyncFacadeTaskState(
        *task_it, runtime, "Inspeccion aplazada: trayectoria fisica en curso");
      RCLCPP_INFO(
        get_logger(),
        "[F6N-FACADE-INSPECTION-DEFERRED] task=%s drone=%u reason=drone_busy",
        task_id.c_str(), task_it->assigned_drone_id);
      return;
    }
    if (!response.success) {
      if (fiducial_interrupt) {
        FinalizeFiducialInterrupt(task_it->assigned_drone_id, "fiducial_inspection_completed");
        return;
      }
      runtime.rejected_inspection_target_ratios.push_back(candidate.target_ratio);
      runtime.inspected_target.reset();
      runtime.awaiting_free_prefix_completion = false;
      runtime.segment_target_ratio.reset();
      runtime.awaiting_depth = false;
      runtime.inspection_stage = FacadeInspectionStage::IDLE;
      SyncFacadeTaskState(
        *task_it, runtime,
        "Objetivo depth descartado; seleccionara otro: " +
        (response.reason.empty() ? "invalid_depth_response" : response.reason));
      return;
    }
    auto facade_observation = response.facade_observation;
    auto target_observation = response.target_observation;
    LimitDepthObservation(&facade_observation);
    LimitDepthObservation(&target_observation);
    const auto facade_failure = DepthObservationFailure(facade_observation);
    const auto target_failure = DepthObservationFailure(target_observation);
    if (!facade_failure.empty() || !target_failure.empty()) {
      if (fiducial_interrupt) {
        FinalizeFiducialInterrupt(task_it->assigned_drone_id, "fiducial_inspection_completed");
        return;
      }
      const auto reason = !facade_failure.empty() ?
        "facade_depth_invalid:" + facade_failure : "target_depth_invalid:" + target_failure;
      runtime.rejected_inspection_target_ratios.push_back(candidate.target_ratio);
      runtime.inspection_stage = FacadeInspectionStage::IDLE;
      SyncFacadeTaskState(*task_it, runtime, "Objetivo depth descartado: " + reason);
      return;
    }
    QueueDepthObservation(facade_observation);
    QueueDepthObservation(target_observation);
    EnqueueDepthIntegration(
      task_id, task_it->assigned_drone_id, facade_observation, target_observation);
    if (fiducial_interrupt) {
      RCLCPP_INFO(
        get_logger(),
        "[F6N-FIDUCIAL-INTERRUPT-DEPTH-QUEUED] task=%s drone=%u first_frame=%lu second_frame=%lu",
        task_id.c_str(), task_it->assigned_drone_id, facade_observation.tracking_frame_id,
        target_observation.tracking_frame_id);
      FinalizeFiducialInterrupt(task_it->assigned_drone_id, "fiducial_inspection_depth_queued");
      return;
    }
    runtime.inspected_target = candidate.target;
    runtime.segment_target_ratio = candidate.target_ratio;
    runtime.movement_yaw_rad = response.facade_yaw_rad;
    runtime.movement_pitch_rad = response.facade_camera_pitch_rad;
    runtime.facade_observation = facade_observation;
    runtime.target_observation = target_observation;
    runtime.awaiting_depth = true;
    runtime.awaiting_free_prefix_completion = false;
    runtime.inspection_stage = FacadeInspectionStage::WAITING_DEPTH_INTEGRATION;
    SyncFacadeTaskState(
      *task_it, runtime, response.target_tracking_risk ?
      "Inspeccion recibida tras TRACKING_RISK; espera integracion FREE exacta" :
      "Inspeccion recibida; espera integracion FREE");
    RCLCPP_INFO(
      get_logger(),
      "[F6N-FACADE-DEPTH-WAIT] task=%s drone=%u stage=%s target_ratio=%.3f risk=%s first_frame=%lu second_frame=%lu",
      task_id.c_str(), task_it->assigned_drone_id,
      FacadeInspectionStageName(runtime.inspection_stage), candidate.target_ratio,
      response.target_tracking_risk ? "true" : "false",
      response.facade_observation.tracking_frame_id,
      response.target_observation.tracking_frame_id);
  }

  std::optional<task_lib::FacadeCandidate> SelectFacadeTaskCandidate(
    const mission_msgs::msg::TaskState & task, const FacadeTaskRuntime & runtime,
    const DronePose & pose) const
  {
    const auto * region = RegionForTask(task);
    if (region == nullptr) {
      return std::nullopt;
    }
    const auto candidate = task_lib::SelectFacadeCoverageCandidate(
      runtime.coverage_plan, geometry_data_.hard_flight_volume, pose.position,
      runtime.active_coverage_sections, facade_preferences_, voxel_map_->Snapshot(),
      voxel_size_, voxel_occupied_score_threshold_);
    return candidate.valid ? std::optional<task_lib::FacadeCandidate>{candidate} : std::nullopt;
  }

  bool StartFacadeInspection(
    mission_msgs::msg::TaskState & task, FacadeTaskRuntime & runtime,
    const DronePose & pose, const task_lib::FacadeCandidate & candidate)
  {
    const auto client = inspect_facade_clients_.find(task.assigned_drone_id);
    if (client == inspect_facade_clients_.end() || !client->second ||
      !client->second->service_is_ready())
    {
      return false;
    }
    auto request = std::make_shared<mission_msgs::srv::InspectFacade::Request>();
    request->mission_id = mission_id_;
    request->task_id = task.task_id;
    request->drone_id = task.assigned_drone_id;
    request->map_epoch = pose.map_epoch;
    request->target_world = ToPoint(candidate.visual_target);
    runtime.inspection_pending = true;
    runtime.inspection_stage = FacadeInspectionStage::INSPECTING;
    client->second->async_send_request(
      request,
      [this, task_id = task.task_id, candidate](
        rclcpp::Client<mission_msgs::srv::InspectFacade>::SharedFuture future) {
        try {
          HandleFacadeInspectionResult(task_id, candidate, *future.get());
        } catch (const std::exception & error) {
          const auto task_it = std::find_if(
            tasks_.begin(), tasks_.end(), [&task_id](const auto & task) {
              return task.task_id == task_id;
            });
          auto runtime_it = facade_runtime_by_task_.find(task_id);
          if (task_it != tasks_.end() && runtime_it != facade_runtime_by_task_.end()) {
            MarkFacadeInspectionFailure(*task_it, runtime_it->second, error.what());
          }
        }
      });
    PublishArchitecture(
      "task_server_to_task_manager_inspect_facade", "INSPECT_FACADE_REQUEST");
    SyncFacadeTaskState(task, runtime, "Inspeccion depth del voxel visual en curso");
    RCLCPP_INFO(
      get_logger(),
      "[F6N-FACADE-INSPECTION-REQUEST] task=%s drone=%u stage=%s visual=(%.2f,%.2f,%.2f) visual_score=%.3f pose=(%.2f,%.2f,%.2f) ratio=%.3f cost=%.3f",
      task.task_id.c_str(), task.assigned_drone_id,
      FacadeInspectionStageName(runtime.inspection_stage), candidate.visual_target.x,
      candidate.visual_target.y, candidate.visual_target.z, candidate.visual_score,
      candidate.target.x, candidate.target.y, candidate.target.z, candidate.target_ratio,
      candidate.score);
    return true;
  }

  bool DispatchInspectedFacadePlan(
    mission_msgs::msg::TaskState & task, FacadeTaskRuntime & runtime, const DronePose & pose,
    bool require_depth_applied = true, bool mark_failure = true)
  {
    if (!runtime.inspected_target.has_value() || !runtime.segment_target_ratio.has_value() ||
      (require_depth_applied &&
      (!DepthObservationApplied(runtime.facade_observation) ||
      !DepthObservationApplied(runtime.target_observation))))
    {
      return false;
    }
    const auto fail = [this, &task, &runtime, mark_failure](const std::string & reason) {
        if (mark_failure) {
          MarkFacadeInspectionFailure(task, runtime, reason);
        }
        return false;
      };
    const auto target = *runtime.inspected_target;
    const double target_ratio = task_lib::ProjectToFacadeRatio(runtime.facade, target);
    std::string planner_reason;
    auto * planner = PlannerFor(task.assigned_drone_id, &planner_reason, true);
    if (planner == nullptr) {
      return fail(planner_reason);
    }
    auto route = planner->Plan(pose.position, target);
    auto execution_target = target;
    bool free_prefix = false;
    if (!route.success) {
      if (runtime.awaiting_free_prefix_completion) {
        runtime.rejected_inspection_target_ratios.push_back(target_ratio);
        runtime.inspected_target.reset();
        runtime.segment_start_ratio.reset();
        runtime.segment_target_ratio.reset();
        runtime.awaiting_depth = false;
        runtime.awaiting_free_prefix_completion = false;
        runtime.inspection_stage = FacadeInspectionStage::IDLE;
        SyncFacadeTaskState(
          task, runtime, "Corredor FREE aun incompleto; candidato visual descartado");
        RCLCPP_INFO(
          get_logger(),
          "[F6N-FACADE-CANDIDATE-SKIPPED] task=%s drone=%u reason=%s",
          task.task_id.c_str(), task.assigned_drone_id, route.failure_reason.c_str());
        return true;
      }
      std::string exploratory_reason;
      auto * exploratory = PlannerFor(task.assigned_drone_id, &exploratory_reason);
      if (exploratory == nullptr) {
        return fail(exploratory_reason);
      }
      const auto exploratory_route = exploratory->Plan(pose.position, target);
      std::optional<task_lib::DStarLiteResult> prefix_route;
      for (auto it = exploratory_route.corridor.rbegin();
        exploratory_route.success && it != exploratory_route.corridor.rend(); ++it)
      {
        if (voxel_map_->StateAt(*it) != task_lib::VoxelState::Free) {
          continue;
        }
        const auto candidate_target = VoxelCenter(*it, voxel_size_);
        if (std::hypot(
            std::hypot(candidate_target.x - pose.position.x, candidate_target.y - pose.position.y),
            candidate_target.z - pose.position.z) < voxel_size_)
        {
          continue;
        }
        const auto candidate_route = planner->Plan(pose.position, candidate_target);
        if (candidate_route.success) {
          prefix_route = candidate_route;
          execution_target = candidate_target;
          break;
        }
      }
      if (!prefix_route.has_value()) {
        runtime.rejected_inspection_target_ratios.push_back(target_ratio);
        runtime.inspected_target.reset();
        runtime.segment_start_ratio.reset();
        runtime.segment_target_ratio.reset();
        runtime.awaiting_depth = false;
        runtime.inspection_stage = FacadeInspectionStage::IDLE;
        SyncFacadeTaskState(task, runtime, "Sin prefijo FREE; candidato visual descartado");
        RCLCPP_INFO(
          get_logger(),
          "[F6N-FACADE-CANDIDATE-SKIPPED] task=%s drone=%u reason=no_free_prefix strict=%s exploratory=%s",
          task.task_id.c_str(), task.assigned_drone_id, route.failure_reason.c_str(),
          exploratory_route.success ? "success" : exploratory_route.failure_reason.c_str());
        return true;
      }
      route = std::move(*prefix_route);
      free_prefix = true;
      runtime.awaiting_free_prefix_completion = true;
    } else {
      runtime.awaiting_free_prefix_completion = false;
    }
    navigation_orientations_[task.assigned_drone_id] = NavigationOrientation{
      runtime.movement_yaw_rad, runtime.movement_pitch_rad, pose.map_epoch, true};
    auto plan = BuildExecutablePlan(
      route, task.task_id, task.assigned_drone_id, pose, execution_nominal_velocity_mps_, true);
    runtime.segment_start_ratio = task_lib::ProjectToFacadeRatio(runtime.facade, pose.position);
    runtime.segment_target_ratio = task_lib::ProjectToFacadeRatio(runtime.facade, execution_target);
    runtime.awaiting_depth = false;
    runtime.inspection_stage = FacadeInspectionStage::IDLE;
    runtime.inspection_failures = 0U;
    const auto target_key = VoxelKeyFromWorld(ToPoint(execution_target), voxel_size_);
    QueueOrDispatchCoveragePlan(plan, target_key, route.incremental_repair, true);
    PublishPlanningFlow(
      route.incremental_repair ? "DSTAR_REPLAN" : "DSTAR_PLAN", task.assigned_drone_id,
      free_prefix ? "facade_free_prefix" : "facade_free", route.map_revision, 0.0,
      route.expanded, plan.trajectory_id);
    RCLCPP_INFO(
      get_logger(),
      "[F6N-FACADE-FREE-PLAN] task=%s drone=%u visual_target=(%.2f,%.2f,%.2f) execution_target=(%.2f,%.2f,%.2f) free_prefix=%s require_known_free=true waypoints=%zu",
      task.task_id.c_str(), task.assigned_drone_id, target.x, target.y, target.z,
      execution_target.x, execution_target.y, execution_target.z,
      free_prefix ? "true" : "false", plan.waypoints.size());
    return true;
  }

  void RunFacadeWorker()
  {
    if (!execute_facade_sweeps_ || !execution_enabled_.load(std::memory_order_acquire)) {
      return;
    }
    for (auto & task : tasks_) {
      if (task.task_type != "MAP_SECTION" ||
        task.state != mission_msgs::msg::TaskState::RUNNING ||
        task.assigned_drone_id == 0U)
      {
        continue;
      }
      auto * runtime = FacadeRuntimeFor(task);
      const auto pose = navigation_poses_.find(task.assigned_drone_id);
      if (runtime == nullptr || pose == navigation_poses_.end() ||
        execution_runtime_by_drone_[task.assigned_drone_id].in_flight)
      {
        continue;
      }
      if (task_lib::FacadeCoverageRatio(runtime->covered) >=
        facade_preferences_.completion_ratio)
      {
        const auto drone_id = task.assigned_drone_id;
        task.state = mission_msgs::msg::TaskState::COMPLETED;
        task.assigned_drone_id = 0U;
        SyncFacadeTaskState(task, *runtime, "Barrido de fachada completado");
        EnqueueTaskAvailableDrone(drone_id, "facade_completed");
        AssignAvailableTasks();
        continue;
      }
      if (!IsDroneReady(task.assigned_drone_id) || runtime->inspection_pending) {
        continue;
      }
      if (runtime->awaiting_depth) {
        if (!DepthObservationApplied(runtime->facade_observation) ||
          !DepthObservationApplied(runtime->target_observation))
        {
          continue;
        }
        CompleteDepthIntegration(task.task_id, "runtime_recheck_ready");
        runtime->inspection_stage = FacadeInspectionStage::RECHECKING_SAME_TARGET;
        SyncFacadeTaskState(
          task, *runtime, "Depth integrado; recomprueba el mismo objetivo de fachada");
        RCLCPP_INFO(
          get_logger(),
          "[F6N-FACADE-RECHECK-SAME-TARGET] task=%s drone=%u target_ratio=%.3f",
          task.task_id.c_str(), task.assigned_drone_id,
          runtime->segment_target_ratio.value_or(-1.0));
        DispatchInspectedFacadePlan(task, *runtime, pose->second);
      } else if (runtime->awaiting_free_prefix_completion) {
        DispatchInspectedFacadePlan(task, *runtime, pose->second, false, false);
      } else {
        const auto candidate = SelectFacadeTaskCandidate(task, *runtime, pose->second);
        if (!candidate.has_value()) {
          runtime->rejected_inspection_target_ratios.clear();
          MarkFacadeInspectionFailure(task, *runtime, "no_facade_candidate");
          continue;
        }
        std::string planner_reason;
        auto * planner = PlannerFor(task.assigned_drone_id, &planner_reason, true);
        const auto route = planner == nullptr ? task_lib::DStarLiteResult{} :
        planner->Plan(pose->second.position, candidate->target);
        if (!route.success) {
          StartFacadeInspection(task, *runtime, pose->second, *candidate);
          continue;
        }
        runtime->movement_yaw_rad = task_lib::FacadeObservationYaw(runtime->facade);
        runtime->movement_pitch_rad = 0.0;
        runtime->segment_start_ratio = task_lib::ProjectToFacadeRatio(
          runtime->facade, pose->second.position);
        runtime->segment_target_ratio = candidate->target_ratio;
        runtime->awaiting_free_prefix_completion = false;
        navigation_orientations_[task.assigned_drone_id] = NavigationOrientation{
          runtime->movement_yaw_rad, runtime->movement_pitch_rad, pose->second.map_epoch, true};
        auto plan = BuildExecutablePlan(
          route, task.task_id, task.assigned_drone_id, pose->second,
          execution_nominal_velocity_mps_, true);
        QueueOrDispatchCoveragePlan(
          plan, VoxelKeyFromWorld(ToPoint(candidate->target), voxel_size_),
          route.incremental_repair, true);
        PublishPlanningFlow(
          route.incremental_repair ? "DSTAR_REPLAN" : "DSTAR_PLAN", task.assigned_drone_id,
          "facade_free", route.map_revision, 0.0, route.expanded, plan.trajectory_id);
        RCLCPP_INFO(
          get_logger(),
          "[F6N-FACADE-FREE-PLAN] task=%s drone=%u visual_target=(%.2f,%.2f,%.2f) execution_target=(%.2f,%.2f,%.2f) free_prefix=false require_known_free=true waypoints=%zu",
          task.task_id.c_str(), task.assigned_drone_id, candidate->visual_target.x,
          candidate->visual_target.y, candidate->visual_target.z,
          candidate->target.x, candidate->target.y, candidate->target.z, plan.waypoints.size());
      }
    }
  }


  void HandleExecutionToggle(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    execution_enabled_.store(request->data, std::memory_order_release);
    execution_gate_update_pending_.store(true, std::memory_order_release);
    response->success = true;
    response->message = request->data ? "ejecucion F6I habilitada" : "ejecucion F6I pausada";
    RCLCPP_INFO(
      get_logger(), "[F6I-EXECUTION-GATE-REQUEST] enabled=%s apply=deferred",
      request->data ? "true" : "false");
  }

  void ApplyExecutionGateRequest()
  {
    if (!execution_gate_update_pending_.exchange(false, std::memory_order_acq_rel)) {
      return;
    }
    const bool enabled = execution_enabled_.load(std::memory_order_acquire);
    RCLCPP_INFO(
      get_logger(), "[F6I-EXECUTION-GATE] enabled=%s pending=%zu",
      enabled ? "true" : "false", pending_execution_plans_.size());
    DispatchNextPendingPlan();
  }

  void QueueOrDispatchCoveragePlan(
    const mission_msgs::msg::TrajectoryPlan & plan, const task_lib::VoxelKey & target,
    bool route_repaired, bool require_known_free = false)
  {
    if (!execution_enabled_.load(std::memory_order_acquire)) {
      pending_execution_plans_[plan.task_id] = {
        plan.task_id, plan.drone_id, target, voxel_map_->revision(), require_known_free};
      planned_route_publisher_->publish(plan);
      RCLCPP_INFO(
        get_logger(), "[F6I-EXECUTION-PENDING] task=%s reason=gate_disabled",
        plan.task_id.c_str());
      return;
    }
    if (!execution_runtime_by_drone_[plan.drone_id].in_flight) {
      if (!IsDroneReady(plan.drone_id)) {
        pending_execution_plans_[plan.task_id] = {
          plan.task_id, plan.drone_id, target, voxel_map_->revision(), require_known_free};
        planned_route_publisher_->publish(plan);
        RCLCPP_INFO(
          get_logger(), "[F6K-SUBTASK-PENDING] task=%s drone=%u reason=awaiting_fifo_turn",
          plan.task_id.c_str(), plan.drone_id);
        return;
      }
      if (DispatchCoveragePlan(plan, target, true, require_known_free)) {
        ConsumeReadyDrone(plan.drone_id);
      }
      return;
    }
    const auto & runtime = execution_runtime_by_drone_[plan.drone_id];
    const bool same_target = runtime.task_id == plan.task_id &&
      runtime.target.has_value() && *runtime.target == target;
    if (same_target && route_repaired && runtime.corridor_affected) {
      RCLCPP_INFO(
        get_logger(), "[F6I-EXECUTION-PREEMPT] task=%s reason=active_corridor_changed",
        plan.task_id.c_str());
      RequestActiveExecutionStop(plan.drone_id, "coverage_route_repair");
      return;
    }
    if (same_target) {
      RCLCPP_INFO(
        get_logger(), "[F6I-EXECUTION-HOLD] task=%s reason=active_corridor_unchanged",
        plan.task_id.c_str());
      return;
    }
    pending_execution_plans_[plan.task_id] = {
      plan.task_id, plan.drone_id, target, voxel_map_->revision(), require_known_free};
    planned_route_publisher_->publish(plan);
    RCLCPP_INFO(
      get_logger(), "[F6I-EXECUTION-PENDING] task=%s reason=distinct_target",
      plan.task_id.c_str());
  }

  void DispatchNextPendingPlan()
  {
    if (!execution_enabled_.load(std::memory_order_acquire) || pending_execution_plans_.empty()) {
      return;
    }
    if (ready_drones_.empty()) {
      return;
    }
    const auto ready_it = std::find_if(
      ready_drones_.begin(), ready_drones_.end(), [this](std::uint32_t drone_id) {
        if (execution_runtime_by_drone_[drone_id].in_flight) {
          return false;
        }
        return std::any_of(
          pending_execution_plans_.begin(), pending_execution_plans_.end(),
          [drone_id](const auto & item) {return item.second.drone_id == drone_id;});
      });
    if (ready_it == ready_drones_.end()) {
      return;
    }
    const auto ready_drone_id = *ready_it;
    const auto pending_it = std::find_if(
      pending_execution_plans_.begin(), pending_execution_plans_.end(),
      [ready_drone_id](const auto & item) {return item.second.drone_id == ready_drone_id;});
    const auto pending = pending_it->second;
    pending_execution_plans_.erase(pending_it);
    const auto pose = navigation_poses_.find(pending.drone_id);
    std::string planner_reason;
    auto * planner = PlannerFor(
      pending.drone_id, &planner_reason, pending.require_known_free);
    if (pose == navigation_poses_.end() || planner == nullptr) {
      RCLCPP_WARN(
        get_logger(), "[F6I-EXECUTION-REPLAN-REJECT] task=%s reason=%s",
        pending.task_id.c_str(), planner_reason.c_str());
      return;
    }
    const auto route =
      planner->Plan(pose->second.position, VoxelCenter(pending.target, voxel_size_));
    if (!route.success) {
      RCLCPP_WARN(
        get_logger(), "[F6I-EXECUTION-REPLAN-REJECT] task=%s reason=%s",
        pending.task_id.c_str(), route.failure_reason.c_str());
      return;
    }
    auto plan = BuildExecutablePlan(
      route, pending.task_id, pending.drone_id, pose->second,
      ExecutionVelocityForTarget(pending.target), pending.require_known_free);
    RCLCPP_INFO(
      get_logger(),
      "[F6I-EXECUTION-REPLAN] task=%s selected_revision=%lu current_revision=%lu",
      pending.task_id.c_str(), pending.selected_revision, voxel_map_->revision());
    if (DispatchCoveragePlan(plan, pending.target, true, pending.require_known_free)) {
      ConsumeReadyDrone(pending.drone_id);
    }
  }

  bool DispatchCoveragePlan(
    const mission_msgs::msg::TrajectoryPlan & plan, const task_lib::VoxelKey & target,
    bool allow_reservation_replan = true, bool require_known_free = false)
  {
    const auto client = execute_trajectory_clients_.find(plan.drone_id);
    if (client == execute_trajectory_clients_.end() ||
      !client->second || !client->second->action_server_is_ready())
    {
      RCLCPP_WARN(
        get_logger(), "[F6I-EXECUTION-REJECT] task=%s reason=task_manager_action_no_disponible",
        plan.task_id.c_str());
      return false;
    }
    const auto body = drone_reservation_body_cells_.find(plan.drone_id);
    const auto corridor = SegmentCorridor(
      plan, voxel_size_, body == drone_reservation_body_cells_.end() ? task_lib::VoxelKey{} :
      body->second, reservation_sweep_sample_step_voxels_);
    const std::string reservation_id = "reservation_" + plan.trajectory_id;
    auto & runtime = execution_runtime_by_drone_[plan.drone_id];
    const bool reservation_committed = runtime.reservation_id.empty() ?
      reservation_overlay_.Commit(
      {reservation_id, plan.drone_id, task_lib::ReservationMode::Moving, corridor, 0U}) :
      reservation_overlay_.Replace(
      runtime.reservation_id,
      {reservation_id, plan.drone_id, task_lib::ReservationMode::Moving, corridor, 0U});
    if (!reservation_committed) {
      if (allow_reservation_replan) {
        const auto pose = navigation_poses_.find(plan.drone_id);
        std::string planner_reason;
        auto * planner = PlannerFor(plan.drone_id, &planner_reason, require_known_free);
        if (pose != navigation_poses_.end() && planner != nullptr) {
          const auto alternative =
            planner->Plan(pose->second.position, VoxelCenter(target, voxel_size_));
          if (alternative.success) {
            const auto replanned = BuildExecutablePlan(
              alternative, plan.task_id, plan.drone_id, pose->second,
              ExecutionVelocityForTarget(target), require_known_free);
            RCLCPP_INFO(
              get_logger(),
              "[F6J-RESERVATION-REPLAN] task=%s drone=%u rejected_trajectory_id=%s alternative_trajectory_id=%s",
              plan.task_id.c_str(), plan.drone_id, plan.trajectory_id.c_str(),
              replanned.trajectory_id.c_str());
            return DispatchCoveragePlan(replanned, target, false, require_known_free);
          }
          planner_reason = alternative.failure_reason;
        }
        RCLCPP_INFO(
          get_logger(),
          "[F6J-RESERVATION-REPLAN-REJECT] task=%s drone=%u reason=%s",
          plan.task_id.c_str(), plan.drone_id, planner_reason.c_str());
      }
      const auto task = std::find_if(
        tasks_.begin(), tasks_.end(), [&plan](const auto & candidate) {
          return candidate.task_id == plan.task_id;
        });
      if (task != tasks_.end()) {
        task->state = mission_msgs::msg::TaskState::WAITING;
        ++task->state_revision;
        task->detail = "waiting_reservation: conflicto espacial con otro dron";
        PublishTaskStates();
      }
      RCLCPP_INFO(
        get_logger(), "[F6J-RESERVATION-WAIT] task=%s drone=%u trajectory_id=%s cells=%zu",
        plan.task_id.c_str(), plan.drone_id, plan.trajectory_id.c_str(), corridor.size());
      PublishFlow("reservation_to_planning", "WAITING_RESERVATION");
      return false;
    }
    runtime.task_id = plan.task_id;
    runtime.trajectory_id = plan.trajectory_id;
    runtime.reservation_id = reservation_id;
    runtime.active_plan = plan;
    runtime.target = target;
    runtime.corridor = corridor;
    runtime.in_flight = true;
    runtime.stop_requested = false;
    runtime.corridor_affected = false;
    runtime.require_known_free = require_known_free;
    RCLCPP_INFO(
      get_logger(), "[F6J-RESERVATION-COMMIT] reservation_id=%s drone=%u trajectory_id=%s cells=%zu step_voxels=%.3f",
      reservation_id.c_str(), plan.drone_id, plan.trajectory_id.c_str(), corridor.size(),
      reservation_sweep_sample_step_voxels_);
    PublishVoxelMap();
    auto dispatched_plan = plan;
    dispatched_plan.execution_state = mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_PLANNED;
    dispatched_plan.execution_detail = "Trayectoria despachada a task_manager";
    execution_plans_[plan.trajectory_id] = dispatched_plan;
    // GUI y task_manager reciben exactamente la misma geometría e identidad.
    planned_route_publisher_->publish(dispatched_plan);
    ExecuteTrajectory::Goal goal;
    goal.plan = dispatched_plan;
    goal.stop_at_current_pose = false;
    rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions options;
    options.goal_response_callback =
      [this, drone_id = plan.drone_id, trajectory_id = plan.trajectory_id](
      std::shared_ptr<rclcpp_action::ClientGoalHandle<ExecuteTrajectory>> handle) {
        auto plan_it = execution_plans_.find(trajectory_id);
        if (plan_it == execution_plans_.end()) {
          return;
        }
        if (!handle) {
          const auto runtime = execution_runtime_by_drone_.find(drone_id);
          const bool stop_requested = runtime != execution_runtime_by_drone_.end() &&
            runtime->second.trajectory_id == trajectory_id && runtime->second.stop_requested;
          if (stop_requested) {
            execution_plans_.erase(plan_it);
            RCLCPP_INFO(
              get_logger(), "[F6I-EXECUTION-ROUTE-REPLACED] trajectory_id=%s state=rejected_before_stop",
              trajectory_id.c_str());
            return;
          }
          plan_it->second.execution_state =
            mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_REJECTED;
          plan_it->second.execution_detail = "task_manager rechazo la trayectoria";
          planned_route_publisher_->publish(plan_it->second);
          execution_plans_.erase(plan_it);
          if (runtime != execution_runtime_by_drone_.end() &&
            runtime->second.trajectory_id == trajectory_id)
          {
            ReleaseExecutionRuntime(drone_id, "trajectory_rejected");
          }
          return;
        }
        const auto runtime = execution_runtime_by_drone_.find(drone_id);
        if (runtime != execution_runtime_by_drone_.end() &&
          runtime->second.trajectory_id == trajectory_id)
        {
          if (runtime->second.stop_requested) {
            RCLCPP_INFO(
              get_logger(), "[F6I-EXECUTION-STOP-WAIT] trajectory_id=%s state=route_accepted",
              trajectory_id.c_str());
            return;
          }
        }
        plan_it->second.execution_state = mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_ACTIVE;
        plan_it->second.execution_detail = "Trayectoria aceptada por task_manager";
        planned_route_publisher_->publish(plan_it->second);
        RCLCPP_INFO(
          get_logger(), "[F6I-TRAJECTORY-ACTIVE] task=%s drone=%u trajectory_id=%s waypoints=%zu",
          plan_it->second.task_id.c_str(), plan_it->second.drone_id,
          trajectory_id.c_str(), plan_it->second.waypoints.size());
      };
    options.result_callback = [this, drone_id = plan.drone_id, trajectory_id = plan.trajectory_id](
      const rclcpp_action::ClientGoalHandle<ExecuteTrajectory>::WrappedResult & result) {
        auto plan_it = execution_plans_.find(trajectory_id);
        if (plan_it == execution_plans_.end()) {
          return;
        }
        const bool succeeded = result.code == rclcpp_action::ResultCode::SUCCEEDED &&
          result.result && result.result->success;
        auto & runtime = execution_runtime_by_drone_[drone_id];
        const bool visual_stop_terminal = result.result &&
          result.result->reason == "visual_risk_stop_started";
        const bool stop_requested = (runtime.trajectory_id == trajectory_id &&
          runtime.stop_requested) || visual_stop_terminal;
        plan_it->second.execution_state = succeeded ?
          mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_COMPLETED :
          mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_CANCELED;
        plan_it->second.execution_detail = succeeded ? "Segmento completado" :
          (result.result ? result.result->reason : "action cancelada");
        planned_route_publisher_->publish(plan_it->second);
        RCLCPP_INFO(
          get_logger(), "[F6I-TRAJECTORY-FINAL] task=%s trajectory_id=%s success=%s detail=%s",
          plan_it->second.task_id.c_str(), trajectory_id.c_str(), succeeded ? "true" : "false",
          plan_it->second.execution_detail.c_str());
        if (stop_requested) {
          execution_plans_.erase(plan_it);
          RCLCPP_INFO(
            get_logger(), "[F6I-EXECUTION-ROUTE-REPLACED] trajectory_id=%s state=terminal_waiting_stop",
            trajectory_id.c_str());
          return;
        }
        const bool primary_task_released = succeeded &&
          RecordFacadeCoverage(drone_id, trajectory_id);
        execution_plans_.erase(plan_it);
        if (runtime.trajectory_id == trajectory_id) {
          ReleaseExecutionRuntime(
            drone_id, succeeded ? "trajectory_completed" : "trajectory_terminal");
        }
        if (primary_task_released) {
          EnqueueTaskAvailableDrone(drone_id, "facade_primary_task_released");
          AssignAvailableTasks();
        }
      };
    client->second->async_send_goal(goal, options);
    RCLCPP_INFO(
      get_logger(), "[F6I-EXECUTION-DISPATCH] task=%s drone=%u trajectory_id=%s waypoints=%zu",
      plan.task_id.c_str(), plan.drone_id, plan.trajectory_id.c_str(), plan.waypoints.size());
    return true;
  }

  bool RecordFacadeCoverage(std::uint32_t drone_id, const std::string & trajectory_id)
  {
    const auto execution = execution_runtime_by_drone_.find(drone_id);
    if (execution == execution_runtime_by_drone_.end() ||
      execution->second.trajectory_id != trajectory_id)
    {
      return false;
    }
    const auto task = std::find_if(
      tasks_.begin(), tasks_.end(), [&execution](const auto & candidate) {
        return candidate.task_id == execution->second.task_id;
      });
    const auto facade = facade_runtime_by_task_.find(execution->second.task_id);
    if (task == tasks_.end() || facade == facade_runtime_by_task_.end() ||
      !facade->second.segment_start_ratio.has_value() ||
      !facade->second.segment_target_ratio.has_value())
    {
      return false;
    }
    auto & runtime = facade->second;
    const auto pose = navigation_poses_.find(drone_id);
    const double orientation_tolerance_rad =
      facade_orientation_tolerance_deg_ * M_PI / 180.0;
    if (pose == navigation_poses_.end() ||
      AngularDistance(pose->second.yaw_rad, runtime.movement_yaw_rad) >
      orientation_tolerance_rad)
    {
      const double reached_yaw_deg = pose == navigation_poses_.end() ?
        std::numeric_limits<double>::quiet_NaN() : pose->second.yaw_rad * 180.0 / M_PI;
      runtime.segment_start_ratio.reset();
      runtime.segment_target_ratio.reset();
      SyncFacadeTaskState(
        *task, runtime, "Tramo no contabilizado: orientacion de fachada fuera de tolerancia");
      RCLCPP_WARN(
        get_logger(),
        "[F6H-FACADE-COVERAGE-SKIPPED] task=%s drone=%u reached_yaw_deg=%.3f expected_yaw_deg=%.3f tolerance_deg=%.3f",
        task->task_id.c_str(), drone_id, reached_yaw_deg,
        runtime.movement_yaw_rad * 180.0 / M_PI, facade_orientation_tolerance_deg_);
      return false;
    }
    runtime.covered.push_back(
      {*runtime.segment_start_ratio, *runtime.segment_target_ratio});
    runtime.covered = task_lib::MergeFacadeCoverage(runtime.covered);
    runtime.segment_start_ratio.reset();
    const double progress = task_lib::FacadeCoverageRatio(runtime.covered);
    const double terminal_ratio = *runtime.segment_target_ratio;
    runtime.segment_target_ratio.reset();
    const bool completed = progress >= facade_preferences_.completion_ratio;
    const bool reached_side = terminal_ratio <= 1e-3 || terminal_ratio >= 1.0 - 1e-3;
    if (completed) {
      task->state = mission_msgs::msg::TaskState::COMPLETED;
      task->assigned_drone_id = 0U;
      SyncFacadeTaskState(*task, runtime, "Barrido de fachada completado");
    } else if (reached_side) {
      task->state = mission_msgs::msg::TaskState::TO_FINISH;
      task->assigned_drone_id = 0U;
      SyncFacadeTaskState(
        *task, runtime, "A terminar: extremo alcanzado con intervalo de fachada pendiente");
    } else {
      SyncFacadeTaskState(
        *task, runtime, "Barrido parcial completado; solicita siguiente subtarea lateral");
    }
    RCLCPP_INFO(
      get_logger(),
      "[F6H-FACADE-COVERAGE] task=%s drone=%u progress=%.3f reached_side=%s state=%u intervals=%zu",
      task->task_id.c_str(), drone_id, progress, reached_side ? "true" : "false",
      task->state, runtime.covered.size());
    return completed || reached_side;
  }


  bool IsRegisteredDrone(std::uint32_t drone_id) const
  {
    const auto registered = registry_->Snapshot();
    return std::any_of(
      registered.begin(), registered.end(), [drone_id](const auto & registration) {
        return registration.drone_id == drone_id;
      });
  }

  bool HasActivePrimaryTask(std::uint32_t drone_id) const
  {
    return std::any_of(
      tasks_.begin(), tasks_.end(), [drone_id](const auto & task) {
        return task.assigned_drone_id == drone_id &&
        (task.state == mission_msgs::msg::TaskState::ASSIGNED ||
        task.state == mission_msgs::msg::TaskState::RUNNING ||
        task.state == mission_msgs::msg::TaskState::PAUSED);
      });
  }

  void EnqueueTaskAvailableDrone(std::uint32_t drone_id, const std::string & reason)
  {
    if (drone_id == 0U || !IsRegisteredDrone(drone_id) ||
      eligible_drones_.count(drone_id) == 0U ||
      HasActivePrimaryTask(drone_id) ||
      fiducial_interrupt_task_by_drone_.count(drone_id) != 0U ||
      task_available_drone_ids_.count(drone_id) != 0U)
    {
      return;
    }
    const auto pose = navigation_poses_.find(drone_id);
    task_server::WorkflowWorkItem item;
    item.queue = task_server::WorkflowQueue::TASK_ASSIGNMENT;
    item.identity.drone_id = drone_id;
    item.identity.task_id = "assignment_pending";
    item.identity.workflow_id = "assignment:" + std::to_string(drone_id);
    item.identity.command_id = item.identity.workflow_id;
    item.identity.map_epoch = pose == navigation_poses_.end() ? 0U : pose->second.map_epoch;
    item.identity.map_revision = voxel_map_->revision();
    if (!workflow_scheduler_.Enqueue(item)) {
      return;
    }
    task_available_drone_ids_.insert(drone_id);
    RCLCPP_INFO(
      get_logger(), "[F6B-ASSIGNMENT-QUEUED] drone=%u queue_size=%zu reason=%s",
      drone_id, workflow_scheduler_.QueueSize(task_server::WorkflowQueue::TASK_ASSIGNMENT),
      reason.c_str());
    PublishFlow("task_worker_registry", "TASK_READY_AFTER_ANCHOR");
  }

  void RemoveTaskAvailableDrone(std::uint32_t drone_id)
  {
    if (task_available_drone_ids_.erase(drone_id) == 0U) {
      return;
    }
  }

  void RemoveReadyDrone(std::uint32_t drone_id)
  {
    if (ready_drone_ids_.erase(drone_id) == 0U) {
      return;
    }
    ready_drones_.erase(
      std::remove(ready_drones_.begin(), ready_drones_.end(), drone_id), ready_drones_.end());
  }

  void FinalizeFiducialInterrupt(std::uint32_t drone_id, const std::string & reason)
  {
    const auto pending = fiducial_interrupt_task_by_drone_.find(drone_id);
    if (pending == fiducial_interrupt_task_by_drone_.end()) {
      return;
    }
    const auto task = std::find_if(
      tasks_.begin(), tasks_.end(), [&pending](const auto & candidate) {
        return candidate.task_id == pending->second;
      });
    if (task != tasks_.end()) {
      task->state = mission_msgs::msg::TaskState::TO_FINISH;
      task->assigned_drone_id = 0U;
      auto * facade = FacadeRuntimeFor(*task);
      if (facade != nullptr) {
        facade->inspection_pending = false;
        facade->awaiting_depth = false;
        facade->inspection_stage = FacadeInspectionStage::IDLE;
        SyncFacadeTaskState(
          *task, *facade, "A terminar: pipeline fiducial primario activado");
      } else {
        ++task->state_revision;
        task->detail = "A terminar: pipeline fiducial primario activado";
        PublishTaskStates();
      }
    }
    fiducial_interrupt_task_by_drone_.erase(pending);
    RemoveReadyDrone(drone_id);
    EnqueueTaskAvailableDrone(drone_id, reason);
    AssignAvailableTasks();
    RCLCPP_INFO(
      get_logger(), "[F6H-FIDUCIAL-INTERRUPT-FINAL] drone=%u reason=%s",
      drone_id, reason.c_str());
  }

  void AssignAvailableTasks()
  {
    while (true) {
      const auto assignment = workflow_scheduler_.Dequeue(
        task_server::WorkflowQueue::TASK_ASSIGNMENT);
      if (!assignment.has_value()) {
        return;
      }
      const auto drone_id = assignment->identity.drone_id;
      if (eligible_drones_.count(drone_id) == 0U || HasActivePrimaryTask(drone_id) ||
        fiducial_interrupt_task_by_drone_.count(drone_id) != 0U)
      {
        task_available_drone_ids_.erase(drone_id);
        continue;
      }
      const auto pose = navigation_poses_.find(drone_id);
      if (pose == navigation_poses_.end()) {
        task_available_drone_ids_.erase(drone_id);
        continue;
      }
      auto selected = tasks_.end();
      double selected_cost = std::numeric_limits<double>::infinity();
      for (auto task = tasks_.begin(); task != tasks_.end(); ++task) {
        if (task->task_type != "MAP_SECTION" ||
          (task->state != mission_msgs::msg::TaskState::PENDING &&
          task->state != mission_msgs::msg::TaskState::TO_FINISH))
        {
          continue;
        }
        const auto region = std::find_if(
          geometry_data_.regions.begin(), geometry_data_.regions.end(),
          [task](const auto & candidate) {
            return candidate.region_id == task->region_id;
          });
        if (region == geometry_data_.regions.end()) {
          continue;
        }
        double cost = SquaredDistanceToBox(pose->second.position, region->bounds);
        if (task->state == mission_msgs::msg::TaskState::TO_FINISH) {
          auto * facade = FacadeRuntimeFor(*task);
          if (facade == nullptr) {
            continue;
          }
          cost = std::numeric_limits<double>::infinity();
          for (const auto & interval : task_lib::UncoveredFacadeIntervals(facade->covered)) {
            const double projected = task_lib::ProjectToFacadeRatio(
              facade->facade, pose->second.position);
            const double ratio = std::clamp(projected, interval.start_ratio, interval.end_ratio);
            cost = std::min(
              cost, SquaredDistance(
                pose->second.position, task_lib::PointOnFacade(facade->facade, ratio)));
          }
        }
        if (cost + 1e-12 < selected_cost ||
          (std::abs(cost - selected_cost) <= 1e-12 &&
          (selected == tasks_.end() || task->region_id < selected->region_id)))
        {
          selected = task;
          selected_cost = cost;
        }
      }
      if (selected == tasks_.end()) {
        workflow_scheduler_.Requeue(*assignment);
        return;
      }
      task_available_drone_ids_.erase(drone_id);
      selected->assigned_drone_id = drone_id;
      selected->state = mission_msgs::msg::TaskState::ASSIGNED;
      ++selected->state_revision;
      selected->detail =
        "Asignada por proximidad al intervalo de fachada pendiente; espera confirmacion";
      PublishTaskStates();
      PublishFlow("task_worker_to_task_manager", "TASK_ASSIGNED");
      PublishArchitecture("task_server_to_task_manager", "TASK_ASSIGNED");
      task_server::WorkflowWorkItem point_selection;
      point_selection.queue = task_server::WorkflowQueue::POINT_SELECTION;
      point_selection.identity.drone_id = drone_id;
      point_selection.identity.task_id = selected->task_id;
      point_selection.identity.workflow_id =
        "point:" + selected->task_id + ":" + std::to_string(selected->state_revision);
      point_selection.identity.command_id = point_selection.identity.workflow_id;
      point_selection.identity.map_epoch = pose->second.map_epoch;
      point_selection.identity.map_revision = voxel_map_->revision();
      workflow_scheduler_.Enqueue(point_selection);
      RCLCPP_INFO(
        get_logger(), "[F6B-ASSIGNED] task=%s drone=%u source=%s distance_m=%.3f point_queue=%zu",
        selected->task_id.c_str(), drone_id, pose_source_.c_str(), std::sqrt(selected_cost),
        workflow_scheduler_.QueueSize(task_server::WorkflowQueue::POINT_SELECTION));
    }
  }

  void EnqueueReadyDrone(
    std::uint32_t drone_id, const std::string & reason, bool priority = false)
  {
    if (drone_id == 0U || ready_drone_ids_.count(drone_id) != 0U) {
      return;
    }
    if (priority) {
      ready_drones_.push_front(drone_id);
    } else {
      ready_drones_.push_back(drone_id);
    }
    ready_drone_ids_.insert(drone_id);
    RCLCPP_INFO(
      get_logger(), "[F6K-SUBTASK-READY] drone=%u queue_size=%zu priority=%s reason=%s",
      drone_id, ready_drones_.size(), priority ? "true" : "false", reason.c_str());
    PublishFlow("task_worker_to_planning", "SUBTASK_READY");
  }

  bool IsDroneReady(std::uint32_t drone_id) const
  {
    return ready_drone_ids_.count(drone_id) != 0U;
  }

  void ConsumeReadyDrone(std::uint32_t drone_id)
  {
    if (!IsDroneReady(drone_id)) {
      return;
    }
    ready_drone_ids_.erase(drone_id);
    const auto ready = std::find(ready_drones_.begin(), ready_drones_.end(), drone_id);
    if (ready != ready_drones_.end()) {
      ready_drones_.erase(ready);
    }
    RCLCPP_INFO(
      get_logger(), "[F6K-SUBTASK-DEQUEUE] drone=%u queue_size=%zu",
      drone_id, ready_drones_.size());
  }

  void WakeWaitingReservationTasks()
  {
    bool changed = false;
    for (auto & task : tasks_) {
      if (task.state != mission_msgs::msg::TaskState::WAITING ||
        task.detail.rfind("waiting_reservation", 0U) != 0U || task.assigned_drone_id == 0U)
      {
        continue;
      }
      task.state = mission_msgs::msg::TaskState::RUNNING;
      task.detail = "Reserva liberada; pendiente de nueva subtarea";
      ++task.state_revision;
      changed = true;
      EnqueueReadyDrone(task.assigned_drone_id, "reservation_released");
    }
    if (changed) {
      PublishTaskStates();
      PublishFlow("reservation_to_planning", "RESERVATION_RELEASED");
    }
  }

  bool ReleaseAutonomousTrajectoryMonitor(
    std::uint32_t drone_id, const std::string & command_id, const std::string & reason,
    std::uint8_t terminal_state)
  {
    const auto runtime = execution_runtime_by_drone_.find(drone_id);
    if (runtime == execution_runtime_by_drone_.end() || !runtime->second.autonomous_workflow ||
      runtime->second.autonomous_command_id != command_id)
    {
      return false;
    }
    const auto trajectory_id = runtime->second.trajectory_id;
    const auto reservation_id = runtime->second.reservation_id;
    const auto plan = execution_plans_.find(trajectory_id);
    if (plan != execution_plans_.end()) {
      plan->second.execution_state = terminal_state;
      plan->second.execution_detail = reason.empty() ?
        "Terminal autonomo recibido" : reason;
      planned_route_publisher_->publish(plan->second);
      execution_plans_.erase(plan);
    }
    if (!reservation_id.empty() && !reservation_overlay_.Release(reservation_id)) {
      RCLCPP_ERROR(
        get_logger(),
        "[F6J-AUTONOMOUS-RESERVATION-RELEASE-REJECTED] drone=%u command=%s reservation_id=%s",
        drone_id, command_id.c_str(), reservation_id.c_str());
      return false;
    }
    runtime->second = ExecutionRuntime{};
    PublishVoxelMap();
    RCLCPP_INFO(
      get_logger(),
      "[F6J-AUTONOMOUS-MONITOR-FINAL] drone=%u command=%s trajectory_id=%s reason=%s",
      drone_id, command_id.c_str(), trajectory_id.c_str(), reason.c_str());
    WakeWaitingReservationTasks();
    return true;
  }

  void ReleaseExecutionRuntime(std::uint32_t drone_id, const std::string & reason)
  {
    const auto runtime_it = execution_runtime_by_drone_.find(drone_id);
    if (runtime_it == execution_runtime_by_drone_.end()) {
      return;
    }
    const auto reservation_id = runtime_it->second.reservation_id;
    const bool released = reservation_id.empty() || reservation_overlay_.Release(reservation_id);
    if (!released) {
      RCLCPP_ERROR(
        get_logger(), "[F6J-RESERVATION-RELEASE-REJECTED] drone=%u reservation_id=%s",
        drone_id, reservation_id.c_str());
      return;
    }
    ExecutionRuntime cleared_runtime;
    runtime_it->second = std::move(cleared_runtime);
    PublishVoxelMap();
    RCLCPP_INFO(
      get_logger(), "[F6J-RESERVATION-RELEASE] drone=%u reservation_id=%s reason=%s",
      drone_id, reservation_id.c_str(), reason.c_str());
    WakeWaitingReservationTasks();
    if (fiducial_interrupt_task_by_drone_.count(drone_id) != 0U) {
      FinalizeFiducialInterrupt(drone_id, reason);
      DispatchNextPendingPlan();
      return;
    }
    EnqueueReadyDrone(drone_id, reason);
    DispatchNextPendingPlan();
  }

  void HandleTaskReport(mission_msgs::msg::TaskReport::ConstSharedPtr report)
  {
    if (!report || report->mission_id != mission_id_ ||
      report->config_revision != geometry_data_.config_revision)
    {
      return;
    }
    const auto task = std::find_if(
      tasks_.begin(), tasks_.end(), [report](const auto & candidate) {
        return candidate.task_id == report->task_id;
      });
    if (task == tasks_.end() || task->assigned_drone_id != report->drone_id ||
      task->state_revision != report->expected_state_revision ||
      report->state != mission_msgs::msg::TaskState::ASSIGNED)
    {
      RCLCPP_WARN(
        get_logger(), "[F6F-TASK-REPORT-REJECT] task=%s drone=%u",
        report->task_id.c_str(), report->drone_id);
      return;
    }
    task->state = mission_msgs::msg::TaskState::RUNNING;
    ++task->state_revision;
    task->detail = "Barrido de fachada: espera inspeccion depth del corredor";
    PublishTaskStates();
    PublishFlow("task_manager_to_task_worker", "TASK_ACCEPTED");
    EnqueueReadyDrone(report->drone_id, "task_accepted");
  }

  void PublishFlow(const std::string & edge, const std::string & event)
  {
    if (!flow_publisher_) {
      return;
    }
    std_msgs::msg::String message;
    std::ostringstream json;
    json << "{\"edge_id\":\"" << edge << "\",\"event\":\"" << event
         << "\",\"detail\":\"" << event << "\",\"mission_id\":\""
         << mission_id_ << "\"}";
    message.data = json.str();
    flow_publisher_->publish(message);
  }

  void PublishGeometryFlow()
  {
    if (!flow_publisher_) {
      return;
    }
    std_msgs::msg::String message;
    std::ostringstream json;
    json << "{\"edge_id\":\"task_worker_geometry\","
         << "\"event\":\"MISSION_GEOMETRY_READY\",\"detail\":\""
         << geometry_data_.regions.size() << " regiones sin asignar\",\"regions\":[";
    for (std::size_t index = 0; index < geometry_data_.regions.size(); ++index) {
      const auto & region = geometry_data_.regions[index];
      if (index != 0U) {
        json << ',';
      }
      json << "{\"id\":\"" << region.region_id << "\",\"level\":"
           << region.level_index << ",\"side\":\"" << task_lib::BaseSideName(region.side)
           << "\",\"min\":[" << region.bounds.min.x << ',' << region.bounds.min.y << ','
           << region.bounds.min.z << "],\"max\":[" << region.bounds.max.x << ','
           << region.bounds.max.y << ',' << region.bounds.max.z << "]}";
    }
    json << "]}";
    message.data = json.str();
    flow_publisher_->publish(message);
  }

  void PublishPlanningFlow(
    const std::string & event, std::uint32_t drone_id, const std::string & reason,
    std::uint64_t map_revision, double latency_ms, std::uint64_t expanded,
    const std::string & trajectory_id)
  {
    if (!flow_publisher_) {
      return;
    }
    std_msgs::msg::String message;
    std::ostringstream json;
    json << "{\"edge_id\":\"planning_to_route\",\"event\":\"" << event
         << "\",\"detail\":\"" << reason << "\",\"mission_id\":\""
         << mission_id_ << "\",\"drone_id\":" << drone_id
         << ",\"map_revision\":" << map_revision << ",\"latency_ms\":" << latency_ms
         << ",\"expanded\":" << expanded << ",\"trajectory_id\":\""
         << trajectory_id << "\"}";
    message.data = json.str();
    flow_publisher_->publish(message);
  }

  void PublishArchitecture(const std::string & edge, const std::string & event)
  {
    if (!architecture_publisher_) {
      return;
    }
    std_msgs::msg::String message;
    const bool mission_flow_edge = edge == "task_server_to_sim_mission_flow";
    const std::string interface = mission_flow_edge ? "/mission/flow_events" :
      (edge == "task_manager_to_task_server" ? "/mission/register_drone" :
      (edge == "task_server_to_task_manager_inspect_facade" ?
      "/dron_X/inspect_facade" :
      (edge == "orbslam3_multi_to_task_server_sparse" ? "/global_sparse_cloud" :
      (edge == "task_server_to_task_manager" ? "/mission/task_states" :
      (edge == "task_server_to_gui_voxels" ? "/mission/voxel_map" :
      (edge == "task_server_to_gui_planned_route" ? "/mission/planned_routes" :
      "/mission/geometry"))))));
    const std::string interface_kind = mission_flow_edge ? "debug_topic" :
      (edge == "task_server_to_task_manager_inspect_facade" ? "service" :
      (edge == "orbslam3_multi_to_task_server_sparse" ? "topic_subscribe" : "topic_publish"));
    std::ostringstream json;
    json << "{\"kind\":\"architecture_activity\",\"edge_id\":\"" << edge
         << "\",\"source\":\"task_server\",\"interface\":\"" << interface
         << "\",\"interface_kind\":\"" << interface_kind << "\",\"timestamp\":"
         << now().seconds() << ",\"detail\":\"" << event << "\"}";
    message.data = json.str();
    architecture_publisher_->publish(message);
  }

  static constexpr std::uint32_t kProtocolVersion = 1U;
  static constexpr std::uint32_t kGeneratorVersion = 1U;
  static constexpr const char * kGeneratorId = "lib_tray";
  std::string mission_id_;
  std::string mission_frame_;
  double voxel_size_ = 0.25;
  float voxel_occupied_score_threshold_ = 0.4F;
  float visual_target_min_score_ = 0.2F;
  float visual_target_max_score_ = 0.6F;
  float sparse_plane_min_map_point_score_ = 0.2F;
  bool depth_evidence_enabled_ = false;
  double depth_min_confidence_ = 0.25;
  std::size_t depth_min_support_points_ = 20U;
  std::size_t depth_max_points_per_observation_ = 512U;
  double depth_far_free_distance_m_ = 5.0;
  double depth_direct_surface_max_incidence_deg_ = 30.0;
  std::size_t sparse_plane_min_inliers_ = 20U;
  double sparse_plane_residual_voxels_ = 0.5;
  std::size_t sparse_plane_max_samples_ = 1024U;
  float sparse_ray_free_min_score_ = 0.5F;
  task_lib::Vec3 free_body_half_extent_;
  bool flow_enabled_ = false;
  std::string pose_source_;
  task_lib::DStarLiteParameters planner_parameters_;
  std::int64_t planning_coarse_voxel_factor_ = 4;
  bool execute_facade_sweeps_ = false;
  std::atomic_bool execution_enabled_{true};
  std::atomic_bool execution_gate_update_pending_{false};
  double execution_nominal_velocity_mps_ = 0.8;
  double execution_timing_factor_ = 2.0;
  double trajectory_waypoint_min_separation_m_ = 1.0;
  double trajectory_min_segment_duration_sec_ = 8.0;
  std::int64_t extra_obstacle_clearance_voxels_ = 1;
  std::int64_t start_escape_radius_voxels_ = 4;
  std::int64_t unknown_fallback_free_radius_voxels_ = 8;
  std::int64_t facade_coverage_offset_voxels_ = 2;
  std::int64_t depth_coverage_neighbor_sections_ = 2;
  std::int64_t voxel_worker_coalesce_ms_ = 100;
  double reservation_sweep_sample_step_voxels_ = 0.5;
  task_lib::FacadePreferences facade_preferences_;
  double facade_candidate_step_m_ = 0.25;
  double facade_orientation_tolerance_deg_ = 25.0;
  std::uint32_t facade_max_inspection_failures_ = 3U;
  std::int64_t facade_worker_period_ms_ = 250;
  bool debug_trajectory_diagnostics_ = false;
  bool debug_facade_dstar_failure_ = false;
  std::map<std::string, mission_msgs::msg::TrajectoryPlan> execution_plans_;
  std::map<std::string, PendingExecutionPlan> pending_execution_plans_;
  std::map<std::uint32_t, ExecutionRuntime> execution_runtime_by_drone_;
  std::map<std::uint32_t, mission_msgs::msg::VisualRiskEvent> visual_risk_events_;
  std::map<std::uint32_t, VisualCaution> visual_cautions_;
  std::deque<std::uint32_t> ready_drones_;
  std::set<std::uint32_t> ready_drone_ids_;
  std::set<std::uint32_t> task_available_drone_ids_;
  std::set<std::tuple<std::uint32_t, std::uint64_t, std::int32_t>> seen_fiducials_;
  std::map<std::uint32_t, std::string> fiducial_interrupt_task_by_drone_;
  task_lib::MissionGeometry geometry_data_;
  std::unique_ptr<task_lib::ReversibleVoxelMap> voxel_map_;
  task_lib::ReservationOverlay reservation_overlay_;
  std::map<std::uint32_t, task_lib::DStarLitePlanner> planners_;
  std::map<std::uint32_t, std::string> drone_navigation_profiles_;
  std::map<std::uint32_t, std::string> drone_facade_navigation_profiles_;
  std::map<std::uint32_t, task_lib::VoxelKey> drone_inflation_cells_;
  std::map<std::uint32_t, task_lib::VoxelKey> drone_reservation_body_cells_;
  std::set<std::string> navigation_profiles_;
  std::map<std::uint32_t, DronePose> navigation_poses_;
  std::map<std::uint32_t, NavigationOrientation> navigation_orientations_;
  bool sparse_bootstrap_received_ = false;
  std::optional<std::vector<task_lib::SparseEvidence>> pending_sparse_snapshot_;
  std::map<std::string, task_lib::SparseEvidence> pending_sparse_upserts_;
  std::set<std::string> pending_sparse_deletes_;
  std::map<KeyframeIdentity, KeyframeFreeEvidence> keyframe_free_evidence_;
  std::map<KeyframeIdentity, DepthKeyframeEvidence> depth_keyframe_evidence_;
  std::deque<PendingDepthIntegration> depth_integrations_;
  std::map<std::string, AutonomousDepthResult> autonomous_depth_results_;
  std::map<std::string, AutonomousWorkflowContext> autonomous_workflow_contexts_;
  std::map<std::string, PendingAutonomousTrajectoryMonitor>
  pending_autonomous_trajectory_monitors_;
  std::map<std::string, PendingDepthContinuation> pending_depth_continuations_;
  std::map<std::string, CoverageSourceContext> coverage_source_contexts_;
  task_server::EvidenceDatabase evidence_database_;
  std::unique_ptr<task_server::KeyframeEvidenceWorker> keyframe_evidence_worker_;
  std::unique_ptr<task_server::DepthIntegrationWorker> depth_integration_worker_;
  std::unique_ptr<task_server::VoxelMapBuilder> voxel_map_builder_;
  std::map<std::string, FacadeTaskRuntime> facade_runtime_by_task_;
  KeyframeIdentity free_requery_cursor_;
  bool free_requery_cursor_valid_ = false;
  KeyframeIdentity depth_requery_cursor_;
  bool depth_requery_cursor_valid_ = false;
  std::uint64_t plan_revision_ = 0U;
  std::uint64_t autonomous_command_revision_ = 0U;
  std::vector<mission_msgs::msg::TaskState> tasks_;
  std::set<std::uint32_t> eligible_drones_;
  std::unique_ptr<task_server::DroneRegistry> registry_;
  rclcpp::Publisher<mission_msgs::msg::MissionGeometry>::SharedPtr geometry_publisher_;
  rclcpp::Publisher<mission_msgs::msg::DroneRegistry>::SharedPtr registry_publisher_;
  rclcpp::Publisher<mission_msgs::msg::TaskStateArray>::SharedPtr task_state_publisher_;
  rclcpp::Publisher<mission_msgs::msg::VoxelMap>::SharedPtr voxel_map_publisher_;
  rclcpp::Publisher<mission_msgs::msg::TrajectoryPlan>::SharedPtr planned_route_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr flow_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr architecture_publisher_;
  rclcpp::Service<mission_msgs::srv::RegisterDrone>::SharedPtr register_service_;
  rclcpp::Service<mission_msgs::srv::PlanRoute>::SharedPtr plan_route_service_;
  rclcpp::Service<ReportAutonomousResult>::SharedPtr autonomous_result_service_;
  rclcpp::TimerBase::SharedPtr depth_integration_worker_timer_;
  rclcpp::TimerBase::SharedPtr point_selection_worker_timer_;
  rclcpp::TimerBase::SharedPtr trajectory_planning_worker_timer_;
  rclcpp::TimerBase::SharedPtr active_trajectory_monitor_worker_timer_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr execution_toggle_service_;
  std::map<std::uint32_t, rclcpp_action::Client<ExecuteTrajectory>::SharedPtr>
  execute_trajectory_clients_;
  std::map<std::uint32_t, rclcpp::Client<mission_msgs::srv::InspectFacade>::SharedPtr>
  inspect_facade_clients_;
  std::map<std::uint32_t, rclcpp::Client<SubmitAutonomousCommand>::SharedPtr>
  autonomous_command_clients_;
  rclcpp::Client<orbslam3_msgs::srv::GetGlobalKeyFramePose>::SharedPtr global_pose_client_;
  rclcpp::Subscription<mission_msgs::msg::TaskReport>::SharedPtr task_report_subscription_;
  rclcpp::Subscription<mission_msgs::msg::VisualRiskEvent>::SharedPtr visual_risk_subscription_;
  rclcpp::Subscription<mission_msgs::msg::KeyframeSparseEvidenceDelta>::SharedPtr
    keyframe_sparse_evidence_subscription_;
  rclcpp::Subscription<mission_msgs::msg::SafetyEvent>::SharedPtr safety_event_subscription_;
  rclcpp::Subscription<mission_msgs::msg::FiducialPrimaryObservation>::SharedPtr
    fiducial_primary_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sparse_subscription_;
  rclcpp::Subscription<mission_msgs::msg::GlobalSparseMapDelta>::SharedPtr
    sparse_delta_subscription_;
  std::vector<rclcpp::Subscription<orbslam3_msgs::msg::NavigationState>::SharedPtr>
  navigation_subscriptions_;
  std::vector<rclcpp::Subscription<orbslam3_msgs::msg::GlobalKeyFramePose>::SharedPtr>
  global_keyframe_pose_subscriptions_;
  rclcpp::TimerBase::SharedPtr flow_republish_timer_;
  rclcpp::TimerBase::SharedPtr free_pose_requery_timer_;
  rclcpp::TimerBase::SharedPtr facade_worker_timer_;
  rclcpp::TimerBase::SharedPtr execution_gate_apply_timer_;
  rclcpp::TimerBase::SharedPtr voxel_worker_timer_;
  rclcpp::CallbackGroup::SharedPtr map_callback_group_;
  rclcpp::CallbackGroup::SharedPtr execution_control_callback_group_;
  rclcpp::CallbackGroup::SharedPtr workflow_ingress_callback_group_;
  rclcpp::CallbackGroup::SharedPtr workflow_dispatch_callback_group_;
  rclcpp::CallbackGroup::SharedPtr voxel_materialization_callback_group_;
  rclcpp::CallbackGroup::SharedPtr trajectory_monitor_callback_group_;
  task_server::WorkflowScheduler workflow_scheduler_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<TaskServerNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
    node.reset();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("task_server"),
      "[F6A-MISSION-CONFIG-INVALID] %s", error.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
