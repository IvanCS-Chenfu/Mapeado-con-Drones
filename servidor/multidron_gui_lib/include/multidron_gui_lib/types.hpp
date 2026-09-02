#pragma once

#include <QColor>
#include <QQuaternion>
#include <QVector3D>

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace multidron_gui_lib
{

struct SparsePoint
{
  QVector3D position;
  float score = 1.0F;
  std::uint32_t drone_id = 0;
  std::uint64_t map_epoch = 0;
  std::uint64_t source_index = 0;
};

struct KeyframeVisual
{
  std::int32_t marker_id = 0;
  std::string marker_namespace;
  QVector3D position;
  QQuaternion orientation;
  QColor color = QColor(255, 255, 255);
  std::vector<QVector3D> line_points_world;
};

struct DroneState
{
  std::uint32_t drone_id = 0;
  std::uint64_t sample_sequence = 0;
  std::uint64_t map_epoch = 0;
  std::uint64_t pose_revision = 0;
  std::uint64_t reference_keyframe_id = 0;
  bool reference_keyframe_valid = false;

  std::int8_t tracking_state = -1;
  std::uint8_t pose_source = 0;
  std::uint8_t global_status = 0;

  bool has_world_pose = false;
  bool lost_or_unavailable = true;
  std::int64_t received_steady_ns = 0;
  QVector3D position;
  QQuaternion orientation;
  double yaw_rad = 0.0;
};

struct FiducialObject
{
  std::int32_t object_id = 0;
  std::string shape = "box";
  QVector3D size_m;
  QVector3D position;
  QQuaternion orientation;
  std::vector<std::int32_t> tag_ids;
};

struct TrajectoryVisual
{
  std::uint32_t drone_id = 0;
  std::string task_id;
  std::string trajectory_id;
  std::uint64_t plan_revision = 0;
  std::uint64_t map_revision = 0;
  std::uint64_t alignment_revision = 0;
  std::vector<QVector3D> samples_world;
};

enum class VoxelState : std::uint8_t
{
  Unknown = 0,
  Free = 1,
  Occupied = 2,
};

struct VoxelVisual
{
  std::int64_t ix = 0;
  std::int64_t iy = 0;
  std::int64_t iz = 0;
  QVector3D center_world;
  float size_m = 0.0F;
  VoxelState state = VoxelState::Unknown;
  float score = 0.0F;
};

struct TaskVisual
{
  std::uint32_t drone_id = 0;
  std::string task_id;
  std::string task_type;
  std::string state;
  std::string detail;
};

struct MissionRegionVisual
{
  std::string region_id;
  std::uint32_t level_index = 0;
  std::string side;
  QVector3D min_world;
  QVector3D max_world;
};

using SparsePointVector = std::vector<SparsePoint>;
using KeyframeVector = std::vector<KeyframeVisual>;
using DroneStateMap = std::map<std::uint32_t, DroneState>;
using FiducialVector = std::vector<FiducialObject>;
using TrajectoryMap = std::map<std::uint32_t, TrajectoryVisual>;
using VoxelVector = std::vector<VoxelVisual>;
using TaskStateMap = std::map<std::uint32_t, TaskVisual>;
using MissionRegionVector = std::vector<MissionRegionVisual>;

struct GuiSnapshot
{
  std::uint64_t generation = 0;
  std::shared_ptr<const SparsePointVector> sparse_points;
  std::shared_ptr<const KeyframeVector> keyframes;
  std::shared_ptr<const DroneStateMap> drones;
  std::shared_ptr<const FiducialVector> fiducials;
  std::shared_ptr<const TrajectoryMap> trajectories;
  std::shared_ptr<const VoxelVector> voxels;
  std::shared_ptr<const TaskStateMap> tasks;
  std::shared_ptr<const MissionRegionVector> mission_regions;
};

}  // namespace multidron_gui_lib
