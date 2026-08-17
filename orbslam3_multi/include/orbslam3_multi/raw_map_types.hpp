#pragma once

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <cstdint>
#include <array>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace orbslam3_multi
{

struct RawSubmapId
{
  uint32_t drone_id = 0;
  uint64_t map_epoch = 0;

  bool operator<(const RawSubmapId & other) const
  {
    return drone_id == other.drone_id ? map_epoch < other.map_epoch : drone_id < other.drone_id;
  }

  bool operator==(const RawSubmapId & other) const
  {
    return drone_id == other.drone_id && map_epoch == other.map_epoch;
  }
};

struct RawKeyFrameId
{
  uint32_t drone_id = 0;
  uint64_t map_epoch = 0;
  uint64_t local_kf_id = 0;

  bool operator<(const RawKeyFrameId & other) const
  {
    if (drone_id != other.drone_id) {
      return drone_id < other.drone_id;
    }
    return map_epoch == other.map_epoch ? local_kf_id < other.local_kf_id : map_epoch <
           other.map_epoch;
  }

  bool operator==(const RawKeyFrameId & other) const
  {
    return drone_id == other.drone_id && map_epoch == other.map_epoch &&
           local_kf_id == other.local_kf_id;
  }
};

struct RawMapPointId
{
  uint32_t drone_id = 0;
  uint64_t map_epoch = 0;
  uint64_t local_mp_id = 0;

  bool operator<(const RawMapPointId & other) const
  {
    if (drone_id != other.drone_id) {
      return drone_id < other.drone_id;
    }
    return map_epoch == other.map_epoch ? local_mp_id < other.local_mp_id : map_epoch <
           other.map_epoch;
  }

  bool operator==(const RawMapPointId & other) const
  {
    return drone_id == other.drone_id && map_epoch == other.map_epoch &&
           local_mp_id == other.local_mp_id;
  }
};

struct RawLoopSemanticRevision
{
  uint64_t raw_revision = 0;
  uint64_t appearance_revision = 0;
  uint64_t geometry_revision = 0;
  uint64_t validation_revision = 0;
};

struct RawDatabaseStats
{
  uint64_t journal_entries = 0;
  uint64_t delta_entries = 0;
  uint64_t submaps = 0;
  uint64_t keyframes = 0;
  uint64_t mappoints = 0;
  uint64_t last_arrival_id = 0;
  uint64_t fiducial_observations = 0;

  bool operator==(const RawDatabaseStats & other) const
  {
    return journal_entries == other.journal_entries && delta_entries == other.delta_entries &&
           submaps == other.submaps && keyframes == other.keyframes &&
           mappoints == other.mappoints && last_arrival_id == other.last_arrival_id &&
           fiducial_observations == other.fiducial_observations;
  }
};

struct RecordedFiducialObservation
{
  uint64_t arrival_id = 0;
  RawKeyFrameId keyframe_id;
  int32_t fiducial_id = 0;
  uint64_t fiducial_visit_id = 0;
  geometry_msgs::msg::Pose world_T_camera_target;
  double keyframe_stamp_sec = 0.0;
  double observation_stamp_sec = 0.0;
  double association_dt_sec = 0.0;
  double distance_to_fiducial_m = 0.0;
  std::string source;
  std::string quality;
};

struct RawAssociationChange
{
  RawKeyFrameId keyframe_id;
  std::vector<RawMapPointId> added_mappoint_ids;
  std::vector<RawMapPointId> removed_mappoint_ids;
};

enum class RawKeyFramePoseChangeKind
{
  New,
  PoseUpdated,
  Invalidated,
};

struct RawKeyFramePoseInput
{
  RawKeyFrameId id;
  uint64_t raw_revision = 0;
  geometry_msgs::msg::Pose local_pose;
  bool active = true;
};

struct RawKeyFramePoseChange
{
  RawKeyFramePoseChangeKind kind = RawKeyFramePoseChangeKind::New;
  RawKeyFramePoseInput keyframe;
};

struct RawSubmapPoseSnapshot
{
  RawSubmapId submap_id;
  uint64_t submap_revision = 0;
  std::vector<RawKeyFramePoseInput> keyframes;
};

struct RawSubmapEntityIds
{
  std::vector<RawKeyFrameId> keyframe_ids;
  std::vector<RawMapPointId> mappoint_ids;
};

struct RawMapPointScoreInput
{
  uint32_t observations_count = 0;
  float found_ratio = 0.0F;
  bool descriptor_valid = false;
  bool is_bad = false;
};

struct RawCameraCalibration
{
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  uint32_t image_width = 0;
  uint32_t image_height = 0;

  bool IsValid() const
  {
    return fx > 0.0 && fy > 0.0 && image_width > 0U && image_height > 0U;
  }
};

struct RawFusionMapPointInput
{
  RawMapPointId id;
  uint64_t raw_revision = 0;
  geometry_msgs::msg::Point position;
  std::array<uint8_t, 32> descriptor{};
  uint32_t observations_count = 0;
  uint64_t reference_keyframe_id = 0;
  std::vector<uint64_t> observer_keyframe_ids;
  bool is_bad = false;
};

struct RawBuilderKeyFrameInput
{
  geometry_msgs::msg::Pose local_pose;
  bool is_bad = false;
};

struct RawBuilderMapPointInput
{
  geometry_msgs::msg::Point position;
  uint64_t reference_keyframe_id = 0;
  std::vector<uint64_t> observer_keyframe_ids;
  bool is_bad = false;
};

struct RawBuilderSnapshot
{
  std::map<RawKeyFrameId, RawBuilderKeyFrameInput> keyframes;
  std::map<RawMapPointId, RawBuilderMapPointInput> mappoints;
  std::set<RawMapPointId> requested_mappoint_ids;
};

struct RawInsertResult
{
  uint64_t arrival_id = 0;
  RawSubmapId submap_id;
  bool full_snapshot = false;
  bool new_submap = false;
  bool has_material_changes = false;
  bool journal_entry_appended = false;
  bool normalized_delta_appended = false;
  uint64_t submap_revision = 0;

  std::vector<RawKeyFrameId> new_keyframe_ids;
  std::vector<RawKeyFrameId> updated_keyframe_ids;
  std::vector<RawKeyFrameId> removed_keyframe_ids;
  std::vector<RawKeyFrameId> unchanged_keyframe_ids;
  std::vector<RawKeyFrameId> pose_changed_keyframe_ids;
  std::vector<RawKeyFrameId> association_changed_keyframe_ids;
  std::vector<RawKeyFrameId> covisibility_changed_keyframe_ids;
  std::vector<RawKeyFrameId> invalidated_keyframe_ids;

  std::vector<RawMapPointId> new_mappoint_ids;
  std::vector<RawMapPointId> updated_mappoint_ids;
  std::vector<RawMapPointId> removed_mappoint_ids;
  std::vector<RawMapPointId> unchanged_mappoint_ids;
  std::vector<RawMapPointId> geometry_changed_mappoint_ids;
  std::vector<RawMapPointId> score_input_changed_mappoint_ids;
  std::vector<RawMapPointId> association_changed_mappoint_ids;
  std::vector<RawMapPointId> invalidated_mappoint_ids;

  std::vector<RawAssociationChange> association_changes;
  std::vector<RawKeyFramePoseChange> pose_changes;
  RawDatabaseStats stats;
};

std::string ToString(const RawSubmapId & id);
const char * ToString(RawKeyFramePoseChangeKind kind);

}  // namespace orbslam3_multi
