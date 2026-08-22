#pragma once

#include "orbslam3_multi/raw_map_types.hpp"

#include "geometry_msgs/msg/pose.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace orbslam3_multi
{

enum class PoseSourceKind
{
  SubmapAnchorDerived,
  FiducialControlDerived,
  FiducialAccepted,
  FiducialOptimized,
  LoopAnchorDerived,
  LoopOptimized,
};

enum class PoseCommitStatus
{
  NoChanges,
  Unanchored,
  Applied,
  PreservedAccepted,
  AlreadyAnchored,
  MissingRawSubmap,
  RevisionConflict,
  HardConstraintViolation,
  AtomicBatchConflict,
};

struct GlobalPoseRecord
{
  RawKeyFrameId keyframe_id;
  geometry_msgs::msg::Pose world_pose;
  geometry_msgs::msg::Pose raw_world_pose;
  geometry_msgs::msg::Pose correction_pose;
  bool active = true;
  uint64_t pose_revision = 0;
  PoseSourceKind source_kind = PoseSourceKind::SubmapAnchorDerived;
  uint64_t source_commit_id = 0;
  uint64_t source_task_id = 0;
  uint64_t parent_commit_id = 0;
  uint64_t base_raw_revision = 0;
  bool hard_fiducial = false;
};

struct AcceptedPoseUpdate
{
  RawKeyFrameId keyframe_id;
  geometry_msgs::msg::Pose world_pose;
  uint64_t base_raw_revision = 0;
  uint64_t expected_pose_revision = 0;
  bool mark_hard_fiducial = false;
};

struct LoopAnchorDependencySnapshot
{
  RawSubmapId child_submap_id;
  RawSubmapId parent_submap_id;
  RawKeyFrameId child_control_keyframe_id;
  RawKeyFrameId parent_control_keyframe_id;
  geometry_msgs::msg::Pose parent_control_T_child_control;
  uint64_t source_commit_id = 0;
};

struct PoseChangeSet
{
  PoseCommitStatus status = PoseCommitStatus::NoChanges;
  RawSubmapId submap_id;
  uint64_t source_task_id = 0;
  uint64_t commit_id = 0;
  uint64_t store_revision_before = 0;
  uint64_t store_revision_after = 0;
  std::vector<RawKeyFrameId> created_ids;
  std::vector<RawKeyFrameId> updated_ids;
  std::vector<RawKeyFrameId> invalidated_ids;
  std::vector<RawKeyFrameId> preserved_ids;
  std::vector<RawKeyFrameId> skipped_unanchored_ids;
  std::vector<RawKeyFrameId> hard_fiducial_ids;
  std::vector<RawKeyFrameId> control_propagated_ids;
  std::vector<RawKeyFrameId> conflict_ids;
};

struct GlobalPoseStoreStats
{
  uint64_t store_revision = 0;
  uint64_t commits = 0;
  uint64_t anchors = 0;
  uint64_t poses = 0;
  uint64_t active_poses = 0;
  uint64_t inactive_poses = 0;
  uint64_t hard_fiducial_keyframes = 0;
};

struct LoopAnchorBatchEntry
{
  RawSubmapPoseSnapshot snapshot;
  geometry_msgs::msg::Pose world_T_local;
  RawKeyFrameId loop_control_keyframe_id;
  std::optional<RawSubmapId> parent_submap_id;
  std::optional<RawKeyFrameId> parent_control_keyframe_id;
};

struct HardCorridorReference
{
  geometry_msgs::msg::Pose world_pose;
  double alpha = 0.0;
  uint64_t reference_revision = 0;
};

struct LoopAnchorBatchResult
{
  PoseCommitStatus status = PoseCommitStatus::NoChanges;
  uint64_t source_task_id = 0;
  uint64_t commit_id = 0;
  uint64_t store_revision_before = 0;
  uint64_t store_revision_after = 0;
  std::vector<PoseChangeSet> submap_changes;
  std::vector<RawSubmapId> anchored_submaps;
  std::vector<RawKeyFrameId> dirty_keyframe_ids;
};

struct AcceptedSubmapPoseBatch
{
  RawSubmapId submap_id;
  std::vector<AcceptedPoseUpdate> updates;
  std::optional<RawKeyFrameId> continuation_control;
};

struct AcceptedPoseBatchResult
{
  PoseCommitStatus status = PoseCommitStatus::NoChanges;
  std::string detail;
  size_t rebased_skipped_controls = 0;
  size_t rebased_inactive_controls = 0;
  uint64_t source_task_id = 0;
  uint64_t commit_id = 0;
  uint64_t store_revision_before = 0;
  uint64_t store_revision_after = 0;
  std::vector<PoseChangeSet> submap_changes;
  std::vector<RawKeyFrameId> dirty_keyframe_ids;
  std::vector<RawKeyFrameId> propagated_keyframe_ids;
};

const char * ToString(PoseSourceKind kind);
const char * ToString(PoseCommitStatus status);

}  // namespace orbslam3_multi
