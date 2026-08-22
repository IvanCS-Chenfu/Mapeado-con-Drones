#pragma once

#include "orbslam3_multi/global_pose_types.hpp"

#include <map>
#include <mutex>
#include <optional>
#include <vector>

namespace orbslam3_multi
{

class GlobalPoseStore
{
public:
  bool HasSubmapAnchor(const RawSubmapId & submap_id) const;
  std::optional<GlobalPoseRecord> GetPose(const RawKeyFrameId & keyframe_id) const;
  std::map<RawKeyFrameId, GlobalPoseRecord> GetSubmapPoses(
    const RawSubmapId & submap_id) const;
  GlobalPoseStoreStats GetStats() const;
  std::optional<geometry_msgs::msg::Pose> GetSubmapAnchorPose(
    const RawSubmapId & submap_id) const;
  uint64_t GetSubmapAnchorRevision(const RawSubmapId & submap_id) const;
  bool HasLoopDependency(const RawSubmapId & submap_id) const;
  std::optional<LoopAnchorDependencySnapshot> GetLoopDependency(
    const RawSubmapId & submap_id) const;
  std::vector<LoopAnchorDependencySnapshot> GetLoopDependencies() const;
  std::optional<RawKeyFrameId> GetContinuationControl(
    const RawSubmapId & submap_id) const;
  std::optional<HardCorridorReference> GetHardCorridorReference(
    const RawKeyFrameId & keyframe_id) const;

  PoseChangeSet CommitAnchor(
    const RawSubmapPoseSnapshot & snapshot,
    const geometry_msgs::msg::Pose & world_T_local,
    uint64_t source_task_id,
    const std::optional<RawKeyFrameId> & hard_fiducial_keyframe = std::nullopt);

  PoseChangeSet ApplyRawPoseChanges(
    const RawSubmapId & submap_id,
    const std::vector<RawKeyFramePoseChange> & changes,
    uint64_t source_task_id);

  PoseChangeSet CommitAcceptedPoses(
    const RawSubmapId & submap_id,
    const std::vector<AcceptedPoseUpdate> & updates,
    PoseSourceKind source_kind,
    uint64_t source_task_id,
    const std::optional<RawKeyFrameId> & continuation_control = std::nullopt,
    const std::optional<geometry_msgs::msg::Pose> & replacement_world_T_local =
    std::nullopt);

  LoopAnchorBatchResult CommitLoopAnchorBatch(
    const std::vector<LoopAnchorBatchEntry> & entries,
    uint64_t source_task_id);

  AcceptedPoseBatchResult CommitAcceptedPoseBatch(
    const std::vector<AcceptedSubmapPoseBatch> & batches,
    PoseSourceKind source_kind,
    uint64_t source_task_id);

private:
  void RefreshHardCorridorLocked(const RawSubmapId & submap_id);
  struct AnchorRecord
  {
    geometry_msgs::msg::Pose world_T_local;
    uint64_t anchor_revision = 0;
    uint64_t source_commit_id = 0;
    uint64_t source_task_id = 0;
    uint64_t base_raw_revision = 0;
  };

  struct ContinuationRecord
  {
    RawKeyFrameId control_keyframe_id;
    geometry_msgs::msg::Pose world_T_local;
    uint64_t continuation_revision = 0;
    uint64_t source_commit_id = 0;
    uint64_t source_task_id = 0;
    uint64_t base_raw_revision = 0;
  };

  struct LoopDependencyRecord
  {
    RawSubmapId child_submap_id;
    RawSubmapId parent_submap_id;
    RawKeyFrameId child_control_keyframe_id;
    RawKeyFrameId parent_control_keyframe_id;
    geometry_msgs::msg::Pose parent_control_T_child_control;
    geometry_msgs::msg::Pose parent_control_world_pose;
    uint64_t source_commit_id = 0;
  };

  mutable std::mutex mutex_;
  std::map<RawSubmapId, AnchorRecord> anchors_;
  std::map<RawSubmapId, ContinuationRecord> continuations_;
  std::map<RawSubmapId, LoopDependencyRecord> loop_dependencies_;
  std::map<RawKeyFrameId, GlobalPoseRecord> poses_;
  std::map<RawKeyFrameId, HardCorridorReference> hard_corridor_references_;
  uint64_t hard_corridor_revision_ = 0;
  uint64_t store_revision_ = 0;
  uint64_t commit_count_ = 0;
  uint64_t next_commit_id_ = 1;
};

}  // namespace orbslam3_multi
