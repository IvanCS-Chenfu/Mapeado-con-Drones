#pragma once

#include "orbslam3_multi/global_pose_store.hpp"
#include "orbslam3_multi/fused_landmark_manager.hpp"
#include "orbslam3_multi/landmark_score_manager.hpp"
#include "orbslam3_multi/raw_map_database.hpp"

#include "geometry_msgs/msg/pose.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <vector>

namespace orbslam3_multi
{

struct GlobalSparsePoint
{
  RawMapPointId mappoint_id;
  FusedTrackId fused_track_id = 0;
  RawKeyFrameId associated_keyframe_id;
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  float score = 0.0F;
};

struct GlobalKeyFrameView
{
  RawKeyFrameId keyframe_id;
  geometry_msgs::msg::Pose world_pose;
};

struct GlobalMapBuildResult
{
  bool changed = false;
  uint64_t publication_revision = 0;
  uint64_t pose_revision = 0;
  uint64_t score_revision = 0;
  uint64_t fusion_revision = 0;
  size_t dirty_keyframes = 0;
  size_t dirty_mappoints = 0;
  size_t deferred_unanchored_submaps = 0;
  size_t deferred_unanchored_keyframes = 0;
  size_t deferred_unanchored_mappoints = 0;
  size_t backfilled_submaps = 0;
  size_t backfilled_keyframes = 0;
  size_t backfilled_mappoints = 0;
  size_t recalculated_keyframes = 0;
  size_t recalculated_mappoints = 0;
  size_t recalculated_fused_tracks = 0;
  size_t skipped_unanchored = 0;
  size_t skipped_bad = 0;
  size_t skipped_invalid = 0;
  size_t skipped_without_world_keyframe = 0;
  size_t reference_associations = 0;
  size_t fallback_associations = 0;
  size_t fallback_submap_points = 0;
  std::vector<GlobalSparsePoint> points;
  std::vector<GlobalKeyFrameView> keyframes;
};

class GlobalMapBuilder
{
public:
  void MarkRawChanges(const RawInsertResult & changes);
  void MarkPoseChanges(const PoseChangeSet & changes);
  void MarkScoreChanges(const ScoreChangeSet & changes);
  void MarkFusionChanges(const FusionChangeSet & changes);

  GlobalMapBuildResult Update(
    const RawMapDatabase & raw_database,
    const GlobalPoseStore & pose_store,
    const LandmarkScoreManager & score_manager,
    const FusedLandmarkManager & fused_manager);

private:
  struct CachedPoint
  {
    GlobalSparsePoint point;
    bool used_reference_keyframe = false;
  };

  struct CachedProjection
  {
    geometry_msgs::msg::Pose local_pose;
    geometry_msgs::msg::Pose world_pose;
    Eigen::Matrix4d world_T_local = Eigen::Matrix4d::Identity();
  };

  bool EnsureKeyFrame(
    const RawKeyFrameId & id,
    const RawBuilderSnapshot & raw_snapshot,
    const GlobalPoseStore & pose_store,
    GlobalMapBuildResult * result);
  bool IsUsableObserver(
    const RawKeyFrameId & id,
    const RawBuilderMapPointInput & mappoint,
    const RawBuilderSnapshot & raw_snapshot,
    const GlobalPoseStore & pose_store,
    GlobalMapBuildResult * result);
  std::optional<std::pair<RawKeyFrameId, bool>> SelectObserver(
    const RawMapPointId & id,
    const RawBuilderMapPointInput & mappoint,
    const RawBuilderSnapshot & raw_snapshot,
    const GlobalPoseStore & pose_store,
    GlobalMapBuildResult * result);
  bool UpdatePoint(
    const RawMapPointId & id,
    const RawBuilderSnapshot & raw_snapshot,
    const GlobalPoseStore & pose_store,
    const LandmarkScoreManager & score_manager,
    const FusedLandmarkManager & fused_manager,
    GlobalMapBuildResult * result);
  bool UpdateFusedTrack(
    FusedTrackId id, const RawMapDatabase & raw_database,
    const GlobalPoseStore & pose_store,
    const LandmarkScoreManager & score_manager,
    const FusedLandmarkManager & fused_manager,
    GlobalMapBuildResult * result);
  bool RemovePoint(const RawMapPointId & id);
  bool RemoveKeyFrame(const RawKeyFrameId & id);
  void StorePoint(CachedPoint point);
  void PopulateOutput(GlobalMapBuildResult * result) const;

  std::map<RawKeyFrameId, GlobalKeyFrameView> keyframe_world_cache_;
  std::map<RawKeyFrameId, CachedProjection> keyframe_projection_cache_;
  std::vector<std::optional<CachedPoint>> sparse_point_slots_;
  std::vector<size_t> free_point_slots_;
  std::map<RawMapPointId, size_t> point_slot_by_id_;
  std::map<RawKeyFrameId, std::set<RawMapPointId>> keyframe_to_mappoints_;
  std::map<FusedTrackId, CachedPoint> fused_point_cache_;
  std::map<RawKeyFrameId, std::set<FusedTrackId>> keyframe_to_fused_tracks_;
  std::set<RawKeyFrameId> dirty_keyframes_;
  std::set<RawMapPointId> dirty_mappoints_;
  std::set<RawMapPointId> removed_mappoints_;
  std::set<FusedTrackId> dirty_fused_tracks_;
  std::set<FusedTrackId> removed_fused_tracks_;
  std::set<RawSubmapId> deferred_unanchored_submaps_;
  std::set<RawKeyFrameId> usable_keyframes_this_update_;
  std::set<RawKeyFrameId> unusable_keyframes_this_update_;
  uint64_t publication_revision_ = 0;
};

}  // namespace orbslam3_multi
