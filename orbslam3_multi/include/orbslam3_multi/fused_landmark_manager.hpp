#pragma once

#include "orbslam3_multi/fused_landmark_types.hpp"
#include "orbslam3_multi/global_pose_store.hpp"
#include "orbslam3_multi/loop_pipeline.hpp"

#include <mutex>

namespace orbslam3_multi
{

struct FusedLandmarkConfig
{
  double max_track_dispersion_m = 0.50;
  double visibility_depth_tolerance_m = 0.30;
  uint32_t visibility_cell_size_px = 8;
  float inlier_reward = 0.04F;
  float visible_miss_penalty = -0.01F;
  float foreground_penalty = -0.03F;
};

class FusedLandmarkManager
{
public:
  void Configure(const FusedLandmarkConfig & config);
  FusionPrepareResult PrepareFusion(
    const LoopTaskComputation & computation,
    const RawMapDatabase & raw_database,
    const GlobalPoseStore & pose_store,
    const LandmarkScoreManager & score_manager) const;
  FusionApplyResult ApplyPatch(const FusionPatch & patch);
  bool RollbackPatch(const FusionRollbackPatch & patch);

  std::optional<FusedTrackId> GetTrackIdForMember(const RawMapPointId & id) const;
  std::optional<FusedLandmarkTrack> GetTrack(FusedTrackId id) const;
  std::vector<FusedLandmarkTrack> GetTracks() const;
  std::vector<FusedLandmarkScoreUpdate> BuildScoreUpdatesForMembers(
    const std::vector<RawMapPointId> & member_ids,
    const LandmarkScoreManager & score_manager) const;
  FusedLandmarkStats GetStats() const;

private:
  mutable std::mutex mutex_;
  std::map<FusedTrackId, FusedLandmarkTrack> tracks_;
  std::map<RawMapPointId, FusedTrackId> member_to_track_;
  FusedTrackId next_track_id_ = 1;
  uint64_t revision_ = 0;
  FusedLandmarkConfig config_;
};

}  // namespace orbslam3_multi
