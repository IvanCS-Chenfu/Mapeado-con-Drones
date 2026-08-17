#pragma once

#include "orbslam3_multi/landmark_score_manager.hpp"
#include "orbslam3_multi/raw_map_types.hpp"

#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace orbslam3_multi
{

using FusedTrackId = uint64_t;

struct FusedLandmarkTrack
{
  FusedTrackId fused_track_id = 0;
  std::set<RawMapPointId> member_mappoint_ids;
  std::set<RawKeyFrameId> observing_keyframes;
  std::set<RawSubmapId> source_submaps;
  std::set<uint32_t> source_drone_ids;
  RawMapPointId representative_member;
  std::array<uint8_t, 32> representative_descriptor{};
  bool descriptor_valid = false;
  bool degraded = false;
  uint64_t support_count = 0;
  uint64_t revision = 0;
  std::set<uint64_t> evidence_ids;
};

enum class FusionPairAction : uint8_t
{
  SameRawNoOp,
  AlreadyFusedNoOp,
  ReinforceTrack,
  CreateTrack,
  AddMember,
  MergeTracks,
  Reject,
};

struct FusionPairResult
{
  RawMapPointId first;
  RawMapPointId second;
  FusionPairAction action = FusionPairAction::Reject;
  FusedTrackId track_id = 0;
  FusedTrackId retired_track_id = 0;
  uint64_t evidence_id = 0;
  std::string reason;
};

struct FusionChangeSet
{
  uint64_t fusion_revision_before = 0;
  uint64_t fusion_revision_after = 0;
  std::vector<FusedTrackId> created_track_ids;
  std::vector<FusedTrackId> updated_track_ids;
  std::vector<FusedTrackId> retired_track_ids;
  std::vector<RawMapPointId> hidden_raw_member_ids;
  std::vector<RawMapPointId> released_raw_member_ids;

  bool HasChanges() const
  {
    return !created_track_ids.empty() || !updated_track_ids.empty() ||
           !retired_track_ids.empty() || !hidden_raw_member_ids.empty() ||
           !released_raw_member_ids.empty();
  }
};

struct FusionRollbackPatch
{
  uint64_t revision_before = 0;
  FusedTrackId next_track_id_before = 1;
  std::map<FusedTrackId, std::optional<FusedLandmarkTrack>> tracks;
  std::map<RawMapPointId, std::optional<FusedTrackId>> member_assignments;
};

struct FusionPatch
{
  uint64_t task_id = 0;
  uint64_t expected_fusion_revision = 0;
  FusedTrackId next_track_id_after = 1;
  std::map<RawMapPointId, uint64_t> expected_raw_revisions;
  std::map<RawKeyFrameId, uint64_t> expected_pose_revisions;
  std::map<FusedTrackId, FusedLandmarkTrack> after_tracks;
  std::set<FusedTrackId> erase_track_ids;
  std::map<RawMapPointId, std::optional<FusedTrackId>> after_member_assignments;
  std::vector<FusionPairResult> pair_results;
  std::vector<LandmarkScoreEvidence> raw_score_evidence;
  std::vector<FusedLandmarkScoreUpdate> fused_score_updates;
  std::vector<FusedTrackId> fused_score_removals;
  size_t visibility_regions_started = 0;
  size_t visibility_regions_completed = 0;
  size_t visibility_projected_points = 0;
  size_t positive_score_events = 0;
  size_t negative_score_events = 0;
  double visibility_elapsed_ms = 0.0;
};

struct FusionPrepareResult
{
  bool ready = false;
  bool no_op = false;
  std::string reason;
  FusionPatch patch;
};

struct FusionApplyResult
{
  bool committed = false;
  bool stale = false;
  std::string reason;
  FusionChangeSet changes;
  FusionRollbackPatch rollback;
};

struct FusedLandmarkStats
{
  uint64_t revision = 0;
  uint64_t tracks = 0;
  uint64_t raw_members = 0;
  uint64_t multi_drone_tracks = 0;
  uint64_t degraded_tracks = 0;
};

const char * ToString(FusionPairAction action);

}  // namespace orbslam3_multi
