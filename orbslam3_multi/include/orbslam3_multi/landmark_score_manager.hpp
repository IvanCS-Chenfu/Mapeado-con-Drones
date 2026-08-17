#pragma once

#include "orbslam3_multi/raw_map_database.hpp"

#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace orbslam3_multi
{

struct LandmarkScoreRecord
{
  RawMapPointId mappoint_id;
  float base_score_orb = 0.0F;
  float positive_adjustment = 0.0F;
  float negative_adjustment = 0.0F;
  float score = 0.0F;
  uint64_t record_revision = 0;
  uint64_t positive_evidence = 0;
  uint64_t negative_evidence = 0;
  uint32_t observations_count = 0;
  float found_ratio = 0.0F;
  bool descriptor_valid = false;
  bool is_bad = false;
};

struct ScoreChangeSet
{
  uint64_t score_revision_before = 0;
  uint64_t score_revision_after = 0;
  std::vector<RawMapPointId> created_ids;
  std::vector<RawMapPointId> updated_ids;
  std::vector<RawMapPointId> invalidated_ids;
  std::vector<RawMapPointId> input_updated_ids;
  std::vector<uint64_t> fused_created_ids;
  std::vector<uint64_t> fused_updated_ids;
  std::vector<uint64_t> fused_removed_ids;

  bool HasChanges() const
  {
    return !created_ids.empty() || !updated_ids.empty() || !invalidated_ids.empty() ||
           !fused_created_ids.empty() || !fused_updated_ids.empty() ||
           !fused_removed_ids.empty();
  }

  bool HasStoreChanges() const
  {
    return HasChanges() || !input_updated_ids.empty();
  }
};

enum class LandmarkScoreEvidenceKind : uint8_t
{
  InlierConfirmed,
  ExpectedVisibleMiss,
  ForegroundContradiction,
};

struct LandmarkScoreEvidence
{
  RawMapPointId mappoint_id;
  uint64_t evidence_id = 0;
  float delta = 0.0F;
  LandmarkScoreEvidenceKind kind = LandmarkScoreEvidenceKind::InlierConfirmed;
};

struct FusedLandmarkScoreRecord
{
  uint64_t fused_track_id = 0;
  float score = 0.0F;
  uint64_t record_revision = 0;
};

struct FusedLandmarkScoreUpdate
{
  uint64_t fused_track_id = 0;
  float score = 0.0F;
};

struct ScorePatch
{
  uint64_t expected_score_revision = 0;
  std::vector<LandmarkScoreEvidence> raw_evidence;
  std::vector<FusedLandmarkScoreUpdate> fused_upserts;
  std::vector<uint64_t> fused_removals;
};

struct ScoreRollbackPatch
{
  uint64_t revision_before = 0;
  std::map<RawMapPointId, std::optional<LandmarkScoreRecord>> raw_records;
  std::map<RawMapPointId, std::set<uint64_t>> raw_evidence;
  std::map<uint64_t, std::optional<FusedLandmarkScoreRecord>> fused_records;
};

struct ScoreApplyResult
{
  bool committed = false;
  bool stale = false;
  std::string reason;
  ScoreChangeSet changes;
  ScoreRollbackPatch rollback;
};

struct LandmarkScoreStats
{
  uint64_t score_revision = 0;
  uint64_t tracked_points = 0;
  uint64_t bad_points = 0;
  float score_min = 0.0F;
  float score_mean = 0.0F;
  float score_max = 0.0F;
};

class LandmarkScoreManager
{
public:
  ScoreChangeSet ApplyRawChanges(
    const RawInsertResult & raw_changes,
    const RawMapDatabase & raw_database);

  std::optional<LandmarkScoreRecord> GetScore(const RawMapPointId & id) const;
  std::optional<FusedLandmarkScoreRecord> GetFusedScore(uint64_t track_id) const;
  ScoreApplyResult ApplyPatch(const ScorePatch & patch);
  bool RollbackPatch(const ScoreRollbackPatch & patch);
  LandmarkScoreStats GetStats() const;

  static float ComputeOrbScore(const orbslam3_msgs::msg::OrbMapPoint & mappoint);

private:
  static float ComputeOrbScore(const RawMapPointScoreInput & input);
  static bool DescriptorValid(const orbslam3_msgs::msg::OrbMapPoint & mappoint);
  static bool Equivalent(
    const LandmarkScoreRecord & lhs,
    const LandmarkScoreRecord & rhs);
  static bool OutputEquivalent(
    const LandmarkScoreRecord & lhs,
    const LandmarkScoreRecord & rhs);

  mutable std::mutex mutex_;
  std::map<RawMapPointId, LandmarkScoreRecord> records_;
  std::map<RawMapPointId, std::set<uint64_t>> applied_evidence_;
  std::map<uint64_t, FusedLandmarkScoreRecord> fused_records_;
  uint64_t score_revision_ = 0;
};

}  // namespace orbslam3_multi
