#include "orbslam3_multi/landmark_score_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace orbslam3_multi
{
namespace
{

float ClampScore(float value)
{
  return std::clamp(value, 0.0F, 1.0F);
}

}  // namespace

bool LandmarkScoreManager::DescriptorValid(
  const orbslam3_msgs::msg::OrbMapPoint & mappoint)
{
  return std::any_of(
    mappoint.descriptor.data.begin(), mappoint.descriptor.data.end(),
    [](uint8_t value) {return value != 0U;});
}

float LandmarkScoreManager::ComputeOrbScore(
  const orbslam3_msgs::msg::OrbMapPoint & mappoint)
{
  return ComputeOrbScore(
    RawMapPointScoreInput{
      mappoint.observations_count, mappoint.found_ratio,
      DescriptorValid(mappoint), mappoint.is_bad});
}

float LandmarkScoreManager::ComputeOrbScore(const RawMapPointScoreInput & input)
{
  if (input.is_bad) {
    return 0.0F;
  }
  const float observations = std::min(
    static_cast<float>(input.observations_count) / 8.0F, 1.0F);
  const float found_ratio = std::clamp(input.found_ratio, 0.0F, 1.0F);
  const float descriptor = input.descriptor_valid ? 1.0F : 0.0F;
  return std::clamp(
    0.55F * observations + 0.35F * found_ratio + 0.10F * descriptor,
    0.0F, 1.0F);
}

bool LandmarkScoreManager::Equivalent(
  const LandmarkScoreRecord & lhs,
  const LandmarkScoreRecord & rhs)
{
  return std::fabs(lhs.base_score_orb - rhs.base_score_orb) <= 1e-6F &&
         std::fabs(lhs.score - rhs.score) <= 1e-6F &&
         lhs.observations_count == rhs.observations_count &&
         std::fabs(lhs.found_ratio - rhs.found_ratio) <= 1e-6F &&
         lhs.descriptor_valid == rhs.descriptor_valid &&
         lhs.is_bad == rhs.is_bad;
}

bool LandmarkScoreManager::OutputEquivalent(
  const LandmarkScoreRecord & lhs,
  const LandmarkScoreRecord & rhs)
{
  return std::fabs(lhs.score - rhs.score) <= 1e-6F && lhs.is_bad == rhs.is_bad;
}

ScoreChangeSet LandmarkScoreManager::ApplyRawChanges(
  const RawInsertResult & raw_changes,
  const RawMapDatabase & raw_database)
{
  ScoreChangeSet result;
  std::lock_guard<std::mutex> lock(mutex_);
  result.score_revision_before = score_revision_;

  std::vector<RawMapPointId> candidates = raw_changes.new_mappoint_ids;
  candidates.insert(
    candidates.end(), raw_changes.score_input_changed_mappoint_ids.begin(),
    raw_changes.score_input_changed_mappoint_ids.end());
  std::sort(candidates.begin(), candidates.end());
  candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
  const auto raw_inputs = raw_database.GetMapPointScoreInputs(candidates);

  for (const auto & id : raw_changes.invalidated_mappoint_ids) {
    // RawMapDatabase reports an ORB point becoming bad as removed, while
    // retaining the raw record for traceability. Only erase a score when the
    // raw point has actually disappeared.
    if (!raw_database.GetMapPoint(id).has_value() && records_.erase(id) != 0U) {
      result.invalidated_ids.push_back(id);
    }
  }

  for (size_t index = 0; index < candidates.size(); ++index) {
    const auto & id = candidates[index];
    const auto & raw = raw_inputs[index];
    if (!raw.has_value()) {
      if (records_.erase(id) != 0U) {
        result.invalidated_ids.push_back(id);
      }
      continue;
    }

    LandmarkScoreRecord next;
    next.mappoint_id = id;
    next.base_score_orb = ComputeOrbScore(*raw);
    next.observations_count = raw->observations_count;
    next.found_ratio = raw->found_ratio;
    next.descriptor_valid = raw->descriptor_valid;
    next.is_bad = raw->is_bad;

    const auto existing = records_.find(id);
    if (existing == records_.end()) {
      next.score = next.base_score_orb;
      next.record_revision = 1;
      records_.emplace(id, next);
      result.created_ids.push_back(id);
    } else {
      next.positive_adjustment = existing->second.positive_adjustment;
      next.negative_adjustment = existing->second.negative_adjustment;
      next.positive_evidence = existing->second.positive_evidence;
      next.negative_evidence = existing->second.negative_evidence;
      next.score = ClampScore(
        next.base_score_orb + next.positive_adjustment + next.negative_adjustment);
      if (Equivalent(existing->second, next)) {
        continue;
      }
      const bool output_changed = !OutputEquivalent(existing->second, next);
      next.record_revision = existing->second.record_revision + 1;
      existing->second = next;
      result.input_updated_ids.push_back(id);
      if (output_changed) {
        result.updated_ids.push_back(id);
      }
    }
  }

  if (result.HasChanges()) {
    ++score_revision_;
  }
  result.score_revision_after = score_revision_;
  return result;
}

std::optional<FusedLandmarkScoreRecord> LandmarkScoreManager::GetFusedScore(
  uint64_t track_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = fused_records_.find(track_id);
  return found == fused_records_.end() ?
         std::nullopt : std::optional<FusedLandmarkScoreRecord>(found->second);
}

ScoreApplyResult LandmarkScoreManager::ApplyPatch(const ScorePatch & patch)
{
  ScoreApplyResult result;
  std::lock_guard<std::mutex> lock(mutex_);
  result.changes.score_revision_before = score_revision_;
  result.changes.score_revision_after = score_revision_;
  if (patch.expected_score_revision != score_revision_) {
    result.stale = true;
    result.reason = "score_revision_changed";
    return result;
  }

  for (const auto & evidence : patch.raw_evidence) {
    const auto found = records_.find(evidence.mappoint_id);
    if (evidence.evidence_id == 0U || !std::isfinite(evidence.delta) ||
      found == records_.end() || found->second.is_bad)
    {
      result.reason = "invalid_or_missing_raw_score_evidence";
      return result;
    }
  }
  for (const auto & update : patch.fused_upserts) {
    if (update.fused_track_id == 0U || !std::isfinite(update.score)) {
      result.reason = "invalid_fused_score";
      return result;
    }
  }

  std::set<RawMapPointId> raw_ids;
  for (const auto & evidence : patch.raw_evidence) {
    raw_ids.insert(evidence.mappoint_id);
  }
  for (const auto & id : raw_ids) {
    result.rollback.raw_records[id] = records_.at(id);
    result.rollback.raw_evidence[id] = applied_evidence_[id];
  }
  std::set<uint64_t> fused_ids(patch.fused_removals.begin(), patch.fused_removals.end());
  for (const auto & update : patch.fused_upserts) {
    fused_ids.insert(update.fused_track_id);
  }
  for (const uint64_t id : fused_ids) {
    const auto found = fused_records_.find(id);
    result.rollback.fused_records[id] = found == fused_records_.end() ?
      std::nullopt : std::optional<FusedLandmarkScoreRecord>(found->second);
  }
  result.rollback.revision_before = score_revision_;

  bool store_changed = false;
  for (const auto & evidence : patch.raw_evidence) {
    auto & applied = applied_evidence_[evidence.mappoint_id];
    if (!applied.insert(evidence.evidence_id).second) {
      continue;
    }
    auto & record = records_.at(evidence.mappoint_id);
    const float previous = record.score;
    if (evidence.delta >= 0.0F) {
      record.positive_adjustment += evidence.delta;
      ++record.positive_evidence;
    } else {
      record.negative_adjustment += evidence.delta;
      ++record.negative_evidence;
    }
    record.score = ClampScore(
      record.base_score_orb + record.positive_adjustment + record.negative_adjustment);
    ++record.record_revision;
    result.changes.input_updated_ids.push_back(evidence.mappoint_id);
    if (std::fabs(previous - record.score) > 1e-6F) {
      result.changes.updated_ids.push_back(evidence.mappoint_id);
    }
    store_changed = true;
  }

  for (const uint64_t id : patch.fused_removals) {
    if (fused_records_.erase(id) != 0U) {
      result.changes.fused_removed_ids.push_back(id);
      store_changed = true;
    }
  }
  for (const auto & update : patch.fused_upserts) {
    const float score = ClampScore(update.score);
    auto found = fused_records_.find(update.fused_track_id);
    if (found == fused_records_.end()) {
      fused_records_[update.fused_track_id] = {update.fused_track_id, score, 1};
      result.changes.fused_created_ids.push_back(update.fused_track_id);
      store_changed = true;
    } else if (std::fabs(found->second.score - score) > 1e-6F) {
      found->second.score = score;
      ++found->second.record_revision;
      result.changes.fused_updated_ids.push_back(update.fused_track_id);
      store_changed = true;
    }
  }

  if (store_changed) {
    ++score_revision_;
  }
  result.changes.score_revision_after = score_revision_;
  result.committed = true;
  result.reason = store_changed ? "applied" : "idempotent_no_change";
  return result;
}

bool LandmarkScoreManager::RollbackPatch(const ScoreRollbackPatch & patch)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & [id, record] : patch.raw_records) {
    if (record.has_value()) {
      records_[id] = *record;
    } else {
      records_.erase(id);
    }
  }
  for (const auto & [id, evidence] : patch.raw_evidence) {
    applied_evidence_[id] = evidence;
  }
  for (const auto & [id, record] : patch.fused_records) {
    if (record.has_value()) {
      fused_records_[id] = *record;
    } else {
      fused_records_.erase(id);
    }
  }
  score_revision_ = patch.revision_before;
  return true;
}

std::optional<LandmarkScoreRecord> LandmarkScoreManager::GetScore(
  const RawMapPointId & id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = records_.find(id);
  return found == records_.end() ?
         std::nullopt : std::optional<LandmarkScoreRecord>(found->second);
}

LandmarkScoreStats LandmarkScoreManager::GetStats() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  LandmarkScoreStats stats;
  stats.score_revision = score_revision_;
  stats.tracked_points = records_.size();
  if (records_.empty()) {
    return stats;
  }

  stats.score_min = std::numeric_limits<float>::max();
  double sum = 0.0;
  for (const auto & [id, record] : records_) {
    (void)id;
    stats.score_min = std::min(stats.score_min, record.score);
    stats.score_max = std::max(stats.score_max, record.score);
    sum += record.score;
    if (record.is_bad) {
      ++stats.bad_points;
    }
  }
  stats.score_mean = static_cast<float>(sum / static_cast<double>(records_.size()));
  return stats;
}

}  // namespace orbslam3_multi
