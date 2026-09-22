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

void LandmarkScoreManager::Configure(const LandmarkScoreConfig & config)
{
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
  config_.isolation_radius_m = std::max(0.01, config_.isolation_radius_m);
  config_.isolation_min_factor = ClampScore(config_.isolation_min_factor);
  config_.suspicious_near_distance_m = std::max(0.0, config_.suspicious_near_distance_m);
  config_.suspicious_near_min_factor = ClampScore(config_.suspicious_near_min_factor);
  config_.far_baseline_multiplier = std::max(1.0, config_.far_baseline_multiplier);
  config_.far_distance_fallback_m = std::max(0.1, config_.far_distance_fallback_m);
  config_.far_min_factor = ClampScore(config_.far_min_factor);
}

void LandmarkScoreManager::RecomputeOutput(LandmarkScoreRecord * record)
{
  // A mature point without spatial support is not usable evidence, even if a
  // later inlier event would otherwise add a positive adjustment.
  if (record->isolation_factor <= 0.0F || record->body_factor <= 0.0F) {
    record->score = 0.0F;
    return;
  }
  record->score = ClampScore(
    record->base_score_orb * record->distance_factor * record->isolation_factor +
    record->positive_adjustment + record->negative_adjustment);
}

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
         std::fabs(lhs.distance_factor - rhs.distance_factor) <= 1e-6F &&
         std::fabs(lhs.isolation_factor - rhs.isolation_factor) <= 1e-6F &&
         std::fabs(lhs.body_factor - rhs.body_factor) <= 1e-6F &&
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
      RecomputeOutput(&next);
      next.record_revision = 1;
      records_.emplace(id, next);
      result.created_ids.push_back(id);
    } else {
      next.positive_adjustment = existing->second.positive_adjustment;
      next.negative_adjustment = existing->second.negative_adjustment;
      next.distance_factor = existing->second.distance_factor;
      next.isolation_factor = existing->second.isolation_factor;
      next.body_factor = existing->second.body_factor;
      next.positive_evidence = existing->second.positive_evidence;
      next.negative_evidence = existing->second.negative_evidence;
      RecomputeOutput(&next);
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

std::array<int64_t, 3> LandmarkScoreManager::VoxelFor(
  const geometry_msgs::msg::Point & point) const
{
  const double size = config_.isolation_radius_m;
  return {
    static_cast<int64_t>(std::floor(point.x / size)),
    static_cast<int64_t>(std::floor(point.y / size)),
    static_cast<int64_t>(std::floor(point.z / size))};
}

size_t LandmarkScoreManager::NeighborCount(
  const RawMapPointId & id, const GeometryState & geometry) const
{
  size_t count = 0;
  const double radius_squared = config_.isolation_radius_m * config_.isolation_radius_m;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        const std::array<int64_t, 3> voxel{
          geometry.voxel[0] + dx, geometry.voxel[1] + dy, geometry.voxel[2] + dz};
        const auto found = spatial_index_.find(voxel);
        if (found == spatial_index_.end()) {
          continue;
        }
        for (const auto & candidate_id : found->second) {
          if (candidate_id == id) {
            continue;
          }
          const auto candidate_record = records_.find(candidate_id);
          const auto candidate_geometry = geometry_.find(candidate_id);
          if (candidate_record == records_.end() || candidate_record->second.is_bad ||
            candidate_geometry == geometry_.end())
          {
            continue;
          }
          const auto & position = candidate_geometry->second.world_position;
          const double dx_position = position.x - geometry.world_position.x;
          const double dy_position = position.y - geometry.world_position.y;
          const double dz_position = position.z - geometry.world_position.z;
          if (dx_position * dx_position + dy_position * dy_position +
            dz_position * dz_position <= radius_squared)
          {
            ++count;
          }
        }
      }
    }
  }
  return count;
}

float LandmarkScoreManager::DistanceFactor(const GeometryState & geometry) const
{
  const double distance = geometry.observer_distance_m;
  if (!std::isfinite(distance) || distance < 0.0) {
    return 1.0F;
  }
  if (config_.suspicious_near_distance_m > 0.0 &&
    distance < config_.suspicious_near_distance_m)
  {
    const double ratio = distance / config_.suspicious_near_distance_m;
    return std::max(
      config_.suspicious_near_min_factor, static_cast<float>(ratio * ratio));
  }
  const double far_limit = geometry.stereo_baseline_m > 0.0 ?
    std::max(
      config_.suspicious_near_distance_m,
      config_.far_baseline_multiplier * geometry.stereo_baseline_m) :
    config_.far_distance_fallback_m;
  if (distance <= far_limit) {
    return 1.0F;
  }
  const double ratio = far_limit / distance;
  return std::max(config_.far_min_factor, static_cast<float>(ratio * ratio));
}

float LandmarkScoreManager::IsolationFactor(
  const RawMapPointId & id, const LandmarkScoreRecord & record) const
{
  const auto geometry = geometry_.find(id);
  if (geometry == geometry_.end() ||
    record.observations_count < config_.isolation_min_observations ||
    config_.isolation_min_neighbors == 0U)
  {
    return 1.0F;
  }
  const size_t neighbors = NeighborCount(id, geometry->second);
  if (neighbors >= config_.isolation_min_neighbors) {
    return 1.0F;
  }
  const float ratio = static_cast<float>(neighbors) /
    static_cast<float>(config_.isolation_min_neighbors);
  return config_.isolation_min_factor + (1.0F - config_.isolation_min_factor) * ratio;
}

std::vector<std::array<int64_t, 3>> LandmarkScoreManager::SphereVoxels(
  const DroneBodySphere & sphere) const
{
  const double radius = std::max(0.0, sphere.radius_m);
  geometry_msgs::msg::Point minimum = sphere.center;
  geometry_msgs::msg::Point maximum = sphere.center;
  minimum.x -= radius;
  minimum.y -= radius;
  minimum.z -= radius;
  maximum.x += radius;
  maximum.y += radius;
  maximum.z += radius;
  const auto first = VoxelFor(minimum);
  const auto last = VoxelFor(maximum);
  std::vector<std::array<int64_t, 3>> voxels;
  for (int64_t x = first[0]; x <= last[0]; ++x) {
    for (int64_t y = first[1]; y <= last[1]; ++y) {
      for (int64_t z = first[2]; z <= last[2]; ++z) {
        voxels.push_back({x, y, z});
      }
    }
  }
  return voxels;
}

void LandmarkScoreManager::CollectPointsInSphere(
  const DroneBodySphere & sphere, std::set<RawMapPointId> * affected) const
{
  for (const auto & voxel : SphereVoxels(sphere)) {
    const auto points = spatial_index_.find(voxel);
    if (points != spatial_index_.end()) {
      affected->insert(points->second.begin(), points->second.end());
    }
  }
}

void LandmarkScoreManager::IndexBodySphere(const DroneBodySphere & sphere)
{
  for (const auto & voxel : SphereVoxels(sphere)) {
    body_spatial_index_[voxel].insert(sphere.keyframe_id);
  }
}

void LandmarkScoreManager::RemoveBodySphere(const DroneBodySphere & sphere)
{
  for (const auto & voxel : SphereVoxels(sphere)) {
    const auto found = body_spatial_index_.find(voxel);
    if (found == body_spatial_index_.end()) {
      continue;
    }
    found->second.erase(sphere.keyframe_id);
    if (found->second.empty()) {
      body_spatial_index_.erase(found);
    }
  }
}

float LandmarkScoreManager::BodyFactor(const geometry_msgs::msg::Point & point) const
{
  if (!config_.drone_body_mask_enabled) {
    return 1.0F;
  }
  const auto candidates = body_spatial_index_.find(VoxelFor(point));
  if (candidates == body_spatial_index_.end()) {
    return 1.0F;
  }
  for (const auto & keyframe_id : candidates->second) {
    const auto sphere = body_spheres_.find(keyframe_id);
    if (sphere == body_spheres_.end()) {
      continue;
    }
    const double dx = point.x - sphere->second.center.x;
    const double dy = point.y - sphere->second.center.y;
    const double dz = point.z - sphere->second.center.z;
    if (dx * dx + dy * dy + dz * dz <= sphere->second.radius_m * sphere->second.radius_m) {
      return 0.0F;
    }
  }
  return 1.0F;
}

ScoreChangeSet LandmarkScoreManager::ApplyGeometryChanges(
  const std::vector<LandmarkScoreGeometryInput> & upserts,
  const std::vector<RawMapPointId> & removals)
{
  ScoreChangeSet result;
  std::lock_guard<std::mutex> lock(mutex_);
  result.score_revision_before = score_revision_;
  if (!config_.drone_body_mask_enabled) {
    result.score_revision_after = score_revision_;
    return result;
  }
  std::set<RawMapPointId> affected;
  std::set<std::array<int64_t, 3>> affected_voxels;
  const auto mark_neighbor_voxels = [&affected_voxels](
      const std::array<int64_t, 3> & center) {
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            affected_voxels.insert(
              {center[0] + dx, center[1] + dy, center[2] + dz});
          }
        }
      }
    };

  const auto remove_geometry = [&](const RawMapPointId & id) {
      const auto existing = geometry_.find(id);
      if (existing == geometry_.end()) {
        return;
      }
      mark_neighbor_voxels(existing->second.voxel);
      auto voxel = spatial_index_.find(existing->second.voxel);
      if (voxel != spatial_index_.end()) {
        voxel->second.erase(id);
        if (voxel->second.empty()) {
          spatial_index_.erase(voxel);
        }
      }
      geometry_.erase(existing);
      affected.insert(id);
    };

  for (const auto & id : removals) {
    remove_geometry(id);
  }
  for (const auto & input : upserts) {
    if (!std::isfinite(input.world_position.x) || !std::isfinite(input.world_position.y) ||
      !std::isfinite(input.world_position.z) || !std::isfinite(input.observer_distance_m))
    {
      remove_geometry(input.mappoint_id);
      continue;
    }
    const auto current = geometry_.find(input.mappoint_id);
    if (current != geometry_.end() &&
      std::fabs(current->second.world_position.x - input.world_position.x) <= 1e-6 &&
      std::fabs(current->second.world_position.y - input.world_position.y) <= 1e-6 &&
      std::fabs(current->second.world_position.z - input.world_position.z) <= 1e-6 &&
      std::fabs(current->second.observer_distance_m - input.observer_distance_m) <= 1e-6 &&
      std::fabs(current->second.stereo_baseline_m - input.stereo_baseline_m) <= 1e-9)
    {
      affected.insert(input.mappoint_id);
      continue;
    }
    remove_geometry(input.mappoint_id);
    GeometryState next;
    next.world_position = input.world_position;
    next.observer_distance_m = input.observer_distance_m;
    next.stereo_baseline_m = std::isfinite(input.stereo_baseline_m) ?
      std::max(0.0, input.stereo_baseline_m) : 0.0;
    next.voxel = VoxelFor(next.world_position);
    geometry_[input.mappoint_id] = next;
    spatial_index_[next.voxel].insert(input.mappoint_id);
    mark_neighbor_voxels(next.voxel);
    affected.insert(input.mappoint_id);
  }

  for (const auto & voxel : affected_voxels) {
    const auto points = spatial_index_.find(voxel);
    if (points != spatial_index_.end()) {
      affected.insert(points->second.begin(), points->second.end());
    }
  }

  for (const auto & id : affected) {
    auto record = records_.find(id);
    if (record == records_.end()) {
      continue;
    }
    const auto previous = record->second;
    const auto geometry = geometry_.find(id);
    record->second.distance_factor = geometry == geometry_.end() ?
      1.0F : DistanceFactor(geometry->second);
    record->second.isolation_factor = IsolationFactor(id, record->second);
    record->second.body_factor = geometry == geometry_.end() ?
      1.0F : BodyFactor(geometry->second.world_position);
    RecomputeOutput(&record->second);
    if (Equivalent(previous, record->second)) {
      continue;
    }
    ++record->second.record_revision;
    result.input_updated_ids.push_back(id);
    if (!OutputEquivalent(previous, record->second)) {
      result.updated_ids.push_back(id);
    }
  }
  if (result.HasChanges()) {
    ++score_revision_;
  }
  result.score_revision_after = score_revision_;
  return result;
}

ScoreChangeSet LandmarkScoreManager::UpdateDroneBodySpheres(
  const std::vector<DroneBodySphere> & upserts,
  const std::vector<RawKeyFrameId> & removals)
{
  ScoreChangeSet result;
  std::lock_guard<std::mutex> lock(mutex_);
  result.score_revision_before = score_revision_;
  std::set<RawMapPointId> affected;

  const auto remove = [&](const RawKeyFrameId & id) {
      const auto existing = body_spheres_.find(id);
      if (existing == body_spheres_.end()) {
        return;
      }
      CollectPointsInSphere(existing->second, &affected);
      RemoveBodySphere(existing->second);
      body_spheres_.erase(existing);
    };

  for (const auto & id : removals) {
    remove(id);
  }
  for (const auto & input : upserts) {
    if (!std::isfinite(input.center.x) || !std::isfinite(input.center.y) ||
      !std::isfinite(input.center.z) || !std::isfinite(input.radius_m) ||
      input.radius_m <= 0.0)
    {
      remove(input.keyframe_id);
      continue;
    }
    const auto existing = body_spheres_.find(input.keyframe_id);
    if (existing != body_spheres_.end()) {
      const auto & previous = existing->second;
      if (std::fabs(previous.center.x - input.center.x) <= 1e-6 &&
        std::fabs(previous.center.y - input.center.y) <= 1e-6 &&
        std::fabs(previous.center.z - input.center.z) <= 1e-6 &&
        std::fabs(previous.radius_m - input.radius_m) <= 1e-6)
      {
        continue;
      }
      CollectPointsInSphere(previous, &affected);
      RemoveBodySphere(previous);
      body_spheres_.erase(existing);
    }
    body_spheres_[input.keyframe_id] = input;
    IndexBodySphere(input);
    CollectPointsInSphere(input, &affected);
  }

  for (const auto & id : affected) {
    const auto record = records_.find(id);
    const auto geometry = geometry_.find(id);
    if (record == records_.end() || geometry == geometry_.end()) {
      continue;
    }
    const auto previous = record->second;
    record->second.body_factor = BodyFactor(geometry->second.world_position);
    RecomputeOutput(&record->second);
    if (Equivalent(previous, record->second)) {
      continue;
    }
    ++record->second.record_revision;
    result.input_updated_ids.push_back(id);
    if (!OutputEquivalent(previous, record->second)) {
      result.updated_ids.push_back(id);
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
    RecomputeOutput(&record);
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
    if (geometry_.count(id) != 0U) {
      ++stats.anchored_points;
    }
    if (record.isolation_factor < 1.0F - 1e-6F) {
      ++stats.isolated_points;
    }
    if (record.body_factor <= 0.0F) {
      ++stats.body_masked_points;
    }
    if (record.distance_factor < 1.0F - 1e-6F) {
      const auto geometry = geometry_.find(id);
      if (geometry != geometry_.end() &&
        geometry->second.observer_distance_m < config_.suspicious_near_distance_m)
      {
        ++stats.suspicious_near_points;
      } else {
        ++stats.far_points;
      }
    }
  }
  stats.score_mean = static_cast<float>(sum / static_cast<double>(records_.size()));
  return stats;
}

}  // namespace orbslam3_multi
