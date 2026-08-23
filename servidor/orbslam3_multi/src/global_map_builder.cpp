#include "orbslam3_multi/global_map_builder.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace orbslam3_multi
{
namespace
{

bool PoseToMatrix(const geometry_msgs::msg::Pose & pose, Eigen::Matrix4d * matrix)
{
  const Eigen::Quaterniond quaternion(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
    !std::isfinite(pose.position.z) || !std::isfinite(quaternion.w()) ||
    !std::isfinite(quaternion.x()) || !std::isfinite(quaternion.y()) ||
    !std::isfinite(quaternion.z()) || quaternion.norm() < 1e-9)
  {
    return false;
  }
  *matrix = Eigen::Matrix4d::Identity();
  matrix->block<3, 3>(0, 0) = quaternion.normalized().toRotationMatrix();
  (*matrix)(0, 3) = pose.position.x;
  (*matrix)(1, 3) = pose.position.y;
  (*matrix)(2, 3) = pose.position.z;
  return matrix->allFinite();
}

bool PoseEquivalent(
  const geometry_msgs::msg::Pose & lhs,
  const geometry_msgs::msg::Pose & rhs)
{
  return std::fabs(lhs.position.x - rhs.position.x) <= 1e-9 &&
         std::fabs(lhs.position.y - rhs.position.y) <= 1e-9 &&
         std::fabs(lhs.position.z - rhs.position.z) <= 1e-9 &&
         std::fabs(lhs.orientation.x - rhs.orientation.x) <= 1e-9 &&
         std::fabs(lhs.orientation.y - rhs.orientation.y) <= 1e-9 &&
         std::fabs(lhs.orientation.z - rhs.orientation.z) <= 1e-9 &&
         std::fabs(lhs.orientation.w - rhs.orientation.w) <= 1e-9;
}

bool PointEquivalent(const GlobalSparsePoint & lhs, const GlobalSparsePoint & rhs)
{
  return lhs.mappoint_id == rhs.mappoint_id &&
         lhs.fused_track_id == rhs.fused_track_id &&
         lhs.associated_keyframe_id == rhs.associated_keyframe_id &&
         std::fabs(lhs.x - rhs.x) <= 1e-5F &&
         std::fabs(lhs.y - rhs.y) <= 1e-5F &&
         std::fabs(lhs.z - rhs.z) <= 1e-5F &&
         std::fabs(lhs.score - rhs.score) <= 1e-6F;
}

}  // namespace

void GlobalMapBuilder::MarkRawChanges(const RawInsertResult & changes)
{
  dirty_keyframes_.insert(changes.new_keyframe_ids.begin(), changes.new_keyframe_ids.end());
  dirty_keyframes_.insert(
    changes.pose_changed_keyframe_ids.begin(), changes.pose_changed_keyframe_ids.end());
  dirty_keyframes_.insert(
    changes.association_changed_keyframe_ids.begin(),
    changes.association_changed_keyframe_ids.end());
  dirty_keyframes_.insert(
    changes.invalidated_keyframe_ids.begin(), changes.invalidated_keyframe_ids.end());
  dirty_mappoints_.insert(changes.new_mappoint_ids.begin(), changes.new_mappoint_ids.end());
  dirty_mappoints_.insert(
    changes.geometry_changed_mappoint_ids.begin(),
    changes.geometry_changed_mappoint_ids.end());
  dirty_mappoints_.insert(
    changes.association_changed_mappoint_ids.begin(),
    changes.association_changed_mappoint_ids.end());
  removed_mappoints_.insert(
    changes.invalidated_mappoint_ids.begin(), changes.invalidated_mappoint_ids.end());
  for (const auto & association : changes.association_changes) {
    dirty_keyframes_.insert(association.keyframe_id);
    dirty_mappoints_.insert(
      association.added_mappoint_ids.begin(), association.added_mappoint_ids.end());
    dirty_mappoints_.insert(
      association.removed_mappoint_ids.begin(), association.removed_mappoint_ids.end());
  }
}

void GlobalMapBuilder::MarkPoseChanges(const PoseChangeSet & changes)
{
  dirty_keyframes_.insert(changes.created_ids.begin(), changes.created_ids.end());
  dirty_keyframes_.insert(changes.updated_ids.begin(), changes.updated_ids.end());
  dirty_keyframes_.insert(changes.invalidated_ids.begin(), changes.invalidated_ids.end());
  for (const auto & id : changes.created_ids) {
    const auto found = keyframe_to_fused_tracks_.find(id);
    if (found != keyframe_to_fused_tracks_.end()) {
      dirty_fused_tracks_.insert(found->second.begin(), found->second.end());
    }
  }
  for (const auto & id : changes.updated_ids) {
    const auto found = keyframe_to_fused_tracks_.find(id);
    if (found != keyframe_to_fused_tracks_.end()) {
      dirty_fused_tracks_.insert(found->second.begin(), found->second.end());
    }
  }
  for (const auto & id : changes.invalidated_ids) {
    const auto found = keyframe_to_fused_tracks_.find(id);
    if (found != keyframe_to_fused_tracks_.end()) {
      dirty_fused_tracks_.insert(found->second.begin(), found->second.end());
    }
  }
}

void GlobalMapBuilder::MarkScoreChanges(const ScoreChangeSet & changes)
{
  dirty_mappoints_.insert(changes.created_ids.begin(), changes.created_ids.end());
  dirty_mappoints_.insert(changes.updated_ids.begin(), changes.updated_ids.end());
  removed_mappoints_.insert(changes.invalidated_ids.begin(), changes.invalidated_ids.end());
  dirty_fused_tracks_.insert(
    changes.fused_created_ids.begin(), changes.fused_created_ids.end());
  dirty_fused_tracks_.insert(
    changes.fused_updated_ids.begin(), changes.fused_updated_ids.end());
  removed_fused_tracks_.insert(
    changes.fused_removed_ids.begin(), changes.fused_removed_ids.end());
}

void GlobalMapBuilder::MarkFusionChanges(const FusionChangeSet & changes)
{
  dirty_fused_tracks_.insert(
    changes.created_track_ids.begin(), changes.created_track_ids.end());
  dirty_fused_tracks_.insert(
    changes.updated_track_ids.begin(), changes.updated_track_ids.end());
  removed_fused_tracks_.insert(
    changes.retired_track_ids.begin(), changes.retired_track_ids.end());
  removed_mappoints_.insert(
    changes.hidden_raw_member_ids.begin(), changes.hidden_raw_member_ids.end());
  dirty_mappoints_.insert(
    changes.released_raw_member_ids.begin(), changes.released_raw_member_ids.end());
}

bool GlobalMapBuilder::RemovePoint(const RawMapPointId & id)
{
  const auto slot = point_slot_by_id_.find(id);
  if (slot == point_slot_by_id_.end()) {
    return false;
  }
  const auto & cached = sparse_point_slots_[slot->second];
  if (cached.has_value()) {
    auto inverse = keyframe_to_mappoints_.find(cached->point.associated_keyframe_id);
    if (inverse != keyframe_to_mappoints_.end()) {
      inverse->second.erase(id);
    }
  }
  sparse_point_slots_[slot->second].reset();
  free_point_slots_.push_back(slot->second);
  point_slot_by_id_.erase(slot);
  return true;
}

bool GlobalMapBuilder::RemoveKeyFrame(const RawKeyFrameId & id)
{
  const bool removed = keyframe_world_cache_.erase(id) != 0U;
  keyframe_projection_cache_.erase(id);
  const auto inverse = keyframe_to_mappoints_.find(id);
  if (inverse != keyframe_to_mappoints_.end()) {
    dirty_mappoints_.insert(inverse->second.begin(), inverse->second.end());
  }
  return removed;
}

bool GlobalMapBuilder::EnsureKeyFrame(
  const RawKeyFrameId & id,
  const RawBuilderSnapshot & raw_snapshot,
  const GlobalPoseStore & pose_store,
  GlobalMapBuildResult * result)
{
  if (usable_keyframes_this_update_.find(id) != usable_keyframes_this_update_.end()) {
    return true;
  }
  if (unusable_keyframes_this_update_.find(id) != unusable_keyframes_this_update_.end()) {
    return false;
  }

  const auto raw = raw_snapshot.keyframes.find(id);
  const auto world = pose_store.GetPose(id);
  if (raw == raw_snapshot.keyframes.end() || !world.has_value() ||
    raw->second.is_bad || !world->active)
  {
    unusable_keyframes_this_update_.insert(id);
    if (RemoveKeyFrame(id)) {
      result->changed = true;
    }
    return false;
  }
  const auto projection = keyframe_projection_cache_.find(id);
  if (projection == keyframe_projection_cache_.end() ||
    !PoseEquivalent(projection->second.local_pose, raw->second.local_pose) ||
    !PoseEquivalent(projection->second.world_pose, world->world_pose))
  {
    Eigen::Matrix4d local_T_kf;
    Eigen::Matrix4d world_T_kf;
    if (!PoseToMatrix(raw->second.local_pose, &local_T_kf) ||
      !PoseToMatrix(world->world_pose, &world_T_kf))
    {
      ++result->skipped_invalid;
      unusable_keyframes_this_update_.insert(id);
      if (RemoveKeyFrame(id)) {
        result->changed = true;
      }
      return false;
    }
    CachedProjection next;
    next.local_pose = raw->second.local_pose;
    next.world_pose = world->world_pose;
    next.world_T_local = world_T_kf * local_T_kf.inverse();
    if (!next.world_T_local.allFinite()) {
      ++result->skipped_invalid;
      unusable_keyframes_this_update_.insert(id);
      if (RemoveKeyFrame(id)) {
        result->changed = true;
      }
      return false;
    }
    keyframe_projection_cache_[id] = std::move(next);
  }

  const auto existing = keyframe_world_cache_.find(id);
  if (existing != keyframe_world_cache_.end() &&
    PoseEquivalent(existing->second.world_pose, world->world_pose))
  {
    usable_keyframes_this_update_.insert(id);
    return true;
  }
  keyframe_world_cache_[id] = {id, world->world_pose};
  ++result->recalculated_keyframes;
  result->changed = true;
  usable_keyframes_this_update_.insert(id);
  return true;
}

bool GlobalMapBuilder::IsUsableObserver(
  const RawKeyFrameId & id,
  const RawBuilderMapPointInput & mappoint,
  const RawBuilderSnapshot & raw_snapshot,
  const GlobalPoseStore & pose_store,
  GlobalMapBuildResult * result)
{
  const bool observes = id.local_kf_id == mappoint.reference_keyframe_id ||
    std::find(
    mappoint.observer_keyframe_ids.begin(), mappoint.observer_keyframe_ids.end(),
    id.local_kf_id) != mappoint.observer_keyframe_ids.end();
  return observes && EnsureKeyFrame(id, raw_snapshot, pose_store, result);
}

std::optional<std::pair<RawKeyFrameId, bool>> GlobalMapBuilder::SelectObserver(
  const RawMapPointId & id,
  const RawBuilderMapPointInput & mappoint,
  const RawBuilderSnapshot & raw_snapshot,
  const GlobalPoseStore & pose_store,
  GlobalMapBuildResult * result)
{
  const auto existing_slot = point_slot_by_id_.find(id);
  if (existing_slot != point_slot_by_id_.end()) {
    const auto & existing = sparse_point_slots_[existing_slot->second];
    if (existing.has_value() && IsUsableObserver(
        existing->point.associated_keyframe_id, mappoint, raw_snapshot, pose_store, result))
    {
      return std::make_pair(
        existing->point.associated_keyframe_id, existing->used_reference_keyframe);
    }
  }

  const RawKeyFrameId reference{id.drone_id, id.map_epoch, mappoint.reference_keyframe_id};
  if (IsUsableObserver(reference, mappoint, raw_snapshot, pose_store, result)) {
    return std::make_pair(reference, true);
  }

  std::set<uint64_t> observer_ids;
  observer_ids.insert(
    mappoint.observer_keyframe_ids.begin(), mappoint.observer_keyframe_ids.end());
  for (const uint64_t local_kf_id : observer_ids) {
    const RawKeyFrameId candidate{id.drone_id, id.map_epoch, local_kf_id};
    if (IsUsableObserver(candidate, mappoint, raw_snapshot, pose_store, result)) {
      return std::make_pair(candidate, false);
    }
  }
  return std::nullopt;
}

void GlobalMapBuilder::StorePoint(CachedPoint point)
{
  const auto existing = point_slot_by_id_.find(point.point.mappoint_id);
  if (existing != point_slot_by_id_.end()) {
    auto & previous = sparse_point_slots_[existing->second];
    if (previous.has_value() &&
      !(previous->point.associated_keyframe_id == point.point.associated_keyframe_id))
    {
      keyframe_to_mappoints_[previous->point.associated_keyframe_id].erase(
        point.point.mappoint_id);
    }
    previous = std::move(point);
    keyframe_to_mappoints_[previous->point.associated_keyframe_id].insert(
      previous->point.mappoint_id);
    return;
  }

  size_t slot = sparse_point_slots_.size();
  if (!free_point_slots_.empty()) {
    slot = free_point_slots_.back();
    free_point_slots_.pop_back();
    sparse_point_slots_[slot] = std::move(point);
  } else {
    sparse_point_slots_.push_back(std::move(point));
  }
  const auto & stored = *sparse_point_slots_[slot];
  point_slot_by_id_[stored.point.mappoint_id] = slot;
  keyframe_to_mappoints_[stored.point.associated_keyframe_id].insert(
    stored.point.mappoint_id);
}

bool GlobalMapBuilder::UpdatePoint(
  const RawMapPointId & id,
  const RawBuilderSnapshot & raw_snapshot,
  const GlobalPoseStore & pose_store,
  const LandmarkScoreManager & score_manager,
  const FusedLandmarkManager & fused_manager,
  GlobalMapBuildResult * result)
{
  if (fused_manager.GetTrackIdForMember(id).has_value()) {
    return RemovePoint(id);
  }
  const auto raw = raw_snapshot.mappoints.find(id);
  const auto score = score_manager.GetScore(id);
  if (raw == raw_snapshot.mappoints.end() || !score.has_value()) {
    return RemovePoint(id);
  }
  if (raw->second.is_bad || score->is_bad) {
    ++result->skipped_bad;
    return RemovePoint(id);
  }
  if (!std::isfinite(raw->second.position.x) || !std::isfinite(raw->second.position.y) ||
    !std::isfinite(raw->second.position.z))
  {
    ++result->skipped_invalid;
    return RemovePoint(id);
  }

  const auto observer = SelectObserver(id, raw->second, raw_snapshot, pose_store, result);
  if (!observer.has_value()) {
    ++result->skipped_without_world_keyframe;
    return RemovePoint(id);
  }

  const auto projection = keyframe_projection_cache_.find(observer->first);
  if (projection == keyframe_projection_cache_.end()) {
    ++result->skipped_invalid;
    return RemovePoint(id);
  }

  const Eigen::Vector4d p_local(
    raw->second.position.x, raw->second.position.y, raw->second.position.z, 1.0);
  const Eigen::Vector4d p_world = projection->second.world_T_local * p_local;
  if (!p_world.allFinite()) {
    ++result->skipped_invalid;
    return RemovePoint(id);
  }

  CachedPoint next;
  next.point = {
    id, 0, observer->first, static_cast<float>(p_world.x()),
    static_cast<float>(p_world.y()), static_cast<float>(p_world.z()), score->score};
  next.used_reference_keyframe = observer->second;

  ++result->recalculated_mappoints;
  observer->second ? ++result->reference_associations : ++result->fallback_associations;

  const auto existing_slot = point_slot_by_id_.find(id);
  if (existing_slot != point_slot_by_id_.end()) {
    const auto & existing = sparse_point_slots_[existing_slot->second];
    if (existing.has_value() && PointEquivalent(existing->point, next.point) &&
      existing->used_reference_keyframe == next.used_reference_keyframe)
    {
      return false;
    }
  }
  StorePoint(std::move(next));
  return true;
}

bool GlobalMapBuilder::UpdateFusedTrack(
  FusedTrackId id, const RawMapDatabase & raw_database,
  const GlobalPoseStore & pose_store,
  const LandmarkScoreManager & score_manager,
  const FusedLandmarkManager & fused_manager,
  GlobalMapBuildResult * result)
{
  const auto remove_cached = [&]() {
      const auto existing = fused_point_cache_.find(id);
      if (existing == fused_point_cache_.end()) {
        return false;
      }
      keyframe_to_fused_tracks_[existing->second.point.associated_keyframe_id].erase(id);
      fused_point_cache_.erase(existing);
      return true;
    };
  const auto track = fused_manager.GetTrack(id);
  const auto fused_score = score_manager.GetFusedScore(id);
  if (!track.has_value() || !fused_score.has_value() ||
    track->member_mappoint_ids.empty())
  {
    return remove_cached();
  }

  const auto raw_snapshot = raw_database.GetBuilderSnapshot(
    {}, track->member_mappoint_ids);
  Eigen::Vector3d weighted_world = Eigen::Vector3d::Zero();
  double total_weight = 0.0;
  std::optional<std::pair<RawKeyFrameId, bool>> selected_observer;
  for (const auto & member : track->member_mappoint_ids) {
    const auto raw = raw_snapshot.mappoints.find(member);
    const auto member_score = score_manager.GetScore(member);
    if (raw == raw_snapshot.mappoints.end() || raw->second.is_bad ||
      !member_score.has_value() || member_score->is_bad)
    {
      continue;
    }
    const auto observer = SelectObserver(
      member, raw->second, raw_snapshot, pose_store, result);
    if (!observer.has_value()) {
      continue;
    }
    const auto projection = keyframe_projection_cache_.find(observer->first);
    if (projection == keyframe_projection_cache_.end()) {
      continue;
    }
    const Eigen::Vector4d local(
      raw->second.position.x, raw->second.position.y, raw->second.position.z, 1.0);
    const Eigen::Vector4d world = projection->second.world_T_local * local;
    if (!world.allFinite()) {
      continue;
    }
    const double weight = std::max(0.01, static_cast<double>(member_score->score));
    weighted_world += weight * world.head<3>();
    total_weight += weight;
    if (!selected_observer.has_value() || member == track->representative_member) {
      selected_observer = observer;
    }
  }
  if (total_weight <= 0.0 || !selected_observer.has_value()) {
    ++result->skipped_without_world_keyframe;
    return remove_cached();
  }

  const Eigen::Vector3d world = weighted_world / total_weight;
  CachedPoint next;
  next.point = {
    track->representative_member, id, selected_observer->first,
    static_cast<float>(world.x()), static_cast<float>(world.y()),
    static_cast<float>(world.z()), fused_score->score};
  next.used_reference_keyframe = selected_observer->second;
  ++result->recalculated_fused_tracks;

  const auto existing = fused_point_cache_.find(id);
  if (existing != fused_point_cache_.end() &&
    PointEquivalent(existing->second.point, next.point) &&
    existing->second.used_reference_keyframe == next.used_reference_keyframe)
  {
    return false;
  }
  if (existing != fused_point_cache_.end()) {
    keyframe_to_fused_tracks_[existing->second.point.associated_keyframe_id].erase(id);
  }
  keyframe_to_fused_tracks_[next.point.associated_keyframe_id].insert(id);
  fused_point_cache_[id] = std::move(next);
  return true;
}

void GlobalMapBuilder::PopulateOutput(GlobalMapBuildResult * result) const
{
  result->points.reserve(point_slot_by_id_.size() + fused_point_cache_.size());
  for (const auto & slot : sparse_point_slots_) {
    if (slot.has_value()) {
      result->points.push_back(slot->point);
    }
  }
  for (const auto & [id, cached] : fused_point_cache_) {
    (void)id;
    result->points.push_back(cached.point);
  }
  result->keyframes.reserve(keyframe_world_cache_.size());
  for (const auto & [id, keyframe] : keyframe_world_cache_) {
    (void)id;
    result->keyframes.push_back(keyframe);
  }
}

GlobalMapBuildResult GlobalMapBuilder::Update(
  const RawMapDatabase & raw_database,
  const GlobalPoseStore & pose_store,
  const LandmarkScoreManager & score_manager,
  const FusedLandmarkManager & fused_manager)
{
  GlobalMapBuildResult result;
  usable_keyframes_this_update_.clear();
  unusable_keyframes_this_update_.clear();

  for (auto deferred = deferred_unanchored_submaps_.begin();
    deferred != deferred_unanchored_submaps_.end();)
  {
    if (!pose_store.HasSubmapAnchor(*deferred)) {
      ++deferred;
      continue;
    }
    const auto ids = raw_database.GetActiveSubmapEntityIds(*deferred);
    if (ids.has_value()) {
      dirty_keyframes_.insert(ids->keyframe_ids.begin(), ids->keyframe_ids.end());
      dirty_mappoints_.insert(ids->mappoint_ids.begin(), ids->mappoint_ids.end());
      ++result.backfilled_submaps;
      result.backfilled_keyframes += ids->keyframe_ids.size();
      result.backfilled_mappoints += ids->mappoint_ids.size();
    }
    deferred = deferred_unanchored_submaps_.erase(deferred);
  }

  result.dirty_keyframes = dirty_keyframes_.size();
  result.dirty_mappoints = dirty_mappoints_.size() + removed_mappoints_.size();
  result.pose_revision = pose_store.GetStats().store_revision;
  result.score_revision = score_manager.GetStats().score_revision;
  result.fusion_revision = fused_manager.GetStats().revision;

  for (const auto & id : dirty_mappoints_) {
    const auto track = fused_manager.GetTrackIdForMember(id);
    if (track.has_value()) {
      dirty_fused_tracks_.insert(*track);
    }
  }

  std::map<RawSubmapId, bool> anchored_submaps;
  const auto is_anchored = [&pose_store, &anchored_submaps](const auto & id) {
      const RawSubmapId submap_id{id.drone_id, id.map_epoch};
      const auto cached = anchored_submaps.find(submap_id);
      if (cached != anchored_submaps.end()) {
        return cached->second;
      }
      const bool anchored = pose_store.HasSubmapAnchor(submap_id);
      anchored_submaps.emplace(submap_id, anchored);
      return anchored;
    };

  for (auto id = dirty_keyframes_.begin(); id != dirty_keyframes_.end();) {
    if (is_anchored(*id)) {
      ++id;
      continue;
    }
    deferred_unanchored_submaps_.insert({id->drone_id, id->map_epoch});
    ++result.deferred_unanchored_keyframes;
    id = dirty_keyframes_.erase(id);
  }
  for (auto id = dirty_mappoints_.begin(); id != dirty_mappoints_.end();) {
    if (is_anchored(*id)) {
      ++id;
      continue;
    }
    deferred_unanchored_submaps_.insert({id->drone_id, id->map_epoch});
    ++result.deferred_unanchored_mappoints;
    id = dirty_mappoints_.erase(id);
  }
  for (auto id = removed_mappoints_.begin(); id != removed_mappoints_.end();) {
    if (is_anchored(*id)) {
      ++id;
      continue;
    }
    deferred_unanchored_submaps_.insert({id->drone_id, id->map_epoch});
    ++result.deferred_unanchored_mappoints;
    id = removed_mappoints_.erase(id);
  }
  result.deferred_unanchored_submaps = deferred_unanchored_submaps_.size();

  for (const auto & id : removed_mappoints_) {
    if (RemovePoint(id)) {
      result.changed = true;
    }
  }
  for (const auto id : removed_fused_tracks_) {
    const auto existing = fused_point_cache_.find(id);
    if (existing != fused_point_cache_.end()) {
      keyframe_to_fused_tracks_[existing->second.point.associated_keyframe_id].erase(id);
      fused_point_cache_.erase(existing);
      result.changed = true;
    }
  }

  std::set<RawMapPointId> requested_mappoints = dirty_mappoints_;
  for (const auto & id : dirty_keyframes_) {
    const auto inverse = keyframe_to_mappoints_.find(id);
    if (inverse != keyframe_to_mappoints_.end()) {
      requested_mappoints.insert(inverse->second.begin(), inverse->second.end());
    }
  }
  auto raw_snapshot = raw_database.GetBuilderSnapshot(
    dirty_keyframes_, requested_mappoints);
  dirty_mappoints_ = raw_snapshot.requested_mappoint_ids;

  const auto dirty_keyframes = dirty_keyframes_;
  for (const auto & id : dirty_keyframes) {
    if (!EnsureKeyFrame(id, raw_snapshot, pose_store, &result)) {
      const auto raw = raw_snapshot.keyframes.find(id);
      if (raw != raw_snapshot.keyframes.end() && !raw->second.is_bad) {
        ++result.skipped_unanchored;
      }
    }
  }

  result.dirty_mappoints = dirty_mappoints_.size() + removed_mappoints_.size();

  for (auto id = dirty_mappoints_.begin(); id != dirty_mappoints_.end(); ++id) {
    if (raw_snapshot.requested_mappoint_ids.find(*id) ==
      raw_snapshot.requested_mappoint_ids.end())
    {
      const auto extra = raw_database.GetBuilderSnapshot({}, {*id});
      raw_snapshot.requested_mappoint_ids.insert(
        extra.requested_mappoint_ids.begin(), extra.requested_mappoint_ids.end());
      raw_snapshot.keyframes.insert(extra.keyframes.begin(), extra.keyframes.end());
      raw_snapshot.mappoints.insert(extra.mappoints.begin(), extra.mappoints.end());
    }
    if (UpdatePoint(
        *id, raw_snapshot, pose_store, score_manager, fused_manager, &result))
    {
      result.changed = true;
    }
  }
  for (const auto id : dirty_fused_tracks_) {
    if (UpdateFusedTrack(
        id, raw_database, pose_store, score_manager, fused_manager, &result))
    {
      result.changed = true;
    }
  }

  dirty_keyframes_.clear();
  dirty_mappoints_.clear();
  removed_mappoints_.clear();
  dirty_fused_tracks_.clear();
  removed_fused_tracks_.clear();
  if (result.changed) {
    ++publication_revision_;
  }
  result.publication_revision = publication_revision_;
  PopulateOutput(&result);
  return result;
}

}  // namespace orbslam3_multi
