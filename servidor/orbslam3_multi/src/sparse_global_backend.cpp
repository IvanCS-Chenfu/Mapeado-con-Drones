#include "orbslam3_multi/sparse_global_backend.hpp"

#include "orbslam3_multi/pose_geometry.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>
#include <utility>

namespace orbslam3_multi
{
namespace
{

bool WithinThreshold(
  const FiducialError & error, const FiducialOptimizationConfig & config)
{
  return error.translation_m <= config.translation_threshold_m &&
         error.rotation_rad <= config.rotation_threshold_rad &&
         error.yaw_rad <= config.yaw_threshold_rad;
}

std::optional<size_t> FindKeyFrameIndex(
  const RawSubmapPoseSnapshot & snapshot, const RawKeyFrameId & id)
{
  const auto found = std::find_if(
    snapshot.keyframes.begin(), snapshot.keyframes.end(),
    [&id](const RawKeyFramePoseInput & input) {return input.id == id;});
  return found == snapshot.keyframes.end() ?
         std::nullopt : std::optional<size_t>(
    static_cast<size_t>(found - snapshot.keyframes.begin()));
}

geometry_msgs::msg::Pose ComposePose(
  const geometry_msgs::msg::Pose & world_T_local,
  const geometry_msgs::msg::Pose & local_T_kf)
{
  Eigen::Isometry3d world;
  Eigen::Isometry3d local;
  if (!PoseToIsometry(world_T_local, &world) || !PoseToIsometry(local_T_kf, &local)) {
    throw std::invalid_argument("pose no valida al preparar reanchor fiducial");
  }
  return IsometryToPose(world * local);
}

CovisibilityPatch BuildServerCovisibilityPatch(
  const LoopTaskComputation & computation,
  const RawMapDatabase & raw_database,
  const GlobalPoseStore & pose_store,
  uint64_t expected_database_revision)
{
  CovisibilityPatch patch;
  patch.source_arrival_id = computation.task.source_arrival_id;
  patch.expected_database_revision = expected_database_revision;
  std::map<std::pair<RawKeyFrameId, RawKeyFrameId>, CovisibilityEdge> best_edges;
  for (const auto & geometry : computation.geometry_results) {
    if (!geometry.accepted || !geometry.fusion_compatible) {
      continue;
    }
    const auto query_raw = raw_database.GetKeyFrame(geometry.query_keyframe_id);
    const auto candidate_raw = raw_database.GetKeyFrame(geometry.candidate_keyframe_id);
    const auto query_world = pose_store.GetPose(geometry.query_keyframe_id);
    const auto candidate_world = pose_store.GetPose(geometry.candidate_keyframe_id);
    Eigen::Isometry3d query_local_T_kf;
    Eigen::Isometry3d candidate_local_T_kf;
    Eigen::Isometry3d world_T_query;
    Eigen::Isometry3d world_T_candidate;
    if (!query_raw.has_value() || !candidate_raw.has_value() ||
      !query_world.has_value() || !candidate_world.has_value() ||
      !query_world->active || !candidate_world->active ||
      !PoseToIsometry(query_raw->pose, &query_local_T_kf) ||
      !PoseToIsometry(candidate_raw->pose, &candidate_local_T_kf) ||
      !PoseToIsometry(query_world->world_pose, &world_T_query) ||
      !PoseToIsometry(candidate_world->world_pose, &world_T_candidate))
    {
      continue;
    }
    const Eigen::Isometry3d candidate_kf_T_query_kf =
      candidate_local_T_kf.inverse() *
      geometry.candidate_local_T_query_local * query_local_T_kf;
    const bool query_first = geometry.query_keyframe_id < geometry.candidate_keyframe_id;
    CovisibilityEdge edge;
    edge.kf_a = query_first ? geometry.query_keyframe_id : geometry.candidate_keyframe_id;
    edge.kf_b = query_first ? geometry.candidate_keyframe_id : geometry.query_keyframe_id;
    edge.source = CovisibilityEdgeSource::ServerLoopGeometric;
    edge.support = std::max<size_t>(1U, geometry.inliers);
    edge.information_weight = static_cast<double>(edge.support) /
      std::max(1e-3, geometry.mean_residual_m);
    edge.relative_pose_measured = IsometryToPose(
      query_first ? candidate_kf_T_query_kf.inverse() : candidate_kf_T_query_kf);
    edge.relative_pose_current = IsometryToPose(
      (query_first ? world_T_query : world_T_candidate).inverse() *
      (query_first ? world_T_candidate : world_T_query));
    edge.dependency_revision_a = query_first ?
      query_world->pose_revision : candidate_world->pose_revision;
    edge.dependency_revision_b = query_first ?
      candidate_world->pose_revision : query_world->pose_revision;
    edge.created_arrival_id = computation.task.source_arrival_id;
    const auto key = std::make_pair(edge.kf_a, edge.kf_b);
    const auto found = best_edges.find(key);
    if (found == best_edges.end() || edge.support > found->second.support) {
      best_edges[key] = std::move(edge);
    }
  }
  for (auto & [key, edge] : best_edges) {
    (void)key;
    patch.upserts.push_back(std::move(edge));
  }
  return patch;
}

void MergeScoreChanges(
  ScoreChangeSet * destination, const ScoreChangeSet & source)
{
  destination->score_revision_after = source.score_revision_after;
  destination->created_ids.insert(
    destination->created_ids.end(), source.created_ids.begin(), source.created_ids.end());
  destination->updated_ids.insert(
    destination->updated_ids.end(), source.updated_ids.begin(), source.updated_ids.end());
  destination->input_updated_ids.insert(
    destination->input_updated_ids.end(), source.input_updated_ids.begin(),
    source.input_updated_ids.end());
  destination->invalidated_ids.insert(
    destination->invalidated_ids.end(), source.invalidated_ids.begin(),
    source.invalidated_ids.end());
  destination->fused_created_ids.insert(
    destination->fused_created_ids.end(), source.fused_created_ids.begin(),
    source.fused_created_ids.end());
  destination->fused_updated_ids.insert(
    destination->fused_updated_ids.end(), source.fused_updated_ids.begin(),
    source.fused_updated_ids.end());
  destination->fused_removed_ids.insert(
    destination->fused_removed_ids.end(), source.fused_removed_ids.begin(),
    source.fused_removed_ids.end());
  const auto unique = [](auto * values) {
      std::sort(values->begin(), values->end());
      values->erase(std::unique(values->begin(), values->end()), values->end());
    };
  unique(&destination->created_ids);
  unique(&destination->updated_ids);
  unique(&destination->invalidated_ids);
  unique(&destination->input_updated_ids);
  unique(&destination->fused_created_ids);
  unique(&destination->fused_updated_ids);
  unique(&destination->fused_removed_ids);
}

void RefreshFusedScores(
  const ScoreChangeSet & raw_score_changes,
  FusedLandmarkManager * fused_manager,
  LandmarkScoreManager * score_manager,
  ScoreChangeSet * combined_changes)
{
  std::vector<RawMapPointId> members = raw_score_changes.created_ids;
  members.insert(
    members.end(), raw_score_changes.updated_ids.begin(),
    raw_score_changes.updated_ids.end());
  members.insert(
    members.end(), raw_score_changes.input_updated_ids.begin(),
    raw_score_changes.input_updated_ids.end());
  members.insert(
    members.end(), raw_score_changes.invalidated_ids.begin(),
    raw_score_changes.invalidated_ids.end());
  auto updates = fused_manager->BuildScoreUpdatesForMembers(members, *score_manager);
  if (updates.empty()) {
    return;
  }
  ScorePatch patch;
  patch.expected_score_revision = score_manager->GetStats().score_revision;
  patch.fused_upserts = std::move(updates);
  const auto refreshed = score_manager->ApplyPatch(patch);
  if (refreshed.committed) {
    MergeScoreChanges(combined_changes, refreshed.changes);
  }
}

}  // namespace

ScoreChangeSet SparseGlobalBackend::RefreshGeometryScores(
  const std::set<RawKeyFrameId> & keyframe_ids,
  const std::set<RawMapPointId> & mappoint_ids,
  const std::vector<RawMapPointId> & removals)
{
  const auto snapshot = raw_database_.GetBuilderSnapshot(keyframe_ids, mappoint_ids);
  std::vector<LandmarkScoreGeometryInput> upserts;
  std::vector<RawMapPointId> geometry_removals = removals;
  std::map<RawSubmapId, double> baselines;
  upserts.reserve(snapshot.mappoints.size());

  for (const auto & [id, point] : snapshot.mappoints) {
    if (point.is_bad) {
      geometry_removals.push_back(id);
      continue;
    }
    std::vector<uint64_t> observers;
    observers.reserve(point.observer_keyframe_ids.size() + 1U);
    observers.push_back(point.reference_keyframe_id);
    observers.insert(
      observers.end(), point.observer_keyframe_ids.begin(),
      point.observer_keyframe_ids.end());

    bool resolved = false;
    for (const uint64_t local_kf_id : observers) {
      const RawKeyFrameId keyframe_id{id.drone_id, id.map_epoch, local_kf_id};
      const auto raw_keyframe = snapshot.keyframes.find(keyframe_id);
      const auto world_keyframe = pose_store_.GetPose(keyframe_id);
      if (raw_keyframe == snapshot.keyframes.end() || raw_keyframe->second.is_bad ||
        !world_keyframe.has_value() || !world_keyframe->active)
      {
        continue;
      }
      Eigen::Isometry3d local_T_keyframe;
      Eigen::Isometry3d world_T_keyframe;
      if (!PoseToIsometry(raw_keyframe->second.local_pose, &local_T_keyframe) ||
        !PoseToIsometry(world_keyframe->world_pose, &world_T_keyframe))
      {
        continue;
      }
      const Eigen::Vector3d local_point(
        point.position.x, point.position.y, point.position.z);
      const Eigen::Vector3d camera_point = local_T_keyframe.inverse() * local_point;
      const Eigen::Vector3d world_point =
        world_T_keyframe * local_T_keyframe.inverse() * local_point;
      if (!camera_point.allFinite() || !world_point.allFinite()) {
        continue;
      }
      LandmarkScoreGeometryInput input;
      input.mappoint_id = id;
      input.world_position.x = world_point.x();
      input.world_position.y = world_point.y();
      input.world_position.z = world_point.z();
      input.observer_distance_m = camera_point.norm();
      const RawSubmapId submap_id{id.drone_id, id.map_epoch};
      auto baseline = baselines.find(submap_id);
      if (baseline == baselines.end()) {
        const auto camera = raw_database_.GetCameraCalibration(submap_id);
        const double value = camera.has_value() && camera->fx > 0.0 && camera->bf > 0.0 ?
          camera->bf / camera->fx : 0.0;
        baseline = baselines.emplace(submap_id, value).first;
      }
      input.stereo_baseline_m = baseline->second;
      upserts.push_back(input);
      resolved = true;
      break;
    }
    if (!resolved) {
      geometry_removals.push_back(id);
    }
  }
  std::sort(geometry_removals.begin(), geometry_removals.end());
  geometry_removals.erase(
    std::unique(geometry_removals.begin(), geometry_removals.end()),
    geometry_removals.end());
  return score_manager_.ApplyGeometryChanges(upserts, geometry_removals);
}

void SparseGlobalBackend::RefreshScoresAfterPoseChanges(
  const std::vector<PoseChangeSet> & changes)
{
  std::set<RawKeyFrameId> keyframes;
  for (const auto & change : changes) {
    keyframes.insert(change.created_ids.begin(), change.created_ids.end());
    keyframes.insert(change.updated_ids.begin(), change.updated_ids.end());
    keyframes.insert(change.invalidated_ids.begin(), change.invalidated_ids.end());
    keyframes.insert(
      change.control_propagated_ids.begin(), change.control_propagated_ids.end());
  }
  auto score_changes = RefreshGeometryScores(keyframes, {}, {});
  RefreshFusedScores(
    score_changes, &fused_landmark_manager_, &score_manager_, &score_changes);
  std::lock_guard<std::mutex> builder_lock(builder_mutex_);
  for (const auto & change : changes) {
    global_map_builder_.MarkPoseChanges(change);
  }
  global_map_builder_.MarkScoreChanges(score_changes);
}

void SparseGlobalBackend::TrackSubmapTransition(const RawInsertResult & raw_result)
{
  const RawSubmapId current = raw_result.submap_id;
  const auto previous = last_submap_by_drone_.find(current.drone_id);
  if (previous != last_submap_by_drone_.end() &&
    !(previous->second == current) && current.map_epoch > previous->second.map_epoch &&
    pose_store_.HasSubmapAnchor(previous->second))
  {
    const auto poses = pose_store_.GetSubmapPoses(previous->second);
    auto trusted = poses.end();
    for (auto it = poses.begin(); it != poses.end(); ++it) {
      if (it->second.active &&
        (trusted == poses.end() ||
        it->first.local_kf_id > trusted->first.local_kf_id))
      {
        trusted = it;
      }
    }
    if (trusted != poses.end()) {
      recent_loss_continuity_[current] = {
        previous->second, trusted->first, trusted->second.world_pose};
    }
  }
  if (previous == last_submap_by_drone_.end() ||
    current.map_epoch >= previous->second.map_epoch)
  {
    last_submap_by_drone_[current.drone_id] = current;
  }
}

bool SparseGlobalBackend::ValidateRecentLossAnchor(
  const LoopAnchorBatchEntry & entry, LoopTaskComputation * computation) const
{
  const auto record = recent_loss_continuity_.find(entry.snapshot.submap_id);
  if (record == recent_loss_continuity_.end()) {
    return true;
  }
  const auto control = FindKeyFrameIndex(entry.snapshot, entry.loop_control_keyframe_id);
  if (!control.has_value()) {
    return false;
  }
  double path_m = 0.0;
  double path_rotation = 0.0;
  for (size_t index = 1U; index <= *control; ++index) {
    Eigen::Isometry3d previous;
    Eigen::Isometry3d current;
    if (!PoseToIsometry(entry.snapshot.keyframes[index - 1U].local_pose, &previous) ||
      !PoseToIsometry(entry.snapshot.keyframes[index].local_pose, &current))
    {
      return false;
    }
    path_m += (current.translation() - previous.translation()).norm();
    path_rotation += RotationErrorRad(previous, current);
  }
  const geometry_msgs::msg::Pose proposed_world = ComposePose(
    entry.world_T_local, entry.snapshot.keyframes[*control].local_pose);
  const auto error = ComputeFiducialError(
    proposed_world, record->second.trusted_world_pose);
  const double translation_limit = loop_pipeline_config_.recent_loss_base_translation_m +
    path_m * (1.0 + loop_pipeline_config_.recent_loss_path_drift_ratio);
  const double rotation_limit = loop_pipeline_config_.recent_loss_base_rotation_rad +
    path_rotation * (1.0 + loop_pipeline_config_.recent_loss_rotation_drift_ratio);
  if (computation != nullptr) {
    computation->recent_loss_gate_checked = true;
    computation->recent_loss_translation_m = error.translation_m;
    computation->recent_loss_translation_limit_m = translation_limit;
    computation->recent_loss_rotation_rad = error.rotation_rad;
    computation->recent_loss_rotation_limit_rad = rotation_limit;
    computation->recent_loss_gate_passed =
      error.translation_m <= translation_limit && error.rotation_rad <= rotation_limit;
  }
  return error.translation_m <= translation_limit && error.rotation_rad <= rotation_limit;
}

PrimaryBackendResult SparseGlobalBackend::InsertDelta(
  uint64_t arrival_id,
  std::shared_ptr<const orbslam3_msgs::msg::OrbMap> delta)
{
  std::lock_guard<std::mutex> state_lock(state_commit_mutex_);
  PrimaryBackendResult result;
  result.had_deferred_snapshot_dirty = deferred_snapshot_dirty_;
  result.raw_result = raw_database_.InsertDelta(arrival_id, std::move(delta));
  TrackSubmapTransition(result.raw_result);
  result.score_changes = score_manager_.ApplyRawChanges(result.raw_result, raw_database_);
  result.pose_stage_executed = !result.raw_result.pose_changes.empty();
  if (result.pose_stage_executed) {
    result.pose_changes = pose_store_.ApplyRawPoseChanges(
      result.raw_result.submap_id, result.raw_result.pose_changes, arrival_id);
  }
  std::set<RawKeyFrameId> keyframes(
    result.raw_result.new_keyframe_ids.begin(), result.raw_result.new_keyframe_ids.end());
  keyframes.insert(
    result.raw_result.pose_changed_keyframe_ids.begin(),
    result.raw_result.pose_changed_keyframe_ids.end());
  keyframes.insert(
    result.raw_result.association_changed_keyframe_ids.begin(),
    result.raw_result.association_changed_keyframe_ids.end());
  keyframes.insert(result.pose_changes.created_ids.begin(), result.pose_changes.created_ids.end());
  keyframes.insert(result.pose_changes.updated_ids.begin(), result.pose_changes.updated_ids.end());
  keyframes.insert(
    result.pose_changes.invalidated_ids.begin(), result.pose_changes.invalidated_ids.end());
  std::set<RawMapPointId> mappoints(
    result.raw_result.new_mappoint_ids.begin(), result.raw_result.new_mappoint_ids.end());
  mappoints.insert(
    result.raw_result.geometry_changed_mappoint_ids.begin(),
    result.raw_result.geometry_changed_mappoint_ids.end());
  mappoints.insert(
    result.raw_result.association_changed_mappoint_ids.begin(),
    result.raw_result.association_changed_mappoint_ids.end());
  mappoints.insert(
    result.raw_result.score_input_changed_mappoint_ids.begin(),
    result.raw_result.score_input_changed_mappoint_ids.end());
  const auto geometry_changes = RefreshGeometryScores(
    keyframes, mappoints, result.raw_result.invalidated_mappoint_ids);
  MergeScoreChanges(&result.score_changes, geometry_changes);
  RefreshFusedScores(
    result.score_changes, &fused_landmark_manager_, &score_manager_,
    &result.score_changes);
  {
    std::lock_guard<std::mutex> builder_lock(builder_mutex_);
    global_map_builder_.MarkRawChanges(result.raw_result);
    global_map_builder_.MarkScoreChanges(result.score_changes);
    if (result.pose_stage_executed) {
      global_map_builder_.MarkPoseChanges(result.pose_changes);
    }
  }
  return result;
}

PrimaryBackendResult SparseGlobalBackend::InsertFullSnapshot(
  uint64_t arrival_id,
  std::shared_ptr<const orbslam3_msgs::msg::OrbMap> snapshot)
{
  std::lock_guard<std::mutex> state_lock(state_commit_mutex_);
  PrimaryBackendResult result;
  result.raw_result = raw_database_.InsertFullSnapshot(arrival_id, std::move(snapshot));
  TrackSubmapTransition(result.raw_result);
  if (!result.raw_result.has_material_changes) {
    return result;
  }

  result.score_changes = score_manager_.ApplyRawChanges(result.raw_result, raw_database_);
  result.pose_stage_executed = !result.raw_result.pose_changes.empty();
  if (result.pose_stage_executed) {
    result.pose_changes = pose_store_.ApplyRawPoseChanges(
      result.raw_result.submap_id, result.raw_result.pose_changes, arrival_id);
  }
  std::set<RawKeyFrameId> keyframes(
    result.raw_result.new_keyframe_ids.begin(), result.raw_result.new_keyframe_ids.end());
  keyframes.insert(
    result.raw_result.pose_changed_keyframe_ids.begin(),
    result.raw_result.pose_changed_keyframe_ids.end());
  keyframes.insert(
    result.raw_result.association_changed_keyframe_ids.begin(),
    result.raw_result.association_changed_keyframe_ids.end());
  keyframes.insert(result.pose_changes.created_ids.begin(), result.pose_changes.created_ids.end());
  keyframes.insert(result.pose_changes.updated_ids.begin(), result.pose_changes.updated_ids.end());
  keyframes.insert(
    result.pose_changes.invalidated_ids.begin(), result.pose_changes.invalidated_ids.end());
  std::set<RawMapPointId> mappoints(
    result.raw_result.new_mappoint_ids.begin(), result.raw_result.new_mappoint_ids.end());
  mappoints.insert(
    result.raw_result.geometry_changed_mappoint_ids.begin(),
    result.raw_result.geometry_changed_mappoint_ids.end());
  mappoints.insert(
    result.raw_result.association_changed_mappoint_ids.begin(),
    result.raw_result.association_changed_mappoint_ids.end());
  mappoints.insert(
    result.raw_result.score_input_changed_mappoint_ids.begin(),
    result.raw_result.score_input_changed_mappoint_ids.end());
  const auto geometry_changes = RefreshGeometryScores(
    keyframes, mappoints, result.raw_result.invalidated_mappoint_ids);
  MergeScoreChanges(&result.score_changes, geometry_changes);
  RefreshFusedScores(
    result.score_changes, &fused_landmark_manager_, &score_manager_,
    &result.score_changes);
  {
    std::lock_guard<std::mutex> builder_lock(builder_mutex_);
    global_map_builder_.MarkRawChanges(result.raw_result);
    global_map_builder_.MarkScoreChanges(result.score_changes);
    if (result.pose_stage_executed) {
      global_map_builder_.MarkPoseChanges(result.pose_changes);
    }
  }
  deferred_snapshot_dirty_ = true;
  return result;
}

void SparseGlobalBackend::SetFiducialPendingCapacityPerDrone(size_t capacity)
{
  raw_database_.SetFiducialPendingCapacityPerDrone(capacity);
}

FiducialBatchSubmitResult SparseGlobalBackend::SubmitFiducialBatch(
  const orbslam3_msgs::msg::FiducialKeyFrameObservations & batch)
{
  return raw_database_.SubmitFiducialBatch(batch);
}

FiducialSyncStats SparseGlobalBackend::GetFiducialSyncStats() const
{
  return raw_database_.GetFiducialSyncStats();
}

PoseChangeSet SparseGlobalBackend::CommitAnchor(
  const RawSubmapId & submap_id,
  const geometry_msgs::msg::Pose & world_T_local,
  uint64_t source_task_id)
{
  std::lock_guard<std::mutex> state_lock(state_commit_mutex_);
  const auto snapshot = raw_database_.GetSubmapPoseSnapshot(submap_id);
  if (!snapshot.has_value()) {
    PoseChangeSet result;
    result.status = PoseCommitStatus::MissingRawSubmap;
    result.submap_id = submap_id;
    result.source_task_id = source_task_id;
    const auto stats = pose_store_.GetStats();
    result.store_revision_before = stats.store_revision;
    result.store_revision_after = stats.store_revision;
    return result;
  }
  auto result = pose_store_.CommitAnchor(*snapshot, world_T_local, source_task_id);
  RefreshScoresAfterPoseChanges({result});
  if (result.status == PoseCommitStatus::Applied) {
    InvalidateRejectedLoopRegions({submap_id});
  }
  return result;
}

FiducialProcessResult SparseGlobalBackend::ProcessFiducialObservation(
  const FiducialObservation & observation,
  bool append_to_journal)
{
  std::lock_guard<std::mutex> state_lock(state_commit_mutex_);
  const auto raw_keyframe = raw_database_.GetKeyFrame(observation.keyframe_id);
  if (!raw_keyframe.has_value()) {
    FiducialProcessResult result;
    result.status = FiducialProcessStatus::MissingRawKeyFrame;
    result.observation = observation;
    result.reason = "raw_keyframe_missing";
    return result;
  }

  if (append_to_journal) {
    raw_database_.AddFiducialObservation(
      {
        observation.arrival_id, observation.keyframe_id, observation.fiducial_id,
        observation.fiducial_visit_id, observation.world_T_camera_target,
        observation.keyframe_stamp_sec,
        observation.observation_stamp_sec, observation.association_dt_sec,
        observation.distance_to_fiducial_m, observation.source, observation.quality});
  }

  const RawSubmapId submap_id{
    observation.keyframe_id.drone_id, observation.keyframe_id.map_epoch};
  const bool has_anchor = pose_store_.HasSubmapAnchor(submap_id);
  const bool loop_anchor_only = pose_store_.HasLoopDependency(submap_id);
  if (loop_anchor_only &&
    !fiducial_anchor_manager_.GetLastAcceptedControl(submap_id).has_value())
  {
    const auto loop_snapshot = raw_database_.GetSubmapPoseSnapshot(submap_id);
    if (loop_snapshot.has_value()) {
      const auto first_active = std::find_if(
        loop_snapshot->keyframes.begin(), loop_snapshot->keyframes.end(),
        [&observation](const auto & input) {
          return input.active &&
                 input.id.local_kf_id <= observation.keyframe_id.local_kf_id;
        });
      if (first_active != loop_snapshot->keyframes.end()) {
        fiducial_anchor_manager_.AcceptControl(submap_id, 0U, first_active->id);
      }
    }
  }
  auto result = fiducial_anchor_manager_.Evaluate(
    observation, raw_keyframe->pose, pose_store_.GetPose(observation.keyframe_id),
    has_anchor);
  if (loop_anchor_only && result.optimization_task.has_value()) {
    result.optimization_task->replaces_soft_loop_anchor = true;
  }
  result.journaled = append_to_journal;
  if (result.status == FiducialProcessStatus::RevisitWithinThreshold &&
    result.promote_control)
  {
    const auto current = pose_store_.GetPose(observation.keyframe_id);
    const auto revision = raw_database_.GetKeyFrameRevision(observation.keyframe_id);
    if (!current.has_value() || !revision.has_value()) {
      result.status = FiducialProcessStatus::MissingGlobalPose;
      result.reason = "coherent_control_state_missing";
      return result;
    }
    result.pose_changes = pose_store_.CommitAcceptedPoses(
      submap_id,
      {{observation.keyframe_id, current->world_pose, *revision,
        current->pose_revision, true}},
      PoseSourceKind::FiducialAccepted, observation.arrival_id,
      observation.keyframe_id);
    if (result.pose_changes.status == PoseCommitStatus::Applied) {
      fiducial_anchor_manager_.AcceptControl(
        submap_id, observation.fiducial_visit_id, observation.keyframe_id);
      result.hard_keyframe = true;
      InvalidateRejectedLoopRegions({submap_id});
    }
    return result;
  }
  if (result.status != FiducialProcessStatus::AnchorCreated) {
    return result;
  }

  const auto snapshot = raw_database_.GetSubmapPoseSnapshot(submap_id);
  if (!snapshot.has_value()) {
    result.status = FiducialProcessStatus::AnchorCommitRejected;
    result.reason = "raw_submap_missing";
    return result;
  }
  if (loop_anchor_only) {
    std::vector<AcceptedPoseUpdate> updates;
    updates.reserve(snapshot->keyframes.size());
    const auto current_poses = pose_store_.GetSubmapPoses(submap_id);
    for (const auto & input : snapshot->keyframes) {
      const auto current = current_poses.find(input.id);
      if (current == current_poses.end()) {
        result.status = FiducialProcessStatus::AnchorCommitRejected;
        result.reason = "loop_reanchor_pose_missing";
        return result;
      }
      updates.push_back({
        input.id, ComposePose(result.world_T_local, input.local_pose), input.raw_revision,
        current->second.pose_revision, input.id == observation.keyframe_id});
    }
    result.pose_changes = pose_store_.CommitAcceptedPoses(
      submap_id, updates, PoseSourceKind::FiducialAccepted,
      observation.arrival_id, observation.keyframe_id, result.world_T_local);
    result.reason = "loop_anchor_replaced_by_first_hard_fiducial";
  } else {
    result.pose_changes = pose_store_.CommitAnchor(
      *snapshot, result.world_T_local, observation.arrival_id, observation.keyframe_id);
  }
  RefreshScoresAfterPoseChanges({result.pose_changes});
  result.hard_keyframe = result.pose_changes.hard_fiducial_ids.size() == 1 &&
    result.pose_changes.hard_fiducial_ids.front() == observation.keyframe_id;
  if (result.pose_changes.status != PoseCommitStatus::Applied) {
    result.status = FiducialProcessStatus::AnchorCommitRejected;
    result.reason = ToString(result.pose_changes.status);
  } else {
    fiducial_anchor_manager_.AcceptControl(
      submap_id, observation.fiducial_visit_id, observation.keyframe_id);
    recent_loss_continuity_.erase(submap_id);
    InvalidateRejectedLoopRegions({submap_id});
  }
  return result;
}

std::optional<orbslam3_msgs::msg::OrbKeyFrame> SparseGlobalBackend::GetRawKeyFrame(
  const RawKeyFrameId & id) const
{
  return raw_database_.GetKeyFrame(id);
}

std::vector<RecordedFiducialObservation>
SparseGlobalBackend::GetFiducialObservationJournal() const
{
  return raw_database_.GetFiducialObservationJournal();
}

SecondaryWorkPlan SparseGlobalBackend::PlanSecondaryWork(
  const RawInsertResult & raw_result) const
{
  SecondaryWorkPlan plan;
  if (!raw_result.has_material_changes) {
    return plan;
  }
  std::set<RawKeyFrameId> eligible;
  eligible.insert(raw_result.new_keyframe_ids.begin(), raw_result.new_keyframe_ids.end());
  eligible.insert(
    raw_result.pose_changed_keyframe_ids.begin(),
    raw_result.pose_changed_keyframe_ids.end());
  eligible.insert(
    raw_result.association_changed_keyframe_ids.begin(),
    raw_result.association_changed_keyframe_ids.end());
  eligible.insert(
    raw_result.covisibility_changed_keyframe_ids.begin(),
    raw_result.covisibility_changed_keyframe_ids.end());
  for (const auto & invalid : raw_result.invalidated_keyframe_ids) {
    eligible.erase(invalid);
  }
  std::vector<RawKeyFrameId> loop_ids(eligible.begin(), eligible.end());
  if (!raw_result.covisibility_changed_keyframe_ids.empty()) {
    DatabaseUpdateTask task;
    task.source_arrival_id = raw_result.arrival_id;
    task.submap_id = raw_result.submap_id;
    task.expected_submap_revision = raw_result.submap_revision;
    task.covisibility_keyframe_ids = raw_result.covisibility_changed_keyframe_ids;
    task.loop_keyframe_ids = std::move(loop_ids);
    plan.database_update = std::move(task);
  } else {
    plan.direct_loop_tasks = CreateLoopTasks(raw_result.arrival_id, loop_ids);
  }
  return plan;
}

std::vector<LoopTask> SparseGlobalBackend::CreateLoopTasks(
  uint64_t source_arrival_id,
  const std::vector<RawKeyFrameId> & keyframe_ids) const
{
  std::vector<LoopTask> tasks;
  std::set<RawKeyFrameId> unique(keyframe_ids.begin(), keyframe_ids.end());
  tasks.reserve(unique.size());
  for (const auto & id : unique) {
    const auto raw = raw_database_.GetKeyFrame(id);
    const auto revision = raw_database_.GetLoopSemanticRevision(
      id, loop_pipeline_config_.strong_covisibility_support,
      loop_pipeline_config_.min_query_mappoints);
    if (!raw.has_value() || !revision.has_value() || raw->is_bad) {
      continue;
    }
    LoopTask task;
    task.source_arrival_id = source_arrival_id;
    task.query_keyframe_id = id;
    task.revision.raw_revision = revision->raw_revision;
    task.revision.appearance_revision = revision->appearance_revision;
    task.revision.geometry_revision = revision->geometry_revision;
    task.revision.validation_revision = revision->validation_revision;
    task.revision.anchor_revision = pose_store_.GetSubmapAnchorRevision(
      {id.drone_id, id.map_epoch});
    tasks.push_back(std::move(task));
  }
  return tasks;
}

std::vector<LoopTask> SparseGlobalBackend::CreateFusionRefreshTasks(
  uint64_t source_arrival_id,
  const std::vector<RawKeyFrameId> & keyframe_ids) const
{
  auto candidates = CreateLoopTasks(source_arrival_id, keyframe_ids);
  std::map<std::tuple<uint32_t, uint64_t, uint64_t>, LoopTask> regions;
  const uint64_t region_width = std::max<uint64_t>(
    1U, 2U * loop_pipeline_config_.temporal_window_radius + 1U);
  for (auto & task : candidates) {
    task.intent = LoopTaskIntent::FusionRefresh;
    const auto key = std::make_tuple(
      task.query_keyframe_id.drone_id, task.query_keyframe_id.map_epoch,
      task.query_keyframe_id.local_kf_id / region_width);
    const auto found = regions.find(key);
    if (found == regions.end() || task.query_keyframe_id.local_kf_id >
      found->second.query_keyframe_id.local_kf_id)
    {
      regions[key] = std::move(task);
    }
  }
  std::vector<LoopTask> grouped;
  grouped.reserve(regions.size());
  for (auto & [key, task] : regions) {
    (void)key;
    grouped.push_back(std::move(task));
  }
  return grouped;
}

bool SparseGlobalBackend::IsProtectedLoopRegion(const RawKeyFrameId & keyframe_id) const
{
  auto directly_protected = [&](const RawKeyFrameId & id) {
      const auto pose = pose_store_.GetPose(id);
      return pose.has_value() && pose->active &&
             (pose->hard_fiducial ||
             pose->source_kind == PoseSourceKind::FiducialAccepted ||
             pose->source_kind == PoseSourceKind::FiducialOptimized ||
             pose_store_.GetHardCorridorReference(id).has_value());
    };
  if (directly_protected(keyframe_id)) {
    return true;
  }
  const RawSubmapId submap{keyframe_id.drone_id, keyframe_id.map_epoch};
  const auto active = raw_database_.GetActiveSubmapEntityIds(submap);
  if (active.has_value()) {
    for (const auto & id : active->keyframe_ids) {
      const uint64_t gap = id.local_kf_id > keyframe_id.local_kf_id ?
        id.local_kf_id - keyframe_id.local_kf_id : keyframe_id.local_kf_id - id.local_kf_id;
      if (gap <= loop_pipeline_config_.temporal_window_radius && directly_protected(id)) {
        return true;
      }
    }
  }
  for (const auto & edge : covisibility_database_.GetNeighbors(
      keyframe_id, loop_pipeline_config_.strong_covisibility_support, 32U))
  {
    const auto neighbor = edge.kf_a == keyframe_id ? edge.kf_b : edge.kf_a;
    if (directly_protected(neighbor)) {
      return true;
    }
  }
  return false;
}

SparseGlobalBackend::LoopRejectionKey SparseGlobalBackend::BuildLoopRejectionKey(
  const LoopGeometryResult & geometry) const
{
  const uint64_t region_width =
    std::max<uint64_t>(1U, 2U * loop_pipeline_config_.temporal_window_radius + 1U);
  const auto & translation = geometry.candidate_local_T_query_local.translation();
  const double yaw = YawFromRotation(geometry.candidate_local_T_query_local.linear());
  return {
    geometry.query_submap_id.drone_id, geometry.query_submap_id.map_epoch,
    geometry.query_keyframe_id.local_kf_id / region_width,
    geometry.candidate_submap_id.drone_id, geometry.candidate_submap_id.map_epoch,
    geometry.candidate_keyframe_id.local_kf_id / region_width,
    static_cast<int64_t>(std::llround(translation.x())),
    static_cast<int64_t>(std::llround(translation.y())),
    static_cast<int64_t>(std::llround(translation.z())),
    static_cast<int64_t>(std::llround(yaw / 0.35)),
    pose_store_.GetSubmapAnchorRevision(geometry.query_submap_id),
    pose_store_.GetSubmapAnchorRevision(geometry.candidate_submap_id)};
}

void SparseGlobalBackend::ApplyProtectedRegionGuard(LoopTaskComputation * computation)
{
  if (computation == nullptr ||
    computation->decision != LoopTaskDecisionKind::OptimizationEvidence)
  {
    return;
  }
  std::vector<size_t> accepted_indices;
  for (const size_t index : computation->optimization_geometry_indices) {
    if (index >= computation->geometry_results.size()) {
      continue;
    }
    const auto & geometry = computation->geometry_results[index];
    const auto key = BuildLoopRejectionKey(geometry);
    {
      std::lock_guard<std::mutex> lock(loop_rejection_mutex_);
      if (loop_rejection_ledger_.count(key) != 0U) {
        computation->rejection_ledger_hit = true;
        continue;
      }
    }
    computation->protected_region_checked = true;
    const bool query_stable = IsProtectedLoopRegion(geometry.query_keyframe_id);
    const bool candidate_stable = IsProtectedLoopRegion(geometry.candidate_keyframe_id);
    computation->protected_query_stable =
      computation->protected_query_stable || query_stable;
    computation->protected_candidate_stable =
      computation->protected_candidate_stable || candidate_stable;
    computation->protected_translation_error_m = std::max(
      computation->protected_translation_error_m, geometry.current_translation_error_m);
    computation->protected_rotation_error_rad = std::max(
      computation->protected_rotation_error_rad, geometry.current_rotation_error_rad);
    if (query_stable && candidate_stable &&
      (geometry.current_translation_error_m >
      loop_pipeline_config_.hard_corridor_max_translation_m ||
      geometry.current_rotation_error_rad >
      loop_pipeline_config_.hard_corridor_max_rotation_rad))
    {
      computation->protected_region_rejected = true;
      std::lock_guard<std::mutex> lock(loop_rejection_mutex_);
      loop_rejection_ledger_.insert(key);
      continue;
    }
    accepted_indices.push_back(index);
  }
  computation->optimization_geometry_indices = std::move(accepted_indices);
  if (computation->optimization_geometry_indices.empty()) {
    computation->decision = LoopTaskDecisionKind::GeometryRejected;
    computation->reason = computation->rejection_ledger_hit ?
      "regional_rejection_ledger_hit" : "protected_region_far_repeated_loop";
    computation->optimization.reason = computation->reason;
  }
}

void SparseGlobalBackend::RememberRejectedLoopRegions(
  const LoopTaskComputation & computation)
{
  if (!computation.optimization.graph_built ||
    computation.decision != LoopTaskDecisionKind::GeometryRejected)
  {
    return;
  }
  const std::string & reason = computation.reason;
  if (reason.find("degraded") == std::string::npos &&
    reason.find("corridor") == std::string::npos &&
    reason.find("structure") == std::string::npos &&
    reason.find("hard") == std::string::npos)
  {
    return;
  }
  std::vector<LoopRejectionKey> rejected;
  for (const size_t index : computation.optimization_geometry_indices) {
    if (index < computation.geometry_results.size()) {
      rejected.push_back(BuildLoopRejectionKey(computation.geometry_results[index]));
    }
  }
  std::lock_guard<std::mutex> lock(loop_rejection_mutex_);
  loop_rejection_ledger_.insert(rejected.begin(), rejected.end());
  while (loop_rejection_ledger_.size() > 1024U) {
    loop_rejection_ledger_.erase(loop_rejection_ledger_.begin());
  }
}

void SparseGlobalBackend::InvalidateRejectedLoopRegions(
  const std::set<RawSubmapId> & submaps)
{
  if (submaps.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(loop_rejection_mutex_);
  for (auto it = loop_rejection_ledger_.begin(); it != loop_rejection_ledger_.end();) {
    const RawSubmapId query{std::get<0>(*it), std::get<1>(*it)};
    const RawSubmapId candidate{std::get<3>(*it), std::get<4>(*it)};
    if (submaps.count(query) != 0U || submaps.count(candidate) != 0U) {
      it = loop_rejection_ledger_.erase(it);
    } else {
      ++it;
    }
  }
}

CovisibilityUpdateResult SparseGlobalBackend::ProcessDatabaseUpdate(
  const DatabaseUpdateTask & task)
{
  DatabaseUpdateTask current = task;
  const auto snapshot = raw_database_.GetSubmapPoseSnapshot(task.submap_id);
  if (!snapshot.has_value()) {
    CovisibilityUpdateResult result;
    result.stale = true;
    result.reason = "raw_submap_missing";
    return result;
  }
  current.expected_submap_revision = snapshot->submap_revision;
  const auto patch = CovisibilityDatabase::PrepareOrbslam3Patch(
    raw_database_, current);
  if (!patch.has_value()) {
    CovisibilityUpdateResult result;
    result.stale = true;
    result.reason = "raw_revision_changed_during_prepare";
    return result;
  }
  return covisibility_database_.ApplyPatch(*patch);
}

LoopTaskComputation SparseGlobalBackend::CommitLoopFusion(
  LoopTaskComputation computation)
{
  computation.fusion.attempted = true;
  const auto prepare_start = std::chrono::steady_clock::now();
  const uint64_t expected_score_revision = score_manager_.GetStats().score_revision;
  const uint64_t expected_covisibility_revision =
    covisibility_database_.GetStats().revision;
  auto prepared = fused_landmark_manager_.PrepareFusion(
    computation, raw_database_, pose_store_, score_manager_);
  auto covisibility_patch = BuildServerCovisibilityPatch(
    computation, raw_database_, pose_store_, expected_covisibility_revision);
  computation.fusion.prepare_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - prepare_start).count();
  computation.fusion.pair_results = prepared.patch.pair_results.size();
  computation.fusion.score_positive_events = prepared.patch.positive_score_events;
  computation.fusion.score_negative_events = prepared.patch.negative_score_events;
  computation.fusion.score_visibility_diagnostics =
    prepared.patch.visibility_diagnostic_events;
  computation.fusion.visibility_regions_started =
    prepared.patch.visibility_regions_started;
  computation.fusion.visibility_regions_completed =
    prepared.patch.visibility_regions_completed;
  computation.fusion.visibility_projected_points =
    prepared.patch.visibility_projected_points;
  computation.fusion.visibility_ms = prepared.patch.visibility_elapsed_ms;
  if (!prepared.ready) {
    computation.decision = LoopTaskDecisionKind::GeometryRejected;
    computation.reason = prepared.reason;
    computation.fusion.reason = prepared.reason;
    return computation;
  }
  computation.fusion.prepared = true;

  ScorePatch score_patch;
  score_patch.expected_score_revision = expected_score_revision;
  score_patch.raw_evidence = prepared.patch.raw_score_evidence;
  score_patch.fused_upserts = prepared.patch.fused_score_updates;
  score_patch.fused_removals = prepared.patch.fused_score_removals;

  const auto commit_start = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> state_lock(state_commit_mutex_);
  const auto query_revision = raw_database_.GetLoopSemanticRevision(
    computation.task.query_keyframe_id,
    loop_pipeline_config_.strong_covisibility_support,
    loop_pipeline_config_.min_query_mappoints);
  bool stale = !query_revision.has_value() ||
    query_revision->appearance_revision != computation.task.revision.appearance_revision ||
    query_revision->validation_revision != computation.task.revision.validation_revision;
  for (const auto & [id, revision] : prepared.patch.expected_raw_revisions) {
    const auto latest = raw_database_.GetMapPointRevision(id);
    stale = stale || !latest.has_value() || *latest != revision;
  }
  for (const auto & [id, revision] : prepared.patch.expected_pose_revisions) {
    const auto latest = pose_store_.GetPose(id);
    stale = stale || !latest.has_value() || !latest->active ||
      latest->pose_revision != revision;
  }
  if (stale) {
    computation.decision = LoopTaskDecisionKind::Stale;
    computation.reason = "fusion_dependencies_changed_before_commit";
    computation.fusion.stale = true;
    computation.fusion.reason = computation.reason;
    computation.fusion.commit_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - commit_start).count();
    return computation;
  }

  auto fusion_apply = fused_landmark_manager_.ApplyPatch(prepared.patch);
  if (!fusion_apply.committed) {
    computation.decision = LoopTaskDecisionKind::Stale;
    computation.reason = fusion_apply.reason;
    computation.fusion.stale = fusion_apply.stale;
    computation.fusion.reason = fusion_apply.reason;
    return computation;
  }
  auto covisibility_apply =
    covisibility_database_.ApplyPatchTransactional(covisibility_patch);
  if (!covisibility_apply.update.committed) {
    computation.fusion.rolled_back =
      fused_landmark_manager_.RollbackPatch(fusion_apply.rollback);
    computation.decision = LoopTaskDecisionKind::Stale;
    computation.reason = covisibility_apply.update.reason;
    computation.fusion.stale = covisibility_apply.update.stale;
    computation.fusion.reason = computation.reason;
    return computation;
  }
  auto score_apply = score_manager_.ApplyPatch(score_patch);
  if (!score_apply.committed) {
    const bool covisibility_rolled_back =
      covisibility_database_.RollbackPatch(covisibility_apply.rollback);
    const bool fusion_rolled_back =
      fused_landmark_manager_.RollbackPatch(fusion_apply.rollback);
    computation.fusion.rolled_back =
      covisibility_rolled_back && fusion_rolled_back;
    computation.decision = LoopTaskDecisionKind::Stale;
    computation.reason = score_apply.reason;
    computation.fusion.stale = score_apply.stale;
    computation.fusion.reason = computation.reason;
    return computation;
  }

  {
    std::lock_guard<std::mutex> builder_lock(builder_mutex_);
    global_map_builder_.MarkFusionChanges(fusion_apply.changes);
    global_map_builder_.MarkScoreChanges(score_apply.changes);
  }
  computation.fusion.committed = true;
  computation.fusion.tracks_created = fusion_apply.changes.created_track_ids.size();
  computation.fusion.tracks_updated = fusion_apply.changes.updated_track_ids.size();
  computation.fusion.tracks_retired = fusion_apply.changes.retired_track_ids.size();
  computation.fusion.hidden_raw_members =
    fusion_apply.changes.hidden_raw_member_ids.size();
  computation.fusion.score_raw_updates = score_apply.changes.updated_ids.size();
  computation.fusion.covisibility_added = covisibility_apply.update.added;
  computation.fusion.covisibility_updated = covisibility_apply.update.updated;
  computation.fusion.reason = computation.optimization.committed ?
    "loop_fusion_after_optimization_committed" : "loop_fusion_committed";
  computation.fusion.commit_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - commit_start).count();
  computation.reason = computation.fusion.reason;
  std::set<RawSubmapId> changed_submaps;
  for (const auto & geometry : computation.geometry_results) {
    if (geometry.accepted && geometry.fusion_compatible) {
      changed_submaps.insert(geometry.query_submap_id);
      changed_submaps.insert(geometry.candidate_submap_id);
    }
  }
  InvalidateRejectedLoopRegions(changed_submaps);
  return computation;
}

LoopTaskComputation SparseGlobalBackend::ProcessLoopTask(const LoopTask & task)
{
  auto computation = loop_pipeline_.Process(
    task, raw_database_, pose_store_, covisibility_database_);
  if (task.intent == LoopTaskIntent::Full &&
    computation.decision == LoopTaskDecisionKind::OptimizationEvidence)
  {
    ApplyProtectedRegionGuard(&computation);
  }
  if (computation.decision == LoopTaskDecisionKind::FusionCandidate) {
    computation.fusion.attempted = true;
    const auto prepare_start = std::chrono::steady_clock::now();
    const uint64_t expected_score_revision = score_manager_.GetStats().score_revision;
    const uint64_t expected_covisibility_revision =
      covisibility_database_.GetStats().revision;
    auto prepared = fused_landmark_manager_.PrepareFusion(
      computation, raw_database_, pose_store_, score_manager_);
    auto covisibility_patch = BuildServerCovisibilityPatch(
      computation, raw_database_, pose_store_, expected_covisibility_revision);
    computation.fusion.prepare_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - prepare_start).count();
    computation.fusion.pair_results = prepared.patch.pair_results.size();
    computation.fusion.score_positive_events = prepared.patch.positive_score_events;
    computation.fusion.score_negative_events = prepared.patch.negative_score_events;
    computation.fusion.score_visibility_diagnostics =
      prepared.patch.visibility_diagnostic_events;
    computation.fusion.visibility_regions_started =
      prepared.patch.visibility_regions_started;
    computation.fusion.visibility_regions_completed =
      prepared.patch.visibility_regions_completed;
    computation.fusion.visibility_projected_points =
      prepared.patch.visibility_projected_points;
    computation.fusion.visibility_ms = prepared.patch.visibility_elapsed_ms;
    if (!prepared.ready) {
      computation.decision = LoopTaskDecisionKind::GeometryRejected;
      computation.reason = prepared.reason;
      computation.fusion.reason = prepared.reason;
      return computation;
    }
    computation.fusion.prepared = true;

    ScorePatch score_patch;
    score_patch.expected_score_revision = expected_score_revision;
    score_patch.raw_evidence = prepared.patch.raw_score_evidence;
    score_patch.fused_upserts = prepared.patch.fused_score_updates;
    score_patch.fused_removals = prepared.patch.fused_score_removals;

    const auto commit_start = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> state_lock(state_commit_mutex_);
    const auto query_revision = raw_database_.GetLoopSemanticRevision(
      task.query_keyframe_id, loop_pipeline_config_.strong_covisibility_support,
      loop_pipeline_config_.min_query_mappoints);
    bool stale = !query_revision.has_value() ||
      query_revision->appearance_revision != task.revision.appearance_revision ||
      query_revision->validation_revision != task.revision.validation_revision;
    for (const auto & [id, revision] : prepared.patch.expected_raw_revisions) {
      const auto latest = raw_database_.GetMapPointRevision(id);
      stale = stale || !latest.has_value() || *latest != revision;
    }
    for (const auto & [id, revision] : prepared.patch.expected_pose_revisions) {
      const auto latest = pose_store_.GetPose(id);
      stale = stale || !latest.has_value() || !latest->active ||
        latest->pose_revision != revision;
    }
    if (stale) {
      computation.decision = LoopTaskDecisionKind::Stale;
      computation.reason = "fusion_dependencies_changed_before_commit";
      computation.fusion.stale = true;
      computation.fusion.reason = computation.reason;
      computation.fusion.commit_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - commit_start).count();
      return computation;
    }

    auto fusion_apply = fused_landmark_manager_.ApplyPatch(prepared.patch);
    if (!fusion_apply.committed) {
      computation.decision = LoopTaskDecisionKind::Stale;
      computation.reason = fusion_apply.reason;
      computation.fusion.stale = fusion_apply.stale;
      computation.fusion.reason = fusion_apply.reason;
      return computation;
    }
    auto covisibility_apply =
      covisibility_database_.ApplyPatchTransactional(covisibility_patch);
    if (!covisibility_apply.update.committed) {
      computation.fusion.rolled_back =
        fused_landmark_manager_.RollbackPatch(fusion_apply.rollback);
      computation.decision = LoopTaskDecisionKind::Stale;
      computation.reason = covisibility_apply.update.reason;
      computation.fusion.stale = covisibility_apply.update.stale;
      computation.fusion.reason = computation.reason;
      return computation;
    }
    auto score_apply = score_manager_.ApplyPatch(score_patch);
    if (!score_apply.committed) {
      const bool covisibility_rolled_back =
        covisibility_database_.RollbackPatch(covisibility_apply.rollback);
      const bool fusion_rolled_back =
        fused_landmark_manager_.RollbackPatch(fusion_apply.rollback);
      computation.fusion.rolled_back =
        covisibility_rolled_back && fusion_rolled_back;
      computation.decision = LoopTaskDecisionKind::Stale;
      computation.reason = score_apply.reason;
      computation.fusion.stale = score_apply.stale;
      computation.fusion.reason = computation.reason;
      return computation;
    }

    {
      std::lock_guard<std::mutex> builder_lock(builder_mutex_);
      global_map_builder_.MarkFusionChanges(fusion_apply.changes);
      global_map_builder_.MarkScoreChanges(score_apply.changes);
    }
    computation.fusion.committed = true;
    computation.fusion.tracks_created = fusion_apply.changes.created_track_ids.size();
    computation.fusion.tracks_updated = fusion_apply.changes.updated_track_ids.size();
    computation.fusion.tracks_retired = fusion_apply.changes.retired_track_ids.size();
    computation.fusion.hidden_raw_members =
      fusion_apply.changes.hidden_raw_member_ids.size();
    computation.fusion.score_raw_updates = score_apply.changes.updated_ids.size();
    computation.fusion.covisibility_added = covisibility_apply.update.added;
    computation.fusion.covisibility_updated = covisibility_apply.update.updated;
    computation.fusion.reason = "loop_fusion_committed";
    computation.fusion.commit_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - commit_start).count();
    computation.reason = computation.fusion.reason;
    std::set<RawSubmapId> changed_submaps;
    for (const auto & geometry : computation.geometry_results) {
      if (geometry.accepted && geometry.fusion_compatible) {
        changed_submaps.insert(geometry.query_submap_id);
        changed_submaps.insert(geometry.candidate_submap_id);
      }
    }
    InvalidateRejectedLoopRegions(changed_submaps);
    return computation;
  }
  if (task.intent == LoopTaskIntent::FusionRefresh &&
    computation.decision == LoopTaskDecisionKind::OptimizationEvidence)
  {
    computation.decision = LoopTaskDecisionKind::Deferred;
    computation.reason = "fusion_refresh_optimization_deferred";
    computation.optimization.reason = computation.reason;
    return computation;
  }
  if (computation.decision != LoopTaskDecisionKind::AnchorProposed ||
    computation.anchor_entries.empty())
  {
    return computation;
  }

  std::lock_guard<std::mutex> state_lock(state_commit_mutex_);
  const auto query_revision = raw_database_.GetLoopSemanticRevision(
    task.query_keyframe_id, loop_pipeline_config_.strong_covisibility_support,
    loop_pipeline_config_.min_query_mappoints);
  if (!query_revision.has_value() ||
    query_revision->appearance_revision != task.revision.appearance_revision ||
    query_revision->validation_revision != task.revision.validation_revision)
  {
    computation.decision = LoopTaskDecisionKind::Stale;
    computation.reason = "query_changed_before_anchor_commit";
    computation.anchor_entries.clear();
    return computation;
  }
  for (auto & entry : computation.anchor_entries) {
    const auto latest = raw_database_.GetSubmapPoseSnapshot(entry.snapshot.submap_id);
    if (!latest.has_value() || pose_store_.HasSubmapAnchor(entry.snapshot.submap_id)) {
      computation.decision = LoopTaskDecisionKind::Stale;
      computation.reason = "anchor_component_changed_before_commit";
      computation.anchor_entries.clear();
      return computation;
    }
    entry.snapshot = *latest;
    if (!ValidateRecentLossAnchor(entry, &computation)) {
      computation.decision = LoopTaskDecisionKind::Deferred;
      computation.reason = "recent_loss_continuity_rejected";
      computation.anchor_entries.clear();
      return computation;
    }
  }
  computation.anchor_commit = pose_store_.CommitLoopAnchorBatch(
    computation.anchor_entries, task.task_id);
  if (computation.anchor_commit.status != PoseCommitStatus::Applied) {
    computation.decision = LoopTaskDecisionKind::Stale;
    computation.reason = ToString(computation.anchor_commit.status);
    return computation;
  }
  RefreshScoresAfterPoseChanges(computation.anchor_commit.submap_changes);
  for (const auto & entry : computation.anchor_entries) {
    recent_loss_continuity_.erase(entry.snapshot.submap_id);
    const auto ids = raw_database_.GetActiveSubmapEntityIds(entry.snapshot.submap_id);
    if (ids.has_value()) {
      computation.rerun_keyframe_ids.insert(
        computation.rerun_keyframe_ids.end(), ids->keyframe_ids.begin(),
        ids->keyframe_ids.end());
    }
  }
  std::set<RawSubmapId> anchored_submaps(
    computation.anchor_commit.anchored_submaps.begin(),
    computation.anchor_commit.anchored_submaps.end());
  InvalidateRejectedLoopRegions(anchored_submaps);
  computation.reason = "loop_anchor_batch_committed";
  return computation;
}

AcceptedPoseBatchResult SparseGlobalBackend::CommitGraphProposal(
  const PoseGraphProblem & problem,
  const OptimizationProposal & proposal,
  PoseSourceKind source_kind,
  uint64_t source_task_id,
  const std::optional<RawKeyFrameId> & hard_fiducial_keyframe)
{
  AcceptedPoseBatchResult failed;
  failed.source_task_id = source_task_id;
  std::lock_guard<std::mutex> state_lock(state_commit_mutex_);
  std::map<RawKeyFrameId, geometry_msgs::msg::Pose> proposed_controls;
  for (const auto & control : proposal.controls) {
    proposed_controls[control.id] = control.world_pose;
  }
  std::set<size_t> required_control_indices;
  for (const auto & edge : problem.loop_edges) {
    required_control_indices.insert(edge.from_index);
    required_control_indices.insert(edge.to_index);
  }
  for (const size_t index : problem.control_indices) {
    if (problem.keyframes[index].fixed) {
      required_control_indices.insert(index);
    }
  }
  size_t rebased_skipped_controls = 0U;
  size_t rebased_inactive_controls = 0U;
  struct ControlCorrection
  {
    uint64_t local_kf_id = 0;
    Eigen::Isometry3d correction = Eigen::Isometry3d::Identity();
  };
  std::vector<AcceptedSubmapPoseBatch> batches;
  for (const auto & window : problem.submap_windows) {
    const auto snapshot = raw_database_.GetSubmapPoseSnapshot(window.submap_id);
    if (!snapshot.has_value()) {
      failed.status = PoseCommitStatus::RevisionConflict;
      failed.detail = "commit_snapshot_missing";
      return failed;
    }
    const auto current_poses = pose_store_.GetSubmapPoses(window.submap_id);
    std::vector<ControlCorrection> corrections;
    size_t active_corrections = 0U;
    for (const size_t index : problem.control_indices) {
      const auto & graph_keyframe = problem.keyframes[index];
      if (!(RawSubmapId{graph_keyframe.id.drone_id, graph_keyframe.id.map_epoch} ==
        window.submap_id))
      {
        continue;
      }
      const auto proposed = proposed_controls.find(graph_keyframe.id);
      if (proposed == proposed_controls.end()) {
        failed.status = PoseCommitStatus::AtomicBatchConflict;
        failed.detail = "commit_proposed_control_missing";
        return failed;
      }
      Eigen::Isometry3d desired;
      Eigen::Isometry3d current;
      if (!PoseToIsometry(proposed->second, &desired) ||
        !PoseToIsometry(graph_keyframe.current_world_pose, &current))
      {
        failed.status = PoseCommitStatus::AtomicBatchConflict;
        failed.detail = "commit_control_pose_invalid";
        return failed;
      }
      if (source_kind == PoseSourceKind::LoopOptimized) {
        const auto latest_raw_index = FindKeyFrameIndex(*snapshot, graph_keyframe.id);
        const auto latest_pose = current_poses.find(graph_keyframe.id);
        const bool required = required_control_indices.count(index) > 0U;
        if (!latest_raw_index.has_value() || latest_pose == current_poses.end())
        {
          if (required) {
            failed.status = PoseCommitStatus::RevisionConflict;
            failed.detail = "commit_required_control_missing_or_inactive";
            return failed;
          }
          ++rebased_skipped_controls;
          continue;
        }
        Eigen::Isometry3d original_raw;
        Eigen::Isometry3d latest_raw;
        if (!PoseToIsometry(graph_keyframe.raw_local_pose, &original_raw) ||
          !PoseToIsometry(
            snapshot->keyframes[*latest_raw_index].local_pose, &latest_raw))
        {
          failed.status = PoseCommitStatus::AtomicBatchConflict;
          failed.detail = "commit_raw_pose_invalid";
          return failed;
        }
        const Eigen::Isometry3d raw_delta = original_raw.inverse() * latest_raw;
        const double raw_translation = raw_delta.translation().norm();
        const double raw_rotation =
          Eigen::AngleAxisd(raw_delta.linear()).angle();
        if (raw_translation > loop_pipeline_config_.hypothesis_translation_tolerance_m ||
          raw_rotation > loop_pipeline_config_.hypothesis_rotation_tolerance_rad)
        {
          if (required) {
            failed.status = PoseCommitStatus::RevisionConflict;
            failed.detail = "commit_required_control_raw_drift";
            return failed;
          }
          ++rebased_skipped_controls;
          continue;
        }
        if (!snapshot->keyframes[*latest_raw_index].active || !latest_pose->second.active) {
          ++rebased_inactive_controls;
        } else {
          ++active_corrections;
        }
      }
      corrections.push_back(
        {graph_keyframe.id.local_kf_id, desired * current.inverse()});
    }
    std::sort(
      corrections.begin(), corrections.end(),
      [](const auto & lhs, const auto & rhs) {
        return lhs.local_kf_id < rhs.local_kf_id;
      });
    if (corrections.size() < 2U ||
      (source_kind == PoseSourceKind::LoopOptimized && active_corrections < 2U))
    {
      failed.status = PoseCommitStatus::AtomicBatchConflict;
      failed.detail = "commit_insufficient_active_controls";
      return failed;
    }

    AcceptedSubmapPoseBatch batch;
    batch.submap_id = window.submap_id;
    batch.continuation_control = window.continuation_keyframe_id;
    bool past_window = false;
    for (const auto & raw : snapshot->keyframes) {
      if (raw.id.local_kf_id < window.first_keyframe_id.local_kf_id || !raw.active) {
        continue;
      }
      if (raw.id.local_kf_id > window.last_keyframe_id.local_kf_id) {
        past_window = true;
      }
      const auto current_pose = current_poses.find(raw.id);
      if (current_pose == current_poses.end() || !current_pose->second.active) {
        continue;
      }
      if (past_window && current_pose->second.hard_fiducial) {
        break;
      }
      Eigen::Isometry3d correction = Eigen::Isometry3d::Identity();
      auto upper = std::lower_bound(
        corrections.begin(), corrections.end(), raw.id.local_kf_id,
        [](const ControlCorrection & item, uint64_t id) {
          return item.local_kf_id < id;
        });
      if (upper != corrections.end() && upper->local_kf_id == raw.id.local_kf_id) {
        correction = upper->correction;
      } else if (upper == corrections.begin()) {
        correction = upper->correction;
      } else if (upper == corrections.end()) {
        correction = corrections.back().correction;
      } else {
        const auto & lower = *(upper - 1);
        const double alpha = static_cast<double>(
          raw.id.local_kf_id - lower.local_kf_id) /
          static_cast<double>(upper->local_kf_id - lower.local_kf_id);
        correction = InterpolateIsometry(
          lower.correction, upper->correction, std::clamp(alpha, 0.0, 1.0));
      }
      Eigen::Isometry3d current;
      if (!PoseToIsometry(current_pose->second.world_pose, &current)) {
        failed.status = PoseCommitStatus::AtomicBatchConflict;
        failed.detail = "commit_current_pose_invalid";
        return failed;
      }
      batch.updates.push_back(
        {raw.id, IsometryToPose(correction * current), raw.raw_revision,
          0U,
          hard_fiducial_keyframe.has_value() && raw.id == *hard_fiducial_keyframe});
    }
    if (batch.continuation_control.has_value() && std::none_of(
        batch.updates.begin(), batch.updates.end(),
        [&batch](const AcceptedPoseUpdate & update) {
          return update.keyframe_id == *batch.continuation_control;
        }))
    {
      if (batch.updates.empty()) {
        failed.status = PoseCommitStatus::RevisionConflict;
        failed.detail = "commit_continuation_missing";
        return failed;
      }
      batch.continuation_control = batch.updates.back().keyframe_id;
    }
    batches.push_back(std::move(batch));
  }

  if (hard_fiducial_keyframe.has_value() && std::none_of(
      batches.begin(), batches.end(),
      [&hard_fiducial_keyframe](const AcceptedSubmapPoseBatch & batch) {
        return std::any_of(
          batch.updates.begin(), batch.updates.end(),
          [&hard_fiducial_keyframe](const AcceptedPoseUpdate & update) {
            return update.keyframe_id == *hard_fiducial_keyframe &&
                   update.mark_hard_fiducial;
          });
      }))
  {
    failed.status = PoseCommitStatus::RevisionConflict;
    failed.detail = "commit_hard_fiducial_missing";
    return failed;
  }

  auto result = pose_store_.CommitAcceptedPoseBatch(
    batches, source_kind, source_task_id);
  result.rebased_skipped_controls = rebased_skipped_controls;
  result.rebased_inactive_controls = rebased_inactive_controls;
  if (result.status != PoseCommitStatus::Applied && result.detail.empty()) {
    result.detail = "commit_pose_store_" + std::string(ToString(result.status));
  }
  if (result.status == PoseCommitStatus::Applied) {
    RefreshScoresAfterPoseChanges(result.submap_changes);
  }
  return result;
}

LoopTaskComputation SparseGlobalBackend::ProcessLoopOptimization(
  LoopTaskComputation computation)
{
  computation.optimization.attempted = true;
  if (computation.decision != LoopTaskDecisionKind::OptimizationEvidence) {
    computation.optimization.reason = "optimization_evidence_missing";
    return computation;
  }

  PoseGraphBuildResult graph;
  OptimizationProposal proposal;
  ValidationResult validation;
  for (size_t pass = 0U; pass < 2U; ++pass) {
    const auto graph_start = std::chrono::steady_clock::now();
    std::string capture_failure;
    {
      std::lock_guard<std::mutex> state_lock(state_commit_mutex_);
      std::map<RawSubmapId, RawSubmapPoseSnapshot> snapshots;
      std::map<RawKeyFrameId, GlobalPoseRecord> poses;
      std::map<RawSubmapId, LoopAnchorDependencySnapshot> dependencies;
      auto capture_submap = [&](const RawSubmapId & submap) {
          if (snapshots.count(submap) != 0U) {
            return true;
          }
          const auto snapshot = raw_database_.GetSubmapPoseSnapshot(submap);
          if (!snapshot.has_value()) {
            return false;
          }
          snapshots[submap] = *snapshot;
          const auto submap_poses = pose_store_.GetSubmapPoses(submap);
          poses.insert(submap_poses.begin(), submap_poses.end());
          return true;
        };
      std::set<RawSubmapId> component;
      for (const size_t geometry_index : computation.optimization_geometry_indices) {
        if (geometry_index >= computation.geometry_results.size()) {
          capture_failure = "loop_geometry_index_invalid_before_graph";
          break;
        }
        const auto & geometry = computation.geometry_results[geometry_index];
        for (const auto & submap :
          {geometry.query_submap_id, geometry.candidate_submap_id})
        {
          component.insert(submap);
        }
        if (!capture_failure.empty()) {
          break;
        }
      }
      const auto server_edges = covisibility_database_.GetEdgesBySource(
        CovisibilityEdgeSource::ServerLoopGeometric);
      const auto dependency_list = pose_store_.GetLoopDependencies();
      bool expanded = true;
      while (capture_failure.empty() && expanded) {
        expanded = false;
        for (const auto & edge : server_edges) {
          const RawSubmapId first{edge.kf_a.drone_id, edge.kf_a.map_epoch};
          const RawSubmapId second{edge.kf_b.drone_id, edge.kf_b.map_epoch};
          if (component.count(first) != 0U && component.insert(second).second) {
            expanded = true;
          }
          if (component.count(second) != 0U && component.insert(first).second) {
            expanded = true;
          }
        }
        for (const auto & dependency : dependency_list) {
          if (component.count(dependency.child_submap_id) != 0U &&
            component.insert(dependency.parent_submap_id).second)
          {
            expanded = true;
          }
          if (component.count(dependency.parent_submap_id) != 0U &&
            component.insert(dependency.child_submap_id).second)
          {
            expanded = true;
          }
        }
      }
      for (const auto & submap : component) {
        if (!capture_submap(submap)) {
          capture_failure = "connected_submap_missing_before_graph";
          break;
        }
      }
      for (const auto & dependency : dependency_list) {
        if (component.count(dependency.child_submap_id) != 0U &&
          component.count(dependency.parent_submap_id) != 0U)
        {
          dependencies[dependency.child_submap_id] = dependency;
        }
      }
      if (capture_failure.empty()) {
        graph = pose_graph_builder_.BuildLoop(
          computation, snapshots, poses, pose_store_.GetStats().store_revision,
          covisibility_database_, loop_pipeline_config_, dependencies);
        if (graph.success) {
          for (auto & keyframe : graph.problem.keyframes) {
            const auto corridor = pose_store_.GetHardCorridorReference(keyframe.id);
            if (corridor.has_value()) {
              keyframe.hard_corridor = true;
              keyframe.hard_corridor_reference_pose = corridor->world_pose;
              keyframe.hard_corridor_alpha = corridor->alpha;
            }
          }
        }
      }
    }
    computation.optimization.graph_ms += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - graph_start).count();
    if (!capture_failure.empty()) {
      computation.decision = LoopTaskDecisionKind::Stale;
      computation.reason = capture_failure;
      computation.optimization.stale = true;
      computation.optimization.reason = capture_failure;
      return computation;
    }
    if (!graph.success) {
      computation.decision = LoopTaskDecisionKind::GeometryRejected;
      computation.reason = graph.reason;
      computation.optimization.reason = graph.reason;
      return computation;
    }
    computation.optimization.graph_built = true;
    computation.optimization.submaps = graph.problem.submap_windows.size();
    computation.optimization.window_keyframes = graph.problem.keyframes.size();
    computation.optimization.controls = graph.problem.control_indices.size();
    computation.optimization.temporal_edges = graph.problem.temporal_edges.size();
    computation.optimization.covisibility_edges = graph.problem.covisibility_edges.size();
    computation.optimization.loop_edges = graph.problem.loop_edges.size();

    const auto solve_start = std::chrono::steady_clock::now();
    proposal = optimization_manager_.Optimize(graph.problem);
    computation.optimization.solve_ms += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - solve_start).count();
    computation.optimization.optimized =
      proposal.status == OptimizationSolverStatus::Converged ||
      proposal.status == OptimizationSolverStatus::MaxIterations;
    computation.optimization.iterations += proposal.iterations;
    computation.optimization.initial_translation_error_m =
      proposal.initial_error.translation_m;
    computation.optimization.final_translation_error_m =
      proposal.final_error.translation_m;
    computation.optimization.initial_rotation_error_rad =
      proposal.initial_error.rotation_rad;
    computation.optimization.final_rotation_error_rad =
      proposal.final_error.rotation_rad;
    computation.optimization.initial_cost = proposal.initial_cost;
    computation.optimization.final_cost = proposal.final_cost;
    const auto validation_start = std::chrono::steady_clock::now();
    validation = optimization_validator_.Validate(graph.problem, proposal);
    computation.optimization.validation_ms += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - validation_start).count();
    computation.optimization.structural_edges_checked =
      validation.structural_edges_checked;
    computation.optimization.hard_corridor_keyframes_checked =
      validation.hard_corridor_keyframes_checked;
    computation.optimization.max_structural_translation_increase_m =
      validation.max_structural_translation_increase_m;
    computation.optimization.max_structural_rotation_increase_rad =
      validation.max_structural_rotation_increase_rad;
    computation.optimization.max_corridor_translation_excess_before_m =
      validation.max_corridor_translation_excess_before_m;
    computation.optimization.max_corridor_translation_excess_after_m =
      validation.max_corridor_translation_excess_after_m;
    computation.optimization.max_corridor_rotation_excess_before_rad =
      validation.max_corridor_rotation_excess_before_rad;
    computation.optimization.max_corridor_rotation_excess_after_rad =
      validation.max_corridor_rotation_excess_after_rad;
    if (validation.decision == ValidationDecision::AcceptFull) {
      break;
    }

    std::optional<size_t> discordant_geometry;
    size_t satisfied = 0U;
    if (pass == 0U && graph.problem.loop_edges.size() > 1U &&
      proposal.final_loop_errors.size() == graph.problem.loop_edges.size())
    {
      for (size_t index = 0; index < proposal.final_loop_errors.size(); ++index) {
        const auto & error = proposal.final_loop_errors[index];
        if (error.translation_m <= graph.problem.loop_translation_threshold_m &&
          error.rotation_rad <= graph.problem.loop_rotation_threshold_rad)
        {
          ++satisfied;
        } else if (!discordant_geometry.has_value()) {
          discordant_geometry = graph.problem.loop_edges[index].source_geometry_index;
        } else {
          discordant_geometry.reset();
          break;
        }
      }
    }
    if (discordant_geometry.has_value() && satisfied > 0U) {
      const auto found = std::find(
        computation.optimization_geometry_indices.begin(),
        computation.optimization_geometry_indices.end(), *discordant_geometry);
      if (found != computation.optimization_geometry_indices.end()) {
        computation.optimization_geometry_indices.erase(found);
        ++computation.optimization.rebuilds;
        ++computation.optimization.discarded_loop_regions;
        continue;
      }
    }
    computation.decision = LoopTaskDecisionKind::GeometryRejected;
    computation.reason = validation.reason;
    computation.optimization.reason = validation.reason;
    RememberRejectedLoopRegions(computation);
    return computation;
  }
  if (validation.decision != ValidationDecision::AcceptFull) {
    computation.decision = LoopTaskDecisionKind::GeometryRejected;
    computation.reason = validation.reason;
    computation.optimization.reason = validation.reason;
    RememberRejectedLoopRegions(computation);
    return computation;
  }
  computation.optimization.accepted = true;

  const auto commit_start = std::chrono::steady_clock::now();
  const auto commit = CommitGraphProposal(
    graph.problem, proposal, PoseSourceKind::LoopOptimized,
    graph.problem.loop_task.task_id);
  computation.optimization.commit_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - commit_start).count();
  if (commit.status != PoseCommitStatus::Applied) {
    computation.decision = commit.status == PoseCommitStatus::RevisionConflict ?
      LoopTaskDecisionKind::Stale : LoopTaskDecisionKind::Error;
    computation.reason = commit.detail.empty() ? ToString(commit.status) : commit.detail;
    computation.optimization.stale =
      commit.status == PoseCommitStatus::RevisionConflict;
    computation.optimization.reason = computation.reason;
    return computation;
  }
  computation.optimization.committed = true;
  computation.optimization.moved_keyframes = commit.dirty_keyframe_ids.size();
  computation.optimization.propagated_keyframes =
    commit.propagated_keyframe_ids.size();
  computation.optimization.rebased_skipped_controls =
    commit.rebased_skipped_controls;
  computation.optimization.rebased_inactive_controls =
    commit.rebased_inactive_controls;
  computation.rerun_keyframe_ids = commit.dirty_keyframe_ids;
  computation.rerun_keyframe_ids.insert(
    computation.rerun_keyframe_ids.end(), commit.propagated_keyframe_ids.begin(),
    commit.propagated_keyframe_ids.end());
  std::sort(computation.rerun_keyframe_ids.begin(), computation.rerun_keyframe_ids.end());
  computation.rerun_keyframe_ids.erase(
    std::unique(
      computation.rerun_keyframe_ids.begin(), computation.rerun_keyframe_ids.end()),
    computation.rerun_keyframe_ids.end());
  computation.optimization.reason = "atomic_covisible_loop_commit";
  computation.decision = LoopTaskDecisionKind::OptimizationCommitted;
  std::set<RawSubmapId> optimized_submaps;
  for (const auto & window : graph.problem.submap_windows) {
    optimized_submaps.insert(window.submap_id);
  }
  InvalidateRejectedLoopRegions(optimized_submaps);

  computation.fusion_pairs.clear();
  std::set<size_t> accepted_geometry_indices;
  for (const auto & edge : graph.problem.loop_edges) {
    accepted_geometry_indices.insert(edge.source_geometry_index);
  }
  for (size_t index = 0; index < computation.geometry_results.size(); ++index) {
    auto & geometry = computation.geometry_results[index];
    if (!geometry.accepted || accepted_geometry_indices.count(index) == 0U) {
      continue;
    }
    geometry.fusion_compatible = true;
    computation.fusion_pairs.insert(
      computation.fusion_pairs.end(), geometry.inlier_pairs.begin(),
      geometry.inlier_pairs.end());
  }
  std::sort(computation.fusion_pairs.begin(), computation.fusion_pairs.end());
  computation.fusion_pairs.erase(
    std::unique(computation.fusion_pairs.begin(), computation.fusion_pairs.end()),
    computation.fusion_pairs.end());
  if (computation.fusion_pairs.empty()) {
    computation.reason = "fusion_skipped_after_optimization_no_inliers";
    return computation;
  }
  auto fused = CommitLoopFusion(std::move(computation));
  fused.optimization.fusion_after_optimization = fused.fusion.committed;
  if (!fused.fusion.committed && fused.decision != LoopTaskDecisionKind::Stale) {
    fused.decision = LoopTaskDecisionKind::OptimizationCommitted;
    fused.reason = "fusion_skipped_after_optimization:" + fused.fusion.reason;
  } else if (fused.fusion.committed) {
    fused.decision = LoopTaskDecisionKind::OptimizationCommitted;
  }
  return fused;
}

CovisibilityDatabaseStats SparseGlobalBackend::GetCovisibilityStats() const
{
  return covisibility_database_.GetStats();
}

FusedLandmarkStats SparseGlobalBackend::GetFusedLandmarkStats() const
{
  return fused_landmark_manager_.GetStats();
}

void SparseGlobalBackend::ConfigureFiducialOptimization(
  const FiducialOptimizationConfig & config)
{
  std::lock_guard<std::mutex> state_lock(state_commit_mutex_);
  fiducial_optimization_config_ = config;
  fiducial_anchor_manager_.Configure(config);
  pose_graph_builder_.Configure(config);
  optimization_manager_.Configure(config);
  optimization_validator_.Configure(config);
}

void SparseGlobalBackend::ConfigureLoopPipeline(const LoopPipelineConfig & config)
{
  std::lock_guard<std::mutex> state_lock(state_commit_mutex_);
  loop_pipeline_config_ = config;
  loop_pipeline_.Configure(config);
}

void SparseGlobalBackend::ConfigureFusedLandmarks(const FusedLandmarkConfig & config)
{
  std::lock_guard<std::mutex> state_lock(state_commit_mutex_);
  fused_landmark_manager_.Configure(config);
}

void SparseGlobalBackend::ConfigureLandmarkScores(const LandmarkScoreConfig & config)
{
  std::lock_guard<std::mutex> state_lock(state_commit_mutex_);
  score_manager_.Configure(config);
}

FiducialTaskRevalidation SparseGlobalBackend::RevalidateFiducialTask(
  const FiducialOptimizationTask & task)
{
  std::lock_guard<std::mutex> state_lock(state_commit_mutex_);
  FiducialTaskRevalidation result;
  result.task = task;
  const auto pose = pose_store_.GetPose(task.keyframe_id);
  if (!pose.has_value() || !pose->active) {
    result.decision = FiducialTaskDecision::Stale;
    result.reason = "target_global_pose_missing";
    return result;
  }
  const auto control = fiducial_anchor_manager_.GetLastAcceptedControl(task.submap_id);
  if (!control.has_value()) {
    result.reason = "last_accepted_control_missing";
    return result;
  }
  const auto raw_snapshot = raw_database_.GetSubmapPoseSnapshot(task.submap_id);
  if (!raw_snapshot.has_value()) {
    result.reason = "raw_submap_missing";
    return result;
  }
  const auto control_position = std::find_if(
    raw_snapshot->keyframes.begin(), raw_snapshot->keyframes.end(),
    [&control](const RawKeyFramePoseInput & input) {return input.id == *control;});
  const auto target_position = std::find_if(
    raw_snapshot->keyframes.begin(), raw_snapshot->keyframes.end(),
    [&task](const RawKeyFramePoseInput & input) {return input.id == task.keyframe_id;});
  if (control_position == raw_snapshot->keyframes.end()) {
    result.reason = "last_accepted_control_raw_missing";
    return result;
  }
  if (target_position == raw_snapshot->keyframes.end()) {
    result.reason = "target_raw_missing";
    return result;
  }
  if (target_position <= control_position) {
    result.decision = FiducialTaskDecision::Stale;
    result.reason = "target_not_newer_than_current_control";
    return result;
  }
  result.error = ComputeFiducialError(pose->world_pose, task.target_world_T_kf);
  if (WithinThreshold(result.error, fiducial_optimization_config_)) {
    result.decision = FiducialTaskDecision::Stale;
    result.reason = "target_already_within_threshold";
    return result;
  }
  result.task.control_keyframe_id = *control;
  result.task.observation_pose_revision = pose->pose_revision;
  result.decision = FiducialTaskDecision::Ready;
  result.reason = "target_still_requires_optimization";
  return result;
}

PoseGraphBuildResult SparseGlobalBackend::BuildFiducialPoseGraph(
  const FiducialOptimizationTask & task)
{
  std::lock_guard<std::mutex> state_lock(state_commit_mutex_);
  const auto raw_snapshot = raw_database_.GetSubmapPoseSnapshot(task.submap_id);
  if (!raw_snapshot.has_value()) {
    PoseGraphBuildResult result;
    result.reason = "raw_submap_missing";
    return result;
  }

  const auto server_edges = covisibility_database_.GetEdgesBySource(
    CovisibilityEdgeSource::ServerLoopGeometric);
  const auto dependency_list = pose_store_.GetLoopDependencies();
  std::set<RawSubmapId> component{task.submap_id};
  bool expanded = true;
  while (expanded) {
    expanded = false;
    for (const auto & edge : server_edges) {
      const RawSubmapId first{edge.kf_a.drone_id, edge.kf_a.map_epoch};
      const RawSubmapId second{edge.kf_b.drone_id, edge.kf_b.map_epoch};
      if (first == second) {
        continue;
      }
      if (component.count(first) != 0U && component.insert(second).second) {
        expanded = true;
      }
      if (component.count(second) != 0U && component.insert(first).second) {
        expanded = true;
      }
    }
    for (const auto & dependency : dependency_list) {
      if (component.count(dependency.child_submap_id) != 0U &&
        component.insert(dependency.parent_submap_id).second)
      {
        expanded = true;
      }
      if (component.count(dependency.parent_submap_id) != 0U &&
        component.insert(dependency.child_submap_id).second)
      {
        expanded = true;
      }
    }
  }
  if (component.size() == 1U) {
    return pose_graph_builder_.Build(
      task, *raw_snapshot, pose_store_.GetSubmapPoses(task.submap_id),
      pose_store_.GetStats().store_revision, &covisibility_database_);
  }

  std::map<RawSubmapId, RawSubmapPoseSnapshot> snapshots;
  std::map<RawKeyFrameId, GlobalPoseRecord> poses;
  for (const auto & submap : component) {
    const auto snapshot = raw_database_.GetSubmapPoseSnapshot(submap);
    if (!snapshot.has_value()) {
      PoseGraphBuildResult result;
      result.reason = "connected_raw_submap_missing";
      return result;
    }
    snapshots[submap] = *snapshot;
    const auto submap_poses = pose_store_.GetSubmapPoses(submap);
    if (submap_poses.empty()) {
      PoseGraphBuildResult result;
      result.reason = "connected_global_submap_missing";
      return result;
    }
    poses.insert(submap_poses.begin(), submap_poses.end());
  }

  LoopTaskComputation synthetic;
  synthetic.task.task_id = task.task_id;
  synthetic.task.query_keyframe_id = task.keyframe_id;
  LoopGeometryResult source_window;
  source_window.accepted = true;
  source_window.query_keyframe_id = task.keyframe_id;
  source_window.candidate_keyframe_id = task.control_keyframe_id;
  source_window.query_submap_id = task.submap_id;
  source_window.candidate_submap_id = task.submap_id;
  source_window.matches = 1U;
  source_window.inliers = 1U;
  source_window.candidate_local_T_query_local = Eigen::Isometry3d::Identity();
  synthetic.geometry_results.push_back(source_window);
  synthetic.optimization_geometry_indices.push_back(0U);

  std::map<size_t, CovisibilityEdge> server_edge_by_geometry;
  for (const auto & edge : server_edges) {
    const RawSubmapId first{edge.kf_a.drone_id, edge.kf_a.map_epoch};
    const RawSubmapId second{edge.kf_b.drone_id, edge.kf_b.map_epoch};
    if (first == second || component.count(first) == 0U ||
      component.count(second) == 0U)
    {
      continue;
    }
    LoopGeometryResult geometry;
    geometry.accepted = true;
    geometry.query_keyframe_id = edge.kf_b;
    geometry.candidate_keyframe_id = edge.kf_a;
    geometry.query_submap_id = second;
    geometry.candidate_submap_id = first;
    geometry.matches = std::max<size_t>(1U, edge.support);
    geometry.inliers = geometry.matches;
    geometry.candidate_local_T_query_local = Eigen::Isometry3d::Identity();
    const size_t geometry_index = synthetic.geometry_results.size();
    synthetic.geometry_results.push_back(std::move(geometry));
    synthetic.optimization_geometry_indices.push_back(geometry_index);
    server_edge_by_geometry[geometry_index] = edge;
  }
  std::map<RawSubmapId, LoopAnchorDependencySnapshot> dependencies;
  for (const auto & dependency : dependency_list) {
    if (component.count(dependency.child_submap_id) != 0U &&
      component.count(dependency.parent_submap_id) != 0U)
    {
      dependencies[dependency.child_submap_id] = dependency;
    }
  }

  auto result = pose_graph_builder_.BuildLoop(
    synthetic, snapshots, poses, pose_store_.GetStats().store_revision,
    covisibility_database_, loop_pipeline_config_, dependencies);
  if (!result.success) {
    return result;
  }
  std::set<std::pair<size_t, size_t>> confirmed_pairs;
  std::vector<PoseGraphEdge> confirmed_edges;
  for (const auto & loop_edge : result.problem.loop_edges) {
    const auto source = server_edge_by_geometry.find(loop_edge.source_geometry_index);
    if (source == server_edge_by_geometry.end()) {
      continue;
    }
    PoseGraphEdge edge = loop_edge;
    edge.relative_raw_pose = source->second.relative_pose_measured;
    edge.supporting_keyframes = source->second.support;
    edge.information_weight = std::max(1.0, source->second.information_weight);
    edge.kind = PoseGraphEdgeKind::PriorLoop;
    confirmed_pairs.insert(std::minmax(edge.from_index, edge.to_index));
    confirmed_edges.push_back(std::move(edge));
  }
  result.problem.covisibility_edges.erase(
    std::remove_if(
      result.problem.covisibility_edges.begin(),
      result.problem.covisibility_edges.end(),
      [&confirmed_pairs](const PoseGraphEdge & edge) {
        return edge.kind == PoseGraphEdgeKind::PriorLoop &&
               confirmed_pairs.count(std::minmax(edge.from_index, edge.to_index)) != 0U;
      }),
    result.problem.covisibility_edges.end());
  result.problem.covisibility_edges.insert(
    result.problem.covisibility_edges.end(),
    confirmed_edges.begin(), confirmed_edges.end());
  result.problem.loop_edges.clear();
  result.problem.kind = PoseGraphProblemKind::FiducialAbsolute;
  result.problem.task = task;
  result.problem.raw_submap_revision = raw_snapshot->submap_revision;
  result.reason = "confirmed_multi_submap_fiducial_graph";
  return result;
}

OptimizationProposal SparseGlobalBackend::OptimizeFiducialPoseGraph(
  const PoseGraphProblem & problem) const
{
  return optimization_manager_.Optimize(problem);
}

ValidationResult SparseGlobalBackend::ValidateFiducialProposal(
  const PoseGraphProblem & problem,
  const OptimizationProposal & proposal) const
{
  return optimization_validator_.Validate(problem, proposal);
}

FiducialCommitResult SparseGlobalBackend::CommitFiducialProposal(
  const PoseGraphProblem & problem,
  const OptimizationProposal & proposal,
  const ValidationResult & validation)
{
  FiducialCommitResult result;
  result.final_error = validation.final_error;
  if (validation.decision == ValidationDecision::HardFailure) {
    result.reason = validation.reason;
    return result;
  }

  if (problem.submap_windows.size() > 1U) {
    const auto commit = CommitGraphProposal(
      problem, proposal, PoseSourceKind::FiducialOptimized,
      problem.task.task_id,
      validation.decision == ValidationDecision::AcceptFull ?
      std::optional<RawKeyFrameId>(problem.task.keyframe_id) : std::nullopt);
    if (commit.status != PoseCommitStatus::Applied) {
      result.reason = ToString(commit.status);
      return result;
    }
    for (const auto & changes : commit.submap_changes) {
      if (changes.submap_id == problem.task.submap_id) {
        result.pose_changes = changes;
        break;
      }
    }
    result.committed = true;
    result.rerun_keyframe_ids = commit.dirty_keyframe_ids;
    result.rerun_keyframe_ids.insert(
      result.rerun_keyframe_ids.end(), commit.propagated_keyframe_ids.begin(),
      commit.propagated_keyframe_ids.end());
    std::sort(result.rerun_keyframe_ids.begin(), result.rerun_keyframe_ids.end());
    result.rerun_keyframe_ids.erase(
      std::unique(result.rerun_keyframe_ids.begin(), result.rerun_keyframe_ids.end()),
      result.rerun_keyframe_ids.end());
    result.full_accept = validation.decision == ValidationDecision::AcceptFull;
    result.window_keyframes = problem.keyframes.size();
    result.reason = result.full_accept ?
      "atomic_multi_submap_full_commit" : "atomic_multi_submap_partial_commit";
    if (result.full_accept) {
      fiducial_anchor_manager_.AcceptControl(
        problem.task.submap_id, problem.task.fiducial_visit_id,
        problem.task.keyframe_id);
      recent_loss_continuity_.erase(problem.task.submap_id);
    }
    std::set<RawSubmapId> fiducial_submaps;
    for (const auto & changes : commit.submap_changes) {
      fiducial_submaps.insert(changes.submap_id);
    }
    InvalidateRejectedLoopRegions(fiducial_submaps);
    return result;
  }

  std::lock_guard<std::mutex> state_lock(state_commit_mutex_);
  const auto raw_snapshot = raw_database_.GetSubmapPoseSnapshot(problem.task.submap_id);
  if (!raw_snapshot.has_value()) {
    result.reason = "raw_submap_missing_at_commit";
    return result;
  }
  const auto poses = pose_store_.GetSubmapPoses(problem.task.submap_id);
  const auto control_index = FindKeyFrameIndex(*raw_snapshot, problem.task.control_keyframe_id);
  const auto target_index = FindKeyFrameIndex(*raw_snapshot, problem.task.keyframe_id);
  if (!control_index.has_value() || !target_index.has_value() ||
    *control_index >= *target_index)
  {
    result.reason = "commit_window_invalid";
    return result;
  }

  std::map<RawKeyFrameId, const PoseGraphKeyFrame *> graph_keyframes;
  for (const auto & keyframe : problem.keyframes) {
    graph_keyframes[keyframe.id] = &keyframe;
    const auto raw_index = FindKeyFrameIndex(*raw_snapshot, keyframe.id);
    const auto current_pose = poses.find(keyframe.id);
    if (!raw_index.has_value() || current_pose == poses.end() ||
      raw_snapshot->keyframes[*raw_index].raw_revision != keyframe.raw_revision ||
      current_pose->second.pose_revision != keyframe.pose_revision)
    {
      result.reason = "scoped_revision_conflict";
      return result;
    }
  }
  const auto start_pose = poses.find(problem.task.control_keyframe_id);
  if (start_pose == poses.end() ||
    (!start_pose->second.hard_fiducial &&
    !problem.task.replaces_soft_loop_anchor) ||
    !PosesNear(
      start_pose->second.world_pose, problem.keyframes.front().current_world_pose,
      1e-8, 1e-8))
  {
    result.reason = "hard_control_changed";
    return result;
  }

  std::map<RawKeyFrameId, geometry_msgs::msg::Pose> proposed_controls;
  for (const auto & control : proposal.controls) {
    proposed_controls[control.id] = control.world_pose;
  }
  if (proposed_controls.find(problem.task.control_keyframe_id) == proposed_controls.end() ||
    proposed_controls.find(problem.task.keyframe_id) == proposed_controls.end())
  {
    result.reason = "proposal_endpoint_missing";
    return result;
  }

  const size_t window_count = *target_index - *control_index + 1U;
  std::vector<double> path(window_count, 0.0);
  for (size_t offset = 1U; offset < window_count; ++offset) {
    Eigen::Isometry3d previous;
    Eigen::Isometry3d current;
    if (!PoseToIsometry(
        raw_snapshot->keyframes[*control_index + offset - 1U].local_pose, &previous) ||
      !PoseToIsometry(
        raw_snapshot->keyframes[*control_index + offset].local_pose, &current))
    {
      result.reason = "raw_window_pose_invalid";
      return result;
    }
    path[offset] = path[offset - 1U] +
      (current.translation() - previous.translation()).norm();
  }
  const double total_path = path.back();

  struct ControlCorrection
  {
    size_t raw_index = 0;
    double path_value = 0.0;
    Eigen::Isometry3d correction = Eigen::Isometry3d::Identity();
  };
  std::vector<ControlCorrection> corrections;
  corrections.reserve(problem.control_indices.size());
  for (const size_t graph_control_index : problem.control_indices) {
    const auto & graph_control = problem.keyframes[graph_control_index];
    const auto current_raw_index = FindKeyFrameIndex(*raw_snapshot, graph_control.id);
    const auto proposed = proposed_controls.find(graph_control.id);
    const auto current_pose = poses.find(graph_control.id);
    if (!current_raw_index.has_value() || proposed == proposed_controls.end() ||
      current_pose == poses.end())
    {
      result.reason = "control_missing_at_commit";
      return result;
    }
    Eigen::Isometry3d proposed_transform;
    Eigen::Isometry3d current_transform;
    if (!PoseToIsometry(proposed->second, &proposed_transform) ||
      !PoseToIsometry(current_pose->second.world_pose, &current_transform))
    {
      result.reason = "control_transform_invalid";
      return result;
    }
    corrections.push_back(
      {*current_raw_index,
        path[*current_raw_index - *control_index],
        proposed_transform * current_transform.inverse()});
  }

  std::vector<AcceptedPoseUpdate> updates;
  updates.reserve(raw_snapshot->keyframes.size() - *control_index);
  size_t late_window = 0;
  const size_t first_update_index = problem.task.replaces_soft_loop_anchor ?
    *control_index : *control_index + 1U;
  for (size_t raw_index = first_update_index; raw_index <= *target_index; ++raw_index) {
    const auto & raw = raw_snapshot->keyframes[raw_index];
    const auto current_pose = poses.find(raw.id);
    if (current_pose == poses.end() || !current_pose->second.active) {
      continue;
    }
    auto upper = std::lower_bound(
      corrections.begin(), corrections.end(), raw_index,
      [](const ControlCorrection & correction, size_t index) {
        return correction.raw_index < index;
      });
    Eigen::Isometry3d correction = Eigen::Isometry3d::Identity();
    if (upper != corrections.end() && upper->raw_index == raw_index) {
      correction = upper->correction;
    } else {
      if (upper == corrections.begin() || upper == corrections.end()) {
        result.reason = "window_control_bracket_missing";
        return result;
      }
      const auto & lower = *(upper - 1);
      const double current_path = path[raw_index - *control_index];
      const double denominator = upper->path_value - lower.path_value;
      const double alpha = denominator > 1e-9 ?
        (current_path - lower.path_value) / denominator :
        static_cast<double>(raw_index - lower.raw_index) /
        static_cast<double>(upper->raw_index - lower.raw_index);
      correction = InterpolateIsometry(lower.correction, upper->correction, alpha);
    }
    Eigen::Isometry3d current_transform;
    if (!PoseToIsometry(current_pose->second.world_pose, &current_transform)) {
      result.reason = "window_world_pose_invalid";
      return result;
    }
    const bool is_target = raw.id == problem.task.keyframe_id;
    updates.push_back(
      {raw.id, IsometryToPose(correction * current_transform), raw.raw_revision,
        current_pose->second.pose_revision,
        is_target && validation.decision == ValidationDecision::AcceptFull});
    if (graph_keyframes.find(raw.id) == graph_keyframes.end()) {
      ++late_window;
    }
  }

  Eigen::Isometry3d accepted_target;
  Eigen::Isometry3d raw_target;
  if (updates.empty() || !(updates.back().keyframe_id == problem.task.keyframe_id)) {
    result.reason = "target_update_missing";
    return result;
  }
  if (!PoseToIsometry(updates.back().world_pose, &accepted_target) ||
    !PoseToIsometry(raw_snapshot->keyframes[*target_index].local_pose, &raw_target))
  {
    result.reason = "target_rebase_pose_invalid";
    return result;
  }
  size_t tail_count = 0;
  for (size_t raw_index = *target_index + 1U;
    raw_index < raw_snapshot->keyframes.size(); ++raw_index)
  {
    const auto & raw = raw_snapshot->keyframes[raw_index];
    const auto current_pose = poses.find(raw.id);
    if (current_pose == poses.end() || !current_pose->second.active) {
      continue;
    }
    Eigen::Isometry3d raw_pose;
    if (!PoseToIsometry(raw.local_pose, &raw_pose)) {
      result.reason = "tail_raw_pose_invalid";
      return result;
    }
    updates.push_back(
      {raw.id, IsometryToPose(accepted_target * raw_target.inverse() * raw_pose),
        raw.raw_revision, current_pose->second.pose_revision, false});
    ++tail_count;
  }

  result.pose_changes = pose_store_.CommitAcceptedPoses(
    problem.task.submap_id, updates, PoseSourceKind::FiducialOptimized,
    problem.task.task_id,
    validation.decision == ValidationDecision::AcceptFull ?
    std::optional<RawKeyFrameId>(problem.task.keyframe_id) : std::nullopt);
  if (result.pose_changes.status != PoseCommitStatus::Applied) {
    result.reason = ToString(result.pose_changes.status);
    return result;
  }
  RefreshScoresAfterPoseChanges({result.pose_changes});
  result.committed = true;
  result.rerun_keyframe_ids = result.pose_changes.updated_ids;
  result.rerun_keyframe_ids.insert(
    result.rerun_keyframe_ids.end(), result.pose_changes.control_propagated_ids.begin(),
    result.pose_changes.control_propagated_ids.end());
  std::sort(result.rerun_keyframe_ids.begin(), result.rerun_keyframe_ids.end());
  result.rerun_keyframe_ids.erase(
    std::unique(result.rerun_keyframe_ids.begin(), result.rerun_keyframe_ids.end()),
    result.rerun_keyframe_ids.end());
  result.full_accept = validation.decision == ValidationDecision::AcceptFull;
  result.window_keyframes = window_count;
  result.late_window_keyframes = late_window;
  result.tail_keyframes = tail_count;
  result.reason = result.full_accept ? "atomic_full_commit" : "atomic_partial_commit";
  if (result.full_accept) {
    fiducial_anchor_manager_.AcceptControl(
      problem.task.submap_id, problem.task.fiducial_visit_id,
      problem.task.keyframe_id);
    recent_loss_continuity_.erase(problem.task.submap_id);
  }
  InvalidateRejectedLoopRegions({problem.task.submap_id});
  return result;
}

RawDatabaseStats SparseGlobalBackend::GetRawStats() const
{
  return raw_database_.GetStats();
}

GlobalPoseStoreStats SparseGlobalBackend::GetPoseStats() const
{
  return pose_store_.GetStats();
}

LandmarkScoreStats SparseGlobalBackend::GetScoreStats() const
{
  return score_manager_.GetStats();
}

std::optional<GlobalPoseRecord> SparseGlobalBackend::GetGlobalPose(
  const RawKeyFrameId & id) const
{
  return pose_store_.GetPose(id);
}

GlobalMapBuildResult SparseGlobalBackend::BuildGlobalMap()
{
  std::lock_guard<std::mutex> state_lock(state_commit_mutex_);
  std::lock_guard<std::mutex> builder_lock(builder_mutex_);
  auto result = global_map_builder_.Update(
    raw_database_, pose_store_, score_manager_, fused_landmark_manager_);
  deferred_snapshot_dirty_ = false;
  return result;
}

bool SparseGlobalBackend::StartRawRecord(
  const std::string & path,
  std::string * error_message)
{
  return raw_database_.StartIncrementalRecord(path, error_message);
}

bool SparseGlobalBackend::FinalizeRawRecord(std::string * error_message)
{
  return raw_database_.FinalizeIncrementalRecord(error_message);
}

void SparseGlobalBackend::DisableRawJournalRetention()
{
  raw_database_.DisableJournalRetention();
}

RawJournalStorageStats SparseGlobalBackend::GetRawJournalStorageStats() const
{
  return raw_database_.GetJournalStorageStats();
}

bool SparseGlobalBackend::SaveRawRecord(
  const std::string & path,
  std::string * error_message) const
{
  return raw_database_.SaveToPath(path, error_message);
}

}  // namespace orbslam3_multi
