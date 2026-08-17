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
  destination->updated_ids.insert(
    destination->updated_ids.end(), source.updated_ids.begin(), source.updated_ids.end());
  destination->input_updated_ids.insert(
    destination->input_updated_ids.end(), source.input_updated_ids.begin(),
    source.input_updated_ids.end());
  destination->fused_created_ids.insert(
    destination->fused_created_ids.end(), source.fused_created_ids.begin(),
    source.fused_created_ids.end());
  destination->fused_updated_ids.insert(
    destination->fused_updated_ids.end(), source.fused_updated_ids.begin(),
    source.fused_updated_ids.end());
  destination->fused_removed_ids.insert(
    destination->fused_removed_ids.end(), source.fused_removed_ids.begin(),
    source.fused_removed_ids.end());
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

PrimaryBackendResult SparseGlobalBackend::InsertDelta(
  uint64_t arrival_id,
  std::shared_ptr<const orbslam3_msgs::msg::OrbMap> delta)
{
  std::lock_guard<std::mutex> state_lock(state_commit_mutex_);
  PrimaryBackendResult result;
  result.had_deferred_snapshot_dirty = deferred_snapshot_dirty_;
  result.raw_result = raw_database_.InsertDelta(arrival_id, std::move(delta));
  std::lock_guard<std::mutex> builder_lock(builder_mutex_);
  global_map_builder_.MarkRawChanges(result.raw_result);
  result.score_changes = score_manager_.ApplyRawChanges(result.raw_result, raw_database_);
  RefreshFusedScores(
    result.score_changes, &fused_landmark_manager_, &score_manager_,
    &result.score_changes);
  global_map_builder_.MarkScoreChanges(result.score_changes);
  result.pose_stage_executed = !result.raw_result.pose_changes.empty();
  if (result.pose_stage_executed) {
    result.pose_changes = pose_store_.ApplyRawPoseChanges(
      result.raw_result.submap_id, result.raw_result.pose_changes, arrival_id);
    global_map_builder_.MarkPoseChanges(result.pose_changes);
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
  if (!result.raw_result.has_material_changes) {
    return result;
  }

  std::lock_guard<std::mutex> builder_lock(builder_mutex_);
  global_map_builder_.MarkRawChanges(result.raw_result);
  result.score_changes = score_manager_.ApplyRawChanges(result.raw_result, raw_database_);
  RefreshFusedScores(
    result.score_changes, &fused_landmark_manager_, &score_manager_,
    &result.score_changes);
  global_map_builder_.MarkScoreChanges(result.score_changes);
  result.pose_stage_executed = !result.raw_result.pose_changes.empty();
  if (result.pose_stage_executed) {
    result.pose_changes = pose_store_.ApplyRawPoseChanges(
      result.raw_result.submap_id, result.raw_result.pose_changes, arrival_id);
    global_map_builder_.MarkPoseChanges(result.pose_changes);
  }
  deferred_snapshot_dirty_ = true;
  return result;
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
  std::lock_guard<std::mutex> builder_lock(builder_mutex_);
  global_map_builder_.MarkPoseChanges(result);
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
  {
    std::lock_guard<std::mutex> builder_lock(builder_mutex_);
    global_map_builder_.MarkPoseChanges(result.pose_changes);
  }
  result.hard_keyframe = result.pose_changes.hard_fiducial_ids.size() == 1 &&
    result.pose_changes.hard_fiducial_ids.front() == observation.keyframe_id;
  if (result.pose_changes.status != PoseCommitStatus::Applied) {
    result.status = FiducialProcessStatus::AnchorCommitRejected;
    result.reason = ToString(result.pose_changes.status);
  } else {
    fiducial_anchor_manager_.AcceptControl(
      submap_id, observation.fiducial_visit_id, observation.keyframe_id);
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
  return computation;
}

LoopTaskComputation SparseGlobalBackend::ProcessLoopTask(const LoopTask & task)
{
  auto computation = loop_pipeline_.Process(
    task, raw_database_, pose_store_, covisibility_database_);
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
  }
  computation.anchor_commit = pose_store_.CommitLoopAnchorBatch(
    computation.anchor_entries, task.task_id);
  if (computation.anchor_commit.status != PoseCommitStatus::Applied) {
    computation.decision = LoopTaskDecisionKind::Stale;
    computation.reason = ToString(computation.anchor_commit.status);
    return computation;
  }
  {
    std::lock_guard<std::mutex> builder_lock(builder_mutex_);
    for (const auto & changes : computation.anchor_commit.submap_changes) {
      global_map_builder_.MarkPoseChanges(changes);
    }
  }
  for (const auto & entry : computation.anchor_entries) {
    const auto ids = raw_database_.GetActiveSubmapEntityIds(entry.snapshot.submap_id);
    if (ids.has_value()) {
      computation.rerun_keyframe_ids.insert(
        computation.rerun_keyframe_ids.end(), ids->keyframe_ids.begin(),
        ids->keyframe_ids.end());
    }
  }
  computation.reason = "loop_anchor_batch_committed";
  return computation;
}

AcceptedPoseBatchResult SparseGlobalBackend::CommitLoopProposal(
  const PoseGraphProblem & problem,
  const OptimizationProposal & proposal)
{
  AcceptedPoseBatchResult failed;
  failed.source_task_id = problem.loop_task.task_id;
  std::lock_guard<std::mutex> state_lock(state_commit_mutex_);
  std::map<RawKeyFrameId, geometry_msgs::msg::Pose> proposed_controls;
  for (const auto & control : proposal.controls) {
    proposed_controls[control.id] = control.world_pose;
  }
  std::map<RawKeyFrameId, const PoseGraphKeyFrame *> graph_by_id;
  for (const auto & keyframe : problem.keyframes) {
    graph_by_id[keyframe.id] = &keyframe;
    const auto raw_revision = raw_database_.GetKeyFrameRevision(keyframe.id);
    const auto pose = pose_store_.GetPose(keyframe.id);
    if (!raw_revision.has_value() || !pose.has_value() || !pose->active ||
      *raw_revision != keyframe.raw_revision ||
      pose->pose_revision != keyframe.pose_revision)
    {
      failed.status = PoseCommitStatus::RevisionConflict;
      return failed;
    }
  }

  struct ControlCorrection
  {
    uint64_t local_kf_id = 0;
    Eigen::Isometry3d correction = Eigen::Isometry3d::Identity();
  };
  std::vector<AcceptedSubmapPoseBatch> batches;
  for (const auto & window : problem.submap_windows) {
    const auto snapshot = raw_database_.GetSubmapPoseSnapshot(window.submap_id);
    if (!snapshot.has_value() ||
      snapshot->submap_revision != window.raw_submap_revision)
    {
      failed.status = PoseCommitStatus::RevisionConflict;
      return failed;
    }
    const auto current_poses = pose_store_.GetSubmapPoses(window.submap_id);
    std::vector<ControlCorrection> corrections;
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
        return failed;
      }
      Eigen::Isometry3d desired;
      Eigen::Isometry3d current;
      if (!PoseToIsometry(proposed->second, &desired) ||
        !PoseToIsometry(graph_keyframe.current_world_pose, &current))
      {
        failed.status = PoseCommitStatus::AtomicBatchConflict;
        return failed;
      }
      corrections.push_back(
        {graph_keyframe.id.local_kf_id, desired * current.inverse()});
    }
    std::sort(
      corrections.begin(), corrections.end(),
      [](const auto & lhs, const auto & rhs) {
        return lhs.local_kf_id < rhs.local_kf_id;
      });
    if (corrections.size() < 2U) {
      failed.status = PoseCommitStatus::AtomicBatchConflict;
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
        return failed;
      }
      batch.updates.push_back(
        {raw.id, IsometryToPose(correction * current), raw.raw_revision,
          current_pose->second.pose_revision, false});
    }
    batches.push_back(std::move(batch));
  }

  auto result = pose_store_.CommitAcceptedPoseBatch(
    batches, PoseSourceKind::LoopOptimized, problem.loop_task.task_id);
  if (result.status == PoseCommitStatus::Applied) {
    std::lock_guard<std::mutex> builder_lock(builder_mutex_);
    for (const auto & changes : result.submap_changes) {
      global_map_builder_.MarkPoseChanges(changes);
    }
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
    std::set<RawSubmapId> dependency_frontier;
    for (const auto & geometry : computation.geometry_results) {
      if (!geometry.accepted || geometry.fusion_compatible) {
        continue;
      }
      for (const auto & submap :
        {geometry.query_submap_id, geometry.candidate_submap_id})
      {
        dependency_frontier.insert(submap);
        if (!capture_submap(submap)) {
          computation.decision = LoopTaskDecisionKind::Stale;
          computation.reason = "loop_submap_missing_before_graph";
          computation.optimization.stale = true;
          computation.optimization.reason = computation.reason;
          return computation;
        }
      }
    }
    std::set<RawSubmapId> visited_dependencies;
    while (!dependency_frontier.empty()) {
      const RawSubmapId child = *dependency_frontier.begin();
      dependency_frontier.erase(dependency_frontier.begin());
      if (!visited_dependencies.insert(child).second) {
        continue;
      }
      const auto dependency = pose_store_.GetLoopDependency(child);
      if (!dependency.has_value()) {
        continue;
      }
      dependencies[child] = *dependency;
      if (!capture_submap(dependency->parent_submap_id)) {
        computation.decision = LoopTaskDecisionKind::Stale;
        computation.reason = "loop_dependency_parent_missing_before_graph";
        computation.optimization.stale = true;
        computation.optimization.reason = computation.reason;
        return computation;
      }
      dependency_frontier.insert(dependency->parent_submap_id);
    }
    graph = pose_graph_builder_.BuildLoop(
      computation, snapshots, poses, pose_store_.GetStats().store_revision,
      covisibility_database_, loop_pipeline_config_, dependencies);
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

  const auto proposal = optimization_manager_.Optimize(graph.problem);
  computation.optimization.optimized =
    proposal.status == OptimizationSolverStatus::Converged ||
    proposal.status == OptimizationSolverStatus::MaxIterations;
  computation.optimization.iterations = proposal.iterations;
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
  const auto validation = optimization_validator_.Validate(graph.problem, proposal);
  if (validation.decision != ValidationDecision::AcceptFull) {
    computation.decision = LoopTaskDecisionKind::GeometryRejected;
    computation.reason = validation.reason;
    computation.optimization.reason = validation.reason;
    return computation;
  }
  computation.optimization.accepted = true;

  const auto commit = CommitLoopProposal(graph.problem, proposal);
  if (commit.status != PoseCommitStatus::Applied) {
    computation.decision = commit.status == PoseCommitStatus::RevisionConflict ?
      LoopTaskDecisionKind::Stale : LoopTaskDecisionKind::Error;
    computation.reason = ToString(commit.status);
    computation.optimization.stale =
      commit.status == PoseCommitStatus::RevisionConflict;
    computation.optimization.reason = computation.reason;
    return computation;
  }
  computation.optimization.committed = true;
  computation.optimization.moved_keyframes = commit.dirty_keyframe_ids.size();
  computation.optimization.propagated_keyframes =
    commit.propagated_keyframe_ids.size();
  computation.optimization.reason = "atomic_covisible_loop_commit";
  computation.decision = LoopTaskDecisionKind::OptimizationCommitted;

  computation.fusion_pairs.clear();
  const auto & accepted_loop_edge = graph.problem.loop_edges.front();
  const auto & accepted_candidate =
    graph.problem.keyframes[accepted_loop_edge.from_index].id;
  const auto & accepted_query =
    graph.problem.keyframes[accepted_loop_edge.to_index].id;
  for (auto & geometry : computation.geometry_results) {
    if (!geometry.accepted ||
      !(geometry.query_keyframe_id == accepted_query) ||
      !(geometry.candidate_keyframe_id == accepted_candidate))
    {
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

FiducialTaskRevalidation SparseGlobalBackend::RevalidateFiducialTask(
  const FiducialOptimizationTask & task)
{
  std::lock_guard<std::mutex> state_lock(state_commit_mutex_);
  FiducialTaskRevalidation result;
  result.task = task;
  const auto pose = pose_store_.GetPose(task.keyframe_id);
  if (!pose.has_value() || !pose->active) {
    result.reason = "target_global_pose_missing";
    return result;
  }
  const auto control = fiducial_anchor_manager_.GetLastAcceptedControl(task.submap_id);
  if (!control.has_value()) {
    result.reason = "last_accepted_control_missing";
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
  return pose_graph_builder_.Build(
    task, *raw_snapshot, pose_store_.GetSubmapPoses(task.submap_id),
    pose_store_.GetStats().store_revision, &covisibility_database_);
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
  {
    std::lock_guard<std::mutex> builder_lock(builder_mutex_);
    global_map_builder_.MarkPoseChanges(result.pose_changes);
  }
  result.committed = true;
  result.full_accept = validation.decision == ValidationDecision::AcceptFull;
  result.window_keyframes = window_count;
  result.late_window_keyframes = late_window;
  result.tail_keyframes = tail_count;
  result.reason = result.full_accept ? "atomic_full_commit" : "atomic_partial_commit";
  if (result.full_accept) {
    fiducial_anchor_manager_.AcceptControl(
      problem.task.submap_id, problem.task.fiducial_visit_id,
      problem.task.keyframe_id);
  }
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
