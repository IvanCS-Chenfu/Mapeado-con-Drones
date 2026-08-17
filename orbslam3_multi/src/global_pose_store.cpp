#include "orbslam3_multi/global_pose_store.hpp"

#include "orbslam3_multi/pose_geometry.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <set>
#include <utility>

namespace orbslam3_multi
{
namespace
{

bool PoseToMatrix(const geometry_msgs::msg::Pose & pose, Eigen::Matrix4d * matrix)
{
  const Eigen::Quaterniond raw(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
    !std::isfinite(pose.position.z) || !std::isfinite(raw.w()) ||
    !std::isfinite(raw.x()) || !std::isfinite(raw.y()) ||
    !std::isfinite(raw.z()) || raw.norm() < 1e-9)
  {
    return false;
  }

  const Eigen::Quaterniond normalized = raw.normalized();
  *matrix = Eigen::Matrix4d::Identity();
  matrix->block<3, 3>(0, 0) = normalized.toRotationMatrix();
  (*matrix)(0, 3) = pose.position.x;
  (*matrix)(1, 3) = pose.position.y;
  (*matrix)(2, 3) = pose.position.z;
  return matrix->allFinite();
}

geometry_msgs::msg::Pose MatrixToPose(const Eigen::Matrix4d & matrix)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = matrix(0, 3);
  pose.position.y = matrix(1, 3);
  pose.position.z = matrix(2, 3);
  const Eigen::Quaterniond quaternion(matrix.block<3, 3>(0, 0));
  const Eigen::Quaterniond normalized = quaternion.normalized();
  pose.orientation.x = normalized.x();
  pose.orientation.y = normalized.y();
  pose.orientation.z = normalized.z();
  pose.orientation.w = normalized.w();
  return pose;
}

geometry_msgs::msg::Pose Compose(
  const geometry_msgs::msg::Pose & world_T_local,
  const geometry_msgs::msg::Pose & local_T_kf)
{
  Eigen::Matrix4d world;
  Eigen::Matrix4d local;
  if (!PoseToMatrix(world_T_local, &world) || !PoseToMatrix(local_T_kf, &local)) {
    throw std::invalid_argument("pose no finita o quaternion no valido");
  }
  return MatrixToPose(world * local);
}

geometry_msgs::msg::Pose CorrectionFromRawWorld(
  const geometry_msgs::msg::Pose & world_pose,
  const geometry_msgs::msg::Pose & raw_world_pose)
{
  Eigen::Matrix4d world;
  Eigen::Matrix4d raw_world;
  if (!PoseToMatrix(world_pose, &world) || !PoseToMatrix(raw_world_pose, &raw_world)) {
    throw std::invalid_argument("pose world/raw no valida para correccion");
  }
  return MatrixToPose(world * raw_world.inverse());
}

bool IsRawMutable(PoseSourceKind source)
{
  return source == PoseSourceKind::SubmapAnchorDerived ||
         source == PoseSourceKind::FiducialControlDerived;
}

RawSubmapId SubmapOf(const RawKeyFrameId & id)
{
  return {id.drone_id, id.map_epoch};
}

}  // namespace

const char * ToString(PoseSourceKind kind)
{
  switch (kind) {
    case PoseSourceKind::SubmapAnchorDerived:
      return "submap_anchor_derived";
    case PoseSourceKind::FiducialControlDerived:
      return "fiducial_control_derived";
    case PoseSourceKind::FiducialAccepted:
      return "fiducial_accepted";
    case PoseSourceKind::FiducialOptimized:
      return "fiducial_optimized";
    case PoseSourceKind::LoopAnchorDerived:
      return "loop_anchor_derived";
    case PoseSourceKind::LoopOptimized:
      return "loop_optimized";
  }
  return "unknown";
}

const char * ToString(PoseCommitStatus status)
{
  switch (status) {
    case PoseCommitStatus::NoChanges:
      return "no_changes";
    case PoseCommitStatus::Unanchored:
      return "unanchored";
    case PoseCommitStatus::Applied:
      return "applied";
    case PoseCommitStatus::PreservedAccepted:
      return "preserved_accepted";
    case PoseCommitStatus::AlreadyAnchored:
      return "already_anchored";
    case PoseCommitStatus::MissingRawSubmap:
      return "missing_raw_submap";
    case PoseCommitStatus::RevisionConflict:
      return "revision_conflict";
    case PoseCommitStatus::HardConstraintViolation:
      return "hard_constraint_violation";
    case PoseCommitStatus::AtomicBatchConflict:
      return "atomic_batch_conflict";
  }
  return "unknown";
}

bool GlobalPoseStore::HasSubmapAnchor(const RawSubmapId & submap_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return anchors_.find(submap_id) != anchors_.end();
}

std::optional<GlobalPoseRecord> GlobalPoseStore::GetPose(
  const RawKeyFrameId & keyframe_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto pose = poses_.find(keyframe_id);
  return pose == poses_.end() ?
         std::nullopt : std::optional<GlobalPoseRecord>(pose->second);
}

std::map<RawKeyFrameId, GlobalPoseRecord> GlobalPoseStore::GetSubmapPoses(
  const RawSubmapId & submap_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::map<RawKeyFrameId, GlobalPoseRecord> result;
  for (const auto & [id, pose] : poses_) {
    if (id.drone_id == submap_id.drone_id && id.map_epoch == submap_id.map_epoch) {
      result.emplace(id, pose);
    }
  }
  return result;
}

GlobalPoseStoreStats GlobalPoseStore::GetStats() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  GlobalPoseStoreStats stats;
  stats.store_revision = store_revision_;
  stats.commits = commit_count_;
  stats.anchors = anchors_.size();
  stats.poses = poses_.size();
  for (const auto & [id, pose] : poses_) {
    (void)id;
    pose.active ? ++stats.active_poses : ++stats.inactive_poses;
    if (pose.hard_fiducial) {
      ++stats.hard_fiducial_keyframes;
    }
  }
  return stats;
}

std::optional<geometry_msgs::msg::Pose> GlobalPoseStore::GetSubmapAnchorPose(
  const RawSubmapId & submap_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = anchors_.find(submap_id);
  return found == anchors_.end() ?
         std::nullopt : std::optional<geometry_msgs::msg::Pose>(found->second.world_T_local);
}

uint64_t GlobalPoseStore::GetSubmapAnchorRevision(const RawSubmapId & submap_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = anchors_.find(submap_id);
  return found == anchors_.end() ? 0U : found->second.anchor_revision;
}

bool GlobalPoseStore::HasLoopDependency(const RawSubmapId & submap_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return loop_dependencies_.find(submap_id) != loop_dependencies_.end();
}

std::optional<LoopAnchorDependencySnapshot> GlobalPoseStore::GetLoopDependency(
  const RawSubmapId & submap_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = loop_dependencies_.find(submap_id);
  if (found == loop_dependencies_.end()) {
    return std::nullopt;
  }
  return LoopAnchorDependencySnapshot{
    found->second.child_submap_id,
    found->second.parent_submap_id,
    found->second.child_control_keyframe_id,
    found->second.parent_control_keyframe_id,
    found->second.parent_control_T_child_control,
    found->second.source_commit_id};
}

std::optional<RawKeyFrameId> GlobalPoseStore::GetContinuationControl(
  const RawSubmapId & submap_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = continuations_.find(submap_id);
  return found == continuations_.end() ?
         std::nullopt : std::optional<RawKeyFrameId>(
    found->second.control_keyframe_id);
}

PoseChangeSet GlobalPoseStore::CommitAnchor(
  const RawSubmapPoseSnapshot & snapshot,
  const geometry_msgs::msg::Pose & world_T_local,
  uint64_t source_task_id,
  const std::optional<RawKeyFrameId> & hard_fiducial_keyframe)
{
  Eigen::Matrix4d anchor_matrix;
  if (!PoseToMatrix(world_T_local, &anchor_matrix)) {
    throw std::invalid_argument("world_T_local no es una pose valida");
  }

  std::vector<std::pair<RawKeyFramePoseInput, geometry_msgs::msg::Pose>> candidates;
  candidates.reserve(snapshot.keyframes.size());
  for (const auto & keyframe : snapshot.keyframes) {
    candidates.emplace_back(keyframe, Compose(world_T_local, keyframe.local_pose));
  }

  std::optional<uint64_t> hard_fiducial_raw_revision;
  if (hard_fiducial_keyframe.has_value()) {
    if (hard_fiducial_keyframe->drone_id != snapshot.submap_id.drone_id ||
      hard_fiducial_keyframe->map_epoch != snapshot.submap_id.map_epoch)
    {
      throw std::invalid_argument("hard fiducial no pertenece al submapa anclado");
    }
    const auto control = std::find_if(
      snapshot.keyframes.begin(), snapshot.keyframes.end(),
      [&hard_fiducial_keyframe](const RawKeyFramePoseInput & input) {
        return input.id == *hard_fiducial_keyframe;
      });
    if (control == snapshot.keyframes.end()) {
      throw std::invalid_argument("hard fiducial no existe en el snapshot anclado");
    }
    hard_fiducial_raw_revision = control->raw_revision;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  PoseChangeSet result;
  result.submap_id = snapshot.submap_id;
  result.source_task_id = source_task_id;
  result.store_revision_before = store_revision_;
  result.store_revision_after = store_revision_;
  if (anchors_.find(snapshot.submap_id) != anchors_.end()) {
    result.status = PoseCommitStatus::AlreadyAnchored;
    return result;
  }
  const uint64_t commit_id = next_commit_id_++;
  anchors_[snapshot.submap_id] = {
    world_T_local, 1, commit_id, source_task_id, snapshot.submap_revision};
  if (hard_fiducial_keyframe.has_value()) {
    continuations_[snapshot.submap_id] = {
      *hard_fiducial_keyframe, world_T_local, 1, commit_id, source_task_id,
      *hard_fiducial_raw_revision};
  }

  for (const auto & [input, world_pose] : candidates) {
    const auto existing = poses_.find(input.id);
    if (existing != poses_.end() && !IsRawMutable(existing->second.source_kind)) {
      result.preserved_ids.push_back(input.id);
      continue;
    }
    GlobalPoseRecord record;
    record.keyframe_id = input.id;
    record.world_pose = world_pose;
    record.raw_world_pose = world_pose;
    record.correction_pose.orientation.w = 1.0;
    record.active = input.active;
    record.pose_revision = existing == poses_.end() ? 1 : existing->second.pose_revision + 1;
    record.hard_fiducial = hard_fiducial_keyframe.has_value() &&
      input.id == *hard_fiducial_keyframe;
    record.source_kind = record.hard_fiducial ?
      PoseSourceKind::FiducialAccepted : PoseSourceKind::SubmapAnchorDerived;
    record.source_commit_id = commit_id;
    record.source_task_id = source_task_id;
    record.parent_commit_id = existing == poses_.end() ? 0 : existing->second.source_commit_id;
    record.base_raw_revision = input.raw_revision;
    poses_[input.id] = record;
    if (record.hard_fiducial) {
      result.hard_fiducial_ids.push_back(input.id);
    }
    if (existing == poses_.end()) {
      result.created_ids.push_back(input.id);
    } else {
      result.updated_ids.push_back(input.id);
    }
    if (!input.active) {
      result.invalidated_ids.push_back(input.id);
    }
  }

  ++store_revision_;
  ++commit_count_;
  result.status = PoseCommitStatus::Applied;
  result.commit_id = commit_id;
  result.store_revision_after = store_revision_;
  return result;
}

LoopAnchorBatchResult GlobalPoseStore::CommitLoopAnchorBatch(
  const std::vector<LoopAnchorBatchEntry> & entries, uint64_t source_task_id)
{
  LoopAnchorBatchResult result;
  result.source_task_id = source_task_id;
  if (entries.empty()) {
    std::lock_guard<std::mutex> lock(mutex_);
    result.store_revision_before = store_revision_;
    result.store_revision_after = store_revision_;
    return result;
  }

  struct PreparedEntry
  {
    const LoopAnchorBatchEntry * entry = nullptr;
    std::vector<std::pair<RawKeyFramePoseInput, geometry_msgs::msg::Pose>> poses;
  };
  std::vector<PreparedEntry> prepared;
  prepared.reserve(entries.size());
  std::set<RawSubmapId> unique_submaps;
  for (const auto & entry : entries) {
    Eigen::Matrix4d anchor_matrix;
    if (!PoseToMatrix(entry.world_T_local, &anchor_matrix) ||
      !unique_submaps.insert(entry.snapshot.submap_id).second)
    {
      result.status = PoseCommitStatus::AtomicBatchConflict;
      return result;
    }
    const bool control_present = std::any_of(
      entry.snapshot.keyframes.begin(), entry.snapshot.keyframes.end(),
      [&entry](const auto & input) {return input.id == entry.loop_control_keyframe_id;});
    if (!control_present) {
      result.status = PoseCommitStatus::AtomicBatchConflict;
      return result;
    }
    PreparedEntry item;
    item.entry = &entry;
    item.poses.reserve(entry.snapshot.keyframes.size());
    for (const auto & input : entry.snapshot.keyframes) {
      item.poses.emplace_back(input, Compose(entry.world_T_local, input.local_pose));
    }
    prepared.push_back(std::move(item));
  }

  std::lock_guard<std::mutex> lock(mutex_);
  result.store_revision_before = store_revision_;
  result.store_revision_after = store_revision_;
  for (const auto & item : prepared) {
    if (anchors_.find(item.entry->snapshot.submap_id) != anchors_.end()) {
      result.status = PoseCommitStatus::AtomicBatchConflict;
      return result;
    }
    if (item.entry->parent_control_keyframe_id.has_value() &&
      poses_.find(*item.entry->parent_control_keyframe_id) == poses_.end())
    {
      result.status = PoseCommitStatus::AtomicBatchConflict;
      return result;
    }
  }

  const uint64_t commit_id = next_commit_id_++;
  for (const auto & item : prepared) {
    const auto & entry = *item.entry;
    PoseChangeSet changes;
    changes.status = PoseCommitStatus::Applied;
    changes.submap_id = entry.snapshot.submap_id;
    changes.source_task_id = source_task_id;
    changes.commit_id = commit_id;
    changes.store_revision_before = store_revision_;
    changes.store_revision_after = store_revision_ + 1;
    anchors_[entry.snapshot.submap_id] = {
      entry.world_T_local, 1, commit_id, source_task_id,
      entry.snapshot.submap_revision};

    const auto control = std::find_if(
      entry.snapshot.keyframes.begin(), entry.snapshot.keyframes.end(),
      [&entry](const auto & input) {return input.id == entry.loop_control_keyframe_id;});
    continuations_[entry.snapshot.submap_id] = {
      entry.loop_control_keyframe_id, entry.world_T_local, 1, commit_id,
      source_task_id, control->raw_revision};

    if (entry.parent_submap_id.has_value() &&
      entry.parent_control_keyframe_id.has_value())
    {
      const auto parent_pose = poses_.find(*entry.parent_control_keyframe_id);
      const auto child_pose = std::find_if(
        item.poses.begin(), item.poses.end(),
        [&entry](const auto & candidate) {
          return candidate.first.id == entry.loop_control_keyframe_id;
        });
      Eigen::Matrix4d parent_world;
      Eigen::Matrix4d child_world;
      if (parent_pose == poses_.end() || child_pose == item.poses.end() ||
        !PoseToMatrix(parent_pose->second.world_pose, &parent_world) ||
        !PoseToMatrix(child_pose->second, &child_world))
      {
        throw std::invalid_argument("dependencia loop sin controles validos");
      }
      loop_dependencies_[entry.snapshot.submap_id] = {
        entry.snapshot.submap_id, *entry.parent_submap_id,
        entry.loop_control_keyframe_id, *entry.parent_control_keyframe_id,
        MatrixToPose(parent_world.inverse() * child_world),
        parent_pose->second.world_pose, commit_id};
    }

    for (const auto & [input, world_pose] : item.poses) {
      GlobalPoseRecord record;
      record.keyframe_id = input.id;
      record.world_pose = world_pose;
      record.raw_world_pose = world_pose;
      record.correction_pose.orientation.w = 1.0;
      record.active = input.active;
      record.pose_revision = 1;
      record.source_kind = PoseSourceKind::LoopAnchorDerived;
      record.source_commit_id = commit_id;
      record.source_task_id = source_task_id;
      record.base_raw_revision = input.raw_revision;
      poses_[input.id] = record;
      changes.created_ids.push_back(input.id);
      result.dirty_keyframe_ids.push_back(input.id);
      if (!input.active) {
        changes.invalidated_ids.push_back(input.id);
      }
    }
    result.anchored_submaps.push_back(entry.snapshot.submap_id);
    result.submap_changes.push_back(std::move(changes));
  }

  ++store_revision_;
  ++commit_count_;
  result.status = PoseCommitStatus::Applied;
  result.commit_id = commit_id;
  result.store_revision_after = store_revision_;
  return result;
}

PoseChangeSet GlobalPoseStore::ApplyRawPoseChanges(
  const RawSubmapId & submap_id,
  const std::vector<RawKeyFramePoseChange> & changes,
  uint64_t source_task_id)
{
  PoseChangeSet result;
  result.submap_id = submap_id;
  result.source_task_id = source_task_id;
  if (changes.empty()) {
    std::lock_guard<std::mutex> lock(mutex_);
    result.store_revision_before = store_revision_;
    result.store_revision_after = store_revision_;
    return result;
  }

  AnchorRecord anchor;
  std::optional<ContinuationRecord> continuation;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    result.store_revision_before = store_revision_;
    result.store_revision_after = store_revision_;
    const auto found = anchors_.find(submap_id);
    if (found == anchors_.end()) {
      result.status = PoseCommitStatus::Unanchored;
      for (const auto & change : changes) {
        result.skipped_unanchored_ids.push_back(change.keyframe.id);
      }
      return result;
    }
    anchor = found->second;
    const auto continuation_found = continuations_.find(submap_id);
    if (continuation_found != continuations_.end()) {
      continuation = continuation_found->second;
    }
  }

  struct PoseCandidates
  {
    geometry_msgs::msg::Pose raw_world;
    geometry_msgs::msg::Pose world;
    bool propagated = false;
  };
  std::map<RawKeyFrameId, PoseCandidates> candidates;
  for (const auto & change : changes) {
    PoseCandidates candidate;
    candidate.raw_world = Compose(anchor.world_T_local, change.keyframe.local_pose);
    candidate.propagated = continuation.has_value() &&
      change.keyframe.id.local_kf_id > continuation->control_keyframe_id.local_kf_id;
    candidate.world = candidate.propagated ?
      Compose(continuation->world_T_local, change.keyframe.local_pose) :
      candidate.raw_world;
    candidates[change.keyframe.id] = std::move(candidate);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  const auto current_anchor = anchors_.find(submap_id);
  if (current_anchor == anchors_.end() ||
    current_anchor->second.source_commit_id != anchor.source_commit_id)
  {
    result.status = PoseCommitStatus::Unanchored;
    result.store_revision_before = store_revision_;
    result.store_revision_after = store_revision_;
    for (const auto & change : changes) {
      result.skipped_unanchored_ids.push_back(change.keyframe.id);
    }
    return result;
  }
  const auto current_continuation = continuations_.find(submap_id);
  if ((continuation.has_value() &&
    (current_continuation == continuations_.end() ||
    current_continuation->second.continuation_revision !=
    continuation->continuation_revision)) ||
    (!continuation.has_value() && current_continuation != continuations_.end()))
  {
    result.status = PoseCommitStatus::RevisionConflict;
    for (const auto & change : changes) {
      result.conflict_ids.push_back(change.keyframe.id);
    }
    return result;
  }

  const uint64_t commit_id = next_commit_id_;
  std::vector<GlobalPoseRecord> staged;
  for (const auto & change : changes) {
    const auto existing = poses_.find(change.keyframe.id);
    if (existing != poses_.end() && !IsRawMutable(existing->second.source_kind) &&
      change.kind != RawKeyFramePoseChangeKind::Invalidated)
    {
      GlobalPoseRecord record = existing->second;
      record.raw_world_pose = candidates.at(change.keyframe.id).raw_world;
      record.correction_pose = CorrectionFromRawWorld(
        record.world_pose, record.raw_world_pose);
      record.pose_revision += 1;
      record.parent_commit_id = existing->second.source_commit_id;
      record.source_commit_id = commit_id;
      record.source_task_id = source_task_id;
      record.base_raw_revision = change.keyframe.raw_revision;
      result.preserved_ids.push_back(change.keyframe.id);
      staged.push_back(std::move(record));
      continue;
    }

    GlobalPoseRecord record;
    if (existing != poses_.end()) {
      record = existing->second;
      record.pose_revision += 1;
      record.parent_commit_id = existing->second.source_commit_id;
    } else {
      record.keyframe_id = change.keyframe.id;
      record.pose_revision = 1;
      record.source_kind = PoseSourceKind::SubmapAnchorDerived;
    }
    record.source_commit_id = commit_id;
    record.source_task_id = source_task_id;
    record.base_raw_revision = change.keyframe.raw_revision;
    if (change.kind == RawKeyFramePoseChangeKind::Invalidated) {
      record.active = false;
      result.invalidated_ids.push_back(change.keyframe.id);
    } else {
      const auto & candidate = candidates.at(change.keyframe.id);
      record.world_pose = candidate.world;
      record.raw_world_pose = candidate.raw_world;
      record.correction_pose = CorrectionFromRawWorld(
        record.world_pose, record.raw_world_pose);
      record.source_kind = candidate.propagated ?
        PoseSourceKind::FiducialControlDerived :
        PoseSourceKind::SubmapAnchorDerived;
      record.active = true;
      if (candidate.propagated) {
        result.control_propagated_ids.push_back(change.keyframe.id);
      }
    }
    if (existing == poses_.end()) {
      result.created_ids.push_back(change.keyframe.id);
    } else {
      result.updated_ids.push_back(change.keyframe.id);
    }
    staged.push_back(std::move(record));
  }

  if (staged.empty()) {
    result.status = PoseCommitStatus::PreservedAccepted;
    result.store_revision_before = store_revision_;
    result.store_revision_after = store_revision_;
    return result;
  }

  for (auto & record : staged) {
    poses_[record.keyframe_id] = std::move(record);
  }
  ++next_commit_id_;
  ++store_revision_;
  ++commit_count_;
  result.status = PoseCommitStatus::Applied;
  result.commit_id = commit_id;
  result.store_revision_before = store_revision_ - 1;
  result.store_revision_after = store_revision_;
  return result;
}

PoseChangeSet GlobalPoseStore::CommitAcceptedPoses(
  const RawSubmapId & submap_id,
  const std::vector<AcceptedPoseUpdate> & updates,
  PoseSourceKind source_kind,
  uint64_t source_task_id,
  const std::optional<RawKeyFrameId> & continuation_control,
  const std::optional<geometry_msgs::msg::Pose> & replacement_world_T_local)
{
  if (source_kind == PoseSourceKind::SubmapAnchorDerived ||
    source_kind == PoseSourceKind::FiducialControlDerived)
  {
    throw std::invalid_argument("una pose aceptada requiere una fuente autoritativa");
  }
  for (const auto & update : updates) {
    Eigen::Matrix4d matrix;
    if (!PoseToMatrix(update.world_pose, &matrix)) {
      throw std::invalid_argument("pose aceptada no valida");
    }
  }
  if (replacement_world_T_local.has_value()) {
    Eigen::Matrix4d replacement_anchor;
    if (!PoseToMatrix(*replacement_world_T_local, &replacement_anchor)) {
      throw std::invalid_argument("anchor fiducial de reemplazo no valido");
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  PoseChangeSet result;
  result.submap_id = submap_id;
  result.source_task_id = source_task_id;
  result.store_revision_before = store_revision_;
  result.store_revision_after = store_revision_;
  if (anchors_.find(submap_id) == anchors_.end()) {
    result.status = PoseCommitStatus::Unanchored;
    return result;
  }
  if (updates.empty()) {
    return result;
  }
  if (replacement_world_T_local.has_value() &&
    loop_dependencies_.find(submap_id) == loop_dependencies_.end())
  {
    result.status = PoseCommitStatus::AlreadyAnchored;
    return result;
  }

  const AcceptedPoseUpdate * control_update = nullptr;
  if (continuation_control.has_value()) {
    if (continuation_control->drone_id != submap_id.drone_id ||
      continuation_control->map_epoch != submap_id.map_epoch)
    {
      throw std::invalid_argument("control de continuidad fuera del submapa");
    }
    const auto found = std::find_if(
      updates.begin(), updates.end(),
      [&continuation_control](const AcceptedPoseUpdate & update) {
        return update.keyframe_id == *continuation_control;
      });
    if (found == updates.end() || !found->mark_hard_fiducial) {
      throw std::invalid_argument("control de continuidad no aceptado como hard");
    }
    control_update = &*found;
  }
  if (replacement_world_T_local.has_value() && control_update == nullptr) {
    throw std::invalid_argument("reanchor fiducial requiere control hard");
  }

  if (replacement_world_T_local.has_value()) {
    std::set<RawKeyFrameId> expected;
    for (const auto & [id, pose] : poses_) {
      (void)pose;
      if (id.drone_id == submap_id.drone_id && id.map_epoch == submap_id.map_epoch) {
        expected.insert(id);
      }
    }
    std::set<RawKeyFrameId> received;
    for (const auto & update : updates) {
      received.insert(update.keyframe_id);
    }
    if (expected != received) {
      result.status = PoseCommitStatus::AtomicBatchConflict;
      return result;
    }
  }

  for (const auto & update : updates) {
    if (update.keyframe_id.drone_id != submap_id.drone_id ||
      update.keyframe_id.map_epoch != submap_id.map_epoch)
    {
      throw std::invalid_argument("pose aceptada fuera del submapa");
    }
    const auto existing = poses_.find(update.keyframe_id);
    if (existing == poses_.end()) {
      result.status = PoseCommitStatus::RevisionConflict;
      result.conflict_ids.push_back(update.keyframe_id);
      return result;
    }
    if (update.expected_pose_revision != 0 &&
      existing->second.pose_revision != update.expected_pose_revision)
    {
      result.status = PoseCommitStatus::RevisionConflict;
      result.conflict_ids.push_back(update.keyframe_id);
      return result;
    }
    if (existing->second.hard_fiducial &&
      !PosesNear(existing->second.world_pose, update.world_pose, 1e-8, 1e-8))
    {
      result.status = PoseCommitStatus::HardConstraintViolation;
      result.conflict_ids.push_back(update.keyframe_id);
      return result;
    }
  }

  geometry_msgs::msg::Pose continuation_world_T_local;
  if (replacement_world_T_local.has_value()) {
    continuation_world_T_local = *replacement_world_T_local;
  } else if (control_update != nullptr) {
    const auto & control_record = poses_.at(control_update->keyframe_id);
    Eigen::Matrix4d accepted_world;
    Eigen::Matrix4d raw_world;
    Eigen::Matrix4d anchor_world;
    if (!PoseToMatrix(control_update->world_pose, &accepted_world) ||
      !PoseToMatrix(control_record.raw_world_pose, &raw_world) ||
      !PoseToMatrix(anchors_.at(submap_id).world_T_local, &anchor_world))
    {
      throw std::invalid_argument("no se puede derivar la continuidad aceptada");
    }
    continuation_world_T_local = MatrixToPose(
      accepted_world * raw_world.inverse() * anchor_world);
  }

  const uint64_t commit_id = next_commit_id_++;
  const bool child_becomes_hard = std::any_of(
    updates.begin(), updates.end(),
    [](const AcceptedPoseUpdate & update) {return update.mark_hard_fiducial;});
  if (child_becomes_hard) {
    loop_dependencies_.erase(submap_id);
  }
  for (const auto & update : updates) {
    const auto existing = poses_.find(update.keyframe_id);
    const GlobalPoseRecord previous = existing->second;
    GlobalPoseRecord record = previous;
    record.world_pose = update.world_pose;
    if (replacement_world_T_local.has_value()) {
      record.raw_world_pose = update.world_pose;
      record.correction_pose = geometry_msgs::msg::Pose();
      record.correction_pose.orientation.w = 1.0;
    } else {
      record.correction_pose = CorrectionFromRawWorld(
        record.world_pose, record.raw_world_pose);
    }
    record.active = true;
    record.pose_revision = previous.pose_revision + 1;
    record.source_kind = replacement_world_T_local.has_value() ?
      (update.mark_hard_fiducial ?
      PoseSourceKind::FiducialAccepted : PoseSourceKind::SubmapAnchorDerived) :
      source_kind;
    record.source_commit_id = commit_id;
    record.source_task_id = source_task_id;
    record.parent_commit_id = previous.source_commit_id;
    record.base_raw_revision = update.base_raw_revision;
    record.hard_fiducial = previous.hard_fiducial || update.mark_hard_fiducial;
    if (!PosesNear(previous.world_pose, update.world_pose, 1e-10, 1e-10)) {
      result.updated_ids.push_back(update.keyframe_id);
    } else {
      result.preserved_ids.push_back(update.keyframe_id);
    }
    if (update.mark_hard_fiducial && !previous.hard_fiducial) {
      result.hard_fiducial_ids.push_back(update.keyframe_id);
    }
    poses_[update.keyframe_id] = std::move(record);
  }

  std::set<RawKeyFrameId> moved_controls;
  for (const auto & update : updates) {
    moved_controls.insert(update.keyframe_id);
  }
  bool propagated = true;
  while (propagated) {
    propagated = false;
    for (auto & [child_submap, dependency] : loop_dependencies_) {
      if (moved_controls.count(dependency.parent_control_keyframe_id) == 0U) {
        continue;
      }
      const auto parent_pose = poses_.find(dependency.parent_control_keyframe_id);
      Eigen::Matrix4d previous_parent;
      Eigen::Matrix4d current_parent;
      if (parent_pose == poses_.end() ||
        !PoseToMatrix(dependency.parent_control_world_pose, &previous_parent) ||
        !PoseToMatrix(parent_pose->second.world_pose, &current_parent))
      {
        throw std::invalid_argument("dependencia loop no propagable");
      }
      const Eigen::Matrix4d delta = current_parent * previous_parent.inverse();
      if ((delta - Eigen::Matrix4d::Identity()).norm() <= 1e-12) {
        dependency.parent_control_world_pose = parent_pose->second.world_pose;
        continue;
      }
      for (auto & [id, pose] : poses_) {
        if (id.drone_id != child_submap.drone_id || id.map_epoch != child_submap.map_epoch) {
          continue;
        }
        if (pose.hard_fiducial) {
          throw std::invalid_argument("propagacion loop intentaria mover hard fiducial");
        }
        Eigen::Matrix4d world;
        Eigen::Matrix4d raw_world;
        if (!PoseToMatrix(pose.world_pose, &world) ||
          !PoseToMatrix(pose.raw_world_pose, &raw_world))
        {
          throw std::invalid_argument("pose hija no propagable");
        }
        const auto previous_world = pose.world_pose;
        pose.world_pose = MatrixToPose(delta * world);
        pose.raw_world_pose = MatrixToPose(delta * raw_world);
        pose.correction_pose = CorrectionFromRawWorld(pose.world_pose, pose.raw_world_pose);
        pose.pose_revision += 1;
        pose.source_kind = PoseSourceKind::LoopAnchorDerived;
        pose.parent_commit_id = pose.source_commit_id;
        pose.source_commit_id = commit_id;
        pose.source_task_id = source_task_id;
        if (!PosesNear(previous_world, pose.world_pose, 1e-10, 1e-10)) {
          result.updated_ids.push_back(id);
          result.control_propagated_ids.push_back(id);
          moved_controls.insert(id);
        }
      }
      Eigen::Matrix4d child_anchor;
      if (!PoseToMatrix(anchors_.at(child_submap).world_T_local, &child_anchor)) {
        throw std::invalid_argument("anchor hijo no propagable");
      }
      anchors_.at(child_submap).world_T_local = MatrixToPose(delta * child_anchor);
      anchors_.at(child_submap).anchor_revision += 1;
      const auto continuation = continuations_.find(child_submap);
      if (continuation != continuations_.end()) {
        Eigen::Matrix4d child_continuation;
        PoseToMatrix(continuation->second.world_T_local, &child_continuation);
        continuation->second.world_T_local = MatrixToPose(delta * child_continuation);
        continuation->second.continuation_revision += 1;
      }
      dependency.parent_control_world_pose = parent_pose->second.world_pose;
      propagated = true;
    }
  }
  if (control_update != nullptr) {
    const auto previous = continuations_.find(submap_id);
    const uint64_t continuation_revision = previous == continuations_.end() ?
      1 : previous->second.continuation_revision + 1;
    continuations_[submap_id] = {
      control_update->keyframe_id, continuation_world_T_local,
      continuation_revision, commit_id, source_task_id,
      control_update->base_raw_revision};
  }
  if (replacement_world_T_local.has_value()) {
    auto & anchor = anchors_.at(submap_id);
    anchor.world_T_local = *replacement_world_T_local;
    anchor.anchor_revision += 1;
    anchor.source_commit_id = commit_id;
    anchor.source_task_id = source_task_id;
    anchor.base_raw_revision = control_update->base_raw_revision;
  }
  std::sort(result.updated_ids.begin(), result.updated_ids.end());
  result.updated_ids.erase(
    std::unique(result.updated_ids.begin(), result.updated_ids.end()),
    result.updated_ids.end());
  std::sort(
    result.control_propagated_ids.begin(), result.control_propagated_ids.end());
  result.control_propagated_ids.erase(
    std::unique(
      result.control_propagated_ids.begin(), result.control_propagated_ids.end()),
    result.control_propagated_ids.end());
  ++store_revision_;
  ++commit_count_;
  result.status = PoseCommitStatus::Applied;
  result.commit_id = commit_id;
  result.store_revision_after = store_revision_;
  return result;
}

AcceptedPoseBatchResult GlobalPoseStore::CommitAcceptedPoseBatch(
  const std::vector<AcceptedSubmapPoseBatch> & batches,
  PoseSourceKind source_kind,
  uint64_t source_task_id)
{
  AcceptedPoseBatchResult result;
  result.source_task_id = source_task_id;
  if (source_kind != PoseSourceKind::LoopOptimized || batches.empty()) {
    result.status = PoseCommitStatus::AtomicBatchConflict;
    return result;
  }
  for (const auto & batch : batches) {
    for (const auto & update : batch.updates) {
      Eigen::Matrix4d pose;
      if (!PoseToMatrix(update.world_pose, &pose)) {
        result.status = PoseCommitStatus::AtomicBatchConflict;
        return result;
      }
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  result.store_revision_before = store_revision_;
  result.store_revision_after = store_revision_;
  std::set<RawSubmapId> explicit_submaps;
  for (const auto & batch : batches) {
    if (!explicit_submaps.insert(batch.submap_id).second ||
      anchors_.find(batch.submap_id) == anchors_.end() || batch.updates.empty())
    {
      result.status = PoseCommitStatus::AtomicBatchConflict;
      return result;
    }
    if (batch.continuation_control.has_value() && std::none_of(
        batch.updates.begin(), batch.updates.end(),
        [&batch](const AcceptedPoseUpdate & update) {
          return update.keyframe_id == *batch.continuation_control;
        }))
    {
      result.status = PoseCommitStatus::AtomicBatchConflict;
      return result;
    }
    for (const auto & update : batch.updates) {
      if (SubmapOf(update.keyframe_id) == batch.submap_id) {
        const auto existing = poses_.find(update.keyframe_id);
        if (existing == poses_.end() ||
          (update.expected_pose_revision != 0U &&
          existing->second.pose_revision != update.expected_pose_revision))
        {
          result.status = PoseCommitStatus::RevisionConflict;
          return result;
        }
        if (existing->second.hard_fiducial && !PosesNear(
            existing->second.world_pose, update.world_pose, 1e-8, 1e-8))
        {
          result.status = PoseCommitStatus::HardConstraintViolation;
          return result;
        }
      } else {
        result.status = PoseCommitStatus::AtomicBatchConflict;
        return result;
      }
    }
  }

  for (const auto & [child_submap, dependency] : loop_dependencies_) {
    const auto parent = poses_.find(dependency.parent_control_keyframe_id);
    Eigen::Matrix4d parent_world;
    Eigen::Matrix4d parent_baseline;
    if (parent == poses_.end() ||
      !PoseToMatrix(parent->second.world_pose, &parent_world) ||
      !PoseToMatrix(dependency.parent_control_world_pose, &parent_baseline))
    {
      result.status = PoseCommitStatus::AtomicBatchConflict;
      return result;
    }
    if (explicit_submaps.count(child_submap) != 0U) {
      continue;
    }
    const auto anchor = anchors_.find(child_submap);
    Eigen::Matrix4d anchor_world;
    if (anchor == anchors_.end() ||
      !PoseToMatrix(anchor->second.world_T_local, &anchor_world))
    {
      result.status = PoseCommitStatus::AtomicBatchConflict;
      return result;
    }
    const auto continuation = continuations_.find(child_submap);
    if (continuation != continuations_.end()) {
      Eigen::Matrix4d continuation_world;
      if (!PoseToMatrix(continuation->second.world_T_local, &continuation_world)) {
        result.status = PoseCommitStatus::AtomicBatchConflict;
        return result;
      }
    }
    for (const auto & [id, pose] : poses_) {
      if (!(SubmapOf(id) == child_submap)) {
        continue;
      }
      Eigen::Matrix4d world;
      Eigen::Matrix4d raw_world;
      if (pose.hard_fiducial || !PoseToMatrix(pose.world_pose, &world) ||
        !PoseToMatrix(pose.raw_world_pose, &raw_world))
      {
        result.status = pose.hard_fiducial ?
          PoseCommitStatus::HardConstraintViolation :
          PoseCommitStatus::AtomicBatchConflict;
        return result;
      }
    }
  }

  const uint64_t commit_id = next_commit_id_++;
  std::set<RawKeyFrameId> moved_controls;
  std::map<RawSubmapId, size_t> changes_by_submap;
  for (const auto & batch : batches) {
    PoseChangeSet changes;
    changes.status = PoseCommitStatus::Applied;
    changes.submap_id = batch.submap_id;
    changes.source_task_id = source_task_id;
    changes.commit_id = commit_id;
    changes.store_revision_before = store_revision_;
    changes.store_revision_after = store_revision_ + 1U;
    changes_by_submap[batch.submap_id] = result.submap_changes.size();
    for (const auto & update : batch.updates) {
      auto & record = poses_.at(update.keyframe_id);
      const auto previous = record;
      record.world_pose = update.world_pose;
      record.correction_pose = CorrectionFromRawWorld(
        record.world_pose, record.raw_world_pose);
      record.pose_revision += 1U;
      record.source_kind = source_kind;
      record.parent_commit_id = previous.source_commit_id;
      record.source_commit_id = commit_id;
      record.source_task_id = source_task_id;
      record.base_raw_revision = update.base_raw_revision;
      if (!PosesNear(previous.world_pose, record.world_pose, 1e-10, 1e-10)) {
        changes.updated_ids.push_back(update.keyframe_id);
        result.dirty_keyframe_ids.push_back(update.keyframe_id);
        moved_controls.insert(update.keyframe_id);
      } else {
        changes.preserved_ids.push_back(update.keyframe_id);
      }
    }
    if (batch.continuation_control.has_value()) {
      const auto & control = poses_.at(*batch.continuation_control);
      Eigen::Matrix4d accepted_world;
      Eigen::Matrix4d raw_world;
      Eigen::Matrix4d anchor_world;
      if (!PoseToMatrix(control.world_pose, &accepted_world) ||
        !PoseToMatrix(control.raw_world_pose, &raw_world) ||
        !PoseToMatrix(anchors_.at(batch.submap_id).world_T_local, &anchor_world))
      {
        result.status = PoseCommitStatus::AtomicBatchConflict;
        return result;
      }
      const auto previous = continuations_.find(batch.submap_id);
      const uint64_t revision = previous == continuations_.end() ?
        1U : previous->second.continuation_revision + 1U;
      continuations_[batch.submap_id] = {
        *batch.continuation_control,
        MatrixToPose(accepted_world * raw_world.inverse() * anchor_world),
        revision, commit_id, source_task_id, control.base_raw_revision};
    }
    result.submap_changes.push_back(std::move(changes));
  }

  bool propagated = true;
  while (propagated) {
    propagated = false;
    for (auto & [child_submap, dependency] : loop_dependencies_) {
      if (moved_controls.count(dependency.parent_control_keyframe_id) == 0U) {
        continue;
      }
      const auto parent = poses_.find(dependency.parent_control_keyframe_id);
      Eigen::Matrix4d previous_parent;
      Eigen::Matrix4d current_parent;
      if (parent == poses_.end() ||
        !PoseToMatrix(dependency.parent_control_world_pose, &previous_parent) ||
        !PoseToMatrix(parent->second.world_pose, &current_parent))
      {
        result.status = PoseCommitStatus::AtomicBatchConflict;
        return result;
      }
      if (explicit_submaps.count(child_submap) != 0U) {
        dependency.parent_control_world_pose = parent->second.world_pose;
        continue;
      }
      const Eigen::Matrix4d delta = current_parent * previous_parent.inverse();
      if ((delta - Eigen::Matrix4d::Identity()).norm() <= 1e-12) {
        dependency.parent_control_world_pose = parent->second.world_pose;
        continue;
      }
      size_t change_index = 0U;
      const auto existing_changes = changes_by_submap.find(child_submap);
      if (existing_changes == changes_by_submap.end()) {
        PoseChangeSet changes;
        changes.status = PoseCommitStatus::Applied;
        changes.submap_id = child_submap;
        changes.source_task_id = source_task_id;
        changes.commit_id = commit_id;
        changes.store_revision_before = store_revision_;
        changes.store_revision_after = store_revision_ + 1U;
        change_index = result.submap_changes.size();
        changes_by_submap[child_submap] = change_index;
        result.submap_changes.push_back(std::move(changes));
      } else {
        change_index = existing_changes->second;
      }
      auto & changes = result.submap_changes[change_index];
      for (auto & [id, pose] : poses_) {
        if (!(SubmapOf(id) == child_submap)) {
          continue;
        }
        if (pose.hard_fiducial) {
          result.status = PoseCommitStatus::HardConstraintViolation;
          return result;
        }
        Eigen::Matrix4d world;
        Eigen::Matrix4d raw_world;
        if (!PoseToMatrix(pose.world_pose, &world) ||
          !PoseToMatrix(pose.raw_world_pose, &raw_world))
        {
          result.status = PoseCommitStatus::AtomicBatchConflict;
          return result;
        }
        pose.world_pose = MatrixToPose(delta * world);
        pose.raw_world_pose = MatrixToPose(delta * raw_world);
        pose.correction_pose = CorrectionFromRawWorld(pose.world_pose, pose.raw_world_pose);
        pose.pose_revision += 1U;
        pose.source_kind = PoseSourceKind::LoopAnchorDerived;
        pose.parent_commit_id = pose.source_commit_id;
        pose.source_commit_id = commit_id;
        pose.source_task_id = source_task_id;
        changes.updated_ids.push_back(id);
        changes.control_propagated_ids.push_back(id);
        result.dirty_keyframe_ids.push_back(id);
        result.propagated_keyframe_ids.push_back(id);
        moved_controls.insert(id);
      }
      Eigen::Matrix4d anchor;
      if (!PoseToMatrix(anchors_.at(child_submap).world_T_local, &anchor)) {
        result.status = PoseCommitStatus::AtomicBatchConflict;
        return result;
      }
      anchors_.at(child_submap).world_T_local = MatrixToPose(delta * anchor);
      anchors_.at(child_submap).anchor_revision += 1U;
      const auto continuation = continuations_.find(child_submap);
      if (continuation != continuations_.end()) {
        Eigen::Matrix4d transform;
        if (!PoseToMatrix(continuation->second.world_T_local, &transform)) {
          result.status = PoseCommitStatus::AtomicBatchConflict;
          return result;
        }
        continuation->second.world_T_local = MatrixToPose(delta * transform);
        continuation->second.continuation_revision += 1U;
      }
      dependency.parent_control_world_pose = parent->second.world_pose;
      propagated = true;
    }
  }

  std::sort(result.dirty_keyframe_ids.begin(), result.dirty_keyframe_ids.end());
  result.dirty_keyframe_ids.erase(
    std::unique(result.dirty_keyframe_ids.begin(), result.dirty_keyframe_ids.end()),
    result.dirty_keyframe_ids.end());
  std::sort(
    result.propagated_keyframe_ids.begin(), result.propagated_keyframe_ids.end());
  result.propagated_keyframe_ids.erase(
    std::unique(
      result.propagated_keyframe_ids.begin(), result.propagated_keyframe_ids.end()),
    result.propagated_keyframe_ids.end());
  ++store_revision_;
  ++commit_count_;
  result.status = PoseCommitStatus::Applied;
  result.commit_id = commit_id;
  result.store_revision_after = store_revision_;
  return result;
}

}  // namespace orbslam3_multi
