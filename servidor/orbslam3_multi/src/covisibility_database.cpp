#include "orbslam3_multi/covisibility_database.hpp"

#include "orbslam3_multi/pose_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <set>

namespace orbslam3_multi
{
namespace
{

bool SameSubmap(const RawKeyFrameId & id, const RawSubmapId & submap)
{
  return id.drone_id == submap.drone_id && id.map_epoch == submap.map_epoch;
}

geometry_msgs::msg::Pose RelativePose(
  const geometry_msgs::msg::Pose & local_T_a,
  const geometry_msgs::msg::Pose & local_T_b)
{
  Eigen::Isometry3d a;
  Eigen::Isometry3d b;
  if (!PoseToIsometry(local_T_a, &a) || !PoseToIsometry(local_T_b, &b)) {
    return geometry_msgs::msg::Pose();
  }
  return IsometryToPose(a.inverse() * b);
}

bool SameEdgeValue(const CovisibilityEdge & lhs, const CovisibilityEdge & rhs)
{
  return lhs.source == rhs.source && lhs.support == rhs.support &&
         std::abs(lhs.information_weight - rhs.information_weight) <= 1e-12 &&
         lhs.dependency_revision_a == rhs.dependency_revision_a &&
         lhs.dependency_revision_b == rhs.dependency_revision_b &&
         PosesNear(lhs.relative_pose_measured, rhs.relative_pose_measured) &&
         PosesNear(lhs.relative_pose_current, rhs.relative_pose_current);
}

}  // namespace

const char * ToString(CovisibilityEdgeSource source)
{
  return source == CovisibilityEdgeSource::Orbslam3Native ?
         "orbslam3_native" : "server_loop_geometric";
}

CovisibilityDatabase::EdgeKey CovisibilityDatabase::CanonicalKey(
  const RawKeyFrameId & first, const RawKeyFrameId & second)
{
  return second < first ? EdgeKey{second, first} : EdgeKey{first, second};
}

CovisibilityDatabase::SourceEdgeKey CovisibilityDatabase::CanonicalSourceKey(
  const RawKeyFrameId & first, const RawKeyFrameId & second,
  CovisibilityEdgeSource source)
{
  const auto key = CanonicalKey(first, second);
  return {key.first, key.second, source};
}

bool CovisibilityDatabase::IsValid(const CovisibilityEdge & edge)
{
  Eigen::Isometry3d measured;
  Eigen::Isometry3d current;
  return !(edge.kf_a == edge.kf_b) && edge.support > 0 &&
         std::isfinite(edge.information_weight) && edge.information_weight > 0.0 &&
         PoseToIsometry(edge.relative_pose_measured, &measured) &&
         PoseToIsometry(edge.relative_pose_current, &current);
}

std::optional<CovisibilityPatch> CovisibilityDatabase::PrepareOrbslam3Patch(
  const RawMapDatabase & raw_database, const DatabaseUpdateTask & task)
{
  const auto snapshot = raw_database.GetSubmapPoseSnapshot(task.submap_id);
  if (!snapshot.has_value() || snapshot->submap_revision != task.expected_submap_revision) {
    return std::nullopt;
  }

  std::map<RawKeyFrameId, RawKeyFramePoseInput> pose_inputs;
  for (const auto & input : snapshot->keyframes) {
    pose_inputs.emplace(input.id, input);
  }

  CovisibilityPatch patch;
  patch.source_arrival_id = task.source_arrival_id;
  patch.submap_id = task.submap_id;
  patch.expected_submap_revision = task.expected_submap_revision;
  for (const auto & id : task.covisibility_keyframe_ids) {
    if (!SameSubmap(id, task.submap_id)) {
      continue;
    }
    patch.replace_neighborhoods.insert(id);
    const auto raw = raw_database.GetKeyFrame(id);
    const auto input = pose_inputs.find(id);
    if (!raw.has_value() || raw->is_bad || input == pose_inputs.end()) {
      continue;
    }
    const size_t count = std::min(
      raw->connected_keyframe_ids.size(), raw->connected_keyframe_weights.size());
    for (size_t index = 0; index < count; ++index) {
      if (raw->connected_keyframe_weights[index] == 0U) {
        continue;
      }
      RawKeyFrameId other{
        id.drone_id, id.map_epoch, raw->connected_keyframe_ids[index]};
      const auto other_input = pose_inputs.find(other);
      if (other_input == pose_inputs.end() || !other_input->second.active || other == id) {
        continue;
      }
      CovisibilityEdge edge;
      const auto key = CanonicalKey(id, other);
      edge.kf_a = key.first;
      edge.kf_b = key.second;
      edge.source = CovisibilityEdgeSource::Orbslam3Native;
      edge.support = raw->connected_keyframe_weights[index];
      edge.information_weight = static_cast<double>(edge.support);
      const auto & a = pose_inputs.at(edge.kf_a);
      const auto & b = pose_inputs.at(edge.kf_b);
      edge.relative_pose_measured = RelativePose(a.local_pose, b.local_pose);
      edge.relative_pose_current = edge.relative_pose_measured;
      edge.dependency_revision_a = a.raw_revision;
      edge.dependency_revision_b = b.raw_revision;
      edge.created_arrival_id = task.source_arrival_id;
      if (IsValid(edge)) {
        patch.upserts.push_back(std::move(edge));
      }
    }
  }
  std::sort(
    patch.upserts.begin(), patch.upserts.end(),
    [](const auto & lhs, const auto & rhs) {
      return CanonicalKey(lhs.kf_a, lhs.kf_b) < CanonicalKey(rhs.kf_a, rhs.kf_b);
    });
  patch.upserts.erase(
    std::unique(
      patch.upserts.begin(), patch.upserts.end(),
      [](const auto & lhs, const auto & rhs) {
        return CanonicalKey(lhs.kf_a, lhs.kf_b) == CanonicalKey(rhs.kf_a, rhs.kf_b);
      }),
    patch.upserts.end());
  return patch;
}

CovisibilityUpdateResult CovisibilityDatabase::ApplyPatch(
  const CovisibilityPatch & patch)
{
  return ApplyPatchTransactional(patch).update;
}

CovisibilityApplyResult CovisibilityDatabase::ApplyPatchTransactional(
  const CovisibilityPatch & patch)
{
  CovisibilityApplyResult transaction;
  auto & result = transaction.update;
  std::lock_guard<std::mutex> lock(mutex_);
  result.revision_before = revision_;
  result.revision_after = revision_;
  transaction.rollback.revision_before = revision_;
  if (patch.expected_database_revision.has_value() &&
    *patch.expected_database_revision != revision_)
  {
    result.stale = true;
    result.reason = "database_revision_changed";
    return transaction;
  }

  std::set<SourceEdgeKey> desired;
  for (const auto & edge : patch.upserts) {
    if (!IsValid(edge)) {
      result.reason = "invalid_edge";
      return transaction;
    }
    desired.insert(CanonicalSourceKey(edge.kf_a, edge.kf_b, edge.source));
  }

  std::map<SourceEdgeKey, std::optional<CovisibilityEdge>> before;
  const auto remember = [&](const SourceEdgeKey & key) {
      if (before.count(key) != 0U) {
        return;
      }
      const auto found = edges_.find(key);
      before[key] = found == edges_.end() ?
        std::nullopt : std::optional<CovisibilityEdge>(found->second);
    };

  for (auto it = edges_.begin(); it != edges_.end();) {
    const auto & first = std::get<0>(it->first);
    const auto & second = std::get<1>(it->first);
    const bool touched = patch.replace_neighborhoods.count(first) != 0U ||
      patch.replace_neighborhoods.count(second) != 0U;
    if (touched && it->second.source == CovisibilityEdgeSource::Orbslam3Native &&
      desired.count(it->first) == 0U)
    {
      remember(it->first);
      it = edges_.erase(it);
      ++result.removed;
    } else {
      ++it;
    }
  }

  for (auto edge : patch.upserts) {
    ++result.examined;
    const auto canonical = CanonicalKey(edge.kf_a, edge.kf_b);
    edge.kf_a = canonical.first;
    edge.kf_b = canonical.second;
    const auto key = CanonicalSourceKey(edge.kf_a, edge.kf_b, edge.source);
    const auto found = edges_.find(key);
    if (found == edges_.end()) {
      remember(key);
      edge.updated_revision = revision_ + 1;
      edges_.emplace(key, std::move(edge));
      ++result.added;
    } else if (SameEdgeValue(found->second, edge)) {
      ++result.unchanged;
    } else {
      remember(key);
      edge.created_arrival_id = found->second.created_arrival_id;
      edge.updated_revision = revision_ + 1;
      found->second = std::move(edge);
      ++result.updated;
    }
  }

  if (result.added != 0U || result.updated != 0U || result.removed != 0U) {
    ++revision_;
  }
  result.committed = true;
  result.revision_after = revision_;
  transaction.rollback.revision_after = revision_;
  for (const auto & [key, previous] : before) {
    transaction.rollback.entries.push_back(
      {std::get<0>(key), std::get<1>(key), std::get<2>(key), previous});
  }
  result.reason = result.revision_after == result.revision_before ?
    "idempotent_no_change" : "applied";
  return transaction;
}

bool CovisibilityDatabase::RollbackPatch(const CovisibilityRollbackPatch & patch)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (revision_ != patch.revision_after) {
    return false;
  }
  for (const auto & entry : patch.entries) {
    const auto key = CanonicalSourceKey(entry.kf_a, entry.kf_b, entry.source);
    if (entry.previous.has_value()) {
      edges_[key] = *entry.previous;
    } else {
      edges_.erase(key);
    }
  }
  revision_ = patch.revision_before;
  return true;
}

std::optional<CovisibilityEdge> CovisibilityDatabase::GetEdge(
  const RawKeyFrameId & first, const RawKeyFrameId & second) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto server = edges_.find(CanonicalSourceKey(
      first, second, CovisibilityEdgeSource::ServerLoopGeometric));
  if (server != edges_.end()) {
    return server->second;
  }
  const auto native = edges_.find(CanonicalSourceKey(
      first, second, CovisibilityEdgeSource::Orbslam3Native));
  return native == edges_.end() ?
         std::nullopt : std::optional<CovisibilityEdge>(native->second);
}

std::vector<CovisibilityEdge> CovisibilityDatabase::GetNeighbors(
  const RawKeyFrameId & keyframe_id, uint64_t min_support, size_t limit) const
{
  std::vector<CovisibilityEdge> result;
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & [key, edge] : edges_) {
    if ((std::get<0>(key) == keyframe_id || std::get<1>(key) == keyframe_id) &&
      edge.support >= min_support)
    {
      result.push_back(edge);
    }
  }
  std::sort(
    result.begin(), result.end(),
    [](const auto & lhs, const auto & rhs) {
      if (lhs.support != rhs.support) {
        return lhs.support > rhs.support;
      }
      return CanonicalKey(lhs.kf_a, lhs.kf_b) < CanonicalKey(rhs.kf_a, rhs.kf_b);
    });
  if (result.size() > limit) {
    result.resize(limit);
  }
  return result;
}

std::vector<CovisibilityEdge> CovisibilityDatabase::GetEdgesBySource(
  CovisibilityEdgeSource source) const
{
  std::vector<CovisibilityEdge> result;
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & [key, edge] : edges_) {
    (void)key;
    if (edge.source == source) {
      result.push_back(edge);
    }
  }
  return result;
}

bool CovisibilityDatabase::HasSource(
  const RawKeyFrameId & first, const RawKeyFrameId & second,
  CovisibilityEdgeSource source) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return edges_.count(CanonicalSourceKey(first, second, source)) != 0U;
}

bool CovisibilityDatabase::UpdateRelativePoseCurrent(
  const RawKeyFrameId & first, const RawKeyFrameId & second,
  const geometry_msgs::msg::Pose & relative_pose_current,
  uint64_t dependency_revision_a, uint64_t dependency_revision_b)
{
  Eigen::Isometry3d transform;
  if (!PoseToIsometry(relative_pose_current, &transform)) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  bool changed = false;
  for (const auto source : {
      CovisibilityEdgeSource::Orbslam3Native,
      CovisibilityEdgeSource::ServerLoopGeometric})
  {
    const auto found = edges_.find(CanonicalSourceKey(first, second, source));
    if (found == edges_.end()) {
      continue;
    }
    found->second.relative_pose_current = relative_pose_current;
    found->second.dependency_revision_a = dependency_revision_a;
    found->second.dependency_revision_b = dependency_revision_b;
    found->second.updated_revision = revision_ + 1;
    changed = true;
  }
  if (changed) {
    ++revision_;
  }
  return changed;
}

CovisibilityDatabaseStats CovisibilityDatabase::GetStats() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  CovisibilityDatabaseStats stats;
  stats.revision = revision_;
  stats.edges = edges_.size();
  for (const auto & [key, edge] : edges_) {
    (void)key;
    edge.source == CovisibilityEdgeSource::Orbslam3Native ?
      ++stats.orbslam3_native_edges : ++stats.server_loop_geometric_edges;
  }
  return stats;
}

}  // namespace orbslam3_multi
