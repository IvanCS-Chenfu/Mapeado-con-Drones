#include "orbslam3_multi/pose_graph_builder.hpp"

#include "orbslam3_multi/pose_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>

namespace orbslam3_multi
{
namespace
{

constexpr size_t kMaxCovisibilityEdgesPerControl = 6U;

double CornerStrength(
  const std::vector<PoseGraphKeyFrame> & keyframes, size_t index)
{
  if (index == 0 || index + 1 >= keyframes.size()) {
    return 0.0;
  }
  Eigen::Isometry3d previous;
  Eigen::Isometry3d current;
  Eigen::Isometry3d next;
  if (!PoseToIsometry(keyframes[index - 1].raw_local_pose, &previous) ||
    !PoseToIsometry(keyframes[index].raw_local_pose, &current) ||
    !PoseToIsometry(keyframes[index + 1].raw_local_pose, &next))
  {
    return 0.0;
  }
  const Eigen::Vector3d before = current.translation() - previous.translation();
  const Eigen::Vector3d after = next.translation() - current.translation();
  double translation_corner = 0.0;
  if (before.norm() > 1e-6 && after.norm() > 1e-6) {
    translation_corner = std::acos(std::clamp(
      before.normalized().dot(after.normalized()), -1.0, 1.0));
  }
  const double rotation_corner = RotationErrorRad(previous, current) +
    RotationErrorRad(current, next);
  return translation_corner + 0.25 * rotation_corner;
}

geometry_msgs::msg::Pose RelativePose(
  const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to)
{
  Eigen::Isometry3d from_transform;
  Eigen::Isometry3d to_transform;
  if (!PoseToIsometry(from, &from_transform) || !PoseToIsometry(to, &to_transform)) {
    geometry_msgs::msg::Pose invalid;
    return invalid;
  }
  return IsometryToPose(from_transform.inverse() * to_transform);
}

RawSubmapId SubmapOf(const RawKeyFrameId & id)
{
  return {id.drone_id, id.map_epoch};
}

bool SameSubmap(const RawKeyFrameId & first, const RawKeyFrameId & second)
{
  return first.drone_id == second.drone_id && first.map_epoch == second.map_epoch;
}

}  // namespace

PoseGraphBuilder::PoseGraphBuilder(FiducialOptimizationConfig config)
: config_(config)
{
}

void PoseGraphBuilder::Configure(const FiducialOptimizationConfig & config)
{
  config_ = config;
}

PoseGraphBuildResult PoseGraphBuilder::Build(
  const FiducialOptimizationTask & task,
  const RawSubmapPoseSnapshot & raw_snapshot,
  const std::map<RawKeyFrameId, GlobalPoseRecord> & poses,
  uint64_t pose_store_revision,
  const CovisibilityDatabase * covisibility_database) const
{
  PoseGraphBuildResult result;
  result.problem.kind = PoseGraphProblemKind::FiducialAbsolute;
  result.problem.task = task;
  result.problem.raw_submap_revision = raw_snapshot.submap_revision;
  result.problem.pose_store_revision = pose_store_revision;
  if (!(raw_snapshot.submap_id == task.submap_id)) {
    result.reason = "submap_mismatch";
    return result;
  }

  auto control = std::find_if(
    raw_snapshot.keyframes.begin(), raw_snapshot.keyframes.end(),
    [&task](const RawKeyFramePoseInput & input) {
      return input.id == task.control_keyframe_id;
    });
  auto target = std::find_if(
    raw_snapshot.keyframes.begin(), raw_snapshot.keyframes.end(),
    [&task](const RawKeyFramePoseInput & input) {return input.id == task.keyframe_id;});
  if (control == raw_snapshot.keyframes.end()) {
    result.reason = "control_keyframe_missing";
    return result;
  }
  if (target == raw_snapshot.keyframes.end()) {
    result.reason = "target_keyframe_missing";
    return result;
  }
  if (control > target) {
    result.reason = "target_not_after_control";
    return result;
  }

  const size_t first = static_cast<size_t>(control - raw_snapshot.keyframes.begin());
  const size_t last = static_cast<size_t>(target - raw_snapshot.keyframes.begin());
  result.problem.keyframes.reserve(last - first + 1U);
  std::vector<double> accumulated_path;
  accumulated_path.reserve(last - first + 1U);
  for (size_t offset = 0; offset <= last - first; ++offset) {
    const auto & raw = raw_snapshot.keyframes[first + offset];
    const auto pose = poses.find(raw.id);
    const bool is_control = raw.id == task.control_keyframe_id;
    const bool is_target = raw.id == task.keyframe_id;
    if (pose == poses.end()) {
      if (!is_control && !is_target) {
        continue;
      }
      result.reason = is_control ? "control_world_pose_missing" :
        "target_world_pose_missing";
      return result;
    }
    if (is_control && !pose->second.hard_fiducial &&
      !task.replaces_soft_loop_anchor)
    {
      result.reason = "control_not_hard_fiducial";
      return result;
    }
    if (is_target && (!raw.active || !pose->second.active)) {
      result.reason = "target_world_pose_inactive";
      return result;
    }
    if (!is_control && !is_target && (!raw.active || !pose->second.active)) {
      continue;
    }
    PoseGraphKeyFrame keyframe;
    keyframe.id = raw.id;
    keyframe.raw_local_pose = raw.local_pose;
    keyframe.current_world_pose = pose->second.world_pose;
    keyframe.raw_revision = raw.raw_revision;
    keyframe.pose_revision = pose->second.pose_revision;
    result.problem.keyframes.push_back(keyframe);
    accumulated_path.push_back(accumulated_path.empty() ? 0.0 : accumulated_path.back());
    if (result.problem.keyframes.size() > 1U) {
      Eigen::Isometry3d previous;
      Eigen::Isometry3d current;
      PoseToIsometry(
        result.problem.keyframes[result.problem.keyframes.size() - 2U].raw_local_pose,
        &previous);
      PoseToIsometry(keyframe.raw_local_pose, &current);
      accumulated_path.back() = accumulated_path[accumulated_path.size() - 2U] +
        (current.translation() - previous.translation()).norm();
    }
  }

  const size_t count = result.problem.keyframes.size();
  if (count < 2U) {
    result.reason = "window_too_small";
    return result;
  }

  const double total_path = accumulated_path.back();
  std::vector<double> coverage_coordinate(count, 0.0);
  for (size_t index = 0; index < count; ++index) {
    const double path_alpha = total_path > 1e-9 ? accumulated_path[index] / total_path :
      static_cast<double>(index) / static_cast<double>(count - 1U);
    const double temporal_alpha =
      static_cast<double>(index) / static_cast<double>(count - 1U);
    result.problem.keyframes[index].path_alpha = path_alpha;
    coverage_coordinate[index] = 0.7 * path_alpha + 0.3 * temporal_alpha;
  }

  const size_t requested_controls = static_cast<size_t>(std::ceil(
    std::clamp(config_.control_vertex_ratio, 0.0, 1.0) * static_cast<double>(count)));
  const size_t control_count = std::min(count, std::max<size_t>(2U, requested_controls));
  std::set<size_t> selected{0U, count - 1U};

  const size_t protected_total = std::min(
    control_count > 2U ? control_count - 2U : 0U,
    static_cast<size_t>(std::floor(
      std::clamp(config_.endpoint_neighborhood_ratio, 0.0, 1.0) *
      static_cast<double>(control_count))));
  const size_t protected_start = protected_total / 2U;
  const size_t protected_end = protected_total - protected_start;
  for (size_t index = 1U; index <= protected_start && index + 1U < count; ++index) {
    selected.insert(index);
  }
  for (size_t step = 1U; step <= protected_end && step + 1U < count; ++step) {
    selected.insert(count - 1U - step);
  }

  for (size_t rank = selected.size(); rank < control_count; ++rank) {
    const double desired = static_cast<double>(rank) /
      static_cast<double>(control_count - 1U);
    size_t best_index = 0U;
    double best_score = std::numeric_limits<double>::infinity();
    for (size_t index = 1U; index + 1U < count; ++index) {
      if (selected.count(index) != 0U) {
        continue;
      }
      const double distance = std::abs(coverage_coordinate[index] - desired);
      const double corner_bonus = std::min(0.03, 0.01 * CornerStrength(
        result.problem.keyframes, index));
      const double score = distance - corner_bonus;
      if (score < best_score ||
        (std::abs(score - best_score) < 1e-12 && index < best_index))
      {
        best_score = score;
        best_index = index;
      }
    }
    selected.insert(best_index);
  }

  if (covisibility_database != nullptr) {
    std::map<RawKeyFrameId, size_t> window_indices;
    for (size_t index = 0; index < count; ++index) {
      window_indices[result.problem.keyframes[index].id] = index;
    }
    for (const auto & [id, index] : window_indices) {
      for (const auto & edge : covisibility_database->GetNeighbors(
          id, config_.covisibility_min_support, 64U))
      {
        if (edge.source != CovisibilityEdgeSource::ServerLoopGeometric) {
          continue;
        }
        const RawKeyFrameId other = edge.kf_a == id ? edge.kf_b : edge.kf_a;
        const auto found = window_indices.find(other);
        if (found != window_indices.end()) {
          selected.insert(index);
          selected.insert(found->second);
        }
      }
    }
  }

  result.problem.control_indices.assign(selected.begin(), selected.end());
  const double endpoint_half_ratio =
    0.5 * std::clamp(config_.endpoint_neighborhood_ratio, 0.0, 1.0);
  for (const size_t index : result.problem.control_indices) {
    auto & keyframe = result.problem.keyframes[index];
    keyframe.control = true;
    keyframe.fixed = index == 0U &&
      !task.replaces_soft_loop_anchor;
    keyframe.protected_neighborhood =
      keyframe.path_alpha <= endpoint_half_ratio ||
      keyframe.path_alpha >= 1.0 - endpoint_half_ratio;
  }

  for (size_t edge_index = 1U; edge_index < result.problem.control_indices.size(); ++edge_index) {
    const size_t from = result.problem.control_indices[edge_index - 1U];
    const size_t to = result.problem.control_indices[edge_index];
    result.problem.temporal_edges.push_back(
      {from, to, RelativePose(
          result.problem.keyframes[from].raw_local_pose,
          result.problem.keyframes[to].raw_local_pose), to - from + 1U});
  }

  if (covisibility_database != nullptr) {
    result.problem.covisibility_revision =
      covisibility_database->GetStats().revision;
    std::map<RawKeyFrameId, size_t> controls_by_id;
    for (const size_t index : result.problem.control_indices) {
      controls_by_id[result.problem.keyframes[index].id] = index;
    }
    std::set<std::pair<size_t, size_t>> inserted;
    for (const auto & [id, from] : controls_by_id) {
      size_t accepted_neighbors = 0U;
      for (const auto & edge : covisibility_database->GetNeighbors(id, 1U, 64U)) {
        if (edge.source == CovisibilityEdgeSource::Orbslam3Native &&
          edge.support < config_.covisibility_min_support)
        {
          continue;
        }
        const RawKeyFrameId other = edge.kf_a == id ? edge.kf_b : edge.kf_a;
        const auto found = controls_by_id.find(other);
        if (found == controls_by_id.end()) {
          continue;
        }
        const auto key = std::minmax(from, found->second);
        if (!inserted.insert(key).second) {
          continue;
        }
        PoseGraphEdge graph_edge;
        graph_edge.from_index = key.first;
        graph_edge.to_index = key.second;
        graph_edge.relative_raw_pose = edge.kf_a == result.problem.keyframes[key.first].id ?
          edge.relative_pose_measured : RelativePose(
            geometry_msgs::msg::Pose(), edge.relative_pose_measured);
        if (!(edge.kf_a == result.problem.keyframes[key.first].id)) {
          Eigen::Isometry3d measured;
          if (PoseToIsometry(edge.relative_pose_measured, &measured)) {
            graph_edge.relative_raw_pose = IsometryToPose(measured.inverse());
          }
        }
        graph_edge.supporting_keyframes = edge.support;
        graph_edge.information_weight = std::max(1.0, edge.information_weight);
        graph_edge.kind = edge.source == CovisibilityEdgeSource::Orbslam3Native ?
          PoseGraphEdgeKind::CovisibilityNative : PoseGraphEdgeKind::PriorLoop;
        result.problem.covisibility_edges.push_back(std::move(graph_edge));
        if (++accepted_neighbors >= kMaxCovisibilityEdgesPerControl) {
          break;
        }
      }
    }
  }

  for (size_t index = 0; index < count; ++index) {
    if (selected.count(index) != 0U) {
      continue;
    }
    const auto upper = std::upper_bound(
      result.problem.control_indices.begin(), result.problem.control_indices.end(), index);
    const size_t upper_control = *upper;
    const size_t lower_control = *(upper - 1);
    const double lower_alpha = result.problem.keyframes[lower_control].path_alpha;
    const double upper_alpha = result.problem.keyframes[upper_control].path_alpha;
    const double segment_alpha = upper_alpha > lower_alpha + 1e-12 ?
      (result.problem.keyframes[index].path_alpha - lower_alpha) /
      (upper_alpha - lower_alpha) :
      static_cast<double>(index - lower_control) /
      static_cast<double>(upper_control - lower_control);
    result.problem.propagation_plan.push_back(
      {index, lower_control, upper_control, std::clamp(segment_alpha, 0.0, 1.0)});
  }

  PoseGraphSubmapWindow window;
  window.submap_id = task.submap_id;
  window.raw_submap_revision = raw_snapshot.submap_revision;
  window.graph_begin = 0U;
  window.graph_end = count - 1U;
  window.first_keyframe_id = task.control_keyframe_id;
  window.last_keyframe_id = task.keyframe_id;
  window.continuation_keyframe_id = task.keyframe_id;
  result.problem.submap_windows.push_back(window);

  result.success = true;
  result.reason = "balanced_temporal_graph";
  return result;
}

PoseGraphBuildResult PoseGraphBuilder::BuildLoop(
  const LoopTaskComputation & computation,
  const std::map<RawSubmapId, RawSubmapPoseSnapshot> & raw_snapshots,
  const std::map<RawKeyFrameId, GlobalPoseRecord> & poses,
  uint64_t pose_store_revision,
  const CovisibilityDatabase & covisibility_database,
  const LoopPipelineConfig & loop_config,
  const std::map<RawSubmapId, LoopAnchorDependencySnapshot> &
  loop_dependencies) const
{
  PoseGraphBuildResult result;
  result.problem.kind = PoseGraphProblemKind::LoopRelative;
  result.problem.loop_task = computation.task;
  result.problem.pose_store_revision = pose_store_revision;
  result.problem.covisibility_revision = covisibility_database.GetStats().revision;
  result.problem.loop_translation_threshold_m =
    loop_config.fusion_translation_threshold_m;
  result.problem.loop_rotation_threshold_rad =
    loop_config.fusion_rotation_threshold_rad;
  result.problem.structural_temporal_increase_m =
    loop_config.structural_temporal_increase_m;
  result.problem.structural_temporal_increase_rad =
    loop_config.structural_temporal_increase_rad;
  result.problem.structural_covisibility_increase_m =
    loop_config.structural_covisibility_increase_m;
  result.problem.structural_covisibility_increase_rad =
    loop_config.structural_covisibility_increase_rad;
  result.problem.structural_prior_loop_increase_m =
    loop_config.structural_prior_loop_increase_m;
  result.problem.structural_prior_loop_increase_rad =
    loop_config.structural_prior_loop_increase_rad;
  result.problem.hard_corridor_max_translation_m =
    loop_config.hard_corridor_max_translation_m;
  result.problem.hard_corridor_max_rotation_rad =
    loop_config.hard_corridor_max_rotation_rad;

  std::vector<std::pair<size_t, const LoopGeometryResult *>> selected_geometries;
  for (const size_t index : computation.optimization_geometry_indices) {
    if (index >= computation.geometry_results.size()) {
      result.reason = "loop_geometry_index_invalid";
      return result;
    }
    const auto & geometry = computation.geometry_results[index];
    if (!geometry.accepted || geometry.fusion_compatible) {
      result.reason = "loop_geometry_index_not_optimizable";
      return result;
    }
    selected_geometries.emplace_back(index, &geometry);
  }
  if (selected_geometries.empty()) {
    size_t best_index = 0U;
    const LoopGeometryResult * best = nullptr;
    for (size_t index = 0; index < computation.geometry_results.size(); ++index) {
      const auto & geometry = computation.geometry_results[index];
      if (!geometry.accepted || geometry.fusion_compatible) {
        continue;
      }
      if (best == nullptr || geometry.inliers > best->inliers ||
        (geometry.inliers == best->inliers &&
        geometry.mean_residual_m < best->mean_residual_m))
      {
        best = &geometry;
        best_index = index;
      }
    }
    if (best != nullptr) {
      selected_geometries.emplace_back(best_index, best);
    }
  }
  if (selected_geometries.empty()) {
    result.reason = "loop_geometry_missing";
    return result;
  }

  std::map<RawSubmapId, std::vector<RawKeyFrameId>> endpoints;
  for (const auto & [geometry_index, selected_geometry] : selected_geometries) {
    (void)geometry_index;
    endpoints[selected_geometry->query_submap_id].push_back(
      selected_geometry->query_keyframe_id);
    endpoints[selected_geometry->candidate_submap_id].push_back(
      selected_geometry->candidate_keyframe_id);
  }
  for (const auto & [child_submap, dependency] : loop_dependencies) {
    (void)child_submap;
    endpoints[dependency.child_submap_id].push_back(
      dependency.child_control_keyframe_id);
    endpoints[dependency.parent_submap_id].push_back(
      dependency.parent_control_keyframe_id);
  }
  for (const auto & edge : covisibility_database.GetEdgesBySource(
      CovisibilityEdgeSource::ServerLoopGeometric))
  {
    const RawSubmapId first{edge.kf_a.drone_id, edge.kf_a.map_epoch};
    const RawSubmapId second{edge.kf_b.drone_id, edge.kf_b.map_epoch};
    if (first == second || raw_snapshots.count(first) == 0U ||
      raw_snapshots.count(second) == 0U)
    {
      continue;
    }
    endpoints[first].push_back(edge.kf_a);
    endpoints[second].push_back(edge.kf_b);
  }
  for (auto & [submap_id, submap_endpoints] : endpoints) {
    (void)submap_id;
    std::sort(submap_endpoints.begin(), submap_endpoints.end());
    submap_endpoints.erase(
      std::unique(submap_endpoints.begin(), submap_endpoints.end()),
      submap_endpoints.end());
  }
  std::map<RawKeyFrameId, size_t> graph_index;
  std::map<RawSubmapId, std::vector<size_t>> submap_indices;

  for (const auto & [submap_id, submap_endpoints] : endpoints) {
    const auto snapshot_found = raw_snapshots.find(submap_id);
    if (snapshot_found == raw_snapshots.end()) {
      result.reason = "loop_raw_submap_missing";
      return result;
    }
    const auto & snapshot = snapshot_found->second;
    std::vector<size_t> endpoint_indices;
    for (const auto & endpoint : submap_endpoints) {
      const auto found = std::find_if(
        snapshot.keyframes.begin(), snapshot.keyframes.end(),
        [&endpoint](const auto & input) {return input.id == endpoint;});
      if (found == snapshot.keyframes.end()) {
        result.reason = "loop_endpoint_missing";
        return result;
      }
      endpoint_indices.push_back(static_cast<size_t>(found - snapshot.keyframes.begin()));
    }

    size_t first = *std::min_element(endpoint_indices.begin(), endpoint_indices.end());
    size_t last = *std::max_element(endpoint_indices.begin(), endpoint_indices.end());
    bool found_hard = false;
    for (const size_t endpoint_index : endpoint_indices) {
      std::optional<size_t> lower;
      std::optional<size_t> upper;
      for (size_t index = 0; index < snapshot.keyframes.size(); ++index) {
        const auto pose = poses.find(snapshot.keyframes[index].id);
        if (pose == poses.end() || !pose->second.hard_fiducial) {
          continue;
        }
        found_hard = true;
        if (index <= endpoint_index) {
          lower = index;
        }
        if (index >= endpoint_index && !upper.has_value()) {
          upper = index;
        }
      }
      if (lower.has_value()) {
        first = std::min(first, *lower);
      }
      if (upper.has_value()) {
        last = std::max(last, *upper);
      }
    }
    if (!found_hard) {
      first = 0U;
      last = snapshot.keyframes.empty() ? 0U : snapshot.keyframes.size() - 1U;
    }
    if (snapshot.keyframes.empty() || first > last) {
      result.reason = "loop_window_empty";
      return result;
    }

    PoseGraphSubmapWindow window;
    window.submap_id = submap_id;
    window.raw_submap_revision = snapshot.submap_revision;
    window.graph_begin = result.problem.keyframes.size();
    window.first_keyframe_id = snapshot.keyframes[first].id;
    window.last_keyframe_id = snapshot.keyframes[last].id;
    window.continuation_keyframe_id = *std::max_element(
      submap_endpoints.begin(), submap_endpoints.end(),
      [](const auto & lhs, const auto & rhs) {
        return lhs.local_kf_id < rhs.local_kf_id;
      });

    std::vector<double> path;
    for (size_t raw_index = first; raw_index <= last; ++raw_index) {
      const auto & raw = snapshot.keyframes[raw_index];
      const auto pose = poses.find(raw.id);
      const bool mandatory_endpoint = std::find(
        submap_endpoints.begin(), submap_endpoints.end(), raw.id) !=
        submap_endpoints.end();
      if (pose == poses.end() || (!raw.active && !mandatory_endpoint) ||
        (!pose->second.active && !mandatory_endpoint))
      {
        continue;
      }
      PoseGraphKeyFrame keyframe;
      keyframe.id = raw.id;
      keyframe.raw_local_pose = raw.local_pose;
      keyframe.current_world_pose = pose->second.world_pose;
      keyframe.raw_revision = raw.raw_revision;
      keyframe.pose_revision = pose->second.pose_revision;
      keyframe.fixed = pose->second.hard_fiducial;
      graph_index[keyframe.id] = result.problem.keyframes.size();
      submap_indices[submap_id].push_back(result.problem.keyframes.size());
      result.problem.keyframes.push_back(std::move(keyframe));
      path.push_back(path.empty() ? 0.0 : path.back());
      if (path.size() > 1U) {
        Eigen::Isometry3d previous;
        Eigen::Isometry3d current;
        PoseToIsometry(result.problem.keyframes[result.problem.keyframes.size() - 2U].raw_local_pose,
          &previous);
        PoseToIsometry(result.problem.keyframes.back().raw_local_pose, &current);
        path.back() = path[path.size() - 2U] +
          (current.translation() - previous.translation()).norm();
      }
    }
    if (submap_indices[submap_id].size() < 2U) {
      result.reason = "loop_submap_window_too_small";
      return result;
    }
    window.graph_end = result.problem.keyframes.size() - 1U;
    if (result.problem.keyframes[window.graph_end].fixed &&
      window.last_keyframe_id.local_kf_id >
      window.continuation_keyframe_id.local_kf_id)
    {
      window.continuation_keyframe_id = window.last_keyframe_id;
    }
    const double total_path = path.back();
    for (size_t offset = 0; offset < submap_indices[submap_id].size(); ++offset) {
      const size_t index = submap_indices[submap_id][offset];
      result.problem.keyframes[index].path_alpha = total_path > 1e-9 ?
        path[offset] / total_path : static_cast<double>(offset) /
        static_cast<double>(submap_indices[submap_id].size() - 1U);
    }
    result.problem.submap_windows.push_back(window);
  }

  std::set<size_t> selected_controls;
  for (const auto & [submap_id, indices] : submap_indices) {
    (void)submap_id;
    selected_controls.insert(indices.front());
    selected_controls.insert(indices.back());
    for (const size_t index : indices) {
      const bool loop_endpoint = std::any_of(
        selected_geometries.begin(), selected_geometries.end(),
        [&result, index](const auto & selected) {
          return result.problem.keyframes[index].id == selected.second->query_keyframe_id ||
                 result.problem.keyframes[index].id == selected.second->candidate_keyframe_id;
        });
      if (result.problem.keyframes[index].fixed || loop_endpoint)
      {
        selected_controls.insert(index);
      }
    }
    const size_t wanted = std::max<size_t>(
      2U, static_cast<size_t>(std::ceil(
        std::clamp(config_.control_vertex_ratio, 0.0, 1.0) * indices.size())));
    for (size_t rank = 0; rank < wanted; ++rank) {
      selected_controls.insert(indices[rank * (indices.size() - 1U) /
        std::max<size_t>(1U, wanted - 1U)]);
    }
  }

  for (const auto & [id, index] : graph_index) {
    for (const auto & edge : covisibility_database.GetNeighbors(
        id, loop_config.strong_covisibility_support, 64U))
    {
      if (edge.source != CovisibilityEdgeSource::ServerLoopGeometric) {
        continue;
      }
      const RawKeyFrameId other = edge.kf_a == id ? edge.kf_b : edge.kf_a;
      const auto found = graph_index.find(other);
      if (found != graph_index.end()) {
        selected_controls.insert(index);
        selected_controls.insert(found->second);
      }
    }
  }
  for (const auto & [child_submap, dependency] : loop_dependencies) {
    (void)child_submap;
    const auto parent = graph_index.find(dependency.parent_control_keyframe_id);
    const auto child = graph_index.find(dependency.child_control_keyframe_id);
    if (parent == graph_index.end() || child == graph_index.end()) {
      result.reason = "loop_dependency_control_missing";
      return result;
    }
    selected_controls.insert(parent->second);
    selected_controls.insert(child->second);
  }
  result.problem.control_indices.assign(
    selected_controls.begin(), selected_controls.end());
  if (std::none_of(
      result.problem.control_indices.begin(), result.problem.control_indices.end(),
      [&result](size_t index) {return result.problem.keyframes[index].fixed;}))
  {
    result.problem.keyframes[result.problem.control_indices.front()].fixed = true;
  }
  for (const size_t index : result.problem.control_indices) {
    result.problem.keyframes[index].control = true;
  }

  for (const auto & [submap_id, indices] : submap_indices) {
    (void)submap_id;
    std::vector<size_t> controls;
    for (const size_t index : indices) {
      if (selected_controls.count(index) != 0U) {
        controls.push_back(index);
      }
    }
    for (size_t edge_index = 1U; edge_index < controls.size(); ++edge_index) {
      const size_t from = controls[edge_index - 1U];
      const size_t to = controls[edge_index];
      PoseGraphEdge edge;
      edge.from_index = from;
      edge.to_index = to;
      edge.relative_raw_pose = RelativePose(
        result.problem.keyframes[from].raw_local_pose,
        result.problem.keyframes[to].raw_local_pose);
      edge.supporting_keyframes = to - from + 1U;
      edge.information_weight = 2.0;
      edge.kind = PoseGraphEdgeKind::Temporal;
      result.problem.temporal_edges.push_back(std::move(edge));
    }
    for (const size_t index : indices) {
      if (selected_controls.count(index) != 0U) {
        continue;
      }
      const auto upper = std::upper_bound(controls.begin(), controls.end(), index);
      if (upper == controls.begin() || upper == controls.end()) {
        continue;
      }
      const size_t lower = *(upper - 1);
      const double denominator =
        result.problem.keyframes[*upper].path_alpha -
        result.problem.keyframes[lower].path_alpha;
      const double alpha = denominator > 1e-9 ?
        (result.problem.keyframes[index].path_alpha -
        result.problem.keyframes[lower].path_alpha) / denominator : 0.5;
      result.problem.propagation_plan.push_back(
        {index, lower, *upper, std::clamp(alpha, 0.0, 1.0)});
    }
  }

  std::set<std::pair<size_t, size_t>> covisibility_pairs;
  for (const size_t from : result.problem.control_indices) {
    const auto & id = result.problem.keyframes[from].id;
    size_t accepted_neighbors = 0U;
    for (const auto & edge : covisibility_database.GetNeighbors(
        id, 1U, std::numeric_limits<size_t>::max()))
    {
      if (edge.source == CovisibilityEdgeSource::Orbslam3Native &&
        edge.support < loop_config.strong_covisibility_support)
      {
        continue;
      }
      if (edge.source == CovisibilityEdgeSource::Orbslam3Native &&
        accepted_neighbors >= kMaxCovisibilityEdgesPerControl)
      {
        continue;
      }
      const RawKeyFrameId other = edge.kf_a == id ? edge.kf_b : edge.kf_a;
      const auto found = graph_index.find(other);
      if (found == graph_index.end() || selected_controls.count(found->second) == 0U) {
        continue;
      }
      const auto key = std::minmax(from, found->second);
      if (!covisibility_pairs.insert(key).second) {
        continue;
      }
      PoseGraphEdge graph_edge;
      graph_edge.from_index = key.first;
      graph_edge.to_index = key.second;
      Eigen::Isometry3d measured;
      if (!PoseToIsometry(edge.relative_pose_measured, &measured)) {
        continue;
      }
      graph_edge.relative_raw_pose = edge.kf_a == result.problem.keyframes[key.first].id ?
        edge.relative_pose_measured : IsometryToPose(measured.inverse());
      graph_edge.supporting_keyframes = edge.support;
      graph_edge.information_weight = edge.source ==
        CovisibilityEdgeSource::ServerLoopGeometric ?
        std::clamp(edge.information_weight, 1.0, 60.0) :
        std::clamp(std::log1p(static_cast<double>(edge.support)), 1.0, 8.0);
      graph_edge.kind = edge.source == CovisibilityEdgeSource::Orbslam3Native ?
        PoseGraphEdgeKind::CovisibilityNative : PoseGraphEdgeKind::PriorLoop;
      result.problem.covisibility_edges.push_back(std::move(graph_edge));
      if (edge.source == CovisibilityEdgeSource::Orbslam3Native) {
        ++accepted_neighbors;
      }
    }
  }

  for (const auto & [child_submap, dependency] : loop_dependencies) {
    (void)child_submap;
    const auto parent = graph_index.find(dependency.parent_control_keyframe_id);
    const auto child = graph_index.find(dependency.child_control_keyframe_id);
    if (parent == graph_index.end() || child == graph_index.end()) {
      result.reason = "loop_dependency_control_missing";
      return result;
    }
    PoseGraphEdge dependency_edge;
    dependency_edge.from_index = parent->second;
    dependency_edge.to_index = child->second;
    dependency_edge.relative_raw_pose =
      dependency.parent_control_T_child_control;
    dependency_edge.supporting_keyframes = 1U;
    dependency_edge.information_weight = 12.0;
    dependency_edge.kind = PoseGraphEdgeKind::PriorLoop;
    result.problem.covisibility_edges.push_back(std::move(dependency_edge));
  }

  for (const auto & [geometry_index, selected_geometry] : selected_geometries) {
    const auto query_graph = graph_index.find(selected_geometry->query_keyframe_id);
    const auto candidate_graph = graph_index.find(selected_geometry->candidate_keyframe_id);
    const auto query_snapshot = raw_snapshots.find(selected_geometry->query_submap_id);
    const auto candidate_snapshot = raw_snapshots.find(selected_geometry->candidate_submap_id);
    if (query_graph == graph_index.end() || candidate_graph == graph_index.end() ||
      query_snapshot == raw_snapshots.end() || candidate_snapshot == raw_snapshots.end())
    {
      result.reason = "loop_endpoint_graph_missing";
      return result;
    }
    Eigen::Isometry3d query_local_T_kf;
    Eigen::Isometry3d candidate_local_T_kf;
    if (!PoseToIsometry(result.problem.keyframes[query_graph->second].raw_local_pose,
        &query_local_T_kf) ||
      !PoseToIsometry(result.problem.keyframes[candidate_graph->second].raw_local_pose,
        &candidate_local_T_kf))
    {
      result.reason = "loop_endpoint_raw_pose_invalid";
      return result;
    }
    PoseGraphEdge loop_edge;
    loop_edge.from_index = candidate_graph->second;
    loop_edge.to_index = query_graph->second;
    loop_edge.relative_raw_pose = IsometryToPose(
      candidate_local_T_kf.inverse() *
      selected_geometry->candidate_local_T_query_local * query_local_T_kf);
    loop_edge.supporting_keyframes = selected_geometry->inliers;
    const double inlier_ratio = selected_geometry->matches == 0U ? 0.0 :
      static_cast<double>(selected_geometry->inliers) /
      static_cast<double>(selected_geometry->matches);
    loop_edge.information_weight = std::clamp(20.0 + 40.0 * inlier_ratio, 20.0, 60.0);
    loop_edge.kind = PoseGraphEdgeKind::CurrentLoop;
    loop_edge.source_geometry_index = geometry_index;
    result.problem.loop_edges.push_back(std::move(loop_edge));
  }
  result.success = true;
  result.reason = SameSubmap(
    selected_geometries.front().second->query_keyframe_id,
    selected_geometries.front().second->candidate_keyframe_id) ?
    "covisible_intra_submap_loop_graph" : "covisible_multi_submap_loop_graph";
  return result;
}

}  // namespace orbslam3_multi
