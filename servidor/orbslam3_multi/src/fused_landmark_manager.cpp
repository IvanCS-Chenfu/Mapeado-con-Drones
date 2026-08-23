#include "orbslam3_multi/fused_landmark_manager.hpp"

#include "orbslam3_multi/pose_geometry.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <set>
#include <tuple>
#include <unordered_map>

namespace orbslam3_multi
{
namespace
{

uint32_t Hamming(
  const std::array<uint8_t, 32> & first,
  const std::array<uint8_t, 32> & second)
{
  uint32_t distance = 0;
  for (size_t index = 0; index < first.size(); index += sizeof(uint64_t)) {
    uint64_t a = 0;
    uint64_t b = 0;
    std::memcpy(&a, first.data() + index, sizeof(uint64_t));
    std::memcpy(&b, second.data() + index, sizeof(uint64_t));
    distance += static_cast<uint32_t>(__builtin_popcountll(a ^ b));
  }
  return distance;
}

bool DescriptorValid(const std::array<uint8_t, 32> & descriptor)
{
  return std::any_of(
    descriptor.begin(), descriptor.end(), [](uint8_t value) {return value != 0U;});
}

void HashValue(uint64_t * seed, uint64_t value)
{
  *seed ^= value + 0x9e3779b97f4a7c15ULL + (*seed << 6U) + (*seed >> 2U);
}

void HashId(uint64_t * seed, const RawMapPointId & id)
{
  HashValue(seed, id.drone_id);
  HashValue(seed, id.map_epoch);
  HashValue(seed, id.local_mp_id);
}

void HashId(uint64_t * seed, const RawKeyFrameId & id)
{
  HashValue(seed, id.drone_id);
  HashValue(seed, id.map_epoch);
  HashValue(seed, id.local_kf_id);
}

uint64_t EvidenceId(
  const RawMapPointId & first, const RawMapPointId & second,
  uint64_t first_revision, uint64_t second_revision,
  const RawKeyFrameId & query, const RawKeyFrameId & candidate,
  uint64_t validation_revision, uint64_t suffix)
{
  uint64_t seed = 0xcbf29ce484222325ULL;
  HashId(&seed, first);
  HashId(&seed, second);
  HashValue(&seed, first_revision);
  HashValue(&seed, second_revision);
  HashId(&seed, query);
  HashId(&seed, candidate);
  HashValue(&seed, validation_revision);
  HashValue(&seed, suffix);
  return seed == 0U ? 1U : seed;
}

struct WorldPoint
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  RawKeyFrameId observer;
  uint64_t pose_revision = 0;
  float score = 0.0F;
  double weight = 1.0;
};

std::optional<WorldPoint> ResolveWorldPoint(
  const RawFusionMapPointInput & input,
  const RawMapDatabase & raw_database,
  const GlobalPoseStore & pose_store,
  const LandmarkScoreManager & score_manager)
{
  std::vector<uint64_t> observers;
  observers.push_back(input.reference_keyframe_id);
  observers.insert(
    observers.end(), input.observer_keyframe_ids.begin(), input.observer_keyframe_ids.end());
  std::sort(observers.begin(), observers.end());
  observers.erase(std::unique(observers.begin(), observers.end()), observers.end());
  for (const uint64_t local_kf_id : observers) {
    const RawKeyFrameId keyframe_id{input.id.drone_id, input.id.map_epoch, local_kf_id};
    const auto raw_keyframe = raw_database.GetKeyFrame(keyframe_id);
    const auto pose = pose_store.GetPose(keyframe_id);
    if (!raw_keyframe.has_value() || raw_keyframe->is_bad ||
      !pose.has_value() || !pose->active)
    {
      continue;
    }
    Eigen::Isometry3d local_T_kf;
    Eigen::Isometry3d world_T_kf;
    if (!PoseToIsometry(raw_keyframe->pose, &local_T_kf) ||
      !PoseToIsometry(pose->world_pose, &world_T_kf))
    {
      continue;
    }
    const Eigen::Vector3d local(
      input.position.x, input.position.y, input.position.z);
    const Eigen::Vector3d world = world_T_kf * local_T_kf.inverse() * local;
    if (!world.allFinite()) {
      continue;
    }
    const auto score = score_manager.GetScore(input.id);
    WorldPoint result;
    result.position = world;
    result.observer = keyframe_id;
    result.pose_revision = pose->pose_revision;
    result.score = score.has_value() ? score->score : 0.0F;
    result.weight = std::max(
      1e-3, static_cast<double>(std::max<uint32_t>(1U, input.observations_count)) *
      static_cast<double>(std::max(0.01F, result.score)));
    return result;
  }
  return std::nullopt;
}

struct ProjectedPoint
{
  int cell_x = 0;
  int cell_y = 0;
  double depth = 0.0;
};

std::optional<ProjectedPoint> Project(
  const Eigen::Vector3d & local_point,
  const geometry_msgs::msg::Pose & local_T_kf,
  const RawCameraCalibration & camera,
  uint32_t cell_size)
{
  Eigen::Isometry3d local_T_camera;
  if (!camera.IsValid() || !PoseToIsometry(local_T_kf, &local_T_camera)) {
    return std::nullopt;
  }
  const Eigen::Vector3d camera_point = local_T_camera.inverse() * local_point;
  if (!camera_point.allFinite() || camera_point.z() <= 1e-6) {
    return std::nullopt;
  }
  const double u = camera.fx * camera_point.x() / camera_point.z() + camera.cx;
  const double v = camera.fy * camera_point.y() / camera_point.z() + camera.cy;
  if (u < 0.0 || v < 0.0 || u >= camera.image_width || v >= camera.image_height) {
    return std::nullopt;
  }
  const int divisor = static_cast<int>(std::max<uint32_t>(1U, cell_size));
  return ProjectedPoint{
    static_cast<int>(u) / divisor, static_cast<int>(v) / divisor, camera_point.z()};
}

float FusedScore(
  const FusedLandmarkTrack & track,
  const LandmarkScoreManager & score_manager,
  float member_bonus,
  const std::map<RawMapPointId, float> & pending_adjustments = {})
{
  if (track.member_mappoint_ids.empty()) {
    return 0.0F;
  }
  double sum = 0.0;
  for (const auto & member : track.member_mappoint_ids) {
    const auto score = score_manager.GetScore(member);
    if (score.has_value()) {
      const auto adjustment = pending_adjustments.find(member);
      sum += std::clamp(
          score->score + (adjustment == pending_adjustments.end() ?
          0.0F : adjustment->second), 0.0F, 1.0F);
    }
  }
  const float count = static_cast<float>(track.member_mappoint_ids.size());
  return std::clamp(
    static_cast<float>(sum / count) + member_bonus * count, 0.0F, 1.0F);
}

}  // namespace

const char * ToString(FusionPairAction action)
{
  switch (action) {
    case FusionPairAction::SameRawNoOp: return "same_raw_noop";
    case FusionPairAction::AlreadyFusedNoOp: return "already_fused_noop";
    case FusionPairAction::ReinforceTrack: return "reinforce_track";
    case FusionPairAction::CreateTrack: return "create_track";
    case FusionPairAction::AddMember: return "add_member";
    case FusionPairAction::MergeTracks: return "merge_tracks";
    case FusionPairAction::Reject: return "reject";
  }
  return "unknown";
}

void FusedLandmarkManager::Configure(const FusedLandmarkConfig & config)
{
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
  config_.member_bonus = std::clamp(config_.member_bonus, 0.0F, 1.0F);
}

FusionPrepareResult FusedLandmarkManager::PrepareFusion(
  const LoopTaskComputation & computation,
  const RawMapDatabase & raw_database,
  const GlobalPoseStore & pose_store,
  const LandmarkScoreManager & score_manager) const
{
  FusionPrepareResult result;
  result.patch.task_id = computation.task.task_id;

  struct PairInput
  {
    RawMapPointId first;
    RawMapPointId second;
    const LoopGeometryResult * geometry = nullptr;
  };
  std::vector<PairInput> pairs;
  std::set<RawMapPointId> raw_ids;
  for (const auto & geometry : computation.geometry_results) {
    if (!geometry.accepted || !geometry.fusion_compatible) {
      continue;
    }
    raw_ids.insert(geometry.query_cloud_ids.begin(), geometry.query_cloud_ids.end());
    raw_ids.insert(geometry.candidate_cloud_ids.begin(), geometry.candidate_cloud_ids.end());
    for (const auto & pair : geometry.inlier_pairs) {
      const auto canonical = pair.second < pair.first ?
        std::make_pair(pair.second, pair.first) : pair;
      pairs.push_back({canonical.first, canonical.second, &geometry});
      raw_ids.insert(canonical.first);
      raw_ids.insert(canonical.second);
    }
  }
  std::sort(
    pairs.begin(), pairs.end(), [](const PairInput & lhs, const PairInput & rhs) {
      return std::tie(lhs.first, lhs.second) < std::tie(rhs.first, rhs.second);
    });
  pairs.erase(
    std::unique(
      pairs.begin(), pairs.end(), [](const PairInput & lhs, const PairInput & rhs) {
        return lhs.first == rhs.first && lhs.second == rhs.second;
      }),
    pairs.end());
  if (pairs.empty()) {
    result.no_op = true;
    result.reason = "no_fusion_pairs";
    return result;
  }

  std::map<FusedTrackId, FusedLandmarkTrack> working_tracks;
  std::map<RawMapPointId, FusedTrackId> working_members;
  FusedTrackId working_next = 1;
  FusedLandmarkConfig config;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    result.patch.expected_fusion_revision = revision_;
    working_next = next_track_id_;
    config = config_;
    std::set<FusedTrackId> affected;
    for (const auto & pair : pairs) {
      const auto first = member_to_track_.find(pair.first);
      const auto second = member_to_track_.find(pair.second);
      if (first != member_to_track_.end()) {
        affected.insert(first->second);
      }
      if (second != member_to_track_.end()) {
        affected.insert(second->second);
      }
    }
    for (const auto id : affected) {
      const auto track = tracks_.find(id);
      if (track == tracks_.end()) {
        continue;
      }
      working_tracks.emplace(id, track->second);
      for (const auto & member : track->second.member_mappoint_ids) {
        working_members[member] = id;
        raw_ids.insert(member);
      }
    }
  }

  const std::vector<RawMapPointId> ordered_raw_ids(raw_ids.begin(), raw_ids.end());
  const auto raw_inputs = raw_database.GetFusionMapPointInputs(ordered_raw_ids);
  std::map<RawMapPointId, RawFusionMapPointInput> inputs;
  for (size_t index = 0; index < ordered_raw_ids.size(); ++index) {
    if (raw_inputs[index].has_value()) {
      inputs.emplace(ordered_raw_ids[index], *raw_inputs[index]);
      result.patch.expected_raw_revisions[ordered_raw_ids[index]] =
        raw_inputs[index]->raw_revision;
    }
  }

  std::map<RawMapPointId, std::optional<WorldPoint>> world_points;
  const auto world_point = [&](const RawMapPointId & id) -> std::optional<WorldPoint> {
      const auto cached = world_points.find(id);
      if (cached != world_points.end()) {
        return cached->second;
      }
      const auto raw = inputs.find(id);
      const auto resolved = raw == inputs.end() ? std::optional<WorldPoint>{} :
        ResolveWorldPoint(raw->second, raw_database, pose_store, score_manager);
      world_points[id] = resolved;
      if (resolved.has_value()) {
        result.patch.expected_pose_revisions[resolved->observer] = resolved->pose_revision;
      }
      return resolved;
    };

  std::set<FusedTrackId> touched_tracks;
  for (const auto & pair : pairs) {
    FusionPairResult pair_result;
    pair_result.first = pair.first;
    pair_result.second = pair.second;
    if (pair.first == pair.second) {
      pair_result.action = FusionPairAction::SameRawNoOp;
      pair_result.reason = "same_raw_mappoint";
      result.patch.pair_results.push_back(pair_result);
      continue;
    }
    const auto first_raw = inputs.find(pair.first);
    const auto second_raw = inputs.find(pair.second);
    if (first_raw == inputs.end() || second_raw == inputs.end() ||
      first_raw->second.is_bad || second_raw->second.is_bad ||
      !DescriptorValid(first_raw->second.descriptor) ||
      !DescriptorValid(second_raw->second.descriptor))
    {
      pair_result.reason = "invalid_raw_pair";
      result.patch.pair_results.push_back(pair_result);
      continue;
    }
    const uint64_t evidence_id = EvidenceId(
      pair.first, pair.second, first_raw->second.raw_revision,
      second_raw->second.raw_revision, pair.geometry->query_keyframe_id,
      pair.geometry->candidate_keyframe_id,
      computation.task.revision.validation_revision, 0U);
    pair_result.evidence_id = evidence_id;

    const auto first_track_it = working_members.find(pair.first);
    const auto second_track_it = working_members.find(pair.second);
    const std::optional<FusedTrackId> first_track = first_track_it == working_members.end() ?
      std::nullopt : std::optional<FusedTrackId>(first_track_it->second);
    const std::optional<FusedTrackId> second_track = second_track_it == working_members.end() ?
      std::nullopt : std::optional<FusedTrackId>(second_track_it->second);
    if ((first_track.has_value() && working_tracks.count(*first_track) == 0U) ||
      (second_track.has_value() && working_tracks.count(*second_track) == 0U))
    {
      pair_result.reason = "working_track_missing";
      result.patch.pair_results.push_back(pair_result);
      continue;
    }

    FusedTrackId target_id = 0;
    FusedTrackId retired_id = 0;
    std::set<RawMapPointId> proposed_members{pair.first, pair.second};
    if (first_track.has_value()) {
      const auto & members = working_tracks.at(*first_track).member_mappoint_ids;
      proposed_members.insert(members.begin(), members.end());
    }
    if (second_track.has_value()) {
      const auto & members = working_tracks.at(*second_track).member_mappoint_ids;
      proposed_members.insert(members.begin(), members.end());
    }

    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double total_weight = 0.0;
    bool world_valid = true;
    for (const auto & member : proposed_members) {
      const auto world = world_point(member);
      if (!world.has_value()) {
        world_valid = false;
        break;
      }
      center += world->weight * world->position;
      total_weight += world->weight;
    }
    if (!world_valid || total_weight <= 0.0) {
      pair_result.reason = "missing_world_member";
      result.patch.pair_results.push_back(pair_result);
      continue;
    }
    center /= total_weight;
    double dispersion = 0.0;
    for (const auto & member : proposed_members) {
      dispersion = std::max(dispersion, (world_point(member)->position - center).norm());
    }
    const bool same_existing_track = first_track.has_value() && second_track.has_value() &&
      *first_track == *second_track;
    if (!same_existing_track && dispersion > config.max_track_dispersion_m) {
      pair_result.reason = "track_dispersion_guard";
      result.patch.pair_results.push_back(pair_result);
      continue;
    }

    if (!first_track.has_value() && !second_track.has_value()) {
      target_id = working_next++;
      FusedLandmarkTrack track;
      track.fused_track_id = target_id;
      track.member_mappoint_ids = proposed_members;
      working_tracks[target_id] = std::move(track);
      pair_result.action = FusionPairAction::CreateTrack;
    } else if (first_track.has_value() && second_track.has_value() &&
      *first_track == *second_track)
    {
      target_id = *first_track;
      auto & track = working_tracks.at(target_id);
      if (track.evidence_ids.count(evidence_id) != 0U) {
        pair_result.action = FusionPairAction::AlreadyFusedNoOp;
        pair_result.track_id = target_id;
        pair_result.reason = "same_evidence_revision";
        result.patch.pair_results.push_back(pair_result);
        continue;
      }
      pair_result.action = FusionPairAction::ReinforceTrack;
    } else if (first_track.has_value() && second_track.has_value()) {
      target_id = std::min(*first_track, *second_track);
      retired_id = std::max(*first_track, *second_track);
      auto & keep = working_tracks.at(target_id);
      const auto retired = working_tracks.at(retired_id);
      keep.member_mappoint_ids.insert(
        retired.member_mappoint_ids.begin(), retired.member_mappoint_ids.end());
      keep.evidence_ids.insert(retired.evidence_ids.begin(), retired.evidence_ids.end());
      keep.support_count += retired.support_count;
      working_tracks.erase(retired_id);
      touched_tracks.erase(retired_id);
      result.patch.erase_track_ids.insert(retired_id);
      result.patch.fused_score_removals.push_back(retired_id);
      pair_result.action = FusionPairAction::MergeTracks;
      pair_result.retired_track_id = retired_id;
    } else {
      target_id = first_track.has_value() ? *first_track : *second_track;
      working_tracks.at(target_id).member_mappoint_ids = proposed_members;
      pair_result.action = FusionPairAction::AddMember;
    }

    auto & target = working_tracks.at(target_id);
    target.degraded = same_existing_track && dispersion > config.max_track_dispersion_m;
    target.evidence_ids.insert(evidence_id);
    ++target.support_count;
    ++target.revision;
    for (const auto & member : target.member_mappoint_ids) {
      working_members[member] = target_id;
    }
    pair_result.track_id = target_id;
    pair_result.reason = "inlier_pair_accepted";
    touched_tracks.insert(target_id);
    result.patch.pair_results.push_back(pair_result);

    result.patch.raw_score_evidence.push_back(
      {pair.first, EvidenceId(
          pair.first, pair.second, first_raw->second.raw_revision,
          second_raw->second.raw_revision, pair.geometry->query_keyframe_id,
          pair.geometry->candidate_keyframe_id,
          computation.task.revision.validation_revision, 1U),
        config.inlier_reward, LandmarkScoreEvidenceKind::InlierConfirmed});
    result.patch.raw_score_evidence.push_back(
      {pair.second, EvidenceId(
          pair.first, pair.second, first_raw->second.raw_revision,
          second_raw->second.raw_revision, pair.geometry->query_keyframe_id,
          pair.geometry->candidate_keyframe_id,
          computation.task.revision.validation_revision, 2U),
        config.inlier_reward, LandmarkScoreEvidenceKind::InlierConfirmed});
    result.patch.positive_score_events += 2U;
  }

  for (const FusedTrackId id : touched_tracks) {
    const auto working_track = working_tracks.find(id);
    if (working_track == working_tracks.end()) {
      continue;
    }
    auto & track = working_track->second;
    track.observing_keyframes.clear();
    track.source_submaps.clear();
    track.source_drone_ids.clear();
    std::vector<RawMapPointId> descriptor_members;
    double best_weight = -1.0;
    for (const auto & member : track.member_mappoint_ids) {
      const auto raw = inputs.find(member);
      const auto world = world_point(member);
      if (raw == inputs.end() || !world.has_value()) {
        continue;
      }
      track.source_drone_ids.insert(member.drone_id);
      track.source_submaps.insert({member.drone_id, member.map_epoch});
      track.observing_keyframes.insert(world->observer);
      for (const uint64_t observer : raw->second.observer_keyframe_ids) {
        track.observing_keyframes.insert({member.drone_id, member.map_epoch, observer});
      }
      if (DescriptorValid(raw->second.descriptor)) {
        descriptor_members.push_back(member);
      }
      if (world->weight > best_weight ||
        (std::fabs(world->weight - best_weight) <= 1e-9 &&
        member < track.representative_member))
      {
        best_weight = world->weight;
        track.representative_member = member;
      }
    }
    uint64_t best_distance = std::numeric_limits<uint64_t>::max();
    RawMapPointId best_member;
    for (const auto & candidate : descriptor_members) {
      uint64_t distance = 0;
      for (const auto & other : descriptor_members) {
        distance += Hamming(inputs.at(candidate).descriptor, inputs.at(other).descriptor);
      }
      if (distance < best_distance || (distance == best_distance && candidate < best_member)) {
        best_distance = distance;
        best_member = candidate;
      }
    }
    track.descriptor_valid = !descriptor_members.empty();
    if (track.descriptor_valid) {
      track.representative_descriptor = inputs.at(best_member).descriptor;
    }
    result.patch.after_tracks[id] = track;
    result.patch.erase_track_ids.insert(id);
    for (const auto & member : track.member_mappoint_ids) {
      result.patch.after_member_assignments[member] = id;
    }
  }

  for (const auto & geometry : computation.geometry_results) {
    if (touched_tracks.empty()) {
      break;
    }
    if (!geometry.accepted || !geometry.fusion_compatible) {
      continue;
    }
    ++result.patch.visibility_regions_started;
    const auto region_start = std::chrono::steady_clock::now();
    const auto query_kf = raw_database.GetKeyFrame(geometry.query_keyframe_id);
    const auto candidate_kf = raw_database.GetKeyFrame(geometry.candidate_keyframe_id);
    const auto query_camera = raw_database.GetCameraCalibration(geometry.query_submap_id);
    const auto candidate_camera = raw_database.GetCameraCalibration(geometry.candidate_submap_id);
    if (!query_kf.has_value() || !candidate_kf.has_value() ||
      !query_camera.has_value() || !candidate_camera.has_value())
    {
      continue;
    }

    const auto build_depth = [&](
      const std::vector<RawMapPointId> & target_cloud,
      const geometry_msgs::msg::Pose & target_local_T_kf,
      const RawCameraCalibration & camera) {
        std::map<std::pair<int, int>, double> depth;
        for (const auto & id : target_cloud) {
          const auto raw = inputs.find(id);
          if (raw == inputs.end()) {
            continue;
          }
          const Eigen::Vector3d point(
            raw->second.position.x, raw->second.position.y, raw->second.position.z);
          const auto projected = Project(
            point, target_local_T_kf, camera, config.visibility_cell_size_px);
          if (!projected.has_value()) {
            continue;
          }
          ++result.patch.visibility_projected_points;
          const auto key = std::make_pair(projected->cell_x, projected->cell_y);
          const auto found = depth.find(key);
          if (found == depth.end() || projected->depth < found->second) {
            depth[key] = projected->depth;
          }
        }
        return depth;
      };
    const auto evaluate_direction = [&](
      const std::map<std::pair<int, int>, double> & depth,
      const RawMapPointId & source_id,
      const Eigen::Isometry3d & target_local_T_source_local,
      const geometry_msgs::msg::Pose & target_local_T_kf,
      const RawCameraCalibration & camera,
      uint64_t suffix) {
        const auto source = inputs.find(source_id);
        if (source == inputs.end()) {
          return;
        }
        const Eigen::Vector3d source_local(
          source->second.position.x, source->second.position.y, source->second.position.z);
        const auto projected = Project(
          target_local_T_source_local * source_local,
          target_local_T_kf, camera, config.visibility_cell_size_px);
        if (!projected.has_value()) {
          return;
        }
        const auto key = std::make_pair(projected->cell_x, projected->cell_y);
        const auto observed = depth.find(key);
        if (observed == depth.end()) {
          ++result.patch.visibility_diagnostic_events;
        } else if (observed->second >
          projected->depth + config.visibility_depth_tolerance_m)
        {
          ++result.patch.visibility_diagnostic_events;
        }
        (void)suffix;
      };

    const bool has_hard_outliers = std::any_of(
      geometry.match_evidence.begin(), geometry.match_evidence.end(),
      [](const LoopGeometryResult::MatchEvidence & match) {return match.hard_outlier;});
    if (has_hard_outliers) {
      const auto candidate_depth = build_depth(
        geometry.candidate_cloud_ids, candidate_kf->pose, *candidate_camera);
      const auto query_depth = build_depth(
        geometry.query_cloud_ids, query_kf->pose, *query_camera);
      for (size_t index = 0; index < geometry.match_evidence.size(); ++index) {
        const auto & match = geometry.match_evidence[index];
        if (!match.hard_outlier) {
          continue;
        }
        evaluate_direction(
          candidate_depth, match.query_mappoint_id,
          geometry.candidate_local_T_query_local, candidate_kf->pose,
          *candidate_camera, 1000U + index * 2U);
        evaluate_direction(
          query_depth, match.candidate_mappoint_id,
          geometry.candidate_local_T_query_local.inverse(), query_kf->pose,
          *query_camera, 1001U + index * 2U);
      }
    }
    const double elapsed = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - region_start).count();
    result.patch.visibility_elapsed_ms += elapsed;
    ++result.patch.visibility_regions_completed;
  }

  std::map<RawMapPointId, float> pending_score_adjustments;
  for (const auto & evidence : result.patch.raw_score_evidence) {
    pending_score_adjustments[evidence.mappoint_id] += evidence.delta;
  }
  for (const auto & [id, track] : result.patch.after_tracks) {
    result.patch.fused_score_updates.push_back(
      {id, FusedScore(track, score_manager, config.member_bonus, pending_score_adjustments)});
  }

  result.patch.next_track_id_after = working_next;
  result.ready = !result.patch.after_tracks.empty() ||
    !result.patch.raw_score_evidence.empty();
  result.no_op = !result.ready;
  result.reason = result.ready ? "fusion_patch_ready" : "all_pairs_rejected_or_noop";
  return result;
}

FusionApplyResult FusedLandmarkManager::ApplyPatch(const FusionPatch & patch)
{
  FusionApplyResult result;
  std::lock_guard<std::mutex> lock(mutex_);
  result.changes.fusion_revision_before = revision_;
  result.changes.fusion_revision_after = revision_;
  if (patch.expected_fusion_revision != revision_) {
    result.stale = true;
    result.reason = "fusion_revision_changed";
    return result;
  }

  std::set<FusedTrackId> affected = patch.erase_track_ids;
  for (const auto & [id, track] : patch.after_tracks) {
    (void)track;
    affected.insert(id);
  }
  result.rollback.revision_before = revision_;
  result.rollback.next_track_id_before = next_track_id_;
  for (const auto id : affected) {
    const auto found = tracks_.find(id);
    result.rollback.tracks[id] = found == tracks_.end() ?
      std::nullopt : std::optional<FusedLandmarkTrack>(found->second);
  }
  for (const auto & [member, assignment] : patch.after_member_assignments) {
    (void)assignment;
    const auto found = member_to_track_.find(member);
    result.rollback.member_assignments[member] = found == member_to_track_.end() ?
      std::nullopt : std::optional<FusedTrackId>(found->second);
  }

  for (const auto id : affected) {
    const bool existed = tracks_.erase(id) != 0U;
    if (existed && patch.after_tracks.count(id) == 0U) {
      result.changes.retired_track_ids.push_back(id);
    }
  }
  for (const auto & [id, track] : patch.after_tracks) {
    const auto before = result.rollback.tracks.find(id);
    tracks_[id] = track;
    if (before == result.rollback.tracks.end() || !before->second.has_value()) {
      result.changes.created_track_ids.push_back(id);
    } else {
      result.changes.updated_track_ids.push_back(id);
    }
  }
  for (const auto & [member, assignment] : patch.after_member_assignments) {
    const auto before = result.rollback.member_assignments.at(member);
    if (assignment.has_value()) {
      member_to_track_[member] = *assignment;
      if (!before.has_value()) {
        result.changes.hidden_raw_member_ids.push_back(member);
      }
    } else {
      member_to_track_.erase(member);
      if (before.has_value()) {
        result.changes.released_raw_member_ids.push_back(member);
      }
    }
  }
  next_track_id_ = patch.next_track_id_after;
  if (result.changes.HasChanges()) {
    ++revision_;
  }
  result.changes.fusion_revision_after = revision_;
  result.committed = true;
  result.reason = result.changes.HasChanges() ? "applied" : "idempotent_no_change";
  return result;
}

bool FusedLandmarkManager::RollbackPatch(const FusionRollbackPatch & patch)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & [id, track] : patch.tracks) {
    if (track.has_value()) {
      tracks_[id] = *track;
    } else {
      tracks_.erase(id);
    }
  }
  for (const auto & [member, assignment] : patch.member_assignments) {
    if (assignment.has_value()) {
      member_to_track_[member] = *assignment;
    } else {
      member_to_track_.erase(member);
    }
  }
  revision_ = patch.revision_before;
  next_track_id_ = patch.next_track_id_before;
  return true;
}

std::optional<FusedTrackId> FusedLandmarkManager::GetTrackIdForMember(
  const RawMapPointId & id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = member_to_track_.find(id);
  return found == member_to_track_.end() ?
         std::nullopt : std::optional<FusedTrackId>(found->second);
}

std::optional<FusedLandmarkTrack> FusedLandmarkManager::GetTrack(FusedTrackId id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = tracks_.find(id);
  return found == tracks_.end() ?
         std::nullopt : std::optional<FusedLandmarkTrack>(found->second);
}

std::vector<FusedLandmarkTrack> FusedLandmarkManager::GetTracks() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<FusedLandmarkTrack> result;
  result.reserve(tracks_.size());
  for (const auto & [id, track] : tracks_) {
    (void)id;
    result.push_back(track);
  }
  return result;
}

std::vector<FusedLandmarkScoreUpdate>
FusedLandmarkManager::BuildScoreUpdatesForMembers(
  const std::vector<RawMapPointId> & member_ids,
  const LandmarkScoreManager & score_manager) const
{
  std::vector<FusedLandmarkScoreUpdate> result;
  std::lock_guard<std::mutex> lock(mutex_);
  std::set<FusedTrackId> affected;
  for (const auto & member : member_ids) {
    const auto found = member_to_track_.find(member);
    if (found != member_to_track_.end()) {
      affected.insert(found->second);
    }
  }
  result.reserve(affected.size());
  for (const auto id : affected) {
    const auto found = tracks_.find(id);
    if (found != tracks_.end()) {
      result.push_back({id, FusedScore(found->second, score_manager, config_.member_bonus)});
    }
  }
  return result;
}

FusedLandmarkStats FusedLandmarkManager::GetStats() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  FusedLandmarkStats stats;
  stats.revision = revision_;
  stats.tracks = tracks_.size();
  stats.raw_members = member_to_track_.size();
  for (const auto & [id, track] : tracks_) {
    (void)id;
    if (track.source_drone_ids.size() > 1U) {
      ++stats.multi_drone_tracks;
    }
    if (track.degraded) {
      ++stats.degraded_tracks;
    }
  }
  return stats;
}

}  // namespace orbslam3_multi
