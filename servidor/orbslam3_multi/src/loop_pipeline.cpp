#include "orbslam3_multi/loop_pipeline.hpp"

#include "orbslam3_multi/pose_geometry.hpp"

#include <Eigen/SVD>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <queue>
#include <random>
#include <unordered_set>

namespace orbslam3_multi
{
namespace
{

RawSubmapId SubmapOf(const RawKeyFrameId & id)
{
  return {id.drone_id, id.map_epoch};
}

std::pair<RawKeyFrameId, RawKeyFrameId> CanonicalKeyFrames(
  const RawKeyFrameId & first, const RawKeyFrameId & second)
{
  return second < first ? std::make_pair(second, first) : std::make_pair(first, second);
}

std::pair<RawSubmapId, RawSubmapId> CanonicalSubmaps(
  const RawSubmapId & first, const RawSubmapId & second)
{
  return second < first ? std::make_pair(second, first) : std::make_pair(first, second);
}

bool SameSubmap(const RawKeyFrameId & first, const RawKeyFrameId & second)
{
  return first.drone_id == second.drone_id && first.map_epoch == second.map_epoch;
}

uint32_t Hamming(
  const std::array<uint8_t, 32> & first,
  const std::array<uint8_t, 32> & second)
{
  uint32_t result = 0;
  for (size_t index = 0; index < first.size(); index += sizeof(uint64_t)) {
    uint64_t first_block = 0;
    uint64_t second_block = 0;
    std::memcpy(&first_block, first.data() + index, sizeof(uint64_t));
    std::memcpy(&second_block, second.data() + index, sizeof(uint64_t));
    result += static_cast<uint32_t>(__builtin_popcountll(first_block ^ second_block));
  }
  return result;
}

bool DescriptorValid(const std::array<uint8_t, 32> & descriptor)
{
  return std::any_of(
    descriptor.begin(), descriptor.end(), [](uint8_t value) {return value != 0U;});
}

struct CloudPoint
{
  RawMapPointId id;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  std::array<uint8_t, 32> descriptor{};
};

struct WorldBounds
{
  Eigen::Vector3d minimum = Eigen::Vector3d::Constant(
    std::numeric_limits<double>::infinity());
  Eigen::Vector3d maximum = Eigen::Vector3d::Constant(
    -std::numeric_limits<double>::infinity());
  bool valid = false;
};

std::vector<CloudPoint> BuildCloud(
  const std::vector<RawKeyFrameId> & keyframe_ids,
  const RawMapDatabase & raw_database)
{
  std::map<RawMapPointId, CloudPoint> unique;
  for (const auto & keyframe_id : keyframe_ids) {
    const auto keyframe = raw_database.GetKeyFrame(keyframe_id);
    if (!keyframe.has_value() || keyframe->is_bad) {
      continue;
    }
    for (const uint64_t local_mp_id : keyframe->mappoint_ids) {
      RawMapPointId id{
        keyframe_id.drone_id, keyframe_id.map_epoch, local_mp_id};
      if (unique.count(id) != 0U) {
        continue;
      }
      const auto raw = raw_database.GetMapPoint(id);
      if (!raw.has_value() || raw->is_bad) {
        continue;
      }
      CloudPoint point;
      point.id = id;
      point.position = Eigen::Vector3d(
        raw->position.x, raw->position.y, raw->position.z);
      point.descriptor = raw->descriptor.data;
      if (point.position.allFinite() && DescriptorValid(point.descriptor)) {
        unique.emplace(id, std::move(point));
      }
    }
  }
  std::vector<CloudPoint> result;
  result.reserve(unique.size());
  for (auto & [id, point] : unique) {
    (void)id;
    result.push_back(std::move(point));
  }
  return result;
}

void LimitCloud(std::vector<CloudPoint> * cloud, size_t limit)
{
  if (cloud == nullptr || limit == 0U || cloud->size() <= limit) {
    return;
  }
  std::vector<CloudPoint> bounded;
  bounded.reserve(limit);
  for (size_t index = 0; index < limit; ++index) {
    const size_t source = index * cloud->size() / limit;
    bounded.push_back(std::move((*cloud)[source]));
  }
  *cloud = std::move(bounded);
}

std::optional<WorldBounds> BuildWorldBounds(
  const RawKeyFrameId & keyframe_id, const RawMapDatabase & raw_database,
  const GlobalPoseStore & pose_store, size_t point_limit)
{
  const auto raw_keyframe = raw_database.GetKeyFrame(keyframe_id);
  const auto world_pose = pose_store.GetPose(keyframe_id);
  if (!raw_keyframe.has_value() || !world_pose.has_value() || !world_pose->active) {
    return std::nullopt;
  }
  Eigen::Isometry3d world_T_keyframe;
  Eigen::Isometry3d local_T_keyframe;
  if (!PoseToIsometry(world_pose->world_pose, &world_T_keyframe) ||
    !PoseToIsometry(raw_keyframe->pose, &local_T_keyframe))
  {
    return std::nullopt;
  }
  auto cloud = BuildCloud({keyframe_id}, raw_database);
  LimitCloud(&cloud, point_limit);
  if (cloud.empty()) {
    return std::nullopt;
  }
  const Eigen::Isometry3d world_T_local = world_T_keyframe * local_T_keyframe.inverse();
  WorldBounds bounds;
  for (const auto & point : cloud) {
    const Eigen::Vector3d world = world_T_local * point.position;
    if (!world.allFinite()) {
      continue;
    }
    bounds.minimum = bounds.minimum.cwiseMin(world);
    bounds.maximum = bounds.maximum.cwiseMax(world);
    bounds.valid = true;
  }
  return bounds.valid ? std::optional<WorldBounds>(bounds) : std::nullopt;
}

double BoundsDistance(const WorldBounds & first, const WorldBounds & second)
{
  double squared = 0.0;
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    double gap = 0.0;
    if (first.maximum[axis] < second.minimum[axis]) {
      gap = second.minimum[axis] - first.maximum[axis];
    } else if (second.maximum[axis] < first.minimum[axis]) {
      gap = first.minimum[axis] - second.maximum[axis];
    }
    squared += gap * gap;
  }
  return std::sqrt(squared);
}

struct Match
{
  size_t query = 0;
  size_t candidate = 0;
  uint32_t distance = 0;
};

std::vector<Match> MatchClouds(
  const std::vector<CloudPoint> & query,
  const std::vector<CloudPoint> & candidate,
  uint32_t max_distance, double ratio)
{
  std::vector<size_t> reverse(candidate.size(), std::numeric_limits<size_t>::max());
  std::vector<uint32_t> reverse_distance(candidate.size(), std::numeric_limits<uint32_t>::max());
  for (size_t c = 0; c < candidate.size(); ++c) {
    for (size_t q = 0; q < query.size(); ++q) {
      const uint32_t distance = Hamming(candidate[c].descriptor, query[q].descriptor);
      if (distance < reverse_distance[c]) {
        reverse_distance[c] = distance;
        reverse[c] = q;
      }
    }
  }

  std::vector<Match> matches;
  std::set<size_t> used_candidates;
  for (size_t q = 0; q < query.size(); ++q) {
    size_t best = std::numeric_limits<size_t>::max();
    uint32_t best_distance = std::numeric_limits<uint32_t>::max();
    uint32_t second_distance = std::numeric_limits<uint32_t>::max();
    for (size_t c = 0; c < candidate.size(); ++c) {
      const uint32_t distance = Hamming(query[q].descriptor, candidate[c].descriptor);
      if (distance < best_distance) {
        second_distance = best_distance;
        best_distance = distance;
        best = c;
      } else if (distance < second_distance) {
        second_distance = distance;
      }
    }
    const bool ratio_ok = second_distance == std::numeric_limits<uint32_t>::max() ||
      static_cast<double>(best_distance) < ratio * static_cast<double>(second_distance);
    if (best != std::numeric_limits<size_t>::max() && best_distance <= max_distance &&
      ratio_ok && reverse[best] == q && used_candidates.insert(best).second)
    {
      matches.push_back({q, best, best_distance});
    }
  }
  return matches;
}

bool EstimateRigid(
  const std::vector<CloudPoint> & query,
  const std::vector<CloudPoint> & candidate,
  const std::vector<Match> & matches,
  const std::vector<size_t> & indices,
  Eigen::Isometry3d * candidate_T_query)
{
  if (candidate_T_query == nullptr || indices.size() < 3) {
    return false;
  }
  Eigen::Vector3d query_center = Eigen::Vector3d::Zero();
  Eigen::Vector3d candidate_center = Eigen::Vector3d::Zero();
  for (const size_t index : indices) {
    query_center += query[matches[index].query].position;
    candidate_center += candidate[matches[index].candidate].position;
  }
  query_center /= static_cast<double>(indices.size());
  candidate_center /= static_cast<double>(indices.size());
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const size_t index : indices) {
    covariance +=
      (candidate[matches[index].candidate].position - candidate_center) *
      (query[matches[index].query].position - query_center).transpose();
  }
  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
    covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
  correction(2, 2) = (svd.matrixU() * svd.matrixV().transpose()).determinant() < 0.0 ?
    -1.0 : 1.0;
  const Eigen::Matrix3d rotation =
    svd.matrixU() * correction * svd.matrixV().transpose();
  if (!rotation.allFinite() || rotation.determinant() < 0.99) {
    return false;
  }
  *candidate_T_query = Eigen::Isometry3d::Identity();
  candidate_T_query->linear() = rotation;
  candidate_T_query->translation() = candidate_center - rotation * query_center;
  return candidate_T_query->matrix().allFinite();
}

double TransformRotationDifference(
  const Eigen::Isometry3d & first, const Eigen::Isometry3d & second)
{
  return RotationErrorRad(first, second);
}

bool Compatible(
  const Eigen::Isometry3d & first, const Eigen::Isometry3d & second,
  double translation, double rotation)
{
  return (first.translation() - second.translation()).norm() <= translation &&
         TransformRotationDifference(first, second) <= rotation;
}

}  // namespace

const char * ToString(LoopTaskDecisionKind decision)
{
  switch (decision) {
    case LoopTaskDecisionKind::Stale: return "stale";
    case LoopTaskDecisionKind::NoBow: return "no_bow";
    case LoopTaskDecisionKind::NoCandidates: return "no_candidates";
    case LoopTaskDecisionKind::GeometryRejected: return "geometry_rejected";
    case LoopTaskDecisionKind::SameSubmapDiagnostic: return "same_submap_diagnostic";
    case LoopTaskDecisionKind::WaitingIndependentSupport: return "waiting_independent_support";
    case LoopTaskDecisionKind::Deferred: return "deferred";
    case LoopTaskDecisionKind::FusionCandidate: return "fusion_candidate";
    case LoopTaskDecisionKind::OptimizationEvidence: return "optimization_evidence";
    case LoopTaskDecisionKind::OptimizationCommitted: return "optimization_committed";
    case LoopTaskDecisionKind::ConstraintActivated: return "constraint_activated";
    case LoopTaskDecisionKind::AnchorProposed: return "anchor_proposed";
    case LoopTaskDecisionKind::Error: return "error";
  }
  return "unknown";
}

bool LoopPipeline::PairAttemptKey::operator<(const PairAttemptKey & other) const
{
  return std::tie(first, second, first_revision, second_revision) <
         std::tie(other.first, other.second, other.first_revision, other.second_revision);
}

void LoopPipeline::Configure(const LoopPipelineConfig & config)
{
  config_ = config;
}

size_t LoopPipeline::IndexedKeyFrames() const
{
  return bow_entries_.size();
}

void LoopPipeline::UpsertBow(
  const RawKeyFrameId & id, uint64_t revision,
  const orbslam3_msgs::msg::OrbKeyFrame & keyframe)
{
  const auto previous = bow_entries_.find(id);
  if (previous != bow_entries_.end()) {
    for (const auto & [word, weight] : previous->second.words) {
      (void)weight;
      auto posting = inverted_index_.find(word);
      if (posting != inverted_index_.end()) {
        posting->second.erase(id);
        if (posting->second.empty()) {
          inverted_index_.erase(posting);
        }
      }
    }
  }
  BowEntry entry;
  entry.revision = revision;
  const size_t count = std::min(keyframe.bow_word_ids.size(), keyframe.bow_word_values.size());
  for (size_t index = 0; index < count; ++index) {
    const double weight = keyframe.bow_word_values[index];
    if (std::isfinite(weight) && weight > 0.0) {
      entry.words[keyframe.bow_word_ids[index]] += weight;
    }
  }
  double norm_squared = 0.0;
  for (const auto & [word, weight] : entry.words) {
    inverted_index_[word][id] = weight;
    norm_squared += weight * weight;
  }
  entry.norm = std::sqrt(norm_squared);
  bow_entries_[id] = std::move(entry);
}

std::vector<std::pair<RawKeyFrameId, double>> LoopPipeline::SearchBow(
  const RawKeyFrameId & query_id) const
{
  const auto query = bow_entries_.find(query_id);
  if (query == bow_entries_.end() || query->second.norm <= 0.0) {
    return {};
  }
  std::map<RawKeyFrameId, double> dot_products;
  for (const auto & [word, query_weight] : query->second.words) {
    const auto posting = inverted_index_.find(word);
    if (posting == inverted_index_.end()) {
      continue;
    }
    for (const auto & [id, weight] : posting->second) {
      if (!(id == query_id)) {
        dot_products[id] += query_weight * weight;
      }
    }
  }
  std::vector<std::pair<RawKeyFrameId, double>> result;
  for (const auto & [id, dot] : dot_products) {
    const auto candidate = bow_entries_.find(id);
    if (candidate == bow_entries_.end() || candidate->second.norm <= 0.0) {
      continue;
    }
    const double score = dot / (query->second.norm * candidate->second.norm);
    if (score >= config_.min_bow_score) {
      result.emplace_back(id, score);
    }
  }
  std::sort(
    result.begin(), result.end(),
    [](const auto & lhs, const auto & rhs) {
      return lhs.second == rhs.second ? lhs.first < rhs.first : lhs.second > rhs.second;
    });
  if (result.size() > config_.max_bow_candidates) {
    result.resize(config_.max_bow_candidates);
  }
  return result;
}

bool LoopPipeline::SpatiallyCompatibleForRefresh(
  const RawKeyFrameId & query_id, const RawKeyFrameId & candidate_id,
  const RawMapDatabase & raw_database, const GlobalPoseStore & pose_store) const
{
  const auto query_bounds = BuildWorldBounds(
    query_id, raw_database, pose_store, config_.max_subcloud_points);
  const auto candidate_bounds = BuildWorldBounds(
    candidate_id, raw_database, pose_store, config_.max_subcloud_points);
  return query_bounds.has_value() && candidate_bounds.has_value() &&
         BoundsDistance(*query_bounds, *candidate_bounds) <=
         config_.fusion_refresh_spatial_margin_m;
}

std::vector<LoopCandidateRegion> LoopPipeline::GroupRegions(
  const LoopTask & task,
  const std::vector<std::pair<RawKeyFrameId, double>> & candidates,
  const GlobalPoseStore & pose_store,
  const CovisibilityDatabase & covisibility_database) const
{
  std::vector<LoopCandidateRegion> regions;
  std::set<RawKeyFrameId> assigned;
  for (const auto & [seed, score] : candidates) {
    if (assigned.count(seed) != 0U) {
      continue;
    }
    LoopCandidateRegion region;
    region.query_keyframe_id = task.query_keyframe_id;
    region.seed_keyframe_id = seed;
    region.bow_score = score;
    region.rank = regions.size();
    region.query_has_world_pose = pose_store.GetPose(task.query_keyframe_id).has_value();
    region.candidate_has_world_pose = pose_store.GetPose(seed).has_value();
    for (const auto & [candidate, ignored_score] : candidates) {
      (void)ignored_score;
      if (assigned.count(candidate) != 0U || !SameSubmap(seed, candidate)) {
        continue;
      }
      const uint64_t gap = seed.local_kf_id > candidate.local_kf_id ?
        seed.local_kf_id - candidate.local_kf_id : candidate.local_kf_id - seed.local_kf_id;
      const auto edge = covisibility_database.GetEdge(seed, candidate);
      if (candidate == seed || gap <= config_.temporal_window_radius ||
        (edge.has_value() && edge->support >= config_.strong_covisibility_support))
      {
        region.member_keyframe_ids.push_back(candidate);
        assigned.insert(candidate);
      }
    }
    regions.push_back(std::move(region));
    if (regions.size() >= config_.max_candidate_regions) {
      break;
    }
  }
  return regions;
}

LoopGeometryResult LoopPipeline::VerifyRegion(
  const LoopTask & task, const LoopCandidateRegion & region,
  const RawMapDatabase & raw_database,
  const GlobalPoseStore & pose_store,
  const CovisibilityDatabase & covisibility_database) const
{
  LoopGeometryResult result;
  result.query_keyframe_id = task.query_keyframe_id;
  result.candidate_keyframe_id = region.seed_keyframe_id;
  result.query_submap_id = SubmapOf(task.query_keyframe_id);
  result.candidate_submap_id = SubmapOf(region.seed_keyframe_id);
  auto query_cloud = BuildCloud({task.query_keyframe_id}, raw_database);
  LimitCloud(&query_cloud, config_.max_subcloud_points);
  result.query_points = query_cloud.size();
  result.query_cloud_ids.reserve(query_cloud.size());
  for (const auto & point : query_cloud) {
    result.query_cloud_ids.push_back(point.id);
  }

  std::set<RawKeyFrameId> candidate_ids(
    region.member_keyframe_ids.begin(), region.member_keyframe_ids.end());
  candidate_ids.insert(region.seed_keyframe_id);
  for (const auto & edge : covisibility_database.GetNeighbors(
      region.seed_keyframe_id, config_.strong_covisibility_support,
      config_.max_candidate_window_keyframes))
  {
    candidate_ids.insert(edge.kf_a == region.seed_keyframe_id ? edge.kf_b : edge.kf_a);
  }
  const auto active = raw_database.GetActiveSubmapEntityIds(result.candidate_submap_id);
  if (active.has_value()) {
    for (const auto & id : active->keyframe_ids) {
      const uint64_t gap = id.local_kf_id > region.seed_keyframe_id.local_kf_id ?
        id.local_kf_id - region.seed_keyframe_id.local_kf_id :
        region.seed_keyframe_id.local_kf_id - id.local_kf_id;
      if (gap <= config_.temporal_window_radius) {
        candidate_ids.insert(id);
      }
    }
  }
  std::vector<RawKeyFrameId> bounded(candidate_ids.begin(), candidate_ids.end());
  if (bounded.size() > config_.max_candidate_window_keyframes) {
    bounded.resize(config_.max_candidate_window_keyframes);
  }
  auto candidate_cloud = BuildCloud(bounded, raw_database);
  LimitCloud(&candidate_cloud, config_.max_subcloud_points);
  result.candidate_points = candidate_cloud.size();
  result.candidate_cloud_ids.reserve(candidate_cloud.size());
  for (const auto & point : candidate_cloud) {
    result.candidate_cloud_ids.push_back(point.id);
  }
  if (query_cloud.size() < config_.min_query_mappoints ||
    candidate_cloud.size() < config_.min_ransac_matches)
  {
    result.reason = "insufficient_subcloud_points";
    return result;
  }

  const auto matches = MatchClouds(
    query_cloud, candidate_cloud, config_.max_hamming_distance,
    config_.descriptor_ratio);
  result.matches = matches.size();
  if (matches.size() < config_.min_ransac_matches) {
    result.reason = "insufficient_descriptor_matches";
    return result;
  }

  const uint64_t seed = task.query_keyframe_id.local_kf_id * 0x9e3779b97f4a7c15ULL ^
    region.seed_keyframe_id.local_kf_id ^ task.revision.geometry_revision;
  std::mt19937_64 random(seed);
  std::vector<size_t> best_inliers;
  Eigen::Isometry3d best_transform = Eigen::Isometry3d::Identity();
  const size_t iterations = std::min(
    config_.max_ransac_iterations,
    std::max<size_t>(1, matches.size() * 8));
  for (size_t iteration = 0; iteration < iterations; ++iteration) {
    std::set<size_t> sample_set;
    while (sample_set.size() < 3U) {
      sample_set.insert(static_cast<size_t>(random() % matches.size()));
    }
    std::vector<size_t> sample(sample_set.begin(), sample_set.end());
    Eigen::Isometry3d estimate;
    if (!EstimateRigid(query_cloud, candidate_cloud, matches, sample, &estimate)) {
      continue;
    }
    std::vector<size_t> inliers;
    for (size_t index = 0; index < matches.size(); ++index) {
      const double residual =
        (estimate * query_cloud[matches[index].query].position -
        candidate_cloud[matches[index].candidate].position).norm();
      if (residual <= config_.ransac_inlier_threshold_m) {
        inliers.push_back(index);
      }
    }
    if (inliers.size() > best_inliers.size()) {
      best_inliers = std::move(inliers);
      best_transform = estimate;
    }
  }
  if (best_inliers.size() < config_.min_ransac_inliers ||
    !EstimateRigid(query_cloud, candidate_cloud, matches, best_inliers, &best_transform))
  {
    result.reason = "ransac_no_consensus";
    return result;
  }

  const std::set<size_t> inlier_indices(best_inliers.begin(), best_inliers.end());
  double residual_sum = 0.0;
  result.match_evidence.reserve(matches.size());
  for (size_t index = 0; index < matches.size(); ++index) {
    const double residual =
      (best_transform * query_cloud[matches[index].query].position -
      candidate_cloud[matches[index].candidate].position).norm();
    const bool inlier = inlier_indices.count(index) != 0U;
    result.match_evidence.push_back(
      {
        query_cloud[matches[index].query].id,
        candidate_cloud[matches[index].candidate].id,
        matches[index].distance, residual, inlier,
        !inlier && residual >= 2.0 * config_.ransac_inlier_threshold_m});
    if (inlier) {
      residual_sum += residual;
      result.max_residual_m = std::max(result.max_residual_m, residual);
      result.inlier_pairs.emplace_back(
        query_cloud[matches[index].query].id,
        candidate_cloud[matches[index].candidate].id);
    }
  }
  result.inliers = best_inliers.size();
  result.inlier_ratio = static_cast<double>(result.inliers) /
    static_cast<double>(matches.size());
  result.mean_residual_m = residual_sum / static_cast<double>(result.inliers);
  result.candidate_local_T_query_local = best_transform;
  result.accepted = result.inlier_ratio >= config_.min_inlier_ratio &&
    result.mean_residual_m <= config_.max_mean_residual_m &&
    result.max_residual_m <= config_.max_residual_m;
  if (!result.accepted) {
    result.reason = "geometry_threshold_rejected";
    return result;
  }

  const auto query_pose = pose_store.GetPose(result.query_keyframe_id);
  const auto candidate_pose = pose_store.GetPose(result.candidate_keyframe_id);
  const auto query_raw = raw_database.GetKeyFrame(result.query_keyframe_id);
  const auto candidate_raw = raw_database.GetKeyFrame(result.candidate_keyframe_id);
  if (query_pose.has_value() && query_pose->active &&
    candidate_pose.has_value() && candidate_pose->active &&
    query_raw.has_value() && candidate_raw.has_value())
  {
    Eigen::Isometry3d world_T_query_kf;
    Eigen::Isometry3d world_T_candidate_kf;
    Eigen::Isometry3d query_local_T_kf;
    Eigen::Isometry3d candidate_local_T_kf;
    PoseToIsometry(query_pose->world_pose, &world_T_query_kf);
    PoseToIsometry(candidate_pose->world_pose, &world_T_candidate_kf);
    PoseToIsometry(query_raw->pose, &query_local_T_kf);
    PoseToIsometry(candidate_raw->pose, &candidate_local_T_kf);
    const Eigen::Isometry3d current =
      world_T_candidate_kf.inverse() * world_T_query_kf;
    const Eigen::Isometry3d measured =
      candidate_local_T_kf.inverse() * best_transform * query_local_T_kf;
    result.current_translation_error_m =
      (current.translation() - measured.translation()).norm();
    result.current_rotation_error_rad = RotationErrorRad(current, measured);
    result.fusion_compatible =
      result.current_translation_error_m <= config_.fusion_translation_threshold_m &&
      result.current_rotation_error_rad <= config_.fusion_rotation_threshold_rad;
  }
  result.reason = "geometry_confirmed";
  return result;
}

bool LoopPipeline::AddHypothesisEvidence(
  const SubmapPair & pair, const Eigen::Isometry3d & first_T_second,
  const HypothesisObservation & observation,
  size_t risk_signals, const LoopPipelineConfig & config, Hypothesis * accepted,
  LoopHypothesisSupportSummary * summary,
  const std::optional<size_t> & required_support_override)
{
  LoopHypothesisSupportSummary local_summary;
  local_summary.observed = true;
  local_summary.ambiguity_margin = config.ambiguity_margin;
  auto & candidates = hypotheses_[pair];
  auto found = std::find_if(
    candidates.begin(), candidates.end(),
    [&](const Hypothesis & hypothesis) {
      return Compatible(
        hypothesis.first_local_T_second_local, first_T_second,
        config.hypothesis_translation_tolerance_m,
        config.hypothesis_rotation_tolerance_rad);
    });
  if (found == candidates.end()) {
    candidates.push_back({first_T_second, {observation}});
    found = std::prev(candidates.end());
    local_summary.independent = true;
  } else {
    local_summary.compatible_hypothesis = true;
    bool independent = true;
    double nearest_translation = std::numeric_limits<double>::infinity();
    double nearest_yaw = std::numeric_limits<double>::infinity();
    for (const auto & previous : found->observations) {
      if (previous.query_keyframe_id == observation.query_keyframe_id) {
        independent = false;
        nearest_translation = 0.0;
        nearest_yaw = 0.0;
        break;
      }
      if (previous.candidate_keyframe_id == observation.candidate_keyframe_id) {
        independent = false;
        nearest_translation = 0.0;
        nearest_yaw = 0.0;
        break;
      }
      if (SameSubmap(previous.query_keyframe_id, observation.query_keyframe_id)) {
        Eigen::Isometry3d first_pose;
        Eigen::Isometry3d second_pose;
        if (!PoseToIsometry(previous.query_local_pose, &first_pose) ||
          !PoseToIsometry(observation.query_local_pose, &second_pose))
        {
          independent = false;
          break;
        }
        const double translation =
          (first_pose.translation() - second_pose.translation()).norm();
        const double yaw = std::abs(NormalizeAngle(
          YawFromRotation(first_pose.linear()) -
          YawFromRotation(second_pose.linear())));
        nearest_translation = std::min(nearest_translation, translation);
        nearest_yaw = std::min(nearest_yaw, yaw);
        if (translation < config.independent_translation_m &&
          yaw < config.independent_yaw_rad)
        {
          independent = false;
          break;
        }
      }
      if (SameSubmap(
          previous.candidate_keyframe_id, observation.candidate_keyframe_id))
      {
        Eigen::Isometry3d first_pose;
        Eigen::Isometry3d second_pose;
        if (!PoseToIsometry(previous.candidate_local_pose, &first_pose) ||
          !PoseToIsometry(observation.candidate_local_pose, &second_pose))
        {
          independent = false;
          break;
        }
        const double translation =
          (first_pose.translation() - second_pose.translation()).norm();
        const double yaw = std::abs(NormalizeAngle(
          YawFromRotation(first_pose.linear()) -
          YawFromRotation(second_pose.linear())));
        nearest_translation = std::min(nearest_translation, translation);
        nearest_yaw = std::min(nearest_yaw, yaw);
        if (translation < config.independent_translation_m &&
          yaw < config.independent_yaw_rad)
        {
          independent = false;
          break;
        }
      }
    }
    local_summary.nearest_translation_separation_m =
      std::isfinite(nearest_translation) ? nearest_translation : 0.0;
    local_summary.nearest_yaw_separation_rad =
      std::isfinite(nearest_yaw) ? nearest_yaw : 0.0;
    local_summary.independent = independent;
    if (independent) {
      found->observations.push_back(observation);
    }
  }

  size_t second_support = 0;
  for (const auto & hypothesis : candidates) {
    if (&hypothesis != &*found) {
      second_support = std::max(second_support, hypothesis.observations.size());
    }
  }
  local_summary.support = found->observations.size();
  local_summary.competing_support = second_support;
  if (second_support != 0U) {
    ++risk_signals;
  }
  local_summary.required_support = required_support_override.value_or(
    risk_signals == 0U ? config.hypothesis_support_no_risk :
    (risk_signals == 1U ? config.hypothesis_support_one_risk :
    config.hypothesis_support_multiple_risks));
  local_summary.ambiguity_satisfied = second_support == 0U ||
    found->observations.size() >= second_support + config.ambiguity_margin;
  const bool enough = found->observations.size() >= local_summary.required_support &&
    local_summary.ambiguity_satisfied;
  local_summary.accepted = enough;
  if (enough && accepted != nullptr) {
    *accepted = *found;
  }
  if (summary != nullptr) {
    *summary = local_summary;
  }
  return enough;
}

std::optional<LoopAnchorBatchEntry> LoopPipeline::BuildSingleRecoveryAnchor(
  const LoopGeometryResult & geometry, const RawKeyFrameId & candidate_control,
  const RawMapDatabase & raw_database, const GlobalPoseStore & pose_store) const
{
  const auto parent_anchor = pose_store.GetSubmapAnchorPose(geometry.candidate_submap_id);
  const auto snapshot = raw_database.GetSubmapPoseSnapshot(geometry.query_submap_id);
  Eigen::Isometry3d world_T_parent_local;
  if (!parent_anchor.has_value() || !snapshot.has_value() ||
    !PoseToIsometry(*parent_anchor, &world_T_parent_local))
  {
    return std::nullopt;
  }
  LoopAnchorBatchEntry entry;
  entry.snapshot = *snapshot;
  entry.world_T_local = IsometryToPose(
    world_T_parent_local * geometry.candidate_local_T_query_local);
  entry.loop_control_keyframe_id = geometry.query_keyframe_id;
  entry.parent_submap_id = geometry.candidate_submap_id;
  entry.parent_control_keyframe_id = candidate_control;
  return entry;
}

std::vector<RawKeyFrameId> LoopPipeline::ConfirmedConstraintComponentKeyFrames(
  const RawSubmapId & seed_submap) const
{
  std::map<RawSubmapId, std::vector<const Constraint *>> adjacency;
  for (const auto & [pair, constraint] : active_constraints_) {
    (void)pair;
    if (constraint.provisional) {
      continue;
    }
    adjacency[constraint.first].push_back(&constraint);
    adjacency[constraint.second].push_back(&constraint);
  }
  if (adjacency.count(seed_submap) == 0U) {
    return {};
  }
  std::set<RawSubmapId> visited{seed_submap};
  std::queue<RawSubmapId> pending;
  pending.push(seed_submap);
  std::set<RawKeyFrameId> keyframes;
  while (!pending.empty()) {
    const RawSubmapId current = pending.front();
    pending.pop();
    for (const auto * constraint : adjacency[current]) {
      keyframes.insert(constraint->first_control);
      keyframes.insert(constraint->second_control);
      const RawSubmapId next = current == constraint->first ?
        constraint->second : constraint->first;
      if (visited.insert(next).second) {
        pending.push(next);
      }
    }
  }
  return {keyframes.begin(), keyframes.end()};
}

std::vector<LoopAnchorBatchEntry> LoopPipeline::BuildAnchorCascade(
  uint64_t task_id, const RawSubmapId & seed_submap,
  const RawMapDatabase & raw_database, const GlobalPoseStore & pose_store) const
{
  (void)task_id;
  std::map<RawSubmapId, std::vector<const Constraint *>> adjacency;
  for (const auto & [pair, constraint] : active_constraints_) {
    (void)pair;
    if (constraint.provisional) {
      continue;
    }
    adjacency[constraint.first].push_back(&constraint);
    adjacency[constraint.second].push_back(&constraint);
  }
  if (adjacency.count(seed_submap) == 0U) {
    return {};
  }

  std::set<RawSubmapId> component{seed_submap};
  std::queue<RawSubmapId> component_pending;
  component_pending.push(seed_submap);
  while (!component_pending.empty()) {
    const RawSubmapId current = component_pending.front();
    component_pending.pop();
    for (const auto * constraint : adjacency[current]) {
      const RawSubmapId next = current == constraint->first ?
        constraint->second : constraint->first;
      if (component.insert(next).second) {
        component_pending.push(next);
      }
    }
  }

  std::map<RawSubmapId, Eigen::Isometry3d> world_anchors;
  std::map<RawSubmapId, RawSubmapId> traversal_parent;
  std::map<RawSubmapId, const Constraint *> traversal_constraint;
  std::queue<RawSubmapId> pending;
  for (const auto & [submap, edges] : adjacency) {
    (void)edges;
    if (component.count(submap) == 0U) {
      continue;
    }
    const auto pose = pose_store.GetSubmapAnchorPose(submap);
    Eigen::Isometry3d transform;
    if (pose.has_value() && PoseToIsometry(*pose, &transform)) {
      world_anchors.emplace(submap, transform);
      pending.push(submap);
    }
  }
  while (!pending.empty()) {
    const RawSubmapId current = pending.front();
    pending.pop();
    for (const auto * constraint : adjacency[current]) {
      const bool from_first = current == constraint->first;
      const RawSubmapId next = from_first ? constraint->second : constraint->first;
      const Eigen::Isometry3d world_T_next = from_first ?
        world_anchors.at(current) * constraint->first_local_T_second_local :
        world_anchors.at(current) * constraint->first_local_T_second_local.inverse();
      const auto existing = world_anchors.find(next);
      if (existing == world_anchors.end()) {
        world_anchors.emplace(next, world_T_next);
        traversal_parent[next] = current;
        traversal_constraint[next] = constraint;
        pending.push(next);
      } else if (!Compatible(
          existing->second, world_T_next,
          config_.hypothesis_translation_tolerance_m,
          config_.hypothesis_rotation_tolerance_rad))
      {
        return {};
      }
    }
  }

  std::vector<LoopAnchorBatchEntry> entries;
  for (const auto & [submap, world_T_local] : world_anchors) {
    if (pose_store.HasSubmapAnchor(submap)) {
      continue;
    }
    const auto snapshot = raw_database.GetSubmapPoseSnapshot(submap);
    if (!snapshot.has_value()) {
      return {};
    }
    RawKeyFrameId control;
    bool found_control = false;
    for (const auto * constraint : adjacency[submap]) {
      control = submap == constraint->first ?
        constraint->first_control : constraint->second_control;
      found_control = true;
      break;
    }
    if (!found_control) {
      return {};
    }
    LoopAnchorBatchEntry entry;
    entry.snapshot = *snapshot;
    entry.world_T_local = IsometryToPose(world_T_local);
    entry.loop_control_keyframe_id = control;
    const auto parent = traversal_parent.find(submap);
    const auto parent_edge = traversal_constraint.find(submap);
    if (parent != traversal_parent.end() && parent_edge != traversal_constraint.end()) {
      entry.parent_submap_id = parent->second;
      const auto * edge = parent_edge->second;
      entry.parent_control_keyframe_id = parent->second == edge->first ?
        edge->first_control : edge->second_control;
    }
    entries.push_back(std::move(entry));
  }
  return entries;
}

LoopTaskComputation LoopPipeline::Process(
  const LoopTask & task, const RawMapDatabase & raw_database,
  const GlobalPoseStore & pose_store,
  const CovisibilityDatabase & covisibility_database,
  const std::optional<RecentLossRecoveryContext> & recent_loss)
{
  LoopTaskComputation result;
  result.task = task;
  const auto query_revision = raw_database.GetLoopSemanticRevision(
    task.query_keyframe_id, config_.strong_covisibility_support,
    config_.min_query_mappoints);
  const auto query = raw_database.GetKeyFrame(task.query_keyframe_id);
  if (!query_revision.has_value() || !query.has_value() || query->is_bad ||
    query_revision->appearance_revision != task.revision.appearance_revision ||
    query_revision->validation_revision != task.revision.validation_revision)
  {
    result.decision = LoopTaskDecisionKind::Stale;
    result.reason = "query_revision_changed_or_inactive";
    return result;
  }
  UpsertBow(task.query_keyframe_id, task.revision.appearance_revision, *query);
  std::optional<std::pair<SubmapPair, Hypothesis>> supported_hypothesis;
  size_t supported_hypothesis_size = 0U;
  auto has_independent_support = [&](const LoopGeometryResult & geometry) {
      const auto pair = CanonicalSubmaps(
        geometry.query_submap_id, geometry.candidate_submap_id);
      Eigen::Isometry3d first_T_second =
        pair.first == geometry.candidate_submap_id ?
        geometry.candidate_local_T_query_local :
        geometry.candidate_local_T_query_local.inverse();
      HypothesisObservation observation;
      observation.query_keyframe_id = task.query_keyframe_id;
      observation.query_local_pose = query->pose;
      observation.candidate_keyframe_id = geometry.candidate_keyframe_id;
      const auto candidate = raw_database.GetKeyFrame(geometry.candidate_keyframe_id);
      if (!candidate.has_value()) {
        return false;
      }
      observation.candidate_local_pose = candidate->pose;
      observation.child_control_keyframe_id = task.query_keyframe_id;
      size_t risk_signals = 0U;
      if (pose_store.HasSubmapAnchor(geometry.query_submap_id) !=
        pose_store.HasSubmapAnchor(geometry.candidate_submap_id))
      {
        ++risk_signals;
      }
      if (geometry.current_translation_error_m >
          config_.hypothesis_large_correction_translation_m ||
        geometry.current_rotation_error_rad >
          config_.hypothesis_large_correction_rotation_rad)
      {
        ++risk_signals;
      }
      Hypothesis accepted;
      LoopHypothesisSupportSummary support;
      const bool enough = AddHypothesisEvidence(
        pair, first_T_second, observation, risk_signals, config_, &accepted, &support);
      if (!result.hypothesis_support.observed || support.accepted ||
        support.support > result.hypothesis_support.support)
      {
        result.hypothesis_support = support;
      }
      if (enough && accepted.observations.size() >= supported_hypothesis_size) {
        supported_hypothesis = std::make_pair(pair, accepted);
        supported_hypothesis_size = accepted.observations.size();
      }
      return enough;
    };
  auto select_optimization_regions = [&]() {
      result.optimization_geometry_indices.clear();
      if (!supported_hypothesis.has_value()) {
        return;
      }
      std::vector<size_t> candidates;
      for (size_t index = 0; index < result.geometry_results.size(); ++index) {
        const auto & geometry = result.geometry_results[index];
        if (!geometry.accepted || geometry.fusion_compatible ||
          CanonicalSubmaps(
            geometry.query_submap_id, geometry.candidate_submap_id) !=
          supported_hypothesis->first)
        {
          continue;
        }
        const Eigen::Isometry3d first_T_second =
          supported_hypothesis->first.first == geometry.candidate_submap_id ?
          geometry.candidate_local_T_query_local :
          geometry.candidate_local_T_query_local.inverse();
        if (!Compatible(
            supported_hypothesis->second.first_local_T_second_local,
            first_T_second, config_.hypothesis_translation_tolerance_m,
            config_.hypothesis_rotation_tolerance_rad))
        {
          continue;
        }
        candidates.push_back(index);
      }
      std::sort(
        candidates.begin(), candidates.end(),
        [&result](size_t lhs, size_t rhs) {
          const auto & first = result.geometry_results[lhs];
          const auto & second = result.geometry_results[rhs];
          return first.inliers != second.inliers ? first.inliers > second.inliers :
                 first.mean_residual_m != second.mean_residual_m ?
                 first.mean_residual_m < second.mean_residual_m :
                 first.candidate_keyframe_id < second.candidate_keyframe_id;
        });
      std::set<RawKeyFrameId> candidates_used;
      for (const size_t index : candidates) {
        if (!candidates_used.insert(
            result.geometry_results[index].candidate_keyframe_id).second)
        {
          continue;
        }
        result.optimization_geometry_indices.push_back(index);
        if (result.optimization_geometry_indices.size() >=
          std::min<size_t>(3U, config_.max_candidate_regions))
        {
          break;
        }
      }
    };
  const auto query_world_pose = pose_store.GetPose(task.query_keyframe_id);
  if (query_world_pose.has_value() && task.intent != LoopTaskIntent::FusionRefresh) {
    Eigen::Isometry3d world_T_query_kf;
    PoseToIsometry(query_world_pose->world_pose, &world_T_query_kf);
    std::vector<std::pair<RawKeyFrameId, double>> nearby;
    for (const auto & [candidate_id, entry] : bow_entries_) {
      (void)entry;
      if (candidate_id == task.query_keyframe_id ||
        SameSubmap(candidate_id, task.query_keyframe_id) ||
        covisibility_database.HasSource(
          task.query_keyframe_id, candidate_id,
          CovisibilityEdgeSource::ServerLoopGeometric))
      {
        continue;
      }
      const auto candidate_pose = pose_store.GetPose(candidate_id);
      Eigen::Isometry3d world_T_candidate_kf;
      if (!candidate_pose.has_value() ||
        !PoseToIsometry(candidate_pose->world_pose, &world_T_candidate_kf))
      {
        continue;
      }
      const double distance =
        (world_T_query_kf.translation() - world_T_candidate_kf.translation()).norm();
      if (distance <= 4.0) {
        nearby.emplace_back(candidate_id, distance);
      }
    }
    std::sort(
      nearby.begin(), nearby.end(),
      [](const auto & lhs, const auto & rhs) {
        return lhs.second == rhs.second ? lhs.first < rhs.first : lhs.second < rhs.second;
      });
    if (nearby.size() > config_.max_candidate_regions) {
      nearby.resize(config_.max_candidate_regions);
    }
    result.used_fast_overlap = !nearby.empty();
    bool fast_high_error = false;
    bool fast_high_error_supported = false;
    for (size_t index = 0; index < nearby.size(); ++index) {
      LoopCandidateRegion region;
      region.query_keyframe_id = task.query_keyframe_id;
      region.seed_keyframe_id = nearby[index].first;
      region.member_keyframe_ids = {nearby[index].first};
      region.rank = index;
      region.query_has_world_pose = true;
      region.candidate_has_world_pose = true;
      auto geometry = VerifyRegion(
        task, region, raw_database, pose_store, covisibility_database);
      if (geometry.accepted && geometry.fusion_compatible) {
        result.regions.push_back(region);
        result.fusion_pairs.insert(
          result.fusion_pairs.end(), geometry.inlier_pairs.begin(),
          geometry.inlier_pairs.end());
        result.geometry_results.push_back(std::move(geometry));
      } else if (geometry.accepted) {
        fast_high_error = true;
        fast_high_error_supported =
          has_independent_support(geometry) || fast_high_error_supported;
        result.regions.push_back(region);
        result.geometry_results.push_back(std::move(geometry));
      }
    }
    if (fast_high_error) {
      select_optimization_regions();
      const bool supported = fast_high_error_supported &&
        !result.optimization_geometry_indices.empty();
      result.decision = supported ?
        LoopTaskDecisionKind::OptimizationEvidence :
        LoopTaskDecisionKind::WaitingIndependentSupport;
      result.reason = supported ?
        "fast_global_overlap_high_error_dominates" :
        "fast_global_overlap_high_error_waiting_independent_support";
      return result;
    }
    if (!result.fusion_pairs.empty()) {
      std::sort(result.fusion_pairs.begin(), result.fusion_pairs.end());
      result.fusion_pairs.erase(
        std::unique(result.fusion_pairs.begin(), result.fusion_pairs.end()),
        result.fusion_pairs.end());
      result.decision = LoopTaskDecisionKind::FusionCandidate;
      result.reason = "fast_global_overlap_all_regions_fusion_compatible";
      return result;
    }
    result.regions.clear();
    result.geometry_results.clear();
  }
  if (query->bow_word_ids.empty() || query->bow_word_values.empty()) {
    result.decision = LoopTaskDecisionKind::NoBow;
    result.reason = "query_bow_empty";
    return result;
  }

  auto candidates = SearchBow(task.query_keyframe_id);
  candidates.erase(
    std::remove_if(
      candidates.begin(), candidates.end(),
      [&](const auto & item) {
        const auto raw = raw_database.GetKeyFrame(item.first);
        const auto revision = raw_database.GetLoopSemanticRevision(
          item.first, config_.strong_covisibility_support,
          config_.min_query_mappoints);
        const auto native_edge = covisibility_database.GetEdge(
          task.query_keyframe_id, item.first);
        if (!raw.has_value() || !revision.has_value() || raw->is_bad ||
          raw->mappoint_ids.size() < config_.min_query_mappoints ||
          (SameSubmap(task.query_keyframe_id, item.first) &&
          native_edge.has_value() &&
          native_edge->source == CovisibilityEdgeSource::Orbslam3Native &&
          native_edge->support >= config_.strong_covisibility_support) ||
          covisibility_database.HasSource(
            task.query_keyframe_id, item.first,
            CovisibilityEdgeSource::ServerLoopGeometric))
        {
          return true;
        }
        if (task.intent == LoopTaskIntent::FusionRefresh) {
          if (!SpatiallyCompatibleForRefresh(
              task.query_keyframe_id, item.first, raw_database, pose_store))
          {
            ++result.refresh_spatial_rejected;
            return true;
          }
          ++result.refresh_spatial_candidates;
        }
        const auto canonical = CanonicalKeyFrames(task.query_keyframe_id, item.first);
        const uint64_t first_revision = canonical.first == task.query_keyframe_id ?
          query_revision->geometry_revision : revision->geometry_revision;
        const uint64_t second_revision = canonical.second == item.first ?
          revision->geometry_revision : query_revision->geometry_revision;
        return geometry_rejections_.count(
          {canonical.first, canonical.second, first_revision, second_revision}) != 0U;
      }),
    candidates.end());
  result.bow_candidates = candidates.size();
  if (candidates.empty()) {
    result.decision = LoopTaskDecisionKind::NoCandidates;
    result.reason = task.intent == LoopTaskIntent::FusionRefresh ?
      "fusion_refresh_no_spatial_candidates" : "no_bow_candidates_after_filters";
    return result;
  }

  result.regions = GroupRegions(
    task, candidates, pose_store, covisibility_database);
  bool accepted_cross_submap_geometry = false;
  bool same_submap_diagnostic = false;
  bool anchored_high_error_geometry = false;
  bool optimization_evidence = false;
  bool constraint_activated = false;
  bool provisional_promoted = false;
  std::optional<RawSubmapId> activated_submap;
  for (const auto & region : result.regions) {
    auto geometry = VerifyRegion(
      task, region, raw_database, pose_store, covisibility_database);
    if (!geometry.accepted) {
      const auto candidate_revision = raw_database.GetLoopSemanticRevision(
        region.seed_keyframe_id, config_.strong_covisibility_support,
        config_.min_query_mappoints);
      if (candidate_revision.has_value()) {
        const auto canonical = CanonicalKeyFrames(task.query_keyframe_id, region.seed_keyframe_id);
        geometry_rejections_.insert(
          {canonical.first, canonical.second,
            canonical.first == task.query_keyframe_id ?
            query_revision->geometry_revision : candidate_revision->geometry_revision,
            canonical.second == region.seed_keyframe_id ?
            candidate_revision->geometry_revision : query_revision->geometry_revision});
      }
      result.geometry_results.push_back(std::move(geometry));
      continue;
    }
    if (geometry.query_submap_id == geometry.candidate_submap_id) {
      same_submap_diagnostic = true;
      const auto same_query_pose = pose_store.GetPose(geometry.query_keyframe_id);
      const auto same_candidate_pose = pose_store.GetPose(geometry.candidate_keyframe_id);
      if (!same_query_pose.has_value() || !same_query_pose->active ||
        !same_candidate_pose.has_value() || !same_candidate_pose->active)
      {
        geometry.reason = "same_submap_unanchored_diagnostic";
        result.geometry_results.push_back(std::move(geometry));
        continue;
      }
      if (geometry.fusion_compatible) {
        result.fusion_pairs.insert(
          result.fusion_pairs.end(), geometry.inlier_pairs.begin(),
          geometry.inlier_pairs.end());
      } else {
        anchored_high_error_geometry = true;
        optimization_evidence =
          has_independent_support(geometry) || optimization_evidence;
      }
      geometry.reason = geometry.fusion_compatible ?
        "same_submap_fusion_geometry" : "same_submap_loop_geometry";
      result.geometry_results.push_back(std::move(geometry));
      continue;
    }
    accepted_cross_submap_geometry = true;
    const auto pair = CanonicalSubmaps(
      geometry.query_submap_id, geometry.candidate_submap_id);
    const auto query_anchor = pose_store.GetSubmapAnchorPose(geometry.query_submap_id);
    const auto candidate_anchor = pose_store.GetSubmapAnchorPose(geometry.candidate_submap_id);
    if (query_anchor.has_value() && candidate_anchor.has_value()) {
      const auto active = active_constraints_.find(pair);
      if (active != active_constraints_.end() && active->second.provisional) {
        const auto candidate = raw_database.GetKeyFrame(region.seed_keyframe_id);
        if (!candidate.has_value()) {
          result.geometry_results.push_back(std::move(geometry));
          continue;
        }
        Eigen::Isometry3d first_T_second =
          pair.first == geometry.candidate_submap_id ?
          geometry.candidate_local_T_query_local :
          geometry.candidate_local_T_query_local.inverse();
        HypothesisObservation confirmation;
        confirmation.query_keyframe_id = task.query_keyframe_id;
        confirmation.query_local_pose = query->pose;
        confirmation.candidate_keyframe_id = region.seed_keyframe_id;
        confirmation.candidate_local_pose = candidate->pose;
        confirmation.child_control_keyframe_id = task.query_keyframe_id;
        const size_t risk_signals =
          geometry.current_translation_error_m >
          config_.hypothesis_large_correction_translation_m ||
          geometry.current_rotation_error_rad >
          config_.hypothesis_large_correction_rotation_rad ? 1U : 0U;
        Hypothesis confirmed;
        LoopHypothesisSupportSummary support;
        if (!AddHypothesisEvidence(
            pair, first_T_second, confirmation, risk_signals, config_,
            &confirmed, &support))
        {
          result.hypothesis_support = support;
          geometry.reason = "provisional_constraint_waiting_confirmation";
          result.geometry_results.push_back(std::move(geometry));
          continue;
        }
        active->second.provisional = false;
        active->second.support = confirmed.observations.size();
        result.hypothesis_support = support;
        constraint_activated = true;
        provisional_promoted = true;
        activated_submap = geometry.query_submap_id;
      }
      if (geometry.fusion_compatible) {
        result.fusion_pairs.insert(
          result.fusion_pairs.end(), geometry.inlier_pairs.begin(), geometry.inlier_pairs.end());
      } else {
        anchored_high_error_geometry = true;
        optimization_evidence =
          has_independent_support(geometry) || optimization_evidence;
      }
      result.geometry_results.push_back(std::move(geometry));
      continue;
    }

    Eigen::Isometry3d first_T_second;
    if (pair.first == geometry.candidate_submap_id) {
      first_T_second = geometry.candidate_local_T_query_local;
    } else {
      first_T_second = geometry.candidate_local_T_query_local.inverse();
    }
    HypothesisObservation observation;
    observation.query_keyframe_id = task.query_keyframe_id;
    observation.query_local_pose = query->pose;
    observation.candidate_keyframe_id = region.seed_keyframe_id;
    const auto candidate = raw_database.GetKeyFrame(region.seed_keyframe_id);
    if (!candidate.has_value()) {
      result.geometry_results.push_back(std::move(geometry));
      continue;
    }
    observation.candidate_local_pose = candidate->pose;
    observation.child_control_keyframe_id =
      !query_anchor.has_value() ? task.query_keyframe_id : region.seed_keyframe_id;
    const bool large_correction = geometry.current_translation_error_m >
        config_.hypothesis_large_correction_translation_m ||
      geometry.current_rotation_error_rad >
        config_.hypothesis_large_correction_rotation_rad;
    size_t risk_signals = query_anchor.has_value() !=
      pose_store.HasSubmapAnchor(geometry.candidate_submap_id) ? 1U : 0U;
    if (large_correction) {
      ++risk_signals;
    }
    bool single_loop_recovery = false;
    if (config_.recent_loss_single_loop_enabled && recent_loss.has_value() &&
      recent_loss->submap_id == geometry.query_submap_id &&
      !query_anchor.has_value() && candidate_anchor.has_value() &&
      result.regions.size() == 1U && !large_correction)
    {
      result.recent_loss_single_loop_checked = true;
      const auto snapshot = raw_database.GetSubmapPoseSnapshot(geometry.query_submap_id);
      if (snapshot.has_value()) {
        const auto query_index = std::find_if(
          snapshot->keyframes.begin(), snapshot->keyframes.end(),
          [&task](const auto & input) {return input.id == task.query_keyframe_id;});
        if (query_index != snapshot->keyframes.end()) {
          double path_m = 0.0;
          for (auto current = snapshot->keyframes.begin() + 1;
            current <= query_index; ++current)
          {
            Eigen::Isometry3d previous_pose;
            Eigen::Isometry3d current_pose;
            if (!PoseToIsometry((current - 1)->local_pose, &previous_pose) ||
              !PoseToIsometry(current->local_pose, &current_pose))
            {
              path_m = std::numeric_limits<double>::infinity();
              break;
            }
            path_m += (current_pose.translation() - previous_pose.translation()).norm();
          }
          result.recent_loss_single_loop_path_m = path_m;
          Eigen::Isometry3d world_T_candidate_local;
          Eigen::Isometry3d query_local_pose;
          if (PoseToIsometry(*candidate_anchor, &world_T_candidate_local) &&
            PoseToIsometry(query->pose, &query_local_pose))
          {
            const auto error = ComputeFiducialError(
              IsometryToPose(
                world_T_candidate_local * geometry.candidate_local_T_query_local *
                query_local_pose),
              recent_loss->trusted_world_pose);
            result.recent_loss_translation_m = error.translation_m;
            result.recent_loss_rotation_rad = error.rotation_rad;
            result.recent_loss_translation_limit_m =
              config_.recent_loss_single_loop_translation_m;
            result.recent_loss_rotation_limit_rad =
              config_.recent_loss_single_loop_rotation_rad;
            single_loop_recovery =
              path_m <= config_.recent_loss_single_loop_max_path_m &&
              error.translation_m <= config_.recent_loss_single_loop_translation_m &&
              error.rotation_rad <= config_.recent_loss_single_loop_rotation_rad;
            result.recent_loss_single_loop_eligible = single_loop_recovery;
          }
        }
      }
    }
    Hypothesis accepted;
    LoopHypothesisSupportSummary support;
    if (AddHypothesisEvidence(
        pair, first_T_second, observation, risk_signals, config_, &accepted, &support,
        single_loop_recovery ? std::optional<size_t>(1U) : std::nullopt))
    {
      Constraint constraint;
      constraint.first = pair.first;
      constraint.second = pair.second;
      constraint.first_local_T_second_local = accepted.first_local_T_second_local;
      constraint.support = accepted.observations.size();
      constraint.first_control = pair.first == geometry.query_submap_id ?
        task.query_keyframe_id : region.seed_keyframe_id;
      constraint.second_control = pair.second == geometry.query_submap_id ?
        task.query_keyframe_id : region.seed_keyframe_id;
      constraint.provisional = single_loop_recovery && accepted.observations.size() == 1U;
      active_constraints_[pair] = constraint;
      constraint_activated = true;
      activated_submap = geometry.query_submap_id;
      if (constraint.provisional) {
        const auto entry = BuildSingleRecoveryAnchor(
          geometry, region.seed_keyframe_id, raw_database, pose_store);
        if (entry.has_value()) {
          result.anchor_entries = {*entry};
          result.recent_loss_single_loop_used = true;
        }
      }
    }
    if (!result.hypothesis_support.observed || support.accepted ||
      support.support > result.hypothesis_support.support)
    {
      result.hypothesis_support = support;
    }
    result.geometry_results.push_back(std::move(geometry));
  }

  if (optimization_evidence) {
    select_optimization_regions();
    if (!result.optimization_geometry_indices.empty()) {
      result.decision = LoopTaskDecisionKind::OptimizationEvidence;
      result.reason = "anchored_high_error_regions_dominate";
      return result;
    }
  }
  if (anchored_high_error_geometry) {
    result.decision = LoopTaskDecisionKind::WaitingIndependentSupport;
    result.reason = "anchored_high_error_waiting_independent_support";
    return result;
  }
  if (constraint_activated) {
    if (result.anchor_entries.empty() && activated_submap.has_value()) {
      result.anchor_entries = BuildAnchorCascade(
        task.task_id, *activated_submap, raw_database, pose_store);
    }
    if (!result.anchor_entries.empty()) {
      result.decision = LoopTaskDecisionKind::AnchorProposed;
      result.reason = result.recent_loss_single_loop_used ?
        "recent_loss_single_loop_anchor_proposed" :
        "coherent_component_has_world_authority";
      return result;
    }
    if (!provisional_promoted) {
      result.decision = LoopTaskDecisionKind::ConstraintActivated;
      result.reason = "unanchored_constraint_activated";
      return result;
    }
  }
  if (!result.fusion_pairs.empty()) {
    std::sort(result.fusion_pairs.begin(), result.fusion_pairs.end());
    result.fusion_pairs.erase(
      std::unique(result.fusion_pairs.begin(), result.fusion_pairs.end()),
    result.fusion_pairs.end());
    result.decision = LoopTaskDecisionKind::FusionCandidate;
    result.reason = "all_anchored_regions_fusion_compatible";
    return result;
  }
  if (accepted_cross_submap_geometry) {
    result.decision = LoopTaskDecisionKind::WaitingIndependentSupport;
    result.reason = "waiting_adaptive_independent_support";
  } else if (same_submap_diagnostic) {
    result.decision = LoopTaskDecisionKind::SameSubmapDiagnostic;
    result.reason = "same_submap_geometry_diagnostic_only";
  } else {
    result.decision = LoopTaskDecisionKind::GeometryRejected;
    result.reason = "all_regions_rejected";
  }
  return result;
}

}  // namespace orbslam3_multi
