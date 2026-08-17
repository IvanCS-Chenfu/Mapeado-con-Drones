#include "orbslam3_multi/fused_landmark_manager.hpp"

#include <gtest/gtest.h>

#include <memory>

namespace
{

geometry_msgs::msg::Pose Pose(double x)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.orientation.w = 1.0;
  return pose;
}

std::shared_ptr<orbslam3_msgs::msg::OrbMap> MakeMap(
  uint32_t drone_id, double point_x)
{
  auto map = std::make_shared<orbslam3_msgs::msg::OrbMap>();
  map->drone_id = drone_id;
  map->map_epoch = 0;
  map->fx = 100.0F;
  map->fy = 100.0F;
  map->cx = 320.0F;
  map->cy = 240.0F;
  map->image_width = 640;
  map->image_height = 480;
  auto & keyframe = map->keyframes.emplace_back();
  keyframe.id = 1;
  keyframe.pose = Pose(0.0);
  keyframe.mappoint_ids = {1};
  auto & point = map->mappoints.emplace_back();
  point.id = 1;
  point.position.x = point_x;
  point.position.z = 2.0;
  point.reference_keyframe_id = 1;
  point.observations_count = 4;
  point.found_ratio = 1.0F;
  point.descriptor.data[0] = 0x5a;
  point.observations.emplace_back().keyframe_id = 1;
  return map;
}

orbslam3_multi::LoopTaskComputation FusionBetween(uint32_t query, uint32_t candidate)
{
  orbslam3_multi::LoopTaskComputation computation;
  computation.decision = orbslam3_multi::LoopTaskDecisionKind::FusionCandidate;
  computation.task.task_id = 100 + query * 10 + candidate;
  computation.task.revision.validation_revision = 7;
  orbslam3_multi::LoopGeometryResult geometry;
  geometry.query_keyframe_id = {query, 0, 1};
  geometry.candidate_keyframe_id = {candidate, 0, 1};
  geometry.query_submap_id = {query, 0};
  geometry.candidate_submap_id = {candidate, 0};
  geometry.accepted = true;
  geometry.fusion_compatible = true;
  geometry.inliers = 1;
  geometry.inlier_pairs.push_back({{query, 0, 1}, {candidate, 0, 1}});
  computation.geometry_results.push_back(std::move(geometry));
  return computation;
}

orbslam3_multi::LoopTaskComputation FusionPairs(
  const std::vector<std::pair<uint32_t, uint32_t>> & pairs,
  uint64_t validation_revision)
{
  orbslam3_multi::LoopTaskComputation computation;
  computation.decision = orbslam3_multi::LoopTaskDecisionKind::FusionCandidate;
  computation.task.task_id = 900 + validation_revision;
  computation.task.revision.validation_revision = validation_revision;
  orbslam3_multi::LoopGeometryResult geometry;
  geometry.query_keyframe_id = {pairs.front().first, 0, 1};
  geometry.candidate_keyframe_id = {pairs.back().second, 0, 1};
  geometry.query_submap_id = {pairs.front().first, 0};
  geometry.candidate_submap_id = {pairs.back().second, 0};
  geometry.accepted = true;
  geometry.fusion_compatible = true;
  geometry.inliers = pairs.size();
  for (const auto & pair : pairs) {
    geometry.inlier_pairs.push_back({{pair.first, 0, 1}, {pair.second, 0, 1}});
  }
  computation.geometry_results.push_back(std::move(geometry));
  return computation;
}

TEST(FusedLandmarkManager, TransitiveUnionIsStableAndDispersionIsGuarded)
{
  orbslam3_multi::RawMapDatabase raw;
  orbslam3_multi::GlobalPoseStore poses;
  orbslam3_multi::LandmarkScoreManager scores;
  for (const auto & input : {
      std::make_pair(1U, 0.00), std::make_pair(2U, 0.02),
      std::make_pair(3U, -0.02), std::make_pair(4U, 2.00)})
  {
    const auto inserted = raw.InsertDelta(input.first, MakeMap(input.first, input.second));
    scores.ApplyRawChanges(inserted, raw);
    const auto snapshot = raw.GetSubmapPoseSnapshot({input.first, 0});
    ASSERT_TRUE(snapshot.has_value());
    ASSERT_EQ(
      poses.CommitAnchor(*snapshot, Pose(0.0), input.first).status,
      orbslam3_multi::PoseCommitStatus::Applied);
  }

  orbslam3_multi::FusedLandmarkManager manager;
  auto first = manager.PrepareFusion(FusionBetween(1, 2), raw, poses, scores);
  ASSERT_TRUE(first.ready);
  ASSERT_TRUE(manager.ApplyPatch(first.patch).committed);
  const auto stable_id = manager.GetTrackIdForMember({1, 0, 1});
  ASSERT_TRUE(stable_id.has_value());

  auto transitive = manager.PrepareFusion(FusionBetween(2, 3), raw, poses, scores);
  ASSERT_TRUE(transitive.ready);
  ASSERT_TRUE(manager.ApplyPatch(transitive.patch).committed);
  EXPECT_EQ(manager.GetTrackIdForMember({2, 0, 1}), stable_id);
  EXPECT_EQ(manager.GetTrackIdForMember({3, 0, 1}), stable_id);
  ASSERT_TRUE(manager.GetTrack(*stable_id).has_value());
  EXPECT_EQ(manager.GetTrack(*stable_id)->member_mappoint_ids.size(), 3U);

  auto dispersed = manager.PrepareFusion(FusionBetween(1, 4), raw, poses, scores);
  EXPECT_FALSE(dispersed.ready);
  ASSERT_FALSE(dispersed.patch.pair_results.empty());
  EXPECT_EQ(dispersed.patch.pair_results.front().reason, "track_dispersion_guard");
  EXPECT_FALSE(manager.GetTrackIdForMember({4, 0, 1}).has_value());
  EXPECT_EQ(manager.GetStats().tracks, 1U);
}

TEST(FusedLandmarkManager, RetiredTouchedTrackIsRemovedFromPreparedPatch)
{
  orbslam3_multi::RawMapDatabase raw;
  orbslam3_multi::GlobalPoseStore poses;
  orbslam3_multi::LandmarkScoreManager scores;
  for (uint32_t drone_id = 1; drone_id <= 4; ++drone_id) {
    const auto inserted = raw.InsertDelta(drone_id, MakeMap(drone_id, 0.01 * drone_id));
    scores.ApplyRawChanges(inserted, raw);
    const auto snapshot = raw.GetSubmapPoseSnapshot({drone_id, 0});
    ASSERT_TRUE(snapshot.has_value());
    ASSERT_EQ(
      poses.CommitAnchor(*snapshot, Pose(0.0), drone_id).status,
      orbslam3_multi::PoseCommitStatus::Applied);
  }

  orbslam3_multi::FusedLandmarkManager manager;
  auto first_track = manager.PrepareFusion(FusionBetween(3, 4), raw, poses, scores);
  ASSERT_TRUE(first_track.ready);
  ASSERT_TRUE(manager.ApplyPatch(first_track.patch).committed);
  const auto stable_id = manager.GetTrackIdForMember({3, 0, 1});
  ASSERT_TRUE(stable_id.has_value());

  auto second_track = manager.PrepareFusion(FusionBetween(1, 2), raw, poses, scores);
  ASSERT_TRUE(second_track.ready);
  ASSERT_TRUE(manager.ApplyPatch(second_track.patch).committed);
  ASSERT_NE(manager.GetTrackIdForMember({1, 0, 1}), stable_id);

  // The first pair touches track 2; the second pair retires it into stable track 1.
  auto merge = manager.PrepareFusion(FusionPairs({{1, 2}, {1, 3}}, 8), raw, poses, scores);
  ASSERT_TRUE(merge.ready);
  ASSERT_TRUE(manager.ApplyPatch(merge.patch).committed);
  EXPECT_EQ(manager.GetTrackIdForMember({1, 0, 1}), stable_id);
  EXPECT_EQ(manager.GetTrackIdForMember({2, 0, 1}), stable_id);
  EXPECT_EQ(manager.GetTrackIdForMember({4, 0, 1}), stable_id);
  ASSERT_TRUE(manager.GetTrack(*stable_id).has_value());
  EXPECT_EQ(manager.GetTrack(*stable_id)->member_mappoint_ids.size(), 4U);
  EXPECT_EQ(manager.GetStats().tracks, 1U);
}

TEST(FusedLandmarkManager, EvaluatesEveryEligibleVisibilityContradiction)
{
  orbslam3_multi::RawMapDatabase raw;
  orbslam3_multi::GlobalPoseStore poses;
  orbslam3_multi::LandmarkScoreManager scores;
  for (const auto & input : {
      std::make_pair(1U, 0.00), std::make_pair(2U, 0.02)})
  {
    const auto inserted = raw.InsertDelta(input.first, MakeMap(input.first, input.second));
    scores.ApplyRawChanges(inserted, raw);
    const auto snapshot = raw.GetSubmapPoseSnapshot({input.first, 0});
    ASSERT_TRUE(snapshot.has_value());
    ASSERT_EQ(
      poses.CommitAnchor(*snapshot, Pose(0.0), input.first).status,
      orbslam3_multi::PoseCommitStatus::Applied);
  }

  auto computation = FusionBetween(1, 2);
  auto & geometry = computation.geometry_results.front();
  geometry.query_cloud_ids = {{1, 0, 1}};
  geometry.candidate_cloud_ids = {{2, 0, 1}};
  geometry.candidate_local_T_query_local.translation().x() = 1.0;
  constexpr size_t kContradictions = 64;
  for (size_t index = 0; index < kContradictions; ++index) {
    orbslam3_multi::LoopGeometryResult::MatchEvidence evidence;
    evidence.query_mappoint_id = {1, 0, 1};
    evidence.candidate_mappoint_id = {2, 0, 1};
    evidence.hard_outlier = true;
    geometry.match_evidence.push_back(evidence);
  }

  orbslam3_multi::FusedLandmarkManager manager;
  const auto prepared = manager.PrepareFusion(computation, raw, poses, scores);
  ASSERT_TRUE(prepared.ready);
  EXPECT_EQ(prepared.patch.visibility_regions_started, 1U);
  EXPECT_EQ(prepared.patch.visibility_regions_completed, 1U);
  EXPECT_EQ(prepared.patch.visibility_projected_points, kContradictions * 2U);
  EXPECT_EQ(prepared.patch.negative_score_events, kContradictions * 2U);
  EXPECT_EQ(
    prepared.patch.raw_score_evidence.size(),
    prepared.patch.positive_score_events + prepared.patch.negative_score_events);
}

}  // namespace
