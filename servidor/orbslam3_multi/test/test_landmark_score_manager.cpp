#include "orbslam3_multi/landmark_score_manager.hpp"

#include <gtest/gtest.h>

#include <memory>

namespace
{

std::shared_ptr<orbslam3_msgs::msg::OrbMap> MakeMap(
  float found_ratio, bool is_bad = false, uint32_t observations_count = 4)
{
  auto map = std::make_shared<orbslam3_msgs::msg::OrbMap>();
  map->drone_id = 1;
  map->map_epoch = 4;
  auto & point = map->mappoints.emplace_back();
  point.id = 10;
  point.observations_count = observations_count;
  point.found_ratio = found_ratio;
  point.is_bad = is_bad;
  point.descriptor.data[0] = 1;
  return map;
}

TEST(LandmarkScoreManager, ComputesDeterministicBoundedOrbScore)
{
  orbslam3_multi::RawMapDatabase raw;
  orbslam3_multi::LandmarkScoreManager scores;
  const auto changes = scores.ApplyRawChanges(raw.InsertDelta(1, MakeMap(0.5F)), raw);

  ASSERT_EQ(changes.created_ids.size(), 1U);
  const auto record = scores.GetScore({1, 4, 10});
  ASSERT_TRUE(record.has_value());
  EXPECT_NEAR(record->score, 0.55F * 0.5F + 0.35F * 0.5F + 0.10F, 1e-6F);
  EXPECT_EQ(changes.score_revision_after, 1U);

  const auto no_op = scores.ApplyRawChanges(raw.InsertDelta(2, MakeMap(0.5F)), raw);
  EXPECT_FALSE(no_op.HasChanges());
  EXPECT_EQ(no_op.score_revision_after, 1U);
}

TEST(LandmarkScoreManager, UpdatesOnlyChangedPointAndMarksBadAsZero)
{
  orbslam3_multi::RawMapDatabase raw;
  orbslam3_multi::LandmarkScoreManager scores;
  scores.ApplyRawChanges(raw.InsertDelta(1, MakeMap(0.5F)), raw);

  const auto updated = scores.ApplyRawChanges(raw.InsertDelta(2, MakeMap(0.9F)), raw);
  ASSERT_EQ(updated.updated_ids.size(), 1U);
  EXPECT_NEAR(scores.GetScore({1, 4, 10})->score, 0.69F, 1e-6F);

  const auto bad = scores.ApplyRawChanges(raw.InsertDelta(3, MakeMap(0.9F, true)), raw);
  ASSERT_EQ(bad.updated_ids.size(), 1U);
  EXPECT_FLOAT_EQ(scores.GetScore({1, 4, 10})->score, 0.0F);
  EXPECT_EQ(scores.GetStats().bad_points, 1U);
}

TEST(LandmarkScoreManager, StoresSaturatedInputWithoutDirtyingPublishedScore)
{
  orbslam3_multi::RawMapDatabase raw;
  orbslam3_multi::LandmarkScoreManager scores;
  const auto created = scores.ApplyRawChanges(raw.InsertDelta(1, MakeMap(0.5F, false, 8)), raw);
  ASSERT_TRUE(created.HasChanges());
  EXPECT_EQ(created.score_revision_after, 1U);

  const auto input_only = scores.ApplyRawChanges(
    raw.InsertDelta(2, MakeMap(0.5F, false, 12)), raw);
  EXPECT_FALSE(input_only.HasChanges());
  EXPECT_TRUE(input_only.HasStoreChanges());
  EXPECT_TRUE(input_only.updated_ids.empty());
  ASSERT_EQ(input_only.input_updated_ids.size(), 1U);
  EXPECT_EQ(input_only.score_revision_after, 1U);
  const auto record = scores.GetScore({1, 4, 10});
  ASSERT_TRUE(record.has_value());
  EXPECT_EQ(record->observations_count, 12U);
  EXPECT_EQ(record->record_revision, 2U);
}

TEST(LandmarkScoreManager, EvidenceIsIdempotentAndPatchCanRollback)
{
  orbslam3_multi::RawMapDatabase raw;
  orbslam3_multi::LandmarkScoreManager scores;
  scores.ApplyRawChanges(raw.InsertDelta(1, MakeMap(0.5F)), raw);
  const orbslam3_multi::RawMapPointId id{1, 4, 10};
  const float initial = scores.GetScore(id)->score;

  orbslam3_multi::ScorePatch patch;
  patch.expected_score_revision = scores.GetStats().score_revision;
  patch.raw_evidence.push_back(
    {id, 77, 0.04F, orbslam3_multi::LandmarkScoreEvidenceKind::InlierConfirmed});
  patch.fused_upserts.push_back({5, 0.8F});
  const auto applied = scores.ApplyPatch(patch);
  ASSERT_TRUE(applied.committed);
  EXPECT_NEAR(scores.GetScore(id)->score, initial + 0.04F, 1e-6F);
  ASSERT_TRUE(scores.GetFusedScore(5).has_value());

  patch.expected_score_revision = scores.GetStats().score_revision;
  const auto repeated = scores.ApplyPatch(patch);
  ASSERT_TRUE(repeated.committed);
  EXPECT_FALSE(repeated.changes.HasStoreChanges());
  EXPECT_NEAR(scores.GetScore(id)->score, initial + 0.04F, 1e-6F);

  ASSERT_TRUE(scores.RollbackPatch(applied.rollback));
  EXPECT_NEAR(scores.GetScore(id)->score, initial, 1e-6F);
  EXPECT_FALSE(scores.GetFusedScore(5).has_value());
}

TEST(LandmarkScoreManager, DistancePenaltyRecoversWhenGeometryMoves)
{
  orbslam3_multi::RawMapDatabase raw;
  orbslam3_multi::LandmarkScoreManager scores;
  orbslam3_multi::LandmarkScoreConfig config;
  config.isolation_min_neighbors = 0;
  scores.Configure(config);
  scores.ApplyRawChanges(raw.InsertDelta(1, MakeMap(1.0F)), raw);
  const orbslam3_multi::RawMapPointId id{1, 4, 10};
  const float base = scores.GetScore(id)->base_score_orb;

  orbslam3_multi::LandmarkScoreGeometryInput geometry;
  geometry.mappoint_id = id;
  geometry.world_position.z = 0.5;
  geometry.observer_distance_m = 0.5;
  geometry.stereo_baseline_m = 0.06;
  ASSERT_TRUE(scores.ApplyGeometryChanges({geometry}, {}).HasChanges());
  EXPECT_NEAR(scores.GetScore(id)->score, base * 0.25F, 1e-6F);
  EXPECT_NEAR(scores.GetScore(id)->distance_factor, 0.25F, 1e-6F);

  geometry.world_position.z = 1.0;
  geometry.observer_distance_m = 1.0;
  ASSERT_TRUE(scores.ApplyGeometryChanges({geometry}, {}).HasChanges());
  EXPECT_NEAR(scores.GetScore(id)->score, base, 1e-6F);

  geometry.world_position.z = 5.0;
  geometry.observer_distance_m = 5.0;
  EXPECT_FALSE(scores.ApplyGeometryChanges({geometry}, {}).HasChanges());
  EXPECT_NEAR(scores.GetScore(id)->score, base, 1e-6F);

  geometry.world_position.z = 10.0;
  geometry.observer_distance_m = 10.0;
  ASSERT_TRUE(scores.ApplyGeometryChanges({geometry}, {}).HasChanges());
  EXPECT_NEAR(scores.GetScore(id)->score, base * 0.25F, 1e-6F);

  geometry.world_position.z = 2.0;
  geometry.observer_distance_m = 2.0;
  ASSERT_TRUE(scores.ApplyGeometryChanges({geometry}, {}).HasChanges());
  EXPECT_NEAR(scores.GetScore(id)->score, base, 1e-6F);
}

TEST(LandmarkScoreManager, BaselineMovesOnlyFarLimitAndFallbackIsFiveMeters)
{
  orbslam3_multi::RawMapDatabase raw;
  orbslam3_multi::LandmarkScoreManager scores;
  orbslam3_multi::LandmarkScoreConfig config;
  config.isolation_min_neighbors = 0;
  scores.Configure(config);
  scores.ApplyRawChanges(raw.InsertDelta(1, MakeMap(1.0F)), raw);
  const orbslam3_multi::RawMapPointId id{1, 4, 10};
  const float base = scores.GetScore(id)->base_score_orb;

  orbslam3_multi::LandmarkScoreGeometryInput geometry;
  geometry.mappoint_id = id;
  geometry.world_position.z = 6.0;
  geometry.observer_distance_m = 6.0;
  geometry.stereo_baseline_m = 0.12;
  scores.ApplyGeometryChanges({geometry}, {});
  EXPECT_NEAR(scores.GetScore(id)->score, base, 1e-6F);

  geometry.world_position.z = 0.5;
  geometry.observer_distance_m = 0.5;
  scores.ApplyGeometryChanges({geometry}, {});
  EXPECT_NEAR(scores.GetScore(id)->distance_factor, 0.25F, 1e-6F);

  geometry.world_position.z = 10.0;
  geometry.observer_distance_m = 10.0;
  geometry.stereo_baseline_m = 0.0;
  scores.ApplyGeometryChanges({geometry}, {});
  EXPECT_NEAR(scores.GetScore(id)->distance_factor, 0.25F, 1e-6F);
}

TEST(LandmarkScoreManager, MatureIsolatedPointRecoversWhenNeighborsArrive)
{
  orbslam3_multi::RawMapDatabase raw;
  orbslam3_multi::LandmarkScoreManager scores;
  orbslam3_multi::LandmarkScoreConfig config;
  config.isolation_radius_m = 0.5;
  config.isolation_min_neighbors = 2;
  config.isolation_min_observations = 3;
  config.isolation_min_factor = 0.30F;
  scores.Configure(config);
  scores.ApplyRawChanges(raw.InsertDelta(1, MakeMap(1.0F)), raw);
  const orbslam3_multi::RawMapPointId id{1, 4, 10};
  const float base = scores.GetScore(id)->base_score_orb;

  orbslam3_multi::LandmarkScoreGeometryInput point;
  point.mappoint_id = id;
  point.world_position.z = 2.0;
  point.observer_distance_m = 2.0;
  scores.ApplyGeometryChanges({point}, {});
  EXPECT_NEAR(scores.GetScore(id)->score, base * 0.30F, 1e-6F);

  auto neighbor_a = point;
  neighbor_a.mappoint_id = {2, 0, 1};
  neighbor_a.world_position.x = 0.1;
  auto neighbor_b = point;
  neighbor_b.mappoint_id = {3, 0, 1};
  neighbor_b.world_position.x = -0.1;
  const auto recovered = scores.ApplyGeometryChanges({neighbor_a, neighbor_b}, {});
  EXPECT_TRUE(recovered.HasChanges());
  EXPECT_NEAR(scores.GetScore(id)->score, base, 1e-6F);
  EXPECT_FLOAT_EQ(scores.GetScore(id)->isolation_factor, 1.0F);
}

TEST(LandmarkScoreManager, InlierRewardIsAddedAfterGeometryFactors)
{
  orbslam3_multi::RawMapDatabase raw;
  orbslam3_multi::LandmarkScoreManager scores;
  orbslam3_multi::LandmarkScoreConfig config;
  config.isolation_min_neighbors = 0;
  scores.Configure(config);
  scores.ApplyRawChanges(raw.InsertDelta(1, MakeMap(1.0F)), raw);
  const orbslam3_multi::RawMapPointId id{1, 4, 10};
  orbslam3_multi::LandmarkScoreGeometryInput geometry;
  geometry.mappoint_id = id;
  geometry.world_position.z = 0.05;
  geometry.observer_distance_m = 0.05;
  scores.ApplyGeometryChanges({geometry}, {});
  const float factored = scores.GetScore(id)->score;

  orbslam3_multi::ScorePatch patch;
  patch.expected_score_revision = scores.GetStats().score_revision;
  patch.raw_evidence.push_back(
    {id, 123, 0.04F, orbslam3_multi::LandmarkScoreEvidenceKind::InlierConfirmed});
  ASSERT_TRUE(scores.ApplyPatch(patch).committed);
  EXPECT_NEAR(scores.GetScore(id)->score, factored + 0.04F, 1e-6F);
}

}  // namespace
