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

}  // namespace
