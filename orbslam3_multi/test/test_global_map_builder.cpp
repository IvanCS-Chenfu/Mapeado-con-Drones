#include "orbslam3_multi/sparse_global_backend.hpp"

#include <gtest/gtest.h>

#include <memory>

namespace
{

geometry_msgs::msg::Pose MakePose(double x)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.orientation.w = 1.0;
  return pose;
}

std::shared_ptr<orbslam3_msgs::msg::OrbMap> MakeMap(double keyframe_x, double point_x)
{
  auto map = std::make_shared<orbslam3_msgs::msg::OrbMap>();
  map->drone_id = 1;
  map->map_epoch = 2;
  auto & keyframe = map->keyframes.emplace_back();
  keyframe.id = 7;
  keyframe.pose = MakePose(keyframe_x);
  keyframe.mappoint_ids = {20};
  auto & point = map->mappoints.emplace_back();
  point.id = 20;
  point.position.x = point_x;
  point.reference_keyframe_id = 7;
  point.observations_count = 8;
  point.found_ratio = 1.0F;
  point.descriptor.data[0] = 1;
  auto & observation = point.observations.emplace_back();
  observation.keyframe_id = 7;
  return map;
}

TEST(GlobalMapBuilder, OmitsUnanchoredThenBackfillsOnFirstAnchor)
{
  orbslam3_multi::SparseGlobalBackend backend;
  auto map = MakeMap(1.0, 3.0);
  auto & unlisted_point = map->mappoints.emplace_back(map->mappoints.front());
  unlisted_point.id = 21;
  unlisted_point.position.x = 4.0;
  backend.InsertDelta(1, map);
  const auto before = backend.BuildGlobalMap();
  EXPECT_TRUE(before.points.empty());
  EXPECT_TRUE(before.keyframes.empty());
  EXPECT_EQ(before.deferred_unanchored_submaps, 1U);
  EXPECT_EQ(before.deferred_unanchored_keyframes, 1U);
  EXPECT_EQ(before.deferred_unanchored_mappoints, 2U);

  const auto anchor = backend.CommitAnchor({1, 2}, MakePose(10.0), 2);
  ASSERT_EQ(anchor.status, orbslam3_multi::PoseCommitStatus::Applied);
  const auto after = backend.BuildGlobalMap();
  ASSERT_TRUE(after.changed);
  ASSERT_EQ(after.keyframes.size(), 1U);
  ASSERT_EQ(after.points.size(), 2U);
  EXPECT_EQ(after.backfilled_submaps, 1U);
  EXPECT_EQ(after.backfilled_keyframes, 1U);
  EXPECT_EQ(after.backfilled_mappoints, 2U);
  EXPECT_EQ(after.deferred_unanchored_submaps, 0U);
  EXPECT_NEAR(after.keyframes[0].world_pose.position.x, 11.0, 1e-9);
  EXPECT_NEAR(after.points[0].x, 13.0F, 1e-5F);
  EXPECT_EQ(after.points[0].associated_keyframe_id.local_kf_id, 7U);
  EXPECT_NEAR(after.points[0].score, 0.35F, 1e-6F);
  EXPECT_EQ(after.fallback_submap_points, 0U);

  const auto no_op = backend.BuildGlobalMap();
  EXPECT_FALSE(no_op.changed);
  EXPECT_EQ(no_op.publication_revision, after.publication_revision);
}

TEST(GlobalMapBuilder, MovingKeyFrameReprojectsOnlyAssociatedPoint)
{
  orbslam3_multi::SparseGlobalBackend backend;
  backend.InsertDelta(1, MakeMap(1.0, 3.0));
  backend.CommitAnchor({1, 2}, MakePose(10.0), 2);
  backend.BuildGlobalMap();

  backend.InsertDelta(3, MakeMap(2.0, 3.0));
  const auto moved = backend.BuildGlobalMap();
  ASSERT_TRUE(moved.changed);
  ASSERT_EQ(moved.points.size(), 1U);
  EXPECT_NEAR(moved.points[0].x, 13.0F, 1e-5F);
  EXPECT_EQ(moved.recalculated_mappoints, 1U);
  EXPECT_EQ(moved.fallback_submap_points, 0U);
}

TEST(GlobalMapBuilder, PointWithoutWorldObserverNeverUsesSubmapFallback)
{
  orbslam3_multi::SparseGlobalBackend backend;
  auto map = MakeMap(1.0, 3.0);
  map->mappoints[0].reference_keyframe_id = 99;
  map->mappoints[0].observations[0].keyframe_id = 99;
  backend.InsertDelta(1, map);
  backend.CommitAnchor({1, 2}, MakePose(10.0), 2);
  const auto built = backend.BuildGlobalMap();
  EXPECT_TRUE(built.points.empty());
  EXPECT_GT(built.skipped_without_world_keyframe, 0U);
  EXPECT_EQ(built.fallback_submap_points, 0U);
}

}  // namespace
