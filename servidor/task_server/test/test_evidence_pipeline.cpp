#include "task_server/evidence_pipeline.hpp"

#include <gtest/gtest.h>

namespace
{

mission_msgs::msg::KeyframeSparseEvidence MakeEvidence(
  std::uint64_t geometry_revision, std::uint64_t pose_revision, double pose_x)
{
  mission_msgs::msg::KeyframeSparseEvidence evidence;
  evidence.drone_id = 1U;
  evidence.map_epoch = 2U;
  evidence.keyframe_id = 3U;
  evidence.geometry_revision = geometry_revision;
  evidence.pose_revision = pose_revision;
  evidence.w_t_keyframe.position.x = pose_x;
  evidence.w_t_keyframe.orientation.w = 1.0;
  for (int x = 0; x < 5; ++x) {
    for (int y = 0; y < 4; ++y) {
      mission_msgs::msg::KeyframeSparsePoint point;
      point.position_k.x = 2.0F;
      point.position_k.y = static_cast<float>(x) * 0.1F;
      point.position_k.z = static_cast<float>(y) * 0.1F;
      point.score = 0.8F;
      evidence.points_k.push_back(point);
    }
  }
  return evidence;
}

task_server::KeyframeEvidenceWorker MakeWorker()
{
  task_server::KeyframeEvidenceParameters parameters;
  parameters.plane_min_inliers = 20U;
  parameters.plane_residual_m = 0.125;
  parameters.voxel_size_m = 0.25;
  return task_server::KeyframeEvidenceWorker(parameters);
}

mission_msgs::msg::DenseKFObservation MakeDepthObservation(std::uint64_t frame_id)
{
  mission_msgs::msg::DenseKFObservation observation;
  observation.valid = true;
  observation.drone_id = 7U;
  observation.map_epoch = 11U;
  observation.local_keyframe_id = 13U;
  observation.source_revision = 17U;
  observation.tracking_frame_id = frame_id;
  observation.k_t_camera.orientation.w = 1.0;
  observation.min_depth_m = 0.5F;
  observation.max_depth_m = 8.0F;
  observation.confidence = 0.9F;
  observation.normal_valid = true;
  observation.normal_k.x = 1.0;
  for (int index = 0; index < 20; ++index) {
    geometry_msgs::msg::Point32 point;
    point.x = 2.0F;
    point.y = static_cast<float>(index % 5) * 0.02F;
    point.z = static_cast<float>(index / 5) * 0.02F;
    observation.points_k.push_back(point);
  }
  return observation;
}

task_server::DepthIntegrationWorker MakeDepthWorker()
{
  task_server::DepthIntegrationParameters parameters;
  parameters.voxel_size_m = 0.25;
  parameters.far_free_distance_m = 5.0;
  parameters.direct_surface_max_incidence_deg = 35.0;
  parameters.min_support_points = 20U;
  parameters.min_confidence = 0.25F;
  return task_server::DepthIntegrationWorker(parameters);
}

geometry_msgs::msg::Pose IdentityPose()
{
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  return pose;
}

TEST(EvidencePipeline, StoresAtomicSourcesAndMaterializesFreeRays)
{
  task_server::EvidenceDatabase database;
  auto worker = MakeWorker();
  mission_msgs::msg::KeyframeSparseEvidenceDelta delta;
  delta.upserts.push_back(MakeEvidence(1U, 1U, 0.0));

  const auto processed = worker.Consume(delta, &database);
  EXPECT_EQ(processed.accepted_keyframes, 1U);
  EXPECT_EQ(database.SourceCount(), 2U);

  task_lib::ReversibleVoxelMap map(0.25);
  task_server::VoxelMapBuilder builder(0.25);
  const auto materialized = builder.Apply(&database, &map);
  EXPECT_EQ(materialized.transactions, 1U);
  EXPECT_EQ(materialized.upserts, 2U);
  EXPECT_TRUE(materialized.changed);
  EXPECT_EQ(map.StateAt({1, 0, 0}), task_lib::VoxelState::Free);
}

TEST(EvidencePipeline, ReprojectsOnlyTheChangedKeyframeSources)
{
  task_server::EvidenceDatabase database;
  auto worker = MakeWorker();
  task_lib::ReversibleVoxelMap map(0.25);
  task_server::VoxelMapBuilder builder(0.25);
  mission_msgs::msg::KeyframeSparseEvidenceDelta first;
  first.upserts.push_back(MakeEvidence(1U, 1U, 0.0));
  worker.Consume(first, &database);
  builder.Apply(&database, &map);
  ASSERT_EQ(map.StateAt({1, 0, 0}), task_lib::VoxelState::Free);

  mission_msgs::msg::KeyframeSparseEvidenceDelta moved;
  moved.upserts.push_back(MakeEvidence(1U, 2U, 1.0));
  worker.Consume(moved, &database);
  const auto materialized = builder.Apply(&database, &map);
  EXPECT_EQ(materialized.transactions, 1U);
  EXPECT_TRUE(materialized.changed);
  EXPECT_EQ(map.StateAt({1, 0, 0}), task_lib::VoxelState::Unknown);
  EXPECT_EQ(map.StateAt({5, 0, 0}), task_lib::VoxelState::Free);
}

TEST(EvidencePipeline, TombstoneRemovesAllSourcesOfTheKeyframe)
{
  task_server::EvidenceDatabase database;
  auto worker = MakeWorker();
  task_lib::ReversibleVoxelMap map(0.25);
  task_server::VoxelMapBuilder builder(0.25);
  mission_msgs::msg::KeyframeSparseEvidenceDelta added;
  added.upserts.push_back(MakeEvidence(1U, 1U, 0.0));
  worker.Consume(added, &database);
  builder.Apply(&database, &map);

  mission_msgs::msg::KeyframeSparseEvidenceDelta deleted;
  deleted.deletes.push_back(MakeEvidence(2U, 2U, 0.0));
  const auto processed = worker.Consume(deleted, &database);
  EXPECT_EQ(processed.tombstones, 1U);
  const auto materialized = builder.Apply(&database, &map);
  EXPECT_EQ(materialized.deletes, 2U);
  EXPECT_EQ(materialized.removed_source_ids.size(), 2U);
  EXPECT_TRUE(materialized.changed);
  EXPECT_EQ(database.SourceCount(), 0U);
  EXPECT_EQ(map.StateAt({1, 0, 0}), task_lib::VoxelState::Unknown);
}

TEST(EvidencePipeline, WallDepthWaitsForPoseThenMaterializesFreeAndOccupied)
{
  task_server::EvidenceDatabase database;
  auto worker = MakeDepthWorker();
  task_server::DepthIntegrationJob job;
  job.command_id = "wall-command";
  job.inspection_kind = task_server::DepthInspectionKind::VIEW_WALL;
  job.observations.push_back(MakeDepthObservation(19U));

  const auto processed = worker.Consume(job, &database);
  ASSERT_TRUE(processed.accepted);
  ASSERT_EQ(processed.sources.size(), 3U);
  task_lib::ReversibleVoxelMap map(0.25);
  task_server::VoxelMapBuilder builder(0.25);
  EXPECT_EQ(builder.Apply(&database, &map).transactions, 0U);

  const task_server::KeyframeEvidenceIdentity identity{7U, 11U, 13U};
  ASSERT_TRUE(database.UpdateKeyframePose(identity, 23U, IdentityPose()));
  const auto materialized = builder.Apply(&database, &map);
  EXPECT_EQ(materialized.transactions, 1U);
  EXPECT_EQ(materialized.applied_source_ids.size(), 3U);
  ASSERT_EQ(materialized.occupied_cells_by_source.size(), 1U);
  EXPECT_FALSE(materialized.occupied_cells_by_source.begin()->second.empty());
  EXPECT_EQ(map.StateAt({4, 0, 0}), task_lib::VoxelState::Free);
  EXPECT_EQ(map.StateAt({8, 0, 0}), task_lib::VoxelState::Occupied);
}

TEST(EvidencePipeline, UnknownDepthNeverMaterializesOccupied)
{
  task_server::EvidenceDatabase database;
  auto worker = MakeDepthWorker();
  task_server::DepthIntegrationJob job;
  job.command_id = "unknown-command";
  job.inspection_kind = task_server::DepthInspectionKind::VIEW_UNKNOWN;
  job.observations.push_back(MakeDepthObservation(29U));

  const auto processed = worker.Consume(job, &database);
  ASSERT_TRUE(processed.accepted);
  ASSERT_EQ(processed.sources.size(), 1U);
  const task_server::KeyframeEvidenceIdentity identity{7U, 11U, 13U};
  ASSERT_TRUE(database.UpdateKeyframePose(identity, 23U, IdentityPose()));
  task_lib::ReversibleVoxelMap map(0.25);
  task_server::VoxelMapBuilder builder(0.25);
  const auto materialized = builder.Apply(&database, &map);
  ASSERT_EQ(materialized.applied_source_ids.size(), 1U);
  EXPECT_EQ(map.StateAt({4, 0, 0}), task_lib::VoxelState::Free);
  EXPECT_EQ(map.StateAt({8, 0, 0}), task_lib::VoxelState::Unknown);
}

TEST(EvidencePipeline, ViewAdvanceCanMaterializeOccupiedOnlyWithFrontalNormal)
{
  task_server::EvidenceDatabase database;
  auto worker = MakeDepthWorker();
  task_server::DepthIntegrationJob job;
  job.command_id = "advance-command";
  job.inspection_kind = task_server::DepthInspectionKind::VIEW_ADVANCE;
  job.observations.push_back(MakeDepthObservation(30U));

  const auto processed = worker.Consume(job, &database);
  ASSERT_TRUE(processed.accepted);
  ASSERT_EQ(processed.sources.size(), 3U);
  const task_server::KeyframeEvidenceIdentity identity{7U, 11U, 13U};
  ASSERT_TRUE(database.UpdateKeyframePose(identity, 23U, IdentityPose()));
  task_lib::ReversibleVoxelMap map(0.25);
  task_server::VoxelMapBuilder builder(0.25);
  builder.Apply(&database, &map);
  EXPECT_EQ(map.StateAt({4, 0, 0}), task_lib::VoxelState::Free);
  EXPECT_EQ(map.StateAt({8, 0, 0}), task_lib::VoxelState::Occupied);
}

TEST(EvidencePipeline, ReusesAnAlreadyMaterializedDepthSourceForNewCorrelation)
{
  task_server::EvidenceDatabase database;
  auto worker = MakeDepthWorker();
  const task_server::KeyframeEvidenceIdentity identity{7U, 11U, 13U};
  ASSERT_TRUE(database.UpdateKeyframePose(identity, 23U, IdentityPose()) == false);

  task_server::DepthIntegrationJob first;
  first.command_id = "first";
  first.observations.push_back(MakeDepthObservation(31U));
  ASSERT_TRUE(worker.Consume(first, &database).accepted);
  ASSERT_TRUE(database.UpdateKeyframePose(identity, 23U, IdentityPose()));
  task_lib::ReversibleVoxelMap map(0.25);
  task_server::VoxelMapBuilder builder(0.25);
  builder.Apply(&database, &map);

  task_server::DepthIntegrationJob repeated = first;
  repeated.command_id = "second";
  ASSERT_TRUE(worker.Consume(repeated, &database).accepted);
  const auto materialized = builder.Apply(&database, &map);
  EXPECT_EQ(materialized.transactions, 1U);
  EXPECT_EQ(materialized.applied_source_ids.size(), 1U);
}

}  // namespace
