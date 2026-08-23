#include "orbslam3_multi/sparse_global_backend.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

namespace
{

using orbslam3_msgs::msg::OrbKeyFrame;
using orbslam3_msgs::msg::OrbMap;
using orbslam3_msgs::msg::OrbMapPoint;
using orbslam3_multi::AcceptedPoseUpdate;
using orbslam3_multi::GlobalPoseStore;
using orbslam3_multi::PoseCommitStatus;
using orbslam3_multi::PoseSourceKind;
using orbslam3_multi::RawKeyFrameId;
using orbslam3_multi::RawKeyFramePoseChange;
using orbslam3_multi::RawKeyFramePoseChangeKind;
using orbslam3_multi::RawKeyFramePoseInput;
using orbslam3_multi::RawSubmapId;
using orbslam3_multi::RawSubmapPoseSnapshot;
using orbslam3_multi::SparseGlobalBackend;

geometry_msgs::msg::Pose MakePose(double x, double y = 0.0, double z = 0.0)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;
  return pose;
}

std::shared_ptr<OrbMap> MakeMap(uint32_t drone_id, uint64_t epoch)
{
  auto map = std::make_shared<OrbMap>();
  map->drone_id = drone_id;
  map->map_epoch = epoch;
  return map;
}

TEST(GlobalPoseStore, StartsEmptyAndRejectsUnanchoredPlaceholders)
{
  GlobalPoseStore store;
  EXPECT_EQ(store.GetStats().poses, 0U);

  RawKeyFramePoseChange change;
  change.kind = RawKeyFramePoseChangeKind::New;
  change.keyframe = {{1, 0, 4}, 1, MakePose(2.0), true};
  const auto result = store.ApplyRawPoseChanges({1, 0}, {change}, 10);

  EXPECT_EQ(result.status, PoseCommitStatus::Unanchored);
  ASSERT_EQ(result.skipped_unanchored_ids.size(), 1U);
  EXPECT_FALSE(store.GetPose({1, 0, 4}).has_value());
  EXPECT_EQ(store.GetStats().store_revision, 0U);
}

TEST(GlobalPoseStore, AnchorsAtomicallyAndTracksDerivedPoseLineage)
{
  GlobalPoseStore store;
  RawSubmapPoseSnapshot snapshot;
  snapshot.submap_id = {1, 0};
  snapshot.submap_revision = 8;
  snapshot.keyframes = {
    {{1, 0, 1}, 3, MakePose(2.0), true},
    {{1, 0, 2}, 1, MakePose(4.0), false}};

  const auto anchored = store.CommitAnchor(snapshot, MakePose(10.0), 20);
  EXPECT_EQ(anchored.status, PoseCommitStatus::Applied);
  EXPECT_EQ(anchored.store_revision_before, 0U);
  EXPECT_EQ(anchored.store_revision_after, 1U);
  ASSERT_EQ(anchored.created_ids.size(), 2U);
  ASSERT_EQ(anchored.invalidated_ids.size(), 1U);

  const auto first = store.GetPose({1, 0, 1});
  ASSERT_TRUE(first.has_value());
  EXPECT_NEAR(first->world_pose.position.x, 12.0, 1e-9);
  EXPECT_EQ(first->pose_revision, 1U);
  EXPECT_EQ(first->base_raw_revision, 3U);
  EXPECT_TRUE(first->active);

  const auto second = store.GetPose({1, 0, 2});
  ASSERT_TRUE(second.has_value());
  EXPECT_FALSE(second->active);
  const auto stats = store.GetStats();
  EXPECT_EQ(stats.anchors, 1U);
  EXPECT_EQ(stats.active_poses, 1U);
  EXPECT_EQ(stats.inactive_poses, 1U);
}

TEST(GlobalPoseStore, UpdatesDerivedPreservesAcceptedAndKeepsInvalidatedHistory)
{
  GlobalPoseStore store;
  RawSubmapPoseSnapshot snapshot;
  snapshot.submap_id = {1, 0};
  snapshot.submap_revision = 1;
  snapshot.keyframes = {{{1, 0, 1}, 1, MakePose(1.0), true}};
  store.CommitAnchor(snapshot, MakePose(10.0), 1);

  RawKeyFramePoseChange moved;
  moved.kind = RawKeyFramePoseChangeKind::PoseUpdated;
  moved.keyframe = {{1, 0, 1}, 2, MakePose(3.0), true};
  const auto moved_result = store.ApplyRawPoseChanges({1, 0}, {moved}, 2);
  EXPECT_EQ(moved_result.status, PoseCommitStatus::Applied);
  EXPECT_NEAR(store.GetPose({1, 0, 1})->world_pose.position.x, 13.0, 1e-9);

  AcceptedPoseUpdate accepted{{1, 0, 1}, MakePose(100.0), 2};
  const auto accepted_result = store.CommitAcceptedPoses(
    {1, 0}, {accepted}, PoseSourceKind::FiducialAccepted, 3);
  EXPECT_EQ(accepted_result.status, PoseCommitStatus::Applied);

  moved.keyframe.raw_revision = 3;
  moved.keyframe.local_pose = MakePose(5.0);
  const auto preserved = store.ApplyRawPoseChanges({1, 0}, {moved}, 4);
  EXPECT_EQ(preserved.status, PoseCommitStatus::Applied);
  ASSERT_EQ(preserved.preserved_ids.size(), 1U);
  const auto rebased = store.GetPose({1, 0, 1});
  ASSERT_TRUE(rebased.has_value());
  EXPECT_NEAR(rebased->world_pose.position.x, 100.0, 1e-9);
  EXPECT_NEAR(rebased->raw_world_pose.position.x, 15.0, 1e-9);
  EXPECT_NEAR(rebased->correction_pose.position.x, 85.0, 1e-9);
  EXPECT_EQ(rebased->base_raw_revision, 3U);

  RawKeyFramePoseChange invalidated;
  invalidated.kind = RawKeyFramePoseChangeKind::Invalidated;
  invalidated.keyframe = {{1, 0, 1}, 4, MakePose(5.0), false};
  const auto invalidated_result = store.ApplyRawPoseChanges({1, 0}, {invalidated}, 5);
  EXPECT_EQ(invalidated_result.status, PoseCommitStatus::Applied);
  const auto retained = store.GetPose({1, 0, 1});
  ASSERT_TRUE(retained.has_value());
  EXPECT_FALSE(retained->active);
  EXPECT_NEAR(retained->world_pose.position.x, 100.0, 1e-9);
  EXPECT_EQ(retained->source_kind, PoseSourceKind::FiducialAccepted);
}

TEST(GlobalPoseStore, CommitsLoopOptimizedSubmapsAsOneRevision)
{
  GlobalPoseStore store;
  for (uint32_t drone = 1; drone <= 2; ++drone) {
    RawSubmapPoseSnapshot snapshot;
    snapshot.submap_id = {drone, 1};
    snapshot.submap_revision = 4;
    snapshot.keyframes = {
      {{drone, 1, 0}, 1, MakePose(0.0), true},
      {{drone, 1, 1}, 1, MakePose(1.0), true}};
    ASSERT_EQ(
      store.CommitAnchor(snapshot, MakePose(0.0, drone), drone, RawKeyFrameId{drone, 1, 0}).status,
      PoseCommitStatus::Applied);
  }
  const auto revision_before = store.GetStats().store_revision;
  std::vector<orbslam3_multi::AcceptedSubmapPoseBatch> batches;
  for (uint32_t drone = 1; drone <= 2; ++drone) {
    const auto first = store.GetPose({drone, 1, 0});
    const auto second = store.GetPose({drone, 1, 1});
    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());
    orbslam3_multi::AcceptedSubmapPoseBatch batch;
    batch.submap_id = {drone, 1};
    batch.continuation_control = RawKeyFrameId{drone, 1, 1};
    batch.updates = {
      {{drone, 1, 0}, first->world_pose, 1, first->pose_revision, false},
      {{drone, 1, 1}, MakePose(1.0, 1.5), 1, second->pose_revision, false}};
    batches.push_back(batch);
  }
  const auto result = store.CommitAcceptedPoseBatch(
    batches, PoseSourceKind::LoopOptimized, 77);
  EXPECT_EQ(result.status, PoseCommitStatus::Applied);
  EXPECT_EQ(result.submap_changes.size(), 2U);
  EXPECT_EQ(result.store_revision_before, revision_before);
  EXPECT_EQ(result.store_revision_after, revision_before + 1U);
  EXPECT_EQ(result.dirty_keyframe_ids.size(), 2U);
  EXPECT_NEAR(store.GetPose({1, 1, 1})->world_pose.position.y, 1.5, 1e-9);
  EXPECT_NEAR(store.GetPose({2, 1, 1})->world_pose.position.y, 1.5, 1e-9);
  EXPECT_TRUE(store.GetPose({1, 1, 0})->hard_fiducial);
  EXPECT_TRUE(store.GetPose({2, 1, 0})->hard_fiducial);

  auto stale_batches = batches;
  stale_batches.front().updates.back().expected_pose_revision = 1U;
  const auto first_before = store.GetPose({1, 1, 1})->world_pose;
  const auto second_before = store.GetPose({2, 1, 1})->world_pose;
  const auto stale = store.CommitAcceptedPoseBatch(
    stale_batches, PoseSourceKind::LoopOptimized, 78);
  EXPECT_EQ(stale.status, PoseCommitStatus::RevisionConflict);
  EXPECT_EQ(store.GetPose({1, 1, 1})->world_pose.position.y, first_before.position.y);
  EXPECT_EQ(store.GetPose({2, 1, 1})->world_pose.position.y, second_before.position.y);
}

TEST(GlobalPoseStore, CommitsFiducialMultiSubmapBatchAndMarksTargetHard)
{
  GlobalPoseStore store;
  for (uint32_t drone = 1; drone <= 2; ++drone) {
    RawSubmapPoseSnapshot snapshot;
    snapshot.submap_id = {drone, 3};
    snapshot.submap_revision = 1U;
    snapshot.keyframes = {
      {{drone, 3, 0}, 1U, MakePose(0.0), true},
      {{drone, 3, 1}, 1U, MakePose(1.0), true}};
    ASSERT_EQ(
      store.CommitAnchor(snapshot, MakePose(0.0, drone), drone).status,
      PoseCommitStatus::Applied);
  }
  std::vector<orbslam3_multi::AcceptedSubmapPoseBatch> batches;
  for (uint32_t drone = 1; drone <= 2; ++drone) {
    const auto first = store.GetPose({drone, 3, 0});
    const auto second = store.GetPose({drone, 3, 1});
    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());
    orbslam3_multi::AcceptedSubmapPoseBatch batch;
    batch.submap_id = {drone, 3};
    batch.continuation_control = RawKeyFrameId{drone, 3, 1};
    batch.updates = {
      {{drone, 3, 0}, first->world_pose, 1U, first->pose_revision, false},
      {{drone, 3, 1}, MakePose(1.0, 2.0), 1U, second->pose_revision, drone == 2U}};
    batches.push_back(std::move(batch));
  }

  const auto result = store.CommitAcceptedPoseBatch(
    batches, PoseSourceKind::FiducialOptimized, 90U);
  ASSERT_EQ(result.status, PoseCommitStatus::Applied);
  ASSERT_EQ(result.submap_changes.size(), 2U);
  EXPECT_TRUE(store.GetPose({2, 3, 1})->hard_fiducial);
  EXPECT_EQ(
    store.GetPose({2, 3, 1})->source_kind,
    PoseSourceKind::FiducialAccepted);
  EXPECT_FALSE(store.GetPose({1, 3, 1})->hard_fiducial);
  EXPECT_EQ(
    store.GetPose({1, 3, 1})->source_kind,
    PoseSourceKind::FiducialOptimized);
}

TEST(SparseGlobalBackend, RoutesOnlyPoseRelevantKeyFrameChanges)
{
  SparseGlobalBackend backend;
  auto first = MakeMap(1, 0);
  auto & keyframe = first->keyframes.emplace_back();
  keyframe.id = 8;
  keyframe.pose = MakePose(1.0);
  const auto inserted = backend.InsertDelta(1, first);
  EXPECT_TRUE(inserted.pose_stage_executed);
  EXPECT_EQ(inserted.pose_changes.status, PoseCommitStatus::Unanchored);

  auto mappoint_only = MakeMap(1, 0);
  mappoint_only->mappoints.emplace_back().id = 90;
  const auto mp_result = backend.InsertDelta(2, mappoint_only);
  EXPECT_FALSE(mp_result.pose_stage_executed);

  const auto anchor = backend.CommitAnchor({1, 0}, MakePose(10.0), 3);
  EXPECT_EQ(anchor.status, PoseCommitStatus::Applied);
  ASSERT_TRUE(backend.GetGlobalPose({1, 0, 8}).has_value());
  EXPECT_NEAR(backend.GetGlobalPose({1, 0, 8})->world_pose.position.x, 11.0, 1e-9);

  auto association_only = MakeMap(1, 0);
  keyframe.mappoint_ids = {90};
  association_only->keyframes.push_back(keyframe);
  const auto association_result = backend.InsertDelta(3, association_only);
  EXPECT_FALSE(association_result.pose_stage_executed);

  auto pose_update = MakeMap(1, 0);
  keyframe.pose = MakePose(4.0);
  pose_update->keyframes.push_back(keyframe);
  const auto pose_result = backend.InsertDelta(4, pose_update);
  EXPECT_TRUE(pose_result.pose_stage_executed);
  EXPECT_EQ(pose_result.pose_changes.status, PoseCommitStatus::Applied);
  EXPECT_NEAR(backend.GetGlobalPose({1, 0, 8})->world_pose.position.x, 14.0, 1e-9);
}

TEST(SparseGlobalBackend, FirstFiducialAnchorsAndHighErrorRevisitCreatesTask)
{
  SparseGlobalBackend backend;
  auto map = MakeMap(2, 4);
  auto & first = map->keyframes.emplace_back();
  first.id = 9;
  first.pose = MakePose(3.0);
  auto & second = map->keyframes.emplace_back();
  second.id = 10;
  second.pose = MakePose(5.0);
  backend.InsertDelta(1, map);

  orbslam3_multi::FiducialObservation observation;
  observation.arrival_id = 1;
  observation.keyframe_id = {2, 4, 9};
  observation.fiducial_id = 2;
  observation.fiducial_visit_id = 1;
  observation.world_T_camera_target = MakePose(10.0);
  observation.source = "simulated_gt";
  observation.quality = "ok";
  const auto anchored = backend.ProcessFiducialObservation(observation, true);

  EXPECT_EQ(anchored.status, orbslam3_multi::FiducialProcessStatus::AnchorCreated);
  EXPECT_EQ(anchored.pose_changes.status, PoseCommitStatus::Applied);
  EXPECT_TRUE(anchored.hard_keyframe);
  EXPECT_EQ(backend.GetPoseStats().anchors, 1U);
  EXPECT_EQ(backend.GetPoseStats().poses, 2U);
  EXPECT_EQ(backend.GetPoseStats().hard_fiducial_keyframes, 1U);
  ASSERT_TRUE(backend.GetGlobalPose({2, 4, 9}).has_value());
  EXPECT_TRUE(backend.GetGlobalPose({2, 4, 9})->hard_fiducial);
  EXPECT_FALSE(backend.GetGlobalPose({2, 4, 10})->hard_fiducial);

  backend.InsertDelta(2, MakeMap(2, 4));
  observation.keyframe_id = {2, 4, 10};
  observation.arrival_id = 2;
  const auto deferred = backend.ProcessFiducialObservation(observation, true);
  EXPECT_EQ(
    deferred.status, orbslam3_multi::FiducialProcessStatus::OptimizationRequired);
  ASSERT_TRUE(deferred.optimization_task.has_value());
  EXPECT_EQ(deferred.optimization_task->control_keyframe_id.local_kf_id, 9U);
  EXPECT_EQ(backend.GetPoseStats().anchors, 1U);
  EXPECT_EQ(backend.GetPoseStats().hard_fiducial_keyframes, 1U);
  EXPECT_EQ(backend.GetFiducialObservationJournal().size(), 2U);
}

TEST(SparseGlobalBackend, FiducialOptimizationCommitsWindowAndLateTailAtomically)
{
  SparseGlobalBackend backend;
  auto initial = MakeMap(1, 7);
  for (uint64_t id = 0; id < 10; ++id) {
    auto & keyframe = initial->keyframes.emplace_back();
    keyframe.id = id;
    keyframe.pose = MakePose(static_cast<double>(id));
  }
  backend.InsertDelta(1, initial);

  orbslam3_multi::FiducialObservation first;
  first.arrival_id = 1;
  first.keyframe_id = {1, 7, 0};
  first.fiducial_id = 2;
  first.fiducial_visit_id = 1;
  first.world_T_camera_target = MakePose(0.0);
  first.source = "synthetic";
  ASSERT_EQ(
    backend.ProcessFiducialObservation(first, false).status,
    orbslam3_multi::FiducialProcessStatus::AnchorCreated);

  auto revisit = first;
  revisit.keyframe_id = {1, 7, 9};
  revisit.fiducial_id = 1;
  revisit.fiducial_visit_id = 2;
  revisit.world_T_camera_target = MakePose(9.0, 3.0);
  const auto revisit_result = backend.ProcessFiducialObservation(revisit, false);
  ASSERT_EQ(
    revisit_result.status,
    orbslam3_multi::FiducialProcessStatus::OptimizationRequired);
  ASSERT_TRUE(revisit_result.optimization_task.has_value());
  const auto revalidation = backend.RevalidateFiducialTask(
    *revisit_result.optimization_task);
  ASSERT_EQ(
    revalidation.decision, orbslam3_multi::FiducialTaskDecision::Ready);
  const auto graph = backend.BuildFiducialPoseGraph(revalidation.task);
  ASSERT_TRUE(graph.success) << graph.reason;
  const auto proposal = backend.OptimizeFiducialPoseGraph(graph.problem);
  const auto validation = backend.ValidateFiducialProposal(graph.problem, proposal);
  ASSERT_EQ(
    validation.decision, orbslam3_multi::ValidationDecision::AcceptFull);

  auto tail = MakeMap(1, 7);
  for (uint64_t id = 10; id < 12; ++id) {
    auto & keyframe = tail->keyframes.emplace_back();
    keyframe.id = id;
    keyframe.pose = MakePose(static_cast<double>(id));
  }
  backend.InsertDelta(2, tail);

  const auto commit = backend.CommitFiducialProposal(
    graph.problem, proposal, validation);
  ASSERT_TRUE(commit.committed) << commit.reason;
  EXPECT_TRUE(commit.full_accept);
  EXPECT_EQ(commit.tail_keyframes, 2U);
  EXPECT_GT(commit.pose_changes.updated_ids.size(), 0U);
  ASSERT_EQ(commit.pose_changes.hard_fiducial_ids.size(), 1U);
  EXPECT_EQ(commit.pose_changes.hard_fiducial_ids.front().local_kf_id, 9U);
  ASSERT_TRUE(backend.GetGlobalPose({1, 7, 9}).has_value());
  EXPECT_NEAR(backend.GetGlobalPose({1, 7, 9})->world_pose.position.y, 3.0, 1e-9);
  EXPECT_TRUE(backend.GetGlobalPose({1, 7, 9})->hard_fiducial);
  EXPECT_NEAR(backend.GetGlobalPose({1, 7, 11})->world_pose.position.y, 3.0, 1e-9);

  orbslam3_multi::FiducialOptimizationTask obsolete;
  obsolete.submap_id = {1, 7};
  obsolete.keyframe_id = {1, 7, 5};
  obsolete.target_world_T_kf = MakePose(5.0, -5.0);
  const auto obsolete_revalidation = backend.RevalidateFiducialTask(obsolete);
  EXPECT_EQ(
    obsolete_revalidation.decision,
    orbslam3_multi::FiducialTaskDecision::Stale);
  EXPECT_EQ(
    obsolete_revalidation.reason, "target_not_newer_than_current_control");

  auto future = MakeMap(1, 7);
  for (uint64_t id = 12; id < 14; ++id) {
    auto & keyframe = future->keyframes.emplace_back();
    keyframe.id = id;
    keyframe.pose = MakePose(static_cast<double>(id));
  }
  const auto future_insert = backend.InsertDelta(3, future);
  ASSERT_EQ(future_insert.pose_changes.control_propagated_ids.size(), 2U);
  const auto future_pose = backend.GetGlobalPose({1, 7, 13});
  ASSERT_TRUE(future_pose.has_value());
  EXPECT_EQ(future_pose->source_kind, PoseSourceKind::FiducialControlDerived);
  EXPECT_NEAR(future_pose->raw_world_pose.position.y, 0.0, 1e-9);
  EXPECT_NEAR(future_pose->world_pose.position.y, 3.0, 1e-9);
  EXPECT_NEAR(future_pose->correction_pose.position.y, 3.0, 1e-9);

  auto coherent = revisit;
  coherent.arrival_id = 3;
  coherent.keyframe_id = {1, 7, 13};
  coherent.world_T_camera_target = MakePose(13.0, 3.0);
  const auto coherent_result = backend.ProcessFiducialObservation(coherent, false);
  EXPECT_EQ(
    coherent_result.status,
    orbslam3_multi::FiducialProcessStatus::RevisitWithinThreshold);
  EXPECT_FALSE(coherent_result.optimization_task.has_value());
}

TEST(SparseGlobalBackend, MissingFiducialTargetIsStaleAndNeverHardFailure)
{
  SparseGlobalBackend backend;
  orbslam3_multi::FiducialOptimizationTask task;
  task.task_id = 300U;
  task.submap_id = {9, 4};
  task.keyframe_id = {9, 4, 20};
  const auto result = backend.RevalidateFiducialTask(task);
  EXPECT_EQ(result.decision, orbslam3_multi::FiducialTaskDecision::Stale);
  EXPECT_EQ(result.reason, "target_global_pose_missing");
}

TEST(GlobalPoseStore, ContinuationIsIsolatedAndDoesNotAdvanceWithoutControl)
{
  GlobalPoseStore store;
  RawSubmapPoseSnapshot first;
  first.submap_id = {1, 0};
  first.submap_revision = 1;
  first.keyframes = {
    {{1, 0, 0}, 1, MakePose(0.0), true},
    {{1, 0, 1}, 1, MakePose(1.0), true}};
  ASSERT_EQ(
    store.CommitAnchor(first, MakePose(10.0), 1, RawKeyFrameId{1, 0, 0}).status,
    PoseCommitStatus::Applied);

  const auto control_pose = store.GetPose({1, 0, 1});
  ASSERT_TRUE(control_pose.has_value());
  AcceptedPoseUpdate control{
    {1, 0, 1}, MakePose(20.0), 1, control_pose->pose_revision, true};
  ASSERT_EQ(
    store.CommitAcceptedPoses(
      {1, 0}, {control}, PoseSourceKind::FiducialOptimized, 2,
      RawKeyFrameId{1, 0, 1}).status,
    PoseCommitStatus::Applied);

  RawKeyFramePoseChange second;
  second.kind = RawKeyFramePoseChangeKind::New;
  second.keyframe = {{1, 0, 2}, 1, MakePose(2.0), true};
  ASSERT_EQ(store.ApplyRawPoseChanges({1, 0}, {second}, 3).status, PoseCommitStatus::Applied);
  EXPECT_NEAR(store.GetPose({1, 0, 2})->world_pose.position.x, 21.0, 1e-9);

  const auto second_pose = store.GetPose({1, 0, 2});
  ASSERT_TRUE(second_pose.has_value());
  AcceptedPoseUpdate partial{
    {1, 0, 2}, MakePose(30.0), 1, second_pose->pose_revision, false};
  ASSERT_EQ(
    store.CommitAcceptedPoses(
      {1, 0}, {partial}, PoseSourceKind::FiducialOptimized, 4).status,
    PoseCommitStatus::Applied);

  RawKeyFramePoseChange third;
  third.kind = RawKeyFramePoseChangeKind::New;
  third.keyframe = {{1, 0, 3}, 1, MakePose(3.0), true};
  ASSERT_EQ(store.ApplyRawPoseChanges({1, 0}, {third}, 5).status, PoseCommitStatus::Applied);
  EXPECT_NEAR(store.GetPose({1, 0, 3})->world_pose.position.x, 22.0, 1e-9);

  RawSubmapPoseSnapshot other;
  other.submap_id = {2, 0};
  other.submap_revision = 1;
  other.keyframes = {{{2, 0, 1}, 1, MakePose(1.0), true}};
  ASSERT_EQ(store.CommitAnchor(other, MakePose(100.0), 6).status, PoseCommitStatus::Applied);
  RawKeyFramePoseChange other_second;
  other_second.kind = RawKeyFramePoseChangeKind::New;
  other_second.keyframe = {{2, 0, 2}, 1, MakePose(2.0), true};
  ASSERT_EQ(
    store.ApplyRawPoseChanges({2, 0}, {other_second}, 7).status,
    PoseCommitStatus::Applied);
  EXPECT_NEAR(store.GetPose({2, 0, 2})->world_pose.position.x, 102.0, 1e-9);
}

TEST(SparseGlobalBackend, SnapshotDefersBuilderUntilNextDelta)
{
  SparseGlobalBackend backend;
  auto initial = MakeMap(1, 2);
  auto & keyframe = initial->keyframes.emplace_back();
  keyframe.id = 3;
  keyframe.pose = MakePose(1.0);
  keyframe.mappoint_ids = {20};
  auto & point = initial->mappoints.emplace_back();
  point.id = 20;
  point.position.x = 1.0;
  point.reference_keyframe_id = 3;
  backend.InsertDelta(1, initial);
  backend.CommitAnchor({1, 2}, MakePose(10.0), 1);
  const auto initial_build = backend.BuildGlobalMap();
  EXPECT_TRUE(initial_build.changed);

  auto snapshot = std::make_shared<OrbMap>(*initial);
  snapshot->keyframes[0].pose = MakePose(2.0);
  snapshot->mappoints[0].position.x = 4.0;
  const auto reconciled = backend.InsertFullSnapshot(2, snapshot);
  EXPECT_TRUE(reconciled.raw_result.has_material_changes);
  EXPECT_FALSE(reconciled.had_deferred_snapshot_dirty);

  const auto next_delta = backend.InsertDelta(3, std::make_shared<OrbMap>(*snapshot));
  EXPECT_TRUE(next_delta.had_deferred_snapshot_dirty);
  const auto deferred_build = backend.BuildGlobalMap();
  EXPECT_TRUE(deferred_build.changed);
  EXPECT_EQ(deferred_build.recalculated_keyframes, 1U);
  EXPECT_EQ(deferred_build.recalculated_mappoints, 1U);

  const auto noop = backend.InsertFullSnapshot(4, snapshot);
  EXPECT_FALSE(noop.raw_result.has_material_changes);
  EXPECT_FALSE(noop.raw_result.journal_entry_appended);
}

}  // namespace
