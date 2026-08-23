#include "orbslam3_multi/covisibility_database.hpp"
#include "orbslam3_multi/sparse_global_backend.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <memory>

namespace
{

geometry_msgs::msg::Pose Pose(double x, double y = 0.0, double z = 0.0)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;
  return pose;
}

orbslam3_msgs::msg::OrbDescriptor Descriptor(size_t index)
{
  orbslam3_msgs::msg::OrbDescriptor descriptor;
  descriptor.data[index % descriptor.data.size()] = static_cast<uint8_t>(1U << (index % 7U));
  descriptor.data[(index * 5U + 3U) % descriptor.data.size()] ^= 0x5aU;
  return descriptor;
}

std::shared_ptr<orbslam3_msgs::msg::OrbMap> MakeGeometricMap(
  uint32_t drone, uint64_t epoch, double keyframe_offset,
  uint64_t keyframe_count = 2)
{
  auto map = std::make_shared<orbslam3_msgs::msg::OrbMap>();
  map->drone_id = drone;
  map->map_epoch = epoch;
  for (uint64_t kf_id = 1; kf_id <= keyframe_count; ++kf_id) {
    auto & keyframe = map->keyframes.emplace_back();
    keyframe.id = kf_id;
    keyframe.pose = Pose(keyframe_offset + static_cast<double>(kf_id - 1));
    keyframe.bow_word_ids = {10, 20, 30};
    keyframe.bow_word_values = {0.6F, 0.3F, 0.1F};
    for (uint64_t mp_id = 1; mp_id <= 12; ++mp_id) {
      keyframe.mappoint_ids.push_back(mp_id);
    }
    keyframe.connected_keyframe_ids = {kf_id == 1 ? 2U : 1U};
    keyframe.connected_keyframe_weights = {20};
  }
  for (uint64_t mp_id = 1; mp_id <= 12; ++mp_id) {
    auto & point = map->mappoints.emplace_back();
    point.id = mp_id;
    point.position.x = 0.25 * static_cast<double>(mp_id % 4U);
    point.position.y = 0.35 * static_cast<double>((mp_id / 4U) % 3U);
    point.position.z = 0.20 * static_cast<double>((mp_id * 7U) % 5U);
    point.descriptor = Descriptor(mp_id);
    point.reference_keyframe_id = 1;
    point.observations_count = 2;
    point.found_ratio = 1.0F;
    point.observations.emplace_back().keyframe_id = 1;
    point.observations.emplace_back().keyframe_id = 2;
  }
  return map;
}

void SetBowWords(
  const std::shared_ptr<orbslam3_msgs::msg::OrbMap> & map,
  const std::array<uint32_t, 3> & words)
{
  for (auto & keyframe : map->keyframes) {
    keyframe.bow_word_ids.assign(words.begin(), words.end());
  }
}

void AnchorWithFiducial(
  orbslam3_multi::SparseGlobalBackend * backend, uint32_t drone,
  double world_x, uint64_t arrival)
{
  orbslam3_multi::FiducialObservation observation;
  observation.arrival_id = arrival;
  observation.keyframe_id = {drone, 0, 1};
  observation.fiducial_id = static_cast<int32_t>(drone);
  observation.fiducial_visit_id = 1;
  observation.world_T_camera_target = Pose(world_x);
  observation.source = "synthetic";
  const auto result = backend->ProcessFiducialObservation(observation, false);
  ASSERT_EQ(result.status, orbslam3_multi::FiducialProcessStatus::AnchorCreated);
  ASSERT_EQ(result.pose_changes.status, orbslam3_multi::PoseCommitStatus::Applied);
}

TEST(CovisibilityDatabase, AppliesCanonicalPatchAndIsIdempotent)
{
  orbslam3_multi::RawMapDatabase raw;
  const auto inserted = raw.InsertDelta(1, MakeGeometricMap(1, 0, 0.0));
  orbslam3_multi::DatabaseUpdateTask task;
  task.source_arrival_id = 1;
  task.submap_id = {1, 0};
  task.expected_submap_revision = inserted.submap_revision;
  task.covisibility_keyframe_ids = inserted.covisibility_changed_keyframe_ids;

  const auto patch = orbslam3_multi::CovisibilityDatabase::PrepareOrbslam3Patch(raw, task);
  ASSERT_TRUE(patch.has_value());
  orbslam3_multi::CovisibilityDatabase database;
  const auto first = database.ApplyPatch(*patch);
  EXPECT_TRUE(first.committed);
  EXPECT_EQ(first.added, 1U);
  const auto edge = database.GetEdge({1, 0, 2}, {1, 0, 1});
  ASSERT_TRUE(edge.has_value());
  EXPECT_EQ(edge->support, 20U);
  EXPECT_EQ(edge->source, orbslam3_multi::CovisibilityEdgeSource::Orbslam3Native);

  const auto second = database.ApplyPatch(*patch);
  EXPECT_TRUE(second.committed);
  EXPECT_EQ(second.revision_after, first.revision_after);
  EXPECT_EQ(second.unchanged, 1U);

  orbslam3_multi::CovisibilityPatch server_patch;
  server_patch.expected_database_revision = second.revision_after;
  auto server_edge = *edge;
  server_edge.source =
    orbslam3_multi::CovisibilityEdgeSource::ServerLoopGeometric;
  server_edge.support = 9;
  server_edge.information_weight = 90.0;
  server_patch.upserts.push_back(server_edge);
  const auto server = database.ApplyPatchTransactional(server_patch);
  ASSERT_TRUE(server.update.committed);
  EXPECT_TRUE(database.HasSource(
    {1, 0, 1}, {1, 0, 2},
    orbslam3_multi::CovisibilityEdgeSource::Orbslam3Native));
  EXPECT_TRUE(database.HasSource(
    {1, 0, 1}, {1, 0, 2},
    orbslam3_multi::CovisibilityEdgeSource::ServerLoopGeometric));
  EXPECT_EQ(database.GetStats().edges, 2U);
  ASSERT_TRUE(database.RollbackPatch(server.rollback));
  EXPECT_TRUE(database.HasSource(
    {1, 0, 1}, {1, 0, 2},
    orbslam3_multi::CovisibilityEdgeSource::Orbslam3Native));
  EXPECT_FALSE(database.HasSource(
    {1, 0, 1}, {1, 0, 2},
    orbslam3_multi::CovisibilityEdgeSource::ServerLoopGeometric));
}

TEST(LoopPipeline, CommitsFusionAndPublishesOneRepresentativePerTrack)
{
  orbslam3_multi::SparseGlobalBackend backend;
  const auto first = backend.InsertDelta(1, MakeGeometricMap(1, 0, 0.0));
  const auto second = backend.InsertDelta(2, MakeGeometricMap(2, 0, 0.0));
  ASSERT_EQ(
    backend.CommitAnchor({1, 0}, Pose(0.0), 10).status,
    orbslam3_multi::PoseCommitStatus::Applied);
  ASSERT_EQ(
    backend.CommitAnchor({2, 0}, Pose(0.0), 11).status,
    orbslam3_multi::PoseCommitStatus::Applied);

  for (const auto * inserted : {&first, &second}) {
    auto plan = backend.PlanSecondaryWork(inserted->raw_result);
    ASSERT_TRUE(plan.database_update.has_value());
    ASSERT_TRUE(backend.ProcessDatabaseUpdate(*plan.database_update).committed);
    auto tasks = backend.CreateLoopTasks(
      inserted->raw_result.arrival_id,
      plan.database_update->loop_keyframe_ids);
    for (auto & task : tasks) {
      task.task_id = 100 + task.query_keyframe_id.drone_id * 10 +
        task.query_keyframe_id.local_kf_id;
      backend.ProcessLoopTask(task);
    }
  }

  const auto fused = backend.GetFusedLandmarkStats();
  EXPECT_GT(fused.tracks, 0U);
  EXPECT_GT(fused.raw_members, fused.tracks);
  EXPECT_GT(backend.GetCovisibilityStats().server_loop_geometric_edges, 0U);
  const auto built = backend.BuildGlobalMap();
  EXPECT_TRUE(built.changed);
  EXPECT_GT(built.recalculated_fused_tracks, 0U);
  EXPECT_LT(built.points.size(), 24U);
  EXPECT_TRUE(std::any_of(
    built.points.begin(), built.points.end(),
    [](const auto & point) {return point.fused_track_id != 0U;}));
}

TEST(LoopPipeline, SemanticRevisionIgnoresTinyRawPoseChurn)
{
  orbslam3_multi::SparseGlobalBackend backend;
  const auto first = backend.InsertDelta(1, MakeGeometricMap(1, 0, 0.0));
  const auto first_tasks = backend.CreateLoopTasks(1, first.raw_result.new_keyframe_ids);
  ASSERT_EQ(first_tasks.size(), 2U);

  const auto tiny = backend.InsertDelta(2, MakeGeometricMap(1, 0, 0.001));
  const auto tiny_tasks = backend.CreateLoopTasks(2, tiny.raw_result.pose_changed_keyframe_ids);
  ASSERT_EQ(tiny_tasks.size(), 2U);
  EXPECT_NE(first_tasks[0].revision.raw_revision, tiny_tasks[0].revision.raw_revision);
  EXPECT_EQ(
    first_tasks[0].revision.appearance_revision,
    tiny_tasks[0].revision.appearance_revision);
  EXPECT_EQ(
    first_tasks[0].revision.geometry_revision,
    tiny_tasks[0].revision.geometry_revision);

  const auto material = backend.InsertDelta(3, MakeGeometricMap(1, 0, 0.60));
  const auto material_tasks = backend.CreateLoopTasks(
    3, material.raw_result.pose_changed_keyframe_ids);
  ASSERT_EQ(material_tasks.size(), 2U);
  EXPECT_NE(
    tiny_tasks[0].revision.geometry_revision,
    material_tasks[0].revision.geometry_revision);
}

TEST(LoopPipeline, SemanticRevisionCoalescesMapPointRefinement)
{
  orbslam3_multi::SparseGlobalBackend backend;
  const auto first = backend.InsertDelta(1, MakeGeometricMap(1, 0, 0.0));
  const auto first_tasks = backend.CreateLoopTasks(1, first.raw_result.new_keyframe_ids);
  ASSERT_EQ(first_tasks.size(), 2U);

  auto refined_map = MakeGeometricMap(1, 0, 0.0);
  for (auto & point : refined_map->mappoints) {
    point.position.x += 0.03;
    point.position.y -= 0.02;
    point.descriptor.data[0] ^= 0xffU;
  }
  backend.InsertDelta(2, refined_map);
  const auto refined_tasks = backend.CreateLoopTasks(2, {{1, 0, 1}, {1, 0, 2}});
  ASSERT_EQ(refined_tasks.size(), 2U);
  EXPECT_EQ(
    first_tasks[0].revision.geometry_revision,
    refined_tasks[0].revision.geometry_revision);
  EXPECT_NE(
    first_tasks[0].revision.validation_revision,
    refined_tasks[0].revision.validation_revision);

  auto mature_map = MakeGeometricMap(1, 0, 0.0);
  for (uint64_t mp_id = 13; mp_id <= 48; ++mp_id) {
    auto & point = mature_map->mappoints.emplace_back();
    point.id = mp_id;
    point.position.x = 0.1 * static_cast<double>(mp_id);
    point.descriptor = Descriptor(mp_id);
    point.observations_count = 2;
    point.found_ratio = 1.0F;
    for (auto & keyframe : mature_map->keyframes) {
      keyframe.mappoint_ids.push_back(mp_id);
    }
  }
  const auto mature = backend.InsertDelta(3, mature_map);
  const auto mature_tasks = backend.CreateLoopTasks(
    3, mature.raw_result.association_changed_keyframe_ids);
  ASSERT_EQ(mature_tasks.size(), 2U);
  EXPECT_NE(
    refined_tasks[0].revision.geometry_revision,
    mature_tasks[0].revision.geometry_revision);
}

TEST(LoopPipeline, AnchorsUnanchoredSubmapAfterTwoIndependentQueries)
{
  orbslam3_multi::SparseGlobalBackend backend;
  const auto first = backend.InsertDelta(1, MakeGeometricMap(1, 0, 0.0));
  ASSERT_EQ(
    backend.CommitAnchor({1, 0}, Pose(10.0), 100).status,
    orbslam3_multi::PoseCommitStatus::Applied);
  const auto second = backend.InsertDelta(2, MakeGeometricMap(2, 0, 0.0));

  auto first_plan = backend.PlanSecondaryWork(first.raw_result);
  ASSERT_TRUE(first_plan.database_update.has_value());
  EXPECT_TRUE(backend.ProcessDatabaseUpdate(*first_plan.database_update).committed);
  auto first_tasks = backend.CreateLoopTasks(
    1, first_plan.database_update->loop_keyframe_ids);
  uint64_t task_id = 1000;
  for (auto & task : first_tasks) {
    task.task_id = task_id++;
    backend.ProcessLoopTask(task);
  }

  auto second_plan = backend.PlanSecondaryWork(second.raw_result);
  ASSERT_TRUE(second_plan.database_update.has_value());
  EXPECT_TRUE(backend.ProcessDatabaseUpdate(*second_plan.database_update).committed);
  auto second_tasks = backend.CreateLoopTasks(
    2, second_plan.database_update->loop_keyframe_ids);
  bool committed = false;
  for (auto & task : second_tasks) {
    task.task_id = task_id++;
    const auto result = backend.ProcessLoopTask(task);
    committed = committed ||
      result.anchor_commit.status == orbslam3_multi::PoseCommitStatus::Applied;
    if (result.anchor_commit.status == orbslam3_multi::PoseCommitStatus::Applied) {
      EXPECT_FALSE(result.recent_loss_gate_checked);
    }
  }

  EXPECT_TRUE(committed);
  EXPECT_EQ(backend.GetPoseStats().anchors, 2U);
  const auto loop_pose = backend.GetGlobalPose({2, 0, 2});
  ASSERT_TRUE(loop_pose.has_value());
  EXPECT_EQ(loop_pose->source_kind, orbslam3_multi::PoseSourceKind::LoopAnchorDerived);
  EXPECT_NEAR(loop_pose->world_pose.position.x, 11.0, 1e-6);

  orbslam3_multi::FiducialObservation fiducial;
  fiducial.arrival_id = 3;
  fiducial.keyframe_id = {2, 0, 2};
  fiducial.fiducial_id = 1;
  fiducial.fiducial_visit_id = 1;
  fiducial.world_T_camera_target = Pose(11.0, 2.0);
  fiducial.source = "synthetic";
  const auto revisit = backend.ProcessFiducialObservation(fiducial, false);
  EXPECT_EQ(
    revisit.status,
    orbslam3_multi::FiducialProcessStatus::OptimizationRequired);
  ASSERT_TRUE(revisit.optimization_task.has_value());
  EXPECT_TRUE(revisit.optimization_task->replaces_soft_loop_anchor);
  const auto revalidation = backend.RevalidateFiducialTask(
    *revisit.optimization_task);
  ASSERT_EQ(
    revalidation.decision,
    orbslam3_multi::FiducialTaskDecision::Ready);
  const auto graph = backend.BuildFiducialPoseGraph(revalidation.task);
  ASSERT_TRUE(graph.success) << graph.reason;
  EXPECT_EQ(graph.problem.submap_windows.size(), 2U);
  EXPECT_TRUE(std::any_of(
    graph.problem.covisibility_edges.begin(),
    graph.problem.covisibility_edges.end(),
    [](const auto & edge) {
      return edge.kind == orbslam3_multi::PoseGraphEdgeKind::PriorLoop;
    }));
  const auto proposal = backend.OptimizeFiducialPoseGraph(graph.problem);
  const auto validation = backend.ValidateFiducialProposal(
    graph.problem, proposal);
  ASSERT_EQ(
    validation.decision,
    orbslam3_multi::ValidationDecision::AcceptFull) << validation.reason;
  auto changed_parent = MakeGeometricMap(1, 0, 0.0);
  for (auto & keyframe : changed_parent->keyframes) {
    keyframe.pose.position.x += 0.75;
  }
  changed_parent->keyframes.front().is_bad = true;
  backend.InsertDelta(4, changed_parent);
  const auto commit_result = backend.CommitFiducialProposal(
    graph.problem, proposal, validation);
  ASSERT_TRUE(commit_result.committed) << commit_result.reason;
  EXPECT_TRUE(commit_result.full_accept);
  const auto hard_pose = backend.GetGlobalPose({2, 0, 2});
  ASSERT_TRUE(hard_pose.has_value());
  EXPECT_TRUE(hard_pose->hard_fiducial);
  EXPECT_NEAR(hard_pose->world_pose.position.x, 11.0, 1e-6);
  EXPECT_NEAR(hard_pose->world_pose.position.y, 2.0, 1e-6);
  const auto reanchored_first = backend.GetGlobalPose({2, 0, 1});
  ASSERT_TRUE(reanchored_first.has_value());
  EXPECT_NEAR(reanchored_first->world_pose.position.x, 10.0, 1e-6);
  EXPECT_NEAR(reanchored_first->world_pose.position.y, 2.0, 1e-6);
}

TEST(LoopPipeline, RejectsFarRepeatedZoneAfterAnchoredEpochLoss)
{
  orbslam3_multi::SparseGlobalBackend backend;
  auto previous = MakeGeometricMap(2, 0, 0.0);
  SetBowWords(previous, {110U, 120U, 130U});
  const auto previous_insert = backend.InsertDelta(1, previous);
  ASSERT_EQ(
    backend.CommitAnchor({2, 0}, Pose(10.0), 100).status,
    orbslam3_multi::PoseCommitStatus::Applied);

  const auto repeated_zone = backend.InsertDelta(
    2, MakeGeometricMap(1, 0, 0.0));
  ASSERT_EQ(
    backend.CommitAnchor({1, 0}, Pose(200.0), 101).status,
    orbslam3_multi::PoseCommitStatus::Applied);

  for (const auto * inserted : {&previous_insert, &repeated_zone}) {
    auto plan = backend.PlanSecondaryWork(inserted->raw_result);
    ASSERT_TRUE(plan.database_update.has_value());
    ASSERT_TRUE(backend.ProcessDatabaseUpdate(*plan.database_update).committed);
    auto tasks = backend.CreateLoopTasks(
      inserted->raw_result.arrival_id,
      plan.database_update->loop_keyframe_ids);
    for (auto & task : tasks) {
      task.task_id = 3000U + task.query_keyframe_id.drone_id * 10U +
        task.query_keyframe_id.local_kf_id;
      backend.ProcessLoopTask(task);
    }
  }

  const auto lost = backend.InsertDelta(3, MakeGeometricMap(2, 1, 0.0));
  auto lost_plan = backend.PlanSecondaryWork(lost.raw_result);
  ASSERT_TRUE(lost_plan.database_update.has_value());
  ASSERT_TRUE(backend.ProcessDatabaseUpdate(*lost_plan.database_update).committed);
  auto lost_tasks = backend.CreateLoopTasks(
    lost.raw_result.arrival_id, lost_plan.database_update->loop_keyframe_ids);

  bool continuity_rejected = false;
  uint64_t task_id = 3100U;
  for (auto & task : lost_tasks) {
    task.task_id = task_id++;
    const auto result = backend.ProcessLoopTask(task);
    if (result.reason == "recent_loss_continuity_rejected") {
      continuity_rejected = true;
      EXPECT_EQ(result.decision, orbslam3_multi::LoopTaskDecisionKind::Deferred);
      EXPECT_TRUE(result.recent_loss_gate_checked);
      EXPECT_FALSE(result.recent_loss_gate_passed);
      EXPECT_GT(
        result.recent_loss_translation_m,
        result.recent_loss_translation_limit_m);
      EXPECT_TRUE(result.anchor_entries.empty());
    }
  }

  EXPECT_TRUE(continuity_rejected);
  EXPECT_EQ(backend.GetPoseStats().anchors, 2U);
  EXPECT_FALSE(backend.GetGlobalPose({2, 1, 1}).has_value());
}

TEST(LoopPipeline, UnanchoredSameSubmapEvidenceRemainsDiagnostic)
{
  orbslam3_multi::SparseGlobalBackend backend;
  const auto inserted = backend.InsertDelta(1, MakeGeometricMap(1, 0, 0.0, 4));
  auto plan = backend.PlanSecondaryWork(inserted.raw_result);
  ASSERT_TRUE(plan.database_update.has_value());
  ASSERT_TRUE(backend.ProcessDatabaseUpdate(*plan.database_update).committed);
  auto tasks = backend.CreateLoopTasks(
    inserted.raw_result.arrival_id, plan.database_update->loop_keyframe_ids);
  ASSERT_EQ(tasks.size(), 4U);

  uint64_t task_id = 1500;
  bool saw_same_submap_geometry = false;
  for (auto & task : tasks) {
    task.task_id = task_id++;
    const auto result = backend.ProcessLoopTask(task);
    EXPECT_NE(
      result.decision,
      orbslam3_multi::LoopTaskDecisionKind::OptimizationEvidence);
    for (const auto & geometry : result.geometry_results) {
      if (geometry.accepted &&
        geometry.query_submap_id == geometry.candidate_submap_id)
      {
        saw_same_submap_geometry = true;
        EXPECT_EQ(geometry.reason, "same_submap_unanchored_diagnostic");
      }
    }
  }

  EXPECT_TRUE(saw_same_submap_geometry);
  EXPECT_EQ(backend.GetPoseStats().anchors, 0U);
}

TEST(LoopPipeline, HighErrorAnchoredLoopOptimizesAndFusesInSameTask)
{
  orbslam3_multi::SparseGlobalBackend backend;
  const auto first = backend.InsertDelta(1, MakeGeometricMap(1, 0, 0.0));
  const auto second = backend.InsertDelta(2, MakeGeometricMap(2, 0, 0.0));
  ASSERT_EQ(
    backend.CommitAnchor({1, 0}, Pose(10.0), 100).status,
    orbslam3_multi::PoseCommitStatus::Applied);
  ASSERT_EQ(
    backend.CommitAnchor({2, 0}, Pose(10.0, 2.0), 101).status,
    orbslam3_multi::PoseCommitStatus::Applied);

  uint64_t task_id = 2000;
  for (const auto * inserted : {&first, &second}) {
    auto plan = backend.PlanSecondaryWork(inserted->raw_result);
    ASSERT_TRUE(plan.database_update.has_value());
    ASSERT_TRUE(backend.ProcessDatabaseUpdate(*plan.database_update).committed);
    auto tasks = backend.CreateLoopTasks(
      inserted->raw_result.arrival_id,
      plan.database_update->loop_keyframe_ids);
    for (auto & task : tasks) {
      task.task_id = task_id++;
      const auto detected = backend.ProcessLoopTask(task);
      if (detected.decision ==
        orbslam3_multi::LoopTaskDecisionKind::OptimizationEvidence)
      {
        auto refresh_task = detected.task;
        refresh_task.task_id = task_id++;
        refresh_task.intent = orbslam3_multi::LoopTaskIntent::FusionRefresh;
        const auto refresh = backend.ProcessLoopTask(refresh_task);
        EXPECT_EQ(refresh.decision, orbslam3_multi::LoopTaskDecisionKind::NoCandidates);
        EXPECT_EQ(refresh.reason, "fusion_refresh_no_spatial_candidates");
        EXPECT_GT(refresh.refresh_spatial_rejected, 0U);
        EXPECT_FALSE(refresh.optimization.attempted);

        const auto optimized = backend.ProcessLoopOptimization(detected);
        EXPECT_EQ(
          optimized.decision,
          orbslam3_multi::LoopTaskDecisionKind::OptimizationCommitted)
          << optimized.reason;
        EXPECT_TRUE(optimized.optimization.graph_built);
        EXPECT_TRUE(optimized.optimization.accepted);
        EXPECT_TRUE(optimized.optimization.committed);
        EXPECT_LT(
          optimized.optimization.final_translation_error_m,
          optimized.optimization.initial_translation_error_m);
        EXPECT_TRUE(optimized.fusion.committed) << optimized.fusion.reason;
        EXPECT_TRUE(optimized.optimization.fusion_after_optimization);
        EXPECT_FALSE(optimized.rerun_keyframe_ids.empty());

        auto reevaluation_tasks = backend.CreateLoopTasks(
          3, {detected.task.query_keyframe_id});
        ASSERT_EQ(reevaluation_tasks.size(), 1U);
        reevaluation_tasks.front().task_id = task_id++;
        const auto reevaluated = backend.ProcessLoopTask(reevaluation_tasks.front());
        EXPECT_NE(
          reevaluated.decision,
          orbslam3_multi::LoopTaskDecisionKind::OptimizationEvidence);
        EXPECT_FALSE(std::any_of(
          reevaluated.geometry_results.begin(), reevaluated.geometry_results.end(),
          [](const auto & geometry) {
            return geometry.accepted && geometry.fusion_compatible;
          })) << "una pareja ServerLoopGeometric no debe fusionarse de nuevo";
        return;
      }
    }
  }
  FAIL() << "no se obtuvo OptimizationEvidence tras dos queries independientes";
}

TEST(LoopPipeline, ProtectedRegionsRejectFarRepeatedLoopBeforeBuilderAndCacheRegion)
{
  orbslam3_multi::SparseGlobalBackend backend;
  const auto first = backend.InsertDelta(1, MakeGeometricMap(1, 0, 0.0, 3));
  const auto second = backend.InsertDelta(2, MakeGeometricMap(2, 0, 0.0, 3));
  AnchorWithFiducial(&backend, 1, 0.0, 10);
  AnchorWithFiducial(&backend, 2, 30.0, 11);

  uint64_t task_id = 3000;
  bool rejected = false;
  bool ledger_hit = false;
  for (const auto * inserted : {&first, &second}) {
    auto plan = backend.PlanSecondaryWork(inserted->raw_result);
    ASSERT_TRUE(plan.database_update.has_value());
    ASSERT_TRUE(backend.ProcessDatabaseUpdate(*plan.database_update).committed);
    auto tasks = backend.CreateLoopTasks(
      inserted->raw_result.arrival_id, plan.database_update->loop_keyframe_ids);
    for (auto & task : tasks) {
      task.task_id = task_id++;
      const auto result = backend.ProcessLoopTask(task);
      rejected = rejected || result.protected_region_rejected;
      ledger_hit = ledger_hit || result.rejection_ledger_hit;
      if (result.protected_region_rejected || result.rejection_ledger_hit) {
        EXPECT_EQ(result.decision, orbslam3_multi::LoopTaskDecisionKind::GeometryRejected);
        EXPECT_FALSE(result.optimization.attempted);
        EXPECT_FALSE(result.optimization.graph_built);
      }
    }
  }
  EXPECT_TRUE(rejected);
  EXPECT_TRUE(ledger_hit);
}

TEST(LoopPipeline, ProtectedToUnreliableKeepsAsymmetricOptimizationEvidence)
{
  orbslam3_multi::SparseGlobalBackend backend;
  const auto first = backend.InsertDelta(1, MakeGeometricMap(1, 0, 0.0, 3));
  const auto second = backend.InsertDelta(2, MakeGeometricMap(2, 0, 0.0, 3));
  AnchorWithFiducial(&backend, 1, 0.0, 10);
  ASSERT_EQ(
    backend.CommitAnchor({2, 0}, Pose(30.0), 11).status,
    orbslam3_multi::PoseCommitStatus::Applied);

  uint64_t task_id = 3100;
  bool saw_optimization = false;
  for (const auto * inserted : {&first, &second}) {
    auto plan = backend.PlanSecondaryWork(inserted->raw_result);
    ASSERT_TRUE(plan.database_update.has_value());
    ASSERT_TRUE(backend.ProcessDatabaseUpdate(*plan.database_update).committed);
    auto tasks = backend.CreateLoopTasks(
      inserted->raw_result.arrival_id, plan.database_update->loop_keyframe_ids);
    for (auto & task : tasks) {
      task.task_id = task_id++;
      const auto result = backend.ProcessLoopTask(task);
      if (result.decision == orbslam3_multi::LoopTaskDecisionKind::OptimizationEvidence) {
        saw_optimization = true;
        EXPECT_TRUE(result.protected_candidate_stable || result.protected_query_stable);
        EXPECT_FALSE(result.protected_region_rejected);
      }
    }
  }
  EXPECT_TRUE(saw_optimization);
}

TEST(LoopPipeline, FusionRefreshOmitsFarBowCandidatesAndGroupsMovedRegion)
{
  orbslam3_multi::SparseGlobalBackend backend;
  const auto first = backend.InsertDelta(1, MakeGeometricMap(1, 0, 0.0, 20));
  const auto second = backend.InsertDelta(2, MakeGeometricMap(2, 0, 0.0, 2));
  ASSERT_EQ(
    backend.CommitAnchor({1, 0}, Pose(0.0), 10).status,
    orbslam3_multi::PoseCommitStatus::Applied);
  ASSERT_EQ(
    backend.CommitAnchor({2, 0}, Pose(30.0), 11).status,
    orbslam3_multi::PoseCommitStatus::Applied);

  auto first_plan = backend.PlanSecondaryWork(first.raw_result);
  ASSERT_TRUE(first_plan.database_update.has_value());
  ASSERT_TRUE(backend.ProcessDatabaseUpdate(*first_plan.database_update).committed);
  auto indexed = backend.CreateLoopTasks(1, first_plan.database_update->loop_keyframe_ids);
  uint64_t task_id = 3200;
  for (auto & task : indexed) {
    task.task_id = task_id++;
    backend.ProcessLoopTask(task);
  }

  auto refresh = backend.CreateFusionRefreshTasks(2, second.raw_result.new_keyframe_ids);
  ASSERT_EQ(refresh.size(), 1U);
  EXPECT_EQ(refresh.front().intent, orbslam3_multi::LoopTaskIntent::FusionRefresh);
  refresh.front().task_id = task_id++;
  const auto result = backend.ProcessLoopTask(refresh.front());
  EXPECT_EQ(result.decision, orbslam3_multi::LoopTaskDecisionKind::NoCandidates);
  EXPECT_EQ(result.reason, "fusion_refresh_no_spatial_candidates");
  EXPECT_GT(result.refresh_spatial_rejected, 0U);

  auto all_ids = first.raw_result.new_keyframe_ids;
  auto grouped = backend.CreateFusionRefreshTasks(3, all_ids);
  EXPECT_LT(grouped.size(), all_ids.size());
  EXPECT_TRUE(std::all_of(
    grouped.begin(), grouped.end(), [](const auto & task) {
      return task.intent == orbslam3_multi::LoopTaskIntent::FusionRefresh;
    }));
}

TEST(GlobalPoseStore, SoftLoopAnchorFollowsParentUntilChildGetsHardFiducial)
{
  orbslam3_multi::GlobalPoseStore store;
  orbslam3_multi::RawSubmapPoseSnapshot parent;
  parent.submap_id = {1, 0};
  parent.submap_revision = 1;
  parent.keyframes = {{{1, 0, 1}, 1, Pose(0.0), true}};
  ASSERT_EQ(
    store.CommitAnchor(parent, Pose(0.0), 1).status,
    orbslam3_multi::PoseCommitStatus::Applied);

  orbslam3_multi::LoopAnchorBatchEntry child;
  child.snapshot.submap_id = {2, 0};
  child.snapshot.submap_revision = 1;
  child.snapshot.keyframes = {{{2, 0, 1}, 1, Pose(0.0), true}};
  child.world_T_local = Pose(0.0);
  child.loop_control_keyframe_id = {2, 0, 1};
  child.parent_submap_id = orbslam3_multi::RawSubmapId{1, 0};
  child.parent_control_keyframe_id = orbslam3_multi::RawKeyFrameId{1, 0, 1};
  ASSERT_EQ(
    store.CommitLoopAnchorBatch({child}, 2).status,
    orbslam3_multi::PoseCommitStatus::Applied);

  orbslam3_multi::AcceptedPoseUpdate move_parent;
  move_parent.keyframe_id = {1, 0, 1};
  move_parent.world_pose = Pose(2.0);
  move_parent.base_raw_revision = 1;
  move_parent.expected_pose_revision = 1;
  const auto propagated = store.CommitAcceptedPoses(
    {1, 0}, {move_parent}, orbslam3_multi::PoseSourceKind::FiducialOptimized, 3);
  EXPECT_EQ(propagated.status, orbslam3_multi::PoseCommitStatus::Applied);
  EXPECT_NEAR(store.GetPose({2, 0, 1})->world_pose.position.x, 2.0, 1e-9);
  EXPECT_EQ(propagated.control_propagated_ids.size(), 1U);

  const auto child_pose = store.GetPose({2, 0, 1});
  ASSERT_TRUE(child_pose.has_value());
  orbslam3_multi::AcceptedPoseUpdate hard_child;
  hard_child.keyframe_id = {2, 0, 1};
  hard_child.world_pose = Pose(7.0);
  hard_child.base_raw_revision = 1;
  hard_child.expected_pose_revision = child_pose->pose_revision;
  hard_child.mark_hard_fiducial = true;
  ASSERT_EQ(
    store.CommitAcceptedPoses(
      {2, 0}, {hard_child}, orbslam3_multi::PoseSourceKind::FiducialAccepted,
      4, hard_child.keyframe_id, Pose(7.0)).status,
    orbslam3_multi::PoseCommitStatus::Applied);
  EXPECT_FALSE(store.HasLoopDependency({2, 0}));
  ASSERT_TRUE(store.GetSubmapAnchorPose({2, 0}).has_value());
  EXPECT_NEAR(store.GetSubmapAnchorPose({2, 0})->position.x, 7.0, 1e-9);

  move_parent.world_pose = Pose(4.0);
  move_parent.expected_pose_revision = store.GetPose({1, 0, 1})->pose_revision;
  ASSERT_EQ(
    store.CommitAcceptedPoses(
      {1, 0}, {move_parent}, orbslam3_multi::PoseSourceKind::FiducialOptimized,
      5).status,
    orbslam3_multi::PoseCommitStatus::Applied);
  EXPECT_NEAR(store.GetPose({2, 0, 1})->world_pose.position.x, 7.0, 1e-9);
  EXPECT_TRUE(store.GetPose({2, 0, 1})->hard_fiducial);
}

}  // namespace
