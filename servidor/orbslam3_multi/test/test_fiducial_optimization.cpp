#include "orbslam3_multi/optimization_manager.hpp"
#include "orbslam3_multi/optimization_validator.hpp"
#include "orbslam3_multi/pose_graph_builder.hpp"
#include "orbslam3_multi/pose_geometry.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <map>

namespace
{

geometry_msgs::msg::Pose MakePose(double x, double y = 0.0, double yaw = 0.0)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.z = std::sin(0.5 * yaw);
  pose.orientation.w = std::cos(0.5 * yaw);
  return pose;
}

void AddCurrentLoopEdge(orbslam3_multi::PoseGraphProblem & problem)
{
  orbslam3_multi::PoseGraphEdge loop;
  loop.from_index = 0;
  loop.to_index = 1;
  loop.relative_raw_pose = MakePose(1.0);
  loop.kind = orbslam3_multi::PoseGraphEdgeKind::CurrentLoop;
  loop.information_weight = 40.0;
  problem.loop_edges.push_back(loop);
}

struct Fixture
{
  orbslam3_multi::RawSubmapPoseSnapshot raw;
  std::map<orbslam3_multi::RawKeyFrameId, orbslam3_multi::GlobalPoseRecord> poses;
  orbslam3_multi::FiducialOptimizationTask task;

  Fixture()
  {
    raw.submap_id = {1, 2};
    raw.submap_revision = 20;
    for (uint64_t id = 0; id < 10; ++id) {
      orbslam3_multi::RawKeyFramePoseInput input;
      input.id = {1, 2, id};
      input.raw_revision = id + 1;
      input.local_pose = MakePose(static_cast<double>(id));
      raw.keyframes.push_back(input);

      orbslam3_multi::GlobalPoseRecord record;
      record.keyframe_id = input.id;
      record.world_pose = input.local_pose;
      record.raw_world_pose = input.local_pose;
      record.pose_revision = id + 3;
      record.active = true;
      record.hard_fiducial = id == 0;
      poses[input.id] = record;
    }
    task.task_id = 44;
    task.submap_id = raw.submap_id;
    task.control_keyframe_id = {1, 2, 0};
    task.keyframe_id = {1, 2, 9};
    task.target_world_T_kf = MakePose(9.0, 3.0, 0.5);
  }
};

TEST(FiducialOptimization, BuildsBalancedTemporalGraph)
{
  Fixture fixture;
  orbslam3_multi::PoseGraphBuilder builder;
  const auto result = builder.Build(fixture.task, fixture.raw, fixture.poses, 7);

  ASSERT_TRUE(result.success) << result.reason;
  EXPECT_EQ(result.problem.keyframes.size(), 10U);
  EXPECT_EQ(result.problem.control_indices.size(), 3U);
  EXPECT_EQ(result.problem.temporal_edges.size(), 2U);
  EXPECT_EQ(result.problem.propagation_plan.size(), 7U);
  EXPECT_EQ(result.problem.control_indices.front(), 0U);
  EXPECT_EQ(result.problem.control_indices.back(), 9U);
  EXPECT_TRUE(result.problem.keyframes.front().fixed);
}

TEST(FiducialOptimization, SkipsInactiveIntermediatesAndKeepsHardControl)
{
  Fixture fixture;
  fixture.raw.keyframes.front().active = false;
  fixture.poses.at({1, 2, 0}).active = false;
  fixture.raw.keyframes[4].active = false;
  fixture.poses.at({1, 2, 4}).active = false;

  orbslam3_multi::PoseGraphBuilder builder;
  const auto result = builder.Build(fixture.task, fixture.raw, fixture.poses, 7);

  ASSERT_TRUE(result.success) << result.reason;
  ASSERT_EQ(result.problem.keyframes.size(), 9U);
  EXPECT_EQ(result.problem.keyframes.front().id.local_kf_id, 0U);
  EXPECT_EQ(result.problem.keyframes.back().id.local_kf_id, 9U);
  EXPECT_TRUE(result.problem.keyframes.front().fixed);
  for (const auto & keyframe : result.problem.keyframes) {
    EXPECT_NE(keyframe.id.local_kf_id, 4U);
  }
}

TEST(FiducialOptimization, FullProposalPinsStartAndReachesAbsoluteTarget)
{
  Fixture fixture;
  orbslam3_multi::PoseGraphBuilder builder;
  const auto graph = builder.Build(fixture.task, fixture.raw, fixture.poses, 7);
  ASSERT_TRUE(graph.success);

  orbslam3_multi::OptimizationManager optimizer;
  const auto proposal = optimizer.Optimize(graph.problem);
  ASSERT_EQ(proposal.status, orbslam3_multi::OptimizationSolverStatus::Converged);
  ASSERT_EQ(proposal.controls.size(), graph.problem.control_indices.size());
  EXPECT_NEAR(proposal.controls.front().world_pose.position.x, 0.0, 1e-9);
  EXPECT_NEAR(proposal.controls.front().world_pose.position.y, 0.0, 1e-9);
  EXPECT_NEAR(proposal.final_error.translation_m, 0.0, 1e-9);
  EXPECT_NEAR(proposal.final_error.rotation_rad, 0.0, 1e-9);

  orbslam3_multi::OptimizationValidator validator;
  const auto validation = validator.Validate(graph.problem, proposal);
  EXPECT_EQ(validation.decision, orbslam3_multi::ValidationDecision::AcceptFull);
}

TEST(FiducialOptimization, StrongCovisibilityPromotesAndConstrainsControls)
{
  Fixture fixture;
  orbslam3_multi::CovisibilityDatabase covisibility;
  orbslam3_multi::CovisibilityPatch patch;
  orbslam3_multi::CovisibilityEdge edge;
  edge.kf_a = {1, 2, 2};
  edge.kf_b = {1, 2, 7};
  edge.source = orbslam3_multi::CovisibilityEdgeSource::ServerLoopGeometric;
  edge.support = 20;
  edge.information_weight = 20.0;
  edge.relative_pose_measured = MakePose(0.0);
  edge.relative_pose_current = MakePose(5.0);
  patch.upserts.push_back(edge);
  ASSERT_TRUE(covisibility.ApplyPatch(patch).committed);

  orbslam3_multi::PoseGraphBuilder builder;
  const auto graph = builder.Build(
    fixture.task, fixture.raw, fixture.poses, 7, &covisibility);
  ASSERT_TRUE(graph.success) << graph.reason;
  EXPECT_GE(graph.problem.covisibility_edges.size(), 1U);
  const auto is_control = [&graph](uint64_t local_id) {
      return std::any_of(
        graph.problem.control_indices.begin(), graph.problem.control_indices.end(),
        [&graph, local_id](size_t index) {
          return graph.problem.keyframes[index].id.local_kf_id == local_id;
        });
    };
  EXPECT_TRUE(is_control(2));
  EXPECT_TRUE(is_control(7));

  orbslam3_multi::OptimizationManager optimizer;
  const auto proposal = optimizer.Optimize(graph.problem);
  ASSERT_EQ(proposal.status, orbslam3_multi::OptimizationSolverStatus::Converged)
    << proposal.reason;
  const auto find_x = [&proposal](uint64_t local_id) {
      const auto found = std::find_if(
        proposal.controls.begin(), proposal.controls.end(),
        [local_id](const auto & control) {
          return control.id.local_kf_id == local_id;
        });
      return found == proposal.controls.end() ?
             std::numeric_limits<double>::quiet_NaN() : found->world_pose.position.x;
    };
  EXPECT_LT(std::abs(find_x(7) - find_x(2)), 5.0);
  EXPECT_NEAR(proposal.final_error.translation_m, 0.0, 1e-9);
}

TEST(FiducialOptimization, SafePartialProposalRequestsAnotherPass)
{
  Fixture fixture;
  orbslam3_multi::PoseGraphBuilder builder;
  const auto graph = builder.Build(fixture.task, fixture.raw, fixture.poses, 7);
  ASSERT_TRUE(graph.success);

  orbslam3_multi::FiducialOptimizationConfig config;
  config.translation_threshold_m = 0.01;
  config.rotation_threshold_rad = 0.01;
  config.yaw_threshold_rad = 0.01;
  config.max_correction_fraction_per_pass = 0.5;
  orbslam3_multi::OptimizationManager optimizer(config);
  const auto proposal = optimizer.Optimize(graph.problem);
  EXPECT_EQ(proposal.status, orbslam3_multi::OptimizationSolverStatus::MaxIterations);
  EXPECT_LT(proposal.final_error.translation_m, proposal.initial_error.translation_m);

  orbslam3_multi::OptimizationValidator validator(config);
  const auto validation = validator.Validate(graph.problem, proposal);
  EXPECT_EQ(
    validation.decision,
    orbslam3_multi::ValidationDecision::AcceptPartialRetry);
}

TEST(LoopOptimization, BuildsAndSolvesJointCovisibleRelativeGraph)
{
  std::map<orbslam3_multi::RawSubmapId,
    orbslam3_multi::RawSubmapPoseSnapshot> snapshots;
  std::map<orbslam3_multi::RawKeyFrameId,
    orbslam3_multi::GlobalPoseRecord> poses;
  for (uint32_t drone = 1; drone <= 2; ++drone) {
    orbslam3_multi::RawSubmapPoseSnapshot snapshot;
    snapshot.submap_id = {drone, 1};
    snapshot.submap_revision = 10 + drone;
    for (uint64_t id = 0; id < 8; ++id) {
      orbslam3_multi::RawKeyFramePoseInput raw;
      raw.id = {drone, 1, id};
      raw.raw_revision = id + 1;
      raw.local_pose = MakePose(static_cast<double>(id));
      snapshot.keyframes.push_back(raw);

      orbslam3_multi::GlobalPoseRecord pose;
      pose.keyframe_id = raw.id;
      pose.world_pose = MakePose(
        static_cast<double>(id), drone == 1 ? 0.0 : 2.0);
      pose.raw_world_pose = pose.world_pose;
      pose.pose_revision = id + 1;
      pose.active = true;
      pose.hard_fiducial = id == 0;
      poses[raw.id] = pose;
    }
    snapshots[snapshot.submap_id] = snapshot;
  }

  orbslam3_multi::LoopTaskComputation computation;
  computation.decision =
    orbslam3_multi::LoopTaskDecisionKind::OptimizationEvidence;
  computation.task.task_id = 90;
  computation.task.query_keyframe_id = {1, 1, 7};
  orbslam3_multi::LoopGeometryResult geometry;
  geometry.query_keyframe_id = {1, 1, 7};
  geometry.candidate_keyframe_id = {2, 1, 7};
  geometry.query_submap_id = {1, 1};
  geometry.candidate_submap_id = {2, 1};
  geometry.accepted = true;
  geometry.fusion_compatible = false;
  geometry.inliers = 25;
  geometry.mean_residual_m = 0.02;
  geometry.candidate_local_T_query_local = Eigen::Isometry3d::Identity();
  computation.geometry_results.push_back(geometry);

  orbslam3_multi::CovisibilityDatabase covisibility;
  orbslam3_multi::CovisibilityPatch patch;
  orbslam3_multi::CovisibilityEdge edge;
  edge.kf_a = {1, 1, 3};
  edge.kf_b = {2, 1, 3};
  edge.source = orbslam3_multi::CovisibilityEdgeSource::ServerLoopGeometric;
  edge.support = 20;
  edge.information_weight = 4.0;
  edge.relative_pose_measured = MakePose(0.0, 2.0);
  edge.relative_pose_current = edge.relative_pose_measured;
  patch.upserts.push_back(edge);
  ASSERT_TRUE(covisibility.ApplyPatch(patch).committed);

  orbslam3_multi::PoseGraphBuilder builder;
  orbslam3_multi::LoopPipelineConfig loop_config;
  const auto graph = builder.BuildLoop(
    computation, snapshots, poses, 50, covisibility, loop_config);
  ASSERT_TRUE(graph.success) << graph.reason;
  EXPECT_EQ(graph.problem.submap_windows.size(), 2U);
  EXPECT_EQ(graph.problem.loop_edges.size(), 1U);
  EXPECT_GE(graph.problem.covisibility_edges.size(), 1U);

  orbslam3_multi::OptimizationManager optimizer;
  const auto proposal = optimizer.Optimize(graph.problem);
  ASSERT_EQ(proposal.status, orbslam3_multi::OptimizationSolverStatus::Converged)
    << proposal.reason;
  EXPECT_LT(
    proposal.final_error.translation_m,
    proposal.initial_error.translation_m);
  EXPECT_LE(
    proposal.final_error.translation_m,
    loop_config.fusion_translation_threshold_m);
  EXPECT_LE(proposal.iterations, 32U);

  orbslam3_multi::OptimizationValidator validator;
  const auto validation = validator.Validate(graph.problem, proposal);
  EXPECT_EQ(validation.decision, orbslam3_multi::ValidationDecision::AcceptFull)
    << validation.reason;
  for (const auto & control : proposal.controls) {
    if (control.id.local_kf_id == 0U) {
      EXPECT_TRUE(orbslam3_multi::PosesNear(
        control.world_pose, poses.at(control.id).world_pose, 1e-8, 1e-8));
    }
  }
}

TEST(LoopOptimization, BuildsAndValidatesThreeCoherentCurrentLoopRegions)
{
  std::map<orbslam3_multi::RawSubmapId,
    orbslam3_multi::RawSubmapPoseSnapshot> snapshots;
  std::map<orbslam3_multi::RawKeyFrameId,
    orbslam3_multi::GlobalPoseRecord> poses;
  for (uint32_t drone = 1; drone <= 2; ++drone) {
    orbslam3_multi::RawSubmapPoseSnapshot snapshot;
    snapshot.submap_id = {drone, 2};
    snapshot.submap_revision = drone;
    for (uint64_t id = 0; id < 10; ++id) {
      orbslam3_multi::RawKeyFramePoseInput raw;
      raw.id = {drone, 2, id};
      raw.raw_revision = id + 1U;
      raw.local_pose = MakePose(static_cast<double>(id));
      snapshot.keyframes.push_back(raw);
      orbslam3_multi::GlobalPoseRecord pose;
      pose.keyframe_id = raw.id;
      pose.world_pose = MakePose(static_cast<double>(id), drone == 1 ? 0.0 : 1.5);
      pose.raw_world_pose = pose.world_pose;
      pose.pose_revision = id + 1U;
      pose.active = true;
      pose.hard_fiducial = id == 0U;
      poses[raw.id] = pose;
    }
    snapshots[snapshot.submap_id] = snapshot;
  }

  orbslam3_multi::LoopTaskComputation computation;
  computation.decision = orbslam3_multi::LoopTaskDecisionKind::OptimizationEvidence;
  computation.task.task_id = 94U;
  for (size_t index = 0; index < 3U; ++index) {
    const uint64_t local_id = 5U + static_cast<uint64_t>(index) * 2U;
    orbslam3_multi::LoopGeometryResult geometry;
    geometry.query_keyframe_id = {1, 2, local_id};
    geometry.candidate_keyframe_id = {2, 2, local_id};
    geometry.query_submap_id = {1, 2};
    geometry.candidate_submap_id = {2, 2};
    geometry.accepted = true;
    geometry.matches = 40U;
    geometry.inliers = 32U - index;
    geometry.mean_residual_m = 0.02;
    geometry.candidate_local_T_query_local = Eigen::Isometry3d::Identity();
    computation.geometry_results.push_back(geometry);
    computation.optimization_geometry_indices.push_back(index);
  }

  orbslam3_multi::PoseGraphBuilder builder;
  orbslam3_multi::CovisibilityDatabase covisibility;
  orbslam3_multi::LoopPipelineConfig loop_config;
  const auto graph = builder.BuildLoop(
    computation, snapshots, poses, 60U, covisibility, loop_config);
  ASSERT_TRUE(graph.success) << graph.reason;
  ASSERT_EQ(graph.problem.loop_edges.size(), 3U);
  EXPECT_EQ(graph.problem.loop_edges[0].source_geometry_index, 0U);
  EXPECT_EQ(graph.problem.loop_edges[1].source_geometry_index, 1U);
  EXPECT_EQ(graph.problem.loop_edges[2].source_geometry_index, 2U);

  orbslam3_multi::OptimizationManager optimizer;
  const auto proposal = optimizer.Optimize(graph.problem);
  ASSERT_EQ(proposal.final_loop_errors.size(), 3U) << proposal.reason;
  orbslam3_multi::OptimizationValidator validator;
  const auto validation = validator.Validate(graph.problem, proposal);
  EXPECT_EQ(validation.decision, orbslam3_multi::ValidationDecision::AcceptFull)
    << validation.reason;
  for (const auto & error : proposal.final_loop_errors) {
    EXPECT_LE(error.translation_m, loop_config.fusion_translation_threshold_m);
    EXPECT_LE(error.rotation_rad, loop_config.fusion_rotation_threshold_rad);
  }
}

TEST(LoopOptimization, DenseNativeCovisibilityKeepsSparseControlGraph)
{
  constexpr uint64_t kKeyFramesPerSubmap = 60U;
  std::map<orbslam3_multi::RawSubmapId,
    orbslam3_multi::RawSubmapPoseSnapshot> snapshots;
  std::map<orbslam3_multi::RawKeyFrameId,
    orbslam3_multi::GlobalPoseRecord> poses;
  orbslam3_multi::CovisibilityPatch patch;
  for (uint32_t drone = 1; drone <= 2; ++drone) {
    orbslam3_multi::RawSubmapPoseSnapshot snapshot;
    snapshot.submap_id = {drone, 1};
    snapshot.submap_revision = drone;
    for (uint64_t id = 0; id < kKeyFramesPerSubmap; ++id) {
      orbslam3_multi::RawKeyFramePoseInput raw;
      raw.id = {drone, 1, id};
      raw.raw_revision = id + 1U;
      raw.local_pose = MakePose(0.2 * static_cast<double>(id));
      snapshot.keyframes.push_back(raw);

      orbslam3_multi::GlobalPoseRecord pose;
      pose.keyframe_id = raw.id;
      pose.world_pose = MakePose(
        0.2 * static_cast<double>(id), drone == 1U ? 0.0 : 2.0);
      pose.raw_world_pose = pose.world_pose;
      pose.pose_revision = id + 1U;
      pose.active = true;
      pose.hard_fiducial = id == 0U;
      poses[raw.id] = pose;

      for (uint64_t gap = 1U; gap <= 12U && id >= gap; ++gap) {
        orbslam3_multi::CovisibilityEdge edge;
        edge.kf_a = {drone, 1, id - gap};
        edge.kf_b = raw.id;
        edge.source = orbslam3_multi::CovisibilityEdgeSource::Orbslam3Native;
        edge.support = 40U - gap;
        edge.information_weight = static_cast<double>(edge.support);
        edge.relative_pose_measured = MakePose(0.2 * static_cast<double>(gap));
        edge.relative_pose_current = edge.relative_pose_measured;
        patch.upserts.push_back(edge);
      }
    }
    snapshots[snapshot.submap_id] = std::move(snapshot);
  }
  orbslam3_multi::CovisibilityDatabase covisibility;
  ASSERT_TRUE(covisibility.ApplyPatch(patch).committed);

  orbslam3_multi::LoopTaskComputation computation;
  computation.decision = orbslam3_multi::LoopTaskDecisionKind::OptimizationEvidence;
  computation.task.task_id = 93;
  computation.task.query_keyframe_id = {1, 1, kKeyFramesPerSubmap - 1U};
  orbslam3_multi::LoopGeometryResult geometry;
  geometry.query_keyframe_id = computation.task.query_keyframe_id;
  geometry.candidate_keyframe_id = {2, 1, kKeyFramesPerSubmap - 1U};
  geometry.query_submap_id = {1, 1};
  geometry.candidate_submap_id = {2, 1};
  geometry.accepted = true;
  geometry.inliers = 40U;
  geometry.mean_residual_m = 0.02;
  geometry.candidate_local_T_query_local = Eigen::Isometry3d::Identity();
  computation.geometry_results.push_back(geometry);

  orbslam3_multi::PoseGraphBuilder builder;
  orbslam3_multi::LoopPipelineConfig loop_config;
  const auto graph = builder.BuildLoop(
    computation, snapshots, poses, 90, covisibility, loop_config);
  ASSERT_TRUE(graph.success) << graph.reason;
  EXPECT_LE(graph.problem.control_indices.size(), 40U);
  EXPECT_LE(
    graph.problem.covisibility_edges.size(),
    graph.problem.control_indices.size() * 6U);
  EXPECT_LT(
    graph.problem.control_indices.size(),
    graph.problem.keyframes.size() / 2U);
}

TEST(LoopOptimization, RejectsConstraintThatCannotMoveHardEndpoints)
{
  orbslam3_multi::PoseGraphProblem problem;
  problem.kind = orbslam3_multi::PoseGraphProblemKind::LoopRelative;
  problem.loop_task.task_id = 91;
  problem.loop_translation_threshold_m = 0.35;
  problem.loop_rotation_threshold_rad = 0.25;
  for (uint32_t drone = 1; drone <= 2; ++drone) {
    orbslam3_multi::PoseGraphKeyFrame keyframe;
    keyframe.id = {drone, 1, 0};
    keyframe.current_world_pose = MakePose(0.0, drone == 1 ? 0.0 : 2.0);
    keyframe.raw_local_pose = MakePose(0.0);
    keyframe.control = true;
    keyframe.fixed = true;
    problem.control_indices.push_back(problem.keyframes.size());
    problem.keyframes.push_back(keyframe);
  }
  orbslam3_multi::PoseGraphEdge loop;
  loop.from_index = 1;
  loop.to_index = 0;
  loop.relative_raw_pose = MakePose(0.0);
  loop.kind = orbslam3_multi::PoseGraphEdgeKind::CurrentLoop;
  loop.information_weight = 40.0;
  problem.loop_edges.push_back(loop);

  orbslam3_multi::OptimizationManager optimizer;
  const auto proposal = optimizer.Optimize(problem);
  orbslam3_multi::OptimizationValidator validator;
  const auto validation = validator.Validate(problem, proposal);
  EXPECT_EQ(validation.decision, orbslam3_multi::ValidationDecision::HardFailure);
  EXPECT_EQ(validation.reason, "relative_loop_still_above_threshold");
}

TEST(LoopOptimization, IncludesSoftAuthorityBetweenHardFiducials)
{
  std::map<orbslam3_multi::RawSubmapId,
    orbslam3_multi::RawSubmapPoseSnapshot> snapshots;
  std::map<orbslam3_multi::RawKeyFrameId,
    orbslam3_multi::GlobalPoseRecord> poses;
  const auto add_submap = [&](uint32_t drone, double y, bool bounded) {
      orbslam3_multi::RawSubmapPoseSnapshot snapshot;
      snapshot.submap_id = {drone, 1};
      snapshot.submap_revision = drone;
      for (uint64_t id = 0; id < 7; ++id) {
        orbslam3_multi::RawKeyFramePoseInput raw;
        raw.id = {drone, 1, id};
        raw.raw_revision = id + 1;
        raw.local_pose = MakePose(static_cast<double>(id));
        snapshot.keyframes.push_back(raw);
        orbslam3_multi::GlobalPoseRecord pose;
        pose.keyframe_id = raw.id;
        pose.world_pose = MakePose(static_cast<double>(id), y);
        pose.raw_world_pose = pose.world_pose;
        pose.pose_revision = id + 1;
        pose.active = true;
        pose.hard_fiducial = id == 0 || (bounded && id == 6);
        poses[raw.id] = pose;
      }
      snapshots[snapshot.submap_id] = snapshot;
    };
  add_submap(1, 0.0, true);
  add_submap(2, 2.0, false);
  add_submap(3, 4.0, false);

  orbslam3_multi::LoopTaskComputation computation;
  computation.decision = orbslam3_multi::LoopTaskDecisionKind::OptimizationEvidence;
  computation.task.task_id = 92;
  computation.task.query_keyframe_id = {2, 1, 6};
  orbslam3_multi::LoopGeometryResult geometry;
  geometry.query_keyframe_id = {2, 1, 6};
  geometry.candidate_keyframe_id = {3, 1, 6};
  geometry.query_submap_id = {2, 1};
  geometry.candidate_submap_id = {3, 1};
  geometry.accepted = true;
  geometry.inliers = 25;
  geometry.candidate_local_T_query_local = Eigen::Isometry3d::Identity();
  computation.geometry_results.push_back(geometry);

  orbslam3_multi::LoopAnchorDependencySnapshot dependency;
  dependency.child_submap_id = {2, 1};
  dependency.parent_submap_id = {1, 1};
  dependency.child_control_keyframe_id = {2, 1, 0};
  dependency.parent_control_keyframe_id = {1, 1, 3};
  dependency.parent_control_T_child_control = MakePose(-3.0, 2.0);
  std::map<orbslam3_multi::RawSubmapId,
    orbslam3_multi::LoopAnchorDependencySnapshot> dependencies;
  dependencies[dependency.child_submap_id] = dependency;

  orbslam3_multi::PoseGraphBuilder builder;
  orbslam3_multi::CovisibilityDatabase covisibility;
  orbslam3_multi::LoopPipelineConfig loop_config;
  const auto graph = builder.BuildLoop(
    computation, snapshots, poses, 80, covisibility, loop_config, dependencies);
  ASSERT_TRUE(graph.success) << graph.reason;
  EXPECT_EQ(graph.problem.submap_windows.size(), 3U);
  EXPECT_TRUE(std::any_of(
    graph.problem.covisibility_edges.begin(),
    graph.problem.covisibility_edges.end(),
    [](const auto & graph_edge) {
      return graph_edge.kind == orbslam3_multi::PoseGraphEdgeKind::PriorLoop;
    }));
  const auto parent_window = std::find_if(
    graph.problem.submap_windows.begin(), graph.problem.submap_windows.end(),
    [](const auto & window) {
      return window.submap_id == orbslam3_multi::RawSubmapId{1, 1};
    });
  ASSERT_NE(parent_window, graph.problem.submap_windows.end());
  EXPECT_EQ(parent_window->first_keyframe_id.local_kf_id, 0U);
  EXPECT_EQ(parent_window->last_keyframe_id.local_kf_id, 6U);
}

TEST(LoopOptimization, RejectsLocallyGoodLoopThatBreaksPriorFusion)
{
  orbslam3_multi::PoseGraphProblem problem;
  problem.kind = orbslam3_multi::PoseGraphProblemKind::LoopRelative;
  problem.loop_translation_threshold_m = 0.35;
  problem.loop_rotation_threshold_rad = 0.25;
  problem.structural_prior_loop_increase_m = 0.50;
  problem.structural_prior_loop_increase_rad = 0.35;
  for (uint32_t drone = 1; drone <= 2; ++drone) {
    orbslam3_multi::PoseGraphKeyFrame keyframe;
    keyframe.id = {drone, 0, 1};
    keyframe.current_world_pose = MakePose(static_cast<double>(drone));
    keyframe.raw_local_pose = MakePose(0.0);
    keyframe.control = true;
    problem.control_indices.push_back(problem.keyframes.size());
    problem.keyframes.push_back(keyframe);
  }
  AddCurrentLoopEdge(problem);
  orbslam3_multi::OptimizationProposal proposal;
  proposal.status = orbslam3_multi::OptimizationSolverStatus::Converged;
  proposal.controls = {
    {problem.keyframes[0].id, MakePose(1.0)},
    {problem.keyframes[1].id, MakePose(1.1)}};
  proposal.keyframes = proposal.controls;
  proposal.initial_loop_errors.push_back({10.0, 1.0, 1.0});
  proposal.final_loop_errors.push_back({0.10, 0.05, 0.05});
  proposal.initial_cost = 100.0;
  proposal.final_cost = 1.0;
  proposal.edge_residuals.push_back({
    orbslam3_multi::PoseGraphEdgeKind::PriorLoop,
    problem.keyframes[0].id, problem.keyframes[1].id,
    {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}});

  orbslam3_multi::OptimizationValidator validator;
  const auto validation = validator.Validate(problem, proposal);
  EXPECT_EQ(validation.decision, orbslam3_multi::ValidationDecision::HardFailure);
  EXPECT_EQ(validation.reason, "prior_loop_structure_degraded");
  EXPECT_EQ(validation.structural_edges_checked, 1U);
}

TEST(LoopOptimization, AllowsRigidSoftMotionThatPreservesPriorFusion)
{
  orbslam3_multi::PoseGraphProblem problem;
  problem.kind = orbslam3_multi::PoseGraphProblemKind::LoopRelative;
  problem.loop_translation_threshold_m = 0.35;
  problem.loop_rotation_threshold_rad = 0.25;
  problem.structural_prior_loop_increase_m = 0.50;
  problem.structural_prior_loop_increase_rad = 0.35;
  for (uint32_t drone = 1; drone <= 2; ++drone) {
    orbslam3_multi::PoseGraphKeyFrame keyframe;
    keyframe.id = {drone, 0, 1};
    keyframe.current_world_pose = MakePose(static_cast<double>(drone));
    keyframe.control = true;
    problem.control_indices.push_back(problem.keyframes.size());
    problem.keyframes.push_back(keyframe);
  }
  AddCurrentLoopEdge(problem);
  orbslam3_multi::OptimizationProposal proposal;
  proposal.status = orbslam3_multi::OptimizationSolverStatus::Converged;
  proposal.controls = {
    {problem.keyframes[0].id, MakePose(101.0)},
    {problem.keyframes[1].id, MakePose(102.0)}};
  proposal.keyframes = proposal.controls;
  proposal.initial_loop_errors.push_back({10.0, 1.0, 1.0});
  proposal.final_loop_errors.push_back({0.10, 0.05, 0.05});
  proposal.initial_cost = 100.0;
  proposal.final_cost = 1.0;
  proposal.edge_residuals.push_back({
    orbslam3_multi::PoseGraphEdgeKind::PriorLoop,
    problem.keyframes[0].id, problem.keyframes[1].id,
    {0.01, 0.01, 0.01}, {0.01, 0.01, 0.01}});

  orbslam3_multi::OptimizationValidator validator;
  const auto validation = validator.Validate(problem, proposal);
  EXPECT_EQ(validation.decision, orbslam3_multi::ValidationDecision::AcceptFull);
}

TEST(LoopOptimization, RejectsHardCorridorDisplacement)
{
  orbslam3_multi::PoseGraphProblem problem;
  problem.kind = orbslam3_multi::PoseGraphProblemKind::LoopRelative;
  problem.loop_translation_threshold_m = 0.35;
  problem.loop_rotation_threshold_rad = 0.25;
  problem.hard_corridor_max_translation_m = 5.0;
  problem.hard_corridor_max_rotation_rad = 0.3490658503988659;
  for (uint64_t id = 0; id < 2; ++id) {
    orbslam3_multi::PoseGraphKeyFrame keyframe;
    keyframe.id = {1, 0, id};
    keyframe.current_world_pose = MakePose(static_cast<double>(id));
    keyframe.control = true;
    keyframe.hard_corridor = true;
    keyframe.hard_corridor_reference_pose = keyframe.current_world_pose;
    keyframe.hard_corridor_alpha = 0.5;
    problem.control_indices.push_back(problem.keyframes.size());
    problem.keyframes.push_back(keyframe);
  }
  AddCurrentLoopEdge(problem);
  orbslam3_multi::OptimizationProposal proposal;
  proposal.status = orbslam3_multi::OptimizationSolverStatus::Converged;
  proposal.controls = {
    {problem.keyframes[0].id, MakePose(6.0)},
    {problem.keyframes[1].id, MakePose(7.0)}};
  proposal.keyframes = proposal.controls;
  proposal.initial_loop_errors.push_back({2.0, 0.5, 0.5});
  proposal.final_loop_errors.push_back({0.10, 0.05, 0.05});
  proposal.initial_cost = 10.0;
  proposal.final_cost = 1.0;

  orbslam3_multi::OptimizationValidator validator;
  const auto validation = validator.Validate(problem, proposal);
  EXPECT_EQ(validation.decision, orbslam3_multi::ValidationDecision::HardFailure);
  EXPECT_EQ(validation.reason, "hard_corridor_displacement_exceeded");
  EXPECT_EQ(validation.hard_corridor_keyframes_checked, 1U);
}

TEST(LoopOptimization, AllowsImprovementOfPreexistingCorridorExcess)
{
  orbslam3_multi::PoseGraphProblem problem;
  problem.kind = orbslam3_multi::PoseGraphProblemKind::LoopRelative;
  problem.loop_translation_threshold_m = 0.35;
  problem.loop_rotation_threshold_rad = 0.25;
  problem.hard_corridor_max_translation_m = 5.0;
  problem.hard_corridor_max_rotation_rad = 0.3490658503988659;
  for (uint64_t id = 0; id < 2; ++id) {
    orbslam3_multi::PoseGraphKeyFrame keyframe;
    keyframe.id = {1, 0, id};
    keyframe.current_world_pose = MakePose(6.0 + static_cast<double>(id));
    keyframe.control = true;
    keyframe.hard_corridor = true;
    keyframe.hard_corridor_reference_pose = MakePose(static_cast<double>(id));
    keyframe.hard_corridor_alpha = 0.5;
    problem.control_indices.push_back(problem.keyframes.size());
    problem.keyframes.push_back(keyframe);
  }
  AddCurrentLoopEdge(problem);

  orbslam3_multi::OptimizationProposal proposal;
  proposal.status = orbslam3_multi::OptimizationSolverStatus::Converged;
  proposal.controls = {
    {problem.keyframes[0].id, MakePose(5.5)},
    {problem.keyframes[1].id, MakePose(6.5)}};
  proposal.keyframes = proposal.controls;
  proposal.initial_loop_errors.push_back({2.0, 0.5, 0.5});
  proposal.final_loop_errors.push_back({0.10, 0.05, 0.05});
  proposal.initial_cost = 10.0;
  proposal.final_cost = 1.0;

  orbslam3_multi::OptimizationValidator validator;
  const auto validation = validator.Validate(problem, proposal);
  EXPECT_EQ(validation.decision, orbslam3_multi::ValidationDecision::AcceptFull);
  EXPECT_NEAR(validation.max_corridor_translation_excess_before_m, 1.0, 1e-9);
  EXPECT_NEAR(validation.max_corridor_translation_excess_after_m, 0.5, 1e-9);
}

}  // namespace
