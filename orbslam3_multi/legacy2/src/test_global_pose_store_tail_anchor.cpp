#include "orbslam3_multi/global_pose_store.hpp"
#include "orbslam3_multi/optimization_manager.hpp"

#include <cmath>
#include <initializer_list>
#include <iostream>
#include <utility>

namespace
{

geometry_msgs::msg::Pose MakePose(double x)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.orientation.w = 1.0;
    return pose;
}

orbslam3_msgs::msg::OrbKeyFrame MakeKeyFrame(uint64_t id, double x)
{
    orbslam3_msgs::msg::OrbKeyFrame keyframe;
    keyframe.id = id;
    keyframe.pose = MakePose(x);
    return keyframe;
}

orbslam3_msgs::msg::OrbMap MakeMap(
    uint64_t sequence,
    std::initializer_list<std::pair<uint64_t, double>> keyframes)
{
    orbslam3_msgs::msg::OrbMap map;
    map.drone_id = 2;
    map.drone_name = "dron_2";
    map.map_epoch = 0;
    map.map_sequence = sequence;
    map.map_frame = "dron_2_orb_map";
    for (const auto& [id, x] : keyframes)
    {
        map.keyframes.push_back(MakeKeyFrame(id, x));
    }
    return map;
}

bool Near(double a, double b)
{
    return std::abs(a - b) <= 1e-9;
}

bool Expect(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "[test_global_pose_store_tail_anchor] "
                  << message << '\n';
        return false;
    }
    return true;
}

}  // namespace

int main()
{
    using namespace orbslam3_multi;

    bool ok = true;
    RawMapDatabase raw_db;
    GlobalPoseStore pose_store;
    const RawSubmapId submap_id{2, 0};
    const RawKeyFrameId kf1{2, 0, 1};
    const RawKeyFrameId kf2{2, 0, 2};
    const RawKeyFrameId kf3{2, 0, 3};
    const RawKeyFrameId kf4{2, 0, 4};

    raw_db.InsertDelta(1, MakeMap(1, {{0, 0.0}, {1, 1.0}}));
    ok &= Expect(
        pose_store.AnchorSubmap(
            submap_id,
            Eigen::Matrix4d::Identity(),
            raw_db,
            "TEST_INITIAL_ANCHOR").success,
        "initial anchor failed");
    std::string reason;
    ok &= Expect(
        pose_store.SetActiveTailAnchor(
            kf1, 0, "TEST_INITIAL_TAIL", false, &reason),
        "initial active tail anchor failed");

    raw_db.InsertDelta(2, MakeMap(2, {{2, 2.0}}));
    ok &= Expect(
        pose_store.RegisterNewKeyFrameIfAnchored(
            kf2, raw_db, "TEST_NEW_KF").status ==
            GlobalPoseNewKeyFrameStatus::CorrectionInherited,
        "kf2 was not registered as derived tail");
    const auto kf2_before = pose_store.GetWorldPose(kf2);
    ok &= Expect(kf2_before && Near((*kf2_before)(0, 3), 2.0),
                 "kf2 initial world pose is wrong");

    const auto snapshot_result = raw_db.InsertFullSnapshot(
        3,
        MakeMap(3, {{0, 9.0}, {1, 10.0}, {2, 11.0}, {3, 12.0}}));
    pose_store.ReconcileAfterRawIngestResult(
        snapshot_result, raw_db, "TEST_RAW_RECONCILE");

    const auto kf1_after_raw = pose_store.GetWorldPose(kf1);
    const auto kf2_after_raw = pose_store.GetWorldPose(kf2);
    ok &= Expect(kf1_after_raw && Near((*kf1_after_raw)(0, 3), 1.0),
                 "accepted kf1 moved after raw update");
    ok &= Expect(kf2_after_raw && Near((*kf2_after_raw)(0, 3), 2.0),
                 "derived kf2 lost its relative pose after raw update");

    ok &= Expect(
        pose_store.RegisterNewKeyFrameIfAnchored(
            kf3, raw_db, "TEST_NEW_AFTER_RAW").status !=
            GlobalPoseNewKeyFrameStatus::MissingRawKeyFrame,
        "kf3 registration failed");
    const auto kf3_world = pose_store.GetWorldPose(kf3);
    ok &= Expect(kf3_world && Near((*kf3_world)(0, 3), 3.0),
                 "kf3 did not use the active accepted reference");

    Eigen::Matrix4d optimized_kf3 = Eigen::Matrix4d::Identity();
    optimized_kf3(0, 3) = 4.0;
    ok &= Expect(
        pose_store.SetOptimizedKeyFramePose(
            kf3, optimized_kf3, raw_db, "TEST_OPT").success,
        "kf3 optimization failed");
    ok &= Expect(
        pose_store.SetActiveTailAnchor(
            kf3, 77, "TEST_OPT_ACTIVE_TAIL", true, &reason),
        "optimized active tail anchor failed");

    raw_db.InsertDelta(4, MakeMap(4, {{4, 13.0}}));
    ok &= Expect(
        pose_store.RegisterNewKeyFrameIfAnchored(
            kf4, raw_db, "TEST_NEW_AFTER_OPT").status ==
            GlobalPoseNewKeyFrameStatus::CorrectionInherited,
        "kf4 did not use optimized active tail authority");
    const auto kf4_world = pose_store.GetWorldPose(kf4);
    ok &= Expect(kf4_world && Near((*kf4_world)(0, 3), 5.0),
                 "kf4 active-tail projection is wrong");

    const auto second_snapshot_result = raw_db.InsertFullSnapshot(
        5,
        MakeMap(
            5,
            {{0, 9.0}, {1, 10.0}, {2, 11.0}, {3, 20.0}, {4, 22.0}}));
    pose_store.ReconcileAfterRawIngestResult(
        second_snapshot_result, raw_db, "TEST_SECOND_RAW_RECONCILE");
    const auto accepted_kf3_after_raw = pose_store.GetWorldPose(kf3);
    const auto derived_kf4_after_raw = pose_store.GetWorldPose(kf4);
    ok &= Expect(
        accepted_kf3_after_raw &&
        Near((*accepted_kf3_after_raw)(0, 3), 4.0),
        "accepted kf3 moved after second raw update");
    ok &= Expect(
        derived_kf4_after_raw &&
        Near((*derived_kf4_after_raw)(0, 3), 6.0),
        "derived kf4 was not reprojected after raw update");

    const auto derived_before_backup =
        pose_store.GetPoseStoreStats().derived_tail_keyframes;
    const auto backup = pose_store.CreateApplyBackup(78, {kf3, kf4});
    ok &= Expect(backup.success, "backup failed");
    Eigen::Matrix4d moved_kf4 = Eigen::Matrix4d::Identity();
    moved_kf4(0, 3) = 8.0;
    ok &= Expect(
        pose_store.SetPropagatedKeyFramePose(
            kf4, moved_kf4, raw_db, "TEST_TEMP_APPLY").success,
        "temporary kf4 apply failed");
    ok &= Expect(
        pose_store.SetActiveTailAnchor(
            kf4, 78, "TEST_TEMP_ACTIVE_TAIL", true, &reason),
        "temporary active tail update failed");
    const auto rollback = pose_store.RestoreApplyBackup(78);
    ok &= Expect(rollback.success, "rollback failed");
    const auto restored_kf4 = pose_store.GetWorldPose(kf4);
    const auto restored_tail = pose_store.GetActiveTailAnchor(submap_id);
    ok &= Expect(restored_kf4 && Near((*restored_kf4)(0, 3), 6.0),
                 "rollback did not restore kf4");
    ok &= Expect(
        pose_store.GetPoseStoreStats().derived_tail_keyframes ==
            derived_before_backup,
        "rollback did not restore derived-tail membership");
    ok &= Expect(
        restored_tail &&
        restored_tail->reference_keyframe_id == kf3 &&
        restored_tail->source_task_id == 77,
        "rollback did not restore active tail anchor");

    RawMapDatabase pending_raw_db;
    GlobalPoseStore pending_pose_store;
    pending_raw_db.InsertDelta(
        1,
        MakeMap(
            1,
            {{0, 0.0}, {1, 1.0}, {2, 2.0}, {3, 3.0}, {4, 4.0}}));
    ok &= Expect(
        pending_pose_store.AnchorSubmap(
            submap_id,
            Eigen::Matrix4d::Identity(),
            pending_raw_db,
            "TEST_PENDING_INITIAL_ANCHOR").success,
        "pending-tail initial anchor failed");

    PoseGraphProblem graph;
    graph.task_id = 901;
    graph.target_keyframe_id = kf1;

    OptimizationDryRunResult dry_run;
    dry_run.task_id = graph.task_id;
    dry_run.task_type = "TEST_PENDING_TAIL";
    dry_run.success = true;
    dry_run.useful = true;
    dry_run.precheck_ok = true;
    dry_run.initial_cost = 10.0;
    dry_run.final_cost = 1.0;
    dry_run.before_error_t = 9.0;
    dry_run.after_error_t = 0.0;
    dry_run.solver_iterations = 2;
    dry_run.max_delta_t = 45.0;
    dry_run.max_delta_yaw = 2.4;

    OptimizationPoseProposal target_proposal;
    target_proposal.keyframe_id = kf1;
    target_proposal.before_world_T_kf = Eigen::Matrix4d::Identity();
    target_proposal.before_world_T_kf(0, 3) = 1.0;
    target_proposal.proposed_world_T_kf = Eigen::Matrix4d::Identity();
    target_proposal.proposed_world_T_kf(0, 3) = 10.0;
    target_proposal.variable_vertex = true;
    target_proposal.delta_t_m = 9.0;
    dry_run.proposed_vertex_poses.push_back(target_proposal);
    dry_run.moved_kfs = 1;

    PendingTailFiducialConstraint pending_constraint;
    pending_constraint.keyframe_id = kf3;
    pending_constraint.target_world_T_kf = Eigen::Matrix4d::Identity();
    pending_constraint.target_world_T_kf(0, 3) = 12.0;
    pending_constraint.arrival_id = 8;
    pending_constraint.fiducial_id = 1;

    OptimizationManager optimization_manager;
    const auto pending_apply =
        optimization_manager.ApplyCandidateResult(
            dry_run,
            graph,
            pending_raw_db,
            pending_pose_store,
            false,
            {pending_constraint});
    ok &= Expect(pending_apply.applied, "pending-tail apply failed");
    ok &= Expect(
        pending_apply.pending_tail_fiducial_controls == 1,
        "pending-tail fiducial control was not counted");
    ok &= Expect(
        pending_apply.pending_tail_refined_kfs == 2,
        "pending-tail refined segment count is wrong");
    ok &= Expect(
        pending_apply.pending_tail_derived_kfs == 1,
        "pending-tail derived segment count is wrong");

    const auto pending_kf1 = pending_pose_store.GetWorldPose(kf1);
    const auto pending_kf2 = pending_pose_store.GetWorldPose(kf2);
    const auto pending_kf3 = pending_pose_store.GetWorldPose(kf3);
    const auto pending_kf4 = pending_pose_store.GetWorldPose(kf4);
    ok &= Expect(
        pending_kf1 && Near((*pending_kf1)(0, 3), 10.0),
        "pending-tail graph target pose is wrong");
    ok &= Expect(
        pending_kf2 && Near((*pending_kf2)(0, 3), 11.0),
        "pending-tail interpolation is wrong");
    ok &= Expect(
        pending_kf3 && Near((*pending_kf3)(0, 3), 12.0),
        "pending-tail fiducial control pose is wrong");
    ok &= Expect(
        pending_kf4 && Near((*pending_kf4)(0, 3), 13.0),
        "pending-tail projection after final control is wrong");

    const auto pending_tail =
        pending_pose_store.GetActiveTailAnchor(submap_id);
    ok &= Expect(
        pending_tail &&
        pending_tail->reference_keyframe_id == kf3 &&
        pending_tail->source_task_id == graph.task_id,
        "pending-tail final control was not installed as active anchor");

    RawMapDatabase late_window_raw_db;
    GlobalPoseStore late_window_pose_store;
    const RawKeyFrameId kf0{2, 0, 0};
    late_window_raw_db.InsertDelta(
        1,
        MakeMap(
            1,
            {{0, 0.0}, {1, 1.0}, {2, 2.0}, {3, 3.0}, {4, 4.0}}));
    ok &= Expect(
        late_window_pose_store.AnchorSubmap(
            submap_id,
            Eigen::Matrix4d::Identity(),
            late_window_raw_db,
            "TEST_LATE_WINDOW_INITIAL_ANCHOR").success,
        "late-window initial anchor failed");

    PoseGraphProblem late_window_graph;
    late_window_graph.task_id = 902;
    late_window_graph.submap_id = submap_id;
    late_window_graph.target_keyframe_id = kf3;

    OptimizationDryRunResult late_window_dry_run;
    late_window_dry_run.task_id = late_window_graph.task_id;
    late_window_dry_run.task_type = "TEST_LATE_WINDOW";
    late_window_dry_run.success = true;
    late_window_dry_run.useful = true;
    late_window_dry_run.precheck_ok = true;
    late_window_dry_run.initial_cost = 10.0;
    late_window_dry_run.final_cost = 1.0;
    late_window_dry_run.before_error_t = 9.0;
    late_window_dry_run.after_error_t = 0.0;
    late_window_dry_run.solver_iterations = 2;
    late_window_dry_run.max_delta_t = 9.0;

    OptimizationPoseProposal anchor_proposal;
    anchor_proposal.keyframe_id = kf0;
    anchor_proposal.fixed_vertex = true;
    anchor_proposal.before_world_T_kf = Eigen::Matrix4d::Identity();
    anchor_proposal.proposed_world_T_kf = Eigen::Matrix4d::Identity();
    late_window_dry_run.proposed_vertex_poses.push_back(anchor_proposal);

    OptimizationPoseProposal late_target_proposal;
    late_target_proposal.keyframe_id = kf3;
    late_target_proposal.before_world_T_kf = Eigen::Matrix4d::Identity();
    late_target_proposal.before_world_T_kf(0, 3) = 3.0;
    late_target_proposal.proposed_world_T_kf = Eigen::Matrix4d::Identity();
    late_target_proposal.proposed_world_T_kf(0, 3) = 12.0;
    late_target_proposal.variable_vertex = true;
    late_target_proposal.delta_t_m = 9.0;
    late_window_dry_run.proposed_vertex_poses.push_back(late_target_proposal);
    late_window_dry_run.moved_kfs = 1;

    const auto late_window_apply =
        optimization_manager.ApplyCandidateResult(
            late_window_dry_run,
            late_window_graph,
            late_window_raw_db,
            late_window_pose_store,
            false,
            {});
    ok &= Expect(late_window_apply.applied, "late-window apply failed");
    ok &= Expect(
        late_window_apply.late_window_detected_kfs == 2,
        "late-window KFs were not detected");
    ok &= Expect(
        late_window_apply.late_window_refined_kfs == 2,
        "late-window KFs were not refined");
    ok &= Expect(
        late_window_apply.late_window_skipped_kfs == 0,
        "late-window KFs were unexpectedly skipped");

    const auto late_kf1 = late_window_pose_store.GetWorldPose(kf1);
    const auto late_kf2 = late_window_pose_store.GetWorldPose(kf2);
    const auto late_kf3 = late_window_pose_store.GetWorldPose(kf3);
    const auto late_kf4 = late_window_pose_store.GetWorldPose(kf4);
    ok &= Expect(
        late_kf1 && Near((*late_kf1)(0, 3), 4.0),
        "first late-window interpolation is wrong");
    ok &= Expect(
        late_kf2 && Near((*late_kf2)(0, 3), 8.0),
        "second late-window interpolation is wrong");
    ok &= Expect(
        late_kf3 && Near((*late_kf3)(0, 3), 12.0),
        "late-window target pose is wrong");
    ok &= Expect(
        late_kf4 && Near((*late_kf4)(0, 3), 13.0),
        "late-window right-tail projection is wrong");

    PoseGraphProblem large_correction_graph;
    large_correction_graph.task_id = 903;
    large_correction_graph.task_type = "TEST_LARGE_CORRECTION";
    large_correction_graph.submap_id = submap_id;
    large_correction_graph.target_keyframe_id = kf1;

    PoseGraphVertex fixed_vertex;
    fixed_vertex.keyframe_id = kf0;
    fixed_vertex.submap_id = submap_id;
    fixed_vertex.is_fixed = true;
    large_correction_graph.vertices.push_back(fixed_vertex);
    large_correction_graph.fixed_keyframes.push_back(kf0);

    PoseGraphVertex variable_vertex;
    variable_vertex.keyframe_id = kf1;
    variable_vertex.submap_id = submap_id;
    variable_vertex.initial_world_T_kf(0, 3) = 1.0;
    variable_vertex.is_variable = true;
    large_correction_graph.vertices.push_back(variable_vertex);
    large_correction_graph.variable_keyframes.push_back(kf1);

    PoseGraphEdge temporal_edge;
    temporal_edge.from_keyframe_id = kf0;
    temporal_edge.to_keyframe_id = kf1;
    temporal_edge.relative_T_from_to(0, 3) = 1.0;
    temporal_edge.weight = 1.0;
    large_correction_graph.edges.push_back(temporal_edge);

    PoseGraphPrior target_prior;
    target_prior.keyframe_id = kf1;
    target_prior.prior_type = PoseGraphPriorType::FiducialTarget;
    target_prior.target_world_T_kf(0, 3) = 51.0;
    target_prior.weight_translation = 5000.0;
    target_prior.weight_rotation = 2500.0;
    large_correction_graph.priors.push_back(target_prior);

    const auto large_correction_dry_run =
        optimization_manager.RunDryRunGraphOnly(large_correction_graph);
    ok &= Expect(
        large_correction_dry_run.success,
        "large-correction dry-run failed");
    ok &= Expect(
        large_correction_dry_run.max_delta_t > 30.0,
        "large-correction regression did not exceed the removed threshold");
    ok &= Expect(
        large_correction_dry_run.useful,
        "large-correction dry-run was rejected because of movement magnitude");

    return ok ? 0 : 1;
}
