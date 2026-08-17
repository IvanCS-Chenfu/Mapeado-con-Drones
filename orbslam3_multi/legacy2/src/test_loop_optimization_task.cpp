#include "orbslam3_multi/loop_optimization_task.hpp"
#include "orbslam3_multi/loop_detector.hpp"
#include "orbslam3_multi/optimization_manager.hpp"
#include "orbslam3_multi/pose_graph_builder.hpp"

#include <cmath>
#include <iostream>

namespace
{

geometry_msgs::msg::Pose MakePose(double x)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.orientation.w = 1.0;
    return pose;
}

orbslam3_msgs::msg::OrbMap MakeMap()
{
    orbslam3_msgs::msg::OrbMap map;
    map.drone_id = 1;
    map.drone_name = "dron_1";
    map.map_epoch = 0;
    map.map_sequence = 1;
    map.map_frame = "dron_1_orb_map";
    for (uint64_t id = 0; id <= 4; ++id)
    {
        orbslam3_msgs::msg::OrbKeyFrame keyframe;
        keyframe.id = id;
        keyframe.pose = MakePose(static_cast<double>(id));
        keyframe.bow_word_ids = {7};
        keyframe.bow_word_values = {1.0F};
        map.keyframes.push_back(keyframe);
    }
    return map;
}

bool Expect(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "[test_loop_optimization_task] " << message << '\n';
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
    const RawSubmapId submap{1, 0};
    const RawKeyFrameId anchor{1, 0, 0};
    const RawKeyFrameId query{1, 0, 4};

    raw_db.InsertDelta(1, MakeMap());

    LoopDetector detector;
    LoopDetectorConfig detector_config;
    detector_config.min_mappoints = 0;
    detector_config.min_bow_score = 0.0;
    detector.Configure(detector_config);
    CovisibilityDatabase empty_covisibility;
    const auto causal_candidates = detector.ProcessNewKeyFrame(
        RawKeyFrameId{1, 0, 2}, raw_db, nullptr, empty_covisibility);
    ok &= Expect(
        causal_candidates.skipped_noncausal_same_submap == 2,
        "el detector no descarto los KFs posteriores a la query");
    for (const auto& candidate : causal_candidates.candidates)
    {
        ok &= Expect(
            candidate.candidate_kf_id.local_kf_id < 2,
            "el detector devolvio un candidato no causal del mismo submapa");
    }

    ok &= Expect(
        pose_store.AnchorSubmap(
            submap, Eigen::Matrix4d::Identity(), raw_db, "TEST_ANCHOR").success,
        "no se pudo anclar el submapa");
    ok &= Expect(
        pose_store.MarkHardFiducialKeyFrame(anchor, "TEST_FIDUCIAL"),
        "no se pudo fijar el anchor fiducial");

    LoopOptimizationTask task;
    task.task_id = 1001;
    task.arrival_id = 7;
    task.source = "TEST_LOOP";
    task.verification.query_kf_id = query;
    task.verification.candidate_seed_kf_id = anchor;
    task.verification.query_submap_id = submap;
    task.verification.candidate_submap_id = submap;
    task.verification.geometry_confirmed = true;
    task.verification.decision =
        LoopGeometryDecision::LoopOptimizationCandidate;
    task.verification.error_t = 1.0;
    task.verification.ransac_inliers = 100;
    task.verification.loop_confidence = 1.0;
    task.verification.mean_residual = 0.02;
    task.verification.relative_pose_measured = Eigen::Matrix4d::Identity();
    task.verification.relative_pose_measured(0, 3) = -1.0;

    PoseGraphBuilder builder;
    PoseGraphBuilderConfig builder_config;
    builder_config.min_vertices = 2;
    builder_config.vertex_selection_ratio = 1.0;
    builder.Configure(builder_config);
    const auto graph = builder.BuildForLoopTask(
        task, raw_db, pose_store, nullptr);
    ok &= Expect(graph.success, "el grafo de loop fue rechazado");
    ok &= Expect(
        graph.problem.task_type == "LOOP_OPTIMIZATION",
        "el grafo perdio el tipo de tarea loop");
    ok &= Expect(
        graph.problem.target_keyframe_id == query,
        "el target del grafo no es el KF query");

    const PoseGraphEdge* loop_edge = nullptr;
    bool has_absolute_target = false;
    for (const auto& edge : graph.problem.edges)
    {
        if (edge.edge_type == PoseGraphEdgeType::LoopRelative)
        {
            loop_edge = &edge;
            break;
        }
    }
    for (const auto& prior : graph.problem.priors)
    {
        has_absolute_target = has_absolute_target ||
            prior.prior_type == PoseGraphPriorType::FiducialTarget;
    }
    ok &= Expect(loop_edge != nullptr, "falta la arista relativa del loop");
    ok &= Expect(
        loop_edge && std::abs(loop_edge->relative_T_from_to(0, 3) + 3.0) < 1e-9,
        "la medida query-candidate del loop es incorrecta");
    ok &= Expect(
        !has_absolute_target,
        "el loop se convirtio indebidamente en un prior world absoluto");

    OptimizationManager manager;
    OptimizationManagerConfig manager_config;
    manager_config.dryrun_require_cost_decrease = false;
    manager.Configure(manager_config);
    const auto dry_run = manager.RunDryRun(
        graph.problem, raw_db, pose_store);
    ok &= Expect(dry_run.precheck_ok, "el dry-run no supero el precheck");
    ok &= Expect(dry_run.success, "el solver no termino correctamente");
    ok &= Expect(dry_run.useful, "la correccion simple no fue considerada util");
    ok &= Expect(
        pose_store.GetWorldPose(query) &&
        std::abs(pose_store.GetWorldPose(query)->operator()(0, 3) - 4.0) < 1e-9,
        "el dry-run modifico el pose store de entrada");

    if (!ok)
    {
        return 1;
    }
    std::cout << "[test_loop_optimization_task] PASS\n";
    return 0;
}
