#include "orbslam3_multi/fiducial_anchor_manager.hpp"
#include "orbslam3_multi/covisibility_database.hpp"
#include "orbslam3_multi/fused_landmark_manager.hpp"
#include "orbslam3_multi/global_map_builder.hpp"
#include "orbslam3_multi/global_pose_store.hpp"
#include "orbslam3_multi/landmark_score_manager.hpp"
#include "orbslam3_multi/loop_detector.hpp"
#include "orbslam3_multi/loop_decision_manager.hpp"
#include "orbslam3_multi/loop_optimization_task.hpp"
#include "orbslam3_multi/loop_pair_attempt_database.hpp"
#include "orbslam3_multi/optimization_debug_exporter.hpp"
#include "orbslam3_multi/optimization_manager.hpp"
#include "orbslam3_multi/optimization_result.hpp"
#include "orbslam3_multi/pose_graph_builder.hpp"
#include "orbslam3_multi/pose_graph_problem_io.hpp"
#include "orbslam3_multi/raw_map_database.hpp"
#include "orbslam3_multi/subcloud_loop_verifier.hpp"
#include "orbslam3_msgs/msg/orb_map.hpp"
#include "orbslam3_msgs/srv/get_orb_map.hpp"
#include "orbslam3_server/secondary_task_order.hpp"

#include <Eigen/Geometry>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/exceptions.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <map>
#include <mutex>
#include <limits>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

// F1B: servidor ROS 2 minimo para validar que llegan deltas OrbMap desde
// todos los drones. No almacena raw, no publica mapa global y no ejecuta
// fiduciales/loops/optimizacion; esas responsabilidades empiezan en fases
// posteriores sobre una base ya observable.
namespace
{
using OrbMap = orbslam3_msgs::msg::OrbMap;
using RawDatabaseStats = orbslam3_multi::RawDatabaseStats;
using RawKeyFrameId = orbslam3_multi::RawKeyFrameId;
using RawJournalEntry = orbslam3_multi::RawJournalEntry;
using RawJournalEntryKind = orbslam3_multi::RawJournalEntryKind;
using RawMapDatabase = orbslam3_multi::RawMapDatabase;
using RecordedFiducialObservation = orbslam3_multi::RecordedFiducialObservation;
using RawSubmapId = orbslam3_multi::RawSubmapId;
using BodyCameraTransformConfig = orbslam3_multi::BodyCameraTransformConfig;
using CovisibilityDatabase = orbslam3_multi::CovisibilityDatabase;
using CovisibilityDatabaseStats = orbslam3_multi::CovisibilityDatabaseStats;
using CovisibilityStrengthConfig = orbslam3_multi::CovisibilityStrengthConfig;
using FiducialAnchorManager = orbslam3_multi::FiducialAnchorManager;
using FiducialObservation = orbslam3_multi::FiducialObservation;
using FiducialOptimizationTask = orbslam3_multi::FiducialOptimizationTask;
using FiducialRevisitConfig = orbslam3_multi::FiducialRevisitConfig;
using FusedLandmarkManager = orbslam3_multi::FusedLandmarkManager;
using GlobalMapBuildResult = orbslam3_multi::GlobalMapBuildResult;
using GlobalMapBuilder = orbslam3_multi::GlobalMapBuilder;
using GlobalPoseNewKeyFrameStatus = orbslam3_multi::GlobalPoseNewKeyFrameStatus;
using GlobalPoseStore = orbslam3_multi::GlobalPoseStore;
using LandmarkScoreManager = orbslam3_multi::LandmarkScoreManager;
using LoopCandidate = orbslam3_multi::LoopCandidate;
using LoopCandidateResult = orbslam3_multi::LoopCandidateResult;
using LoopDecisionManager = orbslam3_multi::LoopDecisionManager;
using LoopDetector = orbslam3_multi::LoopDetector;
using LoopDetectorConfig = orbslam3_multi::LoopDetectorConfig;
using LoopVerificationResult = orbslam3_multi::LoopVerificationResult;
using LoopPairAttemptDatabase = orbslam3_multi::LoopPairAttemptDatabase;
using LoopPairState = orbslam3_multi::LoopPairState;
using LoopOptimizationTask = orbslam3_multi::LoopOptimizationTask;
using RawKeyFrameRevision = orbslam3_multi::RawKeyFrameRevision;
using CapturedLoopVerification = orbslam3_multi::CapturedLoopVerification;
using PreparedLoopVerification = orbslam3_multi::PreparedLoopVerification;
using LandmarkScoreUpdateResult = orbslam3_multi::LandmarkScoreUpdateResult;
using OptimizationDebugExporter = orbslam3_multi::OptimizationDebugExporter;
using OptimizationApplyResult = orbslam3_multi::OptimizationApplyResult;
using OptimizationDryRunResult = orbslam3_multi::OptimizationDryRunResult;
using OptimizationManager = orbslam3_multi::OptimizationManager;
using OptimizationManagerConfig = orbslam3_multi::OptimizationManagerConfig;
using PendingTailFiducialConstraint =
    orbslam3_multi::PendingTailFiducialConstraint;
using PostApplyDecision = orbslam3_multi::PostApplyDecision;
using PostApplyValidationResult = orbslam3_multi::PostApplyValidationResult;
using PoseGraphBuildResult = orbslam3_multi::PoseGraphBuildResult;
using PoseGraphBuilder = orbslam3_multi::PoseGraphBuilder;
using PoseGraphBuilderConfig = orbslam3_multi::PoseGraphBuilderConfig;
using PoseGraphProblem = orbslam3_multi::PoseGraphProblem;
using RawMapPointId = orbslam3_multi::RawMapPointId;
using SubcloudLoopVerifier = orbslam3_multi::SubcloudLoopVerifier;
using SubcloudLoopVerifierConfig = orbslam3_multi::SubcloudLoopVerifierConfig;
using GetOrbMap = orbslam3_msgs::srv::GetOrbMap;

// F1B: contadores ligeros por dron para validar recepcion multi-dron sin
// introducir todavia una base de datos raw persistente.
struct DroneRxStats
{
    uint64_t maps = 0;
    uint64_t keyframes = 0;
    uint64_t mappoints = 0;
    uint64_t last_epoch = 0;
    uint64_t last_sequence = 0;
    bool has_last_message = false;
};

struct GroundTruthSample
{
    rclcpp::Time stamp;
    Eigen::Matrix4d world_T_body = Eigen::Matrix4d::Identity();
};

struct DebugGtKeyFramePose
{
    Eigen::Matrix4d world_T_kf_gt = Eigen::Matrix4d::Identity();
    double kf_stamp_sec = 0.0;
    double gt_stamp_sec = 0.0;
    double association_dt_sec = 0.0;
    std::string association_quality;
};

struct DebugGtWindowStats
{
    uint64_t valid_kfs = 0;
    double mean_error_t = 0.0;
    double max_error_t = 0.0;
    std::map<RawKeyFrameId, double> error_t_by_keyframe;
};

struct FiducialConfig
{
    int id = 0;
    Eigen::Matrix4d world_T_fiducial = Eigen::Matrix4d::Identity();
    double radius_m = 0.0;
};

struct LoopTask
{
    uint64_t task_id = 0;
    RawKeyFrameId query_kf_id;
    orbslam3_multi::RawInsertResult insert_result;
    RawKeyFrameRevision admitted_revision;
    std::string trigger;
    bool material_new = false;
    uint32_t retry_count = 0;
};

struct ComputedLoopCandidate
{
    LoopCandidate candidate;
    LoopVerificationResult verification;
    std::map<RawKeyFrameId, RawKeyFrameRevision> captured_raw_revisions;
    std::map<RawKeyFrameId, Eigen::Matrix4d> captured_world_poses;
    PoseGraphBuildResult graph;
    OptimizationDryRunResult optimization;
    bool optimization_computed = false;
    uint64_t prior_loop_supports = 0;
};

}  // namespace

class GlobalMapServer : public rclcpp::Node
{
public:
    GlobalMapServer()
        : Node("global_orb_map_server")
    {
        // F1B: el constructor fija solo contratos de adaptador ROS. Estos
        // parametros permiten lanzar el mismo nodo desde simulacion o pruebas
        // futuras sin recuperar dependencias del servidor monolitico.
        use_sim_time_ = DeclareOrReadUseSimTime();
        world_frame_ = declare_parameter<std::string>("world_frame", "world");
        namespace_base_ = declare_parameter<std::string>("namespace_base", "dron");
        n_drones_ = declare_parameter<int>("n_drones", 1);
        stats_period_s_ = declare_parameter<double>("f1b_stats_period_s", 2.0);
        rawdb_record_enabled_ = declare_parameter<bool>("rawdb_record_enabled", true);
        rawdb_record_path_ = declare_parameter<std::string>(
            "rawdb_record_path",
            "src/codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record");
        rawdb_replay_enabled_ = declare_parameter<bool>("rawdb_replay_enabled", false);
        rawdb_replay_path_ = declare_parameter<std::string>(
            "rawdb_replay_path",
            "src/codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record");
        rawdb_replay_period_sec_ = declare_parameter<double>("rawdb_replay_period_sec", 0.5);
        f1m_covisibility_min_weight_ =
            declare_parameter<double>("f1m_covisibility_min_weight", 15.0);
        loop_bow_min_kf_gap_same_submap_ =
            declare_parameter<int>("loop_bow_min_kf_gap_same_submap", 20);
        loop_bow_max_candidates_ =
            declare_parameter<int>("loop_bow_max_candidates", 10);
        loop_bow_max_candidates_per_submap_ =
            declare_parameter<int>("loop_bow_max_candidates_per_submap", 3);
        loop_bow_min_mappoints_ =
            declare_parameter<int>("loop_bow_min_mappoints", 15);
        loop_bow_min_score_ =
            declare_parameter<double>("loop_bow_min_score", 0.01);
        loop_verify_max_candidates_per_query_ =
            declare_parameter<int>("loop_verify_max_candidates_per_query", 1);
        loop_verify_query_subcloud_min_points_ =
            declare_parameter<int>("loop_verify_query_subcloud_min_points", 12);
        loop_verify_candidate_window_max_kfs_ =
            declare_parameter<int>("loop_verify_candidate_window_max_kfs", 12);
        loop_verify_candidate_window_covisibility_min_weight_ =
            declare_parameter<int>("loop_verify_candidate_window_covisibility_min_weight", 15);
        loop_verify_candidate_window_temporal_kf_radius_ =
            declare_parameter<int>("loop_verify_candidate_window_temporal_kf_radius", 8);
        loop_verify_candidate_window_spatial_radius_m_ =
            declare_parameter<double>("loop_verify_candidate_window_spatial_radius_m", 4.0);
        loop_verify_candidate_subcloud_min_points_ =
            declare_parameter<int>("loop_verify_candidate_subcloud_min_points", 20);
        loop_verify_candidate_subcloud_max_points_ =
            declare_parameter<int>("loop_verify_candidate_subcloud_max_points", 900);
        loop_verify_candidate_subcloud_min_score_ =
            declare_parameter<double>("loop_verify_candidate_subcloud_min_score", 0.0);
        loop_verify_orb_match_max_hamming_ =
            declare_parameter<int>("loop_verify_orb_match_max_hamming", 80);
        loop_verify_orb_match_ratio_test_ =
            declare_parameter<double>("loop_verify_orb_match_ratio_test", 0.90);
        loop_verify_orb_match_cross_check_ =
            declare_parameter<bool>("loop_verify_orb_match_cross_check", true);
        loop_verify_min_initial_matches_ =
            declare_parameter<int>("loop_verify_min_initial_matches", 8);
        loop_verify_candidate_reduce_enabled_ =
            declare_parameter<bool>("loop_verify_candidate_reduce_enabled", true);
        loop_verify_candidate_reduce_min_initial_matches_ =
            declare_parameter<int>("loop_verify_candidate_reduce_min_initial_matches", 8);
        loop_verify_candidate_reduce_margin_m_ =
            declare_parameter<double>("loop_verify_candidate_reduce_margin_m", 0.75);
        loop_verify_candidate_reduce_min_points_after_ =
            declare_parameter<int>("loop_verify_candidate_reduce_min_points_after", 20);
        loop_verify_ransac_min_matches_ =
            declare_parameter<int>("loop_verify_ransac_min_matches", 6);
        loop_verify_ransac_max_iterations_ =
            declare_parameter<int>("loop_verify_ransac_max_iterations", 120);
        loop_verify_ransac_inlier_threshold_m_ =
            declare_parameter<double>("loop_verify_ransac_inlier_threshold_m", 0.30);
        loop_verify_ransac_min_inliers_ =
            declare_parameter<int>("loop_verify_ransac_min_inliers", 6);
        loop_verify_ransac_min_inlier_ratio_ =
            declare_parameter<double>("loop_verify_ransac_min_inlier_ratio", 0.25);
        loop_verify_accept_mean_residual_m_ =
            declare_parameter<double>("loop_verify_accept_mean_residual_m", 0.20);
        loop_verify_accept_max_residual_m_ =
            declare_parameter<double>("loop_verify_accept_max_residual_m", 0.75);
        loop_verify_fusion_error_t_m_ =
            declare_parameter<double>("loop_verify_fusion_error_t_m", 0.35);
        loop_verify_fusion_error_yaw_rad_ =
            declare_parameter<double>("loop_verify_fusion_error_yaw_rad", 0.25);
        loop_optimization_min_prior_supports_ = declare_parameter<int>(
            "loop_optimization_min_prior_supports", 2);
        aligned_overlap_enabled_ =
            declare_parameter<bool>("aligned_overlap_enabled", true);
        aligned_overlap_keyframe_radius_m_ =
            declare_parameter<double>("aligned_overlap_keyframe_radius_m", 4.0);
        aligned_overlap_max_candidate_kfs_ =
            declare_parameter<int>("aligned_overlap_max_candidate_kfs", 12);
        aligned_overlap_strict_position_m_ =
            declare_parameter<double>("aligned_overlap_strict_position_m", 0.40);
        aligned_overlap_strict_max_hamming_ =
            declare_parameter<int>("aligned_overlap_strict_max_hamming", 50);
        aligned_overlap_strict_ratio_test_ =
            declare_parameter<double>("aligned_overlap_strict_ratio_test", 0.80);
        aligned_overlap_strict_min_matches_ =
            declare_parameter<int>("aligned_overlap_strict_min_matches", 8);
        aligned_overlap_strict_min_match_ratio_ =
            declare_parameter<double>("aligned_overlap_strict_min_match_ratio", 0.10);
        aligned_overlap_strict_min_image_bins_ =
            declare_parameter<int>("aligned_overlap_strict_min_image_bins", 3);
        aligned_overlap_strict_min_3d_span_ratio_ =
            declare_parameter<double>("aligned_overlap_strict_min_3d_span_ratio", 0.35);
        aligned_overlap_strict_mean_residual_m_ =
            declare_parameter<double>("aligned_overlap_strict_mean_residual_m", 0.12);
        aligned_overlap_strict_max_residual_m_ =
            declare_parameter<double>("aligned_overlap_strict_max_residual_m", 0.30);
        aligned_overlap_expand_position_m_ =
            declare_parameter<double>("aligned_overlap_expand_position_m", 0.30);
        aligned_overlap_expand_max_hamming_ =
            declare_parameter<int>("aligned_overlap_expand_max_hamming", 80);
        aligned_overlap_expand_ratio_test_ =
            declare_parameter<double>("aligned_overlap_expand_ratio_test", 0.90);
        loop_backlog_high_watermark_ =
            declare_parameter<int>("loop_backlog_high_watermark", 10);
        loop_backlog_low_watermark_ =
            declare_parameter<int>("loop_backlog_low_watermark", 3);
        loop_task_max_pending_ =
            declare_parameter<int>("loop_task_max_pending", 4096);
        mapping_backpressure_topic_ = declare_parameter<std::string>(
            "mapping_backpressure_topic",
            "/global_mapping/backpressure_active");
        flow_telemetry_enabled_ = declare_parameter<bool>(
            "flow_telemetry_enabled", true);
        flow_telemetry_topic_ = declare_parameter<std::string>(
            "flow_telemetry_topic", "/global_mapping/flow_events");
        pose_store_debug_enabled_ = declare_parameter<bool>("pose_store_debug_enabled", false);
        pose_store_debug_anchor_after_deltas_ =
            declare_parameter<int>("pose_store_debug_anchor_after_deltas", 5);
        pose_store_debug_anchor_drone_id_ =
            declare_parameter<int>("pose_store_debug_anchor_drone_id", 1);
        pose_store_debug_anchor_epoch_ =
            declare_parameter<int>("pose_store_debug_anchor_epoch", 0);
        pose_store_debug_anchor_world_x_ =
            declare_parameter<double>("pose_store_debug_anchor_world_x", 2.0);
        pose_store_debug_anchor_world_y_ =
            declare_parameter<double>("pose_store_debug_anchor_world_y", 0.0);
        pose_store_debug_anchor_world_z_ =
            declare_parameter<double>("pose_store_debug_anchor_world_z", 0.0);
        pose_store_debug_anchor_yaw_ =
            declare_parameter<double>("pose_store_debug_anchor_yaw", 0.0);
        pose_store_debug_opt_enabled_ =
            declare_parameter<bool>("pose_store_debug_opt_enabled", false);
        pose_store_debug_opt_after_deltas_ =
            declare_parameter<int>("pose_store_debug_opt_after_deltas", 10);
        pose_store_debug_opt_kf_id_ =
            declare_parameter<int>("pose_store_debug_opt_kf_id", 0);
        pose_store_debug_opt_dx_ =
            declare_parameter<double>("pose_store_debug_opt_dx", 0.15);
        pose_store_debug_opt_dy_ =
            declare_parameter<double>("pose_store_debug_opt_dy", -0.03);
        pose_store_debug_opt_dz_ =
            declare_parameter<double>("pose_store_debug_opt_dz", 0.0);
        pose_store_debug_opt_dyaw_ =
            declare_parameter<double>("pose_store_debug_opt_dyaw", 0.05);
        fiducial_sim_enabled_ = declare_parameter<bool>("fiducial_sim_enabled", true);
        fiducial_gt_max_dt_sec_ =
            declare_parameter<double>("fiducial_gt_max_dt_sec", 1.0);
        fiducial_gt_buffer_max_samples_ =
            declare_parameter<int>("fiducial_gt_buffer_max_samples", 250);
        fiducial_ids_ = declare_parameter<std::vector<int64_t>>("fiducials.ids", {2});
        fiducial_x_ = declare_parameter<std::vector<double>>("fiducials.x", {0.0});
        fiducial_y_ = declare_parameter<std::vector<double>>("fiducials.y", {-9.0});
        fiducial_z_ = declare_parameter<std::vector<double>>("fiducials.z", {1.0});
        fiducial_yaw_ = declare_parameter<std::vector<double>>("fiducials.yaw", {0.0});
        fiducial_radius_ = declare_parameter<std::vector<double>>("fiducials.radius", {2.0});
        fiducial_revisit_error_threshold_m_ =
            declare_parameter<double>("fiducial_revisit_error_threshold_m", 0.35);
        fiducial_revisit_yaw_threshold_rad_ =
            declare_parameter<double>("fiducial_revisit_yaw_threshold_rad", 0.25);
        fiducial_revisit_rot_threshold_rad_ =
            declare_parameter<double>("fiducial_revisit_rot_threshold_rad", 0.35);
        body_T_camera_x_ = declare_parameter<double>("body_T_camera_x", 0.10);
        body_T_camera_y_ = declare_parameter<double>("body_T_camera_y", 0.03);
        body_T_camera_z_ = declare_parameter<double>("body_T_camera_z", 0.03);
        body_T_camera_roll_deg_ =
            declare_parameter<double>("body_T_camera_roll_deg", 0.0);
        body_T_camera_pitch_deg_ =
            declare_parameter<double>("body_T_camera_pitch_deg", -90.0);
        body_T_camera_yaw_deg_ =
            declare_parameter<double>("body_T_camera_yaw_deg", 90.0);
        use_camera_optical_frame_convention_ =
            declare_parameter<bool>("use_camera_optical_frame_convention", true);
        global_sparse_cloud_topic_ =
            declare_parameter<std::string>("global_sparse_cloud_topic", "/global_sparse_cloud");
        global_map_min_score_to_publish_ =
            declare_parameter<double>("global_map_min_score_to_publish", 0.0);
        global_map_publish_period_sec_ =
            declare_parameter<double>("global_map_publish_period_sec", 1.0);
        global_keyframes_topic_ =
            declare_parameter<std::string>("global_keyframes_topic", "/global_keyframes");
        global_keyframes_publish_enabled_ =
            declare_parameter<bool>("global_keyframes_publish_enabled", true);
        global_keyframes_frustum_scale_ =
            declare_parameter<double>("global_keyframes_frustum_scale", 0.20);
        global_keyframes_labels_enabled_ =
            declare_parameter<bool>("global_keyframes_labels_enabled", false);
        f1g_full_snapshot_enabled_ =
            declare_parameter<bool>("f1g_full_snapshot_enabled", true);
        f1g_full_snapshot_startup_delay_sec_ =
            declare_parameter<double>("f1g_full_snapshot_startup_delay_sec", 35.0);
        f1g_full_snapshot_period_sec_ =
            declare_parameter<double>("f1g_full_snapshot_period_sec", 35.0);
        f1g_debug_mark_optimized_kf_ =
            declare_parameter<bool>("f1g_debug_mark_optimized_kf", true);
        pose_graph_min_vertices_ =
            declare_parameter<int>("pose_graph_min_vertices", 2);
        pose_graph_vertex_selection_ratio_ =
            declare_parameter<double>("pose_graph_vertex_selection_ratio", 0.30);
        pose_graph_anchor_stop_enabled_ =
            declare_parameter<bool>("pose_graph_anchor_stop_enabled", true);
        pose_graph_fiducial_connectivity_enabled_ =
            declare_parameter<bool>("pose_graph_fiducial_connectivity_enabled", true);
        pose_graph_branch_selection_enabled_ =
            declare_parameter<bool>("pose_graph_branch_selection_enabled", true);
        pose_graph_subdivision_candidate_radius_m_ =
            declare_parameter<double>("pose_graph_subdivision_candidate_radius_m", 2.0);
        pose_graph_include_temporal_edges_ =
            declare_parameter<bool>("pose_graph_include_temporal_edges", true);
        pose_graph_use_covisibility_edges_ =
            declare_parameter<bool>("pose_graph_use_covisibility_edges", false);
        pose_graph_temporal_edge_weight_ =
            declare_parameter<double>("pose_graph_temporal_edge_weight", 25.0);
        pose_graph_temporal_edge_weight_sparse_ =
            declare_parameter<double>("pose_graph_temporal_edge_weight_sparse", 10.0);
        pose_graph_fiducial_hard_weight_ =
            declare_parameter<double>("pose_graph_fiducial_hard_weight", 1000000.0);
        pose_graph_fiducial_target_translation_weight_ =
            declare_parameter<double>(
                "pose_graph_fiducial_target_translation_weight",
                5000.0);
        pose_graph_fiducial_target_rotation_weight_ =
            declare_parameter<double>(
                "pose_graph_fiducial_target_rotation_weight",
                2500.0);
        pose_graph_current_pose_soft_weight_ =
            declare_parameter<double>("pose_graph_current_pose_soft_weight", 5.0);
        pose_graph_fiducial_neighborhood_radius_m_ =
            declare_parameter<double>("pose_graph_fiducial_neighborhood_radius_m", 4.0);
        pose_graph_fiducial_neighborhood_radius_kfs_ =
            declare_parameter<int>("pose_graph_fiducial_neighborhood_radius_kfs", 3);
        pose_graph_fiducial_neighborhood_vertex_ratio_ =
            declare_parameter<double>(
                "pose_graph_fiducial_neighborhood_vertex_ratio",
                0.20);
        pose_graph_corner_yaw_threshold_rad_ =
            declare_parameter<double>("pose_graph_corner_yaw_threshold_rad", 0.5235987756);
        f1i_debug_force_task_enabled_ =
            declare_parameter<bool>("f1i_debug_force_task_enabled", false);
        f1i_debug_task_dx_ =
            declare_parameter<double>("f1i_debug_task_dx", 0.75);
        f1i_debug_task_dy_ =
            declare_parameter<double>("f1i_debug_task_dy", 0.0);
        f1i_debug_task_dz_ =
            declare_parameter<double>("f1i_debug_task_dz", 0.0);
        f1i_debug_task_dyaw_ =
            declare_parameter<double>("f1i_debug_task_dyaw", 0.20);
        f1j_dryrun_enabled_ =
            declare_parameter<bool>("f1j_dryrun_enabled", true);
        f1j_dryrun_min_improvement_ratio_ =
            declare_parameter<double>("f1j_dryrun_min_improvement_ratio", 0.05);
        f1j_dryrun_partial_min_improvement_ratio_ =
            declare_parameter<double>("f1j_dryrun_partial_min_improvement_ratio", 0.70);
        f1j_dryrun_max_final_error_t_ =
            declare_parameter<double>("f1j_dryrun_max_final_error_t", 0.35);
        f1j_dryrun_require_cost_decrease_ =
            declare_parameter<bool>("f1j_dryrun_require_cost_decrease", false);
        f1j_solver_step_fraction_ =
            declare_parameter<double>("f1j_solver_step_fraction", 0.95);
        f1j_export_debug_plot_ =
            declare_parameter<bool>("f1j_export_debug_plot", false);
        f1j_debug_output_dir_ =
            declare_parameter<std::string>(
                "f1j_debug_output_dir",
                "src/codex/archivos_auxiliares");
        f1k_apply_enabled_ =
            declare_parameter<bool>("f1k_apply_enabled", true);
        f1l_validation_enabled_ =
            declare_parameter<bool>("f1l_validation_enabled", true);
        f1l_partial_apply_enabled_ =
            declare_parameter<bool>("f1l_partial_apply_enabled", true);
        f1l_max_partial_retries_ =
            declare_parameter<int>("f1l_max_partial_retries", 3);
        f1l_debug_force_reject_once_ =
            declare_parameter<bool>("f1l_debug_force_reject_once", false);
        f1l_debug_force_reject_task_id_ =
            declare_parameter<int>("f1l_debug_force_reject_task_id", -1);
        f1l_post_apply_internal_broken_edge_t_ =
            declare_parameter<double>("f1l_post_apply_internal_broken_edge_t", 2.50);
        f1l_post_apply_internal_max_growth_t_ =
            declare_parameter<double>("f1l_post_apply_internal_max_growth_t", 1.50);
        f1l_post_apply_fiducial_absurd_error_t_ =
            declare_parameter<double>("f1l_post_apply_fiducial_absurd_error_t", 10.0);
        f1l_gt_kf_debug_enabled_ =
            declare_parameter<bool>("f1l_gt_kf_debug_enabled", false);
        f1l_gt_kf_debug_max_dt_sec_ =
            declare_parameter<double>("f1l_gt_kf_debug_max_dt_sec", fiducial_gt_max_dt_sec_);
        f1l_debug_animation_enabled_ =
            declare_parameter<bool>("f1l_debug_animation_enabled", true);
        f1l_debug_animation_output_dir_ =
            declare_parameter<std::string>(
                "f1l_debug_animation_output_dir",
                "src/codex/archivos_auxiliares/html");
        f1l_graph_dump_enabled_ =
            declare_parameter<bool>("f1l_graph_dump_enabled", false);
        f1l_graph_dump_output_dir_ =
            declare_parameter<std::string>(
                "f1l_graph_dump_output_dir",
                "src/codex/archivos_auxiliares/repeticiones");

        // F1B: el servidor minimo debe sobrevivir a configuraciones invalidas
        // para que la simulacion deje evidencia en logs en vez de abortar antes
        // de suscribirse a los topics.
        if (n_drones_ < 1)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1B-SERVER-PARAMS] invalid_n_drones=%d forcing=1",
                n_drones_);
            n_drones_ = 1;
        }

        // F1B: limitar el periodo evita spam extremo si el launch pasa un valor
        // accidentalmente bajo, pero conserva observabilidad frecuente.
        if (stats_period_s_ < 0.2)
        {
            stats_period_s_ = 0.2;
        }
        if (rawdb_replay_period_sec_ < 0.01)
        {
            rawdb_replay_period_sec_ = 0.01;
        }
        if (loop_bow_min_kf_gap_same_submap_ < 0)
        {
            loop_bow_min_kf_gap_same_submap_ = 0;
        }
        if (loop_bow_max_candidates_ < 1)
        {
            loop_bow_max_candidates_ = 1;
        }
        if (loop_bow_max_candidates_per_submap_ < 1)
        {
            loop_bow_max_candidates_per_submap_ = 1;
        }
        if (loop_bow_min_mappoints_ < 0)
        {
            loop_bow_min_mappoints_ = 0;
        }
        if (loop_bow_min_score_ < 0.0)
        {
            loop_bow_min_score_ = 0.0;
        }
        if (loop_bow_min_score_ > 1.0)
        {
            loop_bow_min_score_ = 1.0;
        }
        if (loop_verify_max_candidates_per_query_ < 1)
        {
            loop_verify_max_candidates_per_query_ = 1;
        }
        if (loop_verify_query_subcloud_min_points_ < 1)
        {
            loop_verify_query_subcloud_min_points_ = 1;
        }
        if (loop_verify_candidate_window_max_kfs_ < 1)
        {
            loop_verify_candidate_window_max_kfs_ = 1;
        }
        if (loop_verify_candidate_window_covisibility_min_weight_ < 0)
        {
            loop_verify_candidate_window_covisibility_min_weight_ = 0;
        }
        if (loop_verify_candidate_window_temporal_kf_radius_ < 0)
        {
            loop_verify_candidate_window_temporal_kf_radius_ = 0;
        }
        if (loop_verify_candidate_window_spatial_radius_m_ < 0.0)
        {
            loop_verify_candidate_window_spatial_radius_m_ = 0.0;
        }
        if (loop_verify_candidate_subcloud_min_points_ < 1)
        {
            loop_verify_candidate_subcloud_min_points_ = 1;
        }
        if (loop_verify_candidate_subcloud_max_points_ <
            loop_verify_candidate_subcloud_min_points_)
        {
            loop_verify_candidate_subcloud_max_points_ =
                loop_verify_candidate_subcloud_min_points_;
        }
        if (loop_verify_candidate_subcloud_min_score_ < 0.0)
        {
            loop_verify_candidate_subcloud_min_score_ = 0.0;
        }
        if (loop_verify_orb_match_max_hamming_ < 0)
        {
            loop_verify_orb_match_max_hamming_ = 0;
        }
        if (loop_verify_orb_match_max_hamming_ > 256)
        {
            loop_verify_orb_match_max_hamming_ = 256;
        }
        if (loop_verify_orb_match_ratio_test_ <= 0.0)
        {
            loop_verify_orb_match_ratio_test_ = 1.0;
        }
        if (loop_verify_min_initial_matches_ < 1)
        {
            loop_verify_min_initial_matches_ = 1;
        }
        if (loop_verify_candidate_reduce_min_initial_matches_ < 1)
        {
            loop_verify_candidate_reduce_min_initial_matches_ = 1;
        }
        if (loop_verify_candidate_reduce_margin_m_ < 0.0)
        {
            loop_verify_candidate_reduce_margin_m_ = 0.0;
        }
        if (loop_verify_candidate_reduce_min_points_after_ < 1)
        {
            loop_verify_candidate_reduce_min_points_after_ = 1;
        }
        if (loop_verify_ransac_min_matches_ < 3)
        {
            loop_verify_ransac_min_matches_ = 3;
        }
        if (loop_verify_ransac_max_iterations_ < 1)
        {
            loop_verify_ransac_max_iterations_ = 1;
        }
        if (loop_verify_ransac_inlier_threshold_m_ <= 0.0)
        {
            loop_verify_ransac_inlier_threshold_m_ = 0.30;
        }
        if (loop_verify_ransac_min_inliers_ < 3)
        {
            loop_verify_ransac_min_inliers_ = 3;
        }
        if (loop_verify_ransac_min_inlier_ratio_ < 0.0)
        {
            loop_verify_ransac_min_inlier_ratio_ = 0.0;
        }
        if (loop_verify_ransac_min_inlier_ratio_ > 1.0)
        {
            loop_verify_ransac_min_inlier_ratio_ = 1.0;
        }
        if (loop_verify_accept_mean_residual_m_ < 0.0)
        {
            loop_verify_accept_mean_residual_m_ = 0.0;
        }
        if (loop_verify_accept_max_residual_m_ < 0.0)
        {
            loop_verify_accept_max_residual_m_ = 0.0;
        }
        if (loop_verify_fusion_error_t_m_ < 0.0)
        {
            loop_verify_fusion_error_t_m_ = 0.0;
        }
        if (loop_verify_fusion_error_yaw_rad_ < 0.0)
        {
            loop_verify_fusion_error_yaw_rad_ = 0.0;
        }
        loop_optimization_min_prior_supports_ =
            std::max(0, loop_optimization_min_prior_supports_);
        aligned_overlap_keyframe_radius_m_ =
            std::max(0.0, aligned_overlap_keyframe_radius_m_);
        aligned_overlap_max_candidate_kfs_ =
            std::max(1, aligned_overlap_max_candidate_kfs_);
        aligned_overlap_strict_position_m_ =
            std::max(0.01, aligned_overlap_strict_position_m_);
        aligned_overlap_strict_max_hamming_ =
            std::max(0, std::min(256, aligned_overlap_strict_max_hamming_));
        aligned_overlap_strict_ratio_test_ =
            std::max(0.01, std::min(1.0, aligned_overlap_strict_ratio_test_));
        aligned_overlap_strict_min_matches_ =
            std::max(3, aligned_overlap_strict_min_matches_);
        aligned_overlap_strict_min_match_ratio_ =
            std::max(
                0.0,
                std::min(1.0, aligned_overlap_strict_min_match_ratio_));
        aligned_overlap_strict_min_image_bins_ =
            std::max(1, std::min(4, aligned_overlap_strict_min_image_bins_));
        aligned_overlap_strict_min_3d_span_ratio_ =
            std::max(
                0.0,
                std::min(1.0, aligned_overlap_strict_min_3d_span_ratio_));
        aligned_overlap_strict_mean_residual_m_ =
            std::max(0.0, aligned_overlap_strict_mean_residual_m_);
        aligned_overlap_strict_max_residual_m_ =
            std::max(
                aligned_overlap_strict_mean_residual_m_,
                aligned_overlap_strict_max_residual_m_);
        aligned_overlap_expand_position_m_ =
            std::max(0.01, aligned_overlap_expand_position_m_);
        aligned_overlap_expand_max_hamming_ =
            std::max(0, std::min(256, aligned_overlap_expand_max_hamming_));
        aligned_overlap_expand_ratio_test_ =
            std::max(0.01, std::min(1.0, aligned_overlap_expand_ratio_test_));
        if (loop_backlog_high_watermark_ < 1)
        {
            loop_backlog_high_watermark_ = 1;
        }
        if (loop_backlog_low_watermark_ < 0)
        {
            loop_backlog_low_watermark_ = 0;
        }
        if (loop_backlog_low_watermark_ >= loop_backlog_high_watermark_)
        {
            loop_backlog_low_watermark_ =
                std::max(0, loop_backlog_high_watermark_ - 1);
        }
        loop_task_max_pending_ = std::max(
            loop_backlog_high_watermark_, loop_task_max_pending_);
        if (pose_store_debug_anchor_after_deltas_ < 1)
        {
            pose_store_debug_anchor_after_deltas_ = 1;
        }
        if (pose_store_debug_opt_after_deltas_ < pose_store_debug_anchor_after_deltas_)
        {
            pose_store_debug_opt_after_deltas_ = pose_store_debug_anchor_after_deltas_ + 1;
        }
        if (pose_store_debug_anchor_drone_id_ < 1)
        {
            pose_store_debug_anchor_drone_id_ = 1;
        }
        if (pose_store_debug_anchor_epoch_ < 0)
        {
            pose_store_debug_anchor_epoch_ = 0;
        }
        if (pose_store_debug_opt_kf_id_ < 0)
        {
            pose_store_debug_opt_kf_id_ = 0;
        }
        if (fiducial_gt_max_dt_sec_ < 0.0)
        {
            fiducial_gt_max_dt_sec_ = 0.0;
        }
        if (fiducial_gt_buffer_max_samples_ < 1)
        {
            fiducial_gt_buffer_max_samples_ = 1;
        }
        if (f1l_gt_kf_debug_max_dt_sec_ < 0.0)
        {
            f1l_gt_kf_debug_max_dt_sec_ = 0.0;
        }
        if (fiducial_revisit_error_threshold_m_ < 0.0)
        {
            fiducial_revisit_error_threshold_m_ = 0.0;
        }
        if (fiducial_revisit_yaw_threshold_rad_ < 0.0)
        {
            fiducial_revisit_yaw_threshold_rad_ = 0.0;
        }
        if (fiducial_revisit_rot_threshold_rad_ < 0.0)
        {
            fiducial_revisit_rot_threshold_rad_ = 0.0;
        }
        if (global_map_min_score_to_publish_ < 0.0)
        {
            global_map_min_score_to_publish_ = 0.0;
        }
        if (global_map_min_score_to_publish_ > 1.0)
        {
            global_map_min_score_to_publish_ = 1.0;
        }
        if (global_map_publish_period_sec_ < 0.2)
        {
            global_map_publish_period_sec_ = 0.2;
        }
        if (global_keyframes_frustum_scale_ < 0.02)
        {
            global_keyframes_frustum_scale_ = 0.02;
        }
        if (f1g_full_snapshot_startup_delay_sec_ < 1.0)
        {
            f1g_full_snapshot_startup_delay_sec_ = 1.0;
        }
        if (f1g_full_snapshot_period_sec_ < 5.0)
        {
            f1g_full_snapshot_period_sec_ = 5.0;
        }
        if (pose_graph_min_vertices_ < 2)
        {
            pose_graph_min_vertices_ = 2;
        }
        pose_graph_vertex_selection_ratio_ = std::max(
            0.0, std::min(1.0, pose_graph_vertex_selection_ratio_));
        if (pose_graph_fiducial_neighborhood_radius_kfs_ < 0)
        {
            pose_graph_fiducial_neighborhood_radius_kfs_ = 0;
        }
        if (pose_graph_fiducial_neighborhood_radius_m_ < 0.0)
        {
            pose_graph_fiducial_neighborhood_radius_m_ = 0.0;
        }
        if (pose_graph_fiducial_neighborhood_vertex_ratio_ < 0.0)
        {
            pose_graph_fiducial_neighborhood_vertex_ratio_ = 0.0;
        }
        if (pose_graph_corner_yaw_threshold_rad_ < 0.0)
        {
            pose_graph_corner_yaw_threshold_rad_ = 0.0;
        }
        if (pose_graph_temporal_edge_weight_ < 0.0)
        {
            pose_graph_temporal_edge_weight_ = 0.0;
        }
        if (pose_graph_subdivision_candidate_radius_m_ < 0.0)
        {
            pose_graph_subdivision_candidate_radius_m_ = 0.0;
        }
        if (pose_graph_temporal_edge_weight_sparse_ < 0.0)
        {
            pose_graph_temporal_edge_weight_sparse_ = 0.0;
        }
        if (f1m_covisibility_min_weight_ < 0.0)
        {
            f1m_covisibility_min_weight_ = 0.0;
        }
        if (pose_graph_fiducial_hard_weight_ < 0.0)
        {
            pose_graph_fiducial_hard_weight_ = 0.0;
        }
        if (pose_graph_fiducial_target_translation_weight_ < 0.0)
        {
            pose_graph_fiducial_target_translation_weight_ = 0.0;
        }
        if (pose_graph_fiducial_target_rotation_weight_ < 0.0)
        {
            pose_graph_fiducial_target_rotation_weight_ = 0.0;
        }
        if (pose_graph_current_pose_soft_weight_ < 0.0)
        {
            pose_graph_current_pose_soft_weight_ = 0.0;
        }
        if (f1j_dryrun_min_improvement_ratio_ < 0.0)
        {
            f1j_dryrun_min_improvement_ratio_ = 0.0;
        }
        if (f1j_dryrun_partial_min_improvement_ratio_ < 0.0)
        {
            f1j_dryrun_partial_min_improvement_ratio_ = 0.0;
        }
        if (f1j_dryrun_max_final_error_t_ < 0.0)
        {
            f1j_dryrun_max_final_error_t_ = 0.0;
        }
        if (f1j_solver_step_fraction_ < 0.0)
        {
            f1j_solver_step_fraction_ = 0.0;
        }
        if (f1j_solver_step_fraction_ > 1.0)
        {
            f1j_solver_step_fraction_ = 1.0;
        }
        if (f1l_debug_force_reject_task_id_ < -1)
        {
            f1l_debug_force_reject_task_id_ = -1;
        }
        if (f1l_max_partial_retries_ < 0)
        {
            f1l_max_partial_retries_ = 0;
        }
        if (f1l_post_apply_internal_broken_edge_t_ < 0.0)
        {
            f1l_post_apply_internal_broken_edge_t_ = 0.0;
        }
        if (f1l_post_apply_internal_max_growth_t_ < 0.0)
        {
            f1l_post_apply_internal_max_growth_t_ = 0.0;
        }
        if (f1l_post_apply_fiducial_absurd_error_t_ < 0.0)
        {
            f1l_post_apply_fiducial_absurd_error_t_ = 0.0;
        }

        LoadFiducialConfig();
        ConfigureFiducialRevisit();
        ConfigureBodyCameraTransform();
        ConfigurePoseGraphBuilder();
        ConfigureOptimizationManager();
        ConfigureLoopDetector();
        ConfigureSubcloudLoopVerifier();
        global_sparse_cloud_pub_ =
            create_publisher<sensor_msgs::msg::PointCloud2>(
                global_sparse_cloud_topic_,
                // F1H-hotfix: RViz2 solo necesita la ultima nube sparse. Con
                // una cola mayor podia recibir nubes antiguas despues de una
                // nueva y verse un parpadeo entre estados consecutivos.
                rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
        global_keyframes_pub_ =
            create_publisher<visualization_msgs::msg::MarkerArray>(
                global_keyframes_topic_,
                rclcpp::QoS(rclcpp::KeepLast(1))
                    .reliable()
                    .transient_local());
        mapping_backpressure_pub_ =
            create_publisher<std_msgs::msg::Bool>(
                mapping_backpressure_topic_,
                rclcpp::QoS(rclcpp::KeepLast(1))
                    .reliable()
                    .transient_local());
        flow_telemetry_pub_ = create_publisher<std_msgs::msg::String>(
            flow_telemetry_topic_, rclcpp::QoS(rclcpp::KeepLast(64)));
        PublishMappingBackpressure(false, "startup");

        RCLCPP_WARN(
            get_logger(),
            "[F1B-SERVER-INIT] node=global_orb_map_server mode=minimal_rx_only");

        RCLCPP_WARN(
            get_logger(),
            "[F1B-SERVER-PARAMS] use_sim_time=%s world_frame=%s namespace_base=%s drones=%d stats_period_s=%.3f",
            use_sim_time_ ? "true" : "false",
            world_frame_.c_str(),
            namespace_base_.c_str(),
            n_drones_,
            stats_period_s_);

        RCLCPP_WARN(
            get_logger(),
            "[F1C-RAWDB-INIT] record_enabled=%s record_path=%s replay_enabled=%s replay_path=%s replay_period_sec=%.3f",
            rawdb_record_enabled_ ? "true" : "false",
            rawdb_record_path_.c_str(),
            rawdb_replay_enabled_ ? "true" : "false",
            rawdb_replay_path_.c_str(),
            rawdb_replay_period_sec_);

        LogPoseStoreInit();
        LogFiducialInit();
        LogScoreInit();
        RCLCPP_WARN(
            get_logger(),
            "[F1F-GLOBALMAP-PUBLISHER] topic=%s frame_id=%s min_score_to_publish=%.3f publish_period_sec=%.3f",
            global_sparse_cloud_topic_.c_str(),
            world_frame_.c_str(),
            global_map_min_score_to_publish_,
            global_map_publish_period_sec_);
        RCLCPP_WARN(
            get_logger(),
            "[F1T-RVIZ-KF-CONFIG] enabled=%s topic=%s frame_id=%s frustum_scale=%.3f labels=%s qos=reliable_keep_last_1_transient_local",
            global_keyframes_publish_enabled_ ? "true" : "false",
            global_keyframes_topic_.c_str(),
            world_frame_.c_str(),
            global_keyframes_frustum_scale_,
            global_keyframes_labels_enabled_ ? "true" : "false");

        // F1C: en modo replay no nos suscribimos a wrappers reales. El origen
        // de verdad pasa a ser el journal guardado para que la prueba sea
        // repetible sin Gazebo ni ORB-SLAM3 produciendo datos nuevos.
        if (rawdb_replay_enabled_)
        {
            LoadReplayDataset();
        }
        else
        {
            CreateOrbMapSubscriptions();
            if (f1g_full_snapshot_enabled_)
            {
                CreateFullSnapshotClients();
            }
            if (fiducial_sim_enabled_)
            {
                CreateGroundTruthSubscriptions();
            }
        }

        stats_timer_ = create_wall_timer(
            std::chrono::duration<double>(stats_period_s_),
            [this]()
            {
                std::lock_guard<std::recursive_mutex> lock(live_state_mutex_);
                PublishStatsLog();
            });

        // F1F/F1H-hotfix: el timer republica la ultima nube ya construida para
        // que RViz2 pueda engancharse tarde sin alternar una reconstruccion
        // antigua con la actual recibida por delta/snapshot.
        global_map_publish_timer_ = create_wall_timer(
            std::chrono::duration<double>(global_map_publish_period_sec_),
            [this]()
            {
                RepublishLastGlobalSparseCloud("timer");
            });
        flow_telemetry_timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            [this]()
            {
                DrainFlowTelemetry();
            });

        // F1C: el guardado periodico evita depender solo del destructor cuando
        // `run_simulation.sh` cierre el launch. La DB sigue siendo raw: guardar
        // no transforma ni optimiza ningun dato.
        if (rawdb_record_enabled_ && !rawdb_replay_enabled_)
        {
            rawdb_save_timer_ = create_wall_timer(
                std::chrono::seconds(5),
                [this]()
                {
                    SaveRawDatabase("periodic");
                });
        }

        if (!rawdb_replay_enabled_ && f1g_full_snapshot_enabled_)
        {
            full_snapshot_startup_timer_ = create_wall_timer(
                std::chrono::duration<double>(f1g_full_snapshot_startup_delay_sec_),
                [this]()
                {
                    std::lock_guard<std::recursive_mutex> lock(live_state_mutex_);
                    if (full_snapshot_startup_timer_)
                    {
                        full_snapshot_startup_timer_->cancel();
                    }
                    RequestFullSnapshots("startup_resync");
                    full_snapshot_periodic_timer_ = create_wall_timer(
                        std::chrono::duration<double>(f1g_full_snapshot_period_sec_),
                        [this]()
                        {
                            std::lock_guard<std::recursive_mutex> lock(
                                live_state_mutex_);
                            RequestFullSnapshots("periodic_resync");
                        });
                });
        }

        publication_worker_thread_ = std::thread(
            [this]()
            {
                GlobalPublicationWorker();
            });
        secondary_worker_thread_ = std::thread(
            [this]()
            {
                SecondaryWorkerLoop();
            });

        RCLCPP_WARN(
            get_logger(),
            "[F1K-SECONDARY-WORKER-CONFIG] workers=1 priority=fiducial_then_loop active_task_non_preemptive=true high_watermark=%d low_watermark=%d max_pending=%d topic=%s queue_payload=ids_revisions_changeset",
            loop_backlog_high_watermark_,
            loop_backlog_low_watermark_,
            loop_task_max_pending_,
            mapping_backpressure_topic_.c_str());
    }

    ~GlobalMapServer() override
    {
        secondary_worker_shutdown_.store(true);
        secondary_task_condition_.notify_all();
        if (secondary_worker_thread_.joinable())
        {
            secondary_worker_thread_.join();
        }
        publication_worker_shutdown_.store(true);
        publication_condition_.notify_all();
        if (publication_worker_thread_.joinable())
        {
            publication_worker_thread_.join();
        }
        if (rawdb_record_enabled_ && !rawdb_replay_enabled_)
        {
            SaveRawDatabase("shutdown");
        }
    }

private:
    // F1B: lee `use_sim_time` de forma tolerante porque ROS 2 puede declararlo
    // desde launch antes de que este nodo lo declare manualmente.
    bool DeclareOrReadUseSimTime()
    {
        bool value = false;

        try
        {
            value = declare_parameter<bool>("use_sim_time", false);
        }
        catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
        {
            get_parameter("use_sim_time", value);
        }

        return value;
    }

    // F1B: construye el namespace publico esperado por `simulacion_dron`
    // (`/dron_1`, `/dron_2`, ...). La regla evita codificar topics sueltos en
    // varios puntos del servidor.
    std::string DroneNamespace(uint32_t drone_id) const
    {
        return "/" + namespace_base_ + "_" + std::to_string(drone_id);
    }

    RawSubmapId DebugPoseStoreSubmapId() const
    {
        return RawSubmapId{
            static_cast<uint32_t>(pose_store_debug_anchor_drone_id_),
            static_cast<uint64_t>(pose_store_debug_anchor_epoch_)};
    }

    Eigen::Matrix4d PlanarTransform(double x, double y, double z, double yaw_rad) const
    {
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        transform.block<3, 3>(0, 0) =
            Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        transform(0, 3) = x;
        transform(1, 3) = y;
        transform(2, 3) = z;
        return transform;
    }

    double YawFromTransform(const Eigen::Matrix4d& transform) const
    {
        return std::atan2(transform(1, 0), transform(0, 0));
    }

    double NormalizeAngle(double angle) const
    {
        const double pi = std::acos(-1.0);
        while (angle > pi)
        {
            angle -= 2.0 * pi;
        }
        while (angle < -pi)
        {
            angle += 2.0 * pi;
        }
        return angle;
    }

    double StampToSeconds(const builtin_interfaces::msg::Time& stamp) const
    {
        return static_cast<double>(stamp.sec) + 1e-9 * static_cast<double>(stamp.nanosec);
    }

    bool PoseMsgToMatrix(const geometry_msgs::msg::Pose& pose, Eigen::Matrix4d& out) const
    {
        const Eigen::Quaterniond q_raw(
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z);
        if (!std::isfinite(q_raw.w()) ||
            !std::isfinite(q_raw.x()) ||
            !std::isfinite(q_raw.y()) ||
            !std::isfinite(q_raw.z()) ||
            q_raw.norm() < 1e-9)
        {
            return false;
        }

        const Eigen::Quaterniond q = q_raw.normalized();
        out = Eigen::Matrix4d::Identity();
        out.block<3, 3>(0, 0) = q.toRotationMatrix();
        out(0, 3) = pose.position.x;
        out(1, 3) = pose.position.y;
        out(2, 3) = pose.position.z;
        return out.allFinite();
    }

    RecordedFiducialObservation ToRecordedObservation(
        const FiducialObservation& observation) const
    {
        const Eigen::Matrix3d rotation =
            observation.world_T_body_fiducial.block<3, 3>(0, 0);
        const Eigen::Quaterniond q(rotation);

        RecordedFiducialObservation recorded;
        recorded.arrival_id = observation.arrival_id;
        recorded.drone_id = observation.drone_id;
        recorded.map_epoch = observation.map_epoch;
        recorded.local_keyframe_id = observation.local_keyframe_id;
        recorded.global_keyframe_id = observation.global_keyframe_id;
        recorded.fiducial_id = observation.fiducial_id;
        recorded.world_x = observation.world_T_body_fiducial(0, 3);
        recorded.world_y = observation.world_T_body_fiducial(1, 3);
        recorded.world_z = observation.world_T_body_fiducial(2, 3);
        recorded.world_qx = q.x();
        recorded.world_qy = q.y();
        recorded.world_qz = q.z();
        recorded.world_qw = q.w();
        recorded.keyframe_stamp_sec = observation.keyframe_stamp_sec;
        recorded.gt_stamp_sec = observation.gt_stamp_sec;
        recorded.association_dt_sec = observation.association_dt_sec;
        recorded.distance_to_fiducial_m = observation.distance_to_fiducial_m;
        recorded.source = observation.source;
        return recorded;
    }

    FiducialObservation FromRecordedObservation(
        const RecordedFiducialObservation& recorded,
        const std::string& source_override) const
    {
        const Eigen::Quaterniond q_raw(
            recorded.world_qw,
            recorded.world_qx,
            recorded.world_qy,
            recorded.world_qz);
        const Eigen::Quaterniond q =
            q_raw.norm() > 1e-9 ? q_raw.normalized() : Eigen::Quaterniond::Identity();

        FiducialObservation observation;
        observation.arrival_id = recorded.arrival_id;
        observation.drone_id = recorded.drone_id;
        observation.map_epoch = recorded.map_epoch;
        observation.local_keyframe_id = recorded.local_keyframe_id;
        observation.global_keyframe_id = recorded.global_keyframe_id;
        observation.fiducial_id = recorded.fiducial_id;
        observation.world_T_body_fiducial = Eigen::Matrix4d::Identity();
        observation.world_T_body_fiducial.block<3, 3>(0, 0) = q.toRotationMatrix();
        observation.world_T_body_fiducial(0, 3) = recorded.world_x;
        observation.world_T_body_fiducial(1, 3) = recorded.world_y;
        observation.world_T_body_fiducial(2, 3) = recorded.world_z;
        observation.keyframe_stamp_sec = recorded.keyframe_stamp_sec;
        observation.gt_stamp_sec = recorded.gt_stamp_sec;
        observation.association_dt_sec = recorded.association_dt_sec;
        observation.distance_to_fiducial_m = recorded.distance_to_fiducial_m;
        observation.source = source_override;
        return observation;
    }

    void ConfigureBodyCameraTransform()
    {
        // F1F-hotfix: la observacion fiducial live/replay persiste la pose GT
        // del cuerpo del dron. El backend de poses necesita convertirla a la
        // camara de ORB-SLAM3 antes de calcular `world_T_local`.
        BodyCameraTransformConfig config;
        config.x = body_T_camera_x_;
        config.y = body_T_camera_y_;
        config.z = body_T_camera_z_;
        config.roll_deg = body_T_camera_roll_deg_;
        config.pitch_deg = body_T_camera_pitch_deg_;
        config.yaw_deg = body_T_camera_yaw_deg_;
        config.use_camera_optical_frame_convention =
            use_camera_optical_frame_convention_;
        config.source = "launch";
        pose_store_.ConfigureBodyCameraTransform(config);

        const Eigen::Matrix4d body_T_camera =
            pose_store_.GetBodyCameraTransform();
        RCLCPP_WARN(
            get_logger(),
            "[F1F-BODY-CAMERA-CONFIG] source=launch use_optical=%s body_T_camera_t=(%.3f,%.3f,%.3f) rpy_deg=(%.3f,%.3f,%.3f) matrix_t=(%.3f,%.3f,%.3f)",
            config.use_camera_optical_frame_convention ? "true" : "false",
            config.x,
            config.y,
            config.z,
            config.roll_deg,
            config.pitch_deg,
            config.yaw_deg,
            body_T_camera(0, 3),
            body_T_camera(1, 3),
            body_T_camera(2, 3));
    }

    void LoadFiducialConfig()
    {
        fiducials_.clear();
        const size_t count = fiducial_ids_.size();
        for (size_t i = 0; i < count; ++i)
        {
            if (i >= fiducial_x_.size() ||
                i >= fiducial_y_.size() ||
                i >= fiducial_z_.size() ||
                i >= fiducial_yaw_.size() ||
                i >= fiducial_radius_.size())
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1E-FID-CONFIG] status=skip index=%zu reason=vector_size_mismatch",
                    i);
                continue;
            }

            FiducialConfig config;
            config.id = static_cast<int>(fiducial_ids_[i]);
            config.world_T_fiducial =
                PlanarTransform(fiducial_x_[i], fiducial_y_[i], fiducial_z_[i], fiducial_yaw_[i]);
            config.radius_m = std::max(0.0, fiducial_radius_[i]);
            fiducials_.push_back(config);
        }
    }

    void ConfigureFiducialRevisit()
    {
        // F1H: estos umbrales solo clasifican una segunda observacion fiducial
        // como OK o como tarea pendiente. No mueven anchors ni ejecutan solver.
        FiducialRevisitConfig config;
        config.error_threshold_m = fiducial_revisit_error_threshold_m_;
        config.yaw_threshold_rad = fiducial_revisit_yaw_threshold_rad_;
        config.rot_threshold_rad = fiducial_revisit_rot_threshold_rad_;
        config.source = "launch";
        fiducial_anchor_manager_.ConfigureRevisitThresholds(config);

        RCLCPP_WARN(
            get_logger(),
            "[F1H-FID-REVISIT-CONFIG] threshold_t=%.3f threshold_yaw=%.3f threshold_rot=%.3f source=%s",
            config.error_threshold_m,
            config.yaw_threshold_rad,
            config.rot_threshold_rad,
            config.source.c_str());
    }

    void ConfigurePoseGraphBuilder()
    {
        // F1I: configura solo la construccion del problema temporal. Estos
        // parametros no habilitan solver ni aplicacion de poses.
        PoseGraphBuilderConfig config;
        config.min_vertices = static_cast<uint64_t>(pose_graph_min_vertices_);
        config.vertex_selection_ratio = pose_graph_vertex_selection_ratio_;
        config.anchor_stop_enabled = pose_graph_anchor_stop_enabled_;
        config.fiducial_connectivity_enabled =
            pose_graph_fiducial_connectivity_enabled_;
        config.branch_selection_enabled = pose_graph_branch_selection_enabled_;
        config.subdivision_candidate_radius_m =
            pose_graph_subdivision_candidate_radius_m_;
        config.fiducial_neighborhood_radius_m =
            pose_graph_fiducial_neighborhood_radius_m_;
        config.include_temporal_edges = pose_graph_include_temporal_edges_;
        config.temporal_edge_weight = pose_graph_temporal_edge_weight_;
        config.temporal_edge_weight_sparse = pose_graph_temporal_edge_weight_sparse_;
        config.covisibility_min_weight = f1m_covisibility_min_weight_;
        config.fiducial_hard_weight = pose_graph_fiducial_hard_weight_;
        config.fiducial_target_translation_weight =
            pose_graph_fiducial_target_translation_weight_;
        config.fiducial_target_rotation_weight =
            pose_graph_fiducial_target_rotation_weight_;
        config.current_pose_soft_weight = pose_graph_current_pose_soft_weight_;
        config.fiducial_neighborhood_radius_kfs = static_cast<uint64_t>(
            pose_graph_fiducial_neighborhood_radius_kfs_);
        config.fiducial_neighborhood_vertex_ratio =
            pose_graph_fiducial_neighborhood_vertex_ratio_;
        config.corner_yaw_threshold_rad = pose_graph_corner_yaw_threshold_rad_;
        pose_graph_builder_.Configure(config);

        RCLCPP_WARN(
            get_logger(),
            "[F1I-GRAPH-BUILDER-CONFIG] vertex_policy=balanced_coverage_sample vertex_selection_ratio=%.3f vertex_limit=none edge_length_limit_m=none mandatory_fiducial_vertices=2 min_vertices=%llu anchor_stop=%s fid_connectivity=%s branch_selection=%s subdivision_radius=%.3f fiducial_neighborhood_radius_kfs=%llu fiducial_neighborhood_vertex_ratio=%.3f corner_3d_threshold_rad=%.3f temporal_edges=%s covisibility_edges=%s weights_temporal=%.3f sparse=%.3f fid_hard=%.3f fid_target_t=%.3f fid_target_rot=%.3f current_soft=%.3f debug_force=%s",
            config.vertex_selection_ratio,
            static_cast<unsigned long long>(config.min_vertices),
            config.anchor_stop_enabled ? "true" : "false",
            config.fiducial_connectivity_enabled ? "true" : "false",
            config.branch_selection_enabled ? "true" : "false",
            config.subdivision_candidate_radius_m,
            static_cast<unsigned long long>(config.fiducial_neighborhood_radius_kfs),
            config.fiducial_neighborhood_vertex_ratio,
            config.corner_yaw_threshold_rad,
            config.include_temporal_edges ? "true" : "false",
            pose_graph_use_covisibility_edges_ ? "true" : "false",
            config.temporal_edge_weight,
            config.temporal_edge_weight_sparse,
            config.fiducial_hard_weight,
            config.fiducial_target_translation_weight,
            config.fiducial_target_rotation_weight,
            config.current_pose_soft_weight,
            f1i_debug_force_task_enabled_ ? "true" : "false");
        // Nombre de marcador legacy: la vecindad fiducial se construye en 1I.
        // Se conserva `F1L-*` en logs para no romper reducciones históricas.
        RCLCPP_WARN(
            get_logger(),
            "[F1L-FIDUCIAL-NEIGHBORHOOD-CONFIG] radius_m=%.3f legacy_radius_kfs=%llu policy=metric_pose_distance",
            config.fiducial_neighborhood_radius_m,
            static_cast<unsigned long long>(config.fiducial_neighborhood_radius_kfs));
    }

    void ConfigureLoopDetector()
    {
        // F1N: BoW propone candidatos visuales; no confirma loops, no construye
        // subnubes y no toca poses. La covisibilidad confirmada se usa solo para
        // saltar pares que ya no necesitan verificacion geometrica costosa.
        LoopDetectorConfig config;
        config.min_kf_gap_same_submap = static_cast<uint64_t>(
            loop_bow_min_kf_gap_same_submap_);
        config.max_candidates = static_cast<uint64_t>(loop_bow_max_candidates_);
        config.max_candidates_per_submap = static_cast<uint64_t>(
            loop_bow_max_candidates_per_submap_);
        config.min_mappoints = static_cast<uint64_t>(loop_bow_min_mappoints_);
        config.min_bow_score = loop_bow_min_score_;
        config.covisibility_strength.min_support = static_cast<uint64_t>(
            aligned_overlap_strict_min_matches_);
        config.covisibility_strength.min_shared_ratio =
            aligned_overlap_strict_min_match_ratio_;
        config.covisibility_strength.min_image_bins = static_cast<uint64_t>(
            aligned_overlap_strict_min_image_bins_);
        config.covisibility_strength.min_spatial_coverage_ratio =
            aligned_overlap_strict_min_3d_span_ratio_;
        loop_detector_.Configure(config);

        RCLCPP_WARN(
            get_logger(),
            "[F1N-LOOP-DETECTOR-CONFIG] min_kf_gap_same_submap=%llu max_candidates=%llu max_candidates_per_submap=%llu min_mappoints=%llu min_bow_score=%.6f",
            static_cast<unsigned long long>(config.min_kf_gap_same_submap),
            static_cast<unsigned long long>(config.max_candidates),
            static_cast<unsigned long long>(config.max_candidates_per_submap),
            static_cast<unsigned long long>(config.min_mappoints),
            config.min_bow_score);
    }

    void ConfigureSubcloudLoopVerifier()
    {
        // F1O: comprueba geometria para candidatos BoW y calcula error de pose
        // preliminar. No fusiona landmarks, no optimiza y no cambia poses.
        SubcloudLoopVerifierConfig config;
        config.query_subcloud_min_points = static_cast<uint64_t>(
            loop_verify_query_subcloud_min_points_);
        config.candidate_window_max_kfs = static_cast<uint64_t>(
            loop_verify_candidate_window_max_kfs_);
        config.candidate_window_covisibility_min_weight = static_cast<uint64_t>(
            loop_verify_candidate_window_covisibility_min_weight_);
        config.candidate_window_temporal_kf_radius = static_cast<uint64_t>(
            loop_verify_candidate_window_temporal_kf_radius_);
        config.candidate_window_spatial_radius_m =
            loop_verify_candidate_window_spatial_radius_m_;
        config.candidate_subcloud_min_points = static_cast<uint64_t>(
            loop_verify_candidate_subcloud_min_points_);
        config.candidate_subcloud_max_points = static_cast<uint64_t>(
            loop_verify_candidate_subcloud_max_points_);
        config.candidate_subcloud_min_score = static_cast<float>(
            loop_verify_candidate_subcloud_min_score_);
        config.orb_match_max_hamming = static_cast<uint32_t>(
            loop_verify_orb_match_max_hamming_);
        config.orb_match_ratio_test = loop_verify_orb_match_ratio_test_;
        config.orb_match_cross_check = loop_verify_orb_match_cross_check_;
        config.min_initial_matches = static_cast<uint64_t>(
            loop_verify_min_initial_matches_);
        config.candidate_reduce_enabled = loop_verify_candidate_reduce_enabled_;
        config.candidate_reduce_min_initial_matches = static_cast<uint64_t>(
            loop_verify_candidate_reduce_min_initial_matches_);
        config.candidate_reduce_margin_m = loop_verify_candidate_reduce_margin_m_;
        config.candidate_reduce_min_points_after = static_cast<uint64_t>(
            loop_verify_candidate_reduce_min_points_after_);
        config.ransac_min_matches = static_cast<uint64_t>(
            loop_verify_ransac_min_matches_);
        config.ransac_max_iterations = static_cast<uint64_t>(
            loop_verify_ransac_max_iterations_);
        config.ransac_inlier_threshold_m =
            loop_verify_ransac_inlier_threshold_m_;
        config.ransac_min_inliers = static_cast<uint64_t>(
            loop_verify_ransac_min_inliers_);
        config.ransac_min_inlier_ratio = loop_verify_ransac_min_inlier_ratio_;
        config.accept_mean_residual_m = loop_verify_accept_mean_residual_m_;
        config.accept_max_residual_m = loop_verify_accept_max_residual_m_;
        config.fusion_error_t_m = loop_verify_fusion_error_t_m_;
        config.fusion_error_yaw_rad = loop_verify_fusion_error_yaw_rad_;
        config.aligned_overlap_enabled = aligned_overlap_enabled_;
        config.aligned_overlap_keyframe_radius_m =
            aligned_overlap_keyframe_radius_m_;
        config.aligned_overlap_max_candidate_kfs = static_cast<uint64_t>(
            aligned_overlap_max_candidate_kfs_);
        config.aligned_overlap_strict_position_m =
            aligned_overlap_strict_position_m_;
        config.aligned_overlap_strict_max_hamming = static_cast<uint32_t>(
            aligned_overlap_strict_max_hamming_);
        config.aligned_overlap_strict_ratio_test =
            aligned_overlap_strict_ratio_test_;
        config.aligned_overlap_strict_min_matches = static_cast<uint64_t>(
            aligned_overlap_strict_min_matches_);
        config.aligned_overlap_strict_min_match_ratio =
            aligned_overlap_strict_min_match_ratio_;
        config.aligned_overlap_strict_min_image_bins = static_cast<uint64_t>(
            aligned_overlap_strict_min_image_bins_);
        config.aligned_overlap_strict_min_3d_span_ratio =
            aligned_overlap_strict_min_3d_span_ratio_;
        config.aligned_overlap_strict_mean_residual_m =
            aligned_overlap_strict_mean_residual_m_;
        config.aligned_overlap_strict_max_residual_m =
            aligned_overlap_strict_max_residual_m_;
        config.aligned_overlap_expand_position_m =
            aligned_overlap_expand_position_m_;
        config.aligned_overlap_expand_max_hamming = static_cast<uint32_t>(
            aligned_overlap_expand_max_hamming_);
        config.aligned_overlap_expand_ratio_test =
            aligned_overlap_expand_ratio_test_;
        config.covisibility_strength.min_support = static_cast<uint64_t>(
            aligned_overlap_strict_min_matches_);
        config.covisibility_strength.min_shared_ratio =
            aligned_overlap_strict_min_match_ratio_;
        config.covisibility_strength.min_image_bins = static_cast<uint64_t>(
            aligned_overlap_strict_min_image_bins_);
        config.covisibility_strength.min_spatial_coverage_ratio =
            aligned_overlap_strict_min_3d_span_ratio_;

        subcloud_loop_verifier_.Configure(config);

        RCLCPP_WARN(
            get_logger(),
            "[F1N-SUBCLOUD-VERIFIER-CONFIG] max_candidates_per_query=%d query_min_points=%llu window_max_kfs=%llu covis_min_weight=%llu temporal_radius=%llu spatial_radius_m=%.3f candidate_min_points=%llu candidate_max_points=%llu min_score=%.3f max_hamming=%u ratio_test=%.3f cross_check=%s min_initial_matches=%llu reduce_enabled=%s reduce_min_matches=%llu reduce_margin_m=%.3f reduce_min_points_after=%llu ransac_min_matches=%llu ransac_iterations=%llu ransac_threshold_m=%.3f ransac_min_inliers=%llu ransac_min_ratio=%.3f accept_mean_m=%.3f accept_max_m=%.3f fusion_error_t_m=%.3f fusion_error_yaw_rad=%.3f",
            loop_verify_max_candidates_per_query_,
            static_cast<unsigned long long>(config.query_subcloud_min_points),
            static_cast<unsigned long long>(config.candidate_window_max_kfs),
            static_cast<unsigned long long>(config.candidate_window_covisibility_min_weight),
            static_cast<unsigned long long>(config.candidate_window_temporal_kf_radius),
            config.candidate_window_spatial_radius_m,
            static_cast<unsigned long long>(config.candidate_subcloud_min_points),
            static_cast<unsigned long long>(config.candidate_subcloud_max_points),
            config.candidate_subcloud_min_score,
            config.orb_match_max_hamming,
            config.orb_match_ratio_test,
            config.orb_match_cross_check ? "true" : "false",
            static_cast<unsigned long long>(config.min_initial_matches),
            config.candidate_reduce_enabled ? "true" : "false",
            static_cast<unsigned long long>(config.candidate_reduce_min_initial_matches),
            config.candidate_reduce_margin_m,
            static_cast<unsigned long long>(config.candidate_reduce_min_points_after),
            static_cast<unsigned long long>(config.ransac_min_matches),
            static_cast<unsigned long long>(config.ransac_max_iterations),
            config.ransac_inlier_threshold_m,
            static_cast<unsigned long long>(config.ransac_min_inliers),
            config.ransac_min_inlier_ratio,
            config.accept_mean_residual_m,
            config.accept_max_residual_m,
            config.fusion_error_t_m,
            config.fusion_error_yaw_rad);
        RCLCPP_WARN(
            get_logger(),
            "[F1P-ALIGNED-CONFIG] enabled=%s kf_radius_m=%.3f max_candidate_kfs=%llu strict_position_m=%.3f strict_max_hamming=%u strict_ratio=%.3f strict_min_matches=%llu strict_min_match_ratio=%.3f strict_min_image_bins=%llu strict_min_3d_span_ratio=%.3f strict_mean_residual_m=%.3f strict_max_residual_m=%.3f expand_position_m=%.3f expand_max_hamming=%u expand_ratio=%.3f",
            config.aligned_overlap_enabled ? "true" : "false",
            config.aligned_overlap_keyframe_radius_m,
            static_cast<unsigned long long>(
                config.aligned_overlap_max_candidate_kfs),
            config.aligned_overlap_strict_position_m,
            config.aligned_overlap_strict_max_hamming,
            config.aligned_overlap_strict_ratio_test,
            static_cast<unsigned long long>(
                config.aligned_overlap_strict_min_matches),
            config.aligned_overlap_strict_min_match_ratio,
            static_cast<unsigned long long>(
                config.aligned_overlap_strict_min_image_bins),
            config.aligned_overlap_strict_min_3d_span_ratio,
            config.aligned_overlap_strict_mean_residual_m,
            config.aligned_overlap_strict_max_residual_m,
            config.aligned_overlap_expand_position_m,
            config.aligned_overlap_expand_max_hamming,
            config.aligned_overlap_expand_ratio_test);
    }

    void ConfigureOptimizationManager()
    {
        // F1J: el dry-run calcula propuestas en memoria. La exportacion SVG es
        // opcional y desactivada por defecto; no forma parte del camino normal
        // ni se lee de vuelta desde el servidor.
        OptimizationManagerConfig config;
        config.dryrun_min_improvement_ratio = f1j_dryrun_min_improvement_ratio_;
        config.dryrun_partial_min_improvement_ratio =
            f1j_dryrun_partial_min_improvement_ratio_;
        config.dryrun_max_final_error_t = f1j_dryrun_max_final_error_t_;
        config.dryrun_require_cost_decrease = f1j_dryrun_require_cost_decrease_;
        config.solver_step_fraction = f1j_solver_step_fraction_;
        config.post_apply_internal_broken_edge_t =
            f1l_post_apply_internal_broken_edge_t_;
        config.post_apply_internal_max_growth_t =
            f1l_post_apply_internal_max_growth_t_;
        config.post_apply_fiducial_absurd_error_t =
            f1l_post_apply_fiducial_absurd_error_t_;
        optimization_manager_.Configure(config);

        RCLCPP_WARN(
            get_logger(),
            "[F1J-OPT-CONFIG] enabled=%s min_improvement=%.3f partial_min_improvement=%.3f max_final_error_t=%.3f require_cost_decrease=%s step=%.3f movement_deltas=diagnostic_only export_debug_plot=%s output_dir=%s",
            f1j_dryrun_enabled_ ? "true" : "false",
            config.dryrun_min_improvement_ratio,
            config.dryrun_partial_min_improvement_ratio,
            config.dryrun_max_final_error_t,
            config.dryrun_require_cost_decrease ? "true" : "false",
            config.solver_step_fraction,
            f1j_export_debug_plot_ ? "true" : "false",
            f1j_debug_output_dir_.c_str());
        RCLCPP_WARN(
            get_logger(),
            "[F1K-OPT-APPLY-CONFIG] enabled=%s policy=stable_priority_single_persistent_worker_candidate_validate_atomic_commit_async_publication partial_candidate_policy=f1l_controlled",
            f1k_apply_enabled_ ? "true" : "false");
        RCLCPP_WARN(
            get_logger(),
            "[F1L-POST-APPLY-CONFIG] validation_enabled=%s partial_apply_enabled=%s max_partial_retries=%d force_reject_once=%s force_reject_task_id=%d internal_broken_edge_t=%.3f internal_max_growth_t=%.3f fiducial_absurd_error_t=%.3f",
            f1l_validation_enabled_ ? "true" : "false",
            f1l_partial_apply_enabled_ ? "true" : "false",
            f1l_max_partial_retries_,
            f1l_debug_force_reject_once_ ? "true" : "false",
            f1l_debug_force_reject_task_id_,
            f1l_post_apply_internal_broken_edge_t_,
            f1l_post_apply_internal_max_growth_t_,
            f1l_post_apply_fiducial_absurd_error_t_);
        RCLCPP_WARN(
            get_logger(),
            "[F1L-GT-DEBUG-CONFIG] enabled=%s max_dt=%.3f source=GAZEBO_GT_DEBUG usage=metrics_only",
            f1l_gt_kf_debug_enabled_ ? "true" : "false",
            f1l_gt_kf_debug_max_dt_sec_);
        // Nombre de parámetro legacy: este HTML visualiza el dry-run de 1J.
        // 1L solo lo compara contra logs/RViz2 si hay diagnóstico post-apply.
        RCLCPP_WARN(
            get_logger(),
            "[F1L-DEBUG-ANIMATION-CONFIG] enabled=%s output_dir=%s format=html_animation usage=diagnostic_only",
            f1l_debug_animation_enabled_ ? "true" : "false",
            f1l_debug_animation_output_dir_.c_str());
    }

    void LogFiducialInit()
    {
        RCLCPP_WARN(
            get_logger(),
            "[F1E-FID-INIT] sim_enabled=%s fiducials=%zu gt_max_dt=%.3f gt_buffer=%d",
            fiducial_sim_enabled_ ? "true" : "false",
            fiducials_.size(),
            fiducial_gt_max_dt_sec_,
            fiducial_gt_buffer_max_samples_);

        for (const auto& fiducial : fiducials_)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1E-FID-CONFIG] id=%d world_t=(%.3f,%.3f,%.3f) yaw=%.3f radius=%.3f",
                fiducial.id,
                fiducial.world_T_fiducial(0, 3),
                fiducial.world_T_fiducial(1, 3),
                fiducial.world_T_fiducial(2, 3),
                YawFromTransform(fiducial.world_T_fiducial),
                fiducial.radius_m);
        }
    }

    void LogPoseStoreInit()
    {
        const auto stats = pose_store_.GetPoseStoreStats();
        RCLCPP_WARN(
            get_logger(),
            "[F1D-POSESTORE-INIT] anchors=%llu world_poses=%llu optimized_kfs=%llu propagated_kfs=%llu rebased_kfs=%llu corrections=%llu hard_fiducial_kfs=%llu accepted_kf_anchors=%llu active_tail_anchors=%llu derived_tail_kfs=%llu debug_enabled=%s",
            static_cast<unsigned long long>(stats.anchored_submaps),
            static_cast<unsigned long long>(stats.keyframe_world_poses),
            static_cast<unsigned long long>(stats.optimized_keyframes),
            static_cast<unsigned long long>(stats.propagated_keyframes),
            static_cast<unsigned long long>(stats.rebased_keyframes),
            static_cast<unsigned long long>(stats.submap_corrections),
            static_cast<unsigned long long>(stats.hard_fiducial_keyframes),
            static_cast<unsigned long long>(stats.accepted_keyframe_anchors),
            static_cast<unsigned long long>(stats.active_tail_anchors),
            static_cast<unsigned long long>(stats.derived_tail_keyframes),
            pose_store_debug_enabled_ ? "true" : "false");
        LogPoseStoreStats("init");
    }

    void LogPoseStoreStats(const std::string& reason)
    {
        const auto stats = pose_store_.GetPoseStoreStats();
        RCLCPP_WARN(
            get_logger(),
            "[F1D-POSESTORE-STATS] reason=%s anchors=%llu world_poses=%llu optimized_kfs=%llu propagated_kfs=%llu rebased_kfs=%llu corrections=%llu hard_fiducial_kfs=%llu accepted_kf_anchors=%llu active_tail_anchors=%llu derived_tail_kfs=%llu",
            reason.c_str(),
            static_cast<unsigned long long>(stats.anchored_submaps),
            static_cast<unsigned long long>(stats.keyframe_world_poses),
            static_cast<unsigned long long>(stats.optimized_keyframes),
            static_cast<unsigned long long>(stats.propagated_keyframes),
            static_cast<unsigned long long>(stats.rebased_keyframes),
            static_cast<unsigned long long>(stats.submap_corrections),
            static_cast<unsigned long long>(stats.hard_fiducial_keyframes),
            static_cast<unsigned long long>(stats.accepted_keyframe_anchors),
            static_cast<unsigned long long>(stats.active_tail_anchors),
            static_cast<unsigned long long>(stats.derived_tail_keyframes));

        RCLCPP_WARN(
            get_logger(),
            "[F1D-SERVER-POSESTORE-STATS] reason=%s debug_enabled=%s anchor_done=%s opt_done=%s anchors=%llu world_poses=%llu optimized_kfs=%llu propagated_kfs=%llu rebased_kfs=%llu corrections=%llu hard_fiducial_kfs=%llu accepted_kf_anchors=%llu active_tail_anchors=%llu derived_tail_kfs=%llu",
            reason.c_str(),
            pose_store_debug_enabled_ ? "true" : "false",
            pose_store_debug_anchor_done_ ? "true" : "false",
            pose_store_debug_opt_done_ ? "true" : "false",
            static_cast<unsigned long long>(stats.anchored_submaps),
            static_cast<unsigned long long>(stats.keyframe_world_poses),
            static_cast<unsigned long long>(stats.optimized_keyframes),
            static_cast<unsigned long long>(stats.propagated_keyframes),
            static_cast<unsigned long long>(stats.rebased_keyframes),
            static_cast<unsigned long long>(stats.submap_corrections),
            static_cast<unsigned long long>(stats.hard_fiducial_keyframes),
            static_cast<unsigned long long>(stats.accepted_keyframe_anchors),
            static_cast<unsigned long long>(stats.active_tail_anchors),
            static_cast<unsigned long long>(stats.derived_tail_keyframes));

        const auto fid_stats = fiducial_anchor_manager_.GetStats();
        RCLCPP_WARN(
            get_logger(),
            "[F1E-POSESTORE-STATS] reason=%s anchors=%llu world_poses=%llu hard_fiducial_kfs=%llu fid_observations=%llu fid_accepted=%llu fid_replay=%llu",
            reason.c_str(),
            static_cast<unsigned long long>(stats.anchored_submaps),
            static_cast<unsigned long long>(stats.keyframe_world_poses),
            static_cast<unsigned long long>(stats.hard_fiducial_keyframes),
            static_cast<unsigned long long>(fid_stats.observations),
            static_cast<unsigned long long>(fid_stats.accepted),
            static_cast<unsigned long long>(fid_stats.replay_observations));
    }

    void LogScoreInit()
    {
        const auto stats = score_manager_.GetStats();
        RCLCPP_WARN(
            get_logger(),
            "[F1F-SCORE-INIT] tracked_points=%llu policy=orbslam_raw_quality observations_weight=0.55 found_ratio_weight=0.35 descriptor_weight=0.10",
            static_cast<unsigned long long>(stats.tracked_points));
    }

    void UpdateScoresFromMap(const OrbMap& map, uint64_t arrival_id)
    {
        // F1F: el servidor informa eventos raw al manager, pero no calcula la
        // formula de score. La politica vive en LandmarkScoreManager.
        uint64_t created = 0;
        uint64_t updated = 0;
        uint64_t marked_bad = 0;
        bool has_first_created = false;
        LandmarkScoreUpdateResult first_created;

        for (const auto& mappoint : map.mappoints)
        {
            const RawMapPointId id{map.drone_id, map.map_epoch, mappoint.id};
            const auto update = score_manager_.ApplyOrbSlamQuality(id, mappoint);
            if (update.created)
            {
                ++created;
                if (!has_first_created)
                {
                    first_created = update;
                    has_first_created = true;
                }
            }
            else
            {
                ++updated;
            }
            if (update.record.is_bad)
            {
                ++marked_bad;
            }
        }

        if (has_first_created)
        {
            const auto& record = first_created.record;
            RCLCPP_WARN(
                get_logger(),
                "[F1F-SCORE-INIT] arrival_id=%llu mp=%llu drone_id=%u epoch=%llu obs=%u found_ratio=%.3f is_bad=%s descriptor_valid=%s score=%.3f reason=%s new_points=%llu",
                static_cast<unsigned long long>(arrival_id),
                static_cast<unsigned long long>(record.id.local_mp_id),
                record.id.drone_id,
                static_cast<unsigned long long>(record.id.map_epoch),
                record.observations,
                record.found_ratio,
                record.is_bad ? "true" : "false",
                record.descriptor_valid ? "true" : "false",
                record.score,
                orbslam3_multi::ToString(record.last_event),
                static_cast<unsigned long long>(created));
        }

        const auto stats = score_manager_.GetStats();
        RCLCPP_WARN(
            get_logger(),
            "[F1F-SCORE-UPDATE-ORBSLAM] arrival_id=%llu drone_id=%u epoch=%llu delta_mps=%zu created=%llu updated=%llu marked_bad=%llu tracked_points=%llu",
            static_cast<unsigned long long>(arrival_id),
            map.drone_id,
            static_cast<unsigned long long>(map.map_epoch),
            map.mappoints.size(),
            static_cast<unsigned long long>(created),
            static_cast<unsigned long long>(updated),
            static_cast<unsigned long long>(marked_bad),
            static_cast<unsigned long long>(stats.tracked_points));
        LogScoreStats("orbslam_raw_update");
    }

    void LogScoreStats(const std::string& reason)
    {
        const auto stats = score_manager_.GetStats();
        RCLCPP_WARN(
            get_logger(),
            "[F1F-SCORE-STATS] reason=%s tracked_points=%llu score_min=%.3f score_mean=%.3f score_max=%.3f bad_points=%llu",
            reason.c_str(),
            static_cast<unsigned long long>(stats.tracked_points),
            stats.score_min,
            stats.score_mean,
            stats.score_max,
            static_cast<unsigned long long>(stats.bad_points));
    }

    sensor_msgs::msg::PointCloud2 BuildPointCloud2(
        const std::vector<orbslam3_multi::GlobalSparsePoint>& points) const
    {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp = now();
        cloud.header.frame_id = world_frame_;
        cloud.height = 1;
        cloud.width = static_cast<uint32_t>(points.size());
        cloud.is_bigendian = false;
        cloud.is_dense = false;

        auto add_field =
            [&cloud](const std::string& name,
                     uint32_t offset,
                     uint8_t datatype)
            {
                sensor_msgs::msg::PointField field;
                field.name = name;
                field.offset = offset;
                field.datatype = datatype;
                field.count = 1;
                cloud.fields.push_back(field);
            };

        add_field("x", 0, sensor_msgs::msg::PointField::FLOAT32);
        add_field("y", 4, sensor_msgs::msg::PointField::FLOAT32);
        add_field("z", 8, sensor_msgs::msg::PointField::FLOAT32);
        add_field("score", 12, sensor_msgs::msg::PointField::FLOAT32);
        add_field("drone_id", 16, sensor_msgs::msg::PointField::UINT32);
        add_field("map_epoch", 20, sensor_msgs::msg::PointField::UINT32);

        cloud.point_step = 24;
        cloud.row_step = cloud.point_step * cloud.width;
        cloud.data.resize(static_cast<size_t>(cloud.row_step));

        for (size_t i = 0; i < points.size(); ++i)
        {
            const auto& point = points[i];
            const size_t base = i * cloud.point_step;
            const float x = static_cast<float>(point.x);
            const float y = static_cast<float>(point.y);
            const float z = static_cast<float>(point.z);
            const float score = point.score;
            const uint32_t drone_id = point.drone_id;
            const uint32_t map_epoch =
                point.map_epoch > std::numeric_limits<uint32_t>::max()
                    ? std::numeric_limits<uint32_t>::max()
                    : static_cast<uint32_t>(point.map_epoch);

            std::memcpy(&cloud.data[base + 0], &x, sizeof(float));
            std::memcpy(&cloud.data[base + 4], &y, sizeof(float));
            std::memcpy(&cloud.data[base + 8], &z, sizeof(float));
            std::memcpy(&cloud.data[base + 12], &score, sizeof(float));
            std::memcpy(&cloud.data[base + 16], &drone_id, sizeof(uint32_t));
            std::memcpy(&cloud.data[base + 20], &map_epoch, sizeof(uint32_t));
        }

        return cloud;
    }

    using MarkerIdentity = std::pair<std::string, int32_t>;

    struct KeyFrameMarkerBuildResult
    {
        visualization_msgs::msg::MarkerArray message;
        std::set<MarkerIdentity> identities;
        uint64_t keyframes_published = 0;
        uint64_t keyframes_without_world_pose = 0;
        uint64_t bad_keyframes_skipped = 0;
        std::set<RawSubmapId> submaps_published;
    };

    static std::array<float, 3> ColorForSubmap(const RawSubmapId& submap_id)
    {
        static constexpr std::array<std::array<float, 3>, 12> palette{{
            {{0.90F, 0.20F, 0.20F}}, {{0.15F, 0.65F, 0.95F}},
            {{0.20F, 0.75F, 0.35F}}, {{0.95F, 0.65F, 0.10F}},
            {{0.65F, 0.30F, 0.90F}}, {{0.10F, 0.75F, 0.75F}},
            {{0.95F, 0.35F, 0.65F}}, {{0.55F, 0.75F, 0.15F}},
            {{0.25F, 0.40F, 0.95F}}, {{0.90F, 0.45F, 0.15F}},
            {{0.45F, 0.80F, 0.65F}}, {{0.75F, 0.25F, 0.45F}},
        }};
        const uint64_t hash =
            (static_cast<uint64_t>(submap_id.drone_id) * 0x9e3779b185ebca87ULL) ^
            (submap_id.map_epoch * 0xc2b2ae3d27d4eb4fULL);
        return palette[hash % palette.size()];
    }

    KeyFrameMarkerBuildResult BuildGlobalKeyFrameMarkers(
        const RawMapDatabase& raw_snapshot,
        const GlobalPoseStore& pose_snapshot,
        const builtin_interfaces::msg::Time& stamp) const
    {
        KeyFrameMarkerBuildResult result;
        const double depth = global_keyframes_frustum_scale_;
        const double half_width = depth * 0.60;
        const double half_height = depth * 0.40;

        for (const auto& submap_id : raw_snapshot.GetSubmapIds())
        {
            const auto color = ColorForSubmap(submap_id);
            for (const auto& keyframe_id :
                 raw_snapshot.GetKeyFrameIdsForSubmap(submap_id))
            {
                const auto* raw_keyframe = raw_snapshot.GetKeyFrame(keyframe_id);
                if (!raw_keyframe || raw_keyframe->is_bad)
                {
                    ++result.bad_keyframes_skipped;
                    continue;
                }
                const auto world_T_keyframe = pose_snapshot.GetWorldPose(keyframe_id);
                if (!world_T_keyframe || !world_T_keyframe->allFinite())
                {
                    ++result.keyframes_without_world_pose;
                    continue;
                }

                Eigen::Quaterniond orientation(
                    world_T_keyframe->block<3, 3>(0, 0));
                if (!std::isfinite(orientation.norm()) || orientation.norm() < 1e-9)
                {
                    ++result.keyframes_without_world_pose;
                    continue;
                }
                orientation.normalize();

                const std::string prefix =
                    "dron_" + std::to_string(keyframe_id.drone_id) +
                    "/epoch_" + std::to_string(keyframe_id.map_epoch) +
                    "/kf_" + std::to_string(keyframe_id.local_kf_id);
                visualization_msgs::msg::Marker frustum;
                frustum.header.frame_id = world_frame_;
                frustum.header.stamp = stamp;
                frustum.ns = prefix + "/frustum";
                frustum.id = 0;
                frustum.type = visualization_msgs::msg::Marker::LINE_LIST;
                frustum.action = visualization_msgs::msg::Marker::ADD;
                frustum.pose.position.x = (*world_T_keyframe)(0, 3);
                frustum.pose.position.y = (*world_T_keyframe)(1, 3);
                frustum.pose.position.z = (*world_T_keyframe)(2, 3);
                frustum.pose.orientation.x = orientation.x();
                frustum.pose.orientation.y = orientation.y();
                frustum.pose.orientation.z = orientation.z();
                frustum.pose.orientation.w = orientation.w();
                frustum.scale.x = std::max(0.005, depth * 0.06);
                frustum.color.r = color[0];
                frustum.color.g = color[1];
                frustum.color.b = color[2];
                frustum.color.a = 1.0F;

                const auto point = [](double x, double y, double z)
                {
                    geometry_msgs::msg::Point value;
                    value.x = x;
                    value.y = y;
                    value.z = z;
                    return value;
                };
                const geometry_msgs::msg::Point origin = point(0.0, 0.0, 0.0);
                const std::array<geometry_msgs::msg::Point, 4> corners{{
                    point(-half_width, -half_height, depth),
                    point(half_width, -half_height, depth),
                    point(half_width, half_height, depth),
                    point(-half_width, half_height, depth),
                }};
                for (const auto& corner : corners)
                {
                    frustum.points.push_back(origin);
                    frustum.points.push_back(corner);
                }
                for (size_t i = 0; i < corners.size(); ++i)
                {
                    frustum.points.push_back(corners[i]);
                    frustum.points.push_back(corners[(i + 1U) % corners.size()]);
                }
                result.identities.insert({frustum.ns, frustum.id});
                result.message.markers.push_back(std::move(frustum));

                if (global_keyframes_labels_enabled_)
                {
                    visualization_msgs::msg::Marker label;
                    label.header.frame_id = world_frame_;
                    label.header.stamp = stamp;
                    label.ns = prefix + "/label";
                    label.id = 1;
                    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                    label.action = visualization_msgs::msg::Marker::ADD;
                    label.pose.position.x = (*world_T_keyframe)(0, 3);
                    label.pose.position.y = (*world_T_keyframe)(1, 3);
                    label.pose.position.z =
                        (*world_T_keyframe)(2, 3) + depth * 0.75;
                    label.pose.orientation.w = 1.0;
                    label.scale.z = std::max(0.06, depth * 0.55);
                    label.color.r = color[0];
                    label.color.g = color[1];
                    label.color.b = color[2];
                    label.color.a = 1.0F;
                    label.text =
                        "d" + std::to_string(keyframe_id.drone_id) +
                        " e" + std::to_string(keyframe_id.map_epoch) +
                        " kf" + std::to_string(keyframe_id.local_kf_id);
                    result.identities.insert({label.ns, label.id});
                    result.message.markers.push_back(std::move(label));
                }

                ++result.keyframes_published;
                result.submaps_published.insert(submap_id);
            }
        }
        return result;
    }

    void BuildAndPublishGlobalState(
        const std::string& reason,
        uint64_t publication_revision)
    {
        const auto capture_start = std::chrono::steady_clock::now();
        RawMapDatabase raw_snapshot;
        GlobalPoseStore pose_snapshot;
        LandmarkScoreManager score_snapshot;
        FusedLandmarkManager fused_snapshot;
        {
            std::lock_guard<std::recursive_mutex> lock(live_state_mutex_);
            raw_snapshot = raw_db_.CreateStateSnapshot();
            pose_snapshot = pose_store_;
            score_snapshot = score_manager_;
            fused_snapshot = fused_landmark_manager_;
        }
        const auto raw_stats = raw_snapshot.GetDatabaseStats();
        TryTraceFlow(
            "raw_db_map_builder",
            "snapshot",
            reason,
            0,
            raw_stats.keyframes);
        TryTraceFlow(
            "pose_db_map_builder",
            "snapshot",
            reason,
            0,
            raw_stats.keyframes);
        TryTraceFlow(
            "score_db_map_builder",
            "snapshot",
            reason,
            0,
            raw_stats.mappoints);
        TryTraceFlow(
            "fused_db_map_builder",
            "snapshot",
            reason,
            0,
            fused_snapshot.GetStats().tracks);
        {
            std::lock_guard<std::mutex> lock(publication_mutex_);
            publication_captured_revision_ = std::max(
                publication_captured_revision_, publication_revision);
        }
        publication_condition_.notify_all();
        const auto build_start = std::chrono::steady_clock::now();
        const GlobalMapBuildResult build =
            global_map_builder_.Build(
                raw_snapshot,
                pose_snapshot,
                score_snapshot,
                &fused_snapshot,
                static_cast<float>(global_map_min_score_to_publish_));
        TryTraceFlow(
            "map_builder_server_view",
            "ready",
            reason,
            0,
            build.stats.returned_points);

        RCLCPP_WARN(
            get_logger(),
            "[F1F-GLOBALMAP-BUILD] reason=%s anchored_submaps=%llu skipped_unanchored=%llu raw_points=%llu candidate_points=%llu returned_points=%llu",
            reason.c_str(),
            static_cast<unsigned long long>(build.stats.anchored_submaps),
            static_cast<unsigned long long>(build.stats.skipped_unanchored_submaps),
            static_cast<unsigned long long>(build.stats.raw_points),
            static_cast<unsigned long long>(build.stats.candidate_points),
            static_cast<unsigned long long>(build.stats.returned_points));

        if (build.stats.skipped_unanchored_submaps > 0)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1F-GLOBALMAP-SKIP-UNANCHORED] reason=%s skipped_unanchored_submaps=%llu",
                reason.c_str(),
                static_cast<unsigned long long>(build.stats.skipped_unanchored_submaps));
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1F-GLOBALMAP-POINT-STATS] reason=%s points=%llu server_corrected_points=%llu server_corrected_mappoint_candidates=%llu server_corrected_missing_keyframe_skipped=%llu keyframe_projected_points=%llu fallback_submap_points=%llu score_min=%.3f score_mean=%.3f score_max=%.3f bad_skipped=%llu invalid_pose_skipped=%llu below_score_skipped=%llu",
            reason.c_str(),
            static_cast<unsigned long long>(build.stats.returned_points),
            static_cast<unsigned long long>(build.stats.server_corrected_points),
            static_cast<unsigned long long>(
                build.stats.server_corrected_mappoint_candidates),
            static_cast<unsigned long long>(
                build.stats.server_corrected_missing_keyframe_skipped),
            static_cast<unsigned long long>(
                build.stats.keyframe_projected_points),
            static_cast<unsigned long long>(
                build.stats.fallback_submap_points),
            build.stats.score_min,
            build.stats.score_mean,
            build.stats.score_max,
            static_cast<unsigned long long>(build.stats.bad_skipped),
            static_cast<unsigned long long>(build.stats.invalid_pose_skipped),
            static_cast<unsigned long long>(build.stats.below_score_skipped));

        RCLCPP_WARN(
            get_logger(),
            "[F1P-GLOBALMAP-FUSED-BUILD] reason=%s tracks_considered=%llu tracks_published=%llu raw_members_omitted=%llu returned_points=%llu",
            reason.c_str(),
            static_cast<unsigned long long>(build.stats.fused_tracks_considered),
            static_cast<unsigned long long>(build.stats.fused_tracks_published),
            static_cast<unsigned long long>(build.stats.fused_members_skipped),
            static_cast<unsigned long long>(build.stats.returned_points));
        if (build.stats.fused_tracks_published > 0U)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1P-FUSED-POSITION-UPDATE] reason=%s tracks_updated=%llu policy=weighted_raw_score_observations",
                reason.c_str(),
                static_cast<unsigned long long>(
                build.stats.fused_tracks_published));
        }

        auto cloud = BuildPointCloud2(build.points);
        const auto marker_build_start = std::chrono::steady_clock::now();
        KeyFrameMarkerBuildResult marker_build;
        if (global_keyframes_publish_enabled_)
        {
            marker_build = BuildGlobalKeyFrameMarkers(
                raw_snapshot,
                pose_snapshot,
                cloud.header.stamp);
        }
        const auto publish_start = std::chrono::steady_clock::now();
        size_t deleted_markers = 0;
        bool stale = false;
        bool pending_newer = false;
        uint64_t latest_revision = publication_revision;
        uint64_t coalesced_total = 0;
        uint64_t stale_total = 0;
        {
            std::lock_guard<std::mutex> lock(publication_mutex_);
            if (publication_revision <= publication_published_revision_)
            {
                stale = true;
                ++publication_stale_builds_;
            }
            else
            {
                last_global_sparse_cloud_ = cloud;
                has_last_global_sparse_cloud_ = true;
                global_sparse_cloud_pub_->publish(cloud);

                if (global_keyframes_publish_enabled_)
                {
                    std::vector<visualization_msgs::msg::Marker> deletes;
                    for (const auto& previous : published_marker_ids_)
                    {
                        if (marker_build.identities.find(previous) !=
                            marker_build.identities.end())
                        {
                            continue;
                        }
                        visualization_msgs::msg::Marker marker;
                        marker.header.frame_id = world_frame_;
                        marker.header.stamp = cloud.header.stamp;
                        marker.ns = previous.first;
                        marker.id = previous.second;
                        marker.action = visualization_msgs::msg::Marker::DELETE;
                        deletes.push_back(std::move(marker));
                    }
                    deleted_markers = deletes.size();
                    deletes.insert(
                        deletes.end(),
                        std::make_move_iterator(marker_build.message.markers.begin()),
                        std::make_move_iterator(marker_build.message.markers.end()));
                    marker_build.message.markers = std::move(deletes);
                    global_keyframes_pub_->publish(marker_build.message);
                    published_marker_ids_ = marker_build.identities;
                }
                publication_published_revision_ = publication_revision;
            }
            latest_revision = publication_requested_revision_;
            pending_newer = latest_revision > publication_revision;
            coalesced_total = publication_coalesced_requests_;
            stale_total = publication_stale_builds_;
        }
        publication_condition_.notify_all();
        if (stale)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1T-RVIZ-PUBLICATION-STALE] revision=%llu published_revision=%llu latest_revision=%llu reason=%s action=discard_out_of_order",
                static_cast<unsigned long long>(publication_revision),
                static_cast<unsigned long long>(publication_published_revision_),
                static_cast<unsigned long long>(latest_revision),
                reason.c_str());
            return;
        }
        TryTraceFlow(
            "server_rviz_cloud",
            "publish",
            reason,
            0,
            build.stats.returned_points);
        TryTraceFlow(
            "server_rviz_keyframes",
            "publish",
            reason,
            0,
            marker_build.keyframes_published);

        RCLCPP_WARN(
            get_logger(),
            "[F1P-CLOUD-BUILD-TIMING] reason=%s revision=%llu mutex_capture_ms=%.3f cloud_build_ms=%.3f marker_build_ms=%.3f publish_ms=%.3f publish_outside_mutex=true",
            reason.c_str(),
            static_cast<unsigned long long>(publication_revision),
            std::chrono::duration<double, std::milli>(
                build_start - capture_start).count(),
            std::chrono::duration<double, std::milli>(
                marker_build_start - build_start).count(),
            std::chrono::duration<double, std::milli>(
                publish_start - marker_build_start).count(),
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - publish_start).count());
        RCLCPP_WARN(
            get_logger(),
            "[F1F-GLOBALMAP-PUBLISH] reason=%s topic=%s frame_id=%s points_from_backend=%zu points_published=%u min_score_to_publish=%.3f score_field=true drone_id_field=true map_epoch_field=true",
            reason.c_str(),
            global_sparse_cloud_topic_.c_str(),
            cloud.header.frame_id.c_str(),
            build.points.size(),
            cloud.width,
            global_map_min_score_to_publish_);
        RCLCPP_WARN(
            get_logger(),
            "[F1P-GLOBALMAP-FUSED-PUBLISH] reason=%s topic=%s fused_tracks=%llu raw_members_omitted=%llu points_published=%u",
            reason.c_str(),
            global_sparse_cloud_topic_.c_str(),
            static_cast<unsigned long long>(build.stats.fused_tracks_published),
            static_cast<unsigned long long>(build.stats.fused_members_skipped),
            cloud.width);
        if (global_keyframes_publish_enabled_)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1T-RVIZ-KF-MARKERS] reason=%s revision=%llu topic=%s frame_id=%s keyframes=%llu submaps=%zu without_world_pose=%llu bad_skipped=%llu deleted=%zu labels=%s",
                reason.c_str(),
                static_cast<unsigned long long>(publication_revision),
                global_keyframes_topic_.c_str(),
                world_frame_.c_str(),
                static_cast<unsigned long long>(marker_build.keyframes_published),
                marker_build.submaps_published.size(),
                static_cast<unsigned long long>(
                    marker_build.keyframes_without_world_pose),
                static_cast<unsigned long long>(marker_build.bad_keyframes_skipped),
                deleted_markers,
                global_keyframes_labels_enabled_ ? "true" : "false");
        }
        RCLCPP_WARN(
            get_logger(),
            "[F1T-RVIZ-PUBLICATION-COMMIT] revision=%llu latest_revision=%llu reason=%s cloud_points=%u keyframes=%llu pending_newer=%s coalesced_total=%llu stale_total=%llu",
            static_cast<unsigned long long>(publication_revision),
            static_cast<unsigned long long>(latest_revision),
            reason.c_str(),
            cloud.width,
            static_cast<unsigned long long>(marker_build.keyframes_published),
            pending_newer ? "true" : "false",
            static_cast<unsigned long long>(coalesced_total),
            static_cast<unsigned long long>(stale_total));
    }

    void RepublishLastGlobalSparseCloud(const std::string& reason)
    {
        // F1H-hotfix: el timer no debe reconstruir desde `raw_db_` porque puede
        // intercalarse visualmente con una nube recien publicada por delta o
        // snapshot. Reutiliza el ultimo `PointCloud2` validado y solo refresca
        // el stamp para RViz2.
        sensor_msgs::msg::PointCloud2 cloud;
        bool has_cached_cloud = false;
        {
            std::lock_guard<std::mutex> lock(publication_mutex_);
            if (has_last_global_sparse_cloud_)
            {
                cloud = last_global_sparse_cloud_;
                has_cached_cloud = true;
                cloud.header.stamp = now();
                global_sparse_cloud_pub_->publish(cloud);
            }
        }
        if (!has_cached_cloud)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1F-GLOBALMAP-REPUBLISH-SKIP] reason=%s cause=no_cached_cloud topic=%s",
                reason.c_str(),
                global_sparse_cloud_topic_.c_str());
            return;
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1F-GLOBALMAP-REPUBLISH] reason=%s topic=%s frame_id=%s cached_points=%u",
            reason.c_str(),
            global_sparse_cloud_topic_.c_str(),
            cloud.header.frame_id.c_str(),
            cloud.width);
    }

    static std::string EscapeFlowJson(const std::string& value)
    {
        std::string escaped;
        escaped.reserve(value.size());
        for (const char character : value)
        {
            switch (character)
            {
                case '\\': escaped += "\\\\"; break;
                case '"': escaped += "\\\""; break;
                case '\n': escaped += "\\n"; break;
                case '\r': escaped += "\\r"; break;
                case '\t': escaped += "\\t"; break;
                default: escaped += character; break;
            }
        }
        return escaped;
    }

    void TryTraceFlow(const std::string& edge,
                      const std::string& phase,
                      const std::string& detail,
                      uint64_t task_id = 0,
                      uint64_t count = 0)
    {
        if (!flow_telemetry_enabled_)
        {
            return;
        }
        if (!flow_telemetry_mutex_.try_lock())
        {
            ++flow_telemetry_dropped_;
            return;
        }
        std::unique_lock<std::mutex> lock(
            flow_telemetry_mutex_, std::adopt_lock);
        if (flow_telemetry_queue_.size() >= kFlowTelemetryCapacity)
        {
            ++flow_telemetry_dropped_;
            return;
        }
        const uint64_t sequence = ++flow_telemetry_sequence_;
        std::ostringstream json;
        json << "{\"seq\":" << sequence
             << ",\"stamp_ns\":" << now().nanoseconds()
             << ",\"edge\":\"" << EscapeFlowJson(edge)
             << "\",\"phase\":\"" << EscapeFlowJson(phase)
             << "\",\"detail\":\"" << EscapeFlowJson(detail)
             << "\",\"task_id\":" << task_id
             << ",\"count\":" << count << "}";
        flow_telemetry_queue_.push_back(json.str());
    }

    void DrainFlowTelemetry()
    {
        if (!flow_telemetry_enabled_ || !flow_telemetry_pub_)
        {
            return;
        }
        std::vector<std::string> batch;
        {
            std::lock_guard<std::mutex> lock(flow_telemetry_mutex_);
            const size_t count =
                std::min<size_t>(32U, flow_telemetry_queue_.size());
            batch.reserve(count);
            for (size_t index = 0; index < count; ++index)
            {
                batch.push_back(std::move(flow_telemetry_queue_.front()));
                flow_telemetry_queue_.pop_front();
            }
        }
        for (auto& json : batch)
        {
            std_msgs::msg::String message;
            message.data = std::move(json);
            flow_telemetry_pub_->publish(message);
        }
    }

    uint64_t RequestGlobalStatePublication(const std::string& reason)
    {
        uint64_t revision = 0;
        bool coalesced = false;
        {
            std::lock_guard<std::mutex> lock(publication_mutex_);
            coalesced =
                publication_requested_revision_ > publication_published_revision_;
            revision = ++publication_requested_revision_;
            publication_pending_reason_ = reason;
            if (coalesced)
            {
                ++publication_coalesced_requests_;
            }
        }
        publication_condition_.notify_one();
        TryTraceFlow("server_map_builder", "request", reason, 0, revision);
        RCLCPP_WARN(
            get_logger(),
            "[F1T-RVIZ-PUBLICATION-REQUEST] revision=%llu reason=%s coalesced=%s worker=fixed",
            static_cast<unsigned long long>(revision),
            reason.c_str(),
            coalesced ? "true" : "false");
        return revision;
    }

    uint64_t RequestOptimizationStatePublication(
        uint64_t task_id,
        const std::string& reason)
    {
        const uint64_t revision = RequestGlobalStatePublication(reason);
        RCLCPP_WARN(
            get_logger(),
            "[F1K-PUBLICATION-ASYNC] task_id=%llu revision=%llu reason=%s waited=false",
            static_cast<unsigned long long>(task_id),
            static_cast<unsigned long long>(revision),
            reason.c_str());
        return revision;
    }

    void GlobalPublicationWorker()
    {
        while (true)
        {
            uint64_t revision = 0;
            std::string reason;
            {
                std::unique_lock<std::mutex> lock(publication_mutex_);
                publication_condition_.wait(
                    lock,
                    [this]()
                    {
                        return publication_worker_shutdown_.load() ||
                            publication_requested_revision_ >
                                publication_published_revision_;
                    });
                if (publication_worker_shutdown_.load() &&
                    publication_requested_revision_ <=
                        publication_published_revision_)
                {
                    return;
                }
                revision = publication_requested_revision_;
                reason = publication_pending_reason_;
            }
            BuildAndPublishGlobalState(reason, revision);
        }
    }

    void CreateFullSnapshotClients()
    {
        // F1G: cada wrapper ya ofrece `orbslam/get_full_map`. El servidor solo
        // crea clientes y pide snapshots de forma acotada para resincronizar raw
        // sin bloquear el hilo principal de ROS.
        full_snapshot_clients_.resize(static_cast<size_t>(n_drones_) + 1U);
        full_snapshot_pending_.assign(static_cast<size_t>(n_drones_) + 1U, false);
        for (uint32_t drone_id = 1; drone_id <= static_cast<uint32_t>(n_drones_); ++drone_id)
        {
            const std::string service = DroneNamespace(drone_id) + "/orbslam/get_full_map";
            full_snapshot_clients_[drone_id] = create_client<GetOrbMap>(service);
            RCLCPP_WARN(
                get_logger(),
                "[F1G-FULL-SNAPSHOT-CLIENT-READY] drone_id=%u service=%s startup_delay_sec=%.3f period_sec=%.3f",
                drone_id,
                service.c_str(),
                f1g_full_snapshot_startup_delay_sec_,
                f1g_full_snapshot_period_sec_);
        }
    }

    void RequestFullSnapshots(const std::string& reason)
    {
        for (uint32_t drone_id = 1; drone_id <= static_cast<uint32_t>(n_drones_); ++drone_id)
        {
            RequestFullSnapshot(drone_id, reason);
        }
    }

    void RequestFullSnapshot(uint32_t drone_id, const std::string& reason)
    {
        if (drone_id >= full_snapshot_clients_.size() || !full_snapshot_clients_[drone_id])
        {
            return;
        }

        const std::string service = DroneNamespace(drone_id) + "/orbslam/get_full_map";
        if (full_snapshot_pending_[drone_id])
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1G-FULL-SNAPSHOT-REQUEST] drone_id=%u service=%s reason=%s status=skip_pending",
                drone_id,
                service.c_str(),
                reason.c_str());
            return;
        }
        if (!full_snapshot_clients_[drone_id]->service_is_ready())
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1G-FULL-SNAPSHOT-TIMEOUT] drone_id=%u service=%s reason=%s status=service_not_ready",
                drone_id,
                service.c_str(),
                reason.c_str());
            return;
        }

        full_snapshot_pending_[drone_id] = true;
        auto request = std::make_shared<GetOrbMap::Request>();
        RCLCPP_WARN(
            get_logger(),
            "[F1G-FULL-SNAPSHOT-REQUEST] drone_id=%u service=%s reason=%s status=sent",
            drone_id,
            service.c_str(),
            reason.c_str());
        full_snapshot_clients_[drone_id]->async_send_request(
            request,
            [this, drone_id, service, reason](
                rclcpp::Client<GetOrbMap>::SharedFuture future)
            {
                OnFullSnapshotResponse(drone_id, service, reason, future);
            });
    }

    void OnFullSnapshotResponse(uint32_t requested_drone_id,
                                const std::string& service,
                                const std::string& reason,
                                rclcpp::Client<GetOrbMap>::SharedFuture future)
    {
        std::unique_lock<std::recursive_mutex> live_lock(live_state_mutex_);
        if (requested_drone_id < full_snapshot_pending_.size())
        {
            full_snapshot_pending_[requested_drone_id] = false;
        }
        try
        {
            const auto response = future.get();
            if (!response)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "[F1G-FULL-SNAPSHOT-RX] requested_drone_id=%u service=%s success=false error=null_response",
                    requested_drone_id,
                    service.c_str());
                return;
            }

            OrbMap snapshot = response->map;
            if (snapshot.drone_id == 0 &&
                snapshot.keyframes.empty() &&
                snapshot.mappoints.empty())
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1G-FULL-SNAPSHOT-RX] requested_drone_id=%u service=%s success=false error=empty_or_uninitialized_snapshot reason=%s",
                    requested_drone_id,
                    service.c_str(),
                    reason.c_str());
                return;
            }

            const uint64_t arrival_id = rawdb_next_arrival_id_++;
            RCLCPP_WARN(
                get_logger(),
                "[F1G-FULL-SNAPSHOT-RX] requested_drone_id=%u drone_id=%u epoch=%llu seq=%llu kfs=%zu mps=%zu service=%s reason=%s",
                requested_drone_id,
                snapshot.drone_id,
                static_cast<unsigned long long>(snapshot.map_epoch),
                static_cast<unsigned long long>(snapshot.map_sequence),
                snapshot.keyframes.size(),
                snapshot.mappoints.size(),
                service.c_str(),
                reason.c_str());
            RCLCPP_WARN(
                get_logger(),
                "[F1G-FULL-SNAPSHOT-ARRIVAL] arrival_id=%llu drone_id=%u epoch=%llu seq=%llu kfs=%zu mps=%zu",
                static_cast<unsigned long long>(arrival_id),
                snapshot.drone_id,
                static_cast<unsigned long long>(snapshot.map_epoch),
                static_cast<unsigned long long>(snapshot.map_sequence),
                snapshot.keyframes.size(),
                snapshot.mappoints.size());
            TryTraceFlow(
                "wrapper_server_snapshot",
                "receive",
                "full_snapshot",
                0,
                snapshot.keyframes.size());

            MaybeSetF1GDebugOptimizedKeyFrame(snapshot, "live_full_snapshot");
            const auto insert_result = raw_db_.InsertFullSnapshot(arrival_id, snapshot);
            TryTraceFlow(
                "server_raw_snapshot",
                "commit",
                "full_snapshot",
                0,
                insert_result.new_keyframes + insert_result.updated_keyframes);
            ProcessFullSnapshotAfterInsert(
                snapshot,
                arrival_id,
                insert_result,
                "live_full_snapshot");
            if (insert_result.has_material_changes)
            {
                live_lock.unlock();
            }
        }
        catch (const std::exception& ex)
        {
            RCLCPP_ERROR(
                get_logger(),
                "[F1G-FULL-SNAPSHOT-RX] requested_drone_id=%u service=%s success=false error=%s",
                requested_drone_id,
                service.c_str(),
                ex.what());
        }
    }

    bool MaybeSetF1GDebugOptimizedKeyFrame(const OrbMap& map, const std::string& reason)
    {
        if (!f1g_debug_mark_optimized_kf_ || f1g_debug_optimized_kf_done_)
        {
            return false;
        }

        for (const auto& keyframe : map.keyframes)
        {
            const RawKeyFrameId keyframe_id{map.drone_id, map.map_epoch, keyframe.id};
            const auto current_world_pose = pose_store_.GetWorldPose(keyframe_id);
            if (!current_world_pose)
            {
                continue;
            }

            const auto result =
                pose_store_.SetOptimizedKeyFramePose(
                    keyframe_id,
                    current_world_pose.value(),
                    raw_db_,
                    "F1G_DEBUG_OPTIMIZED_POSE");
            if (!result.success)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1G-DEBUG-OPTIMIZED-KF-SET] status=failed drone_id=%u epoch=%llu kf=%llu reason=%s trigger=%s",
                    keyframe_id.drone_id,
                    static_cast<unsigned long long>(keyframe_id.map_epoch),
                    static_cast<unsigned long long>(keyframe_id.local_kf_id),
                    result.reason.c_str(),
                    reason.c_str());
                return false;
            }

            f1g_debug_optimized_kf_done_ = true;
            RCLCPP_WARN(
                get_logger(),
                "[F1G-DEBUG-OPTIMIZED-KF-SET] status=applied drone_id=%u epoch=%llu kf=%llu source=F1G_DEBUG_OPTIMIZED_POSE trigger=%s",
                keyframe_id.drone_id,
                static_cast<unsigned long long>(keyframe_id.map_epoch),
                static_cast<unsigned long long>(keyframe_id.local_kf_id),
                reason.c_str());
            return true;
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1G-DEBUG-OPTIMIZED-KF-SET] status=waiting reason=no_world_pose_in_snapshot drone_id=%u epoch=%llu trigger=%s",
            map.drone_id,
            static_cast<unsigned long long>(map.map_epoch),
            reason.c_str());
        return false;
    }

    void ProcessFullSnapshotAfterInsert(const OrbMap& map,
                                        uint64_t arrival_id,
                                        const orbslam3_multi::RawInsertResult& insert_result,
                                        const std::string& source)
    {
        RCLCPP_WARN(
            get_logger(),
            "[F1G-RAWDB-INSERT-FULL] arrival_id=%llu drone_id=%u epoch=%llu new_kfs=%llu updated_kfs=%llu new_mps=%llu updated_mps=%llu bad_kfs=%llu bad_mps=%llu raw_pose_changed=%zu large_raw_pose_changed=%llu",
            static_cast<unsigned long long>(arrival_id),
            map.drone_id,
            static_cast<unsigned long long>(map.map_epoch),
            static_cast<unsigned long long>(insert_result.new_keyframes),
            static_cast<unsigned long long>(insert_result.updated_keyframes),
            static_cast<unsigned long long>(insert_result.new_mappoints),
            static_cast<unsigned long long>(insert_result.updated_mappoints),
            static_cast<unsigned long long>(insert_result.bad_keyframes),
            static_cast<unsigned long long>(insert_result.bad_mappoints),
            insert_result.raw_pose_changed_keyframes.size(),
            static_cast<unsigned long long>(insert_result.large_raw_pose_changed_keyframes));

        RCLCPP_WARN(
            get_logger(),
            "[F1G-SNAPSHOT-RECONCILE] arrival_id=%llu drone_id=%u epoch=%llu policy=insert_update_no_absent_delete new_kfs=%llu updated_kfs=%llu unchanged_kfs=%llu new_mps=%llu updated_mps=%llu unchanged_mps=%llu material_changes=%s material_revision=%llu",
            static_cast<unsigned long long>(arrival_id),
            map.drone_id,
            static_cast<unsigned long long>(map.map_epoch),
            static_cast<unsigned long long>(insert_result.new_keyframes),
            static_cast<unsigned long long>(insert_result.updated_keyframes),
            static_cast<unsigned long long>(insert_result.unchanged_keyframes),
            static_cast<unsigned long long>(insert_result.new_mappoints),
            static_cast<unsigned long long>(insert_result.updated_mappoints),
            static_cast<unsigned long long>(insert_result.unchanged_mappoints),
            insert_result.has_material_changes ? "true" : "false",
            static_cast<unsigned long long>(insert_result.material_revision));

        if (!insert_result.has_material_changes)
        {
            LogRawInsert("F1C-RAWDB-INSERT-FULL", arrival_id, insert_result.stats);
            RCLCPP_WARN(
                get_logger(),
                "[F1P-SNAPSHOT-NOOP] source=%s arrival_id=%llu drone_id=%u epoch=%llu received_kfs=%zu received_mps=%zu fastpath_queries=0 bow_queries=0 workers_created=0 cloud_rebuild=false",
                source.c_str(),
                static_cast<unsigned long long>(arrival_id),
                map.drone_id,
                static_cast<unsigned long long>(map.map_epoch),
                map.keyframes.size(),
                map.mappoints.size());
            return;
        }

        for (const auto& change : insert_result.raw_pose_changed_keyframes)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1G-RAW-POSE-CHANGED] arrival_id=%llu drone_id=%u epoch=%llu kf=%llu delta_t=%.6f delta_yaw=%.6f large=%s source=%s",
                static_cast<unsigned long long>(arrival_id),
                change.keyframe_id.drone_id,
                static_cast<unsigned long long>(change.keyframe_id.map_epoch),
                static_cast<unsigned long long>(change.keyframe_id.local_kf_id),
                change.delta_translation_m,
                change.delta_yaw_rad,
                change.large_change ? "true" : "false",
                source.c_str());
        }

        const auto reconcile =
            pose_store_.ReconcileAfterRawIngestResult(
                insert_result,
                raw_db_,
                "F1G_FULL_SNAPSHOT_RECONCILE");
        for (const auto& event : reconcile.events)
        {
            if (event.action == "rebase_anchor")
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1G-POSESTORE-REBASE-ANCHOR] drone_id=%u epoch=%llu kf=%llu delta_t=%.6f delta_yaw=%.6f reason=%s",
                    event.keyframe_id.drone_id,
                    static_cast<unsigned long long>(event.keyframe_id.map_epoch),
                    static_cast<unsigned long long>(event.keyframe_id.local_kf_id),
                    event.raw_delta_translation_m,
                    event.raw_delta_yaw_rad,
                    event.reason.c_str());
            }
            else if (event.action == "keep_optimized")
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1G-POSESTORE-KEEP-OPTIMIZED] drone_id=%u epoch=%llu kf=%llu delta_t=%.6f delta_yaw=%.6f reason=%s",
                    event.keyframe_id.drone_id,
                    static_cast<unsigned long long>(event.keyframe_id.map_epoch),
                    static_cast<unsigned long long>(event.keyframe_id.local_kf_id),
                    event.raw_delta_translation_m,
                    event.raw_delta_yaw_rad,
                    event.reason.c_str());
                RCLCPP_WARN(
                    get_logger(),
                    "[F1K-POSESTORE-KEEP-SERVER-OPTIMIZED] drone_id=%u epoch=%llu kf=%llu raw_pose_changed=true action=recompute_correction_keep_world_pose delta_t=%.6f delta_yaw=%.6f",
                    event.keyframe_id.drone_id,
                    static_cast<unsigned long long>(event.keyframe_id.map_epoch),
                    static_cast<unsigned long long>(event.keyframe_id.local_kf_id),
                    event.raw_delta_translation_m,
                    event.raw_delta_yaw_rad);
                RCLCPP_WARN(
                    get_logger(),
                    "[F1K-POSESTORE-RECOMPUTE-CORRECTION-AFTER-RAW-CHANGE] drone_id=%u epoch=%llu kf=%llu source=full_snapshot",
                    event.keyframe_id.drone_id,
                    static_cast<unsigned long long>(event.keyframe_id.map_epoch),
                    static_cast<unsigned long long>(event.keyframe_id.local_kf_id));
            }
            else if (event.action == "keep_accepted")
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1K-POSESTORE-KEEP-ACCEPTED] drone_id=%u epoch=%llu kf=%llu raw_pose_changed=true action=keep_world_recompute_raw_relation delta_t=%.6f delta_yaw=%.6f reason=%s",
                    event.keyframe_id.drone_id,
                    static_cast<unsigned long long>(event.keyframe_id.map_epoch),
                    static_cast<unsigned long long>(event.keyframe_id.local_kf_id),
                    event.raw_delta_translation_m,
                    event.raw_delta_yaw_rad,
                    event.reason.c_str());
            }
            else if (event.action == "rebase_derived_tail")
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1K-DERIVED-TAIL-REBASE-AFTER-RAW] drone_id=%u epoch=%llu kf=%llu action=reproject_from_accepted_anchor delta_t=%.6f delta_yaw=%.6f reason=%s",
                    event.keyframe_id.drone_id,
                    static_cast<unsigned long long>(event.keyframe_id.map_epoch),
                    static_cast<unsigned long long>(event.keyframe_id.local_kf_id),
                    event.raw_delta_translation_m,
                    event.raw_delta_yaw_rad,
                    event.reason.c_str());
            }
        }

        if (!reconcile.optimized_submaps_affected.empty())
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1G-SNAPSHOT-AFFECTS-OPTIMIZED-KFS] arrival_id=%llu affected_submaps=%zu optimized_kept=%llu",
                static_cast<unsigned long long>(arrival_id),
                reconcile.optimized_submaps_affected.size(),
                static_cast<unsigned long long>(reconcile.optimized_kept));
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1G-POSESTORE-RECONCILE-SUMMARY] arrival_id=%llu raw_pose_changed=%llu anchor_rebased=%llu optimized_kept=%llu no_world_pose=%llu failed=%llu affected_optimized_submaps=%zu",
            static_cast<unsigned long long>(arrival_id),
            static_cast<unsigned long long>(reconcile.raw_pose_changed),
            static_cast<unsigned long long>(reconcile.anchor_rebased),
            static_cast<unsigned long long>(reconcile.optimized_kept),
            static_cast<unsigned long long>(reconcile.no_world_pose),
            static_cast<unsigned long long>(reconcile.failed),
            reconcile.optimized_submaps_affected.size());
        TryTraceFlow(
            "raw_db_pose_snapshot",
            "commit",
            "snapshot_pose_reconcile",
            0,
            reconcile.raw_pose_changed + reconcile.anchor_rebased +
                reconcile.optimized_kept);

        const OrbMap material_map = BuildMaterialMap(map, insert_result);
        LogRawInsert("F1C-RAWDB-INSERT-FULL", arrival_id, insert_result.stats);
        UpdateScoresFromMap(material_map, arrival_id);
        TryTraceFlow(
            "raw_db_score_db",
            "commit",
            "snapshot_scores",
            0,
            material_map.mappoints.size());
        RegisterF1EKeyFramesFromMap(material_map);
        RequestGlobalStatePublication(source + "_raw_commit");

        ImportCovisibilityFromRaw(insert_result, source);
        ProcessFiducialsForDelta(material_map, arrival_id);
        if (insert_result.has_loop_material_changes)
        {
            ScheduleLoopTasks(insert_result, source);
        }
        else
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1P-LOOP-NOOP-METADATA] source=%s arrival_id=%llu metadata_mps=%zu fastpath_queries=0 bow_queries=0 workers_created=0",
                source.c_str(),
                static_cast<unsigned long long>(arrival_id),
                insert_result.metadata_changed_mappoint_ids.size());
        }
        RCLCPP_WARN(
            get_logger(),
            "[F1G-GLOBALMAP-REBUILD-AFTER-SNAPSHOT] arrival_id=%llu drone_id=%u epoch=%llu source=%s",
            static_cast<unsigned long long>(arrival_id),
            map.drone_id,
            static_cast<unsigned long long>(map.map_epoch),
            source.c_str());
    }

    // F1B: crea un subscriber por dron a `orbslam/orb_map_delta`.
    // Entrada: `n_drones_` y `namespace_base_`.
    // Efecto: deja activa la recepcion multi-dron y loggea cada topic usado.
    void CreateOrbMapSubscriptions()
    {
        // F1B: recorremos IDs esperados 1..n para mantener estable la relacion
        // entre topic suscrito y `subscribed_drone_id` usado luego en los logs.
        for (uint32_t drone_id = 1; drone_id <= static_cast<uint32_t>(n_drones_); ++drone_id)
        {
            const std::string topic = DroneNamespace(drone_id) + "/orbslam/orb_map_delta";

            // Callback ROS creado en F1B y ampliado en F1C: mantiene los logs
            // de recepción y además inserta cada delta en RawMapDatabase.
            auto sub = create_subscription<OrbMap>(
                topic,
                rclcpp::QoS(rclcpp::KeepLast(20)),
                [this, drone_id](const OrbMap::SharedPtr msg)
                {
                    OnOrbMapDelta(msg, drone_id);
                });

            orb_map_delta_subs_.push_back(sub);

            RCLCPP_WARN(
                get_logger(),
                "[F1B-SERVER-SUBSCRIBED] drone_id=%u topic=%s",
                drone_id,
                topic.c_str());
        }
    }

    void CreateGroundTruthSubscriptions()
    {
        // F1E: GT solo entra por esta ruta para simular detecciones fiduciales.
        // No se usa para mapa, loops, fused landmarks ni pose final.
        for (uint32_t drone_id = 1; drone_id <= static_cast<uint32_t>(n_drones_); ++drone_id)
        {
            const std::string topic = DroneNamespace(drone_id) + "/sensor/GT/pose";
            auto sub = create_subscription<geometry_msgs::msg::PoseStamped>(
                topic,
                rclcpp::QoS(rclcpp::KeepLast(50)),
                [this, drone_id](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
                {
                    std::lock_guard<std::recursive_mutex> lock(live_state_mutex_);
                    OnGroundTruthPose(msg, drone_id);
                });
            gt_pose_subs_.push_back(sub);

            RCLCPP_WARN(
                get_logger(),
                "[F1E-FID-GT-SUBSCRIBED] drone_id=%u topic=%s",
                drone_id,
                topic.c_str());
        }
    }

    void OnGroundTruthPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg,
                           uint32_t drone_id)
    {
        if (!msg)
        {
            return;
        }

        Eigen::Matrix4d world_T_body = Eigen::Matrix4d::Identity();
        if (!PoseMsgToMatrix(msg->pose, world_T_body))
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1E-FID-CANDIDATE-GT] drone_id=%u status=rejected reason=invalid_gt_pose",
                drone_id);
            return;
        }

        GroundTruthSample sample;
        sample.stamp = rclcpp::Time(msg->header.stamp);
        sample.world_T_body = world_T_body;

        auto& buffer = gt_buffers_[drone_id];
        buffer.push_back(sample);
        while (buffer.size() > static_cast<size_t>(fiducial_gt_buffer_max_samples_))
        {
            buffer.erase(buffer.begin());
        }
    }

    bool FindBestGroundTruth(uint32_t drone_id,
                             const rclcpp::Time& keyframe_stamp,
                             GroundTruthSample& best_sample,
                             double& best_dt_sec,
                             std::string& source,
                             double max_dt_sec = -1.0) const
    {
        const auto buffer_it = gt_buffers_.find(drone_id);
        if (buffer_it == gt_buffers_.end() || buffer_it->second.empty())
        {
            return false;
        }

        const auto& buffer = buffer_it->second;
        if (keyframe_stamp.nanoseconds() <= 0)
        {
            best_sample = buffer.back();
            best_dt_sec = 0.0;
            source = "SIM_GT_NEAREST_POSE";
            return true;
        }

        bool found = false;
        int64_t best_abs_dt_ns = 0;
        for (const auto& sample : buffer)
        {
            const int64_t dt_ns =
                std::llabs((sample.stamp - keyframe_stamp).nanoseconds());
            if (!found || dt_ns < best_abs_dt_ns)
            {
                found = true;
                best_abs_dt_ns = dt_ns;
                best_sample = sample;
            }
        }

        best_dt_sec = static_cast<double>(best_abs_dt_ns) * 1e-9;
        source = "SIM_GT_TEMPORAL_ASSOCIATION";
        const double allowed_dt =
            max_dt_sec >= 0.0 ? max_dt_sec : fiducial_gt_max_dt_sec_;
        return found && best_dt_sec <= allowed_dt;
    }

    void AssociateGtDebugForDelta(const OrbMap& map, uint64_t arrival_id)
    {
        if (!f1l_gt_kf_debug_enabled_ || rawdb_replay_enabled_)
        {
            return;
        }

        for (const auto& keyframe : map.keyframes)
        {
            const RawKeyFrameId keyframe_id{
                map.drone_id,
                map.map_epoch,
                keyframe.id};
            const rclcpp::Time keyframe_stamp(keyframe.stamp);
            GroundTruthSample gt_sample;
            double dt_sec = 0.0;
            std::string source;
            if (!FindBestGroundTruth(
                    map.drone_id,
                    keyframe_stamp,
                    gt_sample,
                    dt_sec,
                    source,
                    f1l_gt_kf_debug_max_dt_sec_))
            {
                if (f1l_gt_debug_missing_logged_.insert(keyframe_id).second)
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "[F1L-GT-KF-ASSOC] arrival_id=%llu drone_id=%u epoch=%llu kf=%llu kf_stamp=%.6f gt_stamp=0.000000 dt=-1.000000 valid=false quality=INVALID source=GAZEBO_GT_DEBUG reason=no_gt_within_threshold",
                        static_cast<unsigned long long>(arrival_id),
                        keyframe_id.drone_id,
                        static_cast<unsigned long long>(keyframe_id.map_epoch),
                        static_cast<unsigned long long>(keyframe_id.local_kf_id),
                        StampToSeconds(keyframe.stamp));
                }
                continue;
            }

            DebugGtKeyFramePose debug_pose;
            debug_pose.world_T_kf_gt =
                pose_store_.TransformBodyPoseToCameraPose(gt_sample.world_T_body);
            debug_pose.kf_stamp_sec = StampToSeconds(keyframe.stamp);
            debug_pose.gt_stamp_sec = gt_sample.stamp.seconds();
            debug_pose.association_dt_sec = dt_sec;
            debug_pose.association_quality =
                dt_sec <= 0.5 * f1l_gt_kf_debug_max_dt_sec_ ? "OK" : "LOW";
            f1l_gt_keyframe_store_[keyframe_id] = debug_pose;

            // 1L: este GT se registra como metrica externa por KF. No se pasa
            // al solver ni cambia pesos/vertices; solo permite diagnosticar si
            // RViz2 muestra deriva o dano colateral tras el apply.
            RCLCPP_WARN(
                get_logger(),
                "[F1L-GT-KF-ASSOC] arrival_id=%llu drone_id=%u epoch=%llu kf=%llu kf_stamp=%.6f gt_stamp=%.6f dt=%.6f valid=true quality=%s source=GAZEBO_GT_DEBUG gt_camera_t=(%.3f,%.3f,%.3f)",
                static_cast<unsigned long long>(arrival_id),
                keyframe_id.drone_id,
                static_cast<unsigned long long>(keyframe_id.map_epoch),
                static_cast<unsigned long long>(keyframe_id.local_kf_id),
                debug_pose.kf_stamp_sec,
                debug_pose.gt_stamp_sec,
                debug_pose.association_dt_sec,
                debug_pose.association_quality.c_str(),
                debug_pose.world_T_kf_gt(0, 3),
                debug_pose.world_T_kf_gt(1, 3),
                debug_pose.world_T_kf_gt(2, 3));
        }
    }

    DebugGtWindowStats LogGtWindowErrors(uint64_t task_id,
                                         const std::vector<RawKeyFrameId>& keyframes,
                                         bool after_apply,
                                         const GlobalPoseStore* pose_override = nullptr) const
    {
        DebugGtWindowStats stats;
        if (!f1l_gt_kf_debug_enabled_)
        {
            return stats;
        }

        std::set<RawKeyFrameId> unique_keyframes(keyframes.begin(), keyframes.end());
        double sum_error = 0.0;
        for (const auto& keyframe_id : unique_keyframes)
        {
            const auto gt_it = f1l_gt_keyframe_store_.find(keyframe_id);
            if (gt_it == f1l_gt_keyframe_store_.end())
            {
                continue;
            }
            const GlobalPoseStore& pose_source =
                pose_override ? *pose_override : pose_store_;
            const auto world_pose = pose_source.GetWorldPose(keyframe_id);
            if (!world_pose.has_value())
            {
                continue;
            }

            const Eigen::Vector3d delta =
                world_pose.value().block<3, 1>(0, 3) -
                gt_it->second.world_T_kf_gt.block<3, 1>(0, 3);
            const double error_t = delta.norm();
            const double error_yaw = std::abs(NormalizeAngle(
                YawFromTransform(world_pose.value()) -
                YawFromTransform(gt_it->second.world_T_kf_gt)));
            stats.error_t_by_keyframe[keyframe_id] = error_t;
            ++stats.valid_kfs;
            sum_error += error_t;
            stats.max_error_t = std::max(stats.max_error_t, error_t);

            if (after_apply)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1L-GT-KF-ERROR-AFTER] task_id=%llu drone_id=%u epoch=%llu kf=%llu error_t=%.6f error_yaw=%.6f map_t=(%.3f,%.3f,%.3f) gt_t=(%.3f,%.3f,%.3f) association_dt=%.6f quality=%s",
                    static_cast<unsigned long long>(task_id),
                    keyframe_id.drone_id,
                    static_cast<unsigned long long>(keyframe_id.map_epoch),
                    static_cast<unsigned long long>(keyframe_id.local_kf_id),
                    error_t,
                    error_yaw,
                    world_pose.value()(0, 3),
                    world_pose.value()(1, 3),
                    world_pose.value()(2, 3),
                    gt_it->second.world_T_kf_gt(0, 3),
                    gt_it->second.world_T_kf_gt(1, 3),
                    gt_it->second.world_T_kf_gt(2, 3),
                    gt_it->second.association_dt_sec,
                    gt_it->second.association_quality.c_str());
            }
            else
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1L-GT-KF-ERROR-BEFORE] task_id=%llu drone_id=%u epoch=%llu kf=%llu error_t=%.6f error_yaw=%.6f map_t=(%.3f,%.3f,%.3f) gt_t=(%.3f,%.3f,%.3f) association_dt=%.6f quality=%s",
                    static_cast<unsigned long long>(task_id),
                    keyframe_id.drone_id,
                    static_cast<unsigned long long>(keyframe_id.map_epoch),
                    static_cast<unsigned long long>(keyframe_id.local_kf_id),
                    error_t,
                    error_yaw,
                    world_pose.value()(0, 3),
                    world_pose.value()(1, 3),
                    world_pose.value()(2, 3),
                    gt_it->second.world_T_kf_gt(0, 3),
                    gt_it->second.world_T_kf_gt(1, 3),
                    gt_it->second.world_T_kf_gt(2, 3),
                    gt_it->second.association_dt_sec,
                    gt_it->second.association_quality.c_str());
            }
        }

        if (stats.valid_kfs > 0)
        {
            stats.mean_error_t = sum_error / static_cast<double>(stats.valid_kfs);
        }
        return stats;
    }

    void LogGtWindowComparison(uint64_t task_id,
                               const DebugGtWindowStats& before,
                               const DebugGtWindowStats& after) const
    {
        if (!f1l_gt_kf_debug_enabled_)
        {
            return;
        }

        uint64_t worsened_kfs = 0;
        for (const auto& [keyframe_id, after_error] : after.error_t_by_keyframe)
        {
            const auto before_it = before.error_t_by_keyframe.find(keyframe_id);
            if (before_it != before.error_t_by_keyframe.end() &&
                after_error > before_it->second + 0.05)
            {
                ++worsened_kfs;
            }
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1L-GT-WINDOW-STATS] task_id=%llu valid_before=%llu valid_after=%llu mean_before=%.6f max_before=%.6f mean_after=%.6f max_after=%.6f worsened_kfs=%llu source=GAZEBO_GT_DEBUG",
            static_cast<unsigned long long>(task_id),
            static_cast<unsigned long long>(before.valid_kfs),
            static_cast<unsigned long long>(after.valid_kfs),
            before.mean_error_t,
            before.max_error_t,
            after.mean_error_t,
            after.max_error_t,
            static_cast<unsigned long long>(worsened_kfs));
    }

    std::vector<RawKeyFrameId> CollectGtCollateralKeyFrames(
        const PoseGraphProblem& problem) const
    {
        std::vector<RawKeyFrameId> keyframes;
        for (const auto& vertex : problem.vertices)
        {
            if (vertex.keyframe_id == problem.target_keyframe_id)
            {
                continue;
            }
            if (vertex.is_fixed || vertex.is_hard_fiducial ||
                vertex.is_anchor_neighborhood)
            {
                keyframes.push_back(vertex.keyframe_id);
            }
        }
        if (keyframes.empty())
        {
            for (const auto& keyframe_id : problem.fixed_keyframes)
            {
                if (!(keyframe_id == problem.target_keyframe_id))
                {
                    keyframes.push_back(keyframe_id);
                }
            }
        }
        return keyframes;
    }

    void LogGtCollateralCheck(uint64_t task_id,
                              const DebugGtWindowStats& before,
                              const DebugGtWindowStats& after,
                              const std::vector<RawKeyFrameId>& collateral) const
    {
        if (!f1l_gt_kf_debug_enabled_)
        {
            return;
        }

        double sum_before = 0.0;
        double sum_after = 0.0;
        uint64_t valid = 0;
        for (const auto& keyframe_id : collateral)
        {
            const auto before_it = before.error_t_by_keyframe.find(keyframe_id);
            const auto after_it = after.error_t_by_keyframe.find(keyframe_id);
            if (before_it == before.error_t_by_keyframe.end() ||
                after_it == after.error_t_by_keyframe.end())
            {
                continue;
            }
            sum_before += before_it->second;
            sum_after += after_it->second;
            ++valid;
        }

        const double mean_before =
            valid == 0 ? 0.0 : sum_before / static_cast<double>(valid);
        const double mean_after =
            valid == 0 ? 0.0 : sum_after / static_cast<double>(valid);
        RCLCPP_WARN(
            get_logger(),
            "[F1L-GT-COLLATERAL-CHECK] task_id=%llu valid_kfs=%llu previous_fid_neighborhood_mean_before=%.6f previous_fid_neighborhood_mean_after=%.6f source=GAZEBO_GT_DEBUG",
            static_cast<unsigned long long>(task_id),
            static_cast<unsigned long long>(valid),
            mean_before,
            mean_after);
    }

    void ExportF1LDebugAnimation(
        const PoseGraphProblem& problem,
        const OptimizationDryRunResult& dry_run,
        const OptimizationApplyResult* apply_result = nullptr,
        const GlobalPoseStore* pose_source_override = nullptr,
        const std::map<RawKeyFrameId, DebugGtKeyFramePose>*
            gt_source_override = nullptr) const
    {
        if (!f1l_debug_animation_enabled_)
        {
            return;
        }

        struct AnimationPoint
        {
            RawKeyFrameId id;
            Eigen::Matrix4d before = Eigen::Matrix4d::Identity();
            std::optional<Eigen::Matrix4d> after;
            std::optional<Eigen::Matrix4d> gt;
            bool is_vertex = false;
            bool is_fixed = false;
            bool is_target = false;
            bool is_anchor_neighborhood = false;
        };

        const GlobalPoseStore& pose_source =
            pose_source_override ? *pose_source_override : pose_store_;
        const auto& gt_source =
            gt_source_override ? *gt_source_override : f1l_gt_keyframe_store_;

        std::map<RawKeyFrameId, AnimationPoint> points;
        auto ensure_point = [&](const RawKeyFrameId& id) -> AnimationPoint&
        {
            auto& point = points[id];
            point.id = id;
            return point;
        };

        for (const auto& vertex : problem.vertices)
        {
            auto& point = ensure_point(vertex.keyframe_id);
            point.before = vertex.initial_world_T_kf;
            point.is_vertex = true;
            point.is_fixed = vertex.is_fixed || vertex.is_hard_fiducial;
            point.is_target = vertex.keyframe_id == problem.target_keyframe_id;
            point.is_anchor_neighborhood = vertex.is_anchor_neighborhood;
        }
        for (const auto& keyframe_id : problem.affected_non_variable_keyframes)
        {
            auto& point = ensure_point(keyframe_id);
            const auto pose = pose_source.GetWorldPose(keyframe_id);
            if (pose.has_value())
            {
                point.before = pose.value();
            }
        }
        for (const auto& proposal : dry_run.proposed_vertex_poses)
        {
            auto& point = ensure_point(proposal.keyframe_id);
            point.before = proposal.before_world_T_kf;
            point.after = proposal.proposed_world_T_kf;
            point.is_vertex = true;
            point.is_fixed = point.is_fixed || proposal.fixed_vertex;
            point.is_target = point.is_target ||
                proposal.keyframe_id == problem.target_keyframe_id;
        }
        for (const auto& proposal : dry_run.proposed_propagated_poses)
        {
            auto& point = ensure_point(proposal.keyframe_id);
            point.before = proposal.before_world_T_kf;
            point.after = proposal.proposed_world_T_kf;
        }
        if (apply_result)
        {
            auto add_applied_record =
                [&](const orbslam3_multi::OptimizationApplyKeyFrameRecord& record)
            {
                if (!record.applied)
                {
                    return;
                }
                auto& point = ensure_point(record.keyframe_id);
                point.before = record.before_world_T_kf;
                point.after = record.after_world_T_kf;
                point.is_target = point.is_target ||
                    record.keyframe_id == problem.target_keyframe_id;
            };
            for (const auto& record : apply_result->optimized_records)
            {
                add_applied_record(record);
            }
            for (const auto& record : apply_result->propagated_records)
            {
                add_applied_record(record);
            }
        }
        for (auto& [keyframe_id, point] : points)
        {
            const auto gt_it = gt_source.find(keyframe_id);
            if (gt_it != gt_source.end())
            {
                point.gt = gt_it->second.world_T_kf_gt;
            }
        }

        double min_x = std::numeric_limits<double>::infinity();
        double max_x = -std::numeric_limits<double>::infinity();
        double min_y = std::numeric_limits<double>::infinity();
        double max_y = -std::numeric_limits<double>::infinity();
        auto observe = [&](const Eigen::Matrix4d& pose)
        {
            min_x = std::min(min_x, pose(0, 3));
            max_x = std::max(max_x, pose(0, 3));
            min_y = std::min(min_y, pose(1, 3));
            max_y = std::max(max_y, pose(1, 3));
        };
        uint64_t vertex_count = 0;
        uint64_t gt_count = 0;
        uint64_t after_count = 0;
        for (const auto& [keyframe_id, point] : points)
        {
            (void)keyframe_id;
            observe(point.before);
            if (point.gt.has_value())
            {
                observe(point.gt.value());
                ++gt_count;
            }
            if (point.after.has_value())
            {
                observe(point.after.value());
                ++after_count;
            }
            if (point.is_vertex)
            {
                ++vertex_count;
            }
        }
        if (points.empty() || !std::isfinite(min_x) || !std::isfinite(min_y))
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1L-DEBUG-ANIMATION-EXPORT] task_id=%llu enabled=true success=false path=none reason=empty_window",
                static_cast<unsigned long long>(problem.task_id));
            return;
        }

        const double margin = 1.0;
        min_x -= margin;
        max_x += margin;
        min_y -= margin;
        max_y += margin;
        const double width = std::max(1.0, max_x - min_x);
        const double height = std::max(1.0, max_y - min_y);
        const double canvas = 1000.0;
        auto sx = [&](double x) { return 50.0 + (x - min_x) / width * (canvas - 100.0); };
        auto sy = [&](double y) { return canvas - 80.0 - (y - min_y) / height * (canvas - 140.0); };

        const std::string slash =
            (!f1l_debug_animation_output_dir_.empty() &&
             f1l_debug_animation_output_dir_.back() == '/') ? "" : "/";
        const std::string path =
            f1l_debug_animation_output_dir_ + slash +
            "f3l_debug_animation_task_" + std::to_string(problem.task_id) + ".html";
        std::error_code mkdir_error;
        std::filesystem::create_directories(
            f1l_debug_animation_output_dir_,
            mkdir_error);
        if (mkdir_error)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1L-DEBUG-ANIMATION-EXPORT] task_id=%llu enabled=true success=false path=%s reason=mkdir_failed_%s",
                static_cast<unsigned long long>(problem.task_id),
                path.c_str(),
                mkdir_error.message().c_str());
            return;
        }
        std::ofstream out(path);
        if (!out)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1L-DEBUG-ANIMATION-EXPORT] task_id=%llu enabled=true success=false path=%s reason=open_failed",
                static_cast<unsigned long long>(problem.task_id),
                path.c_str());
            return;
        }

        auto write_pose_js = [&](const Eigen::Matrix4d& pose)
        {
            Eigen::Vector3d direction = pose.block<3, 3>(0, 0).col(2);
            if (direction.norm() <= 1e-9)
            {
                direction = pose.block<3, 3>(0, 0).col(0);
            }
            if (direction.norm() > 1e-9)
            {
                direction.normalize();
            }
            out << "{p:[" << pose(0, 3) << "," << pose(1, 3) << ","
                << pose(2, 3) << "],d:[" << direction.x() << ","
                << direction.y() << "," << direction.z() << "]}";
        };
        auto write_point_js = [&](const AnimationPoint& point)
        {
            out << "{id:'" << point.id.drone_id << "_"
                << point.id.map_epoch << "_" << point.id.local_kf_id
                << "',kf:" << point.id.local_kf_id
                << ",drone:" << point.id.drone_id
                << ",epoch:" << point.id.map_epoch
                << ",vertex:" << (point.is_vertex ? "true" : "false")
                << ",fixed:" << (point.is_fixed ? "true" : "false")
                << ",target:" << (point.is_target ? "true" : "false")
                << ",before:";
            write_pose_js(point.before);
            out << ",after:";
            if (point.after.has_value())
            {
                write_pose_js(point.after.value());
            }
            else
            {
                out << "null";
            }
            out << ",gt:";
            if (point.gt.has_value())
            {
                write_pose_js(point.gt.value());
            }
            else
            {
                out << "null";
            }
            out << "}";
        };

        out << std::setprecision(10);
        out << "<!doctype html><html><head><meta charset=\"utf-8\"/>\n";
        out << "<title>F1L optimization task " << problem.task_id
            << " 3D</title>\n";
        out << "<style>body{font-family:sans-serif;margin:0;background:#f7f7f4;color:#1f2933;}"
               "header{padding:12px 18px;background:#111827;color:white;}"
               "button{margin:8px 4px;padding:8px 12px;border:1px solid #999;background:white;cursor:pointer;}"
               "button.active{background:#1d4ed8;color:white;border-color:#1d4ed8;}"
               ".wrap{padding:12px 18px}.legend span{display:inline-block;margin-right:16px;font-size:13px}"
               "canvas{background:white;border:1px solid #d1d5db;max-width:100%;height:auto;touch-action:none}"
               "</style></head><body><header><strong>F1L task "
            << problem.task_id << " 3D</strong> &nbsp; before="
            << dry_run.before_error_t << " after=" << dry_run.after_error_t
            << " useful=" << (dry_run.useful ? "true" : "false")
            << " partial=" << (dry_run.partial_candidate ? "true" : "false")
            << "</header><div class=\"wrap\">\n";
        out << "<div><button id=\"b0\" class=\"active\" onclick=\"setMode(0)\">Inicial</button>"
               "<button id=\"b1\" onclick=\"setMode(1)\">Grafo</button>"
               "<button id=\"b2\" onclick=\"setMode(2)\">Optimizado</button>"
               "<button onclick=\"resetView()\">Reset</button></div>\n";
        out << "<p class=\"legend\"><span style=\"color:#dc2626\">rojo: mapa antes</span>"
               "<span style=\"color:#111\">negro: GT</span>"
               "<span style=\"color:#7c3aed\">morado: vertices</span>"
               "<span style=\"color:#f97316\">naranja: fijo/fiducial</span>"
               "<span style=\"color:#2563eb\">azul: propagados</span>"
               "<span style=\"color:#16a34a\">verde: optimizado</span></p>\n";
        out << "<canvas id=\"scene\" width=\"1200\" height=\"820\"></canvas>\n";
        out << "<p>window_kfs=" << points.size()
            << " vertices=" << vertex_count
            << " gt_valid=" << gt_count
            << " optimized_or_propagated=" << after_count
            << ". Arrastra para girar; rueda para zoom. Las flechas usan el eje óptico +Z del KF.</p>\n";
        out << "<script>const points=[";
        bool first_point = true;
        for (const auto& [keyframe_id, point] : points)
        {
            (void)keyframe_id;
            if (!first_point) out << ",";
            first_point = false;
            write_point_js(point);
        }
        out << "];\nconst edges=[";
        bool first_edge = true;
        for (const auto& edge : problem.edges)
        {
            if (!first_edge) out << ",";
            first_edge = false;
            out << "{from:'" << edge.from_keyframe_id.drone_id << "_"
                << edge.from_keyframe_id.map_epoch << "_"
                << edge.from_keyframe_id.local_kf_id << "',to:'"
                << edge.to_keyframe_id.drone_id << "_"
                << edge.to_keyframe_id.map_epoch << "_"
                << edge.to_keyframe_id.local_kf_id << "',support:"
                << edge.support_keyframe_count << "}";
        }
        if (apply_result)
        {
            std::optional<RawKeyFrameId> previous;
            for (const auto& [keyframe_id, point] : points)
            {
                (void)point;
                if (previous.has_value() &&
                    previous->drone_id == keyframe_id.drone_id &&
                    previous->map_epoch == keyframe_id.map_epoch &&
                    (previous->local_kf_id > problem.target_keyframe_id.local_kf_id ||
                     keyframe_id.local_kf_id > problem.target_keyframe_id.local_kf_id))
                {
                    if (!first_edge) out << ",";
                    first_edge = false;
                    out << "{from:'" << previous->drone_id << "_"
                        << previous->map_epoch << "_"
                        << previous->local_kf_id << "',to:'"
                        << keyframe_id.drone_id << "_"
                        << keyframe_id.map_epoch << "_"
                        << keyframe_id.local_kf_id
                        << "',support:1}";
                }
                previous = keyframe_id;
            }
        }
        out << "];\n";
        out << R"JS(
const canvas=document.getElementById('scene'),ctx=canvas.getContext('2d');
const byId=new Map(points.map(p=>[p.id,p]));
let mode=0,yaw=-0.75,pitch=0.85,zoom=44,drag=false,lastX=0,lastY=0;
const center=[0,0,0];let count=0;
for(const p of points){for(const key of ['before','after','gt']){const q=p[key];if(q){center[0]+=q.p[0];center[1]+=q.p[1];center[2]+=q.p[2];count++;}}}
if(count){center[0]/=count;center[1]/=count;center[2]/=count;}
function setMode(m){mode=m;for(let i=0;i<3;i++)document.getElementById('b'+i).classList.toggle('active',i===m);draw();}
function resetView(){yaw=-0.75;pitch=0.85;zoom=44;draw();}
function poseOf(p){if(mode===2&&p.after)return p.after;return p.before;}
function rot(v){const x=v[0]-center[0],y=v[1]-center[1],z=v[2]-center[2];const cy=Math.cos(yaw),sy=Math.sin(yaw),cp=Math.cos(pitch),sp=Math.sin(pitch);const x1=cy*x+sy*y;const y1=-sy*x+cy*y;const z1=z;return [x1,cp*y1-sp*z1,sp*y1+cp*z1];}
function project(pos){const r=rot(pos);const persp=900/(900+r[2]);return [canvas.width/2+r[0]*zoom*persp,canvas.height/2-r[1]*zoom*persp,r[2]];}
function color(p){if(mode===2&&p.after)return p.vertex?'#16a34a':'#2563eb';if(p.fixed)return '#f97316';if(p.vertex)return '#7c3aed';return '#dc2626';}
function arrow(pos,dir,len,colorValue){const a=project(pos),b=project([pos[0]+dir[0]*len,pos[1]+dir[1]*len,pos[2]+dir[2]*len]);ctx.strokeStyle=colorValue;ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(a[0],a[1]);ctx.lineTo(b[0],b[1]);ctx.stroke();const ang=Math.atan2(b[1]-a[1],b[0]-a[0]);ctx.beginPath();ctx.moveTo(b[0],b[1]);ctx.lineTo(b[0]-8*Math.cos(ang-0.45),b[1]-8*Math.sin(ang-0.45));ctx.lineTo(b[0]-8*Math.cos(ang+0.45),b[1]-8*Math.sin(ang+0.45));ctx.closePath();ctx.fillStyle=colorValue;ctx.fill();}
function draw(){ctx.clearRect(0,0,canvas.width,canvas.height);ctx.fillStyle='white';ctx.fillRect(0,0,canvas.width,canvas.height);ctx.strokeStyle='#e5e7eb';ctx.lineWidth=1;for(const e of edges){const a=byId.get(e.from),b=byId.get(e.to);if(!a||!b)continue;const pa=project(poseOf(a).p),pb=project(poseOf(b).p);ctx.beginPath();ctx.moveTo(pa[0],pa[1]);ctx.lineTo(pb[0],pb[1]);ctx.stroke();}const drawable=[];for(const p of points){const q=poseOf(p);drawable.push({p,q,screen:project(q.p)});if(p.gt){const g=project(p.gt.p);ctx.fillStyle='#111';ctx.globalAlpha=0.35;ctx.beginPath();ctx.arc(g[0],g[1],3,0,Math.PI*2);ctx.fill();ctx.globalAlpha=1;}}drawable.sort((a,b)=>a.screen[2]-b.screen[2]);for(const item of drawable){const p=item.p,q=item.q,s=item.screen;ctx.fillStyle=color(p);ctx.globalAlpha=p.vertex?0.95:0.65;ctx.beginPath();ctx.arc(s[0],s[1],p.target?8:(p.vertex?6:3),0,Math.PI*2);ctx.fill();ctx.globalAlpha=1;if(p.vertex)arrow(q.p,q.d,p.target?0.9:0.55,color(p));}ctx.fillStyle='#374151';ctx.fillText('Vista 3D real: X/Y/Z y flechas de orientacion de KF',16,24);}
canvas.addEventListener('mousedown',e=>{drag=true;lastX=e.clientX;lastY=e.clientY;});
window.addEventListener('mouseup',()=>drag=false);
canvas.addEventListener('mousemove',e=>{if(!drag)return;yaw+=(e.clientX-lastX)*0.008;pitch+=(e.clientY-lastY)*0.008;pitch=Math.max(-1.55,Math.min(1.55,pitch));lastX=e.clientX;lastY=e.clientY;draw();});
canvas.addEventListener('wheel',e=>{e.preventDefault();zoom*=Math.exp(-e.deltaY*0.001);zoom=Math.max(5,Math.min(250,zoom));draw();},{passive:false});
draw();
)JS";
        out << "</script></div></body></html>\n";

        RCLCPP_WARN(
            get_logger(),
            "[F1L-DEBUG-ANIMATION-EXPORT] task_id=%llu enabled=true success=true path=%s window_kfs=%llu vertices=%llu gt_valid=%llu optimized_or_propagated=%llu format=html_3d_canvas",
            static_cast<unsigned long long>(problem.task_id),
            path.c_str(),
            static_cast<unsigned long long>(points.size()),
            static_cast<unsigned long long>(vertex_count),
            static_cast<unsigned long long>(gt_count),
            static_cast<unsigned long long>(after_count));
        return;

        auto write_circle = [&](const AnimationPoint& point,
                                const Eigen::Matrix4d& pose,
                                const std::string& fill,
                                double radius,
                                double opacity)
        {
            out << "<circle cx=\"" << sx(pose(0, 3))
                << "\" cy=\"" << sy(pose(1, 3))
                << "\" r=\"" << radius
                << "\" fill=\"" << fill
                << "\" opacity=\"" << opacity
                << "\"><title>kf=" << point.id.local_kf_id
                << " drone=" << point.id.drone_id
                << " epoch=" << point.id.map_epoch
                << " x=" << pose(0, 3)
                << " y=" << pose(1, 3)
                << "</title></circle>\n";
        };

        out << "<!doctype html><html><head><meta charset=\"utf-8\"/>\n";
        out << "<title>F1L optimization task " << problem.task_id << "</title>\n";
        out << "<style>body{font-family:sans-serif;margin:0;background:#f7f7f4;color:#1f2933;}"
               "header{padding:12px 18px;background:#111827;color:white;}"
               "button{margin:8px 4px;padding:8px 12px;border:1px solid #999;background:white;cursor:pointer;}"
               "button.active{background:#1d4ed8;color:white;border-color:#1d4ed8;}"
               ".wrap{padding:12px 18px;}.frame{display:none}.frame.active{display:block}"
               "svg{background:white;border:1px solid #d1d5db;max-width:100%;height:auto}"
               ".legend span{display:inline-block;margin-right:16px;font-size:13px}</style>\n";
        out << "</head><body><header><strong>F1L task " << problem.task_id
            << "</strong> &nbsp; before=" << dry_run.before_error_t
            << " after=" << dry_run.after_error_t
            << " useful=" << (dry_run.useful ? "true" : "false")
            << " partial=" << (dry_run.partial_candidate ? "true" : "false")
            << "</header><div class=\"wrap\">\n";
        out << "<div><button id=\"b0\" class=\"active\" onclick=\"showFrame(0)\">Inicial</button>"
               "<button id=\"b1\" onclick=\"showFrame(1)\">Grafo</button>"
               "<button id=\"b2\" onclick=\"showFrame(2)\">Optimizado</button>"
               "<button onclick=\"togglePlay()\">Play/Pausa</button></div>\n";
        out << "<p class=\"legend\"><span style=\"color:#dc2626\">rojo: mapa antes</span>"
               "<span style=\"color:#111\">negro: GT</span>"
               "<span style=\"color:#7c3aed\">morado: vertices antes</span>"
               "<span style=\"color:#6b7280\">gris: GT de vertices</span>"
               "<span style=\"color:#2563eb\">azul: mapa optimizado/propagado</span>"
               "<span style=\"color:#16a34a\">verde: vertices optimizados</span></p>\n";

        auto write_svg_open = [&]()
        {
            out << "<svg width=\"1000\" height=\"1000\" viewBox=\"0 0 1000 1000\">\n";
            out << "<rect width=\"1000\" height=\"1000\" fill=\"white\"/>\n";
            out << "<text x=\"30\" y=\"970\" font-size=\"14\">XY projection. GT is diagnostic only and never feeds the solver.</text>\n";
        };
        auto write_edges = [&](const std::string& color, bool optimized)
        {
            for (const auto& edge : problem.edges)
            {
                const auto from_it = points.find(edge.from_keyframe_id);
                const auto to_it = points.find(edge.to_keyframe_id);
                if (from_it == points.end() || to_it == points.end())
                {
                    continue;
                }
                const auto& from_pose =
                    optimized && from_it->second.after.has_value()
                        ? from_it->second.after.value()
                        : from_it->second.before;
                const auto& to_pose =
                    optimized && to_it->second.after.has_value()
                        ? to_it->second.after.value()
                        : to_it->second.before;
                out << "<line x1=\"" << sx(from_pose(0, 3))
                    << "\" y1=\"" << sy(from_pose(1, 3))
                    << "\" x2=\"" << sx(to_pose(0, 3))
                    << "\" y2=\"" << sy(to_pose(1, 3))
                    << "\" stroke=\"" << color
                    << "\" stroke-width=\"1.5\" opacity=\"0.55\"/>\n";
            }
        };

        out << "<div id=\"f0\" class=\"frame active\">";
        write_svg_open();
        for (const auto& [keyframe_id, point] : points)
        {
            (void)keyframe_id;
            write_circle(point, point.before, "#dc2626", 3.5, 0.80);
            if (point.gt.has_value())
            {
                write_circle(point, point.gt.value(), "#111111", 3.0, 0.75);
            }
        }
        out << "</svg></div>\n";

        out << "<div id=\"f1\" class=\"frame\">";
        write_svg_open();
        write_edges("#7c3aed", false);
        for (const auto& [keyframe_id, point] : points)
        {
            (void)keyframe_id;
            write_circle(point, point.before, "#dc2626", 2.5, 0.30);
            if (point.gt.has_value())
            {
                write_circle(point, point.gt.value(), "#111111", 2.2, 0.25);
            }
            if (point.is_vertex)
            {
                write_circle(point, point.before,
                             point.is_fixed ? "#f97316" : "#7c3aed",
                             point.is_target ? 10.0 : 7.0,
                             0.95);
                if (point.gt.has_value())
                {
                    write_circle(point, point.gt.value(), "#6b7280",
                                 point.is_target ? 9.0 : 6.0,
                                 0.90);
                }
            }
        }
        out << "</svg></div>\n";

        out << "<div id=\"f2\" class=\"frame\">";
        write_svg_open();
        write_edges("#16a34a", true);
        for (const auto& [keyframe_id, point] : points)
        {
            (void)keyframe_id;
            if (point.gt.has_value())
            {
                write_circle(point, point.gt.value(), "#111111", 2.5, 0.35);
            }
            if (point.after.has_value())
            {
                write_circle(point, point.after.value(),
                             point.is_vertex ? "#16a34a" : "#2563eb",
                             point.is_vertex ? (point.is_target ? 10.0 : 7.0) : 3.5,
                             0.90);
            }
        }
        out << "</svg></div>\n";

        out << "<p>window_kfs=" << points.size()
            << " vertices=" << vertex_count
            << " gt_valid=" << gt_count
            << " optimized_or_propagated=" << after_count
            << "</p>\n";
        out << "<script>let frame=0,playing=true;function showFrame(i){frame=i;"
               "for(let n=0;n<3;n++){document.getElementById('f'+n).classList.toggle('active',n===i);"
               "document.getElementById('b'+n).classList.toggle('active',n===i);}}"
               "function togglePlay(){playing=!playing;}setInterval(()=>{if(playing)showFrame((frame+1)%3);},1400);</script>\n";
        out << "</div></body></html>\n";

        RCLCPP_WARN(
            get_logger(),
            "[F1L-DEBUG-ANIMATION-EXPORT] task_id=%llu enabled=true success=true path=%s window_kfs=%llu vertices=%llu gt_valid=%llu optimized_or_propagated=%llu format=html_animation",
            static_cast<unsigned long long>(problem.task_id),
            path.c_str(),
            static_cast<unsigned long long>(points.size()),
            static_cast<unsigned long long>(vertex_count),
            static_cast<unsigned long long>(gt_count),
            static_cast<unsigned long long>(after_count));
    }

    const FiducialConfig* FindContainingFiducial(const Eigen::Matrix4d& world_T_body,
                                                 double& distance_m) const
    {
        const FiducialConfig* best = nullptr;
        double best_distance = 0.0;
        for (const auto& fiducial : fiducials_)
        {
            const Eigen::Vector3d delta =
                world_T_body.block<3, 1>(0, 3) -
                fiducial.world_T_fiducial.block<3, 1>(0, 3);
            const double distance = delta.norm();
            if (!best || distance < best_distance)
            {
                best = &fiducial;
                best_distance = distance;
            }
        }

        distance_m = best_distance;
        if (!best || best_distance > best->radius_m)
        {
            return nullptr;
        }
        return best;
    }

    uint64_t MakeGlobalKeyFrameId(const RawKeyFrameId& keyframe_id) const
    {
        // F1E: ID compacto solo para logs/journal. La identidad canonica sigue
        // siendo `(drone_id, map_epoch, local_kf_id)`.
        return static_cast<uint64_t>(keyframe_id.drone_id) * 1000000000000ULL +
               keyframe_id.map_epoch * 1000000ULL +
               keyframe_id.local_kf_id;
    }

    std::optional<uint64_t> ProcessFiducialsForDelta(
        const OrbMap& map,
        uint64_t arrival_id)
    {
        if (!fiducial_sim_enabled_ || rawdb_replay_enabled_ || fiducials_.empty())
        {
            return std::nullopt;
        }

        std::optional<uint64_t> latest_publication_revision;
        for (const auto& keyframe : map.keyframes)
        {
            const RawKeyFrameId keyframe_id{
                map.drone_id,
                map.map_epoch,
                keyframe.id};
            if (live_fiducial_observed_keyframes_.find(keyframe_id) !=
                live_fiducial_observed_keyframes_.end())
            {
                continue;
            }

            GroundTruthSample gt_sample;
            double dt_sec = 0.0;
            std::string source;
            const rclcpp::Time keyframe_stamp(keyframe.stamp);
            if (!FindBestGroundTruth(map.drone_id, keyframe_stamp, gt_sample, dt_sec, source))
            {
                continue;
            }

            double distance_m = 0.0;
            const FiducialConfig* fiducial =
                FindContainingFiducial(gt_sample.world_T_body, distance_m);
            RCLCPP_WARN(
                get_logger(),
                "[F1E-FID-CANDIDATE-GT] arrival_id=%llu drone_id=%u epoch=%llu kf=%llu kf_stamp=%.6f gt_stamp=%.6f dt=%.6f dist_to_best_fid=%.3f inside=%s source=%s",
                static_cast<unsigned long long>(arrival_id),
                map.drone_id,
                static_cast<unsigned long long>(map.map_epoch),
                static_cast<unsigned long long>(keyframe.id),
                StampToSeconds(keyframe.stamp),
                gt_sample.stamp.seconds(),
                dt_sec,
                distance_m,
                fiducial ? "true" : "false",
                source.c_str());

            if (!fiducial)
            {
                continue;
            }

            FiducialObservation observation;
            observation.arrival_id = arrival_id;
            observation.drone_id = map.drone_id;
            observation.map_epoch = map.map_epoch;
            observation.local_keyframe_id = keyframe.id;
            observation.global_keyframe_id = MakeGlobalKeyFrameId(keyframe_id);
            observation.fiducial_id = fiducial->id;
            observation.world_T_body_fiducial = gt_sample.world_T_body;
            observation.keyframe_stamp_sec = StampToSeconds(keyframe.stamp);
            observation.gt_stamp_sec = gt_sample.stamp.seconds();
            observation.association_dt_sec = dt_sec;
            observation.distance_to_fiducial_m = distance_m;
            observation.source = source;

            RCLCPP_WARN(
                get_logger(),
                "[F1E-FID-KF-ASSOC] arrival_id=%llu drone_id=%u epoch=%llu kf=%llu fid=%d kf_stamp=%.6f gt_stamp=%.6f dt=%.6f dist_to_fid=%.3f source=%s",
                static_cast<unsigned long long>(arrival_id),
                observation.drone_id,
                static_cast<unsigned long long>(observation.map_epoch),
                static_cast<unsigned long long>(observation.local_keyframe_id),
                observation.fiducial_id,
                observation.keyframe_stamp_sec,
                observation.gt_stamp_sec,
                observation.association_dt_sec,
                observation.distance_to_fiducial_m,
                observation.source.c_str());

            const auto publication_revision =
                HandleFiducialObservation(observation, true);
            if (publication_revision.has_value())
            {
                latest_publication_revision = std::max(
                    latest_publication_revision.value_or(0),
                    publication_revision.value());
            }
            live_fiducial_observed_keyframes_.insert(keyframe_id);
        }
        return latest_publication_revision;
    }

    std::optional<uint64_t> HandleFiducialObservation(
        const FiducialObservation& observation,
        bool persist_observation)
    {
        TryTraceFlow(
            "server_fiducial_observation",
            "observe",
            "fiducial_" + std::to_string(observation.fiducial_id),
            0,
            1);
        RCLCPP_WARN(
            get_logger(),
            "[F1E-FID-OBS] arrival_id=%llu drone_id=%u epoch=%llu kf=%llu fid=%d source=%s",
            static_cast<unsigned long long>(observation.arrival_id),
            observation.drone_id,
            static_cast<unsigned long long>(observation.map_epoch),
            static_cast<unsigned long long>(observation.local_keyframe_id),
            observation.fiducial_id,
            observation.source.c_str());

        const auto result =
            fiducial_anchor_manager_.RegisterFiducialObservation(
                observation,
                raw_db_,
                pose_store_);

        if (!result.observation_accepted)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1E-FID-OBS] arrival_id=%llu status=rejected reason=%s",
                static_cast<unsigned long long>(observation.arrival_id),
                result.reason.c_str());
            return std::nullopt;
        }

        const uint64_t publication_revision = RequestGlobalStatePublication(
            result.first_anchor_created
                ? "fiducial_first_anchor_commit"
                : "fiducial_pose_commit");

        RCLCPP_WARN(
            get_logger(),
            "[F1E-FID-WORLD-T-LOCAL] arrival_id=%llu fid=%d drone_id=%u epoch=%llu kf=%llu world_T_local_t=(%.3f,%.3f,%.3f) yaw=%.3f source=%s",
            static_cast<unsigned long long>(observation.arrival_id),
            observation.fiducial_id,
            observation.drone_id,
            static_cast<unsigned long long>(observation.map_epoch),
            static_cast<unsigned long long>(observation.local_keyframe_id),
            result.world_T_local(0, 3),
            result.world_T_local(1, 3),
            result.world_T_local(2, 3),
            YawFromTransform(result.world_T_local),
            observation.source.c_str());

        RCLCPP_WARN(
            get_logger(),
            "[F1F-BODY-CAMERA-APPLY] arrival_id=%llu fid=%d drone_id=%u epoch=%llu kf=%llu body_t=(%.3f,%.3f,%.3f) camera_t=(%.3f,%.3f,%.3f) source=%s",
            static_cast<unsigned long long>(observation.arrival_id),
            observation.fiducial_id,
            observation.drone_id,
            static_cast<unsigned long long>(observation.map_epoch),
            static_cast<unsigned long long>(observation.local_keyframe_id),
            observation.world_T_body_fiducial(0, 3),
            observation.world_T_body_fiducial(1, 3),
            observation.world_T_body_fiducial(2, 3),
            result.world_T_camera_fiducial(0, 3),
            result.world_T_camera_fiducial(1, 3),
            result.world_T_camera_fiducial(2, 3),
            observation.source.c_str());

        if (result.first_anchor_created)
        {
            TryTraceFlow(
                "fiducial_pose_anchor",
                "commit",
                "first_anchor",
                0,
                1);
            RCLCPP_WARN(
                get_logger(),
                "[F1E-FID-FIRST-ANCHOR] fid=%d drone_id=%u epoch=%llu kf=%llu body_t=(%.3f,%.3f,%.3f) camera_t=(%.3f,%.3f,%.3f) world_T_local_t=(%.3f,%.3f,%.3f) yaw=%.3f",
                observation.fiducial_id,
                observation.drone_id,
                static_cast<unsigned long long>(observation.map_epoch),
                static_cast<unsigned long long>(observation.local_keyframe_id),
                observation.world_T_body_fiducial(0, 3),
                observation.world_T_body_fiducial(1, 3),
                observation.world_T_body_fiducial(2, 3),
                result.world_T_camera_fiducial(0, 3),
                result.world_T_camera_fiducial(1, 3),
                result.world_T_camera_fiducial(2, 3),
                result.world_T_local(0, 3),
                result.world_T_local(1, 3),
                result.world_T_local(2, 3),
                YawFromTransform(result.world_T_local));

            RCLCPP_WARN(
                get_logger(),
                "[F1D-POSESTORE-ANCHOR-SET] drone_id=%u epoch=%llu source=%s replaced=false",
                observation.drone_id,
                static_cast<unsigned long long>(observation.map_epoch),
                observation.source.c_str());
        }

        if (result.keyframe_marked_hard_fiducial)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1E-FID-KF-HARD] drone_id=%u epoch=%llu kf=%llu fid=%d source=%s",
                observation.drone_id,
                static_cast<unsigned long long>(observation.map_epoch),
                static_cast<unsigned long long>(observation.local_keyframe_id),
                observation.fiducial_id,
                observation.source.c_str());
        }

        if (result.revisit)
        {
            const char* decision = "UNKNOWN";
            if (result.revisit_ok)
            {
                decision = "OK";
            }
            else if (result.task_created)
            {
                decision = "TASK_CREATED";
            }
            else if (result.duplicate_task)
            {
                decision = "TASK_DUPLICATE";
            }
            else if (
                result.reason == "same_fiducial_visit_already_accepted")
            {
                decision = "SAME_VISIT_SUPPRESSED";
            }

            RCLCPP_WARN(
                get_logger(),
                "[F1H-FID-REVISIT] arrival_id=%llu fid=%d drone_id=%u epoch=%llu kf=%llu source=%s decision=%s",
                static_cast<unsigned long long>(observation.arrival_id),
                observation.fiducial_id,
                observation.drone_id,
                static_cast<unsigned long long>(observation.map_epoch),
                static_cast<unsigned long long>(observation.local_keyframe_id),
                observation.source.c_str(),
                decision);

            if (result.pose_error_valid)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1H-FID-POSE-ERROR] fid=%d kf=%llu drone_id=%u epoch=%llu error_t=%.6f error_rot=%.6f error_yaw=%.6f threshold_t=%.6f threshold_rot=%.6f threshold_yaw=%.6f decision=%s source=%s",
                    observation.fiducial_id,
                    static_cast<unsigned long long>(observation.local_keyframe_id),
                    observation.drone_id,
                    static_cast<unsigned long long>(observation.map_epoch),
                    result.error_t_m,
                    result.error_rot_rad,
                    result.error_yaw_rad,
                    result.threshold_t_m,
                    result.threshold_rot_rad,
                    result.threshold_yaw_rad,
                    decision,
                    observation.source.c_str());
            }

            if (result.revisit_ok)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1H-FID-OK] arrival_id=%llu fid=%d drone_id=%u epoch=%llu kf=%llu error_t=%.6f error_yaw=%.6f",
                    static_cast<unsigned long long>(observation.arrival_id),
                    observation.fiducial_id,
                    observation.drone_id,
                    static_cast<unsigned long long>(observation.map_epoch),
                    static_cast<unsigned long long>(observation.local_keyframe_id),
                    result.error_t_m,
                    result.error_yaw_rad);
            }
            else if (result.task_created)
            {
                TryTraceFlow(
                    "fiducial_secondary_queue",
                    "enqueue",
                    "fiducial_optimization",
                    result.task_id,
                    1);
                RCLCPP_WARN(
                    get_logger(),
                    "[F1H-FID-TASK-CREATED] task_id=%llu arrival_id=%llu fid=%d drone_id=%u epoch=%llu kf=%llu error_t=%.6f error_rot=%.6f error_yaw=%.6f",
                    static_cast<unsigned long long>(result.task_id),
                    static_cast<unsigned long long>(observation.arrival_id),
                    observation.fiducial_id,
                    observation.drone_id,
                    static_cast<unsigned long long>(observation.map_epoch),
                    static_cast<unsigned long long>(observation.local_keyframe_id),
                    result.error_t_m,
                    result.error_rot_rad,
                    result.error_yaw_rad);
            }
            else if (result.duplicate_task)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1H-FID-TASK-SKIP-DUPLICATE] task_id=%llu arrival_id=%llu fid=%d drone_id=%u epoch=%llu kf=%llu",
                    static_cast<unsigned long long>(result.task_id),
                    static_cast<unsigned long long>(observation.arrival_id),
                    observation.fiducial_id,
                    observation.drone_id,
                    static_cast<unsigned long long>(observation.map_epoch),
                    static_cast<unsigned long long>(observation.local_keyframe_id));
            }
            else if (
                result.reason == "same_fiducial_visit_already_accepted")
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1H-FID-SAME-VISIT-SUPPRESSED] arrival_id=%llu fid=%d drone_id=%u epoch=%llu kf=%llu error_t=%.6f reason=%s",
                    static_cast<unsigned long long>(observation.arrival_id),
                    observation.fiducial_id,
                    observation.drone_id,
                    static_cast<unsigned long long>(observation.map_epoch),
                    static_cast<unsigned long long>(
                        observation.local_keyframe_id),
                    result.error_t_m,
                    result.reason.c_str());
            }
        }

        if (persist_observation)
        {
            raw_db_.AddFiducialObservation(ToRecordedObservation(observation));
            TryTraceFlow(
                "fiducial_raw_journal",
                "commit",
                "fiducial_observation",
                0,
                1);
            const auto fid_count =
                raw_db_.GetDatabaseStats().fiducial_observations;
            RCLCPP_WARN(
                get_logger(),
                "[F1E-FID-JOURNAL-SAVE] arrival_id=%llu fid=%d drone_id=%u epoch=%llu kf=%llu observations=%llu",
                static_cast<unsigned long long>(observation.arrival_id),
                observation.fiducial_id,
                observation.drone_id,
                static_cast<unsigned long long>(observation.map_epoch),
                static_cast<unsigned long long>(observation.local_keyframe_id),
                static_cast<unsigned long long>(fid_count));
        }

        LogFiducialStats(observation.source);
        LogPoseStoreStats("fiducial_observation");
        BuildPoseGraphsForPendingTasks(observation.source);
        return publication_revision;
    }

    void LogFiducialStats(const std::string& reason)
    {
        const auto stats = fiducial_anchor_manager_.GetStats();
        RCLCPP_WARN(
            get_logger(),
            "[F1E-FID-STATS] reason=%s observations=%llu accepted=%llu rejected=%llu anchors_created=%llu replay_observations=%llu hard_fiducial_kfs=%llu",
            reason.c_str(),
            static_cast<unsigned long long>(stats.observations),
            static_cast<unsigned long long>(stats.accepted),
            static_cast<unsigned long long>(stats.rejected),
            static_cast<unsigned long long>(stats.anchors_created),
            static_cast<unsigned long long>(stats.replay_observations),
            static_cast<unsigned long long>(stats.hard_fiducial_keyframes));

        RCLCPP_WARN(
            get_logger(),
            "[F1H-FID-TASK-STATS] reason=%s total=%llu pending=%llu confirmed_ok=%llu high_error=%llu duplicates=%llu same_visit_suppressed=%llu no_pose=%llu revisits=%llu",
            reason.c_str(),
            static_cast<unsigned long long>(stats.tasks_created),
            static_cast<unsigned long long>(
                fiducial_anchor_manager_.PendingFiducialOptimizationTaskCount()),
            static_cast<unsigned long long>(stats.revisit_ok),
            static_cast<unsigned long long>(stats.revisit_high_error),
            static_cast<unsigned long long>(stats.task_duplicates),
            static_cast<unsigned long long>(
                stats.same_visit_high_error_suppressed),
            static_cast<unsigned long long>(stats.revisit_no_pose),
                static_cast<unsigned long long>(stats.revisit_observations));
    }

    void BuildPoseGraphsForPendingTasks(const std::string& reason)
    {
        const auto tasks =
            fiducial_anchor_manager_.GetPendingFiducialOptimizationTasks();
        for (const auto& task : tasks)
        {
            if (f1i_pose_graph_built_task_ids_.find(task.task_id) !=
                f1i_pose_graph_built_task_ids_.end())
            {
                continue;
            }

            f1i_pose_graph_built_task_ids_.insert(task.task_id);
            ScheduleOptimizationWorker(task, reason);
        }
    }


    std::optional<FiducialOptimizationTask> FindPendingTaskLocked(
        uint64_t task_id) const
    {
        const auto pending =
            fiducial_anchor_manager_.GetPendingFiducialOptimizationTasks();
        const auto it = std::find_if(
            pending.begin(), pending.end(),
            [task_id](const FiducialOptimizationTask& task)
            {
                return task.task_id == task_id;
            });
        if (it == pending.end())
        {
            return std::nullopt;
        }
        return *it;
    }

    void UpdatePeakOptimizationWorkers(uint64_t active_workers)
    {
        uint64_t peak = optimization_peak_active_workers_.load();
        while (active_workers > peak &&
               !optimization_peak_active_workers_.compare_exchange_weak(
                   peak,
                   active_workers))
        {
        }
    }

    void LogSecondaryQueueSummary(const std::string& reason)
    {
        RCLCPP_WARN(
            get_logger(),
            "[F1K-QUEUE-SUMMARY] reason=%s scheduled=%llu completed=%llu fiducial_queued=%llu active_secondary_workers=%llu publication_waits=0 in_flight=%llu peak_active_workers=%llu skipped_low_error=%llu loop_queued=%zu",
            reason.c_str(),
            static_cast<unsigned long long>(optimization_workers_scheduled_.load()),
            static_cast<unsigned long long>(optimization_workers_completed_.load()),
            static_cast<unsigned long long>(optimization_queued_jobs_.load()),
            static_cast<unsigned long long>(optimization_active_workers_.load()),
            static_cast<unsigned long long>(optimization_jobs_in_flight_.load()),
            static_cast<unsigned long long>(
                optimization_peak_active_workers_.load()),
            static_cast<unsigned long long>(
                optimization_workers_skipped_low_error_.load()),
            PendingLoopTaskCount());
    }

    void ScheduleOptimizationWorker(
        const FiducialOptimizationTask& task,
        const std::string& trigger)
    {
        const uint64_t in_flight = ++optimization_jobs_in_flight_;
        RCLCPP_WARN(
            get_logger(),
            "[F1K-BACKPRESSURE-JOB] event=scheduled task_id=%llu in_flight=%llu",
            static_cast<unsigned long long>(task.task_id),
            static_cast<unsigned long long>(in_flight));

        try
        {
            const uint64_t scheduled = ++optimization_workers_scheduled_;
            RCLCPP_WARN(
                get_logger(),
                "[F1K-QUEUE-ENQUEUE] task_id=%llu drone_id=%u epoch=%llu scheduled=%llu trigger=%s policy=fifo worker=persistent",
                static_cast<unsigned long long>(task.task_id),
                task.submap_id.drone_id,
                static_cast<unsigned long long>(task.submap_id.map_epoch),
                static_cast<unsigned long long>(scheduled),
                trigger.c_str());

            {
                std::lock_guard<std::mutex> lock(secondary_task_mutex_);
                optimization_queue_.emplace_back(task, trigger);
                ++optimization_queued_jobs_;
            }
            UpdateMappingBackpressure("optimization_scheduled");
            secondary_task_condition_.notify_one();
        }
        catch (...)
        {
            --optimization_jobs_in_flight_;
            UpdateMappingBackpressure("optimization_schedule_failed");
            throw;
        }
    }

    void SecondaryWorkerLoop()
    {
        while (rclcpp::ok() && !secondary_worker_shutdown_.load())
        {
            std::optional<std::pair<FiducialOptimizationTask, std::string>>
                fiducial_job;
            std::optional<LoopTask> loop_job;
            {
                std::unique_lock<std::mutex> lock(secondary_task_mutex_);
                secondary_task_condition_.wait(
                    lock,
                    [this]()
                    {
                        return secondary_worker_shutdown_.load() ||
                            !optimization_queue_.empty() ||
                            !loop_task_queue_.empty();
                    });
                if (secondary_worker_shutdown_.load())
                {
                    return;
                }
                auto selected = orbslam3_server::SelectNextSecondaryTask(
                    optimization_queue_, loop_task_queue_);
                if (selected.fiducial)
                {
                    fiducial_job = std::move(selected.fiducial.value());
                    --optimization_queued_jobs_;
                }
                else if (selected.loop)
                {
                    loop_job = std::move(selected.loop.value());
                    loop_task_keys_.erase(loop_job->query_kf_id);
                }
                secondary_worker_active_.store(true);
            }

            if (fiducial_job)
            {
                RawMapDatabase raw_snapshot;
                GlobalPoseStore pose_snapshot;
                CovisibilityDatabase covisibility_snapshot;
                {
                    std::lock_guard<std::recursive_mutex> lock(
                        live_state_mutex_);
                    const auto current =
                        FindPendingTaskLocked(fiducial_job->first.task_id);
                    if (!current)
                    {
                        ++optimization_workers_completed_;
                        --optimization_jobs_in_flight_;
                        secondary_worker_active_.store(false);
                        UpdateMappingBackpressure(
                            "optimization_missing_task");
                        continue;
                    }
                    fiducial_job->first = current.value();
                    raw_snapshot = raw_db_.CreateStateSnapshot();
                    pose_snapshot = pose_store_;
                    covisibility_snapshot = covisibility_db_;
                }
                RCLCPP_WARN(
                    get_logger(),
                    "[F1K-SECONDARY-TASK-START] task_id=%llu type=FIDUCIAL priority=MAX queued_fiducial=%llu queued_loop=%zu",
                    static_cast<unsigned long long>(
                        fiducial_job->first.task_id),
                    static_cast<unsigned long long>(
                        optimization_queued_jobs_.load()),
                    PendingLoopTaskCount());
                TryTraceFlow(
                    "secondary_queue_pose_graph",
                    "start",
                    "fiducial_optimization",
                    fiducial_job->first.task_id);
                RunOptimizationWorker(
                    fiducial_job->first,
                    fiducial_job->second,
                    std::move(raw_snapshot),
                    std::move(pose_snapshot),
                    std::move(covisibility_snapshot));
            }
            else if (loop_job)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1K-SECONDARY-TASK-START] task_id=%llu type=LOOP priority=NORMAL query=%u:%llu:%llu queued_fiducial=%llu queued_loop=%zu",
                    static_cast<unsigned long long>(loop_job->task_id),
                    loop_job->query_kf_id.drone_id,
                    static_cast<unsigned long long>(
                        loop_job->query_kf_id.map_epoch),
                    static_cast<unsigned long long>(
                        loop_job->query_kf_id.local_kf_id),
                    static_cast<unsigned long long>(
                        optimization_queued_jobs_.load()),
                    PendingLoopTaskCount());
                TryTraceFlow(
                    "secondary_queue_loop_detector",
                    "start",
                    loop_job->trigger,
                    loop_job->task_id);
                RunLoopTask(loop_job.value());
            }
            secondary_worker_active_.store(false);
            UpdateMappingBackpressure("secondary_task_complete");
        }
    }

    void RunOptimizationWorker(
        FiducialOptimizationTask task,
        const std::string& trigger,
        RawMapDatabase raw_snapshot,
        GlobalPoseStore pose_snapshot,
        CovisibilityDatabase covisibility_snapshot)
    {
        const uint64_t active = ++optimization_active_workers_;
        UpdatePeakOptimizationWorkers(active);
        const auto worker_start = std::chrono::steady_clock::now();
        const auto graph_start = std::chrono::steady_clock::now();
        RCLCPP_WARN(
            get_logger(),
            "[F1K-WORKER-STAGE] task_id=%llu stage=graph event=start active_workers=1",
            static_cast<unsigned long long>(task.task_id));
        const auto build_result = BuildAndLogPoseGraphForTask(
            task,
            trigger,
            false,
            &raw_snapshot,
            &pose_snapshot,
            &covisibility_snapshot,
            false,
            true);
        const double graph_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - graph_start).count();
        RCLCPP_WARN(
            get_logger(),
            "[F1K-WORKER-STAGE] task_id=%llu stage=graph event=end success=%s duration_ms=%.3f",
            static_cast<unsigned long long>(task.task_id),
            build_result.success ? "true" : "false",
            graph_ms);
        if (!build_result.success)
        {
            std::lock_guard<std::recursive_mutex> lock(live_state_mutex_);
            if (build_result.reason == "previous_fiducial_anchor_missing")
            {
                CloseFiducialOptimizationTask(
                    task.task_id,
                    "graph_failed_previous_fiducial_anchor_missing");
            }
            else
            {
                f1i_pose_graph_built_task_ids_.erase(task.task_id);
            }
        }
        else
        {
            const auto optimize_start = std::chrono::steady_clock::now();
            TryTraceFlow(
                "pose_graph_optimizer",
                "start",
                "fiducial_solver",
                task.task_id,
                build_result.problem.vertices.size());
            RCLCPP_WARN(
                get_logger(),
                "[F1K-WORKER-STAGE] task_id=%llu stage=optimize event=start active_workers=1",
                static_cast<unsigned long long>(task.task_id));
            RunAndLogOptimizationDryRun(
                task,
                build_result.problem,
                trigger,
                false,
                raw_snapshot,
                pose_snapshot,
                true);
            const double optimize_ms =
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - optimize_start).count();
            RCLCPP_WARN(
                get_logger(),
                "[F1K-WORKER-STAGE] task_id=%llu stage=optimize event=end duration_ms=%.3f",
                static_cast<unsigned long long>(task.task_id),
                optimize_ms);
        }

        const double worker_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - worker_start).count();
        --optimization_active_workers_;
        ++optimization_workers_completed_;
        const uint64_t in_flight = --optimization_jobs_in_flight_;
        UpdateMappingBackpressure("optimization_complete");
        RCLCPP_WARN(
            get_logger(),
            "[F1K-SECONDARY-TASK-END] task_id=%llu type=FIDUCIAL duration_ms=%.3f shutdown=%s in_flight=%llu queued_fiducial=%llu queued_loop=%zu publication_waited=false",
            static_cast<unsigned long long>(task.task_id),
            worker_ms,
            secondary_worker_shutdown_.load() ? "true" : "false",
            static_cast<unsigned long long>(in_flight),
            static_cast<unsigned long long>(optimization_queued_jobs_.load()),
            PendingLoopTaskCount());
        LogSecondaryQueueSummary("secondary_worker_complete");
    }

    void CloseFiducialOptimizationTask(const uint64_t task_id,
                                       const std::string& reason)
    {
        const bool removed =
            fiducial_anchor_manager_.CompleteFiducialOptimizationTask(task_id);
        f1i_pose_graph_built_task_ids_.erase(task_id);
        RCLCPP_WARN(
            get_logger(),
            "[F1H-FID-TASK-CLOSED] task_id=%llu removed=%s reason=%s pending=%llu",
            static_cast<unsigned long long>(task_id),
            removed ? "true" : "false",
            reason.c_str(),
            static_cast<unsigned long long>(
                fiducial_anchor_manager_.PendingFiducialOptimizationTaskCount()));
    }

    PoseGraphBuildResult BuildAndLogPoseGraphForTask(
        const FiducialOptimizationTask& task,
        const std::string& trigger,
        bool debug_task,
        const RawMapDatabase* raw_db_override = nullptr,
        const GlobalPoseStore* pose_store_override = nullptr,
        const CovisibilityDatabase* covisibility_db_override = nullptr,
        bool run_optimization = true,
        bool async_worker = false)
    {
        const RawMapDatabase& raw_source =
            raw_db_override ? *raw_db_override : raw_db_;
        const GlobalPoseStore& pose_source =
            pose_store_override ? *pose_store_override : pose_store_;

        RCLCPP_WARN(
            get_logger(),
            "[F1I-TASK-RX] task_id=%llu type=%s source=%s trigger=%s debug=%s drone_id=%u epoch=%llu kf=%llu fid=%d error_t=%.6f error_rot=%.6f error_yaw=%.6f",
            static_cast<unsigned long long>(task.task_id),
            task.task_type.c_str(),
            task.source.c_str(),
            trigger.c_str(),
            debug_task ? "true" : "false",
            task.drone_id,
            static_cast<unsigned long long>(task.map_epoch),
            static_cast<unsigned long long>(task.keyframe_id.local_kf_id),
            task.fiducial_id,
            task.error_t_m,
            task.error_rot_rad,
            task.error_yaw_rad);

        RCLCPP_WARN(
            get_logger(),
            "[F1I-GRAPH-BUILD-START] task_id=%llu drone_id=%u epoch=%llu target_kf=%llu source=%s",
            static_cast<unsigned long long>(task.task_id),
            task.submap_id.drone_id,
            static_cast<unsigned long long>(task.submap_id.map_epoch),
            static_cast<unsigned long long>(task.keyframe_id.local_kf_id),
            task.source.c_str());

        bool has_partial_checkpoint = false;
        uint64_t partial_parent_task_id = 0;
        uint64_t partial_retry_count = 0;
        {
            std::lock_guard<std::recursive_mutex> lock(live_state_mutex_);
            const auto partial_it =
                f1l_partial_checkpoint_task_by_submap_.find(task.submap_id);
            if (partial_it != f1l_partial_checkpoint_task_by_submap_.end())
            {
                has_partial_checkpoint = true;
                partial_parent_task_id = partial_it->second;
                partial_retry_count =
                    f1l_partial_retry_count_by_submap_[task.submap_id];
            }
        }
        if (has_partial_checkpoint)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1I-GRAPH-REBUILD-FROM-PARTIAL] previous_task_id=%llu new_task_id=%llu drone_id=%u epoch=%llu retry_count=%llu max_retries=%d source=GlobalPoseStore_partial_checkpoint",
                static_cast<unsigned long long>(partial_parent_task_id),
                static_cast<unsigned long long>(task.task_id),
                task.submap_id.drone_id,
                static_cast<unsigned long long>(task.submap_id.map_epoch),
                static_cast<unsigned long long>(partial_retry_count),
                f1l_max_partial_retries_);
        }

        const CovisibilityDatabase* optimization_covisibility_db =
            pose_graph_use_covisibility_edges_
                ? (covisibility_db_override
                       ? covisibility_db_override
                       : &covisibility_db_)
                : nullptr;
        auto build_result = pose_graph_builder_.BuildForFiducialTask(
            task,
            raw_source,
            pose_source,
            optimization_covisibility_db);
        if (build_result.success)
        {
            const size_t covisibility_edges = static_cast<size_t>(std::count_if(
                build_result.problem.edges.begin(),
                build_result.problem.edges.end(),
                [](const auto& edge)
                {
                    return edge.source.rfind("F1M_", 0) == 0;
                }));
            RCLCPP_WARN(
                get_logger(),
                "[F1M-COVIS-QUERY] task_id=%llu enabled_for_fiducial_graph=%s window_kfs=%zu returned_edges=%zu min_weight=%.3f",
                static_cast<unsigned long long>(task.task_id),
                pose_graph_use_covisibility_edges_ ? "true" : "false",
                build_result.problem.vertices.size(),
                covisibility_edges,
                f1m_covisibility_min_weight_);
        }
        if (!build_result.success)
        {
            const auto& coverage = build_result.problem.coverage;
            if (!coverage.reason.empty())
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "[F1I-GRAPH-REJECT] task_id=%llu reason=%s window_keyframes=%llu control_vertices=%llu max_control_span_kfs=%llu max_control_span_m=%.3f uncovered_long_segments=%llu mandatory_vertices_missing=%llu vertex_limit=none edge_length_limit_m=none",
                    static_cast<unsigned long long>(task.task_id),
                    coverage.reason.c_str(),
                    static_cast<unsigned long long>(coverage.window_keyframes),
                    static_cast<unsigned long long>(coverage.control_vertices),
                    static_cast<unsigned long long>(coverage.max_control_span_kfs),
                    coverage.max_control_span_m,
                    static_cast<unsigned long long>(coverage.uncovered_long_segments),
                    static_cast<unsigned long long>(coverage.mandatory_vertices_missing));
            }
            RCLCPP_ERROR(
                get_logger(),
                "[F1I-GRAPH-BUILD-SUMMARY] task_id=%llu success=false reason=%s vertices=0 edges=0 priors=0 propagation=0",
                static_cast<unsigned long long>(task.task_id),
                build_result.reason.c_str());
            return build_result;
        }

        if (has_partial_checkpoint)
        {
            build_result.problem.rebuilt_from_partial_checkpoint = true;
            build_result.problem.partial_parent_task_id = partial_parent_task_id;
            build_result.problem.partial_retry_count = partial_retry_count;
        }

        LogPoseGraphProblem(build_result.problem, build_result.reason);
        if (run_optimization)
        {
            RunAndLogOptimizationDryRun(
                task,
                build_result.problem,
                trigger,
                debug_task,
                raw_source,
                pose_source,
                async_worker);
        }
        return build_result;
    }

    bool SameRawStats(const RawDatabaseStats& a, const RawDatabaseStats& b) const
    {
        return a.journal_entries == b.journal_entries &&
               a.delta_entries == b.delta_entries &&
               a.full_snapshot_entries == b.full_snapshot_entries &&
               a.fiducial_observations == b.fiducial_observations &&
               a.submaps == b.submaps &&
               a.keyframes == b.keyframes &&
               a.mappoints == b.mappoints &&
               a.last_arrival_id == b.last_arrival_id;
    }

    bool SamePoseStoreStats(const orbslam3_multi::GlobalPoseStoreStats& a,
                            const orbslam3_multi::GlobalPoseStoreStats& b) const
    {
        return a.anchored_submaps == b.anchored_submaps &&
               a.keyframe_world_poses == b.keyframe_world_poses &&
               a.optimized_keyframes == b.optimized_keyframes &&
               a.propagated_keyframes == b.propagated_keyframes &&
               a.rebased_keyframes == b.rebased_keyframes &&
               a.submap_corrections == b.submap_corrections &&
               a.hard_fiducial_keyframes == b.hard_fiducial_keyframes &&
               a.accepted_keyframe_anchors == b.accepted_keyframe_anchors &&
               a.active_tail_anchors == b.active_tail_anchors &&
               a.derived_tail_keyframes == b.derived_tail_keyframes;
    }

    void RunAndLogOptimizationDryRun(const FiducialOptimizationTask& task,
                                     const PoseGraphProblem& problem,
                                     const std::string& trigger,
                                     bool debug_task,
                                     const RawMapDatabase& raw_source,
                                     const GlobalPoseStore& pose_source,
                                     bool async_worker)
    {
        if (!f1j_dryrun_enabled_)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1J-SERVER-DRYRUN-DONE] task_id=%llu enabled=false reason=disabled",
                static_cast<unsigned long long>(problem.task_id));
            return;
        }

        const auto raw_stats_before = raw_source.GetDatabaseStats();
        const auto pose_stats_before = pose_source.GetPoseStoreStats();

        RCLCPP_WARN(
            get_logger(),
            "[F1J-SERVER-DRYRUN-REQUEST] task_id=%llu trigger=%s debug=%s",
            static_cast<unsigned long long>(problem.task_id),
            trigger.c_str(),
            debug_task ? "true" : "false");
        RCLCPP_WARN(
            get_logger(),
            "[F1J-OPT-TASK-RX] task_id=%llu type=%s source=%s drone_id=%u epoch=%llu kf=%llu fid=%d before_task_error_t=%.6f before_task_error_yaw=%.6f",
            static_cast<unsigned long long>(task.task_id),
            task.task_type.c_str(),
            task.source.c_str(),
            task.drone_id,
            static_cast<unsigned long long>(task.map_epoch),
            static_cast<unsigned long long>(task.keyframe_id.local_kf_id),
            task.fiducial_id,
            task.error_t_m,
            task.error_yaw_rad);
        RCLCPP_WARN(
            get_logger(),
            "[F1J-OPT-GRAPH-RX] task_id=%llu vertices=%llu variable=%llu fixed=%llu hard_fiducial=%llu edges=%llu priors=%llu propagation=%llu",
            static_cast<unsigned long long>(problem.task_id),
            static_cast<unsigned long long>(problem.summary.vertices),
            static_cast<unsigned long long>(problem.summary.variable_vertices),
            static_cast<unsigned long long>(problem.summary.fixed_vertices),
            static_cast<unsigned long long>(problem.summary.hard_fiducial_vertices),
            static_cast<unsigned long long>(problem.summary.edges),
            static_cast<unsigned long long>(problem.summary.priors),
            static_cast<unsigned long long>(problem.summary.propagation_entries));

        DumpPoseGraphProblemForOffline(problem);

        const OptimizationDryRunResult result =
            optimization_manager_.RunDryRun(problem, raw_source, pose_source);

        if (!result.precheck_ok)
        {
            RCLCPP_ERROR(
                get_logger(),
                "[F1J-OPT-PRECHECK] task_id=%llu ok=false reason=%s vertices=%llu variable=%llu fixed=%llu edges=%llu priors=%llu propagation=%llu",
                static_cast<unsigned long long>(problem.task_id),
                result.reason.c_str(),
                static_cast<unsigned long long>(problem.summary.vertices),
                static_cast<unsigned long long>(problem.summary.variable_vertices),
                static_cast<unsigned long long>(problem.summary.fixed_vertices),
                static_cast<unsigned long long>(problem.summary.edges),
                static_cast<unsigned long long>(problem.summary.priors),
                static_cast<unsigned long long>(problem.summary.propagation_entries));
            RCLCPP_ERROR(
                get_logger(),
                "[F1J-OPT-PRECHECK-FAIL] task_id=%llu reason=%s",
                static_cast<unsigned long long>(problem.task_id),
                result.reason.c_str());
            return;
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1J-OPT-PRECHECK] task_id=%llu ok=true vertices=%llu variable=%llu fixed=%llu edges=%llu priors=%llu propagation=%llu",
            static_cast<unsigned long long>(problem.task_id),
            static_cast<unsigned long long>(problem.summary.vertices),
            static_cast<unsigned long long>(problem.summary.variable_vertices),
            static_cast<unsigned long long>(problem.summary.fixed_vertices),
            static_cast<unsigned long long>(problem.summary.edges),
            static_cast<unsigned long long>(problem.summary.priors),
            static_cast<unsigned long long>(problem.summary.propagation_entries));
        RCLCPP_WARN(
            get_logger(),
            "[F1J-OPT-POSE-COPY] task_id=%llu copied=%llu",
            static_cast<unsigned long long>(problem.task_id),
            static_cast<unsigned long long>(result.copied_poses));
        RCLCPP_WARN(
            get_logger(),
            "[F1J-OPT-CONSTRAINTS] task_id=%llu edges=%llu priors=%llu task_type=%s",
            static_cast<unsigned long long>(problem.task_id),
            static_cast<unsigned long long>(problem.summary.edges),
            static_cast<unsigned long long>(problem.summary.priors),
            problem.task_type.c_str());
        RCLCPP_WARN(
            get_logger(),
            "[F1J-OPT-DRYRUN-START] task_id=%llu solver=fiducial_window_progressive_deformation",
            static_cast<unsigned long long>(problem.task_id));
        RCLCPP_WARN(
            get_logger(),
            "[F1J-OPT-SOLVER-SUMMARY] task_id=%llu success=%s iterations=%llu initial_cost=%.6f final_cost=%.6f moved_kfs=%llu fixed_kfs=%llu max_delta_t=%.6f mean_delta_t=%.6f max_delta_yaw=%.6f hard_fixed_moved=%s",
            static_cast<unsigned long long>(problem.task_id),
            result.success ? "true" : "false",
            static_cast<unsigned long long>(result.solver_iterations),
            result.initial_cost,
            result.final_cost,
            static_cast<unsigned long long>(result.moved_kfs),
            static_cast<unsigned long long>(result.fixed_kfs),
            result.max_delta_t,
            result.mean_delta_t,
            result.max_delta_yaw,
            result.hard_fixed_moved ? "true" : "false");
        RCLCPP_WARN(
            get_logger(),
            "[F1J-OPT-ANCHOR-PRESERVATION] task_id=%llu ok=%s required=%s graph_satisfied=%s previous_fiducial_fixed_count=%llu previous_fiducial_neighborhood_fixed_count=%llu fixed_kfs=%llu hard_fixed_moved=%s max_previous_fiducial_delta_t=%.9f max_previous_fiducial_delta_yaw=%.9f max_previous_fiducial_neighborhood_delta_t=%.9f max_previous_fiducial_neighborhood_delta_yaw=%.9f",
            static_cast<unsigned long long>(problem.task_id),
            result.anchor_preservation_ok ? "true" : "false",
            result.anchor_preservation_required ? "true" : "false",
            result.anchor_preservation_graph_satisfied ? "true" : "false",
            static_cast<unsigned long long>(result.previous_fiducial_fixed_count),
            static_cast<unsigned long long>(
                result.previous_fiducial_neighborhood_fixed_count),
            static_cast<unsigned long long>(result.fixed_kfs),
            result.hard_fixed_moved ? "true" : "false",
            result.max_previous_fiducial_delta_t,
            result.max_previous_fiducial_delta_yaw,
            result.max_previous_fiducial_neighborhood_delta_t,
            result.max_previous_fiducial_neighborhood_delta_yaw);
        RCLCPP_WARN(
            get_logger(),
            "[F1J-OPT-PROPAGATION-DRYRUN] task_id=%llu affected_non_variable=%llu proposed=%llu max_propagated_delta_t=%.6f mean_propagated_delta_t=%.6f mode=path_segment_interpolated",
            static_cast<unsigned long long>(problem.task_id),
            static_cast<unsigned long long>(problem.summary.affected_non_variable_keyframes),
            static_cast<unsigned long long>(result.propagated_kfs),
            result.max_propagated_delta_t,
            result.mean_propagated_delta_t);
        RCLCPP_WARN(
            get_logger(),
            "[F1J-OPT-DRYRUN-RESULT] task_id=%llu task=%s success=%s before_t=%.6f after_t=%.6f before_yaw=%.6f after_yaw=%.6f improvement_ratio=%.6f initial_cost=%.6f final_cost=%.6f",
            static_cast<unsigned long long>(problem.task_id),
            result.task_type.c_str(),
            result.success ? "true" : "false",
            result.before_error_t,
            result.after_error_t,
            result.before_error_yaw,
            result.after_error_yaw,
            result.improvement_ratio,
            result.initial_cost,
            result.final_cost);
        RCLCPP_WARN(
            get_logger(),
            "[F1J-OPT-DRYRUN-DECISION] task_id=%llu useful=%s partial_candidate=%s reason=%s partial_reason=%s before_t=%.6f after_t=%.6f improvement_ratio=%.6f max_delta_t=%.6f max_delta_yaw=%.6f",
            static_cast<unsigned long long>(problem.task_id),
            result.useful ? "true" : "false",
            result.partial_candidate ? "true" : "false",
            result.decision_reason.c_str(),
            result.partial_reason.empty() ? "none" : result.partial_reason.c_str(),
            result.before_error_t,
            result.after_error_t,
            result.improvement_ratio,
            result.max_delta_t,
            result.max_delta_yaw);

        const auto raw_stats_after = raw_source.GetDatabaseStats();
        const auto pose_stats_after = pose_source.GetPoseStoreStats();
        const bool raw_unchanged = SameRawStats(raw_stats_before, raw_stats_after);
        const bool pose_unchanged =
            SamePoseStoreStats(pose_stats_before, pose_stats_after);
        RCLCPP_WARN(
            get_logger(),
            "[F1J-OPT-NO-STATE-MUTATION] task_id=%llu ok=%s raw_unchanged=%s pose_store_unchanged=%s",
            static_cast<unsigned long long>(problem.task_id),
            (raw_unchanged && pose_unchanged) ? "true" : "false",
            raw_unchanged ? "true" : "false",
            pose_unchanged ? "true" : "false");

        if (result.success && f1j_export_debug_plot_)
        {
            const std::string slash =
                (!f1j_debug_output_dir_.empty() &&
                 f1j_debug_output_dir_.back() == '/') ? "" : "/";
            const std::string path =
                f1j_debug_output_dir_ + slash +
                "f1j_dryrun_task_" + std::to_string(problem.task_id) + ".svg";
            const auto export_result =
                optimization_debug_exporter_.ExportDryRun2DPlot(problem, result, path);
            RCLCPP_WARN(
                get_logger(),
                "[F1J-OPT-DEBUG-EXPORT] task_id=%llu enabled=true success=%s path=%s reason=%s",
                static_cast<unsigned long long>(problem.task_id),
                export_result.success ? "true" : "false",
                export_result.path.c_str(),
                export_result.reason.c_str());
        }
        else
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1J-OPT-DEBUG-EXPORT] task_id=%llu enabled=false reason=disabled_by_default",
                static_cast<unsigned long long>(problem.task_id));
        }

        // 1J con nombre legacy `F1L`: exporta una animacion HTML del dry-run
        // con mapa antes, GT externo y propuesta optimizada. No abre ventanas
        // desde ROS ni realimenta el servidor; 1L puede usarla despues como
        // evidencia cruzada.
        std::map<RawKeyFrameId, DebugGtKeyFramePose> gt_snapshot;
        {
            std::lock_guard<std::recursive_mutex> lock(live_state_mutex_);
            gt_snapshot = f1l_gt_keyframe_store_;
        }
        ExportF1LDebugAnimation(
            problem,
            result,
            nullptr,
            &pose_source,
            &gt_snapshot);

        uint64_t partial_retry_count = 0;
        bool has_partial_retry = false;
        {
            std::lock_guard<std::recursive_mutex> lock(live_state_mutex_);
            const auto partial_retry_it =
                f1l_partial_retry_count_by_submap_.find(problem.submap_id);
            has_partial_retry =
                partial_retry_it != f1l_partial_retry_count_by_submap_.end();
            if (has_partial_retry)
            {
                partial_retry_count = partial_retry_it->second;
            }
        }
        const bool partial_retry_limit_ok =
            !has_partial_retry ||
            partial_retry_count < static_cast<uint64_t>(f1l_max_partial_retries_);
        const bool can_apply_partial =
            f1k_apply_enabled_ &&
            f1l_validation_enabled_ &&
            f1l_partial_apply_enabled_ &&
            partial_retry_limit_ok &&
            result.partial_candidate &&
            !result.useful;
        if (result.partial_candidate && !result.useful &&
            !partial_retry_limit_ok)
        {
            RCLCPP_ERROR(
                get_logger(),
                "[F1L-PARTIAL-RETRY-LIMIT] task_id=%llu drone_id=%u epoch=%llu retry_count=%llu max_retries=%d applied=false",
                static_cast<unsigned long long>(problem.task_id),
                problem.submap_id.drone_id,
                static_cast<unsigned long long>(problem.submap_id.map_epoch),
                static_cast<unsigned long long>(partial_retry_count),
                f1l_max_partial_retries_);
        }
        if (result.partial_candidate && !result.useful && !can_apply_partial)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1K-OPT-PARTIAL-PENDING] task_id=%llu applied=false useful=false partial_candidate=true reason=%s required_next=F1L_DECISION",
                static_cast<unsigned long long>(problem.task_id),
                result.partial_reason.empty()
                    ? result.decision_reason.c_str()
                    : result.partial_reason.c_str());
        }
        if (f1k_apply_enabled_ && (result.useful || can_apply_partial))
        {
            if (async_worker)
            {
                std::lock_guard<std::recursive_mutex> lock(live_state_mutex_);
                ApplyAndLogOptimizationResult(task, result, problem, trigger);
            }
            else
            {
                ApplyAndLogOptimizationResult(task, result, problem, trigger);
            }
        }
        else if (!f1k_apply_enabled_ && result.useful)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1K-OPT-APPLY-SKIP] task_id=%llu reason=disabled useful=true",
                static_cast<unsigned long long>(problem.task_id));
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1J-SERVER-DRYRUN-DONE] task_id=%llu success=%s useful=%s partial_candidate=%s no_apply=%s",
            static_cast<unsigned long long>(problem.task_id),
            result.success ? "true" : "false",
            result.useful ? "true" : "false",
            result.partial_candidate ? "true" : "false",
            (f1k_apply_enabled_ && (result.useful || can_apply_partial)) ? "false" : "true");
    }

    std::vector<RawKeyFrameId> CollectApplyAffectedKeyFrames(
        const OptimizationDryRunResult& dry_run,
        const PoseGraphProblem& problem,
        const RawMapDatabase& raw_db) const
    {
        // 1K: el backup cubre variables y propagados antes de escribir en
        // GlobalPoseStore. 1L solo observara el resultado; si hay rechazo, la
        // restauracion sigue siendo una proteccion de apply, no logica nueva.
        // Tambien cubre los KFs de cola ya existentes: los que llegaron durante
        // el calculo de la optimizacion deben moverse con el ultimo KF aplicado.
        std::vector<RawKeyFrameId> affected;
        affected.reserve(
            dry_run.proposed_vertex_poses.size() +
            dry_run.proposed_propagated_poses.size());
        std::map<RawSubmapId, uint64_t> newest_applied_by_submap;
        auto note_tail_reference =
            [&](const orbslam3_multi::OptimizationPoseProposal& proposal)
        {
            const RawSubmapId submap{
                proposal.keyframe_id.drone_id,
                proposal.keyframe_id.map_epoch};
            auto it = newest_applied_by_submap.find(submap);
            if (it == newest_applied_by_submap.end() ||
                proposal.keyframe_id.local_kf_id > it->second)
            {
                newest_applied_by_submap[submap] = proposal.keyframe_id.local_kf_id;
            }
        };
        for (const auto& proposal : dry_run.proposed_vertex_poses)
        {
            affected.push_back(proposal.keyframe_id);
            if (proposal.variable_vertex &&
                !proposal.fixed_vertex &&
                proposal.delta_t_m > 1e-9)
            {
                note_tail_reference(proposal);
            }
        }
        for (const auto& proposal : dry_run.proposed_propagated_poses)
        {
            affected.push_back(proposal.keyframe_id);
            if (proposal.delta_t_m > 1e-9)
            {
                note_tail_reference(proposal);
            }
        }
        for (const auto& [submap, newest_local_kf] : newest_applied_by_submap)
        {
            for (const auto& keyframe_id : raw_db.GetKeyFrameIdsForSubmap(submap))
            {
                if (keyframe_id.local_kf_id > newest_local_kf)
                {
                    affected.push_back(keyframe_id);
                }
            }
        }
        uint64_t oldest_window_kf = problem.target_keyframe_id.local_kf_id;
        uint64_t newest_window_kf = problem.target_keyframe_id.local_kf_id;
        auto note_window_keyframe = [&](const RawKeyFrameId& keyframe_id)
        {
            if (keyframe_id.drone_id != problem.submap_id.drone_id ||
                keyframe_id.map_epoch != problem.submap_id.map_epoch)
            {
                return;
            }
            oldest_window_kf =
                std::min(oldest_window_kf, keyframe_id.local_kf_id);
            newest_window_kf =
                std::max(newest_window_kf, keyframe_id.local_kf_id);
        };
        for (const auto& vertex : problem.vertices)
        {
            note_window_keyframe(vertex.keyframe_id);
        }
        for (const auto& keyframe_id : problem.affected_non_variable_keyframes)
        {
            note_window_keyframe(keyframe_id);
        }
        for (const auto& entry : problem.propagation_plan)
        {
            note_window_keyframe(entry.affected_keyframe_id);
        }
        for (const auto& keyframe_id :
             raw_db.GetKeyFrameIdsForSubmap(problem.submap_id))
        {
            if (keyframe_id.local_kf_id >= oldest_window_kf &&
                keyframe_id.local_kf_id <= newest_window_kf)
            {
                affected.push_back(keyframe_id);
            }
        }
        std::sort(affected.begin(), affected.end());
        affected.erase(std::unique(affected.begin(), affected.end()), affected.end());
        return affected;
    }

    std::vector<PendingTailFiducialConstraint>
    CollectPendingTailFiducialConstraints(
        const FiducialOptimizationTask& task,
        const PoseGraphProblem& problem) const
    {
        // 1K: durante un solver largo pueden llegar mas KFs de la misma visita.
        // Sus observaciones fiduciales ya persistidas son controles absolutos
        // validos para cerrar el tramo pendiente al hacer commit.
        std::map<RawKeyFrameId, PendingTailFiducialConstraint> by_keyframe;
        for (const auto& recorded :
             raw_db_.GetFiducialObservationJournalCopy())
        {
            if (recorded.arrival_id <= task.created_arrival_id ||
                recorded.drone_id != task.drone_id ||
                recorded.map_epoch != task.map_epoch ||
                recorded.fiducial_id != task.fiducial_id ||
                recorded.local_keyframe_id <=
                    problem.target_keyframe_id.local_kf_id)
            {
                continue;
            }

            const RawKeyFrameId keyframe_id{
                recorded.drone_id,
                recorded.map_epoch,
                recorded.local_keyframe_id};
            if (!raw_db_.HasKeyFrame(keyframe_id))
            {
                continue;
            }

            const FiducialObservation observation =
                FromRecordedObservation(
                    recorded,
                    "SERVER_PENDING_TAIL_FIDUCIAL");
            PendingTailFiducialConstraint constraint;
            constraint.keyframe_id = keyframe_id;
            constraint.target_world_T_kf =
                pose_store_.TransformBodyPoseToCameraPose(
                    observation.world_T_body_fiducial);
            constraint.arrival_id = recorded.arrival_id;
            constraint.fiducial_id = recorded.fiducial_id;
            constraint.association_dt_sec = recorded.association_dt_sec;
            constraint.source = observation.source;
            if (constraint.target_world_T_kf.allFinite())
            {
                by_keyframe[keyframe_id] = constraint;
            }
        }

        std::vector<PendingTailFiducialConstraint> constraints;
        constraints.reserve(by_keyframe.size());
        for (const auto& [keyframe_id, constraint] : by_keyframe)
        {
            constraints.push_back(constraint);
            RCLCPP_WARN(
                get_logger(),
                "[F1K-PENDING-TAIL-FIDUCIAL-CONTROL] task_id=%llu drone_id=%u epoch=%llu kf=%llu fid=%d arrival_id=%llu association_dt=%.6f source=%s",
                static_cast<unsigned long long>(task.task_id),
                keyframe_id.drone_id,
                static_cast<unsigned long long>(keyframe_id.map_epoch),
                static_cast<unsigned long long>(keyframe_id.local_kf_id),
                constraint.fiducial_id,
                static_cast<unsigned long long>(constraint.arrival_id),
                constraint.association_dt_sec,
                constraint.source.c_str());
        }
        RCLCPP_WARN(
            get_logger(),
            "[F1K-PENDING-TAIL-FIDUCIAL-SUMMARY] task_id=%llu graph_target_kf=%llu created_arrival_id=%llu controls=%llu",
            static_cast<unsigned long long>(task.task_id),
            static_cast<unsigned long long>(
                problem.target_keyframe_id.local_kf_id),
            static_cast<unsigned long long>(task.created_arrival_id),
            static_cast<unsigned long long>(constraints.size()));
        return constraints;
    }

    bool ShouldForceF1LReject(uint64_t task_id)
    {
        if (!f1l_debug_force_reject_once_ || f1l_debug_force_reject_consumed_)
        {
            return false;
        }
        if (f1l_debug_force_reject_task_id_ >= 0 &&
            task_id != static_cast<uint64_t>(f1l_debug_force_reject_task_id_))
        {
            return false;
        }
        f1l_debug_force_reject_consumed_ = true;
        return true;
    }

    void ApplyAndLogOptimizationResult(const FiducialOptimizationTask& task,
                                       const OptimizationDryRunResult& dry_run,
                                       const PoseGraphProblem& problem,
                                       const std::string& trigger)
    {
        const bool anchor_preservation_required =
            dry_run.anchor_preservation_required ||
            problem.anchor_preservation.required;
        const bool anchor_preservation_graph_satisfied =
            dry_run.anchor_preservation_graph_satisfied &&
            (!problem.anchor_preservation.required ||
             problem.anchor_preservation.satisfied);
        const uint64_t previous_fiducial_fixed_count =
            std::max(
                dry_run.previous_fiducial_fixed_count,
                problem.anchor_preservation.previous_fiducial_fixed_count);
        std::string anchor_precheck_reason = "not_required";
        bool anchor_precheck_ok = true;
        if (anchor_preservation_required)
        {
            anchor_precheck_reason = "ok";
            anchor_precheck_ok =
                anchor_preservation_graph_satisfied &&
                dry_run.anchor_preservation_ok &&
                previous_fiducial_fixed_count > 0 &&
                dry_run.fixed_kfs > 0 &&
                !dry_run.hard_fixed_moved;
            if (!anchor_preservation_graph_satisfied ||
                previous_fiducial_fixed_count == 0 ||
                dry_run.fixed_kfs == 0)
            {
                anchor_precheck_reason = "missing_previous_fiducial_anchor";
            }
            else if (!dry_run.anchor_preservation_ok || dry_run.hard_fixed_moved)
            {
                anchor_precheck_reason = "anchor_preservation_failed";
            }
        }
        // 1K: antes de crear backup o escribir en GlobalPoseStore,
        // exigimos que la evidencia producida por 1I/1J demuestre que el
        // fiducial previo del submapa esta fijo y no se movera con el apply.
        RCLCPP_WARN(
            get_logger(),
            "[F1K-OPT-APPLY-ANCHOR-PRESERVATION-PRECHECK] task_id=%llu ok=%s required=%s graph_satisfied=%s dryrun_ok=%s previous_fiducial_fixed_count=%llu fixed_kfs=%llu hard_fixed_moved=%s max_previous_fiducial_delta_t=%.9f max_previous_fiducial_delta_yaw=%.9f reason=%s",
            static_cast<unsigned long long>(dry_run.task_id),
            anchor_precheck_ok ? "true" : "false",
            anchor_preservation_required ? "true" : "false",
            anchor_preservation_graph_satisfied ? "true" : "false",
            dry_run.anchor_preservation_ok ? "true" : "false",
            static_cast<unsigned long long>(previous_fiducial_fixed_count),
            static_cast<unsigned long long>(dry_run.fixed_kfs),
            dry_run.hard_fixed_moved ? "true" : "false",
            dry_run.max_previous_fiducial_delta_t,
            dry_run.max_previous_fiducial_delta_yaw,
            anchor_precheck_reason.c_str());
        if (!anchor_precheck_ok)
        {
            RCLCPP_ERROR(
                get_logger(),
                "[F1K-OPT-APPLY-PRECHECK-FAIL] task_id=%llu reason=%s",
                static_cast<unsigned long long>(dry_run.task_id),
                anchor_precheck_reason.c_str());
            return;
        }

        // El apply y 1L trabajan sobre una copia privada. El store live solo se
        // sustituye despues de una decision aceptada, bajo el mutex de estado.
        GlobalPoseStore candidate_pose_store = pose_store_;
        FusedLandmarkManager candidate_fused_landmarks =
            fused_landmark_manager_;
        const auto raw_stats_before = raw_db_.GetDatabaseStats();
        const auto pose_stats_before = pose_store_.GetPoseStoreStats();
        const auto pending_tail_constraints =
            CollectPendingTailFiducialConstraints(task, problem);
        const GlobalMapBuildResult map_before =
            global_map_builder_.Build(
                raw_db_,
                pose_store_,
                score_manager_,
                &candidate_fused_landmarks,
                static_cast<float>(global_map_min_score_to_publish_));
        const auto affected_keyframes =
            CollectApplyAffectedKeyFrames(dry_run, problem, raw_db_);
        const DebugGtWindowStats gt_before =
            LogGtWindowErrors(dry_run.task_id, affected_keyframes, false);
        const std::vector<RawKeyFrameId> gt_collateral_keyframes =
            CollectGtCollateralKeyFrames(problem);
        const auto backup =
            candidate_pose_store.CreateApplyBackup(
                dry_run.task_id, affected_keyframes);
        RCLCPP_WARN(
            get_logger(),
            "[F1L-POSESTORE-BACKUP-CREATED] task_id=%llu ok=%s affected_kfs=%llu affected_submaps=%llu reason=%s",
            static_cast<unsigned long long>(dry_run.task_id),
            backup.success ? "true" : "false",
            static_cast<unsigned long long>(backup.affected_keyframes),
            static_cast<unsigned long long>(backup.affected_submaps),
            backup.reason.c_str());
        if (!backup.success)
        {
            RCLCPP_ERROR(
                get_logger(),
                "[F1K-OPT-APPLY-PRECHECK-FAIL] task_id=%llu reason=f1l_backup_failed_%s",
                static_cast<unsigned long long>(dry_run.task_id),
                backup.reason.c_str());
            return;
        }
        const bool allow_partial_candidate =
            f1l_validation_enabled_ && f1l_partial_apply_enabled_;
        const OptimizationApplyResult apply =
            optimization_manager_.ApplyCandidateResult(
                dry_run,
                problem,
                raw_db_,
                candidate_pose_store,
                allow_partial_candidate,
                pending_tail_constraints);
        const auto raw_stats_after = raw_db_.GetDatabaseStats();
        const auto pose_stats_after = candidate_pose_store.GetPoseStoreStats();
        const bool raw_unchanged = SameRawStats(raw_stats_before, raw_stats_after);
        const DebugGtWindowStats gt_after =
            LogGtWindowErrors(
                dry_run.task_id,
                affected_keyframes,
                true,
                &candidate_pose_store);
        LogGtWindowComparison(dry_run.task_id, gt_before, gt_after);
        LogGtCollateralCheck(
            dry_run.task_id,
            gt_before,
            gt_after,
            gt_collateral_keyframes);

        RCLCPP_WARN(
            get_logger(),
            "[F1K-OPT-APPLY-PRECHECK] task_id=%llu ok=%s useful=%s partial_candidate=%s moved_kfs=%llu propagated=%llu fixed=%llu reason=%s trigger=%s",
            static_cast<unsigned long long>(dry_run.task_id),
            apply.precheck_ok ? "true" : "false",
            dry_run.useful ? "true" : "false",
            dry_run.partial_candidate ? "true" : "false",
            static_cast<unsigned long long>(dry_run.moved_kfs),
            static_cast<unsigned long long>(dry_run.propagated_kfs),
            static_cast<unsigned long long>(dry_run.fixed_kfs),
            apply.reason.c_str(),
            trigger.c_str());
        if (!apply.precheck_ok)
        {
            RCLCPP_ERROR(
                get_logger(),
                "[F1K-OPT-APPLY-PRECHECK-FAIL] task_id=%llu reason=%s",
                static_cast<unsigned long long>(dry_run.task_id),
                apply.reason.c_str());
        }

        for (const auto& record : apply.optimized_records)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1K-OPT-APPLY-KF] task_id=%llu drone_id=%u epoch=%llu kf=%llu source=%s applied=%s delta_t=%.6f delta_yaw=%.6f reason=%s",
                static_cast<unsigned long long>(dry_run.task_id),
                record.keyframe_id.drone_id,
                static_cast<unsigned long long>(record.keyframe_id.map_epoch),
                static_cast<unsigned long long>(record.keyframe_id.local_kf_id),
                record.source.c_str(),
                record.applied ? "true" : "false",
                record.delta_t_m,
                record.delta_yaw_rad,
                record.reason.c_str());
            if (record.applied)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1K-POSESTORE-OPTIMIZED-POSE-SET] task_id=%llu drone_id=%u epoch=%llu kf=%llu source=%s delta_t=%.6f delta_yaw=%.6f",
                    static_cast<unsigned long long>(dry_run.task_id),
                    record.keyframe_id.drone_id,
                    static_cast<unsigned long long>(record.keyframe_id.map_epoch),
                    static_cast<unsigned long long>(record.keyframe_id.local_kf_id),
                    record.source.c_str(),
                    record.delta_t_m,
                    record.delta_yaw_rad);
                RCLCPP_WARN(
                    get_logger(),
                    "[F1K-POSESTORE-CORRECTION-SET] task_id=%llu drone_id=%u epoch=%llu kf=%llu correction_t=%.6f correction_yaw=%.6f source=%s",
                    static_cast<unsigned long long>(dry_run.task_id),
                    record.keyframe_id.drone_id,
                    static_cast<unsigned long long>(record.keyframe_id.map_epoch),
                    static_cast<unsigned long long>(record.keyframe_id.local_kf_id),
                    record.correction_t_m,
                    record.correction_yaw_rad,
                    record.source.c_str());
            }
        }

        for (const auto& record : apply.propagated_records)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1K-OPT-APPLY-PROPAGATED-KF] task_id=%llu drone_id=%u epoch=%llu kf=%llu source=%s applied=%s delta_t=%.6f delta_yaw=%.6f reason=%s",
                static_cast<unsigned long long>(dry_run.task_id),
                record.keyframe_id.drone_id,
                static_cast<unsigned long long>(record.keyframe_id.map_epoch),
                static_cast<unsigned long long>(record.keyframe_id.local_kf_id),
                record.source.c_str(),
                record.applied ? "true" : "false",
                record.delta_t_m,
                record.delta_yaw_rad,
                record.reason.c_str());
            if (record.applied)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1K-POSESTORE-PROPAGATED-POSE-SET] task_id=%llu drone_id=%u epoch=%llu kf=%llu source=%s delta_t=%.6f delta_yaw=%.6f",
                    static_cast<unsigned long long>(dry_run.task_id),
                    record.keyframe_id.drone_id,
                    static_cast<unsigned long long>(record.keyframe_id.map_epoch),
                    static_cast<unsigned long long>(record.keyframe_id.local_kf_id),
                    record.source.c_str(),
                    record.delta_t_m,
                    record.delta_yaw_rad);
                RCLCPP_WARN(
                    get_logger(),
                    "[F1K-POSESTORE-CORRECTION-SET] task_id=%llu drone_id=%u epoch=%llu kf=%llu correction_t=%.6f correction_yaw=%.6f source=%s",
                    static_cast<unsigned long long>(dry_run.task_id),
                    record.keyframe_id.drone_id,
                    static_cast<unsigned long long>(record.keyframe_id.map_epoch),
                    static_cast<unsigned long long>(record.keyframe_id.local_kf_id),
                    record.correction_t_m,
                    record.correction_yaw_rad,
                    record.source.c_str());
            }
        }

        if (apply.skipped_propagated_kfs > 0)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1L-PARTIAL-PROPAGATION-SKIP] task_id=%llu skipped_propagated_kfs=%llu reason=partial_checkpoint_requires_graph_rebuild_before_broad_propagation",
                static_cast<unsigned long long>(dry_run.task_id),
                static_cast<unsigned long long>(apply.skipped_propagated_kfs));
        }

        if (apply.active_tail_anchor_set)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1K-ACTIVE-TAIL-ANCHOR] task_id=%llu submap=(%u,%llu) ref_kf=%llu active_tail_anchor_set=true",
                static_cast<unsigned long long>(dry_run.task_id),
                apply.active_tail_anchor_submap.drone_id,
                static_cast<unsigned long long>(apply.active_tail_anchor_submap.map_epoch),
                static_cast<unsigned long long>(apply.active_tail_anchor_keyframe.local_kf_id));
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1K-PENDING-TAIL-REFINE] task_id=%llu controls=%llu refined_kfs=%llu derived_kfs=%llu active_ref_kf=%llu",
            static_cast<unsigned long long>(dry_run.task_id),
            static_cast<unsigned long long>(
                apply.pending_tail_fiducial_controls),
            static_cast<unsigned long long>(
                apply.pending_tail_refined_kfs),
            static_cast<unsigned long long>(
                apply.pending_tail_derived_kfs),
            static_cast<unsigned long long>(
                apply.active_tail_anchor_keyframe.local_kf_id));

        RCLCPP_WARN(
            get_logger(),
            "[F1K-LATE-WINDOW-REFINE] task_id=%llu detected=%llu refined=%llu skipped=%llu",
            static_cast<unsigned long long>(dry_run.task_id),
            static_cast<unsigned long long>(
                apply.late_window_detected_kfs),
            static_cast<unsigned long long>(
                apply.late_window_refined_kfs),
            static_cast<unsigned long long>(
                apply.late_window_skipped_kfs));

        RCLCPP_WARN(
            get_logger(),
            "[F1K-RAWDB-NOT-MODIFIED] task_id=%llu ok=%s before_journal=%llu after_journal=%llu before_kfs=%llu after_kfs=%llu before_mps=%llu after_mps=%llu",
            static_cast<unsigned long long>(dry_run.task_id),
            raw_unchanged ? "true" : "false",
            static_cast<unsigned long long>(raw_stats_before.journal_entries),
            static_cast<unsigned long long>(raw_stats_after.journal_entries),
            static_cast<unsigned long long>(raw_stats_before.keyframes),
            static_cast<unsigned long long>(raw_stats_after.keyframes),
            static_cast<unsigned long long>(raw_stats_before.mappoints),
            static_cast<unsigned long long>(raw_stats_after.mappoints));
        RCLCPP_WARN(
            get_logger(),
            "[F1K-OPT-APPLY-SUMMARY] task_id=%llu applied=%s reason=%s optimized_kfs=%llu propagated_kfs=%llu tail_anchor_rebased_kfs=%llu skipped_propagated_kfs=%llu pending_rebased=%llu pending_tail_controls=%llu pending_tail_refined=%llu pending_tail_derived=%llu late_window_detected=%llu late_window_refined=%llu late_window_skipped=%llu fixed_kfs=%llu hard_fixed_moved=%s max_delta_t=%.6f mean_delta_t=%.6f max_delta_yaw=%.6f active_tail_anchor_set=%s raw_db_modified=%s pose_store_before_optimized=%llu pose_store_after_optimized=%llu pose_store_after_propagated=%llu",
            static_cast<unsigned long long>(dry_run.task_id),
            apply.applied ? "true" : "false",
            apply.reason.c_str(),
            static_cast<unsigned long long>(apply.optimized_kfs),
            static_cast<unsigned long long>(apply.propagated_kfs),
            static_cast<unsigned long long>(apply.tail_rebased_kfs),
            static_cast<unsigned long long>(apply.skipped_propagated_kfs),
            static_cast<unsigned long long>(apply.pending_rebased_kfs),
            static_cast<unsigned long long>(
                apply.pending_tail_fiducial_controls),
            static_cast<unsigned long long>(
                apply.pending_tail_refined_kfs),
            static_cast<unsigned long long>(
                apply.pending_tail_derived_kfs),
            static_cast<unsigned long long>(
                apply.late_window_detected_kfs),
            static_cast<unsigned long long>(
                apply.late_window_refined_kfs),
            static_cast<unsigned long long>(
                apply.late_window_skipped_kfs),
            static_cast<unsigned long long>(apply.fixed_kfs),
            apply.hard_fixed_moved ? "true" : "false",
            apply.max_delta_t,
            apply.mean_delta_t,
            apply.max_delta_yaw,
            apply.active_tail_anchor_set ? "true" : "false",
            raw_unchanged ? "false" : "true",
            static_cast<unsigned long long>(pose_stats_before.optimized_keyframes),
            static_cast<unsigned long long>(pose_stats_after.optimized_keyframes),
            static_cast<unsigned long long>(pose_stats_after.propagated_keyframes));
        RCLCPP_WARN(
            get_logger(),
            "[F1K-OPT-APPLY-RESULT] task_id=%llu applied=%s reason=%s raw_db_modified=%s global_pose_store_modified=%s",
            static_cast<unsigned long long>(dry_run.task_id),
            apply.applied ? "true" : "false",
            apply.reason.c_str(),
            raw_unchanged ? "false" : "true",
            apply.global_pose_store_modified ? "true" : "false");

        if (!apply.applied)
        {
            const auto rollback =
                candidate_pose_store.RestoreApplyBackup(dry_run.task_id);
            RCLCPP_WARN(
                get_logger(),
                "[F1K-APPLY-FAIL-ROLLBACK] task_id=%llu ok=%s reason=%s",
                static_cast<unsigned long long>(dry_run.task_id),
                rollback.success ? "true" : "false",
                rollback.reason.c_str());
            CloseFiducialOptimizationTask(
                dry_run.task_id,
                "apply_not_applied:" + apply.reason);
            return;
        }

        ExportF1LDebugAnimation(problem, dry_run, &apply);

        RCLCPP_WARN(
            get_logger(),
            "[F1K-GLOBALMAP-REBUILD-AFTER-APPLY] task_id=%llu reason=apply_success_pending_f1l_validation",
            static_cast<unsigned long long>(dry_run.task_id));

        if (!f1l_validation_enabled_)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1L-POST-APPLY-START] task_id=%llu enabled=false reason=validation_disabled",
                static_cast<unsigned long long>(dry_run.task_id));
            std::string fiducial_accept_reason;
            const bool fiducial_accepted =
                fiducial_anchor_manager_.AcceptOptimizedFiducialTask(
                    dry_run.task_id,
                    candidate_pose_store,
                    "SERVER_OPTIMIZATION_FIDUCIAL_ACCEPT",
                    &fiducial_accept_reason);
            RCLCPP_WARN(
                get_logger(),
                "[F1K-FIDUCIAL-TARGET-ACCEPTED] task_id=%llu ok=%s reason=%s validation_enabled=false",
                static_cast<unsigned long long>(dry_run.task_id),
                fiducial_accepted ? "true" : "false",
                fiducial_accept_reason.c_str());
            const bool confirmed =
                candidate_pose_store.ConfirmApply(dry_run.task_id);
            if (confirmed)
            {
                pose_store_ = std::move(candidate_pose_store);
                TryTraceFlow(
                    "optimizer_pose_db",
                    "commit",
                    "fiducial_optimization",
                    dry_run.task_id,
                    apply.optimized_kfs + apply.propagated_kfs);
            }
            RCLCPP_WARN(
                get_logger(),
                "[F1K-POSE-COMMIT] task_id=%llu decision=ACCEPT validation_enabled=false committed=%s mode=atomic_candidate_swap",
                static_cast<unsigned long long>(dry_run.task_id),
                confirmed ? "true" : "false");
            CloseFiducialOptimizationTask(
                dry_run.task_id,
                "apply_accept_validation_disabled");
            RequestOptimizationStatePublication(
                dry_run.task_id, "f1k_apply");
            RCLCPP_WARN(
                get_logger(),
                "[F1K-GLOBALMAP-PUBLISH-AFTER-APPLY] task_id=%llu topic=%s frame_id=%s",
                static_cast<unsigned long long>(dry_run.task_id),
                global_sparse_cloud_topic_.c_str(),
                world_frame_.c_str());
            return;
        }

        const bool force_reject = ShouldForceF1LReject(dry_run.task_id);
        RCLCPP_WARN(
            get_logger(),
            "[F1L-POST-APPLY-START] task_id=%llu type=%s useful=%s partial_candidate=%s forced_reject=%s",
            static_cast<unsigned long long>(dry_run.task_id),
            dry_run.task_type.c_str(),
            dry_run.useful ? "true" : "false",
            dry_run.partial_candidate ? "true" : "false",
            force_reject ? "true" : "false");

        PostApplyValidationResult validation =
            optimization_manager_.ValidatePostApply(
                dry_run,
                problem,
                apply,
                candidate_pose_store,
                force_reject);

        const GlobalMapBuildResult map_after =
            global_map_builder_.Build(
                raw_db_,
                candidate_pose_store,
                score_manager_,
                &candidate_fused_landmarks,
                static_cast<float>(global_map_min_score_to_publish_));
        const bool global_map_ok =
            (map_before.stats.returned_points == 0 ||
             map_after.stats.returned_points > 0) &&
            map_after.stats.invalid_pose_skipped <=
                map_before.stats.invalid_pose_skipped + 100;

        RCLCPP_WARN(
            get_logger(),
            "[F1L-POST-APPLY-ERROR] task_id=%llu type=%s before_t=%.6f predicted_after_t=%.6f real_after_t=%.6f before_yaw=%.6f predicted_after_yaw=%.6f real_after_yaw=%.6f improvement_ratio=%.6f",
            static_cast<unsigned long long>(dry_run.task_id),
            dry_run.task_type.c_str(),
            validation.error.before_error_t,
            validation.error.predicted_after_error_t,
            validation.error.real_after_error_t,
            validation.error.before_error_yaw,
            validation.error.predicted_after_error_yaw,
            validation.error.real_after_error_yaw,
            validation.error.improvement_ratio);
        RCLCPP_WARN(
            get_logger(),
            "[F1L-PARTIAL-ABSURD-ERROR-POLICY] task_id=%llu enabled=true absurd_threshold_t=%.6f before_t=%.6f real_after_t=%.6f improvement_ratio=%.6f partial_candidate=%s",
            static_cast<unsigned long long>(dry_run.task_id),
            f1l_post_apply_fiducial_absurd_error_t_,
            validation.error.before_error_t,
            validation.error.real_after_error_t,
            validation.error.improvement_ratio,
            dry_run.partial_candidate ? "true" : "false");
        RCLCPP_WARN(
            get_logger(),
            "[F1L-INTERNAL-EDGE-CLASSIFY] task_id=%llu strong_edges_checked=%llu deformable_edges_checked=%llu strong_edges_broken=%llu deformable_edges_broken=%llu max_strong_internal_after=%.6f max_deformable_internal_after=%.6f",
            static_cast<unsigned long long>(dry_run.task_id),
            static_cast<unsigned long long>(
                validation.internal_error.strong_edges_checked),
            static_cast<unsigned long long>(
                validation.internal_error.deformable_edges_checked),
            static_cast<unsigned long long>(
                validation.internal_error.strong_edges_broken),
            static_cast<unsigned long long>(
                validation.internal_error.deformable_edges_broken),
            validation.internal_error.max_strong_internal_after,
            validation.internal_error.max_deformable_internal_after);
        RCLCPP_WARN(
            get_logger(),
            "[F1L-POST-APPLY-INTERNAL-ERROR] task_id=%llu ok=%s internal_mean_before=%.6f internal_mean_after=%.6f internal_max_before=%.6f internal_max_after=%.6f num_edges_worse=%llu num_edges_broken=%llu strong_edges_broken=%llu deformable_edges_broken=%llu",
            static_cast<unsigned long long>(dry_run.task_id),
            validation.internal_edges_ok ? "true" : "false",
            validation.internal_error.internal_mean_before,
            validation.internal_error.internal_mean_after,
            validation.internal_error.internal_max_before,
            validation.internal_error.internal_max_after,
            static_cast<unsigned long long>(validation.internal_error.num_edges_worse),
            static_cast<unsigned long long>(validation.internal_error.num_edges_broken),
            static_cast<unsigned long long>(
                validation.internal_error.strong_edges_broken),
            static_cast<unsigned long long>(
                validation.internal_error.deformable_edges_broken));
        RCLCPP_WARN(
            get_logger(),
            "[F1L-POST-APPLY-FIXED-CHECK] task_id=%llu ok=%s hard_fixed_moved=%s fixed_moved_count=%llu max_fixed_delta_t=%.9f max_fixed_delta_yaw=%.9f",
            static_cast<unsigned long long>(dry_run.task_id),
            validation.fixed_ok ? "true" : "false",
            validation.fixed_check.hard_fixed_moved ? "true" : "false",
            static_cast<unsigned long long>(validation.fixed_check.fixed_moved_count),
            validation.fixed_check.max_fixed_delta_t,
            validation.fixed_check.max_fixed_delta_yaw);
        RCLCPP_WARN(
            get_logger(),
            "[F1L-ANCHOR-PRESERVATION-CHECK] task_id=%llu ok=%s required=%s graph_satisfied=%s previous_fiducial_fixed_count=%llu previous_fiducial_neighborhood_fixed_count=%llu independent_branches=%llu checked_branch_anchors=%llu subdivision_candidates=%llu max_previous_fiducial_delta_t=%.9f max_previous_fiducial_delta_yaw=%.9f max_previous_fiducial_neighborhood_delta_t=%.9f max_previous_fiducial_neighborhood_delta_yaw=%.9f reason=%s",
            static_cast<unsigned long long>(dry_run.task_id),
            validation.anchor_preservation_ok ? "true" : "false",
            validation.anchor_preservation_check.required ? "true" : "false",
            validation.anchor_preservation_check.graph_satisfied ? "true" : "false",
            static_cast<unsigned long long>(
                validation.anchor_preservation_check.previous_fiducial_fixed_count),
            static_cast<unsigned long long>(
                validation.anchor_preservation_check
                    .previous_fiducial_neighborhood_fixed_count),
            static_cast<unsigned long long>(
                validation.anchor_preservation_check.independent_branches),
            static_cast<unsigned long long>(
                validation.anchor_preservation_check.checked_branch_anchors),
            static_cast<unsigned long long>(
                validation.anchor_preservation_check.subdivision_candidates),
            validation.anchor_preservation_check.max_previous_fiducial_delta_t,
            validation.anchor_preservation_check.max_previous_fiducial_delta_yaw,
            validation.anchor_preservation_check
                .max_previous_fiducial_neighborhood_delta_t,
            validation.anchor_preservation_check
                .max_previous_fiducial_neighborhood_delta_yaw,
            validation.anchor_preservation_check.reason.c_str());
        if (!validation.anchor_preservation_ok)
        {
            RCLCPP_ERROR(
                get_logger(),
                "[F1L-ANCHOR-PRESERVATION-FAIL] task_id=%llu reason=%s",
                static_cast<unsigned long long>(dry_run.task_id),
                validation.anchor_preservation_check.reason.c_str());
        }
        RCLCPP_WARN(
            get_logger(),
            "[F1L-POST-APPLY-PROPAGATION-CHECK] task_id=%llu ok=%s propagated_count=%llu skipped_propagated_count=%llu rebased_count=%llu propagated_max_delta_t=%.6f propagated_mean_delta_t=%.6f propagation_discontinuity_max_t=%.9f propagation_discontinuity_max_yaw=%.9f",
            static_cast<unsigned long long>(dry_run.task_id),
            validation.propagation_ok ? "true" : "false",
            static_cast<unsigned long long>(
                validation.propagation_check.propagated_count),
            static_cast<unsigned long long>(
                validation.propagation_check.skipped_propagated_count),
            static_cast<unsigned long long>(
                validation.propagation_check.rebased_count),
            validation.propagation_check.propagated_max_delta_t,
            validation.propagation_check.propagated_mean_delta_t,
            validation.propagation_check.propagation_discontinuity_max_t,
            validation.propagation_check.propagation_discontinuity_max_yaw);
        RCLCPP_WARN(
            get_logger(),
            "[F1L-POST-APPLY-GLOBALMAP-CHECK] task_id=%llu ok=%s published_points_before=%llu published_points_after=%llu server_corrected_before=%llu server_corrected_after=%llu score_min=%.3f score_mean=%.3f score_max=%.3f nan_points=0 invalid_pose_skipped_before=%llu invalid_pose_skipped_after=%llu anchored_submaps=%llu",
            static_cast<unsigned long long>(dry_run.task_id),
            global_map_ok ? "true" : "false",
            static_cast<unsigned long long>(map_before.stats.returned_points),
            static_cast<unsigned long long>(map_after.stats.returned_points),
            static_cast<unsigned long long>(map_before.stats.server_corrected_points),
            static_cast<unsigned long long>(map_after.stats.server_corrected_points),
            map_after.stats.score_min,
            map_after.stats.score_mean,
            map_after.stats.score_max,
            static_cast<unsigned long long>(map_before.stats.invalid_pose_skipped),
            static_cast<unsigned long long>(map_after.stats.invalid_pose_skipped),
            static_cast<unsigned long long>(map_after.stats.anchored_submaps));
        RCLCPP_WARN(
            get_logger(),
            "[F1L-GLOBALMAP-KF-PROJECTION] task_id=%llu keyframe_projected_before=%llu keyframe_projected_after=%llu fallback_submap_before=%llu fallback_submap_after=%llu server_corrected_candidates_before=%llu server_corrected_candidates_after=%llu server_corrected_missing_kf_before=%llu server_corrected_missing_kf_after=%llu",
            static_cast<unsigned long long>(dry_run.task_id),
            static_cast<unsigned long long>(map_before.stats.keyframe_projected_points),
            static_cast<unsigned long long>(map_after.stats.keyframe_projected_points),
            static_cast<unsigned long long>(map_before.stats.fallback_submap_points),
            static_cast<unsigned long long>(map_after.stats.fallback_submap_points),
            static_cast<unsigned long long>(
                map_before.stats.server_corrected_mappoint_candidates),
            static_cast<unsigned long long>(
                map_after.stats.server_corrected_mappoint_candidates),
            static_cast<unsigned long long>(
                map_before.stats.server_corrected_missing_keyframe_skipped),
            static_cast<unsigned long long>(
                map_after.stats.server_corrected_missing_keyframe_skipped));

        PostApplyDecision final_decision = validation.decision;
        std::string final_reason = validation.reason;
        if (!global_map_ok && final_decision != PostApplyDecision::RejectRollback)
        {
            final_decision = PostApplyDecision::RejectRollback;
            final_reason = "global_map_check_failed";
        }

        if (final_decision == PostApplyDecision::Accept)
        {
            f1l_partial_checkpoint_task_by_submap_.erase(problem.submap_id);
            f1l_partial_retry_count_by_submap_.erase(problem.submap_id);
            RCLCPP_WARN(
                get_logger(),
                "[F1L-POST-APPLY-ACCEPT] task_id=%llu reason=%s real_after_t=%.6f threshold_t=%.6f decision=%s",
                static_cast<unsigned long long>(dry_run.task_id),
                final_reason.c_str(),
                validation.error.real_after_error_t,
                f1j_dryrun_max_final_error_t_,
                orbslam3_multi::ToString(final_decision));
            std::string fiducial_accept_reason;
            const bool fiducial_accepted =
                fiducial_anchor_manager_.AcceptOptimizedFiducialTask(
                    dry_run.task_id,
                    candidate_pose_store,
                    "SERVER_OPTIMIZATION_FIDUCIAL_ACCEPT",
                    &fiducial_accept_reason);
            RCLCPP_WARN(
                get_logger(),
                "[F1K-FIDUCIAL-TARGET-ACCEPTED] task_id=%llu ok=%s reason=%s validation_enabled=true",
                static_cast<unsigned long long>(dry_run.task_id),
                fiducial_accepted ? "true" : "false",
                fiducial_accept_reason.c_str());
            const bool confirmed =
                candidate_pose_store.ConfirmApply(dry_run.task_id);
            if (confirmed)
            {
                pose_store_ = std::move(candidate_pose_store);
                TryTraceFlow(
                    "optimizer_pose_db",
                    "commit",
                    "fiducial_optimization",
                    dry_run.task_id,
                    apply.optimized_kfs + apply.propagated_kfs);
            }
            RCLCPP_WARN(
                get_logger(),
                "[F1L-POSESTORE-COMMIT-CONFIRMED] task_id=%llu ok=%s decision=ACCEPT",
                static_cast<unsigned long long>(dry_run.task_id),
                confirmed ? "true" : "false");
            RCLCPP_WARN(
                get_logger(),
                "[F1K-POSE-COMMIT] task_id=%llu decision=ACCEPT committed=%s mode=atomic_candidate_swap",
                static_cast<unsigned long long>(dry_run.task_id),
                confirmed ? "true" : "false");
            CloseFiducialOptimizationTask(dry_run.task_id, "post_apply_accept");
            RequestOptimizationStatePublication(
                dry_run.task_id, "f1l_accept");
            RCLCPP_WARN(
                get_logger(),
                "[F1K-GLOBALMAP-PUBLISH-AFTER-APPLY] task_id=%llu topic=%s frame_id=%s decision=ACCEPT",
                static_cast<unsigned long long>(dry_run.task_id),
                global_sparse_cloud_topic_.c_str(),
                world_frame_.c_str());
            return;
        }

        if (final_decision == PostApplyDecision::PartialKeepForNextPass)
        {
            auto retry_it =
                f1l_partial_retry_count_by_submap_.find(problem.submap_id);
            const uint64_t retry_count =
                retry_it == f1l_partial_retry_count_by_submap_.end()
                    ? 0U
                    : retry_it->second + 1U;
            f1l_partial_checkpoint_task_by_submap_[problem.submap_id] =
                dry_run.task_id;
            f1l_partial_retry_count_by_submap_[problem.submap_id] = retry_count;
            RCLCPP_WARN(
                get_logger(),
                "[F1L-POST-APPLY-PARTIAL] task_id=%llu reason=%s partial_reason=%s real_after_t=%.6f threshold_t=%.6f needs_second_pass=true partial_pending=true retry_required=true retry_strategy=%s decision=%s",
                static_cast<unsigned long long>(dry_run.task_id),
                final_reason.c_str(),
                dry_run.partial_reason.empty()
                    ? "partial_real_improvement_safe"
                    : dry_run.partial_reason.c_str(),
                validation.error.real_after_error_t,
                f1j_dryrun_max_final_error_t_,
                validation.retry.strategy.c_str(),
                orbslam3_multi::ToString(final_decision));
            RCLCPP_WARN(
                get_logger(),
                "[F1L-PARTIAL-PENDING] task_id=%llu retry_required=true graph_rebuild_required=true retry_count=%llu max_retries=%d retry_strategy=%s reason=%s",
                static_cast<unsigned long long>(dry_run.task_id),
                static_cast<unsigned long long>(retry_count),
                f1l_max_partial_retries_,
                validation.retry.strategy.c_str(),
                final_reason.c_str());
            RCLCPP_WARN(
                get_logger(),
                "[F1L-PARTIAL-CHECKPOINT] task_id=%llu checkpoint_saved=true rollback_target=partial_checkpoint safe=true inherit_submap_last_correction=false real_after_t=%.6f before_t=%.6f strong_edges_broken=%llu deformable_edges_broken=%llu",
                static_cast<unsigned long long>(dry_run.task_id),
                validation.error.real_after_error_t,
                validation.error.before_error_t,
                static_cast<unsigned long long>(
                    validation.internal_error.strong_edges_broken),
                static_cast<unsigned long long>(
                    validation.internal_error.deformable_edges_broken));
            RCLCPP_WARN(
                get_logger(),
                "[F1L-RETRY-SUGGESTION] task_id=%llu strategy=%s retry_allowed=%s reason=%s",
                static_cast<unsigned long long>(dry_run.task_id),
                validation.retry.strategy.c_str(),
                validation.retry.retry_allowed ? "true" : "false",
                validation.retry.reason.c_str());
            const bool confirmed =
                candidate_pose_store.ConfirmApply(dry_run.task_id);
            if (confirmed)
            {
                pose_store_ = std::move(candidate_pose_store);
                TryTraceFlow(
                    "optimizer_pose_db",
                    "commit",
                    "fiducial_partial",
                    dry_run.task_id,
                    apply.optimized_kfs + apply.propagated_kfs);
            }
            RCLCPP_WARN(
                get_logger(),
                "[F1L-POSESTORE-COMMIT-CONFIRMED] task_id=%llu ok=%s decision=PARTIAL_KEEP_FOR_NEXT_PASS",
                static_cast<unsigned long long>(dry_run.task_id),
                confirmed ? "true" : "false");
            RCLCPP_WARN(
                get_logger(),
                "[F1K-POSE-COMMIT] task_id=%llu decision=PARTIAL_KEEP_FOR_NEXT_PASS committed=%s mode=atomic_candidate_swap",
                static_cast<unsigned long long>(dry_run.task_id),
                confirmed ? "true" : "false");
            RequestOptimizationStatePublication(
                dry_run.task_id, "f1l_partial");
            RCLCPP_WARN(
                get_logger(),
                "[F1K-GLOBALMAP-PUBLISH-AFTER-APPLY] task_id=%llu topic=%s frame_id=%s decision=PARTIAL_KEEP_FOR_NEXT_PASS",
                static_cast<unsigned long long>(dry_run.task_id),
                global_sparse_cloud_topic_.c_str(),
                world_frame_.c_str());
            if (confirmed &&
                retry_count < static_cast<uint64_t>(f1l_max_partial_retries_))
            {
                // 1L: la segunda pasada debe usar el mismo fiducial objetivo y
                // las poses del checkpoint recien escrito. Esperar a otro KF
                // crearia una tarea nueva desde una pose raw sin herencia y
                // volveria artificialmente al error absurdo original.
                RCLCPP_WARN(
                    get_logger(),
                    "[F1L-PARTIAL-RETRY-START] task_id=%llu retry_count=%llu max_retries=%d rebuild_same_task=true source=GlobalPoseStore_partial_checkpoint",
                    static_cast<unsigned long long>(dry_run.task_id),
                    static_cast<unsigned long long>(retry_count + 1U),
                    f1l_max_partial_retries_);
                ScheduleOptimizationWorker(
                    task,
                    "f1l_partial_retry_from_checkpoint");
            }
            return;
        }

        RCLCPP_ERROR(
            get_logger(),
            "[F1L-POST-APPLY-REJECT] task_id=%llu reason=%s real_after_t=%.6f before_t=%.6f decision=%s",
            static_cast<unsigned long long>(dry_run.task_id),
            final_reason.c_str(),
            validation.error.real_after_error_t,
            validation.error.before_error_t,
            orbslam3_multi::ToString(final_decision));
        const auto rollback =
            candidate_pose_store.RestoreApplyBackup(dry_run.task_id);
        RCLCPP_WARN(
            get_logger(),
            "[F1L-POSESTORE-ROLLBACK] task_id=%llu ok=%s restored_world=%llu removed_world=%llu restored_optimized=%llu removed_optimized=%llu restored_submap_corrections=%llu removed_submap_corrections=%llu restored_accepted_anchors=%llu removed_accepted_anchors=%llu restored_derived_tail_kfs=%llu removed_derived_tail_kfs=%llu restored_active_tail_anchors=%llu removed_active_tail_anchors=%llu reason=%s live_state_unchanged=true",
            static_cast<unsigned long long>(dry_run.task_id),
            rollback.success ? "true" : "false",
            static_cast<unsigned long long>(rollback.restored_world_poses),
            static_cast<unsigned long long>(rollback.removed_world_poses),
            static_cast<unsigned long long>(rollback.restored_optimized_poses),
            static_cast<unsigned long long>(rollback.removed_optimized_poses),
            static_cast<unsigned long long>(rollback.restored_submap_corrections),
            static_cast<unsigned long long>(rollback.removed_submap_corrections),
            static_cast<unsigned long long>(rollback.restored_accepted_anchors),
            static_cast<unsigned long long>(rollback.removed_accepted_anchors),
            static_cast<unsigned long long>(
                rollback.restored_derived_tail_keyframes),
            static_cast<unsigned long long>(
                rollback.removed_derived_tail_keyframes),
            static_cast<unsigned long long>(rollback.restored_active_tail_anchors),
            static_cast<unsigned long long>(rollback.removed_active_tail_anchors),
            rollback.reason.c_str());
        RCLCPP_WARN(
            get_logger(),
            "[F1L-GLOBALMAP-REBUILD-AFTER-ROLLBACK] task_id=%llu reason=%s",
            static_cast<unsigned long long>(dry_run.task_id),
            final_reason.c_str());
        RequestGlobalStatePublication("f1l_rollback");
        RCLCPP_WARN(
            get_logger(),
            "[F1L-GLOBALMAP-PUBLISH-AFTER-ROLLBACK] task_id=%llu topic=%s frame_id=%s",
            static_cast<unsigned long long>(dry_run.task_id),
            global_sparse_cloud_topic_.c_str(),
            world_frame_.c_str());
        RCLCPP_WARN(
            get_logger(),
            "[F1L-ROLLBACK-DIAGNOSTIC] task_id=%llu type=%s reason=%s before_t=%.6f predicted_after_t=%.6f real_after_t=%.6f internal_max_before=%.6f internal_max_after=%.6f hard_fixed_moved=%s propagation_ok=%s globalmap_ok=%s",
            static_cast<unsigned long long>(dry_run.task_id),
            dry_run.task_type.c_str(),
            final_reason.c_str(),
            validation.error.before_error_t,
            validation.error.predicted_after_error_t,
            validation.error.real_after_error_t,
            validation.internal_error.internal_max_before,
            validation.internal_error.internal_max_after,
            validation.fixed_check.hard_fixed_moved ? "true" : "false",
            validation.propagation_ok ? "true" : "false",
            global_map_ok ? "true" : "false");
        RCLCPP_WARN(
            get_logger(),
            "[F1L-RETRY-SUGGESTION] task_id=%llu strategy=%s retry_allowed=%s reason=%s",
            static_cast<unsigned long long>(dry_run.task_id),
            validation.retry.strategy.c_str(),
            validation.retry.retry_allowed ? "true" : "false",
            validation.retry.reason.c_str());
        CloseFiducialOptimizationTask(
            dry_run.task_id,
            "post_apply_reject:" + final_reason);
    }

    void LogPoseGraphProblem(const PoseGraphProblem& problem,
                             const std::string& reason)
    {
        uint64_t min_kf = problem.target_keyframe_id.local_kf_id;
        uint64_t max_kf = problem.target_keyframe_id.local_kf_id;
        for (const auto& vertex : problem.vertices)
        {
            min_kf = std::min<uint64_t>(min_kf, vertex.keyframe_id.local_kf_id);
            max_kf = std::max<uint64_t>(max_kf, vertex.keyframe_id.local_kf_id);
        }
        for (const auto& keyframe_id : problem.affected_non_variable_keyframes)
        {
            min_kf = std::min<uint64_t>(min_kf, keyframe_id.local_kf_id);
            max_kf = std::max<uint64_t>(max_kf, keyframe_id.local_kf_id);
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1I-GRAPH-WINDOW] task_id=%llu drone_id=%u epoch=%llu target_kf=%llu local_kf_range=[%llu,%llu] vertices=%llu affected_non_vertices=%llu anchor_stop=%s rebuilt_from_partial=%s partial_parent_task_id=%llu partial_retry_count=%llu",
            static_cast<unsigned long long>(problem.task_id),
            problem.submap_id.drone_id,
            static_cast<unsigned long long>(problem.submap_id.map_epoch),
            static_cast<unsigned long long>(problem.target_keyframe_id.local_kf_id),
            static_cast<unsigned long long>(min_kf),
            static_cast<unsigned long long>(max_kf),
            static_cast<unsigned long long>(problem.vertices.size()),
            static_cast<unsigned long long>(problem.affected_non_variable_keyframes.size()),
            pose_graph_anchor_stop_enabled_ ? "true" : "false",
            problem.rebuilt_from_partial_checkpoint ? "true" : "false",
            static_cast<unsigned long long>(problem.partial_parent_task_id),
            static_cast<unsigned long long>(problem.partial_retry_count));

        const auto config = pose_graph_builder_.GetConfig();
        uint64_t corner_vertices = 0;
        uint64_t target_neighborhood_vertices = 0;
        uint64_t previous_neighborhood_vertices = 0;
        for (const auto& vertex : problem.vertices)
        {
            if (vertex.selection_reason == "corner_3d_vertex" ||
                vertex.selection_reason == "corner_yaw_vertex")
            {
                ++corner_vertices;
            }
            if (vertex.selection_reason == "target_fiducial_neighborhood")
            {
                ++target_neighborhood_vertices;
            }
            if (vertex.selection_reason == "previous_fiducial_neighborhood")
            {
                ++previous_neighborhood_vertices;
            }
        }
        RCLCPP_WARN(
            get_logger(),
            "[F1I-GRAPH-VERTEX-COVERAGE] task_id=%llu vertices=%llu window_keyframes=%llu vertex_policy=balanced_coverage_sample vertex_selection_ratio=%.3f vertex_limit=none edge_length_limit_m=none mandatory_fiducial_vertices=2 corner_vertices=%llu target_neighborhood_vertices=%llu previous_fiducial_neighborhood_vertices=%llu max_consecutive_id_gap=%llu max_consecutive_distance_m=%.3f uncovered_long_segments=%llu coverage_complete=%s",
            static_cast<unsigned long long>(problem.task_id),
            static_cast<unsigned long long>(problem.vertices.size()),
            static_cast<unsigned long long>(problem.coverage.window_keyframes),
            config.vertex_selection_ratio,
            static_cast<unsigned long long>(corner_vertices),
            static_cast<unsigned long long>(target_neighborhood_vertices),
            static_cast<unsigned long long>(previous_neighborhood_vertices),
            static_cast<unsigned long long>(problem.coverage.max_control_span_kfs),
            problem.coverage.max_control_span_m,
            static_cast<unsigned long long>(
                problem.coverage.uncovered_long_segments),
            problem.coverage.coverage_complete ? "true" : "false");

        RCLCPP_WARN(
            get_logger(),
            "[F1I-FID-CONNECTIVITY-BRANCHES] task_id=%llu required=%s satisfied=%s independent_branches=%llu branch_anchor_count=%llu previous_fiducial_fixed_count=%llu previous_fiducial_neighborhood_fixed_count=%llu subdivision_candidates=%llu subdivided_confirmed=%llu reason=%s",
            static_cast<unsigned long long>(problem.task_id),
            problem.anchor_preservation.required ? "true" : "false",
            problem.anchor_preservation.satisfied ? "true" : "false",
            static_cast<unsigned long long>(
                problem.anchor_preservation.independent_branches),
            static_cast<unsigned long long>(
                problem.anchor_preservation.branch_anchor_count),
            static_cast<unsigned long long>(
                problem.anchor_preservation.previous_fiducial_fixed_count),
            static_cast<unsigned long long>(
                problem.anchor_preservation
                    .previous_fiducial_neighborhood_fixed_count),
            static_cast<unsigned long long>(
                problem.anchor_preservation.subdivision_candidates),
            static_cast<unsigned long long>(
                problem.anchor_preservation.subdivided_confirmed),
            problem.anchor_preservation.reason.c_str());

        for (const auto& edge : problem.fiducial_connectivity_edges)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1I-FID-CONNECTIVITY-EDGE] task_id=%llu from_fid=%d to_fid=%d from_kf=%llu to_kf=%llu status=%s selected_anchor=%s independent_branch=%s kf_gap=%llu reason=%s",
                static_cast<unsigned long long>(problem.task_id),
                edge.from_fiducial_id,
                edge.to_fiducial_id,
                static_cast<unsigned long long>(edge.from_keyframe_id.local_kf_id),
                static_cast<unsigned long long>(edge.to_keyframe_id.local_kf_id),
                orbslam3_multi::ToString(edge.status),
                edge.selected_as_branch_anchor ? "true" : "false",
                edge.independent_branch ? "true" : "false",
                static_cast<unsigned long long>(edge.kf_gap),
                edge.reason.c_str());
            RCLCPP_WARN(
                get_logger(),
                "[F1I-FID-CONNECTIVITY-SUBDIVISION] task_id=%llu from_fid=%d to_fid=%d edge_status=%s reason=%s",
                static_cast<unsigned long long>(problem.task_id),
                edge.from_fiducial_id,
                edge.to_fiducial_id,
                orbslam3_multi::ToString(edge.status),
                edge.reason.c_str());
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1I-GRAPH-ANCHOR-PRESERVATION] task_id=%llu required=%s satisfied=%s previous_fiducial_fixed_count=%llu previous_fiducial_neighborhood_fixed_count=%llu branch_anchor_count=%llu independent_branches=%llu",
            static_cast<unsigned long long>(problem.task_id),
            problem.anchor_preservation.required ? "true" : "false",
            problem.anchor_preservation.satisfied ? "true" : "false",
            static_cast<unsigned long long>(
                problem.anchor_preservation.previous_fiducial_fixed_count),
            static_cast<unsigned long long>(
                problem.anchor_preservation
                    .previous_fiducial_neighborhood_fixed_count),
            static_cast<unsigned long long>(
                problem.anchor_preservation.branch_anchor_count),
            static_cast<unsigned long long>(
                problem.anchor_preservation.independent_branches));

        for (const auto& vertex : problem.vertices)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1I-GRAPH-VERTEX-SELECT] task_id=%llu drone_id=%u epoch=%llu kf=%llu variable=%s fixed=%s hard_fiducial=%s anchor_neighborhood=%s support=%llu weight=%.3f reason=%s world_t=(%.3f,%.3f,%.3f) yaw=%.3f",
                static_cast<unsigned long long>(problem.task_id),
                vertex.keyframe_id.drone_id,
                static_cast<unsigned long long>(vertex.keyframe_id.map_epoch),
                static_cast<unsigned long long>(vertex.keyframe_id.local_kf_id),
                vertex.is_variable ? "true" : "false",
                vertex.is_fixed ? "true" : "false",
                vertex.is_hard_fiducial ? "true" : "false",
                vertex.is_anchor_neighborhood ? "true" : "false",
                static_cast<unsigned long long>(vertex.support_count),
                vertex.weight,
                vertex.selection_reason.c_str(),
                vertex.initial_world_T_kf(0, 3),
                vertex.initial_world_T_kf(1, 3),
                vertex.initial_world_T_kf(2, 3),
                YawFromTransform(vertex.initial_world_T_kf));
            RCLCPP_WARN(
                get_logger(),
                "[F1I-GRAPH-%s-KF] task_id=%llu drone_id=%u epoch=%llu kf=%llu reason=%s",
                vertex.is_fixed ? "FIXED" : "VARIABLE",
                static_cast<unsigned long long>(problem.task_id),
                vertex.keyframe_id.drone_id,
                static_cast<unsigned long long>(vertex.keyframe_id.map_epoch),
                static_cast<unsigned long long>(vertex.keyframe_id.local_kf_id),
                vertex.selection_reason.c_str());
            if (vertex.selection_reason == "previous_fiducial_anchor")
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1I-GRAPH-PREVIOUS-FIDUCIAL-ANCHOR] task_id=%llu drone_id=%u epoch=%llu kf=%llu fixed=%s hard_fiducial=%s",
                    static_cast<unsigned long long>(problem.task_id),
                    vertex.keyframe_id.drone_id,
                    static_cast<unsigned long long>(vertex.keyframe_id.map_epoch),
                    static_cast<unsigned long long>(vertex.keyframe_id.local_kf_id),
                    vertex.is_fixed ? "true" : "false",
                    vertex.is_hard_fiducial ? "true" : "false");
            }
            if (vertex.selection_reason == "corner_3d_vertex" ||
                vertex.selection_reason == "corner_yaw_vertex")
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1I-GRAPH-CORNER-VERTEX] task_id=%llu drone_id=%u epoch=%llu kf=%llu policy=3d_se3 threshold_rad=%.3f",
                    static_cast<unsigned long long>(problem.task_id),
                    vertex.keyframe_id.drone_id,
                    static_cast<unsigned long long>(vertex.keyframe_id.map_epoch),
                    static_cast<unsigned long long>(vertex.keyframe_id.local_kf_id),
                    config.corner_yaw_threshold_rad);
            }
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1I-GRAPH-EDGES] task_id=%llu count=%llu temporal_edges=%llu source=F1I_TEMPORAL_WINDOW",
            static_cast<unsigned long long>(problem.task_id),
            static_cast<unsigned long long>(problem.edges.size()),
            static_cast<unsigned long long>(problem.edges.size()));
        for (const auto& edge : problem.edges)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1L-GRAPH-EDGE-SUPPORT] task_id=%llu edge_id=%llu from_kf=%llu to_kf=%llu intermediate_kfs=%llu support_kfs=%llu support_length_m=%.6f support_density_kfs_per_m=%.6f support_rigidity_multiplier=%.6f weight=%.6f",
                static_cast<unsigned long long>(problem.task_id),
                static_cast<unsigned long long>(edge.edge_id),
                static_cast<unsigned long long>(edge.from_keyframe_id.local_kf_id),
                static_cast<unsigned long long>(edge.to_keyframe_id.local_kf_id),
                static_cast<unsigned long long>(edge.intermediate_keyframe_count),
                static_cast<unsigned long long>(edge.support_keyframe_count),
                edge.support_length_m,
                edge.support_density_kfs_per_m,
                edge.support_rigidity_multiplier,
                edge.weight);
        }

        uint64_t hard_priors = 0;
        uint64_t target_priors = 0;
        uint64_t soft_priors = 0;
        for (const auto& prior : problem.priors)
        {
            if (prior.prior_type == orbslam3_multi::PoseGraphPriorType::FiducialHard)
            {
                ++hard_priors;
            }
            else if (prior.prior_type == orbslam3_multi::PoseGraphPriorType::FiducialTarget)
            {
                ++target_priors;
            }
            else
            {
                ++soft_priors;
            }
        }
        RCLCPP_WARN(
            get_logger(),
            "[F1I-GRAPH-PRIORS] task_id=%llu count=%llu hard_fiducial=%llu fiducial_target=%llu current_soft=%llu",
            static_cast<unsigned long long>(problem.task_id),
            static_cast<unsigned long long>(problem.priors.size()),
            static_cast<unsigned long long>(hard_priors),
            static_cast<unsigned long long>(target_priors),
            static_cast<unsigned long long>(soft_priors));

        RCLCPP_WARN(
            get_logger(),
            "[F1I-GRAPH-WEIGHTS] task_id=%llu temporal=%.3f temporal_sparse=%.3f fiducial_hard=%.3f fiducial_target_t=%.3f fiducial_target_rot=%.3f current_pose_soft=%.3f",
            static_cast<unsigned long long>(problem.task_id),
            config.temporal_edge_weight,
            config.temporal_edge_weight_sparse,
            config.fiducial_hard_weight,
            config.fiducial_target_translation_weight,
            config.fiducial_target_rotation_weight,
            config.current_pose_soft_weight);

        RCLCPP_WARN(
            get_logger(),
            "[F1I-GRAPH-PROPAGATION-PLAN] task_id=%llu affected_non_variable=%llu entries=%llu mode_policy=path_segment_or_nearest",
            static_cast<unsigned long long>(problem.task_id),
            static_cast<unsigned long long>(problem.affected_non_variable_keyframes.size()),
            static_cast<unsigned long long>(problem.propagation_plan.size()));

        for (const auto& entry : problem.propagation_plan)
        {
            const std::string control_b_text = entry.control_vertex_b
                ? std::to_string(entry.control_vertex_b->local_kf_id)
                : "none";
            RCLCPP_WARN(
                get_logger(),
                "[F1I-GRAPH-PROPAGATION-SEGMENT] task_id=%llu affected_kf=%llu control_a=%llu control_b=%s alpha=%.6f distance_from_a_m=%.3f segment_length_m=%.3f control_span_kf_gap=%llu mode=%s",
                static_cast<unsigned long long>(problem.task_id),
                static_cast<unsigned long long>(entry.affected_keyframe_id.local_kf_id),
                static_cast<unsigned long long>(entry.control_vertex_a.local_kf_id),
                control_b_text.c_str(),
                entry.segment_alpha,
                entry.distance_from_a_m,
                entry.segment_length_m,
                static_cast<unsigned long long>(entry.control_span_kf_gap),
                orbslam3_multi::ToString(entry.mode));
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1I-GRAPH-PROBLEM-CREATED] task_id=%llu type=%s source=%s vertices=%llu edges=%llu priors=%llu variables=%llu fixed=%llu affected_non_variable=%llu propagation=%llu",
            static_cast<unsigned long long>(problem.task_id),
            problem.task_type.c_str(),
            problem.source.c_str(),
            static_cast<unsigned long long>(problem.summary.vertices),
            static_cast<unsigned long long>(problem.summary.edges),
            static_cast<unsigned long long>(problem.summary.priors),
            static_cast<unsigned long long>(problem.summary.variable_vertices),
            static_cast<unsigned long long>(problem.summary.fixed_vertices),
            static_cast<unsigned long long>(
                problem.summary.affected_non_variable_keyframes),
            static_cast<unsigned long long>(problem.summary.propagation_entries));

        RCLCPP_WARN(
            get_logger(),
            "[F1I-GRAPH-BUILD-SUMMARY] task_id=%llu success=true reason=%s vertices=%llu variable=%llu fixed=%llu hard_fiducial=%llu edges=%llu priors=%llu affected_non_variable=%llu propagation=%llu",
            static_cast<unsigned long long>(problem.task_id),
            reason.c_str(),
            static_cast<unsigned long long>(problem.summary.vertices),
            static_cast<unsigned long long>(problem.summary.variable_vertices),
            static_cast<unsigned long long>(problem.summary.fixed_vertices),
            static_cast<unsigned long long>(problem.summary.hard_fiducial_vertices),
            static_cast<unsigned long long>(problem.summary.edges),
            static_cast<unsigned long long>(problem.summary.priors),
            static_cast<unsigned long long>(
                problem.summary.affected_non_variable_keyframes),
            static_cast<unsigned long long>(problem.summary.propagation_entries));
    }

    void PopulatePoseGraphDebugKeyFrames(PoseGraphProblem& problem) const
    {
        std::set<RawKeyFrameId> keyframes;
        for (const auto& vertex : problem.vertices)
        {
            keyframes.insert(vertex.keyframe_id);
        }
        for (const auto& keyframe_id : problem.affected_non_variable_keyframes)
        {
            keyframes.insert(keyframe_id);
        }

        problem.debug_keyframes.clear();
        problem.debug_keyframes.reserve(keyframes.size());
        for (const auto& keyframe_id : keyframes)
        {
            const auto map_pose = pose_store_.GetWorldPose(keyframe_id);
            if (!map_pose.has_value())
            {
                continue;
            }

            orbslam3_multi::PoseGraphDebugKeyFramePose debug;
            debug.keyframe_id = keyframe_id;
            debug.map_world_T_kf = map_pose.value();
            const auto gt_it = f1l_gt_keyframe_store_.find(keyframe_id);
            if (gt_it != f1l_gt_keyframe_store_.end())
            {
                debug.has_gt = true;
                debug.gt_world_T_kf = gt_it->second.world_T_kf_gt;
                debug.association_dt_sec = gt_it->second.association_dt_sec;
                debug.association_quality = gt_it->second.association_quality;
            }
            else
            {
                debug.association_quality = "MISSING";
            }
            problem.debug_keyframes.push_back(debug);
        }
    }

    void DumpPoseGraphProblemForOffline(const PoseGraphProblem& problem) const
    {
        std::lock_guard<std::recursive_mutex> lock(live_state_mutex_);
        if (!f1l_graph_dump_enabled_)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1L-GRAPH-DUMP] task_id=%llu enabled=false reason=disabled",
                static_cast<unsigned long long>(problem.task_id));
            return;
        }

        PoseGraphProblem dump_problem = problem;
        PopulatePoseGraphDebugKeyFrames(dump_problem);
        uint64_t gt_debug_count = 0;
        for (const auto& debug : dump_problem.debug_keyframes)
        {
            if (debug.has_gt)
            {
                ++gt_debug_count;
            }
        }

        const std::string slash =
            (!f1l_graph_dump_output_dir_.empty() &&
             f1l_graph_dump_output_dir_.back() == '/') ? "" : "/";
        const std::string path =
            f1l_graph_dump_output_dir_ + slash +
            "f3l_graph_task_" + std::to_string(problem.task_id) + ".tsv";
        std::error_code mkdir_error;
        std::filesystem::create_directories(
            f1l_graph_dump_output_dir_,
            mkdir_error);
        if (mkdir_error)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1L-GRAPH-DUMP] task_id=%llu enabled=true success=false path=%s reason=mkdir_failed_%s vertices=%llu edges=%llu priors=%llu propagation=%llu debug_kfs=%llu gt_debug_kfs=%llu",
                static_cast<unsigned long long>(problem.task_id),
                path.c_str(),
                mkdir_error.message().c_str(),
                static_cast<unsigned long long>(dump_problem.vertices.size()),
                static_cast<unsigned long long>(dump_problem.edges.size()),
                static_cast<unsigned long long>(dump_problem.priors.size()),
                static_cast<unsigned long long>(dump_problem.propagation_plan.size()),
                static_cast<unsigned long long>(dump_problem.debug_keyframes.size()),
                static_cast<unsigned long long>(gt_debug_count));
            return;
        }
        const auto save = orbslam3_multi::SavePoseGraphProblem(dump_problem, path);
        RCLCPP_WARN(
            get_logger(),
            "[F1L-GRAPH-DUMP] task_id=%llu enabled=true success=%s path=%s reason=%s vertices=%llu edges=%llu priors=%llu propagation=%llu debug_kfs=%llu gt_debug_kfs=%llu",
            static_cast<unsigned long long>(problem.task_id),
            save.success ? "true" : "false",
            path.c_str(),
            save.reason.c_str(),
            static_cast<unsigned long long>(dump_problem.vertices.size()),
            static_cast<unsigned long long>(dump_problem.edges.size()),
            static_cast<unsigned long long>(dump_problem.priors.size()),
            static_cast<unsigned long long>(dump_problem.propagation_plan.size()),
            static_cast<unsigned long long>(dump_problem.debug_keyframes.size()),
            static_cast<unsigned long long>(gt_debug_count));

        const std::string window_path =
            f1l_graph_dump_output_dir_ + slash +
            "f3i_window_task_" + std::to_string(problem.task_id) + ".tsv";
        std::ofstream window_out(window_path);
        if (!window_out)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1I-WINDOW-DUMP] task_id=%llu enabled=true success=false path=%s reason=open_failed",
                static_cast<unsigned long long>(problem.task_id),
                window_path.c_str());
            return;
        }
        auto write_keyframe_id =
            [&](const RawKeyFrameId& id)
        {
            window_out << id.drone_id << '\t'
                       << id.map_epoch << '\t'
                       << id.local_kf_id;
        };
        auto write_matrix =
            [&](const Eigen::Matrix4d& matrix)
        {
            for (int r = 0; r < 4; ++r)
            {
                for (int c = 0; c < 4; ++c)
                {
                    window_out << '\t' << matrix(r, c);
                }
            }
        };
        auto vertex_role =
            [&](const RawKeyFrameId& id) -> std::string
        {
            for (const auto& vertex : dump_problem.vertices)
            {
                if (vertex.keyframe_id == id)
                {
                    return vertex.selection_reason;
                }
            }
            return "window_non_vertex_keyframe";
        };
        auto is_vertex =
            [&](const RawKeyFrameId& id)
        {
            return std::find_if(
                dump_problem.vertices.begin(),
                dump_problem.vertices.end(),
                [&](const orbslam3_multi::PoseGraphVertex& vertex)
                {
                    return vertex.keyframe_id == id;
                }) != dump_problem.vertices.end();
        };
        window_out << std::setprecision(17);
        window_out << "F1I_KF_WINDOW_DUMP\t1\n";
        window_out << "problem\t" << dump_problem.task_id << '\t'
                   << dump_problem.submap_id.drone_id << '\t'
                   << dump_problem.submap_id.map_epoch << '\t';
        write_keyframe_id(dump_problem.target_keyframe_id);
        window_out << '\t' << dump_problem.debug_keyframes.size() << '\n';
        for (const auto& edge : dump_problem.fiducial_connectivity_edges)
        {
            window_out << "fiducial_edge\t" << edge.from_fiducial_id << '\t'
                       << edge.to_fiducial_id << '\t';
            write_keyframe_id(edge.from_keyframe_id);
            window_out << '\t';
            write_keyframe_id(edge.to_keyframe_id);
            window_out << '\t' << orbslam3_multi::ToString(edge.status)
                       << '\t' << (edge.selected_as_branch_anchor ? 1 : 0)
                       << '\n';
        }
        for (const auto& debug : dump_problem.debug_keyframes)
        {
            window_out << "window_kf\t";
            write_keyframe_id(debug.keyframe_id);
            window_out << '\t' << (is_vertex(debug.keyframe_id) ? 1 : 0)
                       << '\t' << vertex_role(debug.keyframe_id);
            write_matrix(debug.map_world_T_kf);
            window_out << '\t' << (debug.has_gt ? 1 : 0);
            write_matrix(debug.gt_world_T_kf);
            window_out << '\t' << debug.association_dt_sec
                       << '\t' << debug.association_quality << '\n';
        }
        RCLCPP_WARN(
            get_logger(),
            "[F1I-WINDOW-DUMP] task_id=%llu enabled=true success=true path=%s window_kfs=%llu gt_debug_kfs=%llu graph_path=%s",
            static_cast<unsigned long long>(problem.task_id),
            window_path.c_str(),
            static_cast<unsigned long long>(dump_problem.debug_keyframes.size()),
            static_cast<unsigned long long>(gt_debug_count),
            path.c_str());
    }

    std::optional<FiducialOptimizationTask> CreateF1IDebugTaskFromCurrentState()
    {
        // F1I: la prueba replay puede usar un dataset cuyo residual fiducial
        // queda bajo umbral. Esta tarea sintetica valida solo el builder sobre
        // KFs reales/replay ya anclados; no se inserta en FiducialAnchorManager.
        const auto submap_ids = raw_db_.GetSubmapIds();
        for (const auto& submap_id : submap_ids)
        {
            if (!pose_store_.HasSubmapAnchor(submap_id))
            {
                continue;
            }

            RawKeyFrameId selected;
            bool found = false;
            const auto keyframe_ids = raw_db_.GetKeyFrameIdsForSubmap(submap_id);
            for (auto it = keyframe_ids.rbegin(); it != keyframe_ids.rend(); ++it)
            {
                if (!pose_store_.GetWorldPose(*it))
                {
                    continue;
                }
                if (!found || !pose_store_.IsHardFiducialKeyFrame(*it))
                {
                    selected = *it;
                    found = true;
                    if (!pose_store_.IsHardFiducialKeyFrame(*it))
                    {
                        break;
                    }
                }
            }
            if (!found)
            {
                continue;
            }

            const auto current_pose = pose_store_.GetWorldPose(selected);
            if (!current_pose)
            {
                continue;
            }

            FiducialOptimizationTask task;
            task.task_id = 9000000000ULL + (++f1i_debug_task_counter_);
            task.task_type = "FIDUCIAL_REVISIT_ERROR_DEBUG";
            task.keyframe_id = selected;
            task.submap_id = submap_id;
            task.drone_id = selected.drone_id;
            task.map_epoch = selected.map_epoch;
            task.fiducial_id = -1;
            task.estimated_world_T_kf = current_pose.value();
            task.target_world_T_kf =
                PlanarTransform(
                    f1i_debug_task_dx_,
                    f1i_debug_task_dy_,
                    f1i_debug_task_dz_,
                    f1i_debug_task_dyaw_) * current_pose.value();
            task.error_t_m =
                std::sqrt(f1i_debug_task_dx_ * f1i_debug_task_dx_ +
                          f1i_debug_task_dy_ * f1i_debug_task_dy_ +
                          f1i_debug_task_dz_ * f1i_debug_task_dz_);
            task.error_yaw_rad = std::abs(f1i_debug_task_dyaw_);
            task.error_rot_rad = std::abs(f1i_debug_task_dyaw_);
            task.threshold_t_m = fiducial_revisit_error_threshold_m_;
            task.threshold_rot_rad = fiducial_revisit_rot_threshold_rad_;
            task.threshold_yaw_rad = fiducial_revisit_yaw_threshold_rad_;
            task.arrival_id = raw_db_.GetDatabaseStats().last_arrival_id;
            task.source = "F1I_DEBUG_FORCE_BUILD";

            RCLCPP_WARN(
                get_logger(),
                "[F1I-DEBUG-TASK-CREATED] task_id=%llu drone_id=%u epoch=%llu kf=%llu dx=%.3f dy=%.3f dz=%.3f dyaw=%.3f",
                static_cast<unsigned long long>(task.task_id),
                task.drone_id,
                static_cast<unsigned long long>(task.map_epoch),
                static_cast<unsigned long long>(task.keyframe_id.local_kf_id),
                f1i_debug_task_dx_,
                f1i_debug_task_dy_,
                f1i_debug_task_dz_,
                f1i_debug_task_dyaw_);
            return task;
        }

        return std::nullopt;
    }

    void TryF1IDebugBuildAfterReplay()
    {
        if (!f1i_debug_force_task_enabled_ || f1i_debug_task_done_)
        {
            return;
        }

        const auto task = CreateF1IDebugTaskFromCurrentState();
        if (!task)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1I-GRAPH-DEBUG-BUILD-ONLY] status=waiting reason=no_anchored_keyframe");
            return;
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1I-GRAPH-DEBUG-BUILD-ONLY] status=building task_id=%llu",
            static_cast<unsigned long long>(task->task_id));
        const auto result =
            BuildAndLogPoseGraphForTask(task.value(), "replay_done_debug", true);
        f1i_debug_task_done_ = result.success;
    }

    // F1B: calcula una huella compacta de IDs locales para KFs o MPs.
    // Entrada: contenedor de mensajes con campo `id`.
    // Salida: `false` si no hay elementos; si hay, rango [min_id, max_id].
    template <typename Container>
    std::pair<bool, std::pair<uint64_t, uint64_t>> ComputeIdRange(const Container& items) const
    {
        if (items.empty())
        {
            return {false, {0, 0}};
        }

        uint64_t min_id = items.front().id;
        uint64_t max_id = items.front().id;

        // F1B: los deltas no tienen por que llegar ordenados; el rango sirve
        // como huella rapida sin asumir orden ni continuidad de IDs locales.
        for (const auto& item : items)
        {
            min_id = std::min<uint64_t>(min_id, item.id);
            max_id = std::max<uint64_t>(max_id, item.id);
        }

        return {true, {min_id, max_id}};
    }

    // F1B: formatea acumulados por dron bajo `stats_mutex_`.
    // Precondicion: el llamador ya mantiene el lock para evitar mezclar datos
    // mientras llega un `OrbMap` nuevo.
    std::string FormatPerDroneStatsLocked() const
    {
        std::ostringstream oss;
        bool first = true;

        // F1B: el `std::map` mantiene salida determinista por `drone_id`, util
        // para comparar logs reducidos entre ejecuciones.
        for (const auto& [drone_id, stats] : per_drone_stats_)
        {
            if (!first)
            {
                oss << ",";
            }
            first = false;

            oss << drone_id << ":maps=" << stats.maps
                << ":kfs=" << stats.keyframes
                << ":mps=" << stats.mappoints;

            if (stats.has_last_message)
            {
                oss << ":last_epoch=" << stats.last_epoch
                    << ":last_seq=" << stats.last_sequence;
            }
        }

        return oss.str();
    }

    void ReconcilePoseStoreAfterDelta(
        const orbslam3_multi::RawInsertResult& insert_result,
        const uint64_t arrival_id,
        const std::string& source)
    {
        if (insert_result.raw_pose_changed_keyframes.empty())
        {
            return;
        }

        const auto reconcile =
            pose_store_.ReconcileAfterRawIngestResult(
                insert_result,
                raw_db_,
                source);
        uint64_t derived_tail_rebased = 0;
        for (const auto& event : reconcile.events)
        {
            if (event.action != "rebase_derived_tail")
            {
                continue;
            }
            ++derived_tail_rebased;
            RCLCPP_WARN(
                get_logger(),
                "[F1K-DERIVED-TAIL-REBASE-AFTER-RAW] arrival_id=%llu drone_id=%u epoch=%llu kf=%llu trigger=delta action=reproject_from_accepted_anchor delta_t=%.6f delta_yaw=%.6f reason=%s",
                static_cast<unsigned long long>(arrival_id),
                event.keyframe_id.drone_id,
                static_cast<unsigned long long>(
                    event.keyframe_id.map_epoch),
                static_cast<unsigned long long>(
                    event.keyframe_id.local_kf_id),
                event.raw_delta_translation_m,
                event.raw_delta_yaw_rad,
                event.reason.c_str());
        }
        RCLCPP_WARN(
            get_logger(),
            "[F1K-POSESTORE-DELTA-RECONCILE] arrival_id=%llu raw_pose_changed=%llu derived_tail_rebased=%llu accepted_kept_or_derived_rebased=%llu failed=%llu source=%s",
            static_cast<unsigned long long>(arrival_id),
            static_cast<unsigned long long>(reconcile.raw_pose_changed),
            static_cast<unsigned long long>(derived_tail_rebased),
            static_cast<unsigned long long>(reconcile.optimized_kept),
            static_cast<unsigned long long>(reconcile.failed),
            source.c_str());
    }

    OrbMap BuildMaterialMap(
        const OrbMap& map,
        const orbslam3_multi::RawInsertResult& insert_result) const
    {
        OrbMap material = map;
        material.keyframes.clear();
        material.mappoints.clear();
        std::set<uint64_t> keyframe_ids;
        for (const auto& id : insert_result.new_keyframe_ids)
        {
            keyframe_ids.insert(id.local_kf_id);
        }
        for (const auto& id : insert_result.updated_keyframe_ids)
        {
            keyframe_ids.insert(id.local_kf_id);
        }
        std::set<uint64_t> mappoint_ids;
        for (const auto& id : insert_result.new_mappoint_ids)
        {
            mappoint_ids.insert(id.local_mp_id);
        }
        for (const auto& id : insert_result.updated_mappoint_ids)
        {
            mappoint_ids.insert(id.local_mp_id);
        }
        for (const auto& keyframe : map.keyframes)
        {
            if (keyframe_ids.find(keyframe.id) != keyframe_ids.end())
            {
                material.keyframes.push_back(keyframe);
            }
        }
        for (const auto& mappoint : map.mappoints)
        {
            if (mappoint_ids.find(mappoint.id) != mappoint_ids.end())
            {
                material.mappoints.push_back(mappoint);
            }
        }
        return material;
    }

    // F1B: callback ROS para `/dron_X/orbslam/orb_map_delta`.
    // Entrada: mensaje `OrbMap` publicado por cada wrapper y el dron inferido
    // desde el topic suscrito.
    // Efecto: actualiza contadores de observabilidad, emite logs y, desde F1C,
    // persiste el delta raw en RawMapDatabase con `arrival_id`.
    void OnOrbMapDelta(const OrbMap::SharedPtr msg, uint32_t subscribed_drone_id)
    {
        // F1B: un mensaje nulo no deberia ocurrir, pero dejarlo como error
        // explicito ayuda a distinguir fallo ROS de ausencia real de deltas.
        if (!msg)
        {
            RCLCPP_ERROR(
                get_logger(),
                "[F1B-ORBMAP-RX] subscribed_drone_id=%u error=null_message",
                subscribed_drone_id);
            return;
        }
        std::unique_lock<std::recursive_mutex> live_lock(live_state_mutex_);
        const uint32_t drone_id = msg->drone_id;
        const uint64_t map_epoch = msg->map_epoch;
        const uint64_t map_sequence = msg->map_sequence;
        const size_t keyframe_count = msg->keyframes.size();
        const size_t mappoint_count = msg->mappoints.size();
        const uint64_t arrival_id = rawdb_next_arrival_id_++;
        TryTraceFlow(
            "wrapper_server_delta",
            "receive",
            "delta",
            0,
            keyframe_count);
        const auto insert_result = raw_db_.InsertDelta(arrival_id, *msg);
        TryTraceFlow(
            "server_raw_delta",
            "commit",
            "delta",
            0,
            insert_result.new_keyframes + insert_result.updated_keyframes);
        ReconcilePoseStoreAfterDelta(
            insert_result,
            arrival_id,
            "F1C_DELTA_RECONCILE");
        TryTraceFlow(
            "raw_db_pose_new_kf",
            "commit",
            "delta_pose_reconcile",
            0,
            insert_result.new_keyframes + insert_result.updated_keyframes);

        {
            // F1B: todos los acumulados se actualizan juntos para que
            // `PublishStatsLog` nunca vea contadores parciales de un mismo
            // delta.
            std::lock_guard<std::mutex> lock(stats_mutex_);

            ++total_maps_;
            total_keyframes_ += keyframe_count;
            total_mappoints_ += mappoint_count;
            drones_seen_.insert(drone_id);
            epochs_seen_.insert({drone_id, map_epoch});

            auto& stats = per_drone_stats_[drone_id];
            ++stats.maps;
            stats.keyframes += keyframe_count;
            stats.mappoints += mappoint_count;
            stats.last_epoch = map_epoch;
            stats.last_sequence = map_sequence;
            stats.has_last_message = true;
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1C-RAWDB-DELTA-RX] arrival_id=%llu drone_id=%u epoch=%llu seq=%llu kfs=%zu mps=%zu",
            static_cast<unsigned long long>(arrival_id),
            drone_id,
            static_cast<unsigned long long>(map_epoch),
            static_cast<unsigned long long>(map_sequence),
            keyframe_count,
            mappoint_count);

        if (insert_result.new_submap)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1C-RAWDB-NEW-SUBMAP] drone_id=%u epoch=%llu arrival_id=%llu",
                drone_id,
                static_cast<unsigned long long>(map_epoch),
                static_cast<unsigned long long>(arrival_id));
        }

        LogRawInsert("F1C-RAWDB-INSERT-DELTA", arrival_id, insert_result.stats);
        if (!insert_result.has_material_changes)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1P-SNAPSHOT-NOOP] source=delta arrival_id=%llu drone_id=%u epoch=%llu received_kfs=%zu received_mps=%zu fastpath_queries=0 bow_queries=0 workers_created=0 cloud_rebuild=false",
                static_cast<unsigned long long>(arrival_id),
                drone_id,
                static_cast<unsigned long long>(map_epoch),
                keyframe_count,
                mappoint_count);
            return;
        }
        const OrbMap material_map = BuildMaterialMap(*msg, insert_result);
        UpdateScoresFromMap(material_map, arrival_id);
        TryTraceFlow(
            "raw_db_score_db",
            "commit",
            "delta_scores",
            0,
            material_map.mappoints.size());
        RegisterF1EKeyFramesFromMap(material_map);
        RequestGlobalStatePublication("delta_raw_commit");

        ImportCovisibilityFromRaw(insert_result, "delta");
        AssociateGtDebugForDelta(material_map, arrival_id);
        ProcessFiducialsForDelta(material_map, arrival_id);
        if (insert_result.has_loop_material_changes)
        {
            ScheduleLoopTasks(insert_result, "delta");
        }
        else
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1P-LOOP-NOOP-METADATA] source=delta arrival_id=%llu metadata_mps=%zu fastpath_queries=0 bow_queries=0 workers_created=0",
                static_cast<unsigned long long>(arrival_id),
                insert_result.metadata_changed_mappoint_ids.size());
        }
        HandlePoseStoreDebugAfterInsert(*msg, insert_result.stats.journal_entries);
        live_lock.unlock();

        RCLCPP_WARN(
            get_logger(),
            "[F1B-ORBMAP-RX] subscribed_drone_id=%u drone_id=%u drone_name=%s epoch=%llu seq=%llu frame_id=%s map_frame=%s kfs=%zu mps=%zu",
            subscribed_drone_id,
            drone_id,
            msg->drone_name.c_str(),
            static_cast<unsigned long long>(map_epoch),
            static_cast<unsigned long long>(map_sequence),
            msg->header.frame_id.c_str(),
            msg->map_frame.c_str(),
            keyframe_count,
            mappoint_count);

        if (drone_id != subscribed_drone_id)
        {
            // F1B: esta advertencia detecta incoherencias entre namespace ROS y
            // contenido del mensaje sin descartar el delta; en migraciones
            // posteriores puede convertirse en validacion mas estricta.
            RCLCPP_WARN(
                get_logger(),
                "[F1B-ORBMAP-RX] subscribed_drone_id=%u drone_id=%u warning=drone_id_topic_mismatch",
                subscribed_drone_id,
                drone_id);
        }

        LogKeyFrameRange(*msg);
        LogMapPointRange(*msg);
    }

    // F1B: loggea conteo y rango de IDs de KeyFrames recibidos en un delta.
    // Sirve para validar que el wrapper manda contenido sin inspeccionar todo
    // el mensaje en logs gigantes.
    void LogKeyFrameRange(const OrbMap& msg)
    {
        const auto range = ComputeIdRange(msg.keyframes);
        if (!range.first)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1B-ORBMAP-RX-KFS] drone_id=%u epoch=%llu seq=%llu count=0 first_kf=none last_kf=none",
                msg.drone_id,
                static_cast<unsigned long long>(msg.map_epoch),
                static_cast<unsigned long long>(msg.map_sequence));
            return;
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1B-ORBMAP-RX-KFS] drone_id=%u epoch=%llu seq=%llu count=%zu first_kf=%llu last_kf=%llu",
            msg.drone_id,
            static_cast<unsigned long long>(msg.map_epoch),
            static_cast<unsigned long long>(msg.map_sequence),
            msg.keyframes.size(),
            static_cast<unsigned long long>(range.second.first),
            static_cast<unsigned long long>(range.second.second));
    }

    // F1B: loggea conteo y rango de IDs de MapPoints recibidos en un delta.
    // Mantiene la misma estructura que KFs para que los patrones de reduccion
    // puedan comparar ambos tipos de datos.
    void LogMapPointRange(const OrbMap& msg)
    {
        const auto range = ComputeIdRange(msg.mappoints);
        if (!range.first)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1B-ORBMAP-RX-MPS] drone_id=%u epoch=%llu seq=%llu count=0 first_mp=none last_mp=none",
                msg.drone_id,
                static_cast<unsigned long long>(msg.map_epoch),
                static_cast<unsigned long long>(msg.map_sequence));
            return;
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1B-ORBMAP-RX-MPS] drone_id=%u epoch=%llu seq=%llu count=%zu first_mp=%llu last_mp=%llu",
            msg.drone_id,
            static_cast<unsigned long long>(msg.map_epoch),
            static_cast<unsigned long long>(msg.map_sequence),
            msg.mappoints.size(),
            static_cast<unsigned long long>(range.second.first),
            static_cast<unsigned long long>(range.second.second));
    }

    // F1B: emite una foto periodica de recepcion acumulada.
    // Efecto: no publica topics ROS; solo deja evidencia textual para decidir
    // si 1B recibe mapas de todos los drones esperados.
    void PublishStatsLog()
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);

        const std::string per_drone = FormatPerDroneStatsLocked();
        const auto raw_stats = raw_db_.GetDatabaseStats();

        RCLCPP_WARN(
            get_logger(),
            "[F1B-SERVER-STATS] rx_maps=%llu rx_kfs=%llu rx_mps=%llu drones_seen=%zu epochs_seen=%zu expected_drones=%d per_drone=\"%s\"",
            static_cast<unsigned long long>(total_maps_),
            static_cast<unsigned long long>(total_keyframes_),
            static_cast<unsigned long long>(total_mappoints_),
            drones_seen_.size(),
            epochs_seen_.size(),
            n_drones_,
            per_drone.c_str());

        RCLCPP_WARN(
            get_logger(),
            "[F1C-RAWDB-STATS] journal=%llu deltas=%llu full=%llu fiducial_observations=%llu submaps=%llu kfs=%llu mps=%llu last_arrival_id=%llu",
            static_cast<unsigned long long>(raw_stats.journal_entries),
            static_cast<unsigned long long>(raw_stats.delta_entries),
            static_cast<unsigned long long>(raw_stats.full_snapshot_entries),
            static_cast<unsigned long long>(raw_stats.fiducial_observations),
            static_cast<unsigned long long>(raw_stats.submaps),
            static_cast<unsigned long long>(raw_stats.keyframes),
            static_cast<unsigned long long>(raw_stats.mappoints),
            static_cast<unsigned long long>(raw_stats.last_arrival_id));

        const CovisibilityDatabaseStats covisibility_stats = covisibility_db_.GetStats();
        RCLCPP_WARN(
            get_logger(),
            "[F1M-COVIS-SUMMARY] confirmed_edges=%llu orbslam3_native=%llu server_loop_geometric=%llu revision=%llu",
            static_cast<unsigned long long>(covisibility_stats.confirmed_edges),
            static_cast<unsigned long long>(covisibility_stats.orbslam3_native_edges),
            static_cast<unsigned long long>(covisibility_stats.server_loop_geometric_edges),
            static_cast<unsigned long long>(covisibility_stats.revision));
    }

    void ImportCovisibilityFromRaw(
        const orbslam3_multi::RawInsertResult& insert_result,
        const std::string& trigger)
    {
        constexpr double kStoreEveryPositiveOrbWeight = 0.0;
        const auto result =
            covisibility_db_.ImportOrbslam3NativeForKeyFrames(
                raw_db_,
                insert_result.covisibility_changed_keyframe_ids,
                insert_result.arrival_id,
                kStoreEveryPositiveOrbWeight);
        RCLCPP_WARN(
            get_logger(),
            "[F1M-COVIS-IMPORT] trigger=%s arrival_id=%llu storage_min_weight=%.3f consumer_min_weight=%.3f keyframes=%llu connections=%llu added=%llu updated=%llu skipped_low_weight=%llu skipped_missing_keyframe=%llu",
            trigger.c_str(),
            static_cast<unsigned long long>(result.arrival_id),
            kStoreEveryPositiveOrbWeight,
            f1m_covisibility_min_weight_,
            static_cast<unsigned long long>(result.keyframes_examined),
            static_cast<unsigned long long>(result.connections_examined),
            static_cast<unsigned long long>(result.edges_added),
            static_cast<unsigned long long>(result.edges_updated),
            static_cast<unsigned long long>(result.edges_skipped_low_weight),
            static_cast<unsigned long long>(result.edges_skipped_missing_keyframe));
        if (result.edges_added > 0)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1M-COVIS-EDGE-ADD] arrival_id=%llu source=ORBSLAM3_NATIVE count=%llu",
                static_cast<unsigned long long>(result.arrival_id),
                static_cast<unsigned long long>(result.edges_added));
        }
        if (result.edges_updated > 0)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1M-COVIS-EDGE-UPDATE] arrival_id=%llu source=ORBSLAM3_NATIVE count=%llu",
                static_cast<unsigned long long>(result.arrival_id),
                static_cast<unsigned long long>(result.edges_updated));
        }
        TryTraceFlow(
            "raw_db_covisibility_db",
            "commit",
            trigger,
            0,
            result.edges_added + result.edges_updated);
    }


    void ProcessF1PIncrementalAndAlignedOverlap(
        const orbslam3_multi::RawInsertResult& insert_result,
        const std::string& trigger,
        const RawMapDatabase& raw_db,
        const GlobalPoseStore& pose_store,
        const CovisibilityDatabase& covisibility_db,
        const FusedLandmarkManager& fused_landmark_manager,
        std::vector<LoopVerificationResult>& verifications)
    {
        const auto started = std::chrono::steady_clock::now();
        uint64_t incremental_queries = 0;
        uint64_t incremental_pairs = 0;
        uint64_t aligned_queries = 0;
        uint64_t aligned_confirmations = 0;
        uint64_t aligned_strict_matches = 0;
        uint64_t aligned_expanded_matches = 0;
        uint64_t same_raw_ids_skipped = 0;

        RCLCPP_WARN(
            get_logger(),
            "[F1P-INCREMENTAL-INGEST] trigger=%s arrival_id=%llu new_kfs=%llu updated_kfs=%llu kfs_without_mp_changes=%llu kf_mp_deltas=%zu added_associations=%llu",
            trigger.c_str(),
            static_cast<unsigned long long>(insert_result.arrival_id),
            static_cast<unsigned long long>(insert_result.new_keyframes),
            static_cast<unsigned long long>(insert_result.updated_keyframes),
            static_cast<unsigned long long>(
                insert_result.keyframes_without_mappoint_changes),
            insert_result.keyframe_mappoint_deltas.size(),
            static_cast<unsigned long long>(
                insert_result.added_keyframe_mappoint_associations));

        for (const auto& delta : insert_result.keyframe_mappoint_deltas)
        {
            if (delta.new_keyframe)
            {
                continue;
            }
            ++incremental_queries;
            const auto incremental =
                subcloud_loop_verifier_
                    .MatchNewMapPointsAgainstConfirmedNeighbors(
                        delta.keyframe_id,
                        delta.added_mappoint_ids,
                        raw_db,
                        pose_store,
                        covisibility_db,
                        nullptr,
                        &fused_landmark_manager);
            same_raw_ids_skipped +=
                incremental.same_raw_ids_skipped;
            for (const auto& verification : incremental.confirmed)
            {
                incremental_pairs +=
                    verification.inlier_mappoint_pairs.size();
                verifications.push_back(verification);
            }
            RCLCPP_WARN(
                get_logger(),
                "[F1P-INCREMENTAL-MP] trigger=%s arrival_id=%llu drone_id=%u epoch=%llu kf=%llu new_mp_ids=%zu query_points=%llu confirmed_neighbors=%llu pairs=%llu same_raw_skipped=%llu reason=%s",
                trigger.c_str(),
                static_cast<unsigned long long>(insert_result.arrival_id),
                delta.keyframe_id.drone_id,
                static_cast<unsigned long long>(
                    delta.keyframe_id.map_epoch),
                static_cast<unsigned long long>(
                    delta.keyframe_id.local_kf_id),
                delta.added_mappoint_ids.size(),
                static_cast<unsigned long long>(
                    incremental.query_points),
                static_cast<unsigned long long>(
                    incremental.confirmed.size()),
                static_cast<unsigned long long>(
                    incremental.expanded_matches),
                static_cast<unsigned long long>(
                    incremental.same_raw_ids_skipped),
                incremental.reason.c_str());
        }

        for (const auto& keyframe_id : insert_result.new_keyframe_ids)
        {
            ++aligned_queries;
            const auto aligned =
                subcloud_loop_verifier_.FindUnknownAlignedOverlaps(
                    keyframe_id,
                    raw_db,
                    pose_store,
                    covisibility_db,
                    nullptr,
                    &fused_landmark_manager);
            aligned_confirmations += aligned.confirmed.size();
            aligned_strict_matches += aligned.strict_matches;
            aligned_expanded_matches += aligned.expanded_matches;
            same_raw_ids_skipped += aligned.same_raw_ids_skipped;
            for (const auto& verification : aligned.confirmed)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1P-ALIGNED-FASTPATH-CONFIRMED] trigger=%s arrival_id=%llu query_drone_id=%u query_epoch=%llu query_kf=%llu candidate_drone_id=%u candidate_epoch=%llu candidate_kf=%llu strict_matches=%llu expanded_matches=%llu error_t=%.6f error_yaw=%.6f mean_residual=%.6f max_residual=%.6f",
                    trigger.c_str(),
                    static_cast<unsigned long long>(
                        insert_result.arrival_id),
                    verification.query_kf_id.drone_id,
                    static_cast<unsigned long long>(
                        verification.query_kf_id.map_epoch),
                    static_cast<unsigned long long>(
                        verification.query_kf_id.local_kf_id),
                    verification.candidate_seed_kf_id.drone_id,
                    static_cast<unsigned long long>(
                        verification.candidate_seed_kf_id.map_epoch),
                    static_cast<unsigned long long>(
                        verification.candidate_seed_kf_id.local_kf_id),
                    static_cast<unsigned long long>(
                        verification.initial_matches),
                    static_cast<unsigned long long>(
                        verification.refined_matches),
                    verification.error_t,
                    verification.error_yaw,
                    verification.mean_residual,
                    verification.max_residual);
                verifications.push_back(verification);
            }
            RCLCPP_WARN(
                get_logger(),
                "[F1P-ALIGNED-FASTPATH] trigger=%s arrival_id=%llu drone_id=%u epoch=%llu kf=%llu query_points=%llu candidate_kfs_examined=%llu skipped_confirmed=%llu rejected=%llu confirmations=%zu strict_matches=%llu expanded_matches=%llu same_raw_skipped=%llu reason=%s",
                trigger.c_str(),
                static_cast<unsigned long long>(
                    insert_result.arrival_id),
                keyframe_id.drone_id,
                static_cast<unsigned long long>(
                    keyframe_id.map_epoch),
                static_cast<unsigned long long>(
                    keyframe_id.local_kf_id),
                static_cast<unsigned long long>(
                    aligned.query_points),
                static_cast<unsigned long long>(
                    aligned.candidate_keyframes_examined),
                static_cast<unsigned long long>(
                    aligned.candidate_keyframes_skipped_confirmed),
                static_cast<unsigned long long>(
                    aligned.candidate_keyframes_rejected),
                aligned.confirmed.size(),
                static_cast<unsigned long long>(
                    aligned.strict_matches),
                static_cast<unsigned long long>(
                    aligned.expanded_matches),
                static_cast<unsigned long long>(
                    aligned.same_raw_ids_skipped),
                aligned.reason.c_str());
        }

        const double elapsed_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started).count();
        RCLCPP_WARN(
            get_logger(),
            "[F1P-INCREMENTAL-FASTPATH-SUMMARY] trigger=%s arrival_id=%llu incremental_queries=%llu incremental_pairs=%llu aligned_queries=%llu aligned_confirmations=%llu strict_matches=%llu expanded_matches=%llu same_raw_skipped=%llu elapsed_ms=%.3f",
            trigger.c_str(),
            static_cast<unsigned long long>(insert_result.arrival_id),
            static_cast<unsigned long long>(incremental_queries),
            static_cast<unsigned long long>(incremental_pairs),
            static_cast<unsigned long long>(aligned_queries),
            static_cast<unsigned long long>(aligned_confirmations),
            static_cast<unsigned long long>(aligned_strict_matches),
            static_cast<unsigned long long>(aligned_expanded_matches),
            static_cast<unsigned long long>(same_raw_ids_skipped),
            elapsed_ms);
    }

    std::set<RawKeyFrameId> SelectF1PGeometryCaptureKeyFrames(
        const orbslam3_multi::RawInsertResult& insert_result) const
    {
        std::set<RawKeyFrameId> selected;
        std::set<RawKeyFrameId> queries(
            insert_result.new_keyframe_ids.begin(),
            insert_result.new_keyframe_ids.end());
        queries.insert(
            insert_result.appearance_changed_keyframe_ids.begin(),
            insert_result.appearance_changed_keyframe_ids.end());
        for (const auto& delta : insert_result.keyframe_mappoint_deltas)
        {
            queries.insert(delta.keyframe_id);
        }
        selected.insert(queries.begin(), queries.end());

        for (const auto& query_id : queries)
        {
            for (const auto& edge : covisibility_db_.GetNeighbors(query_id, 0.0))
            {
                selected.insert(edge.kf_a);
                selected.insert(edge.kf_b);
            }
            const auto query_pose = pose_store_.GetWorldPose(query_id);
            if (!query_pose || !query_pose->allFinite())
            {
                continue;
            }
            const Eigen::Vector3d query_position =
                query_pose->block<3, 1>(0, 3);
            for (const auto& submap_id : raw_db_.GetSubmapIds())
            {
                for (const auto& candidate_id :
                     raw_db_.GetKeyFrameIdsForSubmap(submap_id))
                {
                    const auto candidate_pose =
                        pose_store_.GetWorldPose(candidate_id);
                    if (candidate_pose && candidate_pose->allFinite() &&
                        (candidate_pose->block<3, 1>(0, 3) -
                         query_position).norm() <=
                            aligned_overlap_keyframe_radius_m_)
                    {
                        selected.insert(candidate_id);
                    }
                }
            }
        }
        return selected;
    }

    static orbslam3_multi::RawInsertResult FilterLoopInsertForQuery(
        const orbslam3_multi::RawInsertResult& source,
        const RawKeyFrameId& query_id)
    {
        auto filtered = source;
        auto keep_only_query = [&query_id](std::vector<RawKeyFrameId>& ids)
        {
            ids.erase(
                std::remove_if(
                    ids.begin(), ids.end(),
                    [&query_id](const RawKeyFrameId& id)
                    {
                        return !(id == query_id);
                    }),
                ids.end());
        };
        keep_only_query(filtered.touched_keyframe_ids);
        keep_only_query(filtered.covisibility_changed_keyframe_ids);
        keep_only_query(filtered.new_keyframe_ids);
        keep_only_query(filtered.updated_keyframe_ids);
        keep_only_query(filtered.appearance_changed_keyframe_ids);
        keep_only_query(filtered.geometry_changed_keyframe_ids);
        filtered.keyframe_mappoint_deltas.erase(
            std::remove_if(
                filtered.keyframe_mappoint_deltas.begin(),
                filtered.keyframe_mappoint_deltas.end(),
                [&query_id](const auto& delta)
                {
                    return !(delta.keyframe_id == query_id);
                }),
            filtered.keyframe_mappoint_deltas.end());
        filtered.new_keyframes = filtered.new_keyframe_ids.size();
        filtered.updated_keyframes = filtered.updated_keyframe_ids.size();
        filtered.added_keyframe_mappoint_associations = 0;
        for (const auto& delta : filtered.keyframe_mappoint_deltas)
        {
            filtered.added_keyframe_mappoint_associations +=
                delta.added_mappoint_ids.size();
        }
        return filtered;
    }

    size_t PendingLoopTaskCount() const
    {
        std::lock_guard<std::mutex> lock(secondary_task_mutex_);
        return loop_task_queue_.size();
    }

    void ScheduleLoopTasks(
        const orbslam3_multi::RawInsertResult& insert_result,
        const std::string& trigger)
    {
        const std::set<RawKeyFrameId> new_queries(
            insert_result.new_keyframe_ids.begin(),
            insert_result.new_keyframe_ids.end());
        std::set<RawKeyFrameId> queries = new_queries;
        queries.insert(
            insert_result.appearance_changed_keyframe_ids.begin(),
            insert_result.appearance_changed_keyframe_ids.end());

        uint64_t enqueued = 0;
        uint64_t coalesced = 0;
        uint64_t dropped = 0;
        uint64_t evicted_refresh = 0;
        {
            std::lock_guard<std::mutex> lock(secondary_task_mutex_);
            const size_t capacity = static_cast<size_t>(
                std::max(1, loop_task_max_pending_));
            for (const auto& query_id : queries)
            {
                LoopTask task;
                task.task_id = kLoopTaskIdPrefix |
                    next_loop_task_id_.fetch_add(1);
                task.query_kf_id = query_id;
                task.insert_result =
                    FilterLoopInsertForQuery(insert_result, query_id);
                task.admitted_revision =
                    raw_db_.GetKeyFrameRevision(query_id);
                task.trigger = trigger;
                task.material_new = new_queries.count(query_id) > 0;

                const auto existing = std::find_if(
                    loop_task_queue_.begin(),
                    loop_task_queue_.end(),
                    [&query_id](const LoopTask& pending)
                    {
                        return pending.query_kf_id == query_id;
                    });
                if (existing != loop_task_queue_.end())
                {
                    task.material_new =
                        task.material_new || existing->material_new;
                    *existing = std::move(task);
                    ++coalesced;
                    continue;
                }
                if (loop_task_queue_.size() >= capacity)
                {
                    const auto refresh = std::find_if(
                        loop_task_queue_.rbegin(),
                        loop_task_queue_.rend(),
                        [](const LoopTask& pending)
                        {
                            return !pending.material_new;
                        });
                    if (!task.material_new || refresh == loop_task_queue_.rend())
                    {
                        ++dropped;
                        continue;
                    }
                    loop_task_keys_.erase(refresh->query_kf_id);
                    loop_task_queue_.erase(std::next(refresh).base());
                    ++evicted_refresh;
                }
                loop_task_keys_.insert(query_id);
                loop_task_queue_.push_back(std::move(task));
                ++enqueued;
            }
        }
        if (enqueued > 0)
        {
            secondary_task_condition_.notify_one();
        }
        TryTraceFlow(
            "raw_db_secondary_queue",
            "enqueue",
            trigger,
            0,
            enqueued);
        RCLCPP_WARN(
            get_logger(),
            "[F1K-LOOP-TASK-ADMISSION] arrival_id=%llu requested=%zu new=%zu appearance=%zu enqueued=%llu coalesced=%llu dropped=%llu evicted_refresh=%llu queued_loop=%zu queued_fiducial=%llu policy=bounded_nonblocking",
            static_cast<unsigned long long>(insert_result.arrival_id),
            queries.size(),
            new_queries.size(),
            insert_result.appearance_changed_keyframe_ids.size(),
            static_cast<unsigned long long>(enqueued),
            static_cast<unsigned long long>(coalesced),
            static_cast<unsigned long long>(dropped),
            static_cast<unsigned long long>(evicted_refresh),
            PendingLoopTaskCount(),
            static_cast<unsigned long long>(optimization_queued_jobs_.load()));
        UpdateMappingBackpressure("loop_task_admission");
    }

    bool CommitComputedLoopCandidate(
        const LoopTask& task,
        const ComputedLoopCandidate& computed)
    {
        std::lock_guard<std::recursive_mutex> lock(live_state_mutex_);
        for (const auto& [id, revision] : computed.captured_raw_revisions)
        {
            if (!orbslam3_multi::SameGeometryRevision(
                    raw_db_.GetKeyFrameRevision(id), revision))
            {
                return false;
            }
        }
        for (const auto& [id, pose] : computed.captured_world_poses)
        {
            if (!SameCapturedPose(id, pose))
            {
                return false;
            }
        }

        GlobalPoseStore candidate_pose_store = pose_store_;
        CovisibilityDatabase candidate_covisibility = covisibility_db_;
        FusedLandmarkManager candidate_fused = fused_landmark_manager_;
        const auto decision = loop_decision_manager_.Process(
            computed.verification,
            task.insert_result.arrival_id,
            raw_db_,
            candidate_pose_store,
            score_manager_,
            candidate_covisibility,
            candidate_fused);
        const auto query_revision =
            raw_db_.GetKeyFrameRevision(computed.verification.query_kf_id);
        const auto candidate_revision = raw_db_.GetKeyFrameRevision(
            computed.verification.candidate_seed_kf_id);

        if (computed.verification.decision ==
            orbslam3_multi::LoopGeometryDecision::FusionCandidate)
        {
            if (!decision.handled)
            {
                return true;
            }
            covisibility_db_ = std::move(candidate_covisibility);
            fused_landmark_manager_ = std::move(candidate_fused);
            if (decision.covisibility_edge_changed)
            {
                TryTraceFlow(
                    "loop_decision_covis_db",
                    "commit",
                    "fusion_geometry",
                    task.task_id,
                    1);
            }
            loop_pair_attempt_db_.RecordState(
                computed.verification.query_kf_id,
                query_revision,
                computed.verification.candidate_seed_kf_id,
                candidate_revision,
                task.insert_result.arrival_id,
                LoopPairState::Fused,
                decision.reason,
                &computed.verification);
            RequestGlobalStatePublication("loop_fusion_commit");
            TryTraceFlow(
                "loop_decision_fused_db",
                "commit",
                "fusion",
                task.task_id,
                decision.fusion.pair_results.size());
            RCLCPP_WARN(
                get_logger(),
                "[F1K-LOOP-TASK-COMMIT] task_id=%llu branch=FUSION committed=true pose_changed=false fused_pairs=%zu",
                static_cast<unsigned long long>(task.task_id),
                decision.fusion.pair_results.size());
            return true;
        }

        if (computed.verification.decision !=
            orbslam3_multi::LoopGeometryDecision::LoopOptimizationCandidate)
        {
            return true;
        }

        if (!computed.optimization_computed ||
            !computed.graph.success ||
            !computed.optimization.success ||
            !computed.optimization.useful)
        {
            if (decision.handled)
            {
                covisibility_db_ = std::move(candidate_covisibility);
                TryTraceFlow(
                    "loop_decision_covis_db",
                    "commit",
                    "high_error_geometry",
                    task.task_id,
                    1);
            }
            loop_pair_attempt_db_.RecordState(
                computed.verification.query_kf_id,
                query_revision,
                computed.verification.candidate_seed_kf_id,
                candidate_revision,
                task.insert_result.arrival_id,
                LoopPairState::ConfirmedHighError,
                computed.graph.success
                    ? computed.optimization.decision_reason
                    : computed.graph.reason,
                &computed.verification);
            RCLCPP_WARN(
                get_logger(),
                "[F1Q-LOOP-OPT-COMMIT] task_id=%llu committed=false reason=%s",
                static_cast<unsigned long long>(task.task_id),
                computed.graph.success
                    ? computed.optimization.decision_reason.c_str()
                    : computed.graph.reason.c_str());
            return true;
        }

        const auto affected = CollectApplyAffectedKeyFrames(
            computed.optimization,
            computed.graph.problem,
            raw_db_);
        const auto backup = candidate_pose_store.CreateApplyBackup(
            task.task_id, affected);
        if (!backup.success)
        {
            return true;
        }
        const auto apply = optimization_manager_.ApplyCandidateResult(
            computed.optimization,
            computed.graph.problem,
            raw_db_,
            candidate_pose_store,
            false);
        const auto validation = optimization_manager_.ValidatePostApply(
            computed.optimization,
            computed.graph.problem,
            apply,
            candidate_pose_store,
            false);
        if (!apply.applied ||
            validation.decision != PostApplyDecision::Accept ||
            !candidate_pose_store.ConfirmApply(task.task_id))
        {
            candidate_pose_store.RestoreApplyBackup(task.task_id);
            loop_pair_attempt_db_.RecordState(
                computed.verification.query_kf_id,
                query_revision,
                computed.verification.candidate_seed_kf_id,
                candidate_revision,
                task.insert_result.arrival_id,
                LoopPairState::ConfirmedHighError,
                apply.applied ? validation.reason : apply.reason,
                &computed.verification);
            RCLCPP_WARN(
                get_logger(),
                "[F1Q-LOOP-OPT-COMMIT] task_id=%llu committed=false apply=%s validation=%s reason=%s",
                static_cast<unsigned long long>(task.task_id),
                apply.applied ? "true" : "false",
                orbslam3_multi::ToString(validation.decision),
                apply.applied ? validation.reason.c_str() : apply.reason.c_str());
            return true;
        }

        auto fusion_verification = computed.verification;
        fusion_verification.decision =
            orbslam3_multi::LoopGeometryDecision::FusionCandidate;
        const auto fusion = loop_decision_manager_.Process(
            fusion_verification,
            task.insert_result.arrival_id,
            raw_db_,
            candidate_pose_store,
            score_manager_,
            candidate_covisibility,
            candidate_fused);
        pose_store_ = std::move(candidate_pose_store);
        covisibility_db_ = std::move(candidate_covisibility);
        fused_landmark_manager_ = std::move(candidate_fused);
        TryTraceFlow(
            "loop_decision_covis_db",
            "commit",
            "optimized_loop_geometry",
            task.task_id,
            1);
        TryTraceFlow(
            "loop_decision_fused_db",
            "commit",
            "optimized_loop_fusion",
            task.task_id,
            fusion.fusion.pair_results.size());
        loop_pair_attempt_db_.RecordState(
            computed.verification.query_kf_id,
            query_revision,
            computed.verification.candidate_seed_kf_id,
            candidate_revision,
            task.insert_result.arrival_id,
            LoopPairState::Fused,
            fusion.handled
                ? "loop_optimization_accepted_and_fused"
                : "loop_optimization_accepted",
            &computed.verification);
        RequestGlobalStatePublication("loop_optimization_commit");
        TryTraceFlow(
            "optimizer_pose_db",
            "commit",
            "loop_optimization",
            task.task_id,
            apply.optimized_kfs + apply.propagated_kfs);
        RCLCPP_WARN(
            get_logger(),
            "[F1Q-LOOP-OPT-COMMIT] task_id=%llu committed=true optimized_kfs=%llu propagated_kfs=%llu fixed_kfs=%llu hard_fixed_moved=%s max_delta_t=%.6f max_delta_yaw=%.6f fused_pairs=%zu publication_waited=false",
            static_cast<unsigned long long>(task.task_id),
            static_cast<unsigned long long>(apply.optimized_kfs),
            static_cast<unsigned long long>(apply.propagated_kfs),
            static_cast<unsigned long long>(apply.fixed_kfs),
            apply.hard_fixed_moved ? "true" : "false",
            apply.max_delta_t,
            apply.max_delta_yaw,
            fusion.fusion.pair_results.size());
        return true;
    }

    uint64_t CountPriorLoopOptimizationSupports(
        const LoopVerificationResult& verification,
        const RawMapDatabase& raw_db,
        const CovisibilityDatabase& covisibility_db) const
    {
        std::vector<RawKeyFrameId> window = raw_db.GetKeyFrameIdsForSubmap(
            verification.query_submap_id);
        if (!(verification.query_submap_id ==
              verification.candidate_submap_id))
        {
            const auto candidate_ids = raw_db.GetKeyFrameIdsForSubmap(
                verification.candidate_submap_id);
            window.insert(
                window.end(), candidate_ids.begin(), candidate_ids.end());
        }

        const auto endpoint_submap = [](const RawKeyFrameId& id)
        {
            return RawSubmapId{id.drone_id, id.map_epoch};
        };
        uint64_t supports = 0;
        for (const auto& edge :
             covisibility_db.GetEdgesForWindow(window, 0.0))
        {
            if (edge.source !=
                    orbslam3_multi::CovisibilityEdgeSource::ServerLoopGeometric ||
                !edge.geometry_confirmed ||
                !CovisibilityDatabase::IsStrongEdge(
                    edge, CurrentCovisibilityStrengthConfig()))
            {
                continue;
            }
            const auto submap_a = endpoint_submap(edge.kf_a);
            const auto submap_b = endpoint_submap(edge.kf_b);
            const bool same_direction =
                submap_a == verification.query_submap_id &&
                submap_b == verification.candidate_submap_id;
            const bool reverse_direction =
                submap_b == verification.query_submap_id &&
                submap_a == verification.candidate_submap_id;
            if (same_direction || reverse_direction)
            {
                ++supports;
            }
        }
        return supports;
    }

    void RunLoopTask(const LoopTask& task)
    {
        const auto started = std::chrono::steady_clock::now();
        RawMapDatabase raw_snapshot;
        GlobalPoseStore pose_snapshot;
        CovisibilityDatabase covisibility_snapshot;
        FusedLandmarkManager fused_snapshot;
        LandmarkScoreManager score_snapshot;
        std::map<RawKeyFrameId, uint64_t> bow_processed_snapshot;
        {
            std::lock_guard<std::recursive_mutex> lock(live_state_mutex_);
            raw_snapshot = raw_db_.CreateStateSnapshot();
            pose_snapshot = pose_store_;
            covisibility_snapshot = covisibility_db_;
            fused_snapshot = fused_landmark_manager_;
            score_snapshot = score_manager_;
            bow_processed_snapshot = bow_processed_appearance_revision_;
        }

        std::vector<LoopVerificationResult> fastpath_verifications;
        ProcessF1PIncrementalAndAlignedOverlap(
            task.insert_result,
            task.trigger,
            raw_snapshot,
            pose_snapshot,
            covisibility_snapshot,
            fused_snapshot,
            fastpath_verifications);
        std::vector<LoopCandidateResult> candidate_results;
        DispatchLoopDetector(
            task.insert_result,
            task.trigger,
            raw_snapshot,
            pose_snapshot,
            covisibility_snapshot,
            bow_processed_snapshot,
            candidate_results);
        TryTraceFlow(
            "loop_detector_loop_verifier",
            "candidates",
            "bow_complete",
            task.task_id,
            candidate_results.size());

        std::vector<ComputedLoopCandidate> computed;
        for (const auto& verification : fastpath_verifications)
        {
            ComputedLoopCandidate item;
            item.verification = verification;
            item.candidate.query_kf_id = verification.query_kf_id;
            item.candidate.candidate_kf_id =
                verification.candidate_seed_kf_id;
            item.captured_raw_revisions[verification.query_kf_id] =
                raw_snapshot.GetKeyFrameRevision(verification.query_kf_id);
            item.captured_raw_revisions[verification.candidate_seed_kf_id] =
                raw_snapshot.GetKeyFrameRevision(
                    verification.candidate_seed_kf_id);
            const auto query_pose =
                pose_snapshot.GetWorldPose(verification.query_kf_id);
            const auto candidate_pose = pose_snapshot.GetWorldPose(
                verification.candidate_seed_kf_id);
            if (query_pose)
            {
                item.captured_world_poses[verification.query_kf_id] =
                    query_pose.value();
            }
            if (candidate_pose)
            {
                item.captured_world_poses[
                    verification.candidate_seed_kf_id] =
                    candidate_pose.value();
            }
            computed.push_back(std::move(item));
        }

        for (const auto& result : candidate_results)
        {
            size_t admitted = 0;
            for (const auto& candidate : result.candidates)
            {
                if (admitted >= static_cast<size_t>(
                        std::max(0, loop_verify_max_candidates_per_query_)))
                {
                    break;
                }
                ++admitted;
                if (covisibility_snapshot.HasStrongEdge(
                        candidate.query_kf_id,
                        candidate.candidate_kf_id,
                        CurrentCovisibilityStrengthConfig()))
                {
                    continue;
                }
                ComputedLoopCandidate item;
                item.candidate = candidate;
                const auto captured = subcloud_loop_verifier_.CaptureCandidate(
                    candidate,
                    raw_snapshot,
                    pose_snapshot,
                    &covisibility_snapshot,
                    &score_snapshot);
                for (const auto& [id, pose] : captured.world_T_keyframes)
                {
                    item.captured_world_poses[id] = pose;
                    item.captured_raw_revisions[id] =
                        raw_snapshot.GetKeyFrameRevision(id);
                }
                const auto prepared =
                    subcloud_loop_verifier_.PrepareCapturedCandidate(captured);
                item.verification =
                    subcloud_loop_verifier_.VerifyPreparedCandidate(prepared);
                if (item.candidate.near_same_submap &&
                    item.verification.decision ==
                        orbslam3_multi::LoopGeometryDecision::
                            LoopOptimizationCandidate)
                {
                    item.verification.decision =
                        orbslam3_multi::LoopGeometryDecision::
                            HoldInsufficientEvidence;
                    item.verification.reason =
                        "near_same_submap_optimization_suppressed";
                    RCLCPP_WARN(
                        get_logger(),
                        "[F1Q-LOOP-OPT-SUPPRESSED] task_id=%llu query=%u:%llu:%llu candidate=%u:%llu:%llu kf_gap=%llu min_gap=%d reason=near_same_submap",
                        static_cast<unsigned long long>(task.task_id),
                        item.candidate.query_kf_id.drone_id,
                        static_cast<unsigned long long>(
                            item.candidate.query_kf_id.map_epoch),
                        static_cast<unsigned long long>(
                            item.candidate.query_kf_id.local_kf_id),
                        item.candidate.candidate_kf_id.drone_id,
                        static_cast<unsigned long long>(
                            item.candidate.candidate_kf_id.map_epoch),
                        static_cast<unsigned long long>(
                            item.candidate.candidate_kf_id.local_kf_id),
                        static_cast<unsigned long long>(item.candidate.kf_gap),
                        loop_bow_min_kf_gap_same_submap_);
                }
                TryTraceFlow(
                    "loop_verifier_loop_decision",
                    "result",
                    orbslam3_multi::ToString(item.verification.decision),
                    task.task_id,
                    item.verification.ransac_inliers);
                if (item.verification.decision ==
                    orbslam3_multi::LoopGeometryDecision::LoopOptimizationCandidate)
                {
                    item.prior_loop_supports =
                        CountPriorLoopOptimizationSupports(
                            item.verification,
                            raw_snapshot,
                            covisibility_snapshot);
                    LoopOptimizationTask optimization_task;
                    optimization_task.task_id = task.task_id;
                    optimization_task.arrival_id =
                        task.insert_result.arrival_id;
                    optimization_task.source = task.trigger;
                    optimization_task.verification = item.verification;
                    if (item.prior_loop_supports >= static_cast<uint64_t>(
                            loop_optimization_min_prior_supports_))
                    {
                        TryTraceFlow(
                            "loop_decision_pose_graph",
                            "start",
                            "loop_relative_graph",
                            task.task_id,
                            item.prior_loop_supports);
                        item.graph = pose_graph_builder_.BuildForLoopTask(
                            optimization_task,
                            raw_snapshot,
                            pose_snapshot,
                            &covisibility_snapshot);
                        if (item.graph.success)
                        {
                            TryTraceFlow(
                                "pose_graph_optimizer",
                                "start",
                                "loop_solver",
                                task.task_id,
                                item.graph.problem.vertices.size());
                            item.optimization = optimization_manager_.RunDryRun(
                                item.graph.problem,
                                raw_snapshot,
                                pose_snapshot);
                        }
                    }
                    else
                    {
                        item.graph.reason =
                            "insufficient_independent_loop_support";
                    }
                    item.optimization_computed = true;
                    const auto& query = item.verification.query_kf_id;
                    const auto& candidate =
                        item.verification.candidate_seed_kf_id;
                    RCLCPP_WARN(
                        get_logger(),
                        "[F1Q-LOOP-OPT-COMPUTE] task_id=%llu query=%u:%llu:%llu candidate=%u:%llu:%llu error_t=%.6f error_yaw=%.6f inliers=%llu graph=%s vertices=%zu edges=%zu dryrun=%s useful=%s prior_supports=%llu required_supports=%d reason=%s",
                        static_cast<unsigned long long>(task.task_id),
                        query.drone_id,
                        static_cast<unsigned long long>(query.map_epoch),
                        static_cast<unsigned long long>(query.local_kf_id),
                        candidate.drone_id,
                        static_cast<unsigned long long>(candidate.map_epoch),
                        static_cast<unsigned long long>(candidate.local_kf_id),
                        item.verification.error_t,
                        item.verification.error_yaw,
                        static_cast<unsigned long long>(
                            item.verification.ransac_inliers),
                        item.graph.success ? "true" : "false",
                        item.graph.problem.vertices.size(),
                        item.graph.problem.edges.size(),
                        item.optimization.success ? "true" : "false",
                        item.optimization.useful ? "true" : "false",
                        static_cast<unsigned long long>(
                            item.prior_loop_supports),
                        loop_optimization_min_prior_supports_,
                        item.graph.success
                            ? item.optimization.decision_reason.c_str()
                            : item.graph.reason.c_str());
                }
                computed.push_back(std::move(item));
            }
        }

        bool stale = false;
        uint64_t committed = 0;
        for (const auto& item : computed)
        {
            if (CommitComputedLoopCandidate(task, item))
            {
                ++committed;
            }
            else
            {
                stale = true;
            }
        }
        {
            std::lock_guard<std::recursive_mutex> lock(live_state_mutex_);
            bow_processed_appearance_revision_[task.query_kf_id] =
                raw_snapshot.GetKeyFrameRevision(
                    task.query_kf_id).appearance;
        }
        if (stale && task.retry_count == 0)
        {
            LoopTask retry = task;
            retry.retry_count = 1;
            retry.admitted_revision =
                raw_snapshot.GetKeyFrameRevision(task.query_kf_id);
            {
                std::lock_guard<std::mutex> lock(secondary_task_mutex_);
                if (loop_task_keys_.insert(task.query_kf_id).second)
                {
                    loop_task_queue_.push_back(std::move(retry));
                }
            }
            secondary_task_condition_.notify_one();
        }
        ++loop_jobs_committed_;
        RCLCPP_WARN(
            get_logger(),
            "[F1K-SECONDARY-TASK-END] task_id=%llu type=LOOP query=%u:%llu:%llu bow_results=%zu computed=%zu committed=%llu stale=%s retry=%u duration_ms=%.3f publication_waited=false",
            static_cast<unsigned long long>(task.task_id),
            task.query_kf_id.drone_id,
            static_cast<unsigned long long>(task.query_kf_id.map_epoch),
            static_cast<unsigned long long>(task.query_kf_id.local_kf_id),
            candidate_results.size(),
            computed.size(),
            static_cast<unsigned long long>(committed),
            stale ? "true" : "false",
            task.retry_count,
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started).count());
    }


    void PublishMappingBackpressure(bool active, const std::string& reason)
    {
        const bool previous = mapping_backpressure_active_.exchange(active);
        if (previous == active && reason != "startup")
        {
            return;
        }

        std_msgs::msg::Bool msg;
        msg.data = active;
        mapping_backpressure_pub_->publish(msg);

        size_t queued = 0;
        size_t active_jobs = 0;
        size_t completed_waiting = 0;
        size_t committing_jobs = 0;
        size_t throttled_ready = 0;
        size_t coalesced_ready = 0;
        {
            std::lock_guard<std::mutex> lock(secondary_task_mutex_);
            queued = loop_task_queue_.size();
            active_jobs = secondary_worker_active_.load() ? 1U : 0U;
        }
        RCLCPP_WARN(
            get_logger(),
            "[F1K-BACKPRESSURE] active=%s previous=%s loop_backpressure_active=%s fiducial_queued=%llu fiducial_active=%llu publication_waits=0 secondary_jobs_in_flight=%llu loop_queued=%zu secondary_active=%zu completed_waiting=%zu committing_jobs=%zu throttled_ready=%zu coalesced_ready=%zu mapping_load=%zu high_watermark=%d low_watermark=%d reason=%s",
            active ? "true" : "false",
            previous ? "true" : "false",
            loop_backpressure_latched_.load() ? "true" : "false",
            static_cast<unsigned long long>(optimization_queued_jobs_.load()),
            static_cast<unsigned long long>(optimization_active_workers_.load()),
            static_cast<unsigned long long>(optimization_jobs_in_flight_.load()),
            queued,
            active_jobs,
            completed_waiting,
            committing_jobs,
            throttled_ready,
            coalesced_ready,
            queued + active_jobs + completed_waiting + committing_jobs +
                throttled_ready + coalesced_ready,
            loop_backlog_high_watermark_,
            loop_backlog_low_watermark_,
            reason.c_str());
    }

    void UpdateMappingBackpressure(const std::string& reason)
    {
        size_t backlog = 0;
        {
            std::lock_guard<std::mutex> lock(secondary_task_mutex_);
            backlog = loop_task_queue_.size() +
                (secondary_worker_active_.load() ? 1U : 0U);
            loop_peak_backlog_ = std::max(loop_peak_backlog_, backlog);
        }

        const bool loop_active = loop_backpressure_latched_.load();
        if (!loop_active &&
            backlog >= static_cast<size_t>(loop_backlog_high_watermark_))
        {
            loop_backpressure_latched_.store(true);
        }
        else if (loop_active &&
                 backlog <= static_cast<size_t>(loop_backlog_low_watermark_))
        {
            loop_backpressure_latched_.store(false);
        }

        // La profundidad secundaria es solo una metrica. El productor de
        // deltas/snapshots y el escenario no esperan a que el worker drene.
        PublishMappingBackpressure(false, reason);
    }


    CovisibilityStrengthConfig CurrentCovisibilityStrengthConfig() const
    {
        CovisibilityStrengthConfig config;
        config.min_support = static_cast<uint64_t>(
            aligned_overlap_strict_min_matches_);
        config.min_shared_ratio = aligned_overlap_strict_min_match_ratio_;
        config.min_image_bins = static_cast<uint64_t>(
            aligned_overlap_strict_min_image_bins_);
        config.min_spatial_coverage_ratio =
            aligned_overlap_strict_min_3d_span_ratio_;
        return config;
    }


    bool SameCapturedPose(
        const RawKeyFrameId& id,
        const Eigen::Matrix4d& captured) const
    {
        const auto current = pose_store_.GetWorldPose(id);
        return current && current->allFinite() &&
               captured.allFinite() && current->isApprox(captured, 1e-9);
    }


    void DispatchLoopDetector(
        const orbslam3_multi::RawInsertResult& insert_result,
        const std::string& trigger,
        const RawMapDatabase& raw_db,
        const GlobalPoseStore& pose_store,
        const CovisibilityDatabase& covisibility_db,
        const std::map<RawKeyFrameId, uint64_t>& processed_appearance_revisions,
        std::vector<LoopCandidateResult>& candidate_results)
    {
        std::set<RawKeyFrameId> query_ids(
            insert_result.new_keyframe_ids.begin(),
            insert_result.new_keyframe_ids.end());
        query_ids.insert(
            insert_result.appearance_changed_keyframe_ids.begin(),
            insert_result.appearance_changed_keyframe_ids.end());
        for (const auto& keyframe_id : query_ids)
        {
            const auto appearance_revision =
                raw_db.GetKeyFrameRevision(keyframe_id).appearance;
            const auto processed_it =
                processed_appearance_revisions.find(keyframe_id);
            if (processed_it != processed_appearance_revisions.end() &&
                processed_it->second == appearance_revision)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1P-BOW-REVISION-SKIP] trigger=%s arrival_id=%llu drone_id=%u epoch=%llu kf=%llu appearance_revision=%llu",
                    trigger.c_str(),
                    static_cast<unsigned long long>(insert_result.arrival_id),
                    keyframe_id.drone_id,
                    static_cast<unsigned long long>(keyframe_id.map_epoch),
                    static_cast<unsigned long long>(keyframe_id.local_kf_id),
                    static_cast<unsigned long long>(appearance_revision));
                continue;
            }
            RCLCPP_WARN(
                get_logger(),
                "[F1N-LOOP-NEW-KF-DISPATCH] trigger=%s arrival_id=%llu drone_id=%u epoch=%llu kf=%llu",
                trigger.c_str(),
                static_cast<unsigned long long>(insert_result.arrival_id),
                keyframe_id.drone_id,
                static_cast<unsigned long long>(keyframe_id.map_epoch),
                static_cast<unsigned long long>(keyframe_id.local_kf_id));
            RCLCPP_WARN(
                get_logger(),
                "[F1N-SERVER-LOOP-DETECTOR-CALL] trigger=%s arrival_id=%llu drone_id=%u epoch=%llu kf=%llu",
                trigger.c_str(),
                static_cast<unsigned long long>(insert_result.arrival_id),
                keyframe_id.drone_id,
                static_cast<unsigned long long>(keyframe_id.map_epoch),
                static_cast<unsigned long long>(keyframe_id.local_kf_id));
            const LoopCandidateResult result = loop_detector_.ProcessNewKeyFrame(
                keyframe_id,
                raw_db,
                &pose_store,
                covisibility_db);
            RCLCPP_WARN(
                get_logger(),
                "[F1N-LOOP-KF-QUERY] drone_id=%u epoch=%llu kf=%llu processed=%s reason=%s indexed_kfs=%llu compared_kfs=%llu raw_candidates=%llu filtered_candidates=%llu skipped_confirmed_covisibility=%llu skipped_noncausal_same_submap=%llu near_same_submap_candidates=%llu",
                keyframe_id.drone_id,
                static_cast<unsigned long long>(keyframe_id.map_epoch),
                static_cast<unsigned long long>(keyframe_id.local_kf_id),
                result.processed ? "true" : "false",
                result.reason.c_str(),
                static_cast<unsigned long long>(result.indexed_keyframes),
                static_cast<unsigned long long>(result.compared_keyframes),
                static_cast<unsigned long long>(result.candidates_raw),
                static_cast<unsigned long long>(result.candidates_after_filter),
                static_cast<unsigned long long>(result.skipped_confirmed_covisibility),
                static_cast<unsigned long long>(
                    result.skipped_noncausal_same_submap),
                static_cast<unsigned long long>(result.near_same_submap_candidates));
            RCLCPP_WARN(
                get_logger(),
                "[F1N-LOOP-BOW-SEARCH] trigger=%s drone_id=%u epoch=%llu kf=%llu query_has_bow=%s query_bow_words=%llu indexed_kfs=%llu compared_kfs=%llu raw_candidates=%llu",
                trigger.c_str(),
                keyframe_id.drone_id,
                static_cast<unsigned long long>(keyframe_id.map_epoch),
                static_cast<unsigned long long>(keyframe_id.local_kf_id),
                result.query_has_bow ? "true" : "false",
                static_cast<unsigned long long>(result.query_bow_words),
                static_cast<unsigned long long>(result.indexed_keyframes),
                static_cast<unsigned long long>(result.compared_keyframes),
                static_cast<unsigned long long>(result.candidates_raw));
            for (const auto& filter : result.filter_events)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1N-LOOP-CANDIDATE-FILTER] query_drone_id=%u query_epoch=%llu query_kf=%llu candidate_drone_id=%u candidate_epoch=%llu candidate_kf=%llu reason=%s bow_score=%.6f same_submap=%s kf_gap=%llu confirmed_covisibility=%s",
                    keyframe_id.drone_id,
                    static_cast<unsigned long long>(keyframe_id.map_epoch),
                    static_cast<unsigned long long>(keyframe_id.local_kf_id),
                    filter.candidate_kf_id.drone_id,
                    static_cast<unsigned long long>(filter.candidate_kf_id.map_epoch),
                    static_cast<unsigned long long>(filter.candidate_kf_id.local_kf_id),
                    filter.reason.c_str(),
                    filter.bow_score,
                    filter.same_submap ? "true" : "false",
                    static_cast<unsigned long long>(filter.kf_gap),
                    filter.already_confirmed_covisibility ? "true" : "false");
            }
            if (result.skipped_confirmed_covisibility > 0)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1N-BOW-SKIP-CONFIRMED-COVIS] query_drone_id=%u query_epoch=%llu query_kf=%llu skipped=%llu",
                    keyframe_id.drone_id,
                    static_cast<unsigned long long>(keyframe_id.map_epoch),
                    static_cast<unsigned long long>(keyframe_id.local_kf_id),
                    static_cast<unsigned long long>(result.skipped_confirmed_covisibility));
            }
            RCLCPP_WARN(
                get_logger(),
                "[F1N-LOOP-CANDIDATE-SUMMARY] trigger=%s drone_id=%u epoch=%llu kf=%llu processed=%s reason=%s raw_candidates=%llu filtered_candidates=%llu rejected_by_filter=%llu skipped_noncausal_same_submap=%llu near_same_submap_candidates=%llu best_rank=%llu",
                trigger.c_str(),
                keyframe_id.drone_id,
                static_cast<unsigned long long>(keyframe_id.map_epoch),
                static_cast<unsigned long long>(keyframe_id.local_kf_id),
                result.processed ? "true" : "false",
                result.reason.c_str(),
                static_cast<unsigned long long>(result.candidates_raw),
                static_cast<unsigned long long>(result.candidates_after_filter),
                static_cast<unsigned long long>(result.candidates_rejected_by_filter),
                static_cast<unsigned long long>(
                    result.skipped_noncausal_same_submap),
                static_cast<unsigned long long>(result.near_same_submap_candidates),
                static_cast<unsigned long long>(
                    result.best_candidate ? result.best_candidate->rank : 0));
            if (result.candidates.empty())
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1N-LOOP-NO-CANDIDATES] trigger=%s drone_id=%u epoch=%llu kf=%llu reason=%s raw_candidates=%llu rejected_by_filter=%llu",
                    trigger.c_str(),
                    keyframe_id.drone_id,
                    static_cast<unsigned long long>(keyframe_id.map_epoch),
                    static_cast<unsigned long long>(keyframe_id.local_kf_id),
                    result.reason.c_str(),
                    static_cast<unsigned long long>(result.candidates_raw),
                    static_cast<unsigned long long>(result.candidates_rejected_by_filter));
            }
            else
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1N-LOOP-BOW-CANDIDATES] trigger=%s drone_id=%u epoch=%llu kf=%llu count=%llu",
                    trigger.c_str(),
                    keyframe_id.drone_id,
                    static_cast<unsigned long long>(keyframe_id.map_epoch),
                    static_cast<unsigned long long>(keyframe_id.local_kf_id),
                    static_cast<unsigned long long>(result.candidates.size()));
            }
            for (const auto& candidate : result.candidates)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1N-LOOP-CANDIDATE] query_drone_id=%u query_epoch=%llu query_kf=%llu candidate_drone_id=%u candidate_epoch=%llu candidate_kf=%llu bow_score=%.6f rank=%llu same_drone=%s same_submap=%s kf_gap=%llu near_same_submap=%s candidate_has_world_pose=%s candidate_is_anchored=%s query_mps=%llu candidate_mps=%llu source=%s",
                    candidate.query_kf_id.drone_id,
                    static_cast<unsigned long long>(candidate.query_kf_id.map_epoch),
                    static_cast<unsigned long long>(candidate.query_kf_id.local_kf_id),
                    candidate.candidate_kf_id.drone_id,
                    static_cast<unsigned long long>(candidate.candidate_kf_id.map_epoch),
                    static_cast<unsigned long long>(candidate.candidate_kf_id.local_kf_id),
                    candidate.bow_score,
                    static_cast<unsigned long long>(candidate.rank),
                    candidate.same_drone ? "true" : "false",
                    candidate.same_submap ? "true" : "false",
                    static_cast<unsigned long long>(candidate.kf_gap),
                    candidate.near_same_submap ? "true" : "false",
                    candidate.candidate_has_world_pose ? "true" : "false",
                    candidate.candidate_is_anchored ? "true" : "false",
                    static_cast<unsigned long long>(candidate.query_num_mappoints),
                    static_cast<unsigned long long>(candidate.candidate_num_mappoints),
                    candidate.source.c_str());
                RCLCPP_WARN(
                    get_logger(),
                    "[F1N-LOOP-CANDIDATE-RANK] query_drone_id=%u query_epoch=%llu query_kf=%llu candidate_drone_id=%u candidate_epoch=%llu candidate_kf=%llu rank=%llu bow_score=%.6f",
                    candidate.query_kf_id.drone_id,
                    static_cast<unsigned long long>(candidate.query_kf_id.map_epoch),
                    static_cast<unsigned long long>(candidate.query_kf_id.local_kf_id),
                    candidate.candidate_kf_id.drone_id,
                    static_cast<unsigned long long>(candidate.candidate_kf_id.map_epoch),
                    static_cast<unsigned long long>(candidate.candidate_kf_id.local_kf_id),
                    static_cast<unsigned long long>(candidate.rank),
                    candidate.bow_score);
            }
            candidate_results.push_back(result);
            RCLCPP_WARN(
                get_logger(),
                "[F1N-SERVER-LOOP-CANDIDATES-RX] trigger=%s arrival_id=%llu drone_id=%u epoch=%llu kf=%llu candidates=%llu reason=%s",
                trigger.c_str(),
                static_cast<unsigned long long>(insert_result.arrival_id),
                keyframe_id.drone_id,
                static_cast<unsigned long long>(keyframe_id.map_epoch),
                static_cast<unsigned long long>(keyframe_id.local_kf_id),
                static_cast<unsigned long long>(result.candidates.size()),
                result.reason.c_str());
        }
    }


    void LogRawInsert(const char* marker, uint64_t arrival_id, const RawDatabaseStats& stats)
    {
        RCLCPP_WARN(
            get_logger(),
            "[%s] arrival_id=%llu journal=%llu deltas=%llu full=%llu fiducial_observations=%llu submaps=%llu kfs=%llu mps=%llu",
            marker,
            static_cast<unsigned long long>(arrival_id),
            static_cast<unsigned long long>(stats.journal_entries),
            static_cast<unsigned long long>(stats.delta_entries),
            static_cast<unsigned long long>(stats.full_snapshot_entries),
            static_cast<unsigned long long>(stats.fiducial_observations),
            static_cast<unsigned long long>(stats.submaps),
            static_cast<unsigned long long>(stats.keyframes),
            static_cast<unsigned long long>(stats.mappoints));
    }

    void SaveRawDatabase(const std::string& reason)
    {
        if (!rawdb_record_enabled_ || rawdb_record_path_.empty())
        {
            return;
        }

        RawMapDatabase raw_snapshot;
        {
            std::lock_guard<std::recursive_mutex> lock(live_state_mutex_);
            raw_snapshot = raw_db_;
        }
        std::string error;
        const auto stats = raw_snapshot.GetDatabaseStats();
        const bool ok = raw_snapshot.SaveToPath(rawdb_record_path_, &error);
        if (ok)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1C-RAWDB-SAVE] reason=%s path=%s journal=%llu deltas=%llu full=%llu fiducial_observations=%llu submaps=%llu kfs=%llu mps=%llu",
                reason.c_str(),
                rawdb_record_path_.c_str(),
                static_cast<unsigned long long>(stats.journal_entries),
                static_cast<unsigned long long>(stats.delta_entries),
                static_cast<unsigned long long>(stats.full_snapshot_entries),
                static_cast<unsigned long long>(stats.fiducial_observations),
                static_cast<unsigned long long>(stats.submaps),
                static_cast<unsigned long long>(stats.keyframes),
                static_cast<unsigned long long>(stats.mappoints));
        }
        else
        {
            RCLCPP_ERROR(
                get_logger(),
                "[F1C-RAWDB-SAVE] reason=%s path=%s error=%s",
                reason.c_str(),
                rawdb_record_path_.c_str(),
                error.c_str());
        }
    }

    void LoadReplayDataset()
    {
        std::string error;
        RawMapDatabase loaded_db;
        if (!loaded_db.LoadFromPath(rawdb_replay_path_, &error))
        {
            RCLCPP_ERROR(
                get_logger(),
                "[F1C-REPLAY-LOAD] path=%s success=false error=%s",
                rawdb_replay_path_.c_str(),
                error.c_str());
            return;
        }

        replay_entries_ = loaded_db.GetJournalCopy();
        replay_fiducials_by_arrival_.clear();
        const auto fiducial_journal = loaded_db.GetFiducialObservationJournalCopy();
        for (const auto& observation : fiducial_journal)
        {
            replay_fiducials_by_arrival_[observation.arrival_id].push_back(observation);
        }
        const auto loaded_stats = loaded_db.GetDatabaseStats();
        raw_db_.Clear();
        covisibility_db_.Clear();
        fused_landmark_manager_.Clear();
        {
            std::lock_guard<std::mutex> lock(secondary_task_mutex_);
            loop_task_queue_.clear();
            loop_task_keys_.clear();
        }
        loop_pair_attempt_db_.Clear();
        bow_processed_appearance_revision_.clear();
        replay_index_ = 0;
        replay_done_logged_ = false;

        RCLCPP_WARN(
            get_logger(),
            "[F1C-REPLAY-LOAD] path=%s success=true entries=%zu deltas=%llu full=%llu submaps=%llu kfs=%llu mps=%llu fiducial_observations=%llu",
            rawdb_replay_path_.c_str(),
            replay_entries_.size(),
            static_cast<unsigned long long>(loaded_stats.delta_entries),
            static_cast<unsigned long long>(loaded_stats.full_snapshot_entries),
            static_cast<unsigned long long>(loaded_stats.submaps),
            static_cast<unsigned long long>(loaded_stats.keyframes),
            static_cast<unsigned long long>(loaded_stats.mappoints),
            static_cast<unsigned long long>(loaded_stats.fiducial_observations));

        replay_timer_ = create_wall_timer(
            std::chrono::duration<double>(rawdb_replay_period_sec_),
            [this]()
            {
                ReplayNextEntry();
            });
    }

    void ReplayNextEntry()
    {
        std::unique_lock<std::recursive_mutex> live_lock(live_state_mutex_);
        if (replay_index_ >= replay_entries_.size())
        {
            if (!replay_done_logged_)
            {
                replay_done_logged_ = true;
                const auto stats = raw_db_.GetDatabaseStats();
                const uint64_t replay_fiducials = ReplayFiducialObservationCount();
                RCLCPP_WARN(
                    get_logger(),
                    "[F1C-REPLAY-DONE] entries=%zu journal=%llu deltas=%llu full=%llu fiducial_observations=%llu submaps=%llu kfs=%llu mps=%llu",
                    replay_entries_.size(),
                    static_cast<unsigned long long>(stats.journal_entries),
                    static_cast<unsigned long long>(stats.delta_entries),
                    static_cast<unsigned long long>(stats.full_snapshot_entries),
                    static_cast<unsigned long long>(replay_fiducials),
                    static_cast<unsigned long long>(stats.submaps),
                    static_cast<unsigned long long>(stats.keyframes),
                    static_cast<unsigned long long>(stats.mappoints));
                TryF1IDebugBuildAfterReplay();
            }
            return;
        }

        const RawJournalEntry& entry = replay_entries_[replay_index_++];
        orbslam3_multi::RawInsertResult result;
        if (entry.kind == RawJournalEntryKind::FullSnapshot)
        {
            result = raw_db_.InsertFullSnapshot(entry.arrival_id, entry.map);
            RCLCPP_WARN(
                get_logger(),
                "[F1G-REPLAY-FULL-SNAPSHOT] index=%zu arrival_id=%llu drone_id=%u epoch=%llu seq=%llu kfs=%zu mps=%zu",
                replay_index_,
                static_cast<unsigned long long>(entry.arrival_id),
                entry.map.drone_id,
                static_cast<unsigned long long>(entry.map.map_epoch),
                static_cast<unsigned long long>(entry.map.map_sequence),
                entry.map.keyframes.size(),
                entry.map.mappoints.size());
        }
        else
        {
            result = raw_db_.InsertDelta(entry.arrival_id, entry.map);
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1C-REPLAY-DELTA] index=%zu arrival_id=%llu kind=%s drone_id=%u epoch=%llu seq=%llu kfs=%zu mps=%zu",
            replay_index_,
            static_cast<unsigned long long>(entry.arrival_id),
            orbslam3_multi::ToString(entry.kind),
            entry.map.drone_id,
            static_cast<unsigned long long>(entry.map.map_epoch),
            static_cast<unsigned long long>(entry.map.map_sequence),
            entry.map.keyframes.size(),
            entry.map.mappoints.size());

        if (entry.kind == RawJournalEntryKind::FullSnapshot)
        {
            MaybeSetF1GDebugOptimizedKeyFrame(entry.map, "replay_full_snapshot");
            ProcessFullSnapshotAfterInsert(
                entry.map,
                entry.arrival_id,
                result,
                "replay_full_snapshot");
            if (result.has_material_changes)
            {
                live_lock.unlock();
            }
        }
        else
        {
            ImportCovisibilityFromRaw(result, "replay_delta");
            ReconcilePoseStoreAfterDelta(
                result,
                entry.arrival_id,
                "F1C_REPLAY_DELTA_RECONCILE");
            LogRawInsert("F1C-RAWDB-INSERT-DELTA", entry.arrival_id, result.stats);
            const OrbMap material_map = BuildMaterialMap(entry.map, result);
            UpdateScoresFromMap(material_map, entry.arrival_id);
            ProcessReplayFiducials(entry.arrival_id);
            RegisterF1EKeyFramesFromMap(material_map);
            if (!result.has_material_changes)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1P-SNAPSHOT-NOOP] source=replay_delta arrival_id=%llu fastpath_queries=0 bow_queries=0 workers_created=0 cloud_rebuild=false",
                    static_cast<unsigned long long>(entry.arrival_id));
                return;
            }
            RequestGlobalStatePublication("replay_delta_raw_commit");
            if (result.has_loop_material_changes)
            {
                ScheduleLoopTasks(result, "replay_delta");
            }
            else
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1P-LOOP-NOOP-METADATA] source=replay_delta arrival_id=%llu metadata_mps=%zu fastpath_queries=0 bow_queries=0 workers_created=0",
                    static_cast<unsigned long long>(entry.arrival_id),
                    result.metadata_changed_mappoint_ids.size());
            }
            HandlePoseStoreDebugAfterInsert(entry.map, result.stats.journal_entries);
            live_lock.unlock();
        }
    }

    uint64_t ReplayFiducialObservationCount() const
    {
        uint64_t count = 0;
        for (const auto& [_, observations] : replay_fiducials_by_arrival_)
        {
            count += observations.size();
        }
        return count;
    }

    void ProcessReplayFiducials(uint64_t arrival_id)
    {
        const auto it = replay_fiducials_by_arrival_.find(arrival_id);
        if (it == replay_fiducials_by_arrival_.end())
        {
            return;
        }

        for (const auto& recorded : it->second)
        {
            const auto observation =
                FromRecordedObservation(recorded, "REPLAY_RECORDED_FIDUCIAL");
            RCLCPP_WARN(
                get_logger(),
                "[F1E-FID-REPLAY-OBS] arrival_id=%llu drone_id=%u epoch=%llu kf=%llu fid=%d source=REPLAY_RECORDED_FIDUCIAL",
                static_cast<unsigned long long>(observation.arrival_id),
                observation.drone_id,
                static_cast<unsigned long long>(observation.map_epoch),
                static_cast<unsigned long long>(observation.local_keyframe_id),
                observation.fiducial_id);
            HandleFiducialObservation(observation, false);
        }
    }

    void RegisterF1EKeyFramesFromMap(const OrbMap& map)
    {
        // F1E: despues de que un submapa quede anclado por fiducial, los KFs
        // nuevos del mismo submapa pueden recibir pose world derivada sin tocar
        // RawMapDatabase ni convertirlos en hard fiducial.
        for (const auto& keyframe : map.keyframes)
        {
            const RawKeyFrameId keyframe_id{
                map.drone_id,
                map.map_epoch,
                keyframe.id};
            const auto result =
                pose_store_.RegisterNewKeyFrameIfAnchored(
                    keyframe_id,
                    raw_db_,
                    "F1E_FIDUCIAL_ANCHOR");
            if (result.status == GlobalPoseNewKeyFrameStatus::PoseSet)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1E-POSESTORE-NEW-KF-POSE-SET] drone_id=%u epoch=%llu kf=%llu source=%s",
                    keyframe_id.drone_id,
                    static_cast<unsigned long long>(keyframe_id.map_epoch),
                    static_cast<unsigned long long>(keyframe_id.local_kf_id),
                    result.source.c_str());
            }
            else if (result.status == GlobalPoseNewKeyFrameStatus::CorrectionInherited)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1K-TAIL-ANCHOR-REBASE] drone_id=%u epoch=%llu kf=%llu accepted_reference=true source=%s reason=%s",
                    keyframe_id.drone_id,
                    static_cast<unsigned long long>(keyframe_id.map_epoch),
                    static_cast<unsigned long long>(keyframe_id.local_kf_id),
                    result.source.c_str(),
                    result.reason.c_str());
            }
        }
    }

    void HandlePoseStoreDebugAfterInsert(const OrbMap& map, uint64_t journal_entries)
    {
        if (!pose_store_debug_enabled_)
        {
            return;
        }

        // F1D: el modo debug valida la infraestructura de pose store con datos
        // raw reales/replay. No representa un anchor real; 1E introducira el
        // anclaje por fiducial simulado.
        TryPoseStoreDebugAnchor(journal_entries);
        TryPoseStoreDebugOptimization(journal_entries);
        RegisterPoseStoreKeyFramesFromMap(map);

        if (journal_entries % 25U == 0U)
        {
            LogPoseStoreStats("periodic_debug");
        }
    }

    void TryPoseStoreDebugAnchor(uint64_t journal_entries)
    {
        if (pose_store_debug_anchor_done_ ||
            journal_entries < static_cast<uint64_t>(pose_store_debug_anchor_after_deltas_))
        {
            return;
        }

        const RawSubmapId submap_id = DebugPoseStoreSubmapId();
        if (!raw_db_.HasSubmap(submap_id))
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1D-SERVER-DEBUG-ANCHOR] status=waiting reason=submap_missing drone_id=%u epoch=%llu journal=%llu",
                submap_id.drone_id,
                static_cast<unsigned long long>(submap_id.map_epoch),
                static_cast<unsigned long long>(journal_entries));
            return;
        }

        const Eigen::Matrix4d world_T_local =
            PlanarTransform(pose_store_debug_anchor_world_x_,
                            pose_store_debug_anchor_world_y_,
                            pose_store_debug_anchor_world_z_,
                            pose_store_debug_anchor_yaw_);

        RCLCPP_WARN(
            get_logger(),
            "[F1D-POSESTORE-ANCHOR-REQUEST] drone_id=%u epoch=%llu source=DEBUG_TEST journal=%llu world_t=(%.3f,%.3f,%.3f) yaw=%.3f",
            submap_id.drone_id,
            static_cast<unsigned long long>(submap_id.map_epoch),
            static_cast<unsigned long long>(journal_entries),
            pose_store_debug_anchor_world_x_,
            pose_store_debug_anchor_world_y_,
            pose_store_debug_anchor_world_z_,
            pose_store_debug_anchor_yaw_);

        const auto result =
            pose_store_.AnchorSubmap(submap_id, world_T_local, raw_db_, "DEBUG_TEST");
        if (!result.success)
        {
            RCLCPP_ERROR(
                get_logger(),
                "[F1D-SERVER-DEBUG-ANCHOR] status=failed drone_id=%u epoch=%llu reason=%s",
                submap_id.drone_id,
                static_cast<unsigned long long>(submap_id.map_epoch),
                result.reason.c_str());
            return;
        }

        pose_store_debug_anchor_done_ = true;
        RCLCPP_WARN(
            get_logger(),
            "[F1D-SERVER-DEBUG-ANCHOR] status=applied drone_id=%u epoch=%llu source=DEBUG_TEST anchored_kfs=%llu",
            submap_id.drone_id,
            static_cast<unsigned long long>(submap_id.map_epoch),
            static_cast<unsigned long long>(result.anchored_keyframes));
        RCLCPP_WARN(
            get_logger(),
            "[F1D-POSESTORE-ANCHOR-SET] drone_id=%u epoch=%llu source=DEBUG_TEST replaced=%s",
            submap_id.drone_id,
            static_cast<unsigned long long>(submap_id.map_epoch),
            result.replaced_existing_anchor ? "true" : "false");

        const auto keyframe_ids = raw_db_.GetKeyFrameIdsForSubmap(submap_id);
        for (const auto& keyframe_id : keyframe_ids)
        {
            if (!pose_store_.HasWorldPose(keyframe_id))
            {
                continue;
            }

            RCLCPP_WARN(
                get_logger(),
                "[F1D-POSESTORE-ANCHOR-KF-POSE] drone_id=%u epoch=%llu kf=%llu source=DEBUG_TEST",
                keyframe_id.drone_id,
                static_cast<unsigned long long>(keyframe_id.map_epoch),
                static_cast<unsigned long long>(keyframe_id.local_kf_id));
        }

        RCLCPP_WARN(
            get_logger(),
            "[F1D-POSESTORE-ANCHOR-SUMMARY] drone_id=%u epoch=%llu source=DEBUG_TEST anchored_kfs=%llu world_t=(%.3f,%.3f,%.3f) yaw=%.3f",
            submap_id.drone_id,
            static_cast<unsigned long long>(submap_id.map_epoch),
            static_cast<unsigned long long>(result.anchored_keyframes),
            world_T_local(0, 3),
            world_T_local(1, 3),
            world_T_local(2, 3),
            YawFromTransform(world_T_local));
        LogPoseStoreStats("anchor_applied");
    }

    RawKeyFrameId SelectPoseStoreDebugOptimizationKeyFrame(const RawSubmapId& submap_id) const
    {
        RawKeyFrameId configured{
            submap_id.drone_id,
            submap_id.map_epoch,
            static_cast<uint64_t>(pose_store_debug_opt_kf_id_)};
        if (pose_store_.HasWorldPose(configured))
        {
            return configured;
        }

        const auto keyframe_ids = raw_db_.GetKeyFrameIdsForSubmap(submap_id);
        for (const auto& keyframe_id : keyframe_ids)
        {
            if (pose_store_.HasWorldPose(keyframe_id))
            {
                return keyframe_id;
            }
        }

        return configured;
    }

    void TryPoseStoreDebugOptimization(uint64_t journal_entries)
    {
        if (!pose_store_debug_opt_enabled_ ||
            pose_store_debug_opt_done_ ||
            !pose_store_debug_anchor_done_ ||
            journal_entries < static_cast<uint64_t>(pose_store_debug_opt_after_deltas_))
        {
            return;
        }

        const RawSubmapId submap_id = DebugPoseStoreSubmapId();
        const RawKeyFrameId keyframe_id =
            SelectPoseStoreDebugOptimizationKeyFrame(submap_id);
        const auto current_world_pose = pose_store_.GetWorldPose(keyframe_id);
        if (!current_world_pose)
        {
            RCLCPP_WARN(
                get_logger(),
                "[F1D-SERVER-DEBUG-OPT] status=waiting reason=no_world_pose drone_id=%u epoch=%llu kf=%llu",
                keyframe_id.drone_id,
                static_cast<unsigned long long>(keyframe_id.map_epoch),
                static_cast<unsigned long long>(keyframe_id.local_kf_id));
            return;
        }

        const Eigen::Matrix4d debug_delta =
            PlanarTransform(pose_store_debug_opt_dx_,
                            pose_store_debug_opt_dy_,
                            pose_store_debug_opt_dz_,
                            pose_store_debug_opt_dyaw_);
        const Eigen::Matrix4d optimized_world_T_kf =
            debug_delta * current_world_pose.value();

        RCLCPP_WARN(
            get_logger(),
            "[F1D-SERVER-DEBUG-OPT] status=request drone_id=%u epoch=%llu kf=%llu source=DEBUG_TEST_OPT dx=%.3f dy=%.3f dz=%.3f dyaw=%.3f",
            keyframe_id.drone_id,
            static_cast<unsigned long long>(keyframe_id.map_epoch),
            static_cast<unsigned long long>(keyframe_id.local_kf_id),
            pose_store_debug_opt_dx_,
            pose_store_debug_opt_dy_,
            pose_store_debug_opt_dz_,
            pose_store_debug_opt_dyaw_);

        const auto result =
            pose_store_.SetOptimizedKeyFramePose(
                keyframe_id,
                optimized_world_T_kf,
                raw_db_,
                "DEBUG_TEST_OPT");
        if (!result.success)
        {
            RCLCPP_ERROR(
                get_logger(),
                "[F1D-SERVER-DEBUG-OPT] status=failed drone_id=%u epoch=%llu kf=%llu reason=%s",
                keyframe_id.drone_id,
                static_cast<unsigned long long>(keyframe_id.map_epoch),
                static_cast<unsigned long long>(keyframe_id.local_kf_id),
                result.reason.c_str());
            return;
        }

        pose_store_debug_opt_done_ = true;
        RCLCPP_WARN(
            get_logger(),
            "[F1D-POSESTORE-OPT-POSE-SET] drone_id=%u epoch=%llu kf=%llu source=DEBUG_TEST_OPT",
            keyframe_id.drone_id,
            static_cast<unsigned long long>(keyframe_id.map_epoch),
            static_cast<unsigned long long>(keyframe_id.local_kf_id));
        RCLCPP_WARN(
            get_logger(),
            "[F1D-POSESTORE-CORRECTION-SET] drone_id=%u epoch=%llu kf=%llu source=DEBUG_TEST_OPT dx=%.3f dy=%.3f dz=%.3f dyaw=%.3f",
            keyframe_id.drone_id,
            static_cast<unsigned long long>(keyframe_id.map_epoch),
            static_cast<unsigned long long>(keyframe_id.local_kf_id),
            result.correction_T_latest(0, 3),
            result.correction_T_latest(1, 3),
            result.correction_T_latest(2, 3),
            YawFromTransform(result.correction_T_latest));
        RCLCPP_WARN(
            get_logger(),
            "[F1D-POSESTORE-CORRECTION-STATS] corrections=%llu optimized_kfs=%llu",
            static_cast<unsigned long long>(pose_store_.GetPoseStoreStats().submap_corrections),
            static_cast<unsigned long long>(pose_store_.GetPoseStoreStats().optimized_keyframes));
        LogPoseStoreStats("debug_opt_applied");
    }

    void RegisterPoseStoreKeyFramesFromMap(const OrbMap& map)
    {
        for (const auto& keyframe : map.keyframes)
        {
            const RawKeyFrameId keyframe_id{
                map.drone_id,
                map.map_epoch,
                keyframe.id};
            const auto result =
                pose_store_.RegisterNewKeyFrameIfAnchored(
                    keyframe_id,
                    raw_db_,
                    "DEBUG_TEST_NEW_KF");

            if (result.status == GlobalPoseNewKeyFrameStatus::PoseSet)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1D-POSESTORE-NEW-KF-POSE-SET] drone_id=%u epoch=%llu kf=%llu source=%s",
                    keyframe_id.drone_id,
                    static_cast<unsigned long long>(keyframe_id.map_epoch),
                    static_cast<unsigned long long>(keyframe_id.local_kf_id),
                    result.source.c_str());
            }
            else if (result.status == GlobalPoseNewKeyFrameStatus::CorrectionInherited)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "[F1D-POSESTORE-NEW-KF-CORRECTION-INHERIT] drone_id=%u epoch=%llu kf=%llu correction_source=%s reason=%s",
                    keyframe_id.drone_id,
                    static_cast<unsigned long long>(keyframe_id.map_epoch),
                    static_cast<unsigned long long>(keyframe_id.local_kf_id),
                    result.source.c_str(),
                    result.reason.c_str());
            }
        }
    }

    // F1B/F1C: parámetros y contadores del adaptador ROS. La observabilidad del
    // servidor permanece aquí; el estado raw persistente vive en RawMapDatabase.
    bool use_sim_time_ = false;
    std::string world_frame_ = "world";
    std::string namespace_base_ = "dron";
    int n_drones_ = 1;
    double stats_period_s_ = 2.0;
    double f1m_covisibility_min_weight_ = 15.0;
    int loop_bow_min_kf_gap_same_submap_ = 20;
    int loop_bow_max_candidates_ = 10;
    int loop_bow_max_candidates_per_submap_ = 3;
    int loop_bow_min_mappoints_ = 15;
    double loop_bow_min_score_ = 0.01;
    int loop_verify_max_candidates_per_query_ = 1;
    int loop_verify_query_subcloud_min_points_ = 12;
    int loop_verify_candidate_window_max_kfs_ = 12;
    int loop_verify_candidate_window_covisibility_min_weight_ = 15;
    int loop_verify_candidate_window_temporal_kf_radius_ = 8;
    double loop_verify_candidate_window_spatial_radius_m_ = 4.0;
    int loop_verify_candidate_subcloud_min_points_ = 20;
    int loop_verify_candidate_subcloud_max_points_ = 900;
    double loop_verify_candidate_subcloud_min_score_ = 0.0;
    int loop_verify_orb_match_max_hamming_ = 80;
    double loop_verify_orb_match_ratio_test_ = 0.90;
    bool loop_verify_orb_match_cross_check_ = true;
    int loop_verify_min_initial_matches_ = 8;
    bool loop_verify_candidate_reduce_enabled_ = true;
    int loop_verify_candidate_reduce_min_initial_matches_ = 8;
    double loop_verify_candidate_reduce_margin_m_ = 0.75;
    int loop_verify_candidate_reduce_min_points_after_ = 20;
    int loop_verify_ransac_min_matches_ = 6;
    int loop_verify_ransac_max_iterations_ = 120;
    double loop_verify_ransac_inlier_threshold_m_ = 0.30;
    int loop_verify_ransac_min_inliers_ = 6;
    double loop_verify_ransac_min_inlier_ratio_ = 0.25;
    double loop_verify_accept_mean_residual_m_ = 0.20;
    double loop_verify_accept_max_residual_m_ = 0.75;
    double loop_verify_fusion_error_t_m_ = 0.35;
    double loop_verify_fusion_error_yaw_rad_ = 0.25;
    int loop_optimization_min_prior_supports_ = 2;
    bool aligned_overlap_enabled_ = true;
    double aligned_overlap_keyframe_radius_m_ = 4.0;
    int aligned_overlap_max_candidate_kfs_ = 12;
    double aligned_overlap_strict_position_m_ = 0.40;
    int aligned_overlap_strict_max_hamming_ = 50;
    double aligned_overlap_strict_ratio_test_ = 0.80;
    int aligned_overlap_strict_min_matches_ = 8;
    double aligned_overlap_strict_min_match_ratio_ = 0.10;
    int aligned_overlap_strict_min_image_bins_ = 3;
    double aligned_overlap_strict_min_3d_span_ratio_ = 0.35;
    double aligned_overlap_strict_mean_residual_m_ = 0.12;
    double aligned_overlap_strict_max_residual_m_ = 0.30;
    double aligned_overlap_expand_position_m_ = 0.30;
    int aligned_overlap_expand_max_hamming_ = 80;
    double aligned_overlap_expand_ratio_test_ = 0.90;
    int loop_backlog_high_watermark_ = 10;
    int loop_backlog_low_watermark_ = 3;
    int loop_task_max_pending_ = 4096;
    std::string mapping_backpressure_topic_ =
        "/global_mapping/backpressure_active";
    bool rawdb_record_enabled_ = true;
    bool rawdb_replay_enabled_ = false;
    std::string rawdb_record_path_;
    std::string rawdb_replay_path_;
    double rawdb_replay_period_sec_ = 0.5;
    uint64_t rawdb_next_arrival_id_ = 0;
    bool pose_store_debug_enabled_ = false;
    int pose_store_debug_anchor_after_deltas_ = 5;
    int pose_store_debug_anchor_drone_id_ = 1;
    int pose_store_debug_anchor_epoch_ = 0;
    double pose_store_debug_anchor_world_x_ = 2.0;
    double pose_store_debug_anchor_world_y_ = 0.0;
    double pose_store_debug_anchor_world_z_ = 0.0;
    double pose_store_debug_anchor_yaw_ = 0.0;
    bool pose_store_debug_opt_enabled_ = false;
    int pose_store_debug_opt_after_deltas_ = 10;
    int pose_store_debug_opt_kf_id_ = 0;
    double pose_store_debug_opt_dx_ = 0.15;
    double pose_store_debug_opt_dy_ = -0.03;
    double pose_store_debug_opt_dz_ = 0.0;
    double pose_store_debug_opt_dyaw_ = 0.05;
    bool pose_store_debug_anchor_done_ = false;
    bool pose_store_debug_opt_done_ = false;
    bool fiducial_sim_enabled_ = true;
    double fiducial_gt_max_dt_sec_ = 1.0;
    int fiducial_gt_buffer_max_samples_ = 250;
    std::vector<int64_t> fiducial_ids_;
    std::vector<double> fiducial_x_;
    std::vector<double> fiducial_y_;
    std::vector<double> fiducial_z_;
    std::vector<double> fiducial_yaw_;
    std::vector<double> fiducial_radius_;
    std::vector<FiducialConfig> fiducials_;
    double fiducial_revisit_error_threshold_m_ = 0.35;
    double fiducial_revisit_yaw_threshold_rad_ = 0.25;
    double fiducial_revisit_rot_threshold_rad_ = 0.35;
    double body_T_camera_x_ = 0.10;
    double body_T_camera_y_ = 0.03;
    double body_T_camera_z_ = 0.03;
    double body_T_camera_roll_deg_ = 0.0;
    double body_T_camera_pitch_deg_ = -90.0;
    double body_T_camera_yaw_deg_ = 90.0;
    bool use_camera_optical_frame_convention_ = true;
    std::string global_sparse_cloud_topic_ = "/global_sparse_cloud";
    double global_map_min_score_to_publish_ = 0.0;
    double global_map_publish_period_sec_ = 1.0;
    std::string global_keyframes_topic_ = "/global_keyframes";
    bool global_keyframes_publish_enabled_ = true;
    double global_keyframes_frustum_scale_ = 0.20;
    bool global_keyframes_labels_enabled_ = false;
    bool f1g_full_snapshot_enabled_ = true;
    double f1g_full_snapshot_startup_delay_sec_ = 35.0;
    double f1g_full_snapshot_period_sec_ = 35.0;
    bool f1g_debug_mark_optimized_kf_ = true;
    bool f1g_debug_optimized_kf_done_ = false;
    int pose_graph_min_vertices_ = 2;
    double pose_graph_vertex_selection_ratio_ = 0.30;
    double pose_graph_fiducial_neighborhood_radius_m_ = 4.0;
    int pose_graph_fiducial_neighborhood_radius_kfs_ = 3;
    double pose_graph_fiducial_neighborhood_vertex_ratio_ = 0.20;
    bool pose_graph_anchor_stop_enabled_ = true;
    bool pose_graph_fiducial_connectivity_enabled_ = true;
    bool pose_graph_branch_selection_enabled_ = true;
    double pose_graph_subdivision_candidate_radius_m_ = 2.0;
    double pose_graph_corner_yaw_threshold_rad_ = 0.5235987756;
    bool pose_graph_include_temporal_edges_ = true;
    bool pose_graph_use_covisibility_edges_ = false;
    double pose_graph_temporal_edge_weight_ = 25.0;
    double pose_graph_temporal_edge_weight_sparse_ = 10.0;
    double pose_graph_fiducial_hard_weight_ = 1000000.0;
    double pose_graph_fiducial_target_translation_weight_ = 5000.0;
    double pose_graph_fiducial_target_rotation_weight_ = 2500.0;
    double pose_graph_current_pose_soft_weight_ = 5.0;
    bool f1i_debug_force_task_enabled_ = false;
    double f1i_debug_task_dx_ = 0.75;
    double f1i_debug_task_dy_ = 0.0;
    double f1i_debug_task_dz_ = 0.0;
    double f1i_debug_task_dyaw_ = 0.20;
    bool f1i_debug_task_done_ = false;
    uint64_t f1i_debug_task_counter_ = 0;
    bool f1j_dryrun_enabled_ = true;
    double f1j_dryrun_min_improvement_ratio_ = 0.05;
    double f1j_dryrun_partial_min_improvement_ratio_ = 0.70;
    double f1j_dryrun_max_final_error_t_ = 0.35;
    bool f1j_dryrun_require_cost_decrease_ = false;
    double f1j_solver_step_fraction_ = 0.95;
    bool f1j_export_debug_plot_ = false;
    std::string f1j_debug_output_dir_ = "src/codex/archivos_auxiliares";
    bool f1k_apply_enabled_ = true;
    bool f1l_validation_enabled_ = true;
    bool f1l_partial_apply_enabled_ = true;
    int f1l_max_partial_retries_ = 3;
    bool f1l_debug_force_reject_once_ = false;
    bool f1l_debug_force_reject_consumed_ = false;
    int f1l_debug_force_reject_task_id_ = -1;
    double f1l_post_apply_internal_broken_edge_t_ = 2.50;
    double f1l_post_apply_internal_max_growth_t_ = 1.50;
    double f1l_post_apply_fiducial_absurd_error_t_ = 10.0;
    bool f1l_gt_kf_debug_enabled_ = false;
    double f1l_gt_kf_debug_max_dt_sec_ = 1.0;
    bool f1l_debug_animation_enabled_ = true;
    std::string f1l_debug_animation_output_dir_ = "src/codex/archivos_auxiliares/html";
    bool f1l_graph_dump_enabled_ = false;
    std::string f1l_graph_dump_output_dir_ = "src/codex/archivos_auxiliares/repeticiones";

    std::vector<rclcpp::Subscription<OrbMap>::SharedPtr> orb_map_delta_subs_;
    std::vector<rclcpp::Client<GetOrbMap>::SharedPtr> full_snapshot_clients_;
    std::vector<bool> full_snapshot_pending_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> gt_pose_subs_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_sparse_cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        global_keyframes_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mapping_backpressure_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr flow_telemetry_pub_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    rclcpp::TimerBase::SharedPtr rawdb_save_timer_;
    rclcpp::TimerBase::SharedPtr replay_timer_;
    rclcpp::TimerBase::SharedPtr global_map_publish_timer_;
    rclcpp::TimerBase::SharedPtr full_snapshot_startup_timer_;
    rclcpp::TimerBase::SharedPtr full_snapshot_periodic_timer_;
    rclcpp::TimerBase::SharedPtr flow_telemetry_timer_;
    sensor_msgs::msg::PointCloud2 last_global_sparse_cloud_;
    bool has_last_global_sparse_cloud_ = false;
    std::set<MarkerIdentity> published_marker_ids_;
    RawMapDatabase raw_db_;
    CovisibilityDatabase covisibility_db_;
    LoopPairAttemptDatabase loop_pair_attempt_db_;
    LoopDetector loop_detector_;
    SubcloudLoopVerifier subcloud_loop_verifier_;
    LoopDecisionManager loop_decision_manager_;
    FusedLandmarkManager fused_landmark_manager_;
    GlobalPoseStore pose_store_;
    FiducialAnchorManager fiducial_anchor_manager_;
    LandmarkScoreManager score_manager_;
    GlobalMapBuilder global_map_builder_;
    PoseGraphBuilder pose_graph_builder_;
    OptimizationManager optimization_manager_;
    OptimizationDebugExporter optimization_debug_exporter_;
    std::vector<RawJournalEntry> replay_entries_;
    std::map<uint64_t, std::vector<RecordedFiducialObservation>> replay_fiducials_by_arrival_;
    size_t replay_index_ = 0;
    bool replay_done_logged_ = false;
    std::map<uint32_t, std::vector<GroundTruthSample>> gt_buffers_;
    std::map<RawKeyFrameId, DebugGtKeyFramePose> f1l_gt_keyframe_store_;
    std::set<RawKeyFrameId> f1l_gt_debug_missing_logged_;
    std::set<RawKeyFrameId> live_fiducial_observed_keyframes_;
    std::map<RawKeyFrameId, uint64_t>
        bow_processed_appearance_revision_;
    std::set<uint64_t> f1i_pose_graph_built_task_ids_;
    std::map<RawSubmapId, uint64_t> f1l_partial_checkpoint_task_by_submap_;
    std::map<RawSubmapId, uint64_t> f1l_partial_retry_count_by_submap_;

    mutable std::recursive_mutex live_state_mutex_;
    std::mutex publication_mutex_;
    std::condition_variable publication_condition_;
    std::thread publication_worker_thread_;
    std::atomic<bool> publication_worker_shutdown_{false};
    uint64_t publication_requested_revision_ = 0;
    uint64_t publication_captured_revision_ = 0;
    uint64_t publication_published_revision_ = 0;
    uint64_t publication_coalesced_requests_ = 0;
    uint64_t publication_stale_builds_ = 0;
    std::string publication_pending_reason_ = "startup";
    bool flow_telemetry_enabled_ = true;
    std::string flow_telemetry_topic_ = "/global_mapping/flow_events";
    static constexpr size_t kFlowTelemetryCapacity = 256U;
    std::mutex flow_telemetry_mutex_;
    std::deque<std::string> flow_telemetry_queue_;
    std::atomic<uint64_t> flow_telemetry_sequence_{0};
    std::atomic<uint64_t> flow_telemetry_dropped_{0};
    static constexpr uint64_t kLoopTaskIdPrefix = uint64_t{1} << 63U;
    mutable std::mutex secondary_task_mutex_;
    std::condition_variable secondary_task_condition_;
    std::deque<std::pair<FiducialOptimizationTask, std::string>>
        optimization_queue_;
    std::deque<LoopTask> loop_task_queue_;
    std::set<RawKeyFrameId> loop_task_keys_;
    std::thread secondary_worker_thread_;
    std::atomic<bool> secondary_worker_shutdown_{false};
    std::atomic<bool> secondary_worker_active_{false};
    std::atomic<uint64_t> next_loop_task_id_{1};
    std::atomic<bool> mapping_backpressure_active_{false};
    std::atomic<bool> loop_backpressure_latched_{false};
    size_t loop_peak_backlog_ = 0;
    std::atomic<uint64_t> loop_jobs_committed_{0};
    std::atomic<uint64_t> optimization_workers_scheduled_{0};
    std::atomic<uint64_t> optimization_workers_completed_{0};
    std::atomic<uint64_t> optimization_queued_jobs_{0};
    std::atomic<uint64_t> optimization_active_workers_{0};
    std::atomic<uint64_t> optimization_jobs_in_flight_{0};
    std::atomic<uint64_t> optimization_peak_active_workers_{0};
    std::atomic<uint64_t> optimization_workers_skipped_low_error_{0};

    mutable std::mutex stats_mutex_;
    uint64_t total_maps_ = 0;
    uint64_t total_keyframes_ = 0;
    uint64_t total_mappoints_ = 0;
    std::set<uint32_t> drones_seen_;
    std::set<std::pair<uint32_t, uint64_t>> epochs_seen_;
    std::map<uint32_t, DroneRxStats> per_drone_stats_;
};

int main(int argc, char** argv)
{
    // F1B: punto de entrada del nodo minimo. Mantenerlo simple facilita saber
    // si un fallo viene del runtime ROS o de logica posterior aun no migrada.
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalMapServer>());
    rclcpp::shutdown();
    return 0;
}
