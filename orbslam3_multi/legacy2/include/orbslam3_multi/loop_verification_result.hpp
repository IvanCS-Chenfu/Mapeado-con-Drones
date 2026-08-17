#pragma once

#include "orbslam3_multi/raw_map_types.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace orbslam3_multi
{

enum class LoopGeometryDecision : uint8_t
{
    Reject = 0,
    HoldInsufficientEvidence = 1,
    FusionCandidate = 2,
    LoopOptimizationCandidate = 3,
    AlreadyConfirmedCovisibility = 4,
};

struct Ransac3D3DResult
{
    bool success = false;
    bool degenerate = false;
    uint64_t iterations = 0;
    uint64_t matches = 0;
    uint64_t inliers = 0;
    double inlier_ratio = 0.0;
    double mean_residual = 0.0;
    double max_residual = 0.0;
    Eigen::Matrix4d estimated_candidate_T_query = Eigen::Matrix4d::Identity();
    std::vector<uint64_t> inlier_match_indices;
    std::string reason;
};

struct LoopVerificationResult
{
    RawKeyFrameId query_kf_id;
    RawKeyFrameId candidate_seed_kf_id;
    RawSubmapId query_submap_id;
    RawSubmapId candidate_submap_id;
    double bow_score = 0.0;
    bool already_confirmed_covisibility = false;
    std::string confirmed_covisibility_source;

    uint64_t query_points = 0;
    uint64_t candidate_window_kfs = 0;
    uint64_t candidate_window_covisible_added = 0;
    uint64_t candidate_window_tree_added = 0;
    uint64_t candidate_window_temporal_added = 0;
    uint64_t candidate_window_spatial_added = 0;
    uint64_t candidate_initial_points = 0;
    uint64_t candidate_reduced_points = 0;
    uint64_t initial_matches = 0;
    uint64_t initial_duplicates_removed = 0;
    double initial_mean_descriptor_distance = 0.0;
    uint64_t refined_matches = 0;
    uint64_t refined_duplicates_removed = 0;
    double refined_mean_descriptor_distance = 0.0;
    bool ransac_success = false;
    bool ransac_degenerate = false;
    uint64_t ransac_iterations = 0;
    uint64_t ransac_inliers = 0;
    uint64_t shared_identity_matches = 0;
    double inlier_ratio = 0.0;
    uint64_t image_coverage_bins = 0;
    double spatial_coverage_ratio = 0.0;
    double mean_residual = 0.0;
    double max_residual = 0.0;
    double error_t = 0.0;
    double error_yaw = 0.0;
    double error_rot = 0.0;
    double loop_confidence = 0.0;

    bool reduction_fallback = false;
    std::string reduction_reason;
    bool reduction_box_valid = false;
    Eigen::Vector3d reduction_box_min = Eigen::Vector3d::Zero();
    Eigen::Vector3d reduction_box_max = Eigen::Vector3d::Zero();
    bool geometry_confirmed = false;
    Eigen::Matrix4d estimated_candidate_T_query = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d relative_pose_measured = Eigen::Matrix4d::Identity();
    std::vector<std::pair<RawMapPointId, RawMapPointId>> inlier_mappoint_pairs;

    LoopGeometryDecision decision = LoopGeometryDecision::Reject;
    std::string reason;
};

const char* ToString(LoopGeometryDecision decision);

}  // namespace orbslam3_multi
