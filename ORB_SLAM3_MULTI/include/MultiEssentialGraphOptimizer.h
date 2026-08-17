#pragma once

#include <Eigen/Dense>

#include <cstdint>
#include <vector>

namespace ORB_SLAM3_MULTI
{

struct MultiGraphVertexInput
{
    uint64_t global_kf_id = 0;

    uint32_t drone_id = 0;
    uint64_t map_epoch = 0;
    uint64_t local_kf_id = 0;

    // Pose inicial:
    // world_T_camera
    Eigen::Matrix4d world_T_camera =
        Eigen::Matrix4d::Identity();

    bool fixed = false;
};

enum class MultiGraphEdgeType
{
    LOCAL_PARENT = 0,
    LOCAL_COVISIBILITY = 1,
    LOOP_INTRA = 2,
    LOOP_INTER = 3
};

struct MultiGraphEdgeInput
{
    uint64_t from_kf_id = 0;
    uint64_t to_kf_id = 0;

    // Convención:
    // from_T_to = camera_from_T_camera_to
    Eigen::Matrix4d from_T_to =
        Eigen::Matrix4d::Identity();

    MultiGraphEdgeType type =
        MultiGraphEdgeType::LOCAL_COVISIBILITY;

    int inliers = 0;
    double mean_error_m = 0.0;
};

struct MultiGraphOptimizationParams
{
    int iterations = 10;
    bool fix_scale = true;

    bool use_local_parent_edges = true;
    bool use_local_covisibility_edges = true;
    bool use_loop_edges = true;
};

struct MultiGraphOptimizedPose
{
    uint64_t global_kf_id = 0;

    uint32_t drone_id = 0;
    uint64_t map_epoch = 0;
    uint64_t local_kf_id = 0;

    Eigen::Matrix4d world_T_camera_initial =
        Eigen::Matrix4d::Identity();

    Eigen::Matrix4d world_T_camera_optimized =
        Eigen::Matrix4d::Identity();

    bool valid = false;
};

struct MultiGraphOptimizationResult
{
    bool success = false;

    size_t vertices = 0;
    size_t fixed_vertices = 0;

    size_t edges_total = 0;
    size_t edges_local = 0;
    size_t edges_loop = 0;

    double initial_chi2 = 0.0;
    double final_chi2 = 0.0;

    std::vector<MultiGraphOptimizedPose> optimized_poses;
};

class MultiEssentialGraphOptimizer
{
public:
    static MultiGraphOptimizationResult Optimize(
        const std::vector<MultiGraphVertexInput>& vertices,
        const std::vector<MultiGraphEdgeInput>& edges,
        const MultiGraphOptimizationParams& params);

private:
    static bool IsFiniteMatrix(
        const Eigen::Matrix4d& T);

    static bool IsValidSE3(
        const Eigen::Matrix4d& T);

    static Eigen::Matrix4d ProjectToSE3(
        const Eigen::Matrix4d& T);
};

}  // namespace ORB_SLAM3_MULTI