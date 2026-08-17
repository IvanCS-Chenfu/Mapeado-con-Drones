#pragma once

#include <Eigen/Dense>

#include <cstdint>
#include <vector>

namespace orbslam3_multi
{

struct GlobalPoseGraphOptimizationParams
{
    int iterations = 20;

    bool fix_scale = true;

    int min_loop_inliers = 25;
    double max_loop_mean_error_m = 0.25;
    double min_loop_inlier_ratio = 0.30;

    bool use_local_parent_edges = true;
    bool use_local_covisibility_edges = true;
    bool use_loop_edges = true;

    // Límite máximo del incremento SE3 aplicado a cada vértice por iteración.
    // Antes estaba hardcodeado a 0.20 en GlobalPoseGraphOptimizer.cpp.
    double max_update_step_m = 0.20;

    // Peso máximo real permitido para priors de fiducial.
    // Tu servidor ya genera pesos altos, pero ahora mismo el optimizador
    // los capa a 50.0 internamente.
    double max_fiducial_prior_weight = 120.0;

    // Control de ventana activa del pose graph.
    bool use_active_window_filter = true;
    int active_window_local_kfs = 35;
};

struct GlobalPoseGraphOptimizedPose
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

struct GlobalPoseGraphOptimizationResult
{
    bool success = false;

    size_t vertices = 0;
    size_t fixed_vertices = 0;

    size_t edges_total = 0;
    size_t edges_local = 0;
    size_t edges_loop = 0;

    double initial_chi2 = 0.0;
    double final_chi2 = 0.0;

    std::vector<GlobalPoseGraphOptimizedPose> optimized_poses;
};

}  // namespace orbslam3_multi