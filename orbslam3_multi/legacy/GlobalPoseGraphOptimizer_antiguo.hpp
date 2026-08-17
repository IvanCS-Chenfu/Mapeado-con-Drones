#pragma once

#include "orbslam3_multi/legacy/GlobalPoseGraphTypes_antiguo.hpp"

#include <Eigen/Dense>

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace orbslam3_multi
{

// Forward declaration.
// La definición real de PoseGraphSnapshot está en MultiDroneSystem.hpp.
struct PoseGraphSnapshot;

class GlobalPoseGraphOptimizer
{
public:
    GlobalPoseGraphOptimizer() = default;

    GlobalPoseGraphOptimizationResult Optimize(
        const PoseGraphSnapshot& graph,
        const GlobalPoseGraphOptimizationParams& params) const;

private:
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    using Matrix6d = Eigen::Matrix<double, 6, 6>;

    static bool IsFiniteMatrix(
        const Eigen::Matrix4d& T);

    static bool IsValidSE3(
        const Eigen::Matrix4d& T,
        double max_translation_norm_m = 500.0);

    static Eigen::Matrix4d ProjectToSE3(
        const Eigen::Matrix4d& T);

    static Eigen::Matrix4d InverseSE3(
        const Eigen::Matrix4d& T);

    static Eigen::Matrix3d Skew(
        const Eigen::Vector3d& v);

    static Eigen::Matrix3d ExpSO3(
        const Eigen::Vector3d& w);

    static Eigen::Vector3d LogSO3(
        const Eigen::Matrix3d& R);

    static Eigen::Matrix4d ExpSE3(
        const Vector6d& xi);

    static Vector6d LogSE3(
        const Eigen::Matrix4d& T);

    static Eigen::Matrix4d ApplyRightUpdate(
        const Eigen::Matrix4d& T,
        const Vector6d& dx);

    static Vector6d EdgeError(
        const Eigen::Matrix4d& world_T_from,
        const Eigen::Matrix4d& world_T_to,
        const Eigen::Matrix4d& measured_from_T_to);

    static double EdgeWeight(
        int edge_type_as_int,
        int inliers,
        double mean_error_m);

    static double ClampNorm(
        Vector6d& dx,
        double max_norm);

    static bool SolveLinearSystem(
        const Eigen::MatrixXd& H,
        const Eigen::VectorXd& b,
        Eigen::VectorXd& dx);
};

}  // namespace orbslam3_multi
