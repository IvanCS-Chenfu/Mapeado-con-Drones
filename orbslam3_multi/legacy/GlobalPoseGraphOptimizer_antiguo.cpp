#include "orbslam3_multi/legacy/GlobalPoseGraphOptimizer_antiguo.hpp"
#include "orbslam3_multi/legacy/MultiDroneSystem_antiguo.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <unordered_map>

namespace orbslam3_multi
{

#define EIGEN_OPT_LOG(msg) \
    do { std::cerr << "[EIGEN_OPT] " << msg << std::endl; } while (0)

bool GlobalPoseGraphOptimizer::IsFiniteMatrix(
    const Eigen::Matrix4d& T)
{
    for (int r = 0; r < 4; ++r)
    {
        for (int c = 0; c < 4; ++c)
        {
            if (!std::isfinite(T(r, c)))
                return false;
        }
    }

    return true;
}

bool GlobalPoseGraphOptimizer::IsValidSE3(
    const Eigen::Matrix4d& T,
    double max_translation_norm_m)
{
    if (!IsFiniteMatrix(T))
        return false;

    const Eigen::Matrix3d R =
        T.block<3, 3>(0, 0);

    const Eigen::Vector3d t =
        T.block<3, 1>(0, 3);

    if (!std::isfinite(t.norm()))
        return false;

    if (t.norm() > max_translation_norm_m)
        return false;

    const double det =
        R.determinant();

    if (!std::isfinite(det))
        return false;

    if (std::abs(det - 1.0) > 0.25)
        return false;

    const double ortho_error =
        (R.transpose() * R - Eigen::Matrix3d::Identity()).norm();

    if (!std::isfinite(ortho_error))
        return false;

    if (ortho_error > 0.35)
        return false;

    const double bottom_error =
        std::abs(T(3, 0)) +
        std::abs(T(3, 1)) +
        std::abs(T(3, 2)) +
        std::abs(T(3, 3) - 1.0);

    if (bottom_error > 1e-6)
        return false;

    return true;
}

Eigen::Matrix4d GlobalPoseGraphOptimizer::ProjectToSE3(
    const Eigen::Matrix4d& T)
{
    Eigen::Matrix4d out =
        Eigen::Matrix4d::Identity();

    Eigen::Matrix3d R =
        T.block<3, 3>(0, 0);

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        R,
        Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix3d R_projected =
        svd.matrixU() * svd.matrixV().transpose();

    if (R_projected.determinant() < 0.0)
    {
        Eigen::Matrix3d U =
            svd.matrixU();

        U.col(2) *= -1.0;

        R_projected =
            U * svd.matrixV().transpose();
    }

    out.block<3, 3>(0, 0) =
        R_projected;

    out.block<3, 1>(0, 3) =
        T.block<3, 1>(0, 3);

    return out;
}

Eigen::Matrix4d GlobalPoseGraphOptimizer::InverseSE3(
    const Eigen::Matrix4d& T)
{
    Eigen::Matrix4d inv =
        Eigen::Matrix4d::Identity();

    const Eigen::Matrix3d R =
        T.block<3, 3>(0, 0);

    const Eigen::Vector3d t =
        T.block<3, 1>(0, 3);

    inv.block<3, 3>(0, 0) =
        R.transpose();

    inv.block<3, 1>(0, 3) =
        -R.transpose() * t;

    return inv;
}

Eigen::Matrix3d GlobalPoseGraphOptimizer::Skew(
    const Eigen::Vector3d& v)
{
    Eigen::Matrix3d S;
    S << 0.0, -v.z(), v.y(),
         v.z(), 0.0, -v.x(),
        -v.y(), v.x(), 0.0;

    return S;
}

Eigen::Matrix3d GlobalPoseGraphOptimizer::ExpSO3(
    const Eigen::Vector3d& w)
{
    const double theta =
        w.norm();

    const Eigen::Matrix3d W =
        Skew(w);

    if (theta < 1e-10)
    {
        return Eigen::Matrix3d::Identity() + W;
    }

    const double a =
        std::sin(theta) / theta;

    const double b =
        (1.0 - std::cos(theta)) / (theta * theta);

    return Eigen::Matrix3d::Identity() +
           a * W +
           b * W * W;
}

Eigen::Vector3d GlobalPoseGraphOptimizer::LogSO3(
    const Eigen::Matrix3d& R)
{
    const double cos_theta =
        std::clamp(
            (R.trace() - 1.0) * 0.5,
            -1.0,
            1.0);

    const double theta =
        std::acos(cos_theta);

    Eigen::Vector3d w;
    w << R(2, 1) - R(1, 2),
         R(0, 2) - R(2, 0),
         R(1, 0) - R(0, 1);

    if (theta < 1e-10)
    {
        return 0.5 * w;
    }

    return (theta / (2.0 * std::sin(theta))) * w;
}

Eigen::Matrix4d GlobalPoseGraphOptimizer::ExpSE3(
    const Vector6d& xi)
{
    Eigen::Matrix4d T =
        Eigen::Matrix4d::Identity();

    const Eigen::Vector3d upsilon =
        xi.head<3>();

    const Eigen::Vector3d omega =
        xi.tail<3>();

    const double theta =
        omega.norm();

    const Eigen::Matrix3d Omega =
        Skew(omega);

    Eigen::Matrix3d R =
        ExpSO3(omega);

    Eigen::Matrix3d V =
        Eigen::Matrix3d::Identity();

    if (theta < 1e-10)
    {
        V += 0.5 * Omega;
    }
    else
    {
        const double theta2 =
            theta * theta;

        const double theta3 =
            theta2 * theta;

        V +=
            ((1.0 - std::cos(theta)) / theta2) * Omega +
            ((theta - std::sin(theta)) / theta3) * Omega * Omega;
    }

    T.block<3, 3>(0, 0) =
        R;

    T.block<3, 1>(0, 3) =
        V * upsilon;

    return T;
}

GlobalPoseGraphOptimizer::Vector6d
GlobalPoseGraphOptimizer::LogSE3(
    const Eigen::Matrix4d& T)
{
    Vector6d xi =
        Vector6d::Zero();

    const Eigen::Matrix3d R =
        T.block<3, 3>(0, 0);

    const Eigen::Vector3d t =
        T.block<3, 1>(0, 3);

    const Eigen::Vector3d omega =
        LogSO3(R);

    const double theta =
        omega.norm();

    const Eigen::Matrix3d Omega =
        Skew(omega);

    Eigen::Matrix3d V_inv =
        Eigen::Matrix3d::Identity();

    if (theta < 1e-10)
    {
        V_inv -= 0.5 * Omega;
    }
    else
    {
        const double theta2 =
            theta * theta;

        const double half_theta =
            0.5 * theta;

        const double cot_half_theta =
            1.0 / std::tan(half_theta);

        V_inv -=
            0.5 * Omega;

        V_inv +=
            (1.0 / theta2) *
            (1.0 - half_theta * cot_half_theta) *
            Omega * Omega;
    }

    xi.head<3>() =
        V_inv * t;

    xi.tail<3>() =
        omega;

    return xi;
}

Eigen::Matrix4d GlobalPoseGraphOptimizer::ApplyRightUpdate(
    const Eigen::Matrix4d& T,
    const Vector6d& dx)
{
    return ProjectToSE3(T * ExpSE3(dx));
}

GlobalPoseGraphOptimizer::Vector6d
GlobalPoseGraphOptimizer::EdgeError(
    const Eigen::Matrix4d& world_T_from,
    const Eigen::Matrix4d& world_T_to,
    const Eigen::Matrix4d& measured_from_T_to)
{
    const Eigen::Matrix4d predicted_from_T_to =
        InverseSE3(world_T_from) * world_T_to;

    const Eigen::Matrix4d delta =
        InverseSE3(measured_from_T_to) * predicted_from_T_to;

    return LogSE3(ProjectToSE3(delta));
}

double GlobalPoseGraphOptimizer::EdgeWeight(
    int edge_type_as_int,
    int inliers,
    double mean_error_m)
{
    // edge_type_as_int se usa como int para evitar depender demasiado
    // de nombres concretos de enum en este optimizador.
    //
    // Convención esperada:
    // 0 = LOCAL_PARENT
    // 1 = LOCAL_COVISIBILITY
    // 2 = LOOP_INTRA
    // 3 = LOOP_INTER

    if (edge_type_as_int == 2 || edge_type_as_int == 3)
    {
        const double safe_error =
            std::max(0.05, mean_error_m);

        const double inlier_weight =
            std::sqrt(std::max(1, inliers));

        double w =
            inlier_weight / safe_error;

        w = std::min(w, 100.0);
        w = std::max(w, 1.0);

        return w;
    }

    if (edge_type_as_int == 0)
        return 5.0;

    return 1.0;
}

double GlobalPoseGraphOptimizer::ClampNorm(
    Vector6d& dx,
    double max_norm)
{
    const double n =
        dx.norm();

    if (!std::isfinite(n))
    {
        dx.setZero();
        return 0.0;
    }

    if (n > max_norm && n > 1e-12)
    {
        dx *= max_norm / n;
    }

    return dx.norm();
}

bool GlobalPoseGraphOptimizer::SolveLinearSystem(
    const Eigen::MatrixXd& H,
    const Eigen::VectorXd& b,
    Eigen::VectorXd& dx)
{
    if (H.rows() == 0 || H.cols() == 0)
        return false;

    Eigen::LDLT<Eigen::MatrixXd> ldlt(H);

    if (ldlt.info() != Eigen::Success)
        return false;

    dx =
        ldlt.solve(-b);

    if (ldlt.info() != Eigen::Success)
        return false;

    for (int i = 0; i < dx.size(); ++i)
    {
        if (!std::isfinite(dx[i]))
            return false;
    }

    return true;
}

GlobalPoseGraphOptimizationResult
GlobalPoseGraphOptimizer::Optimize(
    const PoseGraphSnapshot& graph,
    const GlobalPoseGraphOptimizationParams& params) const
{
    GlobalPoseGraphOptimizationResult result;

    EIGEN_OPT_LOG(
        "ENTER vertices=" << graph.vertices.size()
        << " edges=" << graph.edges.size()
        << " fiducial_priors=" << graph.fiducial_constraints.size()
        << " iterations=" << params.iterations
        << " max_update_step_m=" << params.max_update_step_m
        << " max_fiducial_prior_weight=" << params.max_fiducial_prior_weight);

    if (graph.vertices.empty())
        return result;

    std::unordered_map<uint64_t, int> vertex_index_by_id;
    std::unordered_map<uint64_t, Eigen::Matrix4d> poses;
    std::unordered_map<uint64_t, bool> fixed_by_id;

    std::vector<uint64_t> variable_ids;
    variable_ids.reserve(graph.vertices.size());

    for (const auto& v : graph.vertices)
    {
        if (v.global_kf_id == 0)
            continue;

        Eigen::Matrix4d T =
            ProjectToSE3(v.world_T_camera_initial);

        if (!IsValidSE3(T))
        {
            EIGEN_OPT_LOG(
                "skip invalid vertex global_id=" << v.global_kf_id);
            continue;
        }

        poses[v.global_kf_id] =
            T;

        fixed_by_id[v.global_kf_id] =
            v.fixed;

        if (!v.fixed)
        {
            const int idx =
                static_cast<int>(variable_ids.size());

            vertex_index_by_id[v.global_kf_id] =
                idx;

            variable_ids.push_back(v.global_kf_id);
        }
    }

    result.vertices =
        poses.size();

    result.fixed_vertices = 0;

    for (const auto& [id, is_fixed] : fixed_by_id)
    {
        (void)id;

        if (is_fixed)
            result.fixed_vertices++;
    }

    if (poses.empty())
        return result;

    if (variable_ids.empty())
    {
        EIGEN_OPT_LOG("all vertices fixed");
    }

    std::vector<const PoseGraphEdge*> valid_edges;
    valid_edges.reserve(graph.edges.size());

    struct ValidFiducialPrior
    {
        uint64_t kf_id = 0;

        Eigen::Matrix4d measured_world_T_camera =
            Eigen::Matrix4d::Identity();

        double weight = 1.0;

        uint32_t fiducial_id = 0;
    };

    std::vector<ValidFiducialPrior> valid_fiducial_priors;
    valid_fiducial_priors.reserve(graph.fiducial_constraints.size());

    for (const auto& e : graph.edges)
    {
        if (poses.find(e.from_kf_id) == poses.end())
            continue;

        if (poses.find(e.to_kf_id) == poses.end())
            continue;

        if (!IsValidSE3(e.from_T_to))
            continue;

        const bool is_loop =
            static_cast<int>(e.type) == 2 ||
            static_cast<int>(e.type) == 3;

        const bool is_parent =
            static_cast<int>(e.type) == 0;

        const bool is_cov =
            static_cast<int>(e.type) == 1;

        if (is_loop && !params.use_loop_edges)
            continue;

        if (is_parent && !params.use_local_parent_edges)
            continue;

        if (is_cov && !params.use_local_covisibility_edges)
            continue;

        valid_edges.push_back(&e);
    }

    for (const auto& c : graph.fiducial_constraints)
    {
        if (!c.valid)
            continue;

        if (c.global_kf_id == 0)
            continue;

        if (poses.find(c.global_kf_id) == poses.end())
            continue;

        Eigen::Matrix4d measured =
            ProjectToSE3(c.measured_world_T_camera);

        if (!IsValidSE3(measured))
            continue;

        ValidFiducialPrior prior;

        prior.kf_id =
            c.global_kf_id;

        prior.measured_world_T_camera =
            measured;

        // Cap configurable para evitar que un único fiducial destruya el grafo,
        // pero permitiendo que los priors cercanos sean realmente fuertes.
        const double max_prior_weight =
            std::max(1.0, params.max_fiducial_prior_weight);

        prior.weight =
            std::clamp(c.weight, 1.0, max_prior_weight);

        prior.fiducial_id =
            c.fiducial_id;

        valid_fiducial_priors.push_back(prior);
    }

    result.edges_total =
        valid_edges.size() + valid_fiducial_priors.size();

    result.edges_local = 0;
    result.edges_loop = 0;

    for (const auto* e : valid_edges)
    {
        const bool is_loop =
            static_cast<int>(e->type) == 2 ||
            static_cast<int>(e->type) == 3;

        if (is_loop)
            result.edges_loop++;
        else
            result.edges_local++;
    }

    if (valid_edges.empty() && valid_fiducial_priors.empty())
    {
        EIGEN_OPT_LOG("no valid edges and no valid fiducial priors");
        return result;
    }


    auto PriorError =
        [&](const Eigen::Matrix4d& estimated_world_T_camera,
            const Eigen::Matrix4d& measured_world_T_camera) -> Vector6d
        {
            const Eigen::Matrix4d delta =
                InverseSE3(measured_world_T_camera) *
                estimated_world_T_camera;

            return LogSE3(
                ProjectToSE3(delta));
        };

    auto compute_total_chi2 =
        [&]() -> double
        {
            double chi2 = 0.0;

            for (const auto* e : valid_edges)
            {
                const Vector6d err =
                    EdgeError(
                        poses[e->from_kf_id],
                        poses[e->to_kf_id],
                        ProjectToSE3(e->from_T_to));

                const double w =
                    EdgeWeight(
                        static_cast<int>(e->type),
                        e->inliers,
                        e->mean_error_m);

                chi2 +=
                    w * err.squaredNorm();
            }

            for (const auto& prior : valid_fiducial_priors)
            {
                auto pose_it =
                    poses.find(prior.kf_id);

                if (pose_it == poses.end())
                    continue;

                const Vector6d err =
                    PriorError(
                        pose_it->second,
                        prior.measured_world_T_camera);

                chi2 +=
                    prior.weight * err.squaredNorm();
            }

            return chi2;
        };

    result.initial_chi2 =
        compute_total_chi2();

    EIGEN_OPT_LOG(
        "graph vertices=" << result.vertices
        << " fixed=" << result.fixed_vertices
        << " variables=" << variable_ids.size()
        << " edges=" << result.edges_total
        << " local=" << result.edges_local
        << " loop=" << result.edges_loop
        << " fiducial_priors=" << valid_fiducial_priors.size()
        << " initial_chi2=" << result.initial_chi2);

    const int n_vars =
        static_cast<int>(variable_ids.size());

    const int dim =
        6 * n_vars;

    const int iterations =
        std::max(0, params.iterations);

    const double eps =
        1e-6;

    const double lambda =
        1e-4;

    for (int iter = 0; iter < iterations && dim > 0; ++iter)
    {
        Eigen::MatrixXd H =
            Eigen::MatrixXd::Zero(dim, dim);

        Eigen::VectorXd b =
            Eigen::VectorXd::Zero(dim);

        double iter_chi2 = 0.0;

        for (const auto* e : valid_edges)
        {
            const uint64_t from_id =
                e->from_kf_id;

            const uint64_t to_id =
                e->to_kf_id;

            const Eigen::Matrix4d measured =
                ProjectToSE3(e->from_T_to);

            const Vector6d err0 =
                EdgeError(
                    poses[from_id],
                    poses[to_id],
                    measured);

            const double w =
                EdgeWeight(
                    static_cast<int>(e->type),
                    e->inliers,
                    e->mean_error_m);

            iter_chi2 +=
                w * err0.squaredNorm();

            Eigen::Matrix<double, 6, 6> J_from =
                Eigen::Matrix<double, 6, 6>::Zero();

            Eigen::Matrix<double, 6, 6> J_to =
                Eigen::Matrix<double, 6, 6>::Zero();

            const bool from_variable =
                vertex_index_by_id.find(from_id) != vertex_index_by_id.end();

            const bool to_variable =
                vertex_index_by_id.find(to_id) != vertex_index_by_id.end();

            const Eigen::Matrix4d T_from_original =
                poses[from_id];

            const Eigen::Matrix4d T_to_original =
                poses[to_id];

            if (from_variable)
            {
                for (int k = 0; k < 6; ++k)
                {
                    Vector6d delta =
                        Vector6d::Zero();

                    delta[k] = eps;

                    poses[from_id] =
                        ApplyRightUpdate(
                            T_from_original,
                            delta);

                    poses[to_id] =
                        T_to_original;

                    const Vector6d err =
                        EdgeError(
                            poses[from_id],
                            poses[to_id],
                            measured);

                    J_from.col(k) =
                        (err - err0) / eps;
                }

                poses[from_id] =
                    T_from_original;

                poses[to_id] =
                    T_to_original;
            }

            if (to_variable)
            {
                for (int k = 0; k < 6; ++k)
                {
                    Vector6d delta =
                        Vector6d::Zero();

                    delta[k] = eps;

                    poses[from_id] =
                        T_from_original;

                    poses[to_id] =
                        ApplyRightUpdate(
                            T_to_original,
                            delta);

                    const Vector6d err =
                        EdgeError(
                            poses[from_id],
                            poses[to_id],
                            measured);

                    J_to.col(k) =
                        (err - err0) / eps;
                }

                poses[from_id] =
                    T_from_original;

                poses[to_id] =
                    T_to_original;
            }

            if (from_variable)
            {
                const int i =
                    vertex_index_by_id[from_id];

                const int row_i =
                    6 * i;

                H.block<6, 6>(row_i, row_i) +=
                    w * J_from.transpose() * J_from;

                b.segment<6>(row_i) +=
                    w * J_from.transpose() * err0;
            }

            if (to_variable)
            {
                const int j =
                    vertex_index_by_id[to_id];

                const int row_j =
                    6 * j;

                H.block<6, 6>(row_j, row_j) +=
                    w * J_to.transpose() * J_to;

                b.segment<6>(row_j) +=
                    w * J_to.transpose() * err0;
            }

            if (from_variable && to_variable)
            {
                const int i =
                    vertex_index_by_id[from_id];

                const int j =
                    vertex_index_by_id[to_id];

                const int row_i =
                    6 * i;

                const int row_j =
                    6 * j;

                H.block<6, 6>(row_i, row_j) +=
                    w * J_from.transpose() * J_to;

                H.block<6, 6>(row_j, row_i) +=
                    w * J_to.transpose() * J_from;
            }
        }

        // ============================================================
        // Fiducial pose priors:
        // error = Log( inverse(measured_world_T_camera) * estimated_world_T_camera )
        // Solo afecta a un vértice.
        // ============================================================

        for (const auto& prior : valid_fiducial_priors)
        {
            const uint64_t id =
                prior.kf_id;

            auto var_it =
                vertex_index_by_id.find(id);

            // Si el vértice está fijo, cuenta en chi2 pero no se actualiza.
            if (var_it == vertex_index_by_id.end())
            {
                auto pose_it =
                    poses.find(id);

                if (pose_it != poses.end())
                {
                    const Vector6d err_fixed =
                        PriorError(
                            pose_it->second,
                            prior.measured_world_T_camera);

                    iter_chi2 +=
                        prior.weight * err_fixed.squaredNorm();
                }

                continue;
            }

            auto pose_it =
                poses.find(id);

            if (pose_it == poses.end())
                continue;

            const Eigen::Matrix4d T_original =
                pose_it->second;

            const Vector6d err0 =
                PriorError(
                    T_original,
                    prior.measured_world_T_camera);

            iter_chi2 +=
                prior.weight * err0.squaredNorm();

            Eigen::Matrix<double, 6, 6> J =
                Eigen::Matrix<double, 6, 6>::Zero();

            for (int k = 0; k < 6; ++k)
            {
                Vector6d delta =
                    Vector6d::Zero();

                delta[k] = eps;

                poses[id] =
                    ApplyRightUpdate(
                        T_original,
                        delta);

                const Vector6d err =
                    PriorError(
                        poses[id],
                        prior.measured_world_T_camera);

                J.col(k) =
                    (err - err0) / eps;
            }

            poses[id] =
                T_original;

            const int i =
                var_it->second;

            const int row_i =
                6 * i;

            H.block<6, 6>(row_i, row_i) +=
                prior.weight * J.transpose() * J;

            b.segment<6>(row_i) +=
                prior.weight * J.transpose() * err0;
        }

        for (int i = 0; i < dim; ++i)
        {
            H(i, i) += lambda;
        }

        Eigen::VectorXd dx;

        if (!SolveLinearSystem(H, b, dx))
        {
            EIGEN_OPT_LOG(
                "iteration=" << iter
                << " solve failed");
            break;
        }

        double max_step = 0.0;

        const double max_update_step =
            std::max(1e-6, params.max_update_step_m);

        for (int i = 0; i < n_vars; ++i)
        {
            Vector6d dxi =
                dx.segment<6>(6 * i);

            const double step_norm =
                ClampNorm(dxi, max_update_step);

            max_step =
                std::max(max_step, step_norm);

            const uint64_t id =
                variable_ids[i];

            poses[id] =
                ApplyRightUpdate(
                    poses[id],
                    dxi);
        }

        const double new_chi2 =
            compute_total_chi2();

        EIGEN_OPT_LOG(
            "iter=" << iter
            << " chi2_before=" << iter_chi2
            << " chi2_after=" << new_chi2
            << " max_step=" << max_step);

        if (max_step < 1e-6)
            break;
    }

    result.final_chi2 =
        compute_total_chi2();

    result.optimized_poses.clear();
    result.optimized_poses.reserve(graph.vertices.size());

    for (const auto& v : graph.vertices)
    {
        auto it =
            poses.find(v.global_kf_id);

        if (it == poses.end())
            continue;

        GlobalPoseGraphOptimizedPose out;

        out.global_kf_id =
            v.global_kf_id;

        out.drone_id =
            v.drone_id;

        out.map_epoch =
            v.map_epoch;

        out.local_kf_id =
            v.local_kf_id;

        out.world_T_camera_initial =
            ProjectToSE3(v.world_T_camera_initial);

        out.world_T_camera_optimized =
            ProjectToSE3(it->second);

        out.valid =
            IsValidSE3(out.world_T_camera_optimized);

        if (out.valid)
        {
            result.optimized_poses.push_back(out);
        }
    }

    result.success =
        !result.optimized_poses.empty();

    EIGEN_OPT_LOG(
        "DONE success=" << result.success
        << " optimized_poses=" << result.optimized_poses.size()
        << " chi2_initial=" << result.initial_chi2
        << " chi2_final=" << result.final_chi2);

    return result;
}

}  // namespace orbslam3_multi
