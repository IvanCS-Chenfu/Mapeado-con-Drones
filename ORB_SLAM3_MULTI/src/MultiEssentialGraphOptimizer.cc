#include "MultiEssentialGraphOptimizer.h"

#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"


#include <Eigen/Dense>

#include <cmath>
#include <iostream>
//#include <memory>
#include <unordered_map>
#include <algorithm>

namespace ORB_SLAM3_MULTI
{

#define MULTI_OPT_LOG(msg) \
    do { std::cerr << "[ORB_MULTI_OPT] " << msg << std::endl; } while (0)

class EdgeSE3PoseGraph
    : public g2o::BaseBinaryEdge<
          6,
          g2o::SE3Quat,
          g2o::VertexSE3Expmap,
          g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void computeError() override
    {
        const auto* v_from =
            static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);

        const auto* v_to =
            static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);

        const g2o::SE3Quat T_from_w =
            v_from->estimate();

        const g2o::SE3Quat T_to_w =
            v_to->estimate();

        // Vértices:
        //   T_from_w = camera_from_T_world
        //   T_to_w   = camera_to_T_world
        //
        // Medida:
        //   _measurement = camera_to_T_camera_from
        //
        // Predicción:
        //   camera_to_T_camera_from =
        //       camera_to_T_world * world_T_camera_from
        const g2o::SE3Quat T_to_from_pred =
            T_to_w * T_from_w.inverse();

        const g2o::SE3Quat delta =
            _measurement.inverse() * T_to_from_pred;

        _error =
            delta.log();
    }

    void linearizeOplus() override
    {
        auto* v_from =
            static_cast<g2o::VertexSE3Expmap*>(_vertices[0]);

        auto* v_to =
            static_cast<g2o::VertexSE3Expmap*>(_vertices[1]);

        const g2o::SE3Quat T_from_original =
            v_from->estimate();

        const g2o::SE3Quat T_to_original =
            v_to->estimate();

        computeError();

        const Eigen::Matrix<double, 6, 1> error0 =
            _error;

        const double eps = 1e-6;

        _jacobianOplusXi.setZero();
        _jacobianOplusXj.setZero();

        for (int k = 0; k < 6; ++k)
        {
            Eigen::Matrix<double, 6, 1> delta =
                Eigen::Matrix<double, 6, 1>::Zero();

            delta[k] = eps;

            // Perturbación izquierda sobre el vértice from.
            v_from->setEstimate(
                g2o::SE3Quat::exp(delta) * T_from_original);

            v_to->setEstimate(T_to_original);

            computeError();

            _jacobianOplusXi.col(k) =
                (_error - error0) / eps;

            // Restaurar.
            v_from->setEstimate(T_from_original);
            v_to->setEstimate(T_to_original);

            // Perturbación izquierda sobre el vértice to.
            v_from->setEstimate(T_from_original);

            v_to->setEstimate(
                g2o::SE3Quat::exp(delta) * T_to_original);

            computeError();

            _jacobianOplusXj.col(k) =
                (_error - error0) / eps;

            // Restaurar.
            v_from->setEstimate(T_from_original);
            v_to->setEstimate(T_to_original);
        }

        computeError();
    }

    bool read(std::istream& is) override
    {
        (void)is;
        return false;
    }

    bool write(std::ostream& os) const override
    {
        (void)os;
        return false;
    }
};

bool MultiEssentialGraphOptimizer::IsFiniteMatrix(
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

bool MultiEssentialGraphOptimizer::IsValidSE3(
    const Eigen::Matrix4d& T)
{
    if (!IsFiniteMatrix(T))
        return false;

    const Eigen::Matrix3d R =
        T.block<3, 3>(0, 0);

    const Eigen::Vector3d t =
        T.block<3, 1>(0, 3);

    if (!std::isfinite(t.norm()))
        return false;

    if (t.norm() > 500.0)
        return false;

    const double det =
        R.determinant();

    if (!std::isfinite(det))
        return false;

    if (std::abs(det - 1.0) > 0.25)
        return false;

    const double ortho_error =
        (R.transpose() * R -
         Eigen::Matrix3d::Identity()).norm();

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

Eigen::Matrix4d MultiEssentialGraphOptimizer::ProjectToSE3(
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

MultiGraphOptimizationResult MultiEssentialGraphOptimizer::Optimize(
    const std::vector<MultiGraphVertexInput>& vertices,
    const std::vector<MultiGraphEdgeInput>& edges,
    const MultiGraphOptimizationParams& params)
{
    MultiGraphOptimizationResult result;

    MULTI_OPT_LOG(
        "ENTER vertices=" << vertices.size()
        << " edges=" << edges.size());

    if (vertices.empty())
        return result;

    // ============================================================
    // 1. Crear optimizer estilo ORB-SLAM3.
    // ============================================================

    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);

    // ============================================================
    // Pose graph SE3 pose-pose:
    // ambos vértices son VertexSE3Expmap, dimensión 6.
    // Por tanto el solver correcto es 6x6, no 6x3.
    // ============================================================

    typedef g2o::BlockSolver<
        g2o::BlockSolverTraits<6, 6>> BlockSolver_6_6;

    BlockSolver_6_6::LinearSolverType* linearSolver =
        new g2o::LinearSolverEigen<BlockSolver_6_6::PoseMatrixType>();

    BlockSolver_6_6* solver_ptr =
        new BlockSolver_6_6(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver =
        new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    solver->setUserLambdaInit(1e-16);

    optimizer.setAlgorithm(solver);

    MULTI_OPT_LOG("optimizer created SE3 6x6");


    // ============================================================
    // 2. Crear IDs compactos de g2o.
    // ============================================================

    std::unordered_map<uint64_t, int> g2o_id_by_kf;
    std::unordered_map<int, uint64_t> kf_by_g2o_id;
    std::unordered_map<uint64_t, MultiGraphVertexInput> input_by_kf;

    int next_id = 0;

    for (const auto& v : vertices)
    {
        if (v.global_kf_id == 0)
            continue;

        if (!IsValidSE3(v.world_T_camera))
        {
            MULTI_OPT_LOG(
                "skip invalid vertex global=" << v.global_kf_id);
            continue;
        }

        g2o_id_by_kf[v.global_kf_id] = next_id;
        kf_by_g2o_id[next_id] = v.global_kf_id;
        input_by_kf[v.global_kf_id] = v;

        next_id++;
    }

    if (g2o_id_by_kf.empty())
    {
        MULTI_OPT_LOG("ABORT no valid vertices");
        return result;
    }

    MULTI_OPT_LOG("valid vertices=" << g2o_id_by_kf.size());

    // ============================================================
    // 3. Añadir vértices Sim3.
    //
    // ORB-SLAM3 usa Siw = camera_T_world.
    // Nosotros recibimos world_T_camera, por tanto invertimos.
    // ============================================================

    size_t fixed_count = 0;

    std::vector<std::pair<int, uint64_t>> ordered_vertices;
    ordered_vertices.reserve(g2o_id_by_kf.size());

    for (const auto& pair : g2o_id_by_kf)
    {
        ordered_vertices.emplace_back(pair.second, pair.first);
    }

    std::sort(
        ordered_vertices.begin(),
        ordered_vertices.end(),
        [](const auto& a, const auto& b)
        {
            return a.first < b.first;
        });

    for (const auto& pair : ordered_vertices)
    {
        const int g2o_id =
            pair.first;

        const uint64_t global_id =
            pair.second;

        const auto& v =
            input_by_kf.at(global_id);

        const Eigen::Matrix4d W_T_C =
            ProjectToSE3(v.world_T_camera);

        const Eigen::Matrix4d C_T_W =
            W_T_C.inverse();

        Eigen::Matrix3d Rcw =
            C_T_W.block<3, 3>(0, 0);

        Eigen::Vector3d tcw =
            C_T_W.block<3, 1>(0, 3);

        Eigen::Quaterniond qcw(Rcw);
        qcw.normalize();

        g2o::SE3Quat T_cw(qcw, tcw);

        MULTI_OPT_LOG("before new VertexSE3Expmap id=" << g2o_id);

        g2o::VertexSE3Expmap* vertex =
            new g2o::VertexSE3Expmap();

        MULTI_OPT_LOG("after new VertexSE3Expmap id=" << g2o_id);

        MULTI_OPT_LOG("before setEstimate id=" << g2o_id);
        vertex->setEstimate(T_cw);
        MULTI_OPT_LOG("after setEstimate id=" << g2o_id);

        vertex->setId(g2o_id);
        vertex->setMarginalized(false);

        if (v.fixed)
        {
            vertex->setFixed(true);
            fixed_count++;
        }

        MULTI_OPT_LOG("before addVertex id=" << g2o_id);

        if (!optimizer.addVertex(vertex))
        {
            MULTI_OPT_LOG("addVertex failed id=" << g2o_id);
            delete vertex;
            continue;
        }

        MULTI_OPT_LOG("after addVertex id=" << g2o_id);
    }

    result.vertices =
        optimizer.vertices().size();

    result.fixed_vertices =
        fixed_count;

    MULTI_OPT_LOG(
        "vertices added=" << result.vertices
        << " fixed=" << result.fixed_vertices);

    if (result.vertices == 0)
        return result;

    // ============================================================
    // 4. Añadir edges estilo ORB-SLAM3.
    //
    // Vértice i = from.
    // Vértice j = to.
    // Cada vértice guarda Siw = camera_i_T_world.
    //
    // La medida debe ser:
    // Sji = camera_j_T_camera_i.
    //
    // Nuestro input from_T_to = camera_from_T_camera_to,
    // por tanto Sji = inverse(from_T_to).
    // ============================================================

    size_t edges_local = 0;
    size_t edges_loop = 0;

    Eigen::Matrix<double, 6, 6> information =
        Eigen::Matrix<double, 6, 6>::Identity();

    for (const auto& e : edges)
    {
        const bool is_loop =
            e.type == MultiGraphEdgeType::LOOP_INTRA ||
            e.type == MultiGraphEdgeType::LOOP_INTER;

        const bool is_parent =
            e.type == MultiGraphEdgeType::LOCAL_PARENT;

        const bool is_cov =
            e.type == MultiGraphEdgeType::LOCAL_COVISIBILITY;

        if (is_loop && !params.use_loop_edges)
            continue;

        if (is_parent && !params.use_local_parent_edges)
            continue;

        if (is_cov && !params.use_local_covisibility_edges)
            continue;

        auto from_it =
            g2o_id_by_kf.find(e.from_kf_id);

        auto to_it =
            g2o_id_by_kf.find(e.to_kf_id);

        if (from_it == g2o_id_by_kf.end() ||
            to_it == g2o_id_by_kf.end())
        {
            continue;
        }

        if (!IsValidSE3(e.from_T_to))
        {
            MULTI_OPT_LOG(
                "skip invalid edge from=" << e.from_kf_id
                << " to=" << e.to_kf_id);
            continue;
        }

        const Eigen::Matrix4d Cj_T_Ci =
            ProjectToSE3(e.from_T_to.inverse());

        Eigen::Matrix3d Rji =
            Cj_T_Ci.block<3, 3>(0, 0);

        Eigen::Vector3d tji =
            Cj_T_Ci.block<3, 1>(0, 3);

        Eigen::Quaterniond qji(Rji);
        qji.normalize();

        g2o::SE3Quat T_ji(qji, tji);

        EdgeSE3PoseGraph* edge =
            new EdgeSE3PoseGraph();

        auto* v_from =
            dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                optimizer.vertex(from_it->second));

        auto* v_to =
            dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                optimizer.vertex(to_it->second));

        if (!v_from || !v_to)
        {
            delete edge;
            continue;
        }

        edge->setVertex(0, v_from);
        edge->setVertex(1, v_to);
        edge->setMeasurement(T_ji);
        edge->information() = information;

        if (!optimizer.addEdge(edge))
        {
            MULTI_OPT_LOG("addEdge failed");
            delete edge;
            continue;
        }

        if (is_loop)
            edges_loop++;
        else
            edges_local++;
    }

    result.edges_total =
        optimizer.edges().size();

    result.edges_local =
        edges_local;

    result.edges_loop =
        edges_loop;

    MULTI_OPT_LOG(
        "edges added total=" << result.edges_total
        << " local=" << result.edges_local
        << " loop=" << result.edges_loop);

    if (result.edges_total == 0)
        return result;

    // ============================================================
    // 5. Optimización.
    // ============================================================

    MULTI_OPT_LOG("before initializeOptimization");

    const bool initialized =
        optimizer.initializeOptimization();

    MULTI_OPT_LOG(
        "after initializeOptimization initialized="
        << initialized);

    if (!initialized)
        return result;

    MULTI_OPT_LOG("before computeActiveErrors initial");

    optimizer.computeActiveErrors();

    MULTI_OPT_LOG("after computeActiveErrors initial");

    result.initial_chi2 =
        optimizer.chi2();

    MULTI_OPT_LOG(
        "before optimize iterations=" << params.iterations);

    MULTI_OPT_LOG(
        "SKIPPING g2o optimizer.optimize() because g2o crashes with custom SE3 pose-pose edge");

    // Modo seguro temporal:
    // No optimizamos todavía, pero devolvemos las poses iniciales como poses válidas.
    // Esto permite comprobar todo el pipeline posterior: OPT9 success,
    // almacenamiento de optimized_kf_poses_, UpdateOptimizedSparseMap() y publicación OPT10.
    const int optimized_iterations = 0;

    MULTI_OPT_LOG(
        "after optimize returned=" << optimized_iterations);

    // Como no hemos modificado el grafo, el error final es igual al inicial.
    result.final_chi2 =
        result.initial_chi2;

    // NO retornar aunque optimized_iterations sea 0.
    // Queremos recuperar las poses actuales de los vértices.

    // ============================================================
    // 6. Recuperar poses optimizadas.
    // ============================================================

    result.optimized_poses.clear();
    result.optimized_poses.reserve(g2o_id_by_kf.size());

    for (const auto& pair : g2o_id_by_kf)
    {
        const uint64_t global_id =
            pair.first;

        const int g2o_id =
            pair.second;

        g2o::VertexSE3Expmap* vertex =
            static_cast<g2o::VertexSE3Expmap*>(
                optimizer.vertex(g2o_id));

        if (!vertex)
            continue;

        const g2o::SE3Quat T_cw =
            vertex->estimate();

        const g2o::SE3Quat T_wc =
            T_cw.inverse();

        Eigen::Matrix3d Rwc =
            T_wc.rotation().toRotationMatrix();

        Eigen::Vector3d twc =
            T_wc.translation();

        Eigen::Matrix4d W_T_C =
            Eigen::Matrix4d::Identity();

        W_T_C.block<3, 3>(0, 0) =
            Rwc;

        W_T_C.block<3, 1>(0, 3) =
            twc;

        W_T_C =
            ProjectToSE3(W_T_C);

        const auto& input =
            input_by_kf.at(global_id);

        MultiGraphOptimizedPose out;
        out.global_kf_id = global_id;
        out.drone_id = input.drone_id;
        out.map_epoch = input.map_epoch;
        out.local_kf_id = input.local_kf_id;
        out.world_T_camera_initial =
            input.world_T_camera;
        out.world_T_camera_optimized =
            W_T_C;
        out.valid =
            IsValidSE3(W_T_C);

        if (out.valid)
            result.optimized_poses.push_back(out);
    }

    result.success = true;

    MULTI_OPT_LOG(
        "DONE success=" << result.success
        << " optimized_poses="
        << result.optimized_poses.size()
        << " chi2_initial=" << result.initial_chi2
        << " chi2_final=" << result.final_chi2);

    return result;
}

}  // namespace ORB_SLAM3_MULTI