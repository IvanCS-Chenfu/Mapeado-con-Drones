#include "orbslam3_multi/pose_graph_builder.hpp"

#include <Eigen/Geometry>
#include <Eigen/LU>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <vector>

namespace orbslam3_multi
{
namespace
{

RawSubmapId SubmapOf(const RawKeyFrameId& keyframe_id)
{
    return RawSubmapId{keyframe_id.drone_id, keyframe_id.map_epoch};
}

bool SameSubmap(const RawKeyFrameId& keyframe_id, const RawSubmapId& submap_id)
{
    return keyframe_id.drone_id == submap_id.drone_id &&
           keyframe_id.map_epoch == submap_id.map_epoch;
}

uint64_t AbsDiff(uint64_t a, uint64_t b)
{
    return a > b ? a - b : b - a;
}

constexpr double kPi = 3.14159265358979323846;

double NormalizeAngle(double angle)
{
    while (angle > kPi)
    {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi)
    {
        angle += 2.0 * kPi;
    }
    return angle;
}

double SpatialDistance(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b)
{
    return (a.block<3, 1>(0, 3) - b.block<3, 1>(0, 3)).norm();
}

double RotationAngle3D(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b)
{
    const Eigen::Matrix3d delta =
        a.block<3, 3>(0, 0).transpose() * b.block<3, 3>(0, 0);
    Eigen::AngleAxisd angle_axis(delta);
    return std::abs(NormalizeAngle(angle_axis.angle()));
}

double TranslationTurnAngle3D(const Eigen::Matrix4d& previous,
                              const Eigen::Matrix4d& current,
                              const Eigen::Matrix4d& next)
{
    const Eigen::Vector3d in =
        current.block<3, 1>(0, 3) - previous.block<3, 1>(0, 3);
    const Eigen::Vector3d out =
        next.block<3, 1>(0, 3) - current.block<3, 1>(0, 3);
    const double in_norm = in.norm();
    const double out_norm = out.norm();
    if (in_norm <= 1e-9 || out_norm <= 1e-9)
    {
        return 0.0;
    }
    const double cosine =
        std::max(-1.0, std::min(1.0, in.dot(out) / (in_norm * out_norm)));
    return std::acos(cosine);
}

double CornerAngle3D(const Eigen::Matrix4d& previous,
                     const Eigen::Matrix4d& current,
                     const Eigen::Matrix4d& next)
{
    const double translation_turn =
        TranslationTurnAngle3D(previous, current, next);
    const double rotation_turn =
        std::max(RotationAngle3D(previous, current),
                 RotationAngle3D(current, next));
    return std::max(translation_turn, rotation_turn);
}

double SupportRigidityMultiplier(double density_kfs_per_m)
{
    if (!std::isfinite(density_kfs_per_m) || density_kfs_per_m <= 0.0)
    {
        return 0.65;
    }

    // 1I: mas KFs por metro sugiere mas textura/seguimiento local. Saturamos
    // para que esta señal no domine fiduciales ni convierta un tramo curvado en
    // bloque indeformable.
    constexpr double kReferenceDensity = 4.0;
    const double raw = std::sqrt(density_kfs_per_m / kReferenceDensity);
    return std::max(0.65, std::min(2.25, raw));
}

Eigen::Matrix4d RelativeTransform(const Eigen::Matrix4d& world_T_a,
                                  const Eigen::Matrix4d& world_T_b)
{
    return world_T_a.inverse() * world_T_b;
}

PoseGraphProblemSummary Summarize(const PoseGraphProblem& problem)
{
    PoseGraphProblemSummary summary;
    summary.vertices = problem.vertices.size();
    summary.edges = problem.edges.size();
    summary.priors = problem.priors.size();
    summary.fixed_vertices = problem.fixed_keyframes.size();
    summary.variable_vertices = problem.variable_keyframes.size();
    summary.affected_non_variable_keyframes =
        problem.affected_non_variable_keyframes.size();
    summary.propagation_entries = problem.propagation_plan.size();
    for (const auto& vertex : problem.vertices)
    {
        if (vertex.is_hard_fiducial)
        {
            ++summary.hard_fiducial_vertices;
        }
    }
    return summary;
}

std::optional<int32_t> FiducialIdForKeyFrame(
    const RawMapDatabase& raw_db,
    const RawKeyFrameId& keyframe_id)
{
    const auto observations = raw_db.GetFiducialObservationJournalCopy();
    std::optional<int32_t> fiducial_id;
    for (const auto& observation : observations)
    {
        if (observation.drone_id == keyframe_id.drone_id &&
            observation.map_epoch == keyframe_id.map_epoch &&
            observation.local_keyframe_id == keyframe_id.local_kf_id)
        {
            fiducial_id = observation.fiducial_id;
        }
    }
    return fiducial_id;
}

uint64_t KeyFrameGap(const RawKeyFrameId& a, const RawKeyFrameId& b)
{
    return AbsDiff(a.local_kf_id, b.local_kf_id);
}

double CumulativeDistanceBetween(
    const std::map<RawKeyFrameId, double>& cumulative_distance,
    const RawKeyFrameId& a,
    const RawKeyFrameId& b)
{
    const auto a_it = cumulative_distance.find(a);
    const auto b_it = cumulative_distance.find(b);
    if (a_it == cumulative_distance.end() || b_it == cumulative_distance.end())
    {
        return 0.0;
    }
    return std::abs(b_it->second - a_it->second);
}

double CoverageCoordinateFor(
    const std::map<RawKeyFrameId, double>& coverage_coordinate,
    const RawKeyFrameId& keyframe_id)
{
    const auto it = coverage_coordinate.find(keyframe_id);
    if (it == coverage_coordinate.end())
    {
        return 0.0;
    }
    return it->second;
}

}  // namespace

const char* ToString(PoseGraphEdgeType type)
{
    switch (type)
    {
        case PoseGraphEdgeType::TemporalNeighbor:
            return "TEMPORAL_NEIGHBOR";
        case PoseGraphEdgeType::SoftConsistency:
            return "SOFT_CONSISTENCY";
        case PoseGraphEdgeType::LoopRelative:
            return "LOOP_RELATIVE";
    }
    return "UNKNOWN";
}

const char* ToString(PoseGraphPriorType type)
{
    switch (type)
    {
        case PoseGraphPriorType::FiducialHard:
            return "FIDUCIAL_HARD";
        case PoseGraphPriorType::FiducialTarget:
            return "FIDUCIAL_TARGET";
        case PoseGraphPriorType::CurrentPoseSoft:
            return "CURRENT_POSE_SOFT";
    }
    return "UNKNOWN";
}

const char* ToString(PoseGraphPropagationMode mode)
{
    switch (mode)
    {
        case PoseGraphPropagationMode::NearestControlVertex:
            return "NEAREST_CONTROL_VERTEX";
        case PoseGraphPropagationMode::PathSegment:
            return "PATH_SEGMENT";
        case PoseGraphPropagationMode::InheritLastCorrection:
            return "INHERIT_LAST_CORRECTION";
    }
    return "UNKNOWN";
}

const char* ToString(FiducialConnectivityEdgeStatus status)
{
    switch (status)
    {
        case FiducialConnectivityEdgeStatus::DirectObserved:
            return "DIRECT_OBSERVED";
        case FiducialConnectivityEdgeStatus::DirectUncertain:
            return "DIRECT_UNCERTAIN";
        case FiducialConnectivityEdgeStatus::SubdivisionCandidate:
            return "SUBDIVISION_CANDIDATE";
        case FiducialConnectivityEdgeStatus::SubdividedConfirmed:
            return "SUBDIVIDED_CONFIRMED";
        case FiducialConnectivityEdgeStatus::BypassConfirmed:
            return "BYPASS_CONFIRMED";
    }
    return "UNKNOWN";
}

PoseGraphBuilder::PoseGraphBuilder(const PoseGraphBuilderConfig& config)
    : config_(config)
{
}

void PoseGraphBuilder::Configure(const PoseGraphBuilderConfig& config)
{
    config_ = config;
}

const PoseGraphBuilderConfig& PoseGraphBuilder::GetConfig() const
{
    return config_;
}

PoseGraphBuildResult PoseGraphBuilder::BuildForLoopTask(
    const LoopOptimizationTask& task,
    const RawMapDatabase& raw_db,
    const GlobalPoseStore& pose_store,
    const CovisibilityDatabase* covisibility_db) const
{
    const auto& verification = task.verification;
    PoseGraphBuildResult result;
    if (verification.decision !=
            LoopGeometryDecision::LoopOptimizationCandidate ||
        !verification.geometry_confirmed)
    {
        result.reason = "loop_geometry_not_confirmed_for_optimization";
        return result;
    }
    const auto query_pose =
        pose_store.GetWorldPose(verification.query_kf_id);
    const auto candidate_pose =
        pose_store.GetWorldPose(verification.candidate_seed_kf_id);
    if (!query_pose || !candidate_pose ||
        !query_pose->allFinite() || !candidate_pose->allFinite() ||
        !verification.relative_pose_measured.allFinite())
    {
        result.reason = "loop_pose_input_missing_or_invalid";
        return result;
    }

    auto make_side_task = [&](const RawKeyFrameId& keyframe_id,
                              const RawSubmapId& submap_id,
                              const Eigen::Matrix4d& world_T_kf)
    {
        FiducialOptimizationTask side_task;
        side_task.task_id = task.task_id;
        side_task.task_type = "LOOP_OPTIMIZATION_SIDE";
        side_task.keyframe_id = keyframe_id;
        side_task.submap_id = submap_id;
        side_task.drone_id = keyframe_id.drone_id;
        side_task.map_epoch = keyframe_id.map_epoch;
        side_task.fiducial_id = -1;
        side_task.estimated_world_T_kf = world_T_kf;
        side_task.target_world_T_kf = world_T_kf;
        side_task.created_arrival_id = task.arrival_id;
        side_task.latest_arrival_id = task.arrival_id;
        side_task.arrival_id = task.arrival_id;
        side_task.created_source = task.source;
        side_task.latest_source = task.source;
        side_task.source = task.source;
        return side_task;
    };

    PoseGraphBuilderConfig loop_config = config_;
    loop_config.vertex_selection_ratio =
        std::min(loop_config.vertex_selection_ratio, 0.10);
    loop_config.fiducial_neighborhood_vertex_ratio =
        std::min(loop_config.fiducial_neighborhood_vertex_ratio, 0.10);
    PoseGraphBuilder loop_builder(loop_config);

    result = loop_builder.BuildForFiducialTask(
        make_side_task(
            verification.query_kf_id,
            verification.query_submap_id,
            query_pose.value()),
        raw_db,
        pose_store,
        covisibility_db);
    if (!result.success)
    {
        result.reason = "loop_query_" + result.reason;
        return result;
    }

    auto convert_target_prior = [&](PoseGraphPrior& prior)
    {
        if (prior.prior_type != PoseGraphPriorType::FiducialTarget)
        {
            return;
        }
        prior.prior_type = PoseGraphPriorType::CurrentPoseSoft;
        prior.hard = false;
        prior.weight_translation = loop_config.current_pose_soft_weight;
        prior.weight_rotation = loop_config.current_pose_soft_weight;
        prior.source = "F1Q_CURRENT_POSE";
    };
    for (auto& prior : result.problem.priors)
    {
        convert_target_prior(prior);
    }
    for (auto& vertex : result.problem.vertices)
    {
        if (vertex.keyframe_id == verification.query_kf_id)
        {
            vertex.selection_reason = "loop_query";
        }
    }

    auto find_vertex = [&](const RawKeyFrameId& id)
        -> std::vector<PoseGraphVertex>::iterator
    {
        return std::find_if(
            result.problem.vertices.begin(),
            result.problem.vertices.end(),
            [&id](const PoseGraphVertex& vertex)
            {
                return vertex.keyframe_id == id;
            });
    };

    if (!(verification.query_submap_id == verification.candidate_submap_id))
    {
        auto candidate_side = loop_builder.BuildForFiducialTask(
            make_side_task(
                verification.candidate_seed_kf_id,
                verification.candidate_submap_id,
                candidate_pose.value()),
            raw_db,
            pose_store,
            covisibility_db);
        if (!candidate_side.success)
        {
            result.success = false;
            result.reason = "loop_candidate_" + candidate_side.reason;
            return result;
        }

        for (auto vertex : candidate_side.problem.vertices)
        {
            if (find_vertex(vertex.keyframe_id) != result.problem.vertices.end())
            {
                continue;
            }
            vertex.is_fixed = true;
            vertex.is_variable = false;
            vertex.selection_reason =
                vertex.keyframe_id == verification.candidate_seed_kf_id
                    ? "loop_candidate_reference"
                    : "loop_candidate_fixed_window";
            result.problem.vertices.push_back(std::move(vertex));
        }
        for (auto edge : candidate_side.problem.edges)
        {
            edge.edge_id = result.problem.edges.size() + 1U;
            result.problem.edges.push_back(std::move(edge));
        }
        for (auto prior : candidate_side.problem.priors)
        {
            convert_target_prior(prior);
            result.problem.priors.push_back(std::move(prior));
        }
    }

    auto candidate_vertex = find_vertex(verification.candidate_seed_kf_id);
    if (candidate_vertex == result.problem.vertices.end())
    {
        PoseGraphVertex vertex;
        vertex.keyframe_id = verification.candidate_seed_kf_id;
        vertex.submap_id = verification.candidate_submap_id;
        vertex.initial_world_T_kf = candidate_pose.value();
        vertex.is_fixed = true;
        vertex.is_variable = false;
        vertex.is_hard_fiducial = pose_store.IsHardFiducialKeyFrame(
            verification.candidate_seed_kf_id);
        vertex.selection_reason = "loop_candidate_reference";
        vertex.weight = loop_config.fiducial_hard_weight;
        result.problem.vertices.push_back(std::move(vertex));
        candidate_vertex = find_vertex(verification.candidate_seed_kf_id);
    }
    candidate_vertex->is_fixed = true;
    candidate_vertex->is_variable = false;
    candidate_vertex->selection_reason = "loop_candidate_reference";

    PoseGraphEdge loop_edge;
    loop_edge.edge_id = result.problem.edges.size() + 1U;
    loop_edge.from_keyframe_id = verification.query_kf_id;
    loop_edge.to_keyframe_id = verification.candidate_seed_kf_id;
    loop_edge.edge_type = PoseGraphEdgeType::LoopRelative;
    const Eigen::Matrix4d measured_world_T_query =
        verification.relative_pose_measured * query_pose.value();
    loop_edge.relative_T_from_to =
        measured_world_T_query.inverse() * candidate_pose.value();
    loop_edge.weight = std::max(
        1.0,
        static_cast<double>(verification.ransac_inliers) *
            std::max(0.05, verification.loop_confidence) /
            (1.0 + std::max(0.0, verification.mean_residual)));
    loop_edge.support_keyframe_count = verification.ransac_inliers;
    loop_edge.source = "F1Q_LOOP_RELATIVE";
    result.problem.edges.push_back(std::move(loop_edge));

    result.problem.fixed_keyframes.clear();
    result.problem.variable_keyframes.clear();
    for (const auto& vertex : result.problem.vertices)
    {
        if (vertex.is_fixed)
        {
            result.problem.fixed_keyframes.push_back(vertex.keyframe_id);
        }
        else if (vertex.is_variable)
        {
            result.problem.variable_keyframes.push_back(vertex.keyframe_id);
        }
    }
    result.problem.task_type = "LOOP_OPTIMIZATION";
    result.problem.source = task.source;
    result.problem.target_keyframe_id = verification.query_kf_id;
    result.problem.summary = Summarize(result.problem);
    result.reason = "loop_relative_pose_graph_created";
    return result;
}

PoseGraphBuildResult PoseGraphBuilder::BuildForFiducialTask(
    const FiducialOptimizationTask& task,
    const RawMapDatabase& raw_db,
    const GlobalPoseStore& pose_store,
    const CovisibilityDatabase* covisibility_db) const
{
    // F1I: la ventana se calcula solo en el submapa de la tarea. No se expande
    // a loops ni a otros drones porque esas evidencias aun no existen en 1I.
    PoseGraphBuildResult result;
    result.problem.task_id = task.task_id;
    result.problem.task_type = task.task_type;
    result.problem.source = task.source;
    result.problem.submap_id = task.submap_id;
    result.problem.target_keyframe_id = task.keyframe_id;

    if (!SameSubmap(task.keyframe_id, task.submap_id))
    {
        result.reason = "task_keyframe_not_in_task_submap";
        return result;
    }
    if (!raw_db.HasSubmap(task.submap_id))
    {
        result.reason = "raw_submap_missing";
        return result;
    }
    if (!pose_store.GetWorldPose(task.keyframe_id))
    {
        result.reason = "target_world_pose_missing";
        return result;
    }

    std::vector<RawKeyFrameId> all_keyframes =
        raw_db.GetKeyFrameIdsForSubmap(task.submap_id);
    std::sort(all_keyframes.begin(), all_keyframes.end());
    if (all_keyframes.empty())
    {
        result.reason = "submap_without_keyframes";
        return result;
    }

    const auto target_it =
        std::find(all_keyframes.begin(), all_keyframes.end(), task.keyframe_id);
    if (target_it == all_keyframes.end())
    {
        result.reason = "target_keyframe_missing";
        return result;
    }

    std::vector<RawKeyFrameId> world_keyframes;
    world_keyframes.reserve(all_keyframes.size());
    for (const auto& keyframe_id : all_keyframes)
    {
        if (pose_store.GetWorldPose(keyframe_id))
        {
            world_keyframes.push_back(keyframe_id);
        }
    }
    if (world_keyframes.empty())
    {
        result.reason = "window_without_world_poses";
        return result;
    }

    const uint64_t target_local_id = task.keyframe_id.local_kf_id;
    std::vector<RawKeyFrameId> hard_fiducial_keyframes;
    for (const auto& keyframe_id : world_keyframes)
    {
        if (keyframe_id == task.keyframe_id)
        {
            continue;
        }
        const auto fiducial_id = FiducialIdForKeyFrame(raw_db, keyframe_id);
        if (pose_store.IsHardFiducialKeyFrame(keyframe_id) &&
            (!fiducial_id || fiducial_id.value() != task.fiducial_id))
        {
            hard_fiducial_keyframes.push_back(keyframe_id);
        }
    }

    // 1I: la seleccion topologica minima es temporal y conservadora.
    // Elegimos como fronteras independientes el hard fiducial mas cercano antes
    // y despues del target. Esto evita trasladar libremente una ventana sin
    // fijos, y descarta fiduciales mas lejanos dominados por uno mas cercano en
    // el mismo camino temporal.
    std::vector<RawKeyFrameId> branch_anchor_keyframes;
    std::optional<RawKeyFrameId> nearest_left_hard;
    std::optional<RawKeyFrameId> nearest_right_hard;
    for (const auto& keyframe_id : hard_fiducial_keyframes)
    {
        if (keyframe_id.local_kf_id < target_local_id)
        {
            if (!nearest_left_hard ||
                keyframe_id.local_kf_id > nearest_left_hard->local_kf_id)
            {
                nearest_left_hard = keyframe_id;
            }
        }
        else if (keyframe_id.local_kf_id > target_local_id)
        {
            if (!nearest_right_hard ||
                keyframe_id.local_kf_id < nearest_right_hard->local_kf_id)
            {
                nearest_right_hard = keyframe_id;
            }
        }
    }
    // El grafo fiducial usa un unico inicio: el hard fiducial temporalmente
    // anterior mas cercano. El fallback derecho solo cubre datasets antiguos
    // donde el orden local quede invertido respecto a la tarea.
    if (nearest_left_hard)
    {
        branch_anchor_keyframes.push_back(nearest_left_hard.value());
    }
    else if (nearest_right_hard)
    {
        branch_anchor_keyframes.push_back(nearest_right_hard.value());
    }

    result.problem.anchor_preservation.required =
        !hard_fiducial_keyframes.empty();
    result.problem.anchor_preservation.independent_branches =
        branch_anchor_keyframes.size();
    result.problem.anchor_preservation.branch_anchor_count =
        branch_anchor_keyframes.size();
    result.problem.anchor_preservation.reason =
        branch_anchor_keyframes.empty()
            ? "no_previous_fiducial_branch_anchor"
            : "branch_anchors_selected";

    const auto target_fiducial_id = task.fiducial_id;
    for (const auto& anchor : branch_anchor_keyframes)
    {
        FiducialConnectivityEdge edge;
        edge.from_keyframe_id = anchor;
        edge.to_keyframe_id = task.keyframe_id;
        edge.from_fiducial_id = FiducialIdForKeyFrame(raw_db, anchor).value_or(-1);
        edge.to_fiducial_id = target_fiducial_id;
        edge.selected_as_branch_anchor = true;
        edge.independent_branch = true;
        edge.kf_gap = KeyFrameGap(anchor, task.keyframe_id);
        edge.status = FiducialConnectivityEdgeStatus::DirectObserved;
        edge.reason = "selected_nearest_previous_fiducial_anchor";
        result.problem.fiducial_connectivity_edges.push_back(edge);
    }
    for (const auto& hard : hard_fiducial_keyframes)
    {
        if (std::find(branch_anchor_keyframes.begin(),
                      branch_anchor_keyframes.end(),
                      hard) != branch_anchor_keyframes.end())
        {
            continue;
        }
        FiducialConnectivityEdge edge;
        edge.from_keyframe_id = hard;
        edge.to_keyframe_id = task.keyframe_id;
        edge.from_fiducial_id = FiducialIdForKeyFrame(raw_db, hard).value_or(-1);
        edge.to_fiducial_id = target_fiducial_id;
        edge.kf_gap = KeyFrameGap(hard, task.keyframe_id);
        edge.status = FiducialConnectivityEdgeStatus::SubdividedConfirmed;
        edge.reason = "dominated_by_nearer_fiducial_in_same_temporal_branch";
        result.problem.fiducial_connectivity_edges.push_back(edge);
        ++result.problem.anchor_preservation.subdivided_confirmed;
    }

    if (branch_anchor_keyframes.empty())
    {
        result.reason = "previous_fiducial_anchor_missing";
        return result;
    }

    const uint64_t anchor_local_id =
        branch_anchor_keyframes.front().local_kf_id;
    const uint64_t left_id = std::min(anchor_local_id, target_local_id);
    const uint64_t right_id = std::max(anchor_local_id, target_local_id);

    std::vector<RawKeyFrameId> window_keyframes;
    for (const auto& keyframe_id : world_keyframes)
    {
        if (keyframe_id.local_kf_id >= left_id &&
            keyframe_id.local_kf_id <= right_id)
        {
            window_keyframes.push_back(keyframe_id);
        }
    }
    if (window_keyframes.empty())
    {
        result.reason = "empty_window";
        return result;
    }

    std::map<RawKeyFrameId, Eigen::Matrix4d> window_poses;
    for (const auto& keyframe_id : window_keyframes)
    {
        const auto world_pose = pose_store.GetWorldPose(keyframe_id);
        if (world_pose)
        {
            window_poses[keyframe_id] = world_pose.value();
        }
    }
    std::map<RawKeyFrameId, double> cumulative_distance;
    double cumulative = 0.0;
    for (size_t i = 0; i < window_keyframes.size(); ++i)
    {
        if (i > 0)
        {
            const auto prev_it = window_poses.find(window_keyframes[i - 1]);
            const auto curr_it = window_poses.find(window_keyframes[i]);
            if (prev_it != window_poses.end() && curr_it != window_poses.end())
            {
                cumulative += SpatialDistance(prev_it->second, curr_it->second);
            }
        }
        cumulative_distance[window_keyframes[i]] = cumulative;
    }
    const double total_distance = cumulative_distance.empty()
        ? 0.0
        : cumulative_distance.rbegin()->second;
    std::map<RawKeyFrameId, double> coverage_coordinate;
    const double last_index =
        static_cast<double>(std::max<size_t>(1U, window_keyframes.size() - 1U));
    for (size_t i = 0; i < window_keyframes.size(); ++i)
    {
        const auto& keyframe_id = window_keyframes[i];
        const double temporal_coordinate = static_cast<double>(i) / last_index;
        const double metric_coordinate = total_distance > 1e-9
            ? cumulative_distance[keyframe_id] / total_distance
            : temporal_coordinate;
        // Mezcla sin umbrales absolutos: la distancia acumulada reparte por el
        // espacio recorrido y el indice temporal evita dejar sin controles
        // tramos con muchos KFs pero poco desplazamiento estimado.
        const double blended_coordinate = total_distance > 1e-9
            ? 0.70 * metric_coordinate + 0.30 * temporal_coordinate
            : temporal_coordinate;
        coverage_coordinate[keyframe_id] =
            std::max(0.0, std::min(1.0, blended_coordinate));
    }

    // Los dos KFs fiduciales son controles obligatorios. El resto se muestrea
    // sobre la longitud acumulada 3D para cubrir toda la trayectoria sin
    // imponer un numero maximo absoluto de vertices.
    std::set<RawKeyFrameId> selected;
    selected.insert(branch_anchor_keyframes.front());
    selected.insert(task.keyframe_id);
    std::set<RawKeyFrameId> anchor_neighborhood_keyframes;
    std::set<RawKeyFrameId> target_neighborhood_keyframes;
    std::map<RawKeyFrameId, double> corner_scores;
    std::set<RawKeyFrameId> selected_corner_keyframes;
    const uint64_t available = window_keyframes.size();
    const double selection_ratio =
        std::max(0.0, std::min(1.0, config_.vertex_selection_ratio));
    uint64_t desired_vertices = static_cast<uint64_t>(
        std::ceil(selection_ratio * static_cast<double>(available)));
    desired_vertices = std::max<uint64_t>(config_.min_vertices, desired_vertices);
    desired_vertices = std::max<uint64_t>(selected.size(), desired_vertices);
    desired_vertices = std::min<uint64_t>(available, desired_vertices);

    // 1I: proteger vecindades pares alrededor de ambos fiduciales con una
    // proporcion de los controles, no con un umbral absoluto de metros. Esos
    // vertices se fuerzan antes del muestreo de cobertura.
    uint64_t protected_total = static_cast<uint64_t>(std::floor(
        std::max(0.0, config_.fiducial_neighborhood_vertex_ratio) *
        static_cast<double>(desired_vertices)));
    if (protected_total > 0U && protected_total % 2U != 0U)
    {
        --protected_total;
    }
    const uint64_t protected_per_fiducial = protected_total / 2U;
    auto add_nearest_neighborhood =
        [&](const RawKeyFrameId& fiducial,
            uint64_t count,
            std::set<RawKeyFrameId>& neighborhood)
    {
        std::vector<std::pair<double, RawKeyFrameId>> candidates;
        const auto fiducial_distance_it = cumulative_distance.find(fiducial);
        if (fiducial_distance_it == cumulative_distance.end())
        {
            return;
        }
        for (const auto& keyframe_id : window_keyframes)
        {
            if (keyframe_id == fiducial ||
                selected.find(keyframe_id) != selected.end())
            {
                continue;
            }
            const auto distance_it = cumulative_distance.find(keyframe_id);
            if (distance_it == cumulative_distance.end())
            {
                continue;
            }
            candidates.emplace_back(
                std::abs(distance_it->second - fiducial_distance_it->second),
                keyframe_id);
        }
        std::sort(
            candidates.begin(),
            candidates.end(),
            [](const auto& a, const auto& b)
            {
                if (a.first == b.first)
                {
                    return a.second.local_kf_id < b.second.local_kf_id;
                }
                return a.first < b.first;
            });
        for (const auto& candidate : candidates)
        {
            if (neighborhood.size() >= count)
            {
                break;
            }
            neighborhood.insert(candidate.second);
            selected.insert(candidate.second);
        }
    };
    add_nearest_neighborhood(
        branch_anchor_keyframes.front(),
        protected_per_fiducial,
        anchor_neighborhood_keyframes);
    add_nearest_neighborhood(
        task.keyframe_id,
        protected_per_fiducial,
        target_neighborhood_keyframes);

    // 1I: las esquinas se detectan con geometria 3D/SE(3), no solo yaw. Son
    // candidatas preferentes, pero no fuerzan todos los KFs de una esquina como
    // vertices: la cobertura manda para evitar racimos que dejen huecos.
    for (size_t i = 1; i + 1 < window_keyframes.size(); ++i)
    {
        const auto prev_it = window_poses.find(window_keyframes[i - 1]);
        const auto curr_it = window_poses.find(window_keyframes[i]);
        const auto next_it = window_poses.find(window_keyframes[i + 1]);
        if (prev_it == window_poses.end() ||
            curr_it == window_poses.end() ||
            next_it == window_poses.end())
        {
            continue;
        }
        const double corner_score =
            CornerAngle3D(prev_it->second, curr_it->second, next_it->second);
        if (corner_score >= config_.corner_yaw_threshold_rad)
        {
            corner_scores[window_keyframes[i]] = corner_score;
        }
    }

    auto select_next_balanced_vertex =
        [&]() -> std::optional<RawKeyFrameId>
    {
        struct Gap
        {
            double begin = 0.0;
            double end = 0.0;
            double length = 0.0;
        };

        std::vector<std::pair<double, RawKeyFrameId>> ordered_controls;
        ordered_controls.reserve(selected.size());
        for (const auto& control : selected)
        {
            ordered_controls.emplace_back(
                CoverageCoordinateFor(coverage_coordinate, control),
                control);
        }
        std::sort(
            ordered_controls.begin(),
            ordered_controls.end(),
            [](const auto& a, const auto& b)
            {
                if (std::abs(a.first - b.first) <= 1e-12)
                {
                    return a.second.local_kf_id < b.second.local_kf_id;
                }
                return a.first < b.first;
            });

        std::vector<Gap> gaps;
        for (size_t i = 1; i < ordered_controls.size(); ++i)
        {
            const double begin = ordered_controls[i - 1].first;
            const double end = ordered_controls[i].first;
            if (end > begin + 1e-12)
            {
                gaps.push_back(Gap{begin, end, end - begin});
            }
        }
        std::sort(
            gaps.begin(),
            gaps.end(),
            [](const Gap& a, const Gap& b)
            {
                return a.length > b.length;
            });

        for (const auto& gap : gaps)
        {
            std::optional<RawKeyFrameId> best;
            double best_score = -std::numeric_limits<double>::infinity();
            const double center = 0.5 * (gap.begin + gap.end);
            const double half_gap = std::max(1e-12, 0.5 * gap.length);
            for (const auto& candidate : window_keyframes)
            {
                if (selected.find(candidate) != selected.end())
                {
                    continue;
                }
                const double coordinate =
                    CoverageCoordinateFor(coverage_coordinate, candidate);
                if (coordinate <= gap.begin + 1e-12 ||
                    coordinate >= gap.end - 1e-12)
                {
                    continue;
                }
                const double centrality =
                    1.0 - std::min(1.0, std::abs(coordinate - center) / half_gap);
                const auto corner_it = corner_scores.find(candidate);
                const double corner_bonus = corner_it != corner_scores.end()
                    ? 0.35 * std::min(1.0, corner_it->second / kPi)
                    : 0.0;
                const double score = centrality + corner_bonus;
                if (!best || score > best_score ||
                    (std::abs(score - best_score) <= 1e-12 &&
                     candidate.local_kf_id < best->local_kf_id))
                {
                    best = candidate;
                    best_score = score;
                }
            }
            if (best)
            {
                return best;
            }
        }

        std::optional<RawKeyFrameId> best;
        double best_min_distance = -1.0;
        double best_corner_score = -1.0;
        for (const auto& candidate : window_keyframes)
        {
            if (selected.find(candidate) != selected.end())
            {
                continue;
            }
            double min_distance = std::numeric_limits<double>::max();
            for (const auto& control : selected)
            {
                min_distance = std::min(
                    min_distance,
                    std::abs(
                        CoverageCoordinateFor(coverage_coordinate, candidate) -
                        CoverageCoordinateFor(coverage_coordinate, control)));
            }
            const auto corner_it = corner_scores.find(candidate);
            const double corner_score =
                corner_it != corner_scores.end() ? corner_it->second : 0.0;
            if (!best || min_distance > best_min_distance ||
                (std::abs(min_distance - best_min_distance) <= 1e-12 &&
                 corner_score > best_corner_score))
            {
                best = candidate;
                best_min_distance = min_distance;
                best_corner_score = corner_score;
            }
        }
        return best;
    };

    // Completa el porcentaje con muestreo de huecos: en cada iteracion parte el
    // mayor tramo sin control en coordenada relativa ventana/tiempo. Dentro del
    // hueco, una esquina 3D fuerte gana preferencia, pero no puede saturar una
    // zona compacta porque tras seleccionarla ese hueco deja de ser el mayor.
    while (selected.size() < desired_vertices)
    {
        const auto best = select_next_balanced_vertex();
        if (!best)
        {
            break;
        }
        selected.insert(best.value());
        if (corner_scores.find(best.value()) != corner_scores.end())
        {
            selected_corner_keyframes.insert(best.value());
        }
    }

    std::vector<RawKeyFrameId> selected_ordered(selected.begin(), selected.end());
    std::sort(selected_ordered.begin(), selected_ordered.end());

    result.problem.coverage.window_keyframes = window_keyframes.size();
    result.problem.coverage.control_vertices = selected_ordered.size();
    result.problem.coverage.max_vertices_exhausted = false;
    if (selected.find(task.keyframe_id) == selected.end())
    {
        ++result.problem.coverage.mandatory_vertices_missing;
    }
    for (const auto& anchor : branch_anchor_keyframes)
    {
        if (selected.find(anchor) == selected.end())
        {
            ++result.problem.coverage.mandatory_vertices_missing;
        }
    }
    for (size_t i = 1; i < selected_ordered.size(); ++i)
    {
        const auto& from = selected_ordered[i - 1];
        const auto& to = selected_ordered[i];
        const uint64_t gap = KeyFrameGap(from, to);
        const double segment_length =
            CumulativeDistanceBetween(cumulative_distance, from, to);
        result.problem.coverage.max_control_span_kfs =
            std::max<uint64_t>(result.problem.coverage.max_control_span_kfs, gap);
        result.problem.coverage.max_control_span_m =
            std::max<double>(result.problem.coverage.max_control_span_m, segment_length);
    }
    result.problem.coverage.coverage_complete =
        result.problem.coverage.mandatory_vertices_missing == 0U &&
        result.problem.coverage.uncovered_long_segments == 0U;
    if (!result.problem.coverage.coverage_complete)
    {
        result.problem.coverage.reason =
            result.problem.coverage.mandatory_vertices_missing > 0U
                ? "mandatory_vertices_missing"
                : "bad_window_coverage";
        result.reason = result.problem.coverage.reason;
        result.problem.summary = Summarize(result.problem);
        return result;
    }
    result.problem.coverage.reason = "balanced_coverage_sample";

    std::map<RawKeyFrameId, Eigen::Matrix4d> selected_poses;
    for (const auto& keyframe_id : selected_ordered)
    {
        const auto world_pose = pose_store.GetWorldPose(keyframe_id);
        if (!world_pose)
        {
            continue;
        }

        const bool hard = pose_store.IsHardFiducialKeyFrame(keyframe_id);
        const bool previous_anchor =
            std::find(branch_anchor_keyframes.begin(),
                      branch_anchor_keyframes.end(),
                      keyframe_id) != branch_anchor_keyframes.end();
        const bool previous_fiducial_neighborhood =
            anchor_neighborhood_keyframes.find(keyframe_id) !=
            anchor_neighborhood_keyframes.end();
        const bool target_fiducial_neighborhood =
            target_neighborhood_keyframes.find(keyframe_id) !=
            target_neighborhood_keyframes.end();
        const bool anchor_neighborhood =
            previous_fiducial_neighborhood || target_fiducial_neighborhood;
        PoseGraphVertex vertex;
        vertex.keyframe_id = keyframe_id;
        vertex.submap_id = SubmapOf(keyframe_id);
        vertex.initial_world_T_kf = world_pose.value();
        vertex.is_hard_fiducial = hard;
        vertex.is_anchor_neighborhood = anchor_neighborhood;
        // 1I: los vecinos de ambos fiduciales se tratan simetricamente como
        // bloques relativos al fiducial, no como poses absolutas fijas.
        // El fiducial/ancla sigue fijo; su vecindad entra como variable con
        // un lock fuerte en OptimizationManager.
        vertex.is_fixed = hard || previous_anchor;
        vertex.is_variable = !vertex.is_fixed;
        vertex.weight = hard ? config_.fiducial_hard_weight : config_.current_pose_soft_weight;
        vertex.support_count = 1;
        if (keyframe_id == task.keyframe_id)
        {
            vertex.selection_reason = "target_fiducial_error";
        }
        else if (previous_anchor)
        {
            vertex.selection_reason = "previous_fiducial_anchor";
        }
        else if (hard)
        {
            vertex.selection_reason = "hard_fiducial_boundary";
        }
        else if (previous_fiducial_neighborhood)
        {
            vertex.selection_reason = "previous_fiducial_neighborhood";
        }
        else if (target_fiducial_neighborhood)
        {
            vertex.selection_reason = "target_fiducial_neighborhood";
        }
        else if (keyframe_id == window_keyframes.front() || keyframe_id == window_keyframes.back())
        {
            vertex.selection_reason = "window_boundary";
        }
        else if (selected.find(keyframe_id) != selected.end())
        {
            vertex.selection_reason =
                selected_corner_keyframes.find(keyframe_id) !=
                    selected_corner_keyframes.end()
                    ? "corner_3d_vertex"
                    : "balanced_coverage_sample";
        }
        else
        {
            vertex.selection_reason = "temporal_sample";
        }

        if (vertex.is_fixed)
        {
            result.problem.fixed_keyframes.push_back(keyframe_id);
            if (vertex.selection_reason == "previous_fiducial_anchor")
            {
                ++result.problem.anchor_preservation.previous_fiducial_fixed_count;
            }
            if (vertex.is_anchor_neighborhood)
            {
                ++result.problem.anchor_preservation
                      .previous_fiducial_neighborhood_fixed_count;
            }
        }
        if (vertex.is_variable)
        {
            result.problem.variable_keyframes.push_back(keyframe_id);
        }
        selected_poses[keyframe_id] = world_pose.value();
        result.problem.vertices.push_back(vertex);
    }

    if (result.problem.vertices.size() < config_.min_vertices)
    {
        result.reason = "not_enough_vertices";
        return result;
    }

    if (config_.include_temporal_edges)
    {
        uint64_t edge_id = 1;
        // Las aristas conectan controles consecutivos. Los KFs intermedios se
        // conservan en el PropagationPlan y aportan soporte a la restriccion.
        for (size_t i = 1; i < selected_ordered.size(); ++i)
        {
            const auto from_it = selected_poses.find(selected_ordered[i - 1]);
            const auto to_it = selected_poses.find(selected_ordered[i]);
            if (from_it == selected_poses.end() || to_it == selected_poses.end())
            {
                continue;
            }

            PoseGraphEdge edge;
            edge.edge_id = edge_id++;
            edge.from_keyframe_id = selected_ordered[i - 1];
            edge.to_keyframe_id = selected_ordered[i];
            edge.edge_type = PoseGraphEdgeType::TemporalNeighbor;
            edge.relative_T_from_to = RelativeTransform(from_it->second, to_it->second);
            const double segment_length =
                CumulativeDistanceBetween(
                    cumulative_distance,
                    edge.from_keyframe_id,
                    edge.to_keyframe_id);
            const auto from_window_it = std::find(
                window_keyframes.begin(),
                window_keyframes.end(),
                edge.from_keyframe_id);
            const auto to_window_it = std::find(
                window_keyframes.begin(),
                window_keyframes.end(),
                edge.to_keyframe_id);
            const uint64_t support_keyframes =
                from_window_it != window_keyframes.end() &&
                to_window_it != window_keyframes.end() &&
                from_window_it <= to_window_it
                    ? static_cast<uint64_t>(
                          std::distance(from_window_it, to_window_it)) + 1U
                    : 2U;
            edge.intermediate_keyframe_count =
                support_keyframes > 2U ? support_keyframes - 2U : 0U;
            edge.support_keyframe_count = support_keyframes;
            edge.support_length_m = segment_length;
            edge.support_density_kfs_per_m =
                segment_length > 1e-9
                    ? static_cast<double>(edge.support_keyframe_count) / segment_length
                    : static_cast<double>(edge.support_keyframe_count);
            edge.support_rigidity_multiplier =
                SupportRigidityMultiplier(edge.support_density_kfs_per_m);
            const double base_weight = edge.intermediate_keyframe_count == 0U
                ? config_.temporal_edge_weight
                : config_.temporal_edge_weight_sparse;
            edge.weight = base_weight * edge.support_rigidity_multiplier;
            edge.source = "F1I_TEMPORAL_WINDOW";
            result.problem.edges.push_back(edge);
        }

        // F1M: las relaciones confirmadas se convierten en restricciones extra
        // solo cuando ambos extremos ya pertenecen a la ventana seleccionada.
        // No crean vertices y no sustituyen las aristas temporales de 1I.
        if (covisibility_db)
        {
            const auto covisibility_edges = covisibility_db->GetEdgesForWindow(
                selected_ordered,
                config_.covisibility_min_weight);
            for (const auto& covisibility_edge : covisibility_edges)
            {
                PoseGraphEdge edge;
                edge.edge_id = edge_id++;
                edge.from_keyframe_id = covisibility_edge.kf_a;
                edge.to_keyframe_id = covisibility_edge.kf_b;
                edge.edge_type = PoseGraphEdgeType::SoftConsistency;
                edge.relative_T_from_to = covisibility_edge.relative_pose_current;
                edge.weight = covisibility_edge.information_weight *
                    config_.covisibility_edge_weight_scale;
                edge.support_keyframe_count =
                    covisibility_edge.shared_mappoints_or_inliers;
                edge.source = std::string("F1M_") +
                    ToString(covisibility_edge.source);
                result.problem.edges.push_back(edge);
            }
        }
    }

    for (const auto& vertex : result.problem.vertices)
    {
        PoseGraphPrior prior;
        prior.keyframe_id = vertex.keyframe_id;
        prior.target_world_T_kf = vertex.initial_world_T_kf;
        prior.source = "F1I_CURRENT_POSE";
        prior.weight_translation = config_.current_pose_soft_weight;
        prior.weight_rotation = config_.current_pose_soft_weight;

        if (vertex.is_hard_fiducial)
        {
            prior.prior_type = PoseGraphPriorType::FiducialHard;
            prior.hard = true;
            prior.weight_translation = config_.fiducial_hard_weight;
            prior.weight_rotation = config_.fiducial_hard_weight;
            prior.source = "F1I_HARD_FIDUCIAL";
        }
        else if (vertex.keyframe_id == task.keyframe_id)
        {
            prior.prior_type = PoseGraphPriorType::FiducialTarget;
            prior.target_world_T_kf = task.target_world_T_kf;
            prior.weight_translation = config_.fiducial_target_translation_weight;
            prior.weight_rotation = config_.fiducial_target_rotation_weight;
            prior.source = "F1I_FIDUCIAL_TARGET";
        }
        result.problem.priors.push_back(prior);
    }

    // 1I: los KFs de la ventana que no son vertices se guardan en un plan de
    // propagacion. 1J calcula una propuesta con este plan y 1K la escribe si
    // el apply pasa sus prechecks.
    for (const auto& keyframe_id : window_keyframes)
    {
        if (selected.find(keyframe_id) != selected.end())
        {
            continue;
        }

        result.problem.affected_non_variable_keyframes.push_back(keyframe_id);

        PoseGraphPropagationPlanEntry entry;
        entry.affected_keyframe_id = keyframe_id;
        entry.submap_id = task.submap_id;
        entry.path_id = task.task_id;
        entry.reason = "window_non_vertex_keyframe";

        RawKeyFrameId before = selected_ordered.front();
        RawKeyFrameId after = selected_ordered.back();
        bool has_before = false;
        bool has_after = false;
        for (const auto& control : selected_ordered)
        {
            if (control.local_kf_id <= keyframe_id.local_kf_id)
            {
                before = control;
                has_before = true;
            }
            if (control.local_kf_id >= keyframe_id.local_kf_id)
            {
                after = control;
                has_after = true;
                break;
            }
        }

        if (has_before && has_after && !(before == after))
        {
            entry.control_vertex_a = before;
            entry.control_vertex_b = after;
            entry.mode = PoseGraphPropagationMode::PathSegment;
            entry.distance_from_a_m =
                CumulativeDistanceBetween(cumulative_distance, before, keyframe_id);
            entry.segment_length_m =
                CumulativeDistanceBetween(cumulative_distance, before, after);
            entry.segment_alpha = entry.segment_length_m > 1e-9
                ? entry.distance_from_a_m / entry.segment_length_m
                : 0.0;
            entry.control_span_kf_gap = KeyFrameGap(before, after);
        }
        else
        {
            entry.control_vertex_a = has_before ? before : after;
            entry.mode = PoseGraphPropagationMode::NearestControlVertex;
            entry.segment_alpha = 0.0;
            entry.distance_from_a_m = 0.0;
            entry.segment_length_m = 0.0;
            entry.control_span_kf_gap = 0;
        }
        result.problem.propagation_plan.push_back(entry);
    }

    result.problem.summary = Summarize(result.problem);
    result.problem.anchor_preservation.satisfied =
        !result.problem.anchor_preservation.required ||
        result.problem.anchor_preservation.previous_fiducial_fixed_count > 0U;
    result.success = true;
    result.reason = "pose_graph_problem_created";
    return result;
}

}  // namespace orbslam3_multi
