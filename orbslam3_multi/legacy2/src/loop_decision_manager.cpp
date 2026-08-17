#include "orbslam3_multi/loop_decision_manager.hpp"

#include <Eigen/LU>

#include <algorithm>
#include <cmath>

namespace orbslam3_multi
{

LoopDecisionResult LoopDecisionManager::Process(
    const LoopVerificationResult& verification,
    uint64_t arrival_id,
    const RawMapDatabase& raw_db,
    const GlobalPoseStore& pose_store,
    const LandmarkScoreManager& score_manager,
    CovisibilityDatabase& covisibility_db,
    FusedLandmarkManager& fused_landmark_manager) const
{
    LoopDecisionResult result;
    result.decision = verification.decision;
    const bool fusion_candidate =
        verification.decision == LoopGeometryDecision::FusionCandidate;
    const bool optimization_candidate =
        verification.decision ==
        LoopGeometryDecision::LoopOptimizationCandidate;
    if (!fusion_candidate && !optimization_candidate)
    {
        result.reason = "decision_outside_1p";
        return result;
    }
    if (!verification.geometry_confirmed ||
        (verification.inlier_mappoint_pairs.empty() &&
         verification.shared_identity_matches == 0U))
    {
        result.reason = "fusion_candidate_without_confirmed_inliers";
        return result;
    }

    const auto world_T_query =
        pose_store.GetWorldPose(verification.query_kf_id);
    const auto world_T_candidate =
        pose_store.GetWorldPose(verification.candidate_seed_kf_id);
    if (!world_T_query || !world_T_candidate ||
        !world_T_query->allFinite() || !world_T_candidate->allFinite())
    {
        result.reason = "fusion_candidate_without_world_pose";
        return result;
    }

    CovisibilityEdge edge;
    edge.kf_a = verification.query_kf_id;
    edge.kf_b = verification.candidate_seed_kf_id;
    edge.weight = static_cast<double>(verification.ransac_inliers);
    edge.source = CovisibilityEdgeSource::ServerLoopGeometric;
    edge.relative_pose_current =
        world_T_query->inverse() * world_T_candidate.value();

    // RANSAC estimates the correction that maps the query cloud, already in
    // world coordinates, onto the candidate cloud. Applying that correction
    // to the query KF yields the measured query-to-candidate relation.
    const Eigen::Matrix4d measured_world_T_query =
        verification.relative_pose_measured * world_T_query.value();
    edge.relative_pose_measured =
        measured_world_T_query.inverse() * world_T_candidate.value();
    edge.information_weight =
        std::max(
            1.0,
            static_cast<double>(verification.ransac_inliers) *
                std::max(0.05, verification.loop_confidence) /
                (1.0 + std::max(0.0, verification.mean_residual)));
    edge.shared_mappoints_or_inliers = verification.ransac_inliers;
    edge.shared_mappoint_ratio = verification.inlier_ratio;
    edge.image_coverage_bins = verification.image_coverage_bins;
    edge.spatial_coverage_ratio = verification.spatial_coverage_ratio;
    edge.geometry_confirmed = true;
    edge.created_arrival_id = arrival_id;

    bool edge_added = false;
    result.covisibility_edge_changed =
        covisibility_db.AddConfirmedLoopEdge(edge, &edge_added);
    result.covisibility_edge_added = edge_added;
    if (optimization_candidate)
    {
        result.handled = true;
        result.reason = "optimization_pending_recorded";
        return result;
    }
    result.fusion = fused_landmark_manager.FuseInlierPairs(
        verification.inlier_mappoint_pairs,
        raw_db,
        score_manager,
        verification.loop_confidence);
    result.handled =
        result.covisibility_edge_changed || result.fusion.pairs_fused > 0U;
    result.reason = result.fusion.pairs_fused > 0U
        ? "fusion_candidate_applied"
        : (result.covisibility_edge_changed
            ? "covisibility_confirmed_by_shared_raw_identity"
            : "fusion_candidate_without_valid_pairs");
    return result;
}

}  // namespace orbslam3_multi
