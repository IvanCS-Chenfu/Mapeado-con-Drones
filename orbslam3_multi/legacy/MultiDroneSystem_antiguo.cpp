#include "orbslam3_multi/legacy/MultiDroneSystem_antiguo.hpp"
#include "orbslam3_multi/legacy/GlobalPoseGraphOptimizer_antiguo.hpp"

#include <iostream>
#include <limits>

#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cmath>


namespace orbslam3_multi
{


namespace
{

uint64_t MakeSubmapKeyForFusionDiag(
    uint32_t drone_id,
    uint64_t map_epoch)
{
    return
        (static_cast<uint64_t>(drone_id) << 32) |
        (map_epoch & 0xFFFFFFFFULL);
}

double NormalizeAngleRad(
    double angle)
{
    constexpr double kPi =
        3.14159265358979323846;

    while (angle > kPi)
        angle -= 2.0 * kPi;

    while (angle < -kPi)
        angle += 2.0 * kPi;

    return angle;
}


double AngleDistanceRad(
    double a,
    double b)
{
    return std::abs(
        NormalizeAngleRad(a - b));
}


double YawFromMatrix(
    const Eigen::Matrix4d& T)
{
    return std::atan2(
        T(1, 0),
        T(0, 0));
}

}  


MultiDroneSystem::MultiDroneSystem()
{
    atlas_ =
        std::make_shared<GlobalAtlas>();

    keyframe_database_ =
        std::make_shared<GlobalKeyFrameDatabase>(atlas_);
}

void MultiDroneSystem::SetFusionValidationParams(
    const FusionValidationParams& params)
{
    FusionValidationParams clean =
        params;

    if (clean.max_inter_drone_pair_distance_m <= 0.0)
        clean.max_inter_drone_pair_distance_m = 0.35;

    if (clean.max_intra_drone_pair_distance_m <= 0.0)
        clean.max_intra_drone_pair_distance_m = 0.20;

    if (clean.max_cross_epoch_same_drone_pair_distance_m <= 0.0)
        clean.max_cross_epoch_same_drone_pair_distance_m = 0.30;

    if (clean.warn_group_spread_m <= 0.0)
        clean.warn_group_spread_m = 0.50;

    if (clean.max_merged_group_spread_m <= 0.0)
        clean.max_merged_group_spread_m = 0.30;

    if (clean.max_merged_group_spread_m > clean.warn_group_spread_m)
    {
        // El warning debe ser al menos tan permisivo como el límite duro.
        clean.warn_group_spread_m =
            clean.max_merged_group_spread_m;
    }

    if (clean.max_pair_validation_logs == 0)
        clean.max_pair_validation_logs = 30;

    {
        std::lock_guard<std::mutex> lock(
            fusion_validation_params_mutex_);

        fusion_validation_params_ =
            clean;
    }

    if (clean.min_intra_drone_kf_id_gap_for_fusion == 0)
        clean.min_intra_drone_kf_id_gap_for_fusion = 20;

    std::cerr
        << "[FUSION5D-CONFIG] enable_pair_distance_filter="
        << (clean.enable_pair_distance_filter ? 1 : 0)
        << " enable_inter_drone_fusion="
        << (clean.enable_inter_drone_fusion ? 1 : 0)
        << " enable_intra_drone_fusion="
        << (clean.enable_intra_drone_fusion ? 1 : 0)
        << " min_intra_kf_gap="
        << clean.min_intra_drone_kf_id_gap_for_fusion
        << " reject_nearby_intra_kfs="
        << (clean.reject_intra_fusion_for_nearby_keyframes ? 1 : 0)
        << " enable_cross_epoch_same_drone_fusion="
        << (clean.enable_cross_epoch_same_drone_fusion ? 1 : 0)
        << " max_inter=" << clean.max_inter_drone_pair_distance_m
        << " max_intra=" << clean.max_intra_drone_pair_distance_m
        << " max_cross_epoch_same_drone="
        << clean.max_cross_epoch_same_drone_pair_distance_m
        << " group_spread_warning="
        << (clean.enable_group_spread_warning ? 1 : 0)
        << " warn_group_spread_m=" << clean.warn_group_spread_m
        << " split_merged_group_if_spread_too_high="
        << (clean.split_merged_group_if_spread_too_high ? 1 : 0)
        << " max_merged_group_spread_m="
        << clean.max_merged_group_spread_m
        << " publish_rejected_group_as_singletons="
        << (clean.publish_rejected_group_as_singletons ? 1 : 0)
        << " log_samples=" << (clean.log_pair_validation_samples ? 1 : 0)
        << " max_logs=" << clean.max_pair_validation_logs
        << std::endl;
}


FusionValidationParams
MultiDroneSystem::GetFusionValidationParams() const
{
    std::lock_guard<std::mutex> lock(
        fusion_validation_params_mutex_);

    return fusion_validation_params_;
}



void MultiDroneSystem::SetLoopAnchorParams(
    const LoopAnchorParams& params)
{
    LoopAnchorParams clean =
        params;

    if (clean.min_final_inliers < 1)
        clean.min_final_inliers = 30;

    if (clean.min_ransac_inliers < 0)
        clean.min_ransac_inliers = 20;

    if (clean.min_projection_matches < 0)
        clean.min_projection_matches = 20;

    if (clean.min_inlier_ratio < 0.0)
        clean.min_inlier_ratio = 0.25;

    if (clean.max_mean_error_m <= 0.0)
        clean.max_mean_error_m = 0.45;

    if (clean.max_error_m <= 0.0)
        clean.max_error_m = 2.00;

    if (clean.min_final_inliers_relaxed_support < clean.min_final_inliers)
        clean.min_final_inliers_relaxed_support = clean.min_final_inliers;

    if (clean.max_translation_norm_m <= 0.0)
        clean.max_translation_norm_m = 1000.0;

    auto sanitize_weight =
        [](double& w, double fallback)
        {
            if (!std::isfinite(w) || w <= 0.0)
            {
                w = fallback;
            }
        };

    sanitize_weight(clean.loop_weight_intra_fusion, 80.0);
    sanitize_weight(clean.loop_weight_intra_opt_only, 25.0);
    sanitize_weight(clean.loop_weight_inter_fusion_confirmed, 80.0);
    sanitize_weight(clean.loop_weight_inter_opt_only_confirmed, 20.0);
    sanitize_weight(clean.loop_weight_inter_fusion_provisional, 25.0);
    sanitize_weight(clean.loop_weight_inter_opt_only_provisional, 10.0);
    sanitize_weight(clean.loop_weight_max, 100.0);
    sanitize_weight(clean.loop_weight_min, 1.0);

    if (clean.strong_opt_only_min_final_inliers < clean.min_final_inliers)
        clean.strong_opt_only_min_final_inliers = std::max(90, clean.min_final_inliers);

    if (clean.strong_opt_only_min_ransac_inliers < clean.min_ransac_inliers)
        clean.strong_opt_only_min_ransac_inliers = std::max(35, clean.min_ransac_inliers);

    if (clean.strong_opt_only_min_projection_matches < clean.min_projection_matches)
        clean.strong_opt_only_min_projection_matches = std::max(60, clean.min_projection_matches);

    if (clean.strong_opt_only_min_inlier_ratio < clean.min_inlier_ratio)
        clean.strong_opt_only_min_inlier_ratio = std::max(0.55, clean.min_inlier_ratio);

    if (clean.strong_opt_only_max_mean_error_m <= 0.0)
        clean.strong_opt_only_max_mean_error_m = 0.22;

    if (clean.strong_opt_only_max_error_m <= 0.0)
        clean.strong_opt_only_max_error_m = 0.60;

    if (clean.consensus_min_cluster_size < 1)
        clean.consensus_min_cluster_size = 2;

    if (clean.consensus_max_translation_spread_m <= 0.0)
        clean.consensus_max_translation_spread_m = 0.30;

    if (clean.consensus_max_yaw_spread_deg <= 0.0)
        clean.consensus_max_yaw_spread_deg = 10.0;

    if (clean.loop_weight_min > clean.loop_weight_max)
    {
        clean.loop_weight_min = 1.0;
        clean.loop_weight_max = 100.0;
    }

    {
        std::lock_guard<std::mutex> lock(
            loop_anchor_params_mutex_);

        loop_anchor_params_ =
            clean;
    }

    std::cerr
        << "[PHASE7-LOOP-ANCHOR-CONFIG]"
        << " enabled=" << (clean.enable_inter_drone_loop_anchor ? 1 : 0)
        << " min_final=" << clean.min_final_inliers
        << " min_ransac=" << clean.min_ransac_inliers
        << " min_projection=" << clean.min_projection_matches
        << " min_ratio=" << clean.min_inlier_ratio
        << " max_mean=" << clean.max_mean_error_m
        << " max_error=" << clean.max_error_m
        << " relaxed_final=" << clean.min_final_inliers_relaxed_support
        << " max_t_norm=" << clean.max_translation_norm_m
        << " log_rejections=" << (clean.log_rejections ? 1 : 0)
        << " maturity_weights=" << (clean.use_maturity_aware_loop_weights ? 1 : 0)
        << " w_intra_fusion=" << clean.loop_weight_intra_fusion
        << " w_intra_opt=" << clean.loop_weight_intra_opt_only
        << " w_inter_fusion_confirmed=" << clean.loop_weight_inter_fusion_confirmed
        << " w_inter_opt_confirmed=" << clean.loop_weight_inter_opt_only_confirmed
        << " w_inter_fusion_provisional=" << clean.loop_weight_inter_fusion_provisional
        << " w_inter_opt_provisional=" << clean.loop_weight_inter_opt_only_provisional
        << " w_min=" << clean.loop_weight_min
        << " w_max=" << clean.loop_weight_max
        << " require_fusion_anchor=" << (clean.require_fusion_for_loop_anchor ? 1 : 0)
        << " allow_strong_opt_only_anchor=" << (clean.allow_strong_opt_only_anchor ? 1 : 0)
        << " strong_opt_final=" << clean.strong_opt_only_min_final_inliers
        << " strong_opt_ransac=" << clean.strong_opt_only_min_ransac_inliers
        << " strong_opt_proj=" << clean.strong_opt_only_min_projection_matches
        << " strong_opt_ratio=" << clean.strong_opt_only_min_inlier_ratio
        << " strong_opt_mean=" << clean.strong_opt_only_max_mean_error_m
        << " strong_opt_max=" << clean.strong_opt_only_max_error_m
        << " consensus_required="
        << (clean.require_consensus_for_loop_anchor ? 1 : 0)
        << " consensus_min_cluster="
        << clean.consensus_min_cluster_size
        << " consensus_max_trans="
        << clean.consensus_max_translation_spread_m
        << " consensus_max_yaw_deg="
        << clean.consensus_max_yaw_spread_deg
        << " consensus_log_details="
        << (clean.consensus_log_details ? 1 : 0)
        << " use_inter_loop_edges_for_pose_graph="
        << (clean.use_inter_drone_loop_edges_for_pose_graph ? 1 : 0)
        << " inter_loop_require_stable_anchors="
        << (clean.inter_loop_pose_graph_require_stable_anchors ? 1 : 0)
        << std::endl;
}


LoopAnchorParams MultiDroneSystem::GetLoopAnchorParams() const
{
    std::lock_guard<std::mutex> lock(
        loop_anchor_params_mutex_);

    return loop_anchor_params_;
}



// ============================================================
// Inserta o actualiza un KeyFrame.
// También actualiza la base de datos BoW.
// ============================================================

void MultiDroneSystem::InsertOrUpdateKeyFrame(
    const ImportedKeyFrame& kf)
{
    atlas_->InsertOrUpdateKeyFrame(kf);

    if (kf.is_bad)
    {
        keyframe_database_->EraseKeyFrame(kf.global_id);
        return;
    }

    keyframe_database_->AddOrUpdateKeyFrame(kf);
}


// ============================================================
// Inserta o actualiza un MapPoint.
// ============================================================

void MultiDroneSystem::InsertOrUpdateMapPoint(
    const ImportedMapPoint& mp)
{
    atlas_->InsertOrUpdateMapPoint(mp);
}


void MultiDroneSystem::ClearDroneEpoch(
    uint32_t drone_id,
    uint64_t map_epoch)
{
    if (drone_id == 0)
        return;

    size_t removed_from_atlas = 0;

    if (atlas_)
    {
        removed_from_atlas =
            atlas_->EraseDroneMap(
                drone_id,
                map_epoch);
    }

    if (keyframe_database_)
    {
        keyframe_database_->ClearDroneEpoch(
            drone_id,
            map_epoch);
    }

    size_t removed_loop_edges = 0;

    {
        std::lock_guard<std::mutex> lock(loop_edges_mutex_);

        for (auto it = loop_edges_.begin(); it != loop_edges_.end(); )
        {
            const VerifiedLoopEdge& edge =
                it->second;

            const bool query_is_epoch =
                edge.query_drone_id == drone_id &&
                edge.query_map_epoch == map_epoch;

            const bool candidate_is_epoch =
                edge.candidate_drone_id == drone_id &&
                edge.candidate_map_epoch == map_epoch;

            if (query_is_epoch || candidate_is_epoch)
            {
                it = loop_edges_.erase(it);
                removed_loop_edges++;
            }
            else
            {
                ++it;
            }
        }
    }

    {
        std::lock_guard<std::mutex> lock(anchors_mutex_);

        const uint64_t anchor_key =
            MakeCameraKey(
                drone_id,
                map_epoch);

        map_anchors_.erase(anchor_key);
    }

    ClearDirectLandmarkAssociationsForDroneEpoch(
        drone_id,
        map_epoch);

    std::cerr
        << "[MULTI-CLEAR-EPOCH] drone_" << drone_id
        << " epoch=" << map_epoch
        << " removed_from_atlas=" << removed_from_atlas
        << " removed_loop_edges=" << removed_loop_edges
        << std::endl;
}


// ============================================================
// Clave compacta para cámara por dron/map_epoch.
// 32 bits altos: drone_id.
// 32 bits bajos: map_epoch.
// Para TFG/simulación es suficiente.
// ============================================================

uint64_t MultiDroneSystem::MakeCameraKey(
    uint32_t drone_id,
    uint64_t map_epoch) const
{
    return
        (static_cast<uint64_t>(drone_id) << 32) |
        (map_epoch & 0xFFFFFFFFULL);
}


// ============================================================
// Guarda intrínsecas de cámara.
// ============================================================

void MultiDroneSystem::SetCameraInfo(
    uint32_t drone_id,
    uint64_t map_epoch,
    const CameraInfo& info)
{
    std::lock_guard<std::mutex> lock(camera_mutex_);

    const uint64_t key =
        MakeCameraKey(drone_id, map_epoch);

    camera_info_by_drone_epoch_[key] = info;
}


// ============================================================
// Recupera intrínsecas de cámara.
// Si no existen, devuelve CameraInfo inválida.
// ============================================================

CameraInfo MultiDroneSystem::GetCameraInfo(
    uint32_t drone_id,
    uint64_t map_epoch) const
{
    std::lock_guard<std::mutex> lock(camera_mutex_);

    const uint64_t key =
        MakeCameraKey(drone_id, map_epoch);

    auto it =
        camera_info_by_drone_epoch_.find(key);

    if (it == camera_info_by_drone_epoch_.end())
        return CameraInfo();

    return it->second;
}


// ============================================================
// Devuelve contadores del atlas.
// ============================================================

AtlasCounts MultiDroneSystem::GetCounts() const
{
    return atlas_->GetCounts();
}


// ============================================================
// Devuelve nube raw del atlas.
// No está optimizada. Solo sirve para depuración/publicación raw.
// ============================================================

std::vector<RawPoint> MultiDroneSystem::GetRawCloud() const
{
    std::vector<ImportedMapPoint> mps =
        atlas_->GetAllMapPoints();

    std::vector<RawPoint> cloud;
    cloud.reserve(mps.size());

    for (const auto& mp : mps)
    {
        if (mp.is_bad)
            continue;

        RawPoint p;

        p.x = mp.position.x();
        p.y = mp.position.y();
        p.z = mp.position.z();

        p.drone_id = mp.drone_id;
        p.map_epoch = mp.map_epoch;
        p.global_mappoint_id = mp.global_id;

        cloud.push_back(p);
    }

    return cloud;
}


std::vector<PoseGraphVertex> MultiDroneSystem::GetAnchoredKeyFramePoses() const
{
    std::vector<PoseGraphVertex> out;

    if (!atlas_)
        return out;

    const std::vector<ImportedKeyFrame> keyframes =
        atlas_->GetAllKeyFrames();

    out.reserve(keyframes.size());

    for (const auto& kf : keyframes)
    {
        if (kf.global_id == 0 || kf.is_bad)
            continue;

        const uint64_t anchor_key =
            MakeCameraKey(
                kf.drone_id,
                kf.map_epoch);

        MapAnchor anchor;

        {
            std::lock_guard<std::mutex> lock(anchors_mutex_);

            auto it =
                map_anchors_.find(anchor_key);

            if (it == map_anchors_.end() || !it->second.valid)
                continue;

            anchor = it->second;
        }

        PoseGraphVertex v;

        v.global_kf_id = kf.global_id;
        v.drone_id = kf.drone_id;
        v.map_epoch = kf.map_epoch;
        v.local_kf_id = kf.local_id;

        v.local_T_camera =
            KeyFramePoseToMatrix(kf);

        v.world_T_camera_initial =
            anchor.world_T_local * v.local_T_camera;

        {
            std::lock_guard<std::mutex> guess_lock(
                pose_graph_initial_guess_mutex_);

            auto guess_it =
                pose_graph_initial_guesses_.find(
                    v.global_kf_id);

            if (guess_it != pose_graph_initial_guesses_.end())
            {
                v.world_T_camera_initial =
                    guess_it->second;
            }
        }

        v.world_T_camera_optimized =
            v.world_T_camera_initial;

        v.fixed = false;

        out.push_back(v);
    }

    return out;
}

std::vector<RawPoint> MultiDroneSystem::GetAnchoredCloud() const
{
    std::vector<RawPoint> cloud;

    if (!atlas_)
        return cloud;

    const std::vector<ImportedMapPoint> mps =
        atlas_->GetAllMapPoints();

    cloud.reserve(mps.size());

    for (const auto& mp : mps)
    {
        if (mp.global_id == 0 || mp.is_bad)
            continue;

        const uint64_t anchor_key =
            MakeCameraKey(
                mp.drone_id,
                mp.map_epoch);

        MapAnchor anchor;

        {
            std::lock_guard<std::mutex> lock(anchors_mutex_);

            auto it =
                map_anchors_.find(anchor_key);

            if (it == map_anchors_.end() || !it->second.valid)
                continue;

            anchor = it->second;
        }

        Eigen::Vector4d p_local;
        p_local << mp.position.x(), mp.position.y(), mp.position.z(), 1.0;

        const Eigen::Vector4d p_world =
            anchor.world_T_local * p_local;

        if (!std::isfinite(p_world.x()) ||
            !std::isfinite(p_world.y()) ||
            !std::isfinite(p_world.z()))
        {
            continue;
        }

        RawPoint p;
        p.x = p_world.x();
        p.y = p_world.y();
        p.z = p_world.z();
        p.drone_id = mp.drone_id;
        p.map_epoch = mp.map_epoch;
        p.global_mappoint_id = mp.global_id;

        cloud.push_back(p);
    }

    return cloud;
}

std::shared_ptr<GlobalAtlas> MultiDroneSystem::GetAtlas()
{
    return atlas_;
}


std::shared_ptr<GlobalKeyFrameDatabase>
MultiDroneSystem::GetKeyFrameDatabase()
{
    return keyframe_database_;
}


// ============================================================
// Paso 3/4: candidatos BoW.
// ============================================================

std::vector<BowCandidate> MultiDroneSystem::DetectBowCandidates(
    uint64_t query_global_kf_id,
    const BowQueryParams& params) const
{
    return keyframe_database_->DetectCandidates(
        query_global_kf_id,
        params);
}


// ============================================================
// Paso 5: matching SearchByBoW.
// ============================================================

std::vector<FeatureMatch> MultiDroneSystem::SearchByBoW(
    uint64_t query_global_kf_id,
    uint64_t candidate_global_kf_id,
    const SearchByBowParams& params) const
{
    ImportedKeyFrame query =
        atlas_->GetKeyFrame(query_global_kf_id);

    ImportedKeyFrame candidate =
        atlas_->GetKeyFrame(candidate_global_kf_id);

    if (query.global_id == 0 || candidate.global_id == 0)
        return {};

    GlobalORBMatcher matcher(params);

    return matcher.SearchByBoW(
        query,
        candidate);
}


// ============================================================
// Paso 6: verificación geométrica completa.
// ============================================================

GeometryVerificationResult MultiDroneSystem::VerifyGeometry(
    uint64_t query_global_kf_id,
    uint64_t candidate_global_kf_id,
    const std::vector<FeatureMatch>& initial_matches,
    const GeometryVerificationParams& params) const
{
    GeometryVerificationResult empty_result;
    empty_result.query_kf_id = query_global_kf_id;
    empty_result.candidate_kf_id = candidate_global_kf_id;

    ImportedKeyFrame query =
        atlas_->GetKeyFrame(query_global_kf_id);

    ImportedKeyFrame candidate =
        atlas_->GetKeyFrame(candidate_global_kf_id);

    if (query.global_id == 0 || candidate.global_id == 0)
        return empty_result;

    CameraInfo query_camera =
        GetCameraInfo(query.drone_id, query.map_epoch);

    CameraInfo candidate_camera =
        GetCameraInfo(candidate.drone_id, candidate.map_epoch);

    GlobalGeometryVerifier verifier(atlas_);

    return verifier.Verify(
        query,
        candidate,
        initial_matches,
        query_camera,
        candidate_camera,
        params);
}


LoopEdgePairKey MultiDroneSystem::MakeLoopEdgePairKey(
    uint64_t kf_a,
    uint64_t kf_b) const
{
    LoopEdgePairKey key;

    key.a = std::min(kf_a, kf_b);
    key.b = std::max(kf_a, kf_b);

    return key;
}

uint32_t MultiDroneSystem::ExtractDroneId(
    uint64_t global_id) const
{
    return static_cast<uint32_t>((global_id >> 48) & 0xFFFF);
}

uint64_t MultiDroneSystem::ExtractLocalId(
    uint64_t global_id) const
{
    return global_id & 0xFFFFFFFFULL;
}

bool MultiDroneSystem::HasLoopEdge(
    uint64_t kf_a,
    uint64_t kf_b) const
{
    std::lock_guard<std::mutex> lock(loop_edges_mutex_);

    const LoopEdgePairKey key =
        MakeLoopEdgePairKey(kf_a, kf_b);

    return loop_edges_.find(key) != loop_edges_.end();
}



bool MultiDroneSystem::GetLoopEdge(
    uint64_t kf_a,
    uint64_t kf_b,
    VerifiedLoopEdge& edge_out) const
{
    std::lock_guard<std::mutex> lock(loop_edges_mutex_);

    const LoopEdgePairKey key =
        MakeLoopEdgePairKey(kf_a, kf_b);

    auto it =
        loop_edges_.find(key);

    if (it == loop_edges_.end())
    {
        return false;
    }

    edge_out =
        it->second;

    return true;
}


bool MultiDroneSystem::SetLoopEdgeAnchorUsability(
    uint64_t kf_a,
    uint64_t kf_b,
    bool usable_for_anchor,
    const std::string& reason)
{
    std::lock_guard<std::mutex> lock(loop_edges_mutex_);

    const LoopEdgePairKey key =
        MakeLoopEdgePairKey(kf_a, kf_b);

    auto it =
        loop_edges_.find(key);

    if (it == loop_edges_.end())
    {
        std::cerr
            << "[LOOP-ANCHOR-USABILITY-SET-FAIL]"
            << " reason=edge_not_found"
            << " kf_a=" << kf_a
            << " kf_b=" << kf_b
            << " requested_usable=" << (usable_for_anchor ? 1 : 0)
            << " requested_reason=" << reason
            << std::endl;

        return false;
    }

    it->second.usable_for_anchor =
        usable_for_anchor;

    it->second.anchor_reject_reason =
        reason;

    std::cerr
        << "[LOOP-ANCHOR-USABILITY-SET]"
        << " usable_for_anchor=" << (usable_for_anchor ? 1 : 0)
        << " reason=" << reason
        << " q=drone_" << it->second.query_drone_id
        << "/epoch_" << it->second.query_map_epoch
        << "/kf_" << it->second.query_local_kf_id
        << " c=drone_" << it->second.candidate_drone_id
        << "/epoch_" << it->second.candidate_map_epoch
        << "/kf_" << it->second.candidate_local_kf_id
        << " opt=" << (it->second.usable_for_optimization ? 1 : 0)
        << " fusion=" << (it->second.usable_for_fusion ? 1 : 0)
        << std::endl;

    return true;
}



bool MultiDroneSystem::AddVerifiedLoopEdge(
    const GeometryVerificationResult& geom,
    LoopEdgeType type)
{

    // ============================================================
    // FASE 2:
    // Separar loop útil para optimización de loop útil para fusión.
    //
    // Un loop puede ser suficientemente bueno para el pose graph
    // aunque todavía NO sea suficientemente fiable para fusionar
    // landmarks.
    //
    // Política:
    //   usable_for_optimization:
    //      permite guardar el loop y usarlo como edge del pose graph.
    //
    //   usable_for_fusion:
    //      permite usar sus matches para unir MapPoints globales.
    //
    // Por tanto:
    //   optimización = más permisiva
    //   fusión       = más estricta
    // ============================================================

    if (!geom.success)
    {
        std::cerr
            << "[LOOP-REJECT-OPT] reason=geometry_verification_failed"
            << " q_global=" << geom.query_kf_id
            << " c_global=" << geom.candidate_kf_id
            << " initial=" << geom.initial_match_count
            << " ransac=" << geom.ransac_inlier_count
            << " projection=" << geom.projection_match_count
            << " final=" << geom.final_inlier_count
            << " ratio=" << geom.inlier_ratio
            << " mean=" << geom.mean_error_m
            << " max=" << geom.max_error_m
            << std::endl;

        return false;
    }

    if (geom.query_kf_id == 0 || geom.candidate_kf_id == 0)
    {
        std::cerr
            << "[LOOP-REJECT-OPT] reason=invalid_kf_id"
            << " q_global=" << geom.query_kf_id
            << " c_global=" << geom.candidate_kf_id
            << std::endl;

        return false;
    }

    const bool transform_finite =
        geom.candidate_T_query.allFinite();

    const bool has_enough_3d_support =
        geom.final_inlier_count >= 30 ||
        geom.ransac_inlier_count >= 30 ||
        geom.projection_match_count >= 20;

    const bool usable_for_optimization =
        transform_finite &&
        geom.final_inlier_count >= 30 &&
        geom.inlier_ratio >= 0.25 &&
        geom.mean_error_m <= 0.45 &&
        geom.max_error_m <= 2.00 &&
        has_enough_3d_support;

    const bool has_enough_landmark_pairs =
        geom.final_inlier_matches.size() >= 50;

    const bool usable_for_fusion =
        usable_for_optimization &&
        has_enough_landmark_pairs &&
        geom.final_inlier_count >= 60 &&
        geom.inlier_ratio >= 0.45 &&
        geom.mean_error_m <= 0.25 &&
        geom.max_error_m <= 1.00 &&
        geom.projection_match_count >= 20;

    std::string classify_reason;

    if (usable_for_fusion)
    {
        classify_reason = "good_for_optimization_and_fusion";
    }
    else if (usable_for_optimization)
    {
        classify_reason = "good_for_pose_graph_not_for_landmarks";
    }
    else if (!transform_finite)
    {
        classify_reason = "transform_not_finite";
    }
    else if (geom.final_inlier_count < 30)
    {
        classify_reason = "too_few_final_inliers_for_optimization";
    }
    else if (geom.inlier_ratio < 0.25)
    {
        classify_reason = "inlier_ratio_too_low_for_optimization";
    }
    else if (geom.mean_error_m > 0.45)
    {
        classify_reason = "mean_error_too_high_for_optimization";
    }
    else if (geom.max_error_m > 2.00)
    {
        classify_reason = "max_error_too_high_for_optimization";
    }
    else
    {
        classify_reason = "unknown_not_usable_for_optimization";
    }

    std::cerr
        << "[LOOP-CLASSIFY] "
        << "q_global=" << geom.query_kf_id
        << " c_global=" << geom.candidate_kf_id
        << " type=" << (type == LoopEdgeType::INTER_DRONE ? "INTER_DRONE" : "INTRA_DRONE")
        << " initial=" << geom.initial_match_count
        << " ransac=" << geom.ransac_inlier_count
        << " projection=" << geom.projection_match_count
        << " final=" << geom.final_inlier_count
        << " final_matches=" << geom.final_inlier_matches.size()
        << " ratio=" << geom.inlier_ratio
        << " mean=" << geom.mean_error_m
        << " max=" << geom.max_error_m
        << " opt=" << (usable_for_optimization ? 1 : 0)
        << " fusion=" << (usable_for_fusion ? 1 : 0)
        << " reason=" << classify_reason
        << std::endl;

    if (!usable_for_optimization)
    {
        std::cerr
            << "[LOOP-REJECT-OPT] reason=" << classify_reason
            << " q_global=" << geom.query_kf_id
            << " c_global=" << geom.candidate_kf_id
            << " final=" << geom.final_inlier_count
            << " final_matches=" << geom.final_inlier_matches.size()
            << " ratio=" << geom.inlier_ratio
            << " mean=" << geom.mean_error_m
            << " max=" << geom.max_error_m
            << std::endl;

        return false;
    }

    if (!usable_for_fusion)
    {
        std::cerr
            << "[LOOP-REJECT-FUSION] reason=" << classify_reason
            << " but_store_for_optimization=1"
            << " q_global=" << geom.query_kf_id
            << " c_global=" << geom.candidate_kf_id
            << " final=" << geom.final_inlier_count
            << " final_matches=" << geom.final_inlier_matches.size()
            << " ratio=" << geom.inlier_ratio
            << " mean=" << geom.mean_error_m
            << " max=" << geom.max_error_m
            << std::endl;
    }

    const LoopEdgePairKey key =
        MakeLoopEdgePairKey(
            geom.query_kf_id,
            geom.candidate_kf_id);

    std::lock_guard<std::mutex> lock(loop_edges_mutex_);

    if (loop_edges_.find(key) != loop_edges_.end())
    {
        std::cerr
            << "[LOOP7-REJECT] duplicate edge q="
            << ExtractDroneId(geom.query_kf_id) << "/kf_" << ExtractLocalId(geom.query_kf_id)
            << " c="
            << ExtractDroneId(geom.candidate_kf_id) << "/kf_" << ExtractLocalId(geom.candidate_kf_id)
            << std::endl;

        return false;
    }

    // ============================================================
    // Limitar número de loop edges por KeyFrame.
    //
    // Esto evita que un único keyframe, normalmente cerca de un
    // fiducial o zona muy repetida, domine el futuro pose graph.
    // ============================================================

    int count_query = 0;
    int count_candidate = 0;

    for (const auto& [existing_key, existing_edge] : loop_edges_)
    {
        (void)existing_key;

        if (existing_edge.query_kf_id == geom.query_kf_id ||
            existing_edge.candidate_kf_id == geom.query_kf_id)
        {
            count_query++;
        }

        if (existing_edge.query_kf_id == geom.candidate_kf_id ||
            existing_edge.candidate_kf_id == geom.candidate_kf_id)
        {
            count_candidate++;
        }
    }

    const int max_edges_per_keyframe = 10;

    if (count_query >= max_edges_per_keyframe ||
        count_candidate >= max_edges_per_keyframe)
    {
        std::cerr
            << "[LOOP7-REJECT] max_edges_per_keyframe q_count="
            << count_query
            << " c_count=" << count_candidate
            << " max=" << max_edges_per_keyframe
            << " q=drone_" << ExtractDroneId(geom.query_kf_id)
            << "/kf_" << ExtractLocalId(geom.query_kf_id)
            << " c=drone_" << ExtractDroneId(geom.candidate_kf_id)
            << "/kf_" << ExtractLocalId(geom.candidate_kf_id)
            << std::endl;

        return false;
    }

    // ============================================================
    // Guardar edge verificada.
    // ============================================================
    
    VerifiedLoopEdge edge;

    edge.query_kf_id = geom.query_kf_id;
    edge.candidate_kf_id = geom.candidate_kf_id;

    edge.query_drone_id =
        ExtractDroneId(geom.query_kf_id);

    edge.candidate_drone_id =
        ExtractDroneId(geom.candidate_kf_id);

    edge.query_local_kf_id =
        ExtractLocalId(geom.query_kf_id);

    edge.candidate_local_kf_id =
        ExtractLocalId(geom.candidate_kf_id);

    const ImportedKeyFrame query_kf =
        atlas_->GetKeyFrame(edge.query_kf_id);

    const ImportedKeyFrame candidate_kf =
        atlas_->GetKeyFrame(edge.candidate_kf_id);

    if (query_kf.global_id == 0 ||
        candidate_kf.global_id == 0 ||
        query_kf.is_bad ||
        candidate_kf.is_bad)
    {
        std::cerr
            << "[LOOP7-REJECT] missing/bad KF while storing loop q="
            << edge.query_kf_id
            << " c=" << edge.candidate_kf_id
            << std::endl;

        return false;
    }

    edge.query_map_epoch =
        query_kf.map_epoch;

    edge.candidate_map_epoch =
        candidate_kf.map_epoch;

    edge.type = type;

    edge.candidate_T_query =
        geom.candidate_T_query;

    edge.initial_matches =
        geom.initial_match_count;

    edge.ransac_inliers =
        geom.ransac_inlier_count;

    edge.projection_matches =
        geom.projection_match_count;

    edge.final_inliers =
        geom.final_inlier_count;

    edge.inlier_ratio =
        geom.inlier_ratio;

    edge.mean_error_m =
        geom.mean_error_m;

    edge.max_error_m =
        geom.max_error_m;

    // FASE 2:
    // Estos matches solo se usarán si usable_for_fusion=true.
    // Sin esta copia, BuildFusedLandmarksFromLoopEdges() no tiene
    // pares de MapPoints que unir.
    edge.inlier_matches =
        geom.final_inlier_matches;

    // ============================================================
    // Calidad de la loop edge.
    //
    // usable_for_optimization:
    //   Puede usarse como constraint del pose graph.
    //   Es deliberadamente más permisivo.
    //
    // usable_for_fusion:
    //   Puede usarse para unir landmarks.
    //   Es deliberadamente más estricto.
    //
    // IMPORTANTE:
    //   usable_for_optimization=true y usable_for_fusion=false
    //   es un caso esperado y útil.
    // ============================================================


    edge.usable_for_fusion =
        usable_for_fusion;

    edge.usable_for_optimization =
        usable_for_optimization;

    // ============================================================
    // Fase E:
    // Inicialmente un loop usable para optimización puede ser candidato
    // a anchor. El servidor podrá desactivarlo después si falla guards
    // físicos/GT/current-pose.
    // ============================================================

    edge.usable_for_anchor =
        usable_for_optimization;

    edge.anchor_reject_reason =
        usable_for_optimization
            ? "initially_usable_for_anchor"
            : classify_reason;

    edge.quality_score =
        static_cast<double>(edge.final_inliers) *
        edge.inlier_ratio /
        std::max(0.05, edge.mean_error_m);

    std::cerr
        << "[LOOP-STORE] "
        << "q=drone_" << edge.query_drone_id
        << "/epoch_" << edge.query_map_epoch
        << "/kf_" << edge.query_local_kf_id
        << " c=drone_" << edge.candidate_drone_id
        << "/epoch_" << edge.candidate_map_epoch
        << "/kf_" << edge.candidate_local_kf_id
        << " type=" << (edge.type == LoopEdgeType::INTER_DRONE ? "INTER_DRONE" : "INTRA_DRONE")
        << " initial=" << edge.initial_matches
        << " ransac=" << edge.ransac_inliers
        << " projection=" << edge.projection_matches
        << " final=" << edge.final_inliers
        << " final_matches=" << edge.inlier_matches.size()
        << " ratio=" << edge.inlier_ratio
        << " mean=" << edge.mean_error_m
        << " max=" << edge.max_error_m
        << " opt=" << (edge.usable_for_optimization ? 1 : 0)
        << " fusion=" << (edge.usable_for_fusion ? 1 : 0)
        << " quality=" << edge.quality_score
        << " anchor=" << (edge.usable_for_anchor ? 1 : 0)
        << " anchor_reason=" << edge.anchor_reject_reason
        << std::endl;

    loop_edges_[key] = edge;
    
    return true;
}


bool MultiDroneSystem::AddDirectLandmarkAssociation(
    const DirectLandmarkAssociation& association_in)
{
    DirectLandmarkAssociation association =
        association_in;

    if (!association.valid)
        return false;

    if (association.query_kf_id == 0 ||
        association.candidate_kf_id == 0)
        return false;

    if (association.matches.empty())
        return false;

    // ============================================================
    // Normalización para compatibilidad.
    // ============================================================

    if (association.query_drone_id == 0)
    {
        association.query_drone_id =
            association.drone_id;
    }

    if (association.candidate_drone_id == 0)
    {
        association.candidate_drone_id =
            association.drone_id;
    }

    if (association.query_map_epoch == 0)
    {
        association.query_map_epoch =
            association.map_epoch;
    }

    if (association.candidate_map_epoch == 0)
    {
        association.candidate_map_epoch =
            association.map_epoch;
    }

    if (association.query_local_keyframe_id == 0)
    {
        association.query_local_keyframe_id =
            association.query_local_kf_id;
    }

    if (association.candidate_local_keyframe_id == 0)
    {
        association.candidate_local_keyframe_id =
            association.candidate_local_kf_id;
    }

    const bool same_drone =
        association.query_drone_id ==
        association.candidate_drone_id;

    const bool same_epoch =
        association.query_map_epoch ==
        association.candidate_map_epoch;

    association.is_inter_drone =
        !same_drone;

    association.is_cross_epoch_same_drone =
        same_drone && !same_epoch;

    association.is_same_drone_same_epoch =
        same_drone && same_epoch;

    const LoopEdgePairKey key =
        MakeLoopEdgePairKey(
            association.query_kf_id,
            association.candidate_kf_id);

    std::lock_guard<std::mutex> lock(
        direct_landmark_associations_mutex_);

    auto it =
        direct_landmark_associations_.find(key);

    if (it != direct_landmark_associations_.end())
    {
        DirectLandmarkAssociation& existing =
            it->second;

        size_t added_matches = 0;
        size_t duplicate_matches = 0;

        auto same_mp_pair =
            [](const FeatureMatch& a,
               const FeatureMatch& b) -> bool
            {
                return
                    a.query_mappoint_id == b.query_mappoint_id &&
                    a.candidate_mappoint_id == b.candidate_mappoint_id;
            };

        for (const auto& new_match : association.matches)
        {
            bool duplicate = false;

            for (const auto& old_match : existing.matches)
            {
                if (same_mp_pair(new_match, old_match))
                {
                    duplicate = true;
                    break;
                }
            }

            if (duplicate)
            {
                duplicate_matches++;
                continue;
            }

            existing.matches.push_back(
                new_match);

            added_matches++;
        }

        if (added_matches == 0)
        {
            std::cerr
                << "[DIRECT-ASSOC-MERGE]"
                << " accepted=0"
                << " reason=all_matches_duplicate"
                << " type="
                << (association.is_inter_drone
                        ? "INTER_DRONE"
                        : (association.is_cross_epoch_same_drone
                            ? "CROSS_EPOCH_SAME_DRONE"
                            : "SAME_DRONE_SAME_EPOCH"))
                << " q=drone_" << association.query_drone_id
                << "/epoch_" << association.query_map_epoch
                << "/kf_" << association.query_local_keyframe_id
                << " c=drone_" << association.candidate_drone_id
                << "/epoch_" << association.candidate_map_epoch
                << "/kf_" << association.candidate_local_keyframe_id
                << " existing_matches=" << existing.matches.size()
                << " duplicate_matches=" << duplicate_matches
                << std::endl;

            return false;
        }

        existing.candidate_matches +=
            association.candidate_matches;

        existing.accepted_matches =
            existing.matches.size();

        existing.rejected_zero_mp +=
            association.rejected_zero_mp;

        existing.rejected_same_mp +=
            association.rejected_same_mp;

        existing.rejected_missing_mp +=
            association.rejected_missing_mp;

        existing.rejected_bad_mp +=
            association.rejected_bad_mp;

        existing.rejected_distance +=
            association.rejected_distance;

        existing.rejected_descriptor +=
            association.rejected_descriptor;

        const double old_weight =
            std::max<size_t>(
                1,
                existing.accepted_matches - added_matches);

        const double new_weight =
            std::max<size_t>(
                1,
                added_matches);

        existing.mean_distance_m =
            (existing.mean_distance_m * old_weight +
             association.mean_distance_m * new_weight) /
            (old_weight + new_weight);

        existing.max_distance_m =
            std::max(
                existing.max_distance_m,
                association.max_distance_m);

        if (existing.reason.find(association.reason) ==
            std::string::npos)
        {
            existing.reason +=
                "+" + association.reason;
        }

        // Si cualquiera de las asociaciones es inter-dron, el job completo
        // debe tratarse como inter-dron.
        existing.is_inter_drone =
            existing.is_inter_drone ||
            association.is_inter_drone;

        existing.is_cross_epoch_same_drone =
            existing.is_cross_epoch_same_drone ||
            association.is_cross_epoch_same_drone;

        existing.is_same_drone_same_epoch =
            existing.is_same_drone_same_epoch &&
            association.is_same_drone_same_epoch;

        std::cerr
            << "[DIRECT-ASSOC-MERGE]"
            << " accepted=1"
            << " type="
            << (existing.is_inter_drone
                    ? "INTER_DRONE"
                    : (existing.is_cross_epoch_same_drone
                        ? "CROSS_EPOCH_SAME_DRONE"
                        : "SAME_DRONE_SAME_EPOCH"))
            << " q=drone_" << existing.query_drone_id
            << "/epoch_" << existing.query_map_epoch
            << "/kf_" << existing.query_local_keyframe_id
            << " c=drone_" << existing.candidate_drone_id
            << "/epoch_" << existing.candidate_map_epoch
            << "/kf_" << existing.candidate_local_keyframe_id
            << " added_matches=" << added_matches
            << " duplicate_matches=" << duplicate_matches
            << " total_matches=" << existing.matches.size()
            << " candidate_matches=" << existing.candidate_matches
            << " mean_dist=" << existing.mean_distance_m
            << " max_dist=" << existing.max_distance_m
            << std::endl;

        return true;
    }

    association.accepted_matches =
        association.matches.size();

    direct_landmark_associations_[key] =
        association;

    std::cerr
        << "[DIRECT-ASSOC-STORE]"
        << " accepted=1"
        << " type="
        << (association.is_inter_drone
                ? "INTER_DRONE"
                : (association.is_cross_epoch_same_drone
                    ? "CROSS_EPOCH_SAME_DRONE"
                    : "SAME_DRONE_SAME_EPOCH"))
        << " q=drone_" << association.query_drone_id
        << "/epoch_" << association.query_map_epoch
        << "/kf_" << association.query_local_keyframe_id
        << " c=drone_" << association.candidate_drone_id
        << "/epoch_" << association.candidate_map_epoch
        << "/kf_" << association.candidate_local_keyframe_id
        << " reason=" << association.reason
        << " candidate_matches=" << association.candidate_matches
        << " accepted_matches=" << association.accepted_matches
        << " rejected_zero=" << association.rejected_zero_mp
        << " rejected_same=" << association.rejected_same_mp
        << " rejected_missing=" << association.rejected_missing_mp
        << " rejected_bad=" << association.rejected_bad_mp
        << " rejected_distance=" << association.rejected_distance
        << " rejected_descriptor=" << association.rejected_descriptor
        << " mean_dist=" << association.mean_distance_m
        << " max_dist=" << association.max_distance_m
        << std::endl;

    return true;
}



std::vector<DirectLandmarkAssociation>
MultiDroneSystem::GetDirectLandmarkAssociations() const
{
    std::lock_guard<std::mutex> lock(
        direct_landmark_associations_mutex_);

    std::vector<DirectLandmarkAssociation> out;
    out.reserve(direct_landmark_associations_.size());

    for (const auto& [key, assoc] : direct_landmark_associations_)
    {
        (void)key;
        out.push_back(assoc);
    }

    return out;
}


size_t MultiDroneSystem::GetDirectLandmarkAssociationCount() const
{
    std::lock_guard<std::mutex> lock(
        direct_landmark_associations_mutex_);

    return direct_landmark_associations_.size();
}


void MultiDroneSystem::ClearDirectLandmarkAssociationsForDroneEpoch(
    uint32_t drone_id,
    uint64_t map_epoch)
{
    std::lock_guard<std::mutex> lock(
        direct_landmark_associations_mutex_);

    size_t erased = 0;

    for (auto it = direct_landmark_associations_.begin();
         it != direct_landmark_associations_.end(); )
    {
        const auto& assoc =
            it->second;

        if (assoc.drone_id == drone_id &&
            assoc.map_epoch == map_epoch)
        {
            it = direct_landmark_associations_.erase(it);
            erased++;
        }
        else
        {
            ++it;
        }
    }

    if (erased > 0)
    {
        std::cerr
            << "[DIRECT5D-ASSOC-CLEAR]"
            << " drone_" << drone_id
            << " epoch=" << map_epoch
            << " erased=" << erased
            << std::endl;
    }
}


bool MultiDroneSystem::TryAnchorMapFromLoop(
    const GeometryVerificationResult& geom,
    LoopEdgeType type)
{
    // ============================================================
    // FASE 3:
    // Anchor directo desde un loop recién verificado.
    //
    // Esta función se llama justo después de AddVerifiedLoopEdge().
    // Aun así, repetimos checks importantes para que el anchoring sea
    // seguro y para dejar logs claros.
    // ============================================================


    LoopAnchorParams anchor_params =
        GetLoopAnchorParams();

    if (!anchor_params.enable_inter_drone_loop_anchor)
    {
        std::cerr
            << "[LOOP-ANCHOR-REJECT] method=direct_verified_loop"
            << " reason=inter_drone_loop_anchor_disabled"
            << " q_global=" << geom.query_kf_id
            << " c_global=" << geom.candidate_kf_id
            << std::endl;

        return false;
    }


    if (type != LoopEdgeType::INTER_DRONE)
    {
        std::cerr
            << "[LOOP-ANCHOR-REJECT] method=direct_verified_loop"
            << " reason=not_inter_drone"
            << " q_global=" << geom.query_kf_id
            << " c_global=" << geom.candidate_kf_id
            << std::endl;

        return false;
    }

    if (!geom.success)
    {
        std::cerr
            << "[LOOP-ANCHOR-REJECT] method=direct_verified_loop"
            << " reason=geometry_verification_failed"
            << " q_global=" << geom.query_kf_id
            << " c_global=" << geom.candidate_kf_id
            << std::endl;

        return false;
    }

    if (geom.query_kf_id == 0 || geom.candidate_kf_id == 0)
    {
        std::cerr
            << "[LOOP-ANCHOR-REJECT] method=direct_verified_loop"
            << " reason=invalid_kf_id"
            << " q_global=" << geom.query_kf_id
            << " c_global=" << geom.candidate_kf_id
            << std::endl;

        return false;
    }

    const bool transform_finite =
        geom.candidate_T_query.allFinite();

    const bool has_enough_support =
        geom.ransac_inlier_count >= anchor_params.min_ransac_inliers ||
        geom.projection_match_count >= anchor_params.min_projection_matches ||
        geom.final_inlier_count >= anchor_params.min_final_inliers_relaxed_support;

    const bool anchor_quality_ok =
        transform_finite &&
        geom.final_inlier_count >= anchor_params.min_final_inliers &&
        geom.inlier_ratio >= anchor_params.min_inlier_ratio &&
        geom.mean_error_m <= anchor_params.max_mean_error_m &&
        geom.max_error_m <= anchor_params.max_error_m &&
        has_enough_support;

    const bool strong_opt_only_anchor_ok =
        anchor_params.allow_strong_opt_only_anchor &&
        geom.final_inlier_count >= anchor_params.strong_opt_only_min_final_inliers &&
        geom.ransac_inlier_count >= anchor_params.strong_opt_only_min_ransac_inliers &&
        geom.projection_match_count >= anchor_params.strong_opt_only_min_projection_matches &&
        geom.inlier_ratio >= anchor_params.strong_opt_only_min_inlier_ratio &&
        geom.mean_error_m <= anchor_params.strong_opt_only_max_mean_error_m &&
        geom.max_error_m <= anchor_params.strong_opt_only_max_error_m;

    const bool fusion_support_ok =
        !anchor_params.require_fusion_for_loop_anchor ||
        strong_opt_only_anchor_ok;

    if (!anchor_quality_ok)
    {
        std::cerr
            << "[LOOP-ANCHOR-REJECT] method=direct_verified_loop"
            << " reason=quality_not_enough_for_anchor"
            << " q_global=" << geom.query_kf_id
            << " c_global=" << geom.candidate_kf_id
            << " final=" << geom.final_inlier_count
            << " ransac=" << geom.ransac_inlier_count
            << " projection=" << geom.projection_match_count
            << " ratio=" << geom.inlier_ratio
            << " mean=" << geom.mean_error_m
            << " max=" << geom.max_error_m
            << " transform_finite=" << (transform_finite ? 1 : 0)
            << " min_final=" << anchor_params.min_final_inliers
            << " min_ratio=" << anchor_params.min_inlier_ratio
            << " max_mean_allowed=" << anchor_params.max_mean_error_m
            << " max_error_allowed=" << anchor_params.max_error_m
            << std::endl;

        return false;
    }

    if (!fusion_support_ok)
    {
        if (anchor_params.log_rejections)
        {
            std::cerr
                << "[LOOP-ANCHOR-REJECT] method=direct_verified_loop"
                << " reason=anchor_requires_fusion_or_strong_opt_only"
                << " q_global=" << geom.query_kf_id
                << " c_global=" << geom.candidate_kf_id
                << " final=" << geom.final_inlier_count
                << " ransac=" << geom.ransac_inlier_count
                << " projection=" << geom.projection_match_count
                << " ratio=" << geom.inlier_ratio
                << " mean=" << geom.mean_error_m
                << " max=" << geom.max_error_m
                << " require_fusion=" << (anchor_params.require_fusion_for_loop_anchor ? 1 : 0)
                << " strong_opt_only_ok=" << (strong_opt_only_anchor_ok ? 1 : 0)
                << std::endl;
        }

        return false;
    }

    ImportedKeyFrame query =
        atlas_->GetKeyFrame(geom.query_kf_id);

    ImportedKeyFrame candidate =
        atlas_->GetKeyFrame(geom.candidate_kf_id);

    if (query.global_id == 0 ||
        candidate.global_id == 0 ||
        query.is_bad ||
        candidate.is_bad)
    {
        std::cerr
            << "[LOOP-ANCHOR-REJECT] method=direct_verified_loop"
            << " reason=missing_or_bad_keyframe"
            << " q_global=" << geom.query_kf_id
            << " c_global=" << geom.candidate_kf_id
            << " q_bad=" << (query.is_bad ? 1 : 0)
            << " c_bad=" << (candidate.is_bad ? 1 : 0)
            << std::endl;

        return false;
    }

    const uint64_t q_anchor_key =
        MakeCameraKey(
            query.drone_id,
            query.map_epoch);

    const uint64_t c_anchor_key =
        MakeCameraKey(
            candidate.drone_id,
            candidate.map_epoch);

    bool q_has_anchor = false;
    bool c_has_anchor = false;

    MapAnchor q_anchor;
    MapAnchor c_anchor;

    {
        std::lock_guard<std::mutex> lock(anchors_mutex_);

        auto q_it =
            map_anchors_.find(q_anchor_key);

        auto c_it =
            map_anchors_.find(c_anchor_key);

        if (q_it != map_anchors_.end() && q_it->second.valid)
        {
            q_has_anchor = true;
            q_anchor = q_it->second;
        }

        if (c_it != map_anchors_.end() && c_it->second.valid)
        {
            c_has_anchor = true;
            c_anchor = c_it->second;
        }
    }

    if (q_has_anchor == c_has_anchor)
    {
        std::cerr
            << "[LOOP-ANCHOR-REJECT] method=direct_verified_loop"
            << " reason="
            << (q_has_anchor ? "both_already_anchored" : "both_unanchored")
            << " q=drone_" << query.drone_id
            << "/epoch_" << query.map_epoch
            << "/kf_" << query.local_id
            << " c=drone_" << candidate.drone_id
            << "/epoch_" << candidate.map_epoch
            << "/kf_" << candidate.local_id
            << std::endl;

        return false;
    }

    const Eigen::Matrix4d candidate_T_query =
        geom.candidate_T_query;

    Eigen::Matrix4d new_world_T_local =
        Eigen::Matrix4d::Identity();

    uint32_t target_drone_id = 0;
    uint64_t target_map_epoch = 0;

    std::string direction;

    if (c_has_anchor && !q_has_anchor)
    {
        // candidate_T_query transforma query_local -> candidate_local.
        // Si candidate está anclado:
        // W_T_query = W_T_candidate * candidate_T_query
        new_world_T_local =
            c_anchor.world_T_local *
            candidate_T_query;

        target_drone_id =
            query.drone_id;

        target_map_epoch =
            query.map_epoch;

        direction = "candidate_anchor_to_query";
    }
    else if (q_has_anchor && !c_has_anchor)
    {
        // Si query está anclado:
        // W_T_candidate = W_T_query * inverse(candidate_T_query)
        new_world_T_local =
            q_anchor.world_T_local *
            candidate_T_query.inverse();

        target_drone_id =
            candidate.drone_id;

        target_map_epoch =
            candidate.map_epoch;

        direction = "query_anchor_to_candidate";
    }

    if (!new_world_T_local.allFinite())
    {
        std::cerr
            << "[LOOP-ANCHOR-REJECT] method=direct_verified_loop"
            << " reason=new_world_T_local_not_finite"
            << " target=drone_" << target_drone_id
            << "/epoch_" << target_map_epoch
            << std::endl;

        return false;
    }

    const Eigen::Vector3d t =
        new_world_T_local.block<3, 1>(0, 3);

    const double t_norm =
        t.norm();

    if (!std::isfinite(t_norm) ||
        t_norm > anchor_params.max_translation_norm_m)
    {
        std::cerr
            << "[LOOP-ANCHOR-REJECT] method=direct_verified_loop"
            << " reason=translation_norm_unreasonable"
            << " target=drone_" << target_drone_id
            << "/epoch_" << target_map_epoch
            << " t_norm=" << t_norm
            << " max_t_norm=" << anchor_params.max_translation_norm_m
            << std::endl;

        return false;
    }

    {
        MapAnchor existing_target_anchor;
        const bool target_already_has_anchor =
            GetMapAnchorInfo(
                target_drone_id,
                target_map_epoch,
                existing_target_anchor);

        if (target_already_has_anchor &&
            existing_target_anchor.maturity == AnchorMaturity::CONFIRMED)
        {
            std::cerr
                << "[LOOP-ANCHOR-SKIP-CONFIRMED-TARGET]"
                << " method=direct_verified_loop"
                << " target=drone_" << target_drone_id
                << "/epoch_" << target_map_epoch
                << " existing_source="
                << (existing_target_anchor.source == AnchorSource::FIDUCIAL_DIRECT
                        ? "FIDUCIAL_DIRECT"
                        : existing_target_anchor.source == AnchorSource::LOOP_OR_PROPAGATED
                            ? "LOOP_OR_PROPAGATED"
                            : "UNKNOWN")
                << " existing_maturity=CONFIRMED"
                << " action=do_not_create_new_provisional_anchor"
                << std::endl;

            return false;
        }
    }

    SetMapAnchor(
        target_drone_id,
        target_map_epoch,
        new_world_T_local,
        AnchorSource::LOOP_OR_PROPAGATED,
        AnchorMaturity::PROVISIONAL);

    std::cerr
        << "[ANCHOR-MATURITY]"
        << " drone_" << target_drone_id
        << " epoch=" << target_map_epoch
        << " state=PROVISIONAL_LOOP_ANCHOR"
        << " source=direct_verified_loop"
        << " q=drone_" << query.drone_id
        << "/epoch_" << query.map_epoch
        << "/kf_" << query.local_id
        << " c=drone_" << candidate.drone_id
        << "/epoch_" << candidate.map_epoch
        << "/kf_" << candidate.local_id
        << " final=" << geom.final_inlier_count
        << " ransac=" << geom.ransac_inlier_count
        << " projection=" << geom.projection_match_count
        << " ratio=" << geom.inlier_ratio
        << " mean=" << geom.mean_error_m
        << " max=" << geom.max_error_m
        << std::endl;

    return true;
}


bool MultiDroneSystem::TryAnchorMapFromStoredLoopEdge(
    const VerifiedLoopEdge& edge)
{

    // ============================================================
    // Fase E:
    // El servidor puede bloquear un loop para anchoring aunque siga
    // siendo válido como loop de optimización/fusión.
    // ============================================================

    if (!edge.usable_for_anchor)
    {
        std::cerr
            << "[LOOP-ANCHOR-REJECT] method=stored_loop"
            << " reason=loop_marked_not_usable_for_anchor"
            << " anchor_reject_reason=" << edge.anchor_reject_reason
            << " q=drone_" << edge.query_drone_id
            << "/epoch_" << edge.query_map_epoch
            << "/kf_" << edge.query_local_kf_id
            << " c=drone_" << edge.candidate_drone_id
            << "/epoch_" << edge.candidate_map_epoch
            << "/kf_" << edge.candidate_local_kf_id
            << " opt=" << (edge.usable_for_optimization ? 1 : 0)
            << " fusion=" << (edge.usable_for_fusion ? 1 : 0)
            << std::endl;

        return false;
    }

    LoopAnchorParams anchor_params =
        GetLoopAnchorParams();

    const bool strong_opt_only_anchor_ok =
        anchor_params.allow_strong_opt_only_anchor &&
        edge.final_inliers >= anchor_params.strong_opt_only_min_final_inliers &&
        edge.ransac_inliers >= anchor_params.strong_opt_only_min_ransac_inliers &&
        edge.projection_matches >= anchor_params.strong_opt_only_min_projection_matches &&
        edge.inlier_ratio >= anchor_params.strong_opt_only_min_inlier_ratio &&
        edge.mean_error_m <= anchor_params.strong_opt_only_max_mean_error_m &&
        edge.max_error_m <= anchor_params.strong_opt_only_max_error_m;

    if (anchor_params.require_fusion_for_loop_anchor &&
        !edge.usable_for_fusion &&
        !strong_opt_only_anchor_ok)
    {
        std::cerr
            << "[LOOP-ANCHOR-REJECT] method=stored_loop"
            << " reason=anchor_requires_fusion_or_strong_opt_only"
            << " q=drone_" << edge.query_drone_id
            << "/epoch_" << edge.query_map_epoch
            << "/kf_" << edge.query_local_kf_id
            << " c=drone_" << edge.candidate_drone_id
            << "/epoch_" << edge.candidate_map_epoch
            << "/kf_" << edge.candidate_local_kf_id
            << " opt=" << (edge.usable_for_optimization ? 1 : 0)
            << " fusion=" << (edge.usable_for_fusion ? 1 : 0)
            << " final=" << edge.final_inliers
            << " ransac=" << edge.ransac_inliers
            << " projection=" << edge.projection_matches
            << " ratio=" << edge.inlier_ratio
            << " mean=" << edge.mean_error_m
            << " max=" << edge.max_error_m
            << " strong_opt_only_ok=" << (strong_opt_only_anchor_ok ? 1 : 0)
            << std::endl;

        return false;
    }

    if (!anchor_params.enable_inter_drone_loop_anchor)
    {
        std::cerr
            << "[LOOP-ANCHOR-REJECT] method=stored_loop"
            << " reason=inter_drone_loop_anchor_disabled"
            << " q=drone_" << edge.query_drone_id
            << "/epoch_" << edge.query_map_epoch
            << "/kf_" << edge.query_local_kf_id
            << " c=drone_" << edge.candidate_drone_id
            << "/epoch_" << edge.candidate_map_epoch
            << "/kf_" << edge.candidate_local_kf_id
            << std::endl;

        return false;
    }

    // FASE 2:
    // Para propagar anchors entre submapas basta con que el loop sea
    // usable para optimización. No exigimos que sea usable para fusión
    // de landmarks.
    if (!edge.usable_for_optimization)
    {
        std::cerr
            << "[LOOP-ANCHOR-REJECT] reason=not_usable_for_optimization"
            << " q=drone_" << edge.query_drone_id
            << "/epoch_" << edge.query_map_epoch
            << "/kf_" << edge.query_local_kf_id
            << " c=drone_" << edge.candidate_drone_id
            << "/epoch_" << edge.candidate_map_epoch
            << "/kf_" << edge.candidate_local_kf_id
            << " fusion=" << (edge.usable_for_fusion ? 1 : 0)
            << " opt=" << (edge.usable_for_optimization ? 1 : 0)
            << std::endl;

        return false;
    }

    if (edge.query_kf_id == 0 ||
        edge.candidate_kf_id == 0)
    {
        return false;
    }

    const uint64_t query_anchor_key =
        MakeCameraKey(
            edge.query_drone_id,
            edge.query_map_epoch);

    const uint64_t candidate_anchor_key =
        MakeCameraKey(
            edge.candidate_drone_id,
            edge.candidate_map_epoch);

    bool query_has_anchor = false;
    bool candidate_has_anchor = false;

    MapAnchor query_anchor;
    MapAnchor candidate_anchor;

    {
        std::lock_guard<std::mutex> lock(anchors_mutex_);

        auto q_it =
            map_anchors_.find(query_anchor_key);

        if (q_it != map_anchors_.end() &&
            q_it->second.valid)
        {
            query_has_anchor = true;
            query_anchor = q_it->second;
        }

        auto c_it =
            map_anchors_.find(candidate_anchor_key);

        if (c_it != map_anchors_.end() &&
            c_it->second.valid)
        {
            candidate_has_anchor = true;
            candidate_anchor = c_it->second;
        }
    }

    // Solo sirve para propagar anchor si uno está anclado y el otro no.
    if (query_has_anchor == candidate_has_anchor)
    {
        std::cerr
            << "[LOOP-ANCHOR-REJECT] reason="
            << (query_has_anchor ? "both_already_anchored" : "both_unanchored")
            << " q=drone_" << edge.query_drone_id
            << "/epoch_" << edge.query_map_epoch
            << "/kf_" << edge.query_local_kf_id
            << " c=drone_" << edge.candidate_drone_id
            << "/epoch_" << edge.candidate_map_epoch
            << "/kf_" << edge.candidate_local_kf_id
            << " opt=" << (edge.usable_for_optimization ? 1 : 0)
            << " fusion=" << (edge.usable_for_fusion ? 1 : 0)
            << std::endl;

        return false;
    }

    if (!edge.candidate_T_query.allFinite())
    {
        std::cerr
            << "[LOOP-ANCHOR-REJECT] reason=transform_not_finite"
            << " q=drone_" << edge.query_drone_id
            << "/epoch_" << edge.query_map_epoch
            << "/kf_" << edge.query_local_kf_id
            << " c=drone_" << edge.candidate_drone_id
            << "/epoch_" << edge.candidate_map_epoch
            << "/kf_" << edge.candidate_local_kf_id
            << std::endl;

        return false;
    }

    Eigen::Matrix4d new_world_T_local =
        Eigen::Matrix4d::Identity();

    uint32_t new_drone_id = 0;
    uint64_t new_map_epoch = 0;

    if (query_has_anchor && !candidate_has_anchor)
    {
        // candidate_T_query transforma query_local -> candidate_local.
        // Si query está en world:
        // world_T_candidate = world_T_query * inverse(candidate_T_query)
        new_world_T_local =
            query_anchor.world_T_local *
            edge.candidate_T_query.inverse();

        new_drone_id =
            edge.candidate_drone_id;

        new_map_epoch =
            edge.candidate_map_epoch;
    }
    else if (candidate_has_anchor && !query_has_anchor)
    {
        // world_T_query = world_T_candidate * candidate_T_query
        new_world_T_local =
            candidate_anchor.world_T_local *
            edge.candidate_T_query;

        new_drone_id =
            edge.query_drone_id;

        new_map_epoch =
            edge.query_map_epoch;
    }

    if (!new_world_T_local.allFinite())
    {
        std::cerr
            << "[LOOP-ANCHOR-REJECT] method=stored_loop"
            << " reason=new_world_T_local_not_finite"
            << " target=drone_" << new_drone_id
            << "/epoch_" << new_map_epoch
            << std::endl;

        return false;
    }

    // ============================================================
    // Fase F:
    // No permitir que un único loop cree un anchor provisional.
    // Se requiere consenso de varias transformaciones inter-dron
    // compatibles para el mismo submapa flotante.
    // ============================================================

    {
        MapAnchor existing_target_anchor;
        const bool target_already_has_anchor =
            GetMapAnchorInfo(
                new_drone_id,
                new_map_epoch,
                existing_target_anchor);

        if (target_already_has_anchor &&
            existing_target_anchor.maturity == AnchorMaturity::CONFIRMED)
        {
            std::cerr
                << "[LOOP-ANCHOR-SKIP-CONFIRMED-TARGET]"
                << " method=stored_loop"
                << " target=drone_" << new_drone_id
                << "/epoch_" << new_map_epoch
                << " existing_source="
                << (existing_target_anchor.source == AnchorSource::FIDUCIAL_DIRECT
                        ? "FIDUCIAL_DIRECT"
                        : existing_target_anchor.source == AnchorSource::LOOP_OR_PROPAGATED
                            ? "LOOP_OR_PROPAGATED"
                            : "UNKNOWN")
                << " existing_maturity=CONFIRMED"
                << " action=do_not_create_new_provisional_anchor"
                << std::endl;

            return false;
        }
    }

    if (anchor_params.require_consensus_for_loop_anchor &&
        edge.type == LoopEdgeType::INTER_DRONE)
    {
        Eigen::Matrix4d consensus_world_T_local =
            Eigen::Matrix4d::Identity();

        InterAnchorConsensusDiagnostics consensus_diag;

        const bool consensus_ok =
            CheckInterDroneAnchorConsensus(
                new_drone_id,
                new_map_epoch,
                consensus_world_T_local,
                &consensus_diag);

        if (!consensus_ok)
        {
            std::cerr
                << "[LOOP-ANCHOR-REJECT] method=stored_loop"
                << " reason=inter_anchor_consensus_failed"
                << " target=drone_" << new_drone_id
                << "/epoch_" << new_map_epoch
                << " consensus_reason=" << consensus_diag.reason
                << " loops_total=" << consensus_diag.loops_total
                << " usable_loops=" << consensus_diag.usable_loops
                << " candidate_transforms="
                << consensus_diag.candidate_transforms
                << " best_cluster_size="
                << consensus_diag.best_cluster_size
                << " translation_spread="
                << consensus_diag.best_translation_spread_m
                << " yaw_spread_deg="
                << consensus_diag.best_yaw_spread_deg
                << " q=drone_" << edge.query_drone_id
                << "/epoch_" << edge.query_map_epoch
                << "/kf_" << edge.query_local_kf_id
                << " c=drone_" << edge.candidate_drone_id
                << "/epoch_" << edge.candidate_map_epoch
                << "/kf_" << edge.candidate_local_kf_id
                << std::endl;

            return false;
        }

        new_world_T_local =
            consensus_world_T_local;

        std::cerr
            << "[LOOP-ANCHOR-CONSENSUS-ACCEPT]"
            << " target=drone_" << new_drone_id
            << "/epoch_" << new_map_epoch
            << " best_cluster_size="
            << consensus_diag.best_cluster_size
            << " translation_spread="
            << consensus_diag.best_translation_spread_m
            << " yaw_spread_deg="
            << consensus_diag.best_yaw_spread_deg
            << " reason=" << consensus_diag.reason
            << std::endl;
    }

    const Eigen::Vector3d t =
        new_world_T_local.block<3, 1>(0, 3);

    const double t_norm =
        t.norm();

    if (!std::isfinite(t_norm) ||
        t_norm > anchor_params.max_translation_norm_m)
    {
        std::cerr
            << "[LOOP-ANCHOR-REJECT] method=stored_loop"
            << " reason=translation_norm_unreasonable"
            << " target=drone_" << new_drone_id
            << "/epoch_" << new_map_epoch
            << " t_norm=" << t_norm
            << std::endl;

        return false;
    }

    SetMapAnchor(
        new_drone_id,
        new_map_epoch,
        new_world_T_local,
        AnchorSource::LOOP_OR_PROPAGATED,
        AnchorMaturity::PROVISIONAL);

    std::cerr
        << "[ANCHOR-MATURITY]"
        << " drone_" << new_drone_id
        << " epoch=" << new_map_epoch
        << " state=PROVISIONAL_LOOP_ANCHOR"
        << " source=stored_loop"
        << " q=drone_" << edge.query_drone_id
        << "/epoch_" << edge.query_map_epoch
        << "/kf_" << edge.query_local_kf_id
        << " c=drone_" << edge.candidate_drone_id
        << "/epoch_" << edge.candidate_map_epoch
        << "/kf_" << edge.candidate_local_kf_id
        << " final=" << edge.final_inliers
        << " fusion=" << (edge.usable_for_fusion ? 1 : 0)
        << " opt=" << (edge.usable_for_optimization ? 1 : 0)
        << std::endl;


    return true;
}


bool MultiDroneSystem::CheckInterDroneAnchorConsensus(
    uint32_t target_drone_id,
    uint64_t target_map_epoch,
    Eigen::Matrix4d& consensus_world_T_local,
    InterAnchorConsensusDiagnostics* diagnostics) const
{
    InterAnchorConsensusDiagnostics diag;

    diag.target_drone_id =
        target_drone_id;

    diag.target_map_epoch =
        target_map_epoch;

    consensus_world_T_local =
        Eigen::Matrix4d::Identity();

    const LoopAnchorParams anchor_params =
        GetLoopAnchorParams();

    struct CandidateTransform
    {
        Eigen::Matrix4d world_T_local =
            Eigen::Matrix4d::Identity();

        double yaw_rad = 0.0;

        double quality = 0.0;

        uint64_t q_kf = 0;
        uint64_t c_kf = 0;

        uint32_t q_drone = 0;
        uint64_t q_epoch = 0;
        uint64_t q_local_kf = 0;

        uint32_t c_drone = 0;
        uint64_t c_epoch = 0;
        uint64_t c_local_kf = 0;

        bool fusion = false;
    };

    std::vector<VerifiedLoopEdge> loops =
        GetLoopEdges();

    diag.loops_total =
        loops.size();

    std::vector<CandidateTransform> candidates;

    auto passes_anchor_quality_policy =
        [&](const VerifiedLoopEdge& e) -> bool
        {
            const bool strong_opt_only_anchor_ok =
                anchor_params.allow_strong_opt_only_anchor &&
                e.final_inliers >= anchor_params.strong_opt_only_min_final_inliers &&
                e.ransac_inliers >= anchor_params.strong_opt_only_min_ransac_inliers &&
                e.projection_matches >= anchor_params.strong_opt_only_min_projection_matches &&
                e.inlier_ratio >= anchor_params.strong_opt_only_min_inlier_ratio &&
                e.mean_error_m <= anchor_params.strong_opt_only_max_mean_error_m &&
                e.max_error_m <= anchor_params.strong_opt_only_max_error_m;

            if (anchor_params.require_fusion_for_loop_anchor &&
                !e.usable_for_fusion &&
                !strong_opt_only_anchor_ok)
            {
                return false;
            }

            return true;
        };

    for (const auto& e : loops)
    {
        if (e.type != LoopEdgeType::INTER_DRONE)
        {
            diag.rejected_not_inter++;
            continue;
        }

        diag.inter_loops_total++;

        if (!e.usable_for_anchor)
        {
            diag.rejected_not_usable_for_anchor++;
            continue;
        }

        if (!e.usable_for_optimization)
        {
            diag.rejected_not_usable_for_optimization++;
            continue;
        }

        if (!passes_anchor_quality_policy(e))
        {
            diag.rejected_requires_fusion_or_strong++;
            continue;
        }

        if (!e.candidate_T_query.allFinite())
        {
            diag.rejected_bad_transform++;
            continue;
        }

        MapAnchor query_anchor;
        MapAnchor candidate_anchor;

        const bool query_has_anchor =
            GetMapAnchorInfo(
                e.query_drone_id,
                e.query_map_epoch,
                query_anchor);

        const bool candidate_has_anchor =
            GetMapAnchorInfo(
                e.candidate_drone_id,
                e.candidate_map_epoch,
                candidate_anchor);

        if (query_has_anchor && candidate_has_anchor)
        {
            diag.rejected_both_anchored++;
            continue;
        }

        if (!query_has_anchor && !candidate_has_anchor)
        {
            diag.rejected_both_unanchored++;
            continue;
        }

        Eigen::Matrix4d candidate_world_T_local =
            Eigen::Matrix4d::Identity();

        uint32_t candidate_target_drone = 0;
        uint64_t candidate_target_epoch = 0;

        if (query_has_anchor && !candidate_has_anchor)
        {
            candidate_world_T_local =
                query_anchor.world_T_local *
                e.candidate_T_query.inverse();

            candidate_target_drone =
                e.candidate_drone_id;

            candidate_target_epoch =
                e.candidate_map_epoch;
        }
        else
        {
            candidate_world_T_local =
                candidate_anchor.world_T_local *
                e.candidate_T_query;

            candidate_target_drone =
                e.query_drone_id;

            candidate_target_epoch =
                e.query_map_epoch;
        }

        if (candidate_target_drone != target_drone_id ||
            candidate_target_epoch != target_map_epoch)
        {
            diag.rejected_wrong_target++;
            continue;
        }

        if (!candidate_world_T_local.allFinite())
        {
            diag.rejected_bad_transform++;
            continue;
        }

        CandidateTransform c;

        c.world_T_local =
            candidate_world_T_local;

        c.yaw_rad =
            YawFromMatrix(candidate_world_T_local);

        c.quality =
            e.quality_score;

        c.q_kf =
            e.query_kf_id;

        c.c_kf =
            e.candidate_kf_id;

        c.q_drone =
            e.query_drone_id;

        c.q_epoch =
            e.query_map_epoch;

        c.q_local_kf =
            e.query_local_kf_id;

        c.c_drone =
            e.candidate_drone_id;

        c.c_epoch =
            e.candidate_map_epoch;

        c.c_local_kf =
            e.candidate_local_kf_id;

        c.fusion =
            e.usable_for_fusion;

        candidates.push_back(c);

        diag.usable_loops++;
    }

    diag.candidate_transforms =
        candidates.size();

    if (candidates.empty())
    {
        diag.accepted = false;
        diag.reason = "no_candidate_transforms_for_target";

        std::cerr
            << "[INTER-ANCHOR-CONSENSUS]"
            << " method=tight_pairwise"
            << " target=drone_" << target_drone_id
            << "/epoch_" << target_map_epoch
            << " loops_total=" << diag.loops_total
            << " inter_loops=" << diag.inter_loops_total
            << " usable_loops=" << diag.usable_loops
            << " candidate_transforms=0"
            << " clusters=0"
            << " best_cluster_size=0"
            << " accepted=0"
            << " reason=" << diag.reason
            << " reject_not_anchor=" << diag.rejected_not_usable_for_anchor
            << " reject_requires_fusion=" << diag.rejected_requires_fusion_or_strong
            << " reject_wrong_target=" << diag.rejected_wrong_target
            << " reject_both_anchored=" << diag.rejected_both_anchored
            << " reject_both_unanchored=" << diag.rejected_both_unanchored
            << std::endl;

        if (diagnostics)
            *diagnostics = diag;

        return false;
    }

    const double yaw_threshold_rad =
        anchor_params.consensus_max_yaw_spread_deg *
        3.14159265358979323846 /
        180.0;

    auto compatible_pair =
        [&](size_t a, size_t b) -> bool
        {
            const Eigen::Vector3d t_a =
                candidates[a].world_T_local.block<3, 1>(0, 3);

            const Eigen::Vector3d t_b =
                candidates[b].world_T_local.block<3, 1>(0, 3);

            const double translation_diff =
                (t_a - t_b).norm();

            const double yaw_diff =
                AngleDistanceRad(
                    candidates[a].yaw_rad,
                    candidates[b].yaw_rad);

            return
                translation_diff <= anchor_params.consensus_max_translation_spread_m &&
                yaw_diff <= yaw_threshold_rad;
        };

    auto compute_cluster_stats =
        [&](const std::vector<size_t>& cluster,
            double& translation_spread,
            double& yaw_spread,
            double& quality_sum)
        {
            translation_spread = 0.0;
            yaw_spread = 0.0;
            quality_sum = 0.0;

            for (size_t a = 0; a < cluster.size(); ++a)
            {
                quality_sum +=
                    candidates[cluster[a]].quality;

                const Eigen::Vector3d t_a =
                    candidates[cluster[a]].world_T_local.block<3, 1>(0, 3);

                const double yaw_a =
                    candidates[cluster[a]].yaw_rad;

                for (size_t b = a + 1; b < cluster.size(); ++b)
                {
                    const Eigen::Vector3d t_b =
                        candidates[cluster[b]].world_T_local.block<3, 1>(0, 3);

                    const double yaw_b =
                        candidates[cluster[b]].yaw_rad;

                    translation_spread =
                        std::max(
                            translation_spread,
                            (t_a - t_b).norm());

                    yaw_spread =
                        std::max(
                            yaw_spread,
                            AngleDistanceRad(
                                yaw_a,
                                yaw_b));
                }
            }
        };

    size_t best_seed_idx = 0;
    std::vector<size_t> best_cluster;
    double best_quality_sum = -1.0;
    double best_translation_spread = std::numeric_limits<double>::infinity();
    double best_yaw_spread = std::numeric_limits<double>::infinity();

    // ============================================================
    // Fase 3B:
    // Antes se elegía un cluster grande por cercanía a una semilla.
    // Eso podía crear un cluster con forma de cadena:
    //   todos cerca de la semilla, pero no todos cerca entre sí.
    //
    // Ahora construimos un subcluster tight:
    //   cada nuevo miembro debe ser compatible con TODOS los miembros
    //   ya aceptados.
    //
    // Esto mantiene el límite de seguridad de 0.60 m, pero evita rechazar
    // un subgrupo bueno por culpa de outliers o candidatos extendidos.
    // ============================================================

    for (size_t seed = 0; seed < candidates.size(); ++seed)
    {
        std::vector<size_t> neighborhood;
        neighborhood.push_back(seed);

        for (size_t j = 0; j < candidates.size(); ++j)
        {
            if (j == seed)
                continue;

            if (compatible_pair(seed, j))
            {
                neighborhood.push_back(j);
            }
        }

        std::sort(
            neighborhood.begin(),
            neighborhood.end(),
            [&](size_t a, size_t b)
            {
                return candidates[a].quality > candidates[b].quality;
            });

        std::vector<size_t> cluster;

        for (size_t idx : neighborhood)
        {
            bool compatible_with_all =
                true;

            for (size_t existing : cluster)
            {
                if (!compatible_pair(idx, existing))
                {
                    compatible_with_all =
                        false;

                    break;
                }
            }

            if (compatible_with_all)
            {
                cluster.push_back(idx);
            }
        }

        double cluster_translation_spread = 0.0;
        double cluster_yaw_spread = 0.0;
        double cluster_quality_sum = 0.0;

        compute_cluster_stats(
            cluster,
            cluster_translation_spread,
            cluster_yaw_spread,
            cluster_quality_sum);

        diag.clusters++;

        const bool better =
            cluster.size() > best_cluster.size() ||
            (
                cluster.size() == best_cluster.size() &&
                cluster_quality_sum > best_quality_sum
            );

        if (better)
        {
            best_seed_idx =
                seed;

            best_cluster =
                cluster;

            best_quality_sum =
                cluster_quality_sum;

            best_translation_spread =
                cluster_translation_spread;

            best_yaw_spread =
                cluster_yaw_spread;
        }
    }

    diag.best_cluster_size =
        best_cluster.size();

    diag.best_translation_spread_m =
        std::isfinite(best_translation_spread)
            ? best_translation_spread
            : -1.0;

    diag.best_yaw_spread_deg =
        std::isfinite(best_yaw_spread)
            ? best_yaw_spread * 180.0 / 3.14159265358979323846
            : -1.0;

    diag.best_quality_sum =
        best_quality_sum;

    const bool cluster_size_ok =
        diag.best_cluster_size >=
        static_cast<size_t>(
            anchor_params.consensus_min_cluster_size);

    const bool translation_spread_ok =
        diag.best_translation_spread_m >= 0.0 &&
        diag.best_translation_spread_m <=
            anchor_params.consensus_max_translation_spread_m;

    const bool yaw_spread_ok =
        diag.best_yaw_spread_deg >= 0.0 &&
        diag.best_yaw_spread_deg <=
            anchor_params.consensus_max_yaw_spread_deg;

    diag.accepted =
        cluster_size_ok &&
        translation_spread_ok &&
        yaw_spread_ok;

    if (!cluster_size_ok)
    {
        diag.reason =
            "cluster_too_small";
    }
    else if (!translation_spread_ok)
    {
        diag.reason =
            "translation_spread_too_large";
    }
    else if (!yaw_spread_ok)
    {
        diag.reason =
            "yaw_spread_too_large";
    }
    else
    {
        diag.reason =
            "consensus_ok";
    }

    if (diag.accepted)
    {
        Eigen::Vector3d mean_t =
            Eigen::Vector3d::Zero();

        double best_member_quality =
            -1.0;

        size_t best_member_idx =
            best_seed_idx;

        for (size_t idx : best_cluster)
        {
            mean_t +=
                candidates[idx].world_T_local.block<3, 1>(0, 3);

            if (candidates[idx].quality > best_member_quality)
            {
                best_member_quality =
                    candidates[idx].quality;

                best_member_idx =
                    idx;
            }
        }

        mean_t /=
            static_cast<double>(
                best_cluster.size());

        consensus_world_T_local =
            candidates[best_member_idx].world_T_local;

        consensus_world_T_local.block<3, 1>(0, 3) =
            mean_t;
    }

    std::cerr
        << "[INTER-ANCHOR-CONSENSUS]"
        << " method=tight_pairwise"
        << " target=drone_" << target_drone_id
        << "/epoch_" << target_map_epoch
        << " loops_total=" << diag.loops_total
        << " inter_loops=" << diag.inter_loops_total
        << " usable_loops=" << diag.usable_loops
        << " candidate_transforms=" << diag.candidate_transforms
        << " clusters=" << diag.clusters
        << " best_cluster_size=" << diag.best_cluster_size
        << " min_cluster=" << anchor_params.consensus_min_cluster_size
        << " translation_spread=" << diag.best_translation_spread_m
        << " max_translation_spread="
        << anchor_params.consensus_max_translation_spread_m
        << " yaw_spread_deg=" << diag.best_yaw_spread_deg
        << " max_yaw_spread_deg="
        << anchor_params.consensus_max_yaw_spread_deg
        << " quality_sum=" << diag.best_quality_sum
        << " accepted=" << (diag.accepted ? 1 : 0)
        << " reason=" << diag.reason
        << " reject_not_anchor=" << diag.rejected_not_usable_for_anchor
        << " reject_requires_fusion="
        << diag.rejected_requires_fusion_or_strong
        << " reject_wrong_target=" << diag.rejected_wrong_target
        << " reject_both_anchored=" << diag.rejected_both_anchored
        << " reject_both_unanchored=" << diag.rejected_both_unanchored
        << std::endl;

    if (anchor_params.consensus_log_details &&
        diag.accepted)
    {
        for (size_t idx : best_cluster)
        {
            const auto& c =
                candidates[idx];

            std::cerr
                << "[INTER-ANCHOR-CONSENSUS-MEMBER]"
                << " target=drone_" << target_drone_id
                << "/epoch_" << target_map_epoch
                << " q=drone_" << c.q_drone
                << "/epoch_" << c.q_epoch
                << "/kf_" << c.q_local_kf
                << " c=drone_" << c.c_drone
                << "/epoch_" << c.c_epoch
                << "/kf_" << c.c_local_kf
                << " fusion=" << (c.fusion ? 1 : 0)
                << " quality=" << c.quality
                << " t=("
                << c.world_T_local(0, 3) << ","
                << c.world_T_local(1, 3) << ","
                << c.world_T_local(2, 3) << ")"
                << std::endl;
        }
    }

    if (diagnostics)
        *diagnostics = diag;

    return diag.accepted;
}


bool MultiDroneSystem::TryAnchorMapsFromStoredLoops(
    int max_iterations)
{
    bool any_changed = false;

    if (max_iterations < 1)
        max_iterations = 1;

    size_t total_loops_seen = 0;
    size_t total_anchor_attempts = 0;
    size_t total_anchor_success = 0;

    for (int iter = 0; iter < max_iterations; ++iter)
    {
        std::vector<VerifiedLoopEdge> loops =
            GetLoopEdges();

        total_loops_seen += loops.size();

        std::sort(
            loops.begin(),
            loops.end(),
            [](const VerifiedLoopEdge& a,
               const VerifiedLoopEdge& b)
            {
                return a.quality_score > b.quality_score;
            });

        bool changed_this_iter = false;

        size_t iter_attempts = 0;
        size_t iter_success = 0;

        for (const auto& edge : loops)
        {
            if (!edge.usable_for_optimization)
                continue;

            if (!edge.usable_for_anchor)
            {
                std::cerr
                    << "[LOOP-ANCHOR-PROPAGATE-SKIP]"
                    << " reason=not_usable_for_anchor"
                    << " anchor_reject_reason=" << edge.anchor_reject_reason
                    << " q=drone_" << edge.query_drone_id
                    << "/epoch_" << edge.query_map_epoch
                    << "/kf_" << edge.query_local_kf_id
                    << " c=drone_" << edge.candidate_drone_id
                    << "/epoch_" << edge.candidate_map_epoch
                    << "/kf_" << edge.candidate_local_kf_id
                    << std::endl;

                continue;
            }

            iter_attempts++;
            total_anchor_attempts++;

            if (TryAnchorMapFromStoredLoopEdge(edge))
            {
                changed_this_iter = true;
                any_changed = true;
                iter_success++;
                total_anchor_success++;
            }
        }

        std::cerr
            << "[LOOP-ANCHOR-PROPAGATE-ITER] iter=" << iter
            << " loops=" << loops.size()
            << " attempts=" << iter_attempts
            << " success=" << iter_success
            << " changed=" << (changed_this_iter ? 1 : 0)
            << std::endl;

        if (!changed_this_iter)
            break;
    }

    std::cerr
        << "[LOOP-ANCHOR-PROPAGATE-SUMMARY]"
        << " max_iterations=" << max_iterations
        << " loops_seen=" << total_loops_seen
        << " attempts=" << total_anchor_attempts
        << " success=" << total_anchor_success
        << " any_changed=" << (any_changed ? 1 : 0)
        << std::endl;

    return any_changed;
}


std::vector<VerifiedLoopEdge> MultiDroneSystem::GetLoopEdges() const
{
    std::lock_guard<std::mutex> lock(loop_edges_mutex_);

    std::vector<VerifiedLoopEdge> out;
    out.reserve(loop_edges_.size());

    for (const auto& [key, edge] : loop_edges_)
    {
        (void)key;
        out.push_back(edge);
    }

    return out;
}

size_t MultiDroneSystem::GetLoopEdgeCount() const
{
    std::lock_guard<std::mutex> lock(loop_edges_mutex_);
    return loop_edges_.size();
}


Eigen::Matrix4d MultiDroneSystem::KeyFramePoseToMatrix(
    const ImportedKeyFrame& kf) const
{
    Eigen::Matrix4d T =
        Eigen::Matrix4d::Identity();

    Eigen::Quaterniond q =
        kf.orientation;

    q.normalize();

    T.block<3, 3>(0, 0) =
        q.toRotationMatrix();

    T.block<3, 1>(0, 3) =
        kf.position;

    return T;
}

Eigen::Matrix4d MultiDroneSystem::RelativePose(
    const Eigen::Matrix4d& world_T_from,
    const Eigen::Matrix4d& world_T_to) const
{
    return world_T_from.inverse() * world_T_to;
}

void MultiDroneSystem::SetMapAnchor(
    uint32_t drone_id,
    uint64_t map_epoch,
    const Eigen::Matrix4d& world_T_local,
    AnchorSource source,
    AnchorMaturity maturity)
{
    std::lock_guard<std::mutex> lock(anchors_mutex_);

    const uint64_t key =
        MakeCameraKey(drone_id, map_epoch);

    AnchorSource final_source =
        source;

    AnchorMaturity final_maturity =
        maturity;

    auto old_it =
        map_anchors_.find(key);

    if (old_it != map_anchors_.end() &&
        old_it->second.valid)
    {
        const MapAnchor& old_anchor =
            old_it->second;

        const bool old_is_confirmed =
            old_anchor.maturity == AnchorMaturity::CONFIRMED;

        const bool requested_is_provisional =
            maturity == AnchorMaturity::PROVISIONAL ||
            (maturity == AnchorMaturity::UNKNOWN &&
            source == AnchorSource::LOOP_OR_PROPAGATED);

        const bool requested_is_loop_anchor =
            source == AnchorSource::LOOP_OR_PROPAGATED ||
            source == AnchorSource::UNKNOWN;

        if (old_is_confirmed &&
            requested_is_provisional &&
            requested_is_loop_anchor)
        {
            const Eigen::Vector3d old_t =
                old_anchor.world_T_local.block<3, 1>(0, 3);

            const Eigen::Vector3d new_t =
                world_T_local.block<3, 1>(0, 3);

            const double delta_m =
                (old_t - new_t).norm();

            std::cerr
                << "[ANCHOR-MATURITY-DOWNGRADE-BLOCK]"
                << " drone_" << drone_id
                << " epoch=" << map_epoch
                << " old_source="
                << (old_anchor.source == AnchorSource::FIDUCIAL_DIRECT
                        ? "FIDUCIAL_DIRECT"
                        : old_anchor.source == AnchorSource::LOOP_OR_PROPAGATED
                            ? "LOOP_OR_PROPAGATED"
                            : "UNKNOWN")
                << " old_maturity=CONFIRMED"
                << " requested_source="
                << (source == AnchorSource::FIDUCIAL_DIRECT
                        ? "FIDUCIAL_DIRECT"
                        : source == AnchorSource::LOOP_OR_PROPAGATED
                            ? "LOOP_OR_PROPAGATED"
                            : "UNKNOWN")
                << " requested_maturity=PROVISIONAL"
                << " delta_translation_m=" << delta_m
                << " action=keep_existing_confirmed_anchor"
                << std::endl;

            return;
        }

        if (old_anchor.source == AnchorSource::FIDUCIAL_DIRECT &&
            source == AnchorSource::LOOP_OR_PROPAGATED)
        {
            std::cerr
                << "[ANCHOR-SOURCE-DOWNGRADE-BLOCK]"
                << " drone_" << drone_id
                << " epoch=" << map_epoch
                << " old_source=FIDUCIAL_DIRECT"
                << " requested_source=LOOP_OR_PROPAGATED"
                << " action=keep_fiducial_direct_anchor"
                << std::endl;

            return;
        }
    }

    if (old_it != map_anchors_.end() &&
        old_it->second.valid)
    {
        if (final_source == AnchorSource::UNKNOWN)
        {
            final_source =
                old_it->second.source;
        }

        if (final_maturity == AnchorMaturity::UNKNOWN)
        {
            final_maturity =
                old_it->second.maturity;
        }
    }

    if (final_source == AnchorSource::UNKNOWN)
    {
        final_source =
            AnchorSource::LOOP_OR_PROPAGATED;
    }

    if (final_maturity == AnchorMaturity::UNKNOWN)
    {
        final_maturity =
            final_source == AnchorSource::FIDUCIAL_DIRECT
                ? AnchorMaturity::CONFIRMED
                : AnchorMaturity::PROVISIONAL;
    }

    const bool had_old_anchor =
        old_it != map_anchors_.end() &&
        old_it->second.valid;

    AnchorSource old_source =
        AnchorSource::UNKNOWN;

    AnchorMaturity old_maturity =
        AnchorMaturity::UNKNOWN;

    if (had_old_anchor)
    {
        old_source =
            old_it->second.source;

        old_maturity =
            old_it->second.maturity;
    }

    MapAnchor anchor;
    anchor.drone_id = drone_id;
    anchor.map_epoch = map_epoch;
    anchor.world_T_local = world_T_local;
    anchor.valid = true;
    anchor.source = final_source;
    anchor.maturity = final_maturity;

    map_anchors_[key] = anchor;

    std::cerr
        << "[ANCHOR-MATURITY-MULTI-SET]"
        << " drone_" << drone_id
        << " epoch=" << map_epoch
        << " source="
        << (anchor.source == AnchorSource::FIDUCIAL_DIRECT
                ? "FIDUCIAL_DIRECT"
                : anchor.source == AnchorSource::LOOP_OR_PROPAGATED
                    ? "LOOP_OR_PROPAGATED"
                    : "UNKNOWN")
        << " maturity="
        << (anchor.maturity == AnchorMaturity::CONFIRMED
                ? "CONFIRMED"
                : anchor.maturity == AnchorMaturity::PROVISIONAL
                    ? "PROVISIONAL"
                    : "UNKNOWN")
        << " had_old=" << (had_old_anchor ? 1 : 0)
        << " old_source="
        << (old_source == AnchorSource::FIDUCIAL_DIRECT
                ? "FIDUCIAL_DIRECT"
                : old_source == AnchorSource::LOOP_OR_PROPAGATED
                    ? "LOOP_OR_PROPAGATED"
                    : "UNKNOWN")
        << " old_maturity="
        << (old_maturity == AnchorMaturity::CONFIRMED
                ? "CONFIRMED"
                : old_maturity == AnchorMaturity::PROVISIONAL
                    ? "PROVISIONAL"
                    : "UNKNOWN")
        << std::endl;
}

bool MultiDroneSystem::HasMapAnchor(
    uint32_t drone_id,
    uint64_t map_epoch) const
{
    std::lock_guard<std::mutex> lock(anchors_mutex_);

    const uint64_t key =
        MakeCameraKey(drone_id, map_epoch);

    auto it =
        map_anchors_.find(key);

    return it != map_anchors_.end() && it->second.valid;
}


bool MultiDroneSystem::GetMapAnchor(
    uint32_t drone_id,
    uint64_t map_epoch,
    Eigen::Matrix4d& world_T_local_out) const
{
    std::lock_guard<std::mutex> lock(anchors_mutex_);

    const uint64_t key =
        MakeCameraKey(
            drone_id,
            map_epoch);

    auto it =
        map_anchors_.find(key);

    if (it == map_anchors_.end() ||
        !it->second.valid)
    {
        return false;
    }

    world_T_local_out =
        it->second.world_T_local;

    return world_T_local_out.allFinite();
}



bool MultiDroneSystem::GetMapAnchorInfo(
    uint32_t drone_id,
    uint64_t map_epoch,
    MapAnchor& anchor_out) const
{
    std::lock_guard<std::mutex> lock(anchors_mutex_);

    const uint64_t key =
        MakeCameraKey(
            drone_id,
            map_epoch);

    auto it =
        map_anchors_.find(key);

    if (it == map_anchors_.end() ||
        !it->second.valid)
    {
        return false;
    }

    anchor_out =
        it->second;

    return anchor_out.world_T_local.allFinite();
}



uint64_t MultiDroneSystem::MakeSubmapKeyPublic(
    uint32_t drone_id,
    uint64_t map_epoch) const
{
    return MakeSubmapKeyForFusionDiag(
        drone_id,
        map_epoch);
}



PoseGraphSnapshot MultiDroneSystem::BuildPoseGraphSnapshot(
    int min_covisibility_weight,
    int max_covisibility_edges_per_keyframe) const
{
    PoseGraphSnapshot snapshot;

    if (!atlas_)
        return snapshot;

    std::vector<ImportedKeyFrame> keyframes =
        atlas_->GetAllKeyFrames();

    std::unordered_map<uint64_t, PoseGraphVertex> vertices_by_id;

    std::unordered_map<uint64_t, uint64_t> first_local_kf_by_anchor;

    for (const auto& kf : keyframes)
    {
        if (kf.global_id == 0 || kf.is_bad)
            continue;

        const uint64_t anchor_key =
            MakeCameraKey(kf.drone_id, kf.map_epoch);

        bool has_anchor = false;

        {
            std::lock_guard<std::mutex> lock(anchors_mutex_);

            auto it =
                map_anchors_.find(anchor_key);

            has_anchor =
                it != map_anchors_.end() &&
                it->second.valid;
        }

        if (!has_anchor)
            continue;

        auto first_it =
            first_local_kf_by_anchor.find(anchor_key);

        if (first_it == first_local_kf_by_anchor.end() ||
            kf.local_id < first_it->second)
        {
            first_local_kf_by_anchor[anchor_key] =
                kf.local_id;
        }
    }

    // ============================================================
    // 1. Crear vértices.
    // Solo entran KFs cuyo drone/map_epoch tenga anchor world_T_local.
    // ============================================================

    for (const auto& kf : keyframes)
    {
        if (kf.global_id == 0 || kf.is_bad)
            continue;

        const uint64_t anchor_key =
            MakeCameraKey(kf.drone_id, kf.map_epoch);

        MapAnchor anchor;

        {
            std::lock_guard<std::mutex> lock(anchors_mutex_);

            auto it =
                map_anchors_.find(anchor_key);

            if (it == map_anchors_.end() || !it->second.valid)
                continue;

            anchor = it->second;
        }

        PoseGraphVertex v;

        v.global_kf_id = kf.global_id;
        v.drone_id = kf.drone_id;
        v.map_epoch = kf.map_epoch;
        v.local_kf_id = kf.local_id;

        v.local_T_camera =
            KeyFramePoseToMatrix(kf);

        v.world_T_camera_initial =
            anchor.world_T_local * v.local_T_camera;

        v.world_T_camera_optimized =
            v.world_T_camera_initial;

        v.fixed = false;

        v.anchor_source =
            anchor.source;

        v.anchor_maturity =
            anchor.maturity;

        vertices_by_id[v.global_kf_id] = v;
    }

    uint64_t fixed_vertex_id = 0;
    bool has_fixed_vertex = false;

    /*
    * Preferimos fijar un KF con prior fiducial fuerte.
    *
    * Importante:
    * - No fijamos todos los fiducials.
    * - No fijamos varios KFs.
    * - Solo usamos un KF fiducial como gauge del grafo.
    *
    * Así el primer fiducial no se desplaza cuando entra el segundo,
    * pero el resto del grafo sigue pudiendo deformarse para corregir drift.
    */
    {
        std::lock_guard<std::mutex> lock(fiducial_constraints_mutex_);

        uint64_t best_local_kf_id =
            std::numeric_limits<uint64_t>::max();

        double best_weight = -1.0;

        for (const auto& c : fiducial_constraints_)
        {
            if (!c.valid)
                continue;

            if (c.global_kf_id == 0)
                continue;

            if (c.weight < 60.0)
                continue;

            if (vertices_by_id.find(c.global_kf_id) ==
                vertices_by_id.end())
            {
                continue;
            }

            const bool better =
                !has_fixed_vertex ||
                c.local_kf_id < best_local_kf_id ||
                (c.local_kf_id == best_local_kf_id &&
                c.weight > best_weight);

            if (better)
            {
                fixed_vertex_id = c.global_kf_id;
                best_local_kf_id = c.local_kf_id;
                best_weight = c.weight;
                has_fixed_vertex = true;
            }
        }
    }

    /*
    * Fallback: si aún no hay ningún prior fiducial fuerte,
    * usamos el vértice global más pequeño como antes.
    */
    if (!has_fixed_vertex)
    {
        for (const auto& [id, v] : vertices_by_id)
        {
            (void)v;

            if (!has_fixed_vertex || id < fixed_vertex_id)
            {
                fixed_vertex_id = id;
                has_fixed_vertex = true;
            }
        }
    }

    snapshot.fixed_vertex_count = 0;

    // ============================================================
    // Fase G-fix:
    // Fiduciales intocables.
    //
    // Antes solo fijábamos un KF como gauge. Eso evita el gauge libre,
    // pero no impide que el resto del mapa rote/deforme alrededor de
    // ese punto cuando entran loops inter-dron.
    //
    // Ahora:
    //   - todos los KFs con prior fiducial fuerte quedan fixed;
    //   - si no hay ningún prior fiducial fuerte, usamos el fallback
    //     antiguo de un único vértice fijo.
    //
    // Esto protege el fiducial, pero no congela necesariamente todos
    // los KFs del submapa: los KFs fuera del fiducial pueden seguir
    // refinándose por covisibilidad/loops cuando sea seguro.
    // ============================================================

    std::unordered_set<uint64_t> fixed_fiducial_vertices;

    {
        std::lock_guard<std::mutex> lock(
            fiducial_constraints_mutex_);

        for (const auto& c : fiducial_constraints_)
        {
            if (!c.valid)
                continue;

            if (c.global_kf_id == 0)
                continue;

            if (c.weight < 60.0)
                continue;

            if (vertices_by_id.find(c.global_kf_id) ==
                vertices_by_id.end())
            {
                continue;
            }

            fixed_fiducial_vertices.insert(
                c.global_kf_id);
        }
    }

    if (!fixed_fiducial_vertices.empty())
    {
        for (auto& [id, v] : vertices_by_id)
        {
            v.fixed =
                fixed_fiducial_vertices.find(id) !=
                fixed_fiducial_vertices.end();

            if (v.fixed)
            {
                snapshot.fixed_vertex_count++;
            }
        }
    }
    else if (has_fixed_vertex)
    {
        for (auto& [id, v] : vertices_by_id)
        {
            v.fixed =
                id == fixed_vertex_id;
        }

        snapshot.fixed_vertex_count = 1;
    }

    std::cerr
        << "[GRAPH-FIXED-POLICY]"
        << " fixed_fiducial_vertices="
        << fixed_fiducial_vertices.size()
        << " fixed_vertices="
        << snapshot.fixed_vertex_count
        << " fallback_single_fixed="
        << (fixed_fiducial_vertices.empty() && has_fixed_vertex ? 1 : 0)
        << std::endl;

    snapshot.vertices.reserve(vertices_by_id.size());

    for (const auto& [id, v] : vertices_by_id)
    {
        (void)id;
        snapshot.vertices.push_back(v);
    }

    auto has_vertex =
        [&vertices_by_id](uint64_t id) -> bool
        {
            return vertices_by_id.find(id) != vertices_by_id.end();
        };

    auto get_vertex =
        [&vertices_by_id](uint64_t id) -> const PoseGraphVertex&
        {
            return vertices_by_id.at(id);
        };

    std::unordered_set<LoopEdgePairKey, LoopEdgePairKeyHash> added_edges;

    auto try_add_edge =
        [&](uint64_t from_id,
            uint64_t to_id,
            PoseGraphEdgeType type,
            double weight,
            int inliers,
            double mean_error)
        {
            if (from_id == 0 || to_id == 0 || from_id == to_id)
                return;

            if (!has_vertex(from_id) || !has_vertex(to_id))
                return;

            LoopEdgePairKey key =
                MakeLoopEdgePairKey(from_id, to_id);

            if (added_edges.find(key) != added_edges.end())
                return;

            const PoseGraphVertex& from_v =
                get_vertex(from_id);

            const PoseGraphVertex& to_v =
                get_vertex(to_id);

            PoseGraphEdge edge;

            edge.from_kf_id = from_id;
            edge.to_kf_id = to_id;
            edge.type = type;
            edge.weight = weight;
            edge.inliers = inliers;
            edge.mean_error_m = mean_error;

            edge.from_T_to =
                RelativePose(
                    from_v.world_T_camera_initial,
                    to_v.world_T_camera_initial);

            snapshot.edges.push_back(edge);
            added_edges.insert(key);

            if (type == PoseGraphEdgeType::LOCAL_COVISIBILITY ||
                type == PoseGraphEdgeType::LOCAL_PARENT)
            {
                snapshot.local_edge_count++;
            }
            else
            {
                snapshot.loop_edge_count++;
            }
        };

    // ============================================================
    // 2. Edges locales por parent y covisibilidad.
    // ============================================================

    for (const auto& kf : keyframes)
    {
        if (kf.global_id == 0 || kf.is_bad)
            continue;

        if (!has_vertex(kf.global_id))
            continue;

        if (kf.parent_keyframe_id > 0)
        {
            try_add_edge(
                kf.parent_keyframe_id,
                kf.global_id,
                PoseGraphEdgeType::LOCAL_PARENT,
                10.0,
                0,
                0.0);
        }

        int added_cov_edges = 0;

        const size_t n =
            std::min(
                kf.connected_keyframe_ids.size(),
                kf.connected_keyframe_weights.size());

        for (size_t i = 0; i < n; ++i)
        {
            if (added_cov_edges >= max_covisibility_edges_per_keyframe)
                break;

            const uint64_t other_id =
                kf.connected_keyframe_ids[i];

            const int weight =
                static_cast<int>(kf.connected_keyframe_weights[i]);

            if (weight < min_covisibility_weight)
                continue;

            try_add_edge(
                kf.global_id,
                other_id,
                PoseGraphEdgeType::LOCAL_COVISIBILITY,
                static_cast<double>(weight),
                weight,
                0.0);

            added_cov_edges++;
        }
    }

    // ============================================================
    // 3. Edges de loop verificadas.
    // ============================================================


    {
        std::lock_guard<std::mutex> lock(
            fiducial_constraints_mutex_);

        size_t added_fiducial_constraints = 0;
        size_t rejected_no_vertex = 0;
        size_t rejected_invalid = 0;

        for (const auto& c : fiducial_constraints_)
        {
            if (!c.valid)
            {
                rejected_invalid++;
                continue;
            }

            if (!has_vertex(c.global_kf_id))
            {
                rejected_no_vertex++;
                continue;
            }

            snapshot.fiducial_constraints.push_back(c);
            added_fiducial_constraints++;
        }

        snapshot.fiducial_constraint_count =
            snapshot.fiducial_constraints.size();

        std::cerr
            << "[GRAPH8-FID-PRIOR] stored="
            << fiducial_constraints_.size()
            << " added=" << added_fiducial_constraints
            << " rejected_no_vertex=" << rejected_no_vertex
            << " rejected_invalid=" << rejected_invalid
            << std::endl;
    }

    {
        size_t anchors_total = 0;
        size_t anchors_fiducial_confirmed = 0;
        size_t anchors_loop_confirmed = 0;
        size_t anchors_loop_provisional = 0;
        size_t anchors_unknown = 0;

        {
            std::lock_guard<std::mutex> lock(anchors_mutex_);

            anchors_total =
                map_anchors_.size();

            for (const auto& kv : map_anchors_)
            {
                const MapAnchor& a =
                    kv.second;

                if (!a.valid)
                    continue;

                if (a.source == AnchorSource::FIDUCIAL_DIRECT &&
                    a.maturity == AnchorMaturity::CONFIRMED)
                {
                    anchors_fiducial_confirmed++;
                }
                else if (a.source == AnchorSource::LOOP_OR_PROPAGATED &&
                        a.maturity == AnchorMaturity::CONFIRMED)
                {
                    anchors_loop_confirmed++;
                }
                else if (a.source == AnchorSource::LOOP_OR_PROPAGATED &&
                        a.maturity == AnchorMaturity::PROVISIONAL)
                {
                    anchors_loop_provisional++;
                }
                else
                {
                    anchors_unknown++;
                }
            }
        }

        std::cerr
            << "[GRAPH-ANCHOR-SUMMARY]"
            << " anchors_total=" << anchors_total
            << " fiducial_confirmed=" << anchors_fiducial_confirmed
            << " loop_confirmed=" << anchors_loop_confirmed
            << " loop_provisional=" << anchors_loop_provisional
            << " unknown=" << anchors_unknown
            << " vertices=" << vertices_by_id.size()
            << std::endl;
    }


    std::vector<VerifiedLoopEdge> loops =
        GetLoopEdges();

    size_t loop_total = loops.size();
    size_t loop_added = 0;
    size_t loop_reject_no_query_vertex = 0;
    size_t loop_reject_no_candidate_vertex = 0;
    size_t loop_reject_bad_kf = 0;
    size_t loop_reject_bad_transform = 0;
    size_t loop_reject_duplicate = 0;

    int reject_not_usable_for_optimization = 0;

    // Fase G-fix:
    // Inter-loops almacenados pero no admitidos todavía en el pose graph.
    int reject_inter_pose_graph_disabled = 0;
    int reject_inter_unstable_anchor = 0;
    int accepted_inter_pose_graph = 0;

    for (const auto& loop : loops)
    {
        const bool has_query =
            has_vertex(loop.query_kf_id);

        const bool has_candidate =
            has_vertex(loop.candidate_kf_id);

        if (!has_query)
        {
            loop_reject_no_query_vertex++;

            std::cerr
                << "[GRAPH8-LOOP-REJECT] no query vertex q=drone_"
                << loop.query_drone_id
                << "/kf_" << loop.query_local_kf_id
                << " global=" << loop.query_kf_id
                << std::endl;

            continue;
        }

        if (!has_candidate)
        {
            loop_reject_no_candidate_vertex++;

            std::cerr
                << "[GRAPH8-LOOP-REJECT] no candidate vertex c=drone_"
                << loop.candidate_drone_id
                << "/kf_" << loop.candidate_local_kf_id
                << " global=" << loop.candidate_kf_id
                << std::endl;

            continue;
        }

        if (!loop.usable_for_optimization)
        {
            reject_not_usable_for_optimization++;

            std::cerr
                << "[GRAPH-LOOP-REJECT-OPT] reason=not_usable_for_optimization"
                << " q=drone_" << loop.query_drone_id
                << "/epoch_" << loop.query_map_epoch
                << "/kf_" << loop.query_local_kf_id
                << " c=drone_" << loop.candidate_drone_id
                << "/epoch_" << loop.candidate_map_epoch
                << "/kf_" << loop.candidate_local_kf_id
                << " final=" << loop.final_inliers
                << " ratio=" << loop.inlier_ratio
                << " mean=" << loop.mean_error_m
                << " max=" << loop.max_error_m
                << " fusion=" << (loop.usable_for_fusion ? 1 : 0)
                << std::endl;

            continue;
        }

        const LoopAnchorParams anchor_params =
            GetLoopAnchorParams();

        if (loop.type == LoopEdgeType::INTER_DRONE &&
            !anchor_params.use_inter_drone_loop_edges_for_pose_graph)
        {
            reject_inter_pose_graph_disabled++;

            std::cerr
                << "[GRAPH-LOOP-REJECT-INTER-POSE-GRAPH-DISABLED]"
                << " q=drone_" << loop.query_drone_id
                << "/epoch_" << loop.query_map_epoch
                << "/kf_" << loop.query_local_kf_id
                << " c=drone_" << loop.candidate_drone_id
                << "/epoch_" << loop.candidate_map_epoch
                << "/kf_" << loop.candidate_local_kf_id
                << " opt=" << (loop.usable_for_optimization ? 1 : 0)
                << " fusion=" << (loop.usable_for_fusion ? 1 : 0)
                << " action=keep_for_anchor_and_fusion_but_do_not_optimize_graph"
                << std::endl;

            continue;
        }

         const PoseGraphVertex& q_vertex_for_policy =
            vertices_by_id.at(loop.query_kf_id);

        const PoseGraphVertex& c_vertex_for_policy =
            vertices_by_id.at(loop.candidate_kf_id);

        auto anchor_source_to_string =
            [](AnchorSource source) -> const char*
            {
                switch (source)
                {
                    case AnchorSource::FIDUCIAL_DIRECT:
                        return "FIDUCIAL_DIRECT";
                    case AnchorSource::LOOP_OR_PROPAGATED:
                        return "LOOP_OR_PROPAGATED";
                    case AnchorSource::UNKNOWN:
                    default:
                        return "UNKNOWN";
                }
            };

        auto anchor_maturity_to_string =
            [](AnchorMaturity maturity) -> const char*
            {
                switch (maturity)
                {
                    case AnchorMaturity::CONFIRMED:
                        return "CONFIRMED";
                    case AnchorMaturity::PROVISIONAL:
                        return "PROVISIONAL";
                    case AnchorMaturity::UNKNOWN:
                    default:
                        return "UNKNOWN";
                }
            };

        auto stable_for_inter_pose_graph =
            [](const PoseGraphVertex& v) -> bool
            {
                if (v.anchor_source ==
                    AnchorSource::FIDUCIAL_DIRECT)
                {
                    return true;
                }

                if (v.anchor_maturity ==
                    AnchorMaturity::CONFIRMED)
                {
                    return true;
                }

                return false;
            };

        if (loop.type == LoopEdgeType::INTER_DRONE &&
            anchor_params.inter_loop_pose_graph_require_stable_anchors)
        {
            const bool q_stable =
                stable_for_inter_pose_graph(
                    q_vertex_for_policy);

            const bool c_stable =
                stable_for_inter_pose_graph(
                    c_vertex_for_policy);

            std::cerr
                << "[GRAPH-STABLE-ENDPOINT-CHECK]"
                << " q=drone_" << loop.query_drone_id
                << "/epoch_" << loop.query_map_epoch
                << "/kf_" << loop.query_local_kf_id
                << " q_source=" << anchor_source_to_string(q_vertex_for_policy.anchor_source)
                << " q_maturity=" << anchor_maturity_to_string(q_vertex_for_policy.anchor_maturity)
                << " q_stable=" << (q_stable ? 1 : 0)
                << " c=drone_" << loop.candidate_drone_id
                << "/epoch_" << loop.candidate_map_epoch
                << "/kf_" << loop.candidate_local_kf_id
                << " c_source=" << anchor_source_to_string(c_vertex_for_policy.anchor_source)
                << " c_maturity=" << anchor_maturity_to_string(c_vertex_for_policy.anchor_maturity)
                << " c_stable=" << (c_stable ? 1 : 0)
                << " require_stable=1"
                << std::endl;

            if (!q_stable || !c_stable)
            {
                reject_inter_unstable_anchor++;

                std::cerr
                    << "[GRAPH-LOOP-REJECT-INTER-UNSTABLE-ANCHOR]"
                    << " q=drone_" << loop.query_drone_id
                    << "/epoch_" << loop.query_map_epoch
                    << "/kf_" << loop.query_local_kf_id
                    << " q_source=" << anchor_source_to_string(q_vertex_for_policy.anchor_source)
                    << " q_maturity=" << anchor_maturity_to_string(q_vertex_for_policy.anchor_maturity)
                    << " c=drone_" << loop.candidate_drone_id
                    << "/epoch_" << loop.candidate_map_epoch
                    << "/kf_" << loop.candidate_local_kf_id
                    << " c_source=" << anchor_source_to_string(c_vertex_for_policy.anchor_source)
                    << " c_maturity=" << anchor_maturity_to_string(c_vertex_for_policy.anchor_maturity)
                    << " action=keep_for_anchor_and_fusion_but_do_not_optimize_graph"
                    << std::endl;

                continue;
            }

            accepted_inter_pose_graph++;

            std::cerr
                << "[GRAPH-LOOP-ACCEPT-INTER-STABLE]"
                << " q=drone_" << loop.query_drone_id
                << "/epoch_" << loop.query_map_epoch
                << "/kf_" << loop.query_local_kf_id
                << " c=drone_" << loop.candidate_drone_id
                << "/epoch_" << loop.candidate_map_epoch
                << "/kf_" << loop.candidate_local_kf_id
                << " q_source=" << anchor_source_to_string(q_vertex_for_policy.anchor_source)
                << " q_maturity=" << anchor_maturity_to_string(q_vertex_for_policy.anchor_maturity)
                << " c_source=" << anchor_source_to_string(c_vertex_for_policy.anchor_source)
                << " c_maturity=" << anchor_maturity_to_string(c_vertex_for_policy.anchor_maturity)
                << " action=use_as_pose_graph_constraint"
                << std::endl;
        }

        const ImportedKeyFrame query_kf =
            atlas_->GetKeyFrame(loop.query_kf_id);

        const ImportedKeyFrame candidate_kf =
            atlas_->GetKeyFrame(loop.candidate_kf_id);

        if (query_kf.global_id == 0 ||
            candidate_kf.global_id == 0 ||
            query_kf.is_bad ||
            candidate_kf.is_bad)
        {
            loop_reject_bad_kf++;

            std::cerr
                << "[GRAPH8-LOOP-REJECT] bad/missing KF q=drone_"
                << loop.query_drone_id
                << "/kf_" << loop.query_local_kf_id
                << " c=drone_"
                << loop.candidate_drone_id
                << "/kf_" << loop.candidate_local_kf_id
                << std::endl;

            continue;
        }

        const LoopEdgePairKey pair_key =
            MakeLoopEdgePairKey(
                loop.query_kf_id,
                loop.candidate_kf_id);

        if (added_edges.find(pair_key) != added_edges.end())
        {
            loop_reject_duplicate++;
            continue;
        }

        PoseGraphEdge edge;

        edge.from_kf_id =
            loop.candidate_kf_id;

        edge.to_kf_id =
            loop.query_kf_id;

        edge.type =
            (loop.type == LoopEdgeType::INTER_DRONE)
                ? PoseGraphEdgeType::LOOP_INTER
                : PoseGraphEdgeType::LOOP_INTRA;

        const PoseGraphVertex& q_vertex =
            get_vertex(loop.query_kf_id);

        const PoseGraphVertex& c_vertex =
            get_vertex(loop.candidate_kf_id);

        const bool is_inter_loop =
            loop.type == LoopEdgeType::INTER_DRONE;

        const bool endpoint_provisional =
            q_vertex.anchor_maturity == AnchorMaturity::PROVISIONAL ||
            c_vertex.anchor_maturity == AnchorMaturity::PROVISIONAL;

        const bool loop_is_fusion =
            loop.usable_for_fusion;

        double selected_weight = 1.0;
        const char* weight_reason = "legacy_dynamic";

        if (anchor_params.use_maturity_aware_loop_weights)
        {
            if (is_inter_loop)
            {
                if (endpoint_provisional)
                {
                    if (loop_is_fusion)
                    {
                        selected_weight =
                            anchor_params.loop_weight_inter_fusion_provisional;
                        weight_reason =
                            "inter_fusion_provisional";
                    }
                    else
                    {
                        selected_weight =
                            anchor_params.loop_weight_inter_opt_only_provisional;
                        weight_reason =
                            "inter_opt_only_provisional";
                    }
                }
                else
                {
                    if (loop_is_fusion)
                    {
                        selected_weight =
                            anchor_params.loop_weight_inter_fusion_confirmed;
                        weight_reason =
                            "inter_fusion_confirmed";
                    }
                    else
                    {
                        selected_weight =
                            anchor_params.loop_weight_inter_opt_only_confirmed;
                        weight_reason =
                            "inter_opt_only_confirmed";
                    }
                }
            }
            else
            {
                if (loop_is_fusion)
                {
                    selected_weight =
                        anchor_params.loop_weight_intra_fusion;
                    weight_reason =
                        "intra_fusion";
                }
                else
                {
                    selected_weight =
                        anchor_params.loop_weight_intra_opt_only;
                    weight_reason =
                        "intra_opt_only";
                }
            }

            edge.weight =
                std::max(
                    anchor_params.loop_weight_min,
                    std::min(
                        selected_weight,
                        anchor_params.loop_weight_max));
        }
        else
        {
            edge.weight =
                std::max(1.0, static_cast<double>(loop.final_inliers)) /
                std::max(0.05, loop.mean_error_m);

            if (loop.projection_matches > 0)
            {
                edge.weight *= 2.0;
            }

            edge.weight =
                std::min(edge.weight, 100.0);
        }

        edge.inliers =
            loop.final_inliers;

        edge.mean_error_m =
            loop.mean_error_m;

        const Eigen::Matrix4d Lq_T_Q =
            KeyFramePoseToMatrix(query_kf);

        const Eigen::Matrix4d Lc_T_C =
            KeyFramePoseToMatrix(candidate_kf);

        // Convención:
        // loop.candidate_T_query = Lc_T_Lq
        //
        // Queremos:
        // candidate_camera_T_query_camera =
        // inverse(Lc_T_C) * Lc_T_Lq * Lq_T_Q
        edge.from_T_to =
            Lc_T_C.inverse() *
            loop.candidate_T_query *
            Lq_T_Q;

        bool finite = true;

        for (int r = 0; r < 4; ++r)
        {
            for (int c = 0; c < 4; ++c)
            {
                if (!std::isfinite(edge.from_T_to(r, c)))
                {
                    finite = false;
                }
            }
        }

        if (!finite)
        {
            loop_reject_bad_transform++;

            std::cerr
                << "[GRAPH8-LOOP-REJECT] bad transform q=drone_"
                << loop.query_drone_id
                << "/kf_" << loop.query_local_kf_id
                << " c=drone_"
                << loop.candidate_drone_id
                << "/kf_" << loop.candidate_local_kf_id
                << std::endl;

            continue;
        }

        snapshot.edges.push_back(edge);
        snapshot.loop_edge_count++;
        added_edges.insert(pair_key);
        loop_added++;

        std::cerr
            << "[GRAPH-LOOP-WEIGHT]"
            << " type=" << (is_inter_loop ? "INTER" : "INTRA")
            << " fusion=" << (loop.usable_for_fusion ? 1 : 0)
            << " provisional=" << (endpoint_provisional ? 1 : 0)
            << " inliers=" << loop.final_inliers
            << " mean=" << loop.mean_error_m
            << " max=" << loop.max_error_m
            << " weight=" << edge.weight
            << " reason=" << weight_reason
            << std::endl;

        std::cerr
            << "[GRAPH-LOOP-USE] "
            << (edge.type == PoseGraphEdgeType::LOOP_INTER ? "INTER" : "INTRA")
            << " q=drone_" << loop.query_drone_id
            << "/epoch_" << loop.query_map_epoch
            << "/kf_" << loop.query_local_kf_id
            << " c=drone_" << loop.candidate_drone_id
            << "/epoch_" << loop.candidate_map_epoch
            << "/kf_" << loop.candidate_local_kf_id
            << " inliers=" << loop.final_inliers
            << " ratio=" << loop.inlier_ratio
            << " mean=" << loop.mean_error_m
            << " max=" << loop.max_error_m
            << " opt=" << (loop.usable_for_optimization ? 1 : 0)
            << " fusion=" << (loop.usable_for_fusion ? 1 : 0)
            << " q_anchor="
            << (q_vertex.anchor_maturity == AnchorMaturity::CONFIRMED
                    ? "CONFIRMED"
                    : q_vertex.anchor_maturity == AnchorMaturity::PROVISIONAL
                        ? "PROVISIONAL"
                        : "UNKNOWN")
            << " c_anchor="
            << (c_vertex.anchor_maturity == AnchorMaturity::CONFIRMED
                    ? "CONFIRMED"
                    : c_vertex.anchor_maturity == AnchorMaturity::PROVISIONAL
                        ? "PROVISIONAL"
                        : "UNKNOWN")
            << " weight=" << edge.weight
            << " weight_reason=" << weight_reason
            << std::endl;
    }

    std::cerr
        << "[PIPE0-GRAPH-LOOP-SUMMARY] "
        << "stored=" << loop_total
        << " added=" << loop_added
        << " reject_no_query_vertex=" << loop_reject_no_query_vertex
        << " reject_no_candidate_vertex=" << loop_reject_no_candidate_vertex
        << " reject_not_usable_for_optimization=" << reject_not_usable_for_optimization
        << " reject_bad_kf=" << loop_reject_bad_kf
        << " reject_bad_transform=" << loop_reject_bad_transform
        << " reject_duplicate=" << loop_reject_duplicate
        << " snapshot_loop_edges=" << snapshot.loop_edge_count
        << " total_edges=" << snapshot.edges.size()
        << " vertices=" << snapshot.vertices.size()
        << " fixed=" << snapshot.fixed_vertex_count
        << " fiducial_constraints=" << snapshot.fiducial_constraint_count
        << " reject_inter_pose_graph_disabled="
        << reject_inter_pose_graph_disabled
        << " reject_inter_unstable_anchor="
        << reject_inter_unstable_anchor
        << " accepted_inter_pose_graph="
        << accepted_inter_pose_graph
        << std::endl;

    return snapshot;
}

uint64_t MultiDroneSystem::FindUnionRoot(
    uint64_t id,
    std::unordered_map<uint64_t, uint64_t>& parent) const
{
    auto it = parent.find(id);

    if (it == parent.end())
    {
        parent[id] = id;
        return id;
    }

    if (it->second == id)
        return id;

    it->second =
        FindUnionRoot(
            it->second,
            parent);

    return it->second;
}

void MultiDroneSystem::UnionLandmarks(
    uint64_t a,
    uint64_t b,
    std::unordered_map<uint64_t, uint64_t>& parent) const
{
    if (a == 0 || b == 0 || a == b)
        return;

    const uint64_t root_a =
        FindUnionRoot(a, parent);

    const uint64_t root_b =
        FindUnionRoot(b, parent);

    if (root_a == root_b)
        return;

    if (root_a < root_b)
        parent[root_b] = root_a;
    else
        parent[root_a] = root_b;
}

bool MultiDroneSystem::MapPointToWorld(
    const ImportedMapPoint& mp,
    Eigen::Vector3d& p_world_out) const
{
    const uint64_t anchor_key =
        MakeCameraKey(
            mp.drone_id,
            mp.map_epoch);

    MapAnchor anchor;

    {
        std::lock_guard<std::mutex> lock(anchors_mutex_);

        auto it =
            map_anchors_.find(anchor_key);

        if (it == map_anchors_.end() || !it->second.valid)
            return false;

        anchor = it->second;
    }

    Eigen::Vector4d p_local;
    p_local << mp.position.x(), mp.position.y(), mp.position.z(), 1.0;

    Eigen::Vector4d p_world =
        anchor.world_T_local * p_local;

    p_world_out =
        p_world.head<3>();

    return true;
}

std::vector<Eigen::Vector3d> MultiDroneSystem::GetAnchoredMapPointsWorld() const
{
    std::vector<Eigen::Vector3d> out;

    if (!atlas_)
        return out;

    const std::vector<ImportedMapPoint> mps =
        atlas_->GetAllMapPoints();

    out.reserve(mps.size());

    for (const auto& mp : mps)
    {
        if (mp.global_id == 0 || mp.is_bad)
            continue;

        Eigen::Vector3d p_world;
        if (!MapPointToWorld(mp, p_world))
            continue;

        if (!std::isfinite(p_world.x()) ||
            !std::isfinite(p_world.y()) ||
            !std::isfinite(p_world.z()))
        {
            continue;
        }

        out.push_back(p_world);
    }

    return out;
}



static double ComputeLandmarkConfirmationScore(
    const uint32_t confirmed_match_count,
    const size_t source_mappoint_count,
    const uint32_t unique_observing_keyframes,
    const uint32_t unique_drones,
    const uint32_t unique_epochs,
    const uint32_t total_observations,
    const double max_group_spread_m)
{
    double score = 0.0;

    score +=
        1.5 *
        std::log1p(
            static_cast<double>(
                confirmed_match_count));

    score +=
        1.0 *
        std::log1p(
            static_cast<double>(
                source_mappoint_count));

    score +=
        0.7 *
        std::log1p(
            static_cast<double>(
                unique_observing_keyframes));

    if (unique_drones > 1)
    {
        score += 1.0;
    }

    if (unique_epochs > 1)
    {
        score += 0.7;
    }

    score +=
        0.4 *
        std::log1p(
            static_cast<double>(
                total_observations));

    score -=
        2.0 *
        max_group_spread_m;

    return score;
}



std::vector<FusedLandmark>
MultiDroneSystem::BuildFusedLandmarksFromLoopEdges() const
{
    return BuildFusedLandmarksFromLoopEdges(nullptr);
}


std::vector<FusedLandmark>
MultiDroneSystem::BuildFusedLandmarksFromLoopEdges(
    FusionDiagnostics* diagnostics) const
{
    FusionDiagnostics local_diag;

    FusionDiagnostics& diag =
        diagnostics ? *diagnostics : local_diag;

    diag =
        FusionDiagnostics();

    const FusionValidationParams fusion_params =
        GetFusionValidationParams();

    std::vector<FusedLandmark> fused;

    if (!atlas_)
        return fused;

    std::vector<ImportedMapPoint> mappoints =
        atlas_->GetAllMapPoints();

    diag.atlas_mappoints_total =
        mappoints.size();

    std::unordered_map<uint64_t, uint64_t> parent;
    std::unordered_map<uint64_t, uint32_t> root_confirmed_match_count;

    // Fase 8:
    // Conteo del tipo de evidencia que soporta cada grupo union-find.
    std::unordered_map<uint64_t, uint32_t> root_inter_drone_match_count;
    std::unordered_map<uint64_t, uint32_t> root_intra_drone_match_count;
    std::unordered_map<uint64_t, uint32_t> root_cross_epoch_same_drone_match_count;
    std::unordered_map<uint64_t, uint32_t> root_direct_match_count;

    parent.reserve(mappoints.size());

    for (const auto& mp : mappoints)
    {
        if (mp.global_id == 0)
            continue;

        if (mp.is_bad)
        {
            diag.atlas_mappoints_bad++;
            continue;
        }

        diag.atlas_mappoints_valid++;

        parent[mp.global_id] =
            mp.global_id;
    }

    std::vector<VerifiedLoopEdge> loops =
        GetLoopEdges();

    diag.stored_loops =
        loops.size();

    size_t loop_index = 0;

    double pair_distance_sum = 0.0;
    size_t pair_distance_count = 0;

    double accepted_pair_distance_sum = 0.0;
    size_t accepted_pair_distance_count = 0;

    double rejected_pair_distance_sum = 0.0;
    size_t rejected_pair_distance_count = 0;

    size_t pair_validation_logs = 0;

    for (const auto& loop : loops)
    {
        loop_index++;

        if (loop.usable_for_optimization)
            diag.loops_usable_for_optimization++;

        const bool is_inter_loop =
            loop.type == LoopEdgeType::INTER_DRONE;

        const bool is_intra_loop =
            !is_inter_loop;

        const bool allow_opt_only_intra =
            fusion_params.allow_intra_opt_only_loops_for_fusion_candidates &&
            is_intra_loop &&
            loop.usable_for_optimization;

        if (!loop.usable_for_fusion && !allow_opt_only_intra)
        {
            diag.loops_not_usable_for_fusion++;
            continue;
        }

        if (!loop.usable_for_fusion && allow_opt_only_intra)
        {
            diag.loops_not_fusion_but_allowed_opt_only_intra++;
            diag.intra_opt_only_loops_used_as_fusion_candidates++;

            std::cerr
                << "[FUSION6-INTRA-OPT-ONLY-AS-FUSION-CANDIDATE]"
                << " q=" << loop.query_kf_id
                << " c=" << loop.candidate_kf_id
                << " q_local=" << loop.query_local_kf_id
                << " c_local=" << loop.candidate_local_kf_id
                << " final_inliers=" << loop.final_inliers
                << " mean_error=" << loop.mean_error_m
                << " max_error=" << loop.max_error_m
                << std::endl;
        }

        if (is_inter_loop)
        {
            diag.inter_loops_seen++;

            if (!fusion_params.enable_inter_drone_fusion)
            {
                diag.loops_skipped_inter_disabled++;
                continue;
            }

            diag.inter_loops_accepted_for_fusion++;
        }

        if (is_intra_loop)
        {
            diag.intra_loops_seen++;

            if (!fusion_params.enable_intra_drone_fusion)
            {
                diag.loops_skipped_intra_disabled++;
                continue;
            }

            const bool same_drone =
                loop.query_drone_id == loop.candidate_drone_id;

            const bool same_epoch =
                loop.query_map_epoch == loop.candidate_map_epoch;

            const bool cross_epoch_same_drone =
                same_drone && !same_epoch;

            if (cross_epoch_same_drone &&
                !fusion_params.enable_cross_epoch_same_drone_fusion)
            {
                diag.intra_loops_rejected_cross_epoch_disabled++;
                continue;
            }

            if (same_drone &&
                same_epoch &&
                fusion_params.reject_intra_fusion_for_nearby_keyframes)
            {
                const uint64_t q_kf =
                    loop.query_local_kf_id;

                const uint64_t c_kf =
                    loop.candidate_local_kf_id;

                const uint64_t gap =
                    (q_kf > c_kf)
                        ? (q_kf - c_kf)
                        : (c_kf - q_kf);

                if (gap < fusion_params.min_intra_drone_kf_id_gap_for_fusion)
                {
                    diag.intra_loops_rejected_nearby_kf++;

                    std::cerr
                        << "[FUSION5D-INTRA-LOOP-REJECT]"
                        << " reason=nearby_keyframes"
                        << " drone_" << loop.query_drone_id
                        << " epoch=" << loop.query_map_epoch
                        << " q_kf=" << q_kf
                        << " c_kf=" << c_kf
                        << " gap=" << gap
                        << " min_gap="
                        << fusion_params.min_intra_drone_kf_id_gap_for_fusion
                        << std::endl;

                    continue;
                }
            }

            diag.intra_loops_accepted_for_fusion++;
        }

        diag.loops_usable_for_fusion++;

        if (is_inter_loop)
            diag.fusion_loops_inter++;
        else
            diag.fusion_loops_intra++;

        std::cerr
            << "[FUSION5C-LOOP] index=" << loop_index
            << " type=" << (loop.type == LoopEdgeType::INTER_DRONE ? "INTER" : "INTRA")
            << " q=drone_" << loop.query_drone_id
            << "/epoch_" << loop.query_map_epoch
            << "/kf_" << loop.query_local_kf_id
            << " c=drone_" << loop.candidate_drone_id
            << "/epoch_" << loop.candidate_map_epoch
            << "/kf_" << loop.candidate_local_kf_id
            << " final=" << loop.final_inliers
            << " matches=" << loop.inlier_matches.size()
            << " ratio=" << loop.inlier_ratio
            << " mean=" << loop.mean_error_m
            << " max=" << loop.max_error_m
            << " quality=" << loop.quality_score
            << " policy=accepted_for_fusion"
            << std::endl;

        for (const auto& match : loop.inlier_matches)
        {
            diag.candidate_pairs++;

            const uint64_t q_mp_id =
                match.query_mappoint_id;

            const uint64_t c_mp_id =
                match.candidate_mappoint_id;

            if (q_mp_id == 0 || c_mp_id == 0)
            {
                diag.pairs_zero_mp++;
                continue;
            }

            if (q_mp_id == c_mp_id)
            {
                diag.pairs_same_mp++;
                continue;
            }

            ImportedMapPoint q_mp =
                atlas_->GetMapPoint(q_mp_id);

            ImportedMapPoint c_mp =
                atlas_->GetMapPoint(c_mp_id);

            if (q_mp.global_id == 0 ||
                c_mp.global_id == 0)
            {
                diag.pairs_missing_mp++;
                continue;
            }

            if (q_mp.is_bad || c_mp.is_bad)
            {
                diag.pairs_bad_mp++;
                continue;
            }

            const bool inter_drone_pair =
                q_mp.drone_id != c_mp.drone_id;

            const bool cross_epoch_same_drone_pair =
                !inter_drone_pair &&
                q_mp.map_epoch != c_mp.map_epoch;

            if (inter_drone_pair)
            {
                diag.pairs_inter_drone++;
            }
            else
            {
                diag.pairs_intra_drone++;

                if (cross_epoch_same_drone_pair)
                    diag.pairs_cross_epoch_same_drone++;
            }

            bool accept_pair_for_union = true;
            double pair_distance_m =
                std::numeric_limits<double>::infinity();

            if (fusion_params.enable_pair_distance_filter)
            {
                Eigen::Vector3d q_world;
                Eigen::Vector3d c_world;

                const bool q_has_world =
                    MapPointToWorld(
                        q_mp,
                        q_world);

                const bool c_has_world =
                    MapPointToWorld(
                        c_mp,
                        c_world);

                if (!q_has_world || !c_has_world ||
                    !std::isfinite(q_world.x()) ||
                    !std::isfinite(q_world.y()) ||
                    !std::isfinite(q_world.z()) ||
                    !std::isfinite(c_world.x()) ||
                    !std::isfinite(c_world.y()) ||
                    !std::isfinite(c_world.z()))
                {
                    diag.pairs_rejected_no_world_position++;
                    accept_pair_for_union = false;
                }
                else
                {
                    pair_distance_m =
                        (q_world - c_world).norm();

                    diag.pairs_checked_3d++;

                    pair_distance_sum +=
                        pair_distance_m;

                    pair_distance_count++;

                    if (pair_distance_m > diag.max_pair_distance_m)
                        diag.max_pair_distance_m = pair_distance_m;

                    double max_allowed_distance =
                        fusion_params.max_intra_drone_pair_distance_m;

                    if (inter_drone_pair)
                    {
                        max_allowed_distance =
                            fusion_params.max_inter_drone_pair_distance_m;
                    }
                    else if (cross_epoch_same_drone_pair)
                    {
                        max_allowed_distance =
                            fusion_params.max_cross_epoch_same_drone_pair_distance_m;
                    }

                    if (pair_distance_m > max_allowed_distance)
                    {
                        accept_pair_for_union = false;

                        diag.pairs_rejected_distance++;

                        if (inter_drone_pair)
                        {
                            diag.pairs_rejected_distance_inter++;
                        }
                        else if (cross_epoch_same_drone_pair)
                        {
                            diag.pairs_rejected_distance_cross_epoch_same_drone++;
                        }
                        else
                        {
                            diag.pairs_rejected_distance_intra++;
                        }

                        rejected_pair_distance_sum +=
                            pair_distance_m;

                        rejected_pair_distance_count++;

                        if (pair_distance_m > diag.max_rejected_pair_distance_m)
                            diag.max_rejected_pair_distance_m = pair_distance_m;

                        if (fusion_params.log_pair_validation_samples &&
                            pair_validation_logs < fusion_params.max_pair_validation_logs)
                        {
                            std::cerr
                                << "[FUSION5D-PAIR-REJECT]"
                                << " reason=distance"
                                << " type=" << (inter_drone_pair ? "INTER" : "INTRA")
                                << " cross_epoch_same_drone="
                                << (cross_epoch_same_drone_pair ? 1 : 0)
                                << " dist=" << pair_distance_m
                                << " max=" << max_allowed_distance
                                << " q_mp=" << q_mp_id
                                << " c_mp=" << c_mp_id
                                << " q=drone_" << q_mp.drone_id
                                << "/epoch_" << q_mp.map_epoch
                                << " c=drone_" << c_mp.drone_id
                                << "/epoch_" << c_mp.map_epoch
                                << " loop_type=" << (is_inter_loop ? "INTER_LOOP" : "INTRA_LOOP")
                                << std::endl;

                            pair_validation_logs++;
                        }
                    }
                }
            }

            if (!accept_pair_for_union)
                continue;

            if (std::isfinite(pair_distance_m))
            {
                diag.pairs_accepted_3d++;

                accepted_pair_distance_sum +=
                    pair_distance_m;

                accepted_pair_distance_count++;

                if (pair_distance_m > diag.max_accepted_pair_distance_m)
                    diag.max_accepted_pair_distance_m = pair_distance_m;

                if (fusion_params.log_pair_validation_samples &&
                    pair_validation_logs < fusion_params.max_pair_validation_logs)
                {
                    std::cerr
                        << "[FUSION5D-PAIR-ACCEPT]"
                        << " type=" << (inter_drone_pair ? "INTER" : "INTRA")
                        << " cross_epoch_same_drone="
                        << (cross_epoch_same_drone_pair ? 1 : 0)
                        << " dist=" << pair_distance_m
                        << " q_mp=" << q_mp_id
                        << " c_mp=" << c_mp_id
                        << " q=drone_" << q_mp.drone_id
                        << "/epoch_" << q_mp.map_epoch
                        << " c=drone_" << c_mp.drone_id
                        << "/epoch_" << c_mp.map_epoch
                        << " loop_type=" << (is_inter_loop ? "INTER_LOOP" : "INTRA_LOOP")
                        << std::endl;

                    pair_validation_logs++;
                }
            }

            const uint64_t q_submap_key =
                MakeSubmapKeyForFusionDiag(
                    q_mp.drone_id,
                    q_mp.map_epoch);

            const uint64_t c_submap_key =
                MakeSubmapKeyForFusionDiag(
                    c_mp.drone_id,
                    c_mp.map_epoch);

            UnionLandmarks(
                q_mp_id,
                c_mp_id,
                parent);

            diag.pairs_used_for_union++;

            {
                const uint64_t root_after =
                    FindUnionRoot(
                        q_mp_id,
                        parent);

                root_confirmed_match_count[root_after]++;

                if (inter_drone_pair)
                {
                    root_inter_drone_match_count[root_after]++;

                    diag.inter_union_pairs_by_submap[q_submap_key]++;
                    diag.inter_union_pairs_by_submap[c_submap_key]++;

                    diag.inter_loop_union_pairs_by_submap[q_submap_key]++;
                    diag.inter_loop_union_pairs_by_submap[c_submap_key]++;
                }
                else if (cross_epoch_same_drone_pair)
                {
                    root_cross_epoch_same_drone_match_count[root_after]++;
                }
                else
                {
                    root_intra_drone_match_count[root_after]++;
                }
            }

            if (inter_drone_pair)
            {
                diag.pairs_used_for_union_inter++;
            }
            else if (cross_epoch_same_drone_pair)
            {
                diag.pairs_used_for_union_cross_epoch_same_drone++;
            }
            else
            {
                diag.pairs_used_for_union_intra++;
            }
        }
    }

    if (pair_distance_count > 0)
    {
        diag.mean_pair_distance_m =
            pair_distance_sum /
            static_cast<double>(pair_distance_count);
    }

    if (accepted_pair_distance_count > 0)
    {
        diag.mean_accepted_pair_distance_m =
            accepted_pair_distance_sum /
            static_cast<double>(accepted_pair_distance_count);
    }

    if (rejected_pair_distance_count > 0)
    {
        diag.mean_rejected_pair_distance_m =
            rejected_pair_distance_sum /
            static_cast<double>(rejected_pair_distance_count);
    }

    std::cerr
        << "[FUSION5D-PAIR-SUMMARY]"
        << " stored_loops=" << diag.stored_loops
        << " opt_loops=" << diag.loops_usable_for_optimization
        << " fusion_loops=" << diag.loops_usable_for_fusion
        << " fusion_inter=" << diag.fusion_loops_inter
        << " fusion_intra=" << diag.fusion_loops_intra
        << " skipped_inter_disabled=" << diag.loops_skipped_inter_disabled
        << " skipped_intra_disabled=" << diag.loops_skipped_intra_disabled
        << " skipped_not_fusion=" << diag.loops_not_usable_for_fusion
        << " opt_only_allowed=" << diag.loops_not_fusion_but_allowed_opt_only_intra
        << " intra_opt_only_used=" << diag.intra_opt_only_loops_used_as_fusion_candidates
        << " candidate_pairs=" << diag.candidate_pairs
        << " used_for_union=" << diag.pairs_used_for_union
        << " zero_mp=" << diag.pairs_zero_mp
        << " same_mp=" << diag.pairs_same_mp
        << " missing_mp=" << diag.pairs_missing_mp
        << " bad_mp=" << diag.pairs_bad_mp
        << " pair_inter_drone=" << diag.pairs_inter_drone
        << " pair_intra_drone=" << diag.pairs_intra_drone
        << " pair_cross_epoch_same_drone=" << diag.pairs_cross_epoch_same_drone
        << " checked_3d=" << diag.pairs_checked_3d
        << " accepted_3d=" << diag.pairs_accepted_3d
        << " reject_no_world=" << diag.pairs_rejected_no_world_position
        << " reject_distance=" << diag.pairs_rejected_distance
        << " reject_distance_inter=" << diag.pairs_rejected_distance_inter
        << " reject_distance_intra=" << diag.pairs_rejected_distance_intra
        << " reject_distance_cross_epoch_same_drone="
        << diag.pairs_rejected_distance_cross_epoch_same_drone
        << " mean_pair_dist=" << diag.mean_pair_distance_m
        << " max_pair_dist=" << diag.max_pair_distance_m
        << " mean_accept_dist=" << diag.mean_accepted_pair_distance_m
        << " max_accept_dist=" << diag.max_accepted_pair_distance_m
        << " mean_reject_dist=" << diag.mean_rejected_pair_distance_m
        << " max_reject_dist=" << diag.max_rejected_pair_distance_m
        << std::endl;

    // ============================================================
    // FASE 5D-B:
    // Asociaciones directas de MapPoints entre KeyFrames del mismo
    // dron. No dependen de loops.
    // ============================================================

    std::vector<DirectLandmarkAssociation> direct_assocs =
        GetDirectLandmarkAssociations();

    diag.direct_associations_stored =
        direct_assocs.size();

    double direct_distance_sum = 0.0;
    size_t direct_distance_count = 0;

    for (const auto& assoc : direct_assocs)
    {

        const std::string reason =
            assoc.reason;

        const bool direct_inter_drone =
            assoc.is_inter_drone ||
            assoc.query_drone_id != assoc.candidate_drone_id;

        const bool direct_cross_epoch_same_drone =
            assoc.is_cross_epoch_same_drone ||
            (
                assoc.query_drone_id == assoc.candidate_drone_id &&
                assoc.query_map_epoch != assoc.candidate_map_epoch
            );

        const bool direct_same_drone_same_epoch =
            !direct_inter_drone &&
            !direct_cross_epoch_same_drone;

        const bool is_consecutive =
            reason.find("consecutive") != std::string::npos;

        const bool is_revisit =
            reason.find("revisit") != std::string::npos;

        if (direct_inter_drone)
        {
            diag.direct_jobs_inter_drone++;
        }
        else if (direct_cross_epoch_same_drone)
        {
            diag.direct_jobs_cross_epoch_same_drone++;
        }
        else
        {
            diag.direct_jobs_same_drone_same_epoch++;
        }

        if (is_consecutive)
        {
            diag.direct_jobs_consecutive++;
        }
        else if (is_revisit)
        {
            diag.direct_jobs_revisit++;
        }
        else
        {
            diag.direct_jobs_other++;
        }

        if (!assoc.valid)
            continue;

        diag.direct_associations_used++;

        diag.direct_candidate_matches +=
            assoc.candidate_matches;

        if (direct_inter_drone)
        {
            diag.direct_candidate_matches_inter_drone +=
                assoc.candidate_matches;
        }
        else if (direct_cross_epoch_same_drone)
        {
            diag.direct_candidate_matches_cross_epoch_same_drone +=
                assoc.candidate_matches;
        }
        else
        {
            diag.direct_candidate_matches_same_drone_same_epoch +=
                assoc.candidate_matches;
        }

        if (is_consecutive)
        {
            diag.direct_candidate_matches_consecutive +=
                assoc.candidate_matches;
        }
        else if (is_revisit)
        {
            diag.direct_candidate_matches_revisit +=
                assoc.candidate_matches;
        }
        else
        {
            diag.direct_candidate_matches_other +=
                assoc.candidate_matches;
        }

        diag.direct_accepted_matches +=
            assoc.accepted_matches;

        if (direct_inter_drone)
        {
            diag.direct_accepted_matches_inter_drone +=
                assoc.accepted_matches;
        }
        else if (direct_cross_epoch_same_drone)
        {
            diag.direct_accepted_matches_cross_epoch_same_drone +=
                assoc.accepted_matches;
        }
        else
        {
            diag.direct_accepted_matches_same_drone_same_epoch +=
                assoc.accepted_matches;
        }

        if (is_consecutive)
        {
            diag.direct_accepted_matches_consecutive +=
                assoc.accepted_matches;
        }
        else if (is_revisit)
        {
            diag.direct_accepted_matches_revisit +=
                assoc.accepted_matches;
        }
        else
        {
            diag.direct_accepted_matches_other +=
                assoc.accepted_matches;
        }

        diag.direct_rejected_zero_mp +=
            assoc.rejected_zero_mp;

        diag.direct_rejected_same_mp +=
            assoc.rejected_same_mp;

        diag.direct_rejected_missing_mp +=
            assoc.rejected_missing_mp;

        diag.direct_rejected_bad_mp +=
            assoc.rejected_bad_mp;

        diag.direct_rejected_distance +=
            assoc.rejected_distance;

        if (is_consecutive)
        {
            diag.direct_rejected_distance_consecutive +=
                assoc.rejected_distance;
        }
        else if (is_revisit)
        {
            diag.direct_rejected_distance_revisit +=
                assoc.rejected_distance;
        }
        else
        {
            diag.direct_rejected_distance_other +=
                assoc.rejected_distance;
        }

        diag.direct_rejected_descriptor +=
            assoc.rejected_descriptor;

        if (is_consecutive)
        {
            diag.direct_rejected_descriptor_consecutive +=
                assoc.rejected_descriptor;
        }
        else if (is_revisit)
        {
            diag.direct_rejected_descriptor_revisit +=
                assoc.rejected_descriptor;
        }
        else
        {
            diag.direct_rejected_descriptor_other +=
                assoc.rejected_descriptor;
        }

        if (assoc.accepted_matches > 0)
        {
            direct_distance_sum +=
                assoc.mean_distance_m *
                static_cast<double>(assoc.accepted_matches);

            direct_distance_count +=
                assoc.accepted_matches;

            diag.direct_max_pair_distance_m =
                std::max(
                    diag.direct_max_pair_distance_m,
                    assoc.max_distance_m);
        }

        for (const auto& match : assoc.matches)
        {
            if (match.query_mappoint_id == 0 ||
                match.candidate_mappoint_id == 0)
                continue;

            if (match.query_mappoint_id == match.candidate_mappoint_id)
                continue;

            if (parent.find(match.query_mappoint_id) == parent.end() ||
                parent.find(match.candidate_mappoint_id) == parent.end())
                continue;

            UnionLandmarks(
                match.query_mappoint_id,
                match.candidate_mappoint_id,
                parent);

            {
                const uint64_t root_after =
                    FindUnionRoot(
                        match.query_mappoint_id,
                        parent);

                root_confirmed_match_count[root_after]++;
                root_direct_match_count[root_after]++;

                if (direct_inter_drone)
                {
                    root_inter_drone_match_count[root_after]++;
                    diag.direct_pairs_used_for_union_inter_drone++;

                    const uint64_t q_submap_key =
                        MakeSubmapKeyForFusionDiag(
                            assoc.query_drone_id,
                            assoc.query_map_epoch);

                    const uint64_t c_submap_key =
                        MakeSubmapKeyForFusionDiag(
                            assoc.candidate_drone_id,
                            assoc.candidate_map_epoch);

                    diag.inter_union_pairs_by_submap[q_submap_key]++;
                    diag.inter_union_pairs_by_submap[c_submap_key]++;

                    diag.inter_direct_union_pairs_by_submap[q_submap_key]++;
                    diag.inter_direct_union_pairs_by_submap[c_submap_key]++;
                }
                else if (direct_cross_epoch_same_drone)
                {
                    root_cross_epoch_same_drone_match_count[root_after]++;
                    diag.direct_pairs_used_for_union_cross_epoch_same_drone++;
                }
                else
                {
                    root_intra_drone_match_count[root_after]++;
                    diag.direct_pairs_used_for_union_same_drone_same_epoch++;
                }
            }
        }
    }

    if (direct_distance_count > 0)
    {
        diag.direct_mean_pair_distance_m =
            direct_distance_sum /
            static_cast<double>(direct_distance_count);
    }

    std::cerr
        << "[DIRECT5D-FUSION-SUMMARY]"
        << " stored=" << diag.direct_associations_stored
        << " used=" << diag.direct_associations_used
        << " candidate_matches=" << diag.direct_candidate_matches
        << " accepted_matches=" << diag.direct_accepted_matches
        << " rejected_zero=" << diag.direct_rejected_zero_mp
        << " rejected_same=" << diag.direct_rejected_same_mp
        << " rejected_missing=" << diag.direct_rejected_missing_mp
        << " rejected_bad=" << diag.direct_rejected_bad_mp
        << " rejected_distance=" << diag.direct_rejected_distance
        << " rejected_descriptor=" << diag.direct_rejected_descriptor
        << " mean_dist=" << diag.direct_mean_pair_distance_m
        << " max_dist=" << diag.direct_max_pair_distance_m
        << " inter_jobs=" << diag.direct_jobs_inter_drone
        << " cross_epoch_jobs=" << diag.direct_jobs_cross_epoch_same_drone
        << " same_epoch_jobs=" << diag.direct_jobs_same_drone_same_epoch
        << " inter_candidate_matches=" << diag.direct_candidate_matches_inter_drone
        << " inter_accepted_matches=" << diag.direct_accepted_matches_inter_drone
        << " inter_union_pairs=" << diag.direct_pairs_used_for_union_inter_drone
        << std::endl;

    std::unordered_map<uint64_t, uint32_t> final_root_confirmed_match_count;

    for (const auto& kv : root_confirmed_match_count)
    {
        const uint64_t old_root = kv.first;
        const uint32_t count = kv.second;

        const uint64_t final_root =
            FindUnionRoot(
                old_root,
                parent);

        final_root_confirmed_match_count[final_root] += count;
    }

    std::unordered_map<uint64_t, uint32_t> final_root_inter_drone_match_count;
    std::unordered_map<uint64_t, uint32_t> final_root_intra_drone_match_count;
    std::unordered_map<uint64_t, uint32_t> final_root_cross_epoch_same_drone_match_count;
    std::unordered_map<uint64_t, uint32_t> final_root_direct_match_count;

    auto compact_root_counts =
        [this, &parent](const std::unordered_map<uint64_t, uint32_t>& input,
                        std::unordered_map<uint64_t, uint32_t>& output)
        {
            for (const auto& kv : input)
            {
                const uint64_t old_root = kv.first;
                const uint32_t count = kv.second;

                const uint64_t final_root =
                    this->FindUnionRoot(
                        old_root,
                        parent);

                output[final_root] += count;
            }
        };

    compact_root_counts(
        root_inter_drone_match_count,
        final_root_inter_drone_match_count);

    compact_root_counts(
        root_intra_drone_match_count,
        final_root_intra_drone_match_count);

    compact_root_counts(
        root_cross_epoch_same_drone_match_count,
        final_root_cross_epoch_same_drone_match_count);

    compact_root_counts(
        root_direct_match_count,
        final_root_direct_match_count);

    std::unordered_map<uint64_t, std::vector<uint64_t>> groups;

    groups.reserve(parent.size());

    for (const auto& [mp_id, root_id_raw] : parent)
    {
        (void)root_id_raw;

        const uint64_t root_id =
            FindUnionRoot(
                mp_id,
                parent);

        groups[root_id].push_back(mp_id);
    }

    diag.groups_total =
        groups.size();

    fused.reserve(groups.size());

    uint64_t next_fused_id = 1;

    double merged_group_size_sum = 0.0;
    size_t merged_group_count_for_mean = 0;

    double spread_sum = 0.0;
    size_t spread_count = 0;

    size_t logged_group_samples = 0;
    const size_t max_group_sample_logs = 12;

    for (const auto& [root_id, ids] : groups)
    {
        (void)root_id;

        if (ids.empty())
            continue;

        diag.max_group_size =
            std::max(
                diag.max_group_size,
                ids.size());

        Eigen::Vector3d weighted_sum =
            Eigen::Vector3d::Zero();

        uint32_t total_observations = 0;
        int valid_points = 0;

        std::vector<Eigen::Vector3d> valid_positions;
        valid_positions.reserve(ids.size());

        std::vector<std::pair<uint64_t, Eigen::Vector3d>> valid_mp_positions;
        valid_mp_positions.reserve(ids.size());

        std::vector<std::pair<uint64_t, uint32_t>> valid_mp_observations;
        valid_mp_observations.reserve(ids.size());

        std::unordered_set<uint64_t> unique_observing_kfs;
        std::unordered_set<uint32_t> unique_drones;
        std::unordered_set<uint64_t> unique_epochs;

        // ============================================================
        // Fase D:
        // Submapas presentes en este grupo union-find.
        //
        // Necesario para saber si un landmark con soporte inter-dron
        // realmente involucra a un submapa concreto:
        //   drone_id / map_epoch
        // ============================================================

        std::unordered_set<uint64_t> unique_submaps_in_group;

        uint32_t total_observations_for_score = 0;

        FusedLandmark landmark;

        landmark.fused_id =
            next_fused_id++;

        landmark.source_mappoint_ids =
            ids;

        for (uint64_t mp_id : ids)
        {
            ImportedMapPoint mp =
                atlas_->GetMapPoint(mp_id);

            if (mp.global_id == 0 || mp.is_bad)
                continue;

            Eigen::Vector3d p_world;

            if (!MapPointToWorld(mp, p_world))
                continue;

            if (!std::isfinite(p_world.x()) ||
                !std::isfinite(p_world.y()) ||
                !std::isfinite(p_world.z()))
            {
                continue;
            }

            const uint32_t obs =
                std::max<uint32_t>(
                    1,
                    mp.observations_count);

            weighted_sum +=
                static_cast<double>(obs) *
                p_world;

            total_observations +=
                obs;

            total_observations_for_score +=
                obs;

            unique_drones.insert(
                mp.drone_id);

            unique_epochs.insert(
                mp.map_epoch);

            const uint64_t submap_key =
                MakeSubmapKeyForFusionDiag(
                    mp.drone_id,
                    mp.map_epoch);

            unique_submaps_in_group.insert(
                submap_key);

            if (!mp.observations.empty())
            {
                for (const auto& observation : mp.observations)
                {
                    if (observation.global_keyframe_id != 0)
                    {
                        unique_observing_kfs.insert(
                            observation.global_keyframe_id);
                    }
                }
            }
            else if (mp.reference_keyframe_id != 0)
            {
                unique_observing_kfs.insert(
                    mp.reference_keyframe_id);
            }

            valid_points++;

            valid_positions.push_back(
                p_world);

            valid_mp_positions.emplace_back(
                mp_id,
                p_world);

            valid_mp_observations.emplace_back(
                mp_id,
                obs);
        }

        if (valid_points == 0 ||
            total_observations == 0)
        {
            diag.groups_invalid_no_valid_points++;
            continue;
        }

        landmark.position_world =
            weighted_sum /
            static_cast<double>(total_observations);

        landmark.observations =
            total_observations;

        landmark.valid =
            true;

        diag.groups_valid++;

        double max_spread_for_group = 0.0;
        double mean_spread_for_group = 0.0;

        for (const auto& p : valid_positions)
        {
            const double spread =
                (p - landmark.position_world).norm();

            if (spread > max_spread_for_group)
                max_spread_for_group = spread;

            mean_spread_for_group +=
                spread;
        }

        if (!valid_positions.empty())
        {
            mean_spread_for_group /=
                static_cast<double>(
                    valid_positions.size());
        }

        landmark.confirmed_match_count =
            final_root_confirmed_match_count[root_id];

        landmark.unique_observing_keyframes =
            static_cast<uint32_t>(
                unique_observing_kfs.size());

        landmark.unique_drones =
            static_cast<uint32_t>(
                unique_drones.size());

        landmark.inter_drone_match_count =
            final_root_inter_drone_match_count[root_id];

        landmark.intra_drone_match_count =
            final_root_intra_drone_match_count[root_id];

        landmark.cross_epoch_same_drone_match_count =
            final_root_cross_epoch_same_drone_match_count[root_id];

        landmark.direct_match_count =
            final_root_direct_match_count[root_id];

        landmark.has_inter_drone_support =
            landmark.unique_drones >= 2 ||
            landmark.inter_drone_match_count > 0;

        landmark.unique_epochs =
            static_cast<uint32_t>(
                unique_epochs.size());

        landmark.total_observations =
            total_observations_for_score;

        landmark.max_group_spread_m =
            max_spread_for_group;

        landmark.mean_group_spread_m =
            mean_spread_for_group;

        landmark.confirmation_score =
            ComputeLandmarkConfirmationScore(
                landmark.confirmed_match_count,
                landmark.source_mappoint_ids.size(),
                landmark.unique_observing_keyframes,
                landmark.unique_drones,
                landmark.unique_epochs,
                landmark.total_observations,
                landmark.max_group_spread_m);

        if (landmark.source_mappoint_ids.size() < 2)
        {
            landmark.confirmed = false;
            landmark.confirmation_reason = "singleton";
        }
        else
        {
            landmark.confirmed = true;
            landmark.confirmation_reason = "candidate";
        }

        const bool is_merged_group =
            ids.size() > 1;

        if (landmark.has_inter_drone_support)
        {
            diag.groups_with_inter_drone_support++;

            if (is_merged_group)
            {
                diag.groups_merged_with_inter_drone_support++;
            }

            // ========================================================
            // Fase D:
            // Contabilidad por submapa.
            //
            // Si este grupo tiene soporte inter-dron, cada submapa que
            // participa en el grupo recibe crédito target-specific.
            //
            // Esto evita confirmar un anchor provisional usando evidencia
            // inter-dron global que pertenece a otro submapa.
            // ========================================================

            for (uint64_t submap_key : unique_submaps_in_group)
            {
                diag.inter_supported_groups_by_submap[submap_key]++;

                if (is_merged_group)
                {
                    diag.inter_merged_groups_by_submap[submap_key]++;
                }
            }
        }

        if (landmark.unique_drones >= 2)
        {
            diag.groups_unique_drones_ge_2++;
        }

        if (landmark.inter_drone_match_count > 0)
        {
            diag.groups_inter_drone_match_count_gt_0++;
        }

        if (is_merged_group &&
            !landmark.has_inter_drone_support)
        {
            diag.groups_merged_single_drone_only++;
        }

        if (!is_merged_group)
        {
            diag.groups_singletons_single_drone++;
        }

        const bool reject_merged_group_by_spread =
            is_merged_group &&
            fusion_params.split_merged_group_if_spread_too_high &&
            max_spread_for_group > fusion_params.max_merged_group_spread_m;

        if (reject_merged_group_by_spread)
        {
            diag.groups_rejected_by_spread++;

            std::cerr
                << "[FUSION5D-GROUP-REJECT-SPREAD]"
                << " fused_candidate_id=" << landmark.fused_id
                << " group_size=" << ids.size()
                << " valid_points=" << valid_points
                << " max_spread_m=" << max_spread_for_group
                << " max_allowed_m=" << fusion_params.max_merged_group_spread_m
                << " action="
                << (fusion_params.publish_rejected_group_as_singletons
                    ? "split_to_singletons"
                    : "drop_group")
                << std::endl;

            if (fusion_params.publish_rejected_group_as_singletons)
            {
                diag.groups_split_to_singletons++;

                for (const auto& [mp_id, p_world] : valid_mp_positions)
                {
                    FusedLandmark singleton;

                    singleton.fused_id =
                        next_fused_id++;

                    singleton.source_mappoint_ids.clear();
                    singleton.source_mappoint_ids.push_back(mp_id);

                    singleton.position_world =
                        p_world;

                    singleton.observations =
                        1;

                    for (const auto& [obs_mp_id, obs] : valid_mp_observations)
                    {
                        if (obs_mp_id == mp_id)
                        {
                            singleton.observations = obs;
                            break;
                        }
                    }

                    singleton.valid =
                        true;

                    fused.push_back(singleton);

                    diag.groups_valid++;
                    diag.groups_singleton++;
                    diag.singleton_points_from_rejected_groups++;
                }
            }

            continue;
        }

        if (ids.size() > 1)
        {
            spread_sum +=
                max_spread_for_group;

            spread_count++;

            diag.max_group_spread_m =
                std::max(
                    diag.max_group_spread_m,
                    max_spread_for_group);

            if (max_spread_for_group > 0.10)
                diag.groups_with_spread_over_010m++;

            if (max_spread_for_group > 0.25)
                diag.groups_with_spread_over_025m++;

            if (max_spread_for_group > 0.50)
                diag.groups_with_spread_over_050m++;

            if (max_spread_for_group > 1.00)
                diag.groups_with_spread_over_100m++;

            if (fusion_params.enable_group_spread_warning &&
                max_spread_for_group > fusion_params.warn_group_spread_m)
            {
                diag.groups_warn_spread++;

                std::cerr
                    << "[FUSION5C-GROUP-SPREAD-WARN]"
                    << " fused_id=" << landmark.fused_id
                    << " group_size=" << ids.size()
                    << " valid_points=" << valid_points
                    << " max_spread_m=" << max_spread_for_group
                    << " warn_m=" << fusion_params.warn_group_spread_m
                    << " observations=" << total_observations
                    << std::endl;
            }

            if (logged_group_samples < max_group_sample_logs)
            {
                std::cerr
                    << "[FUSION5D-GROUP-SAMPLE]"
                    << " fused_id=" << landmark.fused_id
                    << " group_size=" << ids.size()
                    << " valid_points=" << valid_points
                    << " observations=" << total_observations
                    << " max_spread_m=" << max_spread_for_group
                    << " mean_spread_m=" << mean_spread_for_group
                    << " matches=" << landmark.confirmed_match_count
                    << " inter_matches=" << landmark.inter_drone_match_count
                    << " intra_matches=" << landmark.intra_drone_match_count
                    << " direct_matches=" << landmark.direct_match_count
                    << " unique_kfs=" << landmark.unique_observing_keyframes
                    << " drones=" << landmark.unique_drones
                    << " inter_support=" << (landmark.has_inter_drone_support ? 1 : 0)
                    << " epochs=" << landmark.unique_epochs
                    << " score=" << landmark.confirmation_score
                    << " pos=("
                    << landmark.position_world.x() << ","
                    << landmark.position_world.y() << ","
                    << landmark.position_world.z() << ")"
                    << " source_ids=";

                const size_t max_ids_to_print =
                    std::min<size_t>(ids.size(), 8);

                for (size_t i = 0; i < max_ids_to_print; ++i)
                {
                    if (i > 0)
                        std::cerr << ",";

                    std::cerr << ids[i];
                }

                if (ids.size() > max_ids_to_print)
                    std::cerr << ",...";

                std::cerr << std::endl;

                logged_group_samples++;
            }
        }
        if (ids.size() == 1)
        {
            diag.groups_singleton++;
        }
        else
        {
            diag.groups_merged++;

            merged_group_size_sum +=
                static_cast<double>(ids.size());

            merged_group_count_for_mean++;
        }

        fused.push_back(landmark);
    }

    if (merged_group_count_for_mean > 0)
    {
        diag.mean_merged_group_size =
            merged_group_size_sum /
            static_cast<double>(merged_group_count_for_mean);
    }

    if (spread_count > 0)
    {
        diag.mean_group_spread_m =
            spread_sum /
            static_cast<double>(spread_count);
    }

    diag.fused_landmarks_output =
        fused.size();

    std::cerr
        << "[FUSION5D-GROUP-SUMMARY]"
        << " atlas_total=" << diag.atlas_mappoints_total
        << " atlas_valid=" << diag.atlas_mappoints_valid
        << " atlas_bad=" << diag.atlas_mappoints_bad
        << " groups_total=" << diag.groups_total
        << " groups_valid=" << diag.groups_valid
        << " groups_singleton=" << diag.groups_singleton
        << " groups_merged=" << diag.groups_merged
        << " invalid_no_valid_points=" << diag.groups_invalid_no_valid_points
        << " max_group_size=" << diag.max_group_size
        << " mean_merged_group_size=" << diag.mean_merged_group_size
        << " mean_spread_m=" << diag.mean_group_spread_m
        << " max_spread_m=" << diag.max_group_spread_m
        << " spread_gt_010m=" << diag.groups_with_spread_over_010m
        << " spread_gt_025m=" << diag.groups_with_spread_over_025m
        << " spread_gt_050m=" << diag.groups_with_spread_over_050m
        << " spread_gt_100m=" << diag.groups_with_spread_over_100m
        << " fused_output=" << diag.fused_landmarks_output
        << " groups_warn_spread=" << diag.groups_warn_spread
        << " groups_rejected_by_spread=" << diag.groups_rejected_by_spread
        << " groups_split_to_singletons=" << diag.groups_split_to_singletons
        << " singleton_points_from_rejected_groups="
        << diag.singleton_points_from_rejected_groups
        << " inter_supported_groups=" << diag.groups_with_inter_drone_support
        << " inter_supported_merged=" << diag.groups_merged_with_inter_drone_support
        << " single_drone_merged=" << diag.groups_merged_single_drone_only
        << " single_drone_singletons=" << diag.groups_singletons_single_drone
        << " unique_drones_ge_2=" << diag.groups_unique_drones_ge_2
        << " inter_match_gt_0=" << diag.groups_inter_drone_match_count_gt_0
        << " direct_assoc_stored=" << diag.direct_associations_stored
        << " direct_assoc_used=" << diag.direct_associations_used
        << " direct_candidate_matches=" << diag.direct_candidate_matches
        << " direct_accepted_matches=" << diag.direct_accepted_matches
        << " direct_rejected_distance=" << diag.direct_rejected_distance
        << " direct_mean_dist=" << diag.direct_mean_pair_distance_m
        << " direct_max_dist=" << diag.direct_max_pair_distance_m
        << " submaps_inter_supported="
        << diag.inter_supported_groups_by_submap.size()
        << " submaps_inter_union="
        << diag.inter_union_pairs_by_submap.size()
        << std::endl;

        for (const auto& kv : diag.inter_supported_groups_by_submap)
        {
            const uint64_t submap_key =
                kv.first;

            const uint32_t drone_id =
                static_cast<uint32_t>(submap_key >> 32);

            const uint64_t map_epoch =
                submap_key & 0xFFFFFFFFULL;

            const size_t groups =
                kv.second;

            const size_t merged_groups =
                diag.inter_merged_groups_by_submap.count(submap_key)
                    ? diag.inter_merged_groups_by_submap.at(submap_key)
                    : 0;

            const size_t union_pairs =
                diag.inter_union_pairs_by_submap.count(submap_key)
                    ? diag.inter_union_pairs_by_submap.at(submap_key)
                    : 0;

            const size_t direct_pairs =
                diag.inter_direct_union_pairs_by_submap.count(submap_key)
                    ? diag.inter_direct_union_pairs_by_submap.at(submap_key)
                    : 0;

            const size_t loop_pairs =
                diag.inter_loop_union_pairs_by_submap.count(submap_key)
                    ? diag.inter_loop_union_pairs_by_submap.at(submap_key)
                    : 0;

            std::cerr
                << "[FUSION5D-SUBMAP-SUPPORT]"
                << " drone_" << drone_id
                << " epoch=" << map_epoch
                << " inter_groups=" << groups
                << " inter_merged_groups=" << merged_groups
                << " inter_union_pairs=" << union_pairs
                << " direct_union_pairs=" << direct_pairs
                << " loop_union_pairs=" << loop_pairs
                << std::endl;
        }

    return fused;
}


namespace
{

uint64_t MakeSubmapKeyForFilter(
    uint32_t drone_id,
    uint64_t map_epoch)
{
    return
        (static_cast<uint64_t>(drone_id) << 32) |
        (map_epoch & 0xFFFFFFFFULL);
}


PoseGraphSnapshot FilterPoseGraphSnapshotToActiveWindow(
    const PoseGraphSnapshot& full,
    uint64_t local_window)
{
    PoseGraphSnapshot out;

    if (full.vertices.empty())
        return out;

    if (full.loop_edge_count == 0 &&
        full.fiducial_constraint_count == 0)
    {
        return full;
    }

    std::unordered_map<uint64_t, PoseGraphVertex> vertex_by_id;
    vertex_by_id.reserve(full.vertices.size());

    for (const auto& v : full.vertices)
    {
        vertex_by_id[v.global_kf_id] = v;
    }

    // submap_key -> ids locales semilla
    std::unordered_map<uint64_t, std::vector<uint64_t>> seed_local_ids_by_submap;

    // submap_key -> ids locales de KFs con fiducial prior.
    // Esto se usa para incluir el tramo entre fiduciales.
    std::unordered_map<uint64_t, std::vector<uint64_t>> fiducial_local_ids_by_submap;

    // Vértices que tienen prior de fiducial. Estos NUNCA deben perderse
    // en la ventana activa, porque si se pierde el prior del primer fiducial
    // el submapa puede despegarse al optimizar cerca del segundo.
    std::unordered_set<uint64_t> fiducial_vertex_ids;

    auto add_seed =
        [&](uint64_t global_kf_id)
        {
            auto it = vertex_by_id.find(global_kf_id);
            if (it == vertex_by_id.end())
                return;

            const PoseGraphVertex& v = it->second;

            const uint64_t submap_key =
                MakeSubmapKeyForFilter(
                    v.drone_id,
                    v.map_epoch);

            seed_local_ids_by_submap[submap_key].push_back(
                v.local_kf_id);
        };

    // ============================================================
    // Semillas: fiducials.
    //
    // Además de usarlos como seeds, guardamos sus ids para:
    // 1. forzar que estén incluidos,
    // 2. elegir un fixed vertex no arbitrario.
    // ============================================================

    for (const auto& c : full.fiducial_constraints)
    {
        if (!c.valid)
            continue;

        if (c.global_kf_id == 0)
            continue;

        auto vit =
            vertex_by_id.find(c.global_kf_id);

        if (vit == vertex_by_id.end())
            continue;

        const PoseGraphVertex& v =
            vit->second;

        const uint64_t submap_key =
            MakeSubmapKeyForFilter(
                v.drone_id,
                v.map_epoch);

        add_seed(c.global_kf_id);

        fiducial_vertex_ids.insert(
            c.global_kf_id);

        fiducial_local_ids_by_submap[submap_key].push_back(
            v.local_kf_id);
    }

    // ============================================================
    // Semillas: endpoints de loops.
    // ============================================================

    for (const auto& e : full.edges)
    {
        if (e.type != PoseGraphEdgeType::LOOP_INTRA &&
            e.type != PoseGraphEdgeType::LOOP_INTER)
        {
            continue;
        }

        add_seed(e.from_kf_id);
        add_seed(e.to_kf_id);
    }
    
    if (seed_local_ids_by_submap.empty())
        return full;

    std::unordered_set<uint64_t> included_vertices;

    // ============================================================
    // Expandir ventana local alrededor de todas las semillas.
    // ============================================================

    for (const auto& v : full.vertices)
    {
        const uint64_t submap_key =
            MakeSubmapKeyForFilter(
                v.drone_id,
                v.map_epoch);

        auto seed_it =
            seed_local_ids_by_submap.find(submap_key);

        if (seed_it == seed_local_ids_by_submap.end())
            continue;

        bool include = false;

        for (uint64_t seed_local_id : seed_it->second)
        {
            const uint64_t diff =
                v.local_kf_id > seed_local_id
                    ? v.local_kf_id - seed_local_id
                    : seed_local_id - v.local_kf_id;

            if (diff <= local_window)
            {
                include = true;
                break;
            }
        }

        if (include)
        {
            included_vertices.insert(v.global_kf_id);
        }
    }

    // ============================================================
    // Si un submapa tiene dos o más fiducials, incluimos el tramo
    // completo entre el primer y el último KF con prior de fiducial.
    //
    // Esto evita que la ventana activa cree dos islas desconectadas:
    // una cerca del primer fiducial y otra cerca del segundo.
    // ============================================================

    size_t bridge_vertices_added = 0;

    for (auto& [submap_key, fid_local_ids] : fiducial_local_ids_by_submap)
    {
        if (fid_local_ids.size() < 2)
            continue;

        std::sort(
            fid_local_ids.begin(),
            fid_local_ids.end());

        const uint64_t min_fid_local =
            fid_local_ids.front();

        const uint64_t max_fid_local =
            fid_local_ids.back();

        const uint64_t margin =
            std::min<uint64_t>(
                local_window,
                25);

        const uint64_t min_include =
            min_fid_local > margin
                ? min_fid_local - margin
                : 0;

        const uint64_t max_include =
            max_fid_local + margin;

        for (const auto& v : full.vertices)
        {
            const uint64_t v_submap_key =
                MakeSubmapKeyForFilter(
                    v.drone_id,
                    v.map_epoch);

            if (v_submap_key != submap_key)
                continue;

            if (v.local_kf_id < min_include ||
                v.local_kf_id > max_include)
            {
                continue;
            }

            const auto [_, inserted] =
                included_vertices.insert(
                    v.global_kf_id);

            if (inserted)
                bridge_vertices_added++;
        }
    }

    // ============================================================
    // Refuerzo explícito:
    // aunque la lógica de semillas ya debería incluirlos, insertamos
    // de nuevo todos los KFs con priors de fiducial.
    // ============================================================

    for (uint64_t fid_kf_id : fiducial_vertex_ids)
    {
        included_vertices.insert(fid_kf_id);
    }

    // Si la ventana se queda demasiado pequeña, no filtramos.
    if (included_vertices.size() < 4)
        return full;

    out.vertices.reserve(included_vertices.size());

    for (const auto& v : full.vertices)
    {
        if (included_vertices.find(v.global_kf_id) ==
            included_vertices.end())
        {
            continue;
        }

        PoseGraphVertex v_out = v;
        v_out.fixed = false;

        out.vertices.push_back(v_out);
    }

    std::unordered_set<uint64_t> out_vertex_ids;
    out_vertex_ids.reserve(out.vertices.size());

    for (const auto& v : out.vertices)
    {
        out_vertex_ids.insert(v.global_kf_id);
    }

    // ============================================================
    // Reasignar fixed vertex.
    //
    // No fijamos out.vertices.front(), porque el orden puede hacer que
    // quede fijado un KF cerca del segundo fiducial.
    // Preferimos fijar un KF con prior de fiducial y, dentro de esos,
    // el más antiguo posible.
    // ============================================================

    out.fixed_vertex_count = 0;

    uint64_t fixed_candidate = 0;

    uint32_t best_drone_id =
        std::numeric_limits<uint32_t>::max();

    uint64_t best_map_epoch =
        std::numeric_limits<uint64_t>::max();

    uint64_t best_local_id =
        std::numeric_limits<uint64_t>::max();

    for (const auto& v : out.vertices)
    {
        if (fiducial_vertex_ids.find(v.global_kf_id) ==
            fiducial_vertex_ids.end())
        {
            continue;
        }

        const uint64_t local_id =
            v.local_kf_id;

        const bool better =
            fixed_candidate == 0 ||
            v.drone_id < best_drone_id ||
            (v.drone_id == best_drone_id &&
            v.map_epoch < best_map_epoch) ||
            (v.drone_id == best_drone_id &&
            v.map_epoch == best_map_epoch &&
            local_id < best_local_id);

        if (better)
        {
            fixed_candidate =
                v.global_kf_id;

            best_drone_id =
                v.drone_id;

            best_map_epoch =
                v.map_epoch;

            best_local_id =
                local_id;
        }
    }

    // Fallback: si por alguna razón no hay ningún fiducial vertex en out,
    // fijamos el menor global id para que sea determinista.
    if (fixed_candidate == 0)
    {
        for (const auto& v : out.vertices)
        {
            if (fixed_candidate == 0 ||
                v.global_kf_id < fixed_candidate)
            {
                fixed_candidate =
                    v.global_kf_id;
            }
        }
    }

    for (auto& v : out.vertices)
    {
        v.fixed =
            v.global_kf_id == fixed_candidate;

        if (v.fixed)
        {
            out.fixed_vertex_count++;
        }
    }

    // Seguridad: debe haber exactamente un fixed vertex.
    if (out.fixed_vertex_count == 0 &&
        !out.vertices.empty())
    {
        out.vertices.front().fixed = true;
        out.fixed_vertex_count = 1;
        fixed_candidate = out.vertices.front().global_kf_id;
    }

    // ============================================================
    // Copiar edges cuyos dos extremos estén dentro de la ventana.
    //
    // Importante:
    // No eliminamos LOOP_INTER aquí. Si el dron 2 está conectado al
    // submapa del dron 1 por un merge/loop, esos edges ayudan a mantener
    // la consistencia global durante la optimización.
    // ============================================================
    
    for (const auto& e : full.edges)
    {
        if (out_vertex_ids.find(e.from_kf_id) == out_vertex_ids.end() ||
            out_vertex_ids.find(e.to_kf_id) == out_vertex_ids.end())
        {
            continue;
        }
        out.edges.push_back(e);

        if (e.type == PoseGraphEdgeType::LOCAL_PARENT ||
            e.type == PoseGraphEdgeType::LOCAL_COVISIBILITY)
        {
            out.local_edge_count++;
        }
        else
        {
            out.loop_edge_count++;
        }
    }

    // ============================================================
    // Copiar fiducial priors cuyo KF esté en la ventana.
    //
    // Con el refuerzo anterior, todos los priors válidos cuyo KF exista
    // deberían entrar.
    // ============================================================

    size_t active_fid_rejected_no_vertex = 0;

    for (const auto& c : full.fiducial_constraints)
    {
        if (!c.valid)
            continue;

        if (out_vertex_ids.find(c.global_kf_id) ==
            out_vertex_ids.end())
        {
            active_fid_rejected_no_vertex++;
            continue;
        }

        out.fiducial_constraints.push_back(c);
    }

    out.fiducial_constraint_count =
        out.fiducial_constraints.size();

    // Si no hay fiduciales y al filtrar hemos perdido todos los loops,
    // entonces el filtro no aporta nada y devolvemos el grafo completo.
    //
    // Pero si hay fiducial constraints, NO debemos volver al full graph,
    // porque eso reintroduciría loops inter-drone durante el anclaje fiducial.
    if (full.loop_edge_count > 0 &&
        out.loop_edge_count == 0)
    {
        return full;
    }

    // Si el full graph tenía fiducials pero la ventana activa perdió todos,
    // tampoco sirve.
    if (full.fiducial_constraint_count > 0 &&
        out.fiducial_constraint_count == 0)
    {
        return full;
    }

    std::cerr
        << "[GRAPH8-ACTIVE] full_vertices=" << full.vertices.size()
        << " full_edges=" << full.edges.size()
        << " full_local=" << full.local_edge_count
        << " full_loop=" << full.loop_edge_count
        << " full_fid=" << full.fiducial_constraint_count
        << " -> active_vertices=" << out.vertices.size()
        << " active_edges=" << out.edges.size()
        << " active_local=" << out.local_edge_count
        << " active_loop=" << out.loop_edge_count
        << " active_fid=" << out.fiducial_constraint_count
        << " fid_vertices=" << fiducial_vertex_ids.size()
        << " fid_rejected_no_vertex=" << active_fid_rejected_no_vertex
        << " bridge_vertices=" << bridge_vertices_added
        << " fixed=" << fixed_candidate
        << " fixed_count=" << out.fixed_vertex_count
        << " window=" << local_window
        << std::endl;

    return out;
}

}  // namespace


GlobalPoseGraphOptimizationResult MultiDroneSystem::OptimizeEssentialGraph(
    const GlobalPoseGraphOptimizationParams& params,
    int min_covisibility_weight,
    int max_covisibility_edges_per_keyframe) const
{
    PoseGraphSnapshot graph_full =
        BuildPoseGraphSnapshot(
            min_covisibility_weight,
            max_covisibility_edges_per_keyframe);

    PoseGraphSnapshot graph =
        graph_full;

    // Si el grafo es grande, optimizamos solo una ventana activa alrededor
    // de fiducials y loop edges. Esto evita bloquear el nodo cuando hay
    // submapas preservados o muchos KFs antiguos.
    if (params.use_active_window_filter &&
        params.active_window_local_kfs > 0 &&
        graph_full.vertices.size() > 100)
    {
        graph =
            FilterPoseGraphSnapshotToActiveWindow(
                graph_full,
                params.active_window_local_kfs);
    }

    std::cerr
        << "[OPT9-GRAPH-IN] vertices=" << graph.vertices.size()
        << " edges=" << graph.edges.size()
        << " local=" << graph.local_edge_count
        << " loop=" << graph.loop_edge_count
        << " fiducial_priors=" << graph.fiducial_constraint_count
        << " fixed=" << graph.fixed_vertex_count
        << " active_window_enabled=" << (params.use_active_window_filter ? 1 : 0)
        << " active_window_local_kfs=" << params.active_window_local_kfs
        << std::endl;

    GlobalPoseGraphOptimizer optimizer;

    return optimizer.Optimize(
        graph,
        params);
}


void MultiDroneSystem::DebugPrintPoseGraphValidation(
    int min_covisibility_weight,
    int max_covisibility_edges_per_keyframe) const
{
    PoseGraphSnapshot graph =
        BuildPoseGraphSnapshot(
            min_covisibility_weight,
            max_covisibility_edges_per_keyframe);

    std::cerr
        << "[GRAPH-VALIDATE] vertices=" << graph.vertices.size()
        << " edges=" << graph.edges.size()
        << " local=" << graph.local_edge_count
        << " loop=" << graph.loop_edge_count
        << " fixed=" << graph.fixed_vertex_count
        << std::endl;

    int valid = 0;
    int invalid = 0;

    std::unordered_map<std::string, int> reasons;

    auto check =
        [&reasons](const Eigen::Matrix4d& T) -> bool
        {
            for (int r = 0; r < 4; ++r)
            {
                for (int c = 0; c < 4; ++c)
                {
                    if (!std::isfinite(T(r, c)))
                    {
                        reasons["non_finite"]++;
                        return false;
                    }
                }
            }

            const Eigen::Matrix3d R =
                T.block<3, 3>(0, 0);

            const Eigen::Vector3d t =
                T.block<3, 1>(0, 3);

            if (t.norm() > 200.0)
            {
                reasons["translation_too_large"]++;
                return false;
            }

            const double det =
                R.determinant();

            if (std::abs(det - 1.0) > 0.20)
            {
                reasons["bad_det"]++;
                return false;
            }

            const double ortho =
                (R.transpose() * R -
                 Eigen::Matrix3d::Identity()).norm();

            if (ortho > 0.30)
            {
                reasons["bad_orthogonality"]++;
                return false;
            }

            return true;
        };

    int printed = 0;

    for (const auto& v : graph.vertices)
    {
        if (check(v.world_T_camera_initial))
        {
            valid++;
        }
        else
        {
            invalid++;

            if (printed < 5)
            {
                const Eigen::Vector3d t =
                    v.world_T_camera_initial.block<3,1>(0,3);

                const Eigen::Matrix3d R =
                    v.world_T_camera_initial.block<3,3>(0,0);

                std::cerr
                    << "[GRAPH-VALIDATE] invalid example global="
                    << v.global_kf_id
                    << " drone=" << v.drone_id
                    << " local=" << v.local_kf_id
                    << " t=(" << t.x() << "," << t.y() << "," << t.z() << ")"
                    << " t_norm=" << t.norm()
                    << " det=" << R.determinant()
                    << " ortho="
                    << (R.transpose() * R -
                        Eigen::Matrix3d::Identity()).norm()
                    << std::endl;

                printed++;
            }
        }
    }

    std::cerr
        << "[GRAPH-VALIDATE] valid_vertices=" << valid
        << " invalid_vertices=" << invalid
        << std::endl;

    for (const auto& [reason, count] : reasons)
    {
        std::cerr
            << "[GRAPH-VALIDATE] reason=" << reason
            << " count=" << count
            << std::endl;
    }
}

uint64_t MultiDroneSystem::GetLatestKeyFrameId(
    uint32_t drone_id,
    uint64_t map_epoch) const
{
    if (!atlas_)
        return 0;

    const std::vector<ImportedKeyFrame> keyframes =
        atlas_->GetAllKeyFrames();

    uint64_t best_global_id = 0;
    double best_stamp = -1.0;
    uint64_t best_local_id = 0;

    for (const auto& kf : keyframes)
    {
        if (kf.global_id == 0 || kf.is_bad)
            continue;

        if (kf.drone_id != drone_id ||
            kf.map_epoch != map_epoch)
        {
            continue;
        }

        const bool newer_by_stamp =
            kf.stamp > best_stamp;

        const bool newer_by_local =
            std::abs(kf.stamp - best_stamp) < 1e-9 &&
            kf.local_id > best_local_id;

        if (newer_by_stamp || newer_by_local)
        {
            best_stamp = kf.stamp;
            best_local_id = kf.local_id;
            best_global_id = kf.global_id;
        }
    }

    return best_global_id;
}


bool MultiDroneSystem::AddFiducialPoseConstraint(
    const FiducialPoseConstraint& constraint)
{
    if (!constraint.valid)
        return false;

    if (constraint.global_kf_id == 0)
        return false;

    if (!constraint.measured_world_T_camera.allFinite())
        return false;

    ImportedKeyFrame kf =
        atlas_->GetKeyFrame(
            constraint.global_kf_id);

    if (kf.global_id == 0 || kf.is_bad)
        return false;

    if (kf.drone_id != constraint.drone_id ||
        kf.map_epoch != constraint.map_epoch)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(
        fiducial_constraints_mutex_);

    // Si ya existe una constraint para el mismo KF y fiducial,
    // la actualizamos si la nueva tiene igual o mayor peso.
    //
    // Esto es importante porque una ventana de fiducial puede abrirse
    // cuando el dron aún está en el borde del radio. Más tarde puede llegar
    // una observación mejor del mismo KF/fiducial.
    for (auto& existing : fiducial_constraints_)
    {
        if (existing.global_kf_id == constraint.global_kf_id &&
            existing.fiducial_id == constraint.fiducial_id)
        {
            if (constraint.weight >= existing.weight)
            {
                existing =
                    constraint;

                std::cerr
                    << "[FID-PRIOR] updated constraint drone_"
                    << constraint.drone_id
                    << " epoch=" << constraint.map_epoch
                    << " kf=" << constraint.local_kf_id
                    << " fiducial=" << constraint.fiducial_id
                    << " weight=" << constraint.weight
                    << std::endl;

                return true;
            }

            std::cerr
                << "[FID-PRIOR] duplicate weaker ignored drone_"
                << constraint.drone_id
                << " epoch=" << constraint.map_epoch
                << " kf=" << constraint.local_kf_id
                << " fiducial=" << constraint.fiducial_id
                << " old_weight=" << existing.weight
                << " new_weight=" << constraint.weight
                << std::endl;

            return false;
        }
    }

    fiducial_constraints_.push_back(
        constraint);

    std::cerr
        << "[FID-PRIOR] stored constraint drone_"
        << constraint.drone_id
        << " epoch=" << constraint.map_epoch
        << " kf=" << constraint.local_kf_id
        << " fiducial=" << constraint.fiducial_id
        << " weight=" << constraint.weight
        << std::endl;

    return true;
}


bool MultiDroneSystem::RemoveFiducialPoseConstraint(
    uint32_t drone_id,
    uint64_t map_epoch,
    uint64_t global_kf_id,
    uint32_t fiducial_id)
{
    std::lock_guard<std::mutex> lock(
        fiducial_constraints_mutex_);

    const size_t before =
        fiducial_constraints_.size();

    fiducial_constraints_.erase(
        std::remove_if(
            fiducial_constraints_.begin(),
            fiducial_constraints_.end(),
            [&](const FiducialPoseConstraint& c)
            {
                return
                    c.drone_id == drone_id &&
                    c.map_epoch == map_epoch &&
                    c.global_kf_id == global_kf_id &&
                    c.fiducial_id == fiducial_id;
            }),
        fiducial_constraints_.end());

    const size_t after =
        fiducial_constraints_.size();

    if (after < before)
    {
        std::cerr
            << "[FID-PRIOR] removed constraint drone_"
            << drone_id
            << " epoch=" << map_epoch
            << " global_kf=" << global_kf_id
            << " fiducial=" << fiducial_id
            << " removed=" << (before - after)
            << std::endl;

        return true;
    }

    return false;
}


size_t MultiDroneSystem::GetFiducialConstraintCount() const
{
    std::lock_guard<std::mutex> lock(
        fiducial_constraints_mutex_);

    return fiducial_constraints_.size();
}


void MultiDroneSystem::SetPoseGraphInitialGuesses(
    const std::unordered_map<uint64_t, Eigen::Matrix4d>& guesses)
{
    std::lock_guard<std::mutex> lock(
        pose_graph_initial_guess_mutex_);

    pose_graph_initial_guesses_ = guesses;

    std::cerr
        << "[MULTI-OPT-GUESS] stored initial guesses="
        << pose_graph_initial_guesses_.size()
        << std::endl;
}


void MultiDroneSystem::ClearPoseGraphInitialGuesses()
{
    std::lock_guard<std::mutex> lock(
        pose_graph_initial_guess_mutex_);

    pose_graph_initial_guesses_.clear();

    std::cerr
        << "[MULTI-OPT-GUESS] cleared initial guesses"
        << std::endl;
}


}  // namespace orbslam3_multi
