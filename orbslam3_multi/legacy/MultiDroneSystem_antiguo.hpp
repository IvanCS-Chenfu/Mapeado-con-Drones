#pragma once

#include "orbslam3_multi/legacy/GlobalAtlas_antiguo.hpp"
#include "orbslam3_multi/legacy/GlobalKeyFrameDatabase_antiguo.hpp"
#include "orbslam3_multi/legacy/GlobalORBMatcher_antiguo.hpp"
#include "orbslam3_multi/legacy/GlobalGeometryVerifier_antiguo.hpp"
#include "orbslam3_multi/legacy/GlobalPoseGraphTypes_antiguo.hpp"

#include <memory>
#include <vector>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <mutex>
#include <algorithm>
#include <Eigen/Dense>
#include <string>
#include <limits>

namespace orbslam3_multi
{

// ============================================================
// Punto simple para publicar/debuggear la nube sparse raw.
// ============================================================

struct RawPoint
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    uint32_t drone_id = 0;
    uint64_t map_epoch = 0;
    uint64_t global_mappoint_id = 0;
};



enum class LoopEdgeType
{
    INTRA_DRONE = 0,
    INTER_DRONE = 1
};

enum class AnchorSource
{
    UNKNOWN = 0,
    FIDUCIAL_DIRECT = 1,
    LOOP_OR_PROPAGATED = 2
};

enum class AnchorMaturity
{
    UNKNOWN = 0,
    PROVISIONAL = 1,
    CONFIRMED = 2
};

struct VerifiedLoopEdge
{
    uint64_t query_kf_id = 0;
    uint64_t candidate_kf_id = 0;

    uint32_t query_drone_id = 0;
    uint32_t candidate_drone_id = 0;

    uint64_t query_map_epoch = 0;
    uint64_t candidate_map_epoch = 0;

    uint64_t query_local_kf_id = 0;
    uint64_t candidate_local_kf_id = 0;

    LoopEdgeType type = LoopEdgeType::INTRA_DRONE;

    // Transforma puntos del mapa local query al mapa local candidate.
    Eigen::Matrix4d candidate_T_query = Eigen::Matrix4d::Identity();

    int initial_matches = 0;
    int ransac_inliers = 0;
    int projection_matches = 0;
    int final_inliers = 0;

    double inlier_ratio = 0.0;
    double mean_error_m = 0.0;
    double max_error_m = 0.0;

    std::vector<FeatureMatch> inlier_matches;

    bool usable_for_fusion = false;
    bool usable_for_optimization = false;

    // ============================================================
    // Fase E:
    // Un loop puede ser válido para pose graph o incluso para fusión,
    // pero no necesariamente seguro para crear un anchor provisional.
    //
    // Ejemplo:
    // dos drones con vista casi idéntica pero distinta altura.
    // ============================================================

    bool usable_for_anchor = true;
    std::string anchor_reject_reason;

    double quality_score = 0.0;
};


struct DirectLandmarkAssociation
{
    uint64_t query_kf_id = 0;
    uint64_t candidate_kf_id = 0;

    uint32_t drone_id = 0;
    uint64_t map_epoch = 0;

    uint64_t query_local_kf_id = 0;
    uint64_t candidate_local_kf_id = 0;

    // ============================================================
    // Fase C:
    // Identidad explícita de ambos lados de la asociación.
    // Necesaria para soportar asociaciones inter-dron y cross-epoch.
    // ============================================================

    uint32_t query_drone_id = 0;
    uint64_t query_map_epoch = 0;
    uint64_t query_local_keyframe_id = 0;

    uint32_t candidate_drone_id = 0;
    uint64_t candidate_map_epoch = 0;
    uint64_t candidate_local_keyframe_id = 0;

    bool is_inter_drone = false;
    bool is_cross_epoch_same_drone = false;
    bool is_same_drone_same_epoch = true;

    std::string reason;

    std::vector<FeatureMatch> matches;

    size_t candidate_matches = 0;
    size_t accepted_matches = 0;
    size_t rejected_zero_mp = 0;
    size_t rejected_same_mp = 0;
    size_t rejected_missing_mp = 0;
    size_t rejected_bad_mp = 0;
    size_t rejected_distance = 0;
    size_t rejected_descriptor = 0;

    double mean_distance_m = 0.0;
    double max_distance_m = 0.0;

    bool valid = false;
};

// ============================================================
// Paso 8: estructuras del pose graph global.
//
// Convención:
// - world_T_camera: transforma puntos de cámara a world.
// - a_T_b: transformación relativa entre cámaras:
//          a_T_b = inverse(world_T_a) * world_T_b
// ============================================================

struct MapAnchor
{
    uint32_t drone_id = 0;
    uint64_t map_epoch = 0;

    Eigen::Matrix4d world_T_local =
        Eigen::Matrix4d::Identity();

    bool valid = false;

    // Fase 8B:
    // Origen y madurez del anchor.
    AnchorSource source =
        AnchorSource::UNKNOWN;

    AnchorMaturity maturity =
        AnchorMaturity::UNKNOWN;
};

struct FiducialPoseConstraint
{
    uint32_t drone_id = 0;
    uint64_t map_epoch = 0;

    uint64_t global_kf_id = 0;
    uint64_t local_kf_id = 0;

    uint32_t fiducial_id = 0;

    Eigen::Matrix4d measured_world_T_camera =
        Eigen::Matrix4d::Identity();

    double weight = 100.0;

    bool valid = false;
};

enum class PoseGraphEdgeType
{
    LOCAL_COVISIBILITY = 0,
    LOCAL_PARENT = 1,
    LOOP_INTRA = 2,
    LOOP_INTER = 3
};

struct PoseGraphVertex
{
    uint64_t global_kf_id = 0;

    uint32_t drone_id = 0;
    uint64_t map_epoch = 0;
    uint64_t local_kf_id = 0;

    Eigen::Matrix4d local_T_camera =
        Eigen::Matrix4d::Identity();

    Eigen::Matrix4d world_T_camera_initial =
        Eigen::Matrix4d::Identity();

    Eigen::Matrix4d world_T_camera_optimized =
        Eigen::Matrix4d::Identity();

    bool fixed = false;

    // Fase 8B/8C:
    // El pose graph necesita saber si el submapa viene de fiducial,
    // loop provisional o loop confirmado.
    AnchorSource anchor_source =
        AnchorSource::UNKNOWN;

    AnchorMaturity anchor_maturity =
        AnchorMaturity::UNKNOWN;
};

struct PoseGraphEdge
{
    uint64_t from_kf_id = 0;
    uint64_t to_kf_id = 0;

    PoseGraphEdgeType type =
        PoseGraphEdgeType::LOCAL_COVISIBILITY;

    // Relative pose:
    // from_T_to = inverse(world_T_from) * world_T_to
    Eigen::Matrix4d from_T_to =
        Eigen::Matrix4d::Identity();

    double weight = 1.0;

    int inliers = 0;
    double mean_error_m = 0.0;
};

struct PoseGraphSnapshot
{
    std::vector<PoseGraphVertex> vertices;
    std::vector<PoseGraphEdge> edges;

    std::vector<FiducialPoseConstraint> fiducial_constraints;

    size_t local_edge_count = 0;
    size_t loop_edge_count = 0;
    size_t fiducial_constraint_count = 0;
    size_t fixed_vertex_count = 0;
};

struct FusedLandmark
{
    uint64_t fused_id = 0;
    Eigen::Vector3d position_world = Eigen::Vector3d::Zero();

    std::vector<uint64_t> source_mappoint_ids;

    uint32_t observations = 0;
    bool valid = false;

    // Fase 3: métricas de confirmación.
    uint32_t confirmed_match_count = 0;
    uint32_t unique_observing_keyframes = 0;
    uint32_t unique_drones = 0;
    uint32_t unique_epochs = 0;
    uint32_t total_observations = 0;

    double max_group_spread_m = 0.0;
    double mean_group_spread_m = 0.0;

    double confirmation_score = 0.0;
    bool confirmed = false;
    std::string confirmation_reason;

    // ============================================================
    // Fase 8:
    // Soporte explícito de fusión inter-dron.
    // ============================================================

    // Número de matches aceptados que venían de pares MapPoint-MapPoint
    // entre drones distintos.
    uint32_t inter_drone_match_count = 0;

    // Número de matches aceptados que venían de pares del mismo dron.
    uint32_t intra_drone_match_count = 0;

    // Número de matches aceptados que venían de cross-epoch del mismo dron.
    uint32_t cross_epoch_same_drone_match_count = 0;

    // Número de matches aceptados desde asociaciones directas DIRECT5D.
    uint32_t direct_match_count = 0;

    // true si el landmark tiene soporte multi-dron real.
    // Se activa si:
    //   - contiene MapPoints de más de un dron, o
    //   - tiene matches inter-dron aceptados.
    bool has_inter_drone_support = false;
};

struct LoopEdgePairKey
{
    uint64_t a = 0;
    uint64_t b = 0;

    bool operator==(const LoopEdgePairKey& other) const
    {
        return a == other.a && b == other.b;
    }
};

struct LoopEdgePairKeyHash
{
    std::size_t operator()(const LoopEdgePairKey& key) const
    {
        std::size_t seed = 0;

        seed ^=
            std::hash<uint64_t>{}(key.a) +
            0x9e3779b97f4a7c15ULL +
            (seed << 6) +
            (seed >> 2);

        seed ^=
            std::hash<uint64_t>{}(key.b) +
            0x9e3779b97f4a7c15ULL +
            (seed << 6) +
            (seed >> 2);

        return seed;
    }
};


struct FusionDiagnostics
{
    // ----------------------------
    // Atlas
    // ----------------------------
    size_t atlas_mappoints_total = 0;
    size_t atlas_mappoints_valid = 0;
    size_t atlas_mappoints_bad = 0;

    // ----------------------------
    // Loops
    // ----------------------------
    size_t stored_loops = 0;

    size_t loops_usable_for_optimization = 0;
    size_t loops_usable_for_fusion = 0;
    size_t loops_not_usable_for_fusion = 0;

    size_t fusion_loops_inter = 0;
    size_t fusion_loops_intra = 0;

    // Fase 6E:
    // Loops intra que no eran usable_for_fusion, pero se han permitido
    // como candidatos experimentales porque usable_for_optimization=true.
    size_t intra_opt_only_loops_used_as_fusion_candidates = 0;
    size_t loops_not_fusion_but_allowed_opt_only_intra = 0;

    // FASE 5C:
    size_t loops_skipped_inter_disabled = 0;
    size_t loops_skipped_intra_disabled = 0;

    // FASE 5D:
    size_t intra_loops_seen = 0;
    size_t intra_loops_accepted_for_fusion = 0;
    size_t intra_loops_rejected_nearby_kf = 0;
    size_t intra_loops_rejected_cross_epoch_disabled = 0;

    size_t inter_loops_seen = 0;
    size_t inter_loops_accepted_for_fusion = 0;

    // ----------------------------
    // Pares candidatos de MapPoints
    // ----------------------------
    size_t candidate_pairs = 0;
    size_t pairs_zero_mp = 0;
    size_t pairs_same_mp = 0;
    size_t pairs_missing_mp = 0;
    size_t pairs_bad_mp = 0;
    size_t pairs_used_for_union = 0;

    // FASE 5B:
    // Validación 3D de pares antes de fusionar.
    size_t pairs_checked_3d = 0;
    size_t pairs_accepted_3d = 0;

    size_t pairs_rejected_no_world_position = 0;
    size_t pairs_rejected_distance = 0;
    size_t pairs_rejected_distance_inter = 0;
    size_t pairs_rejected_distance_intra = 0;
    size_t pairs_rejected_distance_cross_epoch_same_drone = 0;

    double mean_pair_distance_m = 0.0;
    double max_pair_distance_m = 0.0;

    double mean_accepted_pair_distance_m = 0.0;
    double max_accepted_pair_distance_m = 0.0;

    double mean_rejected_pair_distance_m = 0.0;
    double max_rejected_pair_distance_m = 0.0;

    size_t pairs_inter_drone = 0;
    size_t pairs_intra_drone = 0;
    size_t pairs_cross_epoch_same_drone = 0;

    // FASE 5D:
    size_t pairs_used_for_union_inter = 0;
    size_t pairs_used_for_union_intra = 0;
    size_t pairs_used_for_union_cross_epoch_same_drone = 0;

    size_t pairs_rejected_nearby_intra_kf = 0;

    // ----------------------------
    // Grupos
    // ----------------------------
    size_t groups_total = 0;
    size_t groups_valid = 0;
    size_t groups_singleton = 0;
    size_t groups_merged = 0;

    // ============================================================
    // Fase 8:
    // Diagnóstico de grupos con soporte inter-dron.
    // ============================================================

    size_t groups_with_inter_drone_support = 0;
    size_t groups_merged_with_inter_drone_support = 0;
    size_t groups_merged_single_drone_only = 0;
    size_t groups_singletons_single_drone = 0;

    size_t groups_unique_drones_ge_2 = 0;
    size_t groups_inter_drone_match_count_gt_0 = 0;

    size_t groups_invalid_no_valid_points = 0;

    size_t max_group_size = 0;

    double mean_merged_group_size = 0.0;

    // Dispersión de grupos fusionados, en metros.
    // Se calcula como distancia máxima de cada punto del grupo
    // al centro fusionado del grupo.
    double mean_group_spread_m = 0.0;
    double max_group_spread_m = 0.0;

    size_t groups_with_spread_over_010m = 0;
    size_t groups_with_spread_over_025m = 0;
    size_t groups_with_spread_over_050m = 0;
    size_t groups_with_spread_over_100m = 0;
    size_t groups_warn_spread = 0;

    // FASE 5C:
    size_t groups_rejected_by_spread = 0;
    size_t groups_split_to_singletons = 0;
    size_t singleton_points_from_rejected_groups = 0;

    // ----------------------------
    // Publicación
    // ----------------------------
    size_t fused_landmarks_output = 0;

    // ============================================================
    // FASE 5D-B:
    // Asociaciones directas de landmarks, sin esperar a loop intra.
    // ============================================================

    size_t direct_associations_stored = 0;
    size_t direct_associations_used = 0;

    size_t direct_candidate_matches = 0;
    size_t direct_accepted_matches = 0;

    size_t direct_rejected_zero_mp = 0;
    size_t direct_rejected_same_mp = 0;
    size_t direct_rejected_missing_mp = 0;
    size_t direct_rejected_bad_mp = 0;
    size_t direct_rejected_distance = 0;
    size_t direct_rejected_descriptor = 0;

    double direct_mean_pair_distance_m = 0.0;
    double direct_max_pair_distance_m = 0.0;

    // Fase 6: desglose por origen de asociación directa.
    size_t direct_jobs_consecutive = 0;
    size_t direct_jobs_revisit = 0;
    size_t direct_jobs_other = 0;

    size_t direct_candidate_matches_consecutive = 0;
    size_t direct_candidate_matches_revisit = 0;
    size_t direct_candidate_matches_other = 0;

    size_t direct_accepted_matches_consecutive = 0;
    size_t direct_accepted_matches_revisit = 0;
    size_t direct_accepted_matches_other = 0;

    // ============================================================
    // Fase C:
    // Desglose de asociaciones directas inter-dron.
    // ============================================================

    size_t direct_jobs_inter_drone = 0;
    size_t direct_jobs_cross_epoch_same_drone = 0;
    size_t direct_jobs_same_drone_same_epoch = 0;

    size_t direct_candidate_matches_inter_drone = 0;
    size_t direct_candidate_matches_cross_epoch_same_drone = 0;
    size_t direct_candidate_matches_same_drone_same_epoch = 0;

    size_t direct_accepted_matches_inter_drone = 0;
    size_t direct_accepted_matches_cross_epoch_same_drone = 0;
    size_t direct_accepted_matches_same_drone_same_epoch = 0;

    size_t direct_pairs_used_for_union_inter_drone = 0;
    size_t direct_pairs_used_for_union_cross_epoch_same_drone = 0;
    size_t direct_pairs_used_for_union_same_drone_same_epoch = 0;

    // ============================================================
    // Fase D:
    // Métricas de soporte inter-dron desglosadas por submapa.
    //
    // La key es:
    //   (drone_id << 32) | (map_epoch & 0xFFFFFFFF)
    //
    // Esto evita confirmar un anchor provisional usando evidencia
    // inter-dron que realmente pertenece a otro submapa.
    // ============================================================

    std::unordered_map<uint64_t, size_t>
        inter_supported_groups_by_submap;

    std::unordered_map<uint64_t, size_t>
        inter_merged_groups_by_submap;

    std::unordered_map<uint64_t, size_t>
        inter_union_pairs_by_submap;

    std::unordered_map<uint64_t, size_t>
        inter_direct_union_pairs_by_submap;

    std::unordered_map<uint64_t, size_t>
        inter_loop_union_pairs_by_submap;

    size_t direct_rejected_distance_consecutive = 0;
    size_t direct_rejected_distance_revisit = 0;
    size_t direct_rejected_distance_other = 0;

    size_t direct_rejected_descriptor_consecutive = 0;
    size_t direct_rejected_descriptor_revisit = 0;
    size_t direct_rejected_descriptor_other = 0;
};



struct InterAnchorConsensusDiagnostics
{
    uint32_t target_drone_id = 0;
    uint64_t target_map_epoch = 0;

    size_t loops_total = 0;
    size_t inter_loops_total = 0;
    size_t usable_loops = 0;
    size_t candidate_transforms = 0;

    size_t rejected_not_inter = 0;
    size_t rejected_not_usable_for_anchor = 0;
    size_t rejected_not_usable_for_optimization = 0;
    size_t rejected_requires_fusion_or_strong = 0;
    size_t rejected_both_anchored = 0;
    size_t rejected_both_unanchored = 0;
    size_t rejected_wrong_target = 0;
    size_t rejected_bad_transform = 0;

    size_t clusters = 0;
    size_t best_cluster_size = 0;

    double best_translation_spread_m =
        std::numeric_limits<double>::infinity();

    double best_yaw_spread_deg =
        std::numeric_limits<double>::infinity();

    double best_quality_sum = 0.0;

    bool accepted = false;

    std::string reason;
};



struct LoopAnchorParams
{
    bool enable_inter_drone_loop_anchor = true;

    int min_final_inliers = 30;
    int min_ransac_inliers = 20;
    int min_projection_matches = 20;

    double min_inlier_ratio = 0.25;
    double max_mean_error_m = 0.45;
    double max_error_m = 2.00;

    int min_final_inliers_relaxed_support = 40;

    double max_translation_norm_m = 1000.0;

    bool log_rejections = true;

    // ============================================================
    // Fase 8C:
    // Pesos del pose graph según tipo de loop y madurez del anchor.
    // ============================================================

    bool use_maturity_aware_loop_weights = true;

    double loop_weight_intra_fusion = 80.0;
    double loop_weight_intra_opt_only = 25.0;

    double loop_weight_inter_fusion_confirmed = 80.0;
    double loop_weight_inter_opt_only_confirmed = 20.0;

    double loop_weight_inter_fusion_provisional = 25.0;
    double loop_weight_inter_opt_only_provisional = 10.0;

    double loop_weight_max = 100.0;
    double loop_weight_min = 1.0;

    // ============================================================
    // Fase 8B-fix:
    // Un anchor inter-dron no debe crearse con un loop opt-only débil.
    // Por defecto exigimos fusion=true o una geometría muy fuerte.
    // ============================================================

    bool require_fusion_for_loop_anchor = true;

    bool allow_strong_opt_only_anchor = false;

    int strong_opt_only_min_final_inliers = 90;
    int strong_opt_only_min_ransac_inliers = 35;
    int strong_opt_only_min_projection_matches = 60;

    double strong_opt_only_min_inlier_ratio = 0.55;
    double strong_opt_only_max_mean_error_m = 0.22;
    double strong_opt_only_max_error_m = 0.60;

    // ============================================================
    // Fase F:
    // Consenso de transformaciones para anchors inter-dron.
    //
    // Evita que un único loop fusion=1 cree un anchor provisional.
    // Se requiere que varios loops estimen una transformación
    // world_T_local compatible para el mismo submapa flotante.
    // ============================================================

    bool require_consensus_for_loop_anchor = true;

    int consensus_min_cluster_size = 2;

    double consensus_max_translation_spread_m = 0.30;

    double consensus_max_yaw_spread_deg = 10.0;

    bool consensus_log_details = true;

    // ============================================================
    // Fase G-fix:
    // Uso de inter-loops dentro del pose graph.
    //
    // Un inter-loop puede ser válido para:
    //   - crear anchor provisional;
    //   - fusionar landmarks;
    //   - diagnosticar solape.
    //
    // Pero solo debe entrar como constraint deformable del pose graph
    // cuando ambos submapas sean estables:
    //   FIDUCIAL_DIRECT o LOOP_CONFIRMED.
    //
    // Esto evita que un submapa provisional haga girar el mapa que ya
    // estaba anclado al fiducial.
    // ============================================================

    bool use_inter_drone_loop_edges_for_pose_graph = true;

    bool inter_loop_pose_graph_require_stable_anchors = true;
};



struct FusionValidationParams
{
    // Activa el filtro de distancia 3D entre MapPoints antes de UnionLandmarks.
    bool enable_pair_distance_filter = true;

    // ============================================================
    // FASE 5C:
    // Política explícita de qué loops pueden generar fusiones.
    //
    // En esta fase queremos cerrar solo la fusión inter-dron robusta.
    // La intra-dron se deja para Fase 5D.
    // ============================================================

    bool enable_inter_drone_fusion = true;

    // FASE 5D:
    // Activamos fusión intra-dron, pero con filtros más estrictos.
    // Esto intenta limpiar duplicados creados por revisitas, resets,
    // drift local o loops intra-dron no fusionados dentro de ORB-SLAM3.
    bool enable_intra_drone_fusion = true;

    // FASE 5D:
    // Evita usar loops intra-dron entre KeyFrames demasiado cercanos.
    // ORB-SLAM local ya debería asociar MapPoints entre KFs próximos.
    // Si fusionamos KFs muy cercanos, corremos riesgo de reforzar ruido.
    uint64_t min_intra_drone_kf_id_gap_for_fusion = 20;

    // Si true, no usamos para fusión intra-dron pares de MapPoints
    // del mismo drone/epoch cuando pertenecen a KFs demasiado cercanos.
    bool reject_intra_fusion_for_nearby_keyframes = true;

    // Permite fusión intra entre epochs del mismo dron.
    // Suele ser útil si ORB-SLAM3 resetea o crea nuevo mapa.
    bool enable_cross_epoch_same_drone_fusion = true;

    // Umbral para pares de MapPoints de drones distintos.
    double max_inter_drone_pair_distance_m = 0.35;

    // Umbral para pares del mismo dron y mismo epoch.
    // Más estricto porque ORB-SLAM local normalmente ya debería haber fusionado bien.
    double max_intra_drone_pair_distance_m = 0.20;

    // Umbral para pares del mismo dron pero distinto epoch.
    double max_cross_epoch_same_drone_pair_distance_m = 0.30;

    // Solo diagnóstico. De momento no se usa para partir grupos.
    bool enable_group_spread_warning = true;
    double warn_group_spread_m = 0.50;

    // Si true, un grupo fusionado demasiado disperso no se publica
    // como un único landmark. En vez de eso, se divide en singletons.
    bool split_merged_group_if_spread_too_high = true;

    // Umbral máximo de dispersión permitido para publicar un grupo fusionado.
    // Debe ser algo mayor que el umbral de pares aceptados, pero no demasiado.
    double max_merged_group_spread_m = 0.30;

    // Si true, se publican como singletons los puntos de un grupo rechazado.
    // Así no perdemos puntos del mapa, simplemente evitamos fusionarlos mal.
    bool publish_rejected_group_as_singletons = true;

    // Logs detallados de pares aceptados/rechazados.
    bool log_pair_validation_samples = true;
    size_t max_pair_validation_logs = 30;

    // Fase 6 experimental:
    // Permite usar loops intra-dron opt-only como fuente de pares para fusión,
    // pero solo si además los pares MapPoint-MapPoint pasan distancia 3D,
    // descriptor y spread final.
    // Por defecto apagado.
    bool allow_intra_opt_only_loops_for_fusion_candidates = false;
};


// ============================================================
// Fachada principal de orbslam3_multi.
//
// Esta clase es la API que debe usar el servidor ROS2.
// El servidor NO debería implementar BoW, matching o geometría.
// Solo debe convertir mensajes ROS a ImportedKeyFrame/MapPoint
// y llamar a esta clase.
// ============================================================

class MultiDroneSystem
{
public:
    MultiDroneSystem();

    // ============================================================
    // Inserción/actualización de datos importados desde wrappers.
    // ============================================================

    void InsertOrUpdateKeyFrame(const ImportedKeyFrame& kf);
    void InsertOrUpdateMapPoint(const ImportedMapPoint& mp);
    void ClearDroneEpoch(
        uint32_t drone_id,
        uint64_t map_epoch);

    // ============================================================
    // Información de cámara por dron/map_epoch.
    //
    // Necesaria para SearchByProjection en el Paso 6.
    // ============================================================

    void SetCameraInfo(
        uint32_t drone_id,
        uint64_t map_epoch,
        const CameraInfo& info);

    CameraInfo GetCameraInfo(
        uint32_t drone_id,
        uint64_t map_epoch) const;

    // ============================================================
    // Información general del atlas.
    // ============================================================

    AtlasCounts GetCounts() const;

    std::vector<RawPoint> GetRawCloud() const;
    std::vector<RawPoint> GetAnchoredCloud() const;
    std::vector<PoseGraphVertex> GetAnchoredKeyFramePoses() const;

    std::shared_ptr<GlobalAtlas> GetAtlas();
    std::shared_ptr<GlobalKeyFrameDatabase> GetKeyFrameDatabase();

    // ============================================================
    // Paso 3/4: búsqueda de candidatos BoW.
    // ============================================================

    std::vector<BowCandidate> DetectBowCandidates(
        uint64_t query_global_kf_id,
        const BowQueryParams& params) const;

    // ============================================================
    // Paso 5: SearchByBoW tipo ORBmatcher.
    // ============================================================

    std::vector<FeatureMatch> SearchByBoW(
        uint64_t query_global_kf_id,
        uint64_t candidate_global_kf_id,
        const SearchByBowParams& params) const;

    // ============================================================
    // Paso 6: verificación geométrica.
    //
    // Hace:
    //   BoW matches -> SE3 RANSAC -> SearchByProjection -> refine SE3.
    // ============================================================

    GeometryVerificationResult VerifyGeometry(
        uint64_t query_global_kf_id,
        uint64_t candidate_global_kf_id,
        const std::vector<FeatureMatch>& initial_matches,
        const GeometryVerificationParams& params) const;


    bool AddVerifiedLoopEdge(
        const GeometryVerificationResult& geom,
        LoopEdgeType type);

    bool AddDirectLandmarkAssociation(
        const DirectLandmarkAssociation& association);

    std::vector<DirectLandmarkAssociation>
    GetDirectLandmarkAssociations() const;

    size_t GetDirectLandmarkAssociationCount() const;

    void ClearDirectLandmarkAssociationsForDroneEpoch(
        uint32_t drone_id,
        uint64_t map_epoch);
    
    bool TryAnchorMapFromLoop(
        const GeometryVerificationResult& geom,
        LoopEdgeType type);

    bool TryAnchorMapsFromStoredLoops(
        int max_iterations = 10);

    bool TryAnchorMapFromStoredLoopEdge(
        const VerifiedLoopEdge& edge);

    bool CheckInterDroneAnchorConsensus(
        uint32_t target_drone_id,
        uint64_t target_map_epoch,
        Eigen::Matrix4d& consensus_world_T_local,
        InterAnchorConsensusDiagnostics* diagnostics = nullptr) const;

    bool HasLoopEdge(
        uint64_t kf_a,
        uint64_t kf_b) const;

    bool GetLoopEdge(
        uint64_t kf_a,
        uint64_t kf_b,
        VerifiedLoopEdge& edge_out) const;

    bool SetLoopEdgeAnchorUsability(
        uint64_t kf_a,
        uint64_t kf_b,
        bool usable_for_anchor,
        const std::string& reason);

    std::vector<VerifiedLoopEdge> GetLoopEdges() const;

    size_t GetLoopEdgeCount() const;


    // ============================================================
    // Paso 8: anchors y pose graph.
    // ============================================================

    void SetMapAnchor(
        uint32_t drone_id,
        uint64_t map_epoch,
        const Eigen::Matrix4d& world_T_local,
        AnchorSource source = AnchorSource::UNKNOWN,
        AnchorMaturity maturity = AnchorMaturity::UNKNOWN);

    bool HasMapAnchor(
        uint32_t drone_id,
        uint64_t map_epoch) const;

    bool GetMapAnchor(
        uint32_t drone_id,
        uint64_t map_epoch,
        Eigen::Matrix4d& world_T_local_out) const;

    bool GetMapAnchorInfo(
        uint32_t drone_id,
        uint64_t map_epoch,
        MapAnchor& anchor_out) const;

    uint64_t MakeSubmapKeyPublic(
        uint32_t drone_id,
        uint64_t map_epoch) const;

    PoseGraphSnapshot BuildPoseGraphSnapshot(
        int min_covisibility_weight,
        int max_covisibility_edges_per_keyframe) const;

    std::vector<FusedLandmark> BuildFusedLandmarksFromLoopEdges() const;

    std::vector<FusedLandmark> BuildFusedLandmarksFromLoopEdges(
        FusionDiagnostics* diagnostics) const;

    void SetFusionValidationParams(
        const FusionValidationParams& params);

    FusionValidationParams GetFusionValidationParams() const;


    void SetLoopAnchorParams(
        const LoopAnchorParams& params);

    LoopAnchorParams GetLoopAnchorParams() const;


    GlobalPoseGraphOptimizationResult OptimizeEssentialGraph(
        const GlobalPoseGraphOptimizationParams& params,
        int min_covisibility_weight,
        int max_covisibility_edges_per_keyframe) const;

    void DebugPrintPoseGraphValidation(
        int min_covisibility_weight,
        int max_covisibility_edges_per_keyframe) const;

    bool AddFiducialPoseConstraint(
        const FiducialPoseConstraint& constraint);

    bool RemoveFiducialPoseConstraint(
        uint32_t drone_id,
        uint64_t map_epoch,
        uint64_t global_kf_id,
        uint32_t fiducial_id);

    size_t GetFiducialConstraintCount() const;

    uint64_t GetLatestKeyFrameId(
        uint32_t drone_id,
        uint64_t map_epoch) const;

    void SetPoseGraphInitialGuesses(
        const std::unordered_map<uint64_t, Eigen::Matrix4d>& guesses);

    void ClearPoseGraphInitialGuesses();

private:
    uint64_t MakeCameraKey(
        uint32_t drone_id,
        uint64_t map_epoch) const;

private:
    std::shared_ptr<GlobalAtlas> atlas_;
    std::shared_ptr<GlobalKeyFrameDatabase> keyframe_database_;

    mutable std::mutex camera_mutex_;
    std::unordered_map<uint64_t, CameraInfo> camera_info_by_drone_epoch_;

    LoopEdgePairKey MakeLoopEdgePairKey(
        uint64_t kf_a,
        uint64_t kf_b) const;

    uint32_t ExtractDroneId(uint64_t global_id) const;
    uint64_t ExtractLocalId(uint64_t global_id) const;

    mutable std::mutex loop_edges_mutex_;

    std::unordered_map<
        LoopEdgePairKey,
        VerifiedLoopEdge,
        LoopEdgePairKeyHash> loop_edges_;

    mutable std::mutex direct_landmark_associations_mutex_;

    std::unordered_map<
        LoopEdgePairKey,
        DirectLandmarkAssociation,
        LoopEdgePairKeyHash>
        direct_landmark_associations_;

    Eigen::Matrix4d KeyFramePoseToMatrix(
        const ImportedKeyFrame& kf) const;

    Eigen::Matrix4d RelativePose(
        const Eigen::Matrix4d& world_T_from,
        const Eigen::Matrix4d& world_T_to) const;

    mutable std::mutex anchors_mutex_;

    std::unordered_map<uint64_t, MapAnchor> map_anchors_;

    uint64_t FindUnionRoot(
        uint64_t id,
        std::unordered_map<uint64_t, uint64_t>& parent) const;

    void UnionLandmarks(
        uint64_t a,
        uint64_t b,
        std::unordered_map<uint64_t, uint64_t>& parent) const;

    bool MapPointToWorld(
        const ImportedMapPoint& mp,
        Eigen::Vector3d& p_world_out) const;
    
    std::vector<Eigen::Vector3d> GetAnchoredMapPointsWorld() const;

    mutable std::mutex fiducial_constraints_mutex_;
    std::vector<FiducialPoseConstraint> fiducial_constraints_;


    mutable std::mutex pose_graph_initial_guess_mutex_;

    std::unordered_map<uint64_t, Eigen::Matrix4d>
        pose_graph_initial_guesses_;

    mutable std::mutex fusion_validation_params_mutex_;
    FusionValidationParams fusion_validation_params_;

    mutable std::mutex loop_anchor_params_mutex_;
    LoopAnchorParams loop_anchor_params_;
    };
}  // namespace orbslam3_multi
