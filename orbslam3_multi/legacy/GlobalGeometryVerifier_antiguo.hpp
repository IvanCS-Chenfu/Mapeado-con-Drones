#pragma once

#include "orbslam3_multi/legacy/GlobalAtlas_antiguo.hpp"
#include "orbslam3_multi/legacy/GlobalORBMatcher_antiguo.hpp"
#include "orbslam3_multi/legacy/ImportedKeyFrame_antiguo.hpp"
#include "orbslam3_multi/legacy/ImportedMapPoint_antiguo.hpp"

#include <Eigen/Dense>

#include <memory>
#include <vector>
#include <cstdint>
#include <random>
#include <unordered_set>
#include <set>

namespace orbslam3_multi
{

// ============================================================
// Intrínsecas de cámara.
// Se guardan por drone_id + map_epoch en MultiDroneSystem.
// ============================================================

struct CameraInfo
{
    float fx = -1.0f;
    float fy = -1.0f;
    float cx = -1.0f;
    float cy = -1.0f;
    float bf = -1.0f;

    uint32_t width = 0;
    uint32_t height = 0;

    bool IsValid() const
    {
        return fx > 0.0f && fy > 0.0f &&
               cx >= 0.0f && cy >= 0.0f &&
               bf > 0.0f &&
               width > 0 && height > 0;
    }
};


// ============================================================
// Parámetros de verificación geométrica.
// Para estéreo empezamos con SE3, es decir, escala fija.
// Esto es equivalente al caso métrico de ORB-SLAM3.
// ============================================================

struct GeometryVerificationParams
{
    // ============================================================
    // RANSAC SE3.
    // ============================================================

    int ransac_iterations = 200;
    double ransac_threshold_m = 0.30;

    int min_ransac_inliers = 8;
    double min_inlier_ratio = 0.15;

    // ============================================================
    // Refinamiento.
    // ============================================================

    double max_mean_error_m = 0.30;
    double max_max_error_m = 1.00;

    // ============================================================
    // SearchByProjection.
    // ============================================================

    bool enable_projection_search = true;
    double projection_radius_px = 8.0;
    int projection_descriptor_threshold = 60;
    int min_projection_matches = 12;

    // ============================================================
    // Para aceptar resultado final.
    // ============================================================

    int min_final_inliers = 10;
};


// ============================================================
// Resultado de verificación geométrica.
// candidate_T_query transforma puntos del mapa local query al
// mapa local candidate.
// ============================================================

struct GeometryVerificationResult
{
    bool success = false;

    uint64_t query_kf_id = 0;
    uint64_t candidate_kf_id = 0;

    Eigen::Matrix4d candidate_T_query =
        Eigen::Matrix4d::Identity();

    std::vector<FeatureMatch> initial_matches;
    std::vector<FeatureMatch> ransac_inlier_matches;
    std::vector<FeatureMatch> projection_matches;
    std::vector<FeatureMatch> final_inlier_matches;

    int initial_match_count = 0;
    int ransac_inlier_count = 0;
    int projection_match_count = 0;
    int final_inlier_count = 0;

    double inlier_ratio = 0.0;
    double mean_error_m = 0.0;
    double max_error_m = 0.0;

    // ============================================================
    // Debug interno de geometría.
    //
    // Estos campos NO son parámetros.
    // Son resultados de diagnóstico para saber por qué falla
    // VerifyGeometry().
    // ============================================================

    int valid_3d_pairs = 0;

    int skipped_no_mappoint = 0;
    int skipped_missing_mappoint = 0;
    int skipped_bad_mappoint = 0;

    int best_ransac_inliers = 0;
    double best_ransac_mean_error_m = 0.0;
    double best_ransac_max_error_m = 0.0;
};


// ============================================================
// Verificador geométrico estilo ORB-SLAM3 para datos importados.
//
// Pipeline:
//   1. FeatureMatch -> pares 3D-3D.
//   2. SE3 RANSAC.
//   3. Refinamiento SE3 con inliers.
//   4. SearchByProjection para ampliar matches.
//   5. Refinamiento final.
// ============================================================

class GlobalGeometryVerifier
{
public:
    explicit GlobalGeometryVerifier(
        std::shared_ptr<GlobalAtlas> atlas);

    GeometryVerificationResult Verify(
        const ImportedKeyFrame& query,
        const ImportedKeyFrame& candidate,
        const std::vector<FeatureMatch>& initial_matches,
        const CameraInfo& query_camera,
        const CameraInfo& candidate_camera,
        const GeometryVerificationParams& params) const;

private:
    struct PointPair3D
    {
        Eigen::Vector3d query_point =
            Eigen::Vector3d::Zero();

        Eigen::Vector3d candidate_point =
            Eigen::Vector3d::Zero();

        FeatureMatch match;
    };

    bool BuildPointPairs3D(
        const std::vector<FeatureMatch>& matches,
        std::vector<PointPair3D>& pairs_out,
        GeometryVerificationResult* debug_result) const;

    bool EstimateSE3SVD(
        const std::vector<PointPair3D>& pairs,
        const std::vector<int>& indices,
        Eigen::Matrix4d& candidate_T_query_out) const;

    bool EstimateSE3Ransac(
        const std::vector<PointPair3D>& pairs,
        const GeometryVerificationParams& params,
        Eigen::Matrix4d& candidate_T_query_out,
        std::vector<int>& inliers_out,
        double& mean_error_out,
        double& max_error_out,
        GeometryVerificationResult* debug_result) const;

    void ComputeInliers(
        const std::vector<PointPair3D>& pairs,
        const Eigen::Matrix4d& candidate_T_query,
        double threshold_m,
        std::vector<int>& inliers_out,
        double& mean_error_out,
        double& max_error_out) const;

    std::vector<FeatureMatch> SearchByProjection(
        const ImportedKeyFrame& query,
        const ImportedKeyFrame& candidate,
        const Eigen::Matrix4d& candidate_T_query,
        const CameraInfo& candidate_camera,
        const GeometryVerificationParams& params) const;

    bool ProjectPointIntoKeyFrame(
        const ImportedKeyFrame& keyframe,
        const Eigen::Vector3d& point_in_candidate_map,
        const CameraInfo& camera,
        double& u_out,
        double& v_out,
        double& depth_out) const;

    bool RefineWithMatches(
        const std::vector<FeatureMatch>& matches,
        Eigen::Matrix4d& candidate_T_query_out,
        double& mean_error_out,
        double& max_error_out,
        std::vector<FeatureMatch>& inlier_matches_out,
        const GeometryVerificationParams& params) const;

    static bool SameMatch(
        const FeatureMatch& a,
        const FeatureMatch& b);

    static void AppendUniqueMatches(
        std::vector<FeatureMatch>& base,
        const std::vector<FeatureMatch>& extra);

private:
    std::shared_ptr<GlobalAtlas> atlas_;
};

}  // namespace orbslam3_multi
