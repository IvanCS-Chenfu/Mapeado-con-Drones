#include "orbslam3_multi/legacy/GlobalGeometryVerifier_antiguo.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace orbslam3_multi
{

GlobalGeometryVerifier::GlobalGeometryVerifier(
    std::shared_ptr<GlobalAtlas> atlas)
    : atlas_(std::move(atlas))
{
}


// ============================================================
// Verificación geométrica principal.
// ============================================================

GeometryVerificationResult GlobalGeometryVerifier::Verify(
    const ImportedKeyFrame& query,
    const ImportedKeyFrame& candidate,
    const std::vector<FeatureMatch>& initial_matches,
    const CameraInfo& query_camera,
    const CameraInfo& candidate_camera,
    const GeometryVerificationParams& params) const
{
    (void)query_camera;

    GeometryVerificationResult result;

    result.query_kf_id = query.global_id;
    result.candidate_kf_id = candidate.global_id;
    result.initial_matches = initial_matches;
    result.initial_match_count =
        static_cast<int>(initial_matches.size());

    if (initial_matches.empty())
        return result;

    // ============================================================
    // 1. Convertir FeatureMatch a pares 3D-3D.
    // ============================================================

    std::vector<PointPair3D> initial_pairs;

    if (!BuildPointPairs3D(
            initial_matches,
            initial_pairs,
            &result))
    {
        return result;
    }

    // Si aquí falla, ahora el servidor podrá ver valid_3d_pairs,
    // skipped_no_mappoint, skipped_missing_mappoint y skipped_bad_mappoint.

    if (initial_pairs.size() <
        static_cast<size_t>(params.min_ransac_inliers))
    {
        return result;
    }

    // ============================================================
    // 2. SE3 RANSAC.
    // ============================================================

    Eigen::Matrix4d candidate_T_query =
        Eigen::Matrix4d::Identity();

    std::vector<int> ransac_inliers;
    double mean_error = 0.0;
    double max_error = 0.0;

    if (!EstimateSE3Ransac(
            initial_pairs,
            params,
            candidate_T_query,
            ransac_inliers,
            mean_error,
            max_error,
            &result))
    {
        return result;
    }

    result.ransac_inlier_count =
        static_cast<int>(ransac_inliers.size());

    result.inlier_ratio =
        static_cast<double>(ransac_inliers.size()) /
        static_cast<double>(initial_pairs.size());

    result.mean_error_m = mean_error;
    result.max_error_m = max_error;

    if (result.ransac_inlier_count < params.min_ransac_inliers)
        return result;

    if (result.inlier_ratio < params.min_inlier_ratio)
        return result;

    if (result.mean_error_m > params.max_mean_error_m)
        return result;

    for (int idx : ransac_inliers)
    {
        if (idx < 0 || idx >= static_cast<int>(initial_pairs.size()))
            continue;

        result.ransac_inlier_matches.push_back(
            initial_pairs[static_cast<size_t>(idx)].match);
    }

    // ============================================================
    // 3. SearchByProjection.
    //
    // Igual que ORB-SLAM3, no nos quedamos solo con matches BoW.
    // Usamos la transformación estimada para buscar más puntos.
    // ============================================================

    std::vector<FeatureMatch> final_matches =
        result.ransac_inlier_matches;

    if (params.enable_projection_search &&
        candidate_camera.IsValid())
    {
        result.projection_matches =
            SearchByProjection(
                query,
                candidate,
                candidate_T_query,
                candidate_camera,
                params);

        result.projection_match_count =
            static_cast<int>(result.projection_matches.size());

        AppendUniqueMatches(
            final_matches,
            result.projection_matches);
    }

    if (final_matches.size() <
        static_cast<size_t>(params.min_final_inliers))
    {
        return result;
    }

    // ============================================================
    // 4. Refinamiento final con los matches ampliados.
    // ============================================================

    std::vector<FeatureMatch> final_inlier_matches;
    double final_mean_error = 0.0;
    double final_max_error = 0.0;

    if (!RefineWithMatches(
            final_matches,
            candidate_T_query,
            final_mean_error,
            final_max_error,
            final_inlier_matches,
            params))
    {
        return result;
    }

    if (final_inlier_matches.size() <
        static_cast<size_t>(params.min_final_inliers))
    {
        return result;
    }

    if (final_mean_error > params.max_mean_error_m)
        return result;

    if (final_max_error > params.max_max_error_m)
        return result;

    // ============================================================
    // Resultado aceptado.
    // ============================================================

    result.success = true;
    result.candidate_T_query = candidate_T_query;

    result.final_inlier_matches = final_inlier_matches;
    result.final_inlier_count =
        static_cast<int>(final_inlier_matches.size());

    result.mean_error_m = final_mean_error;
    result.max_error_m = final_max_error;

    return result;
}


// ============================================================
// Convierte FeatureMatch a pares 3D-3D usando el GlobalAtlas.
// ============================================================

bool GlobalGeometryVerifier::BuildPointPairs3D(
    const std::vector<FeatureMatch>& matches,
    std::vector<PointPair3D>& pairs_out,
    GeometryVerificationResult* debug_result) const
{
    pairs_out.clear();
    pairs_out.reserve(matches.size());

    if (debug_result)
    {
        debug_result->valid_3d_pairs = 0;
        debug_result->skipped_no_mappoint = 0;
        debug_result->skipped_missing_mappoint = 0;
        debug_result->skipped_bad_mappoint = 0;
    }

    if (!atlas_)
        return false;

    for (const auto& m : matches)
    {
        if (m.query_mappoint_id == 0 ||
            m.candidate_mappoint_id == 0)
        {
            if (debug_result)
                debug_result->skipped_no_mappoint++;

            continue;
        }

        ImportedMapPoint query_mp =
            atlas_->GetMapPoint(m.query_mappoint_id);

        ImportedMapPoint candidate_mp =
            atlas_->GetMapPoint(m.candidate_mappoint_id);

        if (query_mp.global_id == 0 ||
            candidate_mp.global_id == 0)
        {
            if (debug_result)
                debug_result->skipped_missing_mappoint++;

            continue;
        }

        if (query_mp.is_bad || candidate_mp.is_bad)
        {
            if (debug_result)
                debug_result->skipped_bad_mappoint++;

            continue;
        }

        PointPair3D pair;

        pair.query_point =
            query_mp.position;

        pair.candidate_point =
            candidate_mp.position;

        pair.match = m;

        pairs_out.push_back(pair);
    }

    if (debug_result)
    {
        debug_result->valid_3d_pairs =
            static_cast<int>(pairs_out.size());
    }

    return pairs_out.size() >= 3;
}


// ============================================================
// Estima SE3 mediante SVD:
//
//   candidate_point ~= R * query_point + t
//
// candidate_T_query transforma del mapa local query al mapa local
// candidate.
// ============================================================

bool GlobalGeometryVerifier::EstimateSE3SVD(
    const std::vector<PointPair3D>& pairs,
    const std::vector<int>& indices,
    Eigen::Matrix4d& candidate_T_query_out) const
{
    if (indices.size() < 3)
        return false;

    Eigen::Vector3d centroid_q =
        Eigen::Vector3d::Zero();

    Eigen::Vector3d centroid_c =
        Eigen::Vector3d::Zero();

    size_t valid_count = 0;

    for (int idx : indices)
    {
        if (idx < 0 || idx >= static_cast<int>(pairs.size()))
            continue;

        centroid_q += pairs[static_cast<size_t>(idx)].query_point;
        centroid_c += pairs[static_cast<size_t>(idx)].candidate_point;
        valid_count++;
    }

    if (valid_count < 3)
        return false;

    centroid_q /= static_cast<double>(valid_count);
    centroid_c /= static_cast<double>(valid_count);

    Eigen::Matrix3d H =
        Eigen::Matrix3d::Zero();

    for (int idx : indices)
    {
        if (idx < 0 || idx >= static_cast<int>(pairs.size()))
            continue;

        const Eigen::Vector3d q =
            pairs[static_cast<size_t>(idx)].query_point -
            centroid_q;

        const Eigen::Vector3d c =
            pairs[static_cast<size_t>(idx)].candidate_point -
            centroid_c;

        H += q * c.transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        H,
        Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix3d R =
        svd.matrixV() * svd.matrixU().transpose();

    if (R.determinant() < 0.0)
    {
        Eigen::Matrix3d V =
            svd.matrixV();

        V.col(2) *= -1.0;

        R = V * svd.matrixU().transpose();
    }

    Eigen::Vector3d t =
        centroid_c - R * centroid_q;

    candidate_T_query_out =
        Eigen::Matrix4d::Identity();

    candidate_T_query_out.block<3, 3>(0, 0) = R;
    candidate_T_query_out.block<3, 1>(0, 3) = t;

    return true;
}


// ============================================================
// Calcula inliers para una transformación candidata.
// ============================================================

void GlobalGeometryVerifier::ComputeInliers(
    const std::vector<PointPair3D>& pairs,
    const Eigen::Matrix4d& candidate_T_query,
    double threshold_m,
    std::vector<int>& inliers_out,
    double& mean_error_out,
    double& max_error_out) const
{
    inliers_out.clear();

    mean_error_out = 0.0;
    max_error_out = 0.0;

    if (pairs.empty())
        return;

    const Eigen::Matrix3d R =
        candidate_T_query.block<3, 3>(0, 0);

    const Eigen::Vector3d t =
        candidate_T_query.block<3, 1>(0, 3);

    double sum_error = 0.0;

    for (size_t i = 0; i < pairs.size(); ++i)
    {
        const Eigen::Vector3d predicted =
            R * pairs[i].query_point + t;

        const double error =
            (predicted - pairs[i].candidate_point).norm();

        if (error <= threshold_m)
        {
            inliers_out.push_back(static_cast<int>(i));
            sum_error += error;
            max_error_out = std::max(max_error_out, error);
        }
    }

    if (!inliers_out.empty())
    {
        mean_error_out =
            sum_error / static_cast<double>(inliers_out.size());
    }
}


// ============================================================
// SE3 RANSAC.
// ============================================================

bool GlobalGeometryVerifier::EstimateSE3Ransac(
    const std::vector<PointPair3D>& pairs,
    const GeometryVerificationParams& params,
    Eigen::Matrix4d& candidate_T_query_out,
    std::vector<int>& inliers_out,
    double& mean_error_out,
    double& max_error_out,
    GeometryVerificationResult* debug_result) const
{
    inliers_out.clear();
    mean_error_out = 0.0;
    max_error_out = 0.0;

    if (debug_result)
    {
        debug_result->best_ransac_inliers = 0;
        debug_result->best_ransac_mean_error_m = 0.0;
        debug_result->best_ransac_max_error_m = 0.0;
    }

    if (pairs.size() < 3)
        return false;

    std::mt19937 rng(12345);
    std::uniform_int_distribution<int> dist(
        0,
        static_cast<int>(pairs.size()) - 1);

    std::vector<int> best_inliers;

    double best_mean_error =
        std::numeric_limits<double>::max();

    double best_max_error =
        std::numeric_limits<double>::max();

    Eigen::Matrix4d best_T =
        Eigen::Matrix4d::Identity();

    for (int iter = 0; iter < params.ransac_iterations; ++iter)
    {
        std::set<int> sample_set;

        while (sample_set.size() < 3)
        {
            sample_set.insert(dist(rng));
        }

        std::vector<int> sample_indices(
            sample_set.begin(),
            sample_set.end());

        Eigen::Matrix4d T =
            Eigen::Matrix4d::Identity();

        if (!EstimateSE3SVD(
                pairs,
                sample_indices,
                T))
        {
            continue;
        }

        std::vector<int> inliers;
        double mean_error = 0.0;
        double max_error = 0.0;

        ComputeInliers(
            pairs,
            T,
            params.ransac_threshold_m,
            inliers,
            mean_error,
            max_error);

        if (inliers.size() > best_inliers.size() ||
            (inliers.size() == best_inliers.size() &&
             mean_error < best_mean_error))
        {
            best_inliers = inliers;
            best_mean_error = mean_error;
            best_max_error = max_error;
            best_T = T;
        }
    }

    if (debug_result)
    {
        debug_result->best_ransac_inliers =
            static_cast<int>(best_inliers.size());

        if (best_inliers.empty())
        {
            debug_result->best_ransac_mean_error_m = 0.0;
            debug_result->best_ransac_max_error_m = 0.0;
        }
        else
        {
            debug_result->best_ransac_mean_error_m =
                best_mean_error;

            debug_result->best_ransac_max_error_m =
                best_max_error;
        }
    }

    if (best_inliers.size() <
        static_cast<size_t>(params.min_ransac_inliers))
    {
        return false;
    }

    // Refinar con todos los mejores inliers.
    Eigen::Matrix4d refined_T =
        Eigen::Matrix4d::Identity();

    if (!EstimateSE3SVD(
            pairs,
            best_inliers,
            refined_T))
    {
        return false;
    }

    std::vector<int> refined_inliers;
    double refined_mean_error = 0.0;
    double refined_max_error = 0.0;

    ComputeInliers(
        pairs,
        refined_T,
        params.ransac_threshold_m,
        refined_inliers,
        refined_mean_error,
        refined_max_error);

    if (debug_result)
    {
        debug_result->best_ransac_inliers =
            static_cast<int>(refined_inliers.size());

        debug_result->best_ransac_mean_error_m =
            refined_mean_error;

        debug_result->best_ransac_max_error_m =
            refined_max_error;
    }

    if (refined_inliers.size() <
        static_cast<size_t>(params.min_ransac_inliers))
    {
        return false;
    }

    candidate_T_query_out = refined_T;
    inliers_out = refined_inliers;
    mean_error_out = refined_mean_error;
    max_error_out = refined_max_error;

    return true;
}


// ============================================================
// Proyecta un punto expresado en el mapa local candidate dentro
// del keyframe candidate.
//
// keyframe.position/orientation es Twc.
// Por tanto Tcw = Twc^-1.
// ============================================================

bool GlobalGeometryVerifier::ProjectPointIntoKeyFrame(
    const ImportedKeyFrame& keyframe,
    const Eigen::Vector3d& point_in_candidate_map,
    const CameraInfo& camera,
    double& u_out,
    double& v_out,
    double& depth_out) const
{
    if (!camera.IsValid())
        return false;

    Eigen::Quaterniond q_wc =
        keyframe.orientation;

    q_wc.normalize();

    const Eigen::Matrix3d R_wc =
        q_wc.toRotationMatrix();

    const Eigen::Matrix3d R_cw =
        R_wc.transpose();

    const Eigen::Vector3d t_wc =
        keyframe.position;

    const Eigen::Vector3d p_c =
        R_cw * (point_in_candidate_map - t_wc);

    if (p_c.z() <= 0.05)
        return false;

    const double inv_z =
        1.0 / p_c.z();

    const double u =
        camera.fx * p_c.x() * inv_z + camera.cx;

    const double v =
        camera.fy * p_c.y() * inv_z + camera.cy;

    if (u < 0.0 ||
        v < 0.0 ||
        u >= static_cast<double>(camera.width) ||
        v >= static_cast<double>(camera.height))
    {
        return false;
    }

    u_out = u;
    v_out = v;
    depth_out = p_c.z();

    return true;
}


// ============================================================
// SearchByProjection simplificado estilo ORB-SLAM3.
//
// Usamos candidate_T_query para proyectar MapPoints de query
// dentro del KeyFrame candidate y buscar keypoints cercanos.
// ============================================================

std::vector<FeatureMatch> GlobalGeometryVerifier::SearchByProjection(
    const ImportedKeyFrame& query,
    const ImportedKeyFrame& candidate,
    const Eigen::Matrix4d& candidate_T_query,
    const CameraInfo& candidate_camera,
    const GeometryVerificationParams& params) const
{
    std::vector<FeatureMatch> matches;

    if (!atlas_ || !candidate_camera.IsValid())
        return matches;

    const Eigen::Matrix3d R =
        candidate_T_query.block<3, 3>(0, 0);

    const Eigen::Vector3d t =
        candidate_T_query.block<3, 1>(0, 3);

    std::vector<bool> candidate_used(
        candidate.keypoints.size(),
        false);

    for (size_t idx_q = 0; idx_q < query.keypoints.size(); ++idx_q)
    {
        if (idx_q >= query.mappoint_ids.size())
            continue;

        const uint64_t query_mp_id =
            query.mappoint_ids[idx_q];

        if (query_mp_id == 0)
            continue;

        ImportedMapPoint query_mp =
            atlas_->GetMapPoint(query_mp_id);

        if (query_mp.global_id == 0 || query_mp.is_bad)
            continue;

        const Eigen::Vector3d point_candidate_map =
            R * query_mp.position + t;

        double u = 0.0;
        double v = 0.0;
        double depth = 0.0;

        if (!ProjectPointIntoKeyFrame(
                candidate,
                point_candidate_map,
                candidate_camera,
                u,
                v,
                depth))
        {
            continue;
        }

        (void)depth;

        int best_dist =
            std::numeric_limits<int>::max();

        int second_best_dist =
            std::numeric_limits<int>::max();

        int best_idx_c = -1;

        const auto& desc_q =
            query.keypoints[idx_q].descriptor;

        if (!GlobalORBMatcher::DescriptorIsValid(desc_q))
            continue;

        for (size_t idx_c = 0; idx_c < candidate.keypoints.size(); ++idx_c)
        {
            if (candidate_used[idx_c])
                continue;

            if (idx_c >= candidate.mappoint_ids.size())
                continue;

            const uint64_t candidate_mp_id =
                candidate.mappoint_ids[idx_c];

            if (candidate_mp_id == 0)
                continue;

            const double du =
                candidate.keypoints[idx_c].u - u;

            const double dv =
                candidate.keypoints[idx_c].v - v;

            const double dist_px =
                std::sqrt(du * du + dv * dv);

            if (dist_px > params.projection_radius_px)
                continue;

            const auto& desc_c =
                candidate.keypoints[idx_c].descriptor;

            if (!GlobalORBMatcher::DescriptorIsValid(desc_c))
                continue;

            const int d =
                GlobalORBMatcher::DescriptorDistance(
                    desc_q,
                    desc_c);

            if (d < best_dist)
            {
                second_best_dist = best_dist;
                best_dist = d;
                best_idx_c = static_cast<int>(idx_c);
            }
            else if (d < second_best_dist)
            {
                second_best_dist = d;
            }
        }

        if (best_idx_c < 0)
            continue;

        if (best_dist > params.projection_descriptor_threshold)
            continue;

        // Ratio test suave.
        if (second_best_dist < std::numeric_limits<int>::max())
        {
            if (static_cast<float>(best_dist) >
                0.8f * static_cast<float>(second_best_dist))
            {
                continue;
            }
        }

        FeatureMatch m;

        m.idx_query = idx_q;
        m.idx_candidate = static_cast<size_t>(best_idx_c);

        m.query_mappoint_id = query_mp_id;
        m.candidate_mappoint_id =
            candidate.mappoint_ids[static_cast<size_t>(best_idx_c)];

        m.distance = best_dist;

        matches.push_back(m);

        candidate_used[static_cast<size_t>(best_idx_c)] = true;
    }

    return matches;
}


// ============================================================
// Refina SE3 a partir de una lista de FeatureMatch.
// ============================================================

bool GlobalGeometryVerifier::RefineWithMatches(
    const std::vector<FeatureMatch>& matches,
    Eigen::Matrix4d& candidate_T_query_out,
    double& mean_error_out,
    double& max_error_out,
    std::vector<FeatureMatch>& inlier_matches_out,
    const GeometryVerificationParams& params) const
{
    inlier_matches_out.clear();

    std::vector<PointPair3D> pairs;

    if (!BuildPointPairs3D(
            matches,
            pairs,
            nullptr))
    {
        return false;
    }

    if (pairs.size() < 3)
        return false;

    std::vector<int> all_indices;
    all_indices.reserve(pairs.size());

    for (size_t i = 0; i < pairs.size(); ++i)
    {
        all_indices.push_back(static_cast<int>(i));
    }

    if (!EstimateSE3SVD(
            pairs,
            all_indices,
            candidate_T_query_out))
    {
        return false;
    }

    std::vector<int> inliers;

    ComputeInliers(
        pairs,
        candidate_T_query_out,
        params.ransac_threshold_m,
        inliers,
        mean_error_out,
        max_error_out);

    if (inliers.empty())
        return false;

    for (int idx : inliers)
    {
        if (idx < 0 || idx >= static_cast<int>(pairs.size()))
            continue;

        inlier_matches_out.push_back(
            pairs[static_cast<size_t>(idx)].match);
    }

    return true;
}


bool GlobalGeometryVerifier::SameMatch(
    const FeatureMatch& a,
    const FeatureMatch& b)
{
    return
        a.query_mappoint_id == b.query_mappoint_id &&
        a.candidate_mappoint_id == b.candidate_mappoint_id;
}


void GlobalGeometryVerifier::AppendUniqueMatches(
    std::vector<FeatureMatch>& base,
    const std::vector<FeatureMatch>& extra)
{
    for (const auto& e : extra)
    {
        bool exists = false;

        for (const auto& b : base)
        {
            if (SameMatch(b, e))
            {
                exists = true;
                break;
            }
        }

        if (!exists)
        {
            base.push_back(e);
        }
    }
}

}  // namespace orbslam3_multi
