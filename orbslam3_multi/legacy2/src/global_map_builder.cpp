#include "orbslam3_multi/global_map_builder.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <optional>

namespace orbslam3_multi
{
namespace
{

bool PoseMsgToMatrix(const geometry_msgs::msg::Pose& pose, Eigen::Matrix4d& out)
{
    const Eigen::Quaterniond q_raw(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z);

    if (!std::isfinite(q_raw.w()) ||
        !std::isfinite(q_raw.x()) ||
        !std::isfinite(q_raw.y()) ||
        !std::isfinite(q_raw.z()) ||
        q_raw.norm() < 1e-9)
    {
        return false;
    }

    const Eigen::Quaterniond q = q_raw.normalized();
    out = Eigen::Matrix4d::Identity();
    out.block<3, 3>(0, 0) = q.toRotationMatrix();
    out(0, 3) = pose.position.x;
    out(1, 3) = pose.position.y;
    out(2, 3) = pose.position.z;
    return out.allFinite();
}

bool GetRawLocalKeyFramePose(
    const RawMapDatabase& raw_db,
    const RawKeyFrameId& keyframe_id,
    Eigen::Matrix4d& local_T_kf)
{
    const auto* keyframe = raw_db.GetKeyFrame(keyframe_id);
    if (!keyframe || keyframe->is_bad)
    {
        return false;
    }
    return PoseMsgToMatrix(keyframe->pose, local_T_kf);
}

struct MapPointPublishingKeyFrameSelection
{
    std::optional<RawKeyFrameId> keyframe_id;
    bool touches_server_corrected_keyframe = false;
    bool selected_server_corrected_keyframe = false;
};

struct FusedPositionAccumulator
{
    Eigen::Vector3d weighted_sum = Eigen::Vector3d::Zero();
    double weight_sum = 0.0;
    uint64_t observations = 0;
};

MapPointPublishingKeyFrameSelection SelectMapPointPublishingKeyFrame(
    const RawSubmapId& submap_id,
    const RawMapDatabase& raw_db,
    const GlobalPoseStore& pose_store,
    const orbslam3_msgs::msg::OrbMapPoint& mappoint)
{
    MapPointPublishingKeyFrameSelection selection;
    std::optional<RawKeyFrameId> first_usable_uncorrected;

    auto evaluate_candidate = [&](uint64_t local_kf_id) -> bool
    {
        const RawKeyFrameId keyframe_id{
            submap_id.drone_id,
            submap_id.map_epoch,
            local_kf_id};
        const bool has_server_correction =
            static_cast<bool>(pose_store.GetKeyFrameServerCorrection(keyframe_id));
        selection.touches_server_corrected_keyframe =
            selection.touches_server_corrected_keyframe || has_server_correction;

        if (!raw_db.HasKeyFrame(keyframe_id) || !pose_store.GetWorldPose(keyframe_id))
        {
            return false;
        }
        Eigen::Matrix4d raw_local_T_kf = Eigen::Matrix4d::Identity();
        if (!GetRawLocalKeyFramePose(raw_db, keyframe_id, raw_local_T_kf))
        {
            return false;
        }
        if (has_server_correction)
        {
            selection.keyframe_id = keyframe_id;
            selection.selected_server_corrected_keyframe = true;
            return true;
        }
        if (!first_usable_uncorrected)
        {
            first_usable_uncorrected = keyframe_id;
        }
        return false;
    };

    if (evaluate_candidate(mappoint.reference_keyframe_id))
    {
        return selection;
    }

    // Se prioriza un KF corregido, pero cualquier observador con pose world y
    // pose raw valida permite proyectar el punto sin recurrir al anchor rigido.
    for (const auto& observation : mappoint.observations)
    {
        if (evaluate_candidate(observation.keyframe_id))
        {
            return selection;
        }
    }

    selection.keyframe_id = first_usable_uncorrected;
    return selection;
}

}  // namespace

GlobalMapBuildResult GlobalMapBuilder::Build(
    const RawMapDatabase& raw_db,
    const GlobalPoseStore& pose_store,
    const LandmarkScoreManager& score_manager,
    FusedLandmarkManager* fused_landmark_manager,
    float min_score_to_publish) const
{
    // F1F: se recorren submapas completos porque la decisión clave es si el
    // submapa está anclado. Submapas sin anchor se saltan enteros para evitar
    // publicar puntos sin frame global válido.
    GlobalMapBuildResult result;
    min_score_to_publish = std::max(0.0F, min_score_to_publish);

    const auto submap_ids = raw_db.GetSubmapIds();
    result.stats.total_submaps = submap_ids.size();

    double score_sum = 0.0;
    bool first_score = true;
    std::map<uint64_t, FusedPositionAccumulator> fused_accumulators;

    for (const auto& submap_id : submap_ids)
    {
        const auto world_T_local = pose_store.GetSubmapWorldTransform(submap_id);
        if (!world_T_local)
        {
            ++result.stats.skipped_unanchored_submaps;
            continue;
        }
        ++result.stats.anchored_submaps;

        const auto mappoint_ids = raw_db.GetMapPointIdsForSubmap(submap_id);
        result.stats.raw_points += mappoint_ids.size();

        for (const auto& mappoint_id : mappoint_ids)
        {
            const auto* mappoint = raw_db.GetMapPoint(mappoint_id);
            if (!mappoint)
            {
                continue;
            }
            if (mappoint->is_bad)
            {
                ++result.stats.bad_skipped;
                continue;
            }

            const float score = score_manager.GetScoreOrDefault(mappoint_id);

            const Eigen::Vector4d local_point(
                mappoint->position.x,
                mappoint->position.y,
                mappoint->position.z,
                1.0);
            if (!local_point.allFinite())
            {
                ++result.stats.invalid_pose_skipped;
                continue;
            }

            Eigen::Vector4d world_point = Eigen::Vector4d::Zero();

            const auto publishing_selection =
                SelectMapPointPublishingKeyFrame(
                    submap_id,
                    raw_db,
                    pose_store,
                    *mappoint);
            if (publishing_selection.touches_server_corrected_keyframe)
            {
                ++result.stats.server_corrected_mappoint_candidates;
            }
            if (publishing_selection.keyframe_id)
            {
                Eigen::Matrix4d raw_local_T_kf = Eigen::Matrix4d::Identity();
                const auto final_world_T_kf =
                    pose_store.GetWorldPose(publishing_selection.keyframe_id.value());
                if (!final_world_T_kf ||
                    !GetRawLocalKeyFramePose(
                        raw_db,
                        publishing_selection.keyframe_id.value(),
                        raw_local_T_kf))
                {
                    ++result.stats.invalid_pose_skipped;
                    continue;
                }

                const Eigen::Vector4d kf_point =
                    raw_local_T_kf.inverse() * local_point;
                world_point = final_world_T_kf.value() * kf_point;
                ++result.stats.keyframe_projected_points;
            }
            else
            {
                ++result.stats.server_corrected_missing_keyframe_skipped;
                ++result.stats.invalid_pose_skipped;
                continue;
            }
            if (!world_point.allFinite())
            {
                ++result.stats.invalid_pose_skipped;
                continue;
            }
            if (publishing_selection.selected_server_corrected_keyframe)
            {
                ++result.stats.server_corrected_points;
            }

            const auto fused_track_id =
                fused_landmark_manager
                    ? fused_landmark_manager->GetTrackIdForMember(mappoint_id)
                    : std::optional<uint64_t>{};
            if (fused_track_id)
            {
                const double weight =
                    std::max(1e-6, static_cast<double>(score)) *
                    static_cast<double>(std::max(mappoint->observations_count, 1U));
                auto& accumulator = fused_accumulators[fused_track_id.value()];
                accumulator.weighted_sum += weight * world_point.head<3>();
                accumulator.weight_sum += weight;
                accumulator.observations +=
                    std::max<uint32_t>(mappoint->observations_count, 1U);
                ++result.stats.fused_members_skipped;
                continue;
            }

            if (score < min_score_to_publish)
            {
                ++result.stats.below_score_skipped;
                continue;
            }

            ++result.stats.candidate_points;

            GlobalSparsePoint point;
            point.global_mappoint_id = MakeGlobalMapPointId(mappoint_id);
            point.drone_id = mappoint_id.drone_id;
            point.map_epoch = mappoint_id.map_epoch;
            point.local_mappoint_id = mappoint_id.local_mp_id;
            point.x = world_point.x();
            point.y = world_point.y();
            point.z = world_point.z();
            point.score = score;
            point.observations = mappoint->observations_count;
            point.from_anchored_submap = true;
            point.is_fused = false;
            result.points.push_back(point);

            if (first_score)
            {
                result.stats.score_min = score;
                result.stats.score_max = score;
                first_score = false;
            }
            else
            {
                result.stats.score_min = std::min(result.stats.score_min, score);
                result.stats.score_max = std::max(result.stats.score_max, score);
            }
            score_sum += static_cast<double>(score);
        }
    }

    result.stats.fused_tracks_considered = fused_accumulators.size();
    for (const auto& [track_id, accumulator] : fused_accumulators)
    {
        const auto track = fused_landmark_manager
            ? fused_landmark_manager->GetTrack(track_id)
            : std::optional<FusedLandmarkTrack>{};
        if (!track || accumulator.weight_sum <= 0.0)
        {
            ++result.stats.invalid_pose_skipped;
            continue;
        }
        if (track->score < min_score_to_publish)
        {
            ++result.stats.below_score_skipped;
            continue;
        }

        const Eigen::Vector3d fused_position =
            accumulator.weighted_sum / accumulator.weight_sum;
        if (!fused_position.allFinite())
        {
            ++result.stats.invalid_pose_skipped;
            continue;
        }
        fused_landmark_manager->UpdateFusedPosition(track_id, fused_position);

        GlobalSparsePoint point;
        point.global_mappoint_id = (1ULL << 63U) | track_id;
        if (!track->member_mappoint_ids.empty())
        {
            const auto& representative = *track->member_mappoint_ids.begin();
            point.drone_id = representative.drone_id;
            point.map_epoch = representative.map_epoch;
        }
        point.local_mappoint_id = track_id;
        point.x = fused_position.x();
        point.y = fused_position.y();
        point.z = fused_position.z();
        point.score = track->score;
        point.observations = static_cast<uint32_t>(
            std::min<uint64_t>(
                accumulator.observations,
                std::numeric_limits<uint32_t>::max()));
        point.from_anchored_submap = true;
        point.is_fused = true;
        result.points.push_back(point);
        ++result.stats.candidate_points;
        ++result.stats.fused_tracks_published;

        if (first_score)
        {
            result.stats.score_min = track->score;
            result.stats.score_max = track->score;
            first_score = false;
        }
        else
        {
            result.stats.score_min =
                std::min(result.stats.score_min, track->score);
            result.stats.score_max =
                std::max(result.stats.score_max, track->score);
        }
        score_sum += static_cast<double>(track->score);
    }

    result.stats.returned_points = result.points.size();
    if (!result.points.empty())
    {
        result.stats.score_mean =
            static_cast<float>(score_sum / static_cast<double>(result.points.size()));
    }
    return result;
}

uint64_t GlobalMapBuilder::MakeGlobalMapPointId(const RawMapPointId& id) const
{
    // F1F: ID compacto para PointCloud/logs. La identidad canonica sigue siendo
    // `(drone_id, map_epoch, local_mp_id)`.
    return static_cast<uint64_t>(id.drone_id) * 1000000000000ULL +
           id.map_epoch * 1000000ULL +
           id.local_mp_id;
}

}  // namespace orbslam3_multi
