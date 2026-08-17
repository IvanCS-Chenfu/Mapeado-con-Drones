#include "orbslam3_multi/subcloud_loop_verifier.hpp"

#include "orbslam3_msgs/msg/orb_key_frame.hpp"
#include "orbslam3_msgs/msg/orb_map_point.hpp"

#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <geometry_msgs/msg/pose.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <random>
#include <set>
#include <sstream>
#include <unordered_map>
#include <vector>

namespace orbslam3_multi
{
namespace
{
using OrbKeyFrame = orbslam3_msgs::msg::OrbKeyFrame;
using OrbMapPoint = orbslam3_msgs::msg::OrbMapPoint;

RawSubmapId ToSubmapId(const RawKeyFrameId& id)
{
    return RawSubmapId{id.drone_id, id.map_epoch};
}

bool IsFiniteVector(const Eigen::Vector3d& value)
{
    return value.allFinite();
}

bool IsFiniteTransform(const Eigen::Matrix4d& value)
{
    return value.allFinite();
}

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
    return IsFiniteTransform(out);
}

double NormalizeAngle(double angle)
{
    constexpr double kPi = 3.14159265358979323846;
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

double YawFromTransform(const Eigen::Matrix4d& transform)
{
    return std::atan2(transform(1, 0), transform(0, 0));
}

double RotationErrorRad(const Eigen::Matrix3d& rotation)
{
    Eigen::AngleAxisd aa(rotation);
    return std::abs(NormalizeAngle(aa.angle()));
}

bool DescriptorValid(const std::array<uint8_t, 32>& descriptor)
{
    for (const auto value : descriptor)
    {
        if (value != 0U)
        {
            return true;
        }
    }
    return false;
}

std::array<uint8_t, 32> DescriptorFromMapPoint(const OrbMapPoint& mappoint)
{
    std::array<uint8_t, 32> descriptor{};
    std::copy(
        mappoint.descriptor.data.begin(),
        mappoint.descriptor.data.end(),
        descriptor.begin());
    return descriptor;
}

std::array<uint8_t, 32> DescriptorForObservation(
    const OrbKeyFrame& keyframe,
    const OrbMapPoint& mappoint,
    size_t keypoint_index)
{
    if (keypoint_index < keyframe.keypoints.size())
    {
        std::array<uint8_t, 32> descriptor{};
        std::copy(
            keyframe.keypoints[keypoint_index].descriptor.data.begin(),
            keyframe.keypoints[keypoint_index].descriptor.data.end(),
            descriptor.begin());
        if (DescriptorValid(descriptor))
        {
            return descriptor;
        }
    }
    return DescriptorFromMapPoint(mappoint);
}

uint32_t HammingDistance(
    const std::array<uint8_t, 32>& a,
    const std::array<uint8_t, 32>& b)
{
    uint32_t distance = 0;
    for (size_t index = 0; index < a.size(); ++index)
    {
        distance += static_cast<uint32_t>(
            __builtin_popcount(static_cast<unsigned int>(a[index] ^ b[index])));
    }
    return distance;
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

bool TransformMapPointThroughKeyFrame(
    const RawMapDatabase& raw_db,
    const GlobalPoseStore& pose_store,
    const RawKeyFrameId& source_kf_id,
    const OrbMapPoint& mappoint,
    Eigen::Vector3d& position_world)
{
    const auto world_T_kf = pose_store.GetWorldPose(source_kf_id);
    if (!world_T_kf || !IsFiniteTransform(*world_T_kf))
    {
        return false;
    }

    Eigen::Matrix4d local_T_kf = Eigen::Matrix4d::Identity();
    if (!GetRawLocalKeyFramePose(raw_db, source_kf_id, local_T_kf))
    {
        return false;
    }

    const Eigen::Vector4d local_point(
        mappoint.position.x,
        mappoint.position.y,
        mappoint.position.z,
        1.0);
    if (!local_point.allFinite())
    {
        return false;
    }

    const Eigen::Vector4d kf_point = local_T_kf.inverse() * local_point;
    const Eigen::Vector4d world_point = world_T_kf.value() * kf_point;
    if (!world_point.allFinite() || std::abs(world_point.w()) < 1e-9)
    {
        return false;
    }

    position_world = world_point.head<3>() / world_point.w();
    return IsFiniteVector(position_world);
}

void UpdateBoundingBox(Subcloud& subcloud)
{
    if (subcloud.points.empty())
    {
        subcloud.bbox_min.setZero();
        subcloud.bbox_max.setZero();
        return;
    }

    subcloud.bbox_min = subcloud.points.front().position_world;
    subcloud.bbox_max = subcloud.points.front().position_world;
    for (const auto& point : subcloud.points)
    {
        subcloud.bbox_min = subcloud.bbox_min.cwiseMin(point.position_world);
        subcloud.bbox_max = subcloud.bbox_max.cwiseMax(point.position_world);
    }
}

bool AddObservationPoint(
    const RawMapDatabase& raw_db,
    const GlobalPoseStore& pose_store,
    const LandmarkScoreManager* score_manager,
    const RawKeyFrameId& source_kf_id,
    const OrbKeyFrame& source_kf,
    size_t keypoint_index,
    SubcloudPointSource source,
    float min_score,
    std::set<RawMapPointId>& seen_mappoints,
    Subcloud& subcloud)
{
    ++subcloud.stats.raw_points;
    if (keypoint_index >= source_kf.mappoint_ids.size())
    {
        return false;
    }

    const RawMapPointId mappoint_id{
        source_kf_id.drone_id,
        source_kf_id.map_epoch,
        source_kf.mappoint_ids[keypoint_index]};
    const auto* mappoint = raw_db.GetMapPoint(mappoint_id);
    if (!mappoint)
    {
        return false;
    }
    if (mappoint->is_bad)
    {
        ++subcloud.stats.bad_removed;
        return false;
    }

    const float score =
        score_manager ? score_manager->GetScoreOrDefault(mappoint_id) : 0.0F;
    if (score < min_score)
    {
        ++subcloud.stats.low_score_removed;
        return false;
    }

    auto descriptor = DescriptorForObservation(source_kf, *mappoint, keypoint_index);
    if (!DescriptorValid(descriptor))
    {
        ++subcloud.stats.no_descriptor_removed;
        return false;
    }

    if (seen_mappoints.find(mappoint_id) != seen_mappoints.end())
    {
        ++subcloud.stats.duplicates_removed;
        return false;
    }

    Eigen::Vector3d position_world = Eigen::Vector3d::Zero();
    if (!TransformMapPointThroughKeyFrame(
            raw_db,
            pose_store,
            source_kf_id,
            *mappoint,
            position_world))
    {
        ++subcloud.stats.no_pose_removed;
        return false;
    }

    seen_mappoints.insert(mappoint_id);
    SubcloudPoint point;
    point.mappoint_id = mappoint_id;
    point.source_kf_id = source_kf_id;
    point.submap_id = ToSubmapId(source_kf_id);
    point.position_world = position_world;
    point.descriptor = descriptor;
    point.score = score;
    point.num_observations = mappoint->observations_count;
    point.is_bad = mappoint->is_bad;
    point.keypoint_index = static_cast<uint32_t>(keypoint_index);
    if (keypoint_index < source_kf.keypoints.size())
    {
        point.keypoint_u = source_kf.keypoints[keypoint_index].u;
        point.keypoint_v = source_kf.keypoints[keypoint_index].v;
        point.keypoint_valid = true;
    }
    point.source = source;
    subcloud.points.push_back(point);
    subcloud.submap_ids.insert(point.submap_id);
    subcloud.source_kf_ids.insert(source_kf_id);
    ++subcloud.stats.filtered_points;
    subcloud.stats.final_points = subcloud.points.size();
    return true;
}

[[maybe_unused]] Subcloud BuildQuerySubcloud(
    const RawKeyFrameId& query_kf_id,
    const RawMapDatabase& raw_db,
    const GlobalPoseStore& pose_store,
    const LandmarkScoreManager* score_manager)
{
    Subcloud subcloud;
    subcloud.cloud_id = "query";
    subcloud.type = SubcloudType::Query;
    subcloud.center_kf_id = query_kf_id;
    subcloud.seed_kf_id = query_kf_id;

    const auto* query_kf = raw_db.GetKeyFrame(query_kf_id);
    if (!query_kf)
    {
        return subcloud;
    }

    std::set<RawMapPointId> seen_mappoints;
    for (size_t index = 0; index < query_kf->mappoint_ids.size(); ++index)
    {
        AddObservationPoint(
            raw_db,
            pose_store,
            score_manager,
            query_kf_id,
            *query_kf,
            index,
            SubcloudPointSource::QueryKeyFrame,
            0.0F,
            seen_mappoints,
            subcloud);
    }
    UpdateBoundingBox(subcloud);
    return subcloud;
}

void AddWindowKeyFrame(
    const RawKeyFrameId& keyframe_id,
    uint64_t& count,
    std::set<RawKeyFrameId>& window,
    const SubcloudLoopVerifierConfig& config)
{
    if (window.size() >= config.candidate_window_max_kfs)
    {
        return;
    }
    if (window.insert(keyframe_id).second)
    {
        ++count;
    }
}

std::vector<RawKeyFrameId> BuildCandidateWindow(
    const RawKeyFrameId& seed_kf_id,
    const RawMapDatabase& raw_db,
    const GlobalPoseStore& pose_store,
    const SubcloudLoopVerifierConfig& config,
    uint64_t& covisible_added,
    uint64_t& tree_added,
    uint64_t& temporal_added,
    uint64_t& spatial_added)
{
    covisible_added = 0;
    tree_added = 0;
    temporal_added = 0;
    spatial_added = 0;

    std::set<RawKeyFrameId> window;
    uint64_t seed_added = 0;
    AddWindowKeyFrame(seed_kf_id, seed_added, window, config);

    const auto* seed_kf = raw_db.GetKeyFrame(seed_kf_id);
    if (!seed_kf)
    {
        return {window.begin(), window.end()};
    }

    for (size_t index = 0;
         index < seed_kf->connected_keyframe_ids.size() &&
         index < seed_kf->connected_keyframe_weights.size();
         ++index)
    {
        if (seed_kf->connected_keyframe_weights[index] <
            config.candidate_window_covisibility_min_weight)
        {
            continue;
        }
        AddWindowKeyFrame(
            RawKeyFrameId{
                seed_kf_id.drone_id,
                seed_kf_id.map_epoch,
                seed_kf->connected_keyframe_ids[index]},
            covisible_added,
            window,
            config);
    }

    if (seed_kf->parent_keyframe_id != seed_kf->id)
    {
        AddWindowKeyFrame(
            RawKeyFrameId{
                seed_kf_id.drone_id,
                seed_kf_id.map_epoch,
                seed_kf->parent_keyframe_id},
            tree_added,
            window,
            config);
    }
    for (const auto child_id : seed_kf->child_keyframe_ids)
    {
        AddWindowKeyFrame(
            RawKeyFrameId{seed_kf_id.drone_id, seed_kf_id.map_epoch, child_id},
            tree_added,
            window,
            config);
    }

    const RawSubmapId submap_id = ToSubmapId(seed_kf_id);
    const uint64_t temporal_radius = config.candidate_window_temporal_kf_radius;
    for (const auto& neighbor_id : raw_db.GetKeyFrameIdsForSubmap(submap_id))
    {
        const uint64_t gap =
            seed_kf_id.local_kf_id > neighbor_id.local_kf_id ?
            seed_kf_id.local_kf_id - neighbor_id.local_kf_id :
            neighbor_id.local_kf_id - seed_kf_id.local_kf_id;
        if (gap <= temporal_radius)
        {
            AddWindowKeyFrame(neighbor_id, temporal_added, window, config);
        }
    }

    const auto seed_world_T_kf = pose_store.GetWorldPose(seed_kf_id);
    if (seed_world_T_kf && IsFiniteTransform(*seed_world_T_kf))
    {
        const Eigen::Vector3d seed_t = seed_world_T_kf->block<3, 1>(0, 3);
        for (const auto& neighbor_id : raw_db.GetKeyFrameIdsForSubmap(submap_id))
        {
            const auto neighbor_world_T_kf = pose_store.GetWorldPose(neighbor_id);
            if (!neighbor_world_T_kf || !IsFiniteTransform(*neighbor_world_T_kf))
            {
                continue;
            }
            const double distance =
                (neighbor_world_T_kf->block<3, 1>(0, 3) - seed_t).norm();
            if (distance <= config.candidate_window_spatial_radius_m)
            {
                AddWindowKeyFrame(neighbor_id, spatial_added, window, config);
            }
        }
    }

    return {window.begin(), window.end()};
}

[[maybe_unused]] Subcloud BuildCandidateSubcloud(
    const RawKeyFrameId& query_kf_id,
    const RawKeyFrameId& seed_kf_id,
    const std::vector<RawKeyFrameId>& window,
    const RawMapDatabase& raw_db,
    const GlobalPoseStore& pose_store,
    const LandmarkScoreManager* score_manager,
    const SubcloudLoopVerifierConfig& config)
{
    Subcloud subcloud;
    subcloud.cloud_id = "candidate_initial";
    subcloud.type = SubcloudType::CandidateInitial;
    subcloud.center_kf_id = query_kf_id;
    subcloud.seed_kf_id = seed_kf_id;

    std::set<RawMapPointId> seen_mappoints;
    for (const auto& source_kf_id : window)
    {
        const auto* source_kf = raw_db.GetKeyFrame(source_kf_id);
        if (!source_kf || source_kf->is_bad)
        {
            continue;
        }
        for (size_t index = 0; index < source_kf->mappoint_ids.size(); ++index)
        {
            if (subcloud.points.size() >= config.candidate_subcloud_max_points)
            {
                break;
            }
            AddObservationPoint(
                raw_db,
                pose_store,
                score_manager,
                source_kf_id,
                *source_kf,
                index,
                SubcloudPointSource::CandidateWindow,
                config.candidate_subcloud_min_score,
                seen_mappoints,
                subcloud);
        }
    }
    UpdateBoundingBox(subcloud);
    return subcloud;
}

const OrbKeyFrame* GetCapturedKeyFrame(
    const CapturedLoopVerification& captured,
    const RawKeyFrameId& keyframe_id)
{
    const auto it = captured.keyframes.find(keyframe_id);
    return it == captured.keyframes.end() ? nullptr : &it->second;
}

const OrbMapPoint* GetCapturedMapPoint(
    const CapturedLoopVerification& captured,
    const RawMapPointId& mappoint_id)
{
    const auto it = captured.mappoints.find(mappoint_id);
    return it == captured.mappoints.end() ? nullptr : &it->second;
}

bool TransformCapturedMapPoint(
    const CapturedLoopVerification& captured,
    const RawKeyFrameId& source_kf_id,
    const OrbMapPoint& mappoint,
    Eigen::Vector3d& position_world)
{
    const auto world_pose_it = captured.world_T_keyframes.find(source_kf_id);
    const auto* source_kf = GetCapturedKeyFrame(captured, source_kf_id);
    if (world_pose_it == captured.world_T_keyframes.end() ||
        !IsFiniteTransform(world_pose_it->second) ||
        !source_kf ||
        source_kf->is_bad)
    {
        return false;
    }

    Eigen::Matrix4d local_T_kf = Eigen::Matrix4d::Identity();
    if (!PoseMsgToMatrix(source_kf->pose, local_T_kf))
    {
        return false;
    }

    const Eigen::Vector4d local_point(
        mappoint.position.x,
        mappoint.position.y,
        mappoint.position.z,
        1.0);
    if (!local_point.allFinite())
    {
        return false;
    }

    const Eigen::Vector4d kf_point = local_T_kf.inverse() * local_point;
    const Eigen::Vector4d world_point = world_pose_it->second * kf_point;
    if (!world_point.allFinite() || std::abs(world_point.w()) < 1e-9)
    {
        return false;
    }

    position_world = world_point.head<3>() / world_point.w();
    return IsFiniteVector(position_world);
}

bool AddCapturedObservationPoint(
    const CapturedLoopVerification& captured,
    const RawKeyFrameId& source_kf_id,
    const OrbKeyFrame& source_kf,
    size_t keypoint_index,
    SubcloudPointSource source,
    float min_score,
    std::set<RawMapPointId>& seen_mappoints,
    Subcloud& subcloud)
{
    ++subcloud.stats.raw_points;
    if (keypoint_index >= source_kf.mappoint_ids.size())
    {
        return false;
    }

    const RawMapPointId mappoint_id{
        source_kf_id.drone_id,
        source_kf_id.map_epoch,
        source_kf.mappoint_ids[keypoint_index]};
    const auto* mappoint = GetCapturedMapPoint(captured, mappoint_id);
    if (!mappoint)
    {
        return false;
    }
    if (mappoint->is_bad)
    {
        ++subcloud.stats.bad_removed;
        return false;
    }

    const auto score_it = captured.scores.find(mappoint_id);
    const float score =
        score_it == captured.scores.end() ? 0.0F : score_it->second;
    if (score < min_score)
    {
        ++subcloud.stats.low_score_removed;
        return false;
    }

    auto descriptor =
        DescriptorForObservation(source_kf, *mappoint, keypoint_index);
    if (!DescriptorValid(descriptor))
    {
        ++subcloud.stats.no_descriptor_removed;
        return false;
    }
    if (seen_mappoints.find(mappoint_id) != seen_mappoints.end())
    {
        ++subcloud.stats.duplicates_removed;
        return false;
    }

    Eigen::Vector3d position_world = Eigen::Vector3d::Zero();
    if (!TransformCapturedMapPoint(
            captured,
            source_kf_id,
            *mappoint,
            position_world))
    {
        ++subcloud.stats.no_pose_removed;
        return false;
    }

    seen_mappoints.insert(mappoint_id);
    SubcloudPoint point;
    point.mappoint_id = mappoint_id;
    point.source_kf_id = source_kf_id;
    point.submap_id = ToSubmapId(source_kf_id);
    point.position_world = position_world;
    point.descriptor = descriptor;
    point.score = score;
    point.num_observations = mappoint->observations_count;
    point.is_bad = mappoint->is_bad;
    point.keypoint_index = static_cast<uint32_t>(keypoint_index);
    if (keypoint_index < source_kf.keypoints.size())
    {
        point.keypoint_u = source_kf.keypoints[keypoint_index].u;
        point.keypoint_v = source_kf.keypoints[keypoint_index].v;
        point.keypoint_valid = true;
    }
    point.source = source;
    subcloud.points.push_back(point);
    subcloud.submap_ids.insert(point.submap_id);
    subcloud.source_kf_ids.insert(source_kf_id);
    ++subcloud.stats.filtered_points;
    subcloud.stats.final_points = subcloud.points.size();
    return true;
}

Subcloud BuildCapturedQuerySubcloud(
    const CapturedLoopVerification& captured)
{
    Subcloud subcloud;
    subcloud.cloud_id = "query";
    subcloud.type = SubcloudType::Query;
    subcloud.center_kf_id = captured.candidate.query_kf_id;
    subcloud.seed_kf_id = captured.candidate.query_kf_id;

    const auto* query_kf =
        GetCapturedKeyFrame(captured, captured.candidate.query_kf_id);
    if (!query_kf)
    {
        return subcloud;
    }

    std::set<RawMapPointId> seen_mappoints;
    for (size_t index = 0; index < query_kf->mappoint_ids.size(); ++index)
    {
        AddCapturedObservationPoint(
            captured,
            captured.candidate.query_kf_id,
            *query_kf,
            index,
            SubcloudPointSource::QueryKeyFrame,
            0.0F,
            seen_mappoints,
            subcloud);
    }
    UpdateBoundingBox(subcloud);
    return subcloud;
}

Subcloud BuildCapturedCandidateSubcloud(
    const CapturedLoopVerification& captured,
    const SubcloudLoopVerifierConfig& config)
{
    Subcloud subcloud;
    subcloud.cloud_id = "candidate_initial";
    subcloud.type = SubcloudType::CandidateInitial;
    subcloud.center_kf_id = captured.candidate.query_kf_id;
    subcloud.seed_kf_id = captured.candidate.candidate_kf_id;

    std::set<RawMapPointId> seen_mappoints;
    for (const auto& source_kf_id : captured.candidate_window)
    {
        const auto* source_kf = GetCapturedKeyFrame(captured, source_kf_id);
        if (!source_kf || source_kf->is_bad)
        {
            continue;
        }
        for (size_t index = 0; index < source_kf->mappoint_ids.size(); ++index)
        {
            if (subcloud.points.size() >= config.candidate_subcloud_max_points)
            {
                break;
            }
            AddCapturedObservationPoint(
                captured,
                source_kf_id,
                *source_kf,
                index,
                SubcloudPointSource::CandidateWindow,
                config.candidate_subcloud_min_score,
                seen_mappoints,
                subcloud);
        }
    }
    UpdateBoundingBox(subcloud);
    return subcloud;
}

std::vector<DescriptorMatch> MatchSubclouds(
    const Subcloud& query,
    const Subcloud& candidate,
    const SubcloudLoopVerifierConfig& config,
    uint64_t& duplicates_removed,
    double& mean_descriptor_distance)
{
    duplicates_removed = 0;
    mean_descriptor_distance = 0.0;
    std::vector<DescriptorMatch> tentative;

    for (size_t qi = 0; qi < query.points.size(); ++qi)
    {
        uint32_t best_distance = std::numeric_limits<uint32_t>::max();
        uint32_t second_distance = std::numeric_limits<uint32_t>::max();
        size_t best_index = 0;

        for (size_t ci = 0; ci < candidate.points.size(); ++ci)
        {
            const uint32_t distance =
                HammingDistance(query.points[qi].descriptor, candidate.points[ci].descriptor);
            if (distance < best_distance)
            {
                second_distance = best_distance;
                best_distance = distance;
                best_index = ci;
            }
            else if (distance < second_distance)
            {
                second_distance = distance;
            }
        }

        if (best_distance > config.orb_match_max_hamming)
        {
            continue;
        }
        if (second_distance != std::numeric_limits<uint32_t>::max() &&
            static_cast<double>(best_distance) >
                config.orb_match_ratio_test * static_cast<double>(second_distance))
        {
            continue;
        }

        DescriptorMatch match;
        match.query_index = qi;
        match.candidate_index = best_index;
        match.query_mappoint_id = query.points[qi].mappoint_id;
        match.candidate_mappoint_id = candidate.points[best_index].mappoint_id;
        match.query_position_world = query.points[qi].position_world;
        match.candidate_position_world = candidate.points[best_index].position_world;
        match.descriptor_distance = best_distance;
        tentative.push_back(match);
    }

    std::stable_sort(
        tentative.begin(),
        tentative.end(),
        [](const DescriptorMatch& a, const DescriptorMatch& b) {
            if (a.descriptor_distance != b.descriptor_distance)
            {
                return a.descriptor_distance < b.descriptor_distance;
            }
            return a.query_index < b.query_index;
        });

    std::set<uint64_t> used_query;
    std::set<uint64_t> used_candidate;
    std::vector<DescriptorMatch> matches;
    for (const auto& match : tentative)
    {
        if (used_query.find(match.query_index) != used_query.end() ||
            (config.orb_match_cross_check &&
             used_candidate.find(match.candidate_index) != used_candidate.end()))
        {
            ++duplicates_removed;
            continue;
        }
        used_query.insert(match.query_index);
        used_candidate.insert(match.candidate_index);
        mean_descriptor_distance += static_cast<double>(match.descriptor_distance);
        matches.push_back(match);
    }

    if (!matches.empty())
    {
        mean_descriptor_distance /= static_cast<double>(matches.size());
    }
    return matches;
}

double Percentile(std::vector<double> values, double percentile)
{
    if (values.empty())
    {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const double clamped = std::max(0.0, std::min(100.0, percentile));
    const double position =
        (clamped / 100.0) * static_cast<double>(values.size() - 1U);
    const auto low = static_cast<size_t>(std::floor(position));
    const auto high = static_cast<size_t>(std::ceil(position));
    if (low == high)
    {
        return values[low];
    }
    const double alpha = position - static_cast<double>(low);
    return values[low] * (1.0 - alpha) + values[high] * alpha;
}

Subcloud ReduceCandidateSubcloud(
    const Subcloud& initial,
    const std::vector<DescriptorMatch>& initial_matches,
    const SubcloudLoopVerifierConfig& config,
    bool& fallback,
    std::string& reason,
    Eigen::Vector3d& box_min,
    Eigen::Vector3d& box_max)
{
    fallback = false;
    reason = "robust_match_box";
    Subcloud reduced = initial;
    reduced.cloud_id = "candidate_reduced";
    reduced.type = SubcloudType::CandidateReduced;
    reduced.points.clear();
    reduced.stats = SubcloudStats{};

    box_min.setZero();
    box_max.setZero();

    if (!config.candidate_reduce_enabled)
    {
        fallback = true;
        reason = "disabled";
        return initial;
    }
    if (initial_matches.size() < config.candidate_reduce_min_initial_matches)
    {
        fallback = true;
        reason = "not_enough_matches";
        return initial;
    }

    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> zs;
    xs.reserve(initial_matches.size());
    ys.reserve(initial_matches.size());
    zs.reserve(initial_matches.size());
    for (const auto& match : initial_matches)
    {
        xs.push_back(match.candidate_position_world.x());
        ys.push_back(match.candidate_position_world.y());
        zs.push_back(match.candidate_position_world.z());
    }

    box_min.x() = Percentile(xs, config.candidate_reduce_percentile_low) -
                  config.candidate_reduce_margin_m;
    box_min.y() = Percentile(ys, config.candidate_reduce_percentile_low) -
                  config.candidate_reduce_margin_m;
    box_min.z() = Percentile(zs, config.candidate_reduce_percentile_low) -
                  config.candidate_reduce_margin_m;
    box_max.x() = Percentile(xs, config.candidate_reduce_percentile_high) +
                  config.candidate_reduce_margin_m;
    box_max.y() = Percentile(ys, config.candidate_reduce_percentile_high) +
                  config.candidate_reduce_margin_m;
    box_max.z() = Percentile(zs, config.candidate_reduce_percentile_high) +
                  config.candidate_reduce_margin_m;

    if (!IsFiniteVector(box_min) || !IsFiniteVector(box_max))
    {
        fallback = true;
        reason = "box_not_finite";
        return initial;
    }

    for (const auto& point : initial.points)
    {
        ++reduced.stats.raw_points;
        const bool inside =
            point.position_world.x() >= box_min.x() &&
            point.position_world.y() >= box_min.y() &&
            point.position_world.z() >= box_min.z() &&
            point.position_world.x() <= box_max.x() &&
            point.position_world.y() <= box_max.y() &&
            point.position_world.z() <= box_max.z();
        if (!inside)
        {
            continue;
        }
        reduced.points.push_back(point);
        reduced.submap_ids.insert(point.submap_id);
        reduced.source_kf_ids.insert(point.source_kf_id);
        ++reduced.stats.filtered_points;
    }

    reduced.stats.final_points = reduced.points.size();
    if (reduced.points.size() < config.candidate_reduce_min_points_after)
    {
        fallback = config.candidate_reduce_fallback_to_initial;
        reason = "too_few_points_after_reduce";
        return fallback ? initial : reduced;
    }
    if (reduced.points.size() >= initial.points.size())
    {
        fallback = true;
        reason = "box_not_selective";
        return initial;
    }

    UpdateBoundingBox(reduced);
    return reduced;
}

bool EstimateRigidTransform(
    const std::vector<DescriptorMatch>& matches,
    const std::vector<uint64_t>& indices,
    bool degeneracy_check,
    Eigen::Matrix4d& candidate_T_query,
    bool& degenerate)
{
    degenerate = false;
    if (indices.size() < 3)
    {
        degenerate = true;
        return false;
    }

    Eigen::Vector3d query_centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d candidate_centroid = Eigen::Vector3d::Zero();
    for (const auto index : indices)
    {
        query_centroid += matches[index].query_position_world;
        candidate_centroid += matches[index].candidate_position_world;
    }
    query_centroid /= static_cast<double>(indices.size());
    candidate_centroid /= static_cast<double>(indices.size());

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto index : indices)
    {
        const Eigen::Vector3d q = matches[index].query_position_world - query_centroid;
        const Eigen::Vector3d c =
            matches[index].candidate_position_world - candidate_centroid;
        covariance += c * q.transpose();
    }

    if (!covariance.allFinite())
    {
        degenerate = true;
        return false;
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        covariance,
        Eigen::ComputeFullU | Eigen::ComputeFullV);
    if (degeneracy_check)
    {
        const auto singular_values = svd.singularValues();
        if (!singular_values.allFinite() || singular_values(1) < 1e-9)
        {
            degenerate = true;
            return false;
        }
    }

    Eigen::Matrix3d rotation = svd.matrixU() * svd.matrixV().transpose();
    if (rotation.determinant() < 0.0)
    {
        Eigen::Matrix3d u = svd.matrixU();
        u.col(2) *= -1.0;
        rotation = u * svd.matrixV().transpose();
    }

    const Eigen::Vector3d translation = candidate_centroid - rotation * query_centroid;
    candidate_T_query = Eigen::Matrix4d::Identity();
    candidate_T_query.block<3, 3>(0, 0) = rotation;
    candidate_T_query.block<3, 1>(0, 3) = translation;
    if (!IsFiniteTransform(candidate_T_query))
    {
        degenerate = true;
        return false;
    }
    return true;
}

Ransac3D3DResult RunRansac(
    const std::vector<DescriptorMatch>& matches,
    const SubcloudLoopVerifierConfig& config)
{
    Ransac3D3DResult result;
    result.matches = matches.size();
    if (matches.size() < config.ransac_min_matches)
    {
        result.reason = "not_enough_refined_matches";
        return result;
    }

    std::mt19937 rng(1337U);
    std::uniform_int_distribution<size_t> dist(0, matches.size() - 1U);

    double best_mean = std::numeric_limits<double>::infinity();
    std::vector<uint64_t> best_inliers;
    Eigen::Matrix4d best_transform = Eigen::Matrix4d::Identity();

    for (uint64_t iteration = 0; iteration < config.ransac_max_iterations; ++iteration)
    {
        std::set<uint64_t> sample_set;
        while (sample_set.size() < 3U)
        {
            sample_set.insert(static_cast<uint64_t>(dist(rng)));
        }
        std::vector<uint64_t> sample(sample_set.begin(), sample_set.end());

        bool degenerate = false;
        Eigen::Matrix4d candidate_T_query = Eigen::Matrix4d::Identity();
        if (!EstimateRigidTransform(
                matches,
                sample,
                config.ransac_degeneracy_check_enabled,
                candidate_T_query,
                degenerate))
        {
            result.degenerate = result.degenerate || degenerate;
            continue;
        }

        std::vector<uint64_t> inliers;
        double residual_sum = 0.0;
        for (size_t index = 0; index < matches.size(); ++index)
        {
            const Eigen::Vector3d transformed =
                candidate_T_query.block<3, 3>(0, 0) *
                matches[index].query_position_world +
                candidate_T_query.block<3, 1>(0, 3);
            const double residual =
                (transformed - matches[index].candidate_position_world).norm();
            if (std::isfinite(residual) &&
                residual <= config.ransac_inlier_threshold_m)
            {
                inliers.push_back(static_cast<uint64_t>(index));
                residual_sum += residual;
            }
        }

        const double mean =
            inliers.empty() ? std::numeric_limits<double>::infinity() :
                              residual_sum / static_cast<double>(inliers.size());
        if (inliers.size() > best_inliers.size() ||
            (inliers.size() == best_inliers.size() && mean < best_mean))
        {
            best_inliers = inliers;
            best_mean = mean;
            best_transform = candidate_T_query;
        }
        ++result.iterations;
    }

    if (best_inliers.size() < config.ransac_min_inliers)
    {
        result.reason = "not_enough_inliers";
        result.inliers = best_inliers.size();
        result.inlier_ratio =
            static_cast<double>(best_inliers.size()) / static_cast<double>(matches.size());
        return result;
    }

    bool degenerate = false;
    if (!EstimateRigidTransform(
            matches,
            best_inliers,
            false,
            best_transform,
            degenerate))
    {
        result.degenerate = degenerate;
        result.reason = "refit_failed";
        return result;
    }

    double residual_sum = 0.0;
    double max_residual = 0.0;
    std::vector<uint64_t> final_inliers;
    for (size_t index = 0; index < matches.size(); ++index)
    {
        const Eigen::Vector3d transformed =
            best_transform.block<3, 3>(0, 0) * matches[index].query_position_world +
            best_transform.block<3, 1>(0, 3);
        const double residual =
            (transformed - matches[index].candidate_position_world).norm();
        if (std::isfinite(residual) && residual <= config.ransac_inlier_threshold_m)
        {
            final_inliers.push_back(static_cast<uint64_t>(index));
            residual_sum += residual;
            max_residual = std::max(max_residual, residual);
        }
    }

    result.inliers = final_inliers.size();
    result.inlier_ratio =
        static_cast<double>(result.inliers) / static_cast<double>(matches.size());
    result.mean_residual =
        result.inliers == 0U ? 0.0 : residual_sum / static_cast<double>(result.inliers);
    result.max_residual = max_residual;
    result.estimated_candidate_T_query = best_transform;
    result.inlier_match_indices = final_inliers;

    if (result.inliers < config.ransac_min_inliers)
    {
        result.reason = "not_enough_final_inliers";
        return result;
    }
    if (result.inlier_ratio < config.ransac_min_inlier_ratio)
    {
        result.reason = "low_inlier_ratio";
        return result;
    }

    result.success = true;
    result.reason = "success";
    return result;
}

double ConfidenceFromRansac(
    const Ransac3D3DResult& ransac,
    const SubcloudLoopVerifierConfig& config)
{
    if (!ransac.success)
    {
        return 0.0;
    }
    const double ratio_score =
        std::min(1.0, ransac.inlier_ratio / std::max(1e-9, config.ransac_min_inlier_ratio));
    const double residual_score =
        std::max(0.0, 1.0 - ransac.mean_residual / std::max(1e-9, config.accept_mean_residual_m));
    return std::max(0.0, std::min(1.0, 0.65 * ratio_score + 0.35 * residual_score));
}

Subcloud FilterSubcloud(
    const Subcloud& source,
    const std::set<RawMapPointId>& allowed_ids)
{
    Subcloud filtered = source;
    filtered.points.clear();
    filtered.source_kf_ids.clear();
    filtered.submap_ids.clear();
    for (const auto& point : source.points)
    {
        if (allowed_ids.find(point.mappoint_id) == allowed_ids.end())
        {
            continue;
        }
        filtered.points.push_back(point);
        filtered.source_kf_ids.insert(point.source_kf_id);
        filtered.submap_ids.insert(point.submap_id);
    }
    filtered.stats.final_points = filtered.points.size();
    UpdateBoundingBox(filtered);
    return filtered;
}

std::vector<DescriptorMatch> MatchByPositionAndDescriptor(
    const Subcloud& query,
    const Subcloud& candidate,
    const Eigen::Matrix4d& candidate_T_query,
    double max_position_distance_m,
    uint32_t max_hamming,
    double ratio_test,
    uint64_t& same_raw_ids_skipped,
    uint64_t& same_track_matches,
    const FusedLandmarkManager* fused_landmark_manager)
{
    struct SpatialCell
    {
        int64_t x = 0;
        int64_t y = 0;
        int64_t z = 0;

        bool operator==(const SpatialCell& other) const
        {
            return x == other.x && y == other.y && z == other.z;
        }
    };
    struct SpatialCellHash
    {
        size_t operator()(const SpatialCell& cell) const noexcept
        {
            size_t seed = std::hash<int64_t>{}(cell.x);
            seed ^= std::hash<int64_t>{}(cell.y) +
                    0x9e3779b9U + (seed << 6U) + (seed >> 2U);
            seed ^= std::hash<int64_t>{}(cell.z) +
                    0x9e3779b9U + (seed << 6U) + (seed >> 2U);
            return seed;
        }
    };
    struct RankedMatch
    {
        DescriptorMatch match;
        double position_distance_m = 0.0;
    };

    const double cell_size = std::max(1e-6, max_position_distance_m);
    const auto cell_for =
        [cell_size](const Eigen::Vector3d& position)
        {
            return SpatialCell{
                static_cast<int64_t>(
                    std::floor(position.x() / cell_size)),
                static_cast<int64_t>(
                    std::floor(position.y() / cell_size)),
                static_cast<int64_t>(
                    std::floor(position.z() / cell_size))};
        };
    std::unordered_map<SpatialCell, std::vector<size_t>, SpatialCellHash>
        candidate_cells;
    candidate_cells.reserve(candidate.points.size());
    for (size_t ci = 0; ci < candidate.points.size(); ++ci)
    {
        candidate_cells[cell_for(candidate.points[ci].position_world)]
            .push_back(ci);
    }

    std::vector<RankedMatch> tentative;
    for (size_t qi = 0; qi < query.points.size(); ++qi)
    {
        const Eigen::Vector4d query_h(
            query.points[qi].position_world.x(),
            query.points[qi].position_world.y(),
            query.points[qi].position_world.z(),
            1.0);
        const Eigen::Vector4d predicted_h = candidate_T_query * query_h;
        if (!predicted_h.allFinite() || std::abs(predicted_h.w()) < 1e-9)
        {
            continue;
        }
        const Eigen::Vector3d predicted =
            predicted_h.head<3>() / predicted_h.w();

        uint32_t best_distance = std::numeric_limits<uint32_t>::max();
        uint32_t second_distance = std::numeric_limits<uint32_t>::max();
        double best_position_distance = std::numeric_limits<double>::infinity();
        size_t best_index = 0;
        bool found = false;
        bool best_is_known_identity = false;
        const SpatialCell center_cell = cell_for(predicted);
        for (int64_t dx = -1; dx <= 1; ++dx)
        {
            for (int64_t dy = -1; dy <= 1; ++dy)
            {
                for (int64_t dz = -1; dz <= 1; ++dz)
                {
                    const auto cell_it =
                        candidate_cells.find(
                            SpatialCell{
                                center_cell.x + dx,
                                center_cell.y + dy,
                                center_cell.z + dz});
                    if (cell_it == candidate_cells.end())
                    {
                        continue;
                    }
                    for (const auto ci : cell_it->second)
                    {
                        const double position_distance =
                            (predicted -
                             candidate.points[ci].position_world).norm();
                        if (!std::isfinite(position_distance) ||
                            position_distance >
                                max_position_distance_m)
                        {
                            continue;
                        }

                        const bool same_raw =
                            query.points[qi].mappoint_id ==
                            candidate.points[ci].mappoint_id;
                        const auto query_track = fused_landmark_manager
                            ? fused_landmark_manager->GetTrackIdForMember(
                                  query.points[qi].mappoint_id)
                            : std::optional<uint64_t>{};
                        const auto candidate_track = fused_landmark_manager
                            ? fused_landmark_manager->GetTrackIdForMember(
                                  candidate.points[ci].mappoint_id)
                            : std::optional<uint64_t>{};
                        const bool same_track =
                            query_track && candidate_track &&
                            query_track == candidate_track;
                        if (same_raw || same_track)
                        {
                            if (same_raw)
                            {
                                ++same_raw_ids_skipped;
                            }
                            else
                            {
                                ++same_track_matches;
                            }
                            if (!best_is_known_identity ||
                                position_distance < best_position_distance)
                            {
                                best_distance = 0U;
                                second_distance =
                                    std::numeric_limits<uint32_t>::max();
                                best_position_distance = position_distance;
                                best_index = ci;
                                found = true;
                                best_is_known_identity = true;
                            }
                            continue;
                        }
                        if (best_is_known_identity)
                        {
                            continue;
                        }

                        const uint32_t descriptor_distance =
                            HammingDistance(
                                query.points[qi].descriptor,
                                candidate.points[ci].descriptor);
                        if (descriptor_distance < best_distance ||
                            (descriptor_distance == best_distance &&
                             position_distance <
                                 best_position_distance))
                        {
                            second_distance = best_distance;
                            best_distance = descriptor_distance;
                            best_position_distance = position_distance;
                            best_index = ci;
                            found = true;
                        }
                        else if (descriptor_distance < second_distance)
                        {
                            second_distance = descriptor_distance;
                        }
                    }
                }
            }
        }

        if (!found || best_distance > max_hamming)
        {
            continue;
        }
        if (second_distance != std::numeric_limits<uint32_t>::max() &&
            static_cast<double>(best_distance) >
                ratio_test * static_cast<double>(second_distance))
        {
            continue;
        }

        RankedMatch ranked;
        ranked.match.query_index = qi;
        ranked.match.candidate_index = best_index;
        ranked.match.query_mappoint_id = query.points[qi].mappoint_id;
        ranked.match.candidate_mappoint_id =
            candidate.points[best_index].mappoint_id;
        ranked.match.query_position_world =
            query.points[qi].position_world;
        ranked.match.candidate_position_world =
            candidate.points[best_index].position_world;
        ranked.match.descriptor_distance = best_distance;
        ranked.position_distance_m = best_position_distance;
        tentative.push_back(ranked);
    }

    std::stable_sort(
        tentative.begin(),
        tentative.end(),
        [](const RankedMatch& lhs, const RankedMatch& rhs)
        {
            if (lhs.match.descriptor_distance !=
                rhs.match.descriptor_distance)
            {
                return lhs.match.descriptor_distance <
                       rhs.match.descriptor_distance;
            }
            if (lhs.position_distance_m != rhs.position_distance_m)
            {
                return lhs.position_distance_m < rhs.position_distance_m;
            }
            return lhs.match.query_mappoint_id <
                   rhs.match.query_mappoint_id;
        });

    std::set<uint64_t> used_query;
    std::set<uint64_t> used_candidate;
    std::vector<DescriptorMatch> matches;
    for (const auto& ranked : tentative)
    {
        if (!used_query.insert(ranked.match.query_index).second ||
            !used_candidate.insert(ranked.match.candidate_index).second)
        {
            continue;
        }
        matches.push_back(ranked.match);
    }
    return matches;
}

uint64_t ImageCoverageBins(
    const Subcloud& query,
    const std::vector<DescriptorMatch>& matches)
{
    if (query.points.empty())
    {
        return 0;
    }

    double min_u = std::numeric_limits<double>::infinity();
    double max_u = -std::numeric_limits<double>::infinity();
    double min_v = std::numeric_limits<double>::infinity();
    double max_v = -std::numeric_limits<double>::infinity();
    for (const auto& point : query.points)
    {
        if (!point.keypoint_valid)
        {
            continue;
        }
        min_u = std::min(min_u, point.keypoint_u);
        max_u = std::max(max_u, point.keypoint_u);
        min_v = std::min(min_v, point.keypoint_v);
        max_v = std::max(max_v, point.keypoint_v);
    }
    if (!std::isfinite(min_u) || max_u - min_u < 1e-6 ||
        !std::isfinite(min_v) || max_v - min_v < 1e-6)
    {
        return 0;
    }

    std::set<uint64_t> bins;
    const double center_u = 0.5 * (min_u + max_u);
    const double center_v = 0.5 * (min_v + max_v);
    for (const auto& match : matches)
    {
        if (match.query_index >= query.points.size())
        {
            continue;
        }
        const auto& point = query.points[match.query_index];
        if (!point.keypoint_valid)
        {
            continue;
        }
        const uint64_t bin =
            (point.keypoint_u >= center_u ? 1U : 0U) +
            (point.keypoint_v >= center_v ? 2U : 0U);
        bins.insert(bin);
    }
    return bins.size();
}

double SpatialCoverageRatio(
    const Subcloud& query,
    const std::vector<DescriptorMatch>& matches)
{
    if (query.points.empty() || matches.empty())
    {
        return 0.0;
    }
    Eigen::Vector3d min_point =
        Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector3d max_point =
        Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
    for (const auto& match : matches)
    {
        min_point = min_point.cwiseMin(match.query_position_world);
        max_point = max_point.cwiseMax(match.query_position_world);
    }
    const double full_span = (query.bbox_max - query.bbox_min).norm();
    if (!std::isfinite(full_span) || full_span < 1e-6)
    {
        return 0.0;
    }
    return std::max(
        0.0,
        std::min(1.0, (max_point - min_point).norm() / full_span));
}

void ComputeMatchResiduals(
    const std::vector<DescriptorMatch>& matches,
    const Eigen::Matrix4d& candidate_T_query,
    double& mean_residual,
    double& max_residual)
{
    mean_residual = 0.0;
    max_residual = 0.0;
    for (const auto& match : matches)
    {
        const Eigen::Vector4d query_h(
            match.query_position_world.x(),
            match.query_position_world.y(),
            match.query_position_world.z(),
            1.0);
        const Eigen::Vector4d predicted_h = candidate_T_query * query_h;
        if (!predicted_h.allFinite() || std::abs(predicted_h.w()) < 1e-9)
        {
            max_residual = std::numeric_limits<double>::infinity();
            mean_residual = max_residual;
            return;
        }
        const Eigen::Vector3d predicted =
            predicted_h.head<3>() / predicted_h.w();
        const double residual =
            (predicted - match.candidate_position_world).norm();
        mean_residual += residual;
        max_residual = std::max(max_residual, residual);
    }
    if (!matches.empty())
    {
        mean_residual /= static_cast<double>(matches.size());
    }
}

std::vector<DescriptorMatch> MatchSharedRawIdentities(
    const Subcloud& query,
    const Subcloud& candidate)
{
    std::map<RawMapPointId, uint64_t> candidate_by_id;
    for (uint64_t index = 0; index < candidate.points.size(); ++index)
    {
        candidate_by_id.emplace(candidate.points[index].mappoint_id, index);
    }

    std::vector<DescriptorMatch> matches;
    for (uint64_t query_index = 0; query_index < query.points.size(); ++query_index)
    {
        const auto& query_point = query.points[query_index];
        const auto candidate_it = candidate_by_id.find(query_point.mappoint_id);
        if (candidate_it == candidate_by_id.end())
        {
            continue;
        }
        const auto& candidate_point = candidate.points[candidate_it->second];
        DescriptorMatch match;
        match.query_index = query_index;
        match.candidate_index = candidate_it->second;
        match.query_mappoint_id = query_point.mappoint_id;
        match.candidate_mappoint_id = candidate_point.mappoint_id;
        match.query_position_world = query_point.position_world;
        match.candidate_position_world = candidate_point.position_world;
        match.descriptor_distance = 0;
        matches.push_back(match);
    }
    return matches;
}

LoopVerificationResult BuildAlignedVerification(
    const RawKeyFrameId& query_kf_id,
    const RawKeyFrameId& candidate_kf_id,
    const std::vector<DescriptorMatch>& strict_matches,
    const std::vector<DescriptorMatch>& expanded_matches,
    const Eigen::Matrix4d& candidate_T_query,
    double mean_residual,
    double max_residual,
    const std::string& reason,
    const Subcloud& query,
    const FusedLandmarkManager* fused_landmark_manager)
{
    LoopVerificationResult result;
    result.query_kf_id = query_kf_id;
    result.candidate_seed_kf_id = candidate_kf_id;
    result.query_submap_id = ToSubmapId(query_kf_id);
    result.candidate_submap_id = ToSubmapId(candidate_kf_id);
    result.query_points = strict_matches.size();
    result.initial_matches = strict_matches.size();
    result.refined_matches = expanded_matches.size();
    result.ransac_success = true;
    result.ransac_iterations = 0;
    result.ransac_inliers = expanded_matches.size();
    result.inlier_ratio =
        strict_matches.empty()
            ? 1.0
            : std::min(
                  1.0,
                  static_cast<double>(expanded_matches.size()) /
                  static_cast<double>(strict_matches.size()));
    result.image_coverage_bins = ImageCoverageBins(query, expanded_matches);
    result.spatial_coverage_ratio =
        SpatialCoverageRatio(query, expanded_matches);
    result.mean_residual = mean_residual;
    result.max_residual = max_residual;
    result.estimated_candidate_T_query = candidate_T_query;
    result.relative_pose_measured = candidate_T_query;
    result.error_t =
        candidate_T_query.block<3, 1>(0, 3).norm();
    result.error_yaw =
        std::abs(NormalizeAngle(YawFromTransform(candidate_T_query)));
    result.error_rot =
        RotationErrorRad(candidate_T_query.block<3, 3>(0, 0));
    const double residual_quality =
        std::max(0.0, 1.0 - mean_residual / 0.30);
    result.loop_confidence =
        std::max(0.0, std::min(1.0, 0.60 + 0.40 * residual_quality));
    result.geometry_confirmed = true;
    result.decision = LoopGeometryDecision::FusionCandidate;
    result.reason = reason;
    result.inlier_mappoint_pairs.reserve(expanded_matches.size());
    for (const auto& match : expanded_matches)
    {
        const bool same_raw =
            match.query_mappoint_id == match.candidate_mappoint_id;
        const auto query_track = fused_landmark_manager
            ? fused_landmark_manager->GetTrackIdForMember(
                  match.query_mappoint_id)
            : std::optional<uint64_t>{};
        const auto candidate_track = fused_landmark_manager
            ? fused_landmark_manager->GetTrackIdForMember(
                  match.candidate_mappoint_id)
            : std::optional<uint64_t>{};
        if (same_raw ||
            (query_track && candidate_track && query_track == candidate_track))
        {
            ++result.shared_identity_matches;
            continue;
        }
        result.inlier_mappoint_pairs.push_back(
            {match.query_mappoint_id, match.candidate_mappoint_id});
    }
    return result;
}

}  // namespace

const char* ToString(LoopGeometryDecision decision)
{
    switch (decision)
    {
        case LoopGeometryDecision::Reject:
            return "REJECT";
        case LoopGeometryDecision::HoldInsufficientEvidence:
            return "HOLD_INSUFFICIENT_EVIDENCE";
        case LoopGeometryDecision::FusionCandidate:
            return "FUSION_CANDIDATE";
        case LoopGeometryDecision::LoopOptimizationCandidate:
            return "LOOP_OPTIMIZATION_CANDIDATE";
        case LoopGeometryDecision::AlreadyConfirmedCovisibility:
            return "ALREADY_CONFIRMED_COVISIBILITY";
    }
    return "UNKNOWN";
}

SubcloudLoopVerifier::SubcloudLoopVerifier(const SubcloudLoopVerifierConfig& config)
    : config_(config)
{
}

void SubcloudLoopVerifier::Configure(const SubcloudLoopVerifierConfig& config)
{
    config_ = config;
}

const SubcloudLoopVerifierConfig& SubcloudLoopVerifier::GetConfig() const
{
    return config_;
}

LoopVerificationResult SubcloudLoopVerifier::VerifyCandidate(
    const LoopCandidate& candidate,
    const RawMapDatabase& raw_db,
    const GlobalPoseStore& pose_store,
    const CovisibilityDatabase* covisibility_db,
    const LandmarkScoreManager* score_manager) const
{
    return VerifyPreparedCandidate(
        PrepareCandidate(
            candidate,
            raw_db,
            pose_store,
            covisibility_db,
            score_manager));
}

AlignedOverlapSearchResult SubcloudLoopVerifier::FindUnknownAlignedOverlaps(
    const RawKeyFrameId& query_kf_id,
    const RawMapDatabase& raw_db,
    const GlobalPoseStore& pose_store,
    const CovisibilityDatabase& covisibility_db,
    const LandmarkScoreManager* score_manager,
    const FusedLandmarkManager* fused_landmark_manager) const
{
    AlignedOverlapSearchResult search;
    search.query_kf_id = query_kf_id;
    if (!config_.aligned_overlap_enabled)
    {
        search.reason = "disabled";
        return search;
    }

    const auto query_world_T_kf = pose_store.GetWorldPose(query_kf_id);
    if (!query_world_T_kf || !query_world_T_kf->allFinite())
    {
        search.reason = "query_no_world_pose";
        return search;
    }
    const Subcloud query =
        BuildQuerySubcloud(
            query_kf_id,
            raw_db,
            pose_store,
            score_manager);
    search.query_points = query.points.size();
    if (query.points.size() < config_.aligned_overlap_strict_min_matches)
    {
        search.reason = "query_too_small";
        return search;
    }

    struct NearbyKeyFrame
    {
        RawKeyFrameId id;
        double distance_m = 0.0;
    };
    std::vector<NearbyKeyFrame> nearby;
    const Eigen::Vector3d query_position =
        query_world_T_kf->block<3, 1>(0, 3);
    for (const auto& submap_id : raw_db.GetSubmapIds())
    {
        for (const auto& candidate_id :
             raw_db.GetKeyFrameIdsForSubmap(submap_id))
        {
            if (candidate_id == query_kf_id)
            {
                continue;
            }
            if (covisibility_db.HasStrongEdge(
                    query_kf_id,
                    candidate_id,
                    config_.covisibility_strength))
            {
                ++search.candidate_keyframes_skipped_confirmed;
                continue;
            }
            const auto candidate_world_T_kf =
                pose_store.GetWorldPose(candidate_id);
            if (!candidate_world_T_kf ||
                !candidate_world_T_kf->allFinite())
            {
                continue;
            }
            const double distance =
                (candidate_world_T_kf->block<3, 1>(0, 3) -
                 query_position).norm();
            if (std::isfinite(distance) &&
                distance <= config_.aligned_overlap_keyframe_radius_m)
            {
                nearby.push_back({candidate_id, distance});
            }
        }
    }
    std::stable_sort(
        nearby.begin(),
        nearby.end(),
        [](const NearbyKeyFrame& lhs, const NearbyKeyFrame& rhs)
        {
            if (lhs.distance_m != rhs.distance_m)
            {
                return lhs.distance_m < rhs.distance_m;
            }
            return lhs.id < rhs.id;
        });

    std::map<RawSubmapId, uint64_t> examined_per_submap;
    std::set<RawSubmapId> confirmed_submaps;
    for (const auto& nearby_kf : nearby)
    {
        if (search.candidate_keyframes_examined >=
            config_.aligned_overlap_max_candidate_kfs)
        {
            break;
        }
        const RawSubmapId candidate_submap =
            ToSubmapId(nearby_kf.id);
        if (confirmed_submaps.find(candidate_submap) !=
            confirmed_submaps.end())
        {
            continue;
        }
        if (examined_per_submap[candidate_submap] >= 3U)
        {
            continue;
        }
        ++examined_per_submap[candidate_submap];
        ++search.candidate_keyframes_examined;

        Subcloud candidate =
            BuildQuerySubcloud(
                nearby_kf.id,
                raw_db,
                pose_store,
                score_manager);
        candidate.type = SubcloudType::CandidateInitial;
        candidate.seed_kf_id = nearby_kf.id;
        if (candidate.points.size() <
            config_.aligned_overlap_strict_min_matches)
        {
            ++search.candidate_keyframes_rejected;
            continue;
        }

        if (candidate_submap == ToSubmapId(query_kf_id))
        {
            const auto shared_matches =
                MatchSharedRawIdentities(query, candidate);
            const double shared_ratio = query.points.empty()
                ? 0.0
                : static_cast<double>(shared_matches.size()) /
                      static_cast<double>(query.points.size());
            if (shared_matches.size() >=
                    config_.aligned_overlap_strict_min_matches &&
                shared_ratio >=
                    config_.aligned_overlap_strict_min_match_ratio &&
                ImageCoverageBins(query, shared_matches) >=
                    config_.aligned_overlap_strict_min_image_bins &&
                SpatialCoverageRatio(query, shared_matches) >=
                    config_.aligned_overlap_strict_min_3d_span_ratio)
            {
                std::vector<uint64_t> shared_indices(shared_matches.size());
                std::iota(shared_indices.begin(), shared_indices.end(), 0U);
                Eigen::Matrix4d candidate_T_query = Eigen::Matrix4d::Identity();
                bool degenerate = false;
                if (EstimateRigidTransform(
                        shared_matches,
                        shared_indices,
                        true,
                        candidate_T_query,
                        degenerate))
                {
                    double shared_mean_residual = 0.0;
                    double shared_max_residual = 0.0;
                    ComputeMatchResiduals(
                        shared_matches,
                        candidate_T_query,
                        shared_mean_residual,
                        shared_max_residual);
                    const double error_t =
                        candidate_T_query.block<3, 1>(0, 3).norm();
                    const double error_yaw = std::abs(
                        NormalizeAngle(YawFromTransform(candidate_T_query)));
                    const double error_rot = RotationErrorRad(
                        candidate_T_query.block<3, 3>(0, 0));
                    if (shared_mean_residual <=
                            config_.aligned_overlap_strict_mean_residual_m &&
                        shared_max_residual <=
                            config_.aligned_overlap_strict_max_residual_m &&
                        error_t <= config_.fusion_error_t_m &&
                        error_yaw <= config_.fusion_error_yaw_rad &&
                        error_rot <= config_.fusion_error_yaw_rad)
                    {
                        uint64_t same_raw_ids = 0;
                        uint64_t same_track_matches = 0;
                        const auto expanded_matches =
                            MatchByPositionAndDescriptor(
                                query,
                                candidate,
                                candidate_T_query,
                                config_.aligned_overlap_expand_position_m,
                                config_.aligned_overlap_expand_max_hamming,
                                config_.aligned_overlap_expand_ratio_test,
                                same_raw_ids,
                                same_track_matches,
                                fused_landmark_manager);
                        search.same_raw_ids_skipped += same_raw_ids;
                        search.same_track_matches += same_track_matches;
                        double expanded_mean_residual = shared_mean_residual;
                        double expanded_max_residual = shared_max_residual;
                        if (!expanded_matches.empty())
                        {
                            ComputeMatchResiduals(
                                expanded_matches,
                                candidate_T_query,
                                expanded_mean_residual,
                                expanded_max_residual);
                        }
                        auto verification = BuildAlignedVerification(
                            query_kf_id,
                            nearby_kf.id,
                            shared_matches,
                            expanded_matches,
                            candidate_T_query,
                            expanded_mean_residual,
                            expanded_max_residual,
                            "aligned_intra_submap_shared_raw_identity",
                            query,
                            fused_landmark_manager);
                        verification.query_points = query.points.size();
                        verification.candidate_initial_points = candidate.points.size();
                        verification.shared_identity_matches = std::max<uint64_t>(
                            verification.shared_identity_matches,
                            shared_matches.size());
                        verification.ransac_inliers =
                            shared_matches.size() + expanded_matches.size();
                        verification.inlier_ratio = shared_ratio;
                        search.shared_identity_matches += shared_matches.size();
                        search.strict_matches += shared_matches.size();
                        search.expanded_matches += expanded_matches.size();
                        search.confirmed.push_back(std::move(verification));
                        confirmed_submaps.insert(candidate_submap);
                        continue;
                    }
                }
            }
        }

        uint64_t same_raw_ids = 0;
        uint64_t same_track_matches = 0;
        const auto strict_matches =
            MatchByPositionAndDescriptor(
                query,
                candidate,
                Eigen::Matrix4d::Identity(),
                config_.aligned_overlap_strict_position_m,
                config_.aligned_overlap_strict_max_hamming,
                config_.aligned_overlap_strict_ratio_test,
                same_raw_ids,
                same_track_matches,
                fused_landmark_manager);
        search.same_raw_ids_skipped += same_raw_ids;
        search.same_track_matches += same_track_matches;
        const double strict_ratio =
            query.points.empty()
                ? 0.0
                : static_cast<double>(strict_matches.size()) /
                      static_cast<double>(query.points.size());
        if (strict_matches.size() <
                config_.aligned_overlap_strict_min_matches ||
            strict_ratio <
                config_.aligned_overlap_strict_min_match_ratio)
        {
            ++search.candidate_keyframes_rejected;
            continue;
        }

        const uint64_t image_bins =
            ImageCoverageBins(query, strict_matches);
        const double spatial_coverage =
            SpatialCoverageRatio(query, strict_matches);
        if (image_bins <
                config_.aligned_overlap_strict_min_image_bins ||
            spatial_coverage <
                config_.aligned_overlap_strict_min_3d_span_ratio)
        {
            ++search.candidate_keyframes_rejected;
            continue;
        }

        std::vector<uint64_t> strict_indices(strict_matches.size());
        std::iota(
            strict_indices.begin(),
            strict_indices.end(),
            0U);
        Eigen::Matrix4d candidate_T_query =
            Eigen::Matrix4d::Identity();
        bool degenerate = false;
        if (!EstimateRigidTransform(
                strict_matches,
                strict_indices,
                true,
                candidate_T_query,
                degenerate))
        {
            ++search.candidate_keyframes_rejected;
            continue;
        }

        double strict_mean_residual = 0.0;
        double strict_max_residual = 0.0;
        ComputeMatchResiduals(
            strict_matches,
            candidate_T_query,
            strict_mean_residual,
            strict_max_residual);
        const double error_t =
            candidate_T_query.block<3, 1>(0, 3).norm();
        const double error_yaw =
            std::abs(
                NormalizeAngle(
                    YawFromTransform(candidate_T_query)));
        const double error_rot =
            RotationErrorRad(
                candidate_T_query.block<3, 3>(0, 0));
        if (strict_mean_residual >
                config_.aligned_overlap_strict_mean_residual_m ||
            strict_max_residual >
                config_.aligned_overlap_strict_max_residual_m ||
            error_t > config_.fusion_error_t_m ||
            error_yaw > config_.fusion_error_yaw_rad ||
            error_rot > config_.fusion_error_yaw_rad)
        {
            ++search.candidate_keyframes_rejected;
            continue;
        }

        same_raw_ids = 0;
        same_track_matches = 0;
        const auto expanded_matches =
            MatchByPositionAndDescriptor(
                query,
                candidate,
                candidate_T_query,
                config_.aligned_overlap_expand_position_m,
                config_.aligned_overlap_expand_max_hamming,
                config_.aligned_overlap_expand_ratio_test,
                same_raw_ids,
                same_track_matches,
                fused_landmark_manager);
        search.same_raw_ids_skipped += same_raw_ids;
        search.same_track_matches += same_track_matches;
        if (expanded_matches.size() < strict_matches.size())
        {
            ++search.candidate_keyframes_rejected;
            continue;
        }

        double expanded_mean_residual = 0.0;
        double expanded_max_residual = 0.0;
        ComputeMatchResiduals(
            expanded_matches,
            candidate_T_query,
            expanded_mean_residual,
            expanded_max_residual);
        auto verification =
            BuildAlignedVerification(
                query_kf_id,
                nearby_kf.id,
                strict_matches,
                expanded_matches,
                candidate_T_query,
                expanded_mean_residual,
                expanded_max_residual,
                "aligned_overlap_strict_then_expand",
                query,
                fused_landmark_manager);
        verification.query_points = query.points.size();
        verification.candidate_initial_points =
            candidate.points.size();
        search.strict_matches += strict_matches.size();
        search.expanded_matches += expanded_matches.size();
        search.confirmed.push_back(std::move(verification));
        confirmed_submaps.insert(candidate_submap);
    }

    search.reason =
        search.confirmed.empty()
            ? "no_strict_distributed_overlap"
            : "aligned_overlap_confirmed";
    return search;
}

AlignedOverlapSearchResult
SubcloudLoopVerifier::MatchNewMapPointsAgainstConfirmedNeighbors(
    const RawKeyFrameId& query_kf_id,
    const std::vector<RawMapPointId>& new_mappoint_ids,
    const RawMapDatabase& raw_db,
    const GlobalPoseStore& pose_store,
    const CovisibilityDatabase& covisibility_db,
    const LandmarkScoreManager* score_manager,
    const FusedLandmarkManager* fused_landmark_manager) const
{
    AlignedOverlapSearchResult search;
    search.query_kf_id = query_kf_id;
    search.incremental = true;
    if (new_mappoint_ids.empty())
    {
        search.reason = "no_new_mappoints";
        return search;
    }

    const Subcloud full_query =
        BuildQuerySubcloud(
            query_kf_id,
            raw_db,
            pose_store,
            score_manager);
    const std::set<RawMapPointId> allowed(
        new_mappoint_ids.begin(),
        new_mappoint_ids.end());
    const Subcloud query = FilterSubcloud(full_query, allowed);
    search.query_points = query.points.size();
    if (query.points.empty())
    {
        search.reason = "new_mappoints_without_world_points";
        return search;
    }

    auto neighbors =
        covisibility_db.GetNeighbors(query_kf_id, 0.0);
    std::stable_sort(
        neighbors.begin(),
        neighbors.end(),
        [](const CovisibilityEdge& lhs, const CovisibilityEdge& rhs)
        {
            if (lhs.weight != rhs.weight)
            {
                return lhs.weight > rhs.weight;
            }
            if (!(lhs.kf_a == rhs.kf_a))
            {
                return lhs.kf_a < rhs.kf_a;
            }
            return lhs.kf_b < rhs.kf_b;
        });

    std::set<std::pair<RawMapPointId, RawMapPointId>> emitted_pairs;
    for (const auto& edge : neighbors)
    {
        const RawKeyFrameId candidate_id =
            edge.kf_a == query_kf_id ? edge.kf_b : edge.kf_a;
        ++search.candidate_keyframes_examined;
        if (ToSubmapId(candidate_id) == ToSubmapId(query_kf_id))
        {
            const auto* candidate_keyframe =
                raw_db.GetKeyFrame(candidate_id);
            if (candidate_keyframe)
            {
                const std::set<uint64_t> candidate_ids(
                    candidate_keyframe->mappoint_ids.begin(),
                    candidate_keyframe->mappoint_ids.end());
                for (const auto& query_id : new_mappoint_ids)
                {
                    if (candidate_ids.find(query_id.local_mp_id) !=
                        candidate_ids.end())
                    {
                        ++search.same_raw_ids_skipped;
                    }
                }
            }
            continue;
        }
        Subcloud candidate =
            BuildQuerySubcloud(
                candidate_id,
                raw_db,
                pose_store,
                score_manager);
        candidate.type = SubcloudType::CandidateInitial;
        candidate.seed_kf_id = candidate_id;
        if (candidate.points.empty())
        {
            ++search.candidate_keyframes_rejected;
            continue;
        }

        uint64_t same_raw_ids = 0;
        uint64_t same_track_matches = 0;
        const auto matches =
            MatchByPositionAndDescriptor(
                query,
                candidate,
                Eigen::Matrix4d::Identity(),
                config_.aligned_overlap_expand_position_m,
                config_.aligned_overlap_expand_max_hamming,
                config_.aligned_overlap_expand_ratio_test,
                same_raw_ids,
                same_track_matches,
                fused_landmark_manager);
        search.same_raw_ids_skipped += same_raw_ids;
        search.same_track_matches += same_track_matches;

        std::vector<DescriptorMatch> unique_matches;
        for (const auto& match : matches)
        {
            const auto pair =
                std::make_pair(
                    match.query_mappoint_id,
                    match.candidate_mappoint_id);
            if (emitted_pairs.insert(pair).second)
            {
                unique_matches.push_back(match);
            }
        }
        if (unique_matches.empty())
        {
            continue;
        }

        double mean_residual = 0.0;
        double max_residual = 0.0;
        ComputeMatchResiduals(
            unique_matches,
            Eigen::Matrix4d::Identity(),
            mean_residual,
            max_residual);
        const std::vector<DescriptorMatch> no_strict_seed;
        auto verification =
            BuildAlignedVerification(
                query_kf_id,
                candidate_id,
                no_strict_seed,
                unique_matches,
                Eigen::Matrix4d::Identity(),
                mean_residual,
                max_residual,
                "incremental_confirmed_covisibility",
                query,
                fused_landmark_manager);
        verification.already_confirmed_covisibility = true;
        verification.confirmed_covisibility_source =
            ToString(edge.source);
        verification.query_points = query.points.size();
        verification.candidate_initial_points =
            candidate.points.size();
        search.expanded_matches += unique_matches.size();
        search.confirmed.push_back(std::move(verification));
    }

    search.reason =
        search.confirmed.empty()
            ? "incremental_no_new_equivalences"
            : "incremental_fusions_found";
    return search;
}

CapturedLoopVerification SubcloudLoopVerifier::CaptureCandidate(
    const LoopCandidate& candidate,
    const RawMapDatabase& raw_db,
    const GlobalPoseStore& pose_store,
    const CovisibilityDatabase* covisibility_db,
    const LandmarkScoreManager* score_manager) const
{
    CapturedLoopVerification captured;
    captured.candidate = candidate;
    auto& result = captured.result;
    result.query_kf_id = candidate.query_kf_id;
    result.candidate_seed_kf_id = candidate.candidate_kf_id;
    result.query_submap_id = candidate.query_submap_id;
    result.candidate_submap_id = candidate.candidate_submap_id;
    result.bow_score = candidate.bow_score;

    if (covisibility_db &&
        covisibility_db->HasStrongEdge(
            candidate.query_kf_id,
            candidate.candidate_kf_id,
            config_.covisibility_strength))
    {
        result.already_confirmed_covisibility = true;
        result.confirmed_covisibility_source = "CovisibilityDatabase";
        result.decision = LoopGeometryDecision::AlreadyConfirmedCovisibility;
        result.reason = "already_confirmed_covisibility";
        return captured;
    }

    const auto* query_kf = raw_db.GetKeyFrame(candidate.query_kf_id);
    if (!query_kf || query_kf->is_bad)
    {
        result.decision = LoopGeometryDecision::Reject;
        result.reason = "query_keyframe_missing_or_bad";
        return captured;
    }
    const auto query_world_T_kf =
        pose_store.GetWorldPose(candidate.query_kf_id);
    if (!query_world_T_kf || !IsFiniteTransform(*query_world_T_kf))
    {
        result.decision = LoopGeometryDecision::Reject;
        result.reason = "query_no_world_pose";
        return captured;
    }

    const auto* seed_kf = raw_db.GetKeyFrame(candidate.candidate_kf_id);
    if (!seed_kf || seed_kf->is_bad)
    {
        result.decision = LoopGeometryDecision::Reject;
        result.reason = "candidate_seed_missing_or_bad";
        return captured;
    }
    const auto candidate_world_T_kf =
        pose_store.GetWorldPose(candidate.candidate_kf_id);
    if (!candidate_world_T_kf ||
        !IsFiniteTransform(*candidate_world_T_kf))
    {
        result.decision = LoopGeometryDecision::Reject;
        result.reason = "candidate_seed_no_world_pose";
        return captured;
    }

    uint64_t covisible_added = 0;
    uint64_t tree_added = 0;
    uint64_t temporal_added = 0;
    uint64_t spatial_added = 0;
    captured.candidate_window =
        BuildCandidateWindow(
            candidate.candidate_kf_id,
            raw_db,
            pose_store,
            config_,
            covisible_added,
            tree_added,
            temporal_added,
            spatial_added);
    result.candidate_window_kfs = captured.candidate_window.size();
    result.candidate_window_covisible_added = covisible_added;
    result.candidate_window_tree_added = tree_added;
    result.candidate_window_temporal_added = temporal_added;
    result.candidate_window_spatial_added = spatial_added;

    std::set<RawKeyFrameId> source_keyframes(
        captured.candidate_window.begin(),
        captured.candidate_window.end());
    source_keyframes.insert(candidate.query_kf_id);
    for (const auto& source_kf_id : source_keyframes)
    {
        const auto* source_kf = raw_db.GetKeyFrame(source_kf_id);
        if (!source_kf)
        {
            continue;
        }
        captured.keyframes.emplace(source_kf_id, *source_kf);
        const auto world_T_kf = pose_store.GetWorldPose(source_kf_id);
        if (world_T_kf && IsFiniteTransform(*world_T_kf))
        {
            captured.world_T_keyframes.emplace(source_kf_id, *world_T_kf);
        }

        for (const auto local_mappoint_id : source_kf->mappoint_ids)
        {
            const RawMapPointId mappoint_id{
                source_kf_id.drone_id,
                source_kf_id.map_epoch,
                local_mappoint_id};
            if (captured.mappoints.find(mappoint_id) != captured.mappoints.end())
            {
                continue;
            }
            const auto* mappoint = raw_db.GetMapPoint(mappoint_id);
            if (!mappoint)
            {
                continue;
            }
            captured.mappoints.emplace(mappoint_id, *mappoint);
            if (score_manager)
            {
                captured.scores.emplace(
                    mappoint_id,
                    score_manager->GetScoreOrDefault(mappoint_id));
            }
        }
    }

    captured.ready_for_prepare = true;
    result.reason = "captured";
    return captured;
}

PreparedLoopVerification SubcloudLoopVerifier::PrepareCapturedCandidate(
    const CapturedLoopVerification& captured) const
{
    PreparedLoopVerification prepared;
    prepared.candidate = captured.candidate;
    prepared.result = captured.result;
    auto& result = prepared.result;
    if (!captured.ready_for_prepare)
    {
        return prepared;
    }

    prepared.query_subcloud = BuildCapturedQuerySubcloud(captured);
    result.query_points = prepared.query_subcloud.points.size();
    if (prepared.query_subcloud.points.size() < config_.query_subcloud_min_points)
    {
        result.decision = LoopGeometryDecision::Reject;
        result.reason = "query_subcloud_too_small";
        return prepared;
    }

    prepared.candidate_initial =
        BuildCapturedCandidateSubcloud(captured, config_);
    result.candidate_initial_points = prepared.candidate_initial.points.size();
    if (prepared.candidate_initial.points.size() <
        config_.candidate_subcloud_min_points)
    {
        result.decision = LoopGeometryDecision::Reject;
        result.reason = "candidate_subcloud_too_small";
        return prepared;
    }

    prepared.ready_for_compute = true;
    result.reason = "prepared";
    return prepared;
}

PreparedLoopVerification SubcloudLoopVerifier::PrepareCandidate(
    const LoopCandidate& candidate,
    const RawMapDatabase& raw_db,
    const GlobalPoseStore& pose_store,
    const CovisibilityDatabase* covisibility_db,
    const LandmarkScoreManager* score_manager) const
{
    return PrepareCapturedCandidate(
        CaptureCandidate(
            candidate,
            raw_db,
            pose_store,
            covisibility_db,
            score_manager));
}

LoopVerificationResult SubcloudLoopVerifier::VerifyPreparedCandidate(
    const PreparedLoopVerification& prepared) const
{
    LoopVerificationResult result = prepared.result;
    if (!prepared.ready_for_compute)
    {
        return result;
    }

    uint64_t initial_duplicates_removed = 0;
    double initial_mean_descriptor_distance = 0.0;
    const auto initial_matches =
        MatchSubclouds(
            prepared.query_subcloud,
            prepared.candidate_initial,
            config_,
            initial_duplicates_removed,
            initial_mean_descriptor_distance);
    result.initial_matches = initial_matches.size();
    result.initial_duplicates_removed = initial_duplicates_removed;
    result.initial_mean_descriptor_distance = initial_mean_descriptor_distance;
    if (initial_matches.size() < config_.min_initial_matches)
    {
        result.decision = LoopGeometryDecision::Reject;
        result.reason = "not_enough_initial_matches";
        return result;
    }

    Eigen::Vector3d reduce_box_min = Eigen::Vector3d::Zero();
    Eigen::Vector3d reduce_box_max = Eigen::Vector3d::Zero();
    bool reduction_fallback = false;
    std::string reduction_reason;
    const auto candidate_reduced =
        ReduceCandidateSubcloud(
            prepared.candidate_initial,
            initial_matches,
            config_,
            reduction_fallback,
            reduction_reason,
            reduce_box_min,
            reduce_box_max);
    result.reduction_fallback = reduction_fallback;
    result.reduction_reason = reduction_reason;
    result.reduction_box_valid =
        !reduction_fallback && IsFiniteVector(reduce_box_min) && IsFiniteVector(reduce_box_max);
    result.reduction_box_min = reduce_box_min;
    result.reduction_box_max = reduce_box_max;
    result.candidate_reduced_points = candidate_reduced.points.size();

    uint64_t refined_duplicates_removed = 0;
    double refined_mean_descriptor_distance = 0.0;
    const auto refined_matches =
        MatchSubclouds(
            prepared.query_subcloud,
            candidate_reduced,
            config_,
            refined_duplicates_removed,
            refined_mean_descriptor_distance);
    result.refined_matches = refined_matches.size();
    result.refined_duplicates_removed = refined_duplicates_removed;
    result.refined_mean_descriptor_distance = refined_mean_descriptor_distance;

    const auto ransac = RunRansac(refined_matches, config_);
    result.ransac_success = ransac.success;
    result.ransac_degenerate = ransac.degenerate;
    result.ransac_iterations = ransac.iterations;
    result.ransac_inliers = ransac.inliers;
    result.inlier_ratio = ransac.inlier_ratio;
    result.mean_residual = ransac.mean_residual;
    result.max_residual = ransac.max_residual;
    result.estimated_candidate_T_query = ransac.estimated_candidate_T_query;
    result.relative_pose_measured = ransac.estimated_candidate_T_query;

    std::vector<DescriptorMatch> inlier_matches;
    inlier_matches.reserve(ransac.inlier_match_indices.size());
    for (const auto index : ransac.inlier_match_indices)
    {
        if (index < refined_matches.size())
        {
            inlier_matches.push_back(refined_matches[index]);
        }
    }
    result.image_coverage_bins =
        ImageCoverageBins(prepared.query_subcloud, inlier_matches);
    result.spatial_coverage_ratio =
        SpatialCoverageRatio(prepared.query_subcloud, inlier_matches);

    if (!ransac.success)
    {
        result.decision = LoopGeometryDecision::Reject;
        result.reason = ransac.reason;
        return result;
    }

    result.error_t =
        ransac.estimated_candidate_T_query.block<3, 1>(0, 3).norm();
    result.error_yaw =
        std::abs(NormalizeAngle(YawFromTransform(ransac.estimated_candidate_T_query)));
    result.error_rot =
        RotationErrorRad(ransac.estimated_candidate_T_query.block<3, 3>(0, 0));
    result.loop_confidence = ConfidenceFromRansac(ransac, config_);

    for (const auto index : ransac.inlier_match_indices)
    {
        if (index >= refined_matches.size())
        {
            continue;
        }
        result.inlier_mappoint_pairs.push_back(
            {refined_matches[index].query_mappoint_id,
             refined_matches[index].candidate_mappoint_id});
    }

    if (ransac.mean_residual > config_.accept_mean_residual_m ||
        ransac.max_residual > config_.accept_max_residual_m)
    {
        result.decision = LoopGeometryDecision::Reject;
        result.reason = "residual_too_high";
        return result;
    }

    result.geometry_confirmed = true;
    if (result.error_t <= config_.fusion_error_t_m &&
        result.error_yaw <= config_.fusion_error_yaw_rad)
    {
        result.decision = LoopGeometryDecision::FusionCandidate;
        result.reason = "geometry_confirmed_low_pose_error";
    }
    else
    {
        result.decision = LoopGeometryDecision::LoopOptimizationCandidate;
        result.reason = "geometry_confirmed_pose_error";
    }
    return result;
}

}  // namespace orbslam3_multi
