#include "orbslam3_multi/covisibility_database.hpp"

#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace orbslam3_multi
{
namespace
{

Eigen::Matrix4d PoseToMatrix(const geometry_msgs::msg::Pose& pose)
{
    Eigen::Quaterniond rotation(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z);
    if (rotation.norm() <= 1e-12)
    {
        rotation = Eigen::Quaterniond::Identity();
    }
    else
    {
        rotation.normalize();
    }

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = rotation.toRotationMatrix();
    transform(0, 3) = pose.position.x;
    transform(1, 3) = pose.position.y;
    transform(2, 3) = pose.position.z;
    return transform;
}

uint64_t SupportFromWeight(double weight)
{
    return weight > 0.0 ? static_cast<uint64_t>(std::llround(weight)) : 0U;
}

void ComputeNativeCoverage(
    const RawMapDatabase& raw_db,
    const RawKeyFrameId& keyframe_id,
    const orbslam3_msgs::msg::OrbKeyFrame& keyframe,
    const orbslam3_msgs::msg::OrbKeyFrame& connected,
    CovisibilityEdge& edge)
{
    const std::set<uint64_t> connected_ids(
        connected.mappoint_ids.begin(), connected.mappoint_ids.end());
    std::vector<size_t> shared_indices;
    for (size_t index = 0; index < keyframe.mappoint_ids.size(); ++index)
    {
        if (connected_ids.find(keyframe.mappoint_ids[index]) !=
            connected_ids.end())
        {
            shared_indices.push_back(index);
        }
    }
    if (!shared_indices.empty())
    {
        edge.shared_mappoints_or_inliers = shared_indices.size();
    }
    const size_t denominator = std::min(
        keyframe.mappoint_ids.size(), connected.mappoint_ids.size());
    edge.shared_mappoint_ratio = denominator == 0U
        ? 0.0
        : static_cast<double>(edge.shared_mappoints_or_inliers) /
              static_cast<double>(denominator);

    const auto* submap = raw_db.GetSubmap(
        RawSubmapId{keyframe_id.drone_id, keyframe_id.map_epoch});
    const double width = submap && submap->image_width > 0U
        ? static_cast<double>(submap->image_width)
        : 1.0;
    const double height = submap && submap->image_height > 0U
        ? static_cast<double>(submap->image_height)
        : 1.0;
    std::set<std::pair<int, int>> bins;
    Eigen::Vector3d all_min = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::infinity());
    Eigen::Vector3d all_max = Eigen::Vector3d::Constant(
        -std::numeric_limits<double>::infinity());
    Eigen::Vector3d shared_min = all_min;
    Eigen::Vector3d shared_max = all_max;
    std::set<size_t> shared_set(shared_indices.begin(), shared_indices.end());
    for (size_t index = 0; index < keyframe.mappoint_ids.size(); ++index)
    {
        const RawMapPointId point_id{
            keyframe_id.drone_id,
            keyframe_id.map_epoch,
            keyframe.mappoint_ids[index]};
        const auto* point = raw_db.GetMapPoint(point_id);
        if (!point || point->is_bad)
        {
            continue;
        }
        const Eigen::Vector3d position(
            point->position.x, point->position.y, point->position.z);
        if (!position.allFinite())
        {
            continue;
        }
        all_min = all_min.cwiseMin(position);
        all_max = all_max.cwiseMax(position);
        if (shared_set.find(index) != shared_set.end())
        {
            shared_min = shared_min.cwiseMin(position);
            shared_max = shared_max.cwiseMax(position);
            if (index < keyframe.keypoints.size())
            {
                const auto& keypoint = keyframe.keypoints[index];
                const int bx = std::clamp(
                    static_cast<int>(2.0 * keypoint.u / width), 0, 1);
                const int by = std::clamp(
                    static_cast<int>(2.0 * keypoint.v / height), 0, 1);
                bins.insert({bx, by});
            }
        }
    }
    edge.image_coverage_bins = bins.size();
    if (all_min.allFinite() && all_max.allFinite() &&
        shared_min.allFinite() && shared_max.allFinite())
    {
        const double all_span = (all_max - all_min).norm();
        const double shared_span = (shared_max - shared_min).norm();
        edge.spatial_coverage_ratio = all_span <= 1e-9
            ? 0.0
            : std::clamp(shared_span / all_span, 0.0, 1.0);
    }
}

}  // namespace

const char* ToString(CovisibilityEdgeSource source)
{
    switch (source)
    {
        case CovisibilityEdgeSource::Orbslam3Native:
            return "ORBSLAM3_NATIVE";
        case CovisibilityEdgeSource::ServerLoopGeometric:
            return "SERVER_LOOP_GEOMETRIC";
    }
    return "UNKNOWN";
}

CovisibilityImportResult CovisibilityDatabase::ImportOrbslam3Native(
    const RawMapDatabase& raw_db,
    uint64_t arrival_id,
    double min_weight)
{
    std::vector<RawKeyFrameId> all_keyframe_ids;
    for (const auto& submap_id : raw_db.GetSubmapIds())
    {
        const auto keyframe_ids = raw_db.GetKeyFrameIdsForSubmap(submap_id);
        all_keyframe_ids.insert(
            all_keyframe_ids.end(),
            keyframe_ids.begin(),
            keyframe_ids.end());
    }
    return ImportOrbslam3NativeForKeyFrames(
        raw_db,
        all_keyframe_ids,
        arrival_id,
        min_weight);
}

CovisibilityImportResult
CovisibilityDatabase::ImportOrbslam3NativeForKeyFrames(
    const RawMapDatabase& raw_db,
    const std::vector<RawKeyFrameId>& keyframe_ids,
    uint64_t arrival_id,
    double min_weight)
{
    CovisibilityImportResult result;
    result.arrival_id = arrival_id;
    std::set<RawKeyFrameId> examined_ids;
    for (const auto& keyframe_id : keyframe_ids)
    {
        if (!examined_ids.insert(keyframe_id).second)
        {
            continue;
        }
        const auto* keyframe = raw_db.GetKeyFrame(keyframe_id);
        if (!keyframe)
        {
            continue;
        }
        ++result.keyframes_examined;

        const size_t connection_count = std::min(
            keyframe->connected_keyframe_ids.size(),
            keyframe->connected_keyframe_weights.size());
        for (size_t index = 0; index < connection_count; ++index)
        {
            ++result.connections_examined;
            const double weight =
                static_cast<double>(
                    keyframe->connected_keyframe_weights[index]);
            if (!std::isfinite(weight) || weight < min_weight)
            {
                ++result.edges_skipped_low_weight;
                continue;
            }

            const RawKeyFrameId connected_id{
                keyframe_id.drone_id,
                keyframe_id.map_epoch,
                keyframe->connected_keyframe_ids[index]};
            const auto* connected = raw_db.GetKeyFrame(connected_id);
            if (!connected || connected_id == keyframe_id)
            {
                ++result.edges_skipped_missing_keyframe;
                continue;
            }

            CovisibilityEdge edge;
            edge.kf_a = keyframe_id;
            edge.kf_b = connected_id;
            edge.weight = weight;
            edge.source = CovisibilityEdgeSource::Orbslam3Native;
            edge.relative_pose_measured =
                PoseToMatrix(keyframe->pose).inverse().eval() *
                PoseToMatrix(connected->pose);
            edge.relative_pose_current = edge.relative_pose_measured;
            edge.information_weight = weight;
            edge.shared_mappoints_or_inliers = SupportFromWeight(weight);
            ComputeNativeCoverage(
                raw_db, keyframe_id, *keyframe, *connected, edge);
            edge.created_arrival_id = arrival_id;

            bool added = false;
            if (AddConfirmedLoopEdge(edge, &added))
            {
                if (added)
                {
                    ++result.edges_added;
                }
                else
                {
                    ++result.edges_updated;
                }
            }
        }
    }
    return result;
}

bool CovisibilityDatabase::AddConfirmedLoopEdge(
    const CovisibilityEdge& input_edge,
    bool* added)
{
    if (added)
    {
        *added = false;
    }
    if (!IsValidEdge(input_edge))
    {
        return false;
    }

    CovisibilityEdge edge = input_edge;
    Canonicalize(edge);
    const EdgeKey key = MakeKey(edge.kf_a, edge.kf_b);
    const auto existing = edges_.find(key);
    if (existing == edges_.end())
    {
        edge.updated_revision = ++revision_;
        edges_.emplace(key, edge);
        if (added)
        {
            *added = true;
        }
        return true;
    }

    CovisibilityEdge& stored = existing->second;
    const bool stronger_evidence =
        edge.weight > stored.weight ||
        edge.shared_mappoints_or_inliers > stored.shared_mappoints_or_inliers ||
        edge.shared_mappoint_ratio > stored.shared_mappoint_ratio ||
        edge.image_coverage_bins > stored.image_coverage_bins ||
        edge.spatial_coverage_ratio > stored.spatial_coverage_ratio ||
        (edge.geometry_confirmed && !stored.geometry_confirmed) ||
        (edge.source == CovisibilityEdgeSource::ServerLoopGeometric &&
         stored.source != CovisibilityEdgeSource::ServerLoopGeometric);
    if (!stronger_evidence)
    {
        return false;
    }

    // La primera medicion confirmada es evidencia historica y no se reemplaza.
    stored.weight = std::max(stored.weight, edge.weight);
    stored.information_weight = std::max(stored.information_weight, edge.information_weight);
    stored.shared_mappoints_or_inliers = std::max(
        stored.shared_mappoints_or_inliers,
        edge.shared_mappoints_or_inliers);
    stored.shared_mappoint_ratio = std::max(
        stored.shared_mappoint_ratio,
        edge.shared_mappoint_ratio);
    stored.image_coverage_bins = std::max(
        stored.image_coverage_bins,
        edge.image_coverage_bins);
    stored.spatial_coverage_ratio = std::max(
        stored.spatial_coverage_ratio,
        edge.spatial_coverage_ratio);
    stored.geometry_confirmed =
        stored.geometry_confirmed || edge.geometry_confirmed;
    if (edge.source == CovisibilityEdgeSource::ServerLoopGeometric)
    {
        stored.source = edge.source;
    }
    stored.updated_revision = ++revision_;
    return true;
}

bool CovisibilityDatabase::HasStrongEdge(
    const RawKeyFrameId& a,
    const RawKeyFrameId& b,
    const CovisibilityStrengthConfig& config) const
{
    const auto edge = GetEdge(a, b);
    return edge && IsStrongEdge(*edge, config);
}

bool CovisibilityDatabase::IsStrongEdge(
    const CovisibilityEdge& edge,
    const CovisibilityStrengthConfig& config)
{
    return edge.shared_mappoints_or_inliers >= config.min_support &&
           edge.shared_mappoint_ratio >= config.min_shared_ratio &&
           edge.image_coverage_bins >= config.min_image_bins &&
           edge.spatial_coverage_ratio >=
               config.min_spatial_coverage_ratio;
}

bool CovisibilityDatabase::HasConfirmedEdge(
    const RawKeyFrameId& a,
    const RawKeyFrameId& b) const
{
    return edges_.find(MakeKey(a, b)) != edges_.end();
}

std::optional<CovisibilityEdge> CovisibilityDatabase::GetEdge(
    const RawKeyFrameId& a,
    const RawKeyFrameId& b) const
{
    const auto it = edges_.find(MakeKey(a, b));
    if (it == edges_.end())
    {
        return std::nullopt;
    }
    return it->second;
}

std::vector<CovisibilityEdge> CovisibilityDatabase::GetNeighbors(
    const RawKeyFrameId& keyframe_id,
    double min_weight) const
{
    std::vector<CovisibilityEdge> neighbors;
    for (const auto& [_, edge] : edges_)
    {
        if (edge.weight >= min_weight &&
            (edge.kf_a == keyframe_id || edge.kf_b == keyframe_id))
        {
            neighbors.push_back(edge);
        }
    }
    return neighbors;
}

std::vector<CovisibilityEdge> CovisibilityDatabase::GetEdgesForWindow(
    const std::vector<RawKeyFrameId>& keyframe_ids,
    double min_weight) const
{
    const std::set<RawKeyFrameId> window(keyframe_ids.begin(), keyframe_ids.end());
    std::vector<CovisibilityEdge> result;
    for (const auto& [_, edge] : edges_)
    {
        if (edge.weight >= min_weight &&
            window.find(edge.kf_a) != window.end() &&
            window.find(edge.kf_b) != window.end())
        {
            result.push_back(edge);
        }
    }
    return result;
}

bool CovisibilityDatabase::UpdateRelativePoseCurrent(
    const RawKeyFrameId& a,
    const RawKeyFrameId& b,
    const Eigen::Matrix4d& relative_pose_current,
    uint64_t updated_revision)
{
    if (!relative_pose_current.allFinite())
    {
        return false;
    }
    const auto it = edges_.find(MakeKey(a, b));
    if (it == edges_.end() || updated_revision <= it->second.updated_revision)
    {
        return false;
    }

    const bool reverse_direction = b < a;
    it->second.relative_pose_current = reverse_direction
        ? relative_pose_current.inverse().eval()
        : relative_pose_current;
    it->second.updated_revision = updated_revision;
    revision_ = std::max(revision_, updated_revision);
    return true;
}

void CovisibilityDatabase::Clear()
{
    edges_.clear();
    revision_ = 0;
}

CovisibilityDatabaseStats CovisibilityDatabase::GetStats() const
{
    CovisibilityDatabaseStats stats;
    stats.confirmed_edges = edges_.size();
    stats.revision = revision_;
    for (const auto& [_, edge] : edges_)
    {
        if (edge.source == CovisibilityEdgeSource::Orbslam3Native)
        {
            ++stats.orbslam3_native_edges;
        }
        else if (edge.source == CovisibilityEdgeSource::ServerLoopGeometric)
        {
            ++stats.server_loop_geometric_edges;
        }
    }
    return stats;
}

bool CovisibilityDatabase::IsValidEdge(const CovisibilityEdge& edge)
{
    return !(edge.kf_a == edge.kf_b) &&
           std::isfinite(edge.weight) && edge.weight > 0.0 &&
           std::isfinite(edge.information_weight) && edge.information_weight > 0.0 &&
           edge.relative_pose_measured.allFinite() &&
           edge.relative_pose_current.allFinite();
}

void CovisibilityDatabase::Canonicalize(CovisibilityEdge& edge)
{
    if (edge.kf_b < edge.kf_a)
    {
        std::swap(edge.kf_a, edge.kf_b);
        edge.relative_pose_measured = edge.relative_pose_measured.inverse().eval();
        edge.relative_pose_current = edge.relative_pose_current.inverse().eval();
    }
}

CovisibilityDatabase::EdgeKey CovisibilityDatabase::MakeKey(
    const RawKeyFrameId& a,
    const RawKeyFrameId& b)
{
    return b < a ? EdgeKey{b, a} : EdgeKey{a, b};
}

}  // namespace orbslam3_multi
