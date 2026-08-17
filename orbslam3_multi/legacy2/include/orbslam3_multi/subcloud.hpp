#pragma once

#include "orbslam3_multi/raw_map_types.hpp"

#include <Eigen/Core>

#include <array>
#include <cstdint>
#include <set>
#include <string>
#include <vector>

namespace orbslam3_multi
{

enum class SubcloudType : uint8_t
{
    Query = 0,
    CandidateInitial = 1,
    CandidateReduced = 2,
};

enum class SubcloudPointSource : uint8_t
{
    QueryKeyFrame = 0,
    CandidateWindow = 1,
};

struct SubcloudPoint
{
    RawMapPointId mappoint_id;
    RawKeyFrameId source_kf_id;
    RawSubmapId submap_id;
    Eigen::Vector3d position_world = Eigen::Vector3d::Zero();
    std::array<uint8_t, 32> descriptor{};
    float score = 0.0F;
    uint32_t num_observations = 0;
    bool is_bad = false;
    uint32_t keypoint_index = 0;
    double keypoint_u = 0.0;
    double keypoint_v = 0.0;
    bool keypoint_valid = false;
    SubcloudPointSource source = SubcloudPointSource::QueryKeyFrame;
};

struct SubcloudStats
{
    uint64_t raw_points = 0;
    uint64_t filtered_points = 0;
    uint64_t bad_removed = 0;
    uint64_t no_descriptor_removed = 0;
    uint64_t low_score_removed = 0;
    uint64_t no_pose_removed = 0;
    uint64_t duplicates_removed = 0;
    uint64_t final_points = 0;
};

struct Subcloud
{
    std::string cloud_id;
    SubcloudType type = SubcloudType::Query;
    RawKeyFrameId center_kf_id;
    RawKeyFrameId seed_kf_id;
    std::set<RawSubmapId> submap_ids;
    std::set<RawKeyFrameId> source_kf_ids;
    std::vector<SubcloudPoint> points;
    Eigen::Vector3d bbox_min = Eigen::Vector3d::Zero();
    Eigen::Vector3d bbox_max = Eigen::Vector3d::Zero();
    SubcloudStats stats;
};

struct DescriptorMatch
{
    uint64_t query_index = 0;
    uint64_t candidate_index = 0;
    RawMapPointId query_mappoint_id;
    RawMapPointId candidate_mappoint_id;
    Eigen::Vector3d query_position_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d candidate_position_world = Eigen::Vector3d::Zero();
    uint32_t descriptor_distance = 0;
};

}  // namespace orbslam3_multi
