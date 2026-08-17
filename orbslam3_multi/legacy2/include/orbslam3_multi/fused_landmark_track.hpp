#pragma once

#include "orbslam3_multi/raw_map_types.hpp"

#include <Eigen/Core>

#include <array>
#include <cstdint>
#include <set>

namespace orbslam3_multi
{

struct FusedLandmarkTrack
{
    uint64_t fused_track_id = 0;
    std::set<RawMapPointId> member_mappoint_ids;
    std::set<RawKeyFrameId> observing_keyframes;
    std::set<RawSubmapId> source_submaps;
    std::set<uint32_t> source_drone_ids;
    Eigen::Vector3d fused_position_world = Eigen::Vector3d::Zero();
    std::array<uint8_t, 32> representative_descriptor{};
    bool position_valid = false;
    bool descriptor_valid = false;
    float score = 0.0F;
    double confidence = 0.0;
    uint64_t support_count = 0;
    uint64_t revision = 0;
};

}  // namespace orbslam3_multi
