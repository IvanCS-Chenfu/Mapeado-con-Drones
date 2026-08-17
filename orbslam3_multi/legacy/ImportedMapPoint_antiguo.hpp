#pragma once

#include <cstdint>
#include <vector>
#include <array>

#include <Eigen/Dense>

namespace orbslam3_multi
{

struct ImportedObservation
{
    uint64_t global_keyframe_id = 0;
    uint64_t local_keyframe_id = 0;

    uint32_t keypoint_index = 0;
    int32_t right_keypoint_index = -1;
};

struct ImportedMapPoint
{
    uint32_t drone_id = 0;
    uint64_t map_epoch = 0;
    uint64_t local_id = 0;
    uint64_t global_id = 0;

    Eigen::Vector3d position = Eigen::Vector3d::Zero();

    std::array<uint8_t, 32> descriptor{};

    bool is_bad = false;

    uint32_t observations_count = 0;
    float found_ratio = 0.0f;

    Eigen::Vector3d normal = Eigen::Vector3d::Zero();

    float min_distance = 0.0f;
    float max_distance = 0.0f;

    uint64_t reference_keyframe_id = 0;

    std::vector<ImportedObservation> observations;
};

}  // namespace orbslam3_multi