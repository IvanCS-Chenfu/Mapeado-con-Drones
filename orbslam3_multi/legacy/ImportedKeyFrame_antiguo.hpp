#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <unordered_map>
#include <set>
#include <array>

#include <Eigen/Dense>

namespace orbslam3_multi
{

struct ImportedKeyPoint
{
    float u = 0.0f;
    float v = 0.0f;
    float size = 0.0f;
    float angle = 0.0f;
    float response = 0.0f;
    int32_t octave = 0;
    int32_t class_id = 0;

    std::array<uint8_t, 32> descriptor{};

    float u_right = -1.0f;
    float depth = -1.0f;
};

struct ImportedKeyFrame
{
    uint32_t drone_id = 0;
    uint64_t map_epoch = 0;
    uint64_t local_id = 0;
    uint64_t global_id = 0;

    double stamp = 0.0;

    std::string frame_id;
    uint32_t camera_id = 0;

    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

    std::vector<ImportedKeyPoint> keypoints;

    // global MapPoint ids, same size as keypoints.
    std::vector<uint64_t> mappoint_ids;

    // Covisibility graph.
    std::vector<uint64_t> connected_keyframe_ids;
    std::vector<uint32_t> connected_keyframe_weights;

    // BoW vector.
    std::vector<uint32_t> bow_word_ids;
    std::vector<float> bow_word_values;

    // FeatureVector flattened.
    std::vector<uint32_t> feat_node_ids;
    std::vector<uint32_t> feat_node_start_indices;
    std::vector<uint32_t> feat_node_sizes;
    std::vector<uint32_t> feat_indices;

    // Essential graph structure.
    uint64_t parent_keyframe_id = 0;
    std::vector<uint64_t> child_keyframe_ids;
    std::vector<uint64_t> loop_edge_keyframe_ids;

    bool is_bad = false;

    bool anchored_by_fiducial = false;
};

}  // namespace orbslam3_multi