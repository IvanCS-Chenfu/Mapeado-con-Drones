#pragma once

#include "orbslam3_multi/legacy/ImportedKeyFrame_antiguo.hpp"
#include "orbslam3_multi/legacy/GlobalAtlas_antiguo.hpp"

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <memory>
#include <mutex>
#include <cstdint>

namespace orbslam3_multi
{

struct BowCandidate
{
    uint64_t query_kf_id = 0;
    uint64_t candidate_kf_id = 0;

    uint32_t query_drone_id = 0;
    uint32_t candidate_drone_id = 0;

    uint64_t query_local_kf_id = 0;
    uint64_t candidate_local_kf_id = 0;

    int shared_words = 0;
    double bow_score = 0.0;
    double accumulated_score = 0.0;

    bool same_drone = false;
};

struct BowQueryParams
{
    bool enable_intra_drone = true;
    bool enable_inter_drone = true;

    uint64_t min_intra_drone_kf_gap = 30;

    int min_common_words = 8;
    double min_bow_score = 0.015;

    int max_candidates = 20;

    bool use_covisibility_accumulation = true;
    int covisibility_neighbors_for_score = 10;
};

class GlobalKeyFrameDatabase
{
public:
    explicit GlobalKeyFrameDatabase(std::shared_ptr<GlobalAtlas> atlas);

    void AddOrUpdateKeyFrame(const ImportedKeyFrame& kf);
    void EraseKeyFrame(uint64_t global_kf_id);

    void Clear();

    std::vector<BowCandidate> DetectCandidates(
        uint64_t query_global_kf_id,
        const BowQueryParams& params) const;

    size_t Size() const;
    size_t InvertedIndexWords() const;

    void ClearDrone(uint32_t drone_id);

    void ClearDroneEpoch(
        uint32_t drone_id,
        uint64_t map_epoch);

private:
    double ComputeBowScore(
        const ImportedKeyFrame& a,
        const ImportedKeyFrame& b) const;

    int CountSharedWords(
        const ImportedKeyFrame& a,
        const ImportedKeyFrame& b) const;

    std::vector<uint64_t> GetStrongCovisibleKeyFrames(
        const ImportedKeyFrame& kf,
        int max_neighbors) const;

private:
    std::shared_ptr<GlobalAtlas> atlas_;

    mutable std::mutex mutex_;

    // global_kf_id -> keyframe
    std::unordered_map<uint64_t, ImportedKeyFrame> keyframes_;

    // word_id -> global_kf_id list
    std::unordered_map<uint32_t, std::vector<uint64_t>> inverted_file_;

    // global_kf_id -> word ids, para poder borrar rápido
    std::unordered_map<uint64_t, std::vector<uint32_t>> words_by_keyframe_;
};

}  // namespace orbslam3_multi
