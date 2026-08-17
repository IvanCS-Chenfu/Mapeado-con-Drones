#pragma once

#include "orbslam3_multi/legacy/ImportedKeyFrame_antiguo.hpp"
#include "orbslam3_multi/legacy/ImportedMapPoint_antiguo.hpp"

#include <vector>
#include <cstdint>
#include <array>
#include <unordered_map>

namespace orbslam3_multi
{

struct FeatureMatch
{
    size_t idx_query = 0;
    size_t idx_candidate = 0;

    uint64_t query_mappoint_id = 0;
    uint64_t candidate_mappoint_id = 0;

    int distance = 0;
};

struct SearchByBowParams
{
    int th_low = 50;
    float ratio = 0.75f;

    bool check_orientation = true;

    int histo_length = 30;

    size_t min_required_matches = 15;
};

class GlobalORBMatcher
{
public:
    explicit GlobalORBMatcher(const SearchByBowParams& params = SearchByBowParams());

    std::vector<FeatureMatch> SearchByBoW(
        const ImportedKeyFrame& query,
        const ImportedKeyFrame& candidate) const;

    static int DescriptorDistance(
        const std::array<uint8_t, 32>& a,
        const std::array<uint8_t, 32>& b);

    static bool DescriptorIsValid(
        const std::array<uint8_t, 32>& desc);

private:
    struct FeatureNodeSpan
    {
        uint32_t node_id = 0;
        uint32_t start = 0;
        uint32_t size = 0;
    };

    std::unordered_map<uint32_t, FeatureNodeSpan> BuildFeatureNodeMap(
        const ImportedKeyFrame& kf) const;

    std::vector<uint32_t> GetFeatureIndicesForNode(
        const ImportedKeyFrame& kf,
        const FeatureNodeSpan& span) const;

    void ComputeThreeMaxima(
        const std::vector<std::vector<size_t>>& histo,
        int& ind1,
        int& ind2,
        int& ind3) const;

private:
    SearchByBowParams params_;
};

}  // namespace orbslam3_multi
