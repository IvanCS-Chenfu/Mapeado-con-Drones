#include "orbslam3_multi/legacy/GlobalORBMatcher_antiguo.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace orbslam3_multi
{

GlobalORBMatcher::GlobalORBMatcher(
    const SearchByBowParams& params)
    : params_(params)
{
}

bool GlobalORBMatcher::DescriptorIsValid(
    const std::array<uint8_t, 32>& desc)
{
    for (uint8_t v : desc)
    {
        if (v != 0)
            return true;
    }

    return false;
}

int GlobalORBMatcher::DescriptorDistance(
    const std::array<uint8_t, 32>& a,
    const std::array<uint8_t, 32>& b)
{
    int dist = 0;

    for (size_t i = 0; i < 32; ++i)
    {
        const uint8_t x = a[i] ^ b[i];

        dist += __builtin_popcount(
            static_cast<unsigned int>(x));
    }

    return dist;
}

std::unordered_map<uint32_t, GlobalORBMatcher::FeatureNodeSpan>
GlobalORBMatcher::BuildFeatureNodeMap(
    const ImportedKeyFrame& kf) const
{
    std::unordered_map<uint32_t, FeatureNodeSpan> out;

    const size_t n =
        std::min(
            kf.feat_node_ids.size(),
            std::min(
                kf.feat_node_start_indices.size(),
                kf.feat_node_sizes.size()));

    out.reserve(n);

    for (size_t i = 0; i < n; ++i)
    {
        FeatureNodeSpan span;

        span.node_id = kf.feat_node_ids[i];
        span.start = kf.feat_node_start_indices[i];
        span.size = kf.feat_node_sizes[i];

        if (span.size == 0)
            continue;

        if (span.start >= kf.feat_indices.size())
            continue;

        if (static_cast<size_t>(span.start + span.size) >
            kf.feat_indices.size())
        {
            continue;
        }

        out[span.node_id] = span;
    }

    return out;
}

std::vector<uint32_t> GlobalORBMatcher::GetFeatureIndicesForNode(
    const ImportedKeyFrame& kf,
    const FeatureNodeSpan& span) const
{
    std::vector<uint32_t> out;
    out.reserve(span.size);

    const size_t end =
        static_cast<size_t>(span.start + span.size);

    for (size_t i = span.start; i < end; ++i)
    {
        if (i >= kf.feat_indices.size())
            break;

        const uint32_t idx = kf.feat_indices[i];

        if (idx >= kf.keypoints.size())
            continue;

        out.push_back(idx);
    }

    return out;
}

void GlobalORBMatcher::ComputeThreeMaxima(
    const std::vector<std::vector<size_t>>& histo,
    int& ind1,
    int& ind2,
    int& ind3) const
{
    int max1 = 0;
    int max2 = 0;
    int max3 = 0;

    ind1 = -1;
    ind2 = -1;
    ind3 = -1;

    for (int i = 0; i < static_cast<int>(histo.size()); ++i)
    {
        const int s =
            static_cast<int>(histo[i].size());

        if (s > max1)
        {
            max3 = max2;
            ind3 = ind2;

            max2 = max1;
            ind2 = ind1;

            max1 = s;
            ind1 = i;
        }
        else if (s > max2)
        {
            max3 = max2;
            ind3 = ind2;

            max2 = s;
            ind2 = i;
        }
        else if (s > max3)
        {
            max3 = s;
            ind3 = i;
        }
    }

    if (max2 < 0.1f * static_cast<float>(max1))
    {
        ind2 = -1;
        ind3 = -1;
    }
    else if (max3 < 0.1f * static_cast<float>(max1))
    {
        ind3 = -1;
    }
}

std::vector<FeatureMatch> GlobalORBMatcher::SearchByBoW(
    const ImportedKeyFrame& query,
    const ImportedKeyFrame& candidate) const
{
    std::vector<FeatureMatch> matches;

    if (query.keypoints.empty() || candidate.keypoints.empty())
        return matches;

    if (query.feat_node_ids.empty() || candidate.feat_node_ids.empty())
        return matches;

    const auto query_nodes =
        BuildFeatureNodeMap(query);

    const auto candidate_nodes =
        BuildFeatureNodeMap(candidate);

    if (query_nodes.empty() || candidate_nodes.empty())
        return matches;

    std::vector<int> matched_candidate_for_query(
        query.keypoints.size(),
        -1);

    std::vector<int> best_distance_for_query(
        query.keypoints.size(),
        std::numeric_limits<int>::max());

    std::vector<bool> candidate_already_matched(
        candidate.keypoints.size(),
        false);

    std::vector<std::vector<size_t>> rot_histo(
        static_cast<size_t>(params_.histo_length));

    const float factor =
        static_cast<float>(params_.histo_length) / 360.0f;

    // ============================================================
    // Igual que ORB-SLAM3: solo comparar features en nodos comunes
    // ============================================================

    for (const auto& [node_id, query_span] : query_nodes)
    {
        auto cand_node_it =
            candidate_nodes.find(node_id);

        if (cand_node_it == candidate_nodes.end())
            continue;

        const std::vector<uint32_t> query_indices =
            GetFeatureIndicesForNode(query, query_span);

        const std::vector<uint32_t> candidate_indices =
            GetFeatureIndicesForNode(candidate, cand_node_it->second);

        for (uint32_t idx_q : query_indices)
        {
            if (idx_q >= query.keypoints.size())
                continue;

            if (idx_q >= query.mappoint_ids.size())
                continue;

            const uint64_t mp_q =
                query.mappoint_ids[idx_q];

            if (mp_q == 0)
                continue;

            const auto& desc_q =
                query.keypoints[idx_q].descriptor;

            if (!DescriptorIsValid(desc_q))
                continue;

            int best_dist = std::numeric_limits<int>::max();
            int second_best_dist = std::numeric_limits<int>::max();
            int best_idx_c = -1;

            for (uint32_t idx_c : candidate_indices)
            {
                if (idx_c >= candidate.keypoints.size())
                    continue;

                if (idx_c >= candidate.mappoint_ids.size())
                    continue;

                if (candidate_already_matched[idx_c])
                    continue;

                const uint64_t mp_c =
                    candidate.mappoint_ids[idx_c];

                if (mp_c == 0)
                    continue;

                const auto& desc_c =
                    candidate.keypoints[idx_c].descriptor;

                if (!DescriptorIsValid(desc_c))
                    continue;

                const int dist =
                    DescriptorDistance(desc_q, desc_c);

                if (dist < best_dist)
                {
                    second_best_dist = best_dist;
                    best_dist = dist;
                    best_idx_c = static_cast<int>(idx_c);
                }
                else if (dist < second_best_dist)
                {
                    second_best_dist = dist;
                }
            }

            if (best_idx_c < 0)
                continue;

            if (best_dist > params_.th_low)
                continue;

            if (second_best_dist < std::numeric_limits<int>::max())
            {
                if (static_cast<float>(best_dist) >
                    params_.ratio * static_cast<float>(second_best_dist))
                {
                    continue;
                }
            }

            matched_candidate_for_query[idx_q] = best_idx_c;
            best_distance_for_query[idx_q] = best_dist;
            candidate_already_matched[best_idx_c] = true;

            if (params_.check_orientation)
            {
                float rot =
                    query.keypoints[idx_q].angle -
                    candidate.keypoints[static_cast<size_t>(best_idx_c)].angle;

                if (rot < 0.0f)
                    rot += 360.0f;

                int bin =
                    static_cast<int>(std::round(rot * factor));

                if (bin == params_.histo_length)
                    bin = 0;

                if (bin >= 0 && bin < params_.histo_length)
                {
                    rot_histo[static_cast<size_t>(bin)].push_back(idx_q);
                }
            }
        }
    }

    // ============================================================
    // Filtro de orientación, igual que ORBmatcher
    // ============================================================

    if (params_.check_orientation)
    {
        int ind1 = -1;
        int ind2 = -1;
        int ind3 = -1;

        ComputeThreeMaxima(rot_histo, ind1, ind2, ind3);

        for (int i = 0; i < params_.histo_length; ++i)
        {
            if (i == ind1 || i == ind2 || i == ind3)
                continue;

            for (size_t idx_q : rot_histo[static_cast<size_t>(i)])
            {
                if (idx_q < matched_candidate_for_query.size())
                {
                    int idx_c = matched_candidate_for_query[idx_q];

                    if (idx_c >= 0 &&
                        static_cast<size_t>(idx_c) < candidate_already_matched.size())
                    {
                        candidate_already_matched[static_cast<size_t>(idx_c)] = false;
                    }

                    matched_candidate_for_query[idx_q] = -1;
                }
            }
        }
    }

    for (size_t idx_q = 0; idx_q < matched_candidate_for_query.size(); ++idx_q)
    {
        const int idx_c =
            matched_candidate_for_query[idx_q];

        if (idx_c < 0)
            continue;

        if (static_cast<size_t>(idx_c) >= candidate.keypoints.size())
            continue;

        if (idx_q >= query.mappoint_ids.size())
            continue;

        if (static_cast<size_t>(idx_c) >= candidate.mappoint_ids.size())
            continue;

        FeatureMatch m;

        m.idx_query =
            idx_q;

        m.idx_candidate =
            static_cast<size_t>(idx_c);

        m.query_mappoint_id =
            query.mappoint_ids[idx_q];

        m.candidate_mappoint_id =
            candidate.mappoint_ids[static_cast<size_t>(idx_c)];

        m.distance =
            best_distance_for_query[idx_q];

        if (m.query_mappoint_id == 0 ||
            m.candidate_mappoint_id == 0)
        {
            continue;
        }

        matches.push_back(m);
    }

    return matches;
}

}  // namespace orbslam3_multi
