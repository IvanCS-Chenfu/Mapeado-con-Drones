#include "orbslam3_multi/loop_detector.hpp"

#include "orbslam3_msgs/msg/orb_key_frame.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <utility>
#include <vector>

namespace orbslam3_multi
{
namespace
{
using OrbKeyFrame = orbslam3_msgs::msg::OrbKeyFrame;

struct BowVector
{
    std::map<uint32_t, double> weights;
    double norm = 0.0;
};

bool BuildBowVector(const OrbKeyFrame& keyframe, BowVector* bow)
{
    if (!bow)
    {
        return false;
    }
    bow->weights.clear();
    bow->norm = 0.0;

    if (keyframe.bow_word_ids.empty() ||
        keyframe.bow_word_ids.size() != keyframe.bow_word_values.size())
    {
        return false;
    }

    for (size_t index = 0; index < keyframe.bow_word_ids.size(); ++index)
    {
        const double value = static_cast<double>(keyframe.bow_word_values[index]);
        if (!std::isfinite(value) || value <= 0.0)
        {
            continue;
        }
        bow->weights[keyframe.bow_word_ids[index]] += value;
    }

    for (const auto& [_, weight] : bow->weights)
    {
        bow->norm += weight * weight;
    }
    bow->norm = std::sqrt(bow->norm);
    return bow->norm > 0.0 && !bow->weights.empty();
}

double ComputeBowScore(const BowVector& query, const BowVector& candidate)
{
    if (query.norm <= 0.0 || candidate.norm <= 0.0)
    {
        return 0.0;
    }

    double dot = 0.0;
    auto query_it = query.weights.begin();
    auto candidate_it = candidate.weights.begin();
    while (query_it != query.weights.end() && candidate_it != candidate.weights.end())
    {
        if (query_it->first < candidate_it->first)
        {
            ++query_it;
        }
        else if (candidate_it->first < query_it->first)
        {
            ++candidate_it;
        }
        else
        {
            dot += query_it->second * candidate_it->second;
            ++query_it;
            ++candidate_it;
        }
    }

    return dot / (query.norm * candidate.norm);
}

RawSubmapId ToSubmapId(const RawKeyFrameId& id)
{
    return RawSubmapId{id.drone_id, id.map_epoch};
}

uint64_t KeyFrameGap(const RawKeyFrameId& a, const RawKeyFrameId& b)
{
    if (a.local_kf_id > b.local_kf_id)
    {
        return a.local_kf_id - b.local_kf_id;
    }
    return b.local_kf_id - a.local_kf_id;
}

double RankingScore(const LoopCandidate& candidate)
{
    double score = candidate.bow_score;
    if (candidate.candidate_has_world_pose)
    {
        score += 0.02;
    }
    if (candidate.candidate_is_anchored)
    {
        score += 0.02;
    }
    score += std::min<double>(
        0.03,
        static_cast<double>(candidate.candidate_num_mappoints) / 2000.0);
    return score;
}

void AddFilterEvent(
    LoopCandidateResult* result,
    const RawKeyFrameId& candidate_id,
    const std::string& reason,
    double bow_score,
    bool same_submap,
    uint64_t kf_gap,
    bool confirmed_covisibility)
{
    if (!result)
    {
        return;
    }
    ++result->candidates_rejected_by_filter;
    const auto same_reason_count = std::count_if(
        result->filter_events.begin(),
        result->filter_events.end(),
        [&reason](const LoopCandidateFilterEvent& event) {
            return event.reason == reason;
        });
    const bool noisy_reason = reason == "low_bow_score" || reason == "no_bow";
    if (result->filter_events.size() >= 64 ||
        (noisy_reason && same_reason_count >= 3))
    {
        return;
    }
    result->filter_events.push_back(
        LoopCandidateFilterEvent{
            candidate_id,
            reason,
            bow_score,
            same_submap,
            kf_gap,
            confirmed_covisibility});
}
}  // namespace

LoopDetector::LoopDetector(const LoopDetectorConfig& config)
    : config_(config)
{
}

void LoopDetector::Configure(const LoopDetectorConfig& config)
{
    config_ = config;
}

const LoopDetectorConfig& LoopDetector::GetConfig() const
{
    return config_;
}

LoopCandidateResult LoopDetector::ProcessNewKeyFrame(
    const RawKeyFrameId& query_kf_id,
    const RawMapDatabase& raw_db,
    const GlobalPoseStore* pose_store,
    const CovisibilityDatabase& covisibility_db) const
{
    (void)pose_store;
    (void)covisibility_db;

    LoopCandidateResult result;
    result.query_kf_id = query_kf_id;
    const auto* query = raw_db.GetKeyFrame(query_kf_id);
    if (!query)
    {
        result.reason = "query_keyframe_missing";
        return result;
    }

    result.processed = true;
    result.query_bow_words = query->bow_word_ids.size();

    BowVector query_bow;
    result.query_has_bow = BuildBowVector(*query, &query_bow);
    for (const auto& submap_id : raw_db.GetSubmapIds())
    {
        result.indexed_keyframes += raw_db.GetKeyFrameIdsForSubmap(submap_id).size();
    }

    if (!result.query_has_bow)
    {
        result.reason = "query_no_bow";
        return result;
    }
    if (query->is_bad || query->mappoint_ids.size() < config_.min_mappoints)
    {
        result.reason = "query_bad_or_incomplete";
        return result;
    }

    std::vector<LoopCandidate> raw_candidates;
    for (const auto& submap_id : raw_db.GetSubmapIds())
    {
        for (const auto& candidate_id : raw_db.GetKeyFrameIdsForSubmap(submap_id))
        {
            if (candidate_id == query_kf_id)
            {
                continue;
            }

            const auto* candidate = raw_db.GetKeyFrame(candidate_id);
            if (!candidate)
            {
                continue;
            }

            const bool same_submap = ToSubmapId(candidate_id) == ToSubmapId(query_kf_id);
            const uint64_t gap = same_submap ? KeyFrameGap(query_kf_id, candidate_id) : 0;

            // A queued query must not discover a loop against KFs that arrived
            // later in the same submap. The newer KF evaluates the pair in
            // causal order when its own task reaches the worker.
            if (same_submap &&
                candidate_id.local_kf_id > query_kf_id.local_kf_id)
            {
                ++result.skipped_noncausal_same_submap;
                continue;
            }

            BowVector candidate_bow;
            if (!BuildBowVector(*candidate, &candidate_bow))
            {
                AddFilterEvent(
                    &result,
                    candidate_id,
                    "no_bow",
                    0.0,
                    same_submap,
                    gap,
                    false);
                continue;
            }

            ++result.compared_keyframes;
            const double bow_score = ComputeBowScore(query_bow, candidate_bow);
            if (bow_score < config_.min_bow_score)
            {
                AddFilterEvent(
                    &result,
                    candidate_id,
                    "low_bow_score",
                    bow_score,
                    same_submap,
                    gap,
                    false);
                continue;
            }

            ++result.candidates_raw;
            if (candidate->is_bad || candidate->mappoint_ids.size() < config_.min_mappoints)
            {
                AddFilterEvent(
                    &result,
                    candidate_id,
                    "bad_or_incomplete",
                    bow_score,
                    same_submap,
                    gap,
                    false);
                continue;
            }
            const bool confirmed = covisibility_db.HasConfirmedEdge(
                query_kf_id, candidate_id);
            const bool strong = covisibility_db.HasStrongEdge(
                query_kf_id,
                candidate_id,
                config_.covisibility_strength);
            if (strong)
            {
                ++result.skipped_confirmed_covisibility;
                AddFilterEvent(
                    &result,
                    candidate_id,
                    "strong_confirmed_covisibility",
                    bow_score,
                    same_submap,
                    gap,
                    true);
                continue;
            }
            if (confirmed)
            {
                ++result.weak_confirmed_covisibility;
            }

            const bool near_same_submap =
                same_submap && gap < config_.min_kf_gap_same_submap;
            if (near_same_submap)
            {
                ++result.near_same_submap_candidates;
            }

            const bool candidate_has_world_pose =
                pose_store && pose_store->HasWorldPose(candidate_id);
            const bool candidate_is_anchored =
                pose_store && pose_store->HasSubmapAnchor(ToSubmapId(candidate_id));
            raw_candidates.push_back(
                LoopCandidate{
                    query_kf_id,
                    candidate_id,
                    ToSubmapId(query_kf_id),
                    ToSubmapId(candidate_id),
                    bow_score,
                    0,
                    query_kf_id.drone_id == candidate_id.drone_id,
                    same_submap,
                    gap,
                    near_same_submap,
                    candidate_has_world_pose,
                    candidate_is_anchored,
                    static_cast<uint64_t>(query->mappoint_ids.size()),
                    static_cast<uint64_t>(candidate->mappoint_ids.size()),
                    candidate->is_bad,
                    false,
                    "BOW",
                    ""});
        }
    }

    std::stable_sort(
        raw_candidates.begin(),
        raw_candidates.end(),
        [](const LoopCandidate& a, const LoopCandidate& b) {
            const double rank_a = RankingScore(a);
            const double rank_b = RankingScore(b);
            if (std::fabs(rank_a - rank_b) > 1e-12)
            {
                return rank_a > rank_b;
            }
            return a.candidate_kf_id < b.candidate_kf_id;
        });

    std::map<RawSubmapId, uint64_t> selected_per_submap;
    for (const auto& candidate : raw_candidates)
    {
        if (result.candidates.size() >= config_.max_candidates)
        {
            AddFilterEvent(
                &result,
                candidate.candidate_kf_id,
                "max_candidates",
                candidate.bow_score,
                candidate.same_submap,
                candidate.kf_gap,
                false);
            continue;
        }

        auto& selected_for_submap = selected_per_submap[candidate.candidate_submap_id];
        if (selected_for_submap >= config_.max_candidates_per_submap)
        {
            AddFilterEvent(
                &result,
                candidate.candidate_kf_id,
                "max_candidates_per_submap",
                candidate.bow_score,
                candidate.same_submap,
                candidate.kf_gap,
                false);
            continue;
        }

        LoopCandidate ranked = candidate;
        ranked.rank = static_cast<uint64_t>(result.candidates.size() + 1);
        result.candidates.push_back(ranked);
        ++selected_for_submap;
    }

    result.candidates_after_filter = result.candidates.size();
    if (!result.candidates.empty())
    {
        result.best_candidate = result.candidates.front();
        result.reason = "candidates_found";
    }
    else if (result.candidates_raw > 0)
    {
        result.reason = "all_candidates_filtered";
    }
    else
    {
        result.reason = "no_bow_candidates";
    }
    return result;
}

}  // namespace orbslam3_multi
