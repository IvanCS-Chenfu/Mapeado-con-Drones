#pragma once

#include "orbslam3_multi/raw_map_types.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace orbslam3_multi
{

// F1N: candidato visual BoW. BoW solo propone pares visualmente parecidos; la
// confirmacion queda reservada para la verificacion geometrica posterior.
struct LoopCandidate
{
    RawKeyFrameId query_kf_id;
    RawKeyFrameId candidate_kf_id;
    RawSubmapId query_submap_id;
    RawSubmapId candidate_submap_id;
    double bow_score = 0.0;
    uint64_t rank = 0;
    bool same_drone = false;
    bool same_submap = false;
    uint64_t kf_gap = 0;
    bool near_same_submap = false;
    bool candidate_has_world_pose = false;
    bool candidate_is_anchored = false;
    uint64_t query_num_mappoints = 0;
    uint64_t candidate_num_mappoints = 0;
    bool candidate_is_bad = false;
    bool already_confirmed_covisibility = false;
    std::string source = "BOW";
    std::string rejection_reason;
};

struct LoopCandidateFilterEvent
{
    RawKeyFrameId candidate_kf_id;
    std::string reason;
    double bow_score = 0.0;
    bool same_submap = false;
    uint64_t kf_gap = 0;
    bool already_confirmed_covisibility = false;
};

struct LoopCandidateResult
{
    RawKeyFrameId query_kf_id;
    bool processed = false;
    std::string reason;
    uint64_t indexed_keyframes = 0;
    uint64_t compared_keyframes = 0;
    uint64_t candidates_raw = 0;
    uint64_t candidates_after_filter = 0;
    uint64_t skipped_confirmed_covisibility = 0;
    uint64_t skipped_noncausal_same_submap = 0;
    uint64_t weak_confirmed_covisibility = 0;
    uint64_t near_same_submap_candidates = 0;
    uint64_t candidates_rejected_by_filter = 0;
    bool query_has_bow = false;
    uint64_t query_bow_words = 0;
    std::vector<LoopCandidate> candidates;
    std::vector<LoopCandidateFilterEvent> filter_events;
    std::optional<LoopCandidate> best_candidate;
};

}  // namespace orbslam3_multi
