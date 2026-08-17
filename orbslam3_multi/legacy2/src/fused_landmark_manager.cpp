#include "orbslam3_multi/fused_landmark_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace orbslam3_multi
{
namespace
{

bool HasValidDescriptor(const orbslam3_msgs::msg::OrbMapPoint& mappoint)
{
    return std::any_of(
        mappoint.descriptor.data.begin(),
        mappoint.descriptor.data.end(),
        [](uint8_t value) { return value != 0U; });
}

uint32_t HammingDistance(
    const std::array<uint8_t, 32>& lhs,
    const std::array<uint8_t, 32>& rhs)
{
    uint32_t distance = 0;
    for (size_t index = 0; index < lhs.size(); ++index)
    {
        distance += static_cast<uint32_t>(
            __builtin_popcount(
                static_cast<unsigned int>(lhs[index] ^ rhs[index])));
    }
    return distance;
}

float Clamp01(float value)
{
    return std::max(0.0F, std::min(1.0F, value));
}

}  // namespace

const char* ToString(FusedLandmarkPairAction action)
{
    switch (action)
    {
        case FusedLandmarkPairAction::CreateTrack:
            return "CREATE_TRACK";
        case FusedLandmarkPairAction::AddMember:
            return "ADD_MEMBER";
        case FusedLandmarkPairAction::ReinforceTrack:
            return "REINFORCE_TRACK";
        case FusedLandmarkPairAction::MergeTracks:
            return "MERGE_TRACKS";
        case FusedLandmarkPairAction::Reject:
            return "REJECT";
    }
    return "UNKNOWN";
}

FusedLandmarkUpdateResult FusedLandmarkManager::FuseInlierPairs(
    const std::vector<std::pair<RawMapPointId, RawMapPointId>>& pairs,
    const RawMapDatabase& raw_db,
    const LandmarkScoreManager& score_manager,
    double confidence)
{
    FusedLandmarkUpdateResult result;
    result.pairs_received = pairs.size();
    confidence = std::max(0.0, std::min(1.0, confidence));

    std::set<uint64_t> affected_tracks;
    for (const auto& [query_id, candidate_id] : pairs)
    {
        FusedLandmarkPairResult pair_result;
        pair_result.query_mappoint_id = query_id;
        pair_result.candidate_mappoint_id = candidate_id;

        if (!ValidatePair(query_id, candidate_id, raw_db, pair_result.reason))
        {
            ++result.pairs_rejected;
            result.pair_results.push_back(pair_result);
            continue;
        }

        const auto query_track = GetTrackIdForMember(query_id);
        const auto candidate_track = GetTrackIdForMember(candidate_id);
        uint64_t track_id = 0;
        if (!query_track && !candidate_track)
        {
            track_id = CreateTrack(query_id, candidate_id, confidence);
            pair_result.action = FusedLandmarkPairAction::CreateTrack;
            ++result.tracks_created;
        }
        else if (query_track && !candidate_track)
        {
            track_id = query_track.value();
            AddMember(track_id, candidate_id);
            pair_result.action = FusedLandmarkPairAction::AddMember;
            ++result.tracks_updated;
        }
        else if (!query_track && candidate_track)
        {
            track_id = candidate_track.value();
            AddMember(track_id, query_id);
            pair_result.action = FusedLandmarkPairAction::AddMember;
            ++result.tracks_updated;
        }
        else if (query_track.value() == candidate_track.value())
        {
            track_id = query_track.value();
            pair_result.action = FusedLandmarkPairAction::ReinforceTrack;
            ++result.tracks_updated;
        }
        else
        {
            pair_result.merged_track_id =
                std::max(query_track.value(), candidate_track.value());
            track_id = MergeTracks(query_track.value(), candidate_track.value());
            pair_result.action = FusedLandmarkPairAction::MergeTracks;
            ++result.tracks_merged;
        }

        auto track_it = tracks_.find(track_id);
        if (track_it != tracks_.end())
        {
            track_it->second.confidence =
                std::max(track_it->second.confidence, confidence);
            ++track_it->second.support_count;
            track_it->second.revision = ++revision_;
        }
        pair_result.track_id = track_id;
        pair_result.reason = "inlier_pair_fused";
        affected_tracks.insert(track_id);
        ++result.pairs_fused;
        result.pair_results.push_back(pair_result);
    }

    for (const auto track_id : affected_tracks)
    {
        auto it = tracks_.find(track_id);
        if (it != tracks_.end())
        {
            RefreshDerivedData(it->second, raw_db, score_manager);
        }
    }
    return result;
}

std::optional<uint64_t> FusedLandmarkManager::GetTrackIdForMember(
    const RawMapPointId& id) const
{
    const auto it = member_to_track_.find(id);
    if (it == member_to_track_.end())
    {
        return std::nullopt;
    }
    return it->second;
}

std::optional<FusedLandmarkTrack> FusedLandmarkManager::GetTrack(
    uint64_t track_id) const
{
    const auto it = tracks_.find(track_id);
    if (it == tracks_.end())
    {
        return std::nullopt;
    }
    return it->second;
}

std::vector<FusedLandmarkTrack> FusedLandmarkManager::GetTracks() const
{
    std::vector<FusedLandmarkTrack> result;
    result.reserve(tracks_.size());
    for (const auto& [_, track] : tracks_)
    {
        result.push_back(track);
    }
    return result;
}

FusedLandmarkStats FusedLandmarkManager::GetStats() const
{
    FusedLandmarkStats stats;
    stats.tracks = tracks_.size();
    stats.raw_members = member_to_track_.size();
    stats.revision = revision_;
    for (const auto& [_, track] : tracks_)
    {
        if (track.source_drone_ids.size() > 1U)
        {
            ++stats.multi_drone_tracks;
        }
    }
    return stats;
}

bool FusedLandmarkManager::UpdateFusedPosition(
    uint64_t track_id,
    const Eigen::Vector3d& position)
{
    auto it = tracks_.find(track_id);
    if (it == tracks_.end() || !position.allFinite())
    {
        return false;
    }
    it->second.fused_position_world = position;
    it->second.position_valid = true;
    return true;
}

void FusedLandmarkManager::Clear()
{
    tracks_.clear();
    member_to_track_.clear();
    next_track_id_ = 1;
    revision_ = 0;
}

size_t FusedLandmarkManager::RawMapPointIdHash::operator()(
    const RawMapPointId& id) const noexcept
{
    size_t seed = std::hash<uint32_t>{}(id.drone_id);
    seed ^= std::hash<uint64_t>{}(id.map_epoch) +
            0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<uint64_t>{}(id.local_mp_id) +
            0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
}

bool FusedLandmarkManager::ValidatePair(
    const RawMapPointId& query_id,
    const RawMapPointId& candidate_id,
    const RawMapDatabase& raw_db,
    std::string& reason) const
{
    if (query_id == candidate_id)
    {
        reason = "same_raw_mappoint";
        return false;
    }
    const auto* query = raw_db.GetMapPoint(query_id);
    const auto* candidate = raw_db.GetMapPoint(candidate_id);
    if (!query || !candidate)
    {
        reason = "missing_raw_mappoint";
        return false;
    }
    if (query->is_bad || candidate->is_bad)
    {
        reason = "bad_raw_mappoint";
        return false;
    }
    if (!std::isfinite(query->position.x) ||
        !std::isfinite(query->position.y) ||
        !std::isfinite(query->position.z) ||
        !std::isfinite(candidate->position.x) ||
        !std::isfinite(candidate->position.y) ||
        !std::isfinite(candidate->position.z))
    {
        reason = "invalid_raw_position";
        return false;
    }
    if (!HasValidDescriptor(*query) || !HasValidDescriptor(*candidate))
    {
        reason = "invalid_descriptor";
        return false;
    }
    return true;
}

void FusedLandmarkManager::RefreshDerivedData(
    FusedLandmarkTrack& track,
    const RawMapDatabase& raw_db,
    const LandmarkScoreManager& score_manager)
{
    track.observing_keyframes.clear();
    track.source_submaps.clear();
    track.source_drone_ids.clear();

    struct DescriptorCandidate
    {
        RawMapPointId id;
        std::array<uint8_t, 32> descriptor{};
        float quality = 0.0F;
    };
    std::vector<DescriptorCandidate> descriptors;
    float max_raw_score = 0.0F;

    for (const auto& member_id : track.member_mappoint_ids)
    {
        const auto* mappoint = raw_db.GetMapPoint(member_id);
        if (!mappoint || mappoint->is_bad)
        {
            continue;
        }
        track.source_drone_ids.insert(member_id.drone_id);
        track.source_submaps.insert(
            RawSubmapId{member_id.drone_id, member_id.map_epoch});
        track.observing_keyframes.insert(
            RawKeyFrameId{
                member_id.drone_id,
                member_id.map_epoch,
                mappoint->reference_keyframe_id});
        for (const auto& observation : mappoint->observations)
        {
            track.observing_keyframes.insert(
                RawKeyFrameId{
                    member_id.drone_id,
                    member_id.map_epoch,
                    observation.keyframe_id});
        }

        const float quality = score_manager.GetScoreOrDefault(member_id);
        max_raw_score = std::max(max_raw_score, quality);
        if (HasValidDescriptor(*mappoint))
        {
            DescriptorCandidate candidate;
            candidate.id = member_id;
            candidate.quality = quality;
            std::copy(
                mappoint->descriptor.data.begin(),
                mappoint->descriptor.data.end(),
                candidate.descriptor.begin());
            descriptors.push_back(candidate);
        }
    }

    track.descriptor_valid = false;
    if (!descriptors.empty())
    {
        uint64_t best_distance = std::numeric_limits<uint64_t>::max();
        float best_quality = -1.0F;
        RawMapPointId best_id{
            std::numeric_limits<uint32_t>::max(),
            std::numeric_limits<uint64_t>::max(),
            std::numeric_limits<uint64_t>::max()};
        size_t best_index = 0;
        for (size_t i = 0; i < descriptors.size(); ++i)
        {
            uint64_t total_distance = 0;
            for (size_t j = 0; j < descriptors.size(); ++j)
            {
                total_distance += HammingDistance(
                    descriptors[i].descriptor,
                    descriptors[j].descriptor);
            }
            if (total_distance < best_distance ||
                (total_distance == best_distance &&
                 descriptors[i].quality > best_quality) ||
                (total_distance == best_distance &&
                 descriptors[i].quality == best_quality &&
                 descriptors[i].id < best_id))
            {
                best_distance = total_distance;
                best_quality = descriptors[i].quality;
                best_id = descriptors[i].id;
                best_index = i;
            }
        }
        track.representative_descriptor = descriptors[best_index].descriptor;
        track.descriptor_valid = true;
    }

    const float member_bonus = std::min(
        0.25F,
        0.06F * static_cast<float>(
            track.member_mappoint_ids.size() > 0U
                ? track.member_mappoint_ids.size() - 1U
                : 0U));
    const float drone_bonus = std::min(
        0.10F,
        0.04F * static_cast<float>(
            track.source_drone_ids.size() > 0U
                ? track.source_drone_ids.size() - 1U
                : 0U));
    const float submap_bonus = std::min(
        0.10F,
        0.03F * static_cast<float>(
            track.source_submaps.size() > 0U
                ? track.source_submaps.size() - 1U
                : 0U));
    track.score = Clamp01(max_raw_score + member_bonus + drone_bonus + submap_bonus);
}

uint64_t FusedLandmarkManager::CreateTrack(
    const RawMapPointId& query_id,
    const RawMapPointId& candidate_id,
    double confidence)
{
    const uint64_t track_id = next_track_id_++;
    FusedLandmarkTrack track;
    track.fused_track_id = track_id;
    track.member_mappoint_ids.insert(query_id);
    track.member_mappoint_ids.insert(candidate_id);
    track.confidence = confidence;
    track.revision = ++revision_;
    tracks_[track_id] = track;
    member_to_track_[query_id] = track_id;
    member_to_track_[candidate_id] = track_id;
    return track_id;
}

void FusedLandmarkManager::AddMember(
    uint64_t track_id,
    const RawMapPointId& member_id)
{
    auto it = tracks_.find(track_id);
    if (it == tracks_.end())
    {
        return;
    }
    it->second.member_mappoint_ids.insert(member_id);
    member_to_track_[member_id] = track_id;
}

uint64_t FusedLandmarkManager::MergeTracks(
    uint64_t first_track_id,
    uint64_t second_track_id)
{
    const uint64_t keep_id = std::min(first_track_id, second_track_id);
    const uint64_t remove_id = std::max(first_track_id, second_track_id);
    auto keep_it = tracks_.find(keep_id);
    auto remove_it = tracks_.find(remove_id);
    if (keep_it == tracks_.end() || remove_it == tracks_.end())
    {
        return keep_id;
    }

    for (const auto& member_id : remove_it->second.member_mappoint_ids)
    {
        keep_it->second.member_mappoint_ids.insert(member_id);
        member_to_track_[member_id] = keep_id;
    }
    keep_it->second.support_count += remove_it->second.support_count;
    keep_it->second.confidence =
        std::max(keep_it->second.confidence, remove_it->second.confidence);
    tracks_.erase(remove_it);
    return keep_id;
}

}  // namespace orbslam3_multi
