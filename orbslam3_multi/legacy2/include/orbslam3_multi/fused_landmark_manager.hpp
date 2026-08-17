#pragma once

#include "orbslam3_multi/fused_landmark_track.hpp"
#include "orbslam3_multi/landmark_score_manager.hpp"
#include "orbslam3_multi/raw_map_database.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace orbslam3_multi
{

enum class FusedLandmarkPairAction : uint8_t
{
    CreateTrack = 0,
    AddMember = 1,
    ReinforceTrack = 2,
    MergeTracks = 3,
    Reject = 4,
};

struct FusedLandmarkPairResult
{
    RawMapPointId query_mappoint_id;
    RawMapPointId candidate_mappoint_id;
    FusedLandmarkPairAction action = FusedLandmarkPairAction::Reject;
    uint64_t track_id = 0;
    uint64_t merged_track_id = 0;
    std::string reason;
};

struct FusedLandmarkUpdateResult
{
    uint64_t pairs_received = 0;
    uint64_t pairs_fused = 0;
    uint64_t pairs_rejected = 0;
    uint64_t tracks_created = 0;
    uint64_t tracks_updated = 0;
    uint64_t tracks_merged = 0;
    std::vector<FusedLandmarkPairResult> pair_results;
};

struct FusedLandmarkStats
{
    uint64_t tracks = 0;
    uint64_t raw_members = 0;
    uint64_t multi_drone_tracks = 0;
    uint64_t revision = 0;
};

class FusedLandmarkManager
{
public:
    FusedLandmarkUpdateResult FuseInlierPairs(
        const std::vector<std::pair<RawMapPointId, RawMapPointId>>& pairs,
        const RawMapDatabase& raw_db,
        const LandmarkScoreManager& score_manager,
        double confidence);

    std::optional<uint64_t> GetTrackIdForMember(const RawMapPointId& id) const;
    std::optional<FusedLandmarkTrack> GetTrack(uint64_t track_id) const;
    std::vector<FusedLandmarkTrack> GetTracks() const;
    FusedLandmarkStats GetStats() const;

    bool UpdateFusedPosition(uint64_t track_id, const Eigen::Vector3d& position);
    void Clear();

private:
    struct RawMapPointIdHash
    {
        size_t operator()(const RawMapPointId& id) const noexcept;
    };

    bool ValidatePair(
        const RawMapPointId& query_id,
        const RawMapPointId& candidate_id,
        const RawMapDatabase& raw_db,
        std::string& reason) const;
    void RefreshDerivedData(
        FusedLandmarkTrack& track,
        const RawMapDatabase& raw_db,
        const LandmarkScoreManager& score_manager);
    uint64_t CreateTrack(
        const RawMapPointId& query_id,
        const RawMapPointId& candidate_id,
        double confidence);
    void AddMember(uint64_t track_id, const RawMapPointId& member_id);
    uint64_t MergeTracks(uint64_t first_track_id, uint64_t second_track_id);

    std::map<uint64_t, FusedLandmarkTrack> tracks_;
    std::unordered_map<RawMapPointId, uint64_t, RawMapPointIdHash> member_to_track_;
    uint64_t next_track_id_ = 1;
    uint64_t revision_ = 0;
};

const char* ToString(FusedLandmarkPairAction action);

}  // namespace orbslam3_multi
