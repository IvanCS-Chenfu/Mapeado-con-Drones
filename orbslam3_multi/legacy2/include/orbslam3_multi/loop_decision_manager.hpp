#pragma once

#include "orbslam3_multi/covisibility_database.hpp"
#include "orbslam3_multi/fused_landmark_manager.hpp"
#include "orbslam3_multi/global_pose_store.hpp"
#include "orbslam3_multi/loop_verification_result.hpp"

#include <cstdint>
#include <string>

namespace orbslam3_multi
{

struct LoopDecisionResult
{
    LoopGeometryDecision decision = LoopGeometryDecision::Reject;
    bool handled = false;
    bool covisibility_edge_changed = false;
    bool covisibility_edge_added = false;
    FusedLandmarkUpdateResult fusion;
    std::string reason;
};

class LoopDecisionManager
{
public:
    LoopDecisionResult Process(
        const LoopVerificationResult& verification,
        uint64_t arrival_id,
        const RawMapDatabase& raw_db,
        const GlobalPoseStore& pose_store,
        const LandmarkScoreManager& score_manager,
        CovisibilityDatabase& covisibility_db,
        FusedLandmarkManager& fused_landmark_manager) const;
};

}  // namespace orbslam3_multi
