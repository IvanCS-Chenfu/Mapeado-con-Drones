#pragma once

#include "orbslam3_multi/covisibility_database.hpp"
#include "orbslam3_multi/fused_landmark_manager.hpp"
#include "orbslam3_multi/global_pose_store.hpp"
#include "orbslam3_multi/landmark_score_manager.hpp"
#include "orbslam3_multi/loop_candidate.hpp"
#include "orbslam3_multi/loop_verification_result.hpp"
#include "orbslam3_multi/raw_map_database.hpp"
#include "orbslam3_multi/subcloud.hpp"

#include <cstdint>
#include <map>
#include <vector>

namespace orbslam3_multi
{

struct SubcloudLoopVerifierConfig
{
    uint64_t query_subcloud_min_points = 12;
    uint64_t candidate_window_max_kfs = 12;
    uint64_t candidate_window_covisibility_min_weight = 15;
    uint64_t candidate_window_temporal_kf_radius = 8;
    double candidate_window_spatial_radius_m = 4.0;
    uint64_t candidate_subcloud_min_points = 20;
    uint64_t candidate_subcloud_max_points = 1500;
    float candidate_subcloud_min_score = 0.0F;
    uint32_t orb_match_max_hamming = 80;
    double orb_match_ratio_test = 0.90;
    bool orb_match_cross_check = true;
    uint64_t min_initial_matches = 8;
    bool candidate_reduce_enabled = true;
    uint64_t candidate_reduce_min_initial_matches = 8;
    double candidate_reduce_percentile_low = 10.0;
    double candidate_reduce_percentile_high = 90.0;
    double candidate_reduce_margin_m = 0.75;
    uint64_t candidate_reduce_min_points_after = 20;
    bool candidate_reduce_fallback_to_initial = true;
    uint64_t ransac_min_matches = 6;
    uint64_t ransac_max_iterations = 120;
    double ransac_inlier_threshold_m = 0.30;
    uint64_t ransac_min_inliers = 6;
    double ransac_min_inlier_ratio = 0.25;
    bool ransac_degeneracy_check_enabled = true;
    double accept_mean_residual_m = 0.20;
    double accept_max_residual_m = 0.75;
    double fusion_error_t_m = 0.35;
    double fusion_error_yaw_rad = 0.25;
    bool aligned_overlap_enabled = true;
    double aligned_overlap_keyframe_radius_m = 4.0;
    uint64_t aligned_overlap_max_candidate_kfs = 12;
    double aligned_overlap_strict_position_m = 0.40;
    uint32_t aligned_overlap_strict_max_hamming = 50;
    double aligned_overlap_strict_ratio_test = 0.80;
    uint64_t aligned_overlap_strict_min_matches = 8;
    double aligned_overlap_strict_min_match_ratio = 0.10;
    uint64_t aligned_overlap_strict_min_image_bins = 3;
    double aligned_overlap_strict_min_3d_span_ratio = 0.35;
    double aligned_overlap_strict_mean_residual_m = 0.12;
    double aligned_overlap_strict_max_residual_m = 0.30;
    double aligned_overlap_expand_position_m = 0.30;
    uint32_t aligned_overlap_expand_max_hamming = 80;
    double aligned_overlap_expand_ratio_test = 0.90;
    CovisibilityStrengthConfig covisibility_strength;
};

struct AlignedOverlapSearchResult
{
    RawKeyFrameId query_kf_id;
    bool incremental = false;
    uint64_t query_points = 0;
    uint64_t candidate_keyframes_examined = 0;
    uint64_t candidate_keyframes_skipped_confirmed = 0;
    uint64_t candidate_keyframes_rejected = 0;
    uint64_t strict_matches = 0;
    uint64_t expanded_matches = 0;
    uint64_t shared_identity_matches = 0;
    uint64_t same_raw_ids_skipped = 0;
    uint64_t same_track_matches = 0;
    std::vector<LoopVerificationResult> confirmed;
    std::string reason;
};

struct PreparedLoopVerification
{
    LoopCandidate candidate;
    LoopVerificationResult result;
    Subcloud query_subcloud;
    Subcloud candidate_initial;
    bool ready_for_compute = false;
};

struct CapturedLoopVerification
{
    LoopCandidate candidate;
    LoopVerificationResult result;
    std::vector<RawKeyFrameId> candidate_window;
    std::map<RawKeyFrameId, orbslam3_msgs::msg::OrbKeyFrame> keyframes;
    std::map<RawMapPointId, orbslam3_msgs::msg::OrbMapPoint> mappoints;
    std::map<RawKeyFrameId, Eigen::Matrix4d> world_T_keyframes;
    std::map<RawMapPointId, float> scores;
    bool ready_for_prepare = false;
};

class SubcloudLoopVerifier
{
public:
    SubcloudLoopVerifier() = default;
    explicit SubcloudLoopVerifier(const SubcloudLoopVerifierConfig& config);

    void Configure(const SubcloudLoopVerifierConfig& config);
    const SubcloudLoopVerifierConfig& GetConfig() const;

    CapturedLoopVerification CaptureCandidate(
        const LoopCandidate& candidate,
        const RawMapDatabase& raw_db,
        const GlobalPoseStore& pose_store,
        const CovisibilityDatabase* covisibility_db,
        const LandmarkScoreManager* score_manager) const;

    PreparedLoopVerification PrepareCapturedCandidate(
        const CapturedLoopVerification& captured) const;

    PreparedLoopVerification PrepareCandidate(
        const LoopCandidate& candidate,
        const RawMapDatabase& raw_db,
        const GlobalPoseStore& pose_store,
        const CovisibilityDatabase* covisibility_db,
        const LandmarkScoreManager* score_manager) const;

    LoopVerificationResult VerifyPreparedCandidate(
        const PreparedLoopVerification& prepared) const;

    LoopVerificationResult VerifyCandidate(
        const LoopCandidate& candidate,
        const RawMapDatabase& raw_db,
        const GlobalPoseStore& pose_store,
        const CovisibilityDatabase* covisibility_db,
        const LandmarkScoreManager* score_manager) const;

    AlignedOverlapSearchResult FindUnknownAlignedOverlaps(
        const RawKeyFrameId& query_kf_id,
        const RawMapDatabase& raw_db,
        const GlobalPoseStore& pose_store,
        const CovisibilityDatabase& covisibility_db,
        const LandmarkScoreManager* score_manager,
        const FusedLandmarkManager* fused_landmark_manager = nullptr) const;

    AlignedOverlapSearchResult MatchNewMapPointsAgainstConfirmedNeighbors(
        const RawKeyFrameId& query_kf_id,
        const std::vector<RawMapPointId>& new_mappoint_ids,
        const RawMapDatabase& raw_db,
        const GlobalPoseStore& pose_store,
        const CovisibilityDatabase& covisibility_db,
        const LandmarkScoreManager* score_manager,
        const FusedLandmarkManager* fused_landmark_manager = nullptr) const;

private:
    SubcloudLoopVerifierConfig config_;
};

}  // namespace orbslam3_multi
