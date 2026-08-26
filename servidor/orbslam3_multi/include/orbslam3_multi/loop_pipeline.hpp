#pragma once

#include "orbslam3_multi/covisibility_database.hpp"
#include "orbslam3_multi/global_pose_store.hpp"
#include "orbslam3_multi/loop_task.hpp"

#include <Eigen/Geometry>

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace orbslam3_multi
{

struct LoopPipelineConfig
{
  size_t min_query_mappoints = 12;
  size_t max_bow_candidates = 10;
  size_t max_candidate_regions = 3;
  size_t max_candidate_window_keyframes = 12;
  size_t max_subcloud_points = 320;
  uint64_t strong_covisibility_support = 15;
  uint64_t temporal_window_radius = 8;
  double min_bow_score = 0.01;
  uint32_t max_hamming_distance = 80;
  double descriptor_ratio = 0.90;
  size_t min_ransac_matches = 6;
  size_t min_ransac_inliers = 6;
  size_t max_ransac_iterations = 80;
  double ransac_inlier_threshold_m = 0.30;
  double min_inlier_ratio = 0.25;
  double max_mean_residual_m = 0.20;
  double max_residual_m = 0.75;
  double fusion_translation_threshold_m = 0.20;
  double fusion_rotation_threshold_rad = 0.12;
  double optimization_convergence_translation_m = 0.05;
  double optimization_convergence_rotation_rad = 0.03;
  double safe_correction_translation_m = 0.25;
  double safe_correction_rotation_rad = 0.15;
  double independent_translation_m = 0.20;
  double independent_yaw_rad = 0.08726646259971647;
  double hypothesis_translation_tolerance_m = 0.50;
  double hypothesis_rotation_tolerance_rad = 0.35;
  size_t hypothesis_support_no_risk = 2;
  size_t hypothesis_support_one_risk = 4;
  size_t hypothesis_support_multiple_risks = 6;
  double hypothesis_large_correction_translation_m = 1.0;
  double hypothesis_large_correction_rotation_rad = 0.20;
  size_t ambiguity_margin = 2;
  double structural_temporal_increase_m = 2.0;
  double structural_temporal_increase_rad = 0.70;
  double structural_covisibility_increase_m = 1.0;
  double structural_covisibility_increase_rad = 0.50;
  double structural_prior_loop_increase_m = 0.50;
  double structural_prior_loop_increase_rad = 0.35;
  double optimized_keyframe_max_translation_m = 5.0;
  double optimized_keyframe_max_rotation_rad = 0.3490658503988659;
  size_t consensus_min_segments = 3;
  double consensus_min_coverage_ratio = 0.60;
  double consensus_prior_weight_multiplier = 2.0;
  double fusion_refresh_spatial_margin_m = 1.0;
  double recent_loss_base_translation_m = 2.0;
  double recent_loss_path_drift_ratio = 0.20;
  double recent_loss_base_rotation_rad = 0.35;
  double recent_loss_rotation_drift_ratio = 0.20;
  bool recent_loss_single_loop_enabled = true;
  double recent_loss_single_loop_translation_m = 0.50;
  double recent_loss_single_loop_rotation_rad = 0.15;
  double recent_loss_single_loop_max_path_m = 2.0;
};

struct RecentLossRecoveryContext
{
  RawSubmapId submap_id;
  geometry_msgs::msg::Pose trusted_world_pose;
};

struct LoopCandidateRegion
{
  RawKeyFrameId query_keyframe_id;
  RawKeyFrameId seed_keyframe_id;
  std::vector<RawKeyFrameId> member_keyframe_ids;
  double bow_score = 0.0;
  size_t rank = 0;
  bool query_has_world_pose = false;
  bool candidate_has_world_pose = false;
};

struct LoopGeometryResult
{
  RawKeyFrameId query_keyframe_id;
  RawKeyFrameId candidate_keyframe_id;
  RawSubmapId query_submap_id;
  RawSubmapId candidate_submap_id;
  bool accepted = false;
  bool fusion_compatible = false;
  size_t query_points = 0;
  size_t candidate_points = 0;
  size_t matches = 0;
  size_t inliers = 0;
  double inlier_ratio = 0.0;
  double mean_residual_m = 0.0;
  double max_residual_m = 0.0;
  double current_translation_error_m = 0.0;
  double current_rotation_error_rad = 0.0;
  Eigen::Isometry3d candidate_local_T_query_local = Eigen::Isometry3d::Identity();
  struct MatchEvidence
  {
    RawMapPointId query_mappoint_id;
    RawMapPointId candidate_mappoint_id;
    uint32_t hamming_distance = 0;
    double residual_m = 0.0;
    bool inlier = false;
    bool hard_outlier = false;
  };
  std::vector<RawMapPointId> query_cloud_ids;
  std::vector<RawMapPointId> candidate_cloud_ids;
  std::vector<MatchEvidence> match_evidence;
  std::vector<std::pair<RawMapPointId, RawMapPointId>> inlier_pairs;
  std::string reason;
};

enum class LoopTaskDecisionKind
{
  Stale,
  NoBow,
  NoCandidates,
  GeometryRejected,
  SameSubmapDiagnostic,
  WaitingIndependentSupport,
  Deferred,
  FusionCandidate,
  OptimizationEvidence,
  OptimizationCommitted,
  ConstraintActivated,
  AnchorProposed,
  Error,
};

struct LoopFusionSummary
{
  bool attempted = false;
  bool prepared = false;
  bool committed = false;
  bool stale = false;
  bool rolled_back = false;
  size_t pair_results = 0;
  size_t tracks_created = 0;
  size_t tracks_updated = 0;
  size_t tracks_retired = 0;
  size_t hidden_raw_members = 0;
  size_t score_positive_events = 0;
  size_t score_negative_events = 0;
  size_t score_visibility_diagnostics = 0;
  size_t score_raw_updates = 0;
  size_t covisibility_added = 0;
  size_t covisibility_updated = 0;
  size_t visibility_regions_started = 0;
  size_t visibility_regions_completed = 0;
  size_t visibility_projected_points = 0;
  double visibility_ms = 0.0;
  double prepare_ms = 0.0;
  double commit_ms = 0.0;
  std::string reason;
};

struct LoopOptimizationSummary
{
  bool attempted = false;
  bool graph_built = false;
  bool optimized = false;
  bool accepted = false;
  bool committed = false;
  bool stale = false;
  bool fusion_after_optimization = false;
  size_t submaps = 0;
  size_t window_keyframes = 0;
  size_t controls = 0;
  size_t temporal_edges = 0;
  size_t covisibility_edges = 0;
  size_t loop_edges = 0;
  size_t rebuilds = 0;
  size_t discarded_loop_regions = 0;
  size_t moved_keyframes = 0;
  size_t propagated_keyframes = 0;
  size_t rebased_skipped_controls = 0;
  size_t rebased_inactive_controls = 0;
  size_t structural_edges_checked = 0;
  size_t optimized_keyframes_checked = 0;
  size_t iterations = 0;
  double initial_translation_error_m = 0.0;
  double final_translation_error_m = 0.0;
  double initial_rotation_error_rad = 0.0;
  double final_rotation_error_rad = 0.0;
  double initial_cost = 0.0;
  double final_cost = 0.0;
  double graph_ms = 0.0;
  double solve_ms = 0.0;
  double validation_ms = 0.0;
  double commit_ms = 0.0;
  double max_structural_translation_increase_m = 0.0;
  double max_structural_rotation_increase_rad = 0.0;
  double max_optimized_translation_change_m = 0.0;
  double max_optimized_rotation_change_rad = 0.0;
  std::string reason;
};

struct LoopHypothesisSupportSummary
{
  bool observed = false;
  bool compatible_hypothesis = false;
  bool independent = false;
  bool ambiguity_satisfied = false;
  bool accepted = false;
  size_t support = 0;
  size_t required_support = 0;
  size_t competing_support = 0;
  size_t ambiguity_margin = 0;
  double nearest_translation_separation_m = 0.0;
  double nearest_yaw_separation_rad = 0.0;
};

struct LoopTaskComputation
{
  LoopTaskDecisionKind decision = LoopTaskDecisionKind::Error;
  LoopTask task;
  bool used_fast_overlap = false;
  size_t bow_candidates = 0;
  size_t refresh_spatial_candidates = 0;
  size_t refresh_spatial_rejected = 0;
  std::vector<LoopCandidateRegion> regions;
  std::vector<LoopGeometryResult> geometry_results;
  std::vector<size_t> optimization_geometry_indices;
  std::vector<std::pair<RawMapPointId, RawMapPointId>> fusion_pairs;
  LoopHypothesisSupportSummary hypothesis_support;
  LoopFusionSummary fusion;
  LoopOptimizationSummary optimization;
  std::vector<LoopAnchorBatchEntry> anchor_entries;
  LoopAnchorBatchResult anchor_commit;
  std::vector<RawKeyFrameId> rerun_keyframe_ids;
  bool recent_loss_gate_checked = false;
  bool recent_loss_gate_passed = false;
  double recent_loss_translation_m = 0.0;
  double recent_loss_translation_limit_m = 0.0;
  double recent_loss_rotation_rad = 0.0;
  double recent_loss_rotation_limit_rad = 0.0;
  bool recent_loss_single_loop_checked = false;
  bool recent_loss_single_loop_eligible = false;
  bool recent_loss_single_loop_used = false;
  double recent_loss_single_loop_path_m = 0.0;
  bool protected_region_checked = false;
  bool protected_query_stable = false;
  bool protected_candidate_stable = false;
  bool rejection_ledger_hit = false;
  double protected_translation_error_m = 0.0;
  double protected_rotation_error_rad = 0.0;
  std::string reason;
};

/// Pipeline derivado BoW -> regiones -> subnubes/RANSAC -> decisión loop/fusión.
/// Conserva índices y ledgers de hipótesis, pero nunca escribe poses ni datos raw.
class LoopPipeline
{
public:
  void Configure(const LoopPipelineConfig & config);
  LoopTaskComputation Process(
    const LoopTask & task, const RawMapDatabase & raw_database,
    const GlobalPoseStore & pose_store,
    const CovisibilityDatabase & covisibility_database,
    const std::optional<RecentLossRecoveryContext> & recent_loss = std::nullopt);
  std::vector<LoopAnchorBatchEntry> BuildAnchorCascade(
    uint64_t task_id, const RawSubmapId & seed_submap,
    const RawMapDatabase & raw_database, const GlobalPoseStore & pose_store) const;
  std::vector<RawKeyFrameId> ConfirmedConstraintComponentKeyFrames(
    const RawSubmapId & seed_submap) const;
  size_t IndexedKeyFrames() const;

private:
  struct BowEntry
  {
    uint64_t revision = 0;
    std::map<uint32_t, double> words;
    double norm = 0.0;
  };

  struct PairAttemptKey
  {
    RawKeyFrameId first;
    RawKeyFrameId second;
    uint64_t first_revision = 0;
    uint64_t second_revision = 0;

    bool operator<(const PairAttemptKey & other) const;
  };

  struct HypothesisObservation
  {
    RawKeyFrameId query_keyframe_id;
    geometry_msgs::msg::Pose query_local_pose;
    RawKeyFrameId candidate_keyframe_id;
    geometry_msgs::msg::Pose candidate_local_pose;
    RawKeyFrameId child_control_keyframe_id;
  };

  struct Hypothesis
  {
    Eigen::Isometry3d first_local_T_second_local = Eigen::Isometry3d::Identity();
    std::vector<HypothesisObservation> observations;
  };

  struct Constraint
  {
    RawSubmapId first;
    RawSubmapId second;
    Eigen::Isometry3d first_local_T_second_local = Eigen::Isometry3d::Identity();
    RawKeyFrameId first_control;
    RawKeyFrameId second_control;
    size_t support = 0;
    bool provisional = false;
  };

  using SubmapPair = std::pair<RawSubmapId, RawSubmapId>;

  void UpsertBow(
    const RawKeyFrameId & id, uint64_t revision,
    const orbslam3_msgs::msg::OrbKeyFrame & keyframe);
  std::vector<std::pair<RawKeyFrameId, double>> SearchBow(
    const RawKeyFrameId & query_id) const;
  bool SpatiallyCompatibleForRefresh(
    const RawKeyFrameId & query_id, const RawKeyFrameId & candidate_id,
    const RawMapDatabase & raw_database,
    const GlobalPoseStore & pose_store) const;
  std::vector<LoopCandidateRegion> GroupRegions(
    const LoopTask & task,
    const std::vector<std::pair<RawKeyFrameId, double>> & candidates,
    const GlobalPoseStore & pose_store,
    const CovisibilityDatabase & covisibility_database) const;
  LoopGeometryResult VerifyRegion(
    const LoopTask & task, const LoopCandidateRegion & region,
    const RawMapDatabase & raw_database,
    const GlobalPoseStore & pose_store,
    const CovisibilityDatabase & covisibility_database) const;
  bool AddHypothesisEvidence(
    const SubmapPair & pair, const Eigen::Isometry3d & first_T_second,
    const HypothesisObservation & observation,
    size_t risk_signals, const LoopPipelineConfig & config, Hypothesis * accepted,
    LoopHypothesisSupportSummary * summary = nullptr,
    const std::optional<size_t> & required_support_override = std::nullopt);
  std::optional<LoopAnchorBatchEntry> BuildSingleRecoveryAnchor(
    const LoopGeometryResult & geometry, const RawKeyFrameId & candidate_control,
    const RawMapDatabase & raw_database, const GlobalPoseStore & pose_store) const;

  LoopPipelineConfig config_;
  std::map<RawKeyFrameId, BowEntry> bow_entries_;
  std::map<uint32_t, std::map<RawKeyFrameId, double>> inverted_index_;
  std::set<PairAttemptKey> geometry_rejections_;
  std::map<SubmapPair, std::vector<Hypothesis>> hypotheses_;
  std::map<SubmapPair, Constraint> active_constraints_;
};

const char * ToString(LoopTaskDecisionKind decision);

}  // namespace orbslam3_multi
