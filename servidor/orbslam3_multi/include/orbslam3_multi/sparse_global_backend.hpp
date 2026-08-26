#pragma once

#include "orbslam3_multi/global_pose_store.hpp"
#include "orbslam3_multi/fiducial_anchor_manager.hpp"
#include "orbslam3_multi/fused_landmark_manager.hpp"
#include "orbslam3_multi/global_map_builder.hpp"
#include "orbslam3_multi/landmark_score_manager.hpp"
#include "orbslam3_multi/covisibility_database.hpp"
#include "orbslam3_multi/loop_pipeline.hpp"
#include "orbslam3_multi/optimization_manager.hpp"
#include "orbslam3_multi/optimization_validator.hpp"
#include "orbslam3_multi/pose_graph_builder.hpp"
#include "orbslam3_multi/raw_map_database.hpp"

#include <memory>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <tuple>

namespace orbslam3_multi
{

struct PrimaryBackendResult
{
  RawInsertResult raw_result;
  ScoreChangeSet score_changes;
  bool pose_stage_executed = false;
  bool had_deferred_snapshot_dirty = false;
  PoseChangeSet pose_changes;
};

struct SecondaryWorkPlan
{
  std::optional<DatabaseUpdateTask> database_update;
  std::vector<LoopTask> direct_loop_tasks;
};

/// Fachada sin ROS que coordina las autoridades raw, poses, score, fusión y vista pública.
/// Los cálculos secundarios preparan propuestas privadas; solo los métodos Commit* mutan
/// autoridades y state_commit_mutex_ serializa commits que abarcan varios componentes.
class SparseGlobalBackend
{
public:
  // Ruta principal: importa ORB y devuelve únicamente los cambios incrementales derivados.
  PrimaryBackendResult InsertDelta(
    uint64_t arrival_id,
    std::shared_ptr<const orbslam3_msgs::msg::OrbMap> delta);
  PrimaryBackendResult InsertFullSnapshot(
    uint64_t arrival_id,
    std::shared_ptr<const orbslam3_msgs::msg::OrbMap> snapshot);
  void SetFiducialPendingCapacityPerDrone(size_t capacity);
  FiducialBatchSubmitResult SubmitFiducialBatch(
    const orbslam3_msgs::msg::FiducialKeyFrameObservations & batch);
  FiducialSyncStats GetFiducialSyncStats() const;

  PoseChangeSet CommitAnchor(
    const RawSubmapId & submap_id,
    const geometry_msgs::msg::Pose & world_T_local,
    uint64_t source_task_id);

  FiducialProcessResult ProcessFiducialObservation(
    const FiducialObservation & observation,
    bool append_to_journal);
  std::optional<orbslam3_msgs::msg::OrbKeyFrame> GetRawKeyFrame(
    const RawKeyFrameId & id) const;
  std::vector<RecordedFiducialObservation> GetFiducialObservationJournal() const;

  // Ruta secundaria: planifica y calcula fuera del worker principal; nunca publica.
  SecondaryWorkPlan PlanSecondaryWork(const RawInsertResult & raw_result) const;
  std::vector<LoopTask> CreateLoopTasks(
    uint64_t source_arrival_id,
    const std::vector<RawKeyFrameId> & keyframe_ids) const;
  std::vector<LoopTask> CreateFusionRefreshTasks(
    uint64_t source_arrival_id,
    const std::vector<RawKeyFrameId> & keyframe_ids) const;
  CovisibilityUpdateResult ProcessDatabaseUpdate(const DatabaseUpdateTask & task);
  LoopTaskComputation ProcessLoopTask(const LoopTask & task);
  LoopTaskComputation ProcessLoopOptimization(LoopTaskComputation computation);
  CovisibilityDatabaseStats GetCovisibilityStats() const;
  FusedLandmarkStats GetFusedLandmarkStats() const;

  // Configuración inmutable durante una ejecución normal del servidor.
  void ConfigureFiducialOptimization(const FiducialOptimizationConfig & config);
  void ConfigureLoopPipeline(const LoopPipelineConfig & config);
  void ConfigureFusedLandmarks(const FusedLandmarkConfig & config);
  void ConfigureLandmarkScores(const LandmarkScoreConfig & config);
  FiducialTaskRevalidation RevalidateFiducialTask(
    const FiducialOptimizationTask & task);
  PoseGraphBuildResult BuildFiducialPoseGraph(
    const FiducialOptimizationTask & task);
  OptimizationProposal OptimizeFiducialPoseGraph(
    const PoseGraphProblem & problem) const;
  ValidationResult ValidateFiducialProposal(
    const PoseGraphProblem & problem,
    const OptimizationProposal & proposal) const;
  FiducialCommitResult CommitFiducialProposal(
    const PoseGraphProblem & problem,
    const OptimizationProposal & proposal,
    const ValidationResult & validation);

  // Snapshots de diagnóstico y construcción de la vista pública coherente.
  RawDatabaseStats GetRawStats() const;
  GlobalPoseStoreStats GetPoseStats() const;
  LandmarkScoreStats GetScoreStats() const;
  std::optional<GlobalPoseRecord> GetGlobalPose(const RawKeyFrameId & id) const;
  GlobalMapBuildResult BuildGlobalMap();
  bool StartRawRecord(
    const std::string & path, std::string * error_message = nullptr);
  bool FinalizeRawRecord(std::string * error_message = nullptr);
  void DisableRawJournalRetention();
  RawJournalStorageStats GetRawJournalStorageStats() const;
  bool SaveRawRecord(const std::string & path, std::string * error_message = nullptr) const;

private:
  using LoopRejectionKey = std::tuple<
      uint32_t, uint64_t, uint64_t, uint32_t, uint64_t, uint64_t,
      int64_t, int64_t, int64_t, int64_t, uint64_t, uint64_t>;

  struct RecentLossContinuityRecord
  {
    RawSubmapId previous_submap_id;
    RawKeyFrameId trusted_keyframe_id;
    geometry_msgs::msg::Pose trusted_world_pose;
  };

  void TrackSubmapTransition(const RawInsertResult & raw_result);
  bool ValidateRecentLossAnchor(
    const LoopAnchorBatchEntry & entry, LoopTaskComputation * computation) const;
  void ApplyWorldAuthorityCascadeLocked(
    const RawSubmapId & seed_submap, uint64_t source_task_id,
    LoopAnchorBatchResult * anchor_commit,
    std::vector<RawKeyFrameId> * reconciliation_keyframe_ids);
  bool IsProtectedLoopRegion(const RawKeyFrameId & keyframe_id) const;
  LoopRejectionKey BuildLoopRejectionKey(const LoopGeometryResult & geometry) const;
  void ApplyProtectedRegionGuard(LoopTaskComputation * computation);
  void RememberRejectedLoopRegions(const LoopTaskComputation & computation);
  void InvalidateRejectedLoopRegions(const std::set<RawSubmapId> & submaps);
  LoopTaskComputation CommitLoopFusion(LoopTaskComputation computation);
  AcceptedPoseBatchResult CommitGraphProposal(
    const PoseGraphProblem & problem,
    const OptimizationProposal & proposal,
    PoseSourceKind source_kind,
    uint64_t source_task_id,
    const std::optional<RawKeyFrameId> & hard_fiducial_keyframe = std::nullopt);
  ScoreChangeSet RefreshGeometryScores(
    const std::set<RawKeyFrameId> & keyframe_ids,
    const std::set<RawMapPointId> & mappoint_ids,
    const std::vector<RawMapPointId> & removals);
  void RefreshScoresAfterPoseChanges(const std::vector<PoseChangeSet> & changes);

  RawMapDatabase raw_database_;
  GlobalPoseStore pose_store_;
  FiducialAnchorManager fiducial_anchor_manager_;
  PoseGraphBuilder pose_graph_builder_;
  OptimizationManager optimization_manager_;
  OptimizationValidator optimization_validator_;
  LandmarkScoreManager score_manager_;
  GlobalMapBuilder global_map_builder_;
  CovisibilityDatabase covisibility_database_;
  LoopPipeline loop_pipeline_;
  FusedLandmarkManager fused_landmark_manager_;
  LoopPipelineConfig loop_pipeline_config_;
  bool deferred_snapshot_dirty_ = false;
  FiducialOptimizationConfig fiducial_optimization_config_;
  std::map<uint32_t, RawSubmapId> last_submap_by_drone_;
  std::map<RawSubmapId, RecentLossContinuityRecord> recent_loss_continuity_;
  std::set<LoopRejectionKey> loop_rejection_ledger_;
  // Orden de locks: state_commit -> componentes; builder y ledger nunca se toman al revés.
  mutable std::mutex state_commit_mutex_;
  mutable std::mutex loop_pipeline_mutex_;
  mutable std::mutex loop_rejection_mutex_;
  mutable std::mutex builder_mutex_;
};

}  // namespace orbslam3_multi
