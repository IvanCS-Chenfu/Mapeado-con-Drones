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
#include <mutex>
#include <optional>
#include <string>

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

class SparseGlobalBackend
{
public:
  PrimaryBackendResult InsertDelta(
    uint64_t arrival_id,
    std::shared_ptr<const orbslam3_msgs::msg::OrbMap> delta);
  PrimaryBackendResult InsertFullSnapshot(
    uint64_t arrival_id,
    std::shared_ptr<const orbslam3_msgs::msg::OrbMap> snapshot);

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

  SecondaryWorkPlan PlanSecondaryWork(const RawInsertResult & raw_result) const;
  std::vector<LoopTask> CreateLoopTasks(
    uint64_t source_arrival_id,
    const std::vector<RawKeyFrameId> & keyframe_ids) const;
  CovisibilityUpdateResult ProcessDatabaseUpdate(const DatabaseUpdateTask & task);
  LoopTaskComputation ProcessLoopTask(const LoopTask & task);
  LoopTaskComputation ProcessLoopOptimization(LoopTaskComputation computation);
  CovisibilityDatabaseStats GetCovisibilityStats() const;
  FusedLandmarkStats GetFusedLandmarkStats() const;

  void ConfigureFiducialOptimization(const FiducialOptimizationConfig & config);
  void ConfigureLoopPipeline(const LoopPipelineConfig & config);
  void ConfigureFusedLandmarks(const FusedLandmarkConfig & config);
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
  LoopTaskComputation CommitLoopFusion(LoopTaskComputation computation);
  AcceptedPoseBatchResult CommitLoopProposal(
    const PoseGraphProblem & problem,
    const OptimizationProposal & proposal);

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
  mutable std::mutex state_commit_mutex_;
  mutable std::mutex builder_mutex_;
};

}  // namespace orbslam3_multi
