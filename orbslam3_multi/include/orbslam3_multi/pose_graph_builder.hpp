#pragma once

#include "orbslam3_multi/pose_graph_problem.hpp"
#include "orbslam3_multi/loop_pipeline.hpp"
#include "orbslam3_multi/raw_map_types.hpp"

#include <map>

namespace orbslam3_multi
{

class PoseGraphBuilder
{
public:
  explicit PoseGraphBuilder(FiducialOptimizationConfig config = {});

  void Configure(const FiducialOptimizationConfig & config);
  PoseGraphBuildResult Build(
    const FiducialOptimizationTask & task,
    const RawSubmapPoseSnapshot & raw_snapshot,
    const std::map<RawKeyFrameId, GlobalPoseRecord> & poses,
    uint64_t pose_store_revision,
    const CovisibilityDatabase * covisibility_database = nullptr) const;
  PoseGraphBuildResult BuildLoop(
    const LoopTaskComputation & computation,
    const std::map<RawSubmapId, RawSubmapPoseSnapshot> & raw_snapshots,
    const std::map<RawKeyFrameId, GlobalPoseRecord> & poses,
    uint64_t pose_store_revision,
    const CovisibilityDatabase & covisibility_database,
    const LoopPipelineConfig & loop_config,
    const std::map<RawSubmapId, LoopAnchorDependencySnapshot> &
    loop_dependencies = {}) const;

private:
  FiducialOptimizationConfig config_;
};

}  // namespace orbslam3_multi
