#pragma once

#include "orbslam3_multi/global_pose_types.hpp"
#include "orbslam3_multi/fiducial_optimization_task.hpp"

#include "geometry_msgs/msg/pose.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace orbslam3_multi
{

struct FiducialObservation
{
  uint64_t arrival_id = 0;
  RawKeyFrameId keyframe_id;
  int32_t fiducial_id = 0;
  uint64_t fiducial_visit_id = 0;
  geometry_msgs::msg::Pose world_T_camera_target;
  double keyframe_stamp_sec = 0.0;
  double observation_stamp_sec = 0.0;
  double association_dt_sec = 0.0;
  double distance_to_fiducial_m = 0.0;
  std::string source;
  std::string quality;
};

enum class FiducialProcessStatus
{
  AnchorCreated,
  RevisitWithinThreshold,
  OptimizationRequired,
  MissingRawKeyFrame,
  MissingGlobalPose,
  MissingControlKeyFrame,
  InvalidObservation,
  AnchorCommitRejected,
};

struct FiducialProcessResult
{
  FiducialProcessStatus status = FiducialProcessStatus::InvalidObservation;
  FiducialObservation observation;
  geometry_msgs::msg::Pose world_T_local;
  PoseChangeSet pose_changes;
  LoopAnchorBatchResult cascade_anchor_commit;
  std::vector<RawKeyFrameId> reconciliation_keyframe_ids;
  FiducialError error;
  std::optional<FiducialOptimizationTask> optimization_task;
  bool journaled = false;
  bool hard_keyframe = false;
  bool promote_control = false;
  std::string reason;
};

const char * ToString(FiducialProcessStatus status);

}  // namespace orbslam3_multi
