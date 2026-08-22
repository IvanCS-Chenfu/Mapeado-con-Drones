#pragma once

#include "orbslam3_multi/global_pose_types.hpp"

#include "geometry_msgs/msg/pose.hpp"

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>

namespace orbslam3_multi
{

struct FiducialError
{
  double translation_m = 0.0;
  double rotation_rad = 0.0;
  double yaw_rad = 0.0;
};

struct FiducialOptimizationConfig
{
  double translation_threshold_m = 0.35;
  double rotation_threshold_rad = 0.35;
  double yaw_threshold_rad = 0.25;
  double control_vertex_ratio = 0.30;
  double endpoint_neighborhood_ratio = 0.20;
  uint32_t covisibility_min_support = 15;
  double max_correction_fraction_per_pass = 1.0;
  uint32_t max_refinement_passes = 4;
};

struct FiducialOptimizationTask
{
  uint64_t task_id = 0;
  uint64_t enqueue_sequence = 0;
  uint64_t observation_arrival_id = 0;
  RawSubmapId submap_id;
  RawKeyFrameId keyframe_id;
  RawKeyFrameId control_keyframe_id;
  int32_t fiducial_id = 0;
  uint64_t fiducial_visit_id = 0;
  geometry_msgs::msg::Pose target_world_T_kf;
  FiducialError enqueue_error;
  uint64_t observation_pose_revision = 0;
  bool replaces_soft_loop_anchor = false;
  std::string source;
};

enum class FiducialTaskDecision
{
  Ready,
  Stale,
  Invalid,
};

struct FiducialTaskRevalidation
{
  FiducialTaskDecision decision = FiducialTaskDecision::Invalid;
  FiducialOptimizationTask task;
  FiducialError error;
  std::string reason;
};

struct FiducialCommitResult
{
  bool committed = false;
  bool full_accept = false;
  PoseChangeSet pose_changes;
  FiducialError final_error;
  size_t window_keyframes = 0;
  size_t late_window_keyframes = 0;
  size_t tail_keyframes = 0;
  std::vector<RawKeyFrameId> rerun_keyframe_ids;
  std::string reason;
};

const char * ToString(FiducialTaskDecision decision);

}  // namespace orbslam3_multi
