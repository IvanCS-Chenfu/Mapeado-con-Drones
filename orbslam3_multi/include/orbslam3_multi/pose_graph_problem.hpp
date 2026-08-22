#pragma once

#include "orbslam3_multi/fiducial_optimization_task.hpp"
#include "orbslam3_multi/loop_task.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <map>
#include <vector>

namespace orbslam3_multi
{

enum class PoseGraphProblemKind
{
  FiducialAbsolute,
  LoopRelative,
};

enum class PoseGraphEdgeKind
{
  Temporal,
  CovisibilityNative,
  PriorLoop,
  CurrentLoop,
};

struct PoseGraphKeyFrame
{
  RawKeyFrameId id;
  geometry_msgs::msg::Pose raw_local_pose;
  geometry_msgs::msg::Pose current_world_pose;
  uint64_t raw_revision = 0;
  uint64_t pose_revision = 0;
  double path_alpha = 0.0;
  bool control = false;
  bool fixed = false;
  bool protected_neighborhood = false;
  bool hard_corridor = false;
  geometry_msgs::msg::Pose hard_corridor_reference_pose;
  double hard_corridor_alpha = 0.0;
};

struct PoseGraphEdge
{
  size_t from_index = 0;
  size_t to_index = 0;
  geometry_msgs::msg::Pose relative_raw_pose;
  size_t supporting_keyframes = 0;
  double information_weight = 1.0;
  PoseGraphEdgeKind kind = PoseGraphEdgeKind::Temporal;
  size_t source_geometry_index = std::numeric_limits<size_t>::max();
};

struct PoseGraphPropagationEntry
{
  size_t keyframe_index = 0;
  size_t lower_control_index = 0;
  size_t upper_control_index = 0;
  double segment_alpha = 0.0;
};

struct PoseGraphSubmapWindow
{
  RawSubmapId submap_id;
  uint64_t raw_submap_revision = 0;
  size_t graph_begin = 0;
  size_t graph_end = 0;
  RawKeyFrameId first_keyframe_id;
  RawKeyFrameId last_keyframe_id;
  RawKeyFrameId continuation_keyframe_id;
};

struct PoseGraphProblem
{
  PoseGraphProblemKind kind = PoseGraphProblemKind::FiducialAbsolute;
  FiducialOptimizationTask task;
  LoopTask loop_task;
  uint64_t raw_submap_revision = 0;
  uint64_t pose_store_revision = 0;
  uint64_t covisibility_revision = 0;
  std::vector<PoseGraphKeyFrame> keyframes;
  std::vector<size_t> control_indices;
  std::vector<PoseGraphEdge> temporal_edges;
  std::vector<PoseGraphEdge> covisibility_edges;
  std::vector<PoseGraphEdge> loop_edges;
  std::vector<PoseGraphPropagationEntry> propagation_plan;
  std::vector<PoseGraphSubmapWindow> submap_windows;
  double loop_translation_threshold_m = 0.0;
  double loop_rotation_threshold_rad = 0.0;
  double structural_temporal_increase_m = 0.0;
  double structural_temporal_increase_rad = 0.0;
  double structural_covisibility_increase_m = 0.0;
  double structural_covisibility_increase_rad = 0.0;
  double structural_prior_loop_increase_m = 0.0;
  double structural_prior_loop_increase_rad = 0.0;
  double hard_corridor_max_translation_m = 0.0;
  double hard_corridor_max_rotation_rad = 0.0;
};

struct PoseGraphBuildResult
{
  bool success = false;
  PoseGraphProblem problem;
  std::string reason;
};

}  // namespace orbslam3_multi
