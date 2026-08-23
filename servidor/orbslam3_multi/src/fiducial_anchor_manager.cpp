#include "orbslam3_multi/fiducial_anchor_manager.hpp"

#include "orbslam3_multi/pose_geometry.hpp"

#include <Eigen/Geometry>

#include <cmath>

namespace orbslam3_multi
{
namespace
{

bool PoseToMatrix(const geometry_msgs::msg::Pose & pose, Eigen::Matrix4d * matrix)
{
  const Eigen::Quaterniond quaternion(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
    !std::isfinite(pose.position.z) || !std::isfinite(quaternion.w()) ||
    !std::isfinite(quaternion.x()) || !std::isfinite(quaternion.y()) ||
    !std::isfinite(quaternion.z()) || quaternion.norm() < 1e-9)
  {
    return false;
  }
  *matrix = Eigen::Matrix4d::Identity();
  matrix->block<3, 3>(0, 0) = quaternion.normalized().toRotationMatrix();
  (*matrix)(0, 3) = pose.position.x;
  (*matrix)(1, 3) = pose.position.y;
  (*matrix)(2, 3) = pose.position.z;
  return matrix->allFinite();
}

geometry_msgs::msg::Pose MatrixToPose(const Eigen::Matrix4d & matrix)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = matrix(0, 3);
  pose.position.y = matrix(1, 3);
  pose.position.z = matrix(2, 3);
  const Eigen::Quaterniond quaternion(matrix.block<3, 3>(0, 0));
  const auto normalized = quaternion.normalized();
  pose.orientation.x = normalized.x();
  pose.orientation.y = normalized.y();
  pose.orientation.z = normalized.z();
  pose.orientation.w = normalized.w();
  return pose;
}

}  // namespace

const char * ToString(FiducialProcessStatus status)
{
  switch (status) {
    case FiducialProcessStatus::AnchorCreated:
      return "anchor_created";
    case FiducialProcessStatus::RevisitWithinThreshold:
      return "revisit_within_threshold";
    case FiducialProcessStatus::OptimizationRequired:
      return "optimization_required";
    case FiducialProcessStatus::MissingRawKeyFrame:
      return "missing_raw_keyframe";
    case FiducialProcessStatus::MissingGlobalPose:
      return "missing_global_pose";
    case FiducialProcessStatus::MissingControlKeyFrame:
      return "missing_control_keyframe";
    case FiducialProcessStatus::InvalidObservation:
      return "invalid_observation";
    case FiducialProcessStatus::AnchorCommitRejected:
      return "anchor_commit_rejected";
  }
  return "unknown";
}

const char * ToString(FiducialTaskDecision decision)
{
  switch (decision) {
    case FiducialTaskDecision::Ready:
      return "ready";
    case FiducialTaskDecision::Stale:
      return "stale";
    case FiducialTaskDecision::Invalid:
      return "invalid";
  }
  return "unknown";
}

void FiducialAnchorManager::Configure(const FiducialOptimizationConfig & config)
{
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
}

FiducialProcessResult FiducialAnchorManager::Evaluate(
  const FiducialObservation & observation,
  const geometry_msgs::msg::Pose & local_T_camera,
  const std::optional<GlobalPoseRecord> & current_global_pose,
  bool submap_already_anchored)
{
  FiducialProcessResult result;
  result.observation = observation;
  if (observation.arrival_id == 0 || observation.keyframe_id.drone_id == 0 ||
    observation.fiducial_id <= 0 || observation.fiducial_visit_id == 0 ||
    observation.source.empty())
  {
    result.reason = "identity_or_source_invalid";
    return result;
  }

  Eigen::Matrix4d world_T_camera;
  Eigen::Matrix4d local_T_camera_matrix;
  if (!PoseToMatrix(observation.world_T_camera_target, &world_T_camera) ||
    !PoseToMatrix(local_T_camera, &local_T_camera_matrix))
  {
    result.reason = "pose_invalid";
    return result;
  }

  result.world_T_local = MatrixToPose(world_T_camera * local_T_camera_matrix.inverse());
  if (!submap_already_anchored) {
    result.status = FiducialProcessStatus::AnchorCreated;
    result.promote_control = true;
    result.reason = "first_valid_observation";
    return result;
  }

  if (!current_global_pose.has_value()) {
    result.status = FiducialProcessStatus::MissingGlobalPose;
    result.reason = "global_pose_missing";
    return result;
  }
  result.error = ComputeFiducialError(
    current_global_pose->world_pose, observation.world_T_camera_target);

  FiducialOptimizationConfig config;
  std::optional<ControlState> control;
  bool is_first_keyframe_of_visit = false;
  uint64_t task_id = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    config = config_;
    const RawSubmapId submap_id{
      observation.keyframe_id.drone_id, observation.keyframe_id.map_epoch};
    auto & visit_first_keyframes = visit_first_keyframes_[submap_id];
    const auto [visit, inserted] = visit_first_keyframes.emplace(
      observation.fiducial_visit_id, observation.keyframe_id);
    is_first_keyframe_of_visit = inserted || visit->second == observation.keyframe_id;
    const auto found = controls_.find(submap_id);
    if (found != controls_.end()) {
      control = found->second;
    }
    task_id = next_task_id_;
  }

  const bool within_threshold =
    result.error.translation_m <= config.translation_threshold_m &&
    result.error.rotation_rad <= config.rotation_threshold_rad &&
    result.error.yaw_rad <= config.yaw_threshold_rad;
  if (within_threshold) {
    result.status = FiducialProcessStatus::RevisitWithinThreshold;
    result.promote_control = is_first_keyframe_of_visit &&
      (!control.has_value() || control->fiducial_visit_id != observation.fiducial_visit_id);
    result.reason = result.promote_control ?
      "first_coherent_keyframe_of_visit" : "same_visit_within_threshold";
    return result;
  }
  if (!control.has_value()) {
    result.status = FiducialProcessStatus::MissingControlKeyFrame;
    result.reason = "last_accepted_control_missing";
    return result;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    task_id = next_task_id_++;
  }
  FiducialOptimizationTask task;
  task.task_id = task_id;
  task.observation_arrival_id = observation.arrival_id;
  task.submap_id = {
    observation.keyframe_id.drone_id, observation.keyframe_id.map_epoch};
  task.keyframe_id = observation.keyframe_id;
  task.control_keyframe_id = control->keyframe_id;
  task.fiducial_id = observation.fiducial_id;
  task.fiducial_visit_id = observation.fiducial_visit_id;
  task.target_world_T_kf = observation.world_T_camera_target;
  task.enqueue_error = result.error;
  task.observation_pose_revision = current_global_pose->pose_revision;
  task.source = observation.source;
  result.status = FiducialProcessStatus::OptimizationRequired;
  result.optimization_task = task;
  result.reason = "fiducial_error_above_threshold";
  return result;
}

void FiducialAnchorManager::AcceptControl(
  const RawSubmapId & submap_id, uint64_t fiducial_visit_id,
  const RawKeyFrameId & keyframe_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  controls_[submap_id] = {fiducial_visit_id, keyframe_id};
  visit_first_keyframes_[submap_id].try_emplace(fiducial_visit_id, keyframe_id);
}

std::optional<RawKeyFrameId> FiducialAnchorManager::GetLastAcceptedControl(
  const RawSubmapId & submap_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = controls_.find(submap_id);
  return found == controls_.end() ?
         std::nullopt : std::optional<RawKeyFrameId>(found->second.keyframe_id);
}

}  // namespace orbslam3_multi
