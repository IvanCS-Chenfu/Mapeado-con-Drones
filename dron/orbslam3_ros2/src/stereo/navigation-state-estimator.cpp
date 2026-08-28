#include "navigation-state-estimator.hpp"

#include <algorithm>

namespace orbslam3_ros2
{

OrbPosePredictor::OrbPosePredictor(const OrbPosePredictorConfig & config)
: config_(config)
{
}

PredictedOrbPoseState OrbPosePredictor::UpdateMeasurement(
  const Sophus::SE3f & measurement,
  double stamp_sec)
{
  last_update_limited_ = false;
  last_orientation_rejected_ = false;
  last_position_innovation_m_ = 0.0f;
  last_rotation_innovation_rad_ = 0.0f;
  last_rotation_step_rad_ = 0.0f;
  if (!valid_) {
    pose_ = measurement;
    stamp_sec_ = stamp_sec;
    linear_velocity_.setZero();
    angular_velocity_.setZero();
    velocity_valid_ = false;
    valid_ = true;
    return Predict(stamp_sec);
  }

  const double dt = stamp_sec - stamp_sec_;
  if (dt <= 1e-4 || dt > 0.5) {
    Reset();
    pose_ = measurement;
    stamp_sec_ = stamp_sec;
    valid_ = true;
    return Predict(stamp_sec);
  }

  const Sophus::SE3f predicted = Propagate(dt);
  Eigen::Vector3f position_innovation =
    measurement.translation() - predicted.translation();
  last_position_innovation_m_ = position_innovation.norm();
  position_innovation = ClampNorm(
    position_innovation,
    config_.max_position_innovation_m,
    last_update_limited_);

  const Eigen::Vector3f rotation_step =
    (measurement.so3() * pose_.so3().inverse()).log();
  last_rotation_step_rad_ = rotation_step.norm();
  const Eigen::Vector3f rotation_innovation =
    (measurement.so3() * predicted.so3().inverse()).log();
  last_rotation_innovation_rad_ = rotation_innovation.norm();
  last_orientation_rejected_ =
    (config_.max_rotation_innovation_rad > 0.0f &&
    last_rotation_innovation_rad_ > config_.max_rotation_innovation_rad);

  const Eigen::Vector3f target_position =
    predicted.translation() + config_.position_alpha * position_innovation;
  Eigen::Vector3f desired_linear_velocity =
    (target_position - pose_.translation()) / static_cast<float>(dt);
  desired_linear_velocity = ClampNorm(
    desired_linear_velocity, config_.max_linear_speed_mps, last_update_limited_);
  if (velocity_valid_ && config_.max_linear_acceleration_mps2 > 0.0f) {
    Eigen::Vector3f velocity_delta = desired_linear_velocity - linear_velocity_;
    velocity_delta = ClampNorm(
      velocity_delta,
      config_.max_linear_acceleration_mps2 * static_cast<float>(dt),
      last_update_limited_);
    desired_linear_velocity = linear_velocity_ + velocity_delta;
  }
  linear_velocity_ = desired_linear_velocity;

  Sophus::SO3f target_orientation = predicted.so3();
  if (last_orientation_rejected_) {
    ++consecutive_angular_rejections_;
    last_update_limited_ = true;
  } else {
    consecutive_angular_rejections_ = 0;
    target_orientation =
      Sophus::SO3f::exp(config_.orientation_alpha * rotation_innovation) *
      predicted.so3();
  }
  Eigen::Vector3f desired_angular_velocity =
    (target_orientation * pose_.so3().inverse()).log() /
    static_cast<float>(dt);
  desired_angular_velocity = ClampNorm(
    desired_angular_velocity,
    config_.max_angular_speed_radps,
    last_update_limited_);
  if (velocity_valid_ && config_.max_angular_acceleration_radps2 > 0.0f) {
    Eigen::Vector3f velocity_delta =
      desired_angular_velocity - angular_velocity_;
    velocity_delta = ClampNorm(
      velocity_delta,
      config_.max_angular_acceleration_radps2 * static_cast<float>(dt),
      last_update_limited_);
    desired_angular_velocity = angular_velocity_ + velocity_delta;
  }
  angular_velocity_ = desired_angular_velocity;
  pose_ = Sophus::SE3f(
    Sophus::SO3f::exp(angular_velocity_ * static_cast<float>(dt)) * pose_.so3(),
    pose_.translation() + linear_velocity_ * static_cast<float>(dt));
  velocity_valid_ = true;
  stamp_sec_ = stamp_sec;
  return Predict(stamp_sec);
}

PredictedOrbPoseState OrbPosePredictor::Predict(double stamp_sec) const
{
  PredictedOrbPoseState result;
  if (!valid_) {
    return result;
  }
  const double dt = std::max(
    0.0,
    std::min(stamp_sec - stamp_sec_, config_.max_extrapolation_sec));
  result.valid = true;
  result.velocity_valid = velocity_valid_;
  result.pose = Propagate(dt);
  result.linear_velocity = linear_velocity_;
  result.angular_velocity = angular_velocity_;
  return result;
}

void OrbPosePredictor::Reset()
{
  valid_ = false;
  velocity_valid_ = false;
  linear_velocity_.setZero();
  angular_velocity_.setZero();
  consecutive_angular_rejections_ = 0;
  last_orientation_rejected_ = false;
}

bool OrbPosePredictor::last_update_limited() const
{
  return last_update_limited_;
}

bool OrbPosePredictor::last_orientation_rejected() const
{
  return last_orientation_rejected_;
}

bool OrbPosePredictor::healthy() const
{
  return config_.max_consecutive_angular_rejections == 0 ||
         consecutive_angular_rejections_ <
         config_.max_consecutive_angular_rejections;
}

uint32_t OrbPosePredictor::consecutive_angular_rejections() const
{
  return consecutive_angular_rejections_;
}

float OrbPosePredictor::last_position_innovation_m() const
{
  return last_position_innovation_m_;
}

float OrbPosePredictor::last_rotation_innovation_rad() const
{
  return last_rotation_innovation_rad_;
}

float OrbPosePredictor::last_rotation_step_rad() const
{
  return last_rotation_step_rad_;
}

Eigen::Vector3f OrbPosePredictor::ClampNorm(
  const Eigen::Vector3f & value,
  float max_norm,
  bool & limited)
{
  const float norm = value.norm();
  if (max_norm > 0.0f && norm > max_norm) {
    limited = true;
    return value * (max_norm / norm);
  }
  return value;
}

Sophus::SE3f OrbPosePredictor::Propagate(double dt) const
{
  if (!velocity_valid_ || dt <= 0.0) {
    return pose_;
  }
  return Sophus::SE3f(
    Sophus::SO3f::exp(angular_velocity_ * static_cast<float>(dt)) * pose_.so3(),
    pose_.translation() + linear_velocity_ * static_cast<float>(dt));
}

NavigationStateEstimator::NavigationStateEstimator(const ReferenceGateConfig & config)
: config_(config)
{
}

ContinuousPoseResult NavigationStateEstimator::CurrentResult() const
{
  ContinuousPoseResult result;
  result.local_valid = initialized_ && !gap_since_valid_;
  result.continuity_valid = result.local_valid;
  result.initialized = initialized_;
  result.active_reference_valid = initialized_;
  result.active_reference_keyframe_id = reference_keyframe_id_;
  result.active_tcr = active_tcr_;
  result.o_t_camera = last_o_t_camera_;
  result.global_state = global_state_;
  result.pose_revision = global_pose_revision_;
  if (global_state_ != GlobalPoseState::Invalid) {
    result.w_t_camera = last_w_t_camera_;
  }
  return result;
}

void NavigationStateEstimator::ClearPendingReference()
{
  pending_reference_valid_ = false;
  pending_reference_keyframe_id_ = 0;
  pending_o_t_reference_ = Sophus::SE3f();
  pending_last_o_t_camera_ = Sophus::SE3f();
  pending_last_local_t_camera_ = Sophus::SE3f();
  pending_good_frames_ = 0;
}

ContinuousPoseResult NavigationStateEstimator::Update(
  uint64_t map_epoch,
  bool tracking_valid,
  bool reference_valid,
  uint64_t reference_keyframe_id,
  bool tcr_valid,
  const Sophus::SE3f & tcr,
  const Sophus::SE3f & local_t_camera)
{
  constexpr float kThresholdEpsilon = 1e-6f;
  ContinuousPoseResult result;
  if (!tracking_valid || !reference_valid || !tcr_valid) {
    gap_since_valid_ = initialized_;
    ClearPendingReference();
    pending_total_frames_ = 0;
    return result;
  }

  result.local_valid = true;
  const bool new_epoch = initialized_ && map_epoch != map_epoch_;
  if (!initialized_ || new_epoch) {
    if (new_epoch) {
      global_state_ = GlobalPoseState::Invalid;
      global_pose_revision_ = 0;
      w_t_reference_ = Sophus::SE3f();
      last_w_t_camera_ = Sophus::SE3f();
    }
    o_t_reference_ = local_t_camera * tcr;
    last_o_t_camera_ = o_t_reference_ * tcr.inverse();
    last_local_t_camera_ = local_t_camera;
    initialized_ = true;
    gap_since_valid_ = false;
    map_epoch_ = map_epoch;
    reference_keyframe_id_ = reference_keyframe_id;
    active_tcr_ = tcr;
    ClearPendingReference();
    pending_total_frames_ = 0;
    result.continuity_valid = true;
    result.initialized = true;
    result.epoch_changed = new_epoch;
    result.measurement_accepted = true;
    result.active_reference_valid = true;
    result.active_reference_keyframe_id = reference_keyframe_id_;
    result.active_tcr = active_tcr_;
    result.o_t_camera = last_o_t_camera_;
    return result;
  }

  if (gap_since_valid_) {
    o_t_reference_ = local_t_camera * tcr;
    last_o_t_camera_ = o_t_reference_ * tcr.inverse();
    last_local_t_camera_ = local_t_camera;
    gap_since_valid_ = false;
    reference_keyframe_id_ = reference_keyframe_id;
    active_tcr_ = tcr;
    ClearPendingReference();
    pending_total_frames_ = 0;
    result.initialized = true;
    result.measurement_accepted = true;
    result.active_reference_valid = true;
    result.active_reference_keyframe_id = reference_keyframe_id_;
    result.active_tcr = active_tcr_;
    result.o_t_camera = last_o_t_camera_;
    return result;
  }

  const Sophus::SE3f chain_o_t_camera = pending_reference_valid_ ?
    pending_last_o_t_camera_ : last_o_t_camera_;
  const Sophus::SE3f chain_local_t_camera = pending_reference_valid_ ?
    pending_last_local_t_camera_ : last_local_t_camera_;

  if (reference_keyframe_id == reference_keyframe_id_) {
    const Sophus::SE3f next_o_t_camera = o_t_reference_ * tcr.inverse();
    const Sophus::SE3f chain_step =
      chain_o_t_camera.inverse() * next_o_t_camera;
    const float chain_translation_m = chain_step.translation().norm();
    const float chain_rotation_rad = chain_step.so3().log().norm();
    const bool returning_active_plausible =
      !pending_reference_valid_ ||
      ((config_.max_step_translation_m <= 0.0f ||
      chain_translation_m <= config_.max_step_translation_m + kThresholdEpsilon) &&
      (config_.max_step_rotation_rad <= 0.0f ||
      chain_rotation_rad <= config_.max_step_rotation_rad + kThresholdEpsilon));
    if (!returning_active_plausible) {
      ++pending_total_frames_;
      if (
        config_.max_pending_frames > 0 &&
        pending_total_frames_ > config_.max_pending_frames)
      {
        gap_since_valid_ = true;
        ClearPendingReference();
        pending_total_frames_ = 0;
        result.local_valid = false;
        result.continuity_valid = false;
        result.reference_gate_timed_out = true;
        return result;
      }
      result = CurrentResult();
      result.reference_pending = true;
      result.reference_rejected = true;
      result.step_translation_m = chain_translation_m;
      result.step_rotation_rad = chain_rotation_rad;
      return result;
    }

    const Sophus::SE3f published_step =
      last_o_t_camera_.inverse() * next_o_t_camera;
    ClearPendingReference();
    pending_total_frames_ = 0;
    last_o_t_camera_ = next_o_t_camera;
    last_local_t_camera_ = local_t_camera;
    active_tcr_ = tcr;
    if (global_state_ != GlobalPoseState::Invalid) {
      last_w_t_camera_ = w_t_reference_ * tcr.inverse();
    }
    result = CurrentResult();
    result.measurement_accepted = true;
    result.step_translation_m = published_step.translation().norm();
    result.step_rotation_rad = published_step.so3().log().norm();
    return result;
  }

  ++pending_total_frames_;
  if (
    config_.max_pending_frames > 0 &&
    pending_total_frames_ > config_.max_pending_frames)
  {
    gap_since_valid_ = true;
    ClearPendingReference();
    pending_total_frames_ = 0;
    result.local_valid = false;
    result.continuity_valid = false;
    result.reference_gate_timed_out = true;
    return result;
  }

  Sophus::SE3f candidate_o_t_reference;
  Sophus::SE3f candidate_o_t_camera;
  if (
    pending_reference_valid_ &&
    pending_reference_keyframe_id_ == reference_keyframe_id)
  {
    candidate_o_t_reference = pending_o_t_reference_;
    candidate_o_t_camera = candidate_o_t_reference * tcr.inverse();
  } else {
    Sophus::SE3f local_increment =
      chain_local_t_camera.inverse() * local_t_camera;
    const float local_translation_m = local_increment.translation().norm();
    const float local_rotation_rad = local_increment.so3().log().norm();
    const bool local_increment_plausible =
      (config_.max_step_translation_m <= 0.0f ||
      local_translation_m <= config_.max_step_translation_m + kThresholdEpsilon) &&
      (config_.max_step_rotation_rad <= 0.0f ||
      local_rotation_rad <= config_.max_step_rotation_rad + kThresholdEpsilon);
    if (!local_increment_plausible) {
      local_increment = Sophus::SE3f();
    }
    candidate_o_t_camera = chain_o_t_camera * local_increment;
    candidate_o_t_reference = candidate_o_t_camera * tcr;
  }

  const Sophus::SE3f candidate_step =
    chain_o_t_camera.inverse() * candidate_o_t_camera;
  const float candidate_translation_m = candidate_step.translation().norm();
  const float candidate_rotation_rad = candidate_step.so3().log().norm();
  const bool candidate_plausible =
    (config_.max_step_translation_m <= 0.0f ||
    candidate_translation_m <= config_.max_step_translation_m + kThresholdEpsilon) &&
    (config_.max_step_rotation_rad <= 0.0f ||
    candidate_rotation_rad <= config_.max_step_rotation_rad + kThresholdEpsilon);

  if (!candidate_plausible) {
    result = CurrentResult();
    result.reference_pending = true;
    result.reference_rejected = true;
    result.step_translation_m = candidate_translation_m;
    result.step_rotation_rad = candidate_rotation_rad;
    return result;
  }

  pending_reference_valid_ = true;
  pending_reference_keyframe_id_ = reference_keyframe_id;
  pending_o_t_reference_ = candidate_o_t_reference;
  pending_last_o_t_camera_ = candidate_o_t_camera;
  pending_last_local_t_camera_ = local_t_camera;
  ++pending_good_frames_;
  if (pending_good_frames_ < std::max(1U, config_.confirmation_frames)) {
    result = CurrentResult();
    result.reference_pending = true;
    result.step_translation_m = candidate_translation_m;
    result.step_rotation_rad = candidate_rotation_rad;
    return result;
  }

  const Sophus::SE3f step = last_o_t_camera_.inverse() * candidate_o_t_camera;
  o_t_reference_ = candidate_o_t_reference;
  last_o_t_camera_ = candidate_o_t_camera;
  last_local_t_camera_ = local_t_camera;
  active_tcr_ = tcr;
  reference_keyframe_id_ = reference_keyframe_id;
  if (global_state_ != GlobalPoseState::Invalid) {
    w_t_reference_ = last_w_t_camera_ * tcr;
    global_state_ = GlobalPoseState::Provisional;
    global_pose_revision_ = 0;
    last_w_t_camera_ = w_t_reference_ * tcr.inverse();
  }
  ClearPendingReference();
  pending_total_frames_ = 0;

  result = CurrentResult();
  result.reference_changed = true;
  result.measurement_accepted = true;
  result.step_translation_m = step.translation().norm();
  result.step_rotation_rad = step.so3().log().norm();
  return result;
}

bool NavigationStateEstimator::ApplyAuthoritativeGlobalPose(
  uint64_t map_epoch,
  uint64_t reference_keyframe_id,
  uint64_t pose_revision,
  const Sophus::SE3f & w_t_reference)
{
  if (!initialized_ || map_epoch != map_epoch_ ||
    reference_keyframe_id != reference_keyframe_id_ || pose_revision == 0)
  {
    return false;
  }
  if (global_state_ == GlobalPoseState::Authoritative &&
    pose_revision <= global_pose_revision_)
  {
    return false;
  }
  w_t_reference_ = w_t_reference;
  global_pose_revision_ = pose_revision;
  global_state_ = GlobalPoseState::Authoritative;
  return true;
}

void NavigationStateEstimator::InvalidateGlobalPose(
  uint64_t map_epoch,
  uint64_t reference_keyframe_id)
{
  if (initialized_ && map_epoch == map_epoch_ &&
    reference_keyframe_id == reference_keyframe_id_)
  {
    global_state_ = GlobalPoseState::Invalid;
    global_pose_revision_ = 0;
  }
}

void NavigationStateEstimator::Reset()
{
  initialized_ = false;
  gap_since_valid_ = false;
  map_epoch_ = 0;
  reference_keyframe_id_ = 0;
  o_t_reference_ = Sophus::SE3f();
  last_o_t_camera_ = Sophus::SE3f();
  last_local_t_camera_ = Sophus::SE3f();
  active_tcr_ = Sophus::SE3f();
  ClearPendingReference();
  pending_total_frames_ = 0;
  global_state_ = GlobalPoseState::Invalid;
  global_pose_revision_ = 0;
  w_t_reference_ = Sophus::SE3f();
  last_w_t_camera_ = Sophus::SE3f();
}

}  // namespace orbslam3_ros2
