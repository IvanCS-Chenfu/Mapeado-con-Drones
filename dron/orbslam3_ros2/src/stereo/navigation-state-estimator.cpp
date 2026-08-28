#include "navigation-state-estimator.hpp"

#include <algorithm>

namespace orbslam3_ros2
{

namespace
{
constexpr float kConsistencyComparisonEpsilon = 1e-5f;
}

const char * AngularCorrectionClassName(AngularCorrectionClass value)
{
  switch (value) {
    case AngularCorrectionClass::Initializing:
      return "INITIALIZING";
    case AngularCorrectionClass::Small:
      return "SMALL";
    case AngularCorrectionClass::ModeratePending:
      return "MODERATE_PENDING";
    case AngularCorrectionClass::ModerateConfirmed:
      return "MODERATE_CONFIRMED";
    case AngularCorrectionClass::ModerateDiscarded:
      return "MODERATE_DISCARDED";
    case AngularCorrectionClass::RejectedExcessive:
      return "REJECTED_EXCESSIVE";
  }
  return "UNKNOWN";
}

OrbPosePredictor::OrbPosePredictor(const OrbPosePredictorConfig & config)
: config_(config)
{
}

PredictedOrbPoseState OrbPosePredictor::UpdateMeasurement(
  const Sophus::SE3f & measurement,
  double stamp_sec)
{
  return UpdateMeasurement(measurement, stamp_sec, OrbMeasurementContext{});
}

PredictedOrbPoseState OrbPosePredictor::UpdateMeasurement(
  const Sophus::SE3f & measurement,
  double stamp_sec,
  const OrbMeasurementContext & context)
{
  last_update_limited_ = false;
  last_orientation_rejected_ = false;
  last_position_innovation_m_ = 0.0f;
  last_rotation_innovation_rad_ = 0.0f;
  last_rotation_step_rad_ = 0.0f;
  last_diagnostics_ = OrbPosePredictorDiagnostics{};
  last_diagnostics_.measurement_processed = true;
  last_diagnostics_.measurement_stamp_sec = stamp_sec;
  last_diagnostics_.context = context;
  last_diagnostics_.post_reference_switch =
    context.reference_changed ||
    context.frames_since_reference_change <= config_.post_reference_switch_frames;
  if (!valid_) {
    pose_ = measurement;
    stamp_sec_ = stamp_sec;
    linear_velocity_.setZero();
    angular_velocity_.setZero();
    velocity_valid_ = false;
    valid_ = true;
    raw_measurement_valid_ = true;
    last_raw_measurement_ = measurement;
    last_raw_stamp_sec_ = stamp_sec;
    last_diagnostics_.classification = AngularCorrectionClass::Initializing;
    last_diagnostics_.predictor_healthy = true;
    return Predict(stamp_sec);
  }

  const double dt = stamp_sec - stamp_sec_;
  if (dt <= 1e-4 || dt > 0.5) {
    Reset();
    return UpdateMeasurement(measurement, stamp_sec, context);
  }
  last_diagnostics_.dt_sec = dt;
  last_diagnostics_.previous_linear_velocity = linear_velocity_;
  last_diagnostics_.previous_angular_velocity = angular_velocity_;

  if (raw_measurement_valid_) {
    last_diagnostics_.raw_rotation_step =
      (measurement.so3() * last_raw_measurement_.so3().inverse()).log();
    last_diagnostics_.raw_step_rotation_rad =
      last_diagnostics_.raw_rotation_step.norm();
    last_diagnostics_.raw_step_translation_m =
      (measurement.translation() - last_raw_measurement_.translation()).norm();
    last_diagnostics_.implied_angular_velocity =
      last_diagnostics_.raw_rotation_step / static_cast<float>(dt);
    last_diagnostics_.implied_angular_acceleration =
      (last_diagnostics_.implied_angular_velocity - angular_velocity_) /
      static_cast<float>(dt);
  }

  const Sophus::SE3f predicted = Propagate(dt);
  Eigen::Vector3f position_innovation =
    measurement.translation() - predicted.translation();
  last_position_innovation_m_ = position_innovation.norm();
  last_diagnostics_.position_innovation = position_innovation;
  last_diagnostics_.position_innovation_m = last_position_innovation_m_;
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
  last_diagnostics_.rotation_innovation = rotation_innovation;
  last_diagnostics_.rotation_innovation_rad = last_rotation_innovation_rad_;

  const float dynamic_small_allowance =
    config_.max_angular_acceleration_radps2 > 0.0f ?
    0.5f * config_.max_angular_acceleration_radps2 *
    static_cast<float>(dt * dt) : 0.0f;
  float small_rotation_limit =
    std::max(0.0f, config_.small_rotation_innovation_rad) +
    dynamic_small_allowance;
  if (config_.max_rotation_innovation_rad > 0.0f) {
    small_rotation_limit = std::min(
      small_rotation_limit, config_.max_rotation_innovation_rad);
  }
  last_diagnostics_.small_rotation_limit_rad = small_rotation_limit;

  bool apply_rotation_measurement = false;
  const bool excessive =
    config_.max_rotation_innovation_rad > 0.0f &&
    last_rotation_innovation_rad_ > config_.max_rotation_innovation_rad;
  const bool small = last_rotation_innovation_rad_ <= small_rotation_limit;
  const bool same_pending_context =
    pending_map_epoch_ == context.map_epoch &&
    pending_reference_keyframe_id_ == context.reference_keyframe_id;
  const auto snapshot_pending_diagnostics = [this]() {
      last_diagnostics_.pending_correction_id = pending_correction_id_;
      last_diagnostics_.pending_good_frames = pending_good_frames_;
      last_diagnostics_.pending_total_frames = pending_total_frames_;
    };

  if (excessive) {
    last_diagnostics_.classification = AngularCorrectionClass::RejectedExcessive;
    last_orientation_rejected_ = true;
    ++consecutive_angular_rejections_;
    last_update_limited_ = true;
    snapshot_pending_diagnostics();
    ClearModerateState();
  } else if (small) {
    last_diagnostics_.classification = moderate_pending_valid_ ?
      AngularCorrectionClass::ModerateDiscarded : AngularCorrectionClass::Small;
    snapshot_pending_diagnostics();
    ClearModerateState();
    consecutive_angular_rejections_ = 0;
    apply_rotation_measurement = true;
  } else if (moderate_confirmed_active_) {
    const float previous_norm = pending_innovation_.norm();
    const float current_norm = rotation_innovation.norm();
    const float cosine =
      previous_norm > 1e-6f && current_norm > 1e-6f ?
      pending_innovation_.dot(rotation_innovation) /
      (previous_norm * current_norm) : 1.0f;
    last_diagnostics_.consistency_cosine = cosine;
    const bool confirmed_consistent =
      same_pending_context &&
      cosine + kConsistencyComparisonEpsilon >=
      config_.moderate_direction_consistency;
    if (confirmed_consistent) {
      last_diagnostics_.classification = AngularCorrectionClass::ModerateConfirmed;
      pending_innovation_ =
        0.5f * pending_innovation_ + 0.5f * rotation_innovation;
      ++pending_total_frames_;
      ++pending_good_frames_;
      apply_rotation_measurement = true;
      consecutive_angular_rejections_ = 0;
    } else {
      last_diagnostics_.classification = AngularCorrectionClass::ModerateDiscarded;
      snapshot_pending_diagnostics();
      ClearModerateState();
      StartModeratePending(
        rotation_innovation, stamp_sec, context,
        last_diagnostics_.post_reference_switch);
      consecutive_angular_rejections_ = 0;
    }
  } else if (!moderate_pending_valid_ || !same_pending_context) {
    if (moderate_pending_valid_) {
      last_diagnostics_.classification = AngularCorrectionClass::ModerateDiscarded;
      snapshot_pending_diagnostics();
      ClearModerateState();
    } else {
      last_diagnostics_.classification = AngularCorrectionClass::ModeratePending;
    }
    StartModeratePending(
      rotation_innovation, stamp_sec, context,
      last_diagnostics_.post_reference_switch);
    consecutive_angular_rejections_ = 0;
  } else {
    ++pending_total_frames_;
    const float pending_norm = pending_innovation_.norm();
    const float current_norm = rotation_innovation.norm();
    const float cosine =
      pending_norm > 1e-6f && current_norm > 1e-6f ?
      pending_innovation_.dot(rotation_innovation) /
      (pending_norm * current_norm) : 1.0f;
    const float magnitude_ratio =
      std::max(pending_norm, current_norm) > 1e-6f ?
      std::min(pending_norm, current_norm) /
      std::max(pending_norm, current_norm) : 1.0f;
    last_diagnostics_.consistency_cosine = cosine;
    last_diagnostics_.magnitude_ratio = magnitude_ratio;

    const float raw_speed = last_diagnostics_.implied_angular_velocity.norm();
    const float raw_acceleration =
      last_diagnostics_.implied_angular_acceleration.norm();
    const float speed_allowance =
      std::max(0.0f, config_.max_angular_speed_radps) +
      std::max(0.0f, config_.max_angular_acceleration_radps2) *
      static_cast<float>(dt);
    const bool raw_motion_plausible =
      config_.max_angular_speed_radps <= 0.0f || raw_speed <= speed_allowance;
    const bool raw_acceleration_plausible =
      config_.max_angular_acceleration_radps2 <= 0.0f ||
      raw_acceleration <= 2.0f * config_.max_angular_acceleration_radps2;
    const bool persistent_offset =
      last_diagnostics_.raw_step_rotation_rad <= small_rotation_limit;
    const bool coherent =
      cosine + kConsistencyComparisonEpsilon >=
      config_.moderate_direction_consistency &&
      magnitude_ratio + kConsistencyComparisonEpsilon >=
      config_.moderate_magnitude_ratio &&
      (persistent_offset || (raw_motion_plausible && raw_acceleration_plausible));
    const bool pending_timed_out =
      config_.moderate_timeout_sec > 0.0 &&
      stamp_sec - pending_started_stamp_sec_ > config_.moderate_timeout_sec;
    const bool pending_exhausted =
      config_.moderate_max_pending_frames > 0 &&
      pending_total_frames_ > config_.moderate_max_pending_frames;

    if (!coherent || pending_timed_out || pending_exhausted) {
      last_diagnostics_.classification = AngularCorrectionClass::ModerateDiscarded;
      snapshot_pending_diagnostics();
      ClearModerateState();
      if (!pending_timed_out && !pending_exhausted) {
        StartModeratePending(
          rotation_innovation, stamp_sec, context,
          last_diagnostics_.post_reference_switch);
      }
    } else {
      ++pending_good_frames_;
      pending_innovation_ =
        0.5f * pending_innovation_ + 0.5f * rotation_innovation;
      pending_post_reference_switch_ =
        pending_post_reference_switch_ || last_diagnostics_.post_reference_switch;
      const uint32_t required_frames = std::max(
        1U, pending_post_reference_switch_ ?
        config_.moderate_post_reference_confirmation_frames :
        config_.moderate_confirmation_frames);
      if (pending_good_frames_ >= required_frames) {
        moderate_pending_valid_ = false;
        moderate_confirmed_active_ = true;
        last_diagnostics_.classification = AngularCorrectionClass::ModerateConfirmed;
        apply_rotation_measurement = true;
      } else {
        last_diagnostics_.classification = AngularCorrectionClass::ModeratePending;
      }
    }
    consecutive_angular_rejections_ = 0;
  }

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
  if (apply_rotation_measurement) {
    target_orientation =
      Sophus::SO3f::exp(config_.orientation_alpha * rotation_innovation) *
      predicted.so3();
  }
  last_diagnostics_.applied_rotation_correction_rad =
    (target_orientation * predicted.so3().inverse()).log().norm();
  if (last_rotation_innovation_rad_ > 1e-6f) {
    last_diagnostics_.correction_fraction_applied =
      last_diagnostics_.applied_rotation_correction_rad /
      last_rotation_innovation_rad_;
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
  const Sophus::SE3f previous_pose = pose_;
  pose_ = Sophus::SE3f(
    Sophus::SO3f::exp(angular_velocity_ * static_cast<float>(dt)) * pose_.so3(),
    pose_.translation() + linear_velocity_ * static_cast<float>(dt));
  velocity_valid_ = true;
  stamp_sec_ = stamp_sec;
  last_raw_measurement_ = measurement;
  last_raw_stamp_sec_ = stamp_sec;
  raw_measurement_valid_ = true;
  const Sophus::SE3f published_step = previous_pose.inverse() * pose_;
  last_diagnostics_.published_pose_translation_step_m =
    published_step.translation().norm();
  last_diagnostics_.published_pose_rotation_step_rad =
    published_step.so3().log().norm();
  last_diagnostics_.linear_velocity_after_limits = linear_velocity_;
  last_diagnostics_.angular_velocity_after_limits = angular_velocity_;
  if (last_diagnostics_.pending_correction_id == 0) {
    last_diagnostics_.pending_correction_id = pending_correction_id_;
    last_diagnostics_.pending_good_frames = pending_good_frames_;
    last_diagnostics_.pending_total_frames = pending_total_frames_;
  }
  last_diagnostics_.predictor_healthy = healthy();
  last_diagnostics_.consecutive_rejections = consecutive_angular_rejections_;
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
  raw_measurement_valid_ = false;
  next_pending_correction_id_ = 1;
  ClearModerateState();
  last_diagnostics_ = OrbPosePredictorDiagnostics{};
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

const OrbPosePredictorDiagnostics & OrbPosePredictor::last_diagnostics() const
{
  return last_diagnostics_;
}

void OrbPosePredictor::StartModeratePending(
  const Eigen::Vector3f & innovation,
  double stamp_sec,
  const OrbMeasurementContext & context,
  bool post_reference_switch)
{
  moderate_pending_valid_ = true;
  moderate_confirmed_active_ = false;
  pending_correction_id_ = next_pending_correction_id_++;
  pending_started_stamp_sec_ = stamp_sec;
  pending_innovation_ = innovation;
  pending_initial_magnitude_ = innovation.norm();
  pending_good_frames_ = 1;
  pending_total_frames_ = 1;
  pending_map_epoch_ = context.map_epoch;
  pending_reference_keyframe_id_ = context.reference_keyframe_id;
  pending_post_reference_switch_ = post_reference_switch;
}

void OrbPosePredictor::ClearModerateState()
{
  moderate_pending_valid_ = false;
  moderate_confirmed_active_ = false;
  pending_correction_id_ = 0;
  pending_started_stamp_sec_ = 0.0;
  pending_innovation_.setZero();
  pending_initial_magnitude_ = 0.0f;
  pending_good_frames_ = 0;
  pending_total_frames_ = 0;
  pending_map_epoch_ = 0;
  pending_reference_keyframe_id_ = 0;
  pending_post_reference_switch_ = false;
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
