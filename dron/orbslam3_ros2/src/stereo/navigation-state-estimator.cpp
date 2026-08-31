#include "navigation-state-estimator.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace orbslam3_ros2
{

const char * ActuationCoverageStatusName(ActuationCoverageStatus value)
{
  switch (value) {
    case ActuationCoverageStatus::Empty:
      return "EMPTY";
    case ActuationCoverageStatus::MissingPrefix:
      return "MISSING_PREFIX";
    case ActuationCoverageStatus::Full:
      return "FULL";
  }
  return "UNKNOWN";
}

BodyTorqueDynamicPredictor::BodyTorqueDynamicPredictor(
  const Eigen::Matrix3f & inertia_body,
  double max_history_sec)
: max_history_sec_(std::max(0.05, max_history_sec))
{
  SetInertia(inertia_body);
}

void BodyTorqueDynamicPredictor::SetInertia(const Eigen::Matrix3f & inertia_body)
{
  inertia_body_ = inertia_body;
  inertia_body_inverse_ = inertia_body_.inverse();
}

void BodyTorqueDynamicPredictor::AddTorque(
  double stamp_sec,
  const Eigen::Vector3f & torque_body)
{
  if (!std::isfinite(stamp_sec) || !torque_body.allFinite()) {
    return;
  }
  if (!torques_.empty() && stamp_sec + 1e-9 < torques_.back().stamp_sec) {
    return;
  }
  if (!torques_.empty() && std::abs(stamp_sec - torques_.back().stamp_sec) <= 1e-9) {
    torques_.back().torque_body = torque_body;
  } else {
    torques_.push_back({stamp_sec, torque_body});
  }
  const double cutoff_stamp_sec = stamp_sec - max_history_sec_;
  while (torques_.size() > 1U && torques_[1].stamp_sec <= cutoff_stamp_sec) {
    torques_.pop_front();
  }
}

ActuationCoverage BodyTorqueDynamicPredictor::CoverInterval(
  double start_stamp_sec,
  double end_stamp_sec) const
{
  ActuationCoverage result;
  result.interval_start_sec = start_stamp_sec;
  result.interval_end_sec = end_stamp_sec;
  if (torques_.empty()) {
    return result;
  }
  result.oldest_stamp_sec = torques_.front().stamp_sec;
  result.newest_stamp_sec = torques_.back().stamp_sec;
  if (!std::isfinite(start_stamp_sec) || !std::isfinite(end_stamp_sec) ||
    end_stamp_sec < start_stamp_sec || result.oldest_stamp_sec > start_stamp_sec + 1e-9)
  {
    result.status = ActuationCoverageStatus::MissingPrefix;
    result.missing_prefix_sec = std::max(
      0.0, std::min(end_stamp_sec, result.oldest_stamp_sec) - start_stamp_sec);
    return result;
  }
  result.status = ActuationCoverageStatus::Full;
  return result;
}

DynamicAngularPrediction BodyTorqueDynamicPredictor::Predict(
  const Sophus::SO3f & orientation_world_body,
  const Eigen::Vector3f & angular_velocity_world,
  double state_stamp_sec,
  double target_stamp_sec) const
{
  DynamicAngularPrediction result;
  result.orientation = orientation_world_body;
  result.torque_coverage = CoverInterval(state_stamp_sec, target_stamp_sec);
  if (!std::isfinite(state_stamp_sec) || !std::isfinite(target_stamp_sec) ||
    target_stamp_sec < state_stamp_sec ||
    result.torque_coverage.status != ActuationCoverageStatus::Full)
  {
    result.missing_torque_interval = true;
    return result;
  }

  auto active = torques_.end();
  for (auto it = torques_.begin(); it != torques_.end() && it->stamp_sec <= state_stamp_sec; ++it) {
    active = it;
  }
  if (active == torques_.end()) {
    result.missing_torque_interval = true;
    return result;
  }

  Sophus::SO3f orientation = orientation_world_body;
  Eigen::Vector3f omega_body = orientation.inverse() * angular_velocity_world;
  double cursor = state_stamp_sec;
  auto next = std::next(active);
  while (cursor < target_stamp_sec - 1e-9) {
    const double boundary = next != torques_.end() ?
      std::min(target_stamp_sec, next->stamp_sec) : target_stamp_sec;
    const double dt = boundary - cursor;
    if (dt > 0.0) {
      const Eigen::Vector3f angular_acceleration = inertia_body_inverse_ *
        (active->torque_body - omega_body.cross(inertia_body_ * omega_body));
      omega_body += angular_acceleration * static_cast<float>(dt);
      orientation = orientation * Sophus::SO3f::exp(omega_body * static_cast<float>(dt));
      ++result.integration_steps;
      ++result.torque_samples_used;
      cursor = boundary;
    }
    if (next != torques_.end() && next->stamp_sec <= cursor + 1e-9) {
      active = next;
      ++next;
    } else if (dt <= 0.0) {
      result.missing_torque_interval = true;
      return result;
    }
  }

  result.valid = true;
  result.horizon_sec = target_stamp_sec - state_stamp_sec;
  result.orientation = orientation;
  result.angular_velocity_body = omega_body;
  result.angular_velocity_world = orientation * omega_body;
  return result;
}

void BodyTorqueDynamicPredictor::Reset()
{
  torques_.clear();
}

std::size_t BodyTorqueDynamicPredictor::torque_buffer_size() const
{
  return torques_.size();
}

EpochGravityState::EpochGravityState(float gravity_mps2)
: gravity_mps2_(std::abs(gravity_mps2))
{
}

void EpochGravityState::ObserveEpoch(uint64_t map_epoch)
{
  if (!epoch_observed_ || map_epoch != map_epoch_) {
    epoch_observed_ = true;
    map_epoch_ = map_epoch;
    valid_ = false;
    gravity_o_.setZero();
  }
}

bool EpochGravityState::Initialize(uint64_t map_epoch, const Sophus::SO3f & o_r_world)
{
  ObserveEpoch(map_epoch);
  if (valid_) {
    return false;
  }
  gravity_o_ = o_r_world * Eigen::Vector3f(0.0f, 0.0f, -gravity_mps2_);
  valid_ = gravity_o_.allFinite();
  return valid_;
}

bool EpochGravityState::valid() const {return valid_;}
uint64_t EpochGravityState::map_epoch() const {return map_epoch_;}
const Eigen::Vector3f & EpochGravityState::gravity_o() const {return gravity_o_;}

BodyThrustDynamicPredictor::BodyThrustDynamicPredictor(
  float mass_kg,
  const Eigen::Vector3f & gravity_world,
  double max_history_sec)
: max_history_sec_(std::max(0.05, max_history_sec))
{
  SetMass(mass_kg);
  SetGravity(gravity_world);
}

void BodyThrustDynamicPredictor::SetMass(float mass_kg)
{
  mass_kg_ = std::max(1e-6f, mass_kg);
}

void BodyThrustDynamicPredictor::SetGravity(const Eigen::Vector3f & gravity_world)
{
  gravity_world_ = gravity_world;
}

void BodyThrustDynamicPredictor::AddThrust(double stamp_sec, float thrust_newton)
{
  if (!std::isfinite(stamp_sec) || !std::isfinite(thrust_newton)) {
    return;
  }
  if (!thrusts_.empty() && stamp_sec + 1e-9 < thrusts_.back().stamp_sec) {
    return;
  }
  if (!thrusts_.empty() && std::abs(stamp_sec - thrusts_.back().stamp_sec) <= 1e-9) {
    thrusts_.back().thrust_newton = thrust_newton;
  } else {
    thrusts_.push_back({stamp_sec, thrust_newton});
  }
  const double cutoff_stamp_sec = stamp_sec - max_history_sec_;
  while (thrusts_.size() > 1U && thrusts_[1].stamp_sec <= cutoff_stamp_sec) {
    thrusts_.pop_front();
  }
}

ActuationCoverage BodyThrustDynamicPredictor::CoverInterval(
  double start_stamp_sec,
  double end_stamp_sec) const
{
  ActuationCoverage result;
  result.interval_start_sec = start_stamp_sec;
  result.interval_end_sec = end_stamp_sec;
  if (thrusts_.empty()) {
    return result;
  }
  result.oldest_stamp_sec = thrusts_.front().stamp_sec;
  result.newest_stamp_sec = thrusts_.back().stamp_sec;
  if (!std::isfinite(start_stamp_sec) || !std::isfinite(end_stamp_sec) ||
    end_stamp_sec < start_stamp_sec || result.oldest_stamp_sec > start_stamp_sec + 1e-9)
  {
    result.status = ActuationCoverageStatus::MissingPrefix;
    result.missing_prefix_sec = std::max(
      0.0, std::min(end_stamp_sec, result.oldest_stamp_sec) - start_stamp_sec);
    return result;
  }
  result.status = ActuationCoverageStatus::Full;
  return result;
}

DynamicTranslationalPrediction BodyThrustDynamicPredictor::Predict(
  const Eigen::Vector3f & position_world,
  const Eigen::Vector3f & linear_velocity_world,
  const Sophus::SO3f & orientation_world_body,
  const Eigen::Vector3f & angular_velocity_world,
  double state_stamp_sec,
  double target_stamp_sec,
  const BodyTorqueDynamicPredictor & angular_predictor) const
{
  DynamicTranslationalPrediction result;
  result.position_world = position_world;
  result.linear_velocity_world = linear_velocity_world;
  result.thrust_coverage = CoverInterval(state_stamp_sec, target_stamp_sec);
  if (!std::isfinite(state_stamp_sec) || !std::isfinite(target_stamp_sec) ||
    target_stamp_sec < state_stamp_sec ||
    result.thrust_coverage.status != ActuationCoverageStatus::Full)
  {
    result.missing_force_interval = true;
    return result;
  }
  auto active = thrusts_.end();
  for (auto it = thrusts_.begin(); it != thrusts_.end() && it->stamp_sec <= state_stamp_sec; ++it) {
    active = it;
  }
  if (active == thrusts_.end()) {
    result.missing_force_interval = true;
    return result;
  }

  double cursor = state_stamp_sec;
  auto next = std::next(active);
  while (cursor < target_stamp_sec - 1e-9) {
    const double boundary = next != thrusts_.end() ?
      std::min(target_stamp_sec, next->stamp_sec) : target_stamp_sec;
    const double dt = boundary - cursor;
    if (dt > 0.0) {
      const double midpoint = cursor + 0.5 * dt;
      const auto angular = angular_predictor.Predict(
        orientation_world_body, angular_velocity_world, state_stamp_sec, midpoint);
      if (!angular.valid) {
        result.missing_orientation_interval = true;
        return result;
      }
      const Eigen::Vector3f force_body(0.0f, 0.0f, active->thrust_newton);
      result.thrust_newton = active->thrust_newton;
      result.thrust_acceleration_world = angular.orientation * force_body / mass_kg_;
      result.gravity_world = gravity_world_;
      result.acceleration_world = result.thrust_acceleration_world + result.gravity_world;
      const Eigen::Vector3f previous_velocity = result.linear_velocity_world;
      result.linear_velocity_world += result.acceleration_world * static_cast<float>(dt);
      result.position_world += 0.5f *
        (previous_velocity + result.linear_velocity_world) * static_cast<float>(dt);
      ++result.integration_steps;
      ++result.force_samples_used;
      cursor = boundary;
    }
    if (next != thrusts_.end() && next->stamp_sec <= cursor + 1e-9) {
      active = next;
      ++next;
    } else if (dt <= 0.0) {
      result.missing_force_interval = true;
      return result;
    }
  }
  result.valid = true;
  result.horizon_sec = target_stamp_sec - state_stamp_sec;
  return result;
}

void BodyThrustDynamicPredictor::Reset()
{
  thrusts_.clear();
}

std::size_t BodyThrustDynamicPredictor::thrust_buffer_size() const
{
  return thrusts_.size();
}

const char * LinearVelocityEstimatorModeName(LinearVelocityEstimatorMode value)
{
  switch (value) {
    case LinearVelocityEstimatorMode::Init:
      return "INIT";
    case LinearVelocityEstimatorMode::TwoSample:
      return "TWO_SAMPLE";
    case LinearVelocityEstimatorMode::ThreeSamplePredicted:
      return "THREE_SAMPLE_PREDICTED";
    case LinearVelocityEstimatorMode::DegradedDt:
      return "DEGRADED_DT";
    case LinearVelocityEstimatorMode::InvalidDt:
      return "INVALID_DT";
    case LinearVelocityEstimatorMode::Rejected:
      return "REJECTED";
  }
  return "UNKNOWN";
}

CausalLinearVelocityEstimator::CausalLinearVelocityEstimator(
  double max_good_dt_sec,
  double max_degraded_dt_sec,
  float max_speed_mps,
  float max_acceleration_mps2)
: max_good_dt_sec_(std::max(1e-6, max_good_dt_sec)),
  max_degraded_dt_sec_(std::max(max_good_dt_sec_, max_degraded_dt_sec)),
  max_speed_mps_(std::max(1e-6f, max_speed_mps)),
  max_acceleration_mps2_(std::max(1e-6f, max_acceleration_mps2))
{
}

CausalLinearVelocityEstimate CausalLinearVelocityEstimator::AddSample(
  const Eigen::Vector3f & position,
  double stamp_sec,
  uint64_t map_epoch,
  bool accepted)
{
  CausalLinearVelocityEstimate result;
  if (!accepted || !position.allFinite() || !std::isfinite(stamp_sec)) {
    result.mode = LinearVelocityEstimatorMode::Rejected;
    return result;
  }
  if (epoch_valid_ && map_epoch != map_epoch_) {
    Reset();
  }
  epoch_valid_ = true;
  map_epoch_ = map_epoch;
  if (!samples_.empty() && stamp_sec <= samples_.back().stamp_sec + 1e-9) {
    result.mode = LinearVelocityEstimatorMode::InvalidDt;
    return result;
  }
  samples_.push_back({position, stamp_sec});
  while (samples_.size() > 3U) {
    samples_.pop_front();
  }
  result.sample_accepted = true;
  result.p_k = samples_.back().position;
  if (samples_.size() == 1U) {
    result.mode = LinearVelocityEstimatorMode::Init;
    return result;
  }

  const auto & current = samples_.back();
  const auto & previous = samples_[samples_.size() - 2U];
  const double dt_current = current.stamp_sec - previous.stamp_sec;
  result.dt_current_sec = dt_current;
  result.p_k1 = previous.position;
  if (dt_current <= 1e-9 || dt_current > max_degraded_dt_sec_) {
    result.mode = LinearVelocityEstimatorMode::InvalidDt;
    return result;
  }
  result.current_mid_velocity =
    (current.position - previous.position) / static_cast<float>(dt_current);
  result.current_mid_stamp_sec = 0.5 * (current.stamp_sec + previous.stamp_sec);
  result.prediction_horizon_sec = current.stamp_sec - result.current_mid_stamp_sec;
  Eigen::Vector3f velocity_at_sample = result.current_mid_velocity;
  result.mode = dt_current <= max_good_dt_sec_ ?
    LinearVelocityEstimatorMode::TwoSample : LinearVelocityEstimatorMode::DegradedDt;

  if (samples_.size() == 3U) {
    const auto & oldest = samples_.front();
    const double dt_previous = previous.stamp_sec - oldest.stamp_sec;
    result.dt_previous_sec = dt_previous;
    result.p_k2 = oldest.position;
    if (dt_previous > 1e-9 && dt_previous <= max_good_dt_sec_ &&
      dt_current <= max_good_dt_sec_)
    {
      result.previous_mid_velocity =
        (previous.position - oldest.position) / static_cast<float>(dt_previous);
      result.previous_mid_stamp_sec = 0.5 * (previous.stamp_sec + oldest.stamp_sec);
      const double midpoint_dt = result.current_mid_stamp_sec - result.previous_mid_stamp_sec;
      if (midpoint_dt > 1e-9) {
        result.acceleration =
          (result.current_mid_velocity - result.previous_mid_velocity) /
          static_cast<float>(midpoint_dt);
        const float acceleration_norm = result.acceleration.norm();
        if (acceleration_norm > max_acceleration_mps2_) {
          result.acceleration *= max_acceleration_mps2_ / acceleration_norm;
        }
        velocity_at_sample +=
          result.acceleration * static_cast<float>(result.prediction_horizon_sec);
        result.mode = LinearVelocityEstimatorMode::ThreeSamplePredicted;
      }
    } else {
      result.mode = LinearVelocityEstimatorMode::DegradedDt;
    }
  }
  const float speed = velocity_at_sample.norm();
  if (speed > max_speed_mps_) {
    velocity_at_sample *= max_speed_mps_ / speed;
  }
  result.velocity_at_sample = velocity_at_sample;
  result.valid = true;
  return result;
}

void CausalLinearVelocityEstimator::Reset()
{
  samples_.clear();
  epoch_valid_ = false;
  map_epoch_ = 0;
}

MidpointDynamicVelocityEstimate PredictMidpointDynamicVelocity(
  const CausalLinearVelocityEstimate & linear_estimate,
  const Sophus::SO3f & previous_orientation,
  const Sophus::SO3f & current_orientation,
  double previous_image_stamp_sec,
  double current_image_stamp_sec,
  double current_arrival_stamp_sec,
  const BodyTorqueDynamicPredictor & angular_predictor,
  const BodyThrustDynamicPredictor & thrust_predictor)
{
  MidpointDynamicVelocityEstimate result;
  if (!linear_estimate.valid || !linear_estimate.current_mid_velocity.allFinite() ||
    !std::isfinite(previous_image_stamp_sec) || !std::isfinite(current_image_stamp_sec) ||
    !std::isfinite(current_arrival_stamp_sec))
  {
    return result;
  }
  const double image_dt = current_image_stamp_sec - previous_image_stamp_sec;
  if (image_dt <= 1e-9 ||
    std::abs(linear_estimate.current_mid_stamp_sec -
    0.5 * (previous_image_stamp_sec + current_image_stamp_sec)) > 1e-6)
  {
    return result;
  }

  result.image_mid_stamp_sec = linear_estimate.current_mid_stamp_sec;
  result.horizon_sec = current_image_stamp_sec - result.image_mid_stamp_sec;
  result.ros_sample_stamp_sec = current_arrival_stamp_sec;
  result.ros_mid_stamp_sec = current_arrival_stamp_sec - result.horizon_sec;
  result.velocity_mid = linear_estimate.current_mid_velocity;

  const Eigen::Vector3f relative_body =
    (previous_orientation.inverse() * current_orientation).log();
  result.orientation_mid = previous_orientation * Sophus::SO3f::exp(0.5f * relative_body);
  result.angular_velocity_mid =
    (current_orientation * previous_orientation.inverse()).log() /
    static_cast<float>(image_dt);

  const auto angular = angular_predictor.Predict(
    result.orientation_mid, result.angular_velocity_mid,
    result.ros_mid_stamp_sec, result.ros_sample_stamp_sec);
  result.torque_coverage = angular.torque_coverage;
  result.torque_samples_used = angular.torque_samples_used;
  const Eigen::Vector3f midpoint_position =
    0.5f * (linear_estimate.p_k1 + linear_estimate.p_k);
  const auto translational = thrust_predictor.Predict(
    midpoint_position, result.velocity_mid, result.orientation_mid,
    result.angular_velocity_mid, result.ros_mid_stamp_sec,
    result.ros_sample_stamp_sec, angular_predictor);
  result.thrust_coverage = translational.thrust_coverage;
  result.thrust_samples_used = translational.force_samples_used;
  if (!angular.valid || !translational.valid) {
    return result;
  }
  result.orientation_at_sample = angular.orientation;
  result.velocity_at_sample = translational.linear_velocity_world;
  result.valid = result.velocity_at_sample.allFinite();
  return result;
}

std::size_t CausalLinearVelocityEstimator::sample_count() const
{
  return samples_.size();
}

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

const char * AngularBaseUpdateTypeName(AngularBaseUpdateType value)
{
  switch (value) {
    case AngularBaseUpdateType::Initializing:
      return "INITIALIZING";
    case AngularBaseUpdateType::SmallAnchor:
      return "SMALL_ANCHOR";
    case AngularBaseUpdateType::ModeratePending:
      return "MODERATE_PENDING";
    case AngularBaseUpdateType::ModerateConfirmed:
      return "MODERATE_CONFIRMED_ANCHOR";
    case AngularBaseUpdateType::Rejected:
      return "REJECTED";
    case AngularBaseUpdateType::PredictOnly:
      return "PREDICT_ONLY";
  }
  return "UNKNOWN";
}

const char * RawDtQualityName(RawDtQuality value)
{
  switch (value) {
    case RawDtQuality::Invalid:
      return "INVALID_DT";
    case RawDtQuality::Good:
      return "GOOD_DT";
    case RawDtQuality::Degraded:
      return "DEGRADED_DT";
  }
  return "UNKNOWN_DT";
}

const char * RawMotionClassName(RawMotionClass value)
{
  switch (value) {
    case RawMotionClass::Initializing:
      return "INITIALIZING";
    case RawMotionClass::Plausible:
      return "PLAUSIBLE";
    case RawMotionClass::DegradedDt:
      return "DEGRADED_DT";
    case RawMotionClass::Suspicious:
      return "SUSPICIOUS";
    case RawMotionClass::Rejected:
      return "REJECTED";
  }
  return "UNKNOWN";
}

const char * AngularMotionEstimatorModeName(AngularMotionEstimatorMode value)
{
  switch (value) {
    case AngularMotionEstimatorMode::Init:
      return "INIT";
    case AngularMotionEstimatorMode::TwoSample:
      return "TWO_SAMPLE";
    case AngularMotionEstimatorMode::ThreeSamplePredicted:
      return "THREE_SAMPLE_PREDICTED";
    case AngularMotionEstimatorMode::DegradedDt:
      return "DEGRADED_DT";
    case AngularMotionEstimatorMode::Rejected:
      return "REJECTED";
  }
  return "UNKNOWN";
}

const char * BiasCorrectionStateName(BiasCorrectionState value)
{
  switch (value) {
    case BiasCorrectionState::Off:
      return "OFF";
    case BiasCorrectionState::Pending:
      return "PENDING";
    case BiasCorrectionState::Active:
      return "ACTIVE";
    case BiasCorrectionState::Decay:
      return "DECAY";
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
  last_diagnostics_.measurement_orientation = measurement.unit_quaternion();
  last_diagnostics_.context = context;
  last_diagnostics_.raw_history_valid_before = raw_measurement_valid_;
  last_diagnostics_.previous_raw_measurement_stamp_sec = last_raw_stamp_sec_;
  last_diagnostics_.post_reference_switch =
    context.reference_changed ||
    context.frames_since_reference_change <= config_.post_reference_switch_frames;
  if (angular_history_epoch_valid_ && context.map_epoch != angular_history_map_epoch_) {
    raw_measurement_valid_ = false;
    raw_angular_velocity_valid_ = false;
    previous_raw_angular_velocity_.setZero();
    previous_raw_interval_mid_stamp_sec_ = 0.0;
    previous_raw_interval_dt_quality_ = RawDtQuality::Invalid;
    omega_motion_.setZero();
    causal_angular_acceleration_.setZero();
  }
  angular_history_epoch_valid_ = true;
  angular_history_map_epoch_ = context.map_epoch;
  if (!valid_) {
    pose_ = measurement;
    stamp_sec_ = stamp_sec;
    linear_velocity_.setZero();
    angular_velocity_.setZero();
    omega_motion_.setZero();
    omega_bias_.setZero();
    causal_angular_acceleration_.setZero();
    bias_state_ = BiasCorrectionState::Off;
    motion_bias_suppressed_ = false;
    ClearBiasPending();
    velocity_valid_ = false;
    valid_ = true;
    raw_measurement_valid_ = true;
    last_raw_measurement_ = measurement;
    last_raw_stamp_sec_ = stamp_sec;
    last_diagnostics_.raw_history_advanced = true;
    previous_raw_interval_mid_stamp_sec_ = 0.0;
    previous_raw_interval_dt_quality_ = RawDtQuality::Invalid;
    last_diagnostics_.classification = AngularCorrectionClass::Initializing;
    last_diagnostics_.predicted_orientation_before_measurement =
      measurement.unit_quaternion();
    last_diagnostics_.base_orientation_after_measurement =
      measurement.unit_quaternion();
    last_diagnostics_.base_stamp_sec = stamp_sec;
    last_diagnostics_.base_update_type = AngularBaseUpdateType::Initializing;
    last_diagnostics_.base_update_applied = true;
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
    const double raw_dt = stamp_sec - last_raw_stamp_sec_;
    last_diagnostics_.raw_dt_sec = raw_dt;
    if (raw_dt > 1e-4 && raw_dt <= config_.raw_dt_max_good_sec) {
      last_diagnostics_.raw_dt_quality = RawDtQuality::Good;
    } else if (raw_dt > 1e-4 && raw_dt <= config_.raw_dt_max_degraded_sec) {
      last_diagnostics_.raw_dt_quality = RawDtQuality::Degraded;
    }
    last_diagnostics_.raw_rotation_step =
      (measurement.so3() * last_raw_measurement_.so3().inverse()).log();
    last_diagnostics_.raw_step_rotation_rad =
      last_diagnostics_.raw_rotation_step.norm();
    last_diagnostics_.raw_step_translation_m =
      (measurement.translation() - last_raw_measurement_.translation()).norm();
    if (raw_dt > 1e-4) {
      last_diagnostics_.implied_angular_velocity =
        last_diagnostics_.raw_rotation_step / static_cast<float>(raw_dt);
      last_diagnostics_.previous_raw_angular_velocity = previous_raw_angular_velocity_;
      last_diagnostics_.current_interval_mid_stamp_sec =
        0.5 * (last_raw_stamp_sec_ + stamp_sec);
      if (raw_angular_velocity_valid_) {
        last_diagnostics_.previous_interval_mid_stamp_sec =
          previous_raw_interval_mid_stamp_sec_;
        const double midpoint_dt =
          last_diagnostics_.current_interval_mid_stamp_sec -
          previous_raw_interval_mid_stamp_sec_;
        if (midpoint_dt > 1e-4) {
        last_diagnostics_.implied_angular_acceleration =
          (last_diagnostics_.implied_angular_velocity -
          previous_raw_angular_velocity_) / static_cast<float>(midpoint_dt);
        }
      }
    }
  }

  const float raw_step = last_diagnostics_.raw_step_rotation_rad;
  const float raw_speed = last_diagnostics_.implied_angular_velocity.norm();
  const float raw_acceleration = last_diagnostics_.implied_angular_acceleration.norm();
  const bool raw_step_ok = config_.max_raw_rotation_step_rad <= 0.0f ||
    raw_step <= config_.max_raw_rotation_step_rad;
  const bool raw_speed_ok = config_.max_raw_angular_speed_radps <= 0.0f ||
    raw_speed <= config_.max_raw_angular_speed_radps;
  const bool raw_acceleration_ok = !raw_angular_velocity_valid_ ||
    config_.max_raw_angular_acceleration_radps2 <= 0.0f ||
    raw_acceleration <= config_.max_raw_angular_acceleration_radps2;
  const bool raw_dt_usable =
    last_diagnostics_.raw_dt_quality == RawDtQuality::Good ||
    last_diagnostics_.raw_dt_quality == RawDtQuality::Degraded;
  const bool raw_motion_plausible =
    raw_dt_usable && raw_step_ok && raw_speed_ok && raw_acceleration_ok;
  if (raw_motion_plausible) {
    last_diagnostics_.raw_motion_class =
      last_diagnostics_.raw_dt_quality == RawDtQuality::Good ?
      RawMotionClass::Plausible : RawMotionClass::DegradedDt;
    Eigen::Vector3f omega_hat = last_diagnostics_.implied_angular_velocity;
    const bool three_sample_usable = raw_angular_velocity_valid_ &&
      last_diagnostics_.raw_dt_quality == RawDtQuality::Good &&
      previous_raw_interval_dt_quality_ == RawDtQuality::Good;
    if (three_sample_usable) {
      const float prediction_horizon = static_cast<float>(
        stamp_sec - last_diagnostics_.current_interval_mid_stamp_sec);
      last_diagnostics_.causal_angular_acceleration =
        last_diagnostics_.implied_angular_acceleration;
      causal_angular_acceleration_ = last_diagnostics_.causal_angular_acceleration;
      omega_hat += last_diagnostics_.causal_angular_acceleration * prediction_horizon;
      last_diagnostics_.omega_prediction_horizon_sec = prediction_horizon;
      last_diagnostics_.omega_estimator_mode =
        AngularMotionEstimatorMode::ThreeSamplePredicted;
    } else {
      causal_angular_acceleration_.setZero();
      last_diagnostics_.omega_estimator_mode =
        last_diagnostics_.raw_dt_quality == RawDtQuality::Degraded ?
        AngularMotionEstimatorMode::DegradedDt : AngularMotionEstimatorMode::TwoSample;
    }
    const float reversal_noise_step = std::max(0.0f, config_.raw_reversal_noise_step_rad);
    const float current_speed_limit = last_diagnostics_.raw_dt_sec > 1e-4 ?
      reversal_noise_step / static_cast<float>(last_diagnostics_.raw_dt_sec) : 0.0f;
    const bool microscopic_reversal = raw_angular_velocity_valid_ &&
      previous_raw_angular_velocity_.dot(last_diagnostics_.implied_angular_velocity) < 0.0f &&
      raw_step <= reversal_noise_step &&
      previous_raw_angular_velocity_.norm() <= current_speed_limit;
    if (microscopic_reversal || config_.raw_motion_filter_alpha <= 0.0f) {
      omega_hat.setZero();
      causal_angular_acceleration_.setZero();
    }
    bool omega_limited = false;
    omega_hat = ClampNorm(omega_hat, config_.max_raw_angular_speed_radps, omega_limited);
    last_update_limited_ = last_update_limited_ || omega_limited;
    omega_motion_ = omega_hat;
    last_diagnostics_.omega_hat_at_measurement = omega_hat;
    last_diagnostics_.omega_motion_target = omega_hat;
    previous_raw_angular_velocity_ = last_diagnostics_.implied_angular_velocity;
    previous_raw_interval_mid_stamp_sec_ =
      last_diagnostics_.current_interval_mid_stamp_sec;
    previous_raw_interval_dt_quality_ = last_diagnostics_.raw_dt_quality;
    raw_angular_velocity_valid_ = true;
  } else {
    causal_angular_acceleration_.setZero();
    last_diagnostics_.raw_motion_class = raw_dt_usable ?
      RawMotionClass::Rejected : RawMotionClass::Suspicious;
    last_diagnostics_.raw_rejected = true;
    last_diagnostics_.omega_estimator_mode = AngularMotionEstimatorMode::Rejected;
    last_diagnostics_.omega_motion_target.setZero();
    const float previous_motion_norm = omega_motion_.norm();
    if (previous_motion_norm > 1e-6f &&
      config_.rejected_motion_decay_acceleration_radps2 > 0.0f)
    {
      const float decay_dt = static_cast<float>(std::max(0.0, std::min(dt, 0.5)));
      const float decay_amount = std::min(
        previous_motion_norm,
        config_.rejected_motion_decay_acceleration_radps2 * decay_dt);
      omega_motion_ *= (previous_motion_norm - decay_amount) / previous_motion_norm;
      last_diagnostics_.motion_decay_active = true;
      if (decay_dt > 1e-6f) {
        last_diagnostics_.motion_decay_rate_radps2 = decay_amount / decay_dt;
      }
    }
  }

  const float motion_indicator = std::max(raw_speed, omega_motion_.norm());
  const float suppression_enter = std::max(
    0.0f, config_.motion_bias_suppression_enter_radps);
  const float suppression_exit = std::max(
    0.0f, std::min(
      config_.motion_bias_suppression_exit_radps, suppression_enter));
  if (suppression_enter <= 0.0f) {
    motion_bias_suppressed_ = false;
  } else if (motion_bias_suppressed_) {
    if (motion_indicator <= suppression_exit) {
      motion_bias_suppressed_ = false;
    }
  } else if (motion_indicator >= suppression_enter) {
    motion_bias_suppressed_ = true;
  }
  last_diagnostics_.motion_indicator_radps = motion_indicator;
  last_diagnostics_.motion_bias_suppressed = motion_bias_suppressed_;

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
  const Sophus::SO3f angular_prediction =
    Sophus::SO3f::exp(
    (omega_motion_ + omega_bias_) * static_cast<float>(dt)) * pose_.so3();
  const Eigen::Vector3f rotation_innovation =
    (measurement.so3() * angular_prediction.inverse()).log();
  last_rotation_innovation_rad_ = rotation_innovation.norm();
  last_diagnostics_.rotation_innovation = rotation_innovation;
  last_diagnostics_.rotation_innovation_rad = last_rotation_innovation_rad_;

  float small_rotation_limit =
    std::max(0.0f, config_.small_rotation_innovation_rad);
  if (config_.max_rotation_innovation_rad > 0.0f) {
    small_rotation_limit = std::min(
      small_rotation_limit, config_.max_rotation_innovation_rad);
  }
  last_diagnostics_.small_rotation_limit_rad = small_rotation_limit;

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

    const bool persistent_offset =
      last_diagnostics_.raw_step_rotation_rad <= small_rotation_limit;
    const bool coherent =
      cosine + kConsistencyComparisonEpsilon >=
      config_.moderate_direction_consistency &&
      magnitude_ratio + kConsistencyComparisonEpsilon >=
      config_.moderate_magnitude_ratio &&
      (persistent_offset || raw_motion_plausible);
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

  const float bias_enter = std::max(0.0f, config_.bias_deadband_enter_rad);
  const float bias_exit = std::max(
    0.0f, std::min(config_.bias_deadband_exit_rad, bias_enter));
  last_diagnostics_.bias_deadband_enter_rad = bias_enter;
  last_diagnostics_.bias_deadband_exit_rad = bias_exit;
  const bool bias_context_matches =
    bias_pending_map_epoch_ == context.map_epoch &&
    bias_pending_reference_keyframe_id_ == context.reference_keyframe_id;
  const bool bias_forced_to_decay =
    excessive || motion_bias_suppressed_ || last_rotation_innovation_rad_ <= bias_exit;

  if (bias_forced_to_decay) {
    ClearBiasPending();
    bias_state_ = omega_bias_.norm() > 1e-6f ?
      BiasCorrectionState::Decay : BiasCorrectionState::Off;
  } else if (bias_state_ == BiasCorrectionState::Active) {
    const float active_norm = bias_pending_innovation_.norm();
    const float current_norm = rotation_innovation.norm();
    const float cosine =
      active_norm > 1e-6f && current_norm > 1e-6f ?
      bias_pending_innovation_.dot(rotation_innovation) /
      (active_norm * current_norm) : 1.0f;
    if (!bias_context_matches ||
      cosine + kConsistencyComparisonEpsilon < config_.moderate_direction_consistency)
    {
      StartBiasPending(
        rotation_innovation, stamp_sec, context,
        last_diagnostics_.post_reference_switch);
      bias_state_ = BiasCorrectionState::Pending;
    } else {
      bias_pending_innovation_ =
        0.5f * bias_pending_innovation_ + 0.5f * rotation_innovation;
    }
  } else if (!bias_pending_valid_) {
    if (last_rotation_innovation_rad_ >= bias_enter) {
      StartBiasPending(
        rotation_innovation, stamp_sec, context,
        last_diagnostics_.post_reference_switch);
      bias_state_ = BiasCorrectionState::Pending;
    } else {
      bias_state_ = omega_bias_.norm() > 1e-6f ?
        BiasCorrectionState::Decay : BiasCorrectionState::Off;
    }
  } else if (!bias_context_matches) {
    StartBiasPending(
      rotation_innovation, stamp_sec, context,
      last_diagnostics_.post_reference_switch);
    bias_state_ = BiasCorrectionState::Pending;
  } else {
    ++bias_pending_total_frames_;
    const float pending_norm = bias_pending_innovation_.norm();
    const float current_norm = rotation_innovation.norm();
    const float cosine =
      pending_norm > 1e-6f && current_norm > 1e-6f ?
      bias_pending_innovation_.dot(rotation_innovation) /
      (pending_norm * current_norm) : 1.0f;
    const float magnitude_ratio =
      std::max(pending_norm, current_norm) > 1e-6f ?
      std::min(pending_norm, current_norm) /
      std::max(pending_norm, current_norm) : 1.0f;
    const bool coherent =
      cosine + kConsistencyComparisonEpsilon >=
      config_.moderate_direction_consistency &&
      magnitude_ratio + kConsistencyComparisonEpsilon >=
      config_.moderate_magnitude_ratio;
    const bool timed_out = config_.moderate_timeout_sec > 0.0 &&
      stamp_sec - bias_pending_started_stamp_sec_ > config_.moderate_timeout_sec;
    const bool exhausted = config_.moderate_max_pending_frames > 0 &&
      bias_pending_total_frames_ > config_.moderate_max_pending_frames;
    if (!coherent || timed_out || exhausted) {
      ClearBiasPending();
      if (!timed_out && !exhausted &&
        last_rotation_innovation_rad_ >= bias_enter)
      {
        StartBiasPending(
          rotation_innovation, stamp_sec, context,
          last_diagnostics_.post_reference_switch);
        bias_state_ = BiasCorrectionState::Pending;
      } else {
        bias_state_ = omega_bias_.norm() > 1e-6f ?
          BiasCorrectionState::Decay : BiasCorrectionState::Off;
      }
    } else {
      ++bias_pending_good_frames_;
      bias_pending_innovation_ =
        0.5f * bias_pending_innovation_ + 0.5f * rotation_innovation;
      bias_pending_post_reference_switch_ =
        bias_pending_post_reference_switch_ ||
        last_diagnostics_.post_reference_switch;
      const uint32_t required_frames = std::max(
        1U, bias_pending_post_reference_switch_ ?
        config_.bias_post_reference_confirmation_frames :
        config_.bias_confirmation_frames);
      if (bias_pending_good_frames_ >= required_frames) {
        bias_state_ = BiasCorrectionState::Active;
      } else {
        bias_state_ = BiasCorrectionState::Pending;
      }
    }
  }

  Eigen::Vector3f desired_bias_velocity = Eigen::Vector3f::Zero();
  if (bias_state_ == BiasCorrectionState::Active) {
    desired_bias_velocity = ClampNorm(
      rotation_innovation / static_cast<float>(dt),
      config_.max_orientation_bias_correction_rate_radps,
      last_update_limited_);
  }
  last_diagnostics_.omega_bias_target = desired_bias_velocity;
  if (config_.max_orientation_bias_correction_acceleration_radps2 > 0.0f) {
    Eigen::Vector3f bias_delta = desired_bias_velocity - omega_bias_;
    bias_delta = ClampNorm(
      bias_delta,
      config_.max_orientation_bias_correction_acceleration_radps2 *
      static_cast<float>(dt),
      last_update_limited_);
    desired_bias_velocity = omega_bias_ + bias_delta;
  }
  omega_bias_ = desired_bias_velocity;
  if (bias_state_ == BiasCorrectionState::Decay && omega_bias_.norm() <= 1e-6f) {
    bias_state_ = BiasCorrectionState::Off;
  }
  last_diagnostics_.bias_state = bias_state_;
  last_diagnostics_.bias_pending_frames = bias_pending_good_frames_;
  last_diagnostics_.bias_active = bias_state_ == BiasCorrectionState::Active;
  last_diagnostics_.applied_rotation_correction_rad =
    omega_bias_.norm() * static_cast<float>(dt);
  last_diagnostics_.bias_correction_step_rad =
    last_diagnostics_.applied_rotation_correction_rad;
  if (last_rotation_innovation_rad_ > 1e-6f) {
    last_diagnostics_.correction_fraction_applied =
      last_diagnostics_.applied_rotation_correction_rad /
      last_rotation_innovation_rad_;
  }
  Eigen::Vector3f desired_angular_velocity = omega_motion_ + omega_bias_;
  last_diagnostics_.omega_total_before_limits = desired_angular_velocity;
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
  last_diagnostics_.omega_motion = omega_motion_;
  last_diagnostics_.omega_bias = omega_bias_;
  last_diagnostics_.omega_total_after_limits = angular_velocity_;
  const Sophus::SE3f previous_pose = pose_;
  const Sophus::SO3f predicted_base_orientation =
    Sophus::SO3f::exp(angular_velocity_ * static_cast<float>(dt)) * pose_.so3();
  last_diagnostics_.predicted_orientation_before_measurement =
    predicted_base_orientation.unit_quaternion();
  last_diagnostics_.visual_base_error_before_rad =
    (measurement.so3() * predicted_base_orientation.inverse()).log().norm();

  Sophus::SO3f base_orientation = predicted_base_orientation;
  if (last_diagnostics_.classification == AngularCorrectionClass::Small &&
    raw_motion_plausible)
  {
    base_orientation = measurement.so3();
    last_diagnostics_.base_update_type = AngularBaseUpdateType::SmallAnchor;
    last_diagnostics_.base_update_applied = true;
  } else if (
    last_diagnostics_.classification == AngularCorrectionClass::ModerateConfirmed &&
    raw_motion_plausible)
  {
    base_orientation = measurement.so3();
    last_diagnostics_.base_update_type = AngularBaseUpdateType::ModerateConfirmed;
    last_diagnostics_.base_update_applied = true;
    last_diagnostics_.base_rotation_correction_rad =
      last_diagnostics_.visual_base_error_before_rad;
  } else if (
    last_diagnostics_.classification == AngularCorrectionClass::ModeratePending)
  {
    last_diagnostics_.base_update_type = AngularBaseUpdateType::ModeratePending;
  } else if (
    last_diagnostics_.classification == AngularCorrectionClass::RejectedExcessive)
  {
    last_diagnostics_.base_update_type = AngularBaseUpdateType::Rejected;
  } else {
    last_diagnostics_.base_update_type = AngularBaseUpdateType::PredictOnly;
  }

  if (last_diagnostics_.base_update_type == AngularBaseUpdateType::SmallAnchor) {
    last_diagnostics_.base_rotation_correction_rad =
      last_diagnostics_.visual_base_error_before_rad;
  }
  last_diagnostics_.base_orientation_after_measurement =
    base_orientation.unit_quaternion();
  last_diagnostics_.base_stamp_sec = stamp_sec;
  last_diagnostics_.visual_base_error_after_rad =
    (measurement.so3() * base_orientation.inverse()).log().norm();
  pose_ = Sophus::SE3f(
    base_orientation,
    pose_.translation() + linear_velocity_ * static_cast<float>(dt));
  velocity_valid_ = true;
  stamp_sec_ = stamp_sec;
  if (raw_motion_plausible || !raw_measurement_valid_) {
    last_raw_measurement_ = measurement;
    last_raw_stamp_sec_ = stamp_sec;
    raw_measurement_valid_ = true;
    last_diagnostics_.raw_history_advanced = true;
  } else if (
    last_diagnostics_.raw_dt_quality == RawDtQuality::Invalid &&
    last_diagnostics_.raw_dt_sec > config_.raw_dt_max_degraded_sec)
  {
    // The invalid delta remains rejected; rebase only prevents a stale raw
    // baseline from making every later measurement incomparable.
    last_raw_measurement_ = measurement;
    last_raw_stamp_sec_ = stamp_sec;
    raw_measurement_valid_ = true;
    raw_angular_velocity_valid_ = false;
    previous_raw_angular_velocity_.setZero();
    previous_raw_interval_mid_stamp_sec_ = 0.0;
    previous_raw_interval_dt_quality_ = RawDtQuality::Invalid;
    last_diagnostics_.raw_history_advanced = true;
    last_diagnostics_.raw_history_rebased = true;
  }
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
  const double requested_dt = std::max(0.0, stamp_sec - stamp_sec_);
  const double dt = std::min(requested_dt, config_.max_extrapolation_sec);
  result.valid = true;
  result.velocity_valid = velocity_valid_;
  result.prediction_horizon_sec = dt;
  result.prediction_clamped = requested_dt > config_.max_extrapolation_sec;
  result.pose = Propagate(dt);
  result.linear_velocity = linear_velocity_;
  result.angular_velocity = angular_velocity_;
  if (config_.predict_angular_acceleration && velocity_valid_ && dt > 0.0) {
    bool acceleration_limited = false;
    const Eigen::Vector3f acceleration = ClampNorm(
      causal_angular_acceleration_, config_.max_raw_angular_acceleration_radps2,
      acceleration_limited);
    bool speed_limited = false;
    result.angular_velocity = ClampNorm(
      angular_velocity_ + acceleration * static_cast<float>(dt),
      config_.max_angular_speed_radps, speed_limited);
    result.angular_acceleration = acceleration;
    result.angular_acceleration_clamped = acceleration_limited;
    result.angular_velocity_clamped = speed_limited;
    result.angular_prediction_delta =
      angular_velocity_ * static_cast<float>(dt) +
      0.5f * acceleration * static_cast<float>(dt * dt);
    result.pose = Sophus::SE3f(
      Sophus::SO3f::exp(result.angular_prediction_delta) * pose_.so3(),
      pose_.translation() + linear_velocity_ * static_cast<float>(dt));
  }
  return result;
}

void OrbPosePredictor::OverrideAngularVelocityForDiagnostics(
  const Eigen::Vector3f & angular_velocity)
{
  omega_motion_ = angular_velocity;
  omega_bias_.setZero();
  angular_velocity_ = angular_velocity;
  causal_angular_acceleration_.setZero();
  velocity_valid_ = valid_;
}

OrbPredictionTiming ComputeOrbPredictionTiming(
  double measurement_stamp_sec,
  double measurement_arrival_local_sec,
  double target_local_sec)
{
  OrbPredictionTiming timing;
  timing.visual_age_sec = std::max(
    0.0, target_local_sec - measurement_arrival_local_sec);
  timing.target_measurement_stamp_sec =
    measurement_stamp_sec + timing.visual_age_sec;
  return timing;
}

void OrbPosePredictor::Reset()
{
  valid_ = false;
  velocity_valid_ = false;
  linear_velocity_.setZero();
  angular_velocity_.setZero();
  omega_motion_.setZero();
  omega_bias_.setZero();
  causal_angular_acceleration_.setZero();
  bias_state_ = BiasCorrectionState::Off;
  motion_bias_suppressed_ = false;
  consecutive_angular_rejections_ = 0;
  last_orientation_rejected_ = false;
  raw_measurement_valid_ = false;
  raw_angular_velocity_valid_ = false;
  previous_raw_angular_velocity_.setZero();
  previous_raw_interval_mid_stamp_sec_ = 0.0;
  previous_raw_interval_dt_quality_ = RawDtQuality::Invalid;
  angular_history_epoch_valid_ = false;
  angular_history_map_epoch_ = 0;
  next_pending_correction_id_ = 1;
  ClearModerateState();
  ClearBiasPending();
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

void OrbPosePredictor::StartBiasPending(
  const Eigen::Vector3f & innovation,
  double stamp_sec,
  const OrbMeasurementContext & context,
  bool post_reference_switch)
{
  bias_pending_valid_ = true;
  bias_pending_started_stamp_sec_ = stamp_sec;
  bias_pending_innovation_ = innovation;
  bias_pending_good_frames_ = 1;
  bias_pending_total_frames_ = 1;
  bias_pending_map_epoch_ = context.map_epoch;
  bias_pending_reference_keyframe_id_ = context.reference_keyframe_id;
  bias_pending_post_reference_switch_ = post_reference_switch;
}

void OrbPosePredictor::ClearBiasPending()
{
  bias_pending_valid_ = false;
  bias_pending_started_stamp_sec_ = 0.0;
  bias_pending_innovation_.setZero();
  bias_pending_good_frames_ = 0;
  bias_pending_total_frames_ = 0;
  bias_pending_map_epoch_ = 0;
  bias_pending_reference_keyframe_id_ = 0;
  bias_pending_post_reference_switch_ = false;
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
