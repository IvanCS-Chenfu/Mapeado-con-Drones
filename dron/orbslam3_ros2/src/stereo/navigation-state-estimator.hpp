#ifndef ORBSLAM3_ROS2_NAVIGATION_STATE_ESTIMATOR_HPP_
#define ORBSLAM3_ROS2_NAVIGATION_STATE_ESTIMATOR_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>

#include <sophus/se3.hpp>

namespace orbslam3_ros2
{

enum class GlobalPoseState : uint8_t
{
  Invalid = 0,
  Provisional = 1,
  Authoritative = 2,
};

enum class AngularCorrectionClass : uint8_t
{
  Initializing = 0,
  Small = 1,
  ModeratePending = 2,
  ModerateConfirmed = 3,
  ModerateDiscarded = 4,
  RejectedExcessive = 5,
};

const char * AngularCorrectionClassName(AngularCorrectionClass value);

enum class AngularBaseUpdateType : uint8_t
{
  Initializing = 0,
  SmallAnchor = 1,
  ModeratePending = 2,
  ModerateConfirmed = 3,
  Rejected = 4,
  PredictOnly = 5,
};

const char * AngularBaseUpdateTypeName(AngularBaseUpdateType value);

enum class RawDtQuality : uint8_t
{
  Invalid = 0,
  Good = 1,
  Degraded = 2,
};

const char * RawDtQualityName(RawDtQuality value);

enum class RawMotionClass : uint8_t
{
  Initializing = 0,
  Plausible = 1,
  DegradedDt = 2,
  Suspicious = 3,
  Rejected = 4,
};

const char * RawMotionClassName(RawMotionClass value);

enum class AngularMotionEstimatorMode : uint8_t
{
  Init = 0,
  TwoSample = 1,
  ThreeSamplePredicted = 2,
  DegradedDt = 3,
  Rejected = 4,
};

const char * AngularMotionEstimatorModeName(AngularMotionEstimatorMode value);

enum class BiasCorrectionState : uint8_t
{
  Off = 0,
  Pending = 1,
  Active = 2,
  Decay = 3,
};

const char * BiasCorrectionStateName(BiasCorrectionState value);

struct ContinuousPoseResult
{
  bool local_valid = false;
  bool continuity_valid = false;
  bool initialized = false;
  bool epoch_changed = false;
  bool reference_changed = false;
  bool measurement_accepted = false;
  bool reference_pending = false;
  bool reference_rejected = false;
  bool reference_gate_timed_out = false;
  bool active_reference_valid = false;
  uint64_t active_reference_keyframe_id = 0;
  Sophus::SE3f o_t_camera;
  Sophus::SE3f w_t_camera;
  Sophus::SE3f active_tcr;
  GlobalPoseState global_state = GlobalPoseState::Invalid;
  uint64_t pose_revision = 0;
  float step_translation_m = 0.0f;
  float step_rotation_rad = 0.0f;
};

struct OrbPosePredictorConfig
{
  float position_alpha = 0.55f;
  float orientation_alpha = 0.70f;
  float max_position_innovation_m = 0.30f;
  float max_rotation_innovation_rad = 0.35f;
  float max_linear_speed_mps = 1.5f;
  float max_angular_speed_radps = 1.5f;
  float max_linear_acceleration_mps2 = 4.0f;
  float max_angular_acceleration_radps2 = 12.0f;
  uint32_t max_consecutive_angular_rejections = 3;
  double max_extrapolation_sec = 0.10;
  // Laboratorio F5H: predecir pose y omega al mismo target con alpha causal.
  bool predict_angular_acceleration = false;
  float small_rotation_innovation_rad = 0.015f;
  uint32_t moderate_confirmation_frames = 3;
  uint32_t moderate_post_reference_confirmation_frames = 4;
  uint32_t moderate_max_pending_frames = 6;
  float moderate_direction_consistency = 0.85f;
  float moderate_magnitude_ratio = 0.50f;
  double moderate_timeout_sec = 0.35;
  uint32_t post_reference_switch_frames = 5;
  double raw_dt_max_good_sec = 0.075;
  double raw_dt_max_degraded_sec = 0.20;
  float max_raw_rotation_step_rad = 0.12f;
  float max_raw_angular_speed_radps = 1.0f;
  float max_raw_angular_acceleration_radps2 = 10.0f;
  float raw_reversal_noise_step_rad = 0.005f;
  // Legacy/test kill switch. Positive values no longer control a low-pass filter.
  float raw_motion_filter_alpha = 0.35f;
  float max_orientation_bias_correction_rate_radps = 0.08f;
  float max_orientation_bias_correction_acceleration_radps2 = 0.8f;
  float bias_deadband_enter_rad = 0.005f;
  float bias_deadband_exit_rad = 0.002f;
  uint32_t bias_confirmation_frames = 3;
  uint32_t bias_post_reference_confirmation_frames = 4;
  float motion_bias_suppression_enter_radps = 0.10f;
  float motion_bias_suppression_exit_radps = 0.05f;
  float rejected_motion_decay_acceleration_radps2 = 4.0f;
};

struct TimedBodyTorque
{
  double stamp_sec = 0.0;
  Eigen::Vector3f torque_body = Eigen::Vector3f::Zero();
};

enum class ActuationCoverageStatus : uint8_t
{
  Empty = 0,
  MissingPrefix = 1,
  Full = 2
};

const char * ActuationCoverageStatusName(ActuationCoverageStatus value);

struct ActuationCoverage
{
  ActuationCoverageStatus status = ActuationCoverageStatus::Empty;
  double interval_start_sec = 0.0;
  double interval_end_sec = 0.0;
  double oldest_stamp_sec = 0.0;
  double newest_stamp_sec = 0.0;
  double missing_prefix_sec = 0.0;
};

struct DynamicAngularPrediction
{
  bool valid = false;
  bool missing_torque_interval = false;
  ActuationCoverage torque_coverage;
  double horizon_sec = 0.0;
  uint32_t integration_steps = 0;
  uint32_t torque_samples_used = 0;
  Sophus::SO3f orientation;
  Eigen::Vector3f angular_velocity_body = Eigen::Vector3f::Zero();
  Eigen::Vector3f angular_velocity_world = Eigen::Vector3f::Zero();
};

class BodyTorqueDynamicPredictor
{
public:
  explicit BodyTorqueDynamicPredictor(
    const Eigen::Matrix3f & inertia_body = Eigen::Matrix3f::Identity(),
    double max_history_sec = 0.5);

  void SetInertia(const Eigen::Matrix3f & inertia_body);
  void AddTorque(double stamp_sec, const Eigen::Vector3f & torque_body);
  ActuationCoverage CoverInterval(double start_stamp_sec, double end_stamp_sec) const;
  DynamicAngularPrediction Predict(
    const Sophus::SO3f & orientation_world_body,
    const Eigen::Vector3f & angular_velocity_world,
    double state_stamp_sec,
    double target_stamp_sec) const;
  void Reset();
  std::size_t torque_buffer_size() const;

private:
  Eigen::Matrix3f inertia_body_ = Eigen::Matrix3f::Identity();
  Eigen::Matrix3f inertia_body_inverse_ = Eigen::Matrix3f::Identity();
  double max_history_sec_ = 0.5;
  std::deque<TimedBodyTorque> torques_;
};

struct TimedBodyThrust
{
  double stamp_sec = 0.0;
  float thrust_newton = 0.0f;
};

struct DynamicTranslationalPrediction
{
  bool valid = false;
  bool missing_force_interval = false;
  bool missing_orientation_interval = false;
  ActuationCoverage thrust_coverage;
  double horizon_sec = 0.0;
  uint32_t integration_steps = 0;
  uint32_t force_samples_used = 0;
  Eigen::Vector3f position_world = Eigen::Vector3f::Zero();
  Eigen::Vector3f linear_velocity_world = Eigen::Vector3f::Zero();
  Eigen::Vector3f acceleration_world = Eigen::Vector3f::Zero();
  Eigen::Vector3f thrust_acceleration_world = Eigen::Vector3f::Zero();
  Eigen::Vector3f gravity_world = Eigen::Vector3f::Zero();
  float thrust_newton = 0.0f;
};

class EpochGravityState
{
public:
  explicit EpochGravityState(float gravity_mps2 = 9.81f);

  void ObserveEpoch(uint64_t map_epoch);
  bool Initialize(uint64_t map_epoch, const Sophus::SO3f & o_r_world);
  bool valid() const;
  uint64_t map_epoch() const;
  const Eigen::Vector3f & gravity_o() const;

private:
  float gravity_mps2_ = 9.81f;
  bool epoch_observed_ = false;
  bool valid_ = false;
  uint64_t map_epoch_ = 0;
  Eigen::Vector3f gravity_o_ = Eigen::Vector3f::Zero();
};

class BodyThrustDynamicPredictor
{
public:
  explicit BodyThrustDynamicPredictor(
    float mass_kg = 1.0f,
    const Eigen::Vector3f & gravity_world = Eigen::Vector3f(0.0f, 0.0f, -9.81f),
    double max_history_sec = 0.5);

  void SetMass(float mass_kg);
  void SetGravity(const Eigen::Vector3f & gravity_world);
  void AddThrust(double stamp_sec, float thrust_newton);
  ActuationCoverage CoverInterval(double start_stamp_sec, double end_stamp_sec) const;
  DynamicTranslationalPrediction Predict(
    const Eigen::Vector3f & position_world,
    const Eigen::Vector3f & linear_velocity_world,
    const Sophus::SO3f & orientation_world_body,
    const Eigen::Vector3f & angular_velocity_world,
    double state_stamp_sec,
    double target_stamp_sec,
    const BodyTorqueDynamicPredictor & angular_predictor) const;
  void Reset();
  std::size_t thrust_buffer_size() const;

private:
  float mass_kg_ = 1.0f;
  Eigen::Vector3f gravity_world_{0.0f, 0.0f, -9.81f};
  double max_history_sec_ = 0.5;
  std::deque<TimedBodyThrust> thrusts_;
};

enum class LinearVelocityEstimatorMode : uint8_t
{
  Init = 0,
  TwoSample = 1,
  ThreeSamplePredicted = 2,
  DegradedDt = 3,
  InvalidDt = 4,
  Rejected = 5,
};

const char * LinearVelocityEstimatorModeName(LinearVelocityEstimatorMode value);

struct CausalLinearVelocityEstimate
{
  bool valid = false;
  bool sample_accepted = false;
  LinearVelocityEstimatorMode mode = LinearVelocityEstimatorMode::Init;
  Eigen::Vector3f velocity_at_sample = Eigen::Vector3f::Zero();
  Eigen::Vector3f acceleration = Eigen::Vector3f::Zero();
  Eigen::Vector3f previous_mid_velocity = Eigen::Vector3f::Zero();
  Eigen::Vector3f current_mid_velocity = Eigen::Vector3f::Zero();
  double dt_previous_sec = 0.0;
  double dt_current_sec = 0.0;
  double previous_mid_stamp_sec = 0.0;
  double current_mid_stamp_sec = 0.0;
  double prediction_horizon_sec = 0.0;
  Eigen::Vector3f p_k2 = Eigen::Vector3f::Zero();
  Eigen::Vector3f p_k1 = Eigen::Vector3f::Zero();
  Eigen::Vector3f p_k = Eigen::Vector3f::Zero();
};

class CausalLinearVelocityEstimator
{
public:
  explicit CausalLinearVelocityEstimator(
    double max_good_dt_sec = 0.075,
    double max_degraded_dt_sec = 0.20,
    float max_speed_mps = 1.5f,
    float max_acceleration_mps2 = 4.0f);

  CausalLinearVelocityEstimate AddSample(
    const Eigen::Vector3f & position,
    double stamp_sec,
    uint64_t map_epoch,
    bool accepted = true);
  void Reset();
  std::size_t sample_count() const;

private:
  struct PositionSample
  {
    Eigen::Vector3f position = Eigen::Vector3f::Zero();
    double stamp_sec = 0.0;
  };

  double max_good_dt_sec_ = 0.075;
  double max_degraded_dt_sec_ = 0.20;
  float max_speed_mps_ = 1.5f;
  float max_acceleration_mps2_ = 4.0f;
  bool epoch_valid_ = false;
  uint64_t map_epoch_ = 0;
  std::deque<PositionSample> samples_;
};

struct MidpointDynamicVelocityEstimate
{
  bool valid = false;
  double image_mid_stamp_sec = 0.0;
  double ros_mid_stamp_sec = 0.0;
  double ros_sample_stamp_sec = 0.0;
  double horizon_sec = 0.0;
  Sophus::SO3f orientation_mid;
  Sophus::SO3f orientation_at_sample;
  Eigen::Vector3f angular_velocity_mid = Eigen::Vector3f::Zero();
  Eigen::Vector3f velocity_mid = Eigen::Vector3f::Zero();
  Eigen::Vector3f velocity_at_sample = Eigen::Vector3f::Zero();
  ActuationCoverage torque_coverage;
  ActuationCoverage thrust_coverage;
  uint32_t torque_samples_used = 0;
  uint32_t thrust_samples_used = 0;
};

MidpointDynamicVelocityEstimate PredictMidpointDynamicVelocity(
  const CausalLinearVelocityEstimate & linear_estimate,
  const Sophus::SO3f & previous_orientation,
  const Sophus::SO3f & current_orientation,
  double previous_image_stamp_sec,
  double current_image_stamp_sec,
  double current_arrival_stamp_sec,
  const BodyTorqueDynamicPredictor & angular_predictor,
  const BodyThrustDynamicPredictor & thrust_predictor);

struct OrbMeasurementContext
{
  uint64_t map_epoch = 0;
  int32_t tracking_state = 0;
  uint64_t reference_keyframe_id = 0;
  bool reference_changed = false;
  uint32_t frames_since_reference_change = 0xFFFFFFFFU;
};

struct OrbPosePredictorDiagnostics
{
  bool measurement_processed = false;
  double measurement_stamp_sec = 0.0;
  Eigen::Quaternionf measurement_orientation = Eigen::Quaternionf::Identity();
  Eigen::Quaternionf predicted_orientation_before_measurement =
    Eigen::Quaternionf::Identity();
  Eigen::Quaternionf base_orientation_after_measurement =
    Eigen::Quaternionf::Identity();
  double base_stamp_sec = 0.0;
  AngularBaseUpdateType base_update_type = AngularBaseUpdateType::Initializing;
  bool base_update_applied = false;
  float base_rotation_correction_rad = 0.0f;
  float visual_base_error_before_rad = 0.0f;
  float visual_base_error_after_rad = 0.0f;
  double dt_sec = 0.0;
  double raw_dt_sec = 0.0;
  RawDtQuality raw_dt_quality = RawDtQuality::Invalid;
  RawMotionClass raw_motion_class = RawMotionClass::Initializing;
  OrbMeasurementContext context;
  AngularCorrectionClass classification = AngularCorrectionClass::Initializing;
  bool post_reference_switch = false;
  Eigen::Vector3f raw_rotation_step = Eigen::Vector3f::Zero();
  float raw_step_translation_m = 0.0f;
  float raw_step_rotation_rad = 0.0f;
  Eigen::Vector3f position_innovation = Eigen::Vector3f::Zero();
  Eigen::Vector3f rotation_innovation = Eigen::Vector3f::Zero();
  float position_innovation_m = 0.0f;
  float rotation_innovation_rad = 0.0f;
  float small_rotation_limit_rad = 0.0f;
  Eigen::Vector3f previous_linear_velocity = Eigen::Vector3f::Zero();
  Eigen::Vector3f previous_angular_velocity = Eigen::Vector3f::Zero();
  Eigen::Vector3f implied_angular_velocity = Eigen::Vector3f::Zero();
  Eigen::Vector3f implied_angular_acceleration = Eigen::Vector3f::Zero();
  Eigen::Vector3f previous_raw_angular_velocity = Eigen::Vector3f::Zero();
  double previous_interval_mid_stamp_sec = 0.0;
  double current_interval_mid_stamp_sec = 0.0;
  Eigen::Vector3f causal_angular_acceleration = Eigen::Vector3f::Zero();
  Eigen::Vector3f omega_hat_at_measurement = Eigen::Vector3f::Zero();
  double omega_prediction_horizon_sec = 0.0;
  AngularMotionEstimatorMode omega_estimator_mode = AngularMotionEstimatorMode::Init;
  Eigen::Vector3f omega_motion = Eigen::Vector3f::Zero();
  Eigen::Vector3f omega_bias = Eigen::Vector3f::Zero();
  Eigen::Vector3f omega_motion_target = Eigen::Vector3f::Zero();
  Eigen::Vector3f omega_bias_target = Eigen::Vector3f::Zero();
  Eigen::Vector3f omega_total_before_limits = Eigen::Vector3f::Zero();
  Eigen::Vector3f omega_total_after_limits = Eigen::Vector3f::Zero();
  BiasCorrectionState bias_state = BiasCorrectionState::Off;
  float bias_deadband_enter_rad = 0.0f;
  float bias_deadband_exit_rad = 0.0f;
  uint32_t bias_pending_frames = 0;
  bool bias_active = false;
  bool raw_rejected = false;
  bool motion_decay_active = false;
  float motion_decay_rate_radps2 = 0.0f;
  float motion_indicator_radps = 0.0f;
  bool motion_bias_suppressed = false;
  float bias_correction_step_rad = 0.0f;
  uint64_t pending_correction_id = 0;
  uint32_t pending_good_frames = 0;
  uint32_t pending_total_frames = 0;
  float consistency_cosine = 1.0f;
  float magnitude_ratio = 1.0f;
  float correction_fraction_applied = 0.0f;
  float applied_rotation_correction_rad = 0.0f;
  float published_pose_translation_step_m = 0.0f;
  float published_pose_rotation_step_rad = 0.0f;
  Eigen::Vector3f linear_velocity_after_limits = Eigen::Vector3f::Zero();
  Eigen::Vector3f angular_velocity_after_limits = Eigen::Vector3f::Zero();
  bool predictor_healthy = true;
  uint32_t consecutive_rejections = 0;
};

struct ReferenceGateConfig
{
  uint32_t confirmation_frames = 1;
  uint32_t max_pending_frames = 6;
  float max_step_translation_m = 0.10f;
  float max_step_rotation_rad = 0.08f;
};

struct PredictedOrbPoseState
{
  bool valid = false;
  bool velocity_valid = false;
  double prediction_horizon_sec = 0.0;
  bool prediction_clamped = false;
  Sophus::SE3f pose;
  Eigen::Vector3f linear_velocity = Eigen::Vector3f::Zero();
  Eigen::Vector3f angular_velocity = Eigen::Vector3f::Zero();
  Eigen::Vector3f angular_acceleration = Eigen::Vector3f::Zero();
  Eigen::Vector3f angular_prediction_delta = Eigen::Vector3f::Zero();
  bool angular_acceleration_clamped = false;
  bool angular_velocity_clamped = false;
};

struct OrbPredictionTiming
{
  double target_measurement_stamp_sec = 0.0;
  double visual_age_sec = 0.0;
};

OrbPredictionTiming ComputeOrbPredictionTiming(
  double measurement_stamp_sec,
  double measurement_arrival_local_sec,
  double target_local_sec);

// Laboratorio F5H: retirar junto con gt_timing_diagnostic.
struct DiagnosticAngularState
{
  Sophus::SE3f pose;
  Eigen::Vector3f angular_velocity = Eigen::Vector3f::Zero();
};

inline DiagnosticAngularState SelectDiagnosticAngularState(
  const Sophus::SE3f & predicted_pose,
  const Eigen::Vector3f & predicted_angular_velocity,
  const Sophus::SE3f & gt_pose_now,
  const Eigen::Vector3f & gt_angular_velocity_now,
  bool use_gt_orientation,
  bool use_gt_angular_velocity)
{
  return {
    Sophus::SE3f(
      use_gt_orientation ? gt_pose_now.so3() : predicted_pose.so3(),
      predicted_pose.translation()),
    use_gt_angular_velocity ? gt_angular_velocity_now : predicted_angular_velocity};
}

// Laboratorio F5H: seleccion completa para la bateria 288-291.
struct DiagnosticControlState
{
  Sophus::SE3f pose;
  Eigen::Vector3f linear_velocity = Eigen::Vector3f::Zero();
  Eigen::Vector3f angular_velocity = Eigen::Vector3f::Zero();
};

inline DiagnosticControlState SelectDiagnosticControlState(
  const Sophus::SE3f & predicted_pose,
  const Eigen::Vector3f & predicted_linear_velocity,
  const Eigen::Vector3f & predicted_angular_velocity,
  const Sophus::SE3f & gt_pose_now,
  const Eigen::Vector3f & gt_linear_velocity_now,
  const Eigen::Vector3f & gt_angular_velocity_now,
  bool use_gt_position,
  bool use_gt_linear_velocity,
  bool use_gt_orientation,
  bool use_gt_angular_velocity)
{
  return {
    Sophus::SE3f(
      use_gt_orientation ? gt_pose_now.so3() : predicted_pose.so3(),
      use_gt_position ? gt_pose_now.translation() : predicted_pose.translation()),
    use_gt_linear_velocity ? gt_linear_velocity_now : predicted_linear_velocity,
    use_gt_angular_velocity ? gt_angular_velocity_now : predicted_angular_velocity};
}

class OrbPosePredictor
{
public:
  explicit OrbPosePredictor(const OrbPosePredictorConfig & config = {});

  PredictedOrbPoseState UpdateMeasurement(
    const Sophus::SE3f & measurement,
    double stamp_sec);
  PredictedOrbPoseState UpdateMeasurement(
    const Sophus::SE3f & measurement,
    double stamp_sec,
    const OrbMeasurementContext & context);
  PredictedOrbPoseState Predict(double stamp_sec) const;
  // Laboratorio F5H: retirar junto con gt_timing_diagnostic.
  void OverrideAngularVelocityForDiagnostics(
    const Eigen::Vector3f & angular_velocity);
  void Reset();

  bool last_update_limited() const;
  bool last_orientation_rejected() const;
  bool healthy() const;
  uint32_t consecutive_angular_rejections() const;
  float last_position_innovation_m() const;
  float last_rotation_innovation_rad() const;
  float last_rotation_step_rad() const;
  const OrbPosePredictorDiagnostics & last_diagnostics() const;

private:
  static Eigen::Vector3f ClampNorm(
    const Eigen::Vector3f & value,
    float max_norm,
    bool & limited);
  Sophus::SE3f Propagate(double dt) const;
  void StartModeratePending(
    const Eigen::Vector3f & innovation,
    double stamp_sec,
    const OrbMeasurementContext & context,
    bool post_reference_switch);
  void ClearModerateState();
  void StartBiasPending(
    const Eigen::Vector3f & innovation,
    double stamp_sec,
    const OrbMeasurementContext & context,
    bool post_reference_switch);
  void ClearBiasPending();

  OrbPosePredictorConfig config_;
  bool valid_ = false;
  bool velocity_valid_ = false;
  double stamp_sec_ = 0.0;
  Sophus::SE3f pose_;
  Eigen::Vector3f linear_velocity_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f angular_velocity_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f omega_motion_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f omega_bias_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f causal_angular_acceleration_ = Eigen::Vector3f::Zero();
  BiasCorrectionState bias_state_ = BiasCorrectionState::Off;
  bool motion_bias_suppressed_ = false;
  bool last_update_limited_ = false;
  bool last_orientation_rejected_ = false;
  uint32_t consecutive_angular_rejections_ = 0;
  float last_position_innovation_m_ = 0.0f;
  float last_rotation_innovation_rad_ = 0.0f;
  float last_rotation_step_rad_ = 0.0f;
  bool raw_measurement_valid_ = false;
  Sophus::SE3f last_raw_measurement_;
  double last_raw_stamp_sec_ = 0.0;
  bool raw_angular_velocity_valid_ = false;
  Eigen::Vector3f previous_raw_angular_velocity_ = Eigen::Vector3f::Zero();
  double previous_raw_interval_mid_stamp_sec_ = 0.0;
  RawDtQuality previous_raw_interval_dt_quality_ = RawDtQuality::Invalid;
  bool angular_history_epoch_valid_ = false;
  uint64_t angular_history_map_epoch_ = 0;
  uint64_t next_pending_correction_id_ = 1;
  bool moderate_pending_valid_ = false;
  bool moderate_confirmed_active_ = false;
  uint64_t pending_correction_id_ = 0;
  double pending_started_stamp_sec_ = 0.0;
  Eigen::Vector3f pending_innovation_ = Eigen::Vector3f::Zero();
  float pending_initial_magnitude_ = 0.0f;
  uint32_t pending_good_frames_ = 0;
  uint32_t pending_total_frames_ = 0;
  uint64_t pending_map_epoch_ = 0;
  uint64_t pending_reference_keyframe_id_ = 0;
  bool pending_post_reference_switch_ = false;
  bool bias_pending_valid_ = false;
  double bias_pending_started_stamp_sec_ = 0.0;
  Eigen::Vector3f bias_pending_innovation_ = Eigen::Vector3f::Zero();
  uint32_t bias_pending_good_frames_ = 0;
  uint32_t bias_pending_total_frames_ = 0;
  uint64_t bias_pending_map_epoch_ = 0;
  uint64_t bias_pending_reference_keyframe_id_ = 0;
  bool bias_pending_post_reference_switch_ = false;
  OrbPosePredictorDiagnostics last_diagnostics_;
};

class NavigationStateEstimator
{
public:
  explicit NavigationStateEstimator(const ReferenceGateConfig & config = {});

  ContinuousPoseResult Update(
    uint64_t map_epoch,
    bool tracking_valid,
    bool reference_valid,
    uint64_t reference_keyframe_id,
    bool tcr_valid,
    const Sophus::SE3f & tcr,
    const Sophus::SE3f & local_t_camera);

  void Reset();

  bool ApplyAuthoritativeGlobalPose(
    uint64_t map_epoch,
    uint64_t reference_keyframe_id,
    uint64_t pose_revision,
    const Sophus::SE3f & w_t_reference);

  void InvalidateGlobalPose(
    uint64_t map_epoch,
    uint64_t reference_keyframe_id);

private:
  ContinuousPoseResult CurrentResult() const;
  void ClearPendingReference();

  ReferenceGateConfig config_;
  bool initialized_ = false;
  bool gap_since_valid_ = false;
  uint64_t map_epoch_ = 0;
  uint64_t reference_keyframe_id_ = 0;
  Sophus::SE3f o_t_reference_;
  Sophus::SE3f last_o_t_camera_;
  Sophus::SE3f active_tcr_;
  bool pending_reference_valid_ = false;
  uint64_t pending_reference_keyframe_id_ = 0;
  Sophus::SE3f pending_o_t_reference_;
  Sophus::SE3f pending_last_o_t_camera_;
  Sophus::SE3f pending_last_local_t_camera_;
  uint32_t pending_good_frames_ = 0;
  uint32_t pending_total_frames_ = 0;
  Sophus::SE3f last_local_t_camera_;
  GlobalPoseState global_state_ = GlobalPoseState::Invalid;
  uint64_t global_pose_revision_ = 0;
  Sophus::SE3f w_t_reference_;
  Sophus::SE3f last_w_t_camera_;
};

}  // namespace orbslam3_ros2

#endif  // ORBSLAM3_ROS2_NAVIGATION_STATE_ESTIMATOR_HPP_
