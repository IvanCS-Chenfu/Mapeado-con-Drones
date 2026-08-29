#ifndef ORBSLAM3_ROS2_NAVIGATION_STATE_ESTIMATOR_HPP_
#define ORBSLAM3_ROS2_NAVIGATION_STATE_ESTIMATOR_HPP_

#include <cstdint>

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
