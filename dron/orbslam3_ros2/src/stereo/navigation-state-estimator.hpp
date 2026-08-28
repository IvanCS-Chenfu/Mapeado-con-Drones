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
  double dt_sec = 0.0;
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
  Sophus::SE3f pose;
  Eigen::Vector3f linear_velocity = Eigen::Vector3f::Zero();
  Eigen::Vector3f angular_velocity = Eigen::Vector3f::Zero();
};

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

  OrbPosePredictorConfig config_;
  bool valid_ = false;
  bool velocity_valid_ = false;
  double stamp_sec_ = 0.0;
  Sophus::SE3f pose_;
  Eigen::Vector3f linear_velocity_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f angular_velocity_ = Eigen::Vector3f::Zero();
  bool last_update_limited_ = false;
  bool last_orientation_rejected_ = false;
  uint32_t consecutive_angular_rejections_ = 0;
  float last_position_innovation_m_ = 0.0f;
  float last_rotation_innovation_rad_ = 0.0f;
  float last_rotation_step_rad_ = 0.0f;
  bool raw_measurement_valid_ = false;
  Sophus::SE3f last_raw_measurement_;
  double last_raw_stamp_sec_ = 0.0;
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
