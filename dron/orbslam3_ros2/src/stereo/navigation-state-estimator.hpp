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
  PredictedOrbPoseState Predict(double stamp_sec) const;
  void Reset();

  bool last_update_limited() const;
  bool last_orientation_rejected() const;
  bool healthy() const;
  uint32_t consecutive_angular_rejections() const;
  float last_position_innovation_m() const;
  float last_rotation_innovation_rad() const;
  float last_rotation_step_rad() const;

private:
  static Eigen::Vector3f ClampNorm(
    const Eigen::Vector3f & value,
    float max_norm,
    bool & limited);
  Sophus::SE3f Propagate(double dt) const;

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
