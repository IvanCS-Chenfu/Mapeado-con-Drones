#include <gtest/gtest.h>

#include "navigation-state-estimator.hpp"

namespace
{

Sophus::SE3f Translation(float x, float y, float z)
{
  return Sophus::SE3f(Sophus::SO3f(), Eigen::Vector3f(x, y, z));
}

Sophus::SE3f RotationZ(float angle_rad)
{
  return Sophus::SE3f(
    Sophus::SO3f::exp(Eigen::Vector3f(0.0f, 0.0f, angle_rad)),
    Eigen::Vector3f::Zero());
}

TEST(NavigationStateEstimator, IgnoresRawLocalBaJumpForSameReference)
{
  orbslam3_ros2::NavigationStateEstimator estimator;
  const auto first = estimator.Update(
    0, true, true, 10, true, Translation(-1.0f, 0.0f, 0.0f),
    Translation(1.0f, 0.0f, 0.0f));
  ASSERT_TRUE(first.continuity_valid);

  const auto after_ba = estimator.Update(
    0, true, true, 10, true, Translation(-1.2f, 0.0f, 0.0f),
    Translation(20.0f, 0.0f, 0.0f));
  EXPECT_TRUE(after_ba.continuity_valid);
  EXPECT_NEAR(after_ba.o_t_camera.translation().x(), 1.2f, 1e-6f);
  EXPECT_LT(after_ba.step_translation_m, 0.21f);
}

TEST(NavigationStateEstimator, ReferenceSwitchHasNoDiscreteJump)
{
  orbslam3_ros2::ReferenceGateConfig config;
  config.confirmation_frames = 2;
  orbslam3_ros2::NavigationStateEstimator estimator(config);
  estimator.Update(
    0, true, true, 10, true, Translation(-1.0f, 0.0f, 0.0f),
    Translation(1.0f, 0.0f, 0.0f));

  const auto pending = estimator.Update(
    0, true, true, 11, true, Translation(3.0f, 0.0f, 0.0f),
    Translation(50.0f, 0.0f, 0.0f));
  EXPECT_TRUE(pending.reference_pending);
  EXPECT_FALSE(pending.reference_changed);
  EXPECT_EQ(pending.active_reference_keyframe_id, 10U);
  EXPECT_NEAR(pending.o_t_camera.translation().x(), 1.0f, 1e-6f);

  const auto switched = estimator.Update(
    0, true, true, 11, true, Translation(3.0f, 0.0f, 0.0f),
    Translation(50.0f, 0.0f, 0.0f));
  EXPECT_TRUE(switched.reference_changed);
  EXPECT_TRUE(switched.continuity_valid);
  EXPECT_NEAR(switched.o_t_camera.translation().x(), 1.0f, 1e-6f);
  EXPECT_NEAR(switched.step_translation_m, 0.0f, 1e-6f);
}

TEST(NavigationStateEstimator, GapDoesNotPretendContinuity)
{
  orbslam3_ros2::NavigationStateEstimator estimator;
  estimator.Update(
    0, true, true, 10, true, Translation(-1.0f, 0.0f, 0.0f),
    Translation(1.0f, 0.0f, 0.0f));
  const auto lost = estimator.Update(
    0, false, false, 0, false, Sophus::SE3f(), Sophus::SE3f());
  EXPECT_FALSE(lost.local_valid);

  const auto recovered = estimator.Update(
    0, true, true, 20, true, Translation(-2.0f, 0.0f, 0.0f),
    Translation(8.0f, 0.0f, 0.0f));
  EXPECT_TRUE(recovered.local_valid);
  EXPECT_FALSE(recovered.continuity_valid);
  EXPECT_NEAR(recovered.o_t_camera.translation().x(), 8.0f, 1e-6f);
}

TEST(NavigationStateEstimator, NewEpochReinitializesFromRawPose)
{
  orbslam3_ros2::NavigationStateEstimator estimator;
  estimator.Update(
    0, true, true, 10, true, Translation(-1.0f, 0.0f, 0.0f),
    Translation(1.0f, 0.0f, 0.0f));
  const auto next_epoch = estimator.Update(
    1, true, true, 0, true, Translation(-2.0f, 0.0f, 0.0f),
    Translation(5.0f, 0.0f, 0.0f));
  EXPECT_TRUE(next_epoch.epoch_changed);
  EXPECT_TRUE(next_epoch.continuity_valid);
  EXPECT_NEAR(next_epoch.o_t_camera.translation().x(), 5.0f, 1e-6f);
}

TEST(NavigationStateEstimator, GlobalAuthorityAndRevisionNeverMoveLocalPose)
{
  orbslam3_ros2::NavigationStateEstimator estimator;
  estimator.Update(
    0, true, true, 10, true, Translation(-1.0f, 0.0f, 0.0f),
    Translation(1.0f, 0.0f, 0.0f));
  ASSERT_TRUE(
    estimator.ApplyAuthoritativeGlobalPose(
      0, 10, 4, Translation(11.0f, 0.0f, 0.0f)));

  const auto authoritative = estimator.Update(
    0, true, true, 10, true, Translation(-1.5f, 0.0f, 0.0f),
    Translation(99.0f, 0.0f, 0.0f));
  EXPECT_EQ(
    authoritative.global_state,
    orbslam3_ros2::GlobalPoseState::Authoritative);
  EXPECT_EQ(authoritative.pose_revision, 4U);
  EXPECT_NEAR(authoritative.o_t_camera.translation().x(), 1.5f, 1e-6f);
  EXPECT_NEAR(authoritative.w_t_camera.translation().x(), 12.5f, 1e-6f);

  const float local_before = authoritative.o_t_camera.translation().x();
  ASSERT_TRUE(
    estimator.ApplyAuthoritativeGlobalPose(
      0, 10, 5, Translation(20.0f, 0.0f, 0.0f)));
  const auto revised = estimator.Update(
    0, true, true, 10, true, Translation(-1.5f, 0.0f, 0.0f),
    Translation(150.0f, 0.0f, 0.0f));
  EXPECT_NEAR(revised.o_t_camera.translation().x(), local_before, 1e-6f);
  EXPECT_NEAR(revised.w_t_camera.translation().x(), 21.5f, 1e-6f);
  EXPECT_FALSE(
    estimator.ApplyAuthoritativeGlobalPose(
      0, 10, 4, Translation(30.0f, 0.0f, 0.0f)));
}

TEST(NavigationStateEstimator, ReferenceSwitchIsProvisionalUntilMatchingReply)
{
  orbslam3_ros2::ReferenceGateConfig config;
  config.confirmation_frames = 2;
  orbslam3_ros2::NavigationStateEstimator estimator(config);
  estimator.Update(
    2, true, true, 10, true, Translation(-1.0f, 0.0f, 0.0f),
    Translation(1.0f, 0.0f, 0.0f));
  ASSERT_TRUE(
    estimator.ApplyAuthoritativeGlobalPose(
      2, 10, 3, Translation(9.0f, 0.0f, 0.0f)));
  estimator.Update(
    2, true, true, 10, true, Translation(-1.0f, 0.0f, 0.0f),
    Translation(1.0f, 0.0f, 0.0f));

  const auto provisional = estimator.Update(
    2, true, true, 11, true, Translation(4.0f, 0.0f, 0.0f),
    Translation(50.0f, 0.0f, 0.0f));
  EXPECT_TRUE(provisional.reference_pending);
  EXPECT_EQ(
    provisional.global_state,
    orbslam3_ros2::GlobalPoseState::Authoritative);
  const auto accepted = estimator.Update(
    2, true, true, 11, true, Translation(4.0f, 0.0f, 0.0f),
    Translation(50.0f, 0.0f, 0.0f));
  EXPECT_EQ(
    accepted.global_state,
    orbslam3_ros2::GlobalPoseState::Provisional);
  EXPECT_EQ(accepted.pose_revision, 0U);
  EXPECT_FALSE(
    estimator.ApplyAuthoritativeGlobalPose(
      2, 10, 4, Translation(40.0f, 0.0f, 0.0f)));
  EXPECT_TRUE(
    estimator.ApplyAuthoritativeGlobalPose(
      2, 11, 1, Translation(14.0f, 0.0f, 0.0f)));

  const auto resolved = estimator.Update(
    2, true, true, 11, true, Translation(4.0f, 0.0f, 0.0f),
    Translation(50.0f, 0.0f, 0.0f));
  EXPECT_EQ(
    resolved.global_state,
    orbslam3_ros2::GlobalPoseState::Authoritative);
  EXPECT_EQ(resolved.pose_revision, 1U);

  const auto next_epoch = estimator.Update(
    3, true, true, 1, true, Translation(-2.0f, 0.0f, 0.0f),
    Translation(2.0f, 0.0f, 0.0f));
  EXPECT_EQ(next_epoch.global_state, orbslam3_ros2::GlobalPoseState::Invalid);
}

TEST(OrbPosePredictor, FiltersTranslationAndPublishesAtIntermediateTimes)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.position_alpha = 1.0f;
  config.max_position_innovation_m = 1.0f;
  config.max_linear_speed_mps = 5.0f;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  EXPECT_FALSE(
    predictor.UpdateMeasurement(Translation(0.0f, 0.0f, 0.0f), 1.0)
    .velocity_valid);
  const auto measured = predictor.UpdateMeasurement(
    Translation(0.1f, 0.0f, 0.0f), 1.1);
  ASSERT_TRUE(measured.velocity_valid);
  EXPECT_NEAR(measured.linear_velocity.x(), 1.0f, 1e-5f);
  EXPECT_NEAR(predictor.Predict(1.12).pose.translation().x(), 0.12f, 1e-5f);
  EXPECT_NEAR(predictor.Predict(1.14).pose.translation().x(), 0.14f, 1e-5f);
}

TEST(OrbPosePredictor, LimitsPositionOutlierBeforeVelocityUpdate)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.position_alpha = 0.5f;
  config.max_position_innovation_m = 0.1f;
  config.max_linear_speed_mps = 2.0f;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(Translation(0.0f, 0.0f, 0.0f), 1.0);
  const auto filtered = predictor.UpdateMeasurement(
    Translation(10.0f, 0.0f, 0.0f), 1.1);
  EXPECT_TRUE(predictor.last_update_limited());
  EXPECT_NEAR(filtered.pose.translation().x(), 0.05f, 1e-5f);
  EXPECT_NEAR(filtered.linear_velocity.x(), 0.5f, 1e-5f);
}

TEST(OrbPosePredictor, TracksMeasuredOrientationWithCoherentVelocity)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.orientation_alpha = 1.0f;
  config.small_rotation_innovation_rad = 0.2f;
  config.max_rotation_innovation_rad = 0.2f;
  config.max_angular_speed_radps = 5.0f;
  orbslam3_ros2::OrbPosePredictor predictor(config);
  const Sophus::SE3f first;
  const Sophus::SE3f second(
    Sophus::SO3f::exp(Eigen::Vector3f(0.0f, 0.0f, 0.1f)),
    Eigen::Vector3f::Zero());

  predictor.UpdateMeasurement(first, 1.0);
  const auto measured = predictor.UpdateMeasurement(second, 1.1);
  ASSERT_TRUE(measured.velocity_valid);
  EXPECT_NEAR((measured.pose.so3().inverse() * second.so3()).log().norm(), 0.0f, 1e-6f);
  EXPECT_NEAR(measured.angular_velocity.z(), 1.0f, 1e-5f);

  const Sophus::SO3f expected = Sophus::SO3f::exp(
    Eigen::Vector3f(0.0f, 0.0f, 0.15f));
  EXPECT_NEAR(
    (predictor.Predict(1.15).pose.so3().inverse() * expected).log().norm(),
    0.0f,
    1e-5f);
}

TEST(OrbPosePredictor, IsolatedModerateAngularInnovationIsDiscarded)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.orientation_alpha = 1.0f;
  config.small_rotation_innovation_rad = 0.02f;
  config.max_rotation_innovation_rad = 0.35f;
  config.max_angular_speed_radps = 1.5f;
  config.max_angular_acceleration_radps2 = 0.0f;
  config.moderate_confirmation_frames = 3;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(Sophus::SE3f(), 1.0);
  const auto pending = predictor.UpdateMeasurement(RotationZ(0.12f), 1.1);

  EXPECT_FALSE(predictor.last_orientation_rejected());
  EXPECT_EQ(
    predictor.last_diagnostics().classification,
    orbslam3_ros2::AngularCorrectionClass::ModeratePending);
  EXPECT_TRUE(pending.velocity_valid);
  EXPECT_NEAR(pending.angular_velocity.norm(), 0.0f, 1e-6f);
  EXPECT_NEAR(pending.pose.so3().log().norm(), 0.0f, 1e-6f);

  const auto discarded = predictor.UpdateMeasurement(Sophus::SE3f(), 1.2);
  EXPECT_EQ(
    predictor.last_diagnostics().classification,
    orbslam3_ros2::AngularCorrectionClass::ModerateDiscarded);
  EXPECT_NEAR(discarded.angular_velocity.norm(), 0.0f, 1e-6f);
  EXPECT_NEAR(discarded.pose.so3().log().norm(), 0.0f, 1e-6f);
}

TEST(OrbPosePredictor, PersistentModerateAngularInnovationIsConfirmedGradually)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.orientation_alpha = 0.5f;
  config.small_rotation_innovation_rad = 0.02f;
  config.max_rotation_innovation_rad = 0.35f;
  config.max_angular_speed_radps = 1.5f;
  config.max_angular_acceleration_radps2 = 0.0f;
  config.moderate_confirmation_frames = 3;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(Sophus::SE3f(), 1.0);
  predictor.UpdateMeasurement(RotationZ(0.12f), 1.1);
  predictor.UpdateMeasurement(RotationZ(0.12f), 1.2);
  const auto confirmed = predictor.UpdateMeasurement(RotationZ(0.12f), 1.3);

  const auto & diagnostics = predictor.last_diagnostics();
  EXPECT_EQ(
    diagnostics.classification,
    orbslam3_ros2::AngularCorrectionClass::ModerateConfirmed);
  EXPECT_EQ(diagnostics.pending_good_frames, 3U);
  EXPECT_GT(diagnostics.applied_rotation_correction_rad, 0.0f);
  EXPECT_LT(diagnostics.applied_rotation_correction_rad, 0.12f);
  EXPECT_NEAR(
    diagnostics.published_pose_rotation_step_rad,
    confirmed.angular_velocity.norm() * 0.1f,
    1e-5f);
}

TEST(OrbPosePredictor, SustainedPhysicalYawDoesNotFreeze)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.orientation_alpha = 1.0f;
  config.small_rotation_innovation_rad = 0.01f;
  config.max_rotation_innovation_rad = 0.35f;
  config.max_angular_speed_radps = 1.5f;
  config.max_angular_acceleration_radps2 = 4.0f;
  config.moderate_confirmation_frames = 3;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(Sophus::SE3f(), 1.0);
  predictor.UpdateMeasurement(RotationZ(0.04f), 1.1);
  predictor.UpdateMeasurement(RotationZ(0.08f), 1.2);
  const auto confirmed = predictor.UpdateMeasurement(RotationZ(0.12f), 1.3);
  EXPECT_EQ(
    predictor.last_diagnostics().classification,
    orbslam3_ros2::AngularCorrectionClass::ModerateConfirmed);
  EXPECT_GT(confirmed.angular_velocity.z(), 0.0f);
  EXPECT_NEAR(confirmed.angular_velocity.z(), 0.4f, 1e-5f);

  const auto followed = predictor.UpdateMeasurement(RotationZ(0.16f), 1.4);
  EXPECT_TRUE(predictor.healthy());
  EXPECT_GT(followed.pose.so3().log().z(), confirmed.pose.so3().log().z());
}

TEST(OrbPosePredictor, FastPlausibleYawHasBoundedLag)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.orientation_alpha = 1.0f;
  config.small_rotation_innovation_rad = 0.01f;
  config.max_rotation_innovation_rad = 0.35f;
  config.max_angular_speed_radps = 1.5f;
  config.max_angular_acceleration_radps2 = 15.0f;
  config.moderate_confirmation_frames = 2;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(Sophus::SE3f(), 1.0);
  predictor.UpdateMeasurement(RotationZ(0.10f), 1.1);
  const auto confirmed = predictor.UpdateMeasurement(RotationZ(0.20f), 1.2);
  EXPECT_EQ(
    predictor.last_diagnostics().classification,
    orbslam3_ros2::AngularCorrectionClass::ModerateConfirmed);
  EXPECT_LE(confirmed.angular_velocity.norm(), 1.5f + 1e-6f);
  EXPECT_LT(
    (confirmed.pose.so3().inverse() * RotationZ(0.20f).so3()).log().norm(),
    0.11f);

  const auto followed = predictor.UpdateMeasurement(RotationZ(0.30f), 1.3);
  EXPECT_TRUE(predictor.healthy());
  EXPECT_LT(
    (followed.pose.so3().inverse() * RotationZ(0.30f).so3()).log().norm(),
    0.11f);
}

TEST(OrbPosePredictor, SameReferenceTcrCorrectionPassesThroughTemporalGate)
{
  orbslam3_ros2::NavigationStateEstimator estimator;
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.small_rotation_innovation_rad = 0.02f;
  config.max_angular_acceleration_radps2 = 0.0f;
  config.moderate_confirmation_frames = 3;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  const auto initial = estimator.Update(
    0, true, true, 10, true, Sophus::SE3f(), Sophus::SE3f());
  predictor.UpdateMeasurement(initial.o_t_camera, 1.0);
  const auto corrected_raw = estimator.Update(
    0, true, true, 10, true, RotationZ(0.12f), Sophus::SE3f());
  ASSERT_TRUE(corrected_raw.measurement_accepted);
  const auto pending = predictor.UpdateMeasurement(corrected_raw.o_t_camera, 1.1);
  EXPECT_EQ(
    predictor.last_diagnostics().classification,
    orbslam3_ros2::AngularCorrectionClass::ModeratePending);
  EXPECT_NEAR(pending.pose.so3().log().norm(), 0.0f, 1e-6f);

  const auto returned_raw = estimator.Update(
    0, true, true, 10, true, Sophus::SE3f(), Sophus::SE3f());
  const auto discarded = predictor.UpdateMeasurement(returned_raw.o_t_camera, 1.2);
  EXPECT_EQ(
    predictor.last_diagnostics().classification,
    orbslam3_ros2::AngularCorrectionClass::ModerateDiscarded);
  EXPECT_NEAR(
    discarded.pose.so3().log().norm(), 0.0f, 1e-6f);
}

TEST(NavigationStateEstimator, RejectsImplausibleReferenceCandidate)
{
  orbslam3_ros2::ReferenceGateConfig config;
  config.confirmation_frames = 2;
  config.max_step_translation_m = 0.1f;
  orbslam3_ros2::NavigationStateEstimator estimator(config);
  estimator.Update(
    0, true, true, 10, true, Translation(-1.0f, 0.0f, 0.0f),
    Translation(1.0f, 0.0f, 0.0f));

  const auto pending = estimator.Update(
    0, true, true, 11, true, Translation(3.0f, 0.0f, 0.0f),
    Translation(50.0f, 0.0f, 0.0f));
  EXPECT_TRUE(pending.reference_pending);

  const auto rejected = estimator.Update(
    0, true, true, 11, true, Translation(2.0f, 0.0f, 0.0f),
    Translation(50.0f, 0.0f, 0.0f));
  EXPECT_TRUE(rejected.reference_rejected);
  EXPECT_FALSE(rejected.measurement_accepted);
  EXPECT_EQ(rejected.active_reference_keyframe_id, 10U);
  EXPECT_NEAR(rejected.o_t_camera.translation().x(), 1.0f, 1e-6f);
}

TEST(NavigationStateEstimator, GeometricReferenceChurnConfirmsLatestReference)
{
  orbslam3_ros2::ReferenceGateConfig config;
  config.confirmation_frames = 3;
  config.max_pending_frames = 6;
  orbslam3_ros2::NavigationStateEstimator estimator(config);
  estimator.Update(
    0, true, true, 10, true, Translation(-1.0f, 0.0f, 0.0f),
    Translation(1.0f, 0.0f, 0.0f));
  EXPECT_TRUE(
    estimator.Update(
      0, true, true, 11, true, Translation(3.0f, 0.0f, 0.0f),
      Translation(1.02f, 0.0f, 0.0f)).reference_pending);
  EXPECT_TRUE(
    estimator.Update(
      0, true, true, 12, true, Translation(4.0f, 0.0f, 0.0f),
      Translation(1.04f, 0.0f, 0.0f)).reference_pending);
  const auto accepted = estimator.Update(
    0, true, true, 13, true, Translation(5.0f, 0.0f, 0.0f),
    Translation(1.06f, 0.0f, 0.0f));
  EXPECT_TRUE(accepted.reference_changed);
  EXPECT_FALSE(accepted.reference_gate_timed_out);
  EXPECT_EQ(accepted.active_reference_keyframe_id, 13U);
  EXPECT_NEAR(accepted.o_t_camera.translation().x(), 1.06f, 1e-5f);
}

TEST(NavigationStateEstimator, PersistentInconsistentReferenceTimesOut)
{
  orbslam3_ros2::ReferenceGateConfig config;
  config.confirmation_frames = 3;
  config.max_pending_frames = 2;
  config.max_step_translation_m = 0.1f;
  orbslam3_ros2::NavigationStateEstimator estimator(config);
  estimator.Update(
    0, true, true, 10, true, Translation(-1.0f, 0.0f, 0.0f),
    Translation(1.0f, 0.0f, 0.0f));
  EXPECT_TRUE(
    estimator.Update(
      0, true, true, 11, true, Translation(3.0f, 0.0f, 0.0f),
      Translation(1.0f, 0.0f, 0.0f)).reference_pending);
  EXPECT_TRUE(
    estimator.Update(
      0, true, true, 11, true, Translation(2.0f, 0.0f, 0.0f),
      Translation(1.0f, 0.0f, 0.0f)).reference_rejected);
  const auto timed_out = estimator.Update(
    0, true, true, 11, true, Translation(2.0f, 0.0f, 0.0f),
    Translation(1.0f, 0.0f, 0.0f));
  EXPECT_TRUE(timed_out.reference_gate_timed_out);
  EXPECT_FALSE(timed_out.local_valid);
}

TEST(NavigationStateEstimator, ReturningActiveReferenceCancelsCandidate)
{
  orbslam3_ros2::ReferenceGateConfig config;
  config.confirmation_frames = 3;
  orbslam3_ros2::NavigationStateEstimator estimator(config);
  estimator.Update(
    0, true, true, 10, true, Translation(-1.0f, 0.0f, 0.0f),
    Translation(1.0f, 0.0f, 0.0f));
  ASSERT_TRUE(
    estimator.Update(
      0, true, true, 11, true, Translation(3.0f, 0.0f, 0.0f),
      Translation(1.0f, 0.0f, 0.0f)).reference_pending);
  const auto active = estimator.Update(
    0, true, true, 10, true, Translation(-1.1f, 0.0f, 0.0f),
    Translation(1.0f, 0.0f, 0.0f));
  EXPECT_FALSE(active.reference_pending);
  EXPECT_TRUE(active.measurement_accepted);
  EXPECT_EQ(active.active_reference_keyframe_id, 10U);
}

TEST(OrbPosePredictor, PostReferenceModerateCorrectionNeedsExtraConfirmation)
{
  orbslam3_ros2::ReferenceGateConfig reference_config;
  reference_config.confirmation_frames = 1;
  orbslam3_ros2::NavigationStateEstimator estimator(reference_config);
  orbslam3_ros2::OrbPosePredictorConfig predictor_config;
  predictor_config.small_rotation_innovation_rad = 0.02f;
  predictor_config.max_angular_acceleration_radps2 = 0.0f;
  predictor_config.moderate_confirmation_frames = 2;
  predictor_config.moderate_post_reference_confirmation_frames = 4;
  predictor_config.post_reference_switch_frames = 5;
  orbslam3_ros2::OrbPosePredictor predictor(predictor_config);

  const auto initial = estimator.Update(
    0, true, true, 10, true, Sophus::SE3f(), Sophus::SE3f());
  predictor.UpdateMeasurement(initial.o_t_camera, 1.0);
  const auto switched = estimator.Update(
    0, true, true, 11, true, Sophus::SE3f(), Sophus::SE3f());
  ASSERT_TRUE(switched.reference_changed);
  orbslam3_ros2::OrbMeasurementContext switch_context;
  switch_context.map_epoch = 0;
  switch_context.tracking_state = 2;
  switch_context.reference_keyframe_id = 11;
  switch_context.reference_changed = true;
  switch_context.frames_since_reference_change = 0;
  predictor.UpdateMeasurement(switched.o_t_camera, 1.1, switch_context);

  const auto corrected_raw = estimator.Update(
    0, true, true, 11, true, RotationZ(0.12f), Sophus::SE3f());
  orbslam3_ros2::OrbMeasurementContext post_switch_context = switch_context;
  post_switch_context.reference_changed = false;
  post_switch_context.frames_since_reference_change = 1;
  const auto pending = predictor.UpdateMeasurement(
    corrected_raw.o_t_camera, 1.2, post_switch_context);
  EXPECT_EQ(
    predictor.last_diagnostics().classification,
    orbslam3_ros2::AngularCorrectionClass::ModeratePending);
  EXPECT_NEAR(pending.pose.so3().log().norm(), 0.0f, 1e-6f);

  post_switch_context.frames_since_reference_change = 2;
  predictor.UpdateMeasurement(corrected_raw.o_t_camera, 1.3, post_switch_context);
  post_switch_context.frames_since_reference_change = 3;
  predictor.UpdateMeasurement(corrected_raw.o_t_camera, 1.4, post_switch_context);
  EXPECT_EQ(
    predictor.last_diagnostics().classification,
    orbslam3_ros2::AngularCorrectionClass::ModeratePending);
  post_switch_context.frames_since_reference_change = 4;
  predictor.UpdateMeasurement(corrected_raw.o_t_camera, 1.5, post_switch_context);
  EXPECT_EQ(
    predictor.last_diagnostics().classification,
    orbslam3_ros2::AngularCorrectionClass::ModerateConfirmed);
}

TEST(OrbPosePredictor, PublishedPoseMatchesPublishedVelocities)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.position_alpha = 1.0f;
  config.orientation_alpha = 1.0f;
  config.small_rotation_innovation_rad = 0.20f;
  config.max_linear_speed_mps = 10.0f;
  config.max_angular_speed_radps = 10.0f;
  config.max_linear_acceleration_mps2 = 0.0f;
  config.max_angular_acceleration_radps2 = 0.0f;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  const Sophus::SE3f initial;
  const Sophus::SE3f measurement(
    RotationZ(0.05f).so3(), Eigen::Vector3f(0.10f, -0.02f, 0.03f));
  const auto first = predictor.UpdateMeasurement(initial, 1.0);
  const auto second = predictor.UpdateMeasurement(measurement, 1.1);
  ASSERT_TRUE(second.velocity_valid);

  const Sophus::SE3f step = first.pose.inverse() * second.pose;
  EXPECT_NEAR(
    step.translation().x(), second.linear_velocity.x() * 0.1f, 1e-6f);
  EXPECT_NEAR(
    step.translation().y(), second.linear_velocity.y() * 0.1f, 1e-6f);
  EXPECT_NEAR(
    step.translation().z(), second.linear_velocity.z() * 0.1f, 1e-6f);
  EXPECT_NEAR(
    step.so3().log().x(), second.angular_velocity.x() * 0.1f, 1e-6f);
  EXPECT_NEAR(
    step.so3().log().y(), second.angular_velocity.y() * 0.1f, 1e-6f);
  EXPECT_NEAR(
    step.so3().log().z(), second.angular_velocity.z() * 0.1f, 1e-6f);
}

TEST(OrbPosePredictor, RejectsAngularOutlierWithCoherentPoseAndVelocity)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.orientation_alpha = 1.0f;
  config.max_rotation_innovation_rad = 0.08f;
  config.max_angular_speed_radps = 1.5f;
  config.max_consecutive_angular_rejections = 3;
  orbslam3_ros2::OrbPosePredictor predictor(config);
  const Sophus::SE3f first;
  const Sophus::SE3f good(
    Sophus::SO3f::exp(Eigen::Vector3f(0.0f, 0.0f, 0.05f)),
    Eigen::Vector3f::Zero());
  const Sophus::SE3f outlier(
    Sophus::SO3f::exp(Eigen::Vector3f(0.0f, 0.0f, 0.40f)),
    Eigen::Vector3f::Zero());

  predictor.UpdateMeasurement(first, 1.0);
  predictor.UpdateMeasurement(good, 1.1);
  const auto rejected = predictor.UpdateMeasurement(outlier, 1.2);
  EXPECT_TRUE(predictor.last_orientation_rejected());
  EXPECT_TRUE(predictor.healthy());
  EXPECT_NEAR(rejected.angular_velocity.z(), 0.5f, 1e-5f);
  const Sophus::SO3f expected = Sophus::SO3f::exp(
    Eigen::Vector3f(0.0f, 0.0f, 0.10f));
  EXPECT_NEAR(
    (rejected.pose.so3().inverse() * expected).log().norm(), 0.0f, 1e-5f);

  predictor.UpdateMeasurement(outlier, 1.3);
  predictor.UpdateMeasurement(outlier, 1.4);
  EXPECT_FALSE(predictor.healthy());
}

}  // namespace
