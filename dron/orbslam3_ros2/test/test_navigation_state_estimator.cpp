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

TEST(OrbPosePredictor, DiagnosticAngularOverridePropagatesExactKnownVelocity)
{
  orbslam3_ros2::OrbPosePredictor predictor;
  predictor.UpdateMeasurement(Sophus::SE3f(), 1.0);
  const Eigen::Vector3f omega(0.2f, -0.1f, 0.3f);
  predictor.OverrideAngularVelocityForDiagnostics(omega);

  const auto predicted = predictor.Predict(1.1);
  ASSERT_TRUE(predicted.valid);
  ASSERT_TRUE(predicted.velocity_valid);
  EXPECT_NEAR((predicted.angular_velocity - omega).norm(), 0.0f, 1e-6f);
  EXPECT_NEAR(
    (predicted.pose.so3().log() - omega * 0.1f).norm(), 0.0f, 1e-5f);
}

TEST(OrbPosePredictor, PredictionTimingUsesLocalAgeAcrossClockDomains)
{
  const auto timing = orbslam3_ros2::ComputeOrbPredictionTiming(
    10.0, 1787993150.0, 1787993150.08);

  EXPECT_NEAR(timing.visual_age_sec, 0.08, 1e-6);
  EXPECT_NEAR(timing.target_measurement_stamp_sec, 10.08, 1e-6);
}

TEST(OrbPosePredictor, PredictionTimingDoesNotRunBackwards)
{
  const auto timing = orbslam3_ros2::ComputeOrbPredictionTiming(
    10.0, 100.1, 100.0);

  EXPECT_DOUBLE_EQ(timing.visual_age_sec, 0.0);
  EXPECT_DOUBLE_EQ(timing.target_measurement_stamp_sec, 10.0);
}

TEST(OrbPosePredictor, PredictionUsesOneMeasuredAndClampedHorizon)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.max_extrapolation_sec = 0.10;
  config.max_angular_speed_radps = 5.0f;
  config.max_angular_acceleration_radps2 = 0.0f;
  config.raw_motion_filter_alpha = 1.0f;
  config.raw_dt_max_good_sec = 0.075;
  config.small_rotation_innovation_rad = 0.20f;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(Sophus::SE3f(), 10.0);
  predictor.UpdateMeasurement(
    Sophus::SE3f(RotationZ(0.05f).so3(), Eigen::Vector3f::Zero()), 10.05);

  const auto fresh_timing = orbslam3_ros2::ComputeOrbPredictionTiming(
    10.05, 500.0, 500.08);
  const auto fresh = predictor.Predict(fresh_timing.target_measurement_stamp_sec);
  ASSERT_TRUE(fresh.valid);
  EXPECT_NEAR(fresh.prediction_horizon_sec, 0.08, 1e-8);
  EXPECT_FALSE(fresh.prediction_clamped);
  EXPECT_NEAR(fresh.pose.so3().log().z(), 0.13f, 1e-5f);

  const auto repeated = predictor.Predict(fresh_timing.target_measurement_stamp_sec);
  EXPECT_NEAR(
    (fresh.pose.so3().inverse() * repeated.pose.so3()).log().norm(), 0.0f, 1e-7f);

  const auto stale_timing = orbslam3_ros2::ComputeOrbPredictionTiming(
    10.05, 500.0, 500.30);
  const auto stale = predictor.Predict(stale_timing.target_measurement_stamp_sec);
  EXPECT_NEAR(stale.prediction_horizon_sec, 0.10, 1e-8);
  EXPECT_TRUE(stale.prediction_clamped);
  EXPECT_NEAR(stale.pose.so3().log().z(), 0.15f, 1e-5f);
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
  config.raw_motion_filter_alpha = 1.0f;
  config.raw_dt_max_good_sec = 0.11;
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

TEST(OrbPosePredictor, SmallPlausibleMeasurementsReanchorVisualBase)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.small_rotation_innovation_rad = 0.20f;
  config.max_rotation_innovation_rad = 0.35f;
  config.max_angular_speed_radps = 5.0f;
  config.max_angular_acceleration_radps2 = 0.0f;
  config.raw_motion_filter_alpha = 0.20f;
  config.raw_dt_max_good_sec = 0.11;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(RotationZ(0.0f), 1.0);
  for (int index = 1; index <= 4; ++index) {
    const float angle = 0.01f * static_cast<float>(index);
    const auto state = predictor.UpdateMeasurement(
      RotationZ(angle), 1.0 + 0.1 * static_cast<double>(index));
    const auto & diagnostics = predictor.last_diagnostics();
    EXPECT_EQ(
      diagnostics.base_update_type,
      orbslam3_ros2::AngularBaseUpdateType::SmallAnchor);
    EXPECT_TRUE(diagnostics.base_update_applied);
    EXPECT_NEAR(diagnostics.visual_base_error_after_rad, 0.0f, 1e-6f);
    EXPECT_NEAR(state.pose.so3().log().z(), angle, 1e-6f);
  }
}

TEST(OrbPosePredictor, VisualAnchorsRemoveAccumulatedIntegrationError)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.small_rotation_innovation_rad = 0.20f;
  config.max_angular_speed_radps = 5.0f;
  config.max_angular_acceleration_radps2 = 0.0f;
  config.raw_motion_filter_alpha = 0.10f;
  config.raw_dt_max_good_sec = 0.11;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(RotationZ(0.0f), 1.0);
  for (int index = 1; index <= 20; ++index) {
    const float angle = 0.005f * static_cast<float>(index);
    predictor.UpdateMeasurement(
      RotationZ(angle), 1.0 + 0.05 * static_cast<double>(index));
  }

  const auto anchored = predictor.Predict(2.0);
  EXPECT_NEAR(anchored.pose.so3().log().z(), 0.10f, 1e-5f);
  EXPECT_NEAR(
    predictor.last_diagnostics().visual_base_error_after_rad, 0.0f, 1e-6f);
}

TEST(OrbPosePredictor, PendingAndRejectedMeasurementsRemainPredictOnly)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.small_rotation_innovation_rad = 0.015f;
  config.max_rotation_innovation_rad = 0.20f;
  config.max_angular_acceleration_radps2 = 0.0f;
  config.raw_motion_filter_alpha = 0.0f;
  config.raw_dt_max_good_sec = 0.11;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(RotationZ(0.0f), 1.0);
  predictor.UpdateMeasurement(RotationZ(0.05f), 1.1);
  EXPECT_EQ(
    predictor.last_diagnostics().base_update_type,
    orbslam3_ros2::AngularBaseUpdateType::ModeratePending);
  EXPECT_FALSE(predictor.last_diagnostics().base_update_applied);

  predictor.UpdateMeasurement(RotationZ(0.40f), 1.2);
  EXPECT_EQ(
    predictor.last_diagnostics().base_update_type,
    orbslam3_ros2::AngularBaseUpdateType::Rejected);
  EXPECT_FALSE(predictor.last_diagnostics().base_update_applied);
}

TEST(OrbPosePredictor, VisualBaseTimestampUsesMeasurementTime)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.small_rotation_innovation_rad = 0.20f;
  config.raw_motion_filter_alpha = 1.0f;
  config.raw_dt_max_good_sec = 0.11;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(RotationZ(0.0f), 10.0);
  predictor.UpdateMeasurement(RotationZ(0.01f), 10.05);
  EXPECT_NEAR(predictor.last_diagnostics().base_stamp_sec, 10.05, 1e-9);
  EXPECT_EQ(
    predictor.last_diagnostics().base_update_type,
    orbslam3_ros2::AngularBaseUpdateType::SmallAnchor);
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
  config.motion_bias_suppression_enter_radps = 10.0f;
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

TEST(OrbPosePredictor, PersistentModerateAngularInnovationAnchorsAfterConfirmation)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.orientation_alpha = 0.5f;
  config.small_rotation_innovation_rad = 0.02f;
  config.max_rotation_innovation_rad = 0.35f;
  config.max_angular_speed_radps = 1.5f;
  config.max_angular_acceleration_radps2 = 0.0f;
  config.moderate_confirmation_frames = 3;
  config.motion_bias_suppression_enter_radps = 10.0f;
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
  EXPECT_EQ(
    diagnostics.base_update_type,
    orbslam3_ros2::AngularBaseUpdateType::ModerateConfirmed);
  EXPECT_TRUE(diagnostics.base_update_applied);
  EXPECT_NEAR(
    diagnostics.base_rotation_correction_rad,
    diagnostics.visual_base_error_before_rad,
    1e-5f);
  EXPECT_NEAR(diagnostics.visual_base_error_after_rad, 0.0f, 1e-5f);
  EXPECT_NEAR(
    (confirmed.pose.so3() * RotationZ(0.12f).so3().inverse()).log().norm(),
    0.0f,
    1e-5f);
}

TEST(OrbPosePredictor, ModerateAnchorDoesNotCreateArtificialAngularVelocity)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.small_rotation_innovation_rad = 0.02f;
  config.max_rotation_innovation_rad = 0.35f;
  config.max_angular_acceleration_radps2 = 0.0f;
  config.moderate_confirmation_frames = 3;
  config.raw_motion_filter_alpha = 0.0f;
  config.raw_dt_max_good_sec = 0.11;
  config.max_raw_angular_speed_radps = 2.0f;
  config.max_raw_angular_acceleration_radps2 = 20.0f;
  config.bias_deadband_enter_rad = 1.0f;
  config.bias_deadband_exit_rad = 0.5f;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(Sophus::SE3f(), 1.0);
  predictor.UpdateMeasurement(RotationZ(0.12f), 1.1);
  predictor.UpdateMeasurement(RotationZ(0.12f), 1.2);
  const auto anchored = predictor.UpdateMeasurement(RotationZ(0.12f), 1.3);

  const auto & diagnostics = predictor.last_diagnostics();
  ASSERT_EQ(
    diagnostics.classification,
    orbslam3_ros2::AngularCorrectionClass::ModerateConfirmed);
  EXPECT_EQ(
    diagnostics.base_update_type,
    orbslam3_ros2::AngularBaseUpdateType::ModerateConfirmed);
  EXPECT_GT(diagnostics.base_rotation_correction_rad, 0.10f);
  EXPECT_NEAR(diagnostics.visual_base_error_after_rad, 0.0f, 1e-5f);
  EXPECT_NEAR(anchored.angular_velocity.norm(), 0.0f, 1e-6f);
  EXPECT_NEAR(diagnostics.omega_motion.norm(), 0.0f, 1e-6f);
  EXPECT_NEAR(diagnostics.omega_bias.norm(), 0.0f, 1e-6f);
}

TEST(OrbPosePredictor, ConfirmedModerateWithRejectedRawRemainsPredictOnly)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.small_rotation_innovation_rad = 0.02f;
  config.max_rotation_innovation_rad = 0.35f;
  config.max_angular_acceleration_radps2 = 0.0f;
  config.moderate_confirmation_frames = 3;
  config.raw_motion_filter_alpha = 0.0f;
  config.raw_dt_max_good_sec = 0.11;
  config.max_raw_angular_speed_radps = 2.0f;
  config.max_raw_angular_acceleration_radps2 = 20.0f;
  config.bias_deadband_enter_rad = 1.0f;
  config.bias_deadband_exit_rad = 0.5f;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(Sophus::SE3f(), 1.0);
  predictor.UpdateMeasurement(RotationZ(0.12f), 1.1);
  predictor.UpdateMeasurement(RotationZ(0.12f), 1.2);
  const auto anchored = predictor.UpdateMeasurement(RotationZ(0.12f), 1.3);
  ASSERT_EQ(
    predictor.last_diagnostics().base_update_type,
    orbslam3_ros2::AngularBaseUpdateType::ModerateConfirmed);

  orbslam3_ros2::OrbPosePredictor gated_predictor(config);
  gated_predictor.UpdateMeasurement(Sophus::SE3f(), 1.0);
  gated_predictor.UpdateMeasurement(RotationZ(0.12f), 1.1);
  gated_predictor.UpdateMeasurement(RotationZ(0.12f), 1.2);
  gated_predictor.UpdateMeasurement(RotationZ(0.12f), 1.3);
  const auto predict_only =
    gated_predictor.UpdateMeasurement(RotationZ(0.18f), 1.6);

  const auto & diagnostics = gated_predictor.last_diagnostics();
  ASSERT_EQ(
    diagnostics.classification,
    orbslam3_ros2::AngularCorrectionClass::ModerateConfirmed);
  EXPECT_EQ(diagnostics.raw_motion_class, orbslam3_ros2::RawMotionClass::Suspicious);
  EXPECT_EQ(
    diagnostics.base_update_type,
    orbslam3_ros2::AngularBaseUpdateType::PredictOnly);
  EXPECT_FALSE(diagnostics.base_update_applied);
  EXPECT_NEAR(
    (predict_only.pose.so3() * anchored.pose.so3().inverse()).log().norm(),
    0.0f,
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
  config.raw_motion_filter_alpha = 1.0f;
  config.raw_dt_max_good_sec = 0.11;
  config.moderate_confirmation_frames = 3;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(Sophus::SE3f(), 1.0);
  predictor.UpdateMeasurement(RotationZ(0.04f), 1.1);
  predictor.UpdateMeasurement(RotationZ(0.08f), 1.2);
  const auto confirmed = predictor.UpdateMeasurement(RotationZ(0.12f), 1.3);
  EXPECT_EQ(
    predictor.last_diagnostics().classification,
    orbslam3_ros2::AngularCorrectionClass::Small);
  EXPECT_EQ(
    predictor.last_diagnostics().raw_motion_class,
    orbslam3_ros2::RawMotionClass::Plausible);
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
  config.raw_motion_filter_alpha = 1.0f;
  config.raw_dt_max_good_sec = 0.11;
  config.moderate_confirmation_frames = 2;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(Sophus::SE3f(), 1.0);
  predictor.UpdateMeasurement(RotationZ(0.10f), 1.1);
  const auto confirmed = predictor.UpdateMeasurement(RotationZ(0.20f), 1.2);
  EXPECT_EQ(
    predictor.last_diagnostics().classification,
    orbslam3_ros2::AngularCorrectionClass::Small);
  EXPECT_EQ(
    predictor.last_diagnostics().raw_motion_class,
    orbslam3_ros2::RawMotionClass::Plausible);
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

TEST(OrbPosePredictor, VisualPoseCorrectionDoesNotBecomeAngularVelocity)
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
  EXPECT_NEAR(step.so3().log().z(), 0.05f, 1e-6f);
  EXPECT_NEAR(second.angular_velocity.z(), 0.5f, 1e-5f);
  EXPECT_NEAR(step.so3().log().z(), second.angular_velocity.z() * 0.1f, 1e-6f);
  EXPECT_EQ(
    predictor.last_diagnostics().base_update_type,
    orbslam3_ros2::AngularBaseUpdateType::SmallAnchor);
}

TEST(OrbPosePredictor, RejectsAngularOutlierWithCoherentPoseAndVelocity)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.orientation_alpha = 1.0f;
  config.max_rotation_innovation_rad = 0.08f;
  config.max_angular_speed_radps = 1.5f;
  config.max_consecutive_angular_rejections = 3;
  config.raw_motion_filter_alpha = 1.0f;
  config.raw_dt_max_good_sec = 0.11;
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
  EXPECT_NEAR(rejected.angular_velocity.z(), 0.1f, 1e-5f);
  const Sophus::SO3f expected = Sophus::SO3f::exp(
    Eigen::Vector3f(0.0f, 0.0f, 0.06f));
  EXPECT_NEAR(
    (rejected.pose.so3().inverse() * expected).log().norm(), 0.0f, 1e-5f);

  predictor.UpdateMeasurement(outlier, 1.3);
  predictor.UpdateMeasurement(outlier, 1.4);
  EXPECT_FALSE(predictor.healthy());
}

TEST(OrbPosePredictor, RawAngularAccelerationUsesPreviousRawVelocity)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.raw_dt_max_good_sec = 0.075;
  config.raw_dt_max_degraded_sec = 0.20;
  config.small_rotation_innovation_rad = 0.20f;
  config.max_angular_speed_radps = 10.0f;
  config.max_angular_acceleration_radps2 = 0.0f;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(RotationZ(0.0f), 1.0);
  predictor.UpdateMeasurement(RotationZ(0.01f), 1.05);
  EXPECT_EQ(
    predictor.last_diagnostics().raw_dt_quality,
    orbslam3_ros2::RawDtQuality::Good);
  EXPECT_NEAR(
    predictor.last_diagnostics().implied_angular_velocity.z(), 0.2f, 1e-5f);

  predictor.UpdateMeasurement(RotationZ(0.04f), 1.15);
  const auto & diagnostics = predictor.last_diagnostics();
  EXPECT_EQ(diagnostics.raw_dt_quality, orbslam3_ros2::RawDtQuality::Degraded);
  EXPECT_NEAR(diagnostics.previous_raw_angular_velocity.z(), 0.2f, 1e-5f);
  EXPECT_NEAR(diagnostics.implied_angular_velocity.z(), 0.3f, 1e-5f);
  EXPECT_NEAR(diagnostics.implied_angular_acceleration.z(), 4.0f / 3.0f, 1e-4f);
}

TEST(OrbPosePredictor, CausalAngularEstimatorRecoversConstantVelocity)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.max_angular_speed_radps = 5.0f;
  config.max_angular_acceleration_radps2 = 0.0f;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(RotationZ(0.00f), 1.00);
  predictor.UpdateMeasurement(RotationZ(0.02f), 1.05);
  predictor.UpdateMeasurement(RotationZ(0.04f), 1.10);

  const auto & diagnostics = predictor.last_diagnostics();
  EXPECT_EQ(
    diagnostics.omega_estimator_mode,
    orbslam3_ros2::AngularMotionEstimatorMode::ThreeSamplePredicted);
  EXPECT_NEAR(diagnostics.causal_angular_acceleration.z(), 0.0f, 1e-5f);
  EXPECT_NEAR(diagnostics.omega_hat_at_measurement.z(), 0.4f, 1e-5f);
}

TEST(OrbPosePredictor, CausalAngularEstimatorProjectsConstantAccelerationToMeasurement)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.max_raw_angular_acceleration_radps2 = 20.0f;
  config.max_angular_speed_radps = 5.0f;
  config.max_angular_acceleration_radps2 = 0.0f;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(RotationZ(0.0000f), 1.00);
  predictor.UpdateMeasurement(RotationZ(0.0025f), 1.05);
  predictor.UpdateMeasurement(RotationZ(0.0100f), 1.10);

  const auto & diagnostics = predictor.last_diagnostics();
  EXPECT_NEAR(diagnostics.implied_angular_velocity.z(), 0.15f, 1e-5f);
  EXPECT_NEAR(diagnostics.causal_angular_acceleration.z(), 2.0f, 1e-4f);
  EXPECT_NEAR(diagnostics.omega_hat_at_measurement.z(), 0.20f, 1e-4f);
  EXPECT_LT(
    std::abs(diagnostics.omega_hat_at_measurement.z() - 0.20f),
    std::abs(diagnostics.implied_angular_velocity.z() - 0.20f));
}

TEST(OrbPosePredictor, CausalAngularEstimatorChangesSignWithoutLowPassLag)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.max_raw_angular_acceleration_radps2 = 20.0f;
  config.max_angular_speed_radps = 5.0f;
  config.max_angular_acceleration_radps2 = 0.0f;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(RotationZ(0.000f), 1.00);
  predictor.UpdateMeasurement(RotationZ(0.010f), 1.05);
  predictor.UpdateMeasurement(RotationZ(0.005f), 1.10);

  EXPECT_LT(predictor.last_diagnostics().omega_hat_at_measurement.z(), 0.0f);
  EXPECT_GT(predictor.last_diagnostics().omega_hat_at_measurement.z(), -0.5f);
}

TEST(OrbPosePredictor, CausalAngularEstimatorUsesIrregularTimestamps)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.raw_dt_max_good_sec = 0.20;
  config.raw_dt_max_degraded_sec = 0.30;
  config.max_angular_speed_radps = 5.0f;
  config.max_angular_acceleration_radps2 = 0.0f;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(RotationZ(0.000f), 1.00);
  predictor.UpdateMeasurement(RotationZ(0.020f), 1.05);
  predictor.UpdateMeasurement(RotationZ(0.060f), 1.15);

  EXPECT_EQ(
    predictor.last_diagnostics().omega_estimator_mode,
    orbslam3_ros2::AngularMotionEstimatorMode::ThreeSamplePredicted);
  EXPECT_NEAR(predictor.last_diagnostics().implied_angular_velocity.z(), 0.4f, 1e-5f);
}

TEST(OrbPosePredictor, CausalAngularEstimatorDegradesSafelyAcrossLargeGap)
{
  orbslam3_ros2::OrbPosePredictor predictor;
  predictor.UpdateMeasurement(RotationZ(0.000f), 1.00);
  predictor.UpdateMeasurement(RotationZ(0.020f), 1.05);
  predictor.UpdateMeasurement(RotationZ(0.040f), 1.20);

  EXPECT_EQ(
    predictor.last_diagnostics().omega_estimator_mode,
    orbslam3_ros2::AngularMotionEstimatorMode::DegradedDt);
  EXPECT_NEAR(predictor.last_diagnostics().causal_angular_acceleration.norm(), 0.0f, 1e-7f);
}

TEST(OrbPosePredictor, AlphaPredictionMatchesConstantVelocityWhenAlphaIsZero)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.predict_angular_acceleration = true;
  config.max_extrapolation_sec = 0.20;
  config.max_angular_speed_radps = 5.0f;
  orbslam3_ros2::OrbPosePredictor predictor(config);
  predictor.UpdateMeasurement(RotationZ(0.00f), 1.00);
  predictor.UpdateMeasurement(RotationZ(0.02f), 1.05);
  predictor.UpdateMeasurement(RotationZ(0.04f), 1.10);

  const auto predicted = predictor.Predict(1.20);
  EXPECT_NEAR(predicted.angular_acceleration.norm(), 0.0f, 1e-5f);
  EXPECT_NEAR(predicted.angular_velocity.z(), 0.4f, 1e-5f);
  EXPECT_NEAR(predicted.pose.so3().log().z(), 0.08f, 1e-4f);
}

TEST(DiagnosticAngularState, SelectsPredictedOrientationAndCurrentGtOmegaFor285)
{
  const auto predicted = RotationZ(0.2f);
  const auto gt = RotationZ(0.7f);
  const auto selected = orbslam3_ros2::SelectDiagnosticAngularState(
    predicted, Eigen::Vector3f(1.0f, 2.0f, 3.0f), gt,
    Eigen::Vector3f(4.0f, 5.0f, 6.0f), false, true);
  EXPECT_NEAR((selected.pose.so3() * predicted.so3().inverse()).log().norm(), 0.0f, 1e-6f);
  EXPECT_NEAR((selected.angular_velocity - Eigen::Vector3f(4.0f, 5.0f, 6.0f)).norm(), 0.0f, 1e-6f);
}

TEST(DiagnosticAngularState, SelectsCurrentGtOrientationAndPredictedOmegaFor286)
{
  const auto predicted = Sophus::SE3f(
    RotationZ(0.2f).so3(), Eigen::Vector3f(1.0f, 2.0f, 3.0f));
  const auto gt = RotationZ(0.7f);
  const auto selected = orbslam3_ros2::SelectDiagnosticAngularState(
    predicted, Eigen::Vector3f(1.0f, 2.0f, 3.0f), gt,
    Eigen::Vector3f(4.0f, 5.0f, 6.0f), true, false);
  EXPECT_NEAR((selected.pose.so3() * gt.so3().inverse()).log().norm(), 0.0f, 1e-6f);
  EXPECT_NEAR((selected.pose.translation() - predicted.translation()).norm(), 0.0f, 1e-6f);
  EXPECT_NEAR((selected.angular_velocity - Eigen::Vector3f(1.0f, 2.0f, 3.0f)).norm(), 0.0f, 1e-6f);
}

TEST(DiagnosticAngularState, SelectsCurrentGtOrientationAndOmegaFor287)
{
  const auto selected = orbslam3_ros2::SelectDiagnosticAngularState(
    RotationZ(0.2f), Eigen::Vector3f(1.0f, 2.0f, 3.0f), RotationZ(0.7f),
    Eigen::Vector3f(4.0f, 5.0f, 6.0f), true, true);
  EXPECT_NEAR((selected.pose.so3() * RotationZ(0.7f).so3().inverse()).log().norm(), 0.0f, 1e-6f);
  EXPECT_NEAR((selected.angular_velocity - Eigen::Vector3f(4.0f, 5.0f, 6.0f)).norm(), 0.0f, 1e-6f);
}

TEST(DiagnosticControlState, SelectsOnlyCurrentGtPositionAndVelocityFor288)
{
  const auto predicted = Sophus::SE3f(
    RotationZ(0.2f).so3(), Eigen::Vector3f(1.0f, 2.0f, 3.0f));
  const auto gt = Sophus::SE3f(
    RotationZ(0.7f).so3(), Eigen::Vector3f(7.0f, 8.0f, 9.0f));
  const auto selected = orbslam3_ros2::SelectDiagnosticControlState(
    predicted, Eigen::Vector3f(1.0f, 2.0f, 3.0f), Eigen::Vector3f(4.0f, 5.0f, 6.0f),
    gt, Eigen::Vector3f(7.0f, 8.0f, 9.0f), Eigen::Vector3f(10.0f, 11.0f, 12.0f),
    true, true, false, false);
  EXPECT_NEAR((selected.pose.translation() - gt.translation()).norm(), 0.0f, 1e-6f);
  EXPECT_NEAR((selected.pose.so3() * predicted.so3().inverse()).log().norm(), 0.0f, 1e-6f);
  EXPECT_NEAR((selected.linear_velocity - Eigen::Vector3f(7.0f, 8.0f, 9.0f)).norm(), 0.0f, 1e-6f);
  EXPECT_NEAR((selected.angular_velocity - Eigen::Vector3f(4.0f, 5.0f, 6.0f)).norm(), 0.0f, 1e-6f);
}

TEST(DiagnosticControlState, SelectsCurrentGtPositionVelocityAndOmegaFor289)
{
  const auto selected = orbslam3_ros2::SelectDiagnosticControlState(
    RotationZ(0.2f), Eigen::Vector3f::Ones(), Eigen::Vector3f(4.0f, 5.0f, 6.0f),
    RotationZ(0.7f), Eigen::Vector3f(7.0f, 8.0f, 9.0f),
    Eigen::Vector3f(10.0f, 11.0f, 12.0f), true, true, false, true);
  EXPECT_NEAR((selected.pose.so3() * RotationZ(0.2f).so3().inverse()).log().norm(), 0.0f, 1e-6f);
  EXPECT_NEAR((selected.angular_velocity - Eigen::Vector3f(10.0f, 11.0f, 12.0f)).norm(), 0.0f, 1e-6f);
}

TEST(DiagnosticControlState, SelectsCurrentGtPositionVelocityAndOrientationFor290)
{
  const auto selected = orbslam3_ros2::SelectDiagnosticControlState(
    RotationZ(0.2f), Eigen::Vector3f::Ones(), Eigen::Vector3f(4.0f, 5.0f, 6.0f),
    RotationZ(0.7f), Eigen::Vector3f(7.0f, 8.0f, 9.0f),
    Eigen::Vector3f(10.0f, 11.0f, 12.0f), true, true, true, false);
  EXPECT_NEAR((selected.pose.so3() * RotationZ(0.7f).so3().inverse()).log().norm(), 0.0f, 1e-6f);
  EXPECT_NEAR((selected.angular_velocity - Eigen::Vector3f(4.0f, 5.0f, 6.0f)).norm(), 0.0f, 1e-6f);
}

TEST(DiagnosticControlState, SelectsCompleteCurrentGtStateFor291)
{
  const auto gt = Sophus::SE3f(
    RotationZ(0.7f).so3(), Eigen::Vector3f(7.0f, 8.0f, 9.0f));
  const auto selected = orbslam3_ros2::SelectDiagnosticControlState(
    RotationZ(0.2f), Eigen::Vector3f::Ones(), Eigen::Vector3f(4.0f, 5.0f, 6.0f),
    gt, Eigen::Vector3f(7.0f, 8.0f, 9.0f), Eigen::Vector3f(10.0f, 11.0f, 12.0f),
    true, true, true, true);
  EXPECT_NEAR((selected.pose * gt.inverse()).log().norm(), 0.0f, 1e-6f);
  EXPECT_NEAR((selected.linear_velocity - Eigen::Vector3f(7.0f, 8.0f, 9.0f)).norm(), 0.0f, 1e-6f);
  EXPECT_NEAR((selected.angular_velocity - Eigen::Vector3f(10.0f, 11.0f, 12.0f)).norm(), 0.0f, 1e-6f);
}

TEST(DiagnosticControlState, SelectsGtPositionAndPredictedVelocityFor307)
{
  const auto predicted = Sophus::SE3f(
    RotationZ(0.2f).so3(), Eigen::Vector3f(1.0f, 2.0f, 3.0f));
  const auto gt = Sophus::SE3f(
    RotationZ(0.7f).so3(), Eigen::Vector3f(7.0f, 8.0f, 9.0f));
  const auto selected = orbslam3_ros2::SelectDiagnosticControlState(
    predicted, Eigen::Vector3f(1.0f, 2.0f, 3.0f), Eigen::Vector3f(4.0f, 5.0f, 6.0f),
    gt, Eigen::Vector3f(7.0f, 8.0f, 9.0f), Eigen::Vector3f(10.0f, 11.0f, 12.0f),
    true, false, true, true);
  EXPECT_NEAR((selected.pose.translation() - gt.translation()).norm(), 0.0f, 1e-6f);
  EXPECT_NEAR((selected.linear_velocity - Eigen::Vector3f(1.0f, 2.0f, 3.0f)).norm(), 0.0f, 1e-6f);
}

TEST(DiagnosticControlState, SelectsPredictedPositionAndGtVelocityFor308)
{
  const auto predicted = Sophus::SE3f(
    RotationZ(0.2f).so3(), Eigen::Vector3f(1.0f, 2.0f, 3.0f));
  const auto gt = Sophus::SE3f(
    RotationZ(0.7f).so3(), Eigen::Vector3f(7.0f, 8.0f, 9.0f));
  const auto selected = orbslam3_ros2::SelectDiagnosticControlState(
    predicted, Eigen::Vector3f(1.0f, 2.0f, 3.0f), Eigen::Vector3f(4.0f, 5.0f, 6.0f),
    gt, Eigen::Vector3f(7.0f, 8.0f, 9.0f), Eigen::Vector3f(10.0f, 11.0f, 12.0f),
    false, true, true, true);
  EXPECT_NEAR((selected.pose.translation() - predicted.translation()).norm(), 0.0f, 1e-6f);
  EXPECT_NEAR((selected.linear_velocity - Eigen::Vector3f(7.0f, 8.0f, 9.0f)).norm(), 0.0f, 1e-6f);
}

TEST(OrbPosePredictor, AlphaPredictionIntegratesKnownConstantAcceleration)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.predict_angular_acceleration = true;
  config.max_extrapolation_sec = 0.20;
  config.max_angular_speed_radps = 5.0f;
  config.max_raw_angular_acceleration_radps2 = 20.0f;
  orbslam3_ros2::OrbPosePredictor predictor(config);
  predictor.UpdateMeasurement(RotationZ(0.0000f), 1.00);
  predictor.UpdateMeasurement(RotationZ(0.0025f), 1.05);
  predictor.UpdateMeasurement(RotationZ(0.0100f), 1.10);

  const auto predicted = predictor.Predict(1.20);
  EXPECT_NEAR(predicted.angular_acceleration.z(), 2.0f, 1e-4f);
  EXPECT_NEAR(predicted.angular_velocity.z(), 0.4f, 1e-4f);
  EXPECT_NEAR(predicted.angular_prediction_delta.z(), 0.03f, 1e-4f);
  EXPECT_NEAR(predicted.pose.so3().log().z(), 0.04f, 1e-4f);
}

TEST(OrbPosePredictor, AlphaPredictionCrossesAngularVelocitySign)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.predict_angular_acceleration = true;
  config.max_extrapolation_sec = 0.20;
  config.max_angular_speed_radps = 5.0f;
  config.max_raw_angular_acceleration_radps2 = 20.0f;
  orbslam3_ros2::OrbPosePredictor predictor(config);
  predictor.UpdateMeasurement(RotationZ(0.000f), 1.00);
  predictor.UpdateMeasurement(RotationZ(0.010f), 1.05);
  predictor.UpdateMeasurement(RotationZ(0.015f), 1.10);

  const auto predicted = predictor.Predict(1.15);
  EXPECT_NEAR(predicted.angular_acceleration.z(), -2.0f, 1e-4f);
  EXPECT_LT(predicted.angular_velocity.z(), 0.0f);
}

TEST(OrbPosePredictor, AlphaPredictionUsesClampedHorizon)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.predict_angular_acceleration = true;
  config.max_extrapolation_sec = 0.12;
  config.max_angular_speed_radps = 5.0f;
  config.max_raw_angular_acceleration_radps2 = 20.0f;
  orbslam3_ros2::OrbPosePredictor predictor(config);
  predictor.UpdateMeasurement(RotationZ(0.0000f), 1.00);
  predictor.UpdateMeasurement(RotationZ(0.0025f), 1.05);
  predictor.UpdateMeasurement(RotationZ(0.0100f), 1.10);

  const auto predicted = predictor.Predict(1.40);
  EXPECT_TRUE(predicted.prediction_clamped);
  EXPECT_NEAR(predicted.prediction_horizon_sec, 0.12, 1e-6);
  EXPECT_NEAR(predicted.angular_velocity.z(), 0.44f, 1e-4f);
}

TEST(OrbPosePredictor, AlphaPredictionPoseAndOmegaShareTargetTime)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.predict_angular_acceleration = true;
  config.max_extrapolation_sec = 0.20;
  config.max_angular_speed_radps = 5.0f;
  config.max_raw_angular_acceleration_radps2 = 20.0f;
  orbslam3_ros2::OrbPosePredictor predictor(config);
  predictor.UpdateMeasurement(RotationZ(0.0000f), 1.00);
  predictor.UpdateMeasurement(RotationZ(0.0025f), 1.05);
  predictor.UpdateMeasurement(RotationZ(0.0100f), 1.10);

  const auto predicted = predictor.Predict(1.18);
  const float dt = static_cast<float>(predicted.prediction_horizon_sec);
  const float expected_omega = 0.20f + 2.0f * dt;
  const float expected_angle = 0.010f + 0.20f * dt + 0.5f * 2.0f * dt * dt;
  EXPECT_NEAR(predicted.angular_velocity.z(), expected_omega, 1e-4f);
  EXPECT_NEAR(predicted.pose.so3().log().z(), expected_angle, 1e-4f);
}

TEST(OrbPosePredictor, RejectedMeasurementDoesNotEnterCausalAngularHistory)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.max_angular_acceleration_radps2 = 0.0f;
  orbslam3_ros2::OrbPosePredictor predictor(config);
  predictor.UpdateMeasurement(RotationZ(0.000f), 1.00);
  predictor.UpdateMeasurement(RotationZ(0.020f), 1.05);
  predictor.UpdateMeasurement(RotationZ(0.300f), 1.10);
  ASSERT_EQ(
    predictor.last_diagnostics().omega_estimator_mode,
    orbslam3_ros2::AngularMotionEstimatorMode::Rejected);

  predictor.UpdateMeasurement(RotationZ(0.060f), 1.15);
  EXPECT_NEAR(predictor.last_diagnostics().raw_dt_sec, 0.10, 1e-6);
  EXPECT_NEAR(predictor.last_diagnostics().implied_angular_velocity.z(), 0.4f, 1e-5f);
}

TEST(OrbPosePredictor, InvalidRawAgeRebasesWithoutAcceptingInvalidDelta)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.raw_dt_max_good_sec = 0.075;
  config.raw_dt_max_degraded_sec = 0.20;
  config.max_raw_rotation_step_rad = 0.10f;
  config.max_raw_angular_acceleration_radps2 = 0.0f;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(RotationZ(0.00f), 1.00);
  predictor.UpdateMeasurement(RotationZ(0.02f), 1.05);
  predictor.UpdateMeasurement(RotationZ(0.40f), 1.10);
  EXPECT_FALSE(predictor.last_diagnostics().raw_history_advanced);

  predictor.UpdateMeasurement(RotationZ(0.42f), 1.31);
  const auto rebased = predictor.last_diagnostics();
  EXPECT_EQ(rebased.raw_dt_quality, orbslam3_ros2::RawDtQuality::Invalid);
  EXPECT_EQ(rebased.raw_motion_class, orbslam3_ros2::RawMotionClass::Suspicious);
  EXPECT_EQ(rebased.base_update_type, orbslam3_ros2::AngularBaseUpdateType::Rejected);
  EXPECT_TRUE(rebased.raw_history_advanced);
  EXPECT_TRUE(rebased.raw_history_rebased);

  predictor.UpdateMeasurement(RotationZ(0.44f), 1.36);
  const auto recovered = predictor.last_diagnostics();
  EXPECT_EQ(recovered.raw_dt_quality, orbslam3_ros2::RawDtQuality::Good);
  EXPECT_EQ(recovered.raw_motion_class, orbslam3_ros2::RawMotionClass::Plausible);
  EXPECT_FALSE(recovered.raw_history_rebased);
  EXPECT_NEAR(recovered.raw_dt_sec, 0.05, 1e-6);
}

TEST(OrbPosePredictor, EpochChangeResetsCausalAngularHistory)
{
  orbslam3_ros2::OrbPosePredictor predictor;
  orbslam3_ros2::OrbMeasurementContext context{1, 2, 10, false, 100};
  predictor.UpdateMeasurement(RotationZ(0.000f), 1.00, context);
  predictor.UpdateMeasurement(RotationZ(0.020f), 1.05, context);
  context.map_epoch = 2;
  predictor.UpdateMeasurement(RotationZ(0.020f), 1.10, context);

  EXPECT_EQ(
    predictor.last_diagnostics().omega_estimator_mode,
    orbslam3_ros2::AngularMotionEstimatorMode::Rejected);
  EXPECT_NEAR(predictor.last_diagnostics().omega_motion.norm(), 0.0f, 1e-7f);
}

TEST(OrbPosePredictor, VisualAnchorDoesNotAlterCausalAngularEstimate)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.small_rotation_innovation_rad = 0.20f;
  config.max_angular_speed_radps = 5.0f;
  config.max_angular_acceleration_radps2 = 0.0f;
  orbslam3_ros2::OrbPosePredictor predictor(config);
  predictor.UpdateMeasurement(RotationZ(0.00f), 1.00);
  predictor.UpdateMeasurement(RotationZ(0.02f), 1.05);
  predictor.UpdateMeasurement(RotationZ(0.04f), 1.10);

  EXPECT_EQ(
    predictor.last_diagnostics().base_update_type,
    orbslam3_ros2::AngularBaseUpdateType::SmallAnchor);
  EXPECT_NEAR(predictor.last_diagnostics().omega_motion.z(), 0.4f, 1e-5f);
}

TEST(OrbPosePredictor, HoverNoiseDoesNotCreateGrowingAngularMotion)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.raw_motion_filter_alpha = 0.35f;
  config.max_orientation_bias_correction_rate_radps = 0.08f;
  config.max_orientation_bias_correction_acceleration_radps2 = 0.8f;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(RotationZ(0.0f), 1.0);
  for (int index = 1; index <= 80; ++index) {
    const float noise = index % 2 == 0 ? 0.002f : -0.002f;
    predictor.UpdateMeasurement(RotationZ(noise), 1.0 + 0.05 * index);
  }

  const auto state = predictor.Predict(5.0);
  EXPECT_LT(predictor.last_diagnostics().omega_motion.norm(), 0.03f);
  EXPECT_LT(state.angular_velocity.norm(), 0.05f);
  EXPECT_LT(state.pose.so3().log().norm(), 0.015f);
}

TEST(OrbPosePredictor, PersistentAbsoluteOffsetUsesBoundedBiasCorrection)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.raw_motion_filter_alpha = 0.35f;
  config.max_orientation_bias_correction_rate_radps = 0.05f;
  config.max_orientation_bias_correction_acceleration_radps2 = 0.5f;
  config.moderate_confirmation_frames = 3;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(RotationZ(0.0f), 1.0);
  predictor.UpdateMeasurement(RotationZ(0.12f), 1.1);
  predictor.UpdateMeasurement(RotationZ(0.12f), 1.2);
  predictor.UpdateMeasurement(RotationZ(0.12f), 1.3);
  predictor.UpdateMeasurement(RotationZ(0.12f), 1.4);

  const auto & diagnostics = predictor.last_diagnostics();
  EXPECT_LT(diagnostics.omega_motion.norm(), 0.15f);
  EXPECT_LE(diagnostics.omega_bias.norm(), 0.05f + 1e-6f);
  EXPECT_LE(diagnostics.bias_correction_step_rad, 0.005f + 1e-6f);
  EXPECT_LE(diagnostics.published_pose_rotation_step_rad, 0.020001f);
}

TEST(OrbPosePredictor, StableRawPoseDoesNotTurnResidualIntoMotion)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.max_orientation_bias_correction_rate_radps = 0.04f;
  config.max_orientation_bias_correction_acceleration_radps2 = 0.4f;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(RotationZ(0.0f), 1.0);
  predictor.UpdateMeasurement(RotationZ(0.10f), 1.1);
  for (int index = 2; index <= 12; ++index) {
    predictor.UpdateMeasurement(RotationZ(0.10f), 1.0 + 0.1 * index);
  }

  EXPECT_LT(predictor.last_diagnostics().implied_angular_velocity.norm(), 1e-6f);
  EXPECT_LT(predictor.last_diagnostics().omega_motion.norm(), 0.025f);
  EXPECT_LE(predictor.last_diagnostics().omega_bias.norm(), 0.04f + 1e-6f);
}

TEST(OrbPosePredictor, LargeDtDoesNotExpandSmallResidualThreshold)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.small_rotation_innovation_rad = 0.015f;
  config.max_raw_rotation_step_rad = 0.01f;
  config.max_angular_acceleration_radps2 = 12.0f;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(RotationZ(0.0f), 1.0);
  predictor.UpdateMeasurement(RotationZ(0.08f), 1.15);

  EXPECT_EQ(
    predictor.last_diagnostics().raw_dt_quality,
    orbslam3_ros2::RawDtQuality::Degraded);
  EXPECT_NEAR(predictor.last_diagnostics().small_rotation_limit_rad, 0.015f, 1e-6f);
  EXPECT_NE(
    predictor.last_diagnostics().classification,
    orbslam3_ros2::AngularCorrectionClass::Small);
}

TEST(OrbPosePredictor, AcceleratingPhysicalYawUsesRawMotionChannel)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.raw_motion_filter_alpha = 1.0f;
  config.max_raw_angular_acceleration_radps2 = 10.0f;
  config.max_angular_speed_radps = 2.0f;
  config.max_angular_acceleration_radps2 = 20.0f;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(RotationZ(0.0f), 1.0);
  predictor.UpdateMeasurement(RotationZ(0.02f), 1.1);
  predictor.UpdateMeasurement(RotationZ(0.06f), 1.2);

  EXPECT_EQ(
    predictor.last_diagnostics().raw_motion_class,
    orbslam3_ros2::RawMotionClass::DegradedDt);
  EXPECT_NEAR(predictor.last_diagnostics().implied_angular_velocity.z(), 0.4f, 1e-5f);
  EXPECT_NEAR(predictor.last_diagnostics().implied_angular_acceleration.z(), 2.0f, 1e-4f);
  EXPECT_GT(predictor.last_diagnostics().omega_motion.z(), 0.0f);
}

TEST(OrbPosePredictor, SmallResidualInsideDeadbandKeepsBiasOff)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.raw_motion_filter_alpha = 0.0f;
  config.motion_bias_suppression_enter_radps = 10.0f;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(RotationZ(0.0f), 1.0);
  for (int index = 1; index <= 10; ++index) {
    predictor.UpdateMeasurement(RotationZ(0.003f), 1.0 + 0.05 * index);
  }

  EXPECT_EQ(
    predictor.last_diagnostics().bias_state,
    orbslam3_ros2::BiasCorrectionState::Off);
  EXPECT_NEAR(predictor.last_diagnostics().omega_bias.norm(), 0.0f, 1e-7f);
  EXPECT_NEAR(predictor.last_diagnostics().omega_motion.norm(), 0.0f, 1e-7f);
}

TEST(OrbPosePredictor, AlternatingResidualNoiseDoesNotActivateBias)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.raw_motion_filter_alpha = 0.0f;
  config.motion_bias_suppression_enter_radps = 10.0f;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(RotationZ(0.0f), 1.0);
  for (int index = 1; index <= 200; ++index) {
    const float noise = index % 2 == 0 ? 0.0015f : -0.0015f;
    predictor.UpdateMeasurement(RotationZ(noise), 1.0 + 0.05 * index);
  }

  EXPECT_EQ(
    predictor.last_diagnostics().bias_state,
    orbslam3_ros2::BiasCorrectionState::Off);
  EXPECT_NEAR(predictor.last_diagnostics().omega_bias.norm(), 0.0f, 1e-7f);
}

TEST(OrbPosePredictor, BiasDeadbandUsesHysteresis)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.raw_motion_filter_alpha = 0.0f;
  config.motion_bias_suppression_enter_radps = 10.0f;
  config.max_orientation_bias_correction_acceleration_radps2 = 0.8f;
  config.raw_dt_max_good_sec = 0.01;
  config.raw_dt_max_degraded_sec = 0.01;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(RotationZ(0.0f), 1.0);
  predictor.UpdateMeasurement(RotationZ(0.010f), 1.1);
  predictor.UpdateMeasurement(RotationZ(0.010f), 1.2);
  predictor.UpdateMeasurement(RotationZ(0.010f), 1.3);
  ASSERT_EQ(
    predictor.last_diagnostics().bias_state,
    orbslam3_ros2::BiasCorrectionState::Active);

  const float predicted_angle = predictor.Predict(1.4).pose.so3().log().z();
  predictor.UpdateMeasurement(RotationZ(predicted_angle + 0.003f), 1.4);
  EXPECT_EQ(
    predictor.last_diagnostics().bias_state,
    orbslam3_ros2::BiasCorrectionState::Active);

  const float next_predicted_angle = predictor.Predict(1.5).pose.so3().log().z();
  predictor.UpdateMeasurement(RotationZ(next_predicted_angle + 0.001f), 1.5);
  EXPECT_NE(
    predictor.last_diagnostics().bias_state,
    orbslam3_ros2::BiasCorrectionState::Active);
}

TEST(OrbPosePredictor, PersistentResidualOutsideDeadbandActivatesBoundedBias)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.raw_motion_filter_alpha = 0.0f;
  config.motion_bias_suppression_enter_radps = 10.0f;
  config.max_orientation_bias_correction_rate_radps = 0.05f;
  config.max_orientation_bias_correction_acceleration_radps2 = 0.5f;
  config.raw_dt_max_good_sec = 0.01;
  config.raw_dt_max_degraded_sec = 0.01;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(RotationZ(0.0f), 1.0);
  predictor.UpdateMeasurement(RotationZ(0.012f), 1.1);
  EXPECT_EQ(
    predictor.last_diagnostics().bias_state,
    orbslam3_ros2::BiasCorrectionState::Pending);
  predictor.UpdateMeasurement(RotationZ(0.012f), 1.2);
  predictor.UpdateMeasurement(RotationZ(0.012f), 1.3);

  EXPECT_EQ(
    predictor.last_diagnostics().bias_state,
    orbslam3_ros2::BiasCorrectionState::Active);
  EXPECT_GT(predictor.last_diagnostics().omega_bias.norm(), 0.0f);
  EXPECT_LE(predictor.last_diagnostics().omega_bias.norm(), 0.05f + 1e-6f);
}

TEST(OrbPosePredictor, SignificantRawMotionSuppressesBias)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.raw_motion_filter_alpha = 0.5f;
  config.max_angular_acceleration_radps2 = 0.0f;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(RotationZ(0.0f), 1.0);
  predictor.UpdateMeasurement(RotationZ(0.020f), 1.1);

  EXPECT_TRUE(predictor.last_diagnostics().motion_bias_suppressed);
  EXPECT_GT(predictor.last_diagnostics().omega_motion.norm(), 0.0f);
  EXPECT_NEAR(predictor.last_diagnostics().omega_bias_target.norm(), 0.0f, 1e-7f);
  EXPECT_NE(
    predictor.last_diagnostics().bias_state,
    orbslam3_ros2::BiasCorrectionState::Active);
}

TEST(OrbPosePredictor, RejectedRawMotionDecaysInsteadOfHolding)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.raw_motion_filter_alpha = 1.0f;
  config.max_angular_acceleration_radps2 = 0.0f;
  config.rejected_motion_decay_acceleration_radps2 = 4.0f;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(RotationZ(0.0f), 1.0);
  predictor.UpdateMeasurement(RotationZ(0.025f), 1.05);
  const float before_rejection = predictor.last_diagnostics().omega_motion.norm();
  predictor.UpdateMeasurement(RotationZ(0.200f), 1.10);
  const float after_first = predictor.last_diagnostics().omega_motion.norm();
  EXPECT_EQ(
    predictor.last_diagnostics().raw_motion_class,
    orbslam3_ros2::RawMotionClass::Rejected);
  EXPECT_TRUE(predictor.last_diagnostics().motion_decay_active);
  EXPECT_LT(after_first, before_rejection);

  predictor.UpdateMeasurement(RotationZ(0.400f), 1.15);
  EXPECT_LT(predictor.last_diagnostics().omega_motion.norm(), after_first);
  predictor.UpdateMeasurement(RotationZ(0.600f), 1.20);
  EXPECT_NEAR(predictor.last_diagnostics().omega_motion.norm(), 0.0f, 1e-6f);
}

TEST(OrbPosePredictor, RawMotionRecoversSmoothlyAfterRejection)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.raw_motion_filter_alpha = 1.0f;
  config.max_angular_acceleration_radps2 = 0.0f;
  config.rejected_motion_decay_acceleration_radps2 = 4.0f;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(RotationZ(0.0f), 1.0);
  predictor.UpdateMeasurement(RotationZ(0.025f), 1.05);
  predictor.UpdateMeasurement(RotationZ(0.200f), 1.10);
  predictor.UpdateMeasurement(RotationZ(0.035f), 1.15);
  const float decayed = predictor.last_diagnostics().omega_motion.norm();
  predictor.UpdateMeasurement(RotationZ(0.045f), 1.20);

  EXPECT_EQ(
    predictor.last_diagnostics().raw_motion_class,
    orbslam3_ros2::RawMotionClass::Plausible);
  EXPECT_GT(predictor.last_diagnostics().omega_motion.norm(), decayed);
  EXPECT_LE(predictor.last_diagnostics().omega_motion.norm(), 0.20f + 1e-5f);
}

TEST(OrbPosePredictor, LongSyntheticHoverDoesNotCreateAngularFeedback)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  predictor.UpdateMeasurement(RotationZ(0.0f), 1.0);
  for (int index = 1; index <= 4000; ++index) {
    const float noise = static_cast<float>((index % 7) - 3) * 0.0003f;
    predictor.UpdateMeasurement(RotationZ(noise), 1.0 + 0.05 * index);
  }

  const auto state = predictor.Predict(201.0);
  EXPECT_EQ(
    predictor.last_diagnostics().bias_state,
    orbslam3_ros2::BiasCorrectionState::Off);
  EXPECT_NEAR(predictor.last_diagnostics().omega_bias.norm(), 0.0f, 1e-7f);
  EXPECT_LT(predictor.last_diagnostics().omega_motion.norm(), 0.02f);
  EXPECT_LT(state.pose.so3().log().norm(), 0.01f);
}

TEST(OrbPosePredictor, PoseAndOmegaRemainCoherentDuringBiasDecay)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.raw_motion_filter_alpha = 0.0f;
  config.motion_bias_suppression_enter_radps = 10.0f;
  config.raw_dt_max_good_sec = 0.01;
  config.raw_dt_max_degraded_sec = 0.01;
  orbslam3_ros2::OrbPosePredictor predictor(config);

  auto previous = predictor.UpdateMeasurement(RotationZ(0.0f), 1.0);
  previous = predictor.UpdateMeasurement(RotationZ(0.012f), 1.1);
  previous = predictor.UpdateMeasurement(RotationZ(0.012f), 1.2);
  previous = predictor.UpdateMeasurement(RotationZ(0.012f), 1.3);
  const float predicted_angle = predictor.Predict(1.4).pose.so3().log().z();
  const auto decaying = predictor.UpdateMeasurement(
    RotationZ(predicted_angle + 0.001f), 1.4);
  const Sophus::SO3f step = decaying.pose.so3() * previous.pose.so3().inverse();

  EXPECT_NE(
    predictor.last_diagnostics().bias_state,
    orbslam3_ros2::BiasCorrectionState::Active);
  EXPECT_NEAR(
    step.log().z(), decaying.angular_velocity.z() * 0.1f, 1e-5f);
}

TEST(OrbPosePredictor, BiasNeedsExtraConfirmationAfterReferenceChange)
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.raw_motion_filter_alpha = 0.0f;
  config.motion_bias_suppression_enter_radps = 10.0f;
  config.bias_confirmation_frames = 3;
  config.bias_post_reference_confirmation_frames = 4;
  config.raw_dt_max_good_sec = 0.01;
  config.raw_dt_max_degraded_sec = 0.01;
  orbslam3_ros2::OrbPosePredictor predictor(config);
  orbslam3_ros2::OrbMeasurementContext context;
  context.map_epoch = 0;
  context.tracking_state = 2;
  context.reference_keyframe_id = 10;

  predictor.UpdateMeasurement(RotationZ(0.0f), 1.0, context);
  context.reference_keyframe_id = 11;
  context.reference_changed = true;
  context.frames_since_reference_change = 0;
  predictor.UpdateMeasurement(RotationZ(0.010f), 1.1, context);
  context.reference_changed = false;
  context.frames_since_reference_change = 1;
  predictor.UpdateMeasurement(RotationZ(0.010f), 1.2, context);
  context.frames_since_reference_change = 2;
  predictor.UpdateMeasurement(RotationZ(0.010f), 1.3, context);
  EXPECT_EQ(
    predictor.last_diagnostics().bias_state,
    orbslam3_ros2::BiasCorrectionState::Pending);
  context.frames_since_reference_change = 3;
  predictor.UpdateMeasurement(RotationZ(0.010f), 1.4, context);
  EXPECT_EQ(
    predictor.last_diagnostics().bias_state,
    orbslam3_ros2::BiasCorrectionState::Active);
}

TEST(BodyTorqueDynamicPredictor, ZeroTorqueKeepsRestState)
{
  orbslam3_ros2::BodyTorqueDynamicPredictor predictor(Eigen::Matrix3f::Identity());
  predictor.AddTorque(1.0, Eigen::Vector3f::Zero());
  const auto result = predictor.Predict(
    Sophus::SO3f(), Eigen::Vector3f::Zero(), 1.0, 1.1);

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.angular_velocity_body.norm(), 0.0f, 1e-7f);
  EXPECT_NEAR(result.orientation.log().norm(), 0.0f, 1e-7f);
}

TEST(BodyTorqueDynamicPredictor, ConstantPrincipalTorqueMatchesAnalyticOmega)
{
  Eigen::Matrix3f inertia = Eigen::Matrix3f::Identity();
  inertia(0, 0) = 2.0f;
  orbslam3_ros2::BodyTorqueDynamicPredictor predictor(inertia);
  predictor.AddTorque(1.0, Eigen::Vector3f(4.0f, 0.0f, 0.0f));
  const auto result = predictor.Predict(
    Sophus::SO3f(), Eigen::Vector3f::Zero(), 1.0, 1.25);

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.angular_velocity_body.x(), 0.5f, 1e-6f);
  EXPECT_NEAR(result.angular_velocity_body.y(), 0.0f, 1e-7f);
  EXPECT_GT(result.orientation.log().x(), 0.0f);
}

TEST(BodyTorqueDynamicPredictor, TorqueSignControlsAngularAccelerationSign)
{
  orbslam3_ros2::BodyTorqueDynamicPredictor positive(Eigen::Matrix3f::Identity());
  orbslam3_ros2::BodyTorqueDynamicPredictor negative(Eigen::Matrix3f::Identity());
  positive.AddTorque(0.0, Eigen::Vector3f(0.0f, 0.0f, 1.0f));
  negative.AddTorque(0.0, Eigen::Vector3f(0.0f, 0.0f, -1.0f));
  const auto p = positive.Predict(Sophus::SO3f(), Eigen::Vector3f::Zero(), 0.0, 0.1);
  const auto n = negative.Predict(Sophus::SO3f(), Eigen::Vector3f::Zero(), 0.0, 0.1);
  ASSERT_TRUE(p.valid && n.valid);
  EXPECT_GT(p.angular_velocity_body.z(), 0.0f);
  EXPECT_LT(n.angular_velocity_body.z(), 0.0f);
}

TEST(BodyTorqueDynamicPredictor, GyroscopicTermCouplesAnisotropicAxes)
{
  Eigen::Matrix3f inertia = Eigen::Vector3f(1.0f, 2.0f, 3.0f).asDiagonal();
  orbslam3_ros2::BodyTorqueDynamicPredictor predictor(inertia);
  predictor.AddTorque(0.0, Eigen::Vector3f::Zero());
  const auto result = predictor.Predict(
    Sophus::SO3f(), Eigen::Vector3f(1.0f, 1.0f, 0.0f), 0.0, 0.01);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.angular_velocity_body.z(), -1.0f / 300.0f, 1e-6f);
}

TEST(BodyTorqueDynamicPredictor, UsesIrregularTorqueTimestamps)
{
  orbslam3_ros2::BodyTorqueDynamicPredictor predictor(Eigen::Matrix3f::Identity());
  predictor.AddTorque(0.0, Eigen::Vector3f(1.0f, 0.0f, 0.0f));
  predictor.AddTorque(0.03, Eigen::Vector3f(3.0f, 0.0f, 0.0f));
  const auto result = predictor.Predict(
    Sophus::SO3f(), Eigen::Vector3f::Zero(), 0.0, 0.10);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.integration_steps, 2U);
  EXPECT_NEAR(result.angular_velocity_body.x(), 0.24f, 1e-6f);
}

TEST(BodyTorqueDynamicPredictor, ReportsMissingTorqueAtIntervalStart)
{
  orbslam3_ros2::BodyTorqueDynamicPredictor predictor(Eigen::Matrix3f::Identity());
  predictor.AddTorque(1.1, Eigen::Vector3f::Zero());
  const auto result = predictor.Predict(
    Sophus::SO3f(), Eigen::Vector3f::Zero(), 1.0, 1.2);
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.missing_torque_interval);
  EXPECT_EQ(
    result.torque_coverage.status,
    orbslam3_ros2::ActuationCoverageStatus::MissingPrefix);
  EXPECT_NEAR(result.torque_coverage.missing_prefix_sec, 0.1, 1e-9);
}

TEST(BodyTorqueDynamicPredictor, ColdStartZeroSeedCoversUntilFirstCommand)
{
  orbslam3_ros2::BodyTorqueDynamicPredictor predictor(Eigen::Matrix3f::Identity());
  predictor.AddTorque(1.0, Eigen::Vector3f::Zero());
  predictor.AddTorque(1.2, Eigen::Vector3f(2.0f, 0.0f, 0.0f));
  const auto result = predictor.Predict(
    Sophus::SO3f(), Eigen::Vector3f::Zero(), 1.1, 1.3);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(
    result.torque_coverage.status, orbslam3_ros2::ActuationCoverageStatus::Full);
  EXPECT_EQ(result.torque_samples_used, 2U);
  EXPECT_NEAR(result.angular_velocity_body.x(), 0.2f, 1e-6f);
}

TEST(BodyTorqueDynamicPredictor, EmptyHistoryReportsEmptyCoverage)
{
  orbslam3_ros2::BodyTorqueDynamicPredictor predictor(Eigen::Matrix3f::Identity());
  const auto coverage = predictor.CoverInterval(1.0, 1.2);
  EXPECT_EQ(coverage.status, orbslam3_ros2::ActuationCoverageStatus::Empty);
}

TEST(BodyTorqueDynamicPredictor, TrimsBoundedTorqueHistory)
{
  orbslam3_ros2::BodyTorqueDynamicPredictor predictor(Eigen::Matrix3f::Identity(), 0.1);
  predictor.AddTorque(0.0, Eigen::Vector3f::Zero());
  predictor.AddTorque(0.05, Eigen::Vector3f::Zero());
  predictor.AddTorque(0.20, Eigen::Vector3f::Zero());
  EXPECT_EQ(predictor.torque_buffer_size(), 2U);
  const auto coverage = predictor.CoverInterval(0.15, 0.20);
  EXPECT_EQ(coverage.status, orbslam3_ros2::ActuationCoverageStatus::Full);
  EXPECT_NEAR(coverage.oldest_stamp_sec, 0.05, 1e-9);
}

TEST(BodyTorqueDynamicPredictor, KeepsColdStartPredecessorAcrossLongIdle)
{
  orbslam3_ros2::BodyTorqueDynamicPredictor predictor(Eigen::Matrix3f::Identity(), 0.5);
  predictor.AddTorque(0.0, Eigen::Vector3f::Zero());
  predictor.AddTorque(32.0, Eigen::Vector3f(1.0f, 0.0f, 0.0f));
  const auto result = predictor.Predict(
    Sophus::SO3f(), Eigen::Vector3f::Zero(), 31.93, 32.02);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.torque_coverage.status, orbslam3_ros2::ActuationCoverageStatus::Full);
  EXPECT_EQ(result.torque_samples_used, 2U);
  EXPECT_NEAR(result.angular_velocity_body.x(), 0.02f, 1e-5f);
}

TEST(BodyTorqueDynamicPredictor, AdvancesPredecessorWithoutGrowingHistory)
{
  orbslam3_ros2::BodyTorqueDynamicPredictor predictor(Eigen::Matrix3f::Identity(), 0.5);
  predictor.AddTorque(0.0, Eigen::Vector3f::Zero());
  predictor.AddTorque(1.0, Eigen::Vector3f::UnitX());
  predictor.AddTorque(1.4, 2.0f * Eigen::Vector3f::UnitX());
  predictor.AddTorque(1.8, 3.0f * Eigen::Vector3f::UnitX());
  EXPECT_EQ(predictor.torque_buffer_size(), 3U);
  const auto coverage = predictor.CoverInterval(1.35, 1.8);
  EXPECT_EQ(coverage.status, orbslam3_ros2::ActuationCoverageStatus::Full);
  EXPECT_NEAR(coverage.oldest_stamp_sec, 1.0, 1e-9);
}

TEST(BodyTorqueDynamicPredictor, ConvertsWorldOmegaToBodyAndBack)
{
  const Sophus::SO3f orientation = Sophus::SO3f::exp(
    Eigen::Vector3f(0.0f, 0.0f, static_cast<float>(M_PI_2)));
  orbslam3_ros2::BodyTorqueDynamicPredictor predictor(Eigen::Matrix3f::Identity());
  predictor.AddTorque(0.0, Eigen::Vector3f::Zero());
  const auto result = predictor.Predict(
    orientation, Eigen::Vector3f(0.0f, 1.0f, 0.0f), 0.0, 0.01);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.angular_velocity_body.x(), 1.0f, 1e-5f);
  EXPECT_NEAR(result.angular_velocity_world.y(), 1.0f, 1e-5f);
}

TEST(BodyTorqueDynamicPredictor, CompositeInertiaHandlesIrregularOrbTimingTrace)
{
  orbslam3_ros2::BodyTorqueDynamicPredictor predictor;
  Eigen::Matrix3f inertia = Eigen::Matrix3f::Zero();
  inertia.diagonal() << 0.00803107F, 0.00803107F, 0.015805F;
  predictor.SetInertia(inertia);

  const std::vector<double> periods{0.049, 0.050, 0.120, 0.051, 0.049, 0.120};
  double stamp = 1.0;
  for (const double period : periods) {
    predictor.AddTorque(stamp, Eigen::Vector3f(0.001F, 0.0F, 0.0F));
    stamp += period;
  }
  predictor.AddTorque(stamp, Eigen::Vector3f(0.001F, 0.0F, 0.0F));

  const auto result = predictor.Predict(
    Sophus::SO3f(), Eigen::Vector3f::Zero(), 1.0, stamp);
  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.missing_torque_interval);
  EXPECT_NEAR(result.angular_velocity_body.x(),
    static_cast<float>((stamp - 1.0) * 0.001 / 0.00803107), 1e-5F);
  EXPECT_NEAR(result.angular_velocity_body.y(), 0.0F, 1e-6F);
  EXPECT_NEAR(result.angular_velocity_body.z(), 0.0F, 1e-6F);
}

TEST(BodyTorqueDynamicPredictor, PropagatesRetrospectiveGtTkWithoutUsingGtNow)
{
  Eigen::Matrix3f inertia = Eigen::Matrix3f::Zero();
  inertia.diagonal() << 0.00803107F, 0.00803107F, 0.015805F;
  orbslam3_ros2::BodyTorqueDynamicPredictor predictor(inertia);
  predictor.AddTorque(10.0, Eigen::Vector3f(0.001F, 0.0F, 0.0F));
  predictor.AddTorque(10.12, Eigen::Vector3f(0.001F, 0.0F, 0.0F));

  const Eigen::Vector3f omega_gt_tk(0.02F, 0.0F, 0.0F);
  const Eigen::Vector3f omega_gt_now(0.50F, 0.0F, 0.0F);
  const auto result = predictor.Predict(Sophus::SO3f(), omega_gt_tk, 10.0, 10.12);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(
    result.angular_velocity_world.x(),
    omega_gt_tk.x() + 0.12F * 0.001F / 0.00803107F, 1e-5F);
  EXPECT_GT((result.angular_velocity_world - omega_gt_now).norm(), 0.1F);
}

TEST(BodyThrustDynamicPredictor, HoverThrustCancelsGravity)
{
  orbslam3_ros2::BodyTorqueDynamicPredictor angular;
  angular.AddTorque(1.0, Eigen::Vector3f::Zero());
  orbslam3_ros2::BodyThrustDynamicPredictor predictor(1.4f);
  predictor.AddThrust(1.0, 1.4f * 9.81f);
  const auto result = predictor.Predict(
    Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Sophus::SO3f(),
    Eigen::Vector3f::Zero(), 1.0, 1.1, angular);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.acceleration_world.norm(), 0.0f, 1e-5f);
  EXPECT_NEAR(result.position_world.norm(), 0.0f, 1e-5f);
}

TEST(EpochGravityState, AlignedFramesKeepGravityOnNegativeZ)
{
  orbslam3_ros2::EpochGravityState gravity;
  ASSERT_TRUE(gravity.Initialize(0, Sophus::SO3f()));
  EXPECT_NEAR((gravity.gravity_o() - Eigen::Vector3f(0.0f, 0.0f, -9.81f)).norm(), 0.0f, 1e-6f);
}

TEST(EpochGravityState, RotatesWorldGravityIntoLocalFrameAndPreservesNorm)
{
  orbslam3_ros2::EpochGravityState gravity;
  const Sophus::SO3f o_r_world = Sophus::SO3f::exp(
    Eigen::Vector3f(0.5f * static_cast<float>(M_PI), 0.0f, 0.0f));
  ASSERT_TRUE(gravity.Initialize(4, o_r_world));
  const Eigen::Vector3f expected = o_r_world * Eigen::Vector3f(0.0f, 0.0f, -9.81f);
  EXPECT_NEAR((gravity.gravity_o() - expected).norm(), 0.0f, 1e-5f);
  EXPECT_NEAR(gravity.gravity_o().norm(), 9.81f, 1e-5f);
}

TEST(EpochGravityState, UsesOFromWorldRatherThanItsInverse)
{
  orbslam3_ros2::EpochGravityState gravity;
  const Sophus::SO3f o_r_world = Sophus::SO3f::exp(Eigen::Vector3f(0.4f, -0.7f, 0.2f));
  ASSERT_TRUE(gravity.Initialize(2, o_r_world));
  const Eigen::Vector3f g_world(0.0f, 0.0f, -9.81f);
  EXPECT_NEAR((gravity.gravity_o() - o_r_world * g_world).norm(), 0.0f, 1e-5f);
  EXPECT_GT((gravity.gravity_o() - o_r_world.inverse() * g_world).norm(), 1.0f);
}

TEST(EpochGravityState, FreezesFirstAuthoritativeRotationWithinEpoch)
{
  orbslam3_ros2::EpochGravityState gravity;
  const Sophus::SO3f first = Sophus::SO3f::exp(Eigen::Vector3f(0.3f, 0.0f, 0.0f));
  const Sophus::SO3f revision = Sophus::SO3f::exp(Eigen::Vector3f(-0.6f, 0.2f, 0.0f));
  ASSERT_TRUE(gravity.Initialize(7, first));
  const Eigen::Vector3f frozen = gravity.gravity_o();
  EXPECT_FALSE(gravity.Initialize(7, revision));
  EXPECT_NEAR((gravity.gravity_o() - frozen).norm(), 0.0f, 1e-6f);
}

TEST(EpochGravityState, NewEpochInvalidatesUntilNewAnchor)
{
  orbslam3_ros2::EpochGravityState gravity;
  ASSERT_TRUE(gravity.Initialize(0, Sophus::SO3f()));
  gravity.ObserveEpoch(1);
  EXPECT_FALSE(gravity.valid());
  EXPECT_EQ(gravity.map_epoch(), 1U);
  EXPECT_NEAR(gravity.gravity_o().norm(), 0.0f, 1e-6f);
}

TEST(BodyThrustDynamicPredictor, RotatedLocalFrameHoverCancelsGravity)
{
  constexpr float kMass = 1.4f;
  const Sophus::SO3f o_r_world = Sophus::SO3f::exp(
    Eigen::Vector3f(0.5f * static_cast<float>(M_PI), 0.0f, 0.0f));
  const Eigen::Vector3f gravity_o =
    o_r_world * Eigen::Vector3f(0.0f, 0.0f, -9.81f);
  orbslam3_ros2::BodyTorqueDynamicPredictor angular;
  angular.AddTorque(1.0, Eigen::Vector3f::Zero());
  orbslam3_ros2::BodyThrustDynamicPredictor predictor(kMass, gravity_o);
  predictor.AddThrust(1.0, kMass * 9.81f);
  const auto result = predictor.Predict(
    Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), o_r_world,
    Eigen::Vector3f::Zero(), 1.0, 1.1, angular);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.acceleration_world.norm(), 0.0f, 1e-5f);
  EXPECT_NEAR(result.linear_velocity_world.norm(), 0.0f, 1e-5f);
}

TEST(BodyThrustDynamicPredictor, ZeroThrustFallsWithGravity)
{
  orbslam3_ros2::BodyTorqueDynamicPredictor angular;
  angular.AddTorque(1.0, Eigen::Vector3f::Zero());
  orbslam3_ros2::BodyThrustDynamicPredictor predictor(1.4f);
  predictor.AddThrust(1.0, 0.0f);
  const auto result = predictor.Predict(
    Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Sophus::SO3f(),
    Eigen::Vector3f::Zero(), 1.0, 1.1, angular);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.linear_velocity_world.z(), -0.981f, 1e-5f);
  EXPECT_NEAR(result.position_world.z(), -0.04905f, 1e-5f);
}

TEST(BodyThrustDynamicPredictor, ExcessThrustAcceleratesUpward)
{
  orbslam3_ros2::BodyTorqueDynamicPredictor angular;
  angular.AddTorque(1.0, Eigen::Vector3f::Zero());
  orbslam3_ros2::BodyThrustDynamicPredictor predictor(1.4f);
  predictor.AddThrust(1.0, 2.0f * 1.4f * 9.81f);
  const auto result = predictor.Predict(
    Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Sophus::SO3f(),
    Eigen::Vector3f::Zero(), 1.0, 1.1, angular);
  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.linear_velocity_world.z(), 0.0f);
}

TEST(BodyThrustDynamicPredictor, TiltedBodyThrustProducesWorldHorizontalAcceleration)
{
  orbslam3_ros2::BodyTorqueDynamicPredictor angular;
  angular.AddTorque(1.0, Eigen::Vector3f::Zero());
  orbslam3_ros2::BodyThrustDynamicPredictor predictor(1.0f);
  predictor.AddThrust(1.0, 9.81f);
  const Sophus::SO3f tilted = Sophus::SO3f::exp(Eigen::Vector3f(0.0f, 0.2f, 0.0f));
  const auto result = predictor.Predict(
    Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), tilted,
    Eigen::Vector3f::Zero(), 1.0, 1.1, angular);
  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.acceleration_world.x(), 0.0f);
}

TEST(BodyThrustDynamicPredictor, UsesIrregularForceTimestampsAndChanges)
{
  orbslam3_ros2::BodyTorqueDynamicPredictor angular;
  angular.AddTorque(1.0, Eigen::Vector3f::Zero());
  orbslam3_ros2::BodyThrustDynamicPredictor predictor(1.0f);
  predictor.AddThrust(1.0, 9.81f);
  predictor.AddThrust(1.03, 19.62f);
  const auto result = predictor.Predict(
    Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Sophus::SO3f(),
    Eigen::Vector3f::Zero(), 1.0, 1.10, angular);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.integration_steps, 2U);
  EXPECT_EQ(result.force_samples_used, 2U);
  EXPECT_NEAR(result.linear_velocity_world.z(), 9.81f * 0.07f, 1e-5f);
}

TEST(BodyThrustDynamicPredictor, ColdStartZeroSeedUsesZohUntilFirstCommand)
{
  orbslam3_ros2::BodyTorqueDynamicPredictor angular;
  angular.AddTorque(1.0, Eigen::Vector3f::Zero());
  orbslam3_ros2::BodyThrustDynamicPredictor predictor(1.0f);
  predictor.AddThrust(1.0, 0.0f);
  predictor.AddThrust(1.2, 9.81f);
  const auto result = predictor.Predict(
    Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Sophus::SO3f(),
    Eigen::Vector3f::Zero(), 1.1, 1.3, angular);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(
    result.thrust_coverage.status, orbslam3_ros2::ActuationCoverageStatus::Full);
  EXPECT_EQ(result.force_samples_used, 2U);
  EXPECT_LT(result.linear_velocity_world.z(), 0.0f);
}

TEST(BodyThrustDynamicPredictor, MissingPrefixDoesNotAssumeZero)
{
  orbslam3_ros2::BodyTorqueDynamicPredictor angular;
  angular.AddTorque(1.0, Eigen::Vector3f::Zero());
  orbslam3_ros2::BodyThrustDynamicPredictor predictor(1.0f);
  predictor.AddThrust(1.1, 9.81f);
  const auto result = predictor.Predict(
    Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Sophus::SO3f(),
    Eigen::Vector3f::Zero(), 1.0, 1.2, angular);
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.missing_force_interval);
  EXPECT_EQ(
    result.thrust_coverage.status,
    orbslam3_ros2::ActuationCoverageStatus::MissingPrefix);
  EXPECT_NEAR(result.thrust_coverage.missing_prefix_sec, 0.1, 1e-9);
}

TEST(BodyThrustDynamicPredictor, KeepsColdStartPredecessorAcrossLongIdle)
{
  orbslam3_ros2::BodyTorqueDynamicPredictor angular(Eigen::Matrix3f::Identity(), 40.0);
  angular.AddTorque(0.0, Eigen::Vector3f::Zero());
  orbslam3_ros2::BodyThrustDynamicPredictor predictor(1.0f, Eigen::Vector3f::Zero(), 0.5);
  predictor.AddThrust(0.0, 0.0f);
  predictor.AddThrust(32.0, 10.0f);
  const auto result = predictor.Predict(
    Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Sophus::SO3f(),
    Eigen::Vector3f::Zero(), 31.93, 32.02, angular);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.thrust_coverage.status, orbslam3_ros2::ActuationCoverageStatus::Full);
  EXPECT_EQ(result.force_samples_used, 2U);
  EXPECT_NEAR(result.linear_velocity_world.z(), 0.2f, 1e-5f);
}

TEST(BodyThrustDynamicPredictor, AdvancesPredecessorAndKeepsRecentChanges)
{
  orbslam3_ros2::BodyThrustDynamicPredictor predictor(1.0f, Eigen::Vector3f::Zero(), 0.5);
  predictor.AddThrust(0.0, 0.0f);
  predictor.AddThrust(1.0, 1.0f);
  predictor.AddThrust(1.4, 2.0f);
  predictor.AddThrust(1.8, 3.0f);
  EXPECT_EQ(predictor.thrust_buffer_size(), 3U);
  const auto coverage = predictor.CoverInterval(1.35, 1.8);
  EXPECT_EQ(coverage.status, orbslam3_ros2::ActuationCoverageStatus::Full);
  EXPECT_NEAR(coverage.oldest_stamp_sec, 1.0, 1e-9);
}

TEST(BodyThrustDynamicPredictor, PositionAndVelocityShareTargetTimestamp)
{
  orbslam3_ros2::BodyTorqueDynamicPredictor angular;
  angular.AddTorque(2.0, Eigen::Vector3f::Zero());
  orbslam3_ros2::BodyThrustDynamicPredictor predictor(1.0f);
  predictor.AddThrust(2.0, 0.0f);
  const auto result = predictor.Predict(
    Eigen::Vector3f(1.0f, 2.0f, 3.0f), Eigen::Vector3f(2.0f, 0.0f, 0.0f),
    Sophus::SO3f(), Eigen::Vector3f::Zero(), 2.0, 2.2, angular);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.position_world.x(), 1.4f, 1e-5f);
  EXPECT_NEAR(result.linear_velocity_world.z(), -1.962f, 1e-5f);
  EXPECT_NEAR(result.position_world.z(), 3.0f - 0.1962f, 1e-5f);
}

TEST(BodyThrustDynamicPredictor, MassChangesAccelerationUsingSharedPhysicalValue)
{
  orbslam3_ros2::BodyTorqueDynamicPredictor angular;
  angular.AddTorque(1.0, Eigen::Vector3f::Zero());
  orbslam3_ros2::BodyThrustDynamicPredictor predictor(1.4f);
  predictor.AddThrust(1.0, 1.4f * 9.81f);
  const auto shared_mass = predictor.Predict(
    Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Sophus::SO3f(),
    Eigen::Vector3f::Zero(), 1.0, 1.1, angular);
  predictor.SetMass(2.8f);
  const auto doubled_mass = predictor.Predict(
    Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Sophus::SO3f(),
    Eigen::Vector3f::Zero(), 1.0, 1.1, angular);
  ASSERT_TRUE(shared_mass.valid);
  ASSERT_TRUE(doubled_mass.valid);
  EXPECT_NEAR(shared_mass.acceleration_world.z(), 0.0f, 1e-5f);
  EXPECT_LT(doubled_mass.acceleration_world.z(), 0.0f);
}

TEST(CausalLinearVelocityEstimator, RecoversConstantVelocityAtSampleTime)
{
  orbslam3_ros2::CausalLinearVelocityEstimator estimator(0.2, 0.3, 20.0f, 20.0f);
  estimator.AddSample(Eigen::Vector3f(0.0f, 0.0f, 0.0f), 1.0, 1);
  estimator.AddSample(Eigen::Vector3f(0.1f, -0.2f, 0.05f), 1.1, 1);
  const auto result = estimator.AddSample(Eigen::Vector3f(0.2f, -0.4f, 0.1f), 1.2, 1);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.mode, orbslam3_ros2::LinearVelocityEstimatorMode::ThreeSamplePredicted);
  EXPECT_NEAR((result.velocity_at_sample - Eigen::Vector3f(1.0f, -2.0f, 0.5f)).norm(), 0.0f, 1e-5f);
}

TEST(CausalLinearVelocityEstimator, ProjectsConstantAccelerationCloserThanMidpointVelocity)
{
  orbslam3_ros2::CausalLinearVelocityEstimator estimator(0.2, 0.3, 20.0f, 20.0f);
  const auto position = [](float t) {return Eigen::Vector3f(t + t * t, 0.0f, 0.0f);};
  estimator.AddSample(position(0.0f), 1.0, 1);
  estimator.AddSample(position(0.1f), 1.1, 1);
  const auto result = estimator.AddSample(position(0.2f), 1.2, 1);
  const float true_velocity = 1.4f;
  ASSERT_TRUE(result.valid);
  EXPECT_LT(
    std::abs(result.velocity_at_sample.x() - true_velocity),
    std::abs(result.current_mid_velocity.x() - true_velocity));
  EXPECT_NEAR(result.velocity_at_sample.x(), true_velocity, 1e-4f);
}

TEST(CausalLinearVelocityEstimator, ChangesVelocitySignWithoutLargeOvershoot)
{
  orbslam3_ros2::CausalLinearVelocityEstimator estimator(0.2, 0.3, 20.0f, 20.0f);
  estimator.AddSample(Eigen::Vector3f(0.0f, 0.0f, 0.0f), 1.0, 1);
  estimator.AddSample(Eigen::Vector3f(0.05f, 0.0f, 0.0f), 1.1, 1);
  const auto result = estimator.AddSample(Eigen::Vector3f(0.0f, 0.0f, 0.0f), 1.2, 1);
  ASSERT_TRUE(result.valid);
  EXPECT_LT(result.velocity_at_sample.x(), 0.0f);
  EXPECT_NEAR(result.velocity_at_sample.x(), -1.0f, 1e-4f);
}

TEST(CausalLinearVelocityEstimator, UsesIrregularTimestamps)
{
  orbslam3_ros2::CausalLinearVelocityEstimator estimator(0.2, 0.3, 20.0f, 20.0f);
  const auto position = [](float t) {return Eigen::Vector3f(0.5f * t * t, 0.0f, 0.0f);};
  estimator.AddSample(position(0.0f), 2.0, 1);
  estimator.AddSample(position(0.06f), 2.06, 1);
  const auto result = estimator.AddSample(position(0.17f), 2.17, 1);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.velocity_at_sample.x(), 0.17f, 1e-4f);
}

TEST(CausalLinearVelocityEstimator, DegradedDtFallsBackToTwoSamples)
{
  orbslam3_ros2::CausalLinearVelocityEstimator estimator(0.075, 0.20, 20.0f, 20.0f);
  estimator.AddSample(Eigen::Vector3f(0.0f, 0.0f, 0.0f), 1.0, 1);
  estimator.AddSample(Eigen::Vector3f(0.05f, 0.0f, 0.0f), 1.05, 1);
  const auto result = estimator.AddSample(Eigen::Vector3f(0.17f, 0.0f, 0.0f), 1.17, 1);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.mode, orbslam3_ros2::LinearVelocityEstimatorMode::DegradedDt);
  EXPECT_NEAR(result.acceleration.norm(), 0.0f, 1e-6f);
  EXPECT_NEAR(result.velocity_at_sample.x(), 1.0f, 1e-5f);
}

TEST(CausalLinearVelocityEstimator, RejectedSampleDoesNotEnterHistory)
{
  orbslam3_ros2::CausalLinearVelocityEstimator estimator(0.2, 0.3, 20.0f, 20.0f);
  estimator.AddSample(Eigen::Vector3f::Zero(), 1.0, 1);
  const auto rejected = estimator.AddSample(Eigen::Vector3f(10.0f, 0.0f, 0.0f), 1.05, 1, false);
  EXPECT_FALSE(rejected.valid);
  EXPECT_EQ(estimator.sample_count(), 1U);
  const auto result = estimator.AddSample(Eigen::Vector3f(0.1f, 0.0f, 0.0f), 1.1, 1);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.velocity_at_sample.x(), 1.0f, 1e-5f);
}

TEST(CausalLinearVelocityEstimator, EpochChangeResetsHistory)
{
  orbslam3_ros2::CausalLinearVelocityEstimator estimator;
  estimator.AddSample(Eigen::Vector3f::Zero(), 1.0, 1);
  estimator.AddSample(Eigen::Vector3f(0.1f, 0.0f, 0.0f), 1.05, 1);
  const auto result = estimator.AddSample(Eigen::Vector3f(5.0f, 0.0f, 0.0f), 1.10, 2);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.mode, orbslam3_ros2::LinearVelocityEstimatorMode::Init);
  EXPECT_EQ(estimator.sample_count(), 1U);
}

TEST(CausalLinearVelocityEstimator, RejectedPositionCorrectionDoesNotCreateVelocityImpulse)
{
  orbslam3_ros2::CausalLinearVelocityEstimator estimator(0.2, 0.3, 20.0f, 20.0f);
  estimator.AddSample(Eigen::Vector3f::Zero(), 1.0, 1);
  estimator.AddSample(Eigen::Vector3f(0.1f, 0.0f, 0.0f), 1.1, 1);
  estimator.AddSample(Eigen::Vector3f(10.0f, 0.0f, 0.0f), 1.15, 1, false);
  const auto result = estimator.AddSample(Eigen::Vector3f(0.2f, 0.0f, 0.0f), 1.2, 1);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.velocity_at_sample.x(), 1.0f, 1e-5f);
}

orbslam3_ros2::CausalLinearVelocityEstimate MakeMidpointLinearEstimate(
  const Eigen::Vector3f & velocity_mid, double previous_stamp, double current_stamp)
{
  orbslam3_ros2::CausalLinearVelocityEstimate estimate;
  estimate.valid = true;
  estimate.sample_accepted = true;
  estimate.current_mid_velocity = velocity_mid;
  estimate.current_mid_stamp_sec = 0.5 * (previous_stamp + current_stamp);
  estimate.p_k1 = Eigen::Vector3f::Zero();
  estimate.p_k = velocity_mid * static_cast<float>(current_stamp - previous_stamp);
  return estimate;
}

TEST(MidpointDynamicVelocity, ConstantVelocityMatchesTwoSample)
{
  orbslam3_ros2::BodyTorqueDynamicPredictor angular;
  orbslam3_ros2::BodyThrustDynamicPredictor thrust(1.0f);
  angular.AddTorque(9.0, Eigen::Vector3f::Zero());
  thrust.AddThrust(9.0, 9.81f);
  const auto estimate = MakeMidpointLinearEstimate(Eigen::Vector3f(1.0f, -0.2f, 0.1f), 1.0, 1.1);
  const auto result = orbslam3_ros2::PredictMidpointDynamicVelocity(
    estimate, Sophus::SO3f(), Sophus::SO3f(), 1.0, 1.1, 10.0, angular, thrust);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR((result.velocity_at_sample - estimate.current_mid_velocity).norm(), 0.0f, 1e-5f);
}

TEST(MidpointDynamicVelocity, ConstantAccelerationImprovesSampleVelocity)
{
  orbslam3_ros2::BodyTorqueDynamicPredictor angular;
  orbslam3_ros2::BodyThrustDynamicPredictor thrust(1.0f, Eigen::Vector3f::Zero());
  angular.AddTorque(9.0, Eigen::Vector3f::Zero());
  thrust.AddThrust(9.0, 2.0f);
  const auto estimate = MakeMidpointLinearEstimate(Eigen::Vector3f::Zero(), 1.0, 1.2);
  const auto result = orbslam3_ros2::PredictMidpointDynamicVelocity(
    estimate, Sophus::SO3f(), Sophus::SO3f(), 1.0, 1.2, 10.0, angular, thrust);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.velocity_at_sample.z(), 0.2f, 1e-5f);
  EXPECT_LT(std::abs(result.velocity_at_sample.z() - 0.2f), std::abs(0.0f - 0.2f));
}

TEST(MidpointDynamicVelocity, RotatedFrameHoverKeepsVelocity)
{
  const Sophus::SO3f o_r_world = Sophus::SO3f::exp(
    Eigen::Vector3f(0.5f * static_cast<float>(M_PI), 0.0f, 0.0f));
  const Eigen::Vector3f gravity_o = o_r_world * Eigen::Vector3f(0.0f, 0.0f, -9.81f);
  orbslam3_ros2::BodyTorqueDynamicPredictor angular;
  orbslam3_ros2::BodyThrustDynamicPredictor thrust(1.0f, gravity_o);
  angular.AddTorque(9.0, Eigen::Vector3f::Zero());
  thrust.AddThrust(9.0, 9.81f);
  const auto estimate = MakeMidpointLinearEstimate(Eigen::Vector3f(0.2f, 0.0f, 0.0f), 1.0, 1.1);
  const auto result = orbslam3_ros2::PredictMidpointDynamicVelocity(
    estimate, o_r_world, o_r_world, 1.0, 1.1, 10.0, angular, thrust);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR((result.velocity_at_sample - estimate.current_mid_velocity).norm(), 0.0f, 1e-5f);
}

TEST(MidpointDynamicVelocity, UsesThrustChangesWithZoh)
{
  orbslam3_ros2::BodyTorqueDynamicPredictor angular;
  orbslam3_ros2::BodyThrustDynamicPredictor thrust(1.0f, Eigen::Vector3f::Zero());
  angular.AddTorque(9.0, Eigen::Vector3f::Zero());
  thrust.AddThrust(9.0, 0.0f);
  thrust.AddThrust(9.95, 2.0f);
  const auto estimate = MakeMidpointLinearEstimate(Eigen::Vector3f::Zero(), 1.0, 1.2);
  const auto result = orbslam3_ros2::PredictMidpointDynamicVelocity(
    estimate, Sophus::SO3f(), Sophus::SO3f(), 1.0, 1.2, 10.0, angular, thrust);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.thrust_samples_used, 2U);
  EXPECT_NEAR(result.velocity_at_sample.z(), 0.1f, 1e-5f);
}

TEST(MidpointDynamicVelocity, PropagatesDynamicOrientation)
{
  Eigen::Matrix3f inertia = Eigen::Matrix3f::Identity();
  orbslam3_ros2::BodyTorqueDynamicPredictor angular(inertia);
  orbslam3_ros2::BodyThrustDynamicPredictor thrust(1.0f, Eigen::Vector3f::Zero());
  angular.AddTorque(9.0, Eigen::Vector3f(0.0f, 1.0f, 0.0f));
  thrust.AddThrust(9.0, 1.0f);
  const auto estimate = MakeMidpointLinearEstimate(Eigen::Vector3f::Zero(), 1.0, 1.2);
  const auto result = orbslam3_ros2::PredictMidpointDynamicVelocity(
    estimate, Sophus::SO3f(), Sophus::SO3f(), 1.0, 1.2, 10.0, angular, thrust);
  ASSERT_TRUE(result.valid);
  EXPECT_GT((result.orientation_at_sample * result.orientation_mid.inverse()).log().norm(), 0.0f);
  EXPECT_GT(std::abs(result.velocity_at_sample.x()), 0.0f);
}

TEST(MidpointDynamicVelocity, IncompleteCoverageIsInvalid)
{
  orbslam3_ros2::BodyTorqueDynamicPredictor angular;
  orbslam3_ros2::BodyThrustDynamicPredictor thrust(1.0f);
  angular.AddTorque(9.96, Eigen::Vector3f::Zero());
  thrust.AddThrust(9.96, 9.81f);
  const auto estimate = MakeMidpointLinearEstimate(Eigen::Vector3f::Zero(), 1.0, 1.1);
  const auto result = orbslam3_ros2::PredictMidpointDynamicVelocity(
    estimate, Sophus::SO3f(), Sophus::SO3f(), 1.0, 1.1, 10.0, angular, thrust);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.torque_coverage.status, orbslam3_ros2::ActuationCoverageStatus::MissingPrefix);
}

TEST(MidpointDynamicVelocity, UsesIrregularImageDtAndMappedRosHorizon)
{
  orbslam3_ros2::BodyTorqueDynamicPredictor angular;
  orbslam3_ros2::BodyThrustDynamicPredictor thrust(1.0f, Eigen::Vector3f::Zero());
  angular.AddTorque(9.0, Eigen::Vector3f::Zero());
  thrust.AddThrust(9.0, 1.0f);
  const auto estimate = MakeMidpointLinearEstimate(Eigen::Vector3f::Zero(), 1.0, 1.14);
  const auto result = orbslam3_ros2::PredictMidpointDynamicVelocity(
    estimate, Sophus::SO3f(), Sophus::SO3f(), 1.0, 1.14, 10.0, angular, thrust);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.horizon_sec, 0.07, 1e-9);
  EXPECT_NEAR(result.ros_mid_stamp_sec, 9.93, 1e-9);
  EXPECT_NEAR(result.velocity_at_sample.z(), 0.07f, 1e-5f);
}

TEST(MidpointDynamicVelocity, DoesNotDependOnVisualAcceleration)
{
  orbslam3_ros2::BodyTorqueDynamicPredictor angular;
  orbslam3_ros2::BodyThrustDynamicPredictor thrust(1.0f);
  angular.AddTorque(9.0, Eigen::Vector3f::Zero());
  thrust.AddThrust(9.0, 9.81f);
  auto first = MakeMidpointLinearEstimate(Eigen::Vector3f(0.3f, 0.0f, 0.0f), 1.0, 1.1);
  auto second = first;
  first.acceleration = Eigen::Vector3f(100.0f, -50.0f, 25.0f);
  second.acceleration = Eigen::Vector3f(-100.0f, 50.0f, -25.0f);
  const auto a = orbslam3_ros2::PredictMidpointDynamicVelocity(
    first, Sophus::SO3f(), Sophus::SO3f(), 1.0, 1.1, 10.0, angular, thrust);
  const auto b = orbslam3_ros2::PredictMidpointDynamicVelocity(
    second, Sophus::SO3f(), Sophus::SO3f(), 1.0, 1.1, 10.0, angular, thrust);
  ASSERT_TRUE(a.valid);
  ASSERT_TRUE(b.valid);
  EXPECT_NEAR((a.velocity_at_sample - b.velocity_at_sample).norm(), 0.0f, 1e-6f);
}

}  // namespace
