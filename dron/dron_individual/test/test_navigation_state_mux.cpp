#include <gtest/gtest.h>

#include "dron_individual/navigation_state_mux.hpp"

using dron_individual::ContinuousSourcePose;
using dron_individual::DecideNavigationSource;
using dron_individual::DiagnosticGtControlAlignment;
using dron_individual::DiagnosticOrbControlMode;
using dron_individual::EpochAnchorLatch;
using dron_individual::FallbackReason;
using dron_individual::GoalSourceLock;
using dron_individual::NavigationSource;
using dron_individual::OrbTransitionQualifier;
using dron_individual::OrbShadowActivationGate;
using dron_individual::RigidPose;
using dron_individual::ParseDiagnosticOrbControlMode;
using dron_individual::UsesGtPosition;
using dron_individual::UsesGtVelocity;

TEST(NavigationStateMux, DistinguishesEveryFallbackReason)
{
  EXPECT_EQ(
    DecideNavigationSource(true, false, false).fallback_reason,
    FallbackReason::STARTUP_UNANCHORED_ABSOLUTE);
  EXPECT_EQ(
    DecideNavigationSource(false, true, true).fallback_reason,
    FallbackReason::TRACKING_LOST);
  EXPECT_EQ(
    DecideNavigationSource(true, false, true).fallback_reason,
    FallbackReason::NEW_EPOCH_UNANCHORED);
  EXPECT_EQ(
    DecideNavigationSource(true, true, true).source,
    NavigationSource::ORB);
}

TEST(NavigationStateMux, AnchorIsLatchedUntilEpochChanges)
{
  EpochAnchorLatch latch;
  EXPECT_FALSE(latch.Update(1, false));
  EXPECT_TRUE(latch.Update(1, true));
  EXPECT_TRUE(latch.Update(1, false));
  EXPECT_FALSE(latch.Update(2, false));
  EXPECT_TRUE(latch.previously_anchored());
}

TEST(NavigationStateMux, SourceTransitionPreservesControlPose)
{
  ContinuousSourcePose mux;
  RigidPose gt;
  gt.translation = Eigen::Vector3d(3.0, -2.0, 1.0);
  const auto first = mux.Update(NavigationSource::GT_FALLBACK, gt);

  RigidPose orb;
  orb.translation = Eigen::Vector3d(0.2, 0.4, 0.1);
  const auto switched = mux.Update(NavigationSource::ORB, orb);
  EXPECT_NEAR((switched.translation - first.translation).norm(), 0.0, 1e-12);
}

TEST(NavigationStateMux, FallbackPreservesActiveGoalFrame)
{
  ContinuousSourcePose mux;
  RigidPose gt;
  gt.translation = Eigen::Vector3d(3.0, -2.0, 1.0);
  mux.Update(NavigationSource::GT_FALLBACK, gt);

  RigidPose orb;
  orb.translation = Eigen::Vector3d(0.2, 0.4, 0.1);
  mux.Update(NavigationSource::ORB, orb);
  orb.translation.x() += 1.0;
  const auto before_loss = mux.Update(NavigationSource::ORB, orb);

  gt.translation = Eigen::Vector3d(20.0, 5.0, 1.5);
  const auto fallback = mux.Update(NavigationSource::GT_FALLBACK, gt);
  EXPECT_NEAR((fallback.translation - before_loss.translation).norm(), 0.0, 1e-12);
  gt.translation.x() += 0.5;
  const auto continued = mux.Update(NavigationSource::GT_FALLBACK, gt);
  EXPECT_NEAR(continued.translation.x() - fallback.translation.x(), 0.5, 1e-12);
}

TEST(NavigationStateMux, RotatesSourceVelocityIntoContinuousFrame)
{
  ContinuousSourcePose mux;
  RigidPose gt;
  mux.Update(NavigationSource::GT_FALLBACK, gt);

  RigidPose orb;
  orb.rotation = Eigen::Quaterniond(
    Eigen::AngleAxisd(-0.5 * std::acos(-1.0), Eigen::Vector3d::UnitZ()));
  mux.Update(NavigationSource::ORB, orb);
  const Eigen::Vector3d transformed = mux.RotateVectorFromSource(
    Eigen::Vector3d::UnitX());
  EXPECT_NEAR(transformed.x(), 0.0, 1e-12);
  EXPECT_NEAR(transformed.y(), 1.0, 1e-12);
  EXPECT_NEAR(transformed.z(), 0.0, 1e-12);
}

TEST(NavigationStateMux, OrbRequiresConsecutiveTrackingSamples)
{
  OrbTransitionQualifier qualifier;
  dron_individual::OrbQualificationResult result;
  for (int sample = 0; sample < 5; ++sample) {
    result = qualifier.Update(1, 5);
  }
  EXPECT_TRUE(result.qualified);
  EXPECT_TRUE(result.newly_qualified);
}

TEST(NavigationStateMux, OrbQualificationRestartsAfterResetOrNewEpoch)
{
  OrbTransitionQualifier qualifier;
  qualifier.Update(1, 4);
  qualifier.Update(1, 4);
  qualifier.Reset();
  auto result = qualifier.Update(1, 4);
  EXPECT_FALSE(result.qualified);
  EXPECT_EQ(result.consecutive_samples, 1U);
  result = qualifier.Update(2, 4);
  EXPECT_EQ(result.consecutive_samples, 1U);
}

TEST(NavigationStateMux, GoalStartedInFallbackDefersOrbUntilBoundary)
{
  GoalSourceLock lock;
  lock.Begin(NavigationSource::GT_FALLBACK);
  const auto during_goal = lock.Apply({NavigationSource::ORB, FallbackReason::NONE});
  EXPECT_EQ(during_goal.source, NavigationSource::GT_FALLBACK);
  EXPECT_EQ(during_goal.fallback_reason, FallbackReason::TRAJECTORY_SOURCE_LOCKED);
  lock.End();
  const auto at_boundary = lock.Apply({NavigationSource::ORB, FallbackReason::NONE});
  EXPECT_EQ(at_boundary.source, NavigationSource::ORB);
}

TEST(NavigationStateMux, OrbLossLocksFallbackForRestOfGoal)
{
  GoalSourceLock lock;
  lock.Begin(NavigationSource::ORB);
  const auto lost = lock.Apply(
    {NavigationSource::GT_FALLBACK, FallbackReason::TRACKING_LOST});
  EXPECT_EQ(lost.source, NavigationSource::GT_FALLBACK);
  EXPECT_EQ(lock.locked_source(), NavigationSource::GT_FALLBACK);
  const auto recovered = lock.Apply({NavigationSource::ORB, FallbackReason::NONE});
  EXPECT_EQ(recovered.source, NavigationSource::GT_FALLBACK);
}

TEST(NavigationStateMux, ShadowActivationRequiresContinuousSettlingWindow)
{
  OrbShadowActivationGate gate;
  EXPECT_FALSE(gate.Update(true, 0.10, 0.10, 10.0, 1.5, 0.15, 0.15));
  EXPECT_FALSE(gate.Update(true, 0.10, 0.10, 11.4, 1.5, 0.15, 0.15));
  EXPECT_TRUE(gate.Update(true, 0.10, 0.10, 11.5, 1.5, 0.15, 0.15));
}

TEST(NavigationStateMux, ShadowActivationResetsOnMotionOrInvalidOrb)
{
  OrbShadowActivationGate gate;
  gate.Update(true, 0.10, 0.10, 20.0, 1.5, 0.15, 0.15);
  EXPECT_FALSE(gate.Update(true, 0.20, 0.10, 21.0, 1.5, 0.15, 0.15));
  EXPECT_FALSE(gate.Update(true, 0.10, 0.10, 22.0, 1.5, 0.15, 0.15));
  EXPECT_FALSE(gate.Update(false, 0.0, 0.0, 24.0, 1.5, 0.15, 0.15));
  EXPECT_FALSE(gate.ready());
}

TEST(NavigationStateMux, DiagnosticOverrideModesSelectOnlyRequestedLinearComponents)
{
  EXPECT_TRUE(UsesGtPosition(ParseDiagnosticOrbControlMode("position_gt")));
  EXPECT_FALSE(UsesGtVelocity(ParseDiagnosticOrbControlMode("position_gt")));
  EXPECT_FALSE(UsesGtPosition(ParseDiagnosticOrbControlMode("velocity_gt")));
  EXPECT_TRUE(UsesGtVelocity(ParseDiagnosticOrbControlMode("velocity_gt")));
  EXPECT_TRUE(UsesGtPosition(ParseDiagnosticOrbControlMode("position_velocity_gt")));
  EXPECT_TRUE(UsesGtVelocity(ParseDiagnosticOrbControlMode("position_velocity_gt")));
  EXPECT_EQ(ParseDiagnosticOrbControlMode("normal"), DiagnosticOrbControlMode::NORMAL);
}

TEST(NavigationStateMux, DiagnosticGtAlignmentPreservesHandoffAndMotionInControlFrame)
{
  DiagnosticGtControlAlignment alignment;
  RigidPose control;
  control.translation = Eigen::Vector3d(2.0, -3.0, 1.0);
  RigidPose gt;
  gt.translation = Eigen::Vector3d(10.0, 4.0, 1.0);
  alignment.Capture(control, gt);
  EXPECT_NEAR((alignment.TransformPose(gt).translation - control.translation).norm(), 0.0, 1e-12);

  gt.translation += Eigen::Vector3d(0.5, -0.25, 0.1);
  const auto moved = alignment.TransformPose(gt);
  EXPECT_NEAR(moved.translation.x() - control.translation.x(), 0.5, 1e-12);
  EXPECT_NEAR(moved.translation.y() - control.translation.y(), -0.25, 1e-12);
  EXPECT_NEAR(moved.translation.z() - control.translation.z(), 0.1, 1e-12);
}
