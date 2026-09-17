#include <gtest/gtest.h>

#include "dron_individual/navigation_goal_policy.hpp"
#include "dron_individual/trajectory_yaw.hpp"

using dron_individual::EvaluateNavigationGoal;
using dron_individual::NavigationGoalDecision;
using dron_individual::NavigationGoalState;
using dron_individual::NearestEquivalentYaw;

TEST(TrajectoryYaw, PreservesShortestArcAcrossPositivePi)
{
  const double start = 90.0 * M_PI / 180.0;
  const double requested = 181.0 * M_PI / 180.0;
  const double equivalent = NearestEquivalentYaw(requested, start);

  EXPECT_NEAR(equivalent - start, 91.0 * M_PI / 180.0, 1e-12);
  EXPECT_GT(equivalent, M_PI);
}

TEST(TrajectoryYaw, UsesPreviousWaypointAsContinuityReference)
{
  const double first = NearestEquivalentYaw(179.0 * M_PI / 180.0, 170.0 * M_PI / 180.0);
  const double second = NearestEquivalentYaw(-179.0 * M_PI / 180.0, first);

  EXPECT_NEAR(second - first, 2.0 * M_PI / 180.0, 1e-12);
}

TEST(NavigationGoalPolicy, AcceptsRelativeWithFreshContinuousLocalState)
{
  NavigationGoalState state;
  state.received = true;
  state.fresh = true;
  state.local_valid = true;
  state.local_continuity_valid = true;
  state.velocity_valid = true;
  EXPECT_EQ(
    EvaluateNavigationGoal(false, state),
    NavigationGoalDecision::ACCEPT_RELATIVE);
}

TEST(NavigationGoalPolicy, RejectsAbsoluteWithoutGlobalPose)
{
  NavigationGoalState state;
  state.received = true;
  state.fresh = true;
  state.local_valid = true;
  state.local_continuity_valid = true;
  state.velocity_valid = true;
  EXPECT_EQ(
    EvaluateNavigationGoal(true, state),
    NavigationGoalDecision::REJECT_GLOBAL_INVALID);
}

TEST(NavigationGoalPolicy, RejectsMissingStaleInvalidAndDiscontinuousState)
{
  NavigationGoalState state;
  EXPECT_EQ(
    EvaluateNavigationGoal(false, state),
    NavigationGoalDecision::REJECT_NO_STATE);

  state.received = true;
  EXPECT_EQ(
    EvaluateNavigationGoal(false, state),
    NavigationGoalDecision::REJECT_STALE_STATE);

  state.fresh = true;
  EXPECT_EQ(
    EvaluateNavigationGoal(false, state),
    NavigationGoalDecision::REJECT_LOCAL_INVALID);

  state.local_valid = true;
  EXPECT_EQ(
    EvaluateNavigationGoal(false, state),
    NavigationGoalDecision::REJECT_LOCAL_DISCONTINUOUS);
}

TEST(NavigationGoalPolicy, GtFallbackAcceptsAbsoluteGoal)
{
  NavigationGoalState state;
  state.received = true;
  state.fresh = true;
  state.local_valid = true;
  state.local_continuity_valid = true;
  state.velocity_valid = true;
  state.gt_fallback = true;
  EXPECT_EQ(
    EvaluateNavigationGoal(true, state),
    NavigationGoalDecision::ACCEPT_ABSOLUTE);
}

TEST(NavigationGoalPolicy, CachedAbsoluteFrameAcceptsTransientGlobalGap)
{
  NavigationGoalState state;
  state.received = true;
  state.fresh = true;
  state.local_valid = true;
  state.local_continuity_valid = true;
  state.velocity_valid = true;
  state.absolute_frame_valid = true;
  EXPECT_EQ(
    EvaluateNavigationGoal(true, state),
    NavigationGoalDecision::ACCEPT_ABSOLUTE);
}
