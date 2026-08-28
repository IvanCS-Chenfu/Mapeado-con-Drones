#include <gtest/gtest.h>

#include "dron_individual/navigation_goal_policy.hpp"

using dron_individual::EvaluateNavigationGoal;
using dron_individual::NavigationGoalDecision;
using dron_individual::NavigationGoalState;

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
