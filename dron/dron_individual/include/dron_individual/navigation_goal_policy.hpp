#ifndef DRON_INDIVIDUAL_NAVIGATION_GOAL_POLICY_HPP_
#define DRON_INDIVIDUAL_NAVIGATION_GOAL_POLICY_HPP_

#include <cstdint>

namespace dron_individual
{

struct NavigationGoalState
{
  bool received = false;
  bool fresh = false;
  bool local_valid = false;
  bool local_continuity_valid = false;
  bool global_valid = false;
  bool absolute_frame_valid = false;
  bool velocity_valid = false;
  bool gt_fallback = false;
  uint64_t map_epoch = 0;
  uint64_t sample_sequence = 0;
};

enum class NavigationGoalDecision
{
  ACCEPT_RELATIVE,
  ACCEPT_ABSOLUTE,
  REJECT_NO_STATE,
  REJECT_STALE_STATE,
  REJECT_LOCAL_INVALID,
  REJECT_LOCAL_DISCONTINUOUS,
  REJECT_ABSOLUTE_DISABLED,
  REJECT_GLOBAL_INVALID
};

inline NavigationGoalDecision EvaluateNavigationGoal(
  bool requests_absolute, const NavigationGoalState & state)
{
  if (!state.received) {
    return NavigationGoalDecision::REJECT_NO_STATE;
  }
  if (!state.fresh) {
    return NavigationGoalDecision::REJECT_STALE_STATE;
  }
  if (!state.local_valid) {
    return NavigationGoalDecision::REJECT_LOCAL_INVALID;
  }
  if (!state.local_continuity_valid) {
    return NavigationGoalDecision::REJECT_LOCAL_DISCONTINUOUS;
  }
  if (!state.velocity_valid) {
    return NavigationGoalDecision::REJECT_LOCAL_INVALID;
  }
  if (requests_absolute && !state.global_valid && !state.gt_fallback &&
    !state.absolute_frame_valid)
  {
    return NavigationGoalDecision::REJECT_GLOBAL_INVALID;
  }
  return requests_absolute
    ? NavigationGoalDecision::ACCEPT_ABSOLUTE
    : NavigationGoalDecision::ACCEPT_RELATIVE;
}

inline const char * NavigationGoalDecisionName(NavigationGoalDecision decision)
{
  switch (decision) {
    case NavigationGoalDecision::ACCEPT_RELATIVE:
      return "accept_relative";
    case NavigationGoalDecision::ACCEPT_ABSOLUTE:
      return "accept_absolute";
    case NavigationGoalDecision::REJECT_NO_STATE:
      return "reject_no_state";
    case NavigationGoalDecision::REJECT_STALE_STATE:
      return "reject_stale_state";
    case NavigationGoalDecision::REJECT_LOCAL_INVALID:
      return "reject_local_invalid";
    case NavigationGoalDecision::REJECT_LOCAL_DISCONTINUOUS:
      return "reject_local_discontinuous";
    case NavigationGoalDecision::REJECT_ABSOLUTE_DISABLED:
      return "reject_absolute_disabled";
    case NavigationGoalDecision::REJECT_GLOBAL_INVALID:
      return "reject_global_invalid";
  }
  return "reject_unknown";
}

}  // namespace dron_individual

#endif  // DRON_INDIVIDUAL_NAVIGATION_GOAL_POLICY_HPP_
