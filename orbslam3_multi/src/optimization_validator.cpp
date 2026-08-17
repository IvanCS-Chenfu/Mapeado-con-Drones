#include "orbslam3_multi/optimization_validator.hpp"

#include "orbslam3_multi/pose_geometry.hpp"

#include <cmath>
#include <map>

namespace orbslam3_multi
{

OptimizationValidator::OptimizationValidator(FiducialOptimizationConfig config)
: config_(config)
{
}

void OptimizationValidator::Configure(const FiducialOptimizationConfig & config)
{
  config_ = config;
}

const char * ToString(ValidationDecision decision)
{
  switch (decision) {
    case ValidationDecision::AcceptFull:
      return "accept_full";
    case ValidationDecision::AcceptPartialRetry:
      return "accept_partial_retry";
    case ValidationDecision::HardFailure:
      return "hard_failure";
  }
  return "unknown";
}

ValidationResult OptimizationValidator::Validate(
  const PoseGraphProblem & problem,
  const OptimizationProposal & proposal) const
{
  ValidationResult result;
  result.final_error = proposal.final_error;
  if (proposal.status == OptimizationSolverStatus::InvalidProblem ||
    proposal.status == OptimizationSolverStatus::NumericalFailure)
  {
    result.reason = proposal.reason.empty() ? "solver_failure" : proposal.reason;
    return result;
  }
  if (proposal.controls.size() != problem.control_indices.size()) {
    result.reason = "control_coverage_mismatch";
    return result;
  }
  std::map<RawKeyFrameId, geometry_msgs::msg::Pose> proposed;
  for (const auto & control : proposal.controls) {
    Eigen::Isometry3d transform;
    if (!PoseToIsometry(control.world_pose, &transform)) {
      result.reason = "non_finite_control";
      return result;
    }
    proposed[control.id] = control.world_pose;
  }
  for (const size_t index : problem.control_indices) {
    const auto & keyframe = problem.keyframes[index];
    const auto found = proposed.find(keyframe.id);
    if (found == proposed.end()) {
      result.reason = "control_identity_mismatch";
      return result;
    }
    if (keyframe.fixed && !PosesNear(
        found->second, keyframe.current_world_pose, 1e-8, 1e-8))
    {
      result.reason = "hard_control_moved";
      return result;
    }
  }

  if (problem.kind == PoseGraphProblemKind::LoopRelative) {
    const bool within_threshold =
      proposal.final_error.translation_m <= problem.loop_translation_threshold_m &&
      proposal.final_error.rotation_rad <= problem.loop_rotation_threshold_rad;
    const bool improved = proposal.final_error.translation_m + 1e-9 <
      proposal.initial_error.translation_m ||
      proposal.final_error.rotation_rad + 1e-9 <
      proposal.initial_error.rotation_rad;
    const bool cost_improved = std::isfinite(proposal.final_cost) &&
      proposal.final_cost <= proposal.initial_cost + 1e-8;
    if (within_threshold && improved && cost_improved) {
      result.decision = ValidationDecision::AcceptFull;
      result.reason = "relative_loop_within_fusion_threshold";
      return result;
    }
    result.reason = !within_threshold ? "relative_loop_still_above_threshold" :
      (!improved ? "relative_loop_not_improved" : "relative_graph_cost_increased");
    return result;
  }

  const bool within_threshold =
    proposal.final_error.translation_m <= config_.translation_threshold_m &&
    proposal.final_error.rotation_rad <= config_.rotation_threshold_rad &&
    proposal.final_error.yaw_rad <= config_.yaw_threshold_rad;
  if (within_threshold) {
    result.decision = ValidationDecision::AcceptFull;
    result.reason = "target_within_threshold";
    return result;
  }

  const bool improved =
    proposal.final_error.translation_m + 1e-9 < proposal.initial_error.translation_m ||
    proposal.final_error.rotation_rad + 1e-9 < proposal.initial_error.rotation_rad ||
    proposal.final_error.yaw_rad + 1e-9 < proposal.initial_error.yaw_rad;
  if (improved && proposal.correction_fraction > 0.0) {
    result.decision = ValidationDecision::AcceptPartialRetry;
    result.reason = "safe_partial_progress";
    return result;
  }
  result.reason = "no_safe_progress";
  return result;
}

}  // namespace orbslam3_multi
