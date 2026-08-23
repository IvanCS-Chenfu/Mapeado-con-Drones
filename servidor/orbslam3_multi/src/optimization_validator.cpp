#include "orbslam3_multi/optimization_validator.hpp"

#include "orbslam3_multi/pose_geometry.hpp"

#include <cmath>
#include <map>
#include <algorithm>

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
    if (proposal.initial_loop_errors.size() != problem.loop_edges.size() ||
      proposal.final_loop_errors.size() != problem.loop_edges.size())
    {
      result.reason = "relative_loop_error_coverage_mismatch";
      return result;
    }
    bool within_threshold = true;
    bool improved = false;
    for (size_t index = 0; index < proposal.final_loop_errors.size(); ++index) {
      const auto & initial = proposal.initial_loop_errors[index];
      const auto & final = proposal.final_loop_errors[index];
      within_threshold = within_threshold &&
        final.translation_m <= problem.loop_translation_threshold_m &&
        final.rotation_rad <= problem.loop_rotation_threshold_rad;
      improved = improved || final.translation_m + 1e-9 < initial.translation_m ||
        final.rotation_rad + 1e-9 < initial.rotation_rad;
    }
    const bool cost_improved = std::isfinite(proposal.final_cost) &&
      proposal.final_cost <= proposal.initial_cost + 1e-8;
    for (const auto & residual : proposal.edge_residuals) {
      if (residual.kind == PoseGraphEdgeKind::CurrentLoop) {
        continue;
      }
      ++result.structural_edges_checked;
      const double translation_increase = std::max(
        0.0, residual.final_error.translation_m - residual.initial_error.translation_m);
      const double rotation_increase = std::max(
        0.0, residual.final_error.rotation_rad - residual.initial_error.rotation_rad);
      result.max_structural_translation_increase_m = std::max(
        result.max_structural_translation_increase_m, translation_increase);
      result.max_structural_rotation_increase_rad = std::max(
        result.max_structural_rotation_increase_rad, rotation_increase);
      double translation_limit = problem.structural_temporal_increase_m;
      double rotation_limit = problem.structural_temporal_increase_rad;
      if (residual.kind == PoseGraphEdgeKind::CovisibilityNative) {
        translation_limit = problem.structural_covisibility_increase_m;
        rotation_limit = problem.structural_covisibility_increase_rad;
      } else if (residual.kind == PoseGraphEdgeKind::PriorLoop) {
        translation_limit = problem.structural_prior_loop_increase_m;
        rotation_limit = problem.structural_prior_loop_increase_rad;
      }
      if (!std::isfinite(residual.final_error.translation_m) ||
        !std::isfinite(residual.final_error.rotation_rad) ||
        translation_increase > translation_limit || rotation_increase > rotation_limit)
      {
        result.reason = residual.kind == PoseGraphEdgeKind::PriorLoop ?
          "prior_loop_structure_degraded" :
          (residual.kind == PoseGraphEdgeKind::CovisibilityNative ?
          "native_covisibility_structure_degraded" : "temporal_structure_degraded");
        return result;
      }
    }

    std::map<RawKeyFrameId, geometry_msgs::msg::Pose> proposed_keyframes;
    for (const auto & keyframe : proposal.keyframes) {
      proposed_keyframes[keyframe.id] = keyframe.world_pose;
    }
    for (const auto & keyframe : problem.keyframes) {
      if (!keyframe.hard_corridor) {
        continue;
      }
      ++result.hard_corridor_keyframes_checked;
      const auto found = proposed_keyframes.find(keyframe.id);
      if (found == proposed_keyframes.end()) {
        result.reason = "hard_corridor_pose_missing";
        return result;
      }
      const auto initial_error = ComputeFiducialError(
        keyframe.current_world_pose, keyframe.hard_corridor_reference_pose);
      const auto final_error = ComputeFiducialError(
        found->second, keyframe.hard_corridor_reference_pose);
      const double shape = std::clamp(
        4.0 * keyframe.hard_corridor_alpha *
        (1.0 - keyframe.hard_corridor_alpha), 0.0, 1.0);
      const double translation_limit = keyframe.fixed ? 1e-8 :
        std::max(0.05, shape * problem.hard_corridor_max_translation_m);
      const double rotation_limit = keyframe.fixed ? 1e-8 :
        std::max(0.01, shape * problem.hard_corridor_max_rotation_rad);
      const double initial_translation_excess = std::max(
        0.0, initial_error.translation_m - translation_limit);
      const double final_translation_excess = std::max(
        0.0, final_error.translation_m - translation_limit);
      const double initial_rotation_excess = std::max(
        0.0, initial_error.rotation_rad - rotation_limit);
      const double final_rotation_excess = std::max(
        0.0, final_error.rotation_rad - rotation_limit);
      result.max_corridor_translation_excess_before_m = std::max(
        result.max_corridor_translation_excess_before_m, initial_translation_excess);
      result.max_corridor_translation_excess_after_m = std::max(
        result.max_corridor_translation_excess_after_m, final_translation_excess);
      result.max_corridor_rotation_excess_before_rad = std::max(
        result.max_corridor_rotation_excess_before_rad, initial_rotation_excess);
      result.max_corridor_rotation_excess_after_rad = std::max(
        result.max_corridor_rotation_excess_after_rad, final_rotation_excess);
      if (final_translation_excess > initial_translation_excess + 1e-8 ||
        final_rotation_excess > initial_rotation_excess + 1e-8)
      {
        result.reason = "hard_corridor_displacement_exceeded";
        return result;
      }
    }

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
