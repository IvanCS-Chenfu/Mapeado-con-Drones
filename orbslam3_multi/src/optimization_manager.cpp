#include "orbslam3_multi/optimization_manager.hpp"

#include "orbslam3_multi/pose_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

namespace orbslam3_multi
{
namespace
{

FiducialError RelativeConstraintError(
  const Eigen::Isometry3d & from, const Eigen::Isometry3d & to,
  const Eigen::Isometry3d & measured)
{
  const Eigen::Isometry3d current = from.inverse() * to;
  FiducialError error;
  error.translation_m =
    (current.translation() - measured.translation()).norm();
  error.rotation_rad = RotationErrorRad(current, measured);
  error.yaw_rad = std::abs(NormalizeAngle(
    YawFromRotation(current.linear()) - YawFromRotation(measured.linear())));
  return error;
}

double EdgeCost(
  const PoseGraphEdge & edge,
  const std::map<size_t, Eigen::Isometry3d> & poses)
{
  const auto from = poses.find(edge.from_index);
  const auto to = poses.find(edge.to_index);
  Eigen::Isometry3d measured;
  if (from == poses.end() || to == poses.end() ||
    !PoseToIsometry(edge.relative_raw_pose, &measured))
  {
    return std::numeric_limits<double>::infinity();
  }
  const auto error = RelativeConstraintError(from->second, to->second, measured);
  const double squared = error.translation_m * error.translation_m +
    error.rotation_rad * error.rotation_rad;
  const double huber = squared <= 1.0 ? squared : 2.0 * std::sqrt(squared) - 1.0;
  return std::max(0.1, edge.information_weight) * huber;
}

double GraphCost(
  const PoseGraphProblem & problem,
  const std::map<size_t, Eigen::Isometry3d> & poses)
{
  double cost = 0.0;
  for (const auto & edge : problem.temporal_edges) {
    cost += EdgeCost(edge, poses);
  }
  for (const auto & edge : problem.covisibility_edges) {
    cost += EdgeCost(edge, poses);
  }
  for (const auto & edge : problem.loop_edges) {
    cost += EdgeCost(edge, poses);
  }
  return cost;
}

bool RelaxEdge(
  const PoseGraphProblem & problem, const PoseGraphEdge & edge,
  double gain, std::map<size_t, Eigen::Isometry3d> * poses)
{
  auto from = poses->find(edge.from_index);
  auto to = poses->find(edge.to_index);
  Eigen::Isometry3d measured;
  if (from == poses->end() || to == poses->end() ||
    !PoseToIsometry(edge.relative_raw_pose, &measured))
  {
    return false;
  }
  const bool from_fixed = problem.keyframes[edge.from_index].fixed;
  const bool to_fixed = problem.keyframes[edge.to_index].fixed;
  if (from_fixed && to_fixed) {
    return true;
  }
  const Eigen::Isometry3d predicted_to = from->second * measured;
  const Eigen::Isometry3d predicted_from = to->second * measured.inverse();
  if (!from_fixed && !to_fixed) {
    from->second = InterpolateIsometry(
      from->second, predicted_from, 0.5 * gain);
    to->second = InterpolateIsometry(
      to->second, predicted_to, 0.5 * gain);
  } else if (!from_fixed) {
    from->second = InterpolateIsometry(from->second, predicted_from, gain);
  } else {
    to->second = InterpolateIsometry(to->second, predicted_to, gain);
  }
  return from->second.matrix().allFinite() && to->second.matrix().allFinite();
}

}  // namespace

OptimizationManager::OptimizationManager(FiducialOptimizationConfig config)
: config_(config)
{
}

void OptimizationManager::Configure(const FiducialOptimizationConfig & config)
{
  config_ = config;
}

const char * ToString(OptimizationSolverStatus status)
{
  switch (status) {
    case OptimizationSolverStatus::Converged:
      return "converged";
    case OptimizationSolverStatus::MaxIterations:
      return "max_iterations";
    case OptimizationSolverStatus::InvalidProblem:
      return "invalid_problem";
    case OptimizationSolverStatus::NumericalFailure:
      return "numerical_failure";
  }
  return "unknown";
}

OptimizationProposal OptimizationManager::Optimize(
  const PoseGraphProblem & problem) const
{
  OptimizationProposal proposal;
  proposal.task_id = problem.kind == PoseGraphProblemKind::LoopRelative ?
    problem.loop_task.task_id : problem.task.task_id;
  if (problem.control_indices.size() < 2U || problem.keyframes.size() < 2U ||
    problem.control_indices.front() >= problem.keyframes.size() ||
    problem.control_indices.back() >= problem.keyframes.size())
  {
    proposal.reason = "invalid_control_layout";
    return proposal;
  }

  if (problem.kind == PoseGraphProblemKind::LoopRelative) {
    if (problem.loop_edges.size() != 1U) {
      proposal.reason = "current_loop_constraint_missing";
      return proposal;
    }
    std::map<size_t, Eigen::Isometry3d> controls;
    for (const size_t index : problem.control_indices) {
      Eigen::Isometry3d pose;
      if (!PoseToIsometry(problem.keyframes[index].current_world_pose, &pose)) {
        proposal.status = OptimizationSolverStatus::NumericalFailure;
        proposal.reason = "loop_control_pose_invalid";
        return proposal;
      }
      controls[index] = pose;
    }
    const auto & loop_edge = problem.loop_edges.front();
    Eigen::Isometry3d measured;
    if (controls.count(loop_edge.from_index) == 0U ||
      controls.count(loop_edge.to_index) == 0U ||
      !PoseToIsometry(loop_edge.relative_raw_pose, &measured))
    {
      proposal.reason = "loop_endpoint_not_control";
      return proposal;
    }
    proposal.initial_error = RelativeConstraintError(
      controls.at(loop_edge.from_index), controls.at(loop_edge.to_index), measured);
    proposal.initial_cost = GraphCost(problem, controls);
    if (!std::isfinite(proposal.initial_cost)) {
      proposal.status = OptimizationSolverStatus::NumericalFailure;
      proposal.reason = "initial_graph_cost_invalid";
      return proposal;
    }

    constexpr size_t kMaxIterations = 160U;
    double previous_cost = proposal.initial_cost;
    for (size_t iteration = 0; iteration < kMaxIterations; ++iteration) {
      for (const auto & edge : problem.temporal_edges) {
        if (!RelaxEdge(problem, edge, 0.08, &controls)) {
          proposal.status = OptimizationSolverStatus::NumericalFailure;
          proposal.reason = "temporal_relaxation_failed";
          return proposal;
        }
      }
      for (const auto & edge : problem.covisibility_edges) {
        const double gain = edge.kind == PoseGraphEdgeKind::PriorLoop ? 0.18 : 0.10;
        if (!RelaxEdge(problem, edge, gain, &controls)) {
          proposal.status = OptimizationSolverStatus::NumericalFailure;
          proposal.reason = "covisibility_relaxation_failed";
          return proposal;
        }
      }
      if (!RelaxEdge(problem, loop_edge, 0.72, &controls)) {
        proposal.status = OptimizationSolverStatus::NumericalFailure;
        proposal.reason = "loop_relaxation_failed";
        return proposal;
      }
      proposal.iterations = iteration + 1U;
      const double current_cost = GraphCost(problem, controls);
      if (!std::isfinite(current_cost)) {
        proposal.status = OptimizationSolverStatus::NumericalFailure;
        proposal.reason = "graph_cost_invalid";
        return proposal;
      }
      const auto error = RelativeConstraintError(
        controls.at(loop_edge.from_index), controls.at(loop_edge.to_index), measured);
      if (error.translation_m <= 0.5 * problem.loop_translation_threshold_m &&
        error.rotation_rad <= 0.5 * problem.loop_rotation_threshold_rad &&
        std::abs(previous_cost - current_cost) <= 1e-8 * (1.0 + previous_cost))
      {
        break;
      }
      previous_cost = current_cost;
    }

    proposal.controls.reserve(problem.control_indices.size());
    for (const size_t index : problem.control_indices) {
      proposal.controls.push_back(
        {problem.keyframes[index].id, IsometryToPose(controls.at(index))});
    }
    proposal.final_error = RelativeConstraintError(
      controls.at(loop_edge.from_index), controls.at(loop_edge.to_index), measured);
    proposal.final_cost = GraphCost(problem, controls);
    proposal.correction_fraction = 1.0;
    proposal.status = OptimizationSolverStatus::Converged;
    proposal.reason = "covisible_relative_graph_relaxed";
    return proposal;
  }

  if (problem.control_indices.front() != 0U ||
    problem.control_indices.back() + 1U != problem.keyframes.size())
  {
    proposal.reason = "invalid_fiducial_control_layout";
    return proposal;
  }

  const auto & target_keyframe = problem.keyframes.back();
  Eigen::Isometry3d target_current;
  Eigen::Isometry3d target_expected;
  if (!PoseToIsometry(target_keyframe.current_world_pose, &target_current) ||
    !PoseToIsometry(problem.task.target_world_T_kf, &target_expected))
  {
    proposal.status = OptimizationSolverStatus::NumericalFailure;
    proposal.reason = "target_pose_invalid";
    return proposal;
  }
  proposal.initial_error = ComputeFiducialError(
    target_keyframe.current_world_pose, problem.task.target_world_T_kf);

  const Eigen::Isometry3d full_correction = target_expected * target_current.inverse();
  const double fraction = std::clamp(config_.max_correction_fraction_per_pass, 0.0, 1.0);
  proposal.correction_fraction = fraction;
  const double endpoint_half_ratio =
    0.5 * std::clamp(config_.endpoint_neighborhood_ratio, 0.0, 1.0);

  std::map<size_t, Eigen::Isometry3d> initial_controls;
  std::map<size_t, Eigen::Isometry3d> optimized_controls;
  proposal.controls.reserve(problem.control_indices.size());
  for (const size_t index : problem.control_indices) {
    const auto & keyframe = problem.keyframes[index];
    Eigen::Isometry3d current;
    if (!PoseToIsometry(keyframe.current_world_pose, &current)) {
      proposal.status = OptimizationSolverStatus::NumericalFailure;
      proposal.reason = "control_pose_invalid";
      proposal.controls.clear();
      return proposal;
    }
    initial_controls[index] = current;

    double correction_alpha = 0.0;
    if (problem.task.replaces_soft_loop_anchor) {
      correction_alpha = fraction;
    } else if (index == problem.keyframes.size() - 1U) {
      correction_alpha = fraction;
    } else if (keyframe.path_alpha <= endpoint_half_ratio) {
      correction_alpha = 0.0;
    } else if (keyframe.path_alpha >= 1.0 - endpoint_half_ratio) {
      correction_alpha = fraction;
    } else {
      const double span = std::max(1e-9, 1.0 - 2.0 * endpoint_half_ratio);
      const double linear = std::clamp(
        (keyframe.path_alpha - endpoint_half_ratio) / span, 0.0, 1.0);
      const double smooth = linear * linear * (3.0 - 2.0 * linear);
      correction_alpha = fraction * smooth;
    }

    const Eigen::Isometry3d correction = InterpolateIsometry(
      Eigen::Isometry3d::Identity(), full_correction, correction_alpha);
    const Eigen::Isometry3d optimized = correction * current;
    if (!optimized.matrix().allFinite()) {
      proposal.status = OptimizationSolverStatus::NumericalFailure;
      proposal.reason = "optimized_pose_non_finite";
      proposal.controls.clear();
      return proposal;
    }
    optimized_controls[index] = optimized;
  }

  const size_t target_index = problem.keyframes.size() - 1U;
  const Eigen::Isometry3d pinned_target = InterpolateIsometry(
    target_current, target_expected, fraction);
  constexpr size_t kFiducialGraphIterations = 80U;
  for (size_t iteration = 0; iteration < kFiducialGraphIterations; ++iteration) {
    for (const auto & edge : problem.temporal_edges) {
      if (!RelaxEdge(problem, edge, 0.04, &optimized_controls)) {
        proposal.status = OptimizationSolverStatus::NumericalFailure;
        proposal.reason = "fiducial_temporal_relaxation_failed";
        return proposal;
      }
    }
    for (const auto & edge : problem.covisibility_edges) {
      const double gain = edge.kind == PoseGraphEdgeKind::PriorLoop ? 0.12 : 0.07;
      if (!RelaxEdge(problem, edge, gain, &optimized_controls)) {
        proposal.status = OptimizationSolverStatus::NumericalFailure;
        proposal.reason = "fiducial_covisibility_relaxation_failed";
        return proposal;
      }
    }
    for (const size_t index : problem.control_indices) {
      if (problem.keyframes[index].fixed) {
        optimized_controls[index] = initial_controls.at(index);
      }
    }
    optimized_controls[target_index] = pinned_target;
  }

  for (const size_t index : problem.control_indices) {
    proposal.controls.push_back(
      {problem.keyframes[index].id, IsometryToPose(optimized_controls.at(index))});
  }

  proposal.final_error = ComputeFiducialError(
    proposal.controls.back().world_pose, problem.task.target_world_T_kf);
  proposal.status = fraction >= 1.0 - 1e-12 ?
    OptimizationSolverStatus::Converged : OptimizationSolverStatus::MaxIterations;
  proposal.reason = fraction >= 1.0 - 1e-12 ?
    "se3_covisible_absolute_graph" : "se3_covisible_partial_graph";
  proposal.iterations = kFiducialGraphIterations;
  proposal.initial_cost = GraphCost(problem, initial_controls) +
    proposal.initial_error.translation_m *
    proposal.initial_error.translation_m + proposal.initial_error.rotation_rad *
    proposal.initial_error.rotation_rad;
  proposal.final_cost = GraphCost(problem, optimized_controls) +
    proposal.final_error.translation_m *
    proposal.final_error.translation_m + proposal.final_error.rotation_rad *
    proposal.final_error.rotation_rad;
  return proposal;
}

}  // namespace orbslam3_multi
