#pragma once

#include "orbslam3_multi/pose_graph_problem.hpp"

#include <string>
#include <vector>

namespace orbslam3_multi
{

enum class OptimizationSolverStatus
{
  Converged,
  MaxIterations,
  InvalidProblem,
  NumericalFailure,
};

struct OptimizedControlPose
{
  RawKeyFrameId id;
  geometry_msgs::msg::Pose world_pose;
};

struct OptimizationProposal
{
  uint64_t task_id = 0;
  OptimizationSolverStatus status = OptimizationSolverStatus::InvalidProblem;
  std::vector<OptimizedControlPose> controls;
  FiducialError initial_error;
  FiducialError final_error;
  double initial_cost = 0.0;
  double final_cost = 0.0;
  size_t iterations = 0;
  double correction_fraction = 0.0;
  std::string reason;
};

class OptimizationManager
{
public:
  explicit OptimizationManager(FiducialOptimizationConfig config = {});

  void Configure(const FiducialOptimizationConfig & config);
  OptimizationProposal Optimize(const PoseGraphProblem & problem) const;

private:
  FiducialOptimizationConfig config_;
};

const char * ToString(OptimizationSolverStatus status);

}  // namespace orbslam3_multi
