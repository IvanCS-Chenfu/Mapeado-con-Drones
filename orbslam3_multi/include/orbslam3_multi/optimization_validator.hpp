#pragma once

#include "orbslam3_multi/optimization_manager.hpp"

#include <string>

namespace orbslam3_multi
{

enum class ValidationDecision
{
  AcceptFull,
  AcceptPartialRetry,
  HardFailure,
};

struct ValidationResult
{
  ValidationDecision decision = ValidationDecision::HardFailure;
  FiducialError final_error;
  size_t structural_edges_checked = 0;
  size_t hard_corridor_keyframes_checked = 0;
  double max_structural_translation_increase_m = 0.0;
  double max_structural_rotation_increase_rad = 0.0;
  double max_corridor_translation_excess_before_m = 0.0;
  double max_corridor_translation_excess_after_m = 0.0;
  double max_corridor_rotation_excess_before_rad = 0.0;
  double max_corridor_rotation_excess_after_rad = 0.0;
  std::string reason;
};

class OptimizationValidator
{
public:
  explicit OptimizationValidator(FiducialOptimizationConfig config = {});

  void Configure(const FiducialOptimizationConfig & config);
  ValidationResult Validate(
    const PoseGraphProblem & problem,
    const OptimizationProposal & proposal) const;

private:
  FiducialOptimizationConfig config_;
};

const char * ToString(ValidationDecision decision);

}  // namespace orbslam3_multi
