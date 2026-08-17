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
