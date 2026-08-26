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
  bool fusion_compatible = false;
  size_t structural_edges_checked = 0;
  size_t optimized_keyframes_checked = 0;
  double max_structural_translation_increase_m = 0.0;
  double max_structural_rotation_increase_rad = 0.0;
  double max_optimized_translation_change_m = 0.0;
  double max_optimized_rotation_change_rad = 0.0;
  std::string reason;
};

/// Aplica invariantes fiduciales, estructurales y de KFs previamente optimizados.
/// Su resultado es puro: aceptar o rechazar nunca implica por sí mismo un commit.
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
