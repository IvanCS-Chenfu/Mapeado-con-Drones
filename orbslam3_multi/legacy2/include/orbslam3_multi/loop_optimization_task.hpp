#pragma once

#include "orbslam3_multi/loop_verification_result.hpp"

#include <cstdint>
#include <string>

namespace orbslam3_multi
{

// Entrada privada de la rama 1Q. Conserva solo la evidencia geometrica y la
// revision de llegada; no representa estado persistente ni modifica raw DB.
struct LoopOptimizationTask
{
    uint64_t task_id = 0;
    uint64_t arrival_id = 0;
    std::string source;
    LoopVerificationResult verification;
};

}  // namespace orbslam3_multi
