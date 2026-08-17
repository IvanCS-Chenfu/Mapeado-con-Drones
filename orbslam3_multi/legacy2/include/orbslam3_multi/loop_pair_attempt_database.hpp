#pragma once

#include "orbslam3_multi/loop_verification_result.hpp"
#include "orbslam3_multi/raw_map_types.hpp"

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>

namespace orbslam3_multi
{

enum class LoopPairState : uint8_t
{
    Unknown = 0,
    BowRejected = 1,
    DeferredNoPose = 2,
    PendingGeometry = 3,
    GeometryRejected = 4,
    ConfirmedLowError = 5,
    ConfirmedHighError = 6,
    OptimizationPending = 7,
    FusionPending = 8,
    Fused = 9,
};

struct LoopPairAttempt
{
    RawKeyFrameId kf_a;
    RawKeyFrameId kf_b;
    RawKeyFrameRevision revision_a;
    RawKeyFrameRevision revision_b;
    uint64_t arrival_id = 0;
    LoopPairState state = LoopPairState::Unknown;
    std::string reason;
    std::optional<LoopVerificationResult> confirmed_result;
};

struct LoopPairAttemptStats
{
    uint64_t entries = 0;
    uint64_t definitive_rejects_recorded = 0;
    uint64_t bow_rejects_recorded = 0;
    uint64_t state_updates = 0;
    uint64_t geometry_hits = 0;
    uint64_t bow_hits = 0;
    uint64_t terminal_hits = 0;
    uint64_t hits = 0;
    uint64_t misses = 0;
    uint64_t invalidations = 0;
    uint64_t non_cacheable = 0;
};

class LoopPairAttemptDatabase
{
public:
    using PairKey = std::pair<RawKeyFrameId, RawKeyFrameId>;

    static PairKey CanonicalPair(
        const RawKeyFrameId& first,
        const RawKeyFrameId& second);

    bool ShouldSkipDefinitiveReject(
        const RawKeyFrameId& first,
        const RawKeyFrameRevision& first_revision,
        const RawKeyFrameId& second,
        const RawKeyFrameRevision& second_revision,
        LoopPairAttempt* attempt = nullptr);

    bool ShouldSkipBow(
        const RawKeyFrameId& first,
        const RawKeyFrameRevision& first_revision,
        const RawKeyFrameId& second,
        const RawKeyFrameRevision& second_revision,
        LoopPairAttempt* attempt = nullptr);

    bool ShouldSkipTerminalState(
        const RawKeyFrameId& first,
        const RawKeyFrameRevision& first_revision,
        const RawKeyFrameId& second,
        const RawKeyFrameRevision& second_revision,
        LoopPairAttempt* attempt = nullptr);

    void RecordState(
        const RawKeyFrameId& first,
        const RawKeyFrameRevision& first_revision,
        const RawKeyFrameId& second,
        const RawKeyFrameRevision& second_revision,
        uint64_t arrival_id,
        LoopPairState state,
        const std::string& reason,
        const LoopVerificationResult* confirmed_result = nullptr);

    void RecordBowRejected(
        const RawKeyFrameId& first,
        const RawKeyFrameRevision& first_revision,
        const RawKeyFrameId& second,
        const RawKeyFrameRevision& second_revision,
        uint64_t arrival_id,
        const std::string& reason);

    void RecordDefinitiveReject(
        const RawKeyFrameId& first,
        const RawKeyFrameRevision& first_revision,
        const RawKeyFrameId& second,
        const RawKeyFrameRevision& second_revision,
        uint64_t arrival_id,
        const std::string& reason);

    void RecordNonCacheable();
    bool Erase(const RawKeyFrameId& first, const RawKeyFrameId& second);
    std::optional<LoopPairAttempt> Get(
        const RawKeyFrameId& first,
        const RawKeyFrameId& second) const;
    void Clear();
    LoopPairAttemptStats GetStats() const;

private:
    static LoopPairAttempt CanonicalAttempt(
        const RawKeyFrameId& first,
        const RawKeyFrameRevision& first_revision,
        const RawKeyFrameId& second,
        const RawKeyFrameRevision& second_revision);

    std::map<PairKey, LoopPairAttempt> attempts_;
    LoopPairAttemptStats stats_;
};

const char* ToString(LoopPairState state);

}  // namespace orbslam3_multi
