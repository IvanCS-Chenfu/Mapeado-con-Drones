#include "orbslam3_multi/loop_pair_attempt_database.hpp"

namespace orbslam3_multi
{

const char* ToString(LoopPairState state)
{
    switch (state)
    {
        case LoopPairState::Unknown: return "UNKNOWN";
        case LoopPairState::BowRejected: return "BOW_REJECTED";
        case LoopPairState::DeferredNoPose: return "DEFERRED_NO_POSE";
        case LoopPairState::PendingGeometry: return "PENDING_GEOMETRY";
        case LoopPairState::GeometryRejected: return "GEOMETRY_REJECTED";
        case LoopPairState::ConfirmedLowError: return "CONFIRMED_LOW_ERROR";
        case LoopPairState::ConfirmedHighError: return "CONFIRMED_HIGH_ERROR";
        case LoopPairState::OptimizationPending: return "OPTIMIZATION_PENDING";
        case LoopPairState::FusionPending: return "FUSION_PENDING";
        case LoopPairState::Fused: return "FUSED";
    }
    return "UNKNOWN";
}

LoopPairAttemptDatabase::PairKey LoopPairAttemptDatabase::CanonicalPair(
    const RawKeyFrameId& first,
    const RawKeyFrameId& second)
{
    return second < first
        ? PairKey{second, first}
        : PairKey{first, second};
}

LoopPairAttempt LoopPairAttemptDatabase::CanonicalAttempt(
    const RawKeyFrameId& first,
    const RawKeyFrameRevision& first_revision,
    const RawKeyFrameId& second,
    const RawKeyFrameRevision& second_revision)
{
    LoopPairAttempt attempt;
    if (second < first)
    {
        attempt.kf_a = second;
        attempt.kf_b = first;
        attempt.revision_a = second_revision;
        attempt.revision_b = first_revision;
    }
    else
    {
        attempt.kf_a = first;
        attempt.kf_b = second;
        attempt.revision_a = first_revision;
        attempt.revision_b = second_revision;
    }
    return attempt;
}

bool LoopPairAttemptDatabase::ShouldSkipDefinitiveReject(
    const RawKeyFrameId& first,
    const RawKeyFrameRevision& first_revision,
    const RawKeyFrameId& second,
    const RawKeyFrameRevision& second_revision,
    LoopPairAttempt* attempt)
{
    const auto canonical = CanonicalAttempt(
        first,
        first_revision,
        second,
        second_revision);
    const auto key = PairKey{canonical.kf_a, canonical.kf_b};
    const auto it = attempts_.find(key);
    if (it == attempts_.end())
    {
        ++stats_.misses;
        return false;
    }
    if (it->second.state != LoopPairState::GeometryRejected)
    {
        ++stats_.misses;
        return false;
    }
    if (!SameGeometryRevision(it->second.revision_a, canonical.revision_a) ||
        !SameGeometryRevision(it->second.revision_b, canonical.revision_b))
    {
        attempts_.erase(it);
        ++stats_.invalidations;
        stats_.entries = attempts_.size();
        return false;
    }
    ++stats_.hits;
    ++stats_.geometry_hits;
    if (attempt)
    {
        *attempt = it->second;
    }
    return true;
}

bool LoopPairAttemptDatabase::ShouldSkipBow(
    const RawKeyFrameId& first,
    const RawKeyFrameRevision& first_revision,
    const RawKeyFrameId& second,
    const RawKeyFrameRevision& second_revision,
    LoopPairAttempt* attempt)
{
    const auto canonical = CanonicalAttempt(
        first, first_revision, second, second_revision);
    const auto it = attempts_.find({canonical.kf_a, canonical.kf_b});
    if (it == attempts_.end() ||
        it->second.state != LoopPairState::BowRejected)
    {
        ++stats_.misses;
        return false;
    }
    if (!SameAppearanceRevision(it->second.revision_a, canonical.revision_a) ||
        !SameAppearanceRevision(it->second.revision_b, canonical.revision_b))
    {
        attempts_.erase(it);
        ++stats_.invalidations;
        stats_.entries = attempts_.size();
        return false;
    }
    ++stats_.hits;
    ++stats_.bow_hits;
    if (attempt)
    {
        *attempt = it->second;
    }
    return true;
}

bool LoopPairAttemptDatabase::ShouldSkipTerminalState(
    const RawKeyFrameId& first,
    const RawKeyFrameRevision& first_revision,
    const RawKeyFrameId& second,
    const RawKeyFrameRevision& second_revision,
    LoopPairAttempt* attempt)
{
    const auto canonical = CanonicalAttempt(
        first, first_revision, second, second_revision);
    const auto it = attempts_.find({canonical.kf_a, canonical.kf_b});
    if (it == attempts_.end())
    {
        return false;
    }
    const bool terminal =
        it->second.state == LoopPairState::OptimizationPending ||
        it->second.state == LoopPairState::FusionPending ||
        it->second.state == LoopPairState::Fused;
    if (!terminal ||
        !SameGeometryRevision(it->second.revision_a, canonical.revision_a) ||
        !SameGeometryRevision(it->second.revision_b, canonical.revision_b))
    {
        return false;
    }
    ++stats_.terminal_hits;
    if (attempt)
    {
        *attempt = it->second;
    }
    return true;
}

void LoopPairAttemptDatabase::RecordState(
    const RawKeyFrameId& first,
    const RawKeyFrameRevision& first_revision,
    const RawKeyFrameId& second,
    const RawKeyFrameRevision& second_revision,
    uint64_t arrival_id,
    LoopPairState state,
    const std::string& reason,
    const LoopVerificationResult* confirmed_result)
{
    auto attempt = CanonicalAttempt(
        first, first_revision, second, second_revision);
    attempt.arrival_id = arrival_id;
    attempt.state = state;
    attempt.reason = reason;
    if (confirmed_result)
    {
        attempt.confirmed_result = *confirmed_result;
    }
    attempts_[{attempt.kf_a, attempt.kf_b}] = std::move(attempt);
    ++stats_.state_updates;
    stats_.entries = attempts_.size();
}

void LoopPairAttemptDatabase::RecordBowRejected(
    const RawKeyFrameId& first,
    const RawKeyFrameRevision& first_revision,
    const RawKeyFrameId& second,
    const RawKeyFrameRevision& second_revision,
    uint64_t arrival_id,
    const std::string& reason)
{
    RecordState(
        first, first_revision, second, second_revision, arrival_id,
        LoopPairState::BowRejected, reason);
    ++stats_.bow_rejects_recorded;
}

void LoopPairAttemptDatabase::RecordDefinitiveReject(
    const RawKeyFrameId& first,
    const RawKeyFrameRevision& first_revision,
    const RawKeyFrameId& second,
    const RawKeyFrameRevision& second_revision,
    uint64_t arrival_id,
    const std::string& reason)
{
    RecordState(
        first, first_revision, second, second_revision, arrival_id,
        LoopPairState::GeometryRejected, reason);
    ++stats_.definitive_rejects_recorded;
}

std::optional<LoopPairAttempt> LoopPairAttemptDatabase::Get(
    const RawKeyFrameId& first,
    const RawKeyFrameId& second) const
{
    const auto it = attempts_.find(CanonicalPair(first, second));
    return it == attempts_.end()
        ? std::optional<LoopPairAttempt>{}
        : std::optional<LoopPairAttempt>{it->second};
}

void LoopPairAttemptDatabase::RecordNonCacheable()
{
    ++stats_.non_cacheable;
}

bool LoopPairAttemptDatabase::Erase(
    const RawKeyFrameId& first,
    const RawKeyFrameId& second)
{
    const bool erased = attempts_.erase(CanonicalPair(first, second)) > 0;
    stats_.entries = attempts_.size();
    return erased;
}

void LoopPairAttemptDatabase::Clear()
{
    attempts_.clear();
    stats_ = {};
}

LoopPairAttemptStats LoopPairAttemptDatabase::GetStats() const
{
    LoopPairAttemptStats result = stats_;
    result.entries = attempts_.size();
    return result;
}

}  // namespace orbslam3_multi
