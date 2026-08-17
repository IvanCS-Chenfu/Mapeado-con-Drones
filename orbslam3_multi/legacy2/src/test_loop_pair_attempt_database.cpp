#include "orbslam3_multi/loop_pair_attempt_database.hpp"

#include <iostream>

namespace
{

bool Expect(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "[test_loop_pair_attempt_database] " << message << '\n';
        return false;
    }
    return true;
}

}  // namespace

int main()
{
    using namespace orbslam3_multi;
    const RawKeyFrameId a{1, 0, 10};
    const RawKeyFrameId b{1, 0, 30};
    RawKeyFrameRevision revision_a;
    revision_a.material = 4;
    revision_a.pose = 2;
    revision_a.associations = 4;
    revision_a.appearance = 3;
    revision_a.geometry = 4;
    revision_a.metadata = 1;
    RawKeyFrameRevision revision_b;
    revision_b.material = 9;
    revision_b.pose = 8;
    revision_b.associations = 9;
    revision_b.appearance = 7;
    revision_b.geometry = 9;
    revision_b.metadata = 2;

    LoopPairAttemptDatabase db;
    bool ok = true;
    ok &= Expect(
        !db.ShouldSkipDefinitiveReject(a, revision_a, b, revision_b),
        "empty cache unexpectedly hit");
    db.RecordDefinitiveReject(
        a, revision_a, b, revision_b, 12, "not_enough_initial_matches");

    LoopPairAttempt attempt;
    ok &= Expect(
        db.ShouldSkipDefinitiveReject(b, revision_b, a, revision_a, &attempt),
        "reverse lookup did not use canonical pair");
    ok &= Expect(
        attempt.kf_a == a && attempt.kf_b == b,
        "stored pair is not canonical");

    revision_b.associations = 10;
    revision_b.material = 10;
    ok &= Expect(
        !db.ShouldSkipDefinitiveReject(a, revision_a, b, revision_b),
        "material revision did not invalidate rejection");
    ok &= Expect(db.GetStats().invalidations == 1, "invalidation not counted");

    db.RecordBowRejected(a, revision_a, b, revision_b, 13, "low_bow_score");
    ok &= Expect(
        db.ShouldSkipBow(a, revision_a, b, revision_b),
        "BoW rejection did not hit for the same appearance revisions");
    RawKeyFrameRevision metadata_only = revision_b;
    ++metadata_only.metadata;
    ++metadata_only.material;
    ok &= Expect(
        db.ShouldSkipBow(a, revision_a, b, metadata_only),
        "metadata-only change incorrectly invalidated BoW rejection");

    db.RecordState(
        a, revision_a, b, revision_b, 14,
        LoopPairState::OptimizationPending, "high_error_loop");
    ok &= Expect(
        db.ShouldSkipTerminalState(a, revision_a, b, metadata_only),
        "metadata-only change invalidated optimization pending state");
    ++metadata_only.geometry;
    ok &= Expect(
        !db.ShouldSkipTerminalState(a, revision_a, b, metadata_only),
        "geometry change did not invalidate terminal state");

    db.RecordNonCacheable();
    ok &= Expect(db.GetStats().non_cacheable == 1, "non-cacheable result not counted");
    return ok ? 0 : 1;
}
