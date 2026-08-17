#include "orbslam3_multi/fused_landmark_manager.hpp"
#include "orbslam3_multi/global_map_builder.hpp"
#include "orbslam3_multi/loop_decision_manager.hpp"
#include "orbslam3_multi/subcloud_loop_verifier.hpp"

#include <cmath>
#include <iostream>
#include <vector>

namespace
{

geometry_msgs::msg::Pose MakePose(double x)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.orientation.w = 1.0;
    return pose;
}

orbslam3_msgs::msg::OrbMap MakeMap(
    uint32_t drone_id,
    double point_x,
    uint8_t descriptor_value,
    bool add_untracked_point)
{
    orbslam3_msgs::msg::OrbMap map;
    map.drone_id = drone_id;
    map.drone_name = "dron_" + std::to_string(drone_id);
    map.map_epoch = 0;
    map.map_sequence = 1;
    map.map_frame = map.drone_name + "_orb_map";

    orbslam3_msgs::msg::OrbKeyFrame keyframe;
    keyframe.id = 0;
    keyframe.pose = MakePose(0.0);
    map.keyframes.push_back(keyframe);

    orbslam3_msgs::msg::OrbMapPoint point;
    point.id = 10;
    point.position.x = point_x;
    point.descriptor.data.fill(descriptor_value);
    point.observations_count = 4;
    point.found_ratio = 1.0F;
    point.reference_keyframe_id = 0;
    map.mappoints.push_back(point);

    if (add_untracked_point)
    {
        point.id = 20;
        point.position.x = 5.0;
        point.descriptor.data.fill(9U);
        map.mappoints.push_back(point);
    }
    return map;
}

orbslam3_msgs::msg::OrbMap MakeOverlapMap(
    uint32_t drone_id,
    double offset_x,
    double offset_y)
{
    orbslam3_msgs::msg::OrbMap map;
    map.drone_id = drone_id;
    map.drone_name = "dron_" + std::to_string(drone_id);
    map.map_epoch = 0;
    map.map_sequence = 1;
    map.map_frame = map.drone_name + "_orb_map";
    map.image_width = 640;
    map.image_height = 480;

    orbslam3_msgs::msg::OrbKeyFrame keyframe;
    keyframe.id = 0;
    keyframe.pose = MakePose(0.0);
    for (uint64_t index = 0; index < 12; ++index)
    {
        const uint64_t column = index % 4U;
        const uint64_t row = index / 4U;
        const uint64_t mappoint_id = 1000U + index;
        const uint8_t descriptor_value =
            static_cast<uint8_t>(index + 1U);

        orbslam3_msgs::msg::OrbMapPoint point;
        point.id = mappoint_id;
        point.position.x =
            static_cast<double>(column) * 0.60 + offset_x;
        point.position.y =
            static_cast<double>(row) * 0.60 + offset_y;
        point.position.z =
            static_cast<double>((column + row) % 2U) * 0.15;
        point.descriptor.data.fill(descriptor_value);
        point.observations_count = 3;
        point.found_ratio = 1.0F;
        point.reference_keyframe_id = 0;
        map.mappoints.push_back(point);

        orbslam3_msgs::msg::OrbKeyPoint keypoint;
        keypoint.u = static_cast<float>(80U + column * 160U);
        keypoint.v = static_cast<float>(80U + row * 160U);
        keypoint.descriptor.data.fill(descriptor_value);
        keyframe.keypoints.push_back(keypoint);
        keyframe.mappoint_ids.push_back(mappoint_id);
    }
    map.keyframes.push_back(keyframe);
    return map;
}

bool Expect(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "[test_fused_landmark_manager] " << message << '\n';
        return false;
    }
    return true;
}

}  // namespace

int main()
{
    using namespace orbslam3_multi;

    bool ok = true;
    RawMapDatabase raw_db;
    GlobalPoseStore pose_store;
    LandmarkScoreManager score_manager;
    FusedLandmarkManager fused_manager;

    const std::vector<orbslam3_msgs::msg::OrbMap> maps{
        MakeMap(1, 0.0, 1U, true),
        MakeMap(2, 0.1, 1U, false),
        MakeMap(3, 0.2, 3U, false),
        MakeMap(4, 0.3, 7U, false)};
    uint64_t arrival_id = 1;
    for (const auto& map : maps)
    {
        raw_db.InsertDelta(arrival_id++, map);
        const RawSubmapId submap_id{map.drone_id, map.map_epoch};
        ok &= Expect(
            pose_store.AnchorSubmap(
                submap_id,
                Eigen::Matrix4d::Identity(),
                raw_db,
                "TEST_F1P_ANCHOR").success,
            "submap anchor failed");
        for (const auto& point : map.mappoints)
        {
            score_manager.ApplyOrbSlamQuality(
                RawMapPointId{map.drone_id, map.map_epoch, point.id},
                point);
        }
    }

    const RawMapPointId a{1, 0, 10};
    const RawMapPointId b{2, 0, 10};
    const RawMapPointId c{3, 0, 10};
    const RawMapPointId d{4, 0, 10};
    const RawMapPointId untracked{1, 0, 20};
    const auto raw_stats_before = raw_db.GetDatabaseStats();

    const auto first = fused_manager.FuseInlierPairs(
        {{a, b}}, raw_db, score_manager, 0.8);
    const auto second = fused_manager.FuseInlierPairs(
        {{c, d}}, raw_db, score_manager, 0.8);
    const auto merged = fused_manager.FuseInlierPairs(
        {{b, c}}, raw_db, score_manager, 0.9);
    const auto repeated = fused_manager.FuseInlierPairs(
        {{a, d}}, raw_db, score_manager, 0.9);
    const auto rejected = fused_manager.FuseInlierPairs(
        {{a, a}}, raw_db, score_manager, 0.9);

    ok &= Expect(first.tracks_created == 1, "first track was not created");
    ok &= Expect(second.tracks_created == 1, "second track was not created");
    ok &= Expect(merged.tracks_merged == 1, "tracks were not merged transitively");
    ok &= Expect(repeated.pairs_fused == 1, "repeated support was not accepted");
    ok &= Expect(rejected.pairs_rejected == 1, "same raw ID was not rejected");

    const auto track_id = fused_manager.GetTrackIdForMember(a);
    ok &= Expect(track_id.has_value(), "member has no reverse track lookup");
    ok &= Expect(
        fused_manager.GetTrackIdForMember(b) == track_id &&
        fused_manager.GetTrackIdForMember(c) == track_id &&
        fused_manager.GetTrackIdForMember(d) == track_id,
        "transitive members do not share one track");
    ok &= Expect(
        !fused_manager.GetTrackIdForMember(untracked),
        "untracked point unexpectedly belongs to a track");

    const auto track = track_id
        ? fused_manager.GetTrack(track_id.value())
        : std::optional<FusedLandmarkTrack>{};
    ok &= Expect(track && track->member_mappoint_ids.size() == 4U,
                 "merged track does not contain four unique members");
    ok &= Expect(track && track->descriptor_valid,
                 "descriptor medoid was not computed");
    ok &= Expect(track && track->representative_descriptor[0] == 1U,
                 "descriptor medoid is not deterministic");
    ok &= Expect(
        track && track->score > score_manager.GetScoreOrDefault(a),
        "fused score did not increase with independent support");

    CovisibilityDatabase covisibility_db;
    LoopDecisionManager decision_manager;
    FusedLandmarkManager decision_fused_manager;
    LoopVerificationResult verification;
    verification.query_kf_id = RawKeyFrameId{1, 0, 0};
    verification.candidate_seed_kf_id = RawKeyFrameId{2, 0, 0};
    verification.query_submap_id = RawSubmapId{1, 0};
    verification.candidate_submap_id = RawSubmapId{2, 0};
    verification.geometry_confirmed = true;
    verification.ransac_success = true;
    verification.ransac_inliers = 1;
    verification.loop_confidence = 0.9;
    verification.inlier_mappoint_pairs = {{a, b}};
    verification.decision = LoopGeometryDecision::FusionCandidate;
    const auto decision = decision_manager.Process(
        verification,
        99,
        raw_db,
        pose_store,
        score_manager,
        covisibility_db,
        decision_fused_manager);
    ok &= Expect(decision.handled, "fusion decision was not handled");
    ok &= Expect(decision.covisibility_edge_added,
                 "confirmed covisibility edge was not inserted");
    ok &= Expect(
        covisibility_db.HasConfirmedEdge(
            verification.query_kf_id,
            verification.candidate_seed_kf_id),
        "confirmed edge cannot be queried");

    GlobalMapBuilder builder;
    const auto build = builder.Build(
        raw_db,
        pose_store,
        score_manager,
        &fused_manager,
        0.0F);
    ok &= Expect(build.stats.fused_members_skipped == 4,
                 "raw fused members were not omitted");
    ok &= Expect(build.stats.fused_tracks_published == 1,
                 "one fused track was not published");
    ok &= Expect(build.stats.returned_points == 2,
                 "expected one fused and one raw point");

    uint64_t fused_points = 0;
    for (const auto& point : build.points)
    {
        if (point.is_fused)
        {
            ++fused_points;
            ok &= Expect(std::abs(point.x - 0.15) < 1e-9,
                         "fused position is not the mean of all members");
        }
    }
    ok &= Expect(fused_points == 1, "published cloud has wrong fused count");
    ok &= Expect(
        raw_db.GetDatabaseStats().mappoints == raw_stats_before.mappoints,
        "fusion modified RawMapDatabase");

    const RawMapDatabase state_snapshot = raw_db.CreateStateSnapshot();
    ok &= Expect(
        state_snapshot.GetDatabaseStats().mappoints ==
            raw_db.GetDatabaseStats().mappoints,
        "state snapshot lost current MapPoints");
    ok &= Expect(
        state_snapshot.GetDatabaseStats().journal_entries == 0,
        "state snapshot copied the raw journal");

    RawMapDatabase orphan_raw_db;
    GlobalPoseStore orphan_pose_store;
    LandmarkScoreManager orphan_score_manager;
    FusedLandmarkManager orphan_fused_manager;
    auto orphan_map = MakeMap(20, 1.0, 5U, false);
    orphan_map.mappoints.front().reference_keyframe_id = 999U;
    orphan_raw_db.InsertDelta(300, orphan_map);
    ok &= Expect(
        orphan_pose_store.AnchorSubmap(
            RawSubmapId{20, 0},
            Eigen::Matrix4d::Identity(),
            orphan_raw_db,
            "TEST_ORPHAN_ANCHOR").success,
        "orphan submap anchor failed");
    orphan_score_manager.ApplyOrbSlamQuality(
        RawMapPointId{20, 0, 10}, orphan_map.mappoints.front());
    const auto orphan_build = builder.Build(
        orphan_raw_db,
        orphan_pose_store,
        orphan_score_manager,
        &orphan_fused_manager,
        0.0F);
    ok &= Expect(
        orphan_build.stats.returned_points == 0,
        "MapPoint without world KeyFrame was published");
    ok &= Expect(
        orphan_build.stats.fallback_submap_points == 0,
        "rigid submap fallback was used");
    ok &= Expect(
        orphan_build.stats.server_corrected_missing_keyframe_skipped == 1,
        "MapPoint without world KeyFrame was not counted as skipped");

    RawMapDatabase overlap_raw_db;
    GlobalPoseStore overlap_pose_store;
    LandmarkScoreManager overlap_score_manager;
    CovisibilityDatabase overlap_covisibility_db;
    FusedLandmarkManager overlap_fused_manager;
    const auto overlap_query = MakeOverlapMap(10, 0.0, 0.0);
    const auto overlap_candidate = MakeOverlapMap(11, 0.03, -0.02);
    overlap_raw_db.InsertDelta(200, overlap_query);
    overlap_raw_db.InsertDelta(201, overlap_candidate);
    for (const auto& overlap_map :
         std::vector<orbslam3_msgs::msg::OrbMap>{
             overlap_query,
             overlap_candidate})
    {
        ok &= Expect(
            overlap_pose_store.AnchorSubmap(
                RawSubmapId{
                    overlap_map.drone_id,
                    overlap_map.map_epoch},
                Eigen::Matrix4d::Identity(),
                overlap_raw_db,
                "TEST_ALIGNED_OVERLAP").success,
            "aligned-overlap submap anchor failed");
        for (const auto& point : overlap_map.mappoints)
        {
            overlap_score_manager.ApplyOrbSlamQuality(
                RawMapPointId{
                    overlap_map.drone_id,
                    overlap_map.map_epoch,
                    point.id},
                point);
        }
    }

    SubcloudLoopVerifier overlap_verifier;
    const auto overlap_search =
        overlap_verifier.FindUnknownAlignedOverlaps(
            RawKeyFrameId{10, 0, 0},
            overlap_raw_db,
            overlap_pose_store,
            overlap_covisibility_db,
            &overlap_score_manager);
    ok &= Expect(
        overlap_search.confirmed.size() == 1,
        "strict distributed aligned overlap was not confirmed");
    ok &= Expect(
        overlap_search.expanded_matches >= 8,
        "aligned overlap did not expand valid matches");
    if (!overlap_search.confirmed.empty())
    {
        const auto overlap_decision = decision_manager.Process(
            overlap_search.confirmed.front(),
            202,
            overlap_raw_db,
            overlap_pose_store,
            overlap_score_manager,
            overlap_covisibility_db,
            overlap_fused_manager);
        ok &= Expect(
            overlap_decision.covisibility_edge_added,
            "aligned overlap did not insert covisibility first");
        ok &= Expect(
            overlap_decision.fusion.pairs_fused >= 8,
            "aligned overlap pairs were not fused");

        overlap_covisibility_db.Clear();
        auto descriptor_changed_candidate = overlap_candidate;
        descriptor_changed_candidate.map_sequence = 2;
        for (auto& point : descriptor_changed_candidate.mappoints)
        {
            point.descriptor.data.fill(255U);
        }
        for (auto& keypoint :
             descriptor_changed_candidate.keyframes.front().keypoints)
        {
            keypoint.descriptor.data.fill(255U);
        }
        overlap_raw_db.InsertFullSnapshot(
            203, descriptor_changed_candidate);
        const auto tracked_overlap_search =
            overlap_verifier.FindUnknownAlignedOverlaps(
                RawKeyFrameId{10, 0, 0},
                overlap_raw_db,
                overlap_pose_store,
                overlap_covisibility_db,
                &overlap_score_manager,
                &overlap_fused_manager);
        ok &= Expect(
            tracked_overlap_search.same_track_matches >= 8 &&
            tracked_overlap_search.confirmed.size() == 1,
            "known fused tracks did not bypass changed descriptors");
        if (!tracked_overlap_search.confirmed.empty())
        {
            ok &= Expect(
                tracked_overlap_search.confirmed.front()
                    .inlier_mappoint_pairs.empty() &&
                tracked_overlap_search.confirmed.front()
                    .shared_identity_matches >= 8,
                "known fused identities were incorrectly emitted for fusion");
        }
    }

    RawMapDatabase intra_raw_db;
    GlobalPoseStore intra_pose_store;
    LandmarkScoreManager intra_score_manager;
    CovisibilityDatabase intra_covisibility_db;
    FusedLandmarkManager intra_fused_manager;
    auto intra_map = MakeOverlapMap(12, 0.0, 0.0);
    auto second_keyframe = intra_map.keyframes.front();
    second_keyframe.id = 1;
    intra_map.keyframes.push_back(second_keyframe);
    intra_raw_db.InsertDelta(300, intra_map);
    ok &= Expect(
        intra_pose_store.AnchorSubmap(
            RawSubmapId{12, 0},
            Eigen::Matrix4d::Identity(),
            intra_raw_db,
            "TEST_INTRA_SHARED_RAW").success,
        "intra-submap anchor failed");
    for (const auto& point : intra_map.mappoints)
    {
        intra_score_manager.ApplyOrbSlamQuality(
            RawMapPointId{12, 0, point.id}, point);
    }
    const auto intra_search = overlap_verifier.FindUnknownAlignedOverlaps(
        RawKeyFrameId{12, 0, 0},
        intra_raw_db,
        intra_pose_store,
        intra_covisibility_db,
        &intra_score_manager);
    ok &= Expect(
        intra_search.confirmed.size() == 1 &&
        intra_search.shared_identity_matches >= 8,
        "distributed shared RawMapPointIds did not confirm intra-submap overlap");
    if (!intra_search.confirmed.empty())
    {
        const auto intra_decision = decision_manager.Process(
            intra_search.confirmed.front(),
            301,
            intra_raw_db,
            intra_pose_store,
            intra_score_manager,
            intra_covisibility_db,
            intra_fused_manager);
        ok &= Expect(
            intra_decision.covisibility_edge_added,
            "shared raw identity did not insert intra-submap covisibility");
        ok &= Expect(
            intra_fused_manager.GetStats().tracks == 0,
            "shared RawMapPointIds incorrectly created fused tracks");
        ok &= Expect(
            intra_search.confirmed.front().inlier_mappoint_pairs.empty(),
            "shared RawMapPointIds were emitted as distinct fusion pairs");
    }

    return ok ? 0 : 1;
}
