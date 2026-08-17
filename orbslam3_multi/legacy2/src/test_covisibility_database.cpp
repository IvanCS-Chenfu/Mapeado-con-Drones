#include "orbslam3_multi/covisibility_database.hpp"

#include <Eigen/Core>

#include <cmath>
#include <iostream>

namespace
{

geometry_msgs::msg::Pose MakePose(double x, double y, double z)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.w = 1.0;
    return pose;
}

orbslam3_msgs::msg::OrbKeyFrame MakeKeyFrame(uint64_t id, double x)
{
    orbslam3_msgs::msg::OrbKeyFrame keyframe;
    keyframe.id = id;
    keyframe.pose = MakePose(x, 0.0, 0.0);
    return keyframe;
}

bool Near(double a, double b)
{
    return std::abs(a - b) < 1e-9;
}

bool Expect(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "[test_covisibility_database] " << message << '\n';
        return false;
    }
    return true;
}

}  // namespace

int main()
{
    using namespace orbslam3_multi;

    RawMapDatabase raw_db;
    orbslam3_msgs::msg::OrbMap map;
    map.drone_id = 1;
    map.drone_name = "dron_1";
    map.map_epoch = 0;
    map.map_sequence = 0;
    map.map_frame = "dron_1_orb_map";

    auto kf9 = MakeKeyFrame(9, 9.0);
    auto kf10 = MakeKeyFrame(10, 10.0);
    kf10.connected_keyframe_ids.push_back(9);
    kf10.connected_keyframe_weights.push_back(20);
    map.keyframes.push_back(kf9);
    map.keyframes.push_back(kf10);

    raw_db.InsertDelta(42, map);

    CovisibilityDatabase db;
    const auto import_result = db.ImportOrbslam3Native(raw_db, 42, 15.0);
    bool ok = true;
    ok &= Expect(import_result.keyframes_examined == 2, "expected two examined keyframes");
    ok &= Expect(import_result.connections_examined == 1, "expected one examined connection");
    ok &= Expect(import_result.edges_added == 1, "expected one native edge");

    const RawKeyFrameId id9{1, 0, 9};
    const RawKeyFrameId id10{1, 0, 10};
    ok &= Expect(db.HasConfirmedEdge(id9, id10), "canonical edge missing");
    ok &= Expect(db.HasConfirmedEdge(id10, id9), "reverse lookup missing");

    const auto imported = db.GetEdge(id10, id9);
    ok &= Expect(imported.has_value(), "imported edge not returned");
    if (imported)
    {
        ok &= Expect(imported->kf_a == id9, "edge kf_a not canonical");
        ok &= Expect(imported->kf_b == id10, "edge kf_b not canonical");
        ok &= Expect(Near(imported->relative_pose_measured(0, 3), 1.0),
                     "canonical measured pose has wrong x translation");
        ok &= Expect(Near(imported->relative_pose_current(0, 3), 1.0),
                     "canonical current pose has wrong x translation");
    }

    auto kf11 = MakeKeyFrame(11, 11.0);
    kf11.connected_keyframe_ids.push_back(10);
    kf11.connected_keyframe_weights.push_back(1);
    orbslam3_msgs::msg::OrbMap low_weight_map = map;
    low_weight_map.map_sequence = 1;
    low_weight_map.keyframes = {kf11};
    raw_db.InsertDelta(43, low_weight_map);
    const auto low_weight_import =
        db.ImportOrbslam3Native(raw_db, 43, 0.0);
    const RawKeyFrameId id11{1, 0, 11};
    ok &= Expect(
        low_weight_import.edges_added == 1,
        "positive low-weight native edge was not imported");
    ok &= Expect(
        db.HasConfirmedEdge(id10, id11),
        "low-weight native edge is not confirmed");
    ok &= Expect(
        !db.HasStrongEdge(id10, id11, CovisibilityStrengthConfig{}),
        "single-point native edge was incorrectly classified as strong");

    CovisibilityEdge loop_edge;
    loop_edge.kf_a = RawKeyFrameId{2, 0, 21};
    loop_edge.kf_b = RawKeyFrameId{2, 0, 20};
    loop_edge.weight = 30.0;
    loop_edge.source = CovisibilityEdgeSource::ServerLoopGeometric;
    loop_edge.relative_pose_measured = Eigen::Matrix4d::Identity();
    loop_edge.relative_pose_measured(0, 3) = -2.0;
    loop_edge.relative_pose_current = loop_edge.relative_pose_measured;
    loop_edge.information_weight = 30.0;
    loop_edge.shared_mappoints_or_inliers = 30;
    loop_edge.shared_mappoint_ratio = 0.5;
    loop_edge.image_coverage_bins = 4;
    loop_edge.spatial_coverage_ratio = 0.8;
    loop_edge.geometry_confirmed = true;
    loop_edge.created_arrival_id = 44;

    bool added = false;
    ok &= Expect(db.AddConfirmedLoopEdge(loop_edge, &added), "synthetic loop edge rejected");
    ok &= Expect(added, "synthetic loop edge not marked as added");

    const RawKeyFrameId id20{2, 0, 20};
    const RawKeyFrameId id21{2, 0, 21};
    const auto synthetic = db.GetEdge(id20, id21);
    ok &= Expect(synthetic.has_value(), "synthetic edge not returned");
    if (synthetic)
    {
        ok &= Expect(synthetic->source == CovisibilityEdgeSource::ServerLoopGeometric,
                     "synthetic source changed");
        ok &= Expect(Near(synthetic->relative_pose_measured(0, 3), 2.0),
                     "synthetic measured pose not canonicalized");
    }
    ok &= Expect(
        db.HasStrongEdge(id20, id21, CovisibilityStrengthConfig{}),
        "distributed geometric edge was not classified as strong");

    Eigen::Matrix4d updated_current = Eigen::Matrix4d::Identity();
    updated_current(0, 3) = -3.0;
    ok &= Expect(db.UpdateRelativePoseCurrent(id21, id20, updated_current, 100),
                 "reverse current-pose update failed");
    const auto updated = db.GetEdge(id20, id21);
    if (updated)
    {
        ok &= Expect(Near(updated->relative_pose_measured(0, 3), 2.0),
                     "measured pose was overwritten by current update");
        ok &= Expect(Near(updated->relative_pose_current(0, 3), 3.0),
                     "current pose was not canonicalized after update");
    }

    const auto stats = db.GetStats();
    ok &= Expect(stats.confirmed_edges == 3, "unexpected confirmed edge count");
    ok &= Expect(stats.orbslam3_native_edges == 2, "unexpected native edge count");
    ok &= Expect(stats.server_loop_geometric_edges == 1, "unexpected server loop edge count");

    RawMapDatabase incremental_raw_db;
    orbslam3_msgs::msg::OrbMap incremental_map;
    incremental_map.drone_id = 5;
    incremental_map.map_epoch = 2;
    incremental_map.map_sequence = 0;
    auto incremental_kf = MakeKeyFrame(7, 0.0);
    incremental_kf.mappoint_ids = {10, 20};
    incremental_map.keyframes = {incremental_kf};
    const auto first_insert =
        incremental_raw_db.InsertDelta(1, incremental_map);
    ok &= Expect(
        first_insert.keyframe_mappoint_deltas.size() == 1 &&
        first_insert.added_keyframe_mappoint_associations == 2,
        "new KF did not expose all MapPoint associations");

    incremental_map.map_sequence = 1;
    const auto unchanged_insert =
        incremental_raw_db.InsertFullSnapshot(2, incremental_map);
    ok &= Expect(
        unchanged_insert.keyframe_mappoint_deltas.empty() &&
        unchanged_insert.keyframes_without_mappoint_changes == 1 &&
        unchanged_insert.updated_keyframes == 0 &&
        unchanged_insert.unchanged_keyframes == 1 &&
        !unchanged_insert.has_material_changes,
        "unchanged snapshot produced incremental work");

    incremental_map.map_sequence = 2;
    incremental_map.keyframes.front().mappoint_ids.push_back(30);
    const auto changed_insert =
        incremental_raw_db.InsertFullSnapshot(3, incremental_map);
    ok &= Expect(
        changed_insert.keyframe_mappoint_deltas.size() == 1 &&
        changed_insert.keyframe_mappoint_deltas.front()
                .added_mappoint_ids.size() == 1 &&
        changed_insert.keyframe_mappoint_deltas.front()
                .added_mappoint_ids.front() == RawMapPointId{5, 2, 30},
        "snapshot did not expose only the new MapPoint association");
    ok &= Expect(
        changed_insert.has_material_changes &&
        changed_insert.updated_keyframes == 1 &&
        incremental_raw_db.GetKeyFrameRevision(RawKeyFrameId{5, 2, 7})
                .associations >
            first_insert.material_revision,
        "material association revision did not advance");

    RawMapDatabase staged_raw_db;
    orbslam3_msgs::msg::OrbMap staged_map;
    staged_map.drone_id = 6;
    staged_map.map_epoch = 0;
    auto observer = MakeKeyFrame(1, 0.0);
    observer.mappoint_ids = {100};
    auto unrelated = MakeKeyFrame(2, 1.0);
    unrelated.mappoint_ids = {200};
    staged_map.keyframes = {observer, unrelated};
    orbslam3_msgs::msg::OrbMapPoint observed_point;
    observed_point.id = 100;
    observed_point.reference_keyframe_id = 1;
    observed_point.position.x = 0.5;
    observed_point.descriptor.data.fill(1U);
    observed_point.observations_count = 3;
    observed_point.found_ratio = 0.8F;
    staged_map.mappoints = {observed_point};
    staged_raw_db.InsertDelta(10, staged_map);
    const RawKeyFrameId observer_id{6, 0, 1};
    const RawKeyFrameId unrelated_id{6, 0, 2};
    const auto observer_before =
        staged_raw_db.GetKeyFrameRevision(observer_id);
    const auto unrelated_before =
        staged_raw_db.GetKeyFrameRevision(unrelated_id);

    staged_map.keyframes.clear();
    staged_map.mappoints.front().found_ratio = 0.9F;
    const auto metadata_insert =
        staged_raw_db.InsertDelta(11, staged_map);
    ok &= Expect(
        metadata_insert.has_metadata_changes &&
        !metadata_insert.has_loop_material_changes &&
        SameGeometryRevision(
            staged_raw_db.GetKeyFrameRevision(observer_id),
            observer_before),
        "MapPoint metadata incorrectly invalidated geometry");

    staged_map.mappoints.front().position.x = 0.7;
    const auto geometry_insert =
        staged_raw_db.InsertDelta(12, staged_map);
    ok &= Expect(
        geometry_insert.has_loop_material_changes &&
        !SameGeometryRevision(
            staged_raw_db.GetKeyFrameRevision(observer_id),
            observer_before),
        "observing KF geometry was not invalidated");
    ok &= Expect(
        SameGeometryRevision(
            staged_raw_db.GetKeyFrameRevision(unrelated_id),
            unrelated_before),
        "MapPoint geometry invalidated an unrelated KF");

    return ok ? 0 : 1;
}
