#include "orbslam3_multi/raw_map_database.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>

namespace
{

using orbslam3_msgs::msg::OrbKeyFrame;
using orbslam3_msgs::msg::OrbMap;
using orbslam3_msgs::msg::OrbMapPoint;
using orbslam3_multi::RawKeyFrameId;
using orbslam3_multi::RawMapDatabase;
using orbslam3_multi::RawMapPointId;

std::shared_ptr<OrbMap> MakeMap(uint32_t drone_id, uint64_t epoch)
{
  auto map = std::make_shared<OrbMap>();
  map->drone_id = drone_id;
  map->map_epoch = epoch;
  return map;
}

TEST(RawMapDatabase, ProducesPreciseChangesAndRevisions)
{
  RawMapDatabase database;
  auto first = MakeMap(1, 7);
  OrbKeyFrame keyframe;
  keyframe.id = 10;
  keyframe.mappoint_ids = {100};
  keyframe.connected_keyframe_ids = {11};
  keyframe.connected_keyframe_weights = {20};
  first->keyframes.push_back(keyframe);
  OrbMapPoint mappoint;
  mappoint.id = 100;
  first->mappoints.push_back(mappoint);

  const auto inserted = database.InsertDelta(1, first);
  ASSERT_TRUE(inserted.new_submap);
  ASSERT_TRUE(inserted.has_material_changes);
  ASSERT_EQ(inserted.new_keyframe_ids.size(), 1U);
  ASSERT_EQ(inserted.pose_changes.size(), 1U);
  EXPECT_EQ(
    inserted.pose_changes[0].kind,
    orbslam3_multi::RawKeyFramePoseChangeKind::New);
  ASSERT_EQ(inserted.new_mappoint_ids.size(), 1U);
  ASSERT_EQ(inserted.association_changes.size(), 1U);
  EXPECT_EQ(database.GetKeyFrameRevision({1, 7, 10}), 1U);

  const auto unchanged = database.InsertDelta(2, std::make_shared<OrbMap>(*first));
  EXPECT_FALSE(unchanged.has_material_changes);
  ASSERT_EQ(unchanged.unchanged_keyframe_ids.size(), 1U);
  ASSERT_EQ(unchanged.unchanged_mappoint_ids.size(), 1U);
  EXPECT_EQ(database.GetKeyFrameRevision({1, 7, 10}), 1U);

  auto update = MakeMap(1, 7);
  keyframe.mappoint_ids = {101};
  keyframe.connected_keyframe_weights = {30};
  update->keyframes.push_back(keyframe);
  OrbMapPoint replacement;
  replacement.id = 101;
  update->mappoints.push_back(replacement);
  const auto changed = database.InsertDelta(3, update);

  ASSERT_TRUE(changed.has_material_changes);
  ASSERT_EQ(changed.updated_keyframe_ids, std::vector<RawKeyFrameId>({{1, 7, 10}}));
  ASSERT_EQ(changed.new_mappoint_ids, std::vector<RawMapPointId>({{1, 7, 101}}));
  ASSERT_EQ(changed.covisibility_changed_keyframe_ids.size(), 1U);
  ASSERT_EQ(changed.association_changes.size(), 1U);
  EXPECT_TRUE(changed.pose_changes.empty());
  EXPECT_EQ(changed.association_changes[0].added_mappoint_ids[0].local_mp_id, 101U);
  EXPECT_EQ(changed.association_changes[0].removed_mappoint_ids[0].local_mp_id, 100U);
  EXPECT_EQ(database.GetKeyFrameRevision({1, 7, 10}), 2U);

  auto pose_update = MakeMap(1, 7);
  keyframe.pose.position.x = 2.5;
  pose_update->keyframes.push_back(keyframe);
  const auto pose_changed = database.InsertDelta(4, pose_update);
  ASSERT_EQ(pose_changed.pose_changes.size(), 1U);
  EXPECT_EQ(
    pose_changed.pose_changes[0].kind,
    orbslam3_multi::RawKeyFramePoseChangeKind::PoseUpdated);
  EXPECT_DOUBLE_EQ(pose_changed.pose_changes[0].keyframe.local_pose.position.x, 2.5);

  auto invalidation = MakeMap(1, 7);
  keyframe.is_bad = true;
  invalidation->keyframes.push_back(keyframe);
  const auto invalidated = database.InsertDelta(5, invalidation);
  ASSERT_EQ(invalidated.pose_changes.size(), 1U);
  EXPECT_EQ(
    invalidated.pose_changes[0].kind,
    orbslam3_multi::RawKeyFramePoseChangeKind::Invalidated);
  EXPECT_FALSE(invalidated.pose_changes[0].keyframe.active);
}

TEST(RawMapDatabase, ReturnsBoundedPoseSnapshot)
{
  RawMapDatabase database;
  auto map = MakeMap(2, 9);
  auto & first = map->keyframes.emplace_back();
  first.id = 3;
  first.pose.orientation.w = 1.0;
  auto & second = map->keyframes.emplace_back();
  second.id = 7;
  second.pose.orientation.w = 1.0;
  second.is_bad = true;
  const auto inserted = database.InsertDelta(1, map);

  const auto snapshot = database.GetSubmapPoseSnapshot({2, 9});
  ASSERT_TRUE(snapshot.has_value());
  EXPECT_EQ(snapshot->submap_revision, inserted.submap_revision);
  ASSERT_EQ(snapshot->keyframes.size(), 2U);
  EXPECT_EQ(snapshot->keyframes[0].id, RawKeyFrameId({2, 9, 3}));
  EXPECT_TRUE(snapshot->keyframes[0].active);
  EXPECT_FALSE(snapshot->keyframes[1].active);
  EXPECT_FALSE(database.GetSubmapPoseSnapshot({2, 10}).has_value());
}

TEST(RawMapDatabase, AssociationDiffIsSortedForUnorderedInputs)
{
  RawMapDatabase database;
  auto initial = MakeMap(1, 3);
  auto & keyframe = initial->keyframes.emplace_back();
  keyframe.id = 8;
  keyframe.mappoint_ids = {103, 100, 102};
  database.InsertDelta(1, initial);

  auto update = MakeMap(1, 3);
  keyframe.mappoint_ids = {104, 102, 101};
  update->keyframes.push_back(keyframe);
  const auto changed = database.InsertDelta(2, update);

  ASSERT_EQ(changed.association_changes.size(), 1U);
  EXPECT_EQ(
    changed.association_changes[0].added_mappoint_ids,
    std::vector<RawMapPointId>({{1, 3, 101}, {1, 3, 104}}));
  EXPECT_EQ(
    changed.association_changes[0].removed_mappoint_ids,
    std::vector<RawMapPointId>({{1, 3, 100}, {1, 3, 103}}));
}

TEST(RawMapDatabase, KeepsDroneAndEpochIsolated)
{
  RawMapDatabase database;
  auto first = MakeMap(1, 1);
  first->keyframes.emplace_back().id = 5;
  auto second = MakeMap(1, 2);
  second->keyframes.emplace_back().id = 5;
  database.InsertDelta(1, first);
  database.InsertDelta(2, second);

  EXPECT_TRUE(database.GetKeyFrame({1, 1, 5}).has_value());
  EXPECT_TRUE(database.GetKeyFrame({1, 2, 5}).has_value());
  EXPECT_EQ(database.GetStats().submaps, 2U);
}

TEST(RawMapDatabase, SavesLoadsAndReplaysTheSameJournal)
{
  RawMapDatabase database;
  auto first = MakeMap(1, 0);
  first->keyframes.emplace_back().id = 1;
  auto second = MakeMap(2, 0);
  second->mappoints.emplace_back().id = 8;
  database.InsertDelta(1, first);
  database.InsertDelta(2, second);

  const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto path = std::filesystem::temp_directory_path() /
    ("f3c_raw_map_database_" + std::to_string(nonce) + ".record");
  std::string error;
  ASSERT_TRUE(database.SaveToPath(path.string(), &error)) << error;

  std::vector<orbslam3_multi::RawJournalEntry> entries;
  ASSERT_TRUE(RawMapDatabase::LoadRecord(path.string(), &entries, &error)) << error;
  ASSERT_EQ(entries.size(), 2U);
  EXPECT_EQ(entries[0].arrival_id, 1U);
  EXPECT_EQ(entries[1].arrival_id, 2U);

  RawMapDatabase replayed;
  for (const auto & entry : entries) {
    replayed.InsertDelta(entry.arrival_id, entry.map);
  }
  EXPECT_EQ(replayed.GetStats(), database.GetStats());
  std::filesystem::remove(path);
}

TEST(RawMapDatabase, PersistsNormalizedFiducialObservationJournal)
{
  RawMapDatabase database;
  auto map = MakeMap(1, 3);
  auto & keyframe = map->keyframes.emplace_back();
  keyframe.id = 8;
  keyframe.pose.orientation.w = 1.0;
  database.InsertDelta(1, map);

  orbslam3_multi::RecordedFiducialObservation observation;
  observation.arrival_id = 1;
  observation.keyframe_id = {1, 3, 8};
  observation.fiducial_id = 2;
  observation.fiducial_visit_id = 17;
  observation.world_T_camera_target.orientation.w = 1.0;
  observation.world_T_camera_target.position.y = -9.0;
  observation.keyframe_stamp_sec = 10.0;
  observation.observation_stamp_sec = 10.1;
  observation.association_dt_sec = 0.1;
  observation.source = "simulated_gt";
  observation.quality = "ok";
  database.AddFiducialObservation(observation);

  const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto path = std::filesystem::temp_directory_path() /
    ("f3e_raw_map_database_" + std::to_string(nonce) + ".record");
  std::string error;
  ASSERT_TRUE(database.SaveToPath(path.string(), &error)) << error;

  std::vector<orbslam3_multi::RawJournalEntry> entries;
  std::vector<orbslam3_multi::RecordedFiducialObservation> observations;
  ASSERT_TRUE(
    RawMapDatabase::LoadRecord(path.string(), &entries, &observations, &error)) << error;
  ASSERT_EQ(entries.size(), 1U);
  ASSERT_EQ(observations.size(), 1U);
  EXPECT_EQ(observations.front().keyframe_id, observation.keyframe_id);
  EXPECT_EQ(observations.front().fiducial_id, 2);
  EXPECT_EQ(observations.front().source, "simulated_gt");
  EXPECT_NEAR(observations.front().association_dt_sec, 0.1, 1e-12);
  std::filesystem::remove(path);
}

TEST(RawMapDatabase, StreamsVersionTwoRecordWithoutResidentDeltaJournal)
{
  RawMapDatabase database;
  const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto path = std::filesystem::temp_directory_path() /
    ("f3g_incremental_raw_" + std::to_string(nonce) + ".record");
  const auto temporary_path = std::filesystem::path(path.string() + ".in_progress");
  {
    std::ofstream previous(path, std::ios::binary | std::ios::trunc);
    previous << "previous-valid-record-placeholder";
  }
  const auto previous_size = std::filesystem::file_size(path);
  std::string error;
  ASSERT_TRUE(database.StartIncrementalRecord(path.string(), &error)) << error;
  EXPECT_EQ(std::filesystem::file_size(path), previous_size);
  EXPECT_TRUE(std::filesystem::exists(temporary_path));

  auto first = MakeMap(1, 2);
  first->keyframes.emplace_back().id = 4;
  auto second = MakeMap(2, 5);
  second->mappoints.emplace_back().id = 9;
  database.InsertDelta(1, first);
  database.InsertDelta(2, second);

  orbslam3_multi::RecordedFiducialObservation observation;
  observation.arrival_id = 1;
  observation.keyframe_id = {1, 2, 4};
  observation.fiducial_id = 2;
  observation.fiducial_visit_id = 23;
  observation.world_T_camera_target.orientation.w = 1.0;
  observation.source = "simulated_gt";
  observation.quality = "ok";
  database.AddFiducialObservation(observation);

  const auto active = database.GetJournalStorageStats();
  EXPECT_EQ(active.logical_entries, 2U);
  EXPECT_EQ(active.resident_entries, 0U);
  EXPECT_GT(active.record_bytes_written, 16U);
  EXPECT_TRUE(active.incremental_record_active);
  EXPECT_TRUE(active.incremental_record_healthy);
  EXPECT_TRUE(database.GetJournalView().empty());

  ASSERT_TRUE(database.FinalizeIncrementalRecord(&error)) << error;
  EXPECT_TRUE(std::filesystem::exists(path));
  EXPECT_FALSE(std::filesystem::exists(temporary_path));

  std::vector<orbslam3_multi::RawJournalEntry> entries;
  std::vector<orbslam3_multi::RecordedFiducialObservation> observations;
  ASSERT_TRUE(
    RawMapDatabase::LoadRecord(path.string(), &entries, &observations, &error)) << error;
  ASSERT_EQ(entries.size(), 2U);
  EXPECT_EQ(entries[0].arrival_id, 1U);
  EXPECT_EQ(entries[0].map->keyframes.size(), 1U);
  EXPECT_EQ(entries[1].arrival_id, 2U);
  EXPECT_EQ(entries[1].map->mappoints.size(), 1U);
  ASSERT_EQ(observations.size(), 1U);
  EXPECT_EQ(observations[0].keyframe_id, observation.keyframe_id);

  orbslam3_multi::RawRecordMetadata metadata;
  ASSERT_TRUE(RawMapDatabase::ReadRecordMetadata(path.string(), &metadata, &error)) << error;
  EXPECT_EQ(metadata.version, 3U);
  EXPECT_EQ(metadata.entry_count, 2U);
  ASSERT_EQ(metadata.fiducial_observations.size(), 1U);
  std::vector<uint64_t> streamed_arrivals;
  ASSERT_TRUE(RawMapDatabase::StreamRecordEntries(
      path.string(),
      [&streamed_arrivals](orbslam3_multi::RawJournalEntry && entry) {
        streamed_arrivals.push_back(entry.arrival_id);
        return true;
      }, &error)) << error;
  EXPECT_EQ(streamed_arrivals, (std::vector<uint64_t>{1U, 2U}));
  std::filesystem::remove(path);
}

TEST(RawMapDatabase, DisabledJournalCountsEntriesWithoutRetainingMessages)
{
  RawMapDatabase database;
  database.DisableJournalRetention();
  database.InsertDelta(1, MakeMap(1, 0));

  EXPECT_EQ(database.GetStats().journal_entries, 1U);
  const auto storage = database.GetJournalStorageStats();
  EXPECT_EQ(storage.logical_entries, 1U);
  EXPECT_EQ(storage.resident_entries, 0U);
  EXPECT_FALSE(storage.incremental_record_active);
  EXPECT_TRUE(database.GetJournalView().empty());

  const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto path = std::filesystem::temp_directory_path() /
    ("f3g_disabled_raw_" + std::to_string(nonce) + ".record");
  std::string error;
  EXPECT_FALSE(database.SaveToPath(path.string(), &error));
  EXPECT_FALSE(error.empty());
  EXPECT_FALSE(std::filesystem::exists(path));
}

TEST(RawMapDatabase, RejectsInvalidIdentityAndArrivalOrder)
{
  RawMapDatabase database;
  EXPECT_THROW(database.InsertDelta(1, MakeMap(0, 0)), std::invalid_argument);
  database.InsertDelta(2, MakeMap(1, 0));
  EXPECT_THROW(database.InsertDelta(2, MakeMap(1, 0)), std::invalid_argument);
}

TEST(RawMapDatabase, FullSnapshotIsSelectiveInvalidatesAbsentAndRecordsNormalizedDelta)
{
  RawMapDatabase database;
  auto initial = MakeMap(1, 4);
  auto & keyframe = initial->keyframes.emplace_back();
  keyframe.id = 7;
  keyframe.pose.orientation.w = 1.0;
  keyframe.mappoint_ids = {10, 11};
  initial->mappoints.reserve(2);
  auto & first_point = initial->mappoints.emplace_back();
  first_point.id = 10;
  first_point.position.x = 1.0;
  first_point.observations_count = 2;
  auto & missing_point = initial->mappoints.emplace_back();
  missing_point.id = 11;
  missing_point.position.x = 2.0;
  database.InsertDelta(1, initial);

  const auto noop = database.InsertFullSnapshot(2, std::make_shared<OrbMap>(*initial));
  EXPECT_FALSE(noop.has_material_changes);
  EXPECT_FALSE(noop.journal_entry_appended);
  EXPECT_EQ(database.GetJournalView().size(), 1U);

  auto snapshot = MakeMap(1, 4);
  snapshot->header.stamp.sec = 123;
  snapshot->header.stamp.nanosec = 456U;
  snapshot->header.frame_id = "orb_snapshot";
  snapshot->drone_name = "dron_1";
  snapshot->map_frame = "orb_map_1";
  snapshot->map_sequence = 99U;
  snapshot->fx = 500.0F;
  snapshot->fy = 501.0F;
  snapshot->cx = 320.0F;
  snapshot->cy = 240.0F;
  snapshot->bf = 40.0F;
  snapshot->image_width = 640U;
  snapshot->image_height = 480U;
  auto changed_keyframe = keyframe;
  changed_keyframe.pose.position.x = 3.0;
  changed_keyframe.mappoint_ids = {10, 12};
  snapshot->keyframes.push_back(changed_keyframe);
  auto changed_point = first_point;
  changed_point.position.x = 4.0;
  changed_point.observations_count = 8;
  snapshot->mappoints.push_back(changed_point);
  OrbMapPoint new_point;
  new_point.id = 12;
  new_point.position.x = 5.0;
  snapshot->mappoints.push_back(new_point);

  const auto changed = database.InsertFullSnapshot(3, snapshot);
  EXPECT_TRUE(changed.has_material_changes);
  EXPECT_TRUE(changed.normalized_delta_appended);
  EXPECT_EQ(changed.pose_changed_keyframe_ids, std::vector<RawKeyFrameId>({{1, 4, 7}}));
  EXPECT_EQ(
    changed.association_changed_keyframe_ids,
    std::vector<RawKeyFrameId>({{1, 4, 7}}));
  EXPECT_EQ(changed.new_mappoint_ids, std::vector<RawMapPointId>({{1, 4, 12}}));
  EXPECT_NE(
    std::find(
      changed.geometry_changed_mappoint_ids.begin(),
      changed.geometry_changed_mappoint_ids.end(), RawMapPointId{1, 4, 10}),
    changed.geometry_changed_mappoint_ids.end());
  EXPECT_NE(
    std::find(
      changed.score_input_changed_mappoint_ids.begin(),
      changed.score_input_changed_mappoint_ids.end(), RawMapPointId{1, 4, 10}),
    changed.score_input_changed_mappoint_ids.end());
  EXPECT_EQ(
    changed.invalidated_mappoint_ids, std::vector<RawMapPointId>({{1, 4, 11}}));
  ASSERT_TRUE(database.GetMapPoint({1, 4, 11}).has_value());
  EXPECT_TRUE(database.GetMapPoint({1, 4, 11})->is_bad);

  const auto journal = database.GetJournalView();
  ASSERT_EQ(journal.size(), 2U);
  EXPECT_EQ(journal[1].arrival_id, 3U);
  ASSERT_EQ(journal[1].map->keyframes.size(), 1U);
  ASSERT_EQ(journal[1].map->mappoints.size(), 3U);
  EXPECT_EQ(journal[1].map->header, snapshot->header);
  EXPECT_EQ(journal[1].map->drone_id, snapshot->drone_id);
  EXPECT_EQ(journal[1].map->drone_name, snapshot->drone_name);
  EXPECT_EQ(journal[1].map->map_frame, snapshot->map_frame);
  EXPECT_EQ(journal[1].map->map_sequence, snapshot->map_sequence);
  EXPECT_EQ(journal[1].map->map_epoch, snapshot->map_epoch);
  EXPECT_EQ(journal[1].map->fx, snapshot->fx);
  EXPECT_EQ(journal[1].map->fy, snapshot->fy);
  EXPECT_EQ(journal[1].map->cx, snapshot->cx);
  EXPECT_EQ(journal[1].map->cy, snapshot->cy);
  EXPECT_EQ(journal[1].map->bf, snapshot->bf);
  EXPECT_EQ(journal[1].map->image_width, snapshot->image_width);
  EXPECT_EQ(journal[1].map->image_height, snapshot->image_height);

  RawMapDatabase replayed;
  for (const auto & entry : journal) {
    replayed.InsertDelta(entry.arrival_id, entry.map);
  }
  EXPECT_EQ(replayed.GetStats(), database.GetStats());
  EXPECT_EQ(replayed.GetKeyFrame({1, 4, 7}), database.GetKeyFrame({1, 4, 7}));
  EXPECT_EQ(replayed.GetMapPoint({1, 4, 10}), database.GetMapPoint({1, 4, 10}));
  EXPECT_EQ(replayed.GetMapPoint({1, 4, 11}), database.GetMapPoint({1, 4, 11}));
  EXPECT_EQ(replayed.GetMapPoint({1, 4, 12}), database.GetMapPoint({1, 4, 12}));
}

}  // namespace
