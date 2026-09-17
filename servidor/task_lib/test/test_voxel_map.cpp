#include "task_lib/voxel_map.hpp"

#include <gtest/gtest.h>

TEST(ReversibleVoxelMap, SnapshotDiffRemovesMovedAndDeletedSources)
{
  task_lib::ReversibleVoxelMap map(0.5);
  EXPECT_TRUE(map.ApplySparseSnapshot({{"1:0:10", {0.1, 0.1, 0.1}, 0.8F}}));
  ASSERT_EQ(map.Snapshot().size(), 1U);
  EXPECT_TRUE(map.ApplySparseSnapshot({{"1:0:10", {1.1, 0.1, 0.1}, 0.8F}}));
  const auto moved = map.Snapshot();
  ASSERT_EQ(moved.size(), 1U);
  EXPECT_EQ(moved.front().key.ix, 2);
  EXPECT_TRUE(map.ApplySparseSnapshot({}));
  EXPECT_TRUE(map.Snapshot().empty());
}

TEST(ReversibleVoxelMap, RetainsSparseScoreBelowOccupiedThreshold)
{
  task_lib::ReversibleVoxelMap map(1.0);
  EXPECT_TRUE(
    map.ApplySparseSnapshot(
      {{"1:0:low", {0.2, 0.2, 0.2}, 0.19F}, {"1:0:edge", {1.2, 0.2, 0.2}, 0.2F}}, 0.4F));
  const auto snapshot = map.Snapshot();
  ASSERT_EQ(snapshot.size(), 2U);
  EXPECT_EQ(map.StateAt({0, 0, 0}), task_lib::VoxelState::Unknown);
  EXPECT_EQ(map.StateAt({1, 0, 0}), task_lib::VoxelState::Unknown);
}

TEST(ReversibleVoxelMap, AveragesDistinctMapPointScoresReversibly)
{
  task_lib::ReversibleVoxelMap map(1.0);
  const std::vector<task_lib::SparseEvidence> first_three{
    {"1:0:1", {0.2, 0.2, 0.2}, 0.2F},
    {"1:0:2", {0.3, 0.3, 0.3}, 0.6F},
    {"1:0:3", {0.4, 0.4, 0.4}, 0.9F}};
  EXPECT_TRUE(map.ApplySparseSnapshot(first_three, 0.4F));
  ASSERT_EQ(map.Snapshot().size(), 1U);
  EXPECT_EQ(map.Snapshot().front().state, task_lib::VoxelState::Occupied);
  EXPECT_NEAR(map.Snapshot().front().score, (0.2F + 0.6F + 0.9F) / 3.0F, 1e-6F);

  EXPECT_TRUE(
    map.ApplySparseDelta(
      {{"1:0:4", {0.5, 0.5, 0.5}, 0.1F}}, {}, 0.4F));
  ASSERT_EQ(map.Snapshot().size(), 1U);
  EXPECT_EQ(map.Snapshot().front().state, task_lib::VoxelState::Occupied);

  EXPECT_TRUE(
    map.ApplySparseDelta(
      {{"1:0:4", {1.5, 0.5, 0.5}, 0.1F}}, {}, 0.4F));
  EXPECT_EQ(map.StateAt({0, 0, 0}), task_lib::VoxelState::Occupied);
  EXPECT_EQ(map.StateAt({1, 0, 0}), task_lib::VoxelState::Unknown);
}

TEST(ReversibleVoxelMap, TraversedFreeEvidenceOverridesWeakSparseOccupancy)
{
  task_lib::ReversibleVoxelMap map(1.0);
  EXPECT_TRUE(map.AddFreeEvidence("trajectory:1", {0.2, 0.2, 0.2}));
  ASSERT_EQ(map.Snapshot().size(), 1U);
  EXPECT_EQ(map.Snapshot().front().state, task_lib::VoxelState::Free);
  EXPECT_TRUE(map.ApplySparseSnapshot({{"1:0:11", {0.2, 0.2, 0.2}, 1.0F}}));
  ASSERT_EQ(map.Snapshot().size(), 1U);
  EXPECT_EQ(map.Snapshot().front().state, task_lib::VoxelState::Free);
  EXPECT_TRUE(map.RemoveEvidence("trajectory:1"));
  ASSERT_EQ(map.Snapshot().size(), 1U);
  EXPECT_EQ(map.Snapshot().front().state, task_lib::VoxelState::Occupied);
}

TEST(ReversibleVoxelMap, DirectDepthEvidenceOverridesSparseAndRestoresItWhenRemoved)
{
  task_lib::ReversibleVoxelMap map(1.0);
  EXPECT_TRUE(map.ApplySparseSnapshot({{"1:0:7", {0.2, 0.2, 0.2}, 1.0F}}));
  EXPECT_EQ(map.StateAt({0, 0, 0}), task_lib::VoxelState::Occupied);
  EXPECT_TRUE(map.ReplaceDirectDepthFreeCells("depth_free:1", {{0, 0, 0}}));
  EXPECT_EQ(map.StateAt({0, 0, 0}), task_lib::VoxelState::Free);
  EXPECT_TRUE(map.ReplaceDepthOccupiedCells("depth_occupied:1", {{0, 0, 0}}));
  EXPECT_EQ(map.StateAt({0, 0, 0}), task_lib::VoxelState::Occupied);
  EXPECT_TRUE(map.RemoveEvidence("depth_occupied:1"));
  EXPECT_EQ(map.StateAt({0, 0, 0}), task_lib::VoxelState::Free);
  EXPECT_TRUE(map.RemoveEvidence("depth_free:1"));
  EXPECT_EQ(map.StateAt({0, 0, 0}), task_lib::VoxelState::Occupied);
}

TEST(ReversibleVoxelMap, ReplacesTheEntirePhysicalFreeVolumeReversibly)
{
  task_lib::ReversibleVoxelMap map(1.0);
  ASSERT_TRUE(map.ReplaceFreeVolume("trajectory:1:10", {0.5, 0.5, 0.5}, {0.6, 0.1, 0.1}));
  ASSERT_EQ(map.Snapshot().size(), 3U);
  ASSERT_TRUE(map.ReplaceFreeVolume("trajectory:1:10", {3.5, 0.5, 0.5}, {0.1, 0.1, 0.1}));
  const auto snapshot = map.Snapshot();
  ASSERT_EQ(snapshot.size(), 1U);
  EXPECT_EQ(snapshot.front().key.ix, 3);
  EXPECT_EQ(snapshot.front().state, task_lib::VoxelState::Free);
}

TEST(ReversibleVoxelMap, UpdatesOnlyTheAffectedNavigationProfileCells)
{
  task_lib::ReversibleVoxelMap map(1.0);
  ASSERT_TRUE(
    map.RegisterNavigationProfile(
      {"compact", {1, 1, 0}, 2.0}));
  ASSERT_TRUE(map.ApplySparseSnapshot({{"1:0:12", {2.2, 2.2, 0.2}, 1.0F}}));
  const auto update = map.RefreshNavigation("compact", map.TakeChanges());
  EXPECT_EQ(update.raw_revision, map.revision());
  EXPECT_EQ(update.recomputed_cells, 75U);
  EXPECT_FALSE(update.changes.empty());

  const auto snapshot = map.NavigationSnapshotFor("compact");
  const auto blocked = std::find_if(
    snapshot.cells.begin(), snapshot.cells.end(), [](const auto & cell) {
      return cell.key == task_lib::VoxelKey{1, 2, 0};
    });
  ASSERT_NE(blocked, snapshot.cells.end());
  EXPECT_FALSE(blocked->traversable);
  EXPECT_EQ(blocked->neighbor_mask, 0U);
}

TEST(ReversibleVoxelMap, PublishesLocalMacroVoxelDeltasPerProfile)
{
  task_lib::ReversibleVoxelMap map(1.0);
  ASSERT_TRUE(map.RegisterNavigationProfile({"macro", {0, 0, 0}, 2.0, 2}));
  ASSERT_TRUE(map.ApplySparseSnapshot({{"1:0:13", {2.2, 2.2, 0.2}, 1.0F}}));

  const auto update = map.RefreshNavigation("macro", map.TakeChanges());
  EXPECT_FALSE(update.coarse_changes.empty());
  const auto snapshot = map.NavigationSnapshotFor("macro");
  EXPECT_EQ(snapshot.coarse_voxel_factor, 2);
  EXPECT_FALSE(snapshot.coarse_cells.empty());
}

TEST(ReversibleVoxelMap, AppliesSparseDeltaAndRestoresOnlyItsInfluencedCells)
{
  task_lib::ReversibleVoxelMap map(1.0);
  ASSERT_TRUE(map.RegisterNavigationProfile({"incremental", {1, 0, 0}, 2.0}));

  ASSERT_TRUE(map.ApplySparseDelta({{"1:0:14", {4.2, 1.2, 0.2}, 0.8F}}, {}, 0.2F));
  const auto inserted = map.RefreshNavigation("incremental", map.TakeChanges());
  EXPECT_EQ(inserted.recomputed_cells, 45U);
  const auto blocked = std::find_if(
    inserted.changes.begin(), inserted.changes.end(), [](const auto & change) {
      return change.key == task_lib::VoxelKey{3, 1, 0} && !change.after.traversable;
    });
  EXPECT_NE(blocked, inserted.changes.end());

  ASSERT_TRUE(map.ApplySparseDelta({}, {"1:0:14"}, 0.2F));
  const auto removed = map.RefreshNavigation("incremental", map.TakeChanges());
  const auto restored = std::find_if(
    removed.changes.begin(), removed.changes.end(), [](const auto & change) {
      return change.key == task_lib::VoxelKey{3, 1, 0} && change.after.traversable;
    });
  EXPECT_NE(restored, removed.changes.end());
}

TEST(ReversibleVoxelMap, ReportsCurrentRawStateForExecutionTiming)
{
  task_lib::ReversibleVoxelMap map(1.0);
  const task_lib::VoxelKey occupied{2, 0, 0};
  EXPECT_EQ(map.StateAt(occupied), task_lib::VoxelState::Unknown);
  ASSERT_TRUE(map.ApplySparseDelta({{"1:0:15", {2.2, 0.2, 0.2}, 0.8F}}, {}, 0.2F));
  EXPECT_EQ(map.StateAt(occupied), task_lib::VoxelState::Occupied);
  ASSERT_TRUE(map.AddFreeEvidence("trajectory:1", {2.2, 0.2, 0.2}));
  EXPECT_EQ(map.StateAt(occupied), task_lib::VoxelState::Free);
}
