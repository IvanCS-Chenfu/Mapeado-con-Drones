#include "task_lib/dstar_lite.hpp"

#include <gtest/gtest.h>

namespace
{

task_lib::AxisAlignedBox Box(double x, double y, double z)
{
  return task_lib::AxisAlignedBox{{0.0, 0.0, 0.0}, {x, y, z}};
}

task_lib::VoxelCell Occupied(std::int64_t x, std::int64_t y, std::int64_t z)
{
  return task_lib::VoxelCell{{x, y, z}, task_lib::VoxelState::Occupied, 1.0F};
}

task_lib::DStarLitePlanner Planner(const task_lib::AxisAlignedBox & bounds)
{
  task_lib::DStarLitePlanner planner;
  task_lib::DStarLiteParameters parameters;
  parameters.unknown_cost_multiplier = 1.5;
  parameters.occupied_inflation_cells = 0;
  planner.Configure(bounds, 1.0, parameters);
  return planner;
}

}  // namespace

TEST(DStarLite, ReturnsDirectRouteAcrossUnknownSpace)
{
  auto planner = Planner(Box(6.0, 2.0, 1.0));
  planner.SetVoxelSnapshot({}, 1U);
  const auto route = planner.Plan({0.2, 0.2, 0.2}, {5.2, 0.2, 0.2});
  ASSERT_TRUE(route.success);
  ASSERT_EQ(route.waypoints_world.size(), 2U);
  EXPECT_FALSE(route.incremental_repair);
}

TEST(DStarLite, ExplorationProfileTraversesUnknownAtHigherCost)
{
  task_lib::ReversibleVoxelMap map(1.0);
  ASSERT_TRUE(map.RegisterNavigationProfile({"exploration", {0, 0, 0}, 2.0, 1, false}));

  auto planner = Planner(Box(6.0, 2.0, 1.0));
  planner.SetNavigationSnapshot(map.NavigationSnapshotFor("exploration"));
  const auto route = planner.Plan({0.2, 0.2, 0.2}, {5.2, 0.2, 0.2});

  ASSERT_TRUE(route.success);
  ASSERT_FALSE(route.corridor.empty());
  EXPECT_EQ(route.corridor.back(), (task_lib::VoxelKey{5, 0, 0}));
}

TEST(DStarLite, KnownFreeProfileRejectsUnknownCorridor)
{
  task_lib::ReversibleVoxelMap map(1.0);
  ASSERT_TRUE(map.RegisterNavigationProfile({"facade_free", {0, 0, 0}, 2.0, 1, true}));

  auto planner = Planner(Box(6.0, 2.0, 1.0));
  planner.SetNavigationSnapshot(map.NavigationSnapshotFor("facade_free"));
  const auto route = planner.Plan({0.2, 0.2, 0.2}, {5.2, 0.2, 0.2});

  EXPECT_FALSE(route.success);
  EXPECT_NE(route.failure_reason, "");
}

TEST(DStarLite, KnownFreeSafeEscapeNeverCrossesUnknown)
{
  task_lib::ReversibleVoxelMap map(1.0);
  ASSERT_TRUE(map.RegisterNavigationProfile({"facade_free", {1, 0, 0}, 2.0, 1, true}));
  ASSERT_TRUE(map.AddFreeEvidence("start_free", {0.2, 0.2, 0.2}));
  ASSERT_TRUE(map.AddFreeEvidence("goal_free", {4.2, 0.2, 0.2}));
  ASSERT_TRUE(map.ApplySparseSnapshot({{"obstacle", {1.2, 0.2, 0.2}, 1.0F}}));
  map.RefreshNavigation("facade_free", map.TakeChanges());

  auto planner = Planner(Box(6.0, 3.0, 1.0));
  planner.SetNavigationSnapshot(map.NavigationSnapshotFor("facade_free"));
  const auto route = planner.Plan({0.2, 0.2, 0.2}, {4.2, 0.2, 0.2});

  EXPECT_FALSE(route.success);
  EXPECT_EQ(route.failure_reason, "no_safe_escape");
}

TEST(DStarLite, RepairsRouteWhenARelevantVoxelBecomesOccupied)
{
  auto planner = Planner(Box(6.0, 5.0, 1.0));
  planner.SetVoxelSnapshot({}, 1U);
  ASSERT_TRUE(planner.Plan({0.2, 2.2, 0.2}, {5.2, 2.2, 0.2}).success);
  planner.ApplyChanges(
    {task_lib::VoxelChange{{2, 2, 0}, task_lib::VoxelState::Unknown,
        task_lib::VoxelState::Occupied}}, 2U);
  const auto repaired = planner.Plan({0.2, 2.2, 0.2}, {5.2, 2.2, 0.2});
  ASSERT_TRUE(repaired.success);
  EXPECT_TRUE(repaired.incremental_repair);
  for (const auto & key : repaired.corridor) {
    EXPECT_FALSE(key.ix == 2 && key.iy == 2 && key.iz == 0);
  }
}

TEST(DStarLite, FindsTheOnlyGapAroundAnOccupiedWall)
{
  auto planner = Planner(Box(7.0, 5.0, 0.99));
  std::vector<task_lib::VoxelCell> wall;
  for (std::int64_t y = 0; y < 4; ++y) {
    wall.push_back(Occupied(3, y, 0));
  }
  planner.SetVoxelSnapshot(wall, 1U);

  const auto route = planner.Plan({0.2, 2.2, 0.2}, {6.2, 2.2, 0.2});
  ASSERT_TRUE(route.success);
  EXPECT_TRUE(
    std::any_of(
      route.corridor.begin(), route.corridor.end(),
      [](const task_lib::VoxelKey & key) {return key.iy >= 4;}));
}

TEST(DStarLite, RejectsACompletelyBlockedPassage)
{
  auto planner = Planner(Box(5.0, 3.0, 0.99));
  std::vector<task_lib::VoxelCell> wall;
  for (std::int64_t y = 0; y <= 3; ++y) {
    wall.push_back(Occupied(2, y, 0));
  }
  planner.SetVoxelSnapshot(wall, 1U);

  const auto route = planner.Plan({0.2, 1.2, 0.2}, {4.2, 1.2, 0.2});
  EXPECT_FALSE(route.success);
  EXPECT_EQ(route.failure_reason, "unreachable_current_map");
}

TEST(DStarLite, RepairsRouteWhenAnObstacleIsRemoved)
{
  auto planner = Planner(Box(6.0, 5.0, 1.0));
  planner.SetVoxelSnapshot({Occupied(2, 2, 0)}, 1U);
  ASSERT_TRUE(planner.Plan({0.2, 2.2, 0.2}, {5.2, 2.2, 0.2}).success);

  planner.ApplyChanges(
    {task_lib::VoxelChange{{2, 2, 0}, task_lib::VoxelState::Occupied,
        task_lib::VoxelState::Unknown}}, 2U);
  const auto repaired = planner.Plan({0.2, 2.2, 0.2}, {5.2, 2.2, 0.2});
  ASSERT_TRUE(repaired.success);
  EXPECT_TRUE(repaired.incremental_repair);
  EXPECT_EQ(repaired.waypoints_world.size(), 2U);
}

TEST(DStarLite, RejectsDiagonalThatWouldCutOccupiedCorners)
{
  auto planner = Planner(Box(2.0, 2.0, 0.99));
  planner.SetVoxelSnapshot({Occupied(1, 0, 0), Occupied(0, 1, 0)}, 1U);
  const auto route = planner.Plan({0.2, 0.2, 0.2}, {1.2, 1.2, 0.2});
  EXPECT_FALSE(route.success);
}

TEST(DStarLite, AllowsRoutesBeyondTheTrajectoryCreatorLimit)
{
  auto planner = Planner(Box(24.0, 4.0, 1.0));
  planner.SetVoxelSnapshot({}, 1U);
  EXPECT_TRUE(planner.Plan({0.2, 1.2, 0.2}, {20.2, 1.2, 0.2}).success);
}

TEST(DStarLite, KeepsTheSearchWindowStableForSubvoxelPoseJitter)
{
  auto planner = Planner(Box(6.0, 2.0, 1.0));
  planner.SetVoxelSnapshot({}, 1U);
  ASSERT_TRUE(planner.Plan({0.2, 0.2, 0.2}, {2.2, 0.2, 0.2}).success);
  const auto replan = planner.Plan({0.2001, 0.1999, 0.2}, {2.2, 0.2, 0.2});
  EXPECT_TRUE(replan.success);
  EXPECT_TRUE(replan.incremental_repair);
}

TEST(DStarLite, UsesPrecomputedNavigationEdges)
{
  task_lib::ReversibleVoxelMap map(1.0);
  ASSERT_TRUE(map.RegisterNavigationProfile({"compact", {1, 1, 0}, 1.5}));
  ASSERT_TRUE(map.ApplySparseSnapshot({{"wall", {2.2, 1.2, 0.2}, 1.0F}}));
  map.RefreshNavigation("compact", map.TakeChanges());

  auto planner = Planner(Box(6.0, 4.0, 0.99));
  planner.SetNavigationSnapshot(map.NavigationSnapshotFor("compact"));
  const auto route = planner.Plan({0.2, 1.2, 0.2}, {5.2, 1.2, 0.2});
  ASSERT_TRUE(route.success);
  for (const auto & cell : route.corridor) {
    EXPECT_FALSE(cell.ix == 2 && cell.iy == 1);
  }
}

TEST(DStarLite, ReportsCoarseGuideAndQueueMetrics)
{
  task_lib::ReversibleVoxelMap map(1.0);
  ASSERT_TRUE(map.RegisterNavigationProfile({"macro", {0, 0, 0}, 1.5, 2}));
  auto planner = Planner(Box(10.0, 2.0, 0.99));
  planner.SetNavigationSnapshot(map.NavigationSnapshotFor("macro"));

  const auto route = planner.Plan({0.2, 0.2, 0.2}, {9.2, 0.2, 0.2});
  ASSERT_TRUE(route.success);
  EXPECT_EQ(route.coarse_voxel_factor, 2);
  EXPECT_GE(route.queue_pops, route.expanded);
  EXPECT_EQ(route.stale_queue_pops, 0U);
}

TEST(DStarLite, RepairsFromAnIncrementalNavigationUpdate)
{
  task_lib::ReversibleVoxelMap map(1.0);
  ASSERT_TRUE(map.RegisterNavigationProfile({"compact", {0, 0, 0}, 1.5}));
  auto planner = Planner(Box(6.0, 4.0, 0.99));
  planner.SetNavigationSnapshot(map.NavigationSnapshotFor("compact"));
  ASSERT_TRUE(planner.Plan({0.2, 1.2, 0.2}, {5.2, 1.2, 0.2}).success);

  ASSERT_TRUE(map.ApplySparseSnapshot({{"wall", {2.2, 1.2, 0.2}, 1.0F}}));
  const auto update = map.RefreshNavigation("compact", map.TakeChanges());
  planner.ApplyNavigationChanges(update.changes, update.raw_revision);
  const auto repaired = planner.Plan({0.2, 1.2, 0.2}, {5.2, 1.2, 0.2});

  ASSERT_TRUE(repaired.success);
  EXPECT_TRUE(repaired.incremental_repair);
  for (const auto & cell : repaired.corridor) {
    EXPECT_FALSE(cell.ix == 2 && cell.iy == 1);
  }
}

TEST(DStarLite, RejectsInitialClearanceWhenOnlyUnknownEscapeExists)
{
  auto planner = Planner(Box(6.0, 4.0, 0.99));
  std::vector<task_lib::VoxelCell> cells{Occupied(0, 0, 0)};
  task_lib::DStarLiteParameters parameters;
  parameters.unknown_cost_multiplier = 1.5;
  parameters.occupied_inflation_cells = 1;
  planner.Configure(Box(6.0, 4.0, 0.99), 1.0, parameters);
  planner.SetVoxelSnapshot(cells, 1U);

  const auto route = planner.Plan({0.2, 1.2, 0.2}, {5.2, 2.2, 0.2});
  EXPECT_FALSE(route.success);
  EXPECT_EQ(route.failure_reason, "no_safe_escape");
}

TEST(DStarLite, RejectsEscapeWhenFreeVolumeCannotReachClearance)
{
  auto planner = Planner(Box(6.0, 3.0, 0.99));
  task_lib::DStarLiteParameters parameters;
  parameters.unknown_cost_multiplier = 1.5;
  parameters.occupied_inflation_cells = 1;
  planner.Configure(Box(6.0, 3.0, 0.99), 1.0, parameters);
  planner.SetVoxelSnapshot(
  {
    {{1, 1, 0}, task_lib::VoxelState::Free, 0.0F}, Occupied(0, 1, 0),
    Occupied(2, 1, 0), Occupied(1, 0, 0), Occupied(1, 2, 0)}, 1U);

  const auto route = planner.Plan({1.2, 1.2, 0.2}, {5.2, 1.2, 0.2});
  EXPECT_FALSE(route.success);
  EXPECT_EQ(route.failure_reason, "no_safe_escape");
}
