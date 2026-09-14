#include "task_lib/reservation_overlay.hpp"

#include <gtest/gtest.h>

namespace
{

task_lib::VoxelKey Key(std::int64_t x, std::int64_t y, std::int64_t z)
{
  return task_lib::VoxelKey{x, y, z};
}

TEST(ReservationOverlay, BloqueaSoloReservaAjenaSinMutarMapaRaw)
{
  task_lib::ReservationOverlay overlay;
  ASSERT_TRUE(
    overlay.Commit(
      {"r1", 1U, task_lib::ReservationMode::Moving,
        {Key(1, 2, 3)}, 0U}));
  task_lib::NavigationSnapshot snapshot;
  snapshot.unknown_cost_multiplier = 3.0;
  EXPECT_TRUE(overlay.ApplyTo(snapshot, 1U).cells.empty());
  const auto other = overlay.ApplyTo(snapshot, 2U);
  ASSERT_EQ(other.cells.size(), 1U);
  EXPECT_EQ(other.cells.front().raw_state, task_lib::VoxelState::Unknown);
  EXPECT_FALSE(other.cells.front().traversable);
}

TEST(ReservationOverlay, RechazaConflictoYReemplazaAtomico)
{
  task_lib::ReservationOverlay overlay;
  ASSERT_TRUE(
    overlay.Commit(
      {"r1", 1U, task_lib::ReservationMode::Moving,
        {Key(1, 0, 0)}, 0U}));
  EXPECT_FALSE(overlay.CanCommit(2U, {Key(1, 0, 0)}));
  EXPECT_TRUE(
    overlay.Replace(
      "r1", {"r2", 1U, task_lib::ReservationMode::Hold,
        {Key(2, 0, 0)}, 0U}));
  EXPECT_FALSE(overlay.CanCommit(2U, {Key(2, 0, 0)}));
  EXPECT_TRUE(overlay.CanCommit(2U, {Key(1, 0, 0)}));
}

TEST(ReservationOverlay, ExpandeSoloPorFootprintFisicoDelSolicitante)
{
  task_lib::ReservationOverlay overlay;
  ASSERT_TRUE(
    overlay.Commit(
      {"r1", 1U, task_lib::ReservationMode::Moving,
        {Key(0, 0, 0)}, 0U}));

  task_lib::NavigationSnapshot snapshot;
  snapshot.unknown_cost_multiplier = 3.0;
  const auto projected = overlay.ApplyTo(snapshot, 2U, Key(1, 0, 0));

  ASSERT_EQ(projected.cells.size(), 3U);
  for (const auto & cell : projected.cells) {
    EXPECT_FALSE(cell.traversable);
    EXPECT_EQ(cell.key.iy, 0);
    EXPECT_EQ(cell.key.iz, 0);
  }
}

}  // namespace
