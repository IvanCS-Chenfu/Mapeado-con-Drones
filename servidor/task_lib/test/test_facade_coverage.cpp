#include "task_lib/facade_coverage.hpp"

#include <gtest/gtest.h>

#include <set>

namespace
{

task_lib::BaseSubRoi MakeAbRegion()
{
  task_lib::BaseSubRoi region;
  region.region_id = "level_0_AB";
  region.side = task_lib::BaseSide::AB;
  region.bounds = {{-10.0, -10.0, 0.0}, {10.0, 0.0, 4.0}};
  return region;
}

TEST(FacadeCoverage, BuildsAbLineAtPreferredStandoff)
{
  const auto facade = task_lib::EstimateFacadeLine(
    MakeAbRegion(), {{-10.0, -10.0, 0.0}, {10.0, 10.0, 4.0}},
    {{-4.0, -3.0, 2.0}, {0.0, -3.0, 2.0}, {4.0, -3.0, 2.0}}, 2.5);
  EXPECT_NEAR(facade.start.y, -5.5, 1e-9);
  EXPECT_NEAR(facade.end.y, -5.5, 1e-9);
  EXPECT_NEAR(facade.start.z, 2.0, 1e-9);
  EXPECT_NEAR(task_lib::FacadeObservationYaw(facade), 1.5707963267948966, 1e-9);
}

TEST(FacadeCoverage, MergesIntervalsAndMeasuresHalfFacade)
{
  const auto merged = task_lib::MergeFacadeCoverage({{0.5, 0.75}, {0.7, 1.0}});
  ASSERT_EQ(merged.size(), 1U);
  EXPECT_NEAR(merged.front().start_ratio, 0.5, 1e-9);
  EXPECT_NEAR(merged.front().end_ratio, 1.0, 1e-9);
  EXPECT_NEAR(task_lib::FacadeCoverageRatio(merged), 0.5, 1e-9);
}

TEST(FacadeCoverage, SelectsUncoveredCandidateNearPreferredDisplacement)
{
  const auto facade = task_lib::EstimateFacadeLine(
    MakeAbRegion(), {{-10.0, -10.0, 0.0}, {10.0, 10.0, 4.0}}, {}, 2.5);
  task_lib::FacadePreferences preferences;
  preferences.preferred_displacement_m = 2.0;
  const task_lib::AxisAlignedBox region_bounds{{-10.0, -10.0, 0.0}, {10.0, 0.0, 4.0}};
  const task_lib::AxisAlignedBox hard_volume{{-12.0, -12.0, -1.0}, {12.0, 12.0, 5.0}};
  const auto candidate = task_lib::SelectFacadeCandidate(
    facade, region_bounds, hard_volume, task_lib::PointOnFacade(facade, 0.5),
    {{0.0, 0.5}}, preferences, 0.25);
  ASSERT_TRUE(candidate.valid);
  EXPECT_NEAR(candidate.target_ratio, 0.6, 0.02);
}

TEST(FacadeCoverage, StopsAtFirstNonFreeSweptSample)
{
  std::set<task_lib::VoxelKey> free;
  for (std::int64_t ix = 0; ix < 6; ++ix) {
    free.insert({ix, 0, 0});
  }
  const auto result = task_lib::FurthestFreePrefix(
    {0.1, 0.1, 0.1}, {2.9, 0.1, 0.1}, {0.0, 0.0, 0.0}, 0.5, 1.0,
    [&free](const task_lib::VoxelKey & key) {
      return free.count(key) != 0U ? task_lib::VoxelState::Free : task_lib::VoxelState::Unknown;
    });
  EXPECT_TRUE(result.valid);
  EXPECT_GE(result.length_m, 2.0);
  EXPECT_LT(result.length_m, 2.9);
}

}  // namespace
