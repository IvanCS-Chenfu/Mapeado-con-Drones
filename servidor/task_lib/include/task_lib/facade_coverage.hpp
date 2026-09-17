#pragma once

#include "task_lib/mission_config.hpp"
#include "task_lib/voxel_map.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

namespace task_lib
{

struct FacadeLine
{
  Vec3 start;
  Vec3 end;
  Vec3 outward_normal;
  double wall_coordinate = 0.0;
  double length_m = 0.0;
};

struct FacadeCoverageInterval
{
  double start_ratio = 0.0;
  double end_ratio = 0.0;
};

struct FacadePreferences
{
  double preferred_wall_distance_m = 2.5;
  double preferred_displacement_m = 2.0;
  double wall_distance_weight = 1.0;
  double displacement_weight = 1.0;
  double height_weight = 1.0;
  double completion_ratio = 0.99;
};

struct FacadeCandidate
{
  bool valid = false;
  Vec3 visual_target;
  Vec3 target;
  double target_ratio = 0.0;
  double score = 0.0;
  float visual_score = 0.0F;
  std::size_t section_index = 0U;
  double observation_yaw_rad = 0.0;
  bool synthetic_visual_target = false;
};

enum class FacadeCoverageSide : std::uint8_t
{
  MinX = 0U,
  MaxX = 1U,
  MinY = 2U,
  MaxY = 3U,
};

struct FacadeCoverageSection
{
  FacadeCoverageSide side = FacadeCoverageSide::MinY;
  Vec3 start;
  Vec3 end;
  Vec3 outward_normal;
};

struct FacadeCoveragePlan
{
  FacadeCoverageSide open_side = FacadeCoverageSide::MaxY;
  AxisAlignedBox bounds;
  std::vector<FacadeCoverageSection> sections;
};

FacadeCoveragePlan BuildFacadeCoveragePlan(
  const AxisAlignedBox & region_bounds, const AxisAlignedBox & mapping_roi,
  double voxel_size, std::int64_t offset_voxels);
double FacadeCoverageRatio(
  const FacadeCoveragePlan & plan, const std::vector<bool> & active_sections);
std::optional<std::size_t> FacadeCoverageSectionForPoint(
  const FacadeCoveragePlan & plan, const Vec3 & point);
FacadeCandidate SelectFacadeCoverageCandidate(
  const FacadeCoveragePlan & plan, const AxisAlignedBox & hard_flight_volume,
  const Vec3 & drone_position, const std::vector<bool> & active_sections,
  const FacadePreferences & preferences, const std::vector<VoxelCell> & voxel_cells,
  double voxel_size, float occupied_score_threshold);

struct FreePrefixResult
{
  bool valid = false;
  Vec3 end;
  double length_m = 0.0;
};

FacadeLine EstimateFacadeLine(
  const BaseSubRoi & region, const AxisAlignedBox & mapping_roi,
  const std::vector<Vec3> & occupied_points, double preferred_wall_distance_m);

Vec3 PointOnFacade(const FacadeLine & facade, double ratio);
double ProjectToFacadeRatio(const FacadeLine & facade, const Vec3 & point);
double FacadeObservationYaw(const FacadeLine & facade);

std::vector<FacadeCoverageInterval> MergeFacadeCoverage(
  const std::vector<FacadeCoverageInterval> & intervals);
std::vector<FacadeCoverageInterval> UncoveredFacadeIntervals(
  const std::vector<FacadeCoverageInterval> & covered);
double FacadeCoverageRatio(const std::vector<FacadeCoverageInterval> & intervals);

FacadeCandidate SelectFacadeCandidate(
  const FacadeLine & facade, const AxisAlignedBox & region_bounds,
  const AxisAlignedBox & hard_flight_volume, const Vec3 & drone_position,
  const std::vector<FacadeCoverageInterval> & covered,
  const FacadePreferences & preferences, double candidate_step_m,
  const std::vector<VoxelCell> & voxel_cells, double voxel_size,
  float visual_target_min_score, float visual_target_max_score);

FreePrefixResult FurthestFreePrefix(
  const Vec3 & start, const Vec3 & target, const Vec3 & body_half_extent,
  double voxel_size, double minimum_length_m,
  const std::function<VoxelState(const VoxelKey &)> & state_for_key);

}  // namespace task_lib
