#pragma once

#include "task_lib/mission_config.hpp"
#include "task_lib/voxel_map.hpp"

#include <cstddef>
#include <functional>
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
  Vec3 target;
  double target_ratio = 0.0;
  double score = 0.0;
};

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
  const FacadePreferences & preferences, double candidate_step_m);

FreePrefixResult FurthestFreePrefix(
  const Vec3 & start, const Vec3 & target, const Vec3 & body_half_extent,
  double voxel_size, double minimum_length_m,
  const std::function<VoxelState(const VoxelKey &)> & state_for_key);

}  // namespace task_lib
