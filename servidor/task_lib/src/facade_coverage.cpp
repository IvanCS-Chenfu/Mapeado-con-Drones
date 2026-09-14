#include "task_lib/facade_coverage.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace task_lib
{
namespace
{

double Clamp01(double value)
{
  return std::max(0.0, std::min(1.0, value));
}

double Distance(const Vec3 & a, const Vec3 & b)
{
  return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
}

double RobustExtreme(std::vector<double> values, bool high, double fallback)
{
  if (values.empty()) {
    return fallback;
  }
  std::sort(values.begin(), values.end());
  const std::size_t band = std::max<std::size_t>(1U, values.size() / 5U);
  const auto first = high ? values.end() - static_cast<std::ptrdiff_t>(band) : values.begin();
  const auto last = high ? values.end() : values.begin() + static_cast<std::ptrdiff_t>(band);
  double sum = 0.0;
  for (auto it = first; it != last; ++it) {
    sum += *it;
  }
  return sum / static_cast<double>(band);
}

VoxelKey ToKey(const Vec3 & point, double voxel_size)
{
  return {
    static_cast<std::int64_t>(std::floor(point.x / voxel_size)),
    static_cast<std::int64_t>(std::floor(point.y / voxel_size)),
    static_cast<std::int64_t>(std::floor(point.z / voxel_size))};
}

bool SweptSampleFree(
  const Vec3 & center, const Vec3 & half_extent, double voxel_size,
  const std::function<VoxelState(const VoxelKey &)> & state_for_key)
{
  const auto min_key = ToKey(
    {center.x - half_extent.x, center.y - half_extent.y, center.z - half_extent.z}, voxel_size);
  const auto max_key = ToKey(
    {center.x + half_extent.x, center.y + half_extent.y, center.z + half_extent.z}, voxel_size);
  for (std::int64_t ix = min_key.ix; ix <= max_key.ix; ++ix) {
    for (std::int64_t iy = min_key.iy; iy <= max_key.iy; ++iy) {
      for (std::int64_t iz = min_key.iz; iz <= max_key.iz; ++iz) {
        if (state_for_key({ix, iy, iz}) != VoxelState::Free) {
          return false;
        }
      }
    }
  }
  return true;
}

}  // namespace

FacadeLine EstimateFacadeLine(
  const BaseSubRoi & region, const AxisAlignedBox & mapping_roi,
  const std::vector<Vec3> & occupied_points, double preferred_wall_distance_m)
{
  std::vector<double> axis_values;
  axis_values.reserve(occupied_points.size());
  for (const auto & point : occupied_points) {
    if (point.x >= region.bounds.min.x && point.x <= region.bounds.max.x &&
      point.y >= region.bounds.min.y && point.y <= region.bounds.max.y &&
      point.z >= region.bounds.min.z && point.z <= region.bounds.max.z)
    {
      axis_values.push_back(
        region.side == BaseSide::AB || region.side == BaseSide::CD ? point.y : point.x);
    }
  }

  FacadeLine result;
  const double z = 0.5 * (region.bounds.min.z + region.bounds.max.z);
  if (region.side == BaseSide::AB) {
    result.wall_coordinate = RobustExtreme(axis_values, false, mapping_roi.min.y);
    const double y = result.wall_coordinate - preferred_wall_distance_m;
    result.start = {mapping_roi.min.x, y, z};
    result.end = {mapping_roi.max.x, y, z};
    result.outward_normal = {0.0, -1.0, 0.0};
  } else if (region.side == BaseSide::BC) {
    result.wall_coordinate = RobustExtreme(axis_values, true, mapping_roi.max.x);
    const double x = result.wall_coordinate + preferred_wall_distance_m;
    result.start = {x, mapping_roi.min.y, z};
    result.end = {x, mapping_roi.max.y, z};
    result.outward_normal = {1.0, 0.0, 0.0};
  } else if (region.side == BaseSide::CD) {
    result.wall_coordinate = RobustExtreme(axis_values, true, mapping_roi.max.y);
    const double y = result.wall_coordinate + preferred_wall_distance_m;
    result.start = {mapping_roi.min.x, y, z};
    result.end = {mapping_roi.max.x, y, z};
    result.outward_normal = {0.0, 1.0, 0.0};
  } else {
    result.wall_coordinate = RobustExtreme(axis_values, false, mapping_roi.min.x);
    const double x = result.wall_coordinate - preferred_wall_distance_m;
    result.start = {x, mapping_roi.min.y, z};
    result.end = {x, mapping_roi.max.y, z};
    result.outward_normal = {-1.0, 0.0, 0.0};
  }
  result.length_m = Distance(result.start, result.end);
  return result;
}

Vec3 PointOnFacade(const FacadeLine & facade, double ratio)
{
  const double t = Clamp01(ratio);
  return {
    facade.start.x + t * (facade.end.x - facade.start.x),
    facade.start.y + t * (facade.end.y - facade.start.y),
    facade.start.z + t * (facade.end.z - facade.start.z)};
}

double ProjectToFacadeRatio(const FacadeLine & facade, const Vec3 & point)
{
  const Vec3 delta{
    facade.end.x - facade.start.x, facade.end.y - facade.start.y,
    facade.end.z - facade.start.z};
  const double denominator = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;
  if (denominator <= 1e-12) {
    return 0.0;
  }
  return Clamp01(
    ((point.x - facade.start.x) * delta.x + (point.y - facade.start.y) * delta.y +
    (point.z - facade.start.z) * delta.z) / denominator);
}

double FacadeObservationYaw(const FacadeLine & facade)
{
  return std::atan2(-facade.outward_normal.y, -facade.outward_normal.x);
}

std::vector<FacadeCoverageInterval> MergeFacadeCoverage(
  const std::vector<FacadeCoverageInterval> & intervals)
{
  std::vector<FacadeCoverageInterval> normalized;
  normalized.reserve(intervals.size());
  for (const auto & interval : intervals) {
    const double start = Clamp01(std::min(interval.start_ratio, interval.end_ratio));
    const double end = Clamp01(std::max(interval.start_ratio, interval.end_ratio));
    if (end - start > 1e-9) {
      normalized.push_back({start, end});
    }
  }
  std::sort(
    normalized.begin(), normalized.end(), [](const auto & left, const auto & right) {
      return left.start_ratio < right.start_ratio;
    });
  std::vector<FacadeCoverageInterval> merged;
  for (const auto & interval : normalized) {
    if (merged.empty() || interval.start_ratio > merged.back().end_ratio + 1e-9) {
      merged.push_back(interval);
    } else {
      merged.back().end_ratio = std::max(merged.back().end_ratio, interval.end_ratio);
    }
  }
  return merged;
}

std::vector<FacadeCoverageInterval> UncoveredFacadeIntervals(
  const std::vector<FacadeCoverageInterval> & covered)
{
  const auto merged = MergeFacadeCoverage(covered);
  std::vector<FacadeCoverageInterval> result;
  double cursor = 0.0;
  for (const auto & interval : merged) {
    if (interval.start_ratio > cursor + 1e-9) {
      result.push_back({cursor, interval.start_ratio});
    }
    cursor = std::max(cursor, interval.end_ratio);
  }
  if (cursor < 1.0 - 1e-9) {
    result.push_back({cursor, 1.0});
  }
  return result;
}

double FacadeCoverageRatio(const std::vector<FacadeCoverageInterval> & intervals)
{
  double ratio = 0.0;
  for (const auto & interval : MergeFacadeCoverage(intervals)) {
    ratio += interval.end_ratio - interval.start_ratio;
  }
  return Clamp01(ratio);
}

FacadeCandidate SelectFacadeCandidate(
  const FacadeLine & facade, const AxisAlignedBox & region_bounds,
  const AxisAlignedBox & hard_flight_volume, const Vec3 & drone_position,
  const std::vector<FacadeCoverageInterval> & covered,
  const FacadePreferences & preferences, double candidate_step_m)
{
  FacadeCandidate best;
  best.score = std::numeric_limits<double>::infinity();
  if (facade.length_m <= 1e-9) {
    return best;
  }
  const double step = std::max(1e-3, candidate_step_m);
  const double ratio_step = std::max(1e-6, step / facade.length_m);
  std::vector<double> wall_offsets{0.0};
  for (double offset = step; offset <= preferences.preferred_wall_distance_m + 1e-9;
    offset += step)
  {
    wall_offsets.push_back(offset);
    wall_offsets.push_back(-offset);
  }
  std::vector<double> heights{0.5 * (region_bounds.min.z + region_bounds.max.z)};
  for (double z = region_bounds.min.z; z <= region_bounds.max.z + 1e-9; z += step) {
    heights.push_back(std::min(z, region_bounds.max.z));
  }
  for (const auto & interval : UncoveredFacadeIntervals(covered)) {
    for (double ratio = interval.start_ratio; ratio <= interval.end_ratio + 1e-9;
      ratio += ratio_step)
    {
      const double bounded_ratio = std::min(ratio, interval.end_ratio);
      const Vec3 preferred = PointOnFacade(facade, bounded_ratio);
      for (const double wall_offset : wall_offsets) {
        for (const double z : heights) {
          const Vec3 target{
            preferred.x + wall_offset * facade.outward_normal.x,
            preferred.y + wall_offset * facade.outward_normal.y, z};
          if (target.x < hard_flight_volume.min.x || target.x > hard_flight_volume.max.x ||
            target.y < hard_flight_volume.min.y || target.y > hard_flight_volume.max.y ||
            target.z < hard_flight_volume.min.z || target.z > hard_flight_volume.max.z)
          {
            continue;
          }
          const double displacement_cost = std::abs(
            Distance(drone_position, target) - preferences.preferred_displacement_m);
          const double wall_cost = std::abs(wall_offset);
          const double height_cost = std::abs(
            target.z - 0.5 * (facade.start.z + facade.end.z));
          const double score = preferences.displacement_weight * displacement_cost +
            preferences.wall_distance_weight * wall_cost +
            preferences.height_weight * height_cost;
          if (!best.valid || score < best.score - 1e-9 ||
            (std::abs(score - best.score) <= 1e-9 && bounded_ratio < best.target_ratio))
          {
            best = {true, target, bounded_ratio, score};
          }
        }
      }
      if (bounded_ratio >= interval.end_ratio - 1e-9) {
        break;
      }
    }
  }
  return best;
}

FreePrefixResult FurthestFreePrefix(
  const Vec3 & start, const Vec3 & target, const Vec3 & body_half_extent,
  double voxel_size, double minimum_length_m,
  const std::function<VoxelState(const VoxelKey &)> & state_for_key)
{
  FreePrefixResult result;
  const double length = Distance(start, target);
  if (length <= 1e-9 || voxel_size <= 0.0) {
    return result;
  }
  const std::size_t steps = std::max<std::size_t>(
    1U, static_cast<std::size_t>(
      std::ceil(length / std::max(1e-6, 0.5 * voxel_size))));
  for (std::size_t i = 1U; i <= steps; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(steps);
    const Vec3 sample{
      start.x + ratio * (target.x - start.x),
      start.y + ratio * (target.y - start.y),
      start.z + ratio * (target.z - start.z)};
    if (!SweptSampleFree(sample, body_half_extent, voxel_size, state_for_key)) {
      break;
    }
    result.end = sample;
    result.length_m = ratio * length;
  }
  result.valid = result.length_m + 1e-9 >= minimum_length_m;
  return result;
}

}  // namespace task_lib
