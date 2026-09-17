#include "task_lib/facade_coverage.hpp"

#include <algorithm>
#include <array>
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

double SideCoordinate(const AxisAlignedBox & bounds, FacadeCoverageSide side)
{
  switch (side) {
    case FacadeCoverageSide::MinX:
      return bounds.min.x;
    case FacadeCoverageSide::MaxX:
      return bounds.max.x;
    case FacadeCoverageSide::MinY:
      return bounds.min.y;
    case FacadeCoverageSide::MaxY:
    default:
      return bounds.max.y;
  }
}

Vec3 SideNormal(FacadeCoverageSide side)
{
  switch (side) {
    case FacadeCoverageSide::MinX:
      return {-1.0, 0.0, 0.0};
    case FacadeCoverageSide::MaxX:
      return {1.0, 0.0, 0.0};
    case FacadeCoverageSide::MinY:
      return {0.0, -1.0, 0.0};
    case FacadeCoverageSide::MaxY:
    default:
      return {0.0, 1.0, 0.0};
  }
}

bool UsesXCoordinate(FacadeCoverageSide side)
{
  return side == FacadeCoverageSide::MinY || side == FacadeCoverageSide::MaxY;
}

bool Inside(const AxisAlignedBox & bounds, const Vec3 & point)
{
  return point.x >= bounds.min.x && point.x <= bounds.max.x &&
         point.y >= bounds.min.y && point.y <= bounds.max.y &&
         point.z >= bounds.min.z && point.z <= bounds.max.z;
}

double DistanceToSide(const AxisAlignedBox & bounds, FacadeCoverageSide side, const Vec3 & point)
{
  return std::abs(
    (side == FacadeCoverageSide::MinX || side == FacadeCoverageSide::MaxX ? point.x : point.y) -
    SideCoordinate(bounds, side));
}

Vec3 WallPointForSection(const FacadeCoveragePlan & plan, const FacadeCoverageSection & section)
{
  Vec3 result{
    0.5 * (section.start.x + section.end.x),
    0.5 * (section.start.y + section.end.y),
    0.5 * (plan.bounds.min.z + plan.bounds.max.z)};
  if (section.side == FacadeCoverageSide::MinX || section.side == FacadeCoverageSide::MaxX) {
    result.x = SideCoordinate(plan.bounds, section.side);
  } else {
    result.y = SideCoordinate(plan.bounds, section.side);
  }
  return result;
}

}  // namespace

FacadeCoveragePlan BuildFacadeCoveragePlan(
  const AxisAlignedBox & region_bounds, const AxisAlignedBox & mapping_roi,
  double voxel_size, std::int64_t offset_voxels)
{
  FacadeCoveragePlan plan;
  plan.bounds = region_bounds;
  if (!std::isfinite(voxel_size) || voxel_size <= 0.0 || offset_voxels < 0) {
    return plan;
  }
  const Vec3 roi_center{
    0.5 * (mapping_roi.min.x + mapping_roi.max.x),
    0.5 * (mapping_roi.min.y + mapping_roi.max.y), 0.0};
  const std::array<FacadeCoverageSide, 4U> all_sides{
    FacadeCoverageSide::MinX, FacadeCoverageSide::MaxX,
    FacadeCoverageSide::MinY, FacadeCoverageSide::MaxY};
  double best_distance = std::numeric_limits<double>::infinity();
  for (const auto side : all_sides) {
    const double distance = std::abs(
      (side == FacadeCoverageSide::MinX ||
      side == FacadeCoverageSide::MaxX ? roi_center.x : roi_center.y) -
      SideCoordinate(region_bounds, side));
    if (distance < best_distance - 1e-9 ||
      (std::abs(distance - best_distance) <= 1e-9 &&
      static_cast<std::uint8_t>(side) < static_cast<std::uint8_t>(plan.open_side)))
    {
      best_distance = distance;
      plan.open_side = side;
    }
  }
  const double offset = static_cast<double>(offset_voxels) * voxel_size;
  const auto append_side = [&plan, &region_bounds, voxel_size, offset](FacadeCoverageSide side) {
      if (side == plan.open_side) {
        return;
      }
      const bool along_x = UsesXCoordinate(side);
      const double first = along_x ? region_bounds.min.x : region_bounds.min.y;
      const double last = along_x ? region_bounds.max.x : region_bounds.max.y;
      const std::size_t count = std::max<std::size_t>(
        1U, static_cast<std::size_t>(std::ceil((last - first) / voxel_size)));
      for (std::size_t index = 0U; index < count; ++index) {
        const double a = first + std::min(static_cast<double>(index) * voxel_size, last - first);
        const double b = std::min(last, a + voxel_size);
        const auto normal = SideNormal(side);
        const double coordinate = SideCoordinate(region_bounds, side) -
          normal.x * offset - normal.y * offset;
        FacadeCoverageSection section;
        section.side = side;
        section.outward_normal = normal;
        if (along_x) {
          section.start = {a, coordinate, region_bounds.min.z};
          section.end = {b, coordinate, region_bounds.max.z};
        } else {
          section.start = {coordinate, a, region_bounds.min.z};
          section.end = {coordinate, b, region_bounds.max.z};
        }
        plan.sections.push_back(section);
      }
    };
  const auto u_sides = [&plan]() {
      switch (plan.open_side) {
        case FacadeCoverageSide::MinX:
          return std::array<FacadeCoverageSide, 3U>{
          FacadeCoverageSide::MinY, FacadeCoverageSide::MaxX, FacadeCoverageSide::MaxY};
        case FacadeCoverageSide::MaxX:
          return std::array<FacadeCoverageSide, 3U>{
          FacadeCoverageSide::MinY, FacadeCoverageSide::MinX, FacadeCoverageSide::MaxY};
        case FacadeCoverageSide::MinY:
          return std::array<FacadeCoverageSide, 3U>{
          FacadeCoverageSide::MinX, FacadeCoverageSide::MaxY, FacadeCoverageSide::MaxX};
        case FacadeCoverageSide::MaxY:
        default:
          return std::array<FacadeCoverageSide, 3U>{
          FacadeCoverageSide::MinX, FacadeCoverageSide::MinY, FacadeCoverageSide::MaxX};
      }
    }();
  for (const auto side : u_sides) {
    append_side(side);
  }
  return plan;
}

double FacadeCoverageRatio(
  const FacadeCoveragePlan & plan, const std::vector<bool> & active_sections)
{
  if (plan.sections.empty()) {
    return 0.0;
  }
  const std::size_t active = std::count(active_sections.begin(), active_sections.end(), true);
  return Clamp01(static_cast<double>(active) / static_cast<double>(plan.sections.size()));
}

std::optional<std::size_t> FacadeCoverageSectionForPoint(
  const FacadeCoveragePlan & plan, const Vec3 & point)
{
  if (plan.sections.empty() || !Inside(plan.bounds, point)) {
    return std::nullopt;
  }
  std::optional<FacadeCoverageSide> best_side;
  double best_distance = std::numeric_limits<double>::infinity();
  for (const auto side : {FacadeCoverageSide::MinX, FacadeCoverageSide::MaxX,
      FacadeCoverageSide::MinY, FacadeCoverageSide::MaxY})
  {
    if (side == plan.open_side) {
      continue;
    }
    const double distance = DistanceToSide(plan.bounds, side, point);
    if (!best_side.has_value() || distance < best_distance - 1e-9 ||
      (std::abs(distance - best_distance) <= 1e-9 &&
      static_cast<std::uint8_t>(side) < static_cast<std::uint8_t>(*best_side)))
    {
      best_side = side;
      best_distance = distance;
    }
  }
  if (!best_side.has_value()) {
    return std::nullopt;
  }
  double best_projection = std::numeric_limits<double>::infinity();
  std::optional<std::size_t> best_index;
  for (std::size_t index = 0U; index < plan.sections.size(); ++index) {
    const auto & section = plan.sections[index];
    if (section.side != *best_side) {
      continue;
    }
    const double coordinate = UsesXCoordinate(section.side) ? point.x : point.y;
    const double middle = UsesXCoordinate(section.side) ?
      0.5 * (section.start.x + section.end.x) : 0.5 * (section.start.y + section.end.y);
    const double projection = std::abs(coordinate - middle);
    if (!best_index.has_value() || projection < best_projection - 1e-9) {
      best_index = index;
      best_projection = projection;
    }
  }
  return best_index;
}

FacadeCandidate SelectFacadeCoverageCandidate(
  const FacadeCoveragePlan & plan, const AxisAlignedBox & hard_flight_volume,
  const Vec3 & drone_position, const std::vector<bool> & active_sections,
  const FacadePreferences & preferences, const std::vector<VoxelCell> & voxel_cells,
  double voxel_size, float occupied_score_threshold)
{
  FacadeCandidate best;
  best.score = std::numeric_limits<double>::infinity();
  if (plan.sections.empty() || voxel_size <= 0.0 || !std::isfinite(voxel_size)) {
    return best;
  }
  for (std::size_t index = 0U; index < plan.sections.size(); ++index) {
    if (index < active_sections.size() && active_sections[index]) {
      continue;
    }
    const auto & section = plan.sections[index];
    Vec3 visual_target = WallPointForSection(plan, section);
    float visual_score = 0.0F;
    bool synthetic = true;
    double best_visual_distance = std::numeric_limits<double>::infinity();
    for (const auto & cell : voxel_cells) {
      if (cell.state != VoxelState::Occupied || cell.score <= occupied_score_threshold) {
        continue;
      }
      const Vec3 point{
        (static_cast<double>(cell.key.ix) + 0.5) * voxel_size,
        (static_cast<double>(cell.key.iy) + 0.5) * voxel_size,
        (static_cast<double>(cell.key.iz) + 0.5) * voxel_size};
      const auto mapped_section = FacadeCoverageSectionForPoint(plan, point);
      if (!mapped_section.has_value() || *mapped_section != index) {
        continue;
      }
      const double distance = Distance(point, visual_target);
      if (distance < best_visual_distance - 1e-9) {
        visual_target = point;
        visual_score = cell.score;
        best_visual_distance = distance;
        synthetic = false;
      }
    }
    const Vec3 target{
      visual_target.x + section.outward_normal.x * preferences.preferred_wall_distance_m,
      visual_target.y + section.outward_normal.y * preferences.preferred_wall_distance_m,
      visual_target.z + section.outward_normal.z * preferences.preferred_wall_distance_m};
    if (!Inside(hard_flight_volume, target)) {
      continue;
    }
    const double displacement_cost = std::abs(
      Distance(drone_position, target) - preferences.preferred_displacement_m);
    const double height_cost = std::abs(target.z - 0.5 * (plan.bounds.min.z + plan.bounds.max.z));
    const double score = preferences.displacement_weight * displacement_cost +
      preferences.height_weight * height_cost;
    if (!best.valid || score < best.score - 1e-9 ||
      (std::abs(score - best.score) <= 1e-9 && index < best.section_index))
    {
      best.valid = true;
      best.visual_target = visual_target;
      best.target = target;
      best.score = score;
      best.visual_score = visual_score;
      best.section_index = index;
      best.observation_yaw_rad = std::atan2(-section.outward_normal.y, -section.outward_normal.x);
      best.synthetic_visual_target = synthetic;
    }
  }
  return best;
}

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
  const FacadePreferences & preferences, double candidate_step_m,
  const std::vector<VoxelCell> & voxel_cells, double voxel_size,
  float visual_target_min_score, float visual_target_max_score)
{
  FacadeCandidate best;
  best.score = std::numeric_limits<double>::infinity();
  if (facade.length_m <= 1e-9 || voxel_size <= 0.0 ||
    visual_target_min_score > visual_target_max_score)
  {
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
  for (const auto & cell : voxel_cells) {
    if (cell.score < visual_target_min_score || cell.score > visual_target_max_score) {
      continue;
    }
    const Vec3 visual_target{
      (static_cast<double>(cell.key.ix) + 0.5) * voxel_size,
      (static_cast<double>(cell.key.iy) + 0.5) * voxel_size,
      (static_cast<double>(cell.key.iz) + 0.5) * voxel_size};
    if (visual_target.x < region_bounds.min.x || visual_target.x > region_bounds.max.x ||
      visual_target.y < region_bounds.min.y || visual_target.y > region_bounds.max.y ||
      visual_target.z < region_bounds.min.z || visual_target.z > region_bounds.max.z)
    {
      continue;
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
            const double visual_distance_cost = std::abs(
              Distance(target, visual_target) - preferences.preferred_wall_distance_m);
            const double displacement_cost = std::abs(
              Distance(drone_position, target) - preferences.preferred_displacement_m);
            const double height_cost = std::abs(
              target.z - 0.5 * (region_bounds.min.z + region_bounds.max.z));
            const double score = preferences.wall_distance_weight * visual_distance_cost +
              preferences.displacement_weight * displacement_cost +
              preferences.height_weight * height_cost;
            if (!best.valid || score < best.score - 1e-9 ||
              (std::abs(score - best.score) <= 1e-9 && bounded_ratio < best.target_ratio))
            {
              best.valid = true;
              best.visual_target = visual_target;
              best.visual_score = cell.score;
              best.target = target;
              best.target_ratio = bounded_ratio;
              best.score = score;
            }
          }
        }
        if (bounded_ratio >= interval.end_ratio - 1e-9) {
          break;
        }
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
