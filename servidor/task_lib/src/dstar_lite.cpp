#include "task_lib/dstar_lite.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace task_lib
{
namespace
{

constexpr double kInfinity = std::numeric_limits<double>::infinity();
constexpr double kEpsilon = 1e-9;

}  // namespace

bool DStarLitePlanner::QueueEntryLess(
  const QueueEntry & left, const QueueEntry & right)
{
  if (std::abs(left.key.first - right.key.first) > kEpsilon) {
    return left.key.first < right.key.first;
  }
  if (std::abs(left.key.second - right.key.second) > kEpsilon) {
    return left.key.second < right.key.second;
  }
  return DStarLitePlanner::Less(left.state, right.state);
}

bool DStarLitePlanner::Equal(const VoxelKey & left, const VoxelKey & right)
{
  return left.ix == right.ix && left.iy == right.iy && left.iz == right.iz;
}

bool DStarLitePlanner::Less(const VoxelKey & left, const VoxelKey & right)
{
  return left < right;
}

double DStarLitePlanner::Distance(const VoxelKey & left, const VoxelKey & right)
{
  const double dx = static_cast<double>(left.ix - right.ix);
  const double dy = static_cast<double>(left.iy - right.iy);
  const double dz = static_cast<double>(left.iz - right.iz);
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool DStarLitePlanner::KeyLess(const QueueKey & left, const QueueKey & right)
{
  return left.first + kEpsilon < right.first ||
         (std::abs(left.first - right.first) <= kEpsilon && left.second + kEpsilon < right.second);
}

void DStarLitePlanner::Configure(
  const AxisAlignedBox & hard_flight_volume, double voxel_size,
  const DStarLiteParameters & parameters)
{
  if (!std::isfinite(voxel_size) || voxel_size <= 0.0 ||
    !std::isfinite(parameters.unknown_cost_multiplier) ||
    parameters.unknown_cost_multiplier < 1.0 || parameters.occupied_inflation_cells < 0 ||
    !std::isfinite(parameters.search_margin_m) || parameters.search_margin_m < 0.0 ||
    parameters.max_expansions == 0U || !std::isfinite(parameters.heuristic_weight) ||
    parameters.heuristic_weight < 1.0 || parameters.initial_corridor_half_width_cells < 0 ||
    parameters.corridor_expand_step_cells < 1)
  {
    throw std::invalid_argument("parametros D* Lite invalidos");
  }
  hard_flight_volume_ = hard_flight_volume;
  search_volume_ = hard_flight_volume;
  voxel_size_ = voxel_size;
  parameters_ = parameters;
  initialized_ = false;
  configured_ = true;
  cells_.clear();
  navigation_cells_.clear();
  navigation_coarse_cells_.clear();
  guide_cells_.clear();
  uses_navigation_snapshot_ = false;
}

void DStarLitePlanner::SetNavigationSnapshot(const NavigationSnapshot & snapshot)
{
  navigation_cells_.clear();
  for (const auto & cell : snapshot.cells) {
    navigation_cells_[cell.key] = cell;
  }
  navigation_coarse_cells_.clear();
  for (const auto & cell : snapshot.coarse_cells) {
    navigation_coarse_cells_[cell.key] = cell;
  }
  cells_.clear();
  navigation_unknown_cost_multiplier_ = snapshot.unknown_cost_multiplier;
  navigation_require_known_free_ = snapshot.require_known_free;
  coarse_voxel_factor_ = std::max<std::int64_t>(1, snapshot.coarse_voxel_factor);
  uses_navigation_snapshot_ = true;
  map_revision_ = snapshot.raw_revision;
  initialized_ = false;
}

void DStarLitePlanner::ApplyNavigationChanges(
  const std::vector<NavigationChange> & changes, std::uint64_t revision)
{
  ApplyNavigationChanges(changes, {}, revision);
}

void DStarLitePlanner::ApplyNavigationChanges(
  const std::vector<NavigationChange> & changes,
  const std::vector<NavigationCoarseChange> & coarse_changes, std::uint64_t revision)
{
  uses_navigation_snapshot_ = true;
  for (const auto & change : changes) {
    const bool is_default = change.after.raw_state == VoxelState::Unknown &&
      change.after.traversable == !navigation_require_known_free_ &&
      change.after.neighbor_mask ==
      (navigation_require_known_free_ ? 0U : kNavigationAllNeighbors) &&
      std::abs(change.after.entry_cost_multiplier - navigation_unknown_cost_multiplier_) <=
      kEpsilon;
    if (is_default) {
      navigation_cells_.erase(change.key);
    } else {
      navigation_cells_[change.key] = change.after;
    }
    if (initialized_) {
      UpdateAffected(change.key);
    }
  }
  for (const auto & change : coarse_changes) {
    const bool is_default = change.after.traversable == !navigation_require_known_free_ &&
      std::abs(change.after.blocked_fraction) <= kEpsilon &&
      std::abs(change.after.entry_cost_multiplier - navigation_unknown_cost_multiplier_) <=
      kEpsilon;
    if (is_default) {
      navigation_coarse_cells_.erase(change.key);
    } else {
      navigation_coarse_cells_[change.key] = change.after;
    }
  }
  map_revision_ = revision;
}

void DStarLitePlanner::SetVoxelSnapshot(
  const std::vector<VoxelCell> & cells, std::uint64_t revision)
{
  cells_.clear();
  navigation_cells_.clear();
  navigation_coarse_cells_.clear();
  uses_navigation_snapshot_ = false;
  coarse_voxel_factor_ = 1;
  for (const auto & cell : cells) {
    cells_[cell.key] = cell.state;
  }
  map_revision_ = revision;
  if (initialized_) {
    for (const auto & cell : cells_) {
      UpdateAffected(cell.first);
    }
  }
}

void DStarLitePlanner::ApplyChanges(
  const std::vector<VoxelChange> & changes, std::uint64_t revision)
{
  uses_navigation_snapshot_ = false;
  for (const auto & change : changes) {
    if (change.after == VoxelState::Unknown) {
      cells_.erase(change.key);
    } else {
      cells_[change.key] = change.after;
    }
    if (initialized_) {
      UpdateAffected(change.key);
    }
  }
  map_revision_ = revision;
}

DStarLiteResult DStarLitePlanner::Plan(const Vec3 & start_world, const Vec3 & goal_world)
{
  if (!configured_) {
    throw std::logic_error("D* Lite no configurado");
  }
  const auto start = ToKey(start_world);
  const auto goal = ToKey(goal_world);
  DStarLiteResult result;
  result.map_revision = map_revision_;
  UpdateSearchVolume(start_world, goal_world);
  if (!IsInsideSearchVolume(start)) {
    result.failure_reason = "start_outside_hard_volume";
    return result;
  }
  if (!IsInsideSearchVolume(goal)) {
    result.failure_reason = "goal_outside_hard_volume";
    return result;
  }
  if (!IsTraversable(goal)) {
    result.failure_reason = "goal_occupied_or_inflated";
    return result;
  }

  std::vector<VoxelKey> escape;
  VoxelKey planning_start = start;
  if (!IsTraversable(start)) {
    if (RawState(start) == VoxelState::Occupied) {
      result.failure_reason = "start_occupied_or_inflated";
      return result;
    }
    escape = FindSafeEscape(start);
    if (escape.empty()) {
      result.failure_reason = "no_safe_escape";
      return result;
    }
    planning_start = escape.back();
    result.used_safe_escape = true;
  }

  const auto guide = BuildCoarseGuide(planning_start, goal);
  const auto max_extent = std::max(
    {std::llabs(ToKey(search_volume_.max).ix - ToKey(search_volume_.min).ix),
      std::llabs(ToKey(search_volume_.max).iy - ToKey(search_volume_.min).iy),
      std::llabs(ToKey(search_volume_.max).iz - ToKey(search_volume_.min).iz)});
  std::vector<std::int64_t> widths;
  for (std::int64_t width = parameters_.initial_corridor_half_width_cells;
    width < max_extent; width += parameters_.corridor_expand_step_cells)
  {
    widths.push_back(width);
  }
  widths.push_back(0);

  std::vector<VoxelKey> path;
  std::uint64_t total_expanded = 0U;
  std::uint64_t total_queue_pops = 0U;
  std::uint64_t total_stale_queue_pops = 0U;
  bool repair = false;
  for (std::size_t attempt = 0U; attempt < widths.size(); ++attempt) {
    SetGuide(guide, widths[attempt]);
    repair = initialized_ && Equal(goal, goal_);
    if (!repair) {
      ResetSearch(planning_start, goal);
    } else {
      km_ += Distance(last_start_, planning_start);
      start_ = planning_start;
      last_start_ = planning_start;
    }
    if (!ComputeShortestPath()) {
      total_expanded += expanded_;
      total_queue_pops += queue_pops_;
      total_stale_queue_pops += stale_queue_pops_;
      result.expanded = total_expanded;
      result.queue_pops = total_queue_pops;
      result.stale_queue_pops = total_stale_queue_pops;
      result.corridor_widenings = attempt;
      result.coarse_voxel_factor = coarse_voxel_factor_;
      result.corridor_half_width_cells = widths[attempt];
      result.failure_reason = "expansion_budget_exhausted";
      return result;
    }
    path = BuildPath();
    total_expanded += expanded_;
    total_queue_pops += queue_pops_;
    total_stale_queue_pops += stale_queue_pops_;
    if (!path.empty()) {
      result.corridor_widenings = attempt;
      result.corridor_half_width_cells = widths[attempt];
      break;
    }
  }
  if (path.empty()) {
    result.expanded = total_expanded;
    result.queue_pops = total_queue_pops;
    result.stale_queue_pops = total_stale_queue_pops;
    result.coarse_voxel_factor = coarse_voxel_factor_;
    result.corridor_half_width_cells = widths.back();
    result.corridor_widenings = widths.size() - 1U;
    result.failure_reason = "unreachable_current_map";
    return result;
  }
  const auto simplified = SimplifyPath(path);
  result.success = true;
  result.incremental_repair = repair;
  result.expanded = total_expanded;
  result.queue_pops = total_queue_pops;
  result.stale_queue_pops = total_stale_queue_pops;
  result.coarse_voxel_factor = coarse_voxel_factor_;
  result.corridor = escape;
  if (result.corridor.empty()) {
    result.corridor.push_back(start);
  }
  result.corridor.insert(result.corridor.end(), path.begin() + 1U, path.end());
  result.waypoints_world.reserve(escape.size() + simplified.size() + 1U);
  result.waypoints_world.push_back(start_world);
  for (std::size_t index = 1U; index < escape.size(); ++index) {
    result.waypoints_world.push_back(ToCenter(escape[index]));
  }
  for (std::size_t index = 1U; index + 1U < simplified.size(); ++index) {
    result.waypoints_world.push_back(ToCenter(simplified[index]));
  }
  result.waypoints_world.push_back(goal_world);
  return result;
}

bool DStarLitePlanner::IsInside(const VoxelKey & state) const
{
  return IsInsideSearchVolume(state) && IsInsideGuide(state);
}

bool DStarLitePlanner::IsInsideSearchVolume(const VoxelKey & state) const
{
  const Vec3 center = ToCenter(state);
  return center.x >= search_volume_.min.x && center.x <= search_volume_.max.x &&
         center.y >= search_volume_.min.y && center.y <= search_volume_.max.y &&
         center.z >= search_volume_.min.z && center.z <= search_volume_.max.z;
}

bool DStarLitePlanner::IsInsideGuide(const VoxelKey & state) const
{
  if (guide_half_width_cells_ == 0 || guide_cells_.size() < 2U) {
    return true;
  }
  const double limit = static_cast<double>(guide_half_width_cells_);
  const double px = static_cast<double>(state.ix);
  const double py = static_cast<double>(state.iy);
  const double pz = static_cast<double>(state.iz);
  for (std::size_t index = 1U; index < guide_cells_.size(); ++index) {
    const auto & from = guide_cells_[index - 1U];
    const auto & to = guide_cells_[index];
    const double vx = static_cast<double>(to.ix - from.ix);
    const double vy = static_cast<double>(to.iy - from.iy);
    const double vz = static_cast<double>(to.iz - from.iz);
    const double length_squared = vx * vx + vy * vy + vz * vz;
    const double projection = length_squared <= kEpsilon ? 0.0 : std::clamp(
      ((px - from.ix) * vx + (py - from.iy) * vy + (pz - from.iz) * vz) /
      length_squared, 0.0, 1.0);
    const double dx = px - (from.ix + projection * vx);
    const double dy = py - (from.iy + projection * vy);
    const double dz = pz - (from.iz + projection * vz);
    if (dx * dx + dy * dy + dz * dz <= limit * limit) {
      return true;
    }
  }
  return false;
}

bool DStarLitePlanner::UpdateSearchVolume(const Vec3 & start_world, const Vec3 & goal_world)
{
  const double margin = parameters_.search_margin_m;
  const AxisAlignedBox raw{
    Vec3{
      std::max(hard_flight_volume_.min.x, std::min(start_world.x, goal_world.x) - margin),
      std::max(hard_flight_volume_.min.y, std::min(start_world.y, goal_world.y) - margin),
      std::max(hard_flight_volume_.min.z, std::min(start_world.z, goal_world.z) - margin)},
    Vec3{
      std::min(hard_flight_volume_.max.x, std::max(start_world.x, goal_world.x) + margin),
      std::min(hard_flight_volume_.max.y, std::max(start_world.y, goal_world.y) + margin),
      std::min(hard_flight_volume_.max.z, std::max(start_world.z, goal_world.z) + margin)}};
  // La ventana debe permanecer estable ante el ruido subvoxel de la pose.
  const AxisAlignedBox next{ToCenter(ToKey(raw.min)), ToCenter(ToKey(raw.max))};
  const bool changed = std::abs(next.min.x - search_volume_.min.x) > kEpsilon ||
    std::abs(next.min.y - search_volume_.min.y) > kEpsilon ||
    std::abs(next.min.z - search_volume_.min.z) > kEpsilon ||
    std::abs(next.max.x - search_volume_.max.x) > kEpsilon ||
    std::abs(next.max.y - search_volume_.max.y) > kEpsilon ||
    std::abs(next.max.z - search_volume_.max.z) > kEpsilon;
  if (changed) {
    search_volume_ = next;
    initialized_ = false;
  }
  return false;
}

VoxelKey DStarLitePlanner::ToKey(const Vec3 & position_world) const
{
  return VoxelKey{
    static_cast<std::int64_t>(std::floor(position_world.x / voxel_size_)),
    static_cast<std::int64_t>(std::floor(position_world.y / voxel_size_)),
    static_cast<std::int64_t>(std::floor(position_world.z / voxel_size_))};
}

Vec3 DStarLitePlanner::ToCenter(const VoxelKey & state) const
{
  return Vec3{
    (static_cast<double>(state.ix) + 0.5) * voxel_size_,
    (static_cast<double>(state.iy) + 0.5) * voxel_size_,
    (static_cast<double>(state.iz) + 0.5) * voxel_size_};
}

VoxelState DStarLitePlanner::RawState(const VoxelKey & state) const
{
  if (uses_navigation_snapshot_) {
    return NavigationCellFor(state).raw_state;
  }
  const auto iterator = cells_.find(state);
  return iterator == cells_.end() ? VoxelState::Unknown : iterator->second;
}

NavigationCell DStarLitePlanner::NavigationCellFor(const VoxelKey & state) const
{
  const auto iterator = navigation_cells_.find(state);
  if (iterator != navigation_cells_.end()) {
    return iterator->second;
  }
  return NavigationCell{
    state, VoxelState::Unknown, !navigation_require_known_free_,
    navigation_require_known_free_ ? 0U : kNavigationAllNeighbors,
    navigation_unknown_cost_multiplier_};
}

NavigationCoarseCell DStarLitePlanner::NavigationCoarseCellFor(const VoxelKey & state) const
{
  const auto iterator = navigation_coarse_cells_.find(state);
  if (iterator != navigation_coarse_cells_.end()) {
    return iterator->second;
  }
  return NavigationCoarseCell{
    state, !navigation_require_known_free_, navigation_unknown_cost_multiplier_,
    navigation_require_known_free_ ? 1.0 : 0.0};
}

bool DStarLitePlanner::IsTraversable(const VoxelKey & state) const
{
  if (!IsInsideSearchVolume(state)) {
    return false;
  }
  if (uses_navigation_snapshot_) {
    return NavigationCellFor(state).traversable;
  }
  const auto inflation = parameters_.occupied_inflation_cells;
  for (std::int64_t dx = -inflation; dx <= inflation; ++dx) {
    for (std::int64_t dy = -inflation; dy <= inflation; ++dy) {
      for (std::int64_t dz = -inflation; dz <= inflation; ++dz) {
        if (RawState(VoxelKey{state.ix + dx, state.iy + dy, state.iz + dz}) ==
          VoxelState::Occupied)
        {
          return false;
        }
      }
    }
  }
  return true;
}

bool DStarLitePlanner::IsNavigationEdgeAllowed(const VoxelKey & from, const VoxelKey & to) const
{
  const auto dx = to.ix - from.ix;
  const auto dy = to.iy - from.iy;
  const auto dz = to.iz - from.iz;
  if (dx < -1 || dx > 1 || dy < -1 || dy > 1 || dz < -1 || dz > 1 ||
    (dx == 0 && dy == 0 && dz == 0))
  {
    return false;
  }
  return (NavigationCellFor(from).neighbor_mask & NavigationNeighborBit(dx, dy, dz)) != 0U;
}

std::vector<VoxelKey> DStarLitePlanner::FindSafeEscape(const VoxelKey & start) const
{
  if (RawState(start) == VoxelState::Occupied) {
    return {};
  }

  // La pose actual ya existe fisicamente: puede incumplir el clearance de la
  // ruta normal. Se busca la salida local menos costosa hasta la primera celda
  // que recupera ese perfil, sin atravesar OCCUPIED. FREE se prefiere a UNKNOWN.
  std::multimap<double, VoxelKey> pending;
  std::map<VoxelKey, double> best_cost;
  std::map<VoxelKey, VoxelKey> predecessor;
  pending.emplace(0.0, start);
  best_cost.emplace(start, 0.0);
  predecessor.emplace(start, start);
  constexpr std::int64_t offsets[][3] = {
    {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  while (!pending.empty()) {
    const auto entry = pending.begin();
    const double current_cost = entry->first;
    const auto current = entry->second;
    pending.erase(entry);
    if (current_cost > best_cost.at(current) + kEpsilon) {
      continue;
    }
    if (!Equal(current, start) && IsTraversable(current)) {
      std::vector<VoxelKey> escape;
      for (VoxelKey step = current; !Equal(step, start); step = predecessor.at(step)) {
        escape.push_back(step);
      }
      escape.push_back(start);
      std::reverse(escape.begin(), escape.end());
      return escape;
    }
    for (const auto & offset : offsets) {
      const VoxelKey candidate{
        current.ix + offset[0], current.iy + offset[1], current.iz + offset[2]};
      // La salida es local; la guia gruesa solo optimiza la ruta normal y no
      // puede impedir que el dron abandone una posicion inicial estrecha.
      if (!IsInsideSearchVolume(candidate) || RawState(candidate) == VoxelState::Occupied) {
        continue;
      }
      const double multiplier = RawState(candidate) == VoxelState::Unknown ?
        navigation_unknown_cost_multiplier_ : 1.0;
      const double candidate_cost = current_cost + multiplier;
      const auto known_cost = best_cost.find(candidate);
      if (known_cost != best_cost.end() && candidate_cost + kEpsilon >= known_cost->second) {
        continue;
      }
      best_cost[candidate] = candidate_cost;
      predecessor[candidate] = current;
      pending.emplace(candidate_cost, candidate);
    }
  }
  return {};
}

bool DStarLitePlanner::IsSafeDiagonal(const VoxelKey & from, const VoxelKey & to) const
{
  if (uses_navigation_snapshot_) {
    return IsNavigationEdgeAllowed(from, to);
  }
  const std::int64_t dx = to.ix - from.ix;
  const std::int64_t dy = to.iy - from.iy;
  const std::int64_t dz = to.iz - from.iz;
  const int components = (dx != 0) + (dy != 0) + (dz != 0);
  if (components <= 1) {
    return true;
  }
  for (int mask = 1; mask < (1 << components) - 1; ++mask) {
    int bit = 0;
    VoxelKey intermediate = from;
    if (dx != 0) {
      if ((mask & (1 << bit)) != 0) {intermediate.ix += dx;}
      ++bit;
    }
    if (dy != 0) {
      if ((mask & (1 << bit)) != 0) {intermediate.iy += dy;}
      ++bit;
    }
    if (dz != 0) {
      if ((mask & (1 << bit)) != 0) {intermediate.iz += dz;}
    }
    if (!IsTraversable(intermediate)) {
      return false;
    }
  }
  return true;
}

double DStarLitePlanner::StepCost(const VoxelKey & from, const VoxelKey & to) const
{
  if (uses_navigation_snapshot_) {
    if (!IsNavigationEdgeAllowed(from, to)) {
      return kInfinity;
    }
    return Distance(from, to) * NavigationCellFor(to).entry_cost_multiplier;
  }
  if (!IsTraversable(to) || !IsSafeDiagonal(from, to)) {
    return kInfinity;
  }
  const double multiplier = RawState(to) == VoxelState::Unknown ?
    parameters_.unknown_cost_multiplier : 1.0;
  return Distance(from, to) * multiplier;
}

std::vector<VoxelKey> DStarLitePlanner::Neighbors(const VoxelKey & state) const
{
  std::vector<VoxelKey> neighbors;
  neighbors.reserve(26U);
  for (std::int64_t dx = -1; dx <= 1; ++dx) {
    for (std::int64_t dy = -1; dy <= 1; ++dy) {
      for (std::int64_t dz = -1; dz <= 1; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) {
          continue;
        }
        const VoxelKey candidate{state.ix + dx, state.iy + dy, state.iz + dz};
        if (!IsInside(candidate)) {
          continue;
        }
        const bool allowed = uses_navigation_snapshot_ ?
          IsNavigationEdgeAllowed(state, candidate) :
          (IsTraversable(candidate) && IsSafeDiagonal(state, candidate));
        if (allowed) {
          neighbors.push_back(candidate);
        }
      }
    }
  }
  return neighbors;
}

std::vector<VoxelKey> DStarLitePlanner::CoarseNeighbors(const VoxelKey & state) const
{
  std::vector<VoxelKey> neighbors;
  neighbors.reserve(26U);
  const auto factor = coarse_voxel_factor_;
  for (std::int64_t dx = -1; dx <= 1; ++dx) {
    for (std::int64_t dy = -1; dy <= 1; ++dy) {
      for (std::int64_t dz = -1; dz <= 1; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) {
          continue;
        }
        const VoxelKey candidate{state.ix + dx, state.iy + dy, state.iz + dz};
        const VoxelKey fine_center{
          candidate.ix * factor + factor / 2, candidate.iy * factor + factor / 2,
          candidate.iz * factor + factor / 2};
        if (IsInsideSearchVolume(fine_center) && NavigationCoarseCellFor(candidate).traversable) {
          neighbors.push_back(candidate);
        }
      }
    }
  }
  return neighbors;
}

std::vector<VoxelKey> DStarLitePlanner::LineCells(const VoxelKey & from, const VoxelKey & to) const
{
  const std::int64_t dx = to.ix - from.ix;
  const std::int64_t dy = to.iy - from.iy;
  const std::int64_t dz = to.iz - from.iz;
  const std::int64_t steps = std::max({std::llabs(dx), std::llabs(dy), std::llabs(dz)}) * 2;
  std::vector<VoxelKey> cells;
  cells.reserve(static_cast<std::size_t>(steps + 1));
  for (std::int64_t index = 0; index <= steps; ++index) {
    const double fraction = steps == 0 ? 0.0 : static_cast<double>(index) / steps;
    const VoxelKey cell{
      static_cast<std::int64_t>(std::llround(static_cast<double>(from.ix) + dx * fraction)),
      static_cast<std::int64_t>(std::llround(static_cast<double>(from.iy) + dy * fraction)),
      static_cast<std::int64_t>(std::llround(static_cast<double>(from.iz) + dz * fraction))};
    if (cells.empty() || !Equal(cells.back(), cell)) {
      cells.push_back(cell);
    }
  }
  return cells;
}

bool DStarLitePlanner::HasLineOfSight(const VoxelKey & from, const VoxelKey & to) const
{
  const auto cells = LineCells(from, to);
  for (std::size_t index = 1U; index < cells.size(); ++index) {
    if (!IsTraversable(cells[index]) || !IsSafeDiagonal(cells[index - 1U], cells[index])) {
      return false;
    }
  }
  return true;
}

std::vector<VoxelKey> DStarLitePlanner::BuildCoarseGuide(
  const VoxelKey & start, const VoxelKey & goal) const
{
  if (coarse_voxel_factor_ <= 1) {
    return LineCells(start, goal);
  }
  const auto to_coarse = [this](const VoxelKey & fine) {
      const auto divide = [this](std::int64_t value) {
          return value >= 0 ? value / coarse_voxel_factor_ :
                 -(((-value) + coarse_voxel_factor_ - 1) / coarse_voxel_factor_);
        };
      return VoxelKey{divide(fine.ix), divide(fine.iy), divide(fine.iz)};
    };
  const auto coarse_start = to_coarse(start);
  const auto coarse_goal = to_coarse(goal);
  if (!NavigationCoarseCellFor(coarse_start).traversable ||
    !NavigationCoarseCellFor(coarse_goal).traversable)
  {
    return LineCells(start, goal);
  }
  struct CoarseEntry
  {
    double estimate = 0.0;
    double cost = 0.0;
    VoxelKey state;
  };
  const auto compare = [](const CoarseEntry & left, const CoarseEntry & right) {
      if (std::abs(left.estimate - right.estimate) > kEpsilon) {
        return left.estimate > right.estimate;
      }
      return DStarLitePlanner::Less(right.state, left.state);
    };
  std::priority_queue<CoarseEntry, std::vector<CoarseEntry>, decltype(compare)> open(compare);
  std::map<VoxelKey, double> costs;
  std::map<VoxelKey, VoxelKey> predecessor;
  costs.emplace(coarse_start, 0.0);
  open.push(CoarseEntry{Distance(coarse_start, coarse_goal), 0.0, coarse_start});
  std::size_t expanded = 0U;
  const std::size_t limit = std::min<std::size_t>(parameters_.max_expansions, 50000U);
  while (!open.empty() && expanded++ < limit) {
    const auto current = open.top();
    open.pop();
    const auto known = costs.find(current.state);
    if (known == costs.end() || current.cost > known->second + kEpsilon) {
      continue;
    }
    if (Equal(current.state, coarse_goal)) {
      std::vector<VoxelKey> guide;
      for (VoxelKey step = coarse_goal; !Equal(step, coarse_start); step = predecessor.at(step)) {
        guide.push_back(
          VoxelKey{
            step.ix * coarse_voxel_factor_ + coarse_voxel_factor_ / 2,
            step.iy * coarse_voxel_factor_ + coarse_voxel_factor_ / 2,
            step.iz * coarse_voxel_factor_ + coarse_voxel_factor_ / 2});
      }
      guide.push_back(start);
      std::reverse(guide.begin(), guide.end());
      guide.push_back(goal);
      return guide;
    }
    for (const auto & neighbor : CoarseNeighbors(current.state)) {
      const auto descriptor = NavigationCoarseCellFor(neighbor);
      const double multiplier = descriptor.entry_cost_multiplier + 3.0 *
        descriptor.blocked_fraction;
      const double next_cost = current.cost + Distance(current.state, neighbor) * multiplier;
      const auto existing = costs.find(neighbor);
      if (existing != costs.end() && next_cost + kEpsilon >= existing->second) {
        continue;
      }
      costs[neighbor] = next_cost;
      predecessor[neighbor] = current.state;
      open.push(
        CoarseEntry{
          next_cost + Distance(neighbor, coarse_goal), next_cost, neighbor});
    }
  }
  return LineCells(start, goal);
}

bool DStarLitePlanner::SetGuide(
  const std::vector<VoxelKey> & guide, std::int64_t half_width_cells)
{
  const bool changed = guide_cells_ != guide || guide_half_width_cells_ != half_width_cells;
  guide_cells_ = guide;
  guide_half_width_cells_ = half_width_cells;
  if (changed) {
    initialized_ = false;
  }
  return changed;
}

void DStarLitePlanner::ResetSearch(const VoxelKey & start, const VoxelKey & goal)
{
  g_.clear();
  rhs_.clear();
  open_positions_.clear();
  open_.clear();
  expanded_ = 0U;
  queue_pops_ = 0U;
  stale_queue_pops_ = 0U;
  start_ = start;
  last_start_ = start;
  goal_ = goal;
  km_ = 0.0;
  initialized_ = true;
  SetRhs(goal_, 0.0);
  PushOpen(goal_);
}

void DStarLitePlanner::PushOpen(const VoxelKey & state)
{
  const QueueEntry updated{CalculateKey(state), state};
  const auto existing = open_positions_.find(state);
  if (existing == open_positions_.end()) {
    open_positions_[state] = open_.size();
    open_.push_back(updated);
    SiftOpenUp(open_.size() - 1U);
    return;
  }
  const auto index = existing->second;
  const auto previous = open_[index];
  open_[index] = updated;
  if (QueueEntryLess(updated, previous)) {
    SiftOpenUp(index);
  } else if (QueueEntryLess(previous, updated)) {
    SiftOpenDown(index);
  }
}

void DStarLitePlanner::RemoveOpen(const VoxelKey & state)
{
  const auto existing = open_positions_.find(state);
  if (existing == open_positions_.end()) {
    return;
  }
  const auto index = existing->second;
  const auto last = open_.size() - 1U;
  open_positions_.erase(existing);
  if (index == last) {
    open_.pop_back();
    return;
  }
  open_[index] = open_.back();
  open_.pop_back();
  open_positions_[open_[index].state] = index;
  if (index > 0U && QueueEntryLess(open_[index], open_[(index - 1U) / 2U])) {
    SiftOpenUp(index);
  } else {
    SiftOpenDown(index);
  }
}

DStarLitePlanner::QueueEntry DStarLitePlanner::PopOpen()
{
  const auto entry = open_.front();
  RemoveOpen(entry.state);
  return entry;
}

void DStarLitePlanner::SiftOpenUp(std::size_t index)
{
  while (index > 0U) {
    const auto parent = (index - 1U) / 2U;
    if (!QueueEntryLess(open_[index], open_[parent])) {
      return;
    }
    SwapOpen(index, parent);
    index = parent;
  }
}

void DStarLitePlanner::SiftOpenDown(std::size_t index)
{
  while (true) {
    const auto left = index * 2U + 1U;
    if (left >= open_.size()) {
      return;
    }
    auto best = left;
    const auto right = left + 1U;
    if (right < open_.size() && QueueEntryLess(open_[right], open_[left])) {
      best = right;
    }
    if (!QueueEntryLess(open_[best], open_[index])) {
      return;
    }
    SwapOpen(index, best);
    index = best;
  }
}

void DStarLitePlanner::SwapOpen(std::size_t left, std::size_t right)
{
  std::swap(open_[left], open_[right]);
  open_positions_[open_[left].state] = left;
  open_positions_[open_[right].state] = right;
}

void DStarLitePlanner::UpdateVertex(const VoxelKey & state)
{
  if (!IsInside(state)) {
    return;
  }
  if (!Equal(state, goal_)) {
    double best = kInfinity;
    for (const auto & neighbor : Neighbors(state)) {
      best = std::min(best, StepCost(state, neighbor) + G(neighbor));
    }
    SetRhs(state, best);
  }
  if (std::abs(G(state) - Rhs(state)) > kEpsilon) {
    PushOpen(state);
  } else {
    RemoveOpen(state);
  }
}

bool DStarLitePlanner::ComputeShortestPath()
{
  expanded_ = 0U;
  queue_pops_ = 0U;
  stale_queue_pops_ = 0U;
  while (!open_.empty() &&
    (KeyLess(
      open_.front().key,
      CalculateKey(start_)) || std::abs(Rhs(start_) - G(start_)) > kEpsilon))
  {
    const auto entry = PopOpen();
    ++queue_pops_;
    const auto current_key = CalculateKey(entry.state);
    if (KeyLess(entry.key, current_key)) {
      PushOpen(entry.state);
      continue;
    }
    if (std::abs(G(entry.state) - Rhs(entry.state)) <= kEpsilon) {
      ++stale_queue_pops_;
      continue;
    }
    if (++expanded_ > parameters_.max_expansions) {
      return false;
    }
    if (G(entry.state) > Rhs(entry.state)) {
      SetG(entry.state, Rhs(entry.state));
      for (const auto & predecessor : Neighbors(entry.state)) {
        UpdateVertex(predecessor);
      }
    } else {
      SetG(entry.state, kInfinity);
      UpdateVertex(entry.state);
      for (const auto & predecessor : Neighbors(entry.state)) {
        UpdateVertex(predecessor);
      }
    }
  }
  return true;
}

void DStarLitePlanner::UpdateAffected(const VoxelKey & changed)
{
  if (uses_navigation_snapshot_) {
    for (std::int64_t dx = -1; dx <= 1; ++dx) {
      for (std::int64_t dy = -1; dy <= 1; ++dy) {
        for (std::int64_t dz = -1; dz <= 1; ++dz) {
          UpdateVertex(VoxelKey{changed.ix + dx, changed.iy + dy, changed.iz + dz});
        }
      }
    }
    return;
  }
  const auto radius = parameters_.occupied_inflation_cells + 1;
  for (std::int64_t dx = -radius; dx <= radius; ++dx) {
    for (std::int64_t dy = -radius; dy <= radius; ++dy) {
      for (std::int64_t dz = -radius; dz <= radius; ++dz) {
        UpdateVertex(VoxelKey{changed.ix + dx, changed.iy + dy, changed.iz + dz});
      }
    }
  }
}

DStarLitePlanner::QueueKey DStarLitePlanner::CalculateKey(const VoxelKey & state) const
{
  const double minimum = std::min(G(state), Rhs(state));
  return QueueKey{
    minimum + parameters_.heuristic_weight * Distance(start_, state) + km_, minimum};
}

double DStarLitePlanner::G(const VoxelKey & state) const
{
  const auto iterator = g_.find(state);
  return iterator == g_.end() ? kInfinity : iterator->second;
}

double DStarLitePlanner::Rhs(const VoxelKey & state) const
{
  const auto iterator = rhs_.find(state);
  return iterator == rhs_.end() ? kInfinity : iterator->second;
}

void DStarLitePlanner::SetG(const VoxelKey & state, double value)
{
  g_[state] = value;
}

void DStarLitePlanner::SetRhs(const VoxelKey & state, double value)
{
  rhs_[state] = value;
}

std::vector<VoxelKey> DStarLitePlanner::BuildPath() const
{
  if (!std::isfinite(G(start_))) {
    return {};
  }
  std::vector<VoxelKey> path{start_};
  VoxelKey current = start_;
  const std::size_t limit = static_cast<std::size_t>(parameters_.max_expansions);
  for (std::size_t iteration = 0; iteration < limit && !Equal(current, goal_); ++iteration) {
    double best = kInfinity;
    VoxelKey next;
    bool found = false;
    for (const auto & neighbor : Neighbors(current)) {
      const double candidate = StepCost(current, neighbor) + G(neighbor);
      if (candidate + kEpsilon < best ||
        (std::abs(candidate - best) <= kEpsilon && (!found || Less(neighbor, next))))
      {
        best = candidate;
        next = neighbor;
        found = true;
      }
    }
    if (!found || !std::isfinite(best) ||
      std::find_if(
        path.begin(), path.end(),
        [&next](const auto & item) {return Equal(item, next);}) != path.end())
    {
      return {};
    }
    current = next;
    path.push_back(current);
  }
  return Equal(path.back(), goal_) ? path : std::vector<VoxelKey>{};
}

std::vector<VoxelKey> DStarLitePlanner::SimplifyPath(const std::vector<VoxelKey> & path) const
{
  if (path.size() <= 2U) {
    return path;
  }
  std::vector<VoxelKey> simplified{path.front()};
  std::size_t index = 0U;
  while (index + 1U < path.size()) {
    std::size_t furthest = index + 1U;
    for (std::size_t candidate = path.size() - 1U; candidate > index + 1U; --candidate) {
      if (HasLineOfSight(path[index], path[candidate])) {
        furthest = candidate;
        break;
      }
    }
    simplified.push_back(path[furthest]);
    index = furthest;
  }
  return simplified;
}

}  // namespace task_lib
