#include "task_lib/voxel_map.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace task_lib
{
namespace
{

bool SameNavigationCell(const NavigationCell & left, const NavigationCell & right)
{
  return left.key == right.key && left.raw_state == right.raw_state &&
         left.traversable == right.traversable && left.neighbor_mask == right.neighbor_mask &&
         std::abs(left.entry_cost_multiplier - right.entry_cost_multiplier) <= 1e-9;
}

bool SameNavigationProfile(const NavigationProfile & left, const NavigationProfile & right)
{
  return left.profile_id == right.profile_id &&
         left.obstacle_inflation_cells == right.obstacle_inflation_cells &&
         std::abs(left.unknown_cost_multiplier - right.unknown_cost_multiplier) <= 1e-9 &&
         left.coarse_voxel_factor == right.coarse_voxel_factor &&
         left.require_known_free == right.require_known_free;
}

bool SameNavigationCoarseCell(
  const NavigationCoarseCell & left, const NavigationCoarseCell & right)
{
  return left.key == right.key && left.traversable == right.traversable &&
         std::abs(left.entry_cost_multiplier - right.entry_cost_multiplier) <= 1e-9 &&
         std::abs(left.blocked_fraction - right.blocked_fraction) <= 1e-9;
}

}  // namespace

bool VoxelKey::operator==(const VoxelKey & other) const
{
  return ix == other.ix && iy == other.iy && iz == other.iz;
}

bool VoxelKey::operator<(const VoxelKey & other) const
{
  if (ix != other.ix) {
    return ix < other.ix;
  }
  if (iy != other.iy) {
    return iy < other.iy;
  }
  return iz < other.iz;
}

ReversibleVoxelMap::ReversibleVoxelMap(double voxel_size)
: voxel_size_(voxel_size)
{
  if (!std::isfinite(voxel_size_) || voxel_size_ <= 0.0) {
    throw std::invalid_argument("voxel_size debe ser positivo y finito");
  }
}

bool ReversibleVoxelMap::ApplySparseSnapshot(
  const std::vector<SparseEvidence> & evidence, float occupied_score_threshold)
{
  if (!std::isfinite(occupied_score_threshold) || occupied_score_threshold < 0.0F ||
    occupied_score_threshold > 1.0F)
  {
    throw std::invalid_argument("occupied_score_threshold debe estar en [0, 1]");
  }
  sparse_occupied_score_threshold_ = occupied_score_threshold;
  ContributionsBySource next;
  ContributionsByVoxel next_by_voxel;
  for (const auto & point : evidence) {
    if (point.source_id.empty() || !std::isfinite(point.position_world.x) ||
      !std::isfinite(point.position_world.y) || !std::isfinite(point.position_world.z))
    {
      continue;
    }
    if (!std::isfinite(point.score)) {
      continue;
    }
    const double score = std::clamp(static_cast<double>(point.score), 0.0, 1.0);
    const auto contribution = Contribution{ToKey(point.position_world), score};
    const auto inserted = next.emplace(point.source_id, contribution);
    if (inserted.second) {
      next_by_voxel[contribution.key].emplace(point.source_id, contribution);
    }
  }

  std::set<VoxelKey> affected;
  for (const auto & existing : sparse_contributions_) {
    affected.insert(existing.second.key);
  }
  for (const auto & incoming : next) {
    affected.insert(incoming.second.key);
  }
  ContributionsByVoxel before;
  std::map<VoxelKey, VoxelState> states_before;
  for (const auto & key : affected) {
    const auto existing = sparse_contributions_by_voxel_.find(key);
    if (existing != sparse_contributions_by_voxel_.end()) {
      before.emplace(key, existing->second);
    }
    states_before.emplace(key, StateForKey(key));
  }
  sparse_contributions_ = std::move(next);
  sparse_contributions_by_voxel_ = std::move(next_by_voxel);
  const bool changed = ReconcileSparseCells(before, affected);
  if (changed) {
    ++revision_;
    for (const auto & item : states_before) {
      RecordChange(item.first, item.second);
    }
  }
  return changed;
}

bool ReversibleVoxelMap::ApplySparseDelta(
  const std::vector<SparseEvidence> & upserts, const std::vector<std::string> & deletes,
  float occupied_score_threshold)
{
  if (!std::isfinite(occupied_score_threshold) || occupied_score_threshold < 0.0F ||
    occupied_score_threshold > 1.0F)
  {
    throw std::invalid_argument("occupied_score_threshold debe estar en [0, 1]");
  }
  sparse_occupied_score_threshold_ = occupied_score_threshold;
  ContributionsBySource next;
  std::set<std::string> sources(deletes.begin(), deletes.end());
  for (const auto & point : upserts) {
    if (point.source_id.empty() || !std::isfinite(point.position_world.x) ||
      !std::isfinite(point.position_world.y) || !std::isfinite(point.position_world.z) ||
      !std::isfinite(point.score))
    {
      continue;
    }
    sources.insert(point.source_id);
    next.emplace(
      point.source_id, Contribution{
        ToKey(point.position_world), std::clamp(static_cast<double>(point.score), 0.0, 1.0)});
  }

  std::set<VoxelKey> affected;
  for (const auto & source : sources) {
    const auto existing = sparse_contributions_.find(source);
    if (existing != sparse_contributions_.end()) {
      affected.insert(existing->second.key);
    }
    const auto incoming = next.find(source);
    if (incoming != next.end()) {
      affected.insert(incoming->second.key);
    }
  }
  ContributionsByVoxel before;
  std::map<VoxelKey, VoxelState> states_before;
  for (const auto & key : affected) {
    const auto existing = sparse_contributions_by_voxel_.find(key);
    if (existing != sparse_contributions_by_voxel_.end()) {
      before.emplace(key, existing->second);
    }
    states_before.emplace(key, StateForKey(key));
  }
  for (const auto & source : sources) {
    const auto existing = sparse_contributions_.find(source);
    const auto incoming = next.find(source);
    if (existing != sparse_contributions_.end() && incoming != next.end() &&
      SameContribution(existing->second, incoming->second))
    {
      continue;
    }
    if (existing != sparse_contributions_.end()) {
      auto by_voxel = sparse_contributions_by_voxel_.find(existing->second.key);
      if (by_voxel != sparse_contributions_by_voxel_.end()) {
        by_voxel->second.erase(source);
        if (by_voxel->second.empty()) {
          sparse_contributions_by_voxel_.erase(by_voxel);
        }
      }
      sparse_contributions_.erase(existing);
    }
    if (incoming != next.end()) {
      sparse_contributions_.emplace(source, incoming->second);
      sparse_contributions_by_voxel_[incoming->second.key].emplace(source, incoming->second);
    }
  }
  const bool changed = ReconcileSparseCells(before, affected);
  if (changed) {
    ++revision_;
    for (const auto & item : states_before) {
      RecordChange(item.first, item.second);
    }
  }
  return changed;
}

bool ReversibleVoxelMap::ReconcileSparseCells(
  const ContributionsByVoxel & before, const std::set<VoxelKey> & affected)
{
  bool changed = false;
  for (const auto & key : affected) {
    const auto old = before.find(key);
    const ContributionsBySource empty;
    const auto & old_sources = old == before.end() ? empty : old->second;
    const auto current = sparse_contributions_by_voxel_.find(key);
    const auto & current_sources =
      current == sparse_contributions_by_voxel_.end() ? empty : current->second;
    if (SameContributions(old_sources, current_sources)) {
      continue;
    }
    for (const auto & item : old_sources) {
      ApplySparseContribution(item.second, -1.0);
    }
    for (const auto & item : current_sources) {
      ApplySparseContribution(item.second, 1.0);
    }
    changed = true;
  }
  return changed;
}

bool ReversibleVoxelMap::SameContribution(const Contribution & left, const Contribution & right)
{
  return left.key == right.key && std::abs(left.occupied - right.occupied) <= 1e-9;
}

bool ReversibleVoxelMap::SameContributions(
  const ContributionsBySource & left, const ContributionsBySource & right)
{
  if (left.size() != right.size()) {
    return false;
  }
  auto left_it = left.begin();
  auto right_it = right.begin();
  while (left_it != left.end()) {
    if (left_it->first != right_it->first || !SameContribution(left_it->second, right_it->second)) {
      return false;
    }
    ++left_it;
    ++right_it;
  }
  return true;
}

bool ReversibleVoxelMap::AddFreeEvidence(const std::string & source_id, const Vec3 & position_world)
{
  return ReplaceFreeVolume(source_id, position_world, Vec3{});
}

bool ReversibleVoxelMap::AddDepthFreeEvidence(
  const std::string & source_id, const Vec3 & position_world)
{
  return ReplaceDepthFreeVolume(source_id, position_world, Vec3{});
}

bool ReversibleVoxelMap::ReplaceFreeVolume(
  const std::string & source_id, const Vec3 & center_world,
  const Vec3 & half_extent_world)
{
  return ReplaceFreeVolumeImpl(source_id, center_world, half_extent_world, true);
}

bool ReversibleVoxelMap::ReplaceDepthFreeVolume(
  const std::string & source_id, const Vec3 & center_world,
  const Vec3 & half_extent_world)
{
  return ReplaceFreeVolumeImpl(source_id, center_world, half_extent_world, false);
}

bool ReversibleVoxelMap::ReplaceDepthFreeCells(
  const std::string & source_id, const std::set<VoxelKey> & keys)
{
  return ReplaceFreeCellsImpl(source_id, keys, false);
}

bool ReversibleVoxelMap::ReplaceDirectDepthFreeCells(
  const std::string & source_id, const std::set<VoxelKey> & keys)
{
  return ReplaceFreeCellsImpl(source_id, keys, true);
}

bool ReversibleVoxelMap::ReplaceDepthOccupiedCells(
  const std::string & source_id, const std::set<VoxelKey> & keys)
{
  return ReplaceDepthOccupiedCellsImpl(source_id, keys);
}

bool ReversibleVoxelMap::ReplaceFreeVolumeImpl(
  const std::string & source_id, const Vec3 & center_world,
  const Vec3 & half_extent_world, bool overrides_sparse)
{
  if (source_id.empty() || !std::isfinite(center_world.x) || !std::isfinite(center_world.y) ||
    !std::isfinite(center_world.z) || !std::isfinite(half_extent_world.x) ||
    !std::isfinite(half_extent_world.y) || !std::isfinite(half_extent_world.z) ||
    half_extent_world.x < 0.0 || half_extent_world.y < 0.0 || half_extent_world.z < 0.0)
  {
    return false;
  }
  return ReplaceFreeCellsImpl(
    source_id, KeysForVolume(center_world, half_extent_world), overrides_sparse);
}

bool ReversibleVoxelMap::ReplaceFreeCellsImpl(
  const std::string & source_id, const std::set<VoxelKey> & keys,
  bool overrides_sparse)
{
  if (source_id.empty() || keys.empty()) {
    return false;
  }
  const FreeContribution next{keys, overrides_sparse};
  const auto existing = free_contributions_.find(source_id);
  if (existing != free_contributions_.end() && existing->second.keys == next.keys &&
    existing->second.overrides_sparse == next.overrides_sparse)
  {
    return false;
  }
  std::set<VoxelKey> affected = next.keys;
  if (existing != free_contributions_.end()) {
    affected.insert(existing->second.keys.begin(), existing->second.keys.end());
  }
  std::map<VoxelKey, VoxelState> before;
  for (const auto & key : affected) {
    before.emplace(key, StateForKey(key));
  }
  if (existing != free_contributions_.end()) {
    ApplyFreeContribution(existing->second, -1.0);
  }
  ApplyFreeContribution(next, 1.0);
  free_contributions_[source_id] = next;
  ++revision_;
  for (const auto & item : before) {
    RecordChange(item.first, item.second);
  }
  return true;
}

bool ReversibleVoxelMap::ReplaceDepthOccupiedCellsImpl(
  const std::string & source_id, const std::set<VoxelKey> & keys)
{
  if (source_id.empty() || keys.empty()) {
    return false;
  }
  const auto existing = depth_occupied_contributions_.find(source_id);
  if (existing != depth_occupied_contributions_.end() && existing->second == keys) {
    return false;
  }
  std::set<VoxelKey> affected = keys;
  if (existing != depth_occupied_contributions_.end()) {
    affected.insert(existing->second.begin(), existing->second.end());
  }
  std::map<VoxelKey, VoxelState> before;
  for (const auto & key : affected) {
    before.emplace(key, StateForKey(key));
  }
  if (existing != depth_occupied_contributions_.end()) {
    for (const auto & key : existing->second) {
      auto & accumulator = cells_[key];
      accumulator.depth_occupied = std::max(0.0, accumulator.depth_occupied - 1.0);
    }
  }
  for (const auto & key : keys) {
    ++cells_[key].depth_occupied;
  }
  depth_occupied_contributions_[source_id] = keys;
  ++revision_;
  for (const auto & item : before) {
    RecordChange(item.first, item.second);
  }
  return true;
}

bool ReversibleVoxelMap::RemoveEvidence(const std::string & source_id)
{
  const auto sparse = sparse_contributions_.find(source_id);
  if (sparse != sparse_contributions_.end()) {
    const auto key = sparse->second.key;
    const auto before = StateForKey(key);
    ApplySparseContribution(sparse->second, -1.0);
    auto by_voxel = sparse_contributions_by_voxel_.find(key);
    if (by_voxel != sparse_contributions_by_voxel_.end()) {
      by_voxel->second.erase(source_id);
      if (by_voxel->second.empty()) {
        sparse_contributions_by_voxel_.erase(by_voxel);
      }
    }
    sparse_contributions_.erase(sparse);
    ++revision_;
    RecordChange(key, before);
    return true;
  }
  const auto depth_occupied = depth_occupied_contributions_.find(source_id);
  if (depth_occupied != depth_occupied_contributions_.end()) {
    std::map<VoxelKey, VoxelState> before;
    for (const auto & key : depth_occupied->second) {
      before.emplace(key, StateForKey(key));
      auto & accumulator = cells_[key];
      accumulator.depth_occupied = std::max(0.0, accumulator.depth_occupied - 1.0);
    }
    depth_occupied_contributions_.erase(depth_occupied);
    ++revision_;
    for (const auto & item : before) {
      RecordChange(item.first, item.second);
    }
    return true;
  }
  const auto free = free_contributions_.find(source_id);
  if (free == free_contributions_.end()) {
    return false;
  }
  std::map<VoxelKey, VoxelState> before;
  for (const auto & key : free->second.keys) {
    before.emplace(key, StateForKey(key));
  }
  ApplyFreeContribution(free->second, -1.0);
  free_contributions_.erase(free);
  ++revision_;
  for (const auto & item : before) {
    RecordChange(item.first, item.second);
  }
  return true;
}

bool ReversibleVoxelMap::RegisterNavigationProfile(const NavigationProfile & profile)
{
  if (profile.profile_id.empty() || profile.obstacle_inflation_cells.ix < 0 ||
    profile.obstacle_inflation_cells.iy < 0 || profile.obstacle_inflation_cells.iz < 0 ||
    !std::isfinite(profile.unknown_cost_multiplier) || profile.unknown_cost_multiplier < 1.0 ||
    profile.coarse_voxel_factor < 1)
  {
    return false;
  }
  const auto existing = navigation_layers_.find(profile.profile_id);
  if (existing != navigation_layers_.end()) {
    return SameNavigationProfile(existing->second.profile, profile);
  }
  navigation_layers_.emplace(profile.profile_id, NavigationLayer{profile, {}, {}, {}});
  std::vector<VoxelChange> initial_changes;
  initial_changes.reserve(cells_.size());
  for (const auto & cell : cells_) {
    initial_changes.push_back(
      VoxelChange{cell.first, VoxelState::Unknown,
        StateForKey(cell.first)});
  }
  RefreshNavigation(profile.profile_id, initial_changes);
  return true;
}

NavigationSnapshot ReversibleVoxelMap::NavigationSnapshotFor(const std::string & profile_id) const
{
  NavigationSnapshot snapshot;
  const auto layer = navigation_layers_.find(profile_id);
  if (layer == navigation_layers_.end()) {
    return snapshot;
  }
  snapshot.profile_id = profile_id;
  snapshot.raw_revision = revision_;
  snapshot.unknown_cost_multiplier = layer->second.profile.unknown_cost_multiplier;
  snapshot.coarse_voxel_factor = layer->second.profile.coarse_voxel_factor;
  snapshot.require_known_free = layer->second.profile.require_known_free;
  snapshot.cells.reserve(layer->second.cells.size());
  for (const auto & cell : layer->second.cells) {
    snapshot.cells.push_back(cell.second);
  }
  snapshot.coarse_cells.reserve(layer->second.coarse_cells.size());
  for (const auto & cell : layer->second.coarse_cells) {
    snapshot.coarse_cells.push_back(cell.second);
  }
  return snapshot;
}

NavigationUpdate ReversibleVoxelMap::RefreshNavigation(
  const std::string & profile_id, const std::vector<VoxelChange> & raw_changes)
{
  NavigationUpdate update;
  update.profile_id = profile_id;
  update.raw_revision = revision_;
  const auto layer = navigation_layers_.find(profile_id);
  if (layer == navigation_layers_.end() || raw_changes.empty()) {
    return update;
  }
  const auto started = std::chrono::steady_clock::now();
  const auto affected = NavigationAffectedKeys(raw_changes, layer->second.profile);
  update.recomputed_cells = affected.size();

  // Maintain reverse occupancy influence counts first. A raw occupied voxel only
  // touches the navigation cells inside its inflation volume; no cell rescans
  // that same volume to rediscover the answer.
  for (const auto & raw : raw_changes) {
    if (IsOccupied(raw.before) == IsOccupied(raw.after)) {
      continue;
    }
    ApplyObstacleInfluence(
      &layer->second, raw.key, IsOccupied(raw.after) ? 1 : -1);
  }

  // The following edge pass only consults counts cached above or stable cells.
  std::map<VoxelKey, NavigationCell> recomputed;
  for (const auto & key : affected) {
    const auto raw_state = StateForKey(key);
    const auto influence = layer->second.occupied_influence_counts.find(key);
    const bool known_free = raw_state == VoxelState::Free;
    const auto traversable =
      (!layer->second.profile.require_known_free || known_free) &&
      (influence == layer->second.occupied_influence_counts.end() || influence->second <= 0);
    recomputed.emplace(
      key,
      NavigationCell{
        key, raw_state, traversable, traversable ? kNavigationAllNeighbors : 0U,
        raw_state == VoxelState::Unknown ? layer->second.profile.unknown_cost_multiplier : 1.0});
  }

  const auto traversable_after = [&recomputed, &layer](const VoxelKey & key) {
      const auto local = recomputed.find(key);
      if (local != recomputed.end()) {
        return local->second.traversable;
      }
      const auto stable = layer->second.cells.find(key);
      return stable == layer->second.cells.end() ?
             !layer->second.profile.require_known_free : stable->second.traversable;
    };
  const auto safe_diagonal = [&traversable_after](
    const VoxelKey & from, std::int64_t dx, std::int64_t dy, std::int64_t dz) {
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
        if (dz != 0 && (mask & (1 << bit)) != 0) {
          intermediate.iz += dz;
        }
        if (!traversable_after(intermediate)) {
          return false;
        }
      }
      return true;
    };
  for (auto & item : recomputed) {
    auto & cell = item.second;
    if (!cell.traversable) {
      continue;
    }
    cell.neighbor_mask = 0U;
    for (std::int64_t dx = -1; dx <= 1; ++dx) {
      for (std::int64_t dy = -1; dy <= 1; ++dy) {
        for (std::int64_t dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const VoxelKey candidate{item.first.ix + dx, item.first.iy + dy, item.first.iz + dz};
          if (traversable_after(candidate) && safe_diagonal(item.first, dx, dy, dz)) {
            cell.neighbor_mask |= NavigationNeighborBit(dx, dy, dz);
          }
        }
      }
    }
  }
  for (const auto & item : recomputed) {
    const auto & key = item.first;
    const auto previous = layer->second.cells.find(key);
    const auto before = previous == layer->second.cells.end() ?
      NavigationCell{key, VoxelState::Unknown, !layer->second.profile.require_known_free,
      layer->second.profile.require_known_free ? 0U : kNavigationAllNeighbors,
      layer->second.profile.unknown_cost_multiplier} : previous->second;
    const auto & after = item.second;
    if (SameNavigationCell(before, after)) {
      continue;
    }
    if (IsNavigationDefault(after, layer->second.profile)) {
      if (previous != layer->second.cells.end()) {
        layer->second.cells.erase(previous);
      }
    } else {
      layer->second.cells[key] = after;
    }
    update.changes.push_back(NavigationChange{key, before, after});
  }
  std::set<VoxelKey> coarse_affected;
  for (const auto & change : update.changes) {
    coarse_affected.insert(CoarseKeyFor(change.key, layer->second.profile.coarse_voxel_factor));
  }
  for (const auto & key : coarse_affected) {
    const auto previous = layer->second.coarse_cells.find(key);
    const NavigationCoarseCell before = previous == layer->second.coarse_cells.end() ?
      NavigationCoarseCell{key, !layer->second.profile.require_known_free,
      layer->second.profile.unknown_cost_multiplier,
      layer->second.profile.require_known_free ? 1.0 : 0.0} :
    previous->second;
    const auto after = NavigationCoarseCellFor(key, layer->second);
    if (SameNavigationCoarseCell(before, after)) {
      continue;
    }
    if (IsNavigationCoarseDefault(after, layer->second.profile)) {
      if (previous != layer->second.coarse_cells.end()) {
        layer->second.coarse_cells.erase(previous);
      }
    } else {
      layer->second.coarse_cells[key] = after;
    }
    update.coarse_changes.push_back(NavigationCoarseChange{key, before, after});
  }
  update.elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  return update;
}

double ReversibleVoxelMap::voxel_size() const
{
  return voxel_size_;
}

std::uint64_t ReversibleVoxelMap::revision() const
{
  return revision_;
}

VoxelState ReversibleVoxelMap::StateAt(const VoxelKey & key) const
{
  return StateForKey(key);
}

std::vector<VoxelCell> ReversibleVoxelMap::Snapshot() const
{
  std::vector<VoxelCell> snapshot;
  snapshot.reserve(cells_.size());
  for (const auto & item : cells_) {
    const auto & accumulator = item.second;
    if (accumulator.sparse_count == 0U && accumulator.traversed_free <= 1e-9 &&
      accumulator.depth_free <= 1e-9 && accumulator.depth_occupied <= 1e-9)
    {
      continue;
    }
    VoxelCell cell;
    cell.key = item.first;
    cell.state = StateForKey(item.first);
    cell.score = static_cast<float>(ScoreForKey(item.first));
    snapshot.push_back(cell);
  }
  return snapshot;
}

std::vector<VoxelChange> ReversibleVoxelMap::TakeChanges()
{
  std::vector<VoxelChange> changes;
  changes.swap(pending_changes_);
  return changes;
}

VoxelKey ReversibleVoxelMap::ToKey(const Vec3 & position_world) const
{
  return {
    static_cast<std::int64_t>(std::floor(position_world.x / voxel_size_)),
    static_cast<std::int64_t>(std::floor(position_world.y / voxel_size_)),
    static_cast<std::int64_t>(std::floor(position_world.z / voxel_size_))};
}

std::set<VoxelKey> ReversibleVoxelMap::KeysForVolume(
  const Vec3 & center, const Vec3 & half_extent) const
{
  const Vec3 minimum{
    center.x - half_extent.x, center.y - half_extent.y, center.z - half_extent.z};
  const Vec3 maximum{
    std::nextafter(center.x + half_extent.x, -std::numeric_limits<double>::infinity()),
    std::nextafter(center.y + half_extent.y, -std::numeric_limits<double>::infinity()),
    std::nextafter(center.z + half_extent.z, -std::numeric_limits<double>::infinity())};
  const auto min_key = ToKey(minimum);
  const auto max_key = ToKey(maximum);
  std::set<VoxelKey> keys;
  for (std::int64_t ix = min_key.ix; ix <= max_key.ix; ++ix) {
    for (std::int64_t iy = min_key.iy; iy <= max_key.iy; ++iy) {
      for (std::int64_t iz = min_key.iz; iz <= max_key.iz; ++iz) {
        keys.insert(VoxelKey{ix, iy, iz});
      }
    }
  }
  return keys;
}

void ReversibleVoxelMap::ApplySparseContribution(const Contribution & contribution, double factor)
{
  auto & accumulator = cells_[contribution.key];
  accumulator.sparse_score_sum = std::max(
    0.0, accumulator.sparse_score_sum + factor * contribution.occupied);
  if (factor > 0.0) {
    ++accumulator.sparse_count;
  } else if (accumulator.sparse_count > 0U) {
    --accumulator.sparse_count;
  }
  if (accumulator.sparse_count == 0U && accumulator.traversed_free <= 1e-9 &&
    accumulator.depth_free <= 1e-9 && accumulator.depth_occupied <= 1e-9)
  {
    cells_.erase(contribution.key);
  }
}

void ReversibleVoxelMap::ApplyFreeContribution(
  const FreeContribution & contribution, double factor)
{
  for (const auto & key : contribution.keys) {
    auto & accumulator = cells_[key];
    auto & free =
      contribution.overrides_sparse ? accumulator.traversed_free : accumulator.depth_free;
    free = std::max(0.0, free + factor);
    if (accumulator.sparse_count == 0U && accumulator.traversed_free <= 1e-9 &&
      accumulator.depth_free <= 1e-9 && accumulator.depth_occupied <= 1e-9)
    {
      cells_.erase(key);
    }
  }
}

VoxelState ReversibleVoxelMap::StateForKey(const VoxelKey & key) const
{
  const auto iterator = cells_.find(key);
  if (iterator == cells_.end()) {
    return VoxelState::Unknown;
  }
  if (iterator->second.depth_occupied > 1e-9) {
    return VoxelState::Occupied;
  }
  if (iterator->second.traversed_free > 1e-9) {
    return VoxelState::Free;
  }
  if (iterator->second.sparse_count > 0U &&
    ScoreForKey(key) > sparse_occupied_score_threshold_)
  {
    return VoxelState::Occupied;
  }
  return iterator->second.depth_free > 1e-9 ? VoxelState::Free : VoxelState::Unknown;
}

double ReversibleVoxelMap::ScoreForKey(const VoxelKey & key) const
{
  const auto iterator = cells_.find(key);
  if (iterator == cells_.end()) {
    return 0.0;
  }
  const auto & accumulator = iterator->second;
  if (accumulator.depth_occupied > 1e-9) {
    return 1.0;
  }
  if (accumulator.traversed_free > 1e-9) {
    return 0.0;
  }
  if (accumulator.sparse_count == 0U) {
    return 0.0;
  }
  return std::clamp(
    accumulator.sparse_score_sum / static_cast<double>(accumulator.sparse_count), 0.0, 1.0);
}

bool ReversibleVoxelMap::IsNavigationDefault(
  const NavigationCell & cell, const NavigationProfile & profile) const
{
  return cell.raw_state == VoxelState::Unknown &&
         cell.traversable == !profile.require_known_free &&
         cell.neighbor_mask == (profile.require_known_free ? 0U : kNavigationAllNeighbors) &&
         std::abs(cell.entry_cost_multiplier - profile.unknown_cost_multiplier) <= 1e-9;
}

bool ReversibleVoxelMap::IsNavigationCoarseDefault(
  const NavigationCoarseCell & cell, const NavigationProfile & profile) const
{
  return cell.traversable == !profile.require_known_free &&
         std::abs(cell.blocked_fraction - (profile.require_known_free ? 1.0 : 0.0)) <= 1e-9 &&
         std::abs(cell.entry_cost_multiplier - profile.unknown_cost_multiplier) <= 1e-9;
}

bool ReversibleVoxelMap::IsTraversableForProfile(
  const VoxelKey & key, const NavigationProfile & profile) const
{
  for (std::int64_t dx = -profile.obstacle_inflation_cells.ix;
    dx <= profile.obstacle_inflation_cells.ix; ++dx)
  {
    for (std::int64_t dy = -profile.obstacle_inflation_cells.iy;
      dy <= profile.obstacle_inflation_cells.iy; ++dy)
    {
      for (std::int64_t dz = -profile.obstacle_inflation_cells.iz;
        dz <= profile.obstacle_inflation_cells.iz; ++dz)
      {
        if (StateForKey(VoxelKey{key.ix + dx, key.iy + dy, key.iz + dz}) ==
          VoxelState::Occupied)
        {
          return false;
        }
      }
    }
  }
  return true;
}

bool ReversibleVoxelMap::IsOccupied(VoxelState state)
{
  return state == VoxelState::Occupied;
}

void ReversibleVoxelMap::ApplyObstacleInfluence(
  NavigationLayer * layer, const VoxelKey & key, std::int64_t factor)
{
  const auto & radius = layer->profile.obstacle_inflation_cells;
  for (std::int64_t dx = -radius.ix; dx <= radius.ix; ++dx) {
    for (std::int64_t dy = -radius.iy; dy <= radius.iy; ++dy) {
      for (std::int64_t dz = -radius.iz; dz <= radius.iz; ++dz) {
        const VoxelKey influenced{key.ix + dx, key.iy + dy, key.iz + dz};
        auto & count = layer->occupied_influence_counts[influenced];
        count += factor;
        if (count <= 0) {
          layer->occupied_influence_counts.erase(influenced);
        }
      }
    }
  }
}

VoxelKey ReversibleVoxelMap::CoarseKeyFor(const VoxelKey & key, std::int64_t factor)
{
  const auto divide = [factor](std::int64_t value) {
      return value >= 0 ? value / factor : -(((-value) + factor - 1) / factor);
    };
  return VoxelKey{divide(key.ix), divide(key.iy), divide(key.iz)};
}

NavigationCoarseCell ReversibleVoxelMap::NavigationCoarseCellFor(
  const VoxelKey & key, const NavigationLayer & layer) const
{
  const auto factor = layer.profile.coarse_voxel_factor;
  std::size_t traversable = 0U;
  std::size_t blocked = 0U;
  double cost_sum = 0.0;
  const auto cell_count = static_cast<double>(factor * factor * factor);
  for (std::int64_t dx = 0; dx < factor; ++dx) {
    for (std::int64_t dy = 0; dy < factor; ++dy) {
      for (std::int64_t dz = 0; dz < factor; ++dz) {
        const VoxelKey fine{key.ix * factor + dx, key.iy * factor + dy, key.iz * factor + dz};
        const auto item = layer.cells.find(fine);
        if (item == layer.cells.end()) {
          if (layer.profile.require_known_free) {
            ++blocked;
          } else {
            ++traversable;
            cost_sum += layer.profile.unknown_cost_multiplier;
          }
          continue;
        }
        if (!item->second.traversable) {
          ++blocked;
          continue;
        }
        ++traversable;
        cost_sum += item->second.entry_cost_multiplier;
      }
    }
  }
  return NavigationCoarseCell{
    key, traversable != 0U,
    traversable == 0U ? layer.profile.unknown_cost_multiplier : cost_sum / traversable,
    static_cast<double>(blocked) / cell_count};
}

std::set<VoxelKey> ReversibleVoxelMap::NavigationAffectedKeys(
  const std::vector<VoxelChange> & raw_changes, const NavigationProfile & profile) const
{
  std::set<VoxelKey> affected;
  const auto radius_x = profile.obstacle_inflation_cells.ix + 1;
  const auto radius_y = profile.obstacle_inflation_cells.iy + 1;
  const auto radius_z = profile.obstacle_inflation_cells.iz + 1;
  for (const auto & change : raw_changes) {
    for (std::int64_t dx = -radius_x; dx <= radius_x; ++dx) {
      for (std::int64_t dy = -radius_y; dy <= radius_y; ++dy) {
        for (std::int64_t dz = -radius_z; dz <= radius_z; ++dz) {
          affected.insert(VoxelKey{change.key.ix + dx, change.key.iy + dy, change.key.iz + dz});
        }
      }
    }
  }
  return affected;
}

void ReversibleVoxelMap::RecordChange(const VoxelKey & key, VoxelState before)
{
  const auto after = StateForKey(key);
  if (before != after) {
    pending_changes_.push_back(VoxelChange{key, before, after});
  }
}

}  // namespace task_lib
