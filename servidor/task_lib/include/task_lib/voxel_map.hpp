#pragma once

#include "task_lib/mission_config.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace task_lib
{

enum class VoxelState : std::uint8_t
{
  Unknown = 0U,
  Free = 1U,
  Occupied = 2U,
};

struct VoxelKey
{
  std::int64_t ix = 0;
  std::int64_t iy = 0;
  std::int64_t iz = 0;

  bool operator==(const VoxelKey & other) const;
  bool operator<(const VoxelKey & other) const;
};

struct SparseEvidence
{
  std::string source_id;
  Vec3 position_world;
  float score = 1.0F;
};

struct VoxelCell
{
  VoxelKey key;
  VoxelState state = VoxelState::Unknown;
  float score = 0.0F;
};

struct VoxelChange
{
  VoxelKey key;
  VoxelState before = VoxelState::Unknown;
  VoxelState after = VoxelState::Unknown;
};

constexpr std::uint32_t NavigationNeighborBit(
  std::int64_t dx, std::int64_t dy, std::int64_t dz)
{
  return 1U << static_cast<std::uint32_t>((dx + 1) * 9 + (dy + 1) * 3 + (dz + 1));
}

constexpr std::uint32_t kNavigationAllNeighbors =
  ((1U << 27U) - 1U) & ~NavigationNeighborBit(0, 0, 0);

struct NavigationProfile
{
  std::string profile_id;
  VoxelKey obstacle_inflation_cells;
  double unknown_cost_multiplier = 3.0;
  std::int64_t coarse_voxel_factor = 4;
  bool require_known_free = false;
};

struct NavigationCell
{
  VoxelKey key;
  VoxelState raw_state = VoxelState::Unknown;
  bool traversable = true;
  std::uint32_t neighbor_mask = kNavigationAllNeighbors;
  double entry_cost_multiplier = 3.0;
};

struct NavigationChange
{
  VoxelKey key;
  NavigationCell before;
  NavigationCell after;
};

struct NavigationCoarseCell
{
  VoxelKey key;
  bool traversable = true;
  double entry_cost_multiplier = 3.0;
  double blocked_fraction = 0.0;
};

struct NavigationCoarseChange
{
  VoxelKey key;
  NavigationCoarseCell before;
  NavigationCoarseCell after;
};

struct NavigationSnapshot
{
  std::string profile_id;
  std::uint64_t raw_revision = 0U;
  double unknown_cost_multiplier = 3.0;
  std::int64_t coarse_voxel_factor = 4;
  bool require_known_free = false;
  std::vector<NavigationCell> cells;
  std::vector<NavigationCoarseCell> coarse_cells;
};

struct NavigationUpdate
{
  std::string profile_id;
  std::uint64_t raw_revision = 0U;
  std::size_t recomputed_cells = 0U;
  double elapsed_ms = 0.0;
  std::vector<NavigationChange> changes;
  std::vector<NavigationCoarseChange> coarse_changes;
};

class ReversibleVoxelMap
{
public:
  explicit ReversibleVoxelMap(double voxel_size);

  bool ApplySparseSnapshot(
    const std::vector<SparseEvidence> & evidence, float occupied_score_threshold = 0.4F);
  bool ApplySparseDelta(
    const std::vector<SparseEvidence> & upserts, const std::vector<std::string> & deletes,
    float occupied_score_threshold = 0.4F);
  bool AddFreeEvidence(const std::string & source_id, const Vec3 & position_world);
  bool AddDepthFreeEvidence(const std::string & source_id, const Vec3 & position_world);
  bool ReplaceFreeVolume(
    const std::string & source_id, const Vec3 & center_world,
    const Vec3 & half_extent_world);
  bool ReplaceDepthFreeVolume(
    const std::string & source_id, const Vec3 & center_world,
    const Vec3 & half_extent_world);
  bool ReplaceDepthFreeCells(
    const std::string & source_id, const std::set<VoxelKey> & keys);
  bool ReplaceDirectDepthFreeCells(
    const std::string & source_id, const std::set<VoxelKey> & keys);
  bool ReplaceDepthOccupiedCells(
    const std::string & source_id, const std::set<VoxelKey> & keys);
  bool RemoveEvidence(const std::string & source_id);

  bool RegisterNavigationProfile(const NavigationProfile & profile);
  NavigationSnapshot NavigationSnapshotFor(const std::string & profile_id) const;
  NavigationUpdate RefreshNavigation(
    const std::string & profile_id, const std::vector<VoxelChange> & raw_changes);

  double voxel_size() const;
  std::uint64_t revision() const;
  VoxelState StateAt(const VoxelKey & key) const;
  std::vector<VoxelCell> Snapshot() const;
  std::vector<VoxelChange> TakeChanges();

private:
  struct Contribution
  {
    VoxelKey key;
    double occupied = 0.0;
  };
  using ContributionsBySource = std::map<std::string, Contribution>;
  using ContributionsByVoxel = std::map<VoxelKey, ContributionsBySource>;
  struct FreeContribution
  {
    std::set<VoxelKey> keys;
    bool overrides_sparse = true;
  };
  struct Accumulator
  {
    double sparse_score_sum = 0.0;
    std::size_t sparse_count = 0U;
    double traversed_free = 0.0;
    double depth_free = 0.0;
    double depth_occupied = 0.0;
  };
  struct NavigationLayer
  {
    NavigationProfile profile;
    std::map<VoxelKey, NavigationCell> cells;
    std::map<VoxelKey, NavigationCoarseCell> coarse_cells;
    std::map<VoxelKey, std::int64_t> occupied_influence_counts;
  };

  VoxelKey ToKey(const Vec3 & position_world) const;
  std::set<VoxelKey> KeysForVolume(const Vec3 & center, const Vec3 & half_extent) const;
  bool ReconcileSparseCells(
    const ContributionsByVoxel & before, const std::set<VoxelKey> & affected);
  static bool SameContribution(const Contribution & left, const Contribution & right);
  static bool SameContributions(
    const ContributionsBySource & left, const ContributionsBySource & right);
  void ApplySparseContribution(const Contribution & contribution, double factor);
  void ApplyFreeContribution(const FreeContribution & contribution, double factor);
  bool ReplaceFreeVolumeImpl(
    const std::string & source_id, const Vec3 & center_world,
    const Vec3 & half_extent_world, bool overrides_sparse);
  bool ReplaceFreeCellsImpl(
    const std::string & source_id, const std::set<VoxelKey> & keys,
    bool overrides_sparse);
  bool ReplaceDepthOccupiedCellsImpl(
    const std::string & source_id, const std::set<VoxelKey> & keys);
  double ScoreForKey(const VoxelKey & key) const;
  VoxelState StateForKey(const VoxelKey & key) const;
  bool IsNavigationDefault(
    const NavigationCell & cell, const NavigationProfile & profile) const;
  bool IsNavigationCoarseDefault(
    const NavigationCoarseCell & cell, const NavigationProfile & profile) const;
  bool IsTraversableForProfile(
    const VoxelKey & key, const NavigationProfile & profile) const;
  static VoxelKey CoarseKeyFor(const VoxelKey & key, std::int64_t factor);
  NavigationCoarseCell NavigationCoarseCellFor(
    const VoxelKey & key, const NavigationLayer & layer) const;
  std::set<VoxelKey> NavigationAffectedKeys(
    const std::vector<VoxelChange> & raw_changes, const NavigationProfile & profile) const;
  static bool IsOccupied(VoxelState state);
  void ApplyObstacleInfluence(
    NavigationLayer * layer, const VoxelKey & key, std::int64_t factor);
  void RecordChange(const VoxelKey & key, VoxelState before);

  double voxel_size_;
  std::uint64_t revision_ = 0U;
  ContributionsBySource sparse_contributions_;
  ContributionsByVoxel sparse_contributions_by_voxel_;
  std::map<std::string, FreeContribution> free_contributions_;
  std::map<std::string, std::set<VoxelKey>> depth_occupied_contributions_;
  std::map<VoxelKey, Accumulator> cells_;
  std::vector<VoxelChange> pending_changes_;
  std::map<std::string, NavigationLayer> navigation_layers_;
  double sparse_occupied_score_threshold_ = 0.4;
};

}  // namespace task_lib
