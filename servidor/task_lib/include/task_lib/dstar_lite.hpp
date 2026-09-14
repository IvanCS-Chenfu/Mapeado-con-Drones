#pragma once

#include "task_lib/mission_config.hpp"
#include "task_lib/voxel_map.hpp"

#include <cstdint>
#include <map>
#include <queue>
#include <string>
#include <vector>

namespace task_lib
{

struct DStarLiteParameters
{
  double unknown_cost_multiplier = 3.0;
  std::int64_t occupied_inflation_cells = 1;
  double search_margin_m = 2.0;
  std::size_t max_expansions = 250000U;
  double heuristic_weight = 1.2;
  std::int64_t initial_corridor_half_width_cells = 8;
  std::int64_t corridor_expand_step_cells = 4;
};

struct DStarLiteResult
{
  bool success = false;
  bool incremental_repair = false;
  bool used_safe_escape = false;
  std::uint64_t expanded = 0U;
  std::uint64_t queue_pops = 0U;
  std::uint64_t stale_queue_pops = 0U;
  std::uint64_t corridor_widenings = 0U;
  std::uint64_t map_revision = 0U;
  std::int64_t coarse_voxel_factor = 1;
  std::int64_t corridor_half_width_cells = 0;
  std::string failure_reason;
  std::vector<VoxelKey> corridor;
  std::vector<Vec3> waypoints_world;
};

/// D* Lite 3D puro. Mantiene g/rhs entre revisiones del mismo objetivo.
class DStarLitePlanner
{
public:
  DStarLitePlanner() = default;

  void Configure(
    const AxisAlignedBox & hard_flight_volume, double voxel_size,
    const DStarLiteParameters & parameters);
  void SetNavigationSnapshot(const NavigationSnapshot & snapshot);
  void ApplyNavigationChanges(
    const std::vector<NavigationChange> & changes, std::uint64_t revision);
  void ApplyNavigationChanges(
    const std::vector<NavigationChange> & changes,
    const std::vector<NavigationCoarseChange> & coarse_changes, std::uint64_t revision);
  void SetVoxelSnapshot(const std::vector<VoxelCell> & cells, std::uint64_t revision);
  void ApplyChanges(const std::vector<VoxelChange> & changes, std::uint64_t revision);
  DStarLiteResult Plan(const Vec3 & start_world, const Vec3 & goal_world);

private:
  struct QueueKey
  {
    double first = 0.0;
    double second = 0.0;
  };
  struct QueueEntry
  {
    QueueKey key;
    VoxelKey state;
  };

  using StateMap = std::map<VoxelKey, double>;
  using OpenPositionMap = std::map<VoxelKey, std::size_t>;
  using VoxelStateMap = std::map<VoxelKey, VoxelState>;
  using NavigationCellMap = std::map<VoxelKey, NavigationCell>;
  using NavigationCoarseCellMap = std::map<VoxelKey, NavigationCoarseCell>;

  static bool Equal(const VoxelKey & left, const VoxelKey & right);
  static bool Less(const VoxelKey & left, const VoxelKey & right);
  static double Distance(const VoxelKey & left, const VoxelKey & right);
  static bool KeyLess(const QueueKey & left, const QueueKey & right);
  static bool QueueEntryLess(const QueueEntry & left, const QueueEntry & right);

  bool IsInside(const VoxelKey & state) const;
  bool IsInsideSearchVolume(const VoxelKey & state) const;
  bool IsInsideGuide(const VoxelKey & state) const;
  bool UpdateSearchVolume(const Vec3 & start_world, const Vec3 & goal_world);
  VoxelKey ToKey(const Vec3 & position_world) const;
  Vec3 ToCenter(const VoxelKey & state) const;
  VoxelState RawState(const VoxelKey & state) const;
  NavigationCell NavigationCellFor(const VoxelKey & state) const;
  NavigationCoarseCell NavigationCoarseCellFor(const VoxelKey & state) const;
  bool IsTraversable(const VoxelKey & state) const;
  bool IsNavigationEdgeAllowed(const VoxelKey & from, const VoxelKey & to) const;
  std::vector<VoxelKey> FindSafeEscape(const VoxelKey & start) const;
  bool IsSafeDiagonal(const VoxelKey & from, const VoxelKey & to) const;
  double StepCost(const VoxelKey & from, const VoxelKey & to) const;
  std::vector<VoxelKey> Neighbors(const VoxelKey & state) const;
  std::vector<VoxelKey> CoarseNeighbors(const VoxelKey & state) const;
  std::vector<VoxelKey> LineCells(const VoxelKey & from, const VoxelKey & to) const;
  bool HasLineOfSight(const VoxelKey & from, const VoxelKey & to) const;

  void ResetSearch(const VoxelKey & start, const VoxelKey & goal);
  void PushOpen(const VoxelKey & state);
  void RemoveOpen(const VoxelKey & state);
  QueueEntry PopOpen();
  void SiftOpenUp(std::size_t index);
  void SiftOpenDown(std::size_t index);
  void SwapOpen(std::size_t left, std::size_t right);
  std::vector<VoxelKey> BuildCoarseGuide(const VoxelKey & start, const VoxelKey & goal) const;
  bool SetGuide(
    const std::vector<VoxelKey> & guide, std::int64_t half_width_cells);
  void UpdateVertex(const VoxelKey & state);
  bool ComputeShortestPath();
  void UpdateAffected(const VoxelKey & changed);
  QueueKey CalculateKey(const VoxelKey & state) const;
  double G(const VoxelKey & state) const;
  double Rhs(const VoxelKey & state) const;
  void SetG(const VoxelKey & state, double value);
  void SetRhs(const VoxelKey & state, double value);
  std::vector<VoxelKey> BuildPath() const;
  std::vector<VoxelKey> SimplifyPath(const std::vector<VoxelKey> & path) const;

  AxisAlignedBox hard_flight_volume_;
  AxisAlignedBox search_volume_;
  double voxel_size_ = 0.0;
  DStarLiteParameters parameters_;
  VoxelStateMap cells_;
  NavigationCellMap navigation_cells_;
  NavigationCoarseCellMap navigation_coarse_cells_;
  bool uses_navigation_snapshot_ = false;
  double navigation_unknown_cost_multiplier_ = 3.0;
  bool navigation_require_known_free_ = false;
  std::int64_t coarse_voxel_factor_ = 1;
  StateMap g_;
  StateMap rhs_;
  OpenPositionMap open_positions_;
  std::vector<QueueEntry> open_;
  std::vector<VoxelKey> guide_cells_;
  std::int64_t guide_half_width_cells_ = 0;
  VoxelKey start_;
  VoxelKey last_start_;
  VoxelKey goal_;
  bool configured_ = false;
  bool initialized_ = false;
  double km_ = 0.0;
  std::uint64_t map_revision_ = 0U;
  std::uint64_t expanded_ = 0U;
  std::uint64_t queue_pops_ = 0U;
  std::uint64_t stale_queue_pops_ = 0U;
};

}  // namespace task_lib
