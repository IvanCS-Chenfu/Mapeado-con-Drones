#pragma once

#include "orbslam3_multi/raw_map_database.hpp"
#include "orbslam3_multi/loop_task.hpp"

#include "geometry_msgs/msg/pose.hpp"

#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace orbslam3_multi
{

enum class CovisibilityEdgeSource : uint8_t
{
  Orbslam3Native = 0,
  ServerLoopGeometric = 1,
};

struct CovisibilityEdge
{
  RawKeyFrameId kf_a;
  RawKeyFrameId kf_b;
  CovisibilityEdgeSource source = CovisibilityEdgeSource::Orbslam3Native;
  uint64_t support = 0;
  double information_weight = 0.0;
  geometry_msgs::msg::Pose relative_pose_measured;
  geometry_msgs::msg::Pose relative_pose_current;
  uint64_t dependency_revision_a = 0;
  uint64_t dependency_revision_b = 0;
  uint64_t created_arrival_id = 0;
  uint64_t updated_revision = 0;
};

struct CovisibilityPatch
{
  uint64_t source_arrival_id = 0;
  RawSubmapId submap_id;
  uint64_t expected_submap_revision = 0;
  std::optional<uint64_t> expected_database_revision;
  std::set<RawKeyFrameId> replace_neighborhoods;
  std::vector<CovisibilityEdge> upserts;
};

struct CovisibilityRollbackEntry
{
  RawKeyFrameId kf_a;
  RawKeyFrameId kf_b;
  CovisibilityEdgeSource source = CovisibilityEdgeSource::Orbslam3Native;
  std::optional<CovisibilityEdge> previous;
};

struct CovisibilityRollbackPatch
{
  uint64_t revision_before = 0;
  uint64_t revision_after = 0;
  std::vector<CovisibilityRollbackEntry> entries;
};

struct CovisibilityUpdateResult
{
  bool committed = false;
  bool stale = false;
  uint64_t revision_before = 0;
  uint64_t revision_after = 0;
  size_t examined = 0;
  size_t added = 0;
  size_t updated = 0;
  size_t removed = 0;
  size_t unchanged = 0;
  std::string reason;
};

struct CovisibilityApplyResult
{
  CovisibilityUpdateResult update;
  CovisibilityRollbackPatch rollback;
};

struct CovisibilityDatabaseStats
{
  uint64_t revision = 0;
  uint64_t edges = 0;
  uint64_t orbslam3_native_edges = 0;
  uint64_t server_loop_geometric_edges = 0;
};

class CovisibilityDatabase
{
public:
  static std::optional<CovisibilityPatch> PrepareOrbslam3Patch(
    const RawMapDatabase & raw_database,
    const DatabaseUpdateTask & task);
  CovisibilityUpdateResult ApplyPatch(const CovisibilityPatch & patch);
  CovisibilityApplyResult ApplyPatchTransactional(const CovisibilityPatch & patch);
  bool RollbackPatch(const CovisibilityRollbackPatch & patch);

  std::optional<CovisibilityEdge> GetEdge(
    const RawKeyFrameId & first, const RawKeyFrameId & second) const;
  std::vector<CovisibilityEdge> GetNeighbors(
    const RawKeyFrameId & keyframe_id, uint64_t min_support = 1,
    size_t limit = 32) const;
  std::vector<CovisibilityEdge> GetEdgesBySource(
    CovisibilityEdgeSource source) const;
  bool HasSource(
    const RawKeyFrameId & first, const RawKeyFrameId & second,
    CovisibilityEdgeSource source) const;
  bool UpdateRelativePoseCurrent(
    const RawKeyFrameId & first, const RawKeyFrameId & second,
    const geometry_msgs::msg::Pose & relative_pose_current,
    uint64_t dependency_revision_a, uint64_t dependency_revision_b);
  CovisibilityDatabaseStats GetStats() const;

private:
  using EdgeKey = std::pair<RawKeyFrameId, RawKeyFrameId>;
  using SourceEdgeKey =
    std::tuple<RawKeyFrameId, RawKeyFrameId, CovisibilityEdgeSource>;

  static EdgeKey CanonicalKey(
    const RawKeyFrameId & first, const RawKeyFrameId & second);
  static SourceEdgeKey CanonicalSourceKey(
    const RawKeyFrameId & first, const RawKeyFrameId & second,
    CovisibilityEdgeSource source);
  static bool IsValid(const CovisibilityEdge & edge);

  mutable std::mutex mutex_;
  std::map<SourceEdgeKey, CovisibilityEdge> edges_;
  uint64_t revision_ = 0;
};

const char * ToString(CovisibilityEdgeSource source);

}  // namespace orbslam3_multi
