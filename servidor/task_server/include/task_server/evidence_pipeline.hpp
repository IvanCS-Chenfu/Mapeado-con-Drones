#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <mission_msgs/msg/dense_kf_observation.hpp>
#include <mission_msgs/msg/keyframe_sparse_evidence_delta.hpp>
#include <task_lib/mission_config.hpp>
#include <task_lib/voxel_map.hpp>

#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace task_server
{

enum class EvidenceSourceKind : std::uint8_t
{
  FREE = 0U,
  DIRECT_FREE = 1U,
  OCCUPIED = 2U,
};

struct KeyframeEvidenceIdentity
{
  std::uint32_t drone_id = 0U;
  std::uint64_t map_epoch = 0U;
  std::uint64_t keyframe_id = 0U;

  bool operator<(const KeyframeEvidenceIdentity & other) const;
};

struct KeyframeEvidenceSource
{
  std::string source_name;
  EvidenceSourceKind kind = EvidenceSourceKind::FREE;
  std::uint64_t source_revision = 0U;
  task_lib::Vec3 local_origin;
  std::vector<task_lib::Vec3> local_endpoints;
};

struct EvidenceTransaction
{
  KeyframeEvidenceIdentity identity;
  std::uint64_t geometry_revision = 0U;
  std::uint64_t pose_revision = 0U;
  geometry_msgs::msg::Pose w_t_keyframe;
  std::vector<KeyframeEvidenceSource> upserts;
  std::vector<std::string> deletes;
};

class EvidenceDatabase
{
public:
  bool ReplaceKeyframeSources(
    const KeyframeEvidenceIdentity & identity, std::uint64_t geometry_revision,
    std::uint64_t pose_revision, const geometry_msgs::msg::Pose & w_t_keyframe,
    const std::vector<KeyframeEvidenceSource> & sources);
  bool UpsertKeyframeSources(
    const KeyframeEvidenceIdentity & identity,
    const std::vector<KeyframeEvidenceSource> & sources);
  bool UpdateKeyframePose(
    const KeyframeEvidenceIdentity & identity, std::uint64_t pose_revision,
    const geometry_msgs::msg::Pose & w_t_keyframe);
  bool DeleteKeyframe(
    const KeyframeEvidenceIdentity & identity, std::uint64_t geometry_revision,
    std::uint64_t pose_revision);

  std::vector<EvidenceTransaction> TakePendingTransactions();
  std::size_t SourceCount() const;
  bool HasSources(const KeyframeEvidenceIdentity & identity) const;

private:
  struct KeyframeRecord
  {
    std::uint64_t geometry_revision = 0U;
    std::uint64_t pose_revision = 0U;
    bool pose_available = false;
    geometry_msgs::msg::Pose w_t_keyframe;
    std::map<std::string, KeyframeEvidenceSource> sources;
  };

  struct Tombstone
  {
    std::uint64_t geometry_revision = 0U;
    std::uint64_t pose_revision = 0U;
  };

  std::map<KeyframeEvidenceIdentity, KeyframeRecord> records_;
  std::map<KeyframeEvidenceIdentity, Tombstone> tombstones_;
  std::vector<EvidenceTransaction> pending_transactions_;
};

struct KeyframeEvidenceParameters
{
  float plane_min_score = 0.2F;
  float direct_ray_min_score = 0.5F;
  std::size_t plane_min_inliers = 20U;
  double plane_residual_m = 0.125;
  double voxel_size_m = 0.25;
  std::size_t plane_max_samples = 1024U;
};

enum class DepthInspectionKind : std::uint8_t
{
  VIEW_WALL = 0U,
  VIEW_UNKNOWN = 1U,
  VIEW_ADVANCE = 2U,
};

struct DepthIntegrationParameters
{
  double voxel_size_m = 0.25;
  double far_free_distance_m = 5.0;
  double direct_surface_max_incidence_deg = 35.0;
  std::size_t min_support_points = 20U;
  float min_confidence = 0.25F;
};

struct DepthIntegrationJob
{
  std::string command_id;
  DepthInspectionKind inspection_kind = DepthInspectionKind::VIEW_UNKNOWN;
  std::vector<mission_msgs::msg::DenseKFObservation> observations;
};

struct DepthIntegrationSource
{
  KeyframeEvidenceIdentity identity;
  std::string source_name;
};

struct DepthIntegrationProcessResult
{
  bool accepted = false;
  std::string reason;
  std::vector<DepthIntegrationSource> sources;
};

class DepthIntegrationWorker
{
public:
  explicit DepthIntegrationWorker(DepthIntegrationParameters parameters);

  DepthIntegrationProcessResult Consume(
    const DepthIntegrationJob & job, EvidenceDatabase * database) const;

private:
  DepthIntegrationParameters parameters_;
};

struct KeyframeEvidenceProcessStats
{
  std::size_t accepted_keyframes = 0U;
  std::size_t tombstones = 0U;
  std::size_t direct_sources = 0U;
  std::size_t plane_sources = 0U;
};

class KeyframeEvidenceWorker
{
public:
  explicit KeyframeEvidenceWorker(KeyframeEvidenceParameters parameters);

  KeyframeEvidenceProcessStats Consume(
    const mission_msgs::msg::KeyframeSparseEvidenceDelta & delta,
    EvidenceDatabase * database) const;

private:
  KeyframeEvidenceParameters parameters_;
};

struct VoxelMaterializationStats
{
  std::size_t transactions = 0U;
  std::size_t upserts = 0U;
  std::size_t deletes = 0U;
  std::size_t free_cells = 0U;
  std::vector<std::string> applied_source_ids;
  std::vector<std::string> removed_source_ids;
  std::map<std::string, std::set<task_lib::VoxelKey>> occupied_cells_by_source;
  bool changed = false;
};

class VoxelMapBuilder
{
public:
  explicit VoxelMapBuilder(double voxel_size_m);

  VoxelMaterializationStats Apply(
    EvidenceDatabase * database, task_lib::ReversibleVoxelMap * voxel_map) const;

  static std::string SourceId(
    const KeyframeEvidenceIdentity & identity, const std::string & source_name);

private:
  double voxel_size_m_;
};

}  // namespace task_server
