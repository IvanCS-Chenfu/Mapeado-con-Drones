#pragma once

#include "orbslam3_multi/raw_map_types.hpp"

#include "orbslam3_msgs/msg/orb_key_frame.hpp"
#include "orbslam3_msgs/msg/orb_map.hpp"
#include "orbslam3_msgs/msg/orb_map_point.hpp"
#include "orbslam3_msgs/msg/fiducial_key_frame_observations.hpp"

#include <cstdint>
#include <fstream>
#include <functional>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>
#include <unordered_map>

namespace orbslam3_multi
{

struct RawJournalEntry
{
  uint64_t arrival_id = 0;
  std::shared_ptr<const orbslam3_msgs::msg::OrbMap> map;
};

struct RawJournalStorageStats
{
  uint64_t logical_entries = 0;
  uint64_t resident_entries = 0;
  uint64_t record_bytes_written = 0;
  bool incremental_record_active = false;
  bool incremental_record_healthy = true;
};

struct RawRecordMetadata
{
  uint32_t version = 0;
  uint64_t entry_count = 0;
  std::vector<RecordedFiducialObservation> fiducial_observations;
};

/// Autoridad de datos ORB-SLAM3 crudos, indexada siempre por (drone_id, map_epoch).
/// Ninguna optimización escribe aquí: snapshots sustituyen estado raw y deltas lo refinan.
class RawMapDatabase
{
public:
  // Ingesta serializada y versionada; el journal conserva el mismo orden de arrival_id.
  RawInsertResult InsertDelta(
    uint64_t arrival_id,
    std::shared_ptr<const orbslam3_msgs::msg::OrbMap> delta);
  RawInsertResult InsertFullSnapshot(
    uint64_t arrival_id,
    std::shared_ptr<const orbslam3_msgs::msg::OrbMap> snapshot);
  void SetFiducialPendingCapacityPerDrone(size_t capacity);
  FiducialBatchSubmitResult SubmitFiducialBatch(
    const orbslam3_msgs::msg::FiducialKeyFrameObservations & batch);
  FiducialSyncStats GetFiducialSyncStats() const;

  // Lecturas copiadas para que consumidores costosos trabajen fuera del lock interno.
  RawDatabaseStats GetStats() const;
  std::optional<orbslam3_msgs::msg::OrbKeyFrame> GetKeyFrame(const RawKeyFrameId & id) const;
  std::optional<orbslam3_msgs::msg::OrbMapPoint> GetMapPoint(const RawMapPointId & id) const;
  std::vector<std::optional<RawMapPointScoreInput>> GetMapPointScoreInputs(
    const std::vector<RawMapPointId> & ids) const;
  std::vector<std::optional<RawFusionMapPointInput>> GetFusionMapPointInputs(
    const std::vector<RawMapPointId> & ids) const;
  std::optional<RawCameraCalibration> GetCameraCalibration(
    const RawSubmapId & id) const;
  RawBuilderSnapshot GetBuilderSnapshot(
    const std::set<RawKeyFrameId> & keyframe_ids,
    const std::set<RawMapPointId> & mappoint_ids) const;
  std::optional<uint64_t> GetKeyFrameRevision(const RawKeyFrameId & id) const;
  std::optional<RawLoopSemanticRevision> GetLoopSemanticRevision(
    const RawKeyFrameId & id, uint64_t strong_covisibility_support,
    size_t min_query_mappoints) const;
  std::optional<uint64_t> GetMapPointRevision(const RawMapPointId & id) const;
  std::optional<RawSubmapPoseSnapshot> GetSubmapPoseSnapshot(
    const RawSubmapId & id) const;
  std::optional<RawSubmapEntityIds> GetActiveSubmapEntityIds(
    const RawSubmapId & id) const;
  std::vector<RawJournalEntry> GetJournalView() const;
  void AddFiducialObservation(const RecordedFiducialObservation & observation);
  std::vector<RecordedFiducialObservation> GetFiducialObservationJournal() const;
  std::vector<RecordedFiducialObservation> GetFiducialObservationsForArrival(
    uint64_t arrival_id) const;

  // Persistencia record v3. El archivo temporal solo se publica al finalizar correctamente.
  bool StartIncrementalRecord(
    const std::string & path, std::string * error_message = nullptr);
  bool FinalizeIncrementalRecord(std::string * error_message = nullptr);
  void DisableJournalRetention();
  RawJournalStorageStats GetJournalStorageStats() const;

  bool SaveToPath(const std::string & path, std::string * error_message = nullptr) const;
  static bool LoadRecord(
    const std::string & path,
    std::vector<RawJournalEntry> * entries,
    std::vector<RecordedFiducialObservation> * fiducial_observations,
    std::string * error_message = nullptr);
  static bool LoadRecord(
    const std::string & path,
    std::vector<RawJournalEntry> * entries,
    std::string * error_message = nullptr);
  static bool ReadRecordMetadata(
    const std::string & path,
    RawRecordMetadata * metadata,
    std::string * error_message = nullptr);
  static bool StreamRecordEntries(
    const std::string & path,
    const std::function<bool(RawJournalEntry &&)> & consumer,
    std::string * error_message = nullptr);

private:
  enum class JournalMode
  {
    InMemory,
    Disabled,
    Incremental
  };

  struct RawSubmap
  {
    uint64_t revision = 0;
    RawCameraCalibration camera;
    std::map<uint64_t, orbslam3_msgs::msg::OrbKeyFrame> keyframes;
    std::map<uint64_t, orbslam3_msgs::msg::OrbMapPoint> mappoints;
    std::map<uint64_t, uint64_t> keyframe_revisions;
    std::map<uint64_t, uint64_t> mappoint_revisions;
  };

  struct RawKeyFrameIdHash
  {
    size_t operator()(const RawKeyFrameId & id) const;
  };

  struct PendingFiducialBatch
  {
    orbslam3_msgs::msg::FiducialKeyFrameObservations batch;
    uint64_t digest = 0;
  };

  RawDatabaseStats GetStatsLocked() const;
  void AppendJournalEntryLocked(
    uint64_t arrival_id,
    std::shared_ptr<const orbslam3_msgs::msg::OrbMap> map);
  void SetIncrementalRecordErrorLocked(const std::string & error);
  RawInsertResult InsertMap(
    uint64_t arrival_id,
    std::shared_ptr<const orbslam3_msgs::msg::OrbMap> map,
    bool full_snapshot);
  std::optional<std::string> ValidateFiducialBatchLocked(
    const orbslam3_msgs::msg::FiducialKeyFrameObservations & batch,
    const orbslam3_msgs::msg::OrbKeyFrame * raw_keyframe) const;
  void ResolvePendingFiducialBatchesLocked(RawInsertResult * result);
  SynchronizedFiducialBatch MakeSynchronizedFiducialBatchLocked(
    const RawKeyFrameId & id,
    const orbslam3_msgs::msg::OrbKeyFrame & raw_keyframe,
    const orbslam3_msgs::msg::FiducialKeyFrameObservations & batch,
    bool matched_from_pending) const;
  void RemovePendingOrderLocked(const RawKeyFrameId & id);

  mutable std::mutex mutex_;
  std::map<RawSubmapId, RawSubmap> submaps_;
  size_t fiducial_pending_capacity_per_drone_ = 10;
  std::unordered_map<RawKeyFrameId, PendingFiducialBatch, RawKeyFrameIdHash>
    pending_fiducial_batches_;
  std::map<uint32_t, std::deque<RawKeyFrameId>> pending_fiducial_order_by_drone_;
  std::unordered_map<RawKeyFrameId, uint64_t, RawKeyFrameIdHash>
    consumed_fiducial_digests_;
  std::unordered_map<RawKeyFrameId, uint64_t, RawKeyFrameIdHash>
    keyframe_first_arrival_ids_;
  FiducialSyncStats fiducial_sync_stats_;
  std::vector<RawJournalEntry> journal_;
  std::vector<RecordedFiducialObservation> fiducial_observation_journal_;
  JournalMode journal_mode_ = JournalMode::InMemory;
  std::ofstream incremental_record_stream_;
  std::string incremental_record_path_;
  std::string incremental_record_temp_path_;
  std::string incremental_record_error_;
  uint64_t journal_entry_count_ = 0;
  uint64_t record_bytes_written_ = 0;
  uint64_t last_arrival_id_ = 0;
};

}  // namespace orbslam3_multi
