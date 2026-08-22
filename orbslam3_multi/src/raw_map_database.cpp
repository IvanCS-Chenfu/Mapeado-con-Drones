#include "orbslam3_multi/raw_map_database.hpp"

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace orbslam3_multi
{
namespace
{

constexpr uint32_t kRecordMagic = 0x46334352U;  // F3CR
constexpr uint32_t kRecordVersion = 3U;
constexpr uint64_t kMaxPayloadBytes = 512ULL * 1024ULL * 1024ULL;
constexpr uint64_t kHashOffset = 1469598103934665603ULL;
constexpr uint64_t kHashPrime = 1099511628211ULL;

void HashBytes(uint64_t * hash, const void * data, size_t size)
{
  const auto * bytes = static_cast<const uint8_t *>(data);
  for (size_t index = 0; index < size; ++index) {
    *hash ^= bytes[index];
    *hash *= kHashPrime;
  }
}

template<typename T>
void HashValue(uint64_t * hash, const T & value)
{
  HashBytes(hash, &value, sizeof(value));
}

void HashQuantized(uint64_t * hash, double value, double resolution)
{
  const int64_t quantized = std::isfinite(value) ?
    static_cast<int64_t>(std::llround(value / resolution)) :
    std::numeric_limits<int64_t>::min();
  HashValue(hash, quantized);
}

template<typename T>
bool WritePod(std::ostream & out, const T & value)
{
  out.write(reinterpret_cast<const char *>(&value), sizeof(T));
  return static_cast<bool>(out);
}

template<typename T>
bool ReadPod(std::istream & in, T * value)
{
  in.read(reinterpret_cast<char *>(value), sizeof(T));
  return static_cast<bool>(in);
}

bool WriteString(std::ostream & out, const std::string & value)
{
  const uint64_t size = value.size();
  if (!WritePod(out, size)) {
    return false;
  }
  out.write(value.data(), static_cast<std::streamsize>(size));
  return static_cast<bool>(out);
}

bool ReadString(std::istream & in, std::string * value)
{
  uint64_t size = 0;
  if (!ReadPod(in, &size) || size > 4096U) {
    return false;
  }
  value->resize(static_cast<size_t>(size));
  if (size > 0) {
    in.read(value->data(), static_cast<std::streamsize>(size));
  }
  return static_cast<bool>(in);
}

bool WritePose(std::ostream & out, const geometry_msgs::msg::Pose & pose)
{
  return WritePod(out, pose.position.x) && WritePod(out, pose.position.y) &&
         WritePod(out, pose.position.z) && WritePod(out, pose.orientation.x) &&
         WritePod(out, pose.orientation.y) && WritePod(out, pose.orientation.z) &&
         WritePod(out, pose.orientation.w);
}

bool ReadPose(std::istream & in, geometry_msgs::msg::Pose * pose)
{
  return ReadPod(in, &pose->position.x) && ReadPod(in, &pose->position.y) &&
         ReadPod(in, &pose->position.z) && ReadPod(in, &pose->orientation.x) &&
         ReadPod(in, &pose->orientation.y) && ReadPod(in, &pose->orientation.z) &&
         ReadPod(in, &pose->orientation.w);
}

bool WriteFiducialObservation(
  std::ostream & out, const RecordedFiducialObservation & observation)
{
  return WritePod(out, observation.arrival_id) &&
         WritePod(out, observation.keyframe_id.drone_id) &&
         WritePod(out, observation.keyframe_id.map_epoch) &&
         WritePod(out, observation.keyframe_id.local_kf_id) &&
         WritePod(out, observation.fiducial_id) &&
         WritePod(out, observation.fiducial_visit_id) &&
         WritePose(out, observation.world_T_camera_target) &&
         WritePod(out, observation.keyframe_stamp_sec) &&
         WritePod(out, observation.observation_stamp_sec) &&
         WritePod(out, observation.association_dt_sec) &&
         WritePod(out, observation.distance_to_fiducial_m) &&
         WriteString(out, observation.source) && WriteString(out, observation.quality);
}

bool ReadFiducialObservation(
  std::istream & in, uint32_t record_version,
  RecordedFiducialObservation * observation)
{
  const bool prefix = ReadPod(in, &observation->arrival_id) &&
         ReadPod(in, &observation->keyframe_id.drone_id) &&
         ReadPod(in, &observation->keyframe_id.map_epoch) &&
         ReadPod(in, &observation->keyframe_id.local_kf_id) &&
         ReadPod(in, &observation->fiducial_id);
  if (!prefix) {
    return false;
  }
  observation->fiducial_visit_id = 0;
  if (record_version >= 3U && !ReadPod(in, &observation->fiducial_visit_id)) {
    return false;
  }
  return
         ReadPose(in, &observation->world_T_camera_target) &&
         ReadPod(in, &observation->keyframe_stamp_sec) &&
         ReadPod(in, &observation->observation_stamp_sec) &&
         ReadPod(in, &observation->association_dt_sec) &&
         ReadPod(in, &observation->distance_to_fiducial_m) &&
         ReadString(in, &observation->source) && ReadString(in, &observation->quality);
}

void SetError(std::string * error_message, const std::string & value)
{
  if (error_message != nullptr) {
    *error_message = value;
  }
}

std::shared_ptr<orbslam3_msgs::msg::OrbMap> MakeNormalizedDeltaShell(
  const orbslam3_msgs::msg::OrbMap & source)
{
  auto normalized = std::make_shared<orbslam3_msgs::msg::OrbMap>();
  normalized->header = source.header;
  normalized->drone_id = source.drone_id;
  normalized->drone_name = source.drone_name;
  normalized->map_frame = source.map_frame;
  normalized->map_sequence = source.map_sequence;
  normalized->map_epoch = source.map_epoch;
  normalized->fx = source.fx;
  normalized->fy = source.fy;
  normalized->cx = source.cx;
  normalized->cy = source.cy;
  normalized->bf = source.bf;
  normalized->image_width = source.image_width;
  normalized->image_height = source.image_height;
  return normalized;
}

struct BidirectionalDifference
{
  std::vector<uint64_t> added;
  std::vector<uint64_t> removed;
};

BidirectionalDifference ComputeBidirectionalDifference(
  const std::vector<uint64_t> & current,
  const std::vector<uint64_t> & previous)
{
  std::unordered_set<uint64_t> current_ids;
  std::unordered_set<uint64_t> previous_ids;
  current_ids.reserve(current.size());
  previous_ids.reserve(previous.size());
  current_ids.insert(current.begin(), current.end());
  previous_ids.insert(previous.begin(), previous.end());

  BidirectionalDifference result;
  result.added.reserve(current.size());
  result.removed.reserve(previous.size());
  for (const uint64_t id : current) {
    if (previous_ids.count(id) == 0U) {
      result.added.push_back(id);
    }
  }
  for (const uint64_t id : previous) {
    if (current_ids.count(id) == 0U) {
      result.removed.push_back(id);
    }
  }

  const auto sort_unique = [](std::vector<uint64_t> * ids) {
      std::sort(ids->begin(), ids->end());
      ids->erase(std::unique(ids->begin(), ids->end()), ids->end());
    };
  sort_unique(&result.added);
  sort_unique(&result.removed);
  return result;
}

RawKeyFrameId MakeKeyFrameId(const RawSubmapId & submap_id, uint64_t local_id)
{
  return {submap_id.drone_id, submap_id.map_epoch, local_id};
}

RawMapPointId MakeMapPointId(const RawSubmapId & submap_id, uint64_t local_id)
{
  return {submap_id.drone_id, submap_id.map_epoch, local_id};
}

bool DescriptorValid(const orbslam3_msgs::msg::OrbMapPoint & mappoint)
{
  return std::any_of(
    mappoint.descriptor.data.begin(), mappoint.descriptor.data.end(),
    [](uint8_t value) {return value != 0U;});
}

bool MapPointGeometryChanged(
  const orbslam3_msgs::msg::OrbMapPoint & old_value,
  const orbslam3_msgs::msg::OrbMapPoint & new_value)
{
  return old_value.position != new_value.position ||
         old_value.reference_keyframe_id != new_value.reference_keyframe_id ||
         old_value.observations != new_value.observations ||
         old_value.is_bad != new_value.is_bad;
}

bool MapPointScoreInputsChanged(
  const orbslam3_msgs::msg::OrbMapPoint & old_value,
  const orbslam3_msgs::msg::OrbMapPoint & new_value)
{
  return old_value.observations_count != new_value.observations_count ||
         old_value.found_ratio != new_value.found_ratio ||
         DescriptorValid(old_value) != DescriptorValid(new_value) ||
         old_value.is_bad != new_value.is_bad;
}

bool KeyFrameCovisibilityChanged(
  const orbslam3_msgs::msg::OrbKeyFrame & old_value,
  const orbslam3_msgs::msg::OrbKeyFrame & new_value)
{
  return old_value.connected_keyframe_ids != new_value.connected_keyframe_ids ||
         old_value.connected_keyframe_weights != new_value.connected_keyframe_weights ||
         old_value.parent_keyframe_id != new_value.parent_keyframe_id ||
         old_value.child_keyframe_ids != new_value.child_keyframe_ids ||
         old_value.loop_edge_keyframe_ids != new_value.loop_edge_keyframe_ids;
}

}  // namespace

std::string ToString(const RawSubmapId & id)
{
  std::ostringstream out;
  out << '(' << id.drone_id << ',' << id.map_epoch << ')';
  return out.str();
}

const char * ToString(RawKeyFramePoseChangeKind kind)
{
  switch (kind) {
    case RawKeyFramePoseChangeKind::New:
      return "new";
    case RawKeyFramePoseChangeKind::PoseUpdated:
      return "pose_updated";
    case RawKeyFramePoseChangeKind::Invalidated:
      return "invalidated";
  }
  return "unknown";
}

RawInsertResult RawMapDatabase::InsertDelta(
  uint64_t arrival_id,
  std::shared_ptr<const orbslam3_msgs::msg::OrbMap> delta)
{
  return InsertMap(arrival_id, std::move(delta), false);
}

RawInsertResult RawMapDatabase::InsertFullSnapshot(
  uint64_t arrival_id,
  std::shared_ptr<const orbslam3_msgs::msg::OrbMap> snapshot)
{
  return InsertMap(arrival_id, std::move(snapshot), true);
}

RawInsertResult RawMapDatabase::InsertMap(
  uint64_t arrival_id,
  std::shared_ptr<const orbslam3_msgs::msg::OrbMap> map,
  bool full_snapshot)
{
  if (!map) {
    throw std::invalid_argument(full_snapshot ? "snapshot nulo" : "delta nulo");
  }
  if (map->drone_id == 0U) {
    throw std::invalid_argument("drone_id=0 no identifica un submapa raw valido");
  }

  RawInsertResult result;
  result.arrival_id = arrival_id;
  result.submap_id = {map->drone_id, map->map_epoch};
  result.full_snapshot = full_snapshot;

  auto normalized = full_snapshot ? MakeNormalizedDeltaShell(*map) : nullptr;

  std::lock_guard<std::mutex> lock(mutex_);
  if (last_arrival_id_ != 0U && arrival_id <= last_arrival_id_) {
    throw std::invalid_argument("arrival_id debe ser estrictamente creciente");
  }

  auto [submap_it, inserted] = submaps_.try_emplace(result.submap_id);
  RawSubmap & submap = submap_it->second;
  submap.camera = {
    map->fx, map->fy, map->cx, map->cy,
    map->bf, map->image_width, map->image_height};
  result.new_submap = inserted;
  bool material_change = inserted;
  std::unordered_set<uint64_t> received_keyframes;
  std::unordered_set<uint64_t> received_mappoints;
  if (full_snapshot) {
    received_keyframes.reserve(map->keyframes.size());
    received_mappoints.reserve(map->mappoints.size());
  }

  for (const auto & keyframe : map->keyframes) {
    if (full_snapshot) {
      received_keyframes.insert(keyframe.id);
    }
    const RawKeyFrameId id = MakeKeyFrameId(result.submap_id, keyframe.id);
    const auto existing = submap.keyframes.find(keyframe.id);
    if (existing == submap.keyframes.end()) {
      submap.keyframes.emplace(keyframe.id, keyframe);
      submap.keyframe_revisions[keyframe.id] = 1;
      result.new_keyframe_ids.push_back(id);
      result.pose_changed_keyframe_ids.push_back(id);
      result.pose_changes.push_back(
        {keyframe.is_bad ? RawKeyFramePoseChangeKind::Invalidated :
          RawKeyFramePoseChangeKind::New, {id, 1, keyframe.pose, !keyframe.is_bad}});
      if (keyframe.is_bad) {
        result.removed_keyframe_ids.push_back(id);
        result.invalidated_keyframe_ids.push_back(id);
      }
      RawAssociationChange associations;
      associations.keyframe_id = id;
      for (const uint64_t mp_id : keyframe.mappoint_ids) {
        const auto global_mp_id = MakeMapPointId(result.submap_id, mp_id);
        associations.added_mappoint_ids.push_back(global_mp_id);
        result.association_changed_mappoint_ids.push_back(global_mp_id);
      }
      if (!associations.added_mappoint_ids.empty()) {
        result.association_changed_keyframe_ids.push_back(id);
        result.association_changes.push_back(std::move(associations));
      }
      const orbslam3_msgs::msg::OrbKeyFrame empty_keyframe;
      if (KeyFrameCovisibilityChanged(empty_keyframe, keyframe)) {
        result.covisibility_changed_keyframe_ids.push_back(id);
      }
      if (normalized) {
        normalized->keyframes.push_back(keyframe);
      }
      material_change = true;
      continue;
    }

    if (existing->second == keyframe) {
      result.unchanged_keyframe_ids.push_back(id);
      continue;
    }

    const auto & old = existing->second;
    const bool association_changed = old.mappoint_ids != keyframe.mappoint_ids;
    const bool covisibility_changed = KeyFrameCovisibilityChanged(old, keyframe);
    const bool pose_changed = old.pose != keyframe.pose;
    const bool activity_changed = old.is_bad != keyframe.is_bad;

    if (association_changed) {
      RawAssociationChange associations;
      associations.keyframe_id = id;
      const auto difference = ComputeBidirectionalDifference(
        keyframe.mappoint_ids, old.mappoint_ids);
      for (const uint64_t mp_id : difference.added) {
        const auto global_mp_id = MakeMapPointId(result.submap_id, mp_id);
        associations.added_mappoint_ids.push_back(global_mp_id);
        result.association_changed_mappoint_ids.push_back(global_mp_id);
      }
      for (const uint64_t mp_id : difference.removed) {
        const auto global_mp_id = MakeMapPointId(result.submap_id, mp_id);
        associations.removed_mappoint_ids.push_back(global_mp_id);
        result.association_changed_mappoint_ids.push_back(global_mp_id);
      }
      result.association_changed_keyframe_ids.push_back(id);
      result.association_changes.push_back(std::move(associations));
    }
    if (covisibility_changed) {
      result.covisibility_changed_keyframe_ids.push_back(id);
    }
    if (pose_changed || activity_changed) {
      result.pose_changed_keyframe_ids.push_back(id);
    }
    if (!old.is_bad && keyframe.is_bad) {
      result.removed_keyframe_ids.push_back(id);
      result.invalidated_keyframe_ids.push_back(id);
    }

    existing->second = keyframe;
    ++submap.keyframe_revisions[keyframe.id];
    const uint64_t raw_revision = submap.keyframe_revisions[keyframe.id];
    result.updated_keyframe_ids.push_back(id);
    if (pose_changed || activity_changed) {
      result.pose_changes.push_back(
        {keyframe.is_bad ? RawKeyFramePoseChangeKind::Invalidated :
          RawKeyFramePoseChangeKind::PoseUpdated,
          {id, raw_revision, keyframe.pose, !keyframe.is_bad}});
    }
    if (normalized) {
      normalized->keyframes.push_back(keyframe);
    }
    material_change = true;
  }

  if (full_snapshot) {
    for (auto & [local_id, keyframe] : submap.keyframes) {
      if (received_keyframes.count(local_id) != 0U || keyframe.is_bad) {
        continue;
      }
      keyframe.is_bad = true;
      ++submap.keyframe_revisions[local_id];
      const RawKeyFrameId id = MakeKeyFrameId(result.submap_id, local_id);
      result.updated_keyframe_ids.push_back(id);
      result.removed_keyframe_ids.push_back(id);
      result.invalidated_keyframe_ids.push_back(id);
      result.pose_changed_keyframe_ids.push_back(id);
      result.pose_changes.push_back(
        {RawKeyFramePoseChangeKind::Invalidated,
          {id, submap.keyframe_revisions[local_id], keyframe.pose, false}});
      normalized->keyframes.push_back(keyframe);
      material_change = true;
    }
  }

  for (const auto & mappoint : map->mappoints) {
    if (full_snapshot) {
      received_mappoints.insert(mappoint.id);
    }
    const RawMapPointId id = MakeMapPointId(result.submap_id, mappoint.id);
    const auto existing = submap.mappoints.find(mappoint.id);
    if (existing == submap.mappoints.end()) {
      submap.mappoints.emplace(mappoint.id, mappoint);
      submap.mappoint_revisions[mappoint.id] = 1;
      result.new_mappoint_ids.push_back(id);
      result.geometry_changed_mappoint_ids.push_back(id);
      result.score_input_changed_mappoint_ids.push_back(id);
      if (mappoint.is_bad) {
        result.removed_mappoint_ids.push_back(id);
        result.invalidated_mappoint_ids.push_back(id);
      }
      if (normalized) {
        normalized->mappoints.push_back(mappoint);
      }
      material_change = true;
      continue;
    }

    if (existing->second == mappoint) {
      result.unchanged_mappoint_ids.push_back(id);
      continue;
    }
    const auto & old = existing->second;
    if (MapPointGeometryChanged(old, mappoint)) {
      result.geometry_changed_mappoint_ids.push_back(id);
    }
    if (MapPointScoreInputsChanged(old, mappoint)) {
      result.score_input_changed_mappoint_ids.push_back(id);
    }
    if (old.reference_keyframe_id != mappoint.reference_keyframe_id ||
      old.observations != mappoint.observations)
    {
      result.association_changed_mappoint_ids.push_back(id);
    }
    if (!old.is_bad && mappoint.is_bad) {
      result.removed_mappoint_ids.push_back(id);
      result.invalidated_mappoint_ids.push_back(id);
    }
    existing->second = mappoint;
    ++submap.mappoint_revisions[mappoint.id];
    result.updated_mappoint_ids.push_back(id);
    if (normalized) {
      normalized->mappoints.push_back(mappoint);
    }
    material_change = true;
  }

  if (full_snapshot) {
    for (auto & [local_id, mappoint] : submap.mappoints) {
      if (received_mappoints.count(local_id) != 0U || mappoint.is_bad) {
        continue;
      }
      mappoint.is_bad = true;
      ++submap.mappoint_revisions[local_id];
      const RawMapPointId id = MakeMapPointId(result.submap_id, local_id);
      result.updated_mappoint_ids.push_back(id);
      result.removed_mappoint_ids.push_back(id);
      result.invalidated_mappoint_ids.push_back(id);
      result.geometry_changed_mappoint_ids.push_back(id);
      result.score_input_changed_mappoint_ids.push_back(id);
      normalized->mappoints.push_back(mappoint);
      material_change = true;
    }
  }

  std::sort(
    result.association_changed_mappoint_ids.begin(),
    result.association_changed_mappoint_ids.end());
  result.association_changed_mappoint_ids.erase(
    std::unique(
      result.association_changed_mappoint_ids.begin(),
      result.association_changed_mappoint_ids.end()),
    result.association_changed_mappoint_ids.end());

  if (material_change) {
    ++submap.revision;
  }
  result.has_material_changes = material_change;
  result.submap_revision = submap.revision;
  if (!full_snapshot) {
    AppendJournalEntryLocked(arrival_id, std::move(map));
    result.journal_entry_appended = true;
  } else if (material_change) {
    AppendJournalEntryLocked(arrival_id, std::move(normalized));
    result.journal_entry_appended = true;
    result.normalized_delta_appended = true;
  }
  last_arrival_id_ = arrival_id;
  result.stats = GetStatsLocked();
  return result;
}

void RawMapDatabase::SetIncrementalRecordErrorLocked(const std::string & error)
{
  if (incremental_record_error_.empty()) {
    incremental_record_error_ = error;
  }
  if (incremental_record_stream_.is_open()) {
    incremental_record_stream_.close();
  }
}

void RawMapDatabase::AppendJournalEntryLocked(
  uint64_t arrival_id,
  std::shared_ptr<const orbslam3_msgs::msg::OrbMap> map)
{
  ++journal_entry_count_;
  if (journal_mode_ == JournalMode::InMemory) {
    journal_.push_back({arrival_id, std::move(map)});
    return;
  }
  if (journal_mode_ == JournalMode::Disabled || !incremental_record_error_.empty()) {
    return;
  }
  if (!map || !incremental_record_stream_.is_open()) {
    SetIncrementalRecordErrorLocked("record incremental no disponible");
    return;
  }

  try {
    rclcpp::Serialization<orbslam3_msgs::msg::OrbMap> serializer;
    rclcpp::SerializedMessage serialized;
    serializer.serialize_message(map.get(), &serialized);
    const auto & message = serialized.get_rcl_serialized_message();
    const uint64_t payload_size = message.buffer_length;
    if (payload_size > kMaxPayloadBytes) {
      SetIncrementalRecordErrorLocked("delta excede el tamano maximo del record");
      return;
    }
    if (!WritePod(incremental_record_stream_, arrival_id) ||
      !WritePod(incremental_record_stream_, payload_size))
    {
      SetIncrementalRecordErrorLocked("no se pudo escribir metadata incremental");
      return;
    }
    incremental_record_stream_.write(
      reinterpret_cast<const char *>(message.buffer),
      static_cast<std::streamsize>(payload_size));
    if (!incremental_record_stream_) {
      SetIncrementalRecordErrorLocked("no se pudo escribir payload incremental");
      return;
    }
    record_bytes_written_ += sizeof(arrival_id) + sizeof(payload_size) + payload_size;
  } catch (const std::exception & ex) {
    SetIncrementalRecordErrorLocked(ex.what());
  }
}

RawDatabaseStats RawMapDatabase::GetStatsLocked() const
{
  RawDatabaseStats stats;
  stats.journal_entries = journal_entry_count_;
  stats.delta_entries = journal_entry_count_;
  stats.submaps = submaps_.size();
  stats.last_arrival_id = last_arrival_id_;
  stats.fiducial_observations = fiducial_observation_journal_.size();
  for (const auto & [id, submap] : submaps_) {
    (void)id;
    stats.keyframes += submap.keyframes.size();
    stats.mappoints += submap.mappoints.size();
  }
  return stats;
}

RawDatabaseStats RawMapDatabase::GetStats() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return GetStatsLocked();
}

std::optional<orbslam3_msgs::msg::OrbKeyFrame> RawMapDatabase::GetKeyFrame(
  const RawKeyFrameId & id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto submap = submaps_.find({id.drone_id, id.map_epoch});
  if (submap == submaps_.end()) {
    return std::nullopt;
  }
  const auto keyframe = submap->second.keyframes.find(id.local_kf_id);
  return keyframe == submap->second.keyframes.end() ?
         std::nullopt : std::optional<orbslam3_msgs::msg::OrbKeyFrame>(keyframe->second);
}

std::optional<orbslam3_msgs::msg::OrbMapPoint> RawMapDatabase::GetMapPoint(
  const RawMapPointId & id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto submap = submaps_.find({id.drone_id, id.map_epoch});
  if (submap == submaps_.end()) {
    return std::nullopt;
  }
  const auto mappoint = submap->second.mappoints.find(id.local_mp_id);
  return mappoint == submap->second.mappoints.end() ?
         std::nullopt : std::optional<orbslam3_msgs::msg::OrbMapPoint>(mappoint->second);
}

std::vector<std::optional<RawMapPointScoreInput>> RawMapDatabase::GetMapPointScoreInputs(
  const std::vector<RawMapPointId> & ids) const
{
  std::vector<std::optional<RawMapPointScoreInput>> inputs;
  inputs.reserve(ids.size());
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & id : ids) {
    const auto submap = submaps_.find({id.drone_id, id.map_epoch});
    if (submap == submaps_.end()) {
      inputs.emplace_back(std::nullopt);
      continue;
    }
    const auto point = submap->second.mappoints.find(id.local_mp_id);
    if (point == submap->second.mappoints.end()) {
      inputs.emplace_back(std::nullopt);
      continue;
    }
    const auto & raw = point->second;
    const bool descriptor_valid = std::any_of(
      raw.descriptor.data.begin(), raw.descriptor.data.end(),
      [](uint8_t value) {return value != 0U;});
    inputs.emplace_back(
      RawMapPointScoreInput{
        raw.observations_count, raw.found_ratio, descriptor_valid, raw.is_bad});
  }
  return inputs;
}

std::vector<std::optional<RawFusionMapPointInput>>
RawMapDatabase::GetFusionMapPointInputs(
  const std::vector<RawMapPointId> & ids) const
{
  std::vector<std::optional<RawFusionMapPointInput>> inputs;
  inputs.reserve(ids.size());
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & id : ids) {
    const auto submap = submaps_.find({id.drone_id, id.map_epoch});
    if (submap == submaps_.end()) {
      inputs.emplace_back(std::nullopt);
      continue;
    }
    const auto point = submap->second.mappoints.find(id.local_mp_id);
    const auto revision = submap->second.mappoint_revisions.find(id.local_mp_id);
    if (point == submap->second.mappoints.end() ||
      revision == submap->second.mappoint_revisions.end())
    {
      inputs.emplace_back(std::nullopt);
      continue;
    }
    RawFusionMapPointInput input;
    input.id = id;
    input.raw_revision = revision->second;
    input.position = point->second.position;
    input.descriptor = point->second.descriptor.data;
    input.observations_count = point->second.observations_count;
    input.reference_keyframe_id = point->second.reference_keyframe_id;
    input.is_bad = point->second.is_bad;
    input.observer_keyframe_ids.reserve(point->second.observations.size());
    for (const auto & observation : point->second.observations) {
      input.observer_keyframe_ids.push_back(observation.keyframe_id);
    }
    inputs.emplace_back(std::move(input));
  }
  return inputs;
}

std::optional<RawCameraCalibration> RawMapDatabase::GetCameraCalibration(
  const RawSubmapId & id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto submap = submaps_.find(id);
  if (submap == submaps_.end() || !submap->second.camera.IsValid()) {
    return std::nullopt;
  }
  return submap->second.camera;
}

RawBuilderSnapshot RawMapDatabase::GetBuilderSnapshot(
  const std::set<RawKeyFrameId> & keyframe_ids,
  const std::set<RawMapPointId> & mappoint_ids) const
{
  RawBuilderSnapshot snapshot;
  snapshot.requested_mappoint_ids = mappoint_ids;
  std::set<RawKeyFrameId> required_keyframes = keyframe_ids;

  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & id : keyframe_ids) {
    const auto submap = submaps_.find({id.drone_id, id.map_epoch});
    if (submap == submaps_.end()) {
      continue;
    }
    const auto keyframe = submap->second.keyframes.find(id.local_kf_id);
    if (keyframe == submap->second.keyframes.end()) {
      continue;
    }
    for (const uint64_t local_mp_id : keyframe->second.mappoint_ids) {
      snapshot.requested_mappoint_ids.insert({id.drone_id, id.map_epoch, local_mp_id});
    }
  }

  for (const auto & id : snapshot.requested_mappoint_ids) {
    const auto submap = submaps_.find({id.drone_id, id.map_epoch});
    if (submap == submaps_.end()) {
      continue;
    }
    const auto point = submap->second.mappoints.find(id.local_mp_id);
    if (point == submap->second.mappoints.end()) {
      continue;
    }
    const auto & raw = point->second;
    RawBuilderMapPointInput input;
    input.position = raw.position;
    input.reference_keyframe_id = raw.reference_keyframe_id;
    input.is_bad = raw.is_bad;
    input.observer_keyframe_ids.reserve(raw.observations.size());
    required_keyframes.insert({id.drone_id, id.map_epoch, raw.reference_keyframe_id});
    for (const auto & observation : raw.observations) {
      input.observer_keyframe_ids.push_back(observation.keyframe_id);
      required_keyframes.insert({id.drone_id, id.map_epoch, observation.keyframe_id});
    }
    snapshot.mappoints.emplace(id, std::move(input));
  }

  for (const auto & id : required_keyframes) {
    const auto submap = submaps_.find({id.drone_id, id.map_epoch});
    if (submap == submaps_.end()) {
      continue;
    }
    const auto keyframe = submap->second.keyframes.find(id.local_kf_id);
    if (keyframe != submap->second.keyframes.end()) {
      snapshot.keyframes.emplace(
        id, RawBuilderKeyFrameInput{keyframe->second.pose, keyframe->second.is_bad});
    }
  }
  return snapshot;
}

std::optional<uint64_t> RawMapDatabase::GetKeyFrameRevision(const RawKeyFrameId & id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto submap = submaps_.find({id.drone_id, id.map_epoch});
  if (submap == submaps_.end()) {
    return std::nullopt;
  }
  const auto revision = submap->second.keyframe_revisions.find(id.local_kf_id);
  return revision == submap->second.keyframe_revisions.end() ?
         std::nullopt : std::optional<uint64_t>(revision->second);
}

std::optional<RawLoopSemanticRevision> RawMapDatabase::GetLoopSemanticRevision(
  const RawKeyFrameId & id, uint64_t strong_covisibility_support,
  size_t min_query_mappoints) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto submap = submaps_.find({id.drone_id, id.map_epoch});
  if (submap == submaps_.end()) {
    return std::nullopt;
  }
  const auto keyframe = submap->second.keyframes.find(id.local_kf_id);
  const auto raw_revision = submap->second.keyframe_revisions.find(id.local_kf_id);
  if (keyframe == submap->second.keyframes.end() ||
    raw_revision == submap->second.keyframe_revisions.end())
  {
    return std::nullopt;
  }

  RawLoopSemanticRevision result;
  result.raw_revision = raw_revision->second;
  uint64_t appearance = kHashOffset;
  const uint64_t bow_count = std::min(
    keyframe->second.bow_word_ids.size(), keyframe->second.bow_word_values.size());
  HashValue(&appearance, bow_count);
  for (size_t index = 0; index < bow_count; ++index) {
    HashValue(&appearance, keyframe->second.bow_word_ids[index]);
    HashQuantized(&appearance, keyframe->second.bow_word_values[index], 1e-4);
  }
  HashValue(&appearance, keyframe->second.is_bad);
  result.appearance_revision = appearance;

  uint64_t geometry = kHashOffset;
  HashValue(&geometry, keyframe->second.is_bad);
  HashQuantized(&geometry, keyframe->second.pose.position.x, 0.50);
  HashQuantized(&geometry, keyframe->second.pose.position.y, 0.50);
  HashQuantized(&geometry, keyframe->second.pose.position.z, 0.50);
  HashQuantized(&geometry, keyframe->second.pose.orientation.x, 0.10);
  HashQuantized(&geometry, keyframe->second.pose.orientation.y, 0.10);
  HashQuantized(&geometry, keyframe->second.pose.orientation.z, 0.10);
  HashQuantized(&geometry, keyframe->second.pose.orientation.w, 0.10);

  const uint64_t association_count = keyframe->second.mappoint_ids.size();
  const uint64_t minimum_support = std::max<size_t>(1U, min_query_mappoints);
  const uint64_t association_maturity = association_count < minimum_support ?
    0U : (association_count < 4U * minimum_support ? 1U : 2U);
  HashValue(&geometry, association_maturity);

  const size_t covisibility_count = std::min(
    keyframe->second.connected_keyframe_ids.size(),
    keyframe->second.connected_keyframe_weights.size());
  bool has_strong_neighbor = false;
  for (size_t index = 0; index < covisibility_count; ++index) {
    if (keyframe->second.connected_keyframe_weights[index] >=
      strong_covisibility_support)
    {
      has_strong_neighbor = true;
      break;
    }
  }
  HashValue(&geometry, has_strong_neighbor);
  result.geometry_revision = geometry;

  uint64_t validation = kHashOffset;
  HashValue(&validation, keyframe->second.is_bad);
  HashQuantized(&validation, keyframe->second.pose.position.x, 0.01);
  HashQuantized(&validation, keyframe->second.pose.position.y, 0.01);
  HashQuantized(&validation, keyframe->second.pose.position.z, 0.01);
  HashQuantized(&validation, keyframe->second.pose.orientation.x, 0.001);
  HashQuantized(&validation, keyframe->second.pose.orientation.y, 0.001);
  HashQuantized(&validation, keyframe->second.pose.orientation.z, 0.001);
  HashQuantized(&validation, keyframe->second.pose.orientation.w, 0.001);
  HashValue(&validation, association_count);
  for (const uint64_t local_mp_id : keyframe->second.mappoint_ids) {
    HashValue(&validation, local_mp_id);
    const auto point = submap->second.mappoints.find(local_mp_id);
    if (point == submap->second.mappoints.end()) {
      const bool missing = true;
      HashValue(&validation, missing);
      continue;
    }
    const bool missing = false;
    HashValue(&validation, missing);
    HashValue(&validation, point->second.is_bad);
    HashQuantized(&validation, point->second.position.x, 0.001);
    HashQuantized(&validation, point->second.position.y, 0.001);
    HashQuantized(&validation, point->second.position.z, 0.001);
    HashBytes(
      &validation, point->second.descriptor.data.data(),
      point->second.descriptor.data.size());
  }

  HashValue(&validation, static_cast<uint64_t>(covisibility_count));
  for (size_t index = 0; index < covisibility_count; ++index) {
    HashValue(&validation, keyframe->second.connected_keyframe_ids[index]);
    HashValue(&validation, keyframe->second.connected_keyframe_weights[index]);
  }
  result.validation_revision = validation;
  return result;
}

std::optional<uint64_t> RawMapDatabase::GetMapPointRevision(const RawMapPointId & id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto submap = submaps_.find({id.drone_id, id.map_epoch});
  if (submap == submaps_.end()) {
    return std::nullopt;
  }
  const auto revision = submap->second.mappoint_revisions.find(id.local_mp_id);
  return revision == submap->second.mappoint_revisions.end() ?
         std::nullopt : std::optional<uint64_t>(revision->second);
}

std::optional<RawSubmapPoseSnapshot> RawMapDatabase::GetSubmapPoseSnapshot(
  const RawSubmapId & id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto submap = submaps_.find(id);
  if (submap == submaps_.end()) {
    return std::nullopt;
  }

  RawSubmapPoseSnapshot snapshot;
  snapshot.submap_id = id;
  snapshot.submap_revision = submap->second.revision;
  snapshot.keyframes.reserve(submap->second.keyframes.size());
  for (const auto & [local_kf_id, keyframe] : submap->second.keyframes) {
    snapshot.keyframes.push_back(
      {
        {id.drone_id, id.map_epoch, local_kf_id},
        submap->second.keyframe_revisions.at(local_kf_id),
        keyframe.pose,
        !keyframe.is_bad});
  }
  return snapshot;
}

std::optional<RawSubmapEntityIds> RawMapDatabase::GetActiveSubmapEntityIds(
  const RawSubmapId & id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto submap = submaps_.find(id);
  if (submap == submaps_.end()) {
    return std::nullopt;
  }

  RawSubmapEntityIds ids;
  ids.keyframe_ids.reserve(submap->second.keyframes.size());
  for (const auto & [local_kf_id, keyframe] : submap->second.keyframes) {
    if (!keyframe.is_bad) {
      ids.keyframe_ids.push_back({id.drone_id, id.map_epoch, local_kf_id});
    }
  }
  ids.mappoint_ids.reserve(submap->second.mappoints.size());
  for (const auto & [local_mp_id, mappoint] : submap->second.mappoints) {
    if (!mappoint.is_bad) {
      ids.mappoint_ids.push_back({id.drone_id, id.map_epoch, local_mp_id});
    }
  }
  return ids;
}

std::vector<RawJournalEntry> RawMapDatabase::GetJournalView() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return journal_;
}

void RawMapDatabase::AddFiducialObservation(
  const RecordedFiducialObservation & observation)
{
  if (observation.arrival_id == 0 || observation.keyframe_id.drone_id == 0 ||
    observation.fiducial_id <= 0)
  {
    throw std::invalid_argument("observacion fiducial invalida");
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (observation.arrival_id > last_arrival_id_) {
    throw std::invalid_argument("observacion fiducial sin delta raw comprometido");
  }
  fiducial_observation_journal_.push_back(observation);
}

std::vector<RecordedFiducialObservation>
RawMapDatabase::GetFiducialObservationJournal() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return fiducial_observation_journal_;
}

std::vector<RecordedFiducialObservation>
RawMapDatabase::GetFiducialObservationsForArrival(uint64_t arrival_id) const
{
  std::vector<RecordedFiducialObservation> result;
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & observation : fiducial_observation_journal_) {
    if (observation.arrival_id == arrival_id) {
      result.push_back(observation);
    }
  }
  return result;
}

bool RawMapDatabase::StartIncrementalRecord(
  const std::string & path,
  std::string * error_message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (path.empty()) {
    SetError(error_message, "ruta de record incremental vacia");
    return false;
  }
  if (journal_entry_count_ != 0U || !journal_.empty() || last_arrival_id_ != 0U) {
    SetError(error_message, "el record incremental debe iniciarse antes del primer delta");
    return false;
  }
  if (journal_mode_ != JournalMode::InMemory) {
    SetError(error_message, "modo de journal ya configurado");
    return false;
  }

  try {
    const std::filesystem::path output_path(path);
    if (output_path.has_parent_path()) {
      std::filesystem::create_directories(output_path.parent_path());
    }
    incremental_record_path_ = path;
    incremental_record_temp_path_ = path + ".in_progress";
    incremental_record_stream_.open(
      incremental_record_temp_path_, std::ios::binary | std::ios::trunc);
    if (!incremental_record_stream_) {
      SetError(error_message, "no se pudo abrir el record incremental temporal");
      return false;
    }

    const uint64_t placeholder_count = 0;
    if (!WritePod(incremental_record_stream_, kRecordMagic) ||
      !WritePod(incremental_record_stream_, kRecordVersion) ||
      !WritePod(incremental_record_stream_, placeholder_count))
    {
      incremental_record_stream_.close();
      SetError(error_message, "no se pudo escribir la cabecera incremental");
      return false;
    }
    journal_mode_ = JournalMode::Incremental;
    incremental_record_error_.clear();
    record_bytes_written_ = sizeof(kRecordMagic) + sizeof(kRecordVersion) +
      sizeof(placeholder_count);
  } catch (const std::exception & ex) {
    if (incremental_record_stream_.is_open()) {
      incremental_record_stream_.close();
    }
    SetError(error_message, ex.what());
    return false;
  }
  return true;
}

bool RawMapDatabase::FinalizeIncrementalRecord(std::string * error_message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (journal_mode_ != JournalMode::Incremental) {
    SetError(error_message, "no hay un record incremental activo");
    return false;
  }
  if (!incremental_record_error_.empty()) {
    SetError(error_message, incremental_record_error_);
    return false;
  }
  if (!incremental_record_stream_.is_open()) {
    SetError(error_message, "stream incremental cerrado antes de finalizar");
    return false;
  }

  try {
    const uint64_t observation_count = fiducial_observation_journal_.size();
    if (!WritePod(incremental_record_stream_, observation_count)) {
      SetIncrementalRecordErrorLocked("no se pudo escribir el contador fiducial");
      SetError(error_message, incremental_record_error_);
      return false;
    }
    record_bytes_written_ += sizeof(observation_count);
    for (const auto & observation : fiducial_observation_journal_) {
      const auto before = incremental_record_stream_.tellp();
      if (!WriteFiducialObservation(incremental_record_stream_, observation)) {
        SetIncrementalRecordErrorLocked("no se pudo escribir una observacion fiducial");
        SetError(error_message, incremental_record_error_);
        return false;
      }
      const auto after = incremental_record_stream_.tellp();
      if (before >= 0 && after >= before) {
        record_bytes_written_ += static_cast<uint64_t>(after - before);
      }
    }

    incremental_record_stream_.seekp(
      static_cast<std::streamoff>(sizeof(kRecordMagic) + sizeof(kRecordVersion)),
      std::ios::beg);
    if (!incremental_record_stream_ ||
      !WritePod(incremental_record_stream_, journal_entry_count_))
    {
      SetIncrementalRecordErrorLocked("no se pudo actualizar el contador de entradas");
      SetError(error_message, incremental_record_error_);
      return false;
    }
    incremental_record_stream_.flush();
    if (!incremental_record_stream_) {
      SetIncrementalRecordErrorLocked("no se pudo sincronizar el record incremental");
      SetError(error_message, incremental_record_error_);
      return false;
    }
    incremental_record_stream_.close();
    std::filesystem::rename(incremental_record_temp_path_, incremental_record_path_);
    journal_mode_ = JournalMode::Disabled;
  } catch (const std::exception & ex) {
    if (incremental_record_stream_.is_open()) {
      incremental_record_stream_.close();
    }
    SetError(error_message, ex.what());
    return false;
  }
  return true;
}

void RawMapDatabase::DisableJournalRetention()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (journal_entry_count_ != 0U || !journal_.empty() || last_arrival_id_ != 0U) {
    throw std::logic_error("el modo de journal debe fijarse antes del primer delta");
  }
  if (journal_mode_ != JournalMode::InMemory) {
    throw std::logic_error("modo de journal ya configurado");
  }
  journal_mode_ = JournalMode::Disabled;
}

RawJournalStorageStats RawMapDatabase::GetJournalStorageStats() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  RawJournalStorageStats stats;
  stats.logical_entries = journal_entry_count_;
  stats.resident_entries = journal_.size();
  stats.record_bytes_written = record_bytes_written_;
  stats.incremental_record_active =
    journal_mode_ == JournalMode::Incremental && incremental_record_stream_.is_open();
  stats.incremental_record_healthy = incremental_record_error_.empty();
  return stats;
}

bool RawMapDatabase::SaveToPath(const std::string & path, std::string * error_message) const
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (journal_mode_ != JournalMode::InMemory) {
      SetError(error_message, "SaveToPath requiere journal residente");
      return false;
    }
  }
  const auto journal = GetJournalView();
  try {
    const std::filesystem::path output_path(path);
    if (output_path.has_parent_path()) {
      std::filesystem::create_directories(output_path.parent_path());
    }
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) {
      SetError(error_message, "no se pudo abrir el record para escritura");
      return false;
    }

    const uint64_t count = journal.size();
    if (!WritePod(out, kRecordMagic) || !WritePod(out, kRecordVersion) || !WritePod(out, count)) {
      SetError(error_message, "no se pudo escribir la cabecera del record");
      return false;
    }

    rclcpp::Serialization<orbslam3_msgs::msg::OrbMap> serializer;
    for (const auto & entry : journal) {
      if (!entry.map) {
        SetError(error_message, "journal contiene un delta nulo");
        return false;
      }
      rclcpp::SerializedMessage serialized;
      serializer.serialize_message(entry.map.get(), &serialized);
      const auto & message = serialized.get_rcl_serialized_message();
      const uint64_t payload_size = message.buffer_length;
      if (!WritePod(out, entry.arrival_id) || !WritePod(out, payload_size)) {
        SetError(error_message, "no se pudo escribir metadata de entrada");
        return false;
      }
      out.write(reinterpret_cast<const char *>(message.buffer), payload_size);
      if (!out) {
        SetError(error_message, "no se pudo escribir payload de entrada");
        return false;
      }
    }
    const auto observations = GetFiducialObservationJournal();
    const uint64_t observation_count = observations.size();
    if (!WritePod(out, observation_count)) {
      SetError(error_message, "no se pudo escribir el contador fiducial");
      return false;
    }
    for (const auto & observation : observations) {
      if (!WriteFiducialObservation(out, observation)) {
        SetError(error_message, "no se pudo escribir una observacion fiducial");
        return false;
      }
    }
  } catch (const std::exception & ex) {
    SetError(error_message, ex.what());
    return false;
  }
  return true;
}

bool RawMapDatabase::LoadRecord(
  const std::string & path,
  std::vector<RawJournalEntry> * entries,
  std::vector<RecordedFiducialObservation> * fiducial_observations,
  std::string * error_message)
{
  if (entries == nullptr || fiducial_observations == nullptr) {
    SetError(error_message, "destino de journal nulo");
    return false;
  }
  RawRecordMetadata metadata;
  if (!ReadRecordMetadata(path, &metadata, error_message)) {
    return false;
  }
  std::vector<RawJournalEntry> loaded;
  loaded.reserve(metadata.entry_count);
  if (!StreamRecordEntries(
      path,
      [&loaded](RawJournalEntry && entry) {
        loaded.push_back(std::move(entry));
        return true;
      }, error_message))
  {
    return false;
  }
  *entries = std::move(loaded);
  *fiducial_observations = std::move(metadata.fiducial_observations);
  return true;
}

bool RawMapDatabase::LoadRecord(
  const std::string & path,
  std::vector<RawJournalEntry> * entries,
  std::string * error_message)
{
  std::vector<RecordedFiducialObservation> ignored;
  return LoadRecord(path, entries, &ignored, error_message);
}

bool RawMapDatabase::ReadRecordMetadata(
  const std::string & path,
  RawRecordMetadata * metadata,
  std::string * error_message)
{
  if (metadata == nullptr) {
    SetError(error_message, "destino de metadata nulo");
    return false;
  }
  try {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
      SetError(error_message, "no se pudo abrir el record para lectura");
      return false;
    }
    uint32_t magic = 0;
    RawRecordMetadata inspected;
    if (!ReadPod(in, &magic) || !ReadPod(in, &inspected.version) ||
      !ReadPod(in, &inspected.entry_count))
    {
      SetError(error_message, "cabecera de record incompleta");
      return false;
    }
    if (magic != kRecordMagic ||
      (inspected.version < 1U || inspected.version > kRecordVersion))
    {
      SetError(error_message, "formato de record 3C no reconocido");
      return false;
    }

    uint64_t previous_arrival_id = 0;
    for (uint64_t i = 0; i < inspected.entry_count; ++i) {
      uint64_t arrival_id = 0;
      uint64_t payload_size = 0;
      if (!ReadPod(in, &arrival_id) || !ReadPod(in, &payload_size)) {
        SetError(error_message, "metadata de entrada incompleta");
        return false;
      }
      if ((i > 0 && arrival_id <= previous_arrival_id) || payload_size > kMaxPayloadBytes) {
        SetError(error_message, "orden o tamano de entrada invalido");
        return false;
      }
      in.seekg(static_cast<std::streamoff>(payload_size), std::ios::cur);
      if (!in) {
        SetError(error_message, "payload de entrada incompleto");
        return false;
      }
      previous_arrival_id = arrival_id;
    }

    if (inspected.version >= 2U) {
      uint64_t observation_count = 0;
      if (!ReadPod(in, &observation_count) || observation_count > 10000000ULL) {
        SetError(error_message, "contador fiducial invalido");
        return false;
      }
      inspected.fiducial_observations.reserve(observation_count);
      for (uint64_t i = 0; i < observation_count; ++i) {
        RecordedFiducialObservation observation;
        if (!ReadFiducialObservation(in, inspected.version, &observation)) {
          SetError(error_message, "observacion fiducial incompleta");
          return false;
        }
        inspected.fiducial_observations.push_back(std::move(observation));
      }
    }
    *metadata = std::move(inspected);
  } catch (const std::exception & ex) {
    SetError(error_message, ex.what());
    return false;
  }
  return true;
}

bool RawMapDatabase::StreamRecordEntries(
  const std::string & path,
  const std::function<bool(RawJournalEntry &&)> & consumer,
  std::string * error_message)
{
  if (!consumer) {
    SetError(error_message, "consumidor de replay nulo");
    return false;
  }
  try {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
      SetError(error_message, "no se pudo abrir el record para lectura");
      return false;
    }
    uint32_t magic = 0;
    uint32_t version = 0;
    uint64_t count = 0;
    if (!ReadPod(in, &magic) || !ReadPod(in, &version) || !ReadPod(in, &count)) {
      SetError(error_message, "cabecera de record incompleta");
      return false;
    }
    if (magic != kRecordMagic || version < 1U || version > kRecordVersion) {
      SetError(error_message, "formato de record 3C no reconocido");
      return false;
    }

    rclcpp::Serialization<orbslam3_msgs::msg::OrbMap> serializer;
    uint64_t previous_arrival_id = 0;
    for (uint64_t i = 0; i < count; ++i) {
      uint64_t arrival_id = 0;
      uint64_t payload_size = 0;
      if (!ReadPod(in, &arrival_id) || !ReadPod(in, &payload_size)) {
        SetError(error_message, "metadata de entrada incompleta");
        return false;
      }
      if ((i > 0 && arrival_id <= previous_arrival_id) || payload_size > kMaxPayloadBytes) {
        SetError(error_message, "orden o tamano de entrada invalido");
        return false;
      }
      rclcpp::SerializedMessage serialized(payload_size);
      auto & message = serialized.get_rcl_serialized_message();
      in.read(reinterpret_cast<char *>(message.buffer), payload_size);
      if (!in) {
        SetError(error_message, "payload de entrada incompleto");
        return false;
      }
      message.buffer_length = payload_size;
      auto map = std::make_shared<orbslam3_msgs::msg::OrbMap>();
      serializer.deserialize_message(&serialized, map.get());
      if (!consumer({arrival_id, std::move(map)})) {
        SetError(error_message, "consumidor de replay detenido");
        return false;
      }
      previous_arrival_id = arrival_id;
    }
  } catch (const std::exception & ex) {
    SetError(error_message, ex.what());
    return false;
  }
  return true;
}

}  // namespace orbslam3_multi
