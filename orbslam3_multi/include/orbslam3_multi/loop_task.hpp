#pragma once

#include "orbslam3_multi/raw_map_types.hpp"

#include <cstdint>
#include <vector>

namespace orbslam3_multi
{

struct LoopTaskRevision
{
  uint64_t raw_revision = 0;
  uint64_t appearance_revision = 0;
  uint64_t geometry_revision = 0;
  uint64_t validation_revision = 0;
  uint64_t anchor_revision = 0;

  bool operator==(const LoopTaskRevision & other) const
  {
    // raw/validation revisions protect diagnostics and commits; scheduling follows
    // the coarser semantic fingerprints plus anchor authority.
    return appearance_revision == other.appearance_revision &&
           geometry_revision == other.geometry_revision &&
           anchor_revision == other.anchor_revision;
  }
};

struct LoopTask
{
  uint64_t task_id = 0;
  uint64_t source_arrival_id = 0;
  uint64_t enqueue_sequence = 0;
  RawKeyFrameId query_keyframe_id;
  LoopTaskRevision revision;
};

struct DatabaseUpdateTask
{
  uint64_t task_id = 0;
  uint64_t source_arrival_id = 0;
  uint64_t enqueue_sequence = 0;
  RawSubmapId submap_id;
  uint64_t expected_submap_revision = 0;
  std::vector<RawKeyFrameId> covisibility_keyframe_ids;
  std::vector<RawKeyFrameId> loop_keyframe_ids;
};

}  // namespace orbslam3_multi
