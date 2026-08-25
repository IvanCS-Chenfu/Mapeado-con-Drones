#pragma once

#include "orbslam3_multi/raw_map_types.hpp"

#include "geometry_msgs/msg/pose.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Geometry>

namespace orbslam3_server
{

struct FiducialInterpretationConfig
{
  double min_distance_m = 1.0;
  double max_distance_m = 5.0;
  double consistency_translation_m = 0.15;
  double consistency_rotation_rad = 0.2617993877991494;
  double visit_gap_sec = 2.0;
  size_t recent_capacity_per_drone = 50;
};

struct InterpretedFiducialTag
{
  uint32_t tag_id = 0;
  int32_t object_id = 0;
  double distance_m = 0.0;
  double base_weight = 0.0;
  double robust_weight = 0.0;
  double translation_residual_m = 0.0;
  double rotation_residual_rad = 0.0;
  bool in_range = false;
  geometry_msgs::msg::Pose world_T_camera;
};

struct InterpretedFiducialObject
{
  int32_t object_id = 0;
  std::vector<InterpretedFiducialTag> tags;
  geometry_msgs::msg::Pose world_T_camera_fused;
  double quality = 0.0;
  double distance_m = 0.0;
  bool anchor_eligible = false;
  std::string reason;
};

struct FiducialKeyFrameInterpretation
{
  orbslam3_multi::RawKeyFrameId keyframe_id;
  uint64_t raw_arrival_id = 0;
  double stamp_sec = 0.0;
  std::vector<uint32_t> unknown_tag_ids;
  std::vector<InterpretedFiducialObject> objects;
  std::optional<size_t> primary_index;
  uint64_t primary_visit_id = 0;
};

class FiducialObjectInterpreter
{
public:
  void Load(const std::string & yaml_path);
  void Configure(const FiducialInterpretationConfig & config);
  FiducialInterpretationConfig GetConfig() const;

  FiducialKeyFrameInterpretation Interpret(
    const orbslam3_multi::SynchronizedFiducialBatch & match);
  std::vector<FiducialKeyFrameInterpretation> GetRecent(uint32_t drone_id) const;
  size_t ObjectCount() const;
  size_t TagCount() const;

private:
  struct ObjectConfig
  {
    int32_t object_id = 0;
    Eigen::Isometry3d world_T_object = Eigen::Isometry3d::Identity();
  };

  struct TagConfig
  {
    uint32_t tag_id = 0;
    int32_t object_id = 0;
    Eigen::Isometry3d object_T_tag = Eigen::Isometry3d::Identity();
  };

  struct VisitState
  {
    uint64_t visit_id = 0;
    double first_stamp_sec = 0.0;
    double last_stamp_sec = 0.0;
  };

  uint64_t AssignVisitLocked(
    const orbslam3_multi::RawKeyFrameId & keyframe_id, int32_t object_id,
    double stamp_sec);
  void AppendRecentLocked(const FiducialKeyFrameInterpretation & interpretation);

  mutable std::mutex mutex_;
  FiducialInterpretationConfig config_;
  std::map<int32_t, ObjectConfig> objects_;
  std::map<uint32_t, TagConfig> tags_;
  std::map<std::tuple<uint32_t, uint64_t, int32_t>, std::vector<VisitState>> visits_;
  std::map<uint32_t, std::deque<FiducialKeyFrameInterpretation>> recent_by_drone_;
  uint64_t next_visit_id_ = 1;
};

}  // namespace orbslam3_server
