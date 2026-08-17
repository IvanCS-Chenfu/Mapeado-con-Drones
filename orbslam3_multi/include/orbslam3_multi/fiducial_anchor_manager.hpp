#pragma once

#include "orbslam3_multi/fiducial_types.hpp"

#include <map>
#include <mutex>
#include <optional>

namespace orbslam3_multi
{

class FiducialAnchorManager
{
public:
  void Configure(const FiducialOptimizationConfig & config);
  FiducialProcessResult Evaluate(
    const FiducialObservation & observation,
    const geometry_msgs::msg::Pose & local_T_camera,
    const std::optional<GlobalPoseRecord> & current_global_pose,
    bool submap_already_anchored);

  void AcceptControl(
    const RawSubmapId & submap_id, uint64_t fiducial_visit_id,
    const RawKeyFrameId & keyframe_id);
  std::optional<RawKeyFrameId> GetLastAcceptedControl(
    const RawSubmapId & submap_id) const;

private:
  struct ControlState
  {
    uint64_t fiducial_visit_id = 0;
    RawKeyFrameId keyframe_id;
  };

  mutable std::mutex mutex_;
  FiducialOptimizationConfig config_;
  std::map<RawSubmapId, ControlState> controls_;
  std::map<RawSubmapId, std::map<uint64_t, RawKeyFrameId>> visit_first_keyframes_;
  uint64_t next_task_id_ = 1;
};

}  // namespace orbslam3_multi
