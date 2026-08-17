#pragma once

#include "geometry_msgs/msg/pose.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace orbslam3_server
{

struct GroundTruthSample
{
  int64_t stamp_ns = 0;
  geometry_msgs::msg::Pose world_T_body;
  int32_t fiducial_id = 0;
  uint64_t fiducial_visit_id = 0;
  double distance_to_fiducial_m = 0.0;
};

struct GroundTruthMatch
{
  bool matched = false;
  GroundTruthSample sample;
  double dt_sec = 0.0;
  std::string reason;
};

class GroundTruthBuffer
{
public:
  static constexpr size_t kCapacityPerDrone = 50;

  void Push(uint32_t drone_id, const GroundTruthSample & sample)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto & samples = samples_by_drone_[drone_id];
    samples.push_back(sample);
    if (samples.size() > kCapacityPerDrone) {
      samples.pop_front();
    }
  }

  std::vector<GroundTruthSample> Snapshot(uint32_t drone_id) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = samples_by_drone_.find(drone_id);
    return found == samples_by_drone_.end() ?
           std::vector<GroundTruthSample>() :
           std::vector<GroundTruthSample>(found->second.begin(), found->second.end());
  }

  size_t Size(uint32_t drone_id) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = samples_by_drone_.find(drone_id);
    return found == samples_by_drone_.end() ? 0U : found->second.size();
  }

  static GroundTruthMatch FindNearest(
    const std::vector<GroundTruthSample> & samples,
    int64_t keyframe_stamp_ns,
    double max_dt_sec)
  {
    GroundTruthMatch result;
    if (samples.empty()) {
      result.reason = "no_gt_samples";
      return result;
    }
    if (keyframe_stamp_ns <= 0) {
      result.reason = "invalid_keyframe_stamp";
      return result;
    }

    const auto nearest = std::min_element(
      samples.begin(), samples.end(),
      [keyframe_stamp_ns](const auto & lhs, const auto & rhs) {
        return std::llabs(lhs.stamp_ns - keyframe_stamp_ns) <
        std::llabs(rhs.stamp_ns - keyframe_stamp_ns);
      });
    result.sample = *nearest;
    result.dt_sec = static_cast<double>(
      std::llabs(nearest->stamp_ns - keyframe_stamp_ns)) * 1e-9;
    if (result.dt_sec > max_dt_sec) {
      result.reason = keyframe_stamp_ns < samples.front().stamp_ns ?
        "gt_history_expired" : "no_gt_within_threshold";
      return result;
    }
    if (nearest->fiducial_id <= 0) {
      result.reason = "outside_fiducial_radius";
      return result;
    }
    result.matched = true;
    result.reason = "matched";
    return result;
  }

private:
  mutable std::mutex mutex_;
  std::map<uint32_t, std::deque<GroundTruthSample>> samples_by_drone_;
};

}  // namespace orbslam3_server
