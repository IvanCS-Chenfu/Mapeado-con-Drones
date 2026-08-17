#include "orbslam3_multi/sparse_global_backend.hpp"

#include <gtest/gtest.h>

#include <sys/resource.h>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>

namespace
{

constexpr size_t kKeyFramesPerDrone = 50;
constexpr size_t kMapPointsPerDrone = 5000;

geometry_msgs::msg::Pose MakePose(double x, double y = 0.0)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.w = 1.0;
  return pose;
}

std::shared_ptr<orbslam3_msgs::msg::OrbMap> MakeSubmap(
  uint32_t drone_id, double point_offset = 0.0)
{
  auto map = std::make_shared<orbslam3_msgs::msg::OrbMap>();
  map->drone_id = drone_id;
  map->map_epoch = 0;
  map->keyframes.reserve(kKeyFramesPerDrone);
  for (size_t kf_index = 0; kf_index < kKeyFramesPerDrone; ++kf_index) {
    auto & keyframe = map->keyframes.emplace_back();
    keyframe.id = kf_index;
    keyframe.pose = MakePose(
      static_cast<double>(kf_index) * 0.05,
      static_cast<double>(drone_id) * 0.1);
    keyframe.mappoint_ids.reserve(kMapPointsPerDrone / kKeyFramesPerDrone);
  }

  map->mappoints.reserve(kMapPointsPerDrone);
  for (size_t mp_index = 0; mp_index < kMapPointsPerDrone; ++mp_index) {
    const uint64_t keyframe_id = mp_index % kKeyFramesPerDrone;
    auto & point = map->mappoints.emplace_back();
    point.id = mp_index;
    point.position.x = static_cast<double>(mp_index % 100) * 0.02 + point_offset;
    point.position.y = static_cast<double>((mp_index / 100) % 50) * 0.02;
    point.position.z = 1.0 + static_cast<double>(mp_index % 7) * 0.01;
    point.reference_keyframe_id = keyframe_id;
    point.observations_count = 8;
    point.found_ratio = 0.9F;
    point.descriptor.data[0] = 1;
    point.observations.emplace_back().keyframe_id = keyframe_id;
    map->keyframes[keyframe_id].mappoint_ids.push_back(mp_index);
  }
  return map;
}

double MillisecondsSince(const std::chrono::steady_clock::time_point & start)
{
  return std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
}

void RunScale(size_t drone_count)
{
  orbslam3_multi::SparseGlobalBackend backend;
  uint64_t arrival_id = 0;

  const auto insert_start = std::chrono::steady_clock::now();
  for (size_t drone = 1; drone <= drone_count; ++drone) {
    backend.InsertDelta(++arrival_id, MakeSubmap(static_cast<uint32_t>(drone)));
  }
  const double insert_ms = MillisecondsSince(insert_start);

  const auto defer_start = std::chrono::steady_clock::now();
  const auto deferred = backend.BuildGlobalMap();
  const double defer_ms = MillisecondsSince(defer_start);
  EXPECT_TRUE(deferred.points.empty());
  EXPECT_TRUE(deferred.keyframes.empty());
  EXPECT_EQ(deferred.deferred_unanchored_submaps, drone_count);

  orbslam3_multi::GlobalMapBuildResult built;
  const auto anchor_start = std::chrono::steady_clock::now();
  for (size_t drone = 1; drone <= drone_count; ++drone) {
    const auto anchor = backend.CommitAnchor(
      {static_cast<uint32_t>(drone), 0},
      MakePose(static_cast<double>(drone) * 10.0), ++arrival_id);
    ASSERT_EQ(anchor.status, orbslam3_multi::PoseCommitStatus::Applied);
    built = backend.BuildGlobalMap();
  }
  const double anchor_build_ms = MillisecondsSince(anchor_start);

  double update_insert_ms = 0.0;
  double update_builder_ms = 0.0;
  const auto update_start = std::chrono::steady_clock::now();
  for (size_t drone = 1; drone <= drone_count; ++drone) {
    const auto one_insert_start = std::chrono::steady_clock::now();
    backend.InsertDelta(
      ++arrival_id, MakeSubmap(static_cast<uint32_t>(drone), 0.001));
    update_insert_ms += MillisecondsSince(one_insert_start);
    const auto one_builder_start = std::chrono::steady_clock::now();
    built = backend.BuildGlobalMap();
    update_builder_ms += MillisecondsSince(one_builder_start);
    EXPECT_TRUE(built.changed);
  }
  const double update_build_ms = MillisecondsSince(update_start);

  const size_t expected_keyframes = drone_count * kKeyFramesPerDrone;
  const size_t expected_mappoints = drone_count * kMapPointsPerDrone;
  EXPECT_EQ(built.keyframes.size(), expected_keyframes);
  EXPECT_EQ(built.points.size(), expected_mappoints);
  EXPECT_EQ(built.deferred_unanchored_submaps, 0U);

  rusage usage{};
  ASSERT_EQ(getrusage(RUSAGE_SELF, &usage), 0);
  const double max_rss_mib = static_cast<double>(usage.ru_maxrss) / 1024.0;
  std::cout << "[F3G-SCALE] drones=" << drone_count
            << " kfs=" << expected_keyframes
            << " mps=" << expected_mappoints
            << " insert_ms=" << insert_ms
            << " defer_ms=" << defer_ms
            << " anchor_build_ms=" << anchor_build_ms
            << " update_build_ms=" << update_build_ms
            << " update_per_drone_ms=" << update_build_ms / static_cast<double>(drone_count)
            << " update_insert_ms=" << update_insert_ms
            << " update_builder_ms=" << update_builder_ms
            << " max_rss_mib=" << max_rss_mib << std::endl;
}

TEST(Scalability3G, TwoDrones)
{
  RunScale(2);
}

TEST(Scalability3G, FourDrones)
{
  RunScale(4);
}

TEST(Scalability3G, EightDrones)
{
  RunScale(8);
}

}  // namespace
