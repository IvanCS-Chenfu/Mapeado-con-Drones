#include "orbslam3_server/fiducial_object_interpreter.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include <Eigen/Geometry>

namespace
{

using orbslam3_server::FiducialObjectInterpreter;

geometry_msgs::msg::Transform ToTransform(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Transform message;
  message.translation.x = transform.translation().x();
  message.translation.y = transform.translation().y();
  message.translation.z = transform.translation().z();
  const Eigen::Quaterniond quaternion(transform.linear());
  message.rotation.x = quaternion.x();
  message.rotation.y = quaternion.y();
  message.rotation.z = quaternion.z();
  message.rotation.w = quaternion.w();
  return message;
}

Eigen::Isometry3d FaceTransform(const std::string & face)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  if (face == "pos_x") {
    transform.translation() = Eigen::Vector3d(0.2, 0.0, 0.0);
    transform.linear().col(0) = Eigen::Vector3d::UnitY();
    transform.linear().col(1) = Eigen::Vector3d::UnitZ();
    transform.linear().col(2) = Eigen::Vector3d::UnitX();
  } else if (face == "pos_y") {
    transform.translation() = Eigen::Vector3d(0.0, 0.2, 0.0);
    transform.linear().col(0) = -Eigen::Vector3d::UnitX();
    transform.linear().col(1) = Eigen::Vector3d::UnitZ();
    transform.linear().col(2) = Eigen::Vector3d::UnitY();
  } else {
    transform.translation() = Eigen::Vector3d(0.0, 0.0, 0.2);
  }
  return transform;
}

Eigen::Isometry3d WorldObject(int object_id)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  if (object_id == 1) {
    transform.translation() = Eigen::Vector3d(0.0, 8.5, 1.0);
  } else if (object_id == 2) {
    transform.translation() = Eigen::Vector3d(0.0, -8.5, 1.0);
  } else {
    transform.translation() = Eigen::Vector3d(8.5, 0.0, 1.0);
  }
  return transform;
}

orbslam3_msgs::msg::FiducialTagObservation ObservationForCamera(
  uint32_t tag_id, int object_id, const std::string & face,
  const Eigen::Isometry3d & world_T_camera, double quality = 0.9,
  double area = 400.0)
{
  orbslam3_msgs::msg::FiducialTagObservation observation;
  observation.tag_id = tag_id;
  const Eigen::Isometry3d world_T_tag = WorldObject(object_id) * FaceTransform(face);
  observation.camera_t_tag = ToTransform(world_T_camera.inverse() * world_T_tag);
  observation.quality_score = quality;
  observation.reprojection_error_px = 3.0 * (1.0 - quality);
  observation.tag_area_px2 = area;
  observation.pose_ambiguity = 0.5;
  return observation;
}

orbslam3_multi::SynchronizedFiducialBatch Batch(double stamp_sec, uint64_t kf_id = 1)
{
  orbslam3_multi::SynchronizedFiducialBatch batch;
  batch.keyframe_id = {1, 0, kf_id};
  batch.raw_first_arrival_id = kf_id;
  batch.raw_keyframe.stamp.sec = static_cast<int32_t>(std::floor(stamp_sec));
  batch.raw_keyframe.stamp.nanosec = static_cast<uint32_t>(
    (stamp_sec - std::floor(stamp_sec)) * 1e9);
  batch.batch.drone_id = 1;
  batch.batch.map_epoch = 0;
  batch.batch.local_keyframe_id = kf_id;
  batch.batch.header.stamp = batch.raw_keyframe.stamp;
  return batch;
}

std::unique_ptr<FiducialObjectInterpreter> Interpreter()
{
  auto interpreter = std::make_unique<FiducialObjectInterpreter>();
  interpreter->Load(FIDUCIAL_TEST_CONFIG);
  return interpreter;
}

TEST(FiducialObjectInterpreterTest, LoadsCanonicalObjectsAndTags)
{
  auto interpreter = Interpreter();
  EXPECT_EQ(interpreter->ObjectCount(), 3U);
  EXPECT_EQ(interpreter->TagCount(), 15U);
  const auto config = interpreter->GetConfig();
  EXPECT_DOUBLE_EQ(config.min_distance_m, 1.0);
  EXPECT_DOUBLE_EQ(config.max_distance_m, 5.0);
}

TEST(FiducialObjectInterpreterTest, ReconstructsOneFaceAndAssignsPrimary)
{
  auto interpreter = Interpreter();
  auto batch = Batch(10.0);
  Eigen::Isometry3d world_T_camera = Eigen::Isometry3d::Identity();
  world_T_camera.translation() = Eigen::Vector3d(0.0, 6.5, 1.0);
  batch.batch.observations.push_back(
    ObservationForCamera(103, 1, "pos_y", world_T_camera));

  const auto result = interpreter->Interpret(batch);
  ASSERT_EQ(result.objects.size(), 1U);
  ASSERT_TRUE(result.primary_index.has_value());
  EXPECT_EQ(result.primary_visit_id, 1U);
  EXPECT_TRUE(result.objects[0].anchor_eligible);
  EXPECT_NEAR(result.objects[0].world_T_camera_fused.position.x, 0.0, 1e-9);
  EXPECT_NEAR(result.objects[0].world_T_camera_fused.position.y, 6.5, 1e-9);
  EXPECT_NEAR(result.objects[0].world_T_camera_fused.position.z, 1.0, 1e-9);
}

TEST(FiducialObjectInterpreterTest, AnyOutOfRangeTagRejectsWholeObjectButKeepsIt)
{
  auto interpreter = Interpreter();
  auto batch = Batch(10.0);
  Eigen::Isometry3d world_T_camera = Eigen::Isometry3d::Identity();
  world_T_camera.translation() = Eigen::Vector3d(0.0, 6.5, 1.0);
  batch.batch.observations.push_back(
    ObservationForCamera(103, 1, "pos_y", world_T_camera));
  auto far = ObservationForCamera(101, 1, "pos_x", world_T_camera);
  far.camera_t_tag.translation.z += 5.2;
  batch.batch.observations.push_back(far);

  const auto result = interpreter->Interpret(batch);
  ASSERT_EQ(result.objects.size(), 1U);
  EXPECT_FALSE(result.objects[0].anchor_eligible);
  EXPECT_EQ(result.objects[0].reason, "tag_out_of_range");
  EXPECT_FALSE(result.primary_index.has_value());
  EXPECT_EQ(result.objects[0].tags.size(), 2U);
}

TEST(FiducialObjectInterpreterTest, RobustFusionKeepsLowQualityOutlierAuditable)
{
  auto interpreter = Interpreter();
  auto batch = Batch(10.0);
  Eigen::Isometry3d world_T_camera = Eigen::Isometry3d::Identity();
  world_T_camera.translation() = Eigen::Vector3d(0.0, 6.5, 1.0);
  batch.batch.observations.push_back(
    ObservationForCamera(103, 1, "pos_y", world_T_camera, 0.95, 500.0));
  batch.batch.observations.push_back(
    ObservationForCamera(101, 1, "pos_x", world_T_camera, 0.90, 450.0));
  Eigen::Isometry3d bad_camera = world_T_camera;
  bad_camera.translation().x() += 0.8;
  batch.batch.observations.push_back(
    ObservationForCamera(105, 1, "pos_z", bad_camera, 0.10, 100.0));

  const auto result = interpreter->Interpret(batch);
  ASSERT_TRUE(result.primary_index.has_value());
  const auto & object = result.objects[*result.primary_index];
  EXPECT_TRUE(object.anchor_eligible);
  ASSERT_EQ(object.tags.size(), 3U);
  EXPECT_NEAR(object.world_T_camera_fused.position.x, 0.0, 0.08);
  const auto bad = std::find_if(
    object.tags.begin(), object.tags.end(), [](const auto & tag) {return tag.tag_id == 105;});
  ASSERT_NE(bad, object.tags.end());
  EXPECT_LT(bad->robust_weight, bad->base_weight);
}

TEST(FiducialObjectInterpreterTest, VisitGapDoesNotDropKeyFramesAndFifoIsBounded)
{
  auto interpreter = Interpreter();
  auto config = interpreter->GetConfig();
  config.visit_gap_sec = 2.0;
  config.recent_capacity_per_drone = 2;
  interpreter->Configure(config);
  Eigen::Isometry3d world_T_camera = Eigen::Isometry3d::Identity();
  world_T_camera.translation() = Eigen::Vector3d(0.0, 6.5, 1.0);

  auto first = Batch(10.0, 1);
  first.batch.observations.push_back(
    ObservationForCamera(103, 1, "pos_y", world_T_camera));
  auto second = Batch(11.0, 2);
  second.batch.observations = first.batch.observations;
  auto delayed = Batch(10.5, 3);
  delayed.batch.observations = first.batch.observations;
  auto third = Batch(13.1, 4);
  third.batch.observations = first.batch.observations;

  EXPECT_EQ(interpreter->Interpret(first).primary_visit_id, 1U);
  EXPECT_EQ(interpreter->Interpret(second).primary_visit_id, 1U);
  EXPECT_EQ(interpreter->Interpret(delayed).primary_visit_id, 1U);
  EXPECT_EQ(interpreter->Interpret(third).primary_visit_id, 2U);
  const auto recent = interpreter->GetRecent(1);
  ASSERT_EQ(recent.size(), 2U);
  EXPECT_EQ(recent[0].keyframe_id.local_kf_id, 3U);
  EXPECT_EQ(recent[1].keyframe_id.local_kf_id, 4U);
}

}  // namespace
