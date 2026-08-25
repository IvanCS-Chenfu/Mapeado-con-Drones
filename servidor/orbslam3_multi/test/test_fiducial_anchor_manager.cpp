#include "orbslam3_multi/fiducial_anchor_manager.hpp"

#include <gtest/gtest.h>

namespace
{

geometry_msgs::msg::Pose MakePose(double x, double y = 0.0, double z = 0.0)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;
  return pose;
}

orbslam3_multi::FiducialObservation MakeObservation()
{
  orbslam3_multi::FiducialObservation observation;
  observation.arrival_id = 7;
  observation.keyframe_id = {2, 4, 9};
  observation.fiducial_id = 2;
  observation.fiducial_visit_id = 11;
  observation.world_T_camera_target = MakePose(10.0);
  observation.source = "visual_fiducial";
  return observation;
}

TEST(FiducialAnchorManager, ComputesFirstAnchorFromCameraPoses)
{
  orbslam3_multi::FiducialAnchorManager manager;
  const auto result = manager.Evaluate(MakeObservation(), MakePose(3.0), std::nullopt, false);

  EXPECT_EQ(result.status, orbslam3_multi::FiducialProcessStatus::AnchorCreated);
  EXPECT_NEAR(result.world_T_local.position.x, 7.0, 1e-9);
  EXPECT_EQ(result.reason, "first_valid_observation");
}

TEST(FiducialAnchorManager, CreatesTaskForHighErrorRevisit)
{
  orbslam3_multi::FiducialAnchorManager manager;
  const orbslam3_multi::RawSubmapId submap{2, 4};
  manager.AcceptControl(submap, 10, {2, 4, 1});
  orbslam3_multi::GlobalPoseRecord current;
  current.keyframe_id = {2, 4, 9};
  current.world_pose = MakePose(8.0);
  current.pose_revision = 3;
  const auto result = manager.Evaluate(
    MakeObservation(), MakePose(3.0), current, true);

  EXPECT_EQ(
    result.status, orbslam3_multi::FiducialProcessStatus::OptimizationRequired);
  ASSERT_TRUE(result.optimization_task.has_value());
  EXPECT_EQ(result.optimization_task->control_keyframe_id.local_kf_id, 1U);
  EXPECT_NEAR(result.error.translation_m, 2.0, 1e-9);
}

TEST(FiducialAnchorManager, FirstCoherentKeyFramePromotesVisitControl)
{
  orbslam3_multi::FiducialAnchorManager manager;
  const orbslam3_multi::RawSubmapId submap{2, 4};
  manager.AcceptControl(submap, 10, {2, 4, 1});
  auto observation = MakeObservation();
  orbslam3_multi::GlobalPoseRecord current;
  current.keyframe_id = observation.keyframe_id;
  current.world_pose = observation.world_T_camera_target;
  const auto result = manager.Evaluate(observation, MakePose(3.0), current, true);

  EXPECT_EQ(
    result.status, orbslam3_multi::FiducialProcessStatus::RevisitWithinThreshold);
  EXPECT_TRUE(result.promote_control);
}

TEST(FiducialAnchorManager, LaterCoherentKeyFrameCannotOvertakePendingVisitControl)
{
  orbslam3_multi::FiducialAnchorManager manager;
  const orbslam3_multi::RawSubmapId submap{2, 4};
  manager.AcceptControl(submap, 10, {2, 4, 1});

  auto first = MakeObservation();
  orbslam3_multi::GlobalPoseRecord first_current;
  first_current.keyframe_id = first.keyframe_id;
  first_current.world_pose = MakePose(8.0);
  const auto pending = manager.Evaluate(first, MakePose(3.0), first_current, true);
  ASSERT_EQ(
    pending.status, orbslam3_multi::FiducialProcessStatus::OptimizationRequired);
  EXPECT_FALSE(pending.promote_control);

  auto later = first;
  later.arrival_id++;
  later.keyframe_id.local_kf_id++;
  orbslam3_multi::GlobalPoseRecord later_current;
  later_current.keyframe_id = later.keyframe_id;
  later_current.world_pose = later.world_T_camera_target;
  const auto coherent = manager.Evaluate(later, MakePose(3.0), later_current, true);

  EXPECT_EQ(
    coherent.status, orbslam3_multi::FiducialProcessStatus::RevisitWithinThreshold);
  EXPECT_FALSE(coherent.promote_control);
  EXPECT_EQ(coherent.reason, "same_visit_within_threshold");
  ASSERT_TRUE(manager.GetLastAcceptedControl(submap).has_value());
  EXPECT_EQ(manager.GetLastAcceptedControl(submap)->local_kf_id, 1U);
}

TEST(FiducialAnchorManager, RejectsInvalidNormalizedObservation)
{
  orbslam3_multi::FiducialAnchorManager manager;
  auto observation = MakeObservation();
  observation.fiducial_id = 0;
  const auto result = manager.Evaluate(
    observation, MakePose(3.0), std::nullopt, false);

  EXPECT_EQ(result.status, orbslam3_multi::FiducialProcessStatus::InvalidObservation);
}

}  // namespace
