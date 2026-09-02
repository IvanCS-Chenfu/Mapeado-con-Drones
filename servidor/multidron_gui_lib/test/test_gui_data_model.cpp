#include "multidron_gui_lib/gui_data_model.hpp"

#include <gtest/gtest.h>

using multidron_gui_lib::DroneState;
using multidron_gui_lib::GuiDataModel;
using multidron_gui_lib::MissionRegionVisual;
using multidron_gui_lib::MissionRegionVector;
using multidron_gui_lib::SparsePoint;
using multidron_gui_lib::SparsePointVector;
using multidron_gui_lib::TrajectoryVisual;
using multidron_gui_lib::VoxelState;
using multidron_gui_lib::VoxelVisual;
using multidron_gui_lib::VoxelVector;

TEST(GuiDataModel, SwapsLargeSnapshotsWithoutChangingOldSnapshot)
{
  GuiDataModel model;
  SparsePointVector first;
  first.push_back(SparsePoint{QVector3D(1.0F, 2.0F, 3.0F), 0.2F, 1U, 7U, 11U});
  model.SetSparsePoints(first);
  const auto old_snapshot = model.Snapshot();

  SparsePointVector second;
  second.push_back(SparsePoint{QVector3D(4.0F, 5.0F, 6.0F), 0.9F, 2U, 8U, 12U});
  second.push_back(SparsePoint{QVector3D(7.0F, 8.0F, 9.0F), 0.8F, 2U, 8U, 13U});
  model.SetSparsePoints(second);
  const auto new_snapshot = model.Snapshot();

  ASSERT_EQ(old_snapshot.sparse_points->size(), 1U);
  ASSERT_EQ(new_snapshot.sparse_points->size(), 2U);
  EXPECT_GT(new_snapshot.generation, old_snapshot.generation);
}

TEST(GuiDataModel, KeepsOneCurrentTrajectoryPerDrone)
{
  GuiDataModel model;
  TrajectoryVisual first;
  first.drone_id = 1U;
  first.trajectory_id = "traj_A";
  first.plan_revision = 1U;
  first.samples_world = {
    QVector3D(0, 0, 0), QVector3D(0.5F, 0.2F, 0.1F),
    QVector3D(1.0F, 0.5F, 0.2F), QVector3D(1.5F, 0.8F, 0.3F)};
  model.ReplaceTrajectory(first);

  TrajectoryVisual replacement = first;
  replacement.trajectory_id = "traj_B";
  replacement.plan_revision = 2U;
  replacement.samples_world = {
    QVector3D(1.5F, 0.8F, 0.3F), QVector3D(2.0F, 1.1F, 0.4F),
    QVector3D(2.7F, 1.2F, 0.5F), QVector3D(3.3F, 1.0F, 0.6F)};
  model.ReplaceTrajectory(replacement);

  const auto snapshot = model.Snapshot();
  ASSERT_EQ(snapshot.trajectories->size(), 1U);
  EXPECT_EQ(snapshot.trajectories->at(1U).trajectory_id, "traj_B");
  EXPECT_EQ(snapshot.trajectories->at(1U).plan_revision, 2U);
  EXPECT_EQ(snapshot.trajectories->at(1U).samples_world.size(), 4U);
}

TEST(GuiDataModel, StoresAllVoxelStatesForOneGlobalToggle)
{
  GuiDataModel model;
  VoxelVector voxels;
  voxels.push_back(VoxelVisual{0, 0, 0, QVector3D(0, 0, 0), 0.5F, VoxelState::Unknown, 0.0F});
  voxels.push_back(VoxelVisual{1, 0, 0, QVector3D(0.5F, 0, 0), 0.5F, VoxelState::Free, 0.0F});
  voxels.push_back(VoxelVisual{2, 0, 0, QVector3D(1.0F, 0, 0), 0.5F, VoxelState::Occupied, 0.8F});
  model.SetVoxels(voxels);

  const auto snapshot = model.Snapshot();
  ASSERT_EQ(snapshot.voxels->size(), 3U);
  EXPECT_EQ(snapshot.voxels->at(0).state, VoxelState::Unknown);
  EXPECT_EQ(snapshot.voxels->at(1).state, VoxelState::Free);
  EXPECT_EQ(snapshot.voxels->at(2).state, VoxelState::Occupied);
}

TEST(GuiDataModel, DroneUpdatePreservesExplicitLostRepresentation)
{
  GuiDataModel model;
  DroneState drone;
  drone.drone_id = 2U;
  drone.has_world_pose = true;
  drone.lost_or_unavailable = true;
  drone.position = QVector3D(3.0F, 4.0F, 2.0F);
  EXPECT_TRUE(model.UpdateDrone(drone));

  const auto snapshot = model.Snapshot();
  ASSERT_EQ(snapshot.drones->count(2U), 1U);
  EXPECT_TRUE(snapshot.drones->at(2U).lost_or_unavailable);
  EXPECT_EQ(snapshot.drones->at(2U).position, QVector3D(3.0F, 4.0F, 2.0F));
}

TEST(GuiDataModel, RejectsOlderDroneStateWithinEpoch)
{
  GuiDataModel model;
  DroneState current;
  current.drone_id = 1U;
  current.map_epoch = 4U;
  current.sample_sequence = 20U;
  current.pose_revision = 7U;
  current.received_steady_ns = 2000;
  current.position = QVector3D(2.0F, 0.0F, 0.0F);
  EXPECT_TRUE(model.UpdateDrone(current));

  DroneState old = current;
  old.sample_sequence = 19U;
  old.received_steady_ns = 3000;
  old.position = QVector3D(-5.0F, 0.0F, 0.0F);
  EXPECT_FALSE(model.UpdateDrone(old));
  EXPECT_EQ(model.Snapshot().drones->at(1U).position, current.position);
}

TEST(GuiDataModel, MarksSilentDroneStaleAndKeepsLastPose)
{
  GuiDataModel model;
  DroneState drone;
  drone.drone_id = 3U;
  drone.sample_sequence = 1U;
  drone.has_world_pose = true;
  drone.lost_or_unavailable = false;
  drone.received_steady_ns = 1000;
  drone.position = QVector3D(1.0F, 2.0F, 3.0F);
  ASSERT_TRUE(model.UpdateDrone(drone));

  EXPECT_EQ(model.MarkStaleDrones(2501, 1500), 1U);
  const auto stale = model.Snapshot().drones->at(3U);
  EXPECT_TRUE(stale.lost_or_unavailable);
  EXPECT_TRUE(stale.has_world_pose);
  EXPECT_EQ(stale.position, drone.position);
  EXPECT_EQ(model.MarkStaleDrones(5000, 1500), 0U);
}

TEST(GuiDataModel, StoresRealUnassignedMissionRegionsAsSnapshot)
{
  GuiDataModel model;
  MissionRegionVector regions;
  regions.push_back(MissionRegionVisual{
    "level_0_AB", 0U, "AB", QVector3D(-10.0F, -10.0F, 0.0F),
    QVector3D(10.0F, 0.0F, 2.0F)});
  model.SetMissionRegions(regions);
  const auto snapshot = model.Snapshot();
  ASSERT_EQ(snapshot.mission_regions->size(), 1U);
  EXPECT_EQ(snapshot.mission_regions->front().region_id, "level_0_AB");
  EXPECT_EQ(snapshot.mission_regions->front().side, "AB");
}
