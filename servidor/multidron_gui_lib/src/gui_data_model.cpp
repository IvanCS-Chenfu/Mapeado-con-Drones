#include "multidron_gui_lib/gui_data_model.hpp"

#include <utility>

namespace multidron_gui_lib
{

GuiDataModel::GuiDataModel()
: sparse_points_(std::make_shared<const SparsePointVector>()),
  keyframes_(std::make_shared<const KeyframeVector>()),
  drones_(std::make_shared<const DroneStateMap>()),
  fiducials_(std::make_shared<const FiducialVector>()),
  trajectories_(std::make_shared<const TrajectoryMap>()),
  voxels_(std::make_shared<const VoxelVector>()),
  tasks_(std::make_shared<const TaskStateMap>())
{
}

GuiSnapshot GuiDataModel::Snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  GuiSnapshot snapshot;
  snapshot.generation = generation_;
  snapshot.sparse_points = sparse_points_;
  snapshot.keyframes = keyframes_;
  snapshot.drones = drones_;
  snapshot.fiducials = fiducials_;
  snapshot.trajectories = trajectories_;
  snapshot.voxels = voxels_;
  snapshot.tasks = tasks_;
  return snapshot;
}

void GuiDataModel::SetSparsePoints(SparsePointVector points)
{
  auto immutable = std::make_shared<const SparsePointVector>(std::move(points));
  std::lock_guard<std::mutex> lock(mutex_);
  sparse_points_ = std::move(immutable);
  BumpGenerationLocked();
}

void GuiDataModel::SetKeyframes(KeyframeVector keyframes)
{
  auto immutable = std::make_shared<const KeyframeVector>(std::move(keyframes));
  std::lock_guard<std::mutex> lock(mutex_);
  keyframes_ = std::move(immutable);
  BumpGenerationLocked();
}

void GuiDataModel::UpdateDrone(const DroneState & drone)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto next = std::make_shared<DroneStateMap>(*drones_);
  (*next)[drone.drone_id] = drone;
  drones_ = std::move(next);
  BumpGenerationLocked();
}

void GuiDataModel::SetFiducials(FiducialVector fiducials)
{
  auto immutable = std::make_shared<const FiducialVector>(std::move(fiducials));
  std::lock_guard<std::mutex> lock(mutex_);
  fiducials_ = std::move(immutable);
  BumpGenerationLocked();
}

void GuiDataModel::ReplaceTrajectory(const TrajectoryVisual & trajectory)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto next = std::make_shared<TrajectoryMap>(*trajectories_);
  // Contrato visual F7: una sola trayectoria futura vigente por dron.
  (*next)[trajectory.drone_id] = trajectory;
  trajectories_ = std::move(next);
  BumpGenerationLocked();
}

void GuiDataModel::ClearTrajectory(std::uint32_t drone_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto next = std::make_shared<TrajectoryMap>(*trajectories_);
  next->erase(drone_id);
  trajectories_ = std::move(next);
  BumpGenerationLocked();
}

void GuiDataModel::SetVoxels(VoxelVector voxels)
{
  auto immutable = std::make_shared<const VoxelVector>(std::move(voxels));
  std::lock_guard<std::mutex> lock(mutex_);
  voxels_ = std::move(immutable);
  BumpGenerationLocked();
}

void GuiDataModel::UpdateTask(const TaskVisual & task)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto next = std::make_shared<TaskStateMap>(*tasks_);
  (*next)[task.drone_id] = task;
  tasks_ = std::move(next);
  BumpGenerationLocked();
}

void GuiDataModel::ClearTask(std::uint32_t drone_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto next = std::make_shared<TaskStateMap>(*tasks_);
  next->erase(drone_id);
  tasks_ = std::move(next);
  BumpGenerationLocked();
}

void GuiDataModel::BumpGenerationLocked()
{
  ++generation_;
}

}  // namespace multidron_gui_lib
