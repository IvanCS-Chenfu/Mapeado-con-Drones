#pragma once

#include "multidron_gui_lib/types.hpp"

#include <mutex>

namespace multidron_gui_lib
{

/// Frontera thread-safe entre callbacks ROS y el thread de Qt.
/// Las colecciones grandes se intercambian como snapshots inmutables shared_ptr,
/// evitando copiar la nube sparse completa a cada frame gráfico.
class GuiDataModel
{
public:
  GuiDataModel();

  GuiSnapshot Snapshot() const;

  void SetSparsePoints(SparsePointVector points);
  void SetKeyframes(KeyframeVector keyframes);
  bool UpdateDrone(const DroneState & drone);
  std::size_t MarkStaleDrones(std::int64_t now_steady_ns, std::int64_t timeout_ns);
  void SetFiducials(FiducialVector fiducials);

  // Slots de integración para Fase 6. No existe subscriber ROS de estos datos
  // hasta que los contratos reales de task_server/mission_msgs estén cerrados.
  void ReplaceTrajectory(const TrajectoryVisual & trajectory);
  void ClearTrajectory(std::uint32_t drone_id);
  void SetVoxels(VoxelVector voxels);
  void UpdateTask(const TaskVisual & task);
  void ClearTask(std::uint32_t drone_id);
  void SetMissionRegions(MissionRegionVector regions);

private:
  void BumpGenerationLocked();
  static bool IsNewerDroneState(const DroneState & incoming, const DroneState & current);

  mutable std::mutex mutex_;
  std::uint64_t generation_ = 0;

  std::shared_ptr<const SparsePointVector> sparse_points_;
  std::shared_ptr<const KeyframeVector> keyframes_;
  std::shared_ptr<const DroneStateMap> drones_;
  std::shared_ptr<const FiducialVector> fiducials_;
  std::shared_ptr<const TrajectoryMap> trajectories_;
  std::shared_ptr<const VoxelVector> voxels_;
  std::shared_ptr<const TaskStateMap> tasks_;
  std::shared_ptr<const MissionRegionVector> mission_regions_;
};

}  // namespace multidron_gui_lib
