#pragma once

#include "task_lib/voxel_map.hpp"

#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace task_lib
{

enum class ReservationMode : std::uint8_t
{
  Moving = 0U,
  Hold = 1U,
};

struct SpatialReservation
{
  std::string reservation_id;
  std::uint32_t drone_id = 0U;
  ReservationMode mode = ReservationMode::Moving;
  std::set<VoxelKey> cells;
  std::uint64_t revision = 0U;
};

// Overlay dinamico: no toca la evidencia raw del mapa ni sus revisiones.
class ReservationOverlay
{
public:
  bool CanCommit(std::uint32_t drone_id, const std::set<VoxelKey> & cells) const;
  bool Commit(SpatialReservation reservation);
  bool Replace(const std::string & previous_id, SpatialReservation reservation);
  bool Release(const std::string & reservation_id);

  std::uint64_t revision() const;
  std::vector<SpatialReservation> Snapshot() const;
  NavigationSnapshot ApplyTo(
    const NavigationSnapshot & base, std::uint32_t requesting_drone_id,
    VoxelKey requesting_body_half_extent_cells = {}) const;

private:
  std::uint64_t revision_ = 0U;
  std::map<std::string, SpatialReservation> reservations_;
};

}  // namespace task_lib
