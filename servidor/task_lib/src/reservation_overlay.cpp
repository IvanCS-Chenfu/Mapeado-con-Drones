#include "task_lib/reservation_overlay.hpp"

#include <algorithm>
#include <utility>

namespace task_lib
{

bool ReservationOverlay::CanCommit(
  std::uint32_t drone_id, const std::set<VoxelKey> & cells) const
{
  for (const auto & item : reservations_) {
    if (item.second.drone_id == drone_id) {
      continue;
    }
    for (const auto & cell : cells) {
      if (item.second.cells.count(cell) != 0U) {
        return false;
      }
    }
  }
  return true;
}

bool ReservationOverlay::Commit(SpatialReservation reservation)
{
  if (reservation.reservation_id.empty() || reservation.drone_id == 0U ||
    reservation.cells.empty() || reservations_.count(reservation.reservation_id) != 0U ||
    !CanCommit(reservation.drone_id, reservation.cells))
  {
    return false;
  }
  reservation.revision = ++revision_;
  reservations_.emplace(reservation.reservation_id, std::move(reservation));
  return true;
}

bool ReservationOverlay::Replace(const std::string & previous_id, SpatialReservation reservation)
{
  const auto previous = reservations_.find(previous_id);
  if (previous == reservations_.end() || previous->second.drone_id != reservation.drone_id ||
    reservation.reservation_id.empty() || reservation.cells.empty())
  {
    return false;
  }
  const auto preserved = previous->second;
  reservations_.erase(previous);
  if (!CanCommit(reservation.drone_id, reservation.cells)) {
    reservations_.emplace(preserved.reservation_id, preserved);
    return false;
  }
  reservation.revision = ++revision_;
  reservations_.emplace(reservation.reservation_id, std::move(reservation));
  return true;
}

bool ReservationOverlay::Release(const std::string & reservation_id)
{
  const auto existing = reservations_.find(reservation_id);
  if (existing == reservations_.end()) {
    return false;
  }
  reservations_.erase(existing);
  ++revision_;
  return true;
}

std::uint64_t ReservationOverlay::revision() const
{
  return revision_;
}

std::vector<SpatialReservation> ReservationOverlay::Snapshot() const
{
  std::vector<SpatialReservation> result;
  result.reserve(reservations_.size());
  for (const auto & item : reservations_) {
    result.push_back(item.second);
  }
  return result;
}

NavigationSnapshot ReservationOverlay::ApplyTo(
  const NavigationSnapshot & base, std::uint32_t requesting_drone_id,
  VoxelKey requesting_body_half_extent_cells) const
{
  auto result = base;
  std::map<VoxelKey, NavigationCell> cells;
  for (const auto & cell : result.cells) {
    cells.emplace(cell.key, cell);
  }
  for (const auto & item : reservations_) {
    const auto & reservation = item.second;
    if (reservation.drone_id == requesting_drone_id) {
      continue;
    }
    for (const auto & key : reservation.cells) {
      // This is the requesting drone's physical footprint, not inter-drone clearance.
      for (std::int64_t ix = -requesting_body_half_extent_cells.ix;
        ix <= requesting_body_half_extent_cells.ix; ++ix)
      {
        for (std::int64_t iy = -requesting_body_half_extent_cells.iy;
          iy <= requesting_body_half_extent_cells.iy; ++iy)
        {
          for (std::int64_t iz = -requesting_body_half_extent_cells.iz;
            iz <= requesting_body_half_extent_cells.iz; ++iz)
          {
            const VoxelKey blocked{key.ix + ix, key.iy + iy, key.iz + iz};
            auto found = cells.find(blocked);
            if (found == cells.end()) {
              cells.emplace(
                blocked, NavigationCell{blocked, VoxelState::Unknown, false, 0U,
                  result.unknown_cost_multiplier});
            } else {
              found->second.traversable = false;
              found->second.neighbor_mask = 0U;
            }
          }
        }
      }
    }
  }
  result.cells.clear();
  result.cells.reserve(cells.size());
  for (const auto & item : cells) {
    result.cells.push_back(item.second);
  }
  return result;
}

}  // namespace task_lib
