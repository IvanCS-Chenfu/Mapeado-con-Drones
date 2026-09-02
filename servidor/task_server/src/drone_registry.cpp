#include "task_server/drone_registry.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace task_server
{

DroneRegistry::DroneRegistry(std::vector<std::uint32_t> allowed_drones,
  std::uint32_t protocol_version, std::string generator_id,
  std::uint32_t generator_version, std::uint64_t required_capabilities)
: allowed_drones_(std::move(allowed_drones)), protocol_version_(protocol_version),
  generator_id_(std::move(generator_id)), generator_version_(generator_version),
  required_capabilities_(required_capabilities)
{
}

RegistrationResult DroneRegistry::Register(
  const mission_msgs::msg::DroneRegistration & registration)
{
  if (std::find(allowed_drones_.begin(), allowed_drones_.end(), registration.drone_id) ==
    allowed_drones_.end())
  {
    return {false, false, "drone_id no pertenece a la misión"};
  }
  const auto & d = registration.dimensions_m;
  if (!std::isfinite(d.x) || !std::isfinite(d.y) || !std::isfinite(d.z) ||
    d.x <= 0.0 || d.y <= 0.0 || d.z <= 0.0)
  {
    return {false, false, "dimensiones inválidas"};
  }
  if (registration.vehicle_profile.empty()) {
    return {false, false, "vehicle_profile vacío"};
  }
  if (registration.protocol_version != protocol_version_) {
    return {false, false, "protocol_version incompatible"};
  }
  if (registration.trajectory_generator_id != generator_id_ ||
    registration.trajectory_generator_version != generator_version_)
  {
    return {false, false, "generador de trayectoria incompatible"};
  }
  if ((registration.capability_mask & required_capabilities_) != required_capabilities_) {
    return {false, false, "capabilities obligatorias ausentes"};
  }
  const auto existing = registrations_.find(registration.drone_id);
  if (existing != registrations_.end()) {
    if (SameRegistration(existing->second, registration)) {
      return {true, false, "registro ya vigente"};
    }
    return {false, false, "drone_id ya registrado con otro perfil"};
  }
  registrations_.emplace(registration.drone_id, registration);
  return {true, true, "registro aceptado"};
}

std::vector<mission_msgs::msg::DroneRegistration> DroneRegistry::Snapshot() const
{
  std::vector<mission_msgs::msg::DroneRegistration> snapshot;
  snapshot.reserve(registrations_.size());
  for (const auto & item : registrations_) {
    snapshot.push_back(item.second);
  }
  return snapshot;
}

bool DroneRegistry::SameRegistration(const mission_msgs::msg::DroneRegistration & lhs,
  const mission_msgs::msg::DroneRegistration & rhs) const
{
  return lhs.drone_id == rhs.drone_id && lhs.dimensions_m.x == rhs.dimensions_m.x &&
         lhs.dimensions_m.y == rhs.dimensions_m.y && lhs.dimensions_m.z == rhs.dimensions_m.z &&
         lhs.vehicle_profile == rhs.vehicle_profile &&
         lhs.protocol_version == rhs.protocol_version &&
         lhs.trajectory_generator_id == rhs.trajectory_generator_id &&
         lhs.trajectory_generator_version == rhs.trajectory_generator_version &&
         lhs.capability_mask == rhs.capability_mask;
}

}  // namespace task_server
