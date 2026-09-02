#pragma once

#include <mission_msgs/msg/drone_registration.hpp>

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace task_server
{

struct RegistrationResult
{
  bool accepted = false;
  bool changed = false;
  std::string reason;
};

class DroneRegistry
{
public:
  DroneRegistry(std::vector<std::uint32_t> allowed_drones, std::uint32_t protocol_version,
    std::string generator_id, std::uint32_t generator_version,
    std::uint64_t required_capabilities);

  RegistrationResult Register(const mission_msgs::msg::DroneRegistration & registration);
  std::vector<mission_msgs::msg::DroneRegistration> Snapshot() const;

private:
  bool SameRegistration(const mission_msgs::msg::DroneRegistration & lhs,
    const mission_msgs::msg::DroneRegistration & rhs) const;
  std::vector<std::uint32_t> allowed_drones_;
  std::uint32_t protocol_version_;
  std::string generator_id_;
  std::uint32_t generator_version_;
  std::uint64_t required_capabilities_;
  std::map<std::uint32_t, mission_msgs::msg::DroneRegistration> registrations_;
};

}  // namespace task_server
