#pragma once

#include <mission_msgs/msg/drone_registration.hpp>

#include <cstdint>
#include <string>

namespace task_manager_lib
{

struct RegistrationProfile
{
  std::uint32_t drone_id = 0U;
  double length_m = 0.0;
  double width_m = 0.0;
  double height_m = 0.0;
  std::string vehicle_profile;
  std::uint32_t protocol_version = 1U;
  std::string generator_id = "lib_tray";
  std::uint32_t generator_version = 1U;
  std::uint64_t capability_mask = 0U;
};

mission_msgs::msg::DroneRegistration BuildRegistration(const RegistrationProfile & profile);

}  // namespace task_manager_lib
