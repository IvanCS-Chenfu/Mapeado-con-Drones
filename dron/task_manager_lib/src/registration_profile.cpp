#include "task_manager_lib/registration_profile.hpp"

#include <cmath>
#include <stdexcept>

namespace task_manager_lib
{

mission_msgs::msg::DroneRegistration BuildRegistration(const RegistrationProfile & profile)
{
  if (profile.drone_id == 0U) {
    throw std::invalid_argument("drone_id debe ser positivo");
  }
  if (!std::isfinite(profile.length_m) || !std::isfinite(profile.width_m) ||
    !std::isfinite(profile.height_m) || profile.length_m <= 0.0 ||
    profile.width_m <= 0.0 || profile.height_m <= 0.0)
  {
    throw std::invalid_argument("dimensiones del dron invalidas");
  }
  if (profile.vehicle_profile.empty() || profile.generator_id.empty() ||
    profile.protocol_version == 0U || profile.generator_version == 0U)
  {
    throw std::invalid_argument("perfil o version de protocolo/generador invalidos");
  }
  mission_msgs::msg::DroneRegistration message;
  message.drone_id = profile.drone_id;
  message.dimensions_m.x = profile.length_m;
  message.dimensions_m.y = profile.width_m;
  message.dimensions_m.z = profile.height_m;
  message.vehicle_profile = profile.vehicle_profile;
  message.protocol_version = profile.protocol_version;
  message.trajectory_generator_id = profile.generator_id;
  message.trajectory_generator_version = profile.generator_version;
  message.capability_mask = profile.capability_mask;
  return message;
}

}  // namespace task_manager_lib
