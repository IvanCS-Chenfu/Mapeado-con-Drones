#ifndef DRON_INDIVIDUAL_TRAJECTORY_YAW_HPP_
#define DRON_INDIVIDUAL_TRAJECTORY_YAW_HPP_

#include <cmath>

namespace dron_individual
{

inline double NearestEquivalentYaw(double yaw_rad, double reference_yaw_rad)
{
  return reference_yaw_rad + std::remainder(yaw_rad - reference_yaw_rad, 2.0 * M_PI);
}

}  // namespace dron_individual

#endif  // DRON_INDIVIDUAL_TRAJECTORY_YAW_HPP_
