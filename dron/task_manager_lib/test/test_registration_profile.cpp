#include "task_manager_lib/registration_profile.hpp"

#include <gtest/gtest.h>

namespace
{

TEST(RegistrationProfile, BuildsTypedHandshake)
{
  task_manager_lib::RegistrationProfile profile;
  profile.drone_id = 2U;
  profile.length_m = 0.6;
  profile.width_m = 0.6;
  profile.height_m = 0.3;
  profile.vehicle_profile = "x500_depth";
  profile.capability_mask =
    mission_msgs::msg::DroneRegistration::CAPABILITY_SPARSE_MAPPING |
    mission_msgs::msg::DroneRegistration::CAPABILITY_STEREO_DEPTH |
    mission_msgs::msg::DroneRegistration::CAPABILITY_VISUAL_FIDUCIAL;
  const auto result = task_manager_lib::BuildRegistration(profile);
  EXPECT_EQ(result.drone_id, 2U);
  EXPECT_EQ(result.trajectory_generator_id, "lib_tray");
  EXPECT_EQ(result.capability_mask, 11U);
}

TEST(RegistrationProfile, RejectsInvalidIdentityAndDimensions)
{
  task_manager_lib::RegistrationProfile profile;
  EXPECT_THROW(task_manager_lib::BuildRegistration(profile), std::invalid_argument);
  profile.drone_id = 1U;
  profile.length_m = 0.6;
  profile.width_m = 0.6;
  profile.height_m = -0.1;
  profile.vehicle_profile = "x500_depth";
  EXPECT_THROW(task_manager_lib::BuildRegistration(profile), std::invalid_argument);
}

}  // namespace
