#include "task_server/drone_registry.hpp"

#include <gtest/gtest.h>

namespace
{

mission_msgs::msg::DroneRegistration ValidRegistration(std::uint32_t drone_id = 1U)
{
  mission_msgs::msg::DroneRegistration registration;
  registration.drone_id = drone_id;
  registration.dimensions_m.x = 0.6;
  registration.dimensions_m.y = 0.6;
  registration.dimensions_m.z = 0.3;
  registration.vehicle_profile = "x500_depth";
  registration.protocol_version = 1U;
  registration.trajectory_generator_id = "lib_tray";
  registration.trajectory_generator_version = 1U;
  registration.capability_mask =
    mission_msgs::msg::DroneRegistration::CAPABILITY_SPARSE_MAPPING |
    mission_msgs::msg::DroneRegistration::CAPABILITY_STEREO_DEPTH;
  return registration;
}

TEST(DroneRegistry, AcceptsAndMakesExactDuplicateIdempotent)
{
  task_server::DroneRegistry registry({1U, 2U}, 1U, "lib_tray", 1U, 3U);
  const auto first = registry.Register(ValidRegistration());
  const auto duplicate = registry.Register(ValidRegistration());
  EXPECT_TRUE(first.accepted);
  EXPECT_TRUE(first.changed);
  EXPECT_TRUE(duplicate.accepted);
  EXPECT_FALSE(duplicate.changed);
  EXPECT_EQ(registry.Snapshot().size(), 1U);
}

TEST(DroneRegistry, RejectsUnknownVersionCapabilityAndConflictingDuplicate)
{
  task_server::DroneRegistry registry({1U, 2U}, 1U, "lib_tray", 1U, 3U);
  EXPECT_FALSE(registry.Register(ValidRegistration(3U)).accepted);
  auto bad_version = ValidRegistration();
  bad_version.protocol_version = 2U;
  EXPECT_FALSE(registry.Register(bad_version).accepted);
  auto bad_capability = ValidRegistration();
  bad_capability.capability_mask = 1U;
  EXPECT_FALSE(registry.Register(bad_capability).accepted);
  ASSERT_TRUE(registry.Register(ValidRegistration()).accepted);
  auto conflict = ValidRegistration();
  conflict.vehicle_profile = "otro";
  EXPECT_FALSE(registry.Register(conflict).accepted);
}

}  // namespace
