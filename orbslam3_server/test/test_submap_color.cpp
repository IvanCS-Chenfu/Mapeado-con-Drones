#include "orbslam3_server/submap_color.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace
{

double Distance(
  const std_msgs::msg::ColorRGBA & lhs,
  const std_msgs::msg::ColorRGBA & rhs)
{
  const double red = static_cast<double>(lhs.r - rhs.r);
  const double green = static_cast<double>(lhs.g - rhs.g);
  const double blue = static_cast<double>(lhs.b - rhs.b);
  return std::sqrt(red * red + green * green + blue * blue);
}

TEST(SubmapColor, IsStableAndSeparatesConsecutiveEpochs)
{
  for (uint32_t drone_id = 1; drone_id <= 4; ++drone_id) {
    for (uint64_t epoch = 0; epoch < 16; ++epoch) {
      const auto color = orbslam3_server::SubmapColor(drone_id, epoch);
      const auto repeated = orbslam3_server::SubmapColor(drone_id, epoch);
      EXPECT_FLOAT_EQ(color.r, repeated.r);
      EXPECT_FLOAT_EQ(color.g, repeated.g);
      EXPECT_FLOAT_EQ(color.b, repeated.b);
      EXPECT_FLOAT_EQ(color.a, 1.0F);

      const auto next = orbslam3_server::SubmapColor(drone_id, epoch + 1);
      EXPECT_GT(Distance(color, next), 0.45) <<
        "drone=" << drone_id << " epoch=" << epoch;
    }
  }
}

TEST(SubmapColor, SeparatesDronesAtTheSameEpoch)
{
  for (uint64_t epoch = 0; epoch < 16; ++epoch) {
    EXPECT_GT(
      Distance(
        orbslam3_server::SubmapColor(1, epoch),
        orbslam3_server::SubmapColor(2, epoch)),
      0.45) << "epoch=" << epoch;
  }
}

}  // namespace
