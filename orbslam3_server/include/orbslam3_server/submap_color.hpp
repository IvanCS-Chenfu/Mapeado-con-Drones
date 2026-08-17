#pragma once

#include "std_msgs/msg/color_rgba.hpp"

#include <cmath>
#include <cstdint>

namespace orbslam3_server
{

inline std_msgs::msg::ColorRGBA SubmapColor(uint32_t drone_id, uint64_t map_epoch)
{
  constexpr double kDroneHueStep = 67.0;
  constexpr double kEpochHueStep = 137.50776405003785;
  const double hue_degrees = std::fmod(
    static_cast<double>(drone_id) * kDroneHueStep +
    static_cast<double>(map_epoch) * kEpochHueStep,
    360.0);
  const double hue = hue_degrees / 60.0;
  const int sector = static_cast<int>(std::floor(hue)) % 6;
  const float x = static_cast<float>(1.0 - std::fabs(std::fmod(hue, 2.0) - 1.0));
  float red = 0.0F;
  float green = 0.0F;
  float blue = 0.0F;
  switch (sector) {
    case 0: red = 1.0F; green = x; break;
    case 1: red = x; green = 1.0F; break;
    case 2: green = 1.0F; blue = x; break;
    case 3: green = x; blue = 1.0F; break;
    case 4: red = x; blue = 1.0F; break;
    default: red = 1.0F; blue = x; break;
  }

  std_msgs::msg::ColorRGBA color;
  color.r = 0.22F + 0.78F * red;
  color.g = 0.22F + 0.78F * green;
  color.b = 0.22F + 0.78F * blue;
  color.a = 1.0F;
  return color;
}

}  // namespace orbslam3_server
