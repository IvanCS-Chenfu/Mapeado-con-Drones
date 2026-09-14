#pragma once

#include <geometry_msgs/msg/point32.hpp>

#include <opencv2/core.hpp>

#include <cstddef>
#include <limits>
#include <vector>

namespace orbslam3_ros2
{

struct DepthObservationParameters
{
  double min_depth_m = 1.0;
  double max_depth_m = 5.0;
  int pixel_stride = 8;
  std::size_t max_points = 256U;
  double max_disparity_gradient_px_per_pixel = 2.0;
  double min_texture_gradient = 8.0;
  int texture_window_radius_px = 2;
};

struct DepthObservationResult
{
  std::vector<geometry_msgs::msg::Point32> points_k;
  // Geometric candidates inside the valid depth band, before local quality filters.
  std::size_t raw_valid_points = 0U;
  std::size_t texture_rejected_points = 0U;
  std::size_t discontinuity_rejected_points = 0U;
  double confidence = 0.0;
  double nearest_depth_m = std::numeric_limits<double>::infinity();
  bool normal_valid = false;
  cv::Vec3d normal_camera{0.0, 0.0, 0.0};
  std::size_t normal_support = 0U;
  double normal_confidence = 0.0;
};

DepthObservationResult ComputeDepthObservation(
  const cv::Mat & left, const cv::Mat & right, double fx, double fy, double cx, double cy,
  double baseline_m, const DepthObservationParameters & parameters);

}  // namespace orbslam3_ros2
