#include "depth-observation-processor.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>

namespace orbslam3_ros2
{
namespace
{

cv::Mat ToGray(const cv::Mat & image)
{
  if (image.channels() == 1) {
    return image;
  }
  cv::Mat gray;
  cv::cvtColor(image, gray, image.channels() == 4 ? cv::COLOR_BGRA2GRAY : cv::COLOR_BGR2GRAY);
  return gray;
}

bool HasStrongDisparityDiscontinuity(
  const cv::Mat & disparity, int x, int y, float center_disparity,
  double max_gradient_px_per_pixel)
{
  constexpr int kNeighbors[][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  for (const auto & offset : kNeighbors) {
    const int neighbor_x = x + offset[0];
    const int neighbor_y = y + offset[1];
    if (neighbor_x < 0 || neighbor_x >= disparity.cols ||
      neighbor_y < 0 || neighbor_y >= disparity.rows)
    {
      continue;
    }
    const float neighbor_disparity =
      static_cast<float>(disparity.at<std::int16_t>(neighbor_y, neighbor_x)) / 16.0F;
    if (!std::isfinite(neighbor_disparity) || neighbor_disparity <= 0.0F ||
      std::abs(center_disparity - neighbor_disparity) > max_gradient_px_per_pixel)
    {
      return true;
    }
  }
  return false;
}

struct PlaneEstimate
{
  bool valid = false;
  cv::Vec3d normal{0.0, 0.0, 0.0};
  std::size_t support = 0U;
  double confidence = 0.0;
};

PlaneEstimate EstimatePlane(
  const std::vector<geometry_msgs::msg::Point32> & points,
  const std::vector<std::size_t> & indices)
{
  PlaneEstimate result;
  if (indices.size() < 12U) {
    return result;
  }
  cv::Vec3d mean{0.0, 0.0, 0.0};
  for (const auto index : indices) {
    mean += cv::Vec3d{points[index].x, points[index].y, points[index].z};
  }
  mean *= 1.0 / static_cast<double>(indices.size());
  cv::Matx33d covariance = cv::Matx33d::zeros();
  for (const auto index : indices) {
    const cv::Vec3d delta = cv::Vec3d{
      points[index].x, points[index].y, points[index].z} - mean;
    covariance += cv::Matx33d{
      delta[0] * delta[0], delta[0] * delta[1], delta[0] * delta[2],
      delta[1] * delta[0], delta[1] * delta[1], delta[1] * delta[2],
      delta[2] * delta[0], delta[2] * delta[1], delta[2] * delta[2]};
  }
  cv::Mat eigenvalues;
  cv::Mat eigenvectors;
  if (!cv::eigen(cv::Mat(covariance), eigenvalues, eigenvectors)) {
    return result;
  }
  cv::Vec3d normal{
    eigenvectors.at<double>(2, 0), eigenvectors.at<double>(2, 1),
    eigenvectors.at<double>(2, 2)};
  const double norm = cv::norm(normal);
  if (!std::isfinite(norm) || norm <= 1e-9) {
    return result;
  }
  normal *= 1.0 / norm;
  if (normal[2] > 0.0) {
    normal *= -1.0;
  }
  const double sum = eigenvalues.at<double>(0) + eigenvalues.at<double>(1) +
    eigenvalues.at<double>(2);
  const double confidence = sum <= 1e-12 ? 0.0 :
    std::clamp(1.0 - eigenvalues.at<double>(2) / sum, 0.0, 1.0);
  result = {confidence >= 0.7, normal, indices.size(), confidence};
  return result;
}

PlaneEstimate EstimateFacadeNormal(
  const std::vector<geometry_msgs::msg::Point32> & points)
{
  std::vector<std::size_t> all(points.size());
  std::iota(all.begin(), all.end(), 0U);
  const auto dominant = EstimatePlane(points, all);
  if (points.size() < 24U) {
    return dominant;
  }
  std::vector<float> x_values;
  x_values.reserve(points.size());
  for (const auto & point : points) {
    x_values.push_back(point.x);
  }
  const auto middle = x_values.begin() + static_cast<std::ptrdiff_t>(x_values.size() / 2U);
  std::nth_element(x_values.begin(), middle, x_values.end());
  const float median_x = *middle;
  std::vector<std::size_t> left;
  std::vector<std::size_t> right;
  for (std::size_t index = 0U; index < points.size(); ++index) {
    (points[index].x <= median_x ? left : right).push_back(index);
  }
  auto first = EstimatePlane(points, left);
  auto second = EstimatePlane(points, right);
  if (!first.valid || !second.valid) {
    return dominant;
  }
  double dot = first.normal.dot(second.normal);
  if (dot < 0.0) {
    second.normal *= -1.0;
    dot = -dot;
  }
  const double angle_deg = std::acos(std::clamp(dot, -1.0, 1.0)) * 180.0 / CV_PI;
  if (angle_deg < 20.0 || angle_deg > 140.0) {
    return dominant;
  }
  const double first_weight = static_cast<double>(first.support) * first.confidence;
  const double second_weight = static_cast<double>(second.support) * second.confidence;
  cv::Vec3d blended = first_weight * first.normal + second_weight * second.normal;
  const double norm = cv::norm(blended);
  if (norm <= 1e-9) {
    return dominant;
  }
  blended *= 1.0 / norm;
  return {true, blended, first.support + second.support,
    (first_weight + second_weight) /
    static_cast<double>(first.support + second.support)};
}

}  // namespace

DepthObservationResult ComputeDepthObservation(
  const cv::Mat & left, const cv::Mat & right, double fx, double fy, double cx, double cy,
  double baseline_m, const DepthObservationParameters & parameters)
{
  DepthObservationResult result;
  if (left.empty() || right.empty() || left.size() != right.size() || fx <= 0.0 || fy <= 0.0 ||
    baseline_m <= 0.0 || parameters.min_depth_m <= 0.0 ||
    parameters.max_depth_m <= parameters.min_depth_m || parameters.pixel_stride <= 0 ||
    parameters.max_points == 0U || parameters.max_disparity_gradient_px_per_pixel <= 0.0 ||
    parameters.min_texture_gradient < 0.0 || parameters.texture_window_radius_px < 0)
  {
    return result;
  }

  const cv::Mat gray_left = ToGray(left);
  const cv::Mat gray_right = ToGray(right);
  const int num_disparities = std::max(16, ((gray_left.cols / 8 + 15) / 16) * 16);
  const int block_size = 5;
  auto matcher = cv::StereoSGBM::create(
    0, num_disparities, block_size, 8 * block_size * block_size,
    32 * block_size * block_size, 1, 31, 10, 100, 32,
    cv::StereoSGBM::MODE_SGBM_3WAY);
  cv::Mat disparity;
  matcher->compute(gray_left, gray_right, disparity);

  cv::Mat gradient_x;
  cv::Mat gradient_y;
  cv::Mat texture_gradient;
  cv::Mat mean_texture_gradient;
  cv::Sobel(gray_left, gradient_x, CV_32F, 1, 0, 3);
  cv::Sobel(gray_left, gradient_y, CV_32F, 0, 1, 3);
  cv::magnitude(gradient_x, gradient_y, texture_gradient);
  const int texture_window_size = 2 * parameters.texture_window_radius_px + 1;
  cv::boxFilter(
    texture_gradient, mean_texture_gradient, CV_32F,
    cv::Size(texture_window_size, texture_window_size), cv::Point(-1, -1), true,
    cv::BORDER_REPLICATE);

  std::vector<geometry_msgs::msg::Point32> accepted;
  accepted.reserve(static_cast<std::size_t>(disparity.total()));
  for (int y = 0; y < disparity.rows; y += parameters.pixel_stride)
  {
    for (int x = 0; x < disparity.cols; x += parameters.pixel_stride)
    {
      const float disparity_px = static_cast<float>(disparity.at<std::int16_t>(y, x)) / 16.0F;
      if (!std::isfinite(disparity_px) || disparity_px <= 0.0F) {
        continue;
      }
      const double z = fx * baseline_m / static_cast<double>(disparity_px);
      if (!std::isfinite(z) || z < parameters.min_depth_m || z > parameters.max_depth_m) {
        continue;
      }
      ++result.raw_valid_points;
      if (mean_texture_gradient.at<float>(y, x) < parameters.min_texture_gradient) {
        ++result.texture_rejected_points;
        continue;
      }
      if (HasStrongDisparityDiscontinuity(
          disparity, x, y, disparity_px, parameters.max_disparity_gradient_px_per_pixel))
      {
        ++result.discontinuity_rejected_points;
        continue;
      }
      result.nearest_depth_m = std::min(result.nearest_depth_m, z);
      geometry_msgs::msg::Point32 point;
      point.x = static_cast<float>((static_cast<double>(x) - cx) * z / fx);
      point.y = static_cast<float>((static_cast<double>(y) - cy) * z / fy);
      point.z = static_cast<float>(z);
      accepted.push_back(point);
    }
  }
  const std::size_t output_count = std::min(parameters.max_points, accepted.size());
  result.points_k.reserve(output_count);
  for (std::size_t index = 0U; index < output_count; ++index) {
    const std::size_t source_index = output_count == accepted.size() ? index :
      std::min(accepted.size() - 1U,
      static_cast<std::size_t>(std::floor(
        (static_cast<double>(index) + 0.5) * static_cast<double>(accepted.size()) /
        static_cast<double>(output_count))));
    result.points_k.push_back(accepted[source_index]);
  }
  result.confidence = result.raw_valid_points == 0U ? 0.0 :
    static_cast<double>(accepted.size()) / static_cast<double>(result.raw_valid_points);
  const auto normal = EstimateFacadeNormal(result.points_k);
  result.normal_valid = normal.valid;
  result.normal_camera = normal.normal;
  result.normal_support = normal.support;
  result.normal_confidence = normal.confidence;
  return result;
}

}  // namespace orbslam3_ros2
