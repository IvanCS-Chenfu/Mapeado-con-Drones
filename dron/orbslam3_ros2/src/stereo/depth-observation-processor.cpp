#include "depth-observation-processor.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <unordered_map>

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

struct DepthSample
{
  int x = 0;
  int y = 0;
  geometry_msgs::msg::Point32 point;
};

std::uint64_t PixelKey(int x, int y)
{
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(y)) << 32U) |
         static_cast<std::uint32_t>(x);
}

int CircularBinDistance(int first, int second, int bin_count)
{
  const int direct = std::abs(first - second);
  return std::min(direct, bin_count - direct);
}

PlaneEstimate EstimateFacadeNormal(
  const std::vector<DepthSample> & samples, int pixel_stride)
{
  constexpr int kBinCount = 18;
  constexpr int kClusterRadius = 1;
  constexpr double kMinimumHorizontalProjection = 0.6;
  constexpr std::size_t kMinimumLocalSupport = 12U;

  PlaneEstimate result;
  if (samples.size() < kMinimumLocalSupport || pixel_stride <= 0) {
    return result;
  }

  std::unordered_map<std::uint64_t, std::size_t> sample_by_pixel;
  sample_by_pixel.reserve(samples.size());
  for (std::size_t index = 0U; index < samples.size(); ++index) {
    sample_by_pixel.emplace(PixelKey(samples[index].x, samples[index].y), index);
  }

  std::vector<cv::Vec3d> local_normals;
  std::vector<int> normal_bins;
  std::array<std::size_t, kBinCount> histogram{};
  local_normals.reserve(samples.size());
  normal_bins.reserve(samples.size());
  for (const auto & sample : samples) {
    const auto right = sample_by_pixel.find(PixelKey(sample.x + pixel_stride, sample.y));
    const auto down = sample_by_pixel.find(PixelKey(sample.x, sample.y + pixel_stride));
    if (right == sample_by_pixel.end() || down == sample_by_pixel.end()) {
      continue;
    }
    const cv::Vec3d center{sample.point.x, sample.point.y, sample.point.z};
    const auto & right_point = samples[right->second].point;
    const auto & down_point = samples[down->second].point;
    const cv::Vec3d tangent_x{
      right_point.x - sample.point.x, right_point.y - sample.point.y,
      right_point.z - sample.point.z};
    const cv::Vec3d tangent_y{
      down_point.x - sample.point.x, down_point.y - sample.point.y,
      down_point.z - sample.point.z};
    cv::Vec3d normal = tangent_x.cross(tangent_y);
    const double norm = cv::norm(normal);
    if (!std::isfinite(norm) || norm <= 1e-9) {
      continue;
    }
    normal *= 1.0 / norm;
    if (normal.dot(center) > 0.0) {
      normal *= -1.0;
    }
    if (std::hypot(normal[0], normal[2]) < kMinimumHorizontalProjection) {
      continue;
    }
    double axial_angle = std::atan2(normal[0], normal[2]);
    while (axial_angle < 0.0) {
      axial_angle += CV_PI;
    }
    while (axial_angle >= CV_PI) {
      axial_angle -= CV_PI;
    }
    const int bin = std::min(
      kBinCount - 1,
      static_cast<int>(std::floor(axial_angle * static_cast<double>(kBinCount) / CV_PI)));
    local_normals.push_back(normal);
    normal_bins.push_back(bin);
    ++histogram[static_cast<std::size_t>(bin)];
  }
  if (local_normals.size() < kMinimumLocalSupport) {
    return result;
  }

  const auto cluster_support = [&histogram](int center) {
      std::size_t support = 0U;
      for (int offset = -kClusterRadius; offset <= kClusterRadius; ++offset) {
        const int bin = (center + offset + kBinCount) % kBinCount;
        support += histogram[static_cast<std::size_t>(bin)];
      }
      return support;
    };
  int primary_bin = 0;
  std::size_t primary_support = 0U;
  for (int bin = 0; bin < kBinCount; ++bin) {
    const auto support = cluster_support(bin);
    if (support > primary_support) {
      primary_bin = bin;
      primary_support = support;
    }
  }

  int secondary_bin = -1;
  std::size_t secondary_support = 0U;
  for (int bin = 0; bin < kBinCount; ++bin) {
    if (CircularBinDistance(bin, primary_bin, kBinCount) <= 2 * kClusterRadius) {
      continue;
    }
    const auto support = cluster_support(bin);
    if (support > secondary_support) {
      secondary_bin = bin;
      secondary_support = support;
    }
  }
  const double bin_angle = CV_PI / static_cast<double>(kBinCount);
  const double separation_deg = secondary_bin < 0 ? 0.0 :
    static_cast<double>(CircularBinDistance(primary_bin, secondary_bin, kBinCount)) *
    bin_angle * 180.0 / CV_PI;
  const bool use_corner = secondary_bin >= 0 && secondary_support >= kMinimumLocalSupport &&
    secondary_support * 5U >= local_normals.size() && separation_deg >= 20.0 &&
    separation_deg <= 140.0;

  const auto mean_for_cluster = [&](int center, std::size_t & support) {
      const double axis_angle = (static_cast<double>(center) + 0.5) * bin_angle;
      const cv::Vec3d axis{std::sin(axis_angle), 0.0, std::cos(axis_angle)};
      cv::Vec3d sum{0.0, 0.0, 0.0};
      support = 0U;
      for (std::size_t index = 0U; index < local_normals.size(); ++index) {
        if (CircularBinDistance(normal_bins[index], center, kBinCount) > kClusterRadius) {
          continue;
        }
        auto normal = local_normals[index];
        if (normal.dot(axis) < 0.0) {
          normal *= -1.0;
        }
        sum += normal;
        ++support;
      }
      const double norm = cv::norm(sum);
      return norm > 1e-9 ? sum * (1.0 / norm) : cv::Vec3d{0.0, 0.0, 0.0};
    };

  std::size_t selected_primary_support = 0U;
  auto selected_normal = mean_for_cluster(primary_bin, selected_primary_support);
  std::size_t selected_support = selected_primary_support;
  if (use_corner) {
    std::size_t selected_secondary_support = 0U;
    auto secondary_normal = mean_for_cluster(secondary_bin, selected_secondary_support);
    if (selected_normal.dot(secondary_normal) < 0.0) {
      secondary_normal *= -1.0;
    }
    const cv::Vec3d blended =
      static_cast<double>(selected_primary_support) * selected_normal +
      static_cast<double>(selected_secondary_support) * secondary_normal;
    const double blended_norm = cv::norm(blended);
    if (blended_norm > 1e-9) {
      selected_normal = blended * (1.0 / blended_norm);
      selected_support += selected_secondary_support;
    }
  }
  const double confidence = static_cast<double>(selected_support) /
    static_cast<double>(local_normals.size());
  result = {
    selected_support >= kMinimumLocalSupport && confidence >= 0.7,
    selected_normal, selected_support, confidence};
  return result;
}

}  // namespace

DepthObservationResult ComputeDepthObservation(
  const cv::Mat & left, const cv::Mat & right, double fx, double fy, double cx, double cy,
  double baseline_m, const DepthObservationParameters & parameters)
{
  DepthObservationResult result;
  if (left.empty() || right.empty() || left.size() != right.size() || fx <= 0.0 || fy <= 0.0 ||
    baseline_m <= 0.0 || parameters.min_depth_m <= 0.0 ||
    parameters.max_depth_m <= parameters.min_depth_m ||
    parameters.far_measurement_max_distance_m < parameters.max_depth_m ||
    parameters.pixel_stride <= 0 ||
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

  std::vector<DepthSample> accepted;
  std::vector<DepthSample> far_free;
  accepted.reserve(static_cast<std::size_t>(disparity.total()));
  far_free.reserve(static_cast<std::size_t>(disparity.total()));
  for (int y = 0; y < disparity.rows; y += parameters.pixel_stride)
  {
    for (int x = 0; x < disparity.cols; x += parameters.pixel_stride)
    {
      const float disparity_px = static_cast<float>(disparity.at<std::int16_t>(y, x)) / 16.0F;
      if (!std::isfinite(disparity_px) || disparity_px <= 0.0F) {
        continue;
      }
      const double z = fx * baseline_m / static_cast<double>(disparity_px);
      if (!std::isfinite(z) || z < parameters.min_depth_m ||
        z > parameters.far_measurement_max_distance_m)
      {
        continue;
      }
      ++result.raw_valid_points;
      if (HasStrongDisparityDiscontinuity(
          disparity, x, y, disparity_px, parameters.max_disparity_gradient_px_per_pixel))
      {
        ++result.discontinuity_rejected_points;
        continue;
      }
      geometry_msgs::msg::Point32 point;
      point.x = static_cast<float>((static_cast<double>(x) - cx) * z / fx);
      point.y = static_cast<float>((static_cast<double>(y) - cy) * z / fy);
      point.z = static_cast<float>(z);
      if (z > parameters.max_depth_m) {
        far_free.push_back({x, y, point});
        ++result.far_valid_points;
        continue;
      }
      if (mean_texture_gradient.at<float>(y, x) < parameters.min_texture_gradient) {
        ++result.texture_rejected_points;
        continue;
      }
      result.nearest_depth_m = std::min(result.nearest_depth_m, z);
      accepted.push_back({x, y, point});
    }
  }
  const auto append_uniform = [&parameters](
    const std::vector<DepthSample> & samples,
    std::vector<geometry_msgs::msg::Point32> * output)
    {
      if (output == nullptr || samples.empty()) {
        return;
      }
      const std::size_t output_count = std::min(parameters.max_points, samples.size());
      output->reserve(output_count);
      for (std::size_t index = 0U; index < output_count; ++index) {
        const std::size_t source_index = output_count == samples.size() ? index :
          std::min(samples.size() - 1U,
      static_cast<std::size_t>(std::floor(
          (static_cast<double>(index) + 0.5) * static_cast<double>(samples.size()) /
          static_cast<double>(output_count))));
        output->push_back(samples[source_index].point);
      }
    };
  append_uniform(accepted, &result.points_k);
  append_uniform(far_free, &result.far_free_points_k);
  result.confidence = result.raw_valid_points == 0U ? 0.0 :
    static_cast<double>(accepted.size() + far_free.size()) /
    static_cast<double>(result.raw_valid_points);
  const auto normal = EstimateFacadeNormal(accepted, parameters.pixel_stride);
  result.normal_valid = normal.valid;
  result.normal_camera = normal.normal;
  result.normal_support = normal.support;
  result.normal_confidence = normal.confidence;
  return result;
}

}  // namespace orbslam3_ros2
