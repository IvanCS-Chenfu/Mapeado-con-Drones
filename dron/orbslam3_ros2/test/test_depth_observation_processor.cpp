#include "depth-observation-processor.hpp"

#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include <cmath>

TEST(DepthObservationProcessor, ProducesMetricEndpointsInsideConfiguredBand)
{
  cv::Mat left(80, 160, CV_8UC1);
  for (int row = 0; row < left.rows; ++row) {
    for (int col = 0; col < left.cols; ++col) {
      left.at<std::uint8_t>(row, col) = static_cast<std::uint8_t>((col * 7 + row * 3) % 255);
    }
  }
  cv::Mat right = cv::Mat::zeros(left.size(), left.type());
  left(cv::Rect(8, 0, left.cols - 8, left.rows)).copyTo(
    right(cv::Rect(0, 0, right.cols - 8, right.rows)));

  orbslam3_ros2::DepthObservationParameters parameters;
  parameters.min_depth_m = 1.0;
  parameters.max_depth_m = 5.0;
  parameters.pixel_stride = 4;
  parameters.max_points = 128U;
  const auto result = orbslam3_ros2::ComputeDepthObservation(
    left, right, 80.0, 80.0, 80.0, 40.0, 0.2, parameters);

  ASSERT_FALSE(result.points_k.empty());
  EXPECT_GT(result.raw_valid_points, 0U);
  EXPECT_GT(result.confidence, 0.0);
  EXPECT_LE(result.confidence, 1.0);
  EXPECT_TRUE(result.normal_valid);
  EXPECT_GE(result.normal_support, 12U);
  EXPECT_GE(result.normal_confidence, 0.7);
  EXPECT_LT(std::abs(result.normal_camera[0]), 0.25);
  EXPECT_GT(std::abs(result.normal_camera[2]), 0.9);
  for (const auto & point : result.points_k) {
    EXPECT_GE(point.z, 1.0F);
    EXPECT_LE(point.z, 5.0F);
  }
}

TEST(DepthObservationProcessor, RejectsGeometricCandidatesWithInsufficientTexture)
{
  cv::Mat left(80, 160, CV_8UC1);
  for (int row = 0; row < left.rows; ++row) {
    for (int col = 0; col < left.cols; ++col) {
      left.at<std::uint8_t>(row, col) = static_cast<std::uint8_t>((col * 7 + row * 3) % 255);
    }
  }
  cv::Mat right = cv::Mat::zeros(left.size(), left.type());
  left(cv::Rect(8, 0, left.cols - 8, left.rows)).copyTo(
    right(cv::Rect(0, 0, right.cols - 8, right.rows)));

  orbslam3_ros2::DepthObservationParameters parameters;
  parameters.min_depth_m = 1.0;
  parameters.max_depth_m = 5.0;
  parameters.pixel_stride = 4;
  parameters.max_points = 128U;
  parameters.min_texture_gradient = 10000.0;
  const auto result = orbslam3_ros2::ComputeDepthObservation(
    left, right, 80.0, 80.0, 80.0, 40.0, 0.2, parameters);

  EXPECT_GT(result.raw_valid_points, 0U);
  EXPECT_GT(result.texture_rejected_points, 0U);
  EXPECT_TRUE(result.points_k.empty());
  EXPECT_EQ(result.confidence, 0.0);
}

TEST(DepthObservationProcessor, KeepsFarStereoEvidenceForFreeRaysWithoutTextureGate)
{
  cv::Mat left(80, 160, CV_8UC1);
  for (int row = 0; row < left.rows; ++row) {
    for (int col = 0; col < left.cols; ++col) {
      left.at<std::uint8_t>(row, col) = static_cast<std::uint8_t>((col * 7 + row * 3) % 255);
    }
  }
  cv::Mat right = cv::Mat::zeros(left.size(), left.type());
  left(cv::Rect(2, 0, left.cols - 2, left.rows)).copyTo(
    right(cv::Rect(0, 0, right.cols - 2, right.rows)));

  orbslam3_ros2::DepthObservationParameters parameters;
  parameters.min_depth_m = 1.0;
  parameters.max_depth_m = 5.0;
  parameters.far_measurement_max_distance_m = 10.0;
  parameters.pixel_stride = 4;
  parameters.max_points = 128U;
  parameters.min_texture_gradient = 10000.0;
  const auto result = orbslam3_ros2::ComputeDepthObservation(
    left, right, 80.0, 80.0, 80.0, 40.0, 0.2, parameters);

  EXPECT_TRUE(result.points_k.empty());
  ASSERT_FALSE(result.far_free_points_k.empty());
  EXPECT_GT(result.far_valid_points, 0U);
  EXPECT_GT(result.confidence, 0.0);
  for (const auto & point : result.far_free_points_k) {
    EXPECT_GT(point.z, 5.0F);
    EXPECT_LE(point.z, 10.0F);
  }
}

TEST(DepthObservationProcessor, RejectsDisparityDiscontinuitiesBeforePublishing)
{
  cv::Mat left(80, 160, CV_8UC1);
  for (int row = 0; row < left.rows; ++row) {
    for (int col = 0; col < left.cols; ++col) {
      left.at<std::uint8_t>(row, col) = static_cast<std::uint8_t>((col * 7 + row * 3) % 255);
    }
  }
  cv::Mat right = cv::Mat::zeros(left.size(), left.type());
  left(cv::Rect(8, 0, left.cols - 8, left.rows)).copyTo(
    right(cv::Rect(0, 0, right.cols - 8, right.rows)));

  orbslam3_ros2::DepthObservationParameters parameters;
  parameters.min_depth_m = 1.0;
  parameters.max_depth_m = 5.0;
  parameters.pixel_stride = 4;
  parameters.max_points = 128U;
  parameters.max_disparity_gradient_px_per_pixel = 0.001;
  const auto result = orbslam3_ros2::ComputeDepthObservation(
    left, right, 80.0, 80.0, 80.0, 40.0, 0.2, parameters);

  EXPECT_GT(result.raw_valid_points, 0U);
  EXPECT_GT(result.discontinuity_rejected_points, 0U);
  EXPECT_LT(result.points_k.size(), result.raw_valid_points);
}

TEST(DepthObservationProcessor, RejectsInvalidCalibration)
{
  const cv::Mat image = cv::Mat::zeros(20, 20, CV_8UC1);
  const auto result = orbslam3_ros2::ComputeDepthObservation(
    image, image, 0.0, 1.0, 0.0, 0.0, 0.1, {});
  EXPECT_TRUE(result.points_k.empty());
  EXPECT_EQ(result.raw_valid_points, 0U);
  EXPECT_EQ(result.texture_rejected_points, 0U);
  EXPECT_EQ(result.discontinuity_rejected_points, 0U);
}
