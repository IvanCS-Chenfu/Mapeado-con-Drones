#include <gtest/gtest.h>

#include "visual-evidence-metrics.hpp"

namespace
{

TEST(VisualEvidenceMetrics, EmptyInputHasFiniteZeros)
{
    const auto metrics = orbslam3_ros2::ComputeVisualEvidenceMetrics(
        {}, {}, {}, {}, {}, 640, 480);
    EXPECT_EQ(metrics.feature_count, 0U);
    EXPECT_DOUBLE_EQ(metrics.inlier_ratio, 0.0);
    EXPECT_DOUBLE_EQ(metrics.stereo_depth_ratio, 0.0);
    EXPECT_DOUBLE_EQ(metrics.grid_coverage_ratio, 0.0);
}

TEST(VisualEvidenceMetrics, UsesOnlyTrackingInliersForGeometry)
{
    const std::vector<cv::KeyPoint> keys = {
        cv::KeyPoint(10.0f, 10.0f, 1.0f),
        cv::KeyPoint(90.0f, 10.0f, 1.0f),
        cv::KeyPoint(10.0f, 90.0f, 1.0f),
        cv::KeyPoint(90.0f, 90.0f, 1.0f)};
    const auto metrics = orbslam3_ros2::ComputeVisualEvidenceMetrics(
        keys, {1, 1, 1, 0}, {0, 1, 0, 0},
        {8.0f, 80.0f, 6.0f, -1.0f}, {2.0f, 3.0f, 4.0f, -1.0f},
        100, 100, 2, 2);
    EXPECT_EQ(metrics.map_point_match_count, 3U);
    EXPECT_EQ(metrics.tracking_inlier_count, 2U);
    EXPECT_NEAR(metrics.inlier_ratio, 2.0 / 3.0, 1e-12);
    EXPECT_EQ(metrics.valid_stereo_depth_count, 2U);
    EXPECT_DOUBLE_EQ(metrics.stereo_depth_ratio, 1.0);
    EXPECT_DOUBLE_EQ(metrics.grid_coverage_ratio, 0.5);
    EXPECT_DOUBLE_EQ(metrics.depth_p50, 3.0);
    EXPECT_DOUBLE_EQ(metrics.disparity_p50, 3.0);
}

TEST(VisualEvidenceMetrics, QuantilesInterpolateAndClamp)
{
    EXPECT_DOUBLE_EQ(orbslam3_ros2::Quantile({1.0, 3.0}, 0.5), 2.0);
    EXPECT_DOUBLE_EQ(orbslam3_ros2::Quantile({3.0, 1.0}, -1.0), 1.0);
    EXPECT_DOUBLE_EQ(orbslam3_ros2::Quantile({3.0, 1.0}, 2.0), 3.0);
}

}  // namespace
