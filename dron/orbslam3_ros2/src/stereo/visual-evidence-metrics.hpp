#ifndef ORBSLAM3_ROS2_VISUAL_EVIDENCE_METRICS_HPP
#define ORBSLAM3_ROS2_VISUAL_EVIDENCE_METRICS_HPP

#include <cstddef>
#include <vector>

#include <opencv2/features2d.hpp>

namespace orbslam3_ros2
{

struct VisualEvidenceMetrics
{
    std::size_t feature_count = 0;
    std::size_t map_point_match_count = 0;
    std::size_t tracking_inlier_count = 0;
    double inlier_ratio = 0.0;
    std::size_t valid_stereo_depth_count = 0;
    double stereo_depth_ratio = 0.0;
    double depth_p25 = 0.0;
    double depth_p50 = 0.0;
    double depth_p75 = 0.0;
    double depth_p90 = 0.0;
    double disparity_p10 = 0.0;
    double disparity_p25 = 0.0;
    double disparity_p50 = 0.0;
    double disparity_p75 = 0.0;
    double disparity_p90 = 0.0;
    double grid_coverage_ratio = 0.0;
};

double Quantile(std::vector<double> values, double probability);

VisualEvidenceMetrics ComputeVisualEvidenceMetrics(
    const std::vector<cv::KeyPoint>& keypoints,
    const std::vector<unsigned char>& has_map_point,
    const std::vector<unsigned char>& is_outlier,
    const std::vector<float>& right_x,
    const std::vector<float>& depth,
    int image_width,
    int image_height,
    int grid_columns = 4,
    int grid_rows = 3);

}  // namespace orbslam3_ros2

#endif
