#include "visual-evidence-metrics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace orbslam3_ros2
{

double Quantile(std::vector<double> values, double probability)
{
    if (values.empty())
        return 0.0;
    std::sort(values.begin(), values.end());
    probability = std::max(0.0, std::min(1.0, probability));
    const double index = probability * static_cast<double>(values.size() - 1);
    const std::size_t lower = static_cast<std::size_t>(std::floor(index));
    const std::size_t upper = static_cast<std::size_t>(std::ceil(index));
    const double fraction = index - static_cast<double>(lower);
    return values[lower] * (1.0 - fraction) + values[upper] * fraction;
}

VisualEvidenceMetrics ComputeVisualEvidenceMetrics(
    const std::vector<cv::KeyPoint>& keypoints,
    const std::vector<unsigned char>& has_map_point,
    const std::vector<unsigned char>& is_outlier,
    const std::vector<float>& right_x,
    const std::vector<float>& depth,
    int image_width,
    int image_height,
    int grid_columns,
    int grid_rows)
{
    VisualEvidenceMetrics result;
    result.feature_count = keypoints.size();
    std::vector<double> depths;
    std::vector<double> disparities;
    std::vector<unsigned char> occupied(
        static_cast<std::size_t>(std::max(0, grid_columns * grid_rows)), 0U);

    for (std::size_t i = 0; i < keypoints.size(); ++i)
    {
        const bool matched = i < has_map_point.size() && has_map_point[i] != 0U;
        const bool outlier = i < is_outlier.size() && is_outlier[i] != 0U;
        if (matched)
            ++result.map_point_match_count;
        if (!matched || outlier)
            continue;
        ++result.tracking_inlier_count;

        if (image_width > 0 && image_height > 0 &&
            grid_columns > 0 && grid_rows > 0)
        {
            const int column = std::min(
                grid_columns - 1,
                std::max(0, static_cast<int>(
                    keypoints[i].pt.x * grid_columns / image_width)));
            const int row = std::min(
                grid_rows - 1,
                std::max(0, static_cast<int>(
                    keypoints[i].pt.y * grid_rows / image_height)));
            occupied[static_cast<std::size_t>(row * grid_columns + column)] = 1U;
        }

        if (i < depth.size() && std::isfinite(depth[i]) && depth[i] > 0.0f)
        {
            depths.push_back(depth[i]);
            if (i < right_x.size() && std::isfinite(right_x[i]) && right_x[i] >= 0.0f)
            {
                const double disparity = keypoints[i].pt.x - right_x[i];
                if (std::isfinite(disparity) && disparity > 0.0)
                    disparities.push_back(disparity);
            }
        }
    }

    result.inlier_ratio = result.map_point_match_count == 0 ? 0.0 :
        static_cast<double>(result.tracking_inlier_count) /
        static_cast<double>(result.map_point_match_count);
    result.valid_stereo_depth_count = depths.size();
    result.stereo_depth_ratio = result.tracking_inlier_count == 0 ? 0.0 :
        static_cast<double>(depths.size()) /
        static_cast<double>(result.tracking_inlier_count);
    result.depth_p25 = Quantile(depths, 0.25);
    result.depth_p50 = Quantile(depths, 0.50);
    result.depth_p75 = Quantile(depths, 0.75);
    result.depth_p90 = Quantile(depths, 0.90);
    result.disparity_p10 = Quantile(disparities, 0.10);
    result.disparity_p25 = Quantile(disparities, 0.25);
    result.disparity_p50 = Quantile(disparities, 0.50);
    result.disparity_p75 = Quantile(disparities, 0.75);
    result.disparity_p90 = Quantile(disparities, 0.90);
    result.grid_coverage_ratio = occupied.empty() ? 0.0 :
        static_cast<double>(std::count(occupied.begin(), occupied.end(), 1U)) /
        static_cast<double>(occupied.size());
    return result;
}

}  // namespace orbslam3_ros2
