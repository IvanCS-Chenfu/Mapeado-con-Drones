#ifndef ORBSLAM3_ROS2_FIDUCIAL_DETECTOR_HPP
#define ORBSLAM3_ROS2_FIDUCIAL_DETECTOR_HPP

#include <opencv2/core.hpp>

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace orbslam3_ros2
{

struct FiducialDetectorConfig
{
    std::string family;
    std::string corner_refinement;
    std::string pose_solver;
    double max_reprojection_error_px = 0.0;
    std::unordered_map<int, double> tag_sizes_m;
};

struct FiducialCameraModel
{
    cv::Mat camera_matrix;
    cv::Mat distortion;
    int width = 0;
    int height = 0;
    bool rectified = false;
};

struct FiducialDetection
{
    int tag_id = -1;
    std::vector<cv::Point2f> corners;
    bool valid = false;
    std::string rejection_reason;
    cv::Vec3d rotation_vector;
    cv::Vec3d translation_vector;
    double reprojection_error_px = 0.0;
    double marker_area_px2 = 0.0;
    double quality = 0.0;
    double pose_ambiguity_px = 0.0;
};

struct FiducialDetectionResult
{
    std::vector<FiducialDetection> decoded_tags;
    std::size_t undecoded_candidates = 0;
    double detection_ms = 0.0;
    double pose_ms = 0.0;
    double total_ms = 0.0;
};

class FiducialDetector
{
public:
    void Configure(const FiducialDetectorConfig& config);
    FiducialDetectionResult Detect(
        const cv::Mat& image,
        const FiducialCameraModel& camera) const;

private:
    FiducialDetectorConfig config_;
    bool configured_ = false;
};

}  // namespace orbslam3_ros2

#endif
