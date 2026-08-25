#include "../src/stereo/fiducial-detector.hpp"

#include <gtest/gtest.h>
#include <opencv2/aruco.hpp>

#include <cmath>

namespace
{

cv::Mat MarkerImage(int tag_id)
{
    cv::Mat marker;
    cv::aruco::drawMarker(
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11),
        tag_id, 200, marker, 1);
    cv::Mat image(480, 640, CV_8UC1, cv::Scalar(255));
    marker.copyTo(image(cv::Rect(220, 140, 200, 200)));
    return image;
}

orbslam3_ros2::FiducialCameraModel Camera()
{
    orbslam3_ros2::FiducialCameraModel camera;
    camera.camera_matrix = (
        cv::Mat_<double>(3, 3) <<
            500.0, 0.0, 320.0,
            0.0, 500.0, 240.0,
            0.0, 0.0, 1.0);
    camera.distortion = cv::Mat::zeros(1, 5, CV_64F);
    camera.width = 640;
    camera.height = 480;
    camera.rectified = true;
    return camera;
}

orbslam3_ros2::FiducialDetectorConfig Config(int tag_id, double size_m)
{
    orbslam3_ros2::FiducialDetectorConfig config;
    config.family = "APRILTAG_36H11";
    config.corner_refinement = "SUBPIX";
    config.pose_solver = "IPPE_SQUARE";
    config.max_reprojection_error_px = 3.0;
    config.tag_sizes_m.emplace(tag_id, size_m);
    return config;
}

TEST(FiducialDetectorTest, DetectsConfiguredAprilTagWithFinitePose)
{
    orbslam3_ros2::FiducialDetector detector;
    detector.Configure(Config(7, 0.20));
    const auto result = detector.Detect(MarkerImage(7), Camera());

    ASSERT_EQ(result.decoded_tags.size(), 1U);
    const auto& detection = result.decoded_tags.front();
    EXPECT_EQ(detection.tag_id, 7);
    EXPECT_TRUE(detection.valid) << detection.rejection_reason;
    EXPECT_TRUE(std::isfinite(detection.translation_vector[2]));
    EXPECT_GT(detection.translation_vector[2], 0.0);
    EXPECT_LE(detection.reprojection_error_px, 3.0);
    EXPECT_GE(detection.quality, 0.0);
    EXPECT_LE(detection.quality, 1.0);
}

TEST(FiducialDetectorTest, KeepsUnknownDecodedTagAsRejected)
{
    orbslam3_ros2::FiducialDetector detector;
    detector.Configure(Config(8, 0.20));
    const auto result = detector.Detect(MarkerImage(7), Camera());

    ASSERT_EQ(result.decoded_tags.size(), 1U);
    EXPECT_EQ(result.decoded_tags.front().tag_id, 7);
    EXPECT_FALSE(result.decoded_tags.front().valid);
    EXPECT_EQ(
        result.decoded_tags.front().rejection_reason,
        "unknown_tag_id");
}

TEST(FiducialDetectorTest, UsesConfiguredPhysicalSizeForPoseScale)
{
    orbslam3_ros2::FiducialDetector small_detector;
    orbslam3_ros2::FiducialDetector large_detector;
    small_detector.Configure(Config(7, 0.10));
    large_detector.Configure(Config(7, 0.20));

    const auto small = small_detector.Detect(MarkerImage(7), Camera());
    const auto large = large_detector.Detect(MarkerImage(7), Camera());
    ASSERT_TRUE(small.decoded_tags.front().valid);
    ASSERT_TRUE(large.decoded_tags.front().valid);
    EXPECT_NEAR(
        large.decoded_tags.front().translation_vector[2],
        2.0 * small.decoded_tags.front().translation_vector[2],
        1.0e-3);
}

}  // namespace
