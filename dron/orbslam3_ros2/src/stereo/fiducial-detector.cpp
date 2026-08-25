#include "fiducial-detector.hpp"

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace orbslam3_ros2
{
namespace
{

using Clock = std::chrono::steady_clock;

double ElapsedMs(const Clock::time_point& begin, const Clock::time_point& end)
{
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

bool IsFinite(const cv::Vec3d& value)
{
    return std::isfinite(value[0]) && std::isfinite(value[1]) &&
           std::isfinite(value[2]);
}

std::vector<cv::Point3f> MarkerObjectPoints(double size_m)
{
    const float half = static_cast<float>(size_m * 0.5);
    return {
        {-half, half, 0.0f},
        {half, half, 0.0f},
        {half, -half, 0.0f},
        {-half, -half, 0.0f},
    };
}

double ReprojectionError(
    const std::vector<cv::Point3f>& object_points,
    const std::vector<cv::Point2f>& image_points,
    const cv::Vec3d& rvec,
    const cv::Vec3d& tvec,
    const FiducialCameraModel& camera)
{
    std::vector<cv::Point2f> projected;
    cv::projectPoints(
        object_points,
        rvec,
        tvec,
        camera.camera_matrix,
        camera.distortion,
        projected);
    double squared_sum = 0.0;
    for (std::size_t i = 0; i < image_points.size(); ++i)
    {
        const cv::Point2f delta = projected[i] - image_points[i];
        squared_sum += delta.dot(delta);
    }
    return std::sqrt(squared_sum / static_cast<double>(image_points.size()));
}

bool PoseGeometryIsValid(
    const std::vector<cv::Point3f>& object_points,
    const cv::Vec3d& rvec,
    const cv::Vec3d& tvec)
{
    if (!IsFinite(rvec) || !IsFinite(tvec) || tvec[2] <= 0.0)
    {
        return false;
    }
    cv::Mat rotation;
    cv::Rodrigues(rvec, rotation);
    const double determinant = cv::determinant(rotation);
    if (!std::isfinite(determinant) || std::abs(determinant - 1.0) > 1.0e-3)
    {
        return false;
    }
    const cv::Matx33d rotation_matrix(
        rotation.at<double>(0, 0), rotation.at<double>(0, 1),
        rotation.at<double>(0, 2), rotation.at<double>(1, 0),
        rotation.at<double>(1, 1), rotation.at<double>(1, 2),
        rotation.at<double>(2, 0), rotation.at<double>(2, 1),
        rotation.at<double>(2, 2));
    for (const auto& point : object_points)
    {
        const cv::Vec3d object_point(point.x, point.y, point.z);
        const cv::Vec3d camera_point = rotation_matrix * object_point + tvec;
        if (camera_point[2] <= 0.0)
        {
            return false;
        }
    }
    return true;
}

}  // namespace

void FiducialDetector::Configure(const FiducialDetectorConfig& config)
{
    if (config.family != "APRILTAG_36H11" ||
        config.corner_refinement != "SUBPIX" ||
        config.pose_solver != "IPPE_SQUARE" ||
        !std::isfinite(config.max_reprojection_error_px) ||
        config.max_reprojection_error_px <= 0.0)
    {
        throw std::invalid_argument("configuracion fiducial no soportada");
    }
    for (const auto& tag : config.tag_sizes_m)
    {
        if (tag.first < 0 || !std::isfinite(tag.second) || tag.second <= 0.0)
        {
            throw std::invalid_argument("tag_id o size_m invalido");
        }
    }
    config_ = config;
    configured_ = true;
}

FiducialDetectionResult FiducialDetector::Detect(
    const cv::Mat& image,
    const FiducialCameraModel& camera) const
{
    if (!configured_)
    {
        throw std::logic_error("detector fiducial no configurado");
    }
    if (image.empty() || camera.camera_matrix.rows != 3 ||
        camera.camera_matrix.cols != 3 || image.cols != camera.width ||
        image.rows != camera.height)
    {
        throw std::invalid_argument("imagen y calibracion efectiva incompatibles");
    }

    FiducialDetectionResult result;
    const auto total_begin = Clock::now();
    cv::Mat gray;
    if (image.channels() == 1)
    {
        gray = image;
    }
    else
    {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }

    const auto dictionary = cv::aruco::getPredefinedDictionary(
        cv::aruco::DICT_APRILTAG_36h11);
    const auto parameters = cv::aruco::DetectorParameters::create();
    parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<std::vector<cv::Point2f>> rejected;
    const auto detection_begin = Clock::now();
    cv::aruco::detectMarkers(
        gray, dictionary, corners, ids, parameters, rejected);
    const auto detection_end = Clock::now();
    result.detection_ms = ElapsedMs(detection_begin, detection_end);
    result.undecoded_candidates = rejected.size();

    const auto pose_begin = Clock::now();
    result.decoded_tags.reserve(ids.size());
    for (std::size_t index = 0; index < ids.size(); ++index)
    {
        FiducialDetection detection;
        detection.tag_id = ids[index];
        detection.corners = corners[index];
        detection.marker_area_px2 = std::abs(cv::contourArea(corners[index]));
        const auto size_it = config_.tag_sizes_m.find(ids[index]);
        if (size_it == config_.tag_sizes_m.end())
        {
            detection.rejection_reason = "unknown_tag_id";
            result.decoded_tags.push_back(std::move(detection));
            continue;
        }

        const auto object_points = MarkerObjectPoints(size_it->second);
        std::vector<cv::Mat> rvecs;
        std::vector<cv::Mat> tvecs;
        cv::solvePnPGeneric(
            object_points,
            corners[index],
            camera.camera_matrix,
            camera.distortion,
            rvecs,
            tvecs,
            false,
            cv::SOLVEPNP_IPPE_SQUARE);

        double best_error = std::numeric_limits<double>::infinity();
        double second_error = std::numeric_limits<double>::infinity();
        cv::Vec3d best_rvec;
        cv::Vec3d best_tvec;
        for (std::size_t solution = 0; solution < rvecs.size(); ++solution)
        {
            cv::Mat rvec_matrix;
            cv::Mat tvec_matrix;
            rvecs[solution].reshape(1, 3).convertTo(rvec_matrix, CV_64F);
            tvecs[solution].reshape(1, 3).convertTo(tvec_matrix, CV_64F);
            const cv::Vec3d rvec(
                rvec_matrix.at<double>(0),
                rvec_matrix.at<double>(1),
                rvec_matrix.at<double>(2));
            const cv::Vec3d tvec(
                tvec_matrix.at<double>(0),
                tvec_matrix.at<double>(1),
                tvec_matrix.at<double>(2));
            if (!PoseGeometryIsValid(object_points, rvec, tvec))
            {
                continue;
            }
            const double error = ReprojectionError(
                object_points, corners[index], rvec, tvec, camera);
            if (!std::isfinite(error))
            {
                continue;
            }
            if (error < best_error)
            {
                second_error = best_error;
                best_error = error;
                best_rvec = rvec;
                best_tvec = tvec;
            }
            else if (error < second_error)
            {
                second_error = error;
            }
        }

        if (!std::isfinite(best_error))
        {
            detection.rejection_reason = "invalid_pose_geometry";
            result.decoded_tags.push_back(std::move(detection));
            continue;
        }
        detection.rotation_vector = best_rvec;
        detection.translation_vector = best_tvec;
        detection.reprojection_error_px = best_error;
        detection.pose_ambiguity_px = std::isfinite(second_error)
            ? second_error - best_error
            : 0.0;
        detection.quality = std::max(
            0.0,
            std::min(
                1.0,
                1.0 - best_error / config_.max_reprojection_error_px));
        if (best_error > config_.max_reprojection_error_px)
        {
            detection.rejection_reason = "reprojection_error";
        }
        else
        {
            detection.valid = true;
        }
        result.decoded_tags.push_back(std::move(detection));
    }
    const auto pose_end = Clock::now();
    result.pose_ms = ElapsedMs(pose_begin, pose_end);
    result.total_ms = ElapsedMs(total_begin, pose_end);
    return result;
}

}  // namespace orbslam3_ros2
