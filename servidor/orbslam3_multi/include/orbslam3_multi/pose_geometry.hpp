#pragma once

#include "geometry_msgs/msg/pose.hpp"

#include "orbslam3_multi/fiducial_optimization_task.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>

namespace orbslam3_multi
{

inline bool PoseToIsometry(
  const geometry_msgs::msg::Pose & pose, Eigen::Isometry3d * transform)
{
  if (transform == nullptr) {
    return false;
  }
  const Eigen::Quaterniond quaternion(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
    !std::isfinite(pose.position.z) || !std::isfinite(quaternion.w()) ||
    !std::isfinite(quaternion.x()) || !std::isfinite(quaternion.y()) ||
    !std::isfinite(quaternion.z()) || quaternion.norm() < 1e-9)
  {
    return false;
  }
  *transform = Eigen::Isometry3d::Identity();
  transform->linear() = quaternion.normalized().toRotationMatrix();
  transform->translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  return transform->matrix().allFinite();
}

inline geometry_msgs::msg::Pose IsometryToPose(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  const Eigen::Quaterniond quaternion(transform.linear());
  const auto normalized = quaternion.normalized();
  pose.orientation.x = normalized.x();
  pose.orientation.y = normalized.y();
  pose.orientation.z = normalized.z();
  pose.orientation.w = normalized.w();
  return pose;
}

inline Eigen::Isometry3d InterpolateIsometry(
  const Eigen::Isometry3d & from, const Eigen::Isometry3d & to, double alpha)
{
  alpha = std::clamp(alpha, 0.0, 1.0);
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() =
    (1.0 - alpha) * from.translation() + alpha * to.translation();
  const Eigen::Quaterniond from_q(from.linear());
  const Eigen::Quaterniond to_q(to.linear());
  result.linear() = from_q.normalized().slerp(alpha, to_q.normalized()).toRotationMatrix();
  return result;
}

inline double RotationErrorRad(
  const Eigen::Isometry3d & estimated, const Eigen::Isometry3d & target)
{
  const Eigen::AngleAxisd error(target.linear() * estimated.linear().transpose());
  return std::abs(error.angle());
}

inline double YawFromRotation(const Eigen::Matrix3d & rotation)
{
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

inline double NormalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline bool PosesNear(
  const geometry_msgs::msg::Pose & lhs, const geometry_msgs::msg::Pose & rhs,
  double translation_tolerance = 1e-9, double rotation_tolerance = 1e-9)
{
  Eigen::Isometry3d lhs_transform;
  Eigen::Isometry3d rhs_transform;
  return PoseToIsometry(lhs, &lhs_transform) && PoseToIsometry(rhs, &rhs_transform) &&
         (lhs_transform.translation() - rhs_transform.translation()).norm() <=
         translation_tolerance &&
         RotationErrorRad(lhs_transform, rhs_transform) <= rotation_tolerance;
}

inline FiducialError ComputeFiducialError(
  const geometry_msgs::msg::Pose & estimated_pose,
  const geometry_msgs::msg::Pose & target_pose)
{
  FiducialError error;
  Eigen::Isometry3d estimated;
  Eigen::Isometry3d target;
  if (!PoseToIsometry(estimated_pose, &estimated) || !PoseToIsometry(target_pose, &target)) {
    error.translation_m = std::numeric_limits<double>::infinity();
    error.rotation_rad = std::numeric_limits<double>::infinity();
    error.yaw_rad = std::numeric_limits<double>::infinity();
    return error;
  }
  error.translation_m = (target.translation() - estimated.translation()).norm();
  error.rotation_rad = RotationErrorRad(estimated, target);
  error.yaw_rad = std::abs(NormalizeAngle(
    YawFromRotation(target.linear()) - YawFromRotation(estimated.linear())));
  return error;
}

}  // namespace orbslam3_multi
