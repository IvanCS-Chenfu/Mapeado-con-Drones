#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "dron_individual/action/tray_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mission_msgs/msg/fiducial_primary_observation.hpp"
#include "orbslam3_msgs/msg/navigation_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
double StampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

double YawFromPose(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double FeedbackValue(const std::vector<double> & values, std::size_t index)
{
  return index < values.size() ? values[index] : NAN;
}
}  // namespace

class Chapter7OrbTrajectoryRecorder final : public rclcpp::Node
{
public:
  Chapter7OrbTrajectoryRecorder()
  : Node("chapter7_orb_trajectory_recorder")
  {
    const auto output_dir = declare_parameter<std::string>("output_dir", "/tmp/chapter7_orb_trajectory");
    const auto drone_namespace = declare_parameter<std::string>("drone_namespace", "dron_1");
    std::filesystem::create_directories(output_dir);

    navigation_.open(std::filesystem::path(output_dir) / "navigation_state.csv");
    gt_pose_.open(std::filesystem::path(output_dir) / "gt_pose.csv");
    gt_velocity_.open(std::filesystem::path(output_dir) / "gt_velocity.csv");
    trajectory_.open(std::filesystem::path(output_dir) / "trajectory_feedback.csv");
    fiducials_.open(std::filesystem::path(output_dir) / "fiducial_primary_observations.csv");
    if (!navigation_ || !gt_pose_ || !gt_velocity_ || !trajectory_ || !fiducials_) {
      throw std::runtime_error("No se pudieron abrir los CSV de trayectoria ORB del Capitulo 7");
    }

    navigation_ << "stamp_sec,receive_sec,sample_sequence,map_epoch,tracking_state,pose_source,global_status,pose_revision,local_valid,local_continuity_valid,global_valid,velocity_valid,reference_keyframe_valid,reference_keyframe_id,o_x,o_y,o_z,o_yaw,w_x,w_y,w_z,w_yaw,vel_x,vel_y,vel_z,omega_x,omega_y,omega_z\n";
    gt_pose_ << "stamp_sec,receive_sec,x,y,z,yaw\n";
    gt_velocity_ << "stamp_sec,receive_sec,vel_x,vel_y,vel_z,omega_x,omega_y,omega_z\n";
    trajectory_ << "receive_sec,t_act,trajectory_id,piece,x_pos,x_vel,x_acc,x_jerk,x_ratio,y_pos,y_vel,y_acc,y_jerk,y_ratio,z_pos,z_vel,z_acc,z_jerk,z_ratio,yaw_pos,yaw_vel,yaw_acc,yaw_jerk,yaw_ratio\n";
    fiducials_ << "stamp_sec,receive_sec,drone_id,map_epoch,local_keyframe_id,object_id,visit_id,quality,distance_m\n";
    navigation_ << std::fixed << std::setprecision(9);
    gt_pose_ << std::fixed << std::setprecision(9);
    gt_velocity_ << std::fixed << std::setprecision(9);
    trajectory_ << std::fixed << std::setprecision(9);
    fiducials_ << std::fixed << std::setprecision(9);

    const std::string prefix = "/" + drone_namespace;
    const auto qos = rclcpp::QoS(100).best_effort();
    navigation_subscription_ = create_subscription<orbslam3_msgs::msg::NavigationState>(
      prefix + "/orbslam/navigation_state", rclcpp::QoS(100).reliable(),
      std::bind(&Chapter7OrbTrajectoryRecorder::OnNavigation, this, std::placeholders::_1));
    gt_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      prefix + "/sensor/GT/pose", qos,
      std::bind(&Chapter7OrbTrajectoryRecorder::OnGtPose, this, std::placeholders::_1));
    gt_velocity_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      prefix + "/sensor/GT/vel", qos,
      std::bind(&Chapter7OrbTrajectoryRecorder::OnGtVelocity, this, std::placeholders::_1));
    trajectory_subscription_ = create_subscription<dron_individual::action::TrayAction_FeedbackMessage>(
      prefix + "/AccionTrayectoria/_action/feedback", qos,
      std::bind(&Chapter7OrbTrajectoryRecorder::OnTrajectory, this, std::placeholders::_1));
    fiducial_subscription_ = create_subscription<mission_msgs::msg::FiducialPrimaryObservation>(
      "/mission/fiducial_primary_observations", rclcpp::QoS(64).reliable(),
      std::bind(&Chapter7OrbTrajectoryRecorder::OnFiducial, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "[C7-ORB-TRAJECTORY-RECORDER-READY] drone=%s output_dir=%s",
      drone_namespace.c_str(), output_dir.c_str());
  }

private:
  double ReceiveTime() const
  {
    return get_clock()->now().seconds();
  }

  void OnNavigation(const orbslam3_msgs::msg::NavigationState::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    navigation_ << StampSeconds(message->header.stamp) << ',' << ReceiveTime() << ','
                << message->sample_sequence << ',' << message->map_epoch << ','
                << static_cast<int>(message->tracking_state) << ','
                << static_cast<unsigned>(message->pose_source) << ','
                << static_cast<unsigned>(message->global_status) << ',' << message->pose_revision << ','
                << static_cast<int>(message->local_valid) << ','
                << static_cast<int>(message->local_continuity_valid) << ','
                << static_cast<int>(message->global_valid) << ','
                << static_cast<int>(message->velocity_valid) << ','
                << static_cast<int>(message->reference_keyframe_valid) << ','
                << message->reference_keyframe_id << ','
                << message->o_t_body.position.x << ',' << message->o_t_body.position.y << ','
                << message->o_t_body.position.z << ',' << YawFromPose(message->o_t_body) << ','
                << message->w_t_body.position.x << ',' << message->w_t_body.position.y << ','
                << message->w_t_body.position.z << ',' << YawFromPose(message->w_t_body) << ','
                << message->velocity.linear.x << ',' << message->velocity.linear.y << ','
                << message->velocity.linear.z << ',' << message->velocity.angular.x << ','
                << message->velocity.angular.y << ',' << message->velocity.angular.z << '\n';
    navigation_.flush();
  }

  void OnGtPose(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    gt_pose_ << StampSeconds(message->header.stamp) << ',' << ReceiveTime() << ','
             << message->pose.position.x << ',' << message->pose.position.y << ','
             << message->pose.position.z << ',' << YawFromPose(message->pose) << '\n';
    gt_pose_.flush();
  }

  void OnGtVelocity(const geometry_msgs::msg::TwistStamped::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto & twist = message->twist;
    gt_velocity_ << StampSeconds(message->header.stamp) << ',' << ReceiveTime() << ','
                 << twist.linear.x << ',' << twist.linear.y << ',' << twist.linear.z << ','
                 << twist.angular.x << ',' << twist.angular.y << ',' << twist.angular.z << '\n';
    gt_velocity_.flush();
  }

  void OnFiducial(const mission_msgs::msg::FiducialPrimaryObservation::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    fiducials_ << StampSeconds(message->header.stamp) << ',' << ReceiveTime() << ','
               << message->drone_id << ',' << message->map_epoch << ','
               << message->local_keyframe_id << ',' << message->object_id << ','
               << message->visit_id << ',' << message->quality << ',' << message->distance_m << '\n';
    fiducials_.flush();
  }

  void OnTrajectory(const dron_individual::action::TrayAction_FeedbackMessage::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto & feedback = message->feedback;
    trajectory_ << ReceiveTime() << ',' << feedback.t_act << ',' << '"' << feedback.trajectory_id
                << '"' << ',' << feedback.diagnostic_piece_index;
    for (const auto * axis : {&feedback.x, &feedback.y, &feedback.z, &feedback.yaw}) {
      for (std::size_t index = 0; index < 5U; ++index) {
        trajectory_ << ',' << FeedbackValue(axis->data, index);
      }
    }
    trajectory_ << '\n';
    trajectory_.flush();
  }

  std::mutex mutex_;
  std::ofstream navigation_;
  std::ofstream gt_pose_;
  std::ofstream gt_velocity_;
  std::ofstream trajectory_;
  std::ofstream fiducials_;
  rclcpp::Subscription<orbslam3_msgs::msg::NavigationState>::SharedPtr navigation_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gt_pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr gt_velocity_subscription_;
  rclcpp::Subscription<dron_individual::action::TrayAction_FeedbackMessage>::SharedPtr trajectory_subscription_;
  rclcpp::Subscription<mission_msgs::msg::FiducialPrimaryObservation>::SharedPtr fiducial_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Chapter7OrbTrajectoryRecorder>());
  rclcpp::shutdown();
  return 0;
}
