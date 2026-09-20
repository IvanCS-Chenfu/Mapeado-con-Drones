#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "dron_individual/action/tray_action.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
double stamp_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}

class Phase45Recorder final : public rclcpp::Node
{
public:
  Phase45Recorder()
  : Node("fase45_recorder")
  {
    const auto output_dir = this->declare_parameter<std::string>(
      "output_dir", "/tmp/fase45");
    const auto drone_namespace = this->declare_parameter<std::string>(
      "drone_namespace", "dron_1");

    std::filesystem::create_directories(output_dir);
    gt_pose_.open(std::filesystem::path(output_dir) / "gt_pose.csv");
    gt_velocity_.open(std::filesystem::path(output_dir) / "gt_velocity.csv");
    gt_acceleration_.open(std::filesystem::path(output_dir) / "gt_acceleration.csv");
    trajectory_.open(std::filesystem::path(output_dir) / "trajectory_feedback.csv");
    if (!gt_pose_ || !gt_velocity_ || !gt_acceleration_ || !trajectory_) {
      throw std::runtime_error("No se pudieron abrir los CSV de la prueba 4.5");
    }

    gt_pose_ << "stamp_sec,receive_sec,x,y,z,qx,qy,qz,qw,yaw_rad\n";
    gt_velocity_ << "stamp_sec,receive_sec,linear_x,linear_y,linear_z,angular_x,angular_y,angular_z\n";
    gt_acceleration_ << "stamp_sec,receive_sec,linear_x,linear_y,linear_z,angular_x,angular_y,angular_z\n";
    trajectory_ << "receive_sec,t_act,trajectory_id,piece,x_pos,x_vel,x_acc,x_jerk,x_ratio,"
                    "y_pos,y_vel,y_acc,y_jerk,y_ratio,z_pos,z_vel,z_acc,z_jerk,z_ratio,"
                    "yaw_pos,yaw_vel,yaw_acc,yaw_jerk,yaw_ratio\n";
    set_precision();

    const std::string prefix = "/" + drone_namespace;
    const auto qos = rclcpp::QoS(100).best_effort();
    gt_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      prefix + "/sensor/GT/pose", qos,
      std::bind(&Phase45Recorder::on_pose, this, std::placeholders::_1));
    gt_velocity_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      prefix + "/sensor/GT/vel", qos,
      std::bind(&Phase45Recorder::on_velocity, this, std::placeholders::_1));
    gt_acceleration_subscription_ = create_subscription<geometry_msgs::msg::AccelStamped>(
      prefix + "/sensor/GT/acc", qos,
      std::bind(&Phase45Recorder::on_acceleration, this, std::placeholders::_1));
    trajectory_subscription_ = create_subscription<
      dron_individual::action::TrayAction_FeedbackMessage>(
      prefix + "/AccionTrayectoria/_action/feedback", qos,
      std::bind(&Phase45Recorder::on_trajectory, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "[F45-RECORDER-READY] drone=%s output_dir=%s topics=gt_pose,gt_velocity,gt_acceleration,trajectory_feedback",
      drone_namespace.c_str(), output_dir.c_str());
  }

private:
  void set_precision()
  {
    gt_pose_ << std::setprecision(12);
    gt_velocity_ << std::setprecision(12);
    gt_acceleration_ << std::setprecision(12);
    trajectory_ << std::setprecision(12);
  }

  double receive_time() const
  {
    return get_clock()->now().seconds();
  }

  void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    const auto & pose = message->pose;
    gt_pose_ << stamp_seconds(message->header.stamp) << ',' << receive_time() << ','
             << pose.position.x << ',' << pose.position.y << ',' << pose.position.z << ','
             << pose.orientation.x << ',' << pose.orientation.y << ','
             << pose.orientation.z << ',' << pose.orientation.w << ','
             << yaw_from_quaternion(pose.orientation) << '\n';
    gt_pose_.flush();
  }

  void on_velocity(const geometry_msgs::msg::TwistStamped::SharedPtr message)
  {
    const auto & twist = message->twist;
    gt_velocity_ << stamp_seconds(message->header.stamp) << ',' << receive_time() << ','
                 << twist.linear.x << ',' << twist.linear.y << ',' << twist.linear.z << ','
                 << twist.angular.x << ',' << twist.angular.y << ',' << twist.angular.z << '\n';
    gt_velocity_.flush();
  }

  void on_acceleration(const geometry_msgs::msg::AccelStamped::SharedPtr message)
  {
    const auto & acceleration = message->accel;
    gt_acceleration_ << stamp_seconds(message->header.stamp) << ',' << receive_time() << ','
                     << acceleration.linear.x << ',' << acceleration.linear.y << ','
                     << acceleration.linear.z << ',' << acceleration.angular.x << ','
                     << acceleration.angular.y << ',' << acceleration.angular.z << '\n';
    gt_acceleration_.flush();
  }

  static double feedback_value(
    const std::vector<double> & values, std::size_t index)
  {
    return index < values.size() ? values[index] : NAN;
  }

  void on_trajectory(
    const dron_individual::action::TrayAction_FeedbackMessage::SharedPtr message)
  {
    const auto & feedback = message->feedback;
    trajectory_ << receive_time() << ',' << feedback.t_act << ',' << feedback.trajectory_id << ','
                << feedback.diagnostic_piece_index;
    for (const auto * axis : {&feedback.x, &feedback.y, &feedback.z, &feedback.yaw}) {
      for (std::size_t index = 0; index < 5; ++index) {
        trajectory_ << ',' << feedback_value(axis->data, index);
      }
    }
    trajectory_ << '\n';
    trajectory_.flush();
  }

  std::ofstream gt_pose_;
  std::ofstream gt_velocity_;
  std::ofstream gt_acceleration_;
  std::ofstream trajectory_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gt_pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr gt_velocity_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::AccelStamped>::SharedPtr gt_acceleration_subscription_;
  rclcpp::Subscription<dron_individual::action::TrayAction_FeedbackMessage>::SharedPtr
    trajectory_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Phase45Recorder>());
  rclcpp::shutdown();
  return 0;
}
