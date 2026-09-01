#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

namespace gazebo
{
class PluginCameraPitch : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    ros_node_ = gazebo_ros::Node::Get(sdf);
    joint_name_ = sdf->Get<std::string>("joint_name", "stereo_pitch_joint").first;
    command_topic_ = sdf->Get<std::string>("command_topic", "camera_pitch/command").first;
    state_topic_ = sdf->Get<std::string>("state_topic", "camera_pitch/joint_states").first;
    enabled_ = sdf->Get<bool>("enabled", true).first;
    debug_fase_1_ = sdf->Get<bool>("debug_fase_1", false).first;
    lower_ = sdf->Get<double>("lower", -1.2217304764).first;
    upper_ = sdf->Get<double>("upper", 1.2217304764).first;
    max_velocity_ = sdf->Get<double>("max_velocity", 0.35).first;
    max_acceleration_ = sdf->Get<double>("max_acceleration", 0.70).first;
    max_effort_ = sdf->Get<double>("max_effort", 0.002).first;
    kp_ = sdf->Get<double>("kp", 0.01).first;
    ki_ = sdf->Get<double>("ki", 0.0).first;
    kd_ = sdf->Get<double>("kd", 0.004).first;
    integral_limit_ = sdf->Get<double>("integral_limit", 0.10).first;
    velocity_filter_tau_sec_ = std::max(
      0.0, sdf->Get<double>("velocity_filter_tau_sec", 0.05).first);
    publish_rate_ = sdf->Get<double>("publish_rate", 100.0).first;
    body_frame_ = sdf->Get<std::string>("body_frame", model_->GetName() + "/base_link").first;
    camera_frame_ = sdf->Get<std::string>(
      "camera_frame", model_->GetName() + "/camera_left_optical_frame").first;
    camera_x_ = sdf->Get<double>("camera_x", 0.10).first;
    camera_y_ = sdf->Get<double>("camera_y", 0.0285).first;
    camera_z_ = sdf->Get<double>("camera_z", 0.03).first;

    joint_ = model_->GetJoint(joint_name_);
    if (!joint_) {
      RCLCPP_FATAL(
        ros_node_->get_logger(), "[1J-PITCH-ERROR] joint '%s' not found", joint_name_.c_str());
      return;
    }

    state_pub_ = ros_node_->create_publisher<sensor_msgs::msg::JointState>(state_topic_, 20);
    command_sub_ = ros_node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      command_topic_, 10,
      [this](trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {OnCommand(*msg);});
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(ros_node_);
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&PluginCameraPitch::OnUpdate, this, std::placeholders::_1));

    if (debug_fase_1_) {
      RCLCPP_INFO(
        ros_node_->get_logger(),
        "[1J-PITCH-READY] joint=%s enabled=%s limits=(%.6f,%.6f) vmax=%.6f amax=%.6f",
        joint_name_.c_str(), enabled_ ? "true" : "false", lower_, upper_,
        max_velocity_, max_acceleration_);
    }
  }

private:
  void OnCommand(const trajectory_msgs::msg::JointTrajectory & msg)
  {
    if (!enabled_ || msg.points.empty()) {
      return;
    }
    const auto name_it = std::find(msg.joint_names.begin(), msg.joint_names.end(), joint_name_);
    if (name_it == msg.joint_names.end()) {
      RCLCPP_ERROR(
        ros_node_->get_logger(), "[1J-PITCH-COMMAND-REJECTED] missing_joint=%s",
        joint_name_.c_str());
      return;
    }
    const auto index = static_cast<std::size_t>(std::distance(msg.joint_names.begin(), name_it));
    if (index >= msg.points.front().positions.size()) {
      RCLCPP_ERROR(ros_node_->get_logger(), "[1J-PITCH-COMMAND-REJECTED] missing_position");
      return;
    }
    const double requested = msg.points.front().positions[index];
    if (!std::isfinite(requested)) {
      RCLCPP_ERROR(ros_node_->get_logger(), "[1J-PITCH-COMMAND-REJECTED] non_finite");
      return;
    }
    std::lock_guard<std::mutex> lock(command_mutex_);
    target_ = std::clamp(requested, lower_, upper_);
    if (requested != target_) {
      RCLCPP_WARN(
        ros_node_->get_logger(),
        "[1J-PITCH-COMMAND-SATURATED] requested=%.6f target=%.6f",
        requested, target_);
    } else if (debug_fase_1_) {
      RCLCPP_INFO(
        ros_node_->get_logger(), "[1J-PITCH-COMMAND] requested=%.6f target=%.6f",
        requested, target_);
    }
  }

  void OnUpdate(const common::UpdateInfo & info)
  {
    if (!joint_) {
      return;
    }
    const double now = info.simTime.Double();
    const double dt = last_update_time_ > 0.0 ? now - last_update_time_ : 0.0;
    last_update_time_ = now;
    double target;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      target = enabled_ ? target_ : 0.0;
    }
    const double position = joint_->Position(0);
    const double raw_velocity = joint_->GetVelocity(0);
    if (!std::isfinite(position) || !std::isfinite(raw_velocity)) {
      joint_->SetForce(0, 0.0);
      integral_error_ = 0.0;
      velocity_filter_initialized_ = false;
      RCLCPP_ERROR_THROTTLE(
        ros_node_->get_logger(), *ros_node_->get_clock(), 1000,
        "[1J-PITCH-NONFINITE] position=%.9f velocity=%.9f action=zero_torque",
        position, raw_velocity);
      return;
    }
    if (!velocity_filter_initialized_) {
      filtered_velocity_ = raw_velocity;
      velocity_filter_initialized_ = true;
    } else if (dt > 0.0) {
      const double alpha = velocity_filter_tau_sec_ > 0.0 ?
        std::clamp(dt / (velocity_filter_tau_sec_ + dt), 0.0, 1.0) : 1.0;
      filtered_velocity_ += alpha * (raw_velocity - filtered_velocity_);
    }
    if (dt > 0.0) {
      const double error_to_profile = target - profiled_target_;
      const double stopping_velocity = std::sqrt(
        std::max(0.0, 2.0 * max_acceleration_ * std::abs(error_to_profile)));
      const double desired_velocity = std::copysign(
        std::min(max_velocity_, stopping_velocity), error_to_profile);
      const double velocity_step = std::clamp(
        desired_velocity - profiled_velocity_,
        -max_acceleration_ * dt, max_acceleration_ * dt);
      profiled_velocity_ += velocity_step;
      profiled_target_ += profiled_velocity_ * dt;
      if ((target - profiled_target_) * error_to_profile < 0.0) {
        profiled_target_ = target;
        profiled_velocity_ = 0.0;
      }
    }
    const double position_error = profiled_target_ - position;
    const double candidate_integral = std::clamp(
      integral_error_ + position_error * dt, -integral_limit_, integral_limit_);
    const double unsaturated_effort =
      kp_ * position_error + ki_ * candidate_integral +
      kd_ * (profiled_velocity_ - filtered_velocity_);
    const double effort = std::clamp(unsaturated_effort, -max_effort_, max_effort_);
    if (effort == unsaturated_effort || effort * position_error < 0.0) {
      integral_error_ = candidate_integral;
    }
    joint_->SetForce(0, effort);

    if (publish_rate_ <= 0.0 || now - last_publish_time_ >= 1.0 / publish_rate_) {
      last_publish_time_ = now;
      PublishStateAndTf(
        info.simTime, position, filtered_velocity_, raw_velocity, effort, target);
    }
  }

  void PublishStateAndTf(
    const common::Time & sim_time, double position, double velocity,
    double raw_velocity, double effort, double target)
  {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(sim_time.sec);
    stamp.nanosec = static_cast<uint32_t>(sim_time.nsec);
    sensor_msgs::msg::JointState state;
    state.header.stamp = stamp;
    state.name = {joint_name_};
    state.position = {position};
    state.velocity = {velocity};
    state.effort = {effort};
    state_pub_->publish(state);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = body_frame_;
    transform.child_frame_id = camera_frame_;
    transform.transform.translation.x = camera_x_;
    transform.transform.translation.y = camera_y_;
    transform.transform.translation.z = camera_z_;
    // B_R_C(q) = Ry(q) * B_R_C(0), with B_R_C(0) = Rz(-90) Rx(-90).
    const ignition::math::Quaterniond rotation(0.0, position, 0.0);
    const ignition::math::Quaterniond optical(-M_PI_2, 0.0, -M_PI_2);
    const auto result = rotation * optical;
    transform.transform.rotation.x = result.X();
    transform.transform.rotation.y = result.Y();
    transform.transform.rotation.z = result.Z();
    transform.transform.rotation.w = result.W();
    tf_broadcaster_->sendTransform(transform);

    if (debug_fase_1_) {
      RCLCPP_INFO_THROTTLE(
        ros_node_->get_logger(), *ros_node_->get_clock(), 1000,
        "[1J-PITCH-STATE] position=%.6f velocity_filtered=%.6f velocity_raw=%.6f "
        "effort=%.6f target=%.6f",
        position, velocity, raw_velocity, effort, target);
    }
  }

  physics::ModelPtr model_;
  physics::JointPtr joint_;
  std::shared_ptr<rclcpp::Node> ros_node_;
  event::ConnectionPtr update_connection_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr command_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::mutex command_mutex_;
  std::string joint_name_;
  std::string command_topic_;
  std::string state_topic_;
  std::string body_frame_;
  std::string camera_frame_;
  bool enabled_{true};
  bool debug_fase_1_{false};
  double lower_{-1.2217304764};
  double upper_{1.2217304764};
  double max_velocity_{0.35};
  double max_acceleration_{0.70};
  double max_effort_{0.002};
  double kp_{0.01};
  double ki_{0.0};
  double kd_{0.004};
  double integral_limit_{0.10};
  double velocity_filter_tau_sec_{0.05};
  double publish_rate_{100.0};
  double camera_x_{0.10};
  double camera_y_{0.0285};
  double camera_z_{0.03};
  double target_{0.0};
  double profiled_target_{0.0};
  double profiled_velocity_{0.0};
  double integral_error_{0.0};
  double filtered_velocity_{0.0};
  bool velocity_filter_initialized_{false};
  double last_update_time_{0.0};
  double last_publish_time_{0.0};
};

GZ_REGISTER_MODEL_PLUGIN(PluginCameraPitch)
}  // namespace gazebo
