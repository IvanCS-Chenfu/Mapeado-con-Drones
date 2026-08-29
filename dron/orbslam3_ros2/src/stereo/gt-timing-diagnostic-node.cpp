#include "navigation-state-estimator.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <orbslam3_msgs/msg/navigation_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sophus/se3.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

namespace
{

using NavigationState = orbslam3_msgs::msg::NavigationState;

double StampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + 1e-9 * static_cast<double>(stamp.nanosec);
}

Sophus::SE3f FromPose(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaternionf q(
    static_cast<float>(pose.orientation.w), static_cast<float>(pose.orientation.x),
    static_cast<float>(pose.orientation.y), static_cast<float>(pose.orientation.z));
  if (q.norm() < 1e-6f) {
    q = Eigen::Quaternionf::Identity();
  } else {
    q.normalize();
  }
  return Sophus::SE3f(
    q, Eigen::Vector3f(
      static_cast<float>(pose.position.x), static_cast<float>(pose.position.y),
      static_cast<float>(pose.position.z)));
}

geometry_msgs::msg::Pose ToPose(const Sophus::SE3f & pose)
{
  geometry_msgs::msg::Pose result;
  const auto q = pose.unit_quaternion();
  result.position.x = pose.translation().x();
  result.position.y = pose.translation().y();
  result.position.z = pose.translation().z();
  result.orientation.x = q.x();
  result.orientation.y = q.y();
  result.orientation.z = q.z();
  result.orientation.w = q.w();
  return result;
}

class GtTimingDiagnosticNode : public rclcpp::Node
{
public:
  GtTimingDiagnosticNode()
  : Node("f5h_gt_timing_diagnostic")
  {
    declare_parameter<std::string>("mode", "off");
    declare_parameter<int64_t>("drone_id", 0);
    declare_parameter<std::string>("body_frame", "base_link");
    declare_parameter<double>("publish_rate_hz", 50.0);
    mode_ = get_parameter("mode").as_string();
    drone_id_ = static_cast<uint32_t>(get_parameter("drone_id").as_int());
    body_frame_ = get_parameter("body_frame").as_string();
    const double publish_rate = std::max(1.0, get_parameter("publish_rate_hz").as_double());

    publisher_ = create_publisher<NavigationState>(
      "orbslam/navigation_state_orb", rclcpp::QoS(20).reliable());
    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "sensor/GT/pose", rclcpp::QoS(50),
      std::bind(&GtTimingDiagnosticNode::OnPose, this, std::placeholders::_1));
    velocity_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "sensor/GT/vel", rclcpp::QoS(50),
      std::bind(&GtTimingDiagnosticNode::OnVelocity, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate)),
      std::bind(&GtTimingDiagnosticNode::Publish, this));
    RCLCPP_WARN(
      get_logger(),
      "[F5H-GT-TIMING] mode=%s source=GT_LAB_ONLY predictor=OrbPosePredictor publish_hz=%.1f",
      mode_.c_str(), publish_rate);
  }

private:
  struct PendingMeasurement
  {
    Sophus::SE3f pose;
    double input_stamp = 0.0;
    double capture_local_stamp = 0.0;
    double release_local_stamp = 0.0;
    Eigen::Vector3f angular_velocity = Eigen::Vector3f::Zero();
  };

  struct TimedVelocity
  {
    double stamp = 0.0;
    Eigen::Vector3f angular = Eigen::Vector3f::Zero();
  };

  static const std::vector<double> & TracePeriods()
  {
    static const std::vector<double> values = {
      0.049, 0.050, 0.050, 0.050, 0.050, 0.120, 0.050, 0.120, 0.050, 0.050,
      0.050, 0.050, 0.051, 0.049, 0.050, 0.120, 0.050, 0.050, 0.050, 0.051,
      0.049, 0.051, 0.120, 0.049, 0.050, 0.051, 0.049, 0.120, 0.050, 0.050,
      0.120, 0.050, 0.120, 0.050, 0.051, 0.049, 0.120, 0.049, 0.050, 0.051};
    return values;
  }

  static const std::vector<double> & TraceDelays()
  {
    // Traza determinista de 268, centrada en los 80 ms medidos por correlacion.
    static const std::vector<double> values = {
      0.070032, 0.069895, 0.069949, 0.080022, 0.079992, 0.080006, 0.070029,
      0.080078, 0.079965, 0.070007, 0.070125, 0.070018, 0.079079, 0.080016,
      0.080023, 0.070031, 0.070001, 0.080038, 0.079959, 0.079076, 0.080063,
      0.079001, 0.068996, 0.080052, 0.070022, 0.068984, 0.070034, 0.070016,
      0.070021, 0.069943, 0.070070, 0.070044, 0.070034, 0.070015, 0.069017,
      0.070005, 0.069008, 0.070185, 0.070042, 0.078963};
    return values;
  }

  double SamplePeriod() const
  {
    if (mode_ == "gt_50") {
      return 0.02;
    }
    if (mode_ == "gt_orb_timing") {
      return TracePeriods()[trace_index_ % TracePeriods().size()];
    }
    return 0.05;
  }

  bool UsesExactAngularVelocity() const
  {
    return mode_ == "gt_20_exact_omega" || mode_ == "gt_20_exact_omega_hold" ||
      mode_ == "gt_20_exact_omega_extrapolate";
  }

  void OnVelocity(const geometry_msgs::msg::TwistStamped::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    TimedVelocity velocity;
    velocity.stamp = StampSeconds(message->header.stamp);
    velocity.angular = Eigen::Vector3f(
      static_cast<float>(message->twist.angular.x),
      static_cast<float>(message->twist.angular.y),
      static_cast<float>(message->twist.angular.z));
    velocities_.push_back(velocity);
    while (velocities_.size() > 200) {
      velocities_.pop_front();
    }
  }

  double DeliveryDelay() const
  {
    if (mode_ == "gt_20_delay") {
      return 0.08;
    }
    if (mode_ == "gt_orb_timing") {
      return TraceDelays()[trace_index_ % TraceDelays().size()];
    }
    return 0.0;
  }

  void OnPose(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    if (mode_ == "off") {
      return;
    }
    const double input_stamp = StampSeconds(message->header.stamp);
    std::lock_guard<std::mutex> lock(mutex_);
    if (last_sample_stamp_ > 0.0 && input_stamp + 1e-6 < next_sample_stamp_) {
      return;
    }
    const double now = get_clock()->now().seconds();
    PendingMeasurement pending;
    pending.pose = FromPose(message->pose);
    pending.input_stamp = input_stamp;
    pending.capture_local_stamp = now;
    pending.release_local_stamp = now + DeliveryDelay();
    if (UsesExactAngularVelocity()) {
      const auto velocity = std::find_if(
        velocities_.rbegin(), velocities_.rend(),
        [input_stamp](const TimedVelocity & candidate) {
          return candidate.stamp <= input_stamp + 1e-6;
        });
      if (velocity == velocities_.rend()) {
        return;
      }
      pending.angular_velocity = velocity->angular;
    }
    queue_.push_back(pending);
    last_sample_stamp_ = input_stamp;
    next_sample_stamp_ = input_stamp + SamplePeriod();
    ++trace_index_;
  }

  void LogMeasurement(const orbslam3_ros2::OrbPosePredictorDiagnostics & diagnostics)
  {
    const auto & q = diagnostics.measurement_orientation;
    const auto & before = diagnostics.predicted_orientation_before_measurement;
    const auto & after = diagnostics.base_orientation_after_measurement;
    RCLCPP_INFO(
      get_logger(),
      "[F5H-PHASE-MEASUREMENT] drone_id=%u input_stamp=%.9f receive_stamp=%.9f "
      "epoch=1 tracking=2 ref_kf=1 raw_q=(%.9f,%.9f,%.9f,%.9f) base_stamp=%.9f "
      "pred_before_q=(%.9f,%.9f,%.9f,%.9f) base_after_q=(%.9f,%.9f,%.9f,%.9f) "
      "base_update_applied=%s base_update_type=%s base_correction_rad=%.9f "
      "visual_base_error_before=%.9f visual_base_error_after=%.9f "
      "raw_omega_o=(%.9f,%.9f,%.9f) motion_target_o=(%.9f,%.9f,%.9f) "
      "motion_o=(%.9f,%.9f,%.9f) bias_o=(%.9f,%.9f,%.9f) "
      "total_o=(%.9f,%.9f,%.9f) omega_mid_prev=(%.9f,%.9f,%.9f) "
      "alpha_hat=(%.9f,%.9f,%.9f) omega_hat_k=(%.9f,%.9f,%.9f) "
      "t_mid_prev=%.9f t_mid_current=%.9f omega_prediction_horizon=%.9f "
      "omega_estimator_mode=%s raw_class=%s correction_class=%s",
      drone_id_, diagnostics.measurement_stamp_sec, latest_receive_stamp_,
      q.x(), q.y(), q.z(), q.w(), diagnostics.base_stamp_sec,
      before.x(), before.y(), before.z(), before.w(), after.x(), after.y(), after.z(), after.w(),
      diagnostics.base_update_applied ? "true" : "false",
      orbslam3_ros2::AngularBaseUpdateTypeName(diagnostics.base_update_type),
      diagnostics.base_rotation_correction_rad, diagnostics.visual_base_error_before_rad,
      diagnostics.visual_base_error_after_rad, diagnostics.implied_angular_velocity.x(),
      diagnostics.implied_angular_velocity.y(), diagnostics.implied_angular_velocity.z(),
      diagnostics.omega_motion_target.x(), diagnostics.omega_motion_target.y(),
      diagnostics.omega_motion_target.z(), diagnostics.omega_motion.x(),
      diagnostics.omega_motion.y(), diagnostics.omega_motion.z(), diagnostics.omega_bias.x(),
      diagnostics.omega_bias.y(), diagnostics.omega_bias.z(),
      diagnostics.omega_total_after_limits.x(), diagnostics.omega_total_after_limits.y(),
      diagnostics.omega_total_after_limits.z(),
      diagnostics.previous_raw_angular_velocity.x(),
      diagnostics.previous_raw_angular_velocity.y(),
      diagnostics.previous_raw_angular_velocity.z(),
      diagnostics.causal_angular_acceleration.x(),
      diagnostics.causal_angular_acceleration.y(),
      diagnostics.causal_angular_acceleration.z(),
      diagnostics.omega_hat_at_measurement.x(), diagnostics.omega_hat_at_measurement.y(),
      diagnostics.omega_hat_at_measurement.z(), diagnostics.previous_interval_mid_stamp_sec,
      diagnostics.current_interval_mid_stamp_sec, diagnostics.omega_prediction_horizon_sec,
      orbslam3_ros2::AngularMotionEstimatorModeName(diagnostics.omega_estimator_mode),
      orbslam3_ros2::RawMotionClassName(diagnostics.raw_motion_class),
      orbslam3_ros2::AngularCorrectionClassName(diagnostics.classification));
  }

  void Publish()
  {
    if (mode_ == "off") {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = get_clock()->now().seconds();
    while (!queue_.empty() && queue_.front().release_local_stamp <= now) {
      const auto measurement = queue_.front();
      queue_.pop_front();
      const orbslam3_ros2::OrbMeasurementContext context{1, 2, 1, false, 100};
      predictor_.UpdateMeasurement(measurement.pose, measurement.input_stamp, context);
      if (mode_ == "gt_20_exact_omega") {
        predictor_.OverrideAngularVelocityForDiagnostics(measurement.angular_velocity);
      }
      latest_measurement_pose_ = measurement.pose;
      latest_exact_omega_ = measurement.angular_velocity;
      latest_input_stamp_ = measurement.input_stamp;
      latest_arrival_stamp_ = measurement.capture_local_stamp;
      latest_receive_stamp_ = now;
      latest_capture_local_stamp_ = measurement.capture_local_stamp;
      ready_ = true;
      LogMeasurement(predictor_.last_diagnostics());
    }
    if (!ready_) {
      return;
    }
    const double visual_age = std::max(0.0, now - latest_capture_local_stamp_);
    const auto base = predictor_.Predict(latest_input_stamp_);
    auto predicted = predictor_.Predict(latest_input_stamp_ + visual_age);
    if (!predicted.valid || !predictor_.healthy()) {
      return;
    }
    if (mode_ == "gt_20_exact_omega_hold") {
      predicted.pose = Sophus::SE3f(
        latest_measurement_pose_.so3(), predicted.pose.translation());
      predicted.angular_velocity = latest_exact_omega_;
      predicted.velocity_valid = true;
      predicted.prediction_horizon_sec = 0.0;
      predicted.prediction_clamped = false;
    } else if (mode_ == "gt_20_exact_omega_extrapolate") {
      predicted.pose = Sophus::SE3f(
        Sophus::SO3f::exp(latest_exact_omega_ * static_cast<float>(visual_age)) *
        latest_measurement_pose_.so3(), predicted.pose.translation());
      predicted.angular_velocity = latest_exact_omega_;
      predicted.velocity_valid = true;
      predicted.prediction_horizon_sec = visual_age;
      predicted.prediction_clamped = false;
    }

    NavigationState message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = "world";
    message.child_frame_id = body_frame_;
    message.drone_id = drone_id_;
    message.sample_sequence = sequence_++;
    message.map_epoch = 1;
    message.tracking_state = NavigationState::TRACKING_OK;
    message.pose_source = NavigationState::POSE_SOURCE_ORB;
    message.global_status = NavigationState::GLOBAL_STATUS_AUTHORITATIVE;
    message.local_valid = true;
    message.local_continuity_valid = true;
    message.global_valid = true;
    message.velocity_valid = predicted.velocity_valid;
    message.reference_keyframe_valid = true;
    message.reference_keyframe_id = 1;
    message.tcr.orientation.w = 1.0;
    message.o_t_body = ToPose(predicted.pose);
    message.w_t_body = message.o_t_body;
    if (predicted.velocity_valid) {
      message.velocity.linear.x = predicted.linear_velocity.x();
      message.velocity.linear.y = predicted.linear_velocity.y();
      message.velocity.linear.z = predicted.linear_velocity.z();
      message.velocity.angular.x = predicted.angular_velocity.x();
      message.velocity.angular.y = predicted.angular_velocity.y();
      message.velocity.angular.z = predicted.angular_velocity.z();
    }
    publisher_->publish(message);

    const auto visual_q = predictor_.last_diagnostics().measurement_orientation;
    const auto base_q = base.valid ? base.pose.unit_quaternion() : Eigen::Quaternionf::Identity();
    const auto predicted_q = predicted.pose.unit_quaternion();
    RCLCPP_INFO(
      get_logger(),
      "[F5H-PHASE-PUBLISH] drone_id=%u publish_stamp=%.9f input_stamp=%.9f "
      "arrival_stamp=%.9f receive_stamp=%.9f visual_age_local=%.9f "
      "prediction_horizon=%.9f prediction_clamped=%s input_clock=gazebo_sim "
      "receive_clock=ros receive_age=%.9f sample=%lu source=1 epoch=1 tracking=2 "
      "ref_kf=1 local_valid=true velocity_valid=%s "
      "visual_q=(%.9f,%.9f,%.9f,%.9f) base_q=(%.9f,%.9f,%.9f,%.9f) "
      "predicted_q=(%.9f,%.9f,%.9f,%.9f) pose_q=(%.9f,%.9f,%.9f,%.9f) "
      "omega_o=(%.9f,%.9f,%.9f) diagnostic_mode=%s",
      drone_id_, now, latest_input_stamp_, latest_arrival_stamp_, latest_receive_stamp_,
      visual_age, predicted.prediction_horizon_sec,
      predicted.prediction_clamped ? "true" : "false", now - latest_receive_stamp_,
      static_cast<unsigned long>(message.sample_sequence),
      message.velocity_valid ? "true" : "false", visual_q.x(), visual_q.y(), visual_q.z(),
      visual_q.w(), base_q.x(), base_q.y(), base_q.z(), base_q.w(), predicted_q.x(),
      predicted_q.y(), predicted_q.z(), predicted_q.w(), message.o_t_body.orientation.x,
      message.o_t_body.orientation.y, message.o_t_body.orientation.z,
      message.o_t_body.orientation.w, message.velocity.angular.x, message.velocity.angular.y,
      message.velocity.angular.z, mode_.c_str());
  }

  std::string mode_;
  std::string body_frame_;
  uint32_t drone_id_ = 0;
  std::mutex mutex_;
  std::deque<PendingMeasurement> queue_;
  std::deque<TimedVelocity> velocities_;
  orbslam3_ros2::OrbPosePredictor predictor_;
  double last_sample_stamp_ = 0.0;
  double next_sample_stamp_ = 0.0;
  double latest_input_stamp_ = 0.0;
  double latest_arrival_stamp_ = 0.0;
  double latest_receive_stamp_ = 0.0;
  double latest_capture_local_stamp_ = 0.0;
  std::size_t trace_index_ = 0;
  uint64_t sequence_ = 0;
  bool ready_ = false;
  Sophus::SE3f latest_measurement_pose_;
  Eigen::Vector3f latest_exact_omega_ = Eigen::Vector3f::Zero();
  rclcpp::Publisher<NavigationState>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GtTimingDiagnosticNode>());
  rclcpp::shutdown();
  return 0;
}
