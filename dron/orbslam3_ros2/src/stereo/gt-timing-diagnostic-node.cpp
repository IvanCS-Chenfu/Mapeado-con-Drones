#include "navigation-state-estimator.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
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

constexpr double kDiagnosticMaxExtrapolationSec = 0.18;

orbslam3_ros2::OrbPosePredictorConfig DiagnosticPredictorConfig()
{
  orbslam3_ros2::OrbPosePredictorConfig config;
  config.max_extrapolation_sec = kDiagnosticMaxExtrapolationSec;
  config.predict_angular_acceleration = true;
  return config;
}

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
    declare_parameter<std::vector<double>>(
      "fisico.total.matriz_inercia", {1e-4, 1e-4, 1e-4, 0.0, 0.0, 0.0});
    declare_parameter<double>("fisico.total.masa", 1.4);
    mode_ = get_parameter("mode").as_string();
    drone_id_ = static_cast<uint32_t>(get_parameter("drone_id").as_int());
    body_frame_ = get_parameter("body_frame").as_string();
    const double publish_rate = std::max(1.0, get_parameter("publish_rate_hz").as_double());
    const auto inertia = get_parameter("fisico.total.matriz_inercia").as_double_array();
    Eigen::Matrix3f inertia_body;
    inertia_body << inertia[0], inertia[3], inertia[4],
      inertia[3], inertia[1], inertia[5],
      inertia[4], inertia[5], inertia[2];
    dynamic_predictor_.SetInertia(inertia_body);
    mass_kg_ = static_cast<float>(get_parameter("fisico.total.masa").as_double());
    translational_predictor_.SetMass(mass_kg_);
    if (UsesDynamicPredictor()) {
      const double startup_stamp = get_clock()->now().seconds();
      dynamic_predictor_.AddTorque(startup_stamp, Eigen::Vector3f::Zero());
      if (UsesTranslationalDynamicPredictor()) {
        translational_predictor_.AddThrust(startup_stamp, 0.0f);
      }
      RCLCPP_WARN(
        get_logger(),
        "[F5H-ACTUATION-SEED] type=ZERO stamp=%.9f "
        "tau=(0.000000000,0.000000000,0.000000000) thrust=0.000000000 "
        "reason=COLD_START_KNOWN_ZERO",
        startup_stamp);
    }

    publisher_ = create_publisher<NavigationState>(
      "orbslam/navigation_state_orb", rclcpp::QoS(20).reliable());
    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "sensor/GT/pose", rclcpp::QoS(50),
      std::bind(&GtTimingDiagnosticNode::OnPose, this, std::placeholders::_1));
    velocity_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "sensor/GT/vel", rclcpp::QoS(50),
      std::bind(&GtTimingDiagnosticNode::OnVelocity, this, std::placeholders::_1));
    torque_subscription_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "control/tray/torque", rclcpp::QoS(100),
      std::bind(&GtTimingDiagnosticNode::OnTorque, this, std::placeholders::_1));
    thrust_subscription_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "control/tray/thrust", rclcpp::QoS(100),
      std::bind(&GtTimingDiagnosticNode::OnThrust, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate)),
      std::bind(&GtTimingDiagnosticNode::Publish, this));
    RCLCPP_WARN(
      get_logger(),
      "[F5H-GT-TIMING] mode=%s source=GT_LAB_ONLY predictor=OrbPosePredictor "
      "publish_hz=%.1f max_extrapolation_sec=%.3f predict_alpha=true "
      "J=(%.9g,%.9g,%.9g,%.9g,%.9g,%.9g)",
      mode_.c_str(), publish_rate, kDiagnosticMaxExtrapolationSec,
      inertia[0], inertia[1], inertia[2], inertia[3], inertia[4], inertia[5]);
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
    Eigen::Vector3f linear = Eigen::Vector3f::Zero();
    Eigen::Vector3f angular = Eigen::Vector3f::Zero();
  };

  struct TimedPose
  {
    double stamp = 0.0;
    Sophus::SE3f pose;
  };

  bool UsesTraceTiming() const
  {
    return mode_ == "gt_orb_timing" || mode_ == "dynamic_299" ||
      mode_ == "dynamic_303" || mode_ == "dynamic_304" ||
      mode_ == "dynamic_305" || mode_ == "dynamic_306" ||
      mode_ == "dynamic_307" || mode_ == "dynamic_308" ||
      mode_ == "dynamic_309" || mode_ == "dynamic_310" ||
      mode_ == "dynamic_311" || mode_ == "dynamic_312" ||
      mode_ == "dynamic_314" || mode_ == "dynamic_315" ||
      mode_ == "dynamic_316" || mode_ == "dynamic_317" || mode_ == "dynamic_318";
  }

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
    if (UsesTraceTiming()) {
      return TracePeriods()[trace_index_ % TracePeriods().size()];
    }
    return 0.05;
  }

  bool UsesExactAngularVelocity() const
  {
    return mode_ == "gt_20_exact_omega" || mode_ == "gt_20_exact_omega_hold" ||
      mode_ == "gt_20_exact_omega_extrapolate" || mode_ == "dynamic_292";
  }

  bool UsesDynamicPredictor() const
  {
    return mode_ == "dynamic_292" || mode_ == "dynamic_293" ||
      mode_ == "dynamic_294" || mode_ == "dynamic_295" || mode_ == "dynamic_299" ||
      mode_ == "dynamic_303" || mode_ == "dynamic_304" ||
      mode_ == "dynamic_309" || mode_ == "dynamic_310" ||
      mode_ == "dynamic_311" || mode_ == "dynamic_312" ||
      mode_ == "dynamic_314" || mode_ == "dynamic_315" ||
      mode_ == "dynamic_316" || mode_ == "dynamic_317" || mode_ == "dynamic_318";
  }

  bool UsesTranslationalDynamicPredictor() const
  {
    return mode_ == "dynamic_309" || mode_ == "dynamic_310" ||
      mode_ == "dynamic_311" || mode_ == "dynamic_312" ||
      mode_ == "dynamic_314" || mode_ == "dynamic_315" ||
      mode_ == "dynamic_316" || mode_ == "dynamic_317" || mode_ == "dynamic_318";
  }

  bool UsesCausalLinearVelocityEstimator() const
  {
    return mode_ == "dynamic_314" || mode_ == "dynamic_315" ||
      mode_ == "dynamic_316" || mode_ == "dynamic_317" || mode_ == "dynamic_318";
  }

  bool UsesGtOrientationNow() const
  {
    return mode_ == "dynamic_292" || mode_ == "dynamic_293" ||
      mode_ == "gt_20_delay_rgt_omegapred" ||
      mode_ == "gt_20_delay_rgt_omegagt" ||
      mode_ == "gt_20_delay_pvgt_rgt_omegapred" ||
      mode_ == "gt_20_delay_pvgt_rgt_omegagt" || mode_ == "dynamic_305" ||
      mode_ == "dynamic_306" || mode_ == "dynamic_307" || mode_ == "dynamic_308" ||
      mode_ == "dynamic_309" || mode_ == "dynamic_310" ||
      mode_ == "dynamic_314" || mode_ == "dynamic_315";
  }

  bool UsesGtAngularVelocityNow() const
  {
    return mode_ == "gt_20_delay_rpred_omegagt" ||
      mode_ == "gt_20_delay_rgt_omegagt" ||
      mode_ == "gt_20_delay_pvgt_rpred_omegagt" ||
      mode_ == "gt_20_delay_pvgt_rgt_omegagt" || mode_ == "dynamic_305" ||
      mode_ == "dynamic_306" || mode_ == "dynamic_307" || mode_ == "dynamic_308" ||
      mode_ == "dynamic_309" || mode_ == "dynamic_310" ||
      mode_ == "dynamic_314" || mode_ == "dynamic_315";
  }

  bool UsesGtPositionNow() const
  {
    return mode_.find("gt_20_delay_pvgt_") == 0 || mode_ == "dynamic_292" ||
      mode_ == "dynamic_293" || mode_ == "dynamic_294";
  }

  bool UsesGtLinearVelocityNow() const
  {
    return UsesGtPositionNow();
  }

  bool UsesGtPositionFor303To306() const
  {
    return mode_ == "dynamic_303" || mode_ == "dynamic_304" ||
      mode_ == "dynamic_306";
  }

  bool UsesGtPositionFor307To308() const
  {
    return mode_ == "dynamic_307";
  }

  bool UsesGtLinearVelocityFor307To308() const
  {
    return mode_ == "dynamic_308";
  }

  bool UsesCrossDiagnostic() const
  {
    return UsesGtPositionNow() || UsesGtPositionFor303To306() ||
      UsesGtPositionFor307To308() || UsesGtLinearVelocityFor307To308() ||
      UsesGtOrientationNow() || UsesGtAngularVelocityNow();
  }

  void OnTorque(const geometry_msgs::msg::Vector3Stamped::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const Eigen::Vector3f torque_body(
      static_cast<float>(message->vector.x), static_cast<float>(message->vector.y),
      static_cast<float>(message->vector.z));
    double stamp = StampSeconds(message->header.stamp);
    if (stamp <= 0.0) {
      stamp = get_clock()->now().seconds();
    }
    dynamic_predictor_.AddTorque(stamp, torque_body);
    latest_torque_body_ = torque_body;
  }

  void OnThrust(const geometry_msgs::msg::Vector3Stamped::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double stamp = StampSeconds(message->header.stamp);
    translational_predictor_.AddThrust(stamp, static_cast<float>(message->vector.z));
    latest_thrust_newton_ = static_cast<float>(message->vector.z);
  }

  void OnVelocity(const geometry_msgs::msg::TwistStamped::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    TimedVelocity velocity;
    velocity.stamp = StampSeconds(message->header.stamp);
    velocity.linear = Eigen::Vector3f(
      static_cast<float>(message->twist.linear.x),
      static_cast<float>(message->twist.linear.y),
      static_cast<float>(message->twist.linear.z));
    velocity.angular = Eigen::Vector3f(
      static_cast<float>(message->twist.angular.x),
      static_cast<float>(message->twist.angular.y),
      static_cast<float>(message->twist.angular.z));
    latest_gt_omega_now_ = velocity.angular;
    latest_gt_linear_velocity_now_ = velocity.linear;
    latest_gt_omega_input_stamp_ = velocity.stamp;
    latest_gt_omega_receive_local_stamp_ = get_clock()->now().seconds();
    latest_gt_omega_ready_ = true;
    latest_gt_linear_velocity_ready_ = true;
    velocities_.push_back(velocity);
    while (velocities_.size() > 200) {
      velocities_.pop_front();
    }
  }

  double DeliveryDelay() const
  {
    if (UsesTraceTiming()) {
      return TraceDelays()[trace_index_ % TraceDelays().size()];
    }
    if (mode_ == "gt_20_delay" || UsesCrossDiagnostic()) {
      return 0.08;
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
    const double now = get_clock()->now().seconds();
    latest_gt_pose_now_ = FromPose(message->pose);
    latest_gt_pose_input_stamp_ = input_stamp;
    latest_gt_pose_receive_local_stamp_ = now;
    latest_gt_pose_ready_ = true;
    poses_.push_back({input_stamp, latest_gt_pose_now_});
    while (poses_.size() > 200) {
      poses_.pop_front();
    }
    if (last_sample_stamp_ > 0.0 && input_stamp + 1e-6 < next_sample_stamp_) {
      return;
    }
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

  bool InterpolateGtAt(
    double stamp, Sophus::SE3f & pose, Eigen::Vector3f & linear,
    Eigen::Vector3f & angular, double & bracket_sec) const
  {
    const auto pose_hi = std::lower_bound(
      poses_.begin(), poses_.end(), stamp,
      [](const TimedPose & sample, double value) {return sample.stamp < value;});
    const auto vel_hi = std::lower_bound(
      velocities_.begin(), velocities_.end(), stamp,
      [](const TimedVelocity & sample, double value) {return sample.stamp < value;});
    if (pose_hi == poses_.begin() || pose_hi == poses_.end() ||
      vel_hi == velocities_.begin() || vel_hi == velocities_.end())
    {
      return false;
    }
    const auto pose_lo = std::prev(pose_hi);
    const auto vel_lo = std::prev(vel_hi);
    const double pose_dt = pose_hi->stamp - pose_lo->stamp;
    const double vel_dt = vel_hi->stamp - vel_lo->stamp;
    if (pose_dt <= 1e-9 || vel_dt <= 1e-9) {
      return false;
    }
    const float pose_alpha = static_cast<float>((stamp - pose_lo->stamp) / pose_dt);
    const float vel_alpha = static_cast<float>((stamp - vel_lo->stamp) / vel_dt);
    Eigen::Quaternionf q = pose_lo->pose.unit_quaternion().slerp(
      pose_alpha, pose_hi->pose.unit_quaternion());
    q.normalize();
    pose = Sophus::SE3f(
      q, pose_lo->pose.translation() +
      pose_alpha * (pose_hi->pose.translation() - pose_lo->pose.translation()));
    linear = vel_lo->linear + vel_alpha * (vel_hi->linear - vel_lo->linear);
    angular = vel_lo->angular + vel_alpha * (vel_hi->angular - vel_lo->angular);
    bracket_sec = std::max(pose_dt, vel_dt);
    return true;
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
      const auto measurement_state =
        predictor_.UpdateMeasurement(measurement.pose, measurement.input_stamp, context);
      orbslam3_ros2::CausalLinearVelocityEstimate causal_linear_estimate;
      if (UsesCausalLinearVelocityEstimator()) {
        causal_linear_estimate = causal_linear_estimator_.AddSample(
          measurement.pose.translation(), measurement.input_stamp, 1, true);
        latest_causal_linear_estimate_ = causal_linear_estimate;
      }
      if (mode_ == "gt_20_exact_omega") {
        predictor_.OverrideAngularVelocityForDiagnostics(measurement.angular_velocity);
      }
      latest_measurement_pose_ = measurement.pose;
      latest_exact_omega_ = measurement.angular_velocity;
      latest_input_stamp_ = measurement.input_stamp;
      latest_arrival_stamp_ = measurement.capture_local_stamp;
      latest_receive_stamp_ = now;
      latest_capture_local_stamp_ = measurement.capture_local_stamp;
      const bool dynamic_velocity_ready = UsesCausalLinearVelocityEstimator() ?
        causal_linear_estimate.valid : measurement_state.velocity_valid;
      if (UsesDynamicPredictor() && dynamic_velocity_ready) {
        dynamic_base_pose_ = measurement.pose;
        dynamic_base_omega_world_ = mode_ == "dynamic_292" ?
          measurement.angular_velocity : measurement_state.angular_velocity;
        if (mode_ == "dynamic_304") {
          Eigen::Vector3f gt_linear_tk;
          if (!InterpolateGtAt(
              measurement.input_stamp, dynamic_base_pose_, gt_linear_tk,
              dynamic_base_omega_world_, gt_tk_bracket_sec_))
          {
            RCLCPP_WARN(
              get_logger(), "[F5H-GT-TK-INVALID] stamp=%.9f", measurement.input_stamp);
            dynamic_base_ready_ = false;
            continue;
          }
          gt_omega_tk_ = dynamic_base_omega_world_;
          gt_pose_tk_ = dynamic_base_pose_;
          gt_tk_ready_ = true;
        }
        dynamic_base_position_world_ = measurement.pose.translation();
        dynamic_base_linear_velocity_world_ = UsesCausalLinearVelocityEstimator() ?
          causal_linear_estimate.velocity_at_sample : measurement_state.linear_velocity;
        if (mode_ == "dynamic_309") {
          Sophus::SE3f gt_pose_tk;
          Eigen::Vector3f gt_omega_tk;
          if (!InterpolateGtAt(
              measurement.input_stamp, gt_pose_tk, dynamic_base_linear_velocity_world_,
              gt_omega_tk, gt_tk_bracket_sec_))
          {
            RCLCPP_WARN(
              get_logger(), "[F5H-GT-TK-INVALID] stamp=%.9f", measurement.input_stamp);
            dynamic_base_ready_ = false;
            continue;
          }
          dynamic_base_position_world_ = gt_pose_tk.translation();
          gt_tk_ready_ = true;
        }
        if (UsesCausalLinearVelocityEstimator()) {
          gt_tk_ready_ = false;
          Sophus::SE3f gt_pose_tk;
          Eigen::Vector3f gt_omega_tk;
          if (InterpolateGtAt(
              measurement.input_stamp, gt_pose_tk, latest_gt_linear_velocity_tk_,
              gt_omega_tk, gt_tk_bracket_sec_))
          {
            gt_tk_ready_ = true;
          }
          RCLCPP_INFO(
            get_logger(),
            "[F5H-LINEAR-ESTIMATE] mode=%s estimator_mode=%s accepted=%s valid=%s "
            "input_stamp=%.9f dt1=%.9f dt2=%.9f tmid1=%.9f tmid2=%.9f "
            "horizon_tk=%.9f p_k2=(%.9f,%.9f,%.9f) p_k1=(%.9f,%.9f,%.9f) "
            "p_k=(%.9f,%.9f,%.9f) v_mid_prev=(%.9f,%.9f,%.9f) "
            "v_mid_current=(%.9f,%.9f,%.9f) a_hat=(%.9f,%.9f,%.9f) "
            "v_hat_tk=(%.9f,%.9f,%.9f) v_gt_tk=(%.9f,%.9f,%.9f) gt_tk_valid=%s",
            mode_.c_str(), orbslam3_ros2::LinearVelocityEstimatorModeName(
              causal_linear_estimate.mode),
            causal_linear_estimate.sample_accepted ? "true" : "false",
            causal_linear_estimate.valid ? "true" : "false", measurement.input_stamp,
            causal_linear_estimate.dt_previous_sec, causal_linear_estimate.dt_current_sec,
            causal_linear_estimate.previous_mid_stamp_sec,
            causal_linear_estimate.current_mid_stamp_sec,
            causal_linear_estimate.prediction_horizon_sec,
            causal_linear_estimate.p_k2.x(), causal_linear_estimate.p_k2.y(),
            causal_linear_estimate.p_k2.z(), causal_linear_estimate.p_k1.x(),
            causal_linear_estimate.p_k1.y(), causal_linear_estimate.p_k1.z(),
            causal_linear_estimate.p_k.x(), causal_linear_estimate.p_k.y(),
            causal_linear_estimate.p_k.z(), causal_linear_estimate.previous_mid_velocity.x(),
            causal_linear_estimate.previous_mid_velocity.y(),
            causal_linear_estimate.previous_mid_velocity.z(),
            causal_linear_estimate.current_mid_velocity.x(),
            causal_linear_estimate.current_mid_velocity.y(),
            causal_linear_estimate.current_mid_velocity.z(),
            causal_linear_estimate.acceleration.x(), causal_linear_estimate.acceleration.y(),
            causal_linear_estimate.acceleration.z(), causal_linear_estimate.velocity_at_sample.x(),
            causal_linear_estimate.velocity_at_sample.y(),
            causal_linear_estimate.velocity_at_sample.z(), latest_gt_linear_velocity_tk_.x(),
            latest_gt_linear_velocity_tk_.y(), latest_gt_linear_velocity_tk_.z(),
            gt_tk_ready_ ? "true" : "false");
        }
        dynamic_base_local_stamp_ = measurement.capture_local_stamp;
        dynamic_base_ready_ = true;
      }
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
    orbslam3_ros2::DynamicAngularPrediction dynamic_prediction;
    if (UsesDynamicPredictor()) {
      if (!dynamic_base_ready_) {
        return;
      }
      dynamic_prediction = dynamic_predictor_.Predict(
        dynamic_base_pose_.so3(), dynamic_base_omega_world_,
        dynamic_base_local_stamp_, now);
      if (!dynamic_prediction.valid) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[F5H-DYNAMIC-MISSING] mode=%s base=%.9f target=%.9f buffer=%zu "
          "coverage=%s oldest=%.9f newest=%.9f missing_prefix=%.9f",
          mode_.c_str(), dynamic_base_local_stamp_, now,
          dynamic_predictor_.torque_buffer_size(),
          orbslam3_ros2::ActuationCoverageStatusName(
            dynamic_prediction.torque_coverage.status),
          dynamic_prediction.torque_coverage.oldest_stamp_sec,
          dynamic_prediction.torque_coverage.newest_stamp_sec,
          dynamic_prediction.torque_coverage.missing_prefix_sec);
        return;
      }
    }
    orbslam3_ros2::DynamicTranslationalPrediction translational_prediction;
    if (UsesTranslationalDynamicPredictor()) {
      translational_prediction = translational_predictor_.Predict(
        dynamic_base_position_world_, dynamic_base_linear_velocity_world_,
        dynamic_base_pose_.so3(), dynamic_base_omega_world_, dynamic_base_local_stamp_, now,
        dynamic_predictor_);
      if (!translational_prediction.valid) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[F5H-TRANSLATIONAL-MISSING] mode=%s base=%.9f target=%.9f force_buffer=%zu "
          "missing_force=%s missing_orientation=%s coverage=%s oldest=%.9f "
          "newest=%.9f missing_prefix=%.9f",
          mode_.c_str(), dynamic_base_local_stamp_, now,
          translational_predictor_.thrust_buffer_size(),
          translational_prediction.missing_force_interval ? "true" : "false",
          translational_prediction.missing_orientation_interval ? "true" : "false",
          orbslam3_ros2::ActuationCoverageStatusName(
            translational_prediction.thrust_coverage.status),
          translational_prediction.thrust_coverage.oldest_stamp_sec,
          translational_prediction.thrust_coverage.newest_stamp_sec,
          translational_prediction.thrust_coverage.missing_prefix_sec);
        return;
      }
    }
    if (((UsesGtPositionNow() || UsesGtPositionFor303To306() ||
      UsesGtPositionFor307To308() || UsesGtOrientationNow()) &&
      !latest_gt_pose_ready_) ||
      ((UsesGtLinearVelocityNow() || UsesGtPositionFor303To306() ||
      UsesGtLinearVelocityFor307To308()) &&
      !latest_gt_linear_velocity_ready_) ||
      (UsesGtAngularVelocityNow() && !latest_gt_omega_ready_))
    {
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
    const auto predictor_pose = predicted.pose;
    const auto predictor_linear_velocity = predicted.linear_velocity;
    const auto predictor_angular_velocity = predicted.angular_velocity;
    const auto control_state = orbslam3_ros2::SelectDiagnosticControlState(
      predicted.pose, predicted.linear_velocity, predicted.angular_velocity,
      latest_gt_pose_now_, latest_gt_linear_velocity_now_, latest_gt_omega_now_,
      UsesGtPositionNow() || UsesGtPositionFor303To306() || UsesGtPositionFor307To308(),
      UsesGtLinearVelocityNow() || UsesGtPositionFor303To306() ||
      UsesGtLinearVelocityFor307To308(), UsesGtOrientationNow(),
      UsesGtAngularVelocityNow());
    predicted.pose = control_state.pose;
    predicted.linear_velocity = control_state.linear_velocity;
    predicted.angular_velocity = control_state.angular_velocity;
    if (UsesGtLinearVelocityNow()) {
      predicted.velocity_valid = true;
    }
    if (UsesDynamicPredictor()) {
      if (mode_ == "dynamic_294" || mode_ == "dynamic_295" ||
        mode_ == "dynamic_299" || mode_ == "dynamic_303" || mode_ == "dynamic_304" ||
        mode_ == "dynamic_311" || mode_ == "dynamic_312" ||
        mode_ == "dynamic_316" || mode_ == "dynamic_317" || mode_ == "dynamic_318")
      {
        predicted.pose = Sophus::SE3f(
          dynamic_prediction.orientation, predicted.pose.translation());
      }
      if (!UsesGtAngularVelocityNow()) {
        predicted.angular_velocity = dynamic_prediction.angular_velocity_world;
      }
      predicted.velocity_valid = true;
      RCLCPP_INFO(
        get_logger(),
        "[F5H-DYNAMIC-PREDICT] mode=%s base_stamp=%.9f target_stamp=%.9f "
        "horizon=%.9f steps=%u torque_samples=%u missing=%s "
        "omega_initial_o=(%.9f,%.9f,%.9f) tau_body=(%.9f,%.9f,%.9f) "
        "omega_dynamic_o=(%.9f,%.9f,%.9f) omega_dynamic_body=(%.9f,%.9f,%.9f) "
        "omega_gt_o=(%.9f,%.9f,%.9f)",
        mode_.c_str(), dynamic_base_local_stamp_, now, dynamic_prediction.horizon_sec,
        dynamic_prediction.integration_steps, dynamic_prediction.torque_samples_used,
        dynamic_prediction.missing_torque_interval ? "true" : "false",
        dynamic_base_omega_world_.x(), dynamic_base_omega_world_.y(),
        dynamic_base_omega_world_.z(), latest_torque_body_.x(), latest_torque_body_.y(),
        latest_torque_body_.z(), dynamic_prediction.angular_velocity_world.x(),
        dynamic_prediction.angular_velocity_world.y(),
        dynamic_prediction.angular_velocity_world.z(),
        dynamic_prediction.angular_velocity_body.x(),
        dynamic_prediction.angular_velocity_body.y(),
        dynamic_prediction.angular_velocity_body.z(), latest_gt_omega_now_.x(),
        latest_gt_omega_now_.y(), latest_gt_omega_now_.z());
    }
    if (UsesTranslationalDynamicPredictor()) {
      predicted.pose = Sophus::SE3f(
        predicted.pose.so3(), translational_prediction.position_world);
      predicted.linear_velocity = translational_prediction.linear_velocity_world;
      predicted.velocity_valid = true;
      RCLCPP_INFO(
        get_logger(),
        "[F5H-TRANSLATIONAL-PREDICT] mode=%s base_stamp=%.9f target_stamp=%.9f "
        "horizon=%.9f steps=%u force_samples=%u missing_force=%s "
        "mass=%.9f gravity=(0,0,-9.81) thrust=%.9f "
        "p_initial=(%.9f,%.9f,%.9f) v_initial=(%.9f,%.9f,%.9f) "
        "a_dynamic=(%.9f,%.9f,%.9f) p_dynamic=(%.9f,%.9f,%.9f) "
        "v_dynamic=(%.9f,%.9f,%.9f) p_gt_now=(%.9f,%.9f,%.9f) "
        "v_gt_now=(%.9f,%.9f,%.9f)",
        mode_.c_str(), dynamic_base_local_stamp_, now, translational_prediction.horizon_sec,
        translational_prediction.integration_steps, translational_prediction.force_samples_used,
        translational_prediction.missing_force_interval ? "true" : "false", mass_kg_,
        latest_thrust_newton_, dynamic_base_position_world_.x(),
        dynamic_base_position_world_.y(), dynamic_base_position_world_.z(),
        dynamic_base_linear_velocity_world_.x(), dynamic_base_linear_velocity_world_.y(),
        dynamic_base_linear_velocity_world_.z(), translational_prediction.acceleration_world.x(),
        translational_prediction.acceleration_world.y(),
        translational_prediction.acceleration_world.z(),
        translational_prediction.position_world.x(), translational_prediction.position_world.y(),
        translational_prediction.position_world.z(),
        translational_prediction.linear_velocity_world.x(),
        translational_prediction.linear_velocity_world.y(),
        translational_prediction.linear_velocity_world.z(),
        latest_gt_pose_now_.translation().x(), latest_gt_pose_now_.translation().y(),
        latest_gt_pose_now_.translation().z(), latest_gt_linear_velocity_now_.x(),
        latest_gt_linear_velocity_now_.y(), latest_gt_linear_velocity_now_.z());
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
    const auto predicted_q = predictor_pose.unit_quaternion();
    RCLCPP_INFO(
      get_logger(),
      "[F5H-PHASE-PUBLISH] drone_id=%u publish_stamp=%.9f input_stamp=%.9f "
      "arrival_stamp=%.9f receive_stamp=%.9f visual_age_local=%.9f "
      "prediction_horizon=%.9f prediction_clamped=%s input_clock=gazebo_sim "
      "receive_clock=ros receive_age=%.9f sample=%lu source=1 epoch=1 tracking=2 "
      "ref_kf=1 local_valid=true velocity_valid=%s "
      "visual_q=(%.9f,%.9f,%.9f,%.9f) base_q=(%.9f,%.9f,%.9f,%.9f) "
      "predicted_q=(%.9f,%.9f,%.9f,%.9f) pose_q=(%.9f,%.9f,%.9f,%.9f) "
      "omega_o=(%.9f,%.9f,%.9f) alpha_pred=(%.9f,%.9f,%.9f) "
      "delta_theta_pred=(%.9f,%.9f,%.9f) alpha_clamped=%s omega_clamped=%s "
      "diagnostic_mode=%s orientation_source=%s omega_source=%s "
      "position_source=%s velocity_source=%s "
      "p_pred=(%.9f,%.9f,%.9f) p_gt_now=(%.9f,%.9f,%.9f) p_used=(%.9f,%.9f,%.9f) "
      "v_pred=(%.9f,%.9f,%.9f) v_gt_now=(%.9f,%.9f,%.9f) v_used=(%.9f,%.9f,%.9f) "
      "omega_pred_o=(%.9f,%.9f,%.9f) omega_gt_now_o=(%.9f,%.9f,%.9f) "
      "gt_now_q=(%.9f,%.9f,%.9f,%.9f) "
      "gt_pose_input_stamp=%.9f gt_omega_input_stamp=%.9f "
      "gt_pose_age_local=%.9f gt_omega_age_local=%.9f gt_pose_omega_stamp_skew=%.9f "
      "gt_tk_valid=%s gt_tk_bracket_sec=%.9f omega_gt_tk=(%.9f,%.9f,%.9f) "
      "r_dynamic_vs_gt_tk_rad=%.9f",
      drone_id_, now, latest_input_stamp_, latest_arrival_stamp_, latest_receive_stamp_,
      visual_age, predicted.prediction_horizon_sec,
      predicted.prediction_clamped ? "true" : "false", now - latest_receive_stamp_,
      static_cast<unsigned long>(message.sample_sequence),
      message.velocity_valid ? "true" : "false", visual_q.x(), visual_q.y(), visual_q.z(),
      visual_q.w(), base_q.x(), base_q.y(), base_q.z(), base_q.w(), predicted_q.x(),
      predicted_q.y(), predicted_q.z(), predicted_q.w(), message.o_t_body.orientation.x,
      message.o_t_body.orientation.y, message.o_t_body.orientation.z,
      message.o_t_body.orientation.w, message.velocity.angular.x, message.velocity.angular.y,
      message.velocity.angular.z, predicted.angular_acceleration.x(),
      predicted.angular_acceleration.y(), predicted.angular_acceleration.z(),
      predicted.angular_prediction_delta.x(), predicted.angular_prediction_delta.y(),
      predicted.angular_prediction_delta.z(),
      predicted.angular_acceleration_clamped ? "true" : "false",
      predicted.angular_velocity_clamped ? "true" : "false", mode_.c_str(),
      UsesGtOrientationNow() ? "GT_NOW" :
      (UsesDynamicPredictor() ? "DYNAMIC" : "PREDICTED"),
      UsesGtAngularVelocityNow() ? "GT_NOW" :
      (UsesDynamicPredictor() ? "DYNAMIC" : "PREDICTED"),
      (UsesGtPositionNow() || UsesGtPositionFor303To306() ||
      UsesGtPositionFor307To308()) ? "GT_NOW" : "PREDICTED",
      (UsesGtLinearVelocityNow() || UsesGtPositionFor303To306() ||
      UsesGtLinearVelocityFor307To308()) ? "GT_NOW" : "PREDICTED",
      predictor_pose.translation().x(), predictor_pose.translation().y(),
      predictor_pose.translation().z(), latest_gt_pose_now_.translation().x(),
      latest_gt_pose_now_.translation().y(), latest_gt_pose_now_.translation().z(),
      predicted.pose.translation().x(), predicted.pose.translation().y(),
      predicted.pose.translation().z(), predictor_linear_velocity.x(),
      predictor_linear_velocity.y(), predictor_linear_velocity.z(),
      latest_gt_linear_velocity_now_.x(), latest_gt_linear_velocity_now_.y(),
      latest_gt_linear_velocity_now_.z(), predicted.linear_velocity.x(),
      predicted.linear_velocity.y(), predicted.linear_velocity.z(),
      predictor_angular_velocity.x(), predictor_angular_velocity.y(),
      predictor_angular_velocity.z(), latest_gt_omega_now_.x(), latest_gt_omega_now_.y(),
      latest_gt_omega_now_.z(), latest_gt_pose_now_.unit_quaternion().x(),
      latest_gt_pose_now_.unit_quaternion().y(), latest_gt_pose_now_.unit_quaternion().z(),
      latest_gt_pose_now_.unit_quaternion().w(),
      latest_gt_pose_input_stamp_, latest_gt_omega_input_stamp_,
      latest_gt_pose_ready_ ? std::max(0.0, now - latest_gt_pose_receive_local_stamp_) : -1.0,
      latest_gt_omega_ready_ ? std::max(0.0, now - latest_gt_omega_receive_local_stamp_) : -1.0,
      (latest_gt_pose_ready_ && latest_gt_omega_ready_) ?
      std::abs(latest_gt_pose_input_stamp_ - latest_gt_omega_input_stamp_) : -1.0,
      gt_tk_ready_ ? "true" : "false", gt_tk_bracket_sec_, gt_omega_tk_.x(),
      gt_omega_tk_.y(), gt_omega_tk_.z(),
      gt_tk_ready_ ?
      (dynamic_base_pose_.so3() * gt_pose_tk_.so3().inverse()).log().norm() : -1.0);
  }

  std::string mode_;
  std::string body_frame_;
  uint32_t drone_id_ = 0;
  std::mutex mutex_;
  std::deque<PendingMeasurement> queue_;
  std::deque<TimedVelocity> velocities_;
  std::deque<TimedPose> poses_;
  orbslam3_ros2::OrbPosePredictor predictor_{DiagnosticPredictorConfig()};
  orbslam3_ros2::BodyTorqueDynamicPredictor dynamic_predictor_{};
  orbslam3_ros2::BodyThrustDynamicPredictor translational_predictor_{};
  orbslam3_ros2::CausalLinearVelocityEstimator causal_linear_estimator_{};
  orbslam3_ros2::CausalLinearVelocityEstimate latest_causal_linear_estimate_;
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
  Sophus::SE3f latest_gt_pose_now_;
  Eigen::Vector3f latest_gt_omega_now_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f latest_gt_linear_velocity_now_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f latest_gt_linear_velocity_tk_ = Eigen::Vector3f::Zero();
  double latest_gt_pose_input_stamp_ = 0.0;
  double latest_gt_omega_input_stamp_ = 0.0;
  double latest_gt_pose_receive_local_stamp_ = 0.0;
  double latest_gt_omega_receive_local_stamp_ = 0.0;
  bool latest_gt_pose_ready_ = false;
  bool latest_gt_omega_ready_ = false;
  bool latest_gt_linear_velocity_ready_ = false;
  bool dynamic_base_ready_ = false;
  double dynamic_base_local_stamp_ = 0.0;
  Sophus::SE3f dynamic_base_pose_;
  Eigen::Vector3f dynamic_base_omega_world_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f latest_torque_body_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f dynamic_base_position_world_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f dynamic_base_linear_velocity_world_ = Eigen::Vector3f::Zero();
  float latest_thrust_newton_ = 0.0f;
  float mass_kg_ = 1.4f;
  Sophus::SE3f gt_pose_tk_;
  Eigen::Vector3f gt_omega_tk_ = Eigen::Vector3f::Zero();
  double gt_tk_bracket_sec_ = 0.0;
  bool gt_tk_ready_ = false;
  rclcpp::Publisher<NavigationState>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr torque_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr thrust_subscription_;
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
