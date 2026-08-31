#include "dron_individual/navigation_state_mux.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "orbslam3_msgs/msg/navigation_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <sstream>

namespace
{

using NavigationState = orbslam3_msgs::msg::NavigationState;
using dron_individual::FallbackReason;
using dron_individual::NavigationSource;
using dron_individual::RigidPose;

RigidPose FromMessage(const geometry_msgs::msg::Pose & pose)
{
  RigidPose result;
  result.translation = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  result.rotation = Eigen::Quaterniond(
    pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z);
  if (result.rotation.norm() < 1e-9) {
    result.rotation = Eigen::Quaterniond::Identity();
  } else {
    result.rotation.normalize();
  }
  return result;
}

geometry_msgs::msg::Pose ToMessage(const RigidPose & pose)
{
  geometry_msgs::msg::Pose result;
  result.position.x = pose.translation.x();
  result.position.y = pose.translation.y();
  result.position.z = pose.translation.z();
  result.orientation.x = pose.rotation.x();
  result.orientation.y = pose.rotation.y();
  result.orientation.z = pose.rotation.z();
  result.orientation.w = pose.rotation.w();
  return result;
}

class NavigationStateMuxNode : public rclcpp::Node
{
public:
  NavigationStateMuxNode()
  : Node("navigation_state_mux")
  {
    declare_parameter<double>("gt_timeout_sec", 0.5);
    declare_parameter<bool>("gt_fallback_enabled", false);
    declare_parameter<int64_t>("orb_qualification_samples", 20);
    declare_parameter<bool>("debug_architecture_telemetry", false);
    declare_parameter<int64_t>("drone_id", 0);
    declare_parameter<std::string>("f5h_diagnostic_force_source", "normal");
    declare_parameter<std::string>("f5h_orb_control_override", "normal");
    declare_parameter<double>("f5h_shadow_settle_duration_sec", 1.5);
    declare_parameter<double>("f5h_shadow_max_linear_speed", 0.15);
    declare_parameter<double>("f5h_shadow_max_angular_speed", 0.15);
    gt_timeout_sec_ = get_parameter("gt_timeout_sec").as_double();
    gt_fallback_enabled_ = get_parameter("gt_fallback_enabled").as_bool();
    orb_qualification_samples_ = static_cast<std::size_t>(std::max<int64_t>(
        2, get_parameter("orb_qualification_samples").as_int()));
    if (gt_timeout_sec_ <= 0.0) {
      gt_timeout_sec_ = 0.5;
    }
    debug_architecture_telemetry_ =
      get_parameter("debug_architecture_telemetry").as_bool();
    drone_id_ = static_cast<uint32_t>(get_parameter("drone_id").as_int());
    diagnostic_force_source_ =
      get_parameter("f5h_diagnostic_force_source").as_string();
    diagnostic_orb_control_mode_ = dron_individual::ParseDiagnosticOrbControlMode(
      get_parameter("f5h_orb_control_override").as_string());
    RCLCPP_WARN(
      get_logger(), "[F5H-ORB-CONTROL-OVERRIDE] mode=%s",
      get_parameter("f5h_orb_control_override").as_string().c_str());
    shadow_settle_duration_sec_ =
      get_parameter("f5h_shadow_settle_duration_sec").as_double();
    shadow_max_linear_speed_ =
      get_parameter("f5h_shadow_max_linear_speed").as_double();
    shadow_max_angular_speed_ =
      get_parameter("f5h_shadow_max_angular_speed").as_double();
    if (debug_architecture_telemetry_) {
      architecture_activity_pub_ = create_publisher<std_msgs::msg::String>(
        "/system_architecture/activity", rclcpp::QoS(64).best_effort());
    }
    publisher_ = create_publisher<NavigationState>(
      "orbslam/navigation_state", rclcpp::QoS(20).reliable());
    orb_authority_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "control/orb_authority_confirmed",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    PublishOrbAuthority(false);
    orb_subscription_ = create_subscription<NavigationState>(
      "orbslam/navigation_state_orb", rclcpp::QoS(20).reliable(),
      std::bind(&NavigationStateMuxNode::OnOrbState, this, std::placeholders::_1));
    // TODO FASE 6: retirar la suscripcion GT junto con el fallback temporal.
    gt_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "sensor/GT/pose", rclcpp::QoS(20),
      std::bind(&NavigationStateMuxNode::OnGtPose, this, std::placeholders::_1));
    gt_velocity_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "sensor/GT/vel", rclcpp::QoS(20),
      std::bind(&NavigationStateMuxNode::OnGtVelocity, this, std::placeholders::_1));
    trajectory_active_service_ = create_service<std_srvs::srv::SetBool>(
      "control/set_trajectory_active",
      std::bind(
        &NavigationStateMuxNode::OnTrajectoryActive, this,
        std::placeholders::_1, std::placeholders::_2));
    shadow_activation_service_ = create_service<std_srvs::srv::SetBool>(
      "control/activate_orb_shadow",
      std::bind(
        &NavigationStateMuxNode::OnShadowActivation, this,
        std::placeholders::_1, std::placeholders::_2));
    metrics_timer_ = create_wall_timer(
      std::chrono::seconds(10), std::bind(&NavigationStateMuxNode::LogMetrics, this));
  }

private:
  void OnShadowActivation(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (diagnostic_force_source_ != "shadow_gt") {
      response->success = false;
      response->message = "shadow_mode_disabled";
      return;
    }
    if (!request->data) {
      shadow_orb_activated_ = false;
      shadow_gate_.Reset();
      PublishOrbAuthority(false);
      response->success = true;
      response->message = "shadow_gt_authority_reset";
      return;
    }
    if (!shadow_gate_.ready()) {
      response->success = false;
      response->message = "shadow_activation_not_ready";
      return;
    }
    shadow_orb_activated_ = true;
    response->success = true;
    response->message = "shadow_orb_authority_enabled";
    RCLCPP_WARN(
      get_logger(),
      "[F5H-ORB-ACTIVATION-READY] stamp=%.9f tracking=true anchor=true airborne=true "
      "settled=true orb_local_valid=true orb_velocity_valid=true settle_sec=%.3f",
      get_clock()->now().seconds(), shadow_settle_duration_sec_);
  }

  void OnTrajectoryActive(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (request->data) {
      goal_source_lock_.Begin(last_source_);
    } else {
      goal_source_lock_.End();
    }
    response->success = true;
    response->message = request->data ? "trajectory_source_locked" : "trajectory_boundary_open";
    RCLCPP_WARN(
      get_logger(),
      "[F5H-TRAJECTORY-SOURCE-LOCK] active=%s source=%s",
      request->data ? "true" : "false",
      goal_source_lock_.locked_source() == NavigationSource::ORB ? "orb" : "gt_fallback");
  }

  void OnGtPose(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    gt_pose_ = FromMessage(message->pose);
    gt_received_at_ = std::chrono::steady_clock::now();
    gt_valid_ = true;
    if (debug_architecture_telemetry_ && architecture_activity_pub_) {
      std_msgs::msg::String activity;
      std::ostringstream json;
      json << "{\"kind\":\"architecture_activity\",\"edge_id\":"
           << "\"sim_to_dron_gt\",\"interface\":\"sensor/GT/pose\","
           << "\"interface_kind\":\"topic\",\"source\":"
           << "\"navigation_state_mux\",\"namespace\":\""
           << get_namespace() << "\",\"drone_id\":" << drone_id_
           << ",\"timestamp\":" << get_clock()->now().seconds() << "}";
      activity.data = json.str();
      architecture_activity_pub_->publish(activity);
    }
  }

  void OnGtVelocity(const geometry_msgs::msg::TwistStamped::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    gt_velocity_ = message->twist;
    gt_velocity_received_at_ = std::chrono::steady_clock::now();
    gt_velocity_valid_ = true;
  }

  void OnOrbState(const NavigationState::SharedPtr raw)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool authoritative =
      raw->global_status == NavigationState::GLOBAL_STATUS_AUTHORITATIVE;
    const bool anchored = anchor_latch_.Update(raw->map_epoch, authoritative);
    const bool tracking_ok =
      raw->tracking_state == NavigationState::TRACKING_OK && raw->local_valid &&
      raw->local_continuity_valid;
    auto decision = dron_individual::DecideNavigationSource(
      tracking_ok, anchored, anchor_latch_.previously_anchored());

    if (decision.source == NavigationSource::ORB && last_source_ != NavigationSource::ORB) {
      const auto qualification = orb_qualifier_.Update(
        raw->map_epoch, orb_qualification_samples_);
      if (!qualification.qualified) {
        decision = {NavigationSource::GT_FALLBACK, FallbackReason::ORB_QUALIFYING};
      } else if (qualification.newly_qualified) {
        RCLCPP_INFO(
          get_logger(),
          "[F5H-ORB-QUALIFIED] epoch=%lu samples=%zu criterion=tracking_anchor_only",
          static_cast<unsigned long>(raw->map_epoch), qualification.consecutive_samples);
      }
    } else if (decision.source != NavigationSource::ORB) {
      orb_qualifier_.Reset();
    }

    const bool orb_candidate_ready =
      decision.source == NavigationSource::ORB && raw->velocity_valid;
    const double gt_linear_speed = std::sqrt(
      gt_velocity_.linear.x * gt_velocity_.linear.x +
      gt_velocity_.linear.y * gt_velocity_.linear.y +
      gt_velocity_.linear.z * gt_velocity_.linear.z);
    const double gt_angular_speed = std::sqrt(
      gt_velocity_.angular.x * gt_velocity_.angular.x +
      gt_velocity_.angular.y * gt_velocity_.angular.y +
      gt_velocity_.angular.z * gt_velocity_.angular.z);
    if (diagnostic_force_source_ == "shadow_gt" && !shadow_orb_activated_) {
      shadow_gate_.Update(
        orb_candidate_ready && gt_valid_ && gt_velocity_valid_,
        gt_linear_speed, gt_angular_speed, get_clock()->now().seconds(),
        shadow_settle_duration_sec_, shadow_max_linear_speed_,
        shadow_max_angular_speed_);
    }

    decision = goal_source_lock_.Apply(decision);

    // TODO FASE 6: retirar esta seleccion junto con el laboratorio temporal F5H.
    if (diagnostic_force_source_ == "gt") {
      decision = {NavigationSource::GT_FALLBACK, FallbackReason::NONE};
    } else if (diagnostic_force_source_ == "orb") {
      decision = {NavigationSource::ORB, FallbackReason::NONE};
    } else if (diagnostic_force_source_ == "shadow_gt" && !shadow_orb_activated_) {
      decision = {NavigationSource::GT_FALLBACK, FallbackReason::NONE};
    }

    RigidPose source_pose;
    if (decision.source == NavigationSource::ORB) {
      source_pose = FromMessage(raw->o_t_body);
    } else {
      if (!gt_fallback_enabled_) {
        NavigationState invalid = *raw;
        invalid.sample_sequence = output_sequence_++;
        invalid.pose_source = NavigationState::POSE_SOURCE_INVALID;
        invalid.local_valid = false;
        invalid.local_continuity_valid = false;
        invalid.global_valid = false;
        invalid.velocity_valid = false;
        invalid.global_status = NavigationState::GLOBAL_STATUS_INVALID;
        publisher_->publish(invalid);
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "[F5H-FALLBACK-DISABLED] reason=%s",
          dron_individual::FallbackReasonName(decision.fallback_reason));
        return;
      }
      const double gt_pose_age = gt_valid_ ? std::chrono::duration<double>(
        std::chrono::steady_clock::now() - gt_received_at_).count() : 1e9;
      const double gt_velocity_age = gt_velocity_valid_ ? std::chrono::duration<double>(
        std::chrono::steady_clock::now() - gt_velocity_received_at_).count() : 1e9;
      if (
        !gt_valid_ || !gt_velocity_valid_ || gt_pose_age > gt_timeout_sec_ ||
        gt_velocity_age > gt_timeout_sec_)
      {
        NavigationState invalid = *raw;
        invalid.sample_sequence = output_sequence_++;
        invalid.pose_source = NavigationState::POSE_SOURCE_INVALID;
        invalid.local_valid = false;
        invalid.local_continuity_valid = false;
        invalid.global_valid = false;
        invalid.velocity_valid = false;
        invalid.global_status = NavigationState::GLOBAL_STATUS_INVALID;
        invalid.o_t_body = geometry_msgs::msg::Pose();
        invalid.o_t_body.orientation.w = 1.0;
        invalid.w_t_body = geometry_msgs::msg::Pose();
        invalid.w_t_body.orientation.w = 1.0;
        publisher_->publish(invalid);
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "[F5H-FALLBACK-UNAVAILABLE] reason=%s gt_pose_age_sec=%.3f "
          "gt_velocity_age_sec=%.3f",
          dron_individual::FallbackReasonName(decision.fallback_reason),
          gt_pose_age, gt_velocity_age);
        return;
      }
      source_pose = gt_pose_;
    }

    if (
      diagnostic_force_source_ == "shadow_gt" && !shadow_orb_activated_ &&
      gt_valid_ && orb_candidate_ready)
    {
      if (!shadow_pose_initialized_) {
        shadow_pose_.Update(NavigationSource::GT_FALLBACK, gt_pose_);
        shadow_pose_initialized_ = true;
      }
      const RigidPose shadow_orb_pose = shadow_pose_.Update(
        NavigationSource::ORB, FromMessage(raw->o_t_body));
      const Eigen::Vector3d shadow_orb_linear = shadow_pose_.RotateVectorFromSource(
        Eigen::Vector3d(raw->velocity.linear.x, raw->velocity.linear.y, raw->velocity.linear.z));
      const Eigen::Vector3d shadow_orb_angular = shadow_pose_.RotateVectorFromSource(
        Eigen::Vector3d(raw->velocity.angular.x, raw->velocity.angular.y, raw->velocity.angular.z));
      const Eigen::Vector3d gt_linear(
        gt_velocity_.linear.x, gt_velocity_.linear.y, gt_velocity_.linear.z);
      const Eigen::Vector3d gt_angular(
        gt_velocity_.angular.x, gt_velocity_.angular.y, gt_velocity_.angular.z);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "[F5H-ORB-SHADOW] source_authority=GT orb_state_valid=true settled=%s "
        "p_error=%.6f v_error=%.6f r_error=%.6f omega_error=%.6f "
        "gt_linear_speed=%.6f gt_angular_speed=%.6f",
        shadow_gate_.ready() ? "true" : "false",
        (shadow_orb_pose.translation - last_continuous_measurement_.translation).norm(),
        (shadow_orb_linear - gt_linear).norm(),
        dron_individual::RotationDistance(
          shadow_orb_pose.rotation, last_continuous_measurement_.rotation),
        (shadow_orb_angular - gt_angular).norm(), gt_linear_speed, gt_angular_speed);
    }

    // TODO FASE 6: la alineacion de GT desaparece junto con GT_FALLBACK.
    // Update conserva el frame O del goal activo cuando ORB se pierde.
    const RigidPose continuous_pose = continuous_pose_.Update(decision.source, source_pose);
    if (
      continuous_measurement_valid_ && last_source_ != NavigationSource::INVALID &&
      decision.source != last_source_)
    {
      RCLCPP_WARN(
        get_logger(),
        "[F5H-SOURCE-CONTINUITY] previous=%s current=%s translation_jump_m=%.9f "
        "rotation_jump_rad=%.9f",
        last_source_ == NavigationSource::ORB ? "orb" : "gt_fallback",
        decision.source == NavigationSource::ORB ? "orb" : "gt_fallback",
        (continuous_pose.translation - last_continuous_measurement_.translation).norm(),
        dron_individual::RotationDistance(
          continuous_pose.rotation, last_continuous_measurement_.rotation));
    }
    last_continuous_measurement_ = continuous_pose;
    continuous_measurement_valid_ = true;

    NavigationState output = *raw;
    output.sample_sequence = output_sequence_++;
    output.pose_source = decision.source == NavigationSource::ORB ?
      NavigationState::POSE_SOURCE_ORB : NavigationState::POSE_SOURCE_GT_FALLBACK;
    output.local_valid = true;
    output.local_continuity_valid = true;
    output.o_t_body = ToMessage(continuous_pose);
    const geometry_msgs::msg::Twist source_velocity =
      decision.source == NavigationSource::ORB ? raw->velocity : gt_velocity_;
    output.velocity = geometry_msgs::msg::Twist();
    output.velocity_valid = decision.source == NavigationSource::ORB ?
      raw->velocity_valid : true;
    if (output.velocity_valid) {
      const Eigen::Vector3d linear = continuous_pose_.RotateVectorFromSource(
        Eigen::Vector3d(
          source_velocity.linear.x, source_velocity.linear.y, source_velocity.linear.z));
      const Eigen::Vector3d angular = continuous_pose_.RotateVectorFromSource(
        Eigen::Vector3d(
          source_velocity.angular.x, source_velocity.angular.y, source_velocity.angular.z));
      output.velocity.linear.x = linear.x();
      output.velocity.linear.y = linear.y();
      output.velocity.linear.z = linear.z();
      output.velocity.angular.x = angular.x();
      output.velocity.angular.y = angular.y();
      output.velocity.angular.z = angular.z();
    }
    if (
      decision.source == NavigationSource::ORB &&
      last_source_ == NavigationSource::GT_FALLBACK && gt_valid_)
    {
      diagnostic_gt_alignment_.Capture(last_continuous_measurement_, gt_pose_);
    }
    // TODO FASE 6: retirar los overrides GT diagnosticos al eliminar GT_FALLBACK.
    if (
      decision.source == NavigationSource::ORB && diagnostic_gt_alignment_.valid() &&
      gt_valid_ && gt_velocity_valid_)
    {
      if (dron_individual::UsesGtPosition(diagnostic_orb_control_mode_)) {
        const RigidPose gt_in_control = diagnostic_gt_alignment_.TransformPose(gt_pose_);
        output.o_t_body.position.x = gt_in_control.translation.x();
        output.o_t_body.position.y = gt_in_control.translation.y();
        output.o_t_body.position.z = gt_in_control.translation.z();
      }
      if (dron_individual::UsesGtVelocity(diagnostic_orb_control_mode_)) {
        const Eigen::Vector3d gt_linear = diagnostic_gt_alignment_.RotateVector(
          Eigen::Vector3d(
            gt_velocity_.linear.x, gt_velocity_.linear.y, gt_velocity_.linear.z));
        output.velocity.linear.x = gt_linear.x();
        output.velocity.linear.y = gt_linear.y();
        output.velocity.linear.z = gt_linear.z();
      }
    }
    if (
      diagnostic_force_source_ == "shadow_gt" && shadow_orb_activated_ &&
      last_source_ == NavigationSource::GT_FALLBACK && decision.source == NavigationSource::ORB)
    {
      const Eigen::Vector3d previous_linear(
        last_output_velocity_.linear.x, last_output_velocity_.linear.y,
        last_output_velocity_.linear.z);
      const Eigen::Vector3d current_linear(
        output.velocity.linear.x, output.velocity.linear.y, output.velocity.linear.z);
      const Eigen::Vector3d previous_angular(
        last_output_velocity_.angular.x, last_output_velocity_.angular.y,
        last_output_velocity_.angular.z);
      const Eigen::Vector3d current_angular(
        output.velocity.angular.x, output.velocity.angular.y, output.velocity.angular.z);
      RCLCPP_WARN(
        get_logger(),
        "[F5H-ORB-ACTIVATED] stamp=%.9f goal_boundary=true p_jump=%.9f "
        "v_jump=%.9f r_jump=%.9f omega_jump=%.9f",
        get_clock()->now().seconds(),
        (continuous_pose.translation - last_continuous_measurement_.translation).norm(),
        (current_linear - previous_linear).norm(),
        dron_individual::RotationDistance(
          continuous_pose.rotation, last_continuous_measurement_.rotation),
        (current_angular - previous_angular).norm());
    }
    last_output_velocity_ = output.velocity;
    if (decision.source == NavigationSource::GT_FALLBACK) {
      output.global_valid = false;
      output.global_status = NavigationState::GLOBAL_STATUS_INVALID;
      // TODO FASE 6: transporte explicito de GT solo para transformar goals fallback.
      output.w_t_body = ToMessage(source_pose);
    }
    if (decision.source == NavigationSource::GT_FALLBACK) {
      ++fallback_samples_;
    } else {
      ++orb_samples_;
    }
    publisher_->publish(output);

    const bool orb_authority_confirmed =
      diagnostic_force_source_ == "shadow_gt" && shadow_orb_activated_ &&
      decision.source == NavigationSource::ORB;
    if (orb_authority_confirmed && !orb_authority_confirmed_) {
      RCLCPP_WARN(
        get_logger(),
        "[F5H-ORB-AUTHORITY-CONFIRMED] stamp=%.9f source=orb output_sample=%lu",
        get_clock()->now().seconds(), static_cast<unsigned long>(output.sample_sequence));
    }
    PublishOrbAuthority(orb_authority_confirmed);
    orb_authority_confirmed_ = orb_authority_confirmed;

    if (decision.source != last_source_ ||
      decision.fallback_reason != last_fallback_reason_)
    {
      RCLCPP_WARN(
        get_logger(),
        "[F5H-POSE-SOURCE] source=%s reason=%s epoch=%lu tracking=%d "
        "anchored=%s output_sample=%lu",
        decision.source == NavigationSource::ORB ? "orb" : "gt_fallback",
        dron_individual::FallbackReasonName(decision.fallback_reason),
        static_cast<unsigned long>(raw->map_epoch), raw->tracking_state,
        anchored ? "true" : "false",
        static_cast<unsigned long>(output.sample_sequence));
      last_source_ = decision.source;
      last_fallback_reason_ = decision.fallback_reason;
    }
  }

  void LogMetrics()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const uint64_t total = orb_samples_ + fallback_samples_;
    const double fallback_ratio = total == 0 ? 0.0 :
      static_cast<double>(fallback_samples_) / static_cast<double>(total);
    RCLCPP_INFO(
      get_logger(),
      "[F5H-SOURCE-METRICS] orb_samples=%lu fallback_samples=%lu "
      "fallback_ratio=%.6f",
      static_cast<unsigned long>(orb_samples_),
      static_cast<unsigned long>(fallback_samples_), fallback_ratio);
  }

  void PublishOrbAuthority(bool confirmed)
  {
    std_msgs::msg::Bool message;
    message.data = confirmed;
    orb_authority_publisher_->publish(message);
  }

  std::mutex mutex_;
  double gt_timeout_sec_{0.5};
  bool gt_valid_{false};
  bool gt_velocity_valid_{false};
  bool gt_fallback_enabled_{false};
  std::size_t orb_qualification_samples_{20};
  RigidPose gt_pose_;
  geometry_msgs::msg::Twist gt_velocity_;
  std::chrono::steady_clock::time_point gt_received_at_;
  std::chrono::steady_clock::time_point gt_velocity_received_at_;
  uint64_t output_sequence_{0};
  uint64_t orb_samples_{0};
  uint64_t fallback_samples_{0};
  uint32_t drone_id_{0};
  bool debug_architecture_telemetry_{false};
  std::string diagnostic_force_source_{"normal"};
  dron_individual::DiagnosticOrbControlMode diagnostic_orb_control_mode_{
    dron_individual::DiagnosticOrbControlMode::NORMAL};
  double shadow_settle_duration_sec_{1.5};
  double shadow_max_linear_speed_{0.15};
  double shadow_max_angular_speed_{0.15};
  bool shadow_orb_activated_{false};
  bool orb_authority_confirmed_{false};
  bool shadow_pose_initialized_{false};
  NavigationSource last_source_{NavigationSource::INVALID};
  FallbackReason last_fallback_reason_{FallbackReason::NONE};
  dron_individual::EpochAnchorLatch anchor_latch_;
  dron_individual::OrbTransitionQualifier orb_qualifier_;
  dron_individual::GoalSourceLock goal_source_lock_;
  dron_individual::OrbShadowActivationGate shadow_gate_;
  dron_individual::ContinuousSourcePose continuous_pose_;
  dron_individual::ContinuousSourcePose shadow_pose_;
  dron_individual::DiagnosticGtControlAlignment diagnostic_gt_alignment_;
  geometry_msgs::msg::Twist last_output_velocity_;
  RigidPose last_continuous_measurement_;
  bool continuous_measurement_valid_{false};
  rclcpp::Publisher<NavigationState>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr orb_authority_publisher_;
  rclcpp::Subscription<NavigationState>::SharedPtr orb_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gt_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
    gt_velocity_subscription_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr trajectory_active_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr shadow_activation_service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr architecture_activity_pub_;
  rclcpp::TimerBase::SharedPtr metrics_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationStateMuxNode>());
  rclcpp::shutdown();
  return 0;
}
