#include "dron_individual/navigation_state_mux.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "orbslam3_msgs/msg/navigation_state.hpp"
#include "rclcpp/rclcpp.hpp"
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
    if (debug_architecture_telemetry_) {
      architecture_activity_pub_ = create_publisher<std_msgs::msg::String>(
        "/system_architecture/activity", rclcpp::QoS(64).best_effort());
    }
    publisher_ = create_publisher<NavigationState>(
      "orbslam/navigation_state", rclcpp::QoS(20).reliable());
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
    metrics_timer_ = create_wall_timer(
      std::chrono::seconds(10), std::bind(&NavigationStateMuxNode::LogMetrics, this));
  }

private:
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

    decision = goal_source_lock_.Apply(decision);

    // TODO FASE 6: retirar esta seleccion junto con el laboratorio temporal F5H.
    if (diagnostic_force_source_ == "gt") {
      decision = {NavigationSource::GT_FALLBACK, FallbackReason::NONE};
    } else if (diagnostic_force_source_ == "orb") {
      decision = {NavigationSource::ORB, FallbackReason::NONE};
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
  NavigationSource last_source_{NavigationSource::INVALID};
  FallbackReason last_fallback_reason_{FallbackReason::NONE};
  dron_individual::EpochAnchorLatch anchor_latch_;
  dron_individual::OrbTransitionQualifier orb_qualifier_;
  dron_individual::GoalSourceLock goal_source_lock_;
  dron_individual::ContinuousSourcePose continuous_pose_;
  RigidPose last_continuous_measurement_;
  bool continuous_measurement_valid_{false};
  rclcpp::Publisher<NavigationState>::SharedPtr publisher_;
  rclcpp::Subscription<NavigationState>::SharedPtr orb_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gt_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
    gt_velocity_subscription_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr trajectory_active_service_;
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
