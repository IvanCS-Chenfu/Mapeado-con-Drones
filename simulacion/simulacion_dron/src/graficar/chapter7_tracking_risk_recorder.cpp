#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <string>
#include <stdexcept>

#include "dron_individual/action/tray_action.hpp"
#include "mission_msgs/msg/visual_risk_event.hpp"
#include "orbslam3_msgs/msg/navigation_state.hpp"
#include "orbslam3_msgs/msg/visual_tracking_evidence.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

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
}

class Chapter7TrackingRiskRecorder final : public rclcpp::Node
{
public:
  Chapter7TrackingRiskRecorder()
  : Node("chapter7_tracking_risk_recorder")
  {
    const auto output_dir = declare_parameter<std::string>("output_dir", "/tmp/chapter7_tracking_risk");
    const auto drone_namespace = declare_parameter<std::string>("drone_namespace", "dron_1");
    std::filesystem::create_directories(output_dir);
    stream_.open(std::filesystem::path(output_dir) / "tracking_risk_timeline_raw.csv");
    if (!stream_) {
      throw std::runtime_error("No se pudo abrir tracking_risk_timeline_raw.csv");
    }
    stream_ << "time,record_kind,tracking_state,tracking_inliers,left_inliers,right_inliers,top_inliers,bottom_inliers,risk_mask,trajectory_active,stop_started,stop_completed,reorientation_started,reorientation_completed,yaw,yaw_reference,pose_source,local_valid,local_continuity_valid,global_valid,map_epoch,frame_id,event_type,event_success,event_trajectory_id,event_detail\n";
    stream_ << std::fixed << std::setprecision(9);

    const std::string prefix = "/" + drone_namespace;
    navigation_subscription_ = create_subscription<orbslam3_msgs::msg::NavigationState>(
      prefix + "/orbslam/navigation_state", rclcpp::QoS(100).reliable(),
      std::bind(&Chapter7TrackingRiskRecorder::OnNavigation, this, std::placeholders::_1));
    evidence_subscription_ = create_subscription<orbslam3_msgs::msg::VisualTrackingEvidence>(
      prefix + "/orbslam/visual_tracking_evidence", rclcpp::QoS(100).best_effort(),
      std::bind(&Chapter7TrackingRiskRecorder::OnEvidence, this, std::placeholders::_1));
    trajectory_active_subscription_ = create_subscription<std_msgs::msg::Bool>(
      prefix + "/control/trajectory_active", rclcpp::QoS(10).transient_local(),
      std::bind(&Chapter7TrackingRiskRecorder::OnTrajectoryActive, this, std::placeholders::_1));
    trajectory_feedback_subscription_ = create_subscription<dron_individual::action::TrayAction_FeedbackMessage>(
      prefix + "/AccionTrayectoria/_action/feedback", rclcpp::QoS(100).best_effort(),
      std::bind(&Chapter7TrackingRiskRecorder::OnTrajectoryFeedback, this, std::placeholders::_1));
    risk_subscription_ = create_subscription<mission_msgs::msg::VisualRiskEvent>(
      "/mission/visual_risk_events", rclcpp::QoS(20).reliable(),
      std::bind(&Chapter7TrackingRiskRecorder::OnRisk, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "[C7-TRACKING-RISK-RECORDER-READY] drone=%s output_dir=%s",
      drone_namespace.c_str(), output_dir.c_str());
  }

private:
  struct NavigationSnapshot
  {
    bool received = false;
    int tracking_state = 0;
    int pose_source = 0;
    bool local_valid = false;
    bool local_continuity_valid = false;
    bool global_valid = false;
    std::uint64_t map_epoch = 0U;
    double yaw = NAN;
  };

  void OnNavigation(const orbslam3_msgs::msg::NavigationState::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    navigation_.received = true;
    navigation_.tracking_state = message->tracking_state;
    navigation_.pose_source = message->pose_source;
    navigation_.local_valid = message->local_valid;
    navigation_.local_continuity_valid = message->local_continuity_valid;
    navigation_.global_valid = message->global_valid;
    navigation_.map_epoch = message->map_epoch;
    navigation_.yaw = YawFromPose(message->o_t_body);
  }

  void OnTrajectoryActive(const std_msgs::msg::Bool::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    trajectory_active_ = message->data;
  }

  void OnTrajectoryFeedback(const dron_individual::action::TrayAction_FeedbackMessage::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto & values = message->feedback.yaw.data;
    if (!values.empty()) {
      yaw_reference_ = values.front();
    }
  }

  void OnEvidence(const orbslam3_msgs::msg::VisualTrackingEvidence::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    WriteRow(StampSeconds(message->header.stamp), "evidence", message->tracking_state,
      message->tracking_inlier_count, message->left_directional_inlier_count,
      message->right_directional_inlier_count, message->top_directional_inlier_count,
      message->bottom_directional_inlier_count, 0U, message->frame_id, 0U, false, "", "");
  }

  void OnRisk(const mission_msgs::msg::VisualRiskEvent::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool stop_started = message->event_type == mission_msgs::msg::VisualRiskEvent::EVENT_STOP_STARTED;
    const bool stop_completed = message->event_type == mission_msgs::msg::VisualRiskEvent::EVENT_STOP_COMPLETED;
    const bool reorientation_started = message->event_type == mission_msgs::msg::VisualRiskEvent::EVENT_REORIENTATION_STARTED;
    const bool reorientation_completed = message->event_type == mission_msgs::msg::VisualRiskEvent::EVENT_REORIENTATION_COMPLETED;
    WriteRow(StampSeconds(message->header.stamp), "risk_event", navigation_.tracking_state,
      0U, 0U, 0U, 0U, 0U, message->risk_mask, message->frame_id, message->event_type,
      message->success, message->trajectory_id, message->detail, stop_started, stop_completed,
      reorientation_started, reorientation_completed);
  }

  void WriteRow(double time, const char * kind, int tracking_state, std::uint32_t tracking_inliers,
    std::uint32_t left, std::uint32_t right, std::uint32_t top, std::uint32_t bottom,
    std::uint8_t risk_mask, std::uint64_t frame_id, std::uint8_t event_type, bool event_success,
    const std::string & trajectory_id, const std::string & detail, bool stop_started = false,
    bool stop_completed = false, bool reorientation_started = false, bool reorientation_completed = false)
  {
    stream_ << time << ',' << kind << ',' << tracking_state << ',' << tracking_inliers << ','
            << left << ',' << right << ',' << top << ',' << bottom << ','
            << static_cast<unsigned>(risk_mask) << ',' << (trajectory_active_ ? 1 : 0) << ','
            << (stop_started ? 1 : 0) << ',' << (stop_completed ? 1 : 0) << ','
            << (reorientation_started ? 1 : 0) << ',' << (reorientation_completed ? 1 : 0) << ','
            << navigation_.yaw << ',' << yaw_reference_ << ',' << navigation_.pose_source << ','
            << (navigation_.local_valid ? 1 : 0) << ',' << (navigation_.local_continuity_valid ? 1 : 0) << ','
            << (navigation_.global_valid ? 1 : 0) << ',' << navigation_.map_epoch << ',' << frame_id << ','
            << static_cast<unsigned>(event_type) << ',' << (event_success ? 1 : 0) << ','
            << '"' << trajectory_id << '"' << ',' << '"' << detail << '"' << '\n';
  }

  std::mutex mutex_;
  std::ofstream stream_;
  NavigationSnapshot navigation_;
  bool trajectory_active_ = false;
  double yaw_reference_ = NAN;
  rclcpp::Subscription<orbslam3_msgs::msg::NavigationState>::SharedPtr navigation_subscription_;
  rclcpp::Subscription<orbslam3_msgs::msg::VisualTrackingEvidence>::SharedPtr evidence_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trajectory_active_subscription_;
  rclcpp::Subscription<dron_individual::action::TrayAction_FeedbackMessage>::SharedPtr trajectory_feedback_subscription_;
  rclcpp::Subscription<mission_msgs::msg::VisualRiskEvent>::SharedPtr risk_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Chapter7TrackingRiskRecorder>());
  rclcpp::shutdown();
  return 0;
}
