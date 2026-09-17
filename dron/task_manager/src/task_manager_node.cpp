#include <dron_individual/action/tray_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mission_msgs/action/execute_trajectory.hpp>
#include <mission_msgs/msg/drone_registration.hpp>
#include <mission_msgs/msg/fiducial_primary_observation.hpp>
#include <mission_msgs/msg/task_report.hpp>
#include <mission_msgs/msg/task_state_array.hpp>
#include <mission_msgs/msg/safety_event.hpp>
#include <mission_msgs/msg/trajectory_plan.hpp>
#include <mission_msgs/msg/visual_risk_event.hpp>
#include <mission_msgs/srv/capture_depth.hpp>
#include <mission_msgs/srv/inspect_facade.hpp>
#include <mission_msgs/srv/register_drone.hpp>
#include <mission_msgs/srv/report_autonomous_result.hpp>
#include <mission_msgs/srv/submit_autonomous_command.hpp>
#include <orbslam3_msgs/msg/navigation_state.hpp>
#include <orbslam3_msgs/msg/visual_tracking_evidence.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <task_manager_lib/registration_profile.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

using namespace std::chrono_literals;

class TaskManagerNode final : public rclcpp::Node
{
public:
  using ExecuteTrajectory = mission_msgs::action::ExecuteTrajectory;
  using GoalHandleExecuteTrajectory = rclcpp_action::ServerGoalHandle<ExecuteTrajectory>;
  using TrayAction = dron_individual::action::TrayAction;
  using ExecutionFinish = std::function<void(bool, std::uint8_t, const std::string &)>;
  using CaptureDepth = mission_msgs::srv::CaptureDepth;
  using InspectFacade = mission_msgs::srv::InspectFacade;
  using ReportAutonomousResult = mission_msgs::srv::ReportAutonomousResult;
  using SubmitAutonomousCommand = mission_msgs::srv::SubmitAutonomousCommand;

  struct LocalVisualReorientation
  {
    std::uint64_t frame_id = 0U;
    std::uint8_t risk_mask = mission_msgs::msg::VisualRiskEvent::RISK_NONE;
    std::string source_trajectory_id;
    double yaw_rad = 0.0;
    double camera_pitch_rad = 0.0;
  };

  struct FacadeOrientationResult
  {
    double yaw_rad = 0.0;
    double camera_pitch_rad = 0.0;
    bool normal_accepted = false;
    std::string reason;
  };

  struct InspectionFrameCandidate
  {
    std::uint64_t frame_id = 0U;
    double stamp_sec = 0.0;
    double yaw_rad = 0.0;
    double camera_pitch_rad = 0.0;
  };

  struct QueuedAutonomousCommand
  {
    std::string mission_id;
    std::uint32_t drone_id = 0U;
    std::string task_id;
    std::string workflow_id;
    std::string command_id;
    std::uint64_t map_epoch = 0U;
    std::uint64_t map_revision = 0U;
    std::uint8_t command_type = 0U;
    geometry_msgs::msg::Point visual_target;
    mission_msgs::msg::TrajectoryPlan trajectory_plan;
    bool capture_after_command = false;
  };

  TaskManagerNode()
  : Node("task_manager")
  {
    task_manager_lib::RegistrationProfile profile;
    profile.drone_id = static_cast<std::uint32_t>(declare_parameter("drone_id", 0));
    profile.length_m = declare_parameter("dimensions.length_m", 0.60);
    profile.width_m = declare_parameter("dimensions.width_m", 0.60);
    profile.height_m = declare_parameter("dimensions.height_m", 0.30);
    profile.vehicle_profile = declare_parameter("vehicle_profile", "x500_depth");
    profile.protocol_version = static_cast<std::uint32_t>(
      declare_parameter("protocol_version", 1));
    profile.generator_id = declare_parameter("trajectory_generator_id", "lib_tray");
    profile.generator_version = static_cast<std::uint32_t>(
      declare_parameter("trajectory_generator_version", 1));
    profile.capability_mask = static_cast<std::uint64_t>(
      declare_parameter("capability_mask", 11));
    architecture_events_enabled_ =
      declare_parameter("system_architecture_events_enabled", false);
    pose_source_ = declare_parameter("phase5_navigation_source", "orb");
    visual_risk_enabled_ = declare_parameter("visual_risk_enabled", false);
    visual_risk_persistence_frames_ = static_cast<std::uint32_t>(
      declare_parameter("visual_risk_persistence_frames", 3));
    visual_risk_directional_max_inliers_ = static_cast<std::uint32_t>(
      declare_parameter("visual_risk_directional_max_inliers", 3));
    visual_risk_motion_min_mps_ = declare_parameter("visual_risk_motion_min_mps", 0.05);
    visual_risk_reorientation_grace_sec_ = declare_parameter(
      "visual_risk_reorientation_grace_sec", 6.0);
    visual_risk_reorientation_step_deg_ = declare_parameter(
      "visual_risk_reorientation_step_deg", 25.0);
    facade_normal_min_support_ = static_cast<std::uint32_t>(
      declare_parameter("facade_normal_min_support", 40));
    facade_normal_min_confidence_ = declare_parameter(
      "facade_normal_min_confidence", 0.8);
    facade_normal_max_yaw_change_deg_ = declare_parameter(
      "facade_normal_max_yaw_change_deg", 25.0);
    inspection_max_yaw_rate_deg_s_ = declare_parameter(
      "inspection_max_yaw_rate_deg_s", 5.0);
    inspection_candidate_min_tracking_inliers_ = static_cast<std::uint32_t>(
      declare_parameter("inspection_candidate_min_tracking_inliers", 20));
    inspection_candidate_frame_capacity_ = static_cast<std::size_t>(
      declare_parameter("inspection_candidate_frame_capacity", 16));
    inspection_candidate_min_interval_sec_ = declare_parameter(
      "inspection_candidate_min_interval_sec", 0.5);
    inspection_candidate_min_orientation_delta_deg_ = declare_parameter(
      "inspection_candidate_min_orientation_delta_deg", 2.5);
    inspection_depth_min_confidence_ = declare_parameter(
      "inspection_depth_min_confidence", 0.25);
    inspection_depth_min_support_points_ = static_cast<std::uint32_t>(
      declare_parameter("inspection_depth_min_support_points", 20));
    if (visual_risk_reorientation_grace_sec_ < 0.0) {
      throw std::invalid_argument("visual_risk_reorientation_grace_sec no puede ser negativo");
    }
    if (visual_risk_reorientation_step_deg_ <= 0.0 ||
      visual_risk_reorientation_step_deg_ > 70.0)
    {
      throw std::invalid_argument("visual_risk_reorientation_step_deg debe estar en (0, 70]");
    }
    if (facade_normal_min_support_ == 0U || facade_normal_min_confidence_ < 0.0 ||
      facade_normal_min_confidence_ > 1.0 || facade_normal_max_yaw_change_deg_ <= 0.0 ||
      facade_normal_max_yaw_change_deg_ > 90.0 || inspection_max_yaw_rate_deg_s_ <= 0.0 ||
      inspection_candidate_min_tracking_inliers_ == 0U ||
      inspection_candidate_frame_capacity_ == 0U ||
      inspection_candidate_min_interval_sec_ < 0.0 ||
      inspection_candidate_min_orientation_delta_deg_ < 0.0 ||
      inspection_depth_min_confidence_ < 0.0 ||
      inspection_depth_min_confidence_ > 1.0 || inspection_depth_min_support_points_ == 0U)
    {
      throw std::invalid_argument("parametros de orientacion de fachada no validos");
    }
    registration_ = task_manager_lib::BuildRegistration(profile);
    inspect_service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    capture_depth_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    autonomous_command_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_ = create_client<mission_msgs::srv::RegisterDrone>("/mission/register_drone");
    capture_depth_client_ = create_client<CaptureDepth>(
      "orbslam/capture_depth", rclcpp::ServicesQoS(), capture_depth_group_);
    inspect_facade_service_ = create_service<InspectFacade>(
      "inspect_facade",
      std::bind(
        &TaskManagerNode::HandleInspectFacade, this, std::placeholders::_1,
        std::placeholders::_2),
      rclcpp::ServicesQoS(), inspect_service_group_);
    autonomous_command_service_ = create_service<SubmitAutonomousCommand>(
      "autonomous_command",
      std::bind(
        &TaskManagerNode::HandleAutonomousCommand, this, std::placeholders::_1,
        std::placeholders::_2),
      rclcpp::ServicesQoS(), autonomous_command_group_);
    autonomous_result_client_ = create_client<ReportAutonomousResult>(
      "/mission/report_autonomous_result", rclcpp::ServicesQoS(), autonomous_command_group_);
    task_subscription_ = create_subscription<mission_msgs::msg::TaskStateArray>(
      "/mission/task_states", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&TaskManagerNode::HandleTaskStates, this, std::placeholders::_1));
    report_publisher_ = create_publisher<mission_msgs::msg::TaskReport>(
      "/mission/task_reports", rclcpp::QoS(50).reliable());
    visual_risk_publisher_ = create_publisher<mission_msgs::msg::VisualRiskEvent>(
      "/mission/visual_risk_events", rclcpp::QoS(20).reliable());
    safety_event_publisher_ = create_publisher<mission_msgs::msg::SafetyEvent>(
      "/mission/safety_events", rclcpp::QoS(20).reliable());
    safety_event_subscription_ = create_subscription<mission_msgs::msg::SafetyEvent>(
      "/mission/safety_events", rclcpp::QoS(20).reliable(),
      std::bind(&TaskManagerNode::HandleSafetyEvent, this, std::placeholders::_1));
    visual_evidence_subscription_ = create_subscription<orbslam3_msgs::msg::VisualTrackingEvidence>(
      "orbslam/visual_tracking_evidence", rclcpp::QoS(20).best_effort(),
      std::bind(&TaskManagerNode::HandleVisualEvidence, this, std::placeholders::_1));
    navigation_subscription_ = create_subscription<orbslam3_msgs::msg::NavigationState>(
      "orbslam/navigation_state", rclcpp::QoS(20).reliable(),
      std::bind(&TaskManagerNode::HandleNavigation, this, std::placeholders::_1));
    trajectory_active_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "control/trajectory_active", rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::ConstSharedPtr message) {
        physical_trajectory_active_.store(message && message->data);
      });
    fiducial_primary_subscription_ = create_subscription<mission_msgs::msg::FiducialPrimaryObservation>(
      "/mission/fiducial_primary_observations", rclcpp::QoS(64).reliable(),
      std::bind(&TaskManagerNode::HandleFiducialPrimaryObservation, this, std::placeholders::_1));
    tray_client_ = rclcpp_action::create_client<TrayAction>(this, "AccionTrayectoria");
    execute_trajectory_server_ = rclcpp_action::create_server<ExecuteTrajectory>(
      this, "execute_trajectory",
      std::bind(&TaskManagerNode::HandleExecuteGoal, this, std::placeholders::_1,
      std::placeholders::_2),
      std::bind(&TaskManagerNode::HandleExecuteCancel, this, std::placeholders::_1),
      std::bind(&TaskManagerNode::HandleExecuteAccepted, this, std::placeholders::_1));
    autonomous_execute_client_ = rclcpp_action::create_client<ExecuteTrajectory>(
      this, "execute_trajectory");
    autonomous_runtime_timer_ = create_wall_timer(
      20ms, std::bind(&TaskManagerNode::RunAutonomousCommandWorker, this),
      autonomous_command_group_);
    if (architecture_events_enabled_) {
      architecture_publisher_ = create_publisher<std_msgs::msg::String>(
        "/system_architecture/activity", rclcpp::QoS(100).reliable());
    }
    retry_timer_ = create_wall_timer(1s, [this]() {TryRegister();});
  }

private:
  void TryRegister()
  {
    if (registered_ || request_pending_ || !client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<mission_msgs::srv::RegisterDrone::Request>();
    request->registration = registration_;
    request_pending_ = true;
    client_->async_send_request(
      request,
      [this](rclcpp::Client<mission_msgs::srv::RegisterDrone>::SharedFuture future)
      {
        request_pending_ = false;
        const auto response = future.get();
        registered_ = response->accepted;
        RCLCPP_INFO(
          get_logger(),
          "[F6C-TASK-MANAGER] drone=%u accepted=%s mission=%s revision=%lu reason=%s",
          registration_.drone_id, response->accepted ? "true" : "false",
          response->mission_id.c_str(), response->config_revision, response->reason.c_str());
        if (registered_) {
          retry_timer_->cancel();
          PublishArchitecture();
        }
      });
  }

  void PublishArchitecture()
  {
    if (!architecture_publisher_) {
      return;
    }
    std_msgs::msg::String message;
    std::ostringstream json;
    json << "{\"kind\":\"architecture_activity\","
         << "\"edge_id\":\"task_manager_to_task_server\","
         << "\"source\":\"task_manager\","
         << "\"interface\":\"/mission/register_drone\","
         << "\"interface_kind\":\"service\",\"timestamp\":" << now().seconds()
         << ",\"drone_id\":" << registration_.drone_id
         << ",\"detail\":\"REGISTERED_DRONE_" << registration_.drone_id << "\"}";
    message.data = json.str();
    architecture_publisher_->publish(message);
  }

  void PublishCaptureDepthArchitecture()
  {
    if (!architecture_publisher_) {
      return;
    }
    std_msgs::msg::String message;
    std::ostringstream json;
    json << "{\"kind\":\"architecture_activity\","
         << "\"edge_id\":\"task_manager_to_orbslam_capture_depth\","
         << "\"source\":\"task_manager\","
         << "\"interface\":\"/dron_X/orbslam/capture_depth\","
         << "\"interface_kind\":\"service\",\"timestamp\":" << now().seconds()
         << ",\"drone_id\":" << registration_.drone_id
         << ",\"detail\":\"CAPTURE_DEPTH_REQUEST\"}";
    message.data = json.str();
    architecture_publisher_->publish(message);
  }

  void HandleTaskStates(mission_msgs::msg::TaskStateArray::ConstSharedPtr states)
  {
    if (!states || !registered_) {
      return;
    }
    for (const auto & task : states->tasks) {
      if (task.assigned_drone_id == registration_.drone_id) {
        known_tasks_[task.task_id] = task;
        if (task.state == mission_msgs::msg::TaskState::RUNNING) {
          active_task_id_ = task.task_id;
        }
      }
      if (task.assigned_drone_id != registration_.drone_id ||
        task.state != mission_msgs::msg::TaskState::ASSIGNED ||
        (task.task_id == last_reported_task_id_ &&
        task.state_revision == last_reported_task_revision_))
      {
        continue;
      }
      mission_msgs::msg::TaskReport report;
      report.header.stamp = now();
      report.mission_id = states->mission_id;
      report.config_revision = states->config_revision;
      report.task_id = task.task_id;
      report.drone_id = registration_.drone_id;
      report.expected_state_revision = task.state_revision;
      report.state = mission_msgs::msg::TaskState::ASSIGNED;
      report.progress = 0.0F;
      report.progress_known = false;
      report.detail = "Aceptada localmente; ejecutable mediante ExecuteTrajectory F6I";
      report_publisher_->publish(report);
      last_reported_task_id_ = task.task_id;
      last_reported_task_revision_ = task.state_revision;
      PublishTaskArchitecture();
      RCLCPP_INFO(
        get_logger(), "[F6F-TASK-ACCEPT] drone=%u task=%s authority=%s",
        registration_.drone_id, task.task_id.c_str(), pose_source_.c_str());
      return;
    }
  }

  static bool IsSupportedAutonomousCommand(std::uint8_t command_type)
  {
    return command_type == SubmitAutonomousCommand::Request::COMMAND_MOVE_AND_CAPTURE ||
           command_type == SubmitAutonomousCommand::Request::COMMAND_LOOK_AND_CAPTURE ||
           command_type == SubmitAutonomousCommand::Request::COMMAND_MOVE_AND_WATCH_FIDUCIAL;
  }

  void HandleAutonomousCommand(
    const std::shared_ptr<SubmitAutonomousCommand::Request> request,
    std::shared_ptr<SubmitAutonomousCommand::Response> response)
  {
    if (!request || request->drone_id != registration_.drone_id || request->mission_id.empty() ||
      request->task_id.empty() || request->workflow_id.empty() || request->command_id.empty() ||
      !IsSupportedAutonomousCommand(request->command_type))
    {
      response->reason = "invalid_autonomous_command";
      return;
    }
    std::lock_guard<std::mutex> lock(autonomous_command_mutex_);
    if (accepted_autonomous_command_ids_.count(request->command_id) != 0U) {
      response->accepted = true;
      response->duplicate = true;
      response->reason = "duplicate_accepted";
      return;
    }
    if (autonomous_command_active_ || !autonomous_command_queue_.empty()) {
      response->reason = "normal_command_pending";
      return;
    }
    QueuedAutonomousCommand command;
    command.mission_id = request->mission_id;
    command.drone_id = request->drone_id;
    command.task_id = request->task_id;
    command.workflow_id = request->workflow_id;
    command.command_id = request->command_id;
    command.map_epoch = request->map_epoch;
    command.map_revision = request->map_revision;
    command.command_type = request->command_type;
    command.visual_target = request->visual_target;
    command.trajectory_plan = request->trajectory_plan;
    command.capture_after_command = request->capture_after_command;
    autonomous_command_queue_.push_back(std::move(command));
    accepted_autonomous_command_ids_.insert(request->command_id);
    response->accepted = true;
    response->reason = "accepted_queued";
    RCLCPP_INFO(
      get_logger(),
      "[F6C-AUTONOMOUS-COMMAND-ACCEPTED] drone=%u command=%s workflow=%s type=%u queue_size=%zu",
      registration_.drone_id, request->command_id.c_str(), request->workflow_id.c_str(),
      request->command_type, autonomous_command_queue_.size());
  }

  void HandleFiducialPrimaryObservation(
    mission_msgs::msg::FiducialPrimaryObservation::ConstSharedPtr event)
  {
    if (!event || event->drone_id != registration_.drone_id) {
      return;
    }
    std::lock_guard<std::mutex> lock(autonomous_command_mutex_);
    if (active_autonomous_command_.has_value() &&
      active_autonomous_command_->command_type ==
      SubmitAutonomousCommand::Request::COMMAND_MOVE_AND_WATCH_FIDUCIAL &&
      active_autonomous_command_->map_epoch == event->map_epoch)
    {
      active_autonomous_fiducial_seen_ = true;
    }
  }

  static geometry_msgs::msg::Pose NavigationPose(
    const orbslam3_msgs::msg::NavigationState & state)
  {
    return state.global_valid ? state.w_t_body : state.o_t_body;
  }

  bool CurrentNavigationPose(geometry_msgs::msg::Pose * pose, std::string * reason) const
  {
    std::lock_guard<std::mutex> lock(visual_risk_mutex_);
    if (!navigation_state_received_) {
      if (reason) {
        *reason = "navigation_state_unavailable";
      }
      return false;
    }
    if (pose) {
      *pose = NavigationPose(last_navigation_state_);
    }
    return true;
  }

  void ReportAutonomousTerminal(
    const QueuedAutonomousCommand & command, std::uint8_t result_status,
    const geometry_msgs::msg::Pose & final_pose,
    const std::vector<mission_msgs::msg::DenseKFObservation> & observations,
    bool fiducial_seen, const std::string & reason)
  {
    auto request = std::make_shared<ReportAutonomousResult::Request>();
    request->mission_id = command.mission_id;
    request->drone_id = command.drone_id;
    request->task_id = command.task_id;
    request->workflow_id = command.workflow_id;
    request->command_id = command.command_id;
    request->map_epoch = command.map_epoch;
    request->map_revision = command.map_revision;
    request->command_type = command.command_type;
    request->result_status = result_status;
    request->final_pose = final_pose;
    request->depth_observations = observations;
    request->fiducial_seen = fiducial_seen;
    request->reason = reason;
    if (!autonomous_result_client_->service_is_ready()) {
      RCLCPP_ERROR(
        get_logger(), "[F6D-AUTONOMOUS-RESULT-DROPPED] drone=%u command=%s reason=server_unavailable",
        registration_.drone_id, command.command_id.c_str());
      return;
    }
    autonomous_result_client_->async_send_request(
      request,
      [this, command_id = command.command_id](rclcpp::Client<ReportAutonomousResult>::SharedFuture future) {
        const auto response = future.get();
        RCLCPP_INFO(
          get_logger(), "[F6D-AUTONOMOUS-RESULT-ACK] drone=%u command=%s accepted=%s duplicate=%s reason=%s",
          registration_.drone_id, command_id.c_str(), response->accepted ? "true" : "false",
          response->duplicate ? "true" : "false", response->reason.c_str());
      });
  }

  void RunAutonomousCommandWorker()
  {
    QueuedAutonomousCommand command;
    {
      std::lock_guard<std::mutex> lock(autonomous_command_mutex_);
      if (autonomous_command_active_ || autonomous_command_queue_.empty()) {
        return;
      }
      command = std::move(autonomous_command_queue_.front());
      autonomous_command_queue_.pop_front();
      autonomous_command_active_ = true;
      active_autonomous_command_ = command;
      active_autonomous_fiducial_seen_ = false;
      active_autonomous_tracking_risk_ = false;
    }
    std::thread([this, command]() {ExecuteAutonomousCommand(command);}).detach();
  }

  bool WaitForVisualRecovery()
  {
    const auto deadline = std::chrono::steady_clock::now() + 20s;
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(execution_mutex_);
        if (!local_visual_stop_active_ && !local_visual_reorientation_active_ &&
          !local_visual_reorientation_.has_value())
        {
          return true;
        }
      }
      std::this_thread::sleep_for(20ms);
    }
    return false;
  }

  void ExecuteAutonomousCommand(const QueuedAutonomousCommand & command)
  {
    geometry_msgs::msg::Pose final_pose;
    std::string terminal_reason;
    bool command_success = false;
    bool tracking_risk = false;
    bool fiducial_seen = false;
    std::vector<mission_msgs::msg::DenseKFObservation> observations;

    if (!CurrentNavigationPose(&final_pose, &terminal_reason)) {
      terminal_reason = "autonomous_" + terminal_reason;
    } else if (command.command_type ==
      SubmitAutonomousCommand::Request::COMMAND_LOOK_AND_CAPTURE)
    {
      const double target_yaw = std::atan2(
        command.visual_target.y - final_pose.position.y,
        command.visual_target.x - final_pose.position.x);
      std::string orientation_reason;
      command_success = ExecuteInspectionOrientation(
        target_yaw, applied_camera_pitch_rad_, command.command_id, orientation_reason);
      terminal_reason = command_success ? "look_completed" : orientation_reason;
    } else if (!autonomous_execute_client_->wait_for_action_server(2s)) {
      terminal_reason = "execute_trajectory_unavailable";
    } else {
      ExecuteTrajectory::Goal goal;
      goal.plan = command.trajectory_plan;
      goal.stop_at_current_pose = false;
      const auto goal_handle = autonomous_execute_client_->async_send_goal(goal).get();
      if (!goal_handle) {
        terminal_reason = "execute_trajectory_rejected";
      } else {
        const auto result = autonomous_execute_client_->async_get_result(goal_handle).get();
        command_success = result.code == rclcpp_action::ResultCode::SUCCEEDED &&
          result.result && result.result->success;
        terminal_reason = result.result ? result.result->reason : "execute_trajectory_missing_result";
      }
    }

    {
      std::lock_guard<std::mutex> lock(autonomous_command_mutex_);
      tracking_risk = active_autonomous_tracking_risk_;
      fiducial_seen = active_autonomous_fiducial_seen_;
    }
    if (tracking_risk) {
      WaitForVisualRecovery();
    }
    if ((command.capture_after_command || tracking_risk) &&
      command.command_type != SubmitAutonomousCommand::Request::COMMAND_MOVE_AND_WATCH_FIDUCIAL)
    {
      mission_msgs::msg::DenseKFObservation observation;
      std::string depth_reason;
      if (CaptureDepthObservation(0U, false, tracking_risk, 0U, observation, depth_reason)) {
        observations.push_back(std::move(observation));
      } else {
        terminal_reason += ";depth=" + depth_reason;
      }
    }
    CurrentNavigationPose(&final_pose, nullptr);
    const auto result_status = tracking_risk ?
      ReportAutonomousResult::Request::RESULT_ABORTED :
      (command_success ? ReportAutonomousResult::Request::RESULT_COMPLETED :
      (observations.empty() && command.capture_after_command ?
      ReportAutonomousResult::Request::RESULT_NO_USABLE_DEPTH :
      ReportAutonomousResult::Request::RESULT_REJECTED));
    ReportAutonomousTerminal(
      command, result_status, final_pose, observations, fiducial_seen, terminal_reason);
    {
      std::lock_guard<std::mutex> lock(autonomous_command_mutex_);
      active_autonomous_command_.reset();
      active_autonomous_fiducial_seen_ = false;
      active_autonomous_tracking_risk_ = false;
      autonomous_command_active_ = false;
    }
    RCLCPP_INFO(
      get_logger(), "[F6D-AUTONOMOUS-COMMAND-FINAL] drone=%u command=%s status=%u depth=%zu fiducial_seen=%s reason=%s",
      registration_.drone_id, command.command_id.c_str(), result_status, observations.size(),
      fiducial_seen ? "true" : "false", terminal_reason.c_str());
  }

  rclcpp_action::GoalResponse HandleExecuteGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteTrajectory::Goal> goal)
  {
    if (!goal || !registered_) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    const auto & plan = goal->plan;
    const auto task = known_tasks_.find(plan.task_id);
    if (plan.drone_id != registration_.drone_id || plan.trajectory_id.empty() ||
      (!goal->stop_at_current_pose && plan.waypoints.size() < 2U) ||
      task == known_tasks_.end() ||
      (task->second.state != mission_msgs::msg::TaskState::RUNNING &&
      task->second.state != mission_msgs::msg::TaskState::ASSIGNED))
    {
      RCLCPP_WARN(
        get_logger(), "[F6I-TASK-REJECT] drone=%u task=%s trajectory_id=%s",
        registration_.drone_id, plan.task_id.c_str(), plan.trajectory_id.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse HandleExecuteCancel(
    const std::shared_ptr<GoalHandleExecuteTrajectory> &)
  {
    RCLCPP_INFO(
      get_logger(), "[F6I-TASK-CANCEL-REJECTED] drone=%u reason=use_stop_command",
      registration_.drone_id);
    return rclcpp_action::CancelResponse::REJECT;
  }

  void HandleExecuteAccepted(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
  {
    const auto generation = execution_generation_.fetch_add(1U, std::memory_order_acq_rel) + 1U;
    std::thread(
      [this, goal_handle, generation]() {
        if (goal_handle->get_goal()->stop_at_current_pose) {
          ExecuteStopCommand(goal_handle, generation);
        } else {
          ExecuteTrajectoryPlan(goal_handle, generation);
        }
      }).detach();
  }

  void HandleNavigation(orbslam3_msgs::msg::NavigationState::ConstSharedPtr state)
  {
    if (!state) {
      return;
    }
    std::lock_guard<std::mutex> lock(visual_risk_mutex_);
    last_navigation_state_ = *state;
    navigation_state_received_ = true;
  }

  bool PhysicalTrajectoryActive() const
  {
    return physical_trajectory_active_.load();
  }

  static double YawFromPose(const geometry_msgs::msg::Pose & pose)
  {
    const auto & q = pose.orientation;
    return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  static double NormalizeAngle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  static double ShortestAngularDelta(double from, double to)
  {
    return NormalizeAngle(to - from);
  }

  static double AxialPlaneAngle(double angle)
  {
    angle = NormalizeAngle(angle);
    if (angle > M_PI_2) {
      angle -= M_PI;
    } else if (angle < -M_PI_2) {
      angle += M_PI;
    }
    return angle;
  }

  static geometry_msgs::msg::Vector3 RotateVectorInverse(
    const geometry_msgs::msg::Quaternion & q, const geometry_msgs::msg::Vector3 & vector)
  {
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    const double x = norm > 1e-12 ? -q.x / norm : 0.0;
    const double y = norm > 1e-12 ? -q.y / norm : 0.0;
    const double z = norm > 1e-12 ? -q.z / norm : 0.0;
    const double w = norm > 1e-12 ? q.w / norm : 1.0;
    const double tx = 2.0 * (y * vector.z - z * vector.y);
    const double ty = 2.0 * (z * vector.x - x * vector.z);
    const double tz = 2.0 * (x * vector.y - y * vector.x);
    geometry_msgs::msg::Vector3 result;
    result.x = vector.x + w * tx + (y * tz - z * ty);
    result.y = vector.y + w * ty + (z * tx - x * tz);
    result.z = vector.z + w * tz + (x * ty - y * tx);
    return result;
  }

  FacadeOrientationResult FacadeOrientation(
    const mission_msgs::msg::DenseKFObservation & observation,
    const orbslam3_msgs::msg::NavigationState & state) const
  {
    const auto & pose = state.global_valid ? state.w_t_body : state.o_t_body;
    const double current_yaw = YawFromPose(pose);
    double current_pitch = 0.0;
    {
      std::lock_guard<std::mutex> lock(visual_risk_mutex_);
      current_pitch = applied_camera_pitch_rad_;
    }
    FacadeOrientationResult result{current_yaw, current_pitch, false, "normal_not_valid"};
    if (!observation.normal_valid) {
      return result;
    }
    if (observation.normal_support < facade_normal_min_support_) {
      result.reason = "normal_support_below_threshold";
      return result;
    }
    if (observation.normal_confidence < facade_normal_min_confidence_) {
      result.reason = "normal_confidence_below_threshold";
      return result;
    }
    auto direction_camera = RotateVectorInverse(
      observation.k_t_camera.orientation, observation.normal_k);
    const double norm = std::sqrt(
      direction_camera.x * direction_camera.x +
      direction_camera.y * direction_camera.y +
      direction_camera.z * direction_camera.z);
    if (norm <= 1e-9) {
      result.reason = "normal_has_zero_norm";
      return result;
    }
    direction_camera.x /= norm;
    direction_camera.y /= norm;
    direction_camera.z /= norm;
    const double raw_yaw_correction = std::atan2(direction_camera.x, direction_camera.z);
    const double yaw_correction = AxialPlaneAngle(raw_yaw_correction);
    if (std::abs(NormalizeAngle(raw_yaw_correction - yaw_correction)) > M_PI_2) {
      direction_camera.x = -direction_camera.x;
      direction_camera.y = -direction_camera.y;
      direction_camera.z = -direction_camera.z;
    }
    if (std::abs(yaw_correction) * 180.0 / M_PI > facade_normal_max_yaw_change_deg_) {
      result.reason = "normal_yaw_change_above_threshold";
      return result;
    }
    const double pitch_correction = std::atan2(
      direction_camera.y,
      std::hypot(direction_camera.x, direction_camera.z));
    result.yaw_rad = current_yaw - yaw_correction;
    result.camera_pitch_rad = current_pitch + pitch_correction;
    result.normal_accepted = true;
    result.reason = "ok";
    return result;
  }

  std::pair<double, double> TargetOrientation(
    const geometry_msgs::msg::Point & target,
    const orbslam3_msgs::msg::NavigationState & state) const
  {
    const auto & pose = state.global_valid ? state.w_t_body : state.o_t_body;
    const double dx = target.x - pose.position.x;
    const double dy = target.y - pose.position.y;
    const double dz = target.z - pose.position.z;
    return {std::atan2(dy, dx), -std::atan2(dz, std::hypot(dx, dy))};
  }

  static std::uint8_t InspectionRiskMask(
    double current_yaw, double current_pitch, double target_yaw, double target_pitch)
  {
    std::uint8_t mask = mission_msgs::msg::VisualRiskEvent::RISK_NONE;
    const double yaw_delta = ShortestAngularDelta(current_yaw, target_yaw);
    if (yaw_delta > 1e-3) {
      mask |= mission_msgs::msg::VisualRiskEvent::RISK_LEFT;
    } else if (yaw_delta < -1e-3) {
      mask |= mission_msgs::msg::VisualRiskEvent::RISK_RIGHT;
    }
    if (target_pitch < current_pitch - 1e-3) {
      mask |= mission_msgs::msg::VisualRiskEvent::RISK_TOP;
    } else if (target_pitch > current_pitch + 1e-3) {
      mask |= mission_msgs::msg::VisualRiskEvent::RISK_BOTTOM;
    }
    return mask;
  }

  void BeginInspectionVisualPhase(
    const std::string & correlation_id, std::uint8_t expected_risk_mask)
  {
    std::lock_guard<std::mutex> lock(inspection_state_mutex_);
    inspection_visual_monitoring_active_ = true;
    inspection_expected_risk_mask_ = expected_risk_mask;
    inspection_risk_frame_id_ = 0U;
    inspection_risk_mask_ = mission_msgs::msg::VisualRiskEvent::RISK_NONE;
    inspection_stop_completed_ = false;
    inspection_correlation_id_ = correlation_id;
    inspection_frame_candidates_.clear();
  }

  std::pair<std::uint64_t, std::uint8_t> InspectionRisk()
  {
    std::lock_guard<std::mutex> lock(inspection_state_mutex_);
    return {inspection_risk_frame_id_, inspection_risk_mask_};
  }

  void WaitForInspectionStop()
  {
    std::unique_lock<std::mutex> lock(inspection_state_mutex_);
    inspection_state_cv_.wait_for(lock, 8s, [this]() {return inspection_stop_completed_;});
  }

  void EndInspectionVisualMonitoring()
  {
    std::lock_guard<std::mutex> lock(inspection_state_mutex_);
    inspection_visual_monitoring_active_ = false;
    inspection_expected_risk_mask_ = mission_msgs::msg::VisualRiskEvent::RISK_NONE;
    inspection_correlation_id_.clear();
  }

  bool CaptureDepthObservation(
    std::uint64_t frame_id, bool require_exact, bool use_candidate_fallback,
    std::uint64_t minimum_candidate_frame_id,
    mission_msgs::msg::DenseKFObservation & observation, std::string & reason)
  {
    if (!capture_depth_client_->wait_for_service(2s)) {
      reason = "capture_depth_unavailable";
      return false;
    }
    auto request = std::make_shared<CaptureDepth::Request>();
    request->requested_frame_id = frame_id;
    request->require_exact_frame = require_exact;
    request->use_candidate_fallback = use_candidate_fallback;
    request->minimum_candidate_frame_id = minimum_candidate_frame_id;
    request->minimum_confidence = static_cast<float>(inspection_depth_min_confidence_);
    request->minimum_support_points = inspection_depth_min_support_points_;
    auto future = capture_depth_client_->async_send_request(request);
    PublishCaptureDepthArchitecture();
    if (future.wait_for(5s) != std::future_status::ready) {
      reason = "capture_depth_timeout";
      return false;
    }
    const auto response = future.get();
    reason = response->reason;
    observation = response->observation;
    return response->success && observation.valid;
  }

  void ApplyInspectionCandidateOrientation(
    mission_msgs::msg::DenseKFObservation & observation)
  {
    std::lock_guard<std::mutex> lock(inspection_state_mutex_);
    const auto candidate = std::find_if(
      inspection_frame_candidates_.begin(), inspection_frame_candidates_.end(),
      [&observation](const auto & item) {
        return item.frame_id == observation.tracking_frame_id;
      });
    if (candidate == inspection_frame_candidates_.end()) {
      return;
    }
    observation.camera_yaw_rad = candidate->yaw_rad;
    observation.camera_pitch_rad = candidate->camera_pitch_rad;
  }

  bool ExecuteInspectionOrientation(
    double yaw_rad, double pitch_rad, const std::string & trajectory_id,
    std::string & reason)
  {
    if (!tray_client_->wait_for_action_server(2s)) {
      reason = "gen_tray_unavailable";
      return false;
    }
    orbslam3_msgs::msg::NavigationState state;
    {
      std::lock_guard<std::mutex> lock(visual_risk_mutex_);
      if (!navigation_state_received_) {
        reason = "navigation_state_unavailable";
        return false;
      }
      state = last_navigation_state_;
    }
    const double current_yaw = YawFromPose(state.global_valid ? state.w_t_body : state.o_t_body);
    double current_pitch = 0.0;
    {
      std::lock_guard<std::mutex> lock(visual_risk_mutex_);
      current_pitch = applied_camera_pitch_rad_;
    }
    const double yaw_delta = ShortestAngularDelta(current_yaw, yaw_rad);
    const double commanded_yaw = current_yaw + yaw_delta;
    const double angular_distance_deg = std::max(
      std::abs(yaw_delta), std::abs(pitch_rad - current_pitch)) * 180.0 / M_PI;
    // Un Pol3 reposo-a-reposo alcanza 1.5 veces su velocidad angular media.
    const double duration_sec = std::max(
      5.0, 1.5 * angular_distance_deg / inspection_max_yaw_rate_deg_s_);
    TrayAction::Goal goal;
    goal.tipo_trayectoria = 0U;
    goal.absoluto_x = true;
    goal.absoluto_y = true;
    goal.absoluto_z = true;
    goal.absoluto_yaw = true;
    goal.trajectory_id = trajectory_id;
    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = state.global_valid ? "world" : "orb";
    target.pose = state.global_valid ? state.w_t_body : state.o_t_body;
    goal.waypoint_targets.push_back(std::move(target));
    goal.waypoint_times.push_back(static_cast<float>(duration_sec));
    goal.waypoint_yaws_rad.push_back(commanded_yaw);
    goal.waypoint_camera_pitches_rad.push_back(pitch_rad);
    auto goal_future = tray_client_->async_send_goal(goal);
    if (goal_future.wait_for(3s) != std::future_status::ready) {
      reason = "inspection_orientation_goal_timeout";
      return false;
    }
    const auto handle = goal_future.get();
    if (!handle) {
      reason = "inspection_orientation_rejected";
      return false;
    }
    auto result_future = tray_client_->async_get_result(handle);
    const auto result_timeout = std::chrono::duration<double>(duration_sec + 7.0);
    if (result_future.wait_for(result_timeout) != std::future_status::ready) {
      reason = "inspection_orientation_result_timeout";
      return false;
    }
    const auto wrapped = result_future.get();
    const bool success = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
      wrapped.result && wrapped.result->success;
    reason = success ? "ok" :
      (wrapped.result && !wrapped.result->reason.empty() ? wrapped.result->reason :
      "inspection_orientation_failed");
    if (success) {
      std::lock_guard<std::mutex> lock(visual_risk_mutex_);
      applied_camera_pitch_rad_ = pitch_rad;
    }
    RCLCPP_INFO(
      get_logger(),
      "[F6N-INSPECTION-ORIENTATION] drone=%u id=%s yaw_delta_deg=%.3f "
      "pitch_delta_deg=%.3f duration_sec=%.3f success=%s",
      registration_.drone_id, trajectory_id.c_str(), yaw_delta * 180.0 / M_PI,
      (pitch_rad - current_pitch) * 180.0 / M_PI, duration_sec,
      success ? "true" : "false");
    return success;
  }

  void StartInspectionVisualStop(std::uint64_t frame_id, std::uint8_t risk_mask)
  {
    std::string correlation;
    {
      std::lock_guard<std::mutex> lock(inspection_state_mutex_);
      if (!inspection_visual_monitoring_active_ || inspection_risk_frame_id_ != 0U) {
        return;
      }
      inspection_risk_frame_id_ = frame_id;
      inspection_risk_mask_ = risk_mask;
      inspection_stop_completed_ = false;
      correlation = inspection_correlation_id_;
    }
    LocalVisualReorientation event;
    event.frame_id = frame_id;
    event.risk_mask = risk_mask;
    event.source_trajectory_id = correlation;
    PublishVisualRisk(
      mission_msgs::msg::VisualRiskEvent::EVENT_STOP_STARTED, event, false,
      "TRACKING_RISK durante inspeccion: STOP local iniciado");
    if (!tray_client_->action_server_is_ready()) {
      std::lock_guard<std::mutex> lock(inspection_state_mutex_);
      inspection_stop_completed_ = true;
      inspection_state_cv_.notify_all();
      return;
    }
    TrayAction::Goal stop_goal;
    stop_goal.tipo_trayectoria = 0U;
    stop_goal.stop_at_current_pose = true;
    rclcpp_action::Client<TrayAction>::SendGoalOptions options;
    options.result_callback = [this, event](
      const rclcpp_action::ClientGoalHandle<TrayAction>::WrappedResult & wrapped) {
        const bool completed = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
          wrapped.result && wrapped.result->success;
        PublishVisualRisk(
          mission_msgs::msg::VisualRiskEvent::EVENT_STOP_COMPLETED, event, completed,
          completed ? "TRACKING_RISK durante inspeccion: STOP local completado" :
          "TRACKING_RISK durante inspeccion: STOP local no completado");
        {
          std::lock_guard<std::mutex> lock(inspection_state_mutex_);
          inspection_stop_completed_ = true;
        }
        inspection_state_cv_.notify_all();
      };
    tray_client_->async_send_goal(stop_goal, options);
    RCLCPP_WARN(
      get_logger(), "[F6N-INSPECTION-RISK-STOP] drone=%u frame=%lu mask=%u",
      registration_.drone_id, frame_id, risk_mask);
  }

  void HandleInspectFacade(
    const std::shared_ptr<InspectFacade::Request> request,
    std::shared_ptr<InspectFacade::Response> response)
  {
    std::unique_lock<std::mutex> service_lock(inspect_service_mutex_, std::try_to_lock);
    if (!service_lock.owns_lock()) {
      response->reason = "inspection_already_active";
      return;
    }
    if (!request || request->drone_id != registration_.drone_id) {
      response->reason = "wrong_drone";
      return;
    }
    {
      std::lock_guard<std::mutex> lock(execution_mutex_);
      if (active_execute_goal_ || local_visual_stop_active_ || local_depth_stop_active_) {
        response->reason = "drone_busy";
        return;
      }
    }
    if (PhysicalTrajectoryActive()) {
      response->reason = "drone_busy";
      RCLCPP_INFO(
        get_logger(),
        "[F6N-FACADE-INSPECTION-DEFERRED] drone=%u reason=physical_trajectory_active",
        registration_.drone_id);
      return;
    }
    orbslam3_msgs::msg::NavigationState state;
    double current_camera_pitch = 0.0;
    {
      std::lock_guard<std::mutex> lock(visual_risk_mutex_);
      if (!navigation_state_received_ || last_navigation_state_.map_epoch != request->map_epoch) {
        response->reason = "navigation_epoch_unavailable";
        return;
      }
      state = last_navigation_state_;
      current_camera_pitch = applied_camera_pitch_rad_;
    }

    std::string reason;
    if (!CaptureDepthObservation(
        0U, false, false, 0U, response->facade_observation, reason))
    {
      response->reason = "facade_capture_failed:" + reason;
      return;
    }
    const auto facade = FacadeOrientation(response->facade_observation, state);
    const auto target = TargetOrientation(request->target_world, state);
    response->facade_yaw_rad = facade.yaw_rad;
    response->facade_camera_pitch_rad = facade.camera_pitch_rad;
    response->facade_observation.camera_yaw_rad = facade.yaw_rad;
    response->facade_observation.camera_pitch_rad = facade.camera_pitch_rad;
    RCLCPP_INFO(
      get_logger(),
      "[F6N-FACADE-NORMAL-GATE] drone=%u accepted=%s reason=%s support=%u "
      "confidence=%.3f yaw_deg=%.3f pitch_deg=%.3f",
      registration_.drone_id, facade.normal_accepted ? "true" : "false",
      facade.reason.c_str(), response->facade_observation.normal_support,
      response->facade_observation.normal_confidence, facade.yaw_rad * 180.0 / M_PI,
      facade.camera_pitch_rad * 180.0 / M_PI);

    const double current_yaw = YawFromPose(state.global_valid ? state.w_t_body : state.o_t_body);
    const std::string inspection_base_id = "inspect_" + request->task_id + "_" +
      std::to_string(now().nanoseconds());
    const std::string target_correlation_id = inspection_base_id + "_target";
    BeginInspectionVisualPhase(
      target_correlation_id,
      InspectionRiskMask(current_yaw, current_camera_pitch, target.first, target.second));
    std::string target_orientation_reason;
    bool target_orientation_ok = ExecuteInspectionOrientation(
      target.first, target.second, target_correlation_id, target_orientation_reason);
    const auto target_risk = InspectionRisk();
    std::string target_capture_reason;
    const bool target_capture_ok = CaptureDepthObservation(
      0U, false, true, response->facade_observation.tracking_frame_id + 1U,
      response->target_observation, target_capture_reason);
    if (target_capture_ok) {
      ApplyInspectionCandidateOrientation(response->target_observation);
    }

    bool restored = true;
    std::string restore_reason = "ok";
    if (target_risk.first != 0U) {
      response->target_tracking_risk = true;
      WaitForInspectionStop();
      const auto correction = BuildLocalVisualReorientation(
        target_risk.first, target_risk.second, target_correlation_id);
      EndInspectionVisualMonitoring();
      PublishVisualRisk(
        mission_msgs::msg::VisualRiskEvent::EVENT_REORIENTATION_STARTED,
        correction, false, "TRACKING_RISK durante inspeccion: correccion local iniciada");
      std::string correction_reason;
      const bool corrected = ExecuteInspectionOrientation(
        correction.yaw_rad, correction.camera_pitch_rad,
        target_correlation_id + "_local_reorient", correction_reason);
      PublishVisualRisk(
        mission_msgs::msg::VisualRiskEvent::EVENT_REORIENTATION_COMPLETED,
        correction, corrected, corrected ?
        "TRACKING_RISK durante inspeccion: correccion local completada" :
        "TRACKING_RISK durante inspeccion: correccion local no completada");
      restored = corrected;
      restore_reason = corrected ? "ok" : "tracking_risk:" + correction_reason;
    } else {
      EndInspectionVisualMonitoring();
      orbslam3_msgs::msg::NavigationState restore_start_state;
      double restore_start_pitch = target.second;
      {
        std::lock_guard<std::mutex> lock(visual_risk_mutex_);
        restore_start_state = last_navigation_state_;
        restore_start_pitch = applied_camera_pitch_rad_;
      }
      const double restore_start_yaw = YawFromPose(
        restore_start_state.global_valid ?
        restore_start_state.w_t_body : restore_start_state.o_t_body);
      const std::string restore_correlation_id = inspection_base_id + "_restore";
      BeginInspectionVisualPhase(
        restore_correlation_id,
        InspectionRiskMask(
          restore_start_yaw, restore_start_pitch, facade.yaw_rad, facade.camera_pitch_rad));
      restored = ExecuteInspectionOrientation(
        facade.yaw_rad, facade.camera_pitch_rad, restore_correlation_id, restore_reason);
      const auto restore_risk = InspectionRisk();
      if (restore_risk.first != 0U) {
        WaitForInspectionStop();
        const auto correction = BuildLocalVisualReorientation(
          restore_risk.first, restore_risk.second, restore_correlation_id);
        EndInspectionVisualMonitoring();
        PublishVisualRisk(
          mission_msgs::msg::VisualRiskEvent::EVENT_REORIENTATION_STARTED,
          correction, false, "TRACKING_RISK durante restauracion: correccion local iniciada");
        std::string correction_reason;
        const bool corrected = ExecuteInspectionOrientation(
          correction.yaw_rad, correction.camera_pitch_rad,
          restore_correlation_id + "_local_reorient", correction_reason);
        PublishVisualRisk(
          mission_msgs::msg::VisualRiskEvent::EVENT_REORIENTATION_COMPLETED,
          correction, corrected, corrected ?
          "TRACKING_RISK durante restauracion: correccion local completada" :
          "TRACKING_RISK durante restauracion: correccion local no completada");
        restored = false;
        restore_reason = "tracking_risk:" + correction_reason;
      } else {
        EndInspectionVisualMonitoring();
      }
    }
    response->success = target_capture_ok && restored &&
      (target_orientation_ok || response->target_tracking_risk);
    if (response->success) {
      response->reason = "ok";
    } else if (!target_orientation_ok && !response->target_tracking_risk) {
      response->reason = "target_orientation_failed:" + target_orientation_reason;
    } else if (!target_capture_ok) {
      response->reason = "target_capture_failed:" + target_capture_reason;
    } else if (!restored) {
      response->reason = response->target_tracking_risk ?
        "risk_reorientation_failed:" + restore_reason :
        "facade_restore_failed:" + restore_reason;
    } else {
      response->reason = "inspection_failed";
    }
    RCLCPP_INFO(
      get_logger(),
      "[F6N-INSPECT-FACADE] drone=%u task=%s success=%s risk=%s first_points=%zu "
      "second_points=%zu facade_yaw=%.3f facade_pitch=%.3f",
      registration_.drone_id, request->task_id.c_str(), response->success ? "true" : "false",
      response->target_tracking_risk ? "true" : "false",
      response->facade_observation.points_k.size(), response->target_observation.points_k.size(),
      facade.yaw_rad, facade.camera_pitch_rad);
  }

  std::uint8_t DirectionalPoorMask(
    const orbslam3_msgs::msg::VisualTrackingEvidence & evidence,
    const orbslam3_msgs::msg::NavigationState & state) const
  {
    const auto poor = [this](std::uint32_t count) {
        return count <= visual_risk_directional_max_inliers_;
      };
    const double forward = std::abs(state.velocity.linear.x);
    const double lateral = state.velocity.linear.y;
    const double vertical = state.velocity.linear.z;
    const double yaw_rate = state.velocity.angular.z;
    const bool moving_forward = forward >= visual_risk_motion_min_mps_;
    std::uint8_t mask = mission_msgs::msg::VisualRiskEvent::RISK_NONE;
    if (poor(evidence.left_directional_inlier_count) &&
      (moving_forward || lateral > visual_risk_motion_min_mps_ || yaw_rate > visual_risk_motion_min_mps_))
    {
      mask |= mission_msgs::msg::VisualRiskEvent::RISK_LEFT;
    }
    if (poor(evidence.right_directional_inlier_count) &&
      (moving_forward || lateral < -visual_risk_motion_min_mps_ || yaw_rate < -visual_risk_motion_min_mps_))
    {
      mask |= mission_msgs::msg::VisualRiskEvent::RISK_RIGHT;
    }
    if (poor(evidence.top_directional_inlier_count) &&
      (moving_forward || vertical > visual_risk_motion_min_mps_))
    {
      mask |= mission_msgs::msg::VisualRiskEvent::RISK_TOP;
    }
    if (poor(evidence.bottom_directional_inlier_count) &&
      (moving_forward || vertical < -visual_risk_motion_min_mps_))
    {
      mask |= mission_msgs::msg::VisualRiskEvent::RISK_BOTTOM;
    }
    return mask;
  }

  LocalVisualReorientation BuildLocalVisualReorientation(
    std::uint64_t frame_id, std::uint8_t risk_mask, const std::string & trajectory_id)
  {
    orbslam3_msgs::msg::NavigationState state;
    double current_pitch_rad = 0.0;
    {
      std::lock_guard<std::mutex> lock(visual_risk_mutex_);
      state = last_navigation_state_;
      current_pitch_rad = applied_camera_pitch_rad_;
    }
    const auto & orientation = state.global_valid ? state.w_t_body.orientation :
      state.o_t_body.orientation;
    const double sin_yaw = 2.0 * (orientation.w * orientation.z +
      orientation.x * orientation.y);
    const double cos_yaw = 1.0 - 2.0 * (orientation.y * orientation.y +
      orientation.z * orientation.z);
    LocalVisualReorientation reorientation;
    reorientation.frame_id = frame_id;
    reorientation.risk_mask = risk_mask;
    reorientation.source_trajectory_id = trajectory_id;
    reorientation.yaw_rad = std::atan2(sin_yaw, cos_yaw);
    // Las correcciones se expresan en la convención óptica: no buscan el
    // máximo de inliers, solo alejan la mirada del primer semiplano pobre.
    const double reorientation_step_rad =
      visual_risk_reorientation_step_deg_ * M_PI / 180.0;
    if ((risk_mask & mission_msgs::msg::VisualRiskEvent::RISK_LEFT) != 0U) {
      reorientation.yaw_rad -= reorientation_step_rad;
    } else if ((risk_mask & mission_msgs::msg::VisualRiskEvent::RISK_RIGHT) != 0U) {
      reorientation.yaw_rad += reorientation_step_rad;
    }
    reorientation.camera_pitch_rad = current_pitch_rad +
      ((risk_mask & mission_msgs::msg::VisualRiskEvent::RISK_TOP) != 0U ? -reorientation_step_rad :
      ((risk_mask & mission_msgs::msg::VisualRiskEvent::RISK_BOTTOM) != 0U ? reorientation_step_rad : 0.0));
    return reorientation;
  }

  void PublishVisualRisk(
    std::uint8_t event_type, const LocalVisualReorientation & reorientation,
    bool success, const std::string & detail)
  {
    mission_msgs::msg::VisualRiskEvent event;
    event.header.stamp = now();
    event.drone_id = registration_.drone_id;
    event.task_id = active_task_id_;
    event.trajectory_id = reorientation.source_trajectory_id;
    event.frame_id = reorientation.frame_id;
    event.event_type = event_type;
    event.risk_mask = reorientation.risk_mask;
    event.recommended_yaw_rad = reorientation.yaw_rad;
    event.recommended_camera_pitch_rad = reorientation.camera_pitch_rad;
    event.success = success;
    event.detail = detail;
    visual_risk_publisher_->publish(event);
  }

  void StartLocalVisualReorientation()
  {
    LocalVisualReorientation reorientation;
    {
      std::lock_guard<std::mutex> lock(execution_mutex_);
      if (!local_visual_reorientation_.has_value() || local_visual_reorientation_active_) {
        return;
      }
      local_visual_reorientation_active_ = true;
      reorientation = *local_visual_reorientation_;
    }
    if (!tray_client_->action_server_is_ready()) {
      PublishVisualRisk(
        mission_msgs::msg::VisualRiskEvent::EVENT_REORIENTATION_COMPLETED,
        reorientation, false, "TRACKING_RISK: reorientacion local rechazada porque gen_tray no esta disponible");
      std::lock_guard<std::mutex> lock(execution_mutex_);
      local_visual_reorientation_active_ = false;
      local_visual_reorientation_.reset();
      return;
    }

    orbslam3_msgs::msg::NavigationState state;
    {
      std::lock_guard<std::mutex> lock(visual_risk_mutex_);
      state = last_navigation_state_;
      visual_risk_ignore_until_ = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(visual_risk_reorientation_grace_sec_));
    }
    TrayAction::Goal goal;
    goal.tipo_trayectoria = 0U;
    goal.absoluto_x = true;
    goal.absoluto_y = true;
    goal.absoluto_z = true;
    goal.absoluto_yaw = true;
    goal.trajectory_id = reorientation.source_trajectory_id + "_local_reorient";
    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = "world";
    target.pose = state.global_valid ? state.w_t_body : state.o_t_body;
    goal.waypoint_targets.push_back(std::move(target));
    goal.waypoint_times.push_back(5.0F);
    goal.waypoint_yaws_rad.push_back(reorientation.yaw_rad);
    goal.waypoint_camera_pitches_rad.push_back(reorientation.camera_pitch_rad);
    PublishVisualRisk(
      mission_msgs::msg::VisualRiskEvent::EVENT_REORIENTATION_STARTED,
      reorientation, false, "TRACKING_RISK: reorientacion local iniciada");
    rclcpp_action::Client<TrayAction>::SendGoalOptions options;
    options.result_callback = [this, reorientation](
      const rclcpp_action::ClientGoalHandle<TrayAction>::WrappedResult & wrapped) {
        const bool completed = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
          wrapped.result && wrapped.result->success;
        if (completed) {
          std::lock_guard<std::mutex> lock(visual_risk_mutex_);
          applied_camera_pitch_rad_ = reorientation.camera_pitch_rad;
        }
        PublishVisualRisk(
          mission_msgs::msg::VisualRiskEvent::EVENT_REORIENTATION_COMPLETED,
          reorientation, completed, completed ?
          "TRACKING_RISK: reorientacion local completada" :
          "TRACKING_RISK: reorientacion local no completada");
        {
          std::lock_guard<std::mutex> lock(execution_mutex_);
          local_visual_reorientation_active_ = false;
          local_visual_reorientation_.reset();
        }
        RCLCPP_INFO(
          get_logger(),
          "[F6L-LOCAL-REORIENT-FINAL] drone=%u source=%s success=%s yaw_deg=%.3f pitch_deg=%.3f",
          registration_.drone_id, reorientation.source_trajectory_id.c_str(),
          completed ? "true" : "false", reorientation.yaw_rad * 180.0 / M_PI,
          reorientation.camera_pitch_rad * 180.0 / M_PI);
      };
    tray_client_->async_send_goal(goal, options);
    RCLCPP_INFO(
      get_logger(),
      "[F6L-LOCAL-REORIENT-DISPATCH] drone=%u source=%s yaw_deg=%.3f pitch_deg=%.3f",
      registration_.drone_id, reorientation.source_trajectory_id.c_str(),
      reorientation.yaw_rad * 180.0 / M_PI, reorientation.camera_pitch_rad * 180.0 / M_PI);
  }

  void StartLocalVisualStop(std::uint64_t frame_id, std::uint8_t risk_mask)
  {
    std::shared_ptr<GoalHandleExecuteTrajectory> replaced_goal;
    ExecutionFinish replaced_finish;
    std::string trajectory_id;
    {
      std::lock_guard<std::mutex> lock(execution_mutex_);
      if (local_visual_stop_active_ || !active_execute_goal_ || active_trajectory_id_.empty()) {
        return;
      }
      local_visual_stop_active_ = true;
      trajectory_id = active_trajectory_id_;
      replaced_goal = active_execute_goal_;
      replaced_finish = active_execution_finish_;
      active_execute_goal_.reset();
      active_tray_goal_.reset();
      active_execution_finish_ = {};
      active_trajectory_id_.clear();
    }
    {
      std::lock_guard<std::mutex> lock(autonomous_command_mutex_);
      if (autonomous_command_active_) {
        active_autonomous_tracking_risk_ = true;
      }
    }
    const auto reorientation = BuildLocalVisualReorientation(frame_id, risk_mask, trajectory_id);
    {
      std::lock_guard<std::mutex> lock(execution_mutex_);
      local_visual_reorientation_ = reorientation;
    }
    execution_generation_.fetch_add(1U, std::memory_order_acq_rel);
    PublishVisualRisk(
      mission_msgs::msg::VisualRiskEvent::EVENT_STOP_STARTED, reorientation, false,
      "TRACKING_RISK: STOP local iniciado");
    if (replaced_finish) {
      replaced_finish(false, mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_CANCELED,
        "visual_risk_stop_started");
    }
    if (!tray_client_->action_server_is_ready()) {
      PublishVisualRisk(
        mission_msgs::msg::VisualRiskEvent::EVENT_STOP_COMPLETED, reorientation, false,
        "TRACKING_RISK: STOP local rechazado porque gen_tray no esta disponible");
      std::lock_guard<std::mutex> lock(execution_mutex_);
      local_visual_stop_active_ = false;
      local_visual_reorientation_.reset();
      return;
    }
    TrayAction::Goal stop_goal;
    stop_goal.tipo_trayectoria = 0U;
    stop_goal.stop_at_current_pose = true;
    rclcpp_action::Client<TrayAction>::SendGoalOptions options;
    options.result_callback = [this, reorientation](
      const rclcpp_action::ClientGoalHandle<TrayAction>::WrappedResult & wrapped) {
        const bool completed = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
          wrapped.result && wrapped.result->success;
        PublishVisualRisk(
          mission_msgs::msg::VisualRiskEvent::EVENT_STOP_COMPLETED, reorientation, completed,
          completed ? "TRACKING_RISK: STOP local completado" :
          "TRACKING_RISK: STOP local no completado");
        {
          std::lock_guard<std::mutex> lock(execution_mutex_);
          local_visual_stop_active_ = false;
          if (!completed) {
            local_visual_reorientation_.reset();
          }
        }
        if (completed) {
          StartLocalVisualReorientation();
        }
      };
    tray_client_->async_send_goal(stop_goal, options);
    RCLCPP_WARN(
      get_logger(), "[F6L-TRACKING-RISK-STOP] drone=%u trajectory_id=%s frame=%lu mask=%u",
      registration_.drone_id, trajectory_id.c_str(), frame_id, risk_mask);
  }

  void HandleSafetyEvent(mission_msgs::msg::SafetyEvent::ConstSharedPtr event)
  {
    if (!event || event->drone_id != registration_.drone_id ||
      event->event_type != mission_msgs::msg::SafetyEvent::DEPTH_EMERGENCY)
    {
      return;
    }
    StartLocalDepthStop(*event);
  }

  void StartLocalDepthStop(const mission_msgs::msg::SafetyEvent & emergency)
  {
    ExecutionFinish replaced_finish;
    std::string trajectory_id;
    {
      std::lock_guard<std::mutex> lock(execution_mutex_);
      if (local_depth_stop_active_ || local_visual_stop_active_ || !active_execute_goal_ ||
        active_trajectory_id_.empty())
      {
        return;
      }
      local_depth_stop_active_ = true;
      trajectory_id = active_trajectory_id_;
      replaced_finish = active_execution_finish_;
      active_execute_goal_.reset();
      active_tray_goal_.reset();
      active_execution_finish_ = {};
      active_trajectory_id_.clear();
    }
    execution_generation_.fetch_add(1U, std::memory_order_acq_rel);
    if (replaced_finish) {
      replaced_finish(false, mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_CANCELED,
        "depth_emergency_stop_started");
    }
    if (!tray_client_->action_server_is_ready()) {
      std::lock_guard<std::mutex> lock(execution_mutex_);
      local_depth_stop_active_ = false;
      return;
    }
    TrayAction::Goal stop_goal;
    stop_goal.tipo_trayectoria = 0U;
    stop_goal.stop_at_current_pose = true;
    rclcpp_action::Client<TrayAction>::SendGoalOptions options;
    options.result_callback = [this, emergency, trajectory_id](
      const rclcpp_action::ClientGoalHandle<TrayAction>::WrappedResult & wrapped) {
        const bool completed = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
          wrapped.result && wrapped.result->success;
        mission_msgs::msg::SafetyEvent terminal = emergency;
        terminal.header.stamp = now();
        terminal.trajectory_id = trajectory_id;
        terminal.event_type = mission_msgs::msg::SafetyEvent::STOPPED;
        terminal.detail = completed ? "depth STOP local completado" : "depth STOP local no completado";
        safety_event_publisher_->publish(terminal);
        std::lock_guard<std::mutex> lock(execution_mutex_);
        local_depth_stop_active_ = false;
      };
    tray_client_->async_send_goal(stop_goal, options);
    RCLCPP_WARN(
      get_logger(), "[F6N-DEPTH-STOP-DISPATCH] drone=%u trajectory_id=%s detail=%s",
      registration_.drone_id, trajectory_id.c_str(), emergency.detail.c_str());
  }

  void HandleVisualEvidence(orbslam3_msgs::msg::VisualTrackingEvidence::ConstSharedPtr evidence)
  {
    if (!visual_risk_enabled_ || !evidence || evidence->drone_id != registration_.drone_id) {
      return;
    }
    orbslam3_msgs::msg::NavigationState state;
    double current_camera_pitch = 0.0;
    {
      std::lock_guard<std::mutex> lock(visual_risk_mutex_);
      if (!navigation_state_received_) {
        return;
      }
      bool inspection_active = false;
      {
        std::lock_guard<std::mutex> inspection_lock(inspection_state_mutex_);
        inspection_active = inspection_visual_monitoring_active_;
      }
      if (!inspection_active && std::chrono::steady_clock::now() < visual_risk_ignore_until_) {
        return;
      }
      state = last_navigation_state_;
      current_camera_pitch = applied_camera_pitch_rad_;
    }
    std::string trajectory_id;
    std::uint8_t inspection_expected_mask = mission_msgs::msg::VisualRiskEvent::RISK_NONE;
    bool inspection_active = false;
    {
      std::lock_guard<std::mutex> lock(inspection_state_mutex_);
      inspection_active = inspection_visual_monitoring_active_;
      inspection_expected_mask = inspection_expected_risk_mask_;
      trajectory_id = inspection_correlation_id_;
      if (inspection_active &&
        evidence->tracking_inlier_count >= inspection_candidate_min_tracking_inliers_)
      {
        const double stamp_sec = static_cast<double>(evidence->header.stamp.sec) +
          static_cast<double>(evidence->header.stamp.nanosec) * 1e-9;
        const double yaw_rad = YawFromPose(state.global_valid ? state.w_t_body : state.o_t_body);
        bool separated = inspection_frame_candidates_.empty();
        if (!separated) {
          const auto & previous = inspection_frame_candidates_.back();
          const double orientation_delta_deg = std::max(
            std::abs(ShortestAngularDelta(previous.yaw_rad, yaw_rad)),
            std::abs(previous.camera_pitch_rad - current_camera_pitch)) * 180.0 / M_PI;
          separated = stamp_sec - previous.stamp_sec >= inspection_candidate_min_interval_sec_ ||
            orientation_delta_deg >= inspection_candidate_min_orientation_delta_deg_;
        }
        if (separated) {
          if (inspection_frame_candidates_.size() >= inspection_candidate_frame_capacity_) {
            inspection_frame_candidates_.pop_front();
          }
          inspection_frame_candidates_.push_back(InspectionFrameCandidate{
            evidence->frame_id, stamp_sec, yaw_rad, current_camera_pitch});
        }
      }
    }
    {
      std::lock_guard<std::mutex> lock(execution_mutex_);
      if (!inspection_active && (local_visual_stop_active_ || active_trajectory_id_.empty())) {
        return;
      }
      if (!inspection_active) {
        trajectory_id = active_trajectory_id_;
      }
    }
    std::uint8_t mask = DirectionalPoorMask(*evidence, state);
    if (inspection_active) {
      std::uint8_t poor_mask = mission_msgs::msg::VisualRiskEvent::RISK_NONE;
      if (evidence->left_directional_inlier_count <= visual_risk_directional_max_inliers_) {
        poor_mask |= mission_msgs::msg::VisualRiskEvent::RISK_LEFT;
      }
      if (evidence->right_directional_inlier_count <= visual_risk_directional_max_inliers_) {
        poor_mask |= mission_msgs::msg::VisualRiskEvent::RISK_RIGHT;
      }
      if (evidence->top_directional_inlier_count <= visual_risk_directional_max_inliers_) {
        poor_mask |= mission_msgs::msg::VisualRiskEvent::RISK_TOP;
      }
      if (evidence->bottom_directional_inlier_count <= visual_risk_directional_max_inliers_) {
        poor_mask |= mission_msgs::msg::VisualRiskEvent::RISK_BOTTOM;
      }
      mask = static_cast<std::uint8_t>(
        (static_cast<std::uint8_t>(mask | poor_mask)) & inspection_expected_mask);
    }
    const std::uint8_t first_persistent_sector = FirstPersistentPoorSector(trajectory_id, mask);
    if (first_persistent_sector == mission_msgs::msg::VisualRiskEvent::RISK_NONE) {
      return;
    }
    RCLCPP_WARN(
      get_logger(),
      "[F6L-FIRST-POOR-REGION] drone=%u trajectory_id=%s frame=%lu sector=%u fraction=%.2f max_inliers=%u",
      registration_.drone_id, trajectory_id.c_str(), evidence->frame_id, first_persistent_sector,
      evidence->directional_region_fraction, visual_risk_directional_max_inliers_);
    if (inspection_active) {
      StartInspectionVisualStop(evidence->frame_id, first_persistent_sector);
    } else {
      StartLocalVisualStop(evidence->frame_id, first_persistent_sector);
    }
  }

  std::uint8_t FirstPersistentPoorSector(const std::string & trajectory_id, std::uint8_t mask)
  {
    using Risk = mission_msgs::msg::VisualRiskEvent;
    constexpr std::uint8_t sectors[] = {
      Risk::RISK_LEFT, Risk::RISK_RIGHT, Risk::RISK_TOP, Risk::RISK_BOTTOM};
    std::lock_guard<std::mutex> lock(visual_risk_mutex_);
    if (visual_risk_trajectory_id_ != trajectory_id) {
      visual_risk_trajectory_id_ = trajectory_id;
      visual_risk_latched_sector_ = Risk::RISK_NONE;
      visual_risk_frames_by_sector_.clear();
    }
    if (visual_risk_latched_sector_ != Risk::RISK_NONE) {
      return Risk::RISK_NONE;
    }
    for (const auto sector : sectors) {
      auto & consecutive = visual_risk_frames_by_sector_[sector];
      consecutive = (mask & sector) != 0U ? consecutive + 1U : 0U;
      if (consecutive >= visual_risk_persistence_frames_) {
        visual_risk_latched_sector_ = sector;
        visual_risk_frames_by_sector_.clear();
        return sector;
      }
    }
    return Risk::RISK_NONE;
  }

  bool IsCurrentExecutionGeneration(std::uint64_t generation) const
  {
    return execution_generation_.load(std::memory_order_acquire) == generation;
  }

  void FinishSupersededExecution(const std::shared_ptr<GoalHandleExecuteTrajectory> & goal_handle)
  {
    auto result = std::make_shared<ExecuteTrajectory::Result>();
    result->success = false;
    result->final_state = mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_CANCELED;
    result->reason = "replaced_by_stop";
    goal_handle->abort(result);
  }

  void ExecuteStopCommand(
    const std::shared_ptr<GoalHandleExecuteTrajectory> & goal_handle, std::uint64_t generation)
  {
    if (!IsCurrentExecutionGeneration(generation)) {
      FinishSupersededExecution(goal_handle);
      return;
    }
    auto finished = std::make_shared<std::atomic<bool>>(false);
    auto finish = [this, goal_handle, finished](bool success, std::uint8_t final_state,
      const std::string & reason) {
        if (finished->exchange(true)) {
          return;
        }
        {
          std::lock_guard<std::mutex> lock(execution_mutex_);
          if (active_execute_goal_ == goal_handle) {
            active_execute_goal_.reset();
            active_tray_goal_.reset();
            active_execution_finish_ = {};
          }
        }
        auto result = std::make_shared<ExecuteTrajectory::Result>();
        result->success = success;
        result->final_state = final_state;
        result->reason = reason;
        if (success) {
          goal_handle->succeed(result);
        } else {
          goal_handle->abort(result);
        }
      };

    ExecutionFinish replaced_finish;
    {
      std::lock_guard<std::mutex> lock(execution_mutex_);
      replaced_finish = active_execution_finish_;
      active_execute_goal_ = goal_handle;
      active_tray_goal_.reset();
      active_execution_finish_ = finish;
    }
    if (replaced_finish) {
      replaced_finish(false, mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_CANCELED,
        "replaced_by_stop");
    }
    if (!tray_client_->action_server_is_ready()) {
      finish(false, mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_REJECTED,
        "gen_tray no disponible para stop");
      return;
    }

    TrayAction::Goal stop_goal;
    stop_goal.tipo_trayectoria = 0U;
    stop_goal.stop_at_current_pose = true;
    rclcpp_action::Client<TrayAction>::SendGoalOptions options;
    options.goal_response_callback = [this, goal_handle, finish](
      std::shared_ptr<rclcpp_action::ClientGoalHandle<TrayAction>> handle) {
        if (!handle) {
          RCLCPP_ERROR(get_logger(), "[F6I-TASK-STOP-REJECTED] drone=%u", registration_.drone_id);
          finish(false, mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_REJECTED,
            "stop_rejected");
          return;
        }
        {
          std::lock_guard<std::mutex> lock(execution_mutex_);
          if (active_execute_goal_ == goal_handle) {
            active_tray_goal_ = handle;
          }
        }
        RCLCPP_INFO(get_logger(), "[F6I-TASK-STOP-ACTIVE] drone=%u", registration_.drone_id);
      };
    options.result_callback = [this, finish](
      const rclcpp_action::ClientGoalHandle<TrayAction>::WrappedResult & wrapped) {
        const bool completed = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
          wrapped.result && wrapped.result->success;
        RCLCPP_INFO(
          get_logger(), "[F6I-TASK-STOP-FINAL] success=%s reason=%s",
          completed ? "true" : "false",
          wrapped.result ? wrapped.result->reason.c_str() : "missing_result");
        finish(completed,
          completed ? mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_COMPLETED :
          mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_CANCELED,
          completed ? "segmento stop completado por gen_tray" : "stop_failed");
      };
    tray_client_->async_send_goal(stop_goal, options);
    RCLCPP_INFO(get_logger(), "[F6I-TASK-STOP-DISPATCH] drone=%u", registration_.drone_id);
  }

  void ExecuteTrajectoryPlan(
    const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle, std::uint64_t generation)
  {
    if (!IsCurrentExecutionGeneration(generation)) {
      FinishSupersededExecution(goal_handle);
      return;
    }
    const auto & plan = goal_handle->get_goal()->plan;
    auto finished = std::make_shared<std::atomic<bool>>(false);
    {
      std::lock_guard<std::mutex> lock(execution_mutex_);
      active_execute_goal_ = goal_handle;
      active_tray_goal_.reset();
      active_trajectory_id_ = plan.trajectory_id;
    }
    auto finish = [this, goal_handle, finished](bool success, std::uint8_t final_state,
      const std::string & reason) {
        if (finished->exchange(true)) {
        return;
      }
      {
        std::lock_guard<std::mutex> lock(execution_mutex_);
        if (active_execute_goal_ == goal_handle) {
          active_execute_goal_.reset();
          active_tray_goal_.reset();
          active_execution_finish_ = {};
          active_trajectory_id_.clear();
        }
      }
        auto result = std::make_shared<ExecuteTrajectory::Result>();
        result->success = success;
        result->final_state = final_state;
        result->reason = reason;
        if (success) {
          goal_handle->succeed(result);
        } else if (goal_handle->is_canceling()) {
          goal_handle->canceled(result);
        } else {
          goal_handle->abort(result);
        }
      };
    {
      std::lock_guard<std::mutex> lock(execution_mutex_);
      if (active_execute_goal_ == goal_handle) {
        active_execution_finish_ = finish;
      }
    }
    if (!tray_client_->action_server_is_ready()) {
      finish(false, mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_REJECTED,
        "gen_tray no disponible");
      return;
    }
    auto tray_goal = TrayAction::Goal();
    tray_goal.tipo_trayectoria = 0U;
    tray_goal.absoluto_x = true;
    tray_goal.absoluto_y = true;
    tray_goal.absoluto_z = true;
    tray_goal.absoluto_yaw = true;
    tray_goal.trajectory_id = plan.trajectory_id;
    // El primer waypoint del plan es solo el origen visual de la polilínea D*.
    // gen_tray captura su origen canónico al aceptar la acción.
    for (std::size_t index = 1U; index < plan.waypoints.size(); ++index) {
      const auto & waypoint = plan.waypoints[index];
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "world";
      pose.pose.position = waypoint.position_world;
      pose.pose.orientation.w = 1.0;
      tray_goal.waypoint_targets.push_back(std::move(pose));
      tray_goal.waypoint_times.push_back(static_cast<float>(
        static_cast<double>(waypoint.time_from_start.sec) +
        static_cast<double>(waypoint.time_from_start.nanosec) * 1e-9));
      tray_goal.waypoint_yaws_rad.push_back(waypoint.yaw_rad);
      tray_goal.waypoint_camera_pitches_rad.push_back(waypoint.camera_pitch_rad);
    }
    RCLCPP_INFO(
      get_logger(),
      "[F6M-TASK-ORIENTATION] drone=%u trajectory_id=%s waypoints=%zu yaw_rad=%.6f "
      "pitch_rad=%.6f",
      registration_.drone_id, plan.trajectory_id.c_str(), tray_goal.waypoint_targets.size(),
      tray_goal.waypoint_yaws_rad.back(), tray_goal.waypoint_camera_pitches_rad.back());
    rclcpp_action::Client<TrayAction>::SendGoalOptions options;
    options.feedback_callback = [goal_handle, total = tray_goal.waypoint_times.back()](
      rclcpp_action::ClientGoalHandle<TrayAction>::SharedPtr,
      const std::shared_ptr<const TrayAction::Feedback> feedback) {
        auto report = std::make_shared<ExecuteTrajectory::Feedback>();
        report->progress = total > 0.0F ? std::min(1.0F, feedback->t_act / total) : 0.0F;
        report->state = mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_ACTIVE;
        report->detail = "gen_tray ejecutando segmento continuo";
        goal_handle->publish_feedback(report);
      };
    options.goal_response_callback = [this, goal_handle, finish](
      std::shared_ptr<rclcpp_action::ClientGoalHandle<TrayAction>> handle) {
        if (!handle) {
          finish(false, mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_REJECTED,
            "gen_tray rechazo el segmento");
          return;
        }
        {
          std::lock_guard<std::mutex> lock(execution_mutex_);
          if (active_execute_goal_ == goal_handle) {
            active_tray_goal_ = handle;
          }
        }
      };
    options.result_callback = [this, goal_handle, finish](
      const rclcpp_action::ClientGoalHandle<TrayAction>::WrappedResult & wrapped) {
        const bool success = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
          wrapped.result && wrapped.result->success;
        finish(success,
          success ? mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_COMPLETED :
          mission_msgs::msg::TrajectoryPlan::EXECUTION_STATE_CANCELED,
          success ? "segmento completado por gen_tray" :
          (wrapped.result && !wrapped.result->reason.empty() ? wrapped.result->reason :
          "gen_tray cancelo o aborto el segmento"));
      };
    tray_client_->async_send_goal(tray_goal, options);
    RCLCPP_INFO(
      get_logger(), "[F6I-TASK-EXECUTE] drone=%u task=%s trajectory_id=%s waypoints=%zu",
      registration_.drone_id, plan.task_id.c_str(), plan.trajectory_id.c_str(),
      plan.waypoints.size());
  }

  void PublishTaskArchitecture()
  {
    if (!architecture_publisher_) {
      return;
    }
    std_msgs::msg::String message;
    std::ostringstream json;
    json << "{\"kind\":\"architecture_activity\","
         << "\"edge_id\":\"task_manager_to_task_server_task_report\","
         << "\"source\":\"task_manager\","
         << "\"interface\":\"/mission/task_reports\","
         << "\"interface_kind\":\"topic_publish\",\"timestamp\":" << now().seconds()
         << ",\"drone_id\":" << registration_.drone_id
         << ",\"detail\":\"TASK_ACCEPTED\"}";
    message.data = json.str();
    architecture_publisher_->publish(message);
  }

  mission_msgs::msg::DroneRegistration registration_;
  bool architecture_events_enabled_ = false;
  bool registered_ = false;
  bool request_pending_ = false;
  std::string pose_source_;
  std::string last_reported_task_id_;
  std::string active_task_id_;
  std::string active_trajectory_id_;
  uint64_t last_reported_task_revision_ = 0;
  std::map<std::string, mission_msgs::msg::TaskState> known_tasks_;
  rclcpp::Client<mission_msgs::srv::RegisterDrone>::SharedPtr client_;
  rclcpp::Client<CaptureDepth>::SharedPtr capture_depth_client_;
  rclcpp::Client<ReportAutonomousResult>::SharedPtr autonomous_result_client_;
  rclcpp_action::Client<ExecuteTrajectory>::SharedPtr autonomous_execute_client_;
  rclcpp::Service<InspectFacade>::SharedPtr inspect_facade_service_;
  rclcpp::Service<SubmitAutonomousCommand>::SharedPtr autonomous_command_service_;
  rclcpp::TimerBase::SharedPtr autonomous_runtime_timer_;
  rclcpp::CallbackGroup::SharedPtr inspect_service_group_;
  rclcpp::CallbackGroup::SharedPtr capture_depth_group_;
  rclcpp::CallbackGroup::SharedPtr autonomous_command_group_;
  rclcpp::Subscription<mission_msgs::msg::TaskStateArray>::SharedPtr task_subscription_;
  rclcpp::Publisher<mission_msgs::msg::TaskReport>::SharedPtr report_publisher_;
  rclcpp::Publisher<mission_msgs::msg::VisualRiskEvent>::SharedPtr visual_risk_publisher_;
  rclcpp::Publisher<mission_msgs::msg::SafetyEvent>::SharedPtr safety_event_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr architecture_publisher_;
  rclcpp_action::Client<TrayAction>::SharedPtr tray_client_;
  std::mutex execution_mutex_;
  std::shared_ptr<GoalHandleExecuteTrajectory> active_execute_goal_;
  std::shared_ptr<rclcpp_action::ClientGoalHandle<TrayAction>> active_tray_goal_;
  ExecutionFinish active_execution_finish_;
  bool local_visual_stop_active_ = false;
  bool local_depth_stop_active_ = false;
  bool local_visual_reorientation_active_ = false;
  std::optional<LocalVisualReorientation> local_visual_reorientation_;
  std::atomic<std::uint64_t> execution_generation_{0U};
  rclcpp_action::Server<ExecuteTrajectory>::SharedPtr execute_trajectory_server_;
  rclcpp::Subscription<orbslam3_msgs::msg::VisualTrackingEvidence>::SharedPtr
    visual_evidence_subscription_;
  rclcpp::Subscription<mission_msgs::msg::SafetyEvent>::SharedPtr safety_event_subscription_;
  rclcpp::Subscription<orbslam3_msgs::msg::NavigationState>::SharedPtr navigation_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trajectory_active_subscription_;
  rclcpp::Subscription<mission_msgs::msg::FiducialPrimaryObservation>::SharedPtr
    fiducial_primary_subscription_;
  std::atomic<bool> physical_trajectory_active_{false};
  mutable std::mutex visual_risk_mutex_;
  orbslam3_msgs::msg::NavigationState last_navigation_state_;
  bool navigation_state_received_ = false;
  bool visual_risk_enabled_ = false;
  std::uint32_t visual_risk_persistence_frames_ = 3U;
  std::uint32_t visual_risk_directional_max_inliers_ = 3U;
  std::string visual_risk_trajectory_id_;
  std::uint8_t visual_risk_latched_sector_ = mission_msgs::msg::VisualRiskEvent::RISK_NONE;
  std::map<std::uint8_t, std::uint32_t> visual_risk_frames_by_sector_;
  double visual_risk_motion_min_mps_ = 0.05;
  double visual_risk_reorientation_grace_sec_ = 6.0;
  double visual_risk_reorientation_step_deg_ = 25.0;
  std::uint32_t facade_normal_min_support_ = 40U;
  double facade_normal_min_confidence_ = 0.8;
  double facade_normal_max_yaw_change_deg_ = 25.0;
  double inspection_max_yaw_rate_deg_s_ = 5.0;
  std::uint32_t inspection_candidate_min_tracking_inliers_ = 20U;
  std::size_t inspection_candidate_frame_capacity_ = 16U;
  double inspection_candidate_min_interval_sec_ = 0.5;
  double inspection_candidate_min_orientation_delta_deg_ = 2.5;
  double inspection_depth_min_confidence_ = 0.25;
  std::uint32_t inspection_depth_min_support_points_ = 20U;
  double applied_camera_pitch_rad_ = 0.0;
  std::chrono::steady_clock::time_point visual_risk_ignore_until_{};
  std::mutex inspect_service_mutex_;
  std::mutex autonomous_command_mutex_;
  std::deque<QueuedAutonomousCommand> autonomous_command_queue_;
  std::set<std::string> accepted_autonomous_command_ids_;
  bool autonomous_command_active_ = false;
  bool active_autonomous_fiducial_seen_ = false;
  bool active_autonomous_tracking_risk_ = false;
  std::optional<QueuedAutonomousCommand> active_autonomous_command_;
  std::mutex inspection_state_mutex_;
  std::condition_variable inspection_state_cv_;
  bool inspection_visual_monitoring_active_ = false;
  bool inspection_stop_completed_ = false;
  std::uint8_t inspection_expected_risk_mask_ = mission_msgs::msg::VisualRiskEvent::RISK_NONE;
  std::uint8_t inspection_risk_mask_ = mission_msgs::msg::VisualRiskEvent::RISK_NONE;
  std::uint64_t inspection_risk_frame_id_ = 0U;
  std::string inspection_correlation_id_;
  std::deque<InspectionFrameCandidate> inspection_frame_candidates_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<TaskManagerNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4U);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("task_manager"), "%s", error.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
