#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>                      // Librería necesaria para crear la acción.

#include "dron_individual/action/tray_action.hpp"               // Añadir interfaz usada en la acción.
#include "dron_individual/navigation_goal_policy.hpp"
#include "dron_individual/navigation_state_mux.hpp"
#include "orbslam3_msgs/msg/navigation_state.hpp"
#include "lib_tray/gen_tray_elipse.hpp"                         // Librería elipse
#include "lib_tray/gen_tray_pol3_waypoints.hpp"
#include "lib_tray/gen_tray_veltrap_waypoints.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <cmath>
#include <algorithm>
#include <cstdint>
#include <limits>
#include <thread>
#include <mutex>
#include <optional>
#include <array>
#include <vector>
#include <memory>
#include <functional>
#include <future>
#include <exception>
#include <chrono>
#include <condition_variable>
#include <map>
#include <sstream>
#include <string>


class Clase_Servicio_Accion : public rclcpp::Node
{
public:
  using TrayAction = dron_individual::action::TrayAction;
  using GoalHandleTrayAction = rclcpp_action::ServerGoalHandle<TrayAction>;

  Clase_Servicio_Accion()
  : rclcpp::Node("gen_tray")
  {
    sub_navigation_state_ =
      this->create_subscription<orbslam3_msgs::msg::NavigationState>(
      "orbslam/navigation_state", rclcpp::QoS(20).reliable(),
      std::bind(
        &Clase_Servicio_Accion::navigation_state_callback, this,
        std::placeholders::_1));
    trajectory_active_client_ =
      this->create_client<std_srvs::srv::SetBool>(
      "control/set_trajectory_active");
    camera_pitch_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "camera_pitch/command", rclcpp::QoS(10).reliable());
    trajectory_active_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
      "control/trajectory_active", rclcpp::QoS(1).reliable().transient_local());
    PublishTrajectoryActive(false);

    // Añadimos al objeto del servidor de la acción el tipo de interfaz a utilizar, el nombre de la acción el cual
    // el cliente de la acción debe llamar para acceder a él, y el nombre de las función callback.
    objeto_servicio_accion = rclcpp_action::create_server<TrayAction>(
      this, "AccionTrayectoria",
      std::bind(
        &Clase_Servicio_Accion::handle_goal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&Clase_Servicio_Accion::handle_cancel, this, std::placeholders::_1),
      std::bind(&Clase_Servicio_Accion::handle_accepted, this, std::placeholders::_1));

    this->declare_parameter<double>("crear.tray.v_max_lin", 0.8);
    stop_duration_sec_ = this->declare_parameter<double>("stop_duration_sec", 2.0);
    waypoint_blend_sec_ = this->declare_parameter<double>("waypoint_blend_sec", 3.0);
    debug_f6i_trajectory_ = this->declare_parameter<bool>("debug_f6i_trajectory", false);
    this->declare_parameter<double>("crear.tray.v_max_ang", 0.5);
    this->declare_parameter<double>("crear.tray.t_a", 2.0);
    this->declare_parameter<int64_t>("drone_id", 0);
    this->declare_parameter<bool>("debug_architecture_telemetry", false);
    this->declare_parameter<double>("navigation_state_timeout_sec", 0.5);
    this->declare_parameter<double>("orb_loss_hold_sec", 10.0);
    this->declare_parameter<double>("trajectory_source_handshake_timeout_sec", 1.0);

    drone_id_ = static_cast<uint32_t>(this->get_parameter("drone_id").as_int());
    debug_architecture_telemetry_ =
      this->get_parameter("debug_architecture_telemetry").as_bool();
    if (debug_architecture_telemetry_) {
      architecture_activity_pub_ =
        this->create_publisher<std_msgs::msg::String>(
        "/system_architecture/activity",
        rclcpp::QoS(64).best_effort());
    }

    v_max_lin = this->get_parameter("crear.tray.v_max_lin").as_double();
    v_max_ang = this->get_parameter("crear.tray.v_max_ang").as_double();
    t_a = this->get_parameter("crear.tray.t_a").as_double();
    if (stop_duration_sec_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "stop_duration_sec invalido; se usa 2.0 s");
      stop_duration_sec_ = 2.0;
    }
    if (!std::isfinite(waypoint_blend_sec_) || waypoint_blend_sec_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "waypoint_blend_sec invalido; se usa 3.0 s");
      waypoint_blend_sec_ = 3.0;
    }
    navigation_state_timeout_sec_ =
      this->get_parameter("navigation_state_timeout_sec").as_double();
    if (navigation_state_timeout_sec_ <= 0.0) {
      navigation_state_timeout_sec_ = 0.5;
    }
    orb_loss_hold_sec_ = this->get_parameter("orb_loss_hold_sec").as_double();
    if (orb_loss_hold_sec_ <= 0.0) {
      orb_loss_hold_sec_ = 10.0;
    }
    trajectory_source_handshake_timeout_sec_ =
      this->get_parameter("trajectory_source_handshake_timeout_sec").as_double();
    if (trajectory_source_handshake_timeout_sec_ <= 0.0) {
      trajectory_source_handshake_timeout_sec_ = 1.0;
    }
  }

private:
  void PublishCameraPitch(double target_rad)
  {
    if (!std::isfinite(target_rad) || !camera_pitch_publisher_) {
      return;
    }
    trajectory_msgs::msg::JointTrajectory command;
    command.header.stamp = now();
    command.joint_names = {"stereo_pitch_joint"};
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {target_rad};
    command.points = {point};
    camera_pitch_publisher_->publish(command);
    RCLCPP_WARN(
      get_logger(), "[F6M-CAMERA-PITCH] target_rad=%.6f target_deg=%.3f",
      target_rad, target_rad * 180.0 / M_PI);
  }

  void EmitArchitectureActivity(
    const std::string & edge_id,
    const std::string & interface_name,
    const std::string & interface_kind)
  {
    if (!debug_architecture_telemetry_ || !architecture_activity_pub_) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    const auto found = architecture_last_emit_.find(edge_id);
    if (found != architecture_last_emit_.end() &&
      now - found->second < std::chrono::milliseconds(100))
    {
      return;
    }
    architecture_last_emit_[edge_id] = now;
    std::ostringstream json;
    json << "{\"kind\":\"architecture_activity\",\"edge_id\":\""
         << edge_id << "\",\"interface\":\"" << interface_name
         << "\",\"interface_kind\":\"" << interface_kind
         << "\",\"source\":\"gen_tray\",\"namespace\":\""
         << this->get_namespace() << "\",\"drone_id\":" << drone_id_
         << ",\"timestamp\":" << this->get_clock()->now().seconds() << "}";
    std_msgs::msg::String message;
    message.data = json.str();
    architecture_activity_pub_->publish(message);
  }

  ///////////////// CALLBACKS /////////////////
  void navigation_state_callback(
    const orbslam3_msgs::msg::NavigationState::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(navigation_state_mtx_);
      last_navigation_state_ = *msg;
      navigation_state_received_at_ = std::chrono::steady_clock::now();
      navigation_state_received_ = true;
      if (msg->global_valid) {
        const auto o_t_body = PoseFromMessage(msg->o_t_body);
        const auto w_t_body = PoseFromMessage(msg->w_t_body);
        control_t_world_ = dron_individual::Compose(
          o_t_body, dron_individual::Inverse(w_t_body));
        control_t_world_valid_ = true;
        control_t_world_epoch_ = msg->map_epoch;
      } else if (
        msg->pose_source ==
        orbslam3_msgs::msg::NavigationState::POSE_SOURCE_GT_FALLBACK ||
        msg->pose_source ==
        orbslam3_msgs::msg::NavigationState::POSE_SOURCE_GT_FORCED)
      {
        // TODO FASE 6: retirar esta composicion cuando desaparezca GT_FALLBACK.
        // El mux transporta W_T_B de GT con global_valid=false solo para mantener
        // los goals absolutos en el mismo O durante una perdida.
        const auto o_t_body = PoseFromMessage(msg->o_t_body);
        const auto w_t_body = PoseFromMessage(msg->w_t_body);
        control_t_world_ = dron_individual::Compose(
          o_t_body, dron_individual::Inverse(w_t_body));
        control_t_world_valid_ = true;
        control_t_world_epoch_ = msg->map_epoch;
      }
    }
    navigation_state_cv_.notify_all();
  }

  bool SetTrajectoryActive(bool active, bool wait_for_boundary_state)
  {
    const auto timeout = std::chrono::duration<double>(
      trajectory_source_handshake_timeout_sec_);
    uint64_t previous_sequence = 0;
    bool previous_received = false;
    {
      std::lock_guard<std::mutex> lock(navigation_state_mtx_);
      previous_received = navigation_state_received_;
      previous_sequence = last_navigation_state_.sample_sequence;
    }
    if (!trajectory_active_client_->wait_for_service(timeout)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[F5H-TRAJECTORY-SOURCE-HANDSHAKE] active=%s success=false reason=service_unavailable",
        active ? "true" : "false");
      return false;
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = active;
    auto future = trajectory_active_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready || !future.get()->success) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[F5H-TRAJECTORY-SOURCE-HANDSHAKE] active=%s success=false reason=request_failed",
        active ? "true" : "false");
      return false;
    }
    if (wait_for_boundary_state && previous_received) {
      std::unique_lock<std::mutex> lock(navigation_state_mtx_);
      if (!navigation_state_cv_.wait_for(
          lock, timeout,
          [this, previous_sequence]() {
            return last_navigation_state_.sample_sequence != previous_sequence &&
            last_navigation_state_.local_valid &&
            last_navigation_state_.local_continuity_valid &&
            last_navigation_state_.velocity_valid;
          }))
      {
        RCLCPP_ERROR(
          this->get_logger(),
          "[F5H-TRAJECTORY-SOURCE-HANDSHAKE] active=false success=false "
          "reason=boundary_state_timeout sample=%lu",
          static_cast<unsigned long>(previous_sequence));
        return false;
      }
    }
    RCLCPP_INFO(
      this->get_logger(),
      "[F5H-TRAJECTORY-SOURCE-HANDSHAKE] active=%s success=true",
      active ? "true" : "false");
    return true;
  }

  void ReleaseTrajectorySourceIfCurrent(
    const std::shared_ptr<GoalHandleTrayAction> & goal_handle)
  {
    bool current = false;
    {
      std::lock_guard<std::mutex> lock(active_mtx_);
      current = active_goal_ == goal_handle;
      if (current) {
        active_goal_.reset();
      }
    }
    if (current) {
      PublishTrajectoryActive(false);
      RCLCPP_INFO(
        this->get_logger(),
        "[F5H-SOURCE-RETAINED-BETWEEN-GOALS] waiting_for_next_goal=true");
    }
  }

  void PublishTrajectoryActive(bool active)
  {
    std_msgs::msg::Bool message;
    message.data = active;
    trajectory_active_publisher_->publish(message);
    RCLCPP_INFO(
      get_logger(), "[F6K-PHYSICAL-TRAJECTORY-STATE] active=%s",
      active ? "true" : "false");
  }

  enum class GoalTerminal
  {
    kSucceeded,
    kAborted,
    kCanceled
  };

  void FinalizeGoal(
    const std::shared_ptr<GoalHandleTrayAction> & goal_handle,
    const std::shared_ptr<TrayAction::Result> & result, GoalTerminal terminal)
  {
    if (!goal_handle->is_active()) {
      RCLCPP_WARN(
        get_logger(), "[F6I-GOAL-TERMINAL-SKIP] reason=goal_not_active");
      return;
    }
    try {
      switch (terminal) {
        case GoalTerminal::kSucceeded:
          goal_handle->succeed(result);
          return;
        case GoalTerminal::kCanceled:
          goal_handle->canceled(result);
          return;
        case GoalTerminal::kAborted:
          goal_handle->abort(result);
          return;
      }
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        get_logger(), "[F6I-GOAL-TERMINAL-SKIP] reason=%s", error.what());
    }
  }

  dron_individual::RigidPose PoseFromMessage(const geometry_msgs::msg::Pose & pose) const
  {
    dron_individual::RigidPose result;
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


  ///////////////// FUNCIONES /////////////////
  // Proyectar velocidad lineal máxima sobre cada eje
  double proyeccion(
    double velocidad_abs, char eje, double x0, double y0, double z0, double xf,
    double yf, double zf)
  {
    const double dx = xf - x0;
    const double dy = yf - y0;
    const double dz = zf - z0;

    const double L = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (L <= 1e-9) {
      return 0.0;
    }

    double comp = 0.0;

    if (eje == 'X') {comp = std::abs(dx / L);} else if (eje == 'Y') {
      comp = std::abs(dy / L);
    } else if (eje == 'Z') {comp = std::abs(dz / L);}

    double vel_eje = comp * velocidad_abs;

    return vel_eje;
  }

  // Obtener "Yaw" de la variable tipo "Pose"
  double pose2yaw(const geometry_msgs::msg::PoseStamped & pose_stamped)
  {
    double x = pose_stamped.pose.orientation.x;
    double y = pose_stamped.pose.orientation.y;
    double z = pose_stamped.pose.orientation.z;
    double w = pose_stamped.pose.orientation.w;

    double seno = 2.0 * (w * z + x * y);
    double coseno = 1.0 - 2.0 * (y * y + z * z);

    double yaw = std::atan2(seno, coseno);
    return yaw;
  }

  // Normalizar ángulo entre [-pi, pi]
  double normalizar_angulo(double angulo)
  {
    while (angulo > M_PI) {
      angulo -= 2.0 * M_PI;
    }

    while (angulo < -M_PI) {
      angulo += 2.0 * M_PI;
    }

    return angulo;
  }

  // Pasar de std::array a Float64MultiArray
  std_msgs::msg::Float64MultiArray array_to_msg(const std::array<double, 5> & valores)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(valores.begin(), valores.end());
    return msg;
  }


  ///////////////// FUNCIONES ACCIÓN /////////////////
  // Se llama cuando el cliente de la acción realiza la petición
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const TrayAction::Goal> goal)
  {
    if (debug_architecture_telemetry_) {
      EmitArchitectureActivity(
        "sim_to_dron_action", "AccionTrayectoria", "action");
    }
    if (goal->tipo_trayectoria > 2) {
      RCLCPP_WARN(this->get_logger(), "tipo_trayectoria no válido: %u", goal->tipo_trayectoria);
      return rclcpp_action::GoalResponse::REJECT;
    }

    dron_individual::NavigationGoalState state;
    {
      std::lock_guard<std::mutex> lock(navigation_state_mtx_);
      state.received = navigation_state_received_;
      if (navigation_state_received_) {
        const double age_sec = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - navigation_state_received_at_).count();
        state.fresh = age_sec <= navigation_state_timeout_sec_;
        state.local_valid = last_navigation_state_.local_valid;
        state.local_continuity_valid =
          last_navigation_state_.local_continuity_valid;
        state.global_valid = last_navigation_state_.global_valid;
        state.absolute_frame_valid = control_t_world_valid_ &&
          control_t_world_epoch_ == last_navigation_state_.map_epoch;
        state.velocity_valid = last_navigation_state_.velocity_valid;
        state.gt_fallback = last_navigation_state_.pose_source ==
          orbslam3_msgs::msg::NavigationState::POSE_SOURCE_GT_FALLBACK;
        state.gt_forced = last_navigation_state_.pose_source ==
          orbslam3_msgs::msg::NavigationState::POSE_SOURCE_GT_FORCED;
        state.map_epoch = last_navigation_state_.map_epoch;
        state.sample_sequence = last_navigation_state_.sample_sequence;
      }
    }

    const bool requests_absolute =
      goal->absoluto_x || goal->absoluto_y || goal->absoluto_z ||
      goal->absoluto_yaw;
    const auto decision = dron_individual::EvaluateNavigationGoal(requests_absolute, state);
    if (decision != dron_individual::NavigationGoalDecision::ACCEPT_RELATIVE &&
      decision != dron_individual::NavigationGoalDecision::ACCEPT_ABSOLUTE)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "[F5B-GOAL-REJECT] decision=%s absolute=%s state_received=%s "
        "state_fresh=%s local_valid=%s continuity_valid=%s global_valid=%s "
        "absolute_frame_valid=%s "
        "velocity_valid=%s gt_fallback=%s epoch=%lu sample=%lu",
        dron_individual::NavigationGoalDecisionName(decision),
        requests_absolute ? "true" : "false",
        state.received ? "true" : "false",
        state.fresh ? "true" : "false",
        state.local_valid ? "true" : "false",
        state.local_continuity_valid ? "true" : "false",
        state.global_valid ? "true" : "false",
        state.absolute_frame_valid ? "true" : "false",
        state.velocity_valid ? "true" : "false",
        state.gt_fallback ? "true" : "false",
        static_cast<unsigned long>(state.map_epoch),
        static_cast<unsigned long>(state.sample_sequence));
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "[F5B-GOAL-ACCEPT] decision=%s absolute=%s snapshot_epoch=%lu "
      "snapshot_sample=%lu source=%s",
      dron_individual::NavigationGoalDecisionName(decision),
      requests_absolute ? "true" : "false",
      static_cast<unsigned long>(state.map_epoch),
      static_cast<unsigned long>(state.sample_sequence),
      state.gt_forced ? "gt_forced" : (state.gt_fallback ? "gt_fallback" : "orb"));

    // Si devolvemos (ACCEPT_AND_EXECUTE) se llama a la función "handle_accepted" (y se interrumpe la acción anterior)
    // Si devolvemos (ACCEPT_AND_DEFER) se espera a que termine la acción anterior y se llama a "handle_accepted"
    // Si devolvemos (REJECT) rechazamos el goal y no se llama a "handle_accepted"

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Se llama cuando el cliente cancela la acción (por ejemplo: goal_handle.cancel_goal_async()).
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleTrayAction>/*goal_handle*/)
  {
    RCLCPP_INFO(this->get_logger(), "Petición de cancelación recibida");

    // Si devolvemos (ACCEPT), la acción se cancela.
    // Si devolvemos (REJECT), la acción sigue.
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Se llama cuando se acepta la petición en "handle_goal"
  void handle_accepted(const std::shared_ptr<GoalHandleTrayAction> goal_handle)
  {
    // El worker saliente detecta que deja de ser el activo y es el unico que
    // publica su resultado terminal. Abortarlo aqui dejaría su hilo vivo y
    // provocaría una segunda finalizacion del mismo goal.
    {
      std::lock_guard<std::mutex> lock(active_mtx_);
      active_goal_ = goal_handle;
    }
    PublishTrajectoryActive(true);

    std::thread(&Clase_Servicio_Accion::execute, this, goal_handle).detach();
  }

  void ExecutePol3Waypoints(
    const std::shared_ptr<GoalHandleTrayAction> & goal_handle,
    const orbslam3_msgs::msg::NavigationState & initial_state,
    const dron_individual::RigidPose & control_t_world)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<TrayAction::Result>();
    if (goal->waypoint_targets.empty() ||
      goal->waypoint_targets.size() != goal->waypoint_times.size() ||
      (!goal->waypoint_yaws_rad.empty() &&
      goal->waypoint_targets.size() != goal->waypoint_yaws_rad.size()) ||
      (!goal->waypoint_camera_pitches_rad.empty() &&
      goal->waypoint_targets.size() != goal->waypoint_camera_pitches_rad.size()))
    {
      result->success = false;
      result->t_total = 0.0F;
      result->reason = "invalid_waypoint_contract";
      ReleaseTrajectorySourceIfCurrent(goal_handle);
      FinalizeGoal(goal_handle, result, GoalTerminal::kAborted);
      return;
    }

    const bool has_yaw = !goal->waypoint_yaws_rad.empty();
    const bool has_pitch = !goal->waypoint_camera_pitches_rad.empty();
    const double control_yaw = std::atan2(
      2.0 * (control_t_world.rotation.w() * control_t_world.rotation.z() +
      control_t_world.rotation.x() * control_t_world.rotation.y()),
      1.0 - 2.0 * (control_t_world.rotation.y() * control_t_world.rotation.y() +
      control_t_world.rotation.z() * control_t_world.rotation.z()));
    std::vector<std::vector<double>> targets;
    std::vector<double> times;
    targets.reserve(goal->waypoint_targets.size());
    times.reserve(goal->waypoint_times.size());
    for (std::size_t index = 0U; index < goal->waypoint_targets.size(); ++index) {
      const double time = static_cast<double>(goal->waypoint_times[index]);
      if (!std::isfinite(time) || time <= 0.0 ||
        (index > 0U && time <= times.back()))
      {
        result->success = false;
        result->t_total = 0.0F;
        result->reason = "invalid_waypoint_times";
        ReleaseTrajectorySourceIfCurrent(goal_handle);
        FinalizeGoal(goal_handle, result, GoalTerminal::kAborted);
        return;
      }
      const auto & world = goal->waypoint_targets[index].pose.position;
      const Eigen::Vector3d control_target =
        control_t_world.translation + control_t_world.rotation *
        Eigen::Vector3d(world.x, world.y, world.z);
      std::vector<double> target = {control_target.x(), control_target.y(), control_target.z()};
      if (has_yaw) {
        // TrajectoryPlan define yaw en W; Pol3 controla en O igual que la ruta directa.
        target.push_back(normalizar_angulo(control_yaw + goal->waypoint_yaws_rad[index]));
      }
      targets.push_back(std::move(target));
      times.push_back(time);
    }

    const geometry_msgs::msg::PoseStamped initial_pose = [&initial_state]() {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose = initial_state.o_t_body;
        return pose;
      }();
    const double yaw0 = pose2yaw(initial_pose);
    const std::size_t axes = has_yaw ? 4U : 3U;
    lib_tray::GenTrayPol3Waypoints trajectory(axes, waypoint_blend_sec_);
    try {
      trajectory.calcular_trayectoria(
        has_yaw ? std::vector<double>{initial_state.o_t_body.position.x,
          initial_state.o_t_body.position.y, initial_state.o_t_body.position.z, yaw0} :
        std::vector<double>{initial_state.o_t_body.position.x,
          initial_state.o_t_body.position.y, initial_state.o_t_body.position.z},
        has_yaw ? std::vector<double>{initial_state.velocity.linear.x,
          initial_state.velocity.linear.y, initial_state.velocity.linear.z,
          initial_state.velocity.angular.z} :
        std::vector<double>{initial_state.velocity.linear.x,
          initial_state.velocity.linear.y, initial_state.velocity.linear.z},
        targets, times);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "[F6I-POL3-WAYPOINTS-REJECT] reason=%s", error.what());
      result->success = false;
      result->t_total = 0.0F;
      result->reason = "invalid_waypoint_profile";
      ReleaseTrajectorySourceIfCurrent(goal_handle);
      FinalizeGoal(goal_handle, result, GoalTerminal::kAborted);
      return;
    }
    const double total = trajectory.get_tiempo_total();
    RCLCPP_WARN(
      get_logger(),
      "[F6M-WAYPOINT-CONTRACT] trajectory_id=%s targets=%zu has_yaw=%s has_pitch=%s "
      "final_pitch_rad=%.6f",
      goal->trajectory_id.empty() ? "local" : goal->trajectory_id.c_str(), targets.size(),
      has_yaw ? "true" : "false", has_pitch ? "true" : "false",
      has_pitch ? goal->waypoint_camera_pitches_rad.back() : 0.0);
    if (has_pitch) {
      PublishCameraPitch(goal->waypoint_camera_pitches_rad.back());
    }
    if (debug_f6i_trajectory_) {
      RCLCPP_WARN(
        get_logger(),
        "[F6I-REFERENCE-START] trajectory_id=%s targets=%zu pieces=%zu duration_sec=%.3f "
        "blend_sec=%.3f",
        goal->trajectory_id.empty() ? "local" : goal->trajectory_id.c_str(), targets.size(),
        trajectory.get_num_tramos(), total, waypoint_blend_sec_);
    }
    auto feedback = std::make_shared<TrayAction::Feedback>();
    rclcpp::Rate rate(30.0);
    const auto started = get_clock()->now();
    bool canceled = false;
    bool removed = false;
    bool first = true;
    double elapsed = 0.0;
    std::size_t previous_piece = trajectory.get_num_tramos();
    std::optional<std::chrono::steady_clock::time_point> orb_lost_since;
    while (rclcpp::ok() && !removed && elapsed <= total) {
      {
        std::lock_guard<std::mutex> lock(active_mtx_);
        removed = active_goal_ != goal_handle;
      }
      if (goal_handle->is_canceling()) {
        canceled = true;
        break;
      }
      bool orb_lost = false;
      {
        std::lock_guard<std::mutex> lock(navigation_state_mtx_);
        orb_lost = navigation_state_received_ &&
          last_navigation_state_.tracking_state ==
          orbslam3_msgs::msg::NavigationState::TRACKING_LOST &&
          !last_navigation_state_.local_valid;
      }
      if (orb_lost) {
        const auto now = std::chrono::steady_clock::now();
        if (!orb_lost_since.has_value()) {
          orb_lost_since = now;
          RCLCPP_ERROR(get_logger(), "[F6I-ORB-LOST-HOLD] duration_sec=%.1f", orb_loss_hold_sec_);
        }
        if (std::chrono::duration<double>(now - *orb_lost_since).count() >= orb_loss_hold_sec_) {
          removed = true;
          break;
        }
        rate.sleep();
        continue;
      }
      orb_lost_since.reset();
      elapsed = first ? 0.0 : (get_clock()->now() - started).seconds();
      first = false;
      std::vector<std::array<double, 5>> values;
      trajectory.evaluar(elapsed, values);
      const std::size_t piece = trajectory.get_indice_tramo(elapsed);
      if (debug_f6i_trajectory_ && piece != previous_piece) {
        if (piece == 0U) {
          RCLCPP_WARN(
            get_logger(),
            "[F6I-REFERENCE-PIECE] trajectory_id=%s piece=0 time_sec=%.6f event=start "
            "position=(%.6f,%.6f,%.6f) velocity=(%.6f,%.6f,%.6f)",
            goal->trajectory_id.empty() ? "local" : goal->trajectory_id.c_str(), elapsed,
            values[0][0], values[1][0], values[2][0], values[0][1], values[1][1], values[2][1]);
        } else {
          const double boundary = trajectory.get_inicio_tramo(piece);
          const auto before = trajectory.evaluar(
            std::nextafter(boundary, -std::numeric_limits<double>::infinity()));
          const auto after = trajectory.evaluar(boundary);
          RCLCPP_WARN(
            get_logger(),
            "[F6I-REFERENCE-BOUNDARY] trajectory_id=%s from_piece=%zu to_piece=%zu "
            "boundary_sec=%.6f before_p=(%.6f,%.6f,%.6f) before_v=(%.6f,%.6f,%.6f) "
            "after_p=(%.6f,%.6f,%.6f) after_v=(%.6f,%.6f,%.6f)",
            goal->trajectory_id.empty() ? "local" : goal->trajectory_id.c_str(), piece - 1U,
            piece, boundary, before[0][0], before[1][0], before[2][0], before[0][1],
            before[1][1], before[2][1], after[0][0], after[1][0], after[2][0], after[0][1],
            after[1][1], after[2][1]);
        }
        previous_piece = piece;
      }
      const auto yaw = has_yaw ? values[3] : std::array<double, 5>{
        yaw0, 0.0, 0.0, 0.0, 100.0 * std::clamp(elapsed / total, 0.0, 1.0)};
      feedback->t_act = static_cast<float>(elapsed);
      feedback->trajectory_id = goal->trajectory_id;
      feedback->diagnostic_piece_index = static_cast<std::uint32_t>(piece);
      feedback->x = array_to_msg(values[0]);
      feedback->y = array_to_msg(values[1]);
      feedback->z = array_to_msg(values[2]);
      feedback->yaw = array_to_msg(yaw);
      feedback->camera_pitch = array_to_msg(
        std::array<double, 5>{
        has_pitch ? goal->waypoint_camera_pitches_rad.back() : 0.0,
        0.0, 0.0, 0.0, 100.0 * std::clamp(elapsed / total, 0.0, 1.0)});
      {
        std::lock_guard<std::mutex> lock(active_mtx_);
        if (active_goal_ != goal_handle) {
          removed = true;
          break;
        }
      }
      goal_handle->publish_feedback(feedback);
      rate.sleep();
    }
    ReleaseTrajectorySourceIfCurrent(goal_handle);
    result->success = !removed && !canceled && elapsed >= total;
    result->t_total = static_cast<float>(total);
    result->reason = result->success ? "completed" : (canceled ? "canceled" : "aborted");
    RCLCPP_INFO(
      get_logger(), "[F6I-POL3-WAYPOINTS-FINAL] success=%s elapsed_sec=%.3f duration_sec=%.3f",
      result->success ? "true" : "false", elapsed, total);
    if (canceled) {
      FinalizeGoal(goal_handle, result, GoalTerminal::kCanceled);
    } else if (result->success) {
      FinalizeGoal(goal_handle, result, GoalTerminal::kSucceeded);
    } else {
      FinalizeGoal(goal_handle, result, GoalTerminal::kAborted);
    }
  }

  void ExecuteStop(
    const std::shared_ptr<GoalHandleTrayAction> & goal_handle,
    const orbslam3_msgs::msg::NavigationState & initial_state)
  {
    constexpr std::size_t kAxes = 4U;
    auto result = std::make_shared<TrayAction::Result>();
    geometry_msgs::msg::PoseStamped initial_pose;
    initial_pose.pose = initial_state.o_t_body;
    const std::vector<double> position = {
      initial_state.o_t_body.position.x,
      initial_state.o_t_body.position.y,
      initial_state.o_t_body.position.z,
      pose2yaw(initial_pose)};
    const std::vector<double> velocity = {
      initial_state.velocity.linear.x,
      initial_state.velocity.linear.y,
      initial_state.velocity.linear.z,
      initial_state.velocity.angular.z};
    const std::vector<double> duration(kAxes, stop_duration_sec_);
    auto trajectory = std::make_unique<lib_tray::GenTrayPol3Waypoints>(
      kAxes, waypoint_blend_sec_);
    try {
      trajectory->calcular_trayectoria_por_eje(position, velocity, {position}, {duration});
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "[F6I-STOP-ABORT] reason=%s", error.what());
      result->success = false;
      result->t_total = 0.0F;
      result->reason = "stop_generation_failed";
      ReleaseTrajectorySourceIfCurrent(goal_handle);
      FinalizeGoal(goal_handle, result, GoalTerminal::kAborted);
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "[F6I-STOP-START] duration_sec=%.3f pose=(%.3f,%.3f,%.3f) velocity=(%.3f,%.3f,%.3f)",
      stop_duration_sec_, position[0], position[1], position[2], velocity[0], velocity[1],
      velocity[2]);
    auto feedback = std::make_shared<TrayAction::Feedback>();
    rclcpp::Rate rate(30.0);
    const auto started = get_clock()->now();
    bool canceled = false;
    bool removed = false;
    bool first = true;
    double elapsed = 0.0;
    std::vector<std::array<double, 5>> values(kAxes);
    while (rclcpp::ok() && !removed && elapsed <= stop_duration_sec_) {
      {
        std::lock_guard<std::mutex> lock(active_mtx_);
        removed = active_goal_ != goal_handle;
      }
      if (goal_handle->is_canceling()) {
        canceled = true;
        break;
      }
      elapsed = first ? 0.0 : (get_clock()->now() - started).seconds();
      first = false;
      trajectory->evaluar(std::min(elapsed, stop_duration_sec_), values);
      feedback->t_act = static_cast<float>(std::min(elapsed, stop_duration_sec_));
      feedback->x = array_to_msg(values[0]);
      feedback->y = array_to_msg(values[1]);
      feedback->z = array_to_msg(values[2]);
      feedback->yaw = array_to_msg(values[3]);
      goal_handle->publish_feedback(feedback);
      rate.sleep();
    }

    const bool completed = !removed && !canceled && elapsed >= stop_duration_sec_;
    if (completed) {
      const auto hover_axis = [](double value) {
          return std::array<double, 5>{value, 0.0, 0.0, 0.0, 1.0};
        };
      feedback->t_act = static_cast<float>(stop_duration_sec_);
      feedback->x = array_to_msg(hover_axis(position[0]));
      feedback->y = array_to_msg(hover_axis(position[1]));
      feedback->z = array_to_msg(hover_axis(position[2]));
      feedback->yaw = array_to_msg(hover_axis(position[3]));
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(
        get_logger(), "[F6I-STOP-HOVER] duration_sec=%.3f", stop_duration_sec_);
    }
    ReleaseTrajectorySourceIfCurrent(goal_handle);
    result->success = completed;
    result->t_total = static_cast<float>(completed ? stop_duration_sec_ : elapsed);
    result->reason = completed ? "completed" :
      (canceled ? "stop_canceled" : "stop_replaced");
    RCLCPP_INFO(
      get_logger(), "[F6I-STOP-FINAL] success=%s elapsed_sec=%.3f duration_sec=%.3f",
      completed ? "true" : "false", elapsed, stop_duration_sec_);
    if (completed) {
      FinalizeGoal(goal_handle, result, GoalTerminal::kSucceeded);
    } else if (canceled) {
      FinalizeGoal(goal_handle, result, GoalTerminal::kCanceled);
    } else {
      FinalizeGoal(goal_handle, result, GoalTerminal::kAborted);
    }
  }


  ///////////////// FUNCIÓN PRINCIPAL /////////////////
  void execute(const std::shared_ptr<GoalHandleTrayAction> goal_handle)
  {
    auto result = std::make_shared<TrayAction::Result>();
    if (!SetTrajectoryActive(false, true) || !SetTrajectoryActive(true, false)) {
      auto failed_result = std::make_shared<TrayAction::Result>();
      failed_result->success = false;
      failed_result->t_total = 0.0f;
      ReleaseTrajectorySourceIfCurrent(goal_handle);
      FinalizeGoal(goal_handle, failed_result, GoalTerminal::kAborted);
      return;
    }

    orbslam3_msgs::msg::NavigationState initial_state;
    dron_individual::RigidPose control_t_world;
    bool initial_state_valid = false;
    {
      std::lock_guard<std::mutex> lock(navigation_state_mtx_);
      initial_state = last_navigation_state_;
      control_t_world = control_t_world_;
      initial_state_valid = navigation_state_received_ &&
        initial_state.local_valid && initial_state.local_continuity_valid &&
        initial_state.velocity_valid && control_t_world_valid_ &&
        control_t_world_epoch_ == initial_state.map_epoch;
    }
    if (!initial_state_valid) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[F5H-ATOMIC-GOAL-START] success=false reason=invalid_boundary_state");
      result->success = false;
      result->t_total = 0.0f;
      ReleaseTrajectorySourceIfCurrent(goal_handle);
      FinalizeGoal(goal_handle, result, GoalTerminal::kAborted);
      return;
    }

    if (goal_handle->get_goal()->stop_at_current_pose) {
      ExecuteStop(goal_handle, initial_state);
      return;
    }

    if (!goal_handle->get_goal()->waypoint_targets.empty()) {
      ExecutePol3Waypoints(goal_handle, initial_state, control_t_world);
      return;
    }

    PublishCameraPitch(goal_handle->get_goal()->target_camera_pitch_rad);

    // Para que cada vez que se llame a la acción se reinicien las variables.
    bool eliminado = false;                 // al declararse aquí, es variable local de cada thread
    bool cancelado = false;                 // necesario para goal_handle->canceled
    double t = 0.0;

    double t_total = 0.0;

    if (!eliminado) {
      auto feedback_msg = std::make_shared<TrayAction::Feedback>();               // Creamos el objeto de nuestra interfaz que utilizaremos como feedback de la acción

      // Obtenemos los valores del cliente de la acción para utilizarlos en el bucle
      auto goal = goal_handle->get_goal();

      Eigen::Vector3d absolute_target(
        goal->target_pose.pose.position.x,
        goal->target_pose.pose.position.y,
        goal->target_pose.pose.position.z);
      const Eigen::Vector3d world_target = absolute_target;
      if (goal->absoluto_x && goal->absoluto_y && goal->absoluto_z) {
        absolute_target = control_t_world.translation +
          control_t_world.rotation * absolute_target;
      }
      const double control_yaw = std::atan2(
        2.0 * (control_t_world.rotation.w() * control_t_world.rotation.z() +
        control_t_world.rotation.x() * control_t_world.rotation.y()),
        1.0 - 2.0 * (control_t_world.rotation.y() * control_t_world.rotation.y() +
        control_t_world.rotation.z() * control_t_world.rotation.z()));
      const double absolute_yaw = normalizar_angulo(
        control_yaw + pose2yaw(goal->target_pose));

      constexpr std::size_t N_EJES_TRAY = 4;

      geometry_msgs::msg::PoseStamped initial_pose;
      initial_pose.pose = initial_state.o_t_body;
      const double x0 = initial_state.o_t_body.position.x;
      const double y0 = initial_state.o_t_body.position.y;
      const double z0 = initial_state.o_t_body.position.z;
      const double yaw0 = pose2yaw(initial_pose);
      const double vx0 = initial_state.velocity.linear.x;
      const double vy0 = initial_state.velocity.linear.y;
      const double vz0 = initial_state.velocity.linear.z;
      const double vyaw0 = initial_state.velocity.angular.z;

      if (
        initial_state.pose_source ==
        orbslam3_msgs::msg::NavigationState::POSE_SOURCE_ORB &&
        goal->absoluto_x && goal->absoluto_y && goal->absoluto_z)
      {
        const auto o_t_body = PoseFromMessage(initial_state.o_t_body);
        const auto w_t_body = PoseFromMessage(initial_state.w_t_body);
        const Eigen::Vector3d world_x_in_control =
          control_t_world.rotation * Eigen::Vector3d::UnitX();
        const Eigen::Vector3d world_y_in_control =
          control_t_world.rotation * Eigen::Vector3d::UnitY();
        const Eigen::Vector3d world_z_in_control =
          control_t_world.rotation * Eigen::Vector3d::UnitZ();
        RCLCPP_WARN(
          this->get_logger(),
          "[F5H-ABSOLUTE-FRAME-DIAG] part=poses epoch=%lu sample=%lu "
          "global_valid=%s global_status=%u "
          "o_p=(%.6f,%.6f,%.6f) o_q=(%.6f,%.6f,%.6f,%.6f) "
          "w_p=(%.6f,%.6f,%.6f) w_q=(%.6f,%.6f,%.6f,%.6f)",
          static_cast<unsigned long>(initial_state.map_epoch),
          static_cast<unsigned long>(initial_state.sample_sequence),
          initial_state.global_valid ? "true" : "false",
          static_cast<unsigned>(initial_state.global_status),
          o_t_body.translation.x(), o_t_body.translation.y(), o_t_body.translation.z(),
          o_t_body.rotation.x(), o_t_body.rotation.y(), o_t_body.rotation.z(),
          o_t_body.rotation.w(),
          w_t_body.translation.x(), w_t_body.translation.y(), w_t_body.translation.z(),
          w_t_body.rotation.x(), w_t_body.rotation.y(), w_t_body.rotation.z(),
          w_t_body.rotation.w());
        RCLCPP_WARN(
          this->get_logger(),
          "[F5H-ABSOLUTE-FRAME-DIAG] part=target_axes epoch=%lu sample=%lu "
          "world_target=(%.6f,%.6f,%.6f) control_target=(%.6f,%.6f,%.6f) "
          "c_t_w_p=(%.6f,%.6f,%.6f) c_t_w_q=(%.6f,%.6f,%.6f,%.6f) "
          "world_x_in_control=(%.6f,%.6f,%.6f) "
          "world_y_in_control=(%.6f,%.6f,%.6f) "
          "world_z_in_control=(%.6f,%.6f,%.6f)",
          static_cast<unsigned long>(initial_state.map_epoch),
          static_cast<unsigned long>(initial_state.sample_sequence),
          world_target.x(), world_target.y(), world_target.z(),
          absolute_target.x(), absolute_target.y(), absolute_target.z(),
          control_t_world.translation.x(), control_t_world.translation.y(),
          control_t_world.translation.z(),
          control_t_world.rotation.x(), control_t_world.rotation.y(),
          control_t_world.rotation.z(), control_t_world.rotation.w(),
          world_x_in_control.x(), world_x_in_control.y(), world_x_in_control.z(),
          world_y_in_control.x(), world_y_in_control.y(), world_y_in_control.z(),
          world_z_in_control.x(), world_z_in_control.y(), world_z_in_control.z());
      }

      RCLCPP_WARN(
        this->get_logger(),
        "[F5H-ATOMIC-GOAL-START] success=true source=%u epoch=%lu sample=%lu "
        "x0=(%.6f,%.6f,%.6f) v0=(%.6f,%.6f,%.6f) yaw0=%.6f yaw_rate0=%.6f",
        static_cast<unsigned>(initial_state.pose_source),
        static_cast<unsigned long>(initial_state.map_epoch),
        static_cast<unsigned long>(initial_state.sample_sequence),
        x0, y0, z0, vx0, vy0, vz0, yaw0, vyaw0);

      std::vector<std::array<double, 5>> salida_trayectoria(N_EJES_TRAY);

      std::function<void(double, std::vector<std::array<double, 5>> &)> evaluar_trayectoria;

      std::unique_ptr<lib_tray::GenTrayElipse> trayectoria_elipse;
      std::unique_ptr<lib_tray::GenTrayPol3Waypoints> trayectoria_pol3_waypoints;
      std::unique_ptr<lib_tray::GenTrayVelTrapWaypoints> trayectoria_veltrap_waypoints;

      bool flag_angulo = false;
      bool usar_normalizacion_directa = false;

      try {
        ///////////////////////////////
        // TIPO 0: POLINOMIO CÚBICO //
        ///////////////////////////////
        if (goal->tipo_trayectoria == 0) {
          RCLCPP_INFO(this->get_logger(), "Generando trayectoria tipo 0: pol3");

          trayectoria_pol3_waypoints =
            std::make_unique<lib_tray::GenTrayPol3Waypoints>(
            N_EJES_TRAY, waypoint_blend_sec_);

          t_total = std::max(
            {static_cast<double>(goal->tx), static_cast<double>(goal->ty),
              static_cast<double>(goal->tz), static_cast<double>(goal->tyaw)});

          double xf, yf, yawf;

          if (goal->absoluto_x) {
            xf = absolute_target.x();
          } else {
            xf = x0 - goal->target_pose.pose.position.y * sin(yaw0) +
              goal->target_pose.pose.position.x * cos(yaw0);
          }

          if (goal->absoluto_y) {
            yf = absolute_target.y();
          } else {
            yf = y0 + goal->target_pose.pose.position.y * cos(yaw0) +
              goal->target_pose.pose.position.x * sin(yaw0);
          }

          if (goal->absoluto_yaw) {
            yawf = absolute_yaw;
          } else {
            yawf = yaw0 + pose2yaw(goal->target_pose);
            if (yawf > M_PI || yawf < -M_PI) {
              flag_angulo = true;
            }
          }

          const std::vector<double> posiciones_iniciales = {x0, y0, z0, yaw0};
          const std::vector<double> posiciones_finales =
          {xf, yf, goal->absoluto_z ? absolute_target.z() :
            z0 + goal->target_pose.pose.position.z, yawf};
          const std::vector<double> velocidades_iniciales = {vx0, vy0, vz0, vyaw0};
          const std::vector<double> tiempos_finales = {goal->tx, goal->ty, goal->tz, goal->tyaw};

          trayectoria_pol3_waypoints->calcular_trayectoria_por_eje(
            posiciones_iniciales, velocidades_iniciales, {posiciones_finales}, {tiempos_finales});

          evaluar_trayectoria = [&trayectoria_pol3_waypoints](double tiempo,
              std::vector<std::array<double,
              5>> & salida)
            {
              trayectoria_pol3_waypoints->evaluar(tiempo, salida);
            };
        }
        //////////////////////////////////////
        // TIPO 1: VELOCIDAD TRAPEZOIDAL   //
        //////////////////////////////////////
        else if (goal->tipo_trayectoria == 1) {
          RCLCPP_INFO(this->get_logger(), "Generando trayectoria tipo 1: veltrap");

          trayectoria_veltrap_waypoints =
            std::make_unique<lib_tray::GenTrayVelTrapWaypoints>(N_EJES_TRAY);

          double xf, yf, zf_abs, yawf;

          if (goal->absoluto_x) {
            xf = absolute_target.x();
          } else {
            xf = x0 - goal->target_pose.pose.position.y * sin(yaw0) +
              goal->target_pose.pose.position.x * cos(yaw0);
          }

          if (goal->absoluto_y) {
            yf = absolute_target.y();
          } else {
            yf = y0 + goal->target_pose.pose.position.y * cos(yaw0) +
              goal->target_pose.pose.position.x * sin(yaw0);
          }

          if (goal->absoluto_z) {
            zf_abs = absolute_target.z();
          } else {
            zf_abs = z0 + goal->target_pose.pose.position.z;
          }

          if (goal->absoluto_yaw) {
            yawf = absolute_yaw;
          } else {
            yawf = yaw0 + pose2yaw(goal->target_pose);
            if (yawf > M_PI || yawf < -M_PI) {
              flag_angulo = true;
            }
          }

          const double vmax_x = proyeccion(v_max_lin, 'X', x0, y0, z0, xf, yf, zf_abs);
          const double vmax_y = proyeccion(v_max_lin, 'Y', x0, y0, z0, xf, yf, zf_abs);
          const double vmax_z = proyeccion(v_max_lin, 'Z', x0, y0, z0, xf, yf, zf_abs);

          const std::vector<double> posiciones_iniciales = {x0, y0, z0, yaw0};
          const std::vector<double> posiciones_finales =
          {xf, yf, zf_abs, yawf};
          const std::vector<double> velocidades_iniciales = {vx0, vy0, vz0, vyaw0};
          const std::vector<double> velocidades_maximas = {vmax_x, vmax_y, vmax_z, v_max_ang};

          trayectoria_veltrap_waypoints->calcular_trayectoria(
            posiciones_iniciales, velocidades_iniciales, {posiciones_finales},
            velocidades_maximas, t_a);
          t_total = trayectoria_veltrap_waypoints->get_tiempo_total();

          evaluar_trayectoria = [&trayectoria_veltrap_waypoints](double tiempo,
              std::vector<std::array<double,
              5>> & salida)
            {
              trayectoria_veltrap_waypoints->evaluar(tiempo, salida);
            };
        }
        ////////////////////
        // TIPO 2: ELIPSE //
        ////////////////////
        else if (goal->tipo_trayectoria == 2) {
          RCLCPP_INFO(this->get_logger(), "Generando trayectoria tipo 2: elipse");

          trayectoria_elipse = std::make_unique<lib_tray::GenTrayElipse>(N_EJES_TRAY);

          double xc, yc, zc;                     // centro de la elipse
          double alpha;                          // ángulo del radio x de la elipse
          double yaw_ref;                        // yaw absoluto o offset para mirar al centro

          const double rx = static_cast<double>(goal->tx);
          const double ry = static_cast<double>(goal->ty);
          const double angulo_elipse = static_cast<double>(goal->tz);
          const double tiempo_vuelta = static_cast<double>(goal->tyaw);

          if (goal->absoluto_x) {
            xc = absolute_target.x();
          } else {
            xc = x0 - goal->target_pose.pose.position.y * sin(yaw0) +
              goal->target_pose.pose.position.x * cos(yaw0);
          }

          if (goal->absoluto_y) {
            yc = absolute_target.y();
          } else {
            yc = y0 + goal->target_pose.pose.position.y * cos(yaw0) +
              goal->target_pose.pose.position.x * sin(yaw0);
          }

          if (goal->absoluto_z) {
            zc = absolute_target.z();
          } else {
            zc = z0 + goal->target_pose.pose.position.z;
          }

          // Si el centro x/y es absoluto, el ángulo de la elipse es respecto a world.
          // Si el centro x/y es relativo, el ángulo de la elipse es respecto al dron.
          if (goal->absoluto_x && goal->absoluto_y) {
            alpha = angulo_elipse;
          } else {
            alpha = yaw0 + angulo_elipse;
          }

          if (goal->absoluto_yaw) {
            // Yaw fijo absoluto durante toda la vuelta
            yaw_ref = absolute_yaw;
          } else {
            // Offset respecto al yaw necesario para mirar al centro.
            // Si yaw_ref = 0, el dron mira siempre al centro.
            yaw_ref = pose2yaw(goal->target_pose);
          }

          const std::vector<double> posiciones_iniciales = {x0, y0, z0, yaw0};

          // centro_elipse = {xc, yc, zc, yaw_ref}
          const std::vector<double> centro_elipse = {xc, yc, zc, yaw_ref};

          // parametros_elipse = {rx, ry, alpha, tiempo_vuelta}
          const std::vector<double> parametros_elipse = {rx, ry, alpha, tiempo_vuelta};

          trayectoria_elipse->calcular_trayectoria(
            posiciones_iniciales, centro_elipse,
            parametros_elipse, goal->absoluto_yaw, t_a);

          t_total = trayectoria_elipse->get_coeficientes().tf;

          usar_normalizacion_directa = true;

          evaluar_trayectoria = [&trayectoria_elipse](double tiempo, std::vector<std::array<double,
              5>> & salida)
            {
              trayectoria_elipse->evaluar(tiempo, salida);
            };
        } else {
          throw std::invalid_argument("tipo_trayectoria no válido");
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Error calculando trayectoria: %s", e.what());

        result->success = false;
        result->t_total = 0.0f;
        ReleaseTrajectorySourceIfCurrent(goal_handle);
        FinalizeGoal(goal_handle, result, GoalTerminal::kAborted);

        return;
      }

      rclcpp::Rate rate(30.0);               // Delay para verlo más claro en el tópico feedback (junto a rate.sleep())

      auto t0 = this->get_clock()->now();
      RCLCPP_INFO(this->get_logger(), "t=%.6f", t0.seconds());
      bool first_feedback = true;
      std::optional<std::chrono::steady_clock::time_point> orb_lost_since;

      while (rclcpp::ok() && !eliminado && t <= t_total) {
        active_mtx_.lock();
        if (active_goal_ != goal_handle) {
          eliminado = true;
        }
        active_mtx_.unlock();

        if (!goal_handle->is_canceling()) {
          bool orb_lost = false;
          {
            std::lock_guard<std::mutex> lock(navigation_state_mtx_);
            orb_lost = navigation_state_received_ &&
              last_navigation_state_.tracking_state ==
              orbslam3_msgs::msg::NavigationState::TRACKING_LOST &&
              !last_navigation_state_.local_valid;
          }
          if (orb_lost) {
            const auto now = std::chrono::steady_clock::now();
            if (!orb_lost_since.has_value()) {
              orb_lost_since = now;
              RCLCPP_ERROR(
                this->get_logger(),
                "[F5-ORB-LOST-HOLD] duration_sec=%.1f action=freeze_reference",
                orb_loss_hold_sec_);
            }
            if (std::chrono::duration<double>(now - *orb_lost_since).count() >=
              orb_loss_hold_sec_)
            {
              RCLCPP_ERROR(
                this->get_logger(), "[F5-ORB-LOST-ABORT] hold_elapsed_sec=%.1f",
                orb_loss_hold_sec_);
              eliminado = true;
              continue;
            }
            rate.sleep();
            continue;
          }
          orb_lost_since.reset();
          t = first_feedback ? 0.0 : (this->get_clock()->now() - t0).seconds();
          first_feedback = false;

          evaluar_trayectoria(t, salida_trayectoria);

          // Normalizamos yaw para evitar saltos raros fuera de [-pi, pi].
          if (flag_angulo) {
            if (salida_trayectoria[3][0] > M_PI) {
              salida_trayectoria[3][0] -= 2 * M_PI;
            } else if (salida_trayectoria[3][0] < -M_PI) {
              salida_trayectoria[3][0] += 2 * M_PI;
            }
          }

          if (usar_normalizacion_directa) {
            salida_trayectoria[3][0] = normalizar_angulo(salida_trayectoria[3][0]);
          }

          feedback_msg->t_act = static_cast<float>(t);
          feedback_msg->x = array_to_msg(salida_trayectoria[0]);
          feedback_msg->y = array_to_msg(salida_trayectoria[1]);
          feedback_msg->z = array_to_msg(salida_trayectoria[2]);
          feedback_msg->yaw = array_to_msg(salida_trayectoria[3]);
          feedback_msg->camera_pitch = array_to_msg(
            std::array<double, 5>{
            goal->target_camera_pitch_rad, 0.0, 0.0, 0.0,
            100.0 * std::clamp(t / std::max(t_total, 1e-9), 0.0, 1.0)});

          {
            std::lock_guard<std::mutex> lock(active_mtx_);
            if (active_goal_ != goal_handle) {
              eliminado = true;
              continue;
            }
          }

          goal_handle->publish_feedback(feedback_msg);                  // Envía periódicamente los valores al tópico /feedback

          rate.sleep();
        } else {
          cancelado = true;
          eliminado = true;
        }
      }
    }

    ReleaseTrajectorySourceIfCurrent(goal_handle);
    if (eliminado) {
      result->success = false;
      result->t_total = 0.0f;
      RCLCPP_INFO(this->get_logger(), "Acción CANCELADA");
      if (cancelado) {
        FinalizeGoal(goal_handle, result, GoalTerminal::kCanceled);
      } else {
        FinalizeGoal(goal_handle, result, GoalTerminal::kAborted);
      }
    } else {
      result->success = true;
      result->t_total = static_cast<float>(t_total);
      FinalizeGoal(goal_handle, result, GoalTerminal::kSucceeded);
      RCLCPP_INFO(this->get_logger(), "Acción finalizada OK");
    }
  }

  rclcpp_action::Server<TrayAction>::SharedPtr objeto_servicio_accion;           // Creamos el objeto del servicio de la acción
  rclcpp::Subscription<orbslam3_msgs::msg::NavigationState>::SharedPtr
    sub_navigation_state_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr trajectory_active_client_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr camera_pitch_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trajectory_active_publisher_;

  std::mutex active_mtx_;
  std::mutex navigation_state_mtx_;
  std::condition_variable navigation_state_cv_;
  std::shared_ptr<GoalHandleTrayAction> active_goal_;        // goal que consideramos "activo"

  double v_max_lin;
  double stop_duration_sec_ = 2.0;
  double waypoint_blend_sec_ = 3.0;
  bool debug_f6i_trajectory_ = false;
  double v_max_ang;
  double t_a;
  double navigation_state_timeout_sec_{0.5};
  double orb_loss_hold_sec_{10.0};
  double trajectory_source_handshake_timeout_sec_{1.0};
  bool navigation_state_received_{false};
  orbslam3_msgs::msg::NavigationState last_navigation_state_;
  std::chrono::steady_clock::time_point navigation_state_received_at_;
  bool control_t_world_valid_{false};
  uint64_t control_t_world_epoch_{0};
  dron_individual::RigidPose control_t_world_;

  bool debug_architecture_telemetry_{false};
  uint32_t drone_id_{0};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr architecture_activity_pub_;
  std::map<std::string, std::chrono::steady_clock::time_point> architecture_last_emit_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto objeto_nodo = std::make_shared<Clase_Servicio_Accion>();
  rclcpp::spin(objeto_nodo);

  rclcpp::shutdown();
  return 0;
}
