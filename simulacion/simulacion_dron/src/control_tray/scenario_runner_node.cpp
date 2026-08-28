#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <future>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "std_msgs/msg/bool.hpp"
#include "dron_individual/action/tray_action.hpp"

#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

class ScenarioRunnerNode : public rclcpp::Node
{
public:
  using TrayAction = dron_individual::action::TrayAction;
  using GoalHandleTray = rclcpp_action::ClientGoalHandle<TrayAction>;
  using ActionClientTray = rclcpp_action::Client<TrayAction>;

  ScenarioRunnerNode()
  : Node("scenario_runner_node")
  {
    this->declare_parameter<std::string>("scenario_file", "");
    this->declare_parameter<std::string>("sim_config_file", "");
    this->declare_parameter<std::string>("action_name", "AccionTrayectoria");
    this->declare_parameter<std::string>("namespace_base", "");
    this->declare_parameter<double>("default_action_timeout_sec", 120.0);
    this->declare_parameter<std::string>(
      "mapping_backpressure_topic",
      "/global_mapping/backpressure_active");

    scenario_file_ = this->get_parameter("scenario_file").as_string();
    sim_config_file_ = this->get_parameter("sim_config_file").as_string();
    action_name_ = this->get_parameter("action_name").as_string();
    namespace_base_ = this->get_parameter("namespace_base").as_string();
    default_action_timeout_sec_ =
      this->get_parameter("default_action_timeout_sec").as_double();
    mapping_backpressure_topic_ =
      this->get_parameter("mapping_backpressure_topic").as_string();

    if (default_action_timeout_sec_ <= 0.0) {
      default_action_timeout_sec_ = 120.0;
    }
    LoadSimulationConfigIfNeeded();

    mapping_backpressure_sub_ =
      this->create_subscription<std_msgs::msg::Bool>(
      mapping_backpressure_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1))
      .reliable()
      .transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      {
        const bool previous =
        mapping_backpressure_active_.exchange(msg->data);
        if (previous != msg->data) {
          RCLCPP_WARN(
            this->get_logger(),
            "[SCENARIO-RUNNER-BACKPRESSURE] active=%s previous=%s source_topic=%s policy=gate_between_moves",
            msg->data ? "true" : "false",
            previous ? "true" : "false",
            mapping_backpressure_topic_.c_str());
        }
      });

    RCLCPP_INFO(
      this->get_logger(),
      "[SCENARIO-RUNNER-INIT] scenario_file='%s' action_name='%s' namespace_base='%s' default_timeout=%.2f backpressure_topic='%s' backpressure_policy=gate_between_moves wait_steps_blocked=false",
      scenario_file_.c_str(),
      action_name_.c_str(),
      namespace_base_.c_str(),
      default_action_timeout_sec_,
      mapping_backpressure_topic_.c_str());
  }

  bool Run()
  {
    if (scenario_file_.empty()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[SCENARIO-RUNNER-ERROR] scenario_file parameter is empty");

      return false;
    }

    YAML::Node root;

    try {
      root = YAML::LoadFile(scenario_file_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[SCENARIO-RUNNER-ERROR] Could not load scenario YAML '%s': %s",
        scenario_file_.c_str(),
        e.what());

      return false;
    }

    ApplyScenarioOverrides(root);

    const std::string scenario_name =
      YamlGet<std::string>(root, "name", "unnamed_scenario");

    const std::string description =
      YamlGet<std::string>(root, "description", "");

    RCLCPP_WARN(
      this->get_logger(),
      "[SCENARIO-RUNNER-START] name='%s' description='%s' action_name='%s' namespace_base='%s'",
      scenario_name.c_str(),
      description.c_str(),
      action_name_.c_str(),
      namespace_base_.c_str());

    if (!root["steps"] || !root["steps"].IsSequence()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[SCENARIO-RUNNER-ERROR] YAML must contain a sequence field named 'steps'");

      return false;
    }

    const YAML::Node steps = root["steps"];

    for (std::size_t i = 0; i < steps.size(); ++i) {
      const YAML::Node step = steps[i];

      const std::string step_name =
        YamlGet<std::string>(
        step,
        "name",
        "step_" + std::to_string(i + 1));

      const std::string step_type =
        YamlGet<std::string>(step, "type", "move");

      RCLCPP_WARN(
        this->get_logger(),
        "[SCENARIO-RUNNER-STEP-START] index=%zu name='%s' type='%s'",
        i + 1,
        step_name.c_str(),
        step_type.c_str());

      bool ok = false;

      if (step_type == "wait") {
        ok = ExecuteWaitStep(step);
      } else if (step_type == "wait_for_bool") {
        ok = ExecuteWaitForBoolStep(step);
      } else if (step_type == "move") {
        ok = ExecuteMoveStep(step);
      } else {
        RCLCPP_ERROR(
          this->get_logger(),
          "[SCENARIO-RUNNER-ERROR] Unknown step type '%s' in step '%s'",
          step_type.c_str(),
          step_name.c_str());

        return false;
      }

      if (!ok) {
        RCLCPP_ERROR(
          this->get_logger(),
          "[SCENARIO-RUNNER-STEP-FAILED] index=%zu name='%s'",
          i + 1,
          step_name.c_str());

        return false;
      }

      RCLCPP_WARN(
        this->get_logger(),
        "[SCENARIO-RUNNER-STEP-DONE] index=%zu name='%s'",
        i + 1,
        step_name.c_str());
    }

    RCLCPP_WARN(
      this->get_logger(),
      "[SCENARIO-RUNNER-DONE] scenario='%s' success=true",
      scenario_name.c_str());

    return true;
  }

private:
  struct GoalSpec
  {
    std::string drone;
    std::string action_full_name;

    uint8_t tipo_trayectoria = 0;

    std::string frame_id = "map";

    double x = 0.0;
    double y = 0.0;
    double z = 1.0;
    double yaw_deg = 0.0;

    float tx = 20.0f;
    float ty = 20.0f;
    float tz = 20.0f;
    float tyaw = 20.0f;

    bool absoluto_x = true;
    bool absoluto_y = true;
    bool absoluto_z = true;
    bool absoluto_yaw = true;
    bool expect_rejected = false;

    double timeout_sec = 120.0;
  };

  struct ActiveGoal
  {
    GoalSpec spec;
    ActionClientTray::SharedPtr client;
    std::shared_future<GoalHandleTray::SharedPtr> goal_handle_future;
    GoalHandleTray::SharedPtr goal_handle;
    std::shared_future<GoalHandleTray::WrappedResult> result_future;
    bool completed = false;
  };

  enum class GoalBatchOutcome
  {
    Completed,
    Failed,
  };

private:
  std::string scenario_file_;
  std::string sim_config_file_;
  std::string action_name_;
  std::string namespace_base_;
  double default_action_timeout_sec_;
  std::string mapping_backpressure_topic_;

  std::map<std::string, ActionClientTray::SharedPtr> action_clients_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mapping_backpressure_sub_;
  std::atomic<bool> mapping_backpressure_active_{false};

private:
  template<typename T>
  static T YamlGet(
    const YAML::Node & node,
    const std::string & key,
    const T & default_value)
  {
    if (!node || !node[key]) {
      return default_value;
    }

    return node[key].as<T>();
  }

  void LoadSimulationConfigIfNeeded()
  {
    if (!namespace_base_.empty()) {
      return;
    }

    std::string cfg_path = sim_config_file_;

    if (cfg_path.empty()) {
      try {
        cfg_path =
          ament_index_cpp::get_package_share_directory("simulacion_dron") +
          "/config/sim_dron.yaml";
      } catch (const std::exception & e) {
        RCLCPP_WARN(
          this->get_logger(),
          "[SCENARIO-RUNNER-WARN] Could not find simulacion_dron share directory: %s",
          e.what());

        namespace_base_ = "dron";
        return;
      }
    }

    try {
      YAML::Node cfg = YAML::LoadFile(cfg_path);

      if (cfg["/**"] &&
        cfg["/**"]["ros__parameters"] &&
        cfg["/**"]["ros__parameters"]["dron.namespace_base"])
      {
        namespace_base_ =
          cfg["/**"]["ros__parameters"]["dron.namespace_base"].as<std::string>();
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        this->get_logger(),
        "[SCENARIO-RUNNER-WARN] Could not read sim config '%s': %s",
        cfg_path.c_str(),
        e.what());
    }

    if (namespace_base_.empty()) {
      namespace_base_ = "dron";
    }
  }

  void ApplyScenarioOverrides(const YAML::Node & root)
  {
    if (root["action_name"]) {
      action_name_ = root["action_name"].as<std::string>();
    }

    if (root["namespace_base"]) {
      namespace_base_ = root["namespace_base"].as<std::string>();
    }

    if (root["defaults"] && root["defaults"]["timeout_sec"]) {
      default_action_timeout_sec_ =
        root["defaults"]["timeout_sec"].as<double>();
    }

    if (default_action_timeout_sec_ <= 0.0) {
      default_action_timeout_sec_ = 120.0;
    }
  }

  bool ExecuteWaitStep(const YAML::Node & step)
  {
    const double seconds =
      YamlGet<double>(step, "seconds", 1.0);

    if (seconds <= 0.0) {
      return true;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "[SCENARIO-RUNNER-WAIT] seconds=%.3f backpressure_blocks_wait=false",
      seconds);

    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
      const double elapsed =
        std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
      if (elapsed >= seconds) {
        break;
      }
      std::this_thread::sleep_for(100ms);
    }

    return rclcpp::ok();
  }

  bool ExecuteWaitForBoolStep(const YAML::Node & step)
  {
    const std::string topic = YamlGet<std::string>(step, "topic", "");
    const bool expected = YamlGet<bool>(step, "expected", true);
    const double timeout_sec = YamlGet<double>(step, "timeout_sec", 60.0);
    if (topic.empty() || timeout_sec <= 0.0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[SCENARIO-RUNNER-ERROR] wait_for_bool requires topic and timeout_sec > 0");
      return false;
    }

    auto matched = std::make_shared<std::atomic<bool>>(false);
    auto subscription = this->create_subscription<std_msgs::msg::Bool>(
      topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [matched, expected](const std_msgs::msg::Bool::SharedPtr msg)
      {
        if (msg->data == expected) {
          matched->store(true);
        }
      });
    RCLCPP_WARN(
      this->get_logger(),
      "[SCENARIO-RUNNER-READY-WAIT] topic='%s' expected=%s timeout_sec=%.3f",
      topic.c_str(), expected ? "true" : "false", timeout_sec);
    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !matched->load()) {
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
      if (elapsed >= timeout_sec) {
        RCLCPP_ERROR(
          this->get_logger(),
          "[SCENARIO-RUNNER-READY-TIMEOUT] topic='%s' timeout_sec=%.3f",
          topic.c_str(), timeout_sec);
        return false;
      }
      std::this_thread::sleep_for(100ms);
    }
    RCLCPP_WARN(
      this->get_logger(),
      "[SCENARIO-RUNNER-READY] topic='%s' expected=%s",
      topic.c_str(), expected ? "true" : "false");
    return rclcpp::ok();
  }

  bool ExecuteMoveStep(const YAML::Node & step)
  {
    if (!step["goals"] || !step["goals"].IsSequence()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[SCENARIO-RUNNER-ERROR] move step must contain sequence field 'goals'");

      return false;
    }

    const std::string mode =
      YamlGet<std::string>(step, "mode", "simultaneous");

    const YAML::Node goals_node = step["goals"];

    std::vector<GoalSpec> goals;
    goals.reserve(goals_node.size());

    for (std::size_t i = 0; i < goals_node.size(); ++i) {
      try {
        goals.push_back(ParseGoal(goals_node[i]));
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          this->get_logger(),
          "[SCENARIO-RUNNER-ERROR] Could not parse goal %zu: %s",
          i,
          e.what());

        return false;
      }
    }

    if (goals.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "[SCENARIO-RUNNER-WARN] move step has no goals");

      return true;
    }

    if (mode == "sequential") {
      return ExecuteSequentialGoals(goals);
    }

    if (mode == "simultaneous") {
      return ExecuteSimultaneousGoals(goals);
    }

    RCLCPP_ERROR(
      this->get_logger(),
      "[SCENARIO-RUNNER-ERROR] Unknown move mode '%s'. Valid modes: sequential, simultaneous",
      mode.c_str());

    return false;
  }

  GoalSpec ParseGoal(const YAML::Node & goal_node)
  {
    GoalSpec spec;

    const YAML::Node defaults = YAML::Node();

    spec.tipo_trayectoria =
      static_cast<uint8_t>(
      YamlGet<int>(goal_node, "tipo_trayectoria", 0));

    spec.frame_id =
      YamlGet<std::string>(goal_node, "frame_id", "map");

    spec.tx =
      static_cast<float>(
      YamlGet<double>(goal_node, "tx", 20.0));

    spec.ty =
      static_cast<float>(
      YamlGet<double>(goal_node, "ty", 20.0));

    spec.tz =
      static_cast<float>(
      YamlGet<double>(goal_node, "tz", 20.0));

    spec.tyaw =
      static_cast<float>(
      YamlGet<double>(goal_node, "tyaw", 20.0));

    spec.absoluto_x =
      YamlGet<bool>(goal_node, "absoluto_x", true);

    spec.absoluto_y =
      YamlGet<bool>(goal_node, "absoluto_y", true);

    spec.absoluto_z =
      YamlGet<bool>(goal_node, "absoluto_z", true);

    spec.absoluto_yaw =
      YamlGet<bool>(goal_node, "absoluto_yaw", true);

    spec.expect_rejected =
      YamlGet<bool>(goal_node, "expect_rejected", false);

    spec.timeout_sec =
      YamlGet<double>(
      goal_node,
      "timeout_sec",
      default_action_timeout_sec_);

    if (spec.timeout_sec <= 0.0) {
      spec.timeout_sec = default_action_timeout_sec_;
    }

    if (goal_node["action_full_name"]) {
      spec.action_full_name =
        goal_node["action_full_name"].as<std::string>();
    }

    if (goal_node["drone"]) {
      spec.drone =
        goal_node["drone"].as<std::string>();
    } else if (goal_node["drone_id"]) {
      const int drone_id =
        goal_node["drone_id"].as<int>();

      spec.drone =
        namespace_base_ + "_" + std::to_string(drone_id);
    } else if (spec.action_full_name.empty()) {
      throw std::runtime_error(
              "goal must contain 'drone', 'drone_id' or 'action_full_name'");
    }

    if (spec.action_full_name.empty()) {
      spec.action_full_name =
        BuildActionFullName(spec.drone);
    }

    if (!goal_node["target"] ||
      !goal_node["target"].IsSequence() ||
      goal_node["target"].size() < 3)
    {
      throw std::runtime_error(
              "goal must contain target: [x, y, z]");
    }

    spec.x = goal_node["target"][0].as<double>();
    spec.y = goal_node["target"][1].as<double>();
    spec.z = goal_node["target"][2].as<double>();

    spec.yaw_deg =
      YamlGet<double>(goal_node, "yaw_deg", 0.0);

    return spec;
  }

  std::string BuildActionFullName(const std::string & drone_name) const
  {
    std::string drone = drone_name;

    while (!drone.empty() && drone.front() == '/') {
      drone.erase(drone.begin());
    }

    std::string action = action_name_;

    while (!action.empty() && action.front() == '/') {
      action.erase(action.begin());
    }

    return "/" + drone + "/" + action;
  }

  ActionClientTray::SharedPtr GetClient(const std::string & action_full_name)
  {
    const auto it = action_clients_.find(action_full_name);

    if (it != action_clients_.end()) {
      return it->second;
    }

    auto client =
      rclcpp_action::create_client<TrayAction>(
      this,
      action_full_name);

    action_clients_[action_full_name] = client;

    return client;
  }

  TrayAction::Goal BuildActionGoal(const GoalSpec & spec)
  {
    TrayAction::Goal goal;

    goal.tipo_trayectoria = spec.tipo_trayectoria;

    goal.target_pose = geometry_msgs::msg::PoseStamped();
    goal.target_pose.header.frame_id = spec.frame_id;
    goal.target_pose.header.stamp = this->get_clock()->now();

    goal.target_pose.pose.position.x = spec.x;
    goal.target_pose.pose.position.y = spec.y;
    goal.target_pose.pose.position.z = spec.z;

    const double yaw_rad =
      spec.yaw_deg * M_PI / 180.0;

    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = std::sin(yaw_rad / 2.0);
    goal.target_pose.pose.orientation.w = std::cos(yaw_rad / 2.0);

    goal.tx = spec.tx;
    goal.ty = spec.ty;
    goal.tz = spec.tz;
    goal.tyaw = spec.tyaw;

    goal.absoluto_x = spec.absoluto_x;
    goal.absoluto_y = spec.absoluto_y;
    goal.absoluto_z = spec.absoluto_z;
    goal.absoluto_yaw = spec.absoluto_yaw;

    return goal;
  }

  bool StartGoalBatch(
    const std::vector<GoalSpec> & goals,
    const std::string & phase,
    std::vector<ActiveGoal> & active_goals)
  {
    active_goals.clear();
    active_goals.reserve(goals.size());
    for (const auto & spec : goals) {
      auto client = GetClient(spec.action_full_name);
      if (!client->wait_for_action_server(5s)) {
        RCLCPP_ERROR(
          this->get_logger(),
          "[SCENARIO-RUNNER-GOAL-ERROR] action server not available: '%s'",
          spec.action_full_name.c_str());
        return false;
      }

      RCLCPP_WARN(
        this->get_logger(),
        "[SCENARIO-RUNNER-GOAL-SEND] action='%s' target=(%.3f, %.3f, %.3f) yaw_deg=%.3f tipo=%u phase=%s",
        spec.action_full_name.c_str(),
        spec.x,
        spec.y,
        spec.z,
        spec.yaw_deg,
        static_cast<unsigned int>(spec.tipo_trayectoria),
        phase.c_str());

      rclcpp_action::Client<TrayAction>::SendGoalOptions options;
      ActiveGoal active;
      active.spec = spec;
      active.client = client;
      active.goal_handle_future =
        client->async_send_goal(BuildActionGoal(spec), options);
      active_goals.push_back(std::move(active));
    }

    for (auto & active : active_goals) {
      if (!WaitFuture(
          active.goal_handle_future,
          10.0,
          "goal acceptance " + active.spec.action_full_name))
      {
        return false;
      }
      active.goal_handle = active.goal_handle_future.get();
      if (!active.goal_handle) {
        if (active.spec.expect_rejected) {
          active.completed = true;
          RCLCPP_INFO(
            this->get_logger(),
            "[SCENARIO-RUNNER-GOAL-REJECTED-EXPECTED] action='%s' phase=%s",
            active.spec.action_full_name.c_str(),
            phase.c_str());
          continue;
        }
        RCLCPP_ERROR(
          this->get_logger(),
          "[SCENARIO-RUNNER-GOAL-REJECTED] action='%s' phase=%s",
          active.spec.action_full_name.c_str(),
          phase.c_str());
        return false;
      }
      if (active.spec.expect_rejected) {
        RCLCPP_ERROR(
          this->get_logger(),
          "[SCENARIO-RUNNER-GOAL-ACCEPTED-UNEXPECTED] action='%s' phase=%s",
          active.spec.action_full_name.c_str(),
          phase.c_str());
        active.client->async_cancel_goal(active.goal_handle);
        return false;
      }
      active.result_future =
        active.client->async_get_result(active.goal_handle);
    }
    return true;
  }

  GoalBatchOutcome WaitGoalBatch(
    std::vector<ActiveGoal> & active_goals,
    const std::string & phase)
  {
    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
      bool all_completed = true;
      for (auto & active : active_goals) {
        if (active.completed) {
          continue;
        }
        all_completed = false;
        if (active.result_future.wait_for(0ms) !=
          std::future_status::ready)
        {
          continue;
        }

        const auto wrapped_result = active.result_future.get();
        const bool action_succeeded =
          wrapped_result.code ==
          rclcpp_action::ResultCode::SUCCEEDED;
        const bool user_success =
          wrapped_result.result &&
          wrapped_result.result->success;
        const float t_total =
          wrapped_result.result ?
          wrapped_result.result->t_total :
          -1.0F;
        RCLCPP_WARN(
          this->get_logger(),
          "[SCENARIO-RUNNER-GOAL-RESULT] action='%s' action_code=%d success=%s t_total=%.3f phase=%s",
          active.spec.action_full_name.c_str(),
          static_cast<int>(wrapped_result.code),
          user_success ? "true" : "false",
          t_total,
          phase.c_str());
        if (!action_succeeded || !user_success) {
          return GoalBatchOutcome::Failed;
        }
        active.completed = true;
      }

      if (all_completed ||
        std::all_of(
          active_goals.begin(),
          active_goals.end(),
          [](const ActiveGoal & active)
          {
            return active.completed;
          }))
      {
        return GoalBatchOutcome::Completed;
      }
      const double elapsed =
        std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
      for (const auto & active : active_goals) {
        if (!active.completed && elapsed > active.spec.timeout_sec) {
          RCLCPP_ERROR(
            this->get_logger(),
            "[SCENARIO-RUNNER-GOAL-TIMEOUT] action='%s' timeout_sec=%.3f phase=%s",
            active.spec.action_full_name.c_str(),
            active.spec.timeout_sec,
            phase.c_str());
          return GoalBatchOutcome::Failed;
        }
      }
      std::this_thread::sleep_for(100ms);
    }
    return GoalBatchOutcome::Failed;
  }

  void CancelGoalBatch(
    std::vector<ActiveGoal> & active_goals,
    const std::string & reason)
  {
    for (auto & active : active_goals) {
      if (active.completed || !active.goal_handle) {
        continue;
      }
      auto cancel_future =
        active.client->async_cancel_goal(active.goal_handle);
      const bool cancel_ready = WaitFuture(
        cancel_future,
        5.0,
        "controlled cancel " + active.spec.action_full_name);
      RCLCPP_WARN(
        this->get_logger(),
        "[SCENARIO-RUNNER-GOAL-CANCEL] action='%s' reason=%s response_ready=%s expected=true",
        active.spec.action_full_name.c_str(),
        reason.c_str(),
        cancel_ready ? "true" : "false");
    }
  }

  bool WaitForBackpressureClear()
  {
    const auto start = std::chrono::steady_clock::now();
    bool waiting_logged = false;
    while (rclcpp::ok() && mapping_backpressure_active_.load()) {
      if (!waiting_logged) {
        RCLCPP_WARN(
          this->get_logger(),
          "[SCENARIO-RUNNER-MOVE-GATE-WAIT] active=true policy=finish_current_block_next");
        waiting_logged = true;
      }
      std::this_thread::sleep_for(100ms);
    }
    if (waiting_logged && rclcpp::ok()) {
      const double waited_sec =
        std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
      RCLCPP_WARN(
        this->get_logger(),
        "[SCENARIO-RUNNER-MOVE-GATE-CLEAR] active=false waited_sec=%.3f next_move_allowed=true",
        waited_sec);
    }
    return rclcpp::ok();
  }

  bool ExecuteGoalBatchWithBackpressure(
    const std::vector<GoalSpec> & goals)
  {
    if (!WaitForBackpressureClear()) {
      return false;
    }

    std::vector<ActiveGoal> active_goals;
    if (!StartGoalBatch(goals, "mission", active_goals)) {
      return false;
    }

    const auto outcome =
      WaitGoalBatch(active_goals, "mission");
    if (outcome == GoalBatchOutcome::Completed) {
      return true;
    }

    CancelGoalBatch(active_goals, "failure");
    return false;
  }

  bool ExecuteSequentialGoals(const std::vector<GoalSpec> & goals)
  {
    for (const auto & goal : goals) {
      if (!ExecuteGoalBatchWithBackpressure({goal})) {
        return false;
      }
    }

    return true;
  }

  bool ExecuteSimultaneousGoals(const std::vector<GoalSpec> & goals)
  {
    return ExecuteGoalBatchWithBackpressure(goals);
  }

  bool ExecuteSingleGoal(const GoalSpec & spec)
  {
    auto client =
      GetClient(spec.action_full_name);

    RCLCPP_WARN(
      this->get_logger(),
      "[SCENARIO-RUNNER-GOAL-SEND] action='%s' target=(%.3f, %.3f, %.3f) yaw_deg=%.3f tipo=%u mode=sequential",
      spec.action_full_name.c_str(),
      spec.x,
      spec.y,
      spec.z,
      spec.yaw_deg,
      static_cast<unsigned int>(spec.tipo_trayectoria));

    if (!client->wait_for_action_server(5s)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[SCENARIO-RUNNER-GOAL-ERROR] action server not available: '%s'",
        spec.action_full_name.c_str());

      return false;
    }

    auto goal_msg =
      BuildActionGoal(spec);

    rclcpp_action::Client<TrayAction>::SendGoalOptions options;

    options.feedback_callback =
      [this, action_name = spec.action_full_name](
      GoalHandleTray::SharedPtr,
      const std::shared_ptr<const TrayAction::Feedback> feedback)
      {
        RCLCPP_DEBUG(
          this->get_logger(),
          "[SCENARIO-RUNNER-FEEDBACK] action='%s' t_act=%.3f",
          action_name.c_str(),
          feedback->t_act);
      };

    auto goal_handle_future =
      client->async_send_goal(goal_msg, options);

    if (!WaitFuture(
        goal_handle_future,
        10.0,
        "goal acceptance " + spec.action_full_name))
    {
      return false;
    }

    auto goal_handle =
      goal_handle_future.get();

    if (!goal_handle) {
      if (spec.expect_rejected) {
        RCLCPP_INFO(
          this->get_logger(),
          "[SCENARIO-RUNNER-GOAL-REJECTED-EXPECTED] action='%s' mode=sequential",
          spec.action_full_name.c_str());
        return true;
      }
      RCLCPP_ERROR(
        this->get_logger(),
        "[SCENARIO-RUNNER-GOAL-REJECTED] action='%s'",
        spec.action_full_name.c_str());

      return false;
    }

    if (spec.expect_rejected) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[SCENARIO-RUNNER-GOAL-ACCEPTED-UNEXPECTED] action='%s' mode=sequential",
        spec.action_full_name.c_str());
      client->async_cancel_goal(goal_handle);
      return false;
    }

    auto result_future =
      client->async_get_result(goal_handle);

    ActiveGoal active;
    active.spec = spec;
    active.client = client;
    active.goal_handle = goal_handle;
    active.result_future = result_future;

    return WaitAndCheckResult(active);
  }

  bool WaitAndCheckResult(ActiveGoal & active)
  {
    if (!WaitFuture(
        active.result_future,
        active.spec.timeout_sec,
        "goal result " + active.spec.action_full_name))
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "[SCENARIO-RUNNER-GOAL-TIMEOUT] action='%s' timeout_sec=%.3f. Requesting cancel.",
        active.spec.action_full_name.c_str(),
        active.spec.timeout_sec);

      if (active.goal_handle) {
        active.client->async_cancel_goal(active.goal_handle);
      }

      return false;
    }

    const auto wrapped_result =
      active.result_future.get();

    const bool action_succeeded =
      wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED;

    const bool user_success =
      wrapped_result.result &&
      wrapped_result.result->success;

    const float t_total =
      wrapped_result.result ?
      wrapped_result.result->t_total :
      -1.0f;

    RCLCPP_WARN(
      this->get_logger(),
      "[SCENARIO-RUNNER-GOAL-RESULT] action='%s' action_code=%d success=%s t_total=%.3f",
      active.spec.action_full_name.c_str(),
      static_cast<int>(wrapped_result.code),
      user_success ? "true" : "false",
      t_total);

    if (!action_succeeded || !user_success) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[SCENARIO-RUNNER-GOAL-FAILED] action='%s' action_succeeded=%s user_success=%s",
        active.spec.action_full_name.c_str(),
        action_succeeded ? "true" : "false",
        user_success ? "true" : "false");

      return false;
    }

    return true;
  }

  template<typename FutureT>
  bool WaitFuture(
    FutureT & future,
    double timeout_sec,
    const std::string & context)
  {
    const auto start =
      std::chrono::steady_clock::now();

    const bool use_timeout =
      timeout_sec > 0.0;

    while (rclcpp::ok()) {
      const auto status =
        future.wait_for(100ms);

      if (status == std::future_status::ready) {
        return true;
      }

      if (use_timeout) {
        const auto elapsed =
          std::chrono::duration<double>(
          std::chrono::steady_clock::now() - start)
          .count();

        if (elapsed > timeout_sec) {
          RCLCPP_ERROR(
            this->get_logger(),
            "[SCENARIO-RUNNER-TIMEOUT] context='%s' timeout_sec=%.3f",
            context.c_str(),
            timeout_sec);

          return false;
        }
      }
    }

    RCLCPP_ERROR(
      this->get_logger(),
      "[SCENARIO-RUNNER-INTERRUPTED] context='%s'",
      context.c_str());

    return false;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<ScenarioRunnerNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  std::thread spin_thread(
    [&executor]()
    {
      executor.spin();
    });

  const bool success =
    node->Run();

  executor.cancel();

  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  rclcpp::shutdown();

  return success ? EXIT_SUCCESS : EXIT_FAILURE;
}
