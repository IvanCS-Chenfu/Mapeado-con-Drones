#include <mission_msgs/msg/drone_registration.hpp>
#include <mission_msgs/srv/register_drone.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <task_manager_lib/registration_profile.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <sstream>
#include <string>

using namespace std::chrono_literals;

class TaskManagerNode final : public rclcpp::Node
{
public:
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
    registration_ = task_manager_lib::BuildRegistration(profile);
    client_ = create_client<mission_msgs::srv::RegisterDrone>("/mission/register_drone");
    if (architecture_events_enabled_) {
      architecture_publisher_ = create_publisher<std_msgs::msg::String>(
        "/system_architecture/activity", rclcpp::QoS(100).reliable());
    }
    retry_timer_ = create_wall_timer(1s, [this]() { TryRegister(); });
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
    client_->async_send_request(request,
      [this](rclcpp::Client<mission_msgs::srv::RegisterDrone>::SharedFuture future)
      {
        request_pending_ = false;
        const auto response = future.get();
        registered_ = response->accepted;
        RCLCPP_INFO(get_logger(),
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
    json << "{\"edge_id\":\"task_manager_to_task_server\",\"event\":"
         << "\"REGISTERED_DRONE_" << registration_.drone_id << "\"}";
    message.data = json.str();
    architecture_publisher_->publish(message);
  }

  mission_msgs::msg::DroneRegistration registration_;
  bool architecture_events_enabled_ = false;
  bool registered_ = false;
  bool request_pending_ = false;
  rclcpp::Client<mission_msgs::srv::RegisterDrone>::SharedPtr client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr architecture_publisher_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<TaskManagerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("task_manager"), "%s", error.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
