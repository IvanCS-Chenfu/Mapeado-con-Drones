#include "task_server/drone_registry.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <mission_msgs/msg/base_sub_roi.hpp>
#include <mission_msgs/msg/drone_registry.hpp>
#include <mission_msgs/msg/mission_geometry.hpp>
#include <mission_msgs/msg/task_state_array.hpp>
#include <mission_msgs/srv/register_drone.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <task_lib/mission_config.hpp>

#include <memory>
#include <chrono>
#include <sstream>
#include <string>

namespace
{

geometry_msgs::msg::Point ToPoint(const task_lib::Vec3 & value)
{
  geometry_msgs::msg::Point point;
  point.x = value.x;
  point.y = value.y;
  point.z = value.z;
  return point;
}

mission_msgs::msg::AxisAlignedBox ToBox(const task_lib::AxisAlignedBox & value)
{
  mission_msgs::msg::AxisAlignedBox box;
  box.min = ToPoint(value.min);
  box.max = ToPoint(value.max);
  return box;
}

}  // namespace

class TaskServerNode final : public rclcpp::Node
{
public:
  TaskServerNode()
  : Node("task_server")
  {
    const auto default_config =
      ament_index_cpp::get_package_share_directory("task_server") + "/config/mission_house.yaml";
    const auto mission_config_path = declare_parameter("mission_config", default_config);
    voxel_size_ = declare_parameter("voxel_size", 0.25);
    flow_enabled_ = declare_parameter("mission_flow_events_enabled", false);
    const auto architecture_enabled =
      declare_parameter("system_architecture_events_enabled", false);

    const auto config = task_lib::LoadMissionConfig(mission_config_path);
    geometry_data_ = task_lib::BuildMissionGeometry(config);
    mission_id_ = config.mission_id;
    mission_frame_ = config.frame_id;
    registry_ = std::make_unique<task_server::DroneRegistry>(
      config.drones, kProtocolVersion, kGeneratorId, kGeneratorVersion,
      mission_msgs::msg::DroneRegistration::CAPABILITY_SPARSE_MAPPING |
      mission_msgs::msg::DroneRegistration::CAPABILITY_STEREO_DEPTH);

    const auto snapshot_qos = rclcpp::QoS(1).reliable().transient_local();
    geometry_publisher_ = create_publisher<mission_msgs::msg::MissionGeometry>(
      "/mission/geometry", snapshot_qos);
    registry_publisher_ = create_publisher<mission_msgs::msg::DroneRegistry>(
      "/mission/registry", snapshot_qos);
    task_state_publisher_ = create_publisher<mission_msgs::msg::TaskStateArray>(
      "/mission/task_states", snapshot_qos);
    if (flow_enabled_) {
      flow_publisher_ = create_publisher<std_msgs::msg::String>(
        "/mission/flow_events", rclcpp::QoS(100).reliable());
    }
    if (architecture_enabled) {
      architecture_publisher_ = create_publisher<std_msgs::msg::String>(
        "/system_architecture/activity", rclcpp::QoS(100).reliable());
    }
    register_service_ = create_service<mission_msgs::srv::RegisterDrone>(
      "/mission/register_drone",
      [this](const std::shared_ptr<mission_msgs::srv::RegisterDrone::Request> request,
        std::shared_ptr<mission_msgs::srv::RegisterDrone::Response> response)
      {
        HandleRegistration(request, response);
      });

    PublishGeometry();
    PublishRegistry();
    PublishEmptyTaskState();
    PublishGeometryFlow();
    PublishArchitecture("task_server_to_gui_geometry", "MISSION_GEOMETRY_READY");
    if (flow_enabled_) {
      PublishArchitecture("task_server_to_sim_mission_flow", "MISSION_FLOW_EVENT");
      flow_republish_timer_ = create_wall_timer(
        std::chrono::seconds(2), [this]() {
          PublishGeometryFlow();
          flow_republish_timer_->cancel();
        });
    }
    RCLCPP_INFO(get_logger(),
      "[F6A-MISSION-CONFIG] mission=%s revision=%lu frame=%s hard_volume=derived",
      mission_id_.c_str(), geometry_data_.config_revision, mission_frame_.c_str());
    RCLCPP_INFO(get_logger(), "[F6B-GEOMETRY] levels=%zu regions=%zu unassigned=true",
      geometry_data_.levels.size(), geometry_data_.regions.size());
  }

private:
  void PublishGeometry()
  {
    mission_msgs::msg::MissionGeometry message;
    message.header.stamp = now();
    message.header.frame_id = mission_frame_;
    message.mission_id = mission_id_;
    message.config_revision = geometry_data_.config_revision;
    message.mapping_roi = ToBox(geometry_data_.mapping_roi);
    message.hard_flight_volume = ToBox(geometry_data_.hard_flight_volume);
    message.mapping_hysteresis = {geometry_data_.mapping_hysteresis.x,
      geometry_data_.mapping_hysteresis.y, geometry_data_.mapping_hysteresis.z};
    message.level_height = geometry_data_.level_height;
    for (const auto & source : geometry_data_.levels) {
      mission_msgs::msg::MappingLevel level;
      level.level_index = source.level_index;
      level.z_min = source.z_min;
      level.z_max = source.z_max;
      message.levels.push_back(level);
    }
    for (const auto & source : geometry_data_.regions) {
      mission_msgs::msg::BaseSubRoi region;
      region.region_id = source.region_id;
      region.level_index = source.level_index;
      region.side = static_cast<std::uint8_t>(source.side);
      region.bounds = ToBox(source.bounds);
      region.ownership_revision = 0U;
      message.regions.push_back(region);
    }
    geometry_publisher_->publish(message);
  }

  void PublishRegistry()
  {
    mission_msgs::msg::DroneRegistry message;
    message.header.stamp = now();
    message.header.frame_id = mission_frame_;
    message.mission_id = mission_id_;
    message.config_revision = geometry_data_.config_revision;
    message.drones = registry_->Snapshot();
    registry_publisher_->publish(message);
  }

  void PublishEmptyTaskState()
  {
    mission_msgs::msg::TaskStateArray message;
    message.header.stamp = now();
    message.header.frame_id = mission_frame_;
    message.mission_id = mission_id_;
    message.config_revision = geometry_data_.config_revision;
    task_state_publisher_->publish(message);
  }

  void HandleRegistration(
    const std::shared_ptr<mission_msgs::srv::RegisterDrone::Request> request,
    const std::shared_ptr<mission_msgs::srv::RegisterDrone::Response> response)
  {
    PublishFlow("manager_to_registration", "REGISTER_DRONE_REQUEST");
    const auto result = registry_->Register(request->registration);
    response->accepted = result.accepted;
    response->reason = result.reason;
    response->mission_id = mission_id_;
    response->mission_frame = mission_frame_;
    response->config_revision = geometry_data_.config_revision;
    response->voxel_size = voxel_size_;
    response->protocol_version = kProtocolVersion;
    if (result.changed) {
      PublishRegistry();
      PublishFlow("registration_to_task_worker", "DRONE_REGISTERED");
      PublishArchitecture("task_manager_to_task_server", "REGISTER_DRONE");
    }
    RCLCPP_INFO(get_logger(), "[F6C-REGISTRY] drone=%u accepted=%s changed=%s reason=%s",
      request->registration.drone_id, result.accepted ? "true" : "false",
      result.changed ? "true" : "false", result.reason.c_str());
  }

  void PublishFlow(const std::string & edge, const std::string & event)
  {
    if (!flow_publisher_) {
      return;
    }
    std_msgs::msg::String message;
    std::ostringstream json;
    json << "{\"edge_id\":\"" << edge << "\",\"event\":\"" << event
         << "\",\"detail\":\"" << event << "\",\"mission_id\":\""
         << mission_id_ << "\"}";
    message.data = json.str();
    flow_publisher_->publish(message);
  }

  void PublishGeometryFlow()
  {
    if (!flow_publisher_) {
      return;
    }
    std_msgs::msg::String message;
    std::ostringstream json;
    json << "{\"edge_id\":\"task_worker_geometry\","
         << "\"event\":\"MISSION_GEOMETRY_READY\",\"detail\":\""
         << geometry_data_.regions.size() << " regiones sin asignar\",\"regions\":[";
    for (std::size_t index = 0; index < geometry_data_.regions.size(); ++index) {
      const auto & region = geometry_data_.regions[index];
      if (index != 0U) {
        json << ',';
      }
      json << "{\"id\":\"" << region.region_id << "\",\"level\":"
           << region.level_index << ",\"side\":\"" << task_lib::BaseSideName(region.side)
           << "\",\"min\":[" << region.bounds.min.x << ',' << region.bounds.min.y << ','
           << region.bounds.min.z << "],\"max\":[" << region.bounds.max.x << ','
           << region.bounds.max.y << ',' << region.bounds.max.z << "]}";
    }
    json << "]}";
    message.data = json.str();
    flow_publisher_->publish(message);
  }

  void PublishArchitecture(const std::string & edge, const std::string & event)
  {
    if (!architecture_publisher_) {
      return;
    }
    std_msgs::msg::String message;
    message.data = "{\"edge_id\":\"" + edge + "\",\"event\":\"" + event + "\"}";
    architecture_publisher_->publish(message);
  }

  static constexpr std::uint32_t kProtocolVersion = 1U;
  static constexpr std::uint32_t kGeneratorVersion = 1U;
  static constexpr const char * kGeneratorId = "lib_tray";
  std::string mission_id_;
  std::string mission_frame_;
  double voxel_size_ = 0.25;
  bool flow_enabled_ = false;
  task_lib::MissionGeometry geometry_data_;
  std::unique_ptr<task_server::DroneRegistry> registry_;
  rclcpp::Publisher<mission_msgs::msg::MissionGeometry>::SharedPtr geometry_publisher_;
  rclcpp::Publisher<mission_msgs::msg::DroneRegistry>::SharedPtr registry_publisher_;
  rclcpp::Publisher<mission_msgs::msg::TaskStateArray>::SharedPtr task_state_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr flow_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr architecture_publisher_;
  rclcpp::Service<mission_msgs::srv::RegisterDrone>::SharedPtr register_service_;
  rclcpp::TimerBase::SharedPtr flow_republish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<TaskServerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("task_server"),
      "[F6A-MISSION-CONFIG-INVALID] %s", error.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
