#pragma once

#include "multidron_gui_lib/gui_data_model.hpp"

#include "orbslam3_msgs/msg/navigation_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace multidron_gui_lib
{

/// Nodo ROS independiente de la UI. Ningún callback toca widgets Qt.
class RosDataBridge final : public rclcpp::Node
{
public:
  explicit RosDataBridge(
    std::shared_ptr<GuiDataModel> model,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void OnSparseCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);
  void OnKeyframes(visualization_msgs::msg::MarkerArray::ConstSharedPtr markers);
  void OnNavigationState(
    std::uint32_t configured_drone_id,
    orbslam3_msgs::msg::NavigationState::ConstSharedPtr state);

  void LoadFiducialsFromConfig(const std::string & path);

  std::shared_ptr<GuiDataModel> model_;
  std::string sparse_topic_;
  std::string keyframes_topic_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sparse_subscription_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr keyframe_subscription_;
  std::vector<rclcpp::Subscription<orbslam3_msgs::msg::NavigationState>::SharedPtr>
  navigation_subscriptions_;

  std::mutex keyframe_mutex_;
  std::map<std::int32_t, KeyframeVisual> keyframe_cache_;

  std::mutex drone_mutex_;
  std::map<std::uint32_t, DroneState> drone_cache_;
};

}  // namespace multidron_gui_lib
