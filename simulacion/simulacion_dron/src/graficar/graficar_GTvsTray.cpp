#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>

#include "dron_individual/action/tray_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class GtTrayAdapter final : public rclcpp::Node
{
public:
  GtTrayAdapter()
  : Node("graficar_GTvsTray")
  {
    publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "graficas/gt_vs_tray", 10);
    gt_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "sensor/GT/pose", 10,
      std::bind(&GtTrayAdapter::on_gt_pose, this, std::placeholders::_1));
    feedback_subscription_ = create_subscription<
      dron_individual::action::TrayAction_FeedbackMessage>(
      "AccionTrayectoria/_action/feedback", 10,
      std::bind(&GtTrayAdapter::on_feedback, this, std::placeholders::_1));
    publish_timer_ = create_wall_timer(20ms, std::bind(&GtTrayAdapter::publish_sample, this));

    RCLCPP_INFO(
      get_logger(),
      "F1 GT/Tray adapter ready: input=sensor/GT/pose + AccionTrayectoria feedback, "
      "output=graficas/gt_vs_tray");
  }

private:
  static double yaw_from_pose(const geometry_msgs::msg::PoseStamped & pose)
  {
    const auto & q = pose.pose.orientation;
    return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  void on_gt_pose(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    gt_ = {
      message->pose.position.x,
      message->pose.position.y,
      message->pose.position.z,
      yaw_from_pose(*message),
    };
    has_gt_ = true;
  }

  void on_feedback(
    const dron_individual::action::TrayAction_FeedbackMessage::SharedPtr message)
  {
    const auto & feedback = message->feedback;
    if (feedback.x.data.empty() || feedback.y.data.empty() ||
      feedback.z.data.empty() || feedback.yaw.data.empty())
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "GT/Tray adapter ignored feedback without position references");
      return;
    }

    tray_ = {
      feedback.x.data.front(),
      feedback.y.data.front(),
      feedback.z.data.front(),
      feedback.yaw.data.front(),
    };
    has_tray_ = true;
  }

  void publish_sample()
  {
    if (!has_gt_ || !has_tray_) {
      return;
    }

    for (const double value : gt_) {
      if (!std::isfinite(value)) {
        return;
      }
    }
    for (const double value : tray_) {
      if (!std::isfinite(value)) {
        return;
      }
    }

    std_msgs::msg::Float64MultiArray message;
    message.data.reserve(8);
    message.data.insert(message.data.end(), tray_.begin(), tray_.end());
    message.data.insert(message.data.end(), gt_.begin(), gt_.end());
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gt_subscription_;
  rclcpp::Subscription<dron_individual::action::TrayAction_FeedbackMessage>::SharedPtr
    feedback_subscription_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::vector<double> gt_;
  std::vector<double> tray_;
  bool has_gt_{false};
  bool has_tray_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GtTrayAdapter>());
  rclcpp::shutdown();
  return 0;
}
