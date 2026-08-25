#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>

class FiducialVisualizerNode : public rclcpp::Node
{
public:
    FiducialVisualizerNode()
        : Node("fiducial_visualizer")
    {
        declare_parameter<int>("drone_id", 0);
        declare_parameter<std::string>("drone_name", "drone_0");
        declare_parameter<double>("display_seconds", 5.0);

        drone_id_ = static_cast<uint32_t>(get_parameter("drone_id").as_int());
        drone_name_ = get_parameter("drone_name").as_string();
        display_seconds_ = get_parameter("display_seconds").as_double();
        if (!std::isfinite(display_seconds_) || display_seconds_ <= 0.0)
        {
            display_seconds_ = 5.0;
        }
        window_name_ = drone_name_ + " - fiducials";

        const char* display = std::getenv("DISPLAY");
        const char* wayland_display = std::getenv("WAYLAND_DISPLAY");
        visual_enabled_ =
            (display != nullptr && display[0] != '\0') ||
            (wayland_display != nullptr && wayland_display[0] != '\0');
        if (!visual_enabled_)
        {
            RCLCPP_WARN(
                get_logger(),
                "[FID-VISUALIZER-DISABLED] drone_id=%u reason=no_display",
                drone_id_);
            return;
        }

        image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
            "orbslam/fiducial_debug/image",
            rclcpp::SensorDataQoS().keep_last(1),
            std::bind(
                &FiducialVisualizerNode::HandleImage, this,
                std::placeholders::_1));
        gui_timer_ = create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&FiducialVisualizerNode::UpdateWindow, this));

        RCLCPP_INFO(
            get_logger(),
            "[FID-VISUALIZER-READY] drone_id=%u topic=%s display_seconds=%.3f",
            drone_id_, "orbslam/fiducial_debug/image", display_seconds_);
    }

    ~FiducialVisualizerNode() override
    {
        CloseWindow("shutdown");
    }

private:
    void HandleImage(const sensor_msgs::msg::Image::ConstSharedPtr message)
    {
        try
        {
            pending_image_ = cv_bridge::toCvShare(message, "bgr8")->image.clone();
            pending_frame_id_ = message->header.frame_id;
            deadline_ = std::chrono::steady_clock::now() +
                std::chrono::milliseconds(static_cast<int64_t>(
                    display_seconds_ * 1000.0));
        }
        catch (const cv::Exception& error)
        {
            Disable(error.what());
        }
    }

    void UpdateWindow()
    {
        if (!visual_enabled_)
        {
            return;
        }
        try
        {
            if (!pending_image_.empty())
            {
                if (!window_open_)
                {
                    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
                    window_open_ = true;
                    window_was_visible_ = false;
                }
                cv::imshow(window_name_, pending_image_);
                pending_image_.release();
                RCLCPP_INFO(
                    get_logger(),
                    "[FID-VISUALIZER-SHOW] drone_id=%u frame_id=%s",
                    drone_id_, pending_frame_id_.c_str());
            }
            if (!window_open_)
            {
                return;
            }
            cv::waitKey(1);
            const double visibility =
                cv::getWindowProperty(window_name_, cv::WND_PROP_VISIBLE);
            if (visibility >= 1.0)
            {
                window_was_visible_ = true;
            }
            else if (window_was_visible_)
            {
                CloseWindow("user_close");
            }
            else if (std::chrono::steady_clock::now() >= deadline_)
            {
                CloseWindow("timeout");
            }
        }
        catch (const cv::Exception& error)
        {
            Disable(error.what());
        }
    }

    void CloseWindow(const char* reason)
    {
        if (!window_open_)
        {
            return;
        }
        try
        {
            cv::destroyWindow(window_name_);
            cv::waitKey(1);
        }
        catch (const cv::Exception&)
        {
        }
        window_open_ = false;
        window_was_visible_ = false;
        RCLCPP_INFO(
            get_logger(),
            "[FID-VISUALIZER-CLOSE] drone_id=%u reason=%s",
            drone_id_, reason);
    }

    void Disable(const std::string& error)
    {
        CloseWindow("opencv_error");
        visual_enabled_ = false;
        image_subscription_.reset();
        RCLCPP_ERROR(
            get_logger(),
            "[FID-VISUALIZER-DISABLED] drone_id=%u error=%s",
            drone_id_, error.c_str());
    }

    uint32_t drone_id_ = 0;
    std::string drone_name_;
    std::string window_name_;
    double display_seconds_ = 5.0;
    bool visual_enabled_ = false;
    bool window_open_ = false;
    bool window_was_visible_ = false;
    cv::Mat pending_image_;
    std::string pending_frame_id_;
    std::chrono::steady_clock::time_point deadline_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::TimerBase::SharedPtr gui_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FiducialVisualizerNode>());
    rclcpp::shutdown();
    return 0;
}
