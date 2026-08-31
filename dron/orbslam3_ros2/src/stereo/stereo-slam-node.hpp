#ifndef __STEREO_SLAM_NODE_HPP__
#define __STEREO_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"
#include "fiducial-detector.hpp"
#include "navigation-state-estimator.hpp"

/* AÑADIDO */
#include "orbslam3_msgs/msg/orb_map.hpp"
#include "orbslam3_msgs/msg/fiducial_key_frame_observations.hpp"
#include "orbslam3_msgs/msg/navigation_state.hpp"
#include "orbslam3_msgs/msg/global_key_frame_pose.hpp"
#include "orbslam3_msgs/srv/get_orb_map.hpp"
#include "orbslam3_msgs/srv/get_fiducial_config.hpp"
#include "orbslam3_msgs/srv/get_global_key_frame_pose.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "std_msgs/msg/string.hpp"

#include "MapPoint.h"
#include "KeyFrame.h"

#include <unordered_map>
#include <fstream>
#include <unordered_set>
#include <chrono>
#include <cmath>

#include <set>
#include <tuple>
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
/* AÑADIDO */


class StereoSlamNode : public rclcpp::Node
{
    public:
        StereoSlamNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoRectify);

        ~StereoSlamNode();

    private:
        using ImageMsg = sensor_msgs::msg::Image;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_sync_policy;

        void GrabStereo(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD);

        ORB_SLAM3::System* m_SLAM;

        bool doRectify;
        cv::Mat M1l,M2l,M1r,M2r;
        cv::Mat external_rectified_camera_matrix_;
        cv::Mat external_rectified_distortion_;

        cv_bridge::CvImageConstPtr cv_ptrLeft;
        cv_bridge::CvImageConstPtr cv_ptrRight;

        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > left_sub;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > right_sub;

        std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy> > syncApproximate;


        /* AÑADIDO */
        rclcpp::Publisher<orbslam3_msgs::msg::OrbMap>::SharedPtr orb_map_delta_pub_;
        rclcpp::Publisher<
            orbslam3_msgs::msg::FiducialKeyFrameObservations>::SharedPtr
            fiducial_observations_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_local_pub_;
        rclcpp::Publisher<orbslam3_msgs::msg::NavigationState>::SharedPtr
            navigation_state_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr architecture_activity_pub_;
        bool debug_architecture_telemetry_ = false;
        std::unordered_map<std::string, std::chrono::steady_clock::time_point>
            architecture_last_emit_;
        void EmitArchitectureActivity(
            const std::string& edge_id,
            const std::string& interface_name,
            const std::string& interface_kind = "topic");

        enum class FiducialConfigState
        {
            WAIT_SERVICE,
            REQUEST_CONFIG,
            READY,
            DISABLED
        };

        struct FiducialJob
        {
            uint32_t drone_id = 0;
            uint64_t map_epoch = 0;
            ORB_SLAM3::System::KeyFrameCreationEvent event;
            cv::Mat image;
            orbslam3_ros2::FiducialCameraModel camera;
            std::string camera_optical_frame;
        };

        static constexpr const char* kFiducialConfigService =
            "/global_mapping/get_fiducial_config";
        rclcpp::Client<orbslam3_msgs::srv::GetFiducialConfig>::SharedPtr
            fiducial_config_client_;
        rclcpp::TimerBase::SharedPtr fiducial_config_timer_;
        std::atomic<FiducialConfigState> fiducial_config_state_{
            FiducialConfigState::WAIT_SERVICE};
        std::chrono::steady_clock::time_point fiducial_next_retry_;
        std::chrono::steady_clock::time_point fiducial_request_deadline_;
        uint64_t fiducial_request_generation_ = 0;
        int fiducial_queue_capacity_ = 4;
        orbslam3_ros2::FiducialDetector fiducial_detector_;
        std::deque<FiducialJob> fiducial_jobs_;
        std::mutex fiducial_jobs_mutex_;
        std::condition_variable fiducial_jobs_cv_;
        bool fiducial_worker_stop_ = false;
        std::thread fiducial_worker_thread_;

        bool debug_fiducial_visualization_ = false;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
            fiducial_debug_image_pub_;

        void ManageFiducialConfig();
        void HandleFiducialConfigResponse(
            uint64_t generation,
            rclcpp::Client<orbslam3_msgs::srv::GetFiducialConfig>::SharedFuture
                future);
        void EnqueueFiducialJob(
            const ORB_SLAM3::System::StereoTrackingReceipt& receipt,
            const std::string& camera_optical_frame);
        void FiducialWorkerLoop();
        void PublishFiducialObservations(
            const FiducialJob& job,
            const orbslam3_ros2::FiducialDetectionResult& result);
        void PublishFiducialDebugImage(
            const FiducialJob& job,
            const orbslam3_ros2::FiducialDetectionResult& result);

        rclcpp::Service<orbslam3_msgs::srv::GetOrbMap>::SharedPtr full_map_service_;

        uint32_t drone_id_;
        std::string drone_name_;
        std::string local_map_frame_;
        std::string odom_frame_;
        std::string body_frame_;
        Sophus::SE3f body_t_camera_;
        orbslam3_ros2::NavigationStateEstimator navigation_state_estimator_;
        orbslam3_ros2::OrbPosePredictor orb_pose_predictor_;
        orbslam3_ros2::CausalLinearVelocityEstimator causal_linear_estimator_;
        orbslam3_ros2::BodyTorqueDynamicPredictor body_torque_predictor_;
        orbslam3_ros2::BodyThrustDynamicPredictor body_thrust_predictor_;
        orbslam3_ros2::EpochGravityState epoch_gravity_state_;
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr
            torque_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr
            thrust_subscription_;
        rclcpp::TimerBase::SharedPtr navigation_state_timer_;
        orbslam3_msgs::msg::NavigationState latest_navigation_state_;
        bool navigation_state_ready_ = false;
        bool o_t_world_valid_ = false;
        Sophus::SE3f o_t_world_;
        uint64_t navigation_sample_sequence_ = 0;
        uint64_t orb_measurement_count_ = 0;
        uint64_t orb_prediction_count_ = 0;
        uint64_t orb_limited_measurement_count_ = 0;
        double orb_state_publish_rate_hz_ = 50.0;
        std::string navigation_prediction_mode_ = "legacy";
        bool dynamic_base_ready_ = false;
        double dynamic_base_stamp_sec_ = 0.0;
        Sophus::SE3f dynamic_base_pose_;
        Eigen::Vector3f dynamic_base_linear_velocity_ = Eigen::Vector3f::Zero();
        Eigen::Vector3f dynamic_base_angular_velocity_ = Eigen::Vector3f::Zero();
        orbslam3_ros2::CausalLinearVelocityEstimate latest_linear_estimate_;
        bool midpoint_previous_sample_valid_ = false;
        Sophus::SO3f midpoint_previous_orientation_;
        double midpoint_previous_image_stamp_sec_ = 0.0;
        Eigen::Vector3f latest_torque_body_ = Eigen::Vector3f::Zero();
        float latest_thrust_newton_ = 0.0f;
        bool debug_orb_control_state_ = false;
        bool debug_orb_visual_evidence_ = false;
        std::string orb_visual_evidence_output_dir_;
        std::ofstream orb_visual_evidence_stream_;
        uint32_t frames_since_reference_change_ = 0xFFFFFFFFU;
        bool predictor_reference_valid_ = false;
        uint64_t predictor_reference_keyframe_id_ = 0;
        double latest_orb_measurement_stamp_sec_ = 0.0;
        double latest_orb_measurement_input_stamp_sec_ = 0.0;
        double latest_orb_measurement_arrival_stamp_sec_ = 0.0;
        double orb_diagnostic_window_until_sec_ = 0.0;
        bool last_published_orb_pose_valid_ = false;
        Sophus::SE3f last_published_orb_pose_;
        int last_navigation_tracking_state_ = -999;
        rclcpp::Client<orbslam3_msgs::srv::GetGlobalKeyFramePose>::SharedPtr
            global_pose_client_;
        rclcpp::Subscription<orbslam3_msgs::msg::GlobalKeyFramePose>::SharedPtr
            global_pose_subscription_;
        bool global_pose_request_valid_ = false;
        uint64_t global_pose_request_generation_ = 0;
        uint64_t requested_global_epoch_ = 0;
        uint64_t requested_global_keyframe_id_ = 0;

        uint64_t map_sequence_ = 0;
        uint64_t frame_counter_ = 0;

        int delta_publish_period_frames_ = 10;

        std::unordered_map<uint64_t, std::size_t> sent_mappoint_hashes_;
        std::unordered_map<uint64_t, std::size_t> sent_keyframe_hashes_;

        void PublishLocalPose(const builtin_interfaces::msg::Time& stamp, const Sophus::SE3f& Tcw);
        void PublishNavigationState(
            const builtin_interfaces::msg::Time& stamp,
            const ORB_SLAM3::System::StereoTrackingReceipt& receipt,
            const Sophus::SE3f& Tcw,
            double callback_arrival_stamp_sec);
        void PublishPredictedNavigationState();
        void HandleBodyTorque(
            geometry_msgs::msg::Vector3Stamped::ConstSharedPtr message);
        void HandleBodyThrust(
            geometry_msgs::msg::Vector3Stamped::ConstSharedPtr message);
        void ResetDynamicNavigationState();
        void LogOrbMeasurementDiagnostics(
            const orbslam3_ros2::OrbPosePredictorDiagnostics& diagnostics);
        void WriteVisualEvidence(
            const builtin_interfaces::msg::Time& stamp,
            const ORB_SLAM3::System::StereoTrackingReceipt& receipt,
            double callback_arrival_stamp_sec);
        void RequestGlobalPose(uint64_t map_epoch, uint64_t keyframe_id);
        void HandleGlobalPoseResponse(
            uint64_t generation,
            uint64_t map_epoch,
            uint64_t keyframe_id,
            rclcpp::Client<orbslam3_msgs::srv::GetGlobalKeyFramePose>::SharedFuture
                future);
        void HandleGlobalPosePush(
            orbslam3_msgs::msg::GlobalKeyFramePose::ConstSharedPtr message);
        void ApplyGlobalPoseMessage(
            const orbslam3_msgs::msg::GlobalKeyFramePose& message,
            const char* source);

        void PublishOrbMapDelta();

        orbslam3_msgs::msg::OrbMap BuildOrbMap(
            bool full_snapshot,
            bool update_cache);

        void FillMapPointMsg(
            ORB_SLAM3::MapPoint* pMP,
            orbslam3_msgs::msg::OrbMapPoint& mp_msg);

        void FillKeyFrameMsg(
            ORB_SLAM3::KeyFrame* pKF,
            orbslam3_msgs::msg::OrbKeyFrame& kf_msg);

        geometry_msgs::msg::Pose SophusToPoseMsg(const Sophus::SE3f& T);
        Sophus::SE3f PoseMsgToSophus(const geometry_msgs::msg::Pose& pose);

        std::size_t HashMapPoint(ORB_SLAM3::MapPoint* pMP);
        std::size_t HashKeyFrame(ORB_SLAM3::KeyFrame* pKF);
        void HashCombine(std::size_t& seed, std::size_t value);

        void GetFullMapServiceCallback(
            const std::shared_ptr<orbslam3_msgs::srv::GetOrbMap::Request> request,
            std::shared_ptr<orbslam3_msgs::srv::GetOrbMap::Response> response);

        uint64_t map_epoch_ = 0;

        ORB_SLAM3::Map* current_orb_map_ptr_ = nullptr;
        bool has_current_orb_map_ptr_ = false;
        
        // Firma del mapa activo detectado.
        // Esto permite detectar resets aunque el puntero del mapa no cambie.
        bool has_current_map_signature_ = false;
        uint64_t current_map_min_kf_id_ = 0;
        uint64_t current_map_max_kf_id_ = 0;
        size_t current_map_kf_count_ = 0;

        float camera_fx_ = -1.0f;
        float camera_fy_ = -1.0f;
        float camera_cx_ = -1.0f;
        float camera_cy_ = -1.0f;
        float camera_bf_ = -1.0f;

        uint32_t image_width_ = 0;
        uint32_t image_height_ = 0;

        bool has_camera_info_ = false;
        
        void LoadCameraInfoFromSettings(const std::string& settings_file);

        bool UpdateMapEpochFromCurrentMap();

        ORB_SLAM3::Map* GetCurrentMapPointerFromKeyFrames();

        double CvMatAtDouble(const cv::Mat& mat, int r, int c) const;

        bool KeyFrameBelongsToMap(
            ORB_SLAM3::KeyFrame* pKF,
            ORB_SLAM3::Map* pMap) const;

        bool MapPointBelongsToMap(
            ORB_SLAM3::MapPoint* pMP,
            ORB_SLAM3::Map* pMap) const;

        void GetMapKeyFrameStats(
            ORB_SLAM3::Map* pMap,
            size_t& count_out,
            uint64_t& min_kf_id_out,
            uint64_t& max_kf_id_out) const;

        /* AÑADIDO */
};

#endif
