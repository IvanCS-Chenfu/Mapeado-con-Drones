#include "stereo-slam-node.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Geometry>

#include <sstream>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <tuple>

using std::placeholders::_1;
using std::placeholders::_2;

namespace
{

builtin_interfaces::msg::Time TimestampToRosTime(double timestamp)
{
    const int64_t timestamp_ns = static_cast<int64_t>(std::llround(
        timestamp * 1e9));
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(timestamp_ns / 1000000000LL);
    stamp.nanosec = static_cast<uint32_t>(timestamp_ns % 1000000000LL);
    return stamp;
}

double RosTimeToSeconds(const builtin_interfaces::msg::Time& stamp)
{
    return static_cast<double>(stamp.sec) +
           1e-9 * static_cast<double>(stamp.nanosec);
}

geometry_msgs::msg::Quaternion RotationVectorToQuaternion(
    const cv::Vec3d& rotation_vector)
{
    cv::Mat rotation_matrix;
    cv::Rodrigues(rotation_vector, rotation_matrix);
    tf2::Matrix3x3 rotation(
        rotation_matrix.at<double>(0, 0),
        rotation_matrix.at<double>(0, 1),
        rotation_matrix.at<double>(0, 2),
        rotation_matrix.at<double>(1, 0),
        rotation_matrix.at<double>(1, 1),
        rotation_matrix.at<double>(1, 2),
        rotation_matrix.at<double>(2, 0),
        rotation_matrix.at<double>(2, 1),
        rotation_matrix.at<double>(2, 2));
    tf2::Quaternion quaternion;
    rotation.getRotation(quaternion);
    quaternion.normalize();
    geometry_msgs::msg::Quaternion message;
    message.x = quaternion.x();
    message.y = quaternion.y();
    message.z = quaternion.z();
    message.w = quaternion.w();
    return message;
}

}  // namespace


// ============================================================
// Constructor
// ============================================================

StereoSlamNode::StereoSlamNode(
    ORB_SLAM3::System* pSLAM,
    const string& strSettingsFile,
    const string& strDoRectify)
    : Node("ORB_SLAM3_ROS2"),
      m_SLAM(pSLAM)
{
    // ============================================================
    // Parámetros ROS del wrapper
    // ============================================================

    this->declare_parameter<int>("drone_id", 0);
    this->declare_parameter<std::string>("drone_name", "drone_0");
    this->declare_parameter<std::string>("local_map_frame", "orb_map");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("body_frame", "base_link");
    this->declare_parameter<int>("delta_publish_period_frames", 30);
    this->declare_parameter<bool>("debug_architecture_telemetry", false);
    this->declare_parameter<int>("fiducial_queue_capacity", 4);
    this->declare_parameter<bool>("debug_fiducial_visualization", false);
    this->declare_parameter<double>("body_T_camera_x", 0.0);
    this->declare_parameter<double>("body_T_camera_y", 0.0);
    this->declare_parameter<double>("body_T_camera_z", 0.0);
    this->declare_parameter<double>("body_T_camera_roll_deg", 0.0);
    this->declare_parameter<double>("body_T_camera_pitch_deg", 0.0);
    this->declare_parameter<double>("body_T_camera_yaw_deg", 0.0);
    this->declare_parameter<double>("orb_state_publish_rate_hz", 50.0);
    this->declare_parameter<double>("orb_state_filter.position_alpha", 0.55);
    this->declare_parameter<double>("orb_state_filter.orientation_alpha", 0.70);
    this->declare_parameter<double>("orb_state_filter.max_position_innovation_m", 0.30);
    this->declare_parameter<double>("orb_state_filter.max_rotation_innovation_rad", 0.35);
    this->declare_parameter<double>("orb_state_filter.max_linear_speed_mps", 1.5);
    this->declare_parameter<double>("orb_state_filter.max_angular_speed_radps", 1.5);
    this->declare_parameter<double>("orb_state_filter.max_linear_acceleration_mps2", 4.0);
    this->declare_parameter<double>("orb_state_filter.max_angular_acceleration_radps2", 12.0);
    this->declare_parameter<int>("orb_state_filter.max_consecutive_angular_rejections", 3);
    this->declare_parameter<double>("orb_state_filter.max_extrapolation_sec", 0.10);
    this->declare_parameter<double>("orb_state_filter.small_rotation_innovation_rad", 0.015);
    this->declare_parameter<int>("orb_state_filter.moderate_confirmation_frames", 3);
    this->declare_parameter<int>(
        "orb_state_filter.moderate_post_reference_confirmation_frames", 4);
    this->declare_parameter<int>("orb_state_filter.moderate_max_pending_frames", 6);
    this->declare_parameter<double>(
        "orb_state_filter.moderate_direction_consistency", 0.85);
    this->declare_parameter<double>("orb_state_filter.moderate_magnitude_ratio", 0.50);
    this->declare_parameter<double>("orb_state_filter.moderate_timeout_sec", 0.35);
    this->declare_parameter<int>("orb_state_filter.post_reference_switch_frames", 5);
    this->declare_parameter<bool>("debug_orb_control_state", false);
    this->declare_parameter<int>("orb_reference_gate.confirmation_frames", 3);
    this->declare_parameter<int>("orb_reference_gate.max_pending_frames", 6);
    this->declare_parameter<double>("orb_reference_gate.max_step_translation_m", 0.10);
    this->declare_parameter<double>("orb_reference_gate.max_step_rotation_rad", 0.08);

    drone_id_ =
        static_cast<uint32_t>(
            this->get_parameter("drone_id").as_int());

    drone_name_ =
        this->get_parameter("drone_name").as_string();

    local_map_frame_ =
        this->get_parameter("local_map_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    body_frame_ = this->get_parameter("body_frame").as_string();

    constexpr double kDegToRad = M_PI / 180.0;
    const float roll = static_cast<float>(
        this->get_parameter("body_T_camera_roll_deg").as_double() * kDegToRad);
    const float pitch = static_cast<float>(
        this->get_parameter("body_T_camera_pitch_deg").as_double() * kDegToRad);
    const float yaw = static_cast<float>(
        this->get_parameter("body_T_camera_yaw_deg").as_double() * kDegToRad);
    const Eigen::Matrix3f body_r_camera =
        (Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
         Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
         Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())).toRotationMatrix();
    const Eigen::Vector3f body_t_camera_translation(
        static_cast<float>(this->get_parameter("body_T_camera_x").as_double()),
        static_cast<float>(this->get_parameter("body_T_camera_y").as_double()),
        static_cast<float>(this->get_parameter("body_T_camera_z").as_double()));
    body_t_camera_ = Sophus::SE3f(body_r_camera, body_t_camera_translation);

    orb_state_publish_rate_hz_ = std::max(
        1.0, this->get_parameter("orb_state_publish_rate_hz").as_double());
    orbslam3_ros2::OrbPosePredictorConfig predictor_config;
    predictor_config.position_alpha = static_cast<float>(
        this->get_parameter("orb_state_filter.position_alpha").as_double());
    predictor_config.orientation_alpha = static_cast<float>(
        this->get_parameter("orb_state_filter.orientation_alpha").as_double());
    predictor_config.max_position_innovation_m = static_cast<float>(
        this->get_parameter("orb_state_filter.max_position_innovation_m").as_double());
    predictor_config.max_rotation_innovation_rad = static_cast<float>(
        this->get_parameter("orb_state_filter.max_rotation_innovation_rad").as_double());
    predictor_config.max_linear_speed_mps = static_cast<float>(
        this->get_parameter("orb_state_filter.max_linear_speed_mps").as_double());
    predictor_config.max_angular_speed_radps = static_cast<float>(
        this->get_parameter("orb_state_filter.max_angular_speed_radps").as_double());
    predictor_config.max_linear_acceleration_mps2 = static_cast<float>(
        this->get_parameter(
            "orb_state_filter.max_linear_acceleration_mps2").as_double());
    predictor_config.max_angular_acceleration_radps2 = static_cast<float>(
        this->get_parameter(
            "orb_state_filter.max_angular_acceleration_radps2").as_double());
    predictor_config.max_consecutive_angular_rejections = static_cast<uint32_t>(
        std::max<int64_t>(0, this->get_parameter(
            "orb_state_filter.max_consecutive_angular_rejections").as_int()));
    predictor_config.max_extrapolation_sec =
        this->get_parameter("orb_state_filter.max_extrapolation_sec").as_double();
    predictor_config.small_rotation_innovation_rad = static_cast<float>(
        this->get_parameter(
            "orb_state_filter.small_rotation_innovation_rad").as_double());
    predictor_config.moderate_confirmation_frames = static_cast<uint32_t>(
        std::max<int64_t>(1, this->get_parameter(
            "orb_state_filter.moderate_confirmation_frames").as_int()));
    predictor_config.moderate_post_reference_confirmation_frames =
        static_cast<uint32_t>(std::max<int64_t>(1, this->get_parameter(
            "orb_state_filter.moderate_post_reference_confirmation_frames").as_int()));
    predictor_config.moderate_max_pending_frames = static_cast<uint32_t>(
        std::max<int64_t>(1, this->get_parameter(
            "orb_state_filter.moderate_max_pending_frames").as_int()));
    predictor_config.moderate_direction_consistency = static_cast<float>(
        this->get_parameter(
            "orb_state_filter.moderate_direction_consistency").as_double());
    predictor_config.moderate_magnitude_ratio = static_cast<float>(
        this->get_parameter(
            "orb_state_filter.moderate_magnitude_ratio").as_double());
    predictor_config.moderate_timeout_sec =
        this->get_parameter("orb_state_filter.moderate_timeout_sec").as_double();
    predictor_config.post_reference_switch_frames = static_cast<uint32_t>(
        std::max<int64_t>(0, this->get_parameter(
            "orb_state_filter.post_reference_switch_frames").as_int()));
    orb_pose_predictor_ = orbslam3_ros2::OrbPosePredictor(predictor_config);
    orbslam3_ros2::ReferenceGateConfig reference_gate_config;
    reference_gate_config.confirmation_frames = static_cast<uint32_t>(
        std::max<int64_t>(1, this->get_parameter(
            "orb_reference_gate.confirmation_frames").as_int()));
    reference_gate_config.max_pending_frames = static_cast<uint32_t>(
        std::max<int64_t>(1, this->get_parameter(
            "orb_reference_gate.max_pending_frames").as_int()));
    reference_gate_config.max_step_translation_m = static_cast<float>(
        this->get_parameter("orb_reference_gate.max_step_translation_m").as_double());
    reference_gate_config.max_step_rotation_rad = static_cast<float>(
        this->get_parameter("orb_reference_gate.max_step_rotation_rad").as_double());
    navigation_state_estimator_ =
        orbslam3_ros2::NavigationStateEstimator(reference_gate_config);

    delta_publish_period_frames_ =
        this->get_parameter("delta_publish_period_frames").as_int();

    debug_architecture_telemetry_ =
        this->get_parameter("debug_architecture_telemetry").as_bool();
    debug_orb_control_state_ =
        this->get_parameter("debug_orb_control_state").as_bool();

    fiducial_queue_capacity_ =
        this->get_parameter("fiducial_queue_capacity").as_int();
    debug_fiducial_visualization_ =
        this->get_parameter("debug_fiducial_visualization").as_bool();

    if (delta_publish_period_frames_ <= 0)
    {
        delta_publish_period_frames_ = 30;
    }
    if (fiducial_queue_capacity_ <= 0)
    {
        fiducial_queue_capacity_ = 4;
    }
    LoadCameraInfoFromSettings(strSettingsFile);

    map_sequence_ = 0;
    frame_counter_ = 0;

    // ============================================================
    // Publicador incremental de mapa ORB
    // ============================================================

    orb_map_delta_pub_ =
        this->create_publisher<orbslam3_msgs::msg::OrbMap>(
            "orbslam/orb_map_delta",
            rclcpp::QoS(10).reliable());

    fiducial_observations_pub_ =
        this->create_publisher<
            orbslam3_msgs::msg::FiducialKeyFrameObservations>(
            "orbslam/fiducial_keyframe_observations",
            rclcpp::QoS(rclcpp::KeepLast(32)).reliable().durability_volatile());

    // ============================================================
    // Publicador de pose local actual de cámara
    // ============================================================

    pose_local_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "orbslam/pose_local",
            rclcpp::QoS(20));
    navigation_state_pub_ =
        this->create_publisher<orbslam3_msgs::msg::NavigationState>(
            "orbslam/navigation_state",
            rclcpp::QoS(rclcpp::KeepLast(20)).reliable());
    const auto navigation_state_period =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / orb_state_publish_rate_hz_));
    navigation_state_timer_ = this->create_wall_timer(
        navigation_state_period,
        std::bind(&StereoSlamNode::PublishPredictedNavigationState, this));
    global_pose_client_ =
        this->create_client<orbslam3_msgs::srv::GetGlobalKeyFramePose>(
            "/global_mapping/get_global_keyframe_pose");
    global_pose_subscription_ =
        this->create_subscription<orbslam3_msgs::msg::GlobalKeyFramePose>(
            "orbslam/global_keyframe_pose",
            rclcpp::QoS(rclcpp::KeepLast(8)).reliable().durability_volatile(),
            std::bind(
                &StereoSlamNode::HandleGlobalPosePush,
                this,
                std::placeholders::_1));

    if (debug_architecture_telemetry_)
    {
        architecture_activity_pub_ =
            this->create_publisher<std_msgs::msg::String>(
                "/system_architecture/activity",
                rclcpp::QoS(64).best_effort());
    }

    // ============================================================
    // Servicio para snapshot completo del mapa local
    // ============================================================

    full_map_service_ =
        this->create_service<orbslam3_msgs::srv::GetOrbMap>(
            "orbslam/get_full_map",
            std::bind(
                &StereoSlamNode::GetFullMapServiceCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    // ============================================================
    // Rectificación estéreo
    // ============================================================

    stringstream ss(strDoRectify);
    ss >> boolalpha >> doRectify;

    if (doRectify && m_SLAM->UsesInternalStereoRectification())
    {
        throw std::runtime_error(
            "doble rectificacion: wrapper y ORB_SLAM3 estan activos");
    }

    if (doRectify)
    {
        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);

        if (!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            assert(0);
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;

        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() ||
            P_l.empty() || P_r.empty() ||
            R_l.empty() || R_r.empty() ||
            D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 ||
            cols_l == 0 || cols_r == 0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(
            K_l,
            D_l,
            R_l,
            P_l.rowRange(0, 3).colRange(0, 3),
            cv::Size(cols_l, rows_l),
            CV_32F,
            M1l,
            M2l);

        cv::initUndistortRectifyMap(
            K_r,
            D_r,
            R_r,
            P_r.rowRange(0, 3).colRange(0, 3),
            cv::Size(cols_r, rows_r),
            CV_32F,
            M1r,
            M2r);

        P_l.rowRange(0, 3).colRange(0, 3).convertTo(
            external_rectified_camera_matrix_, CV_64F);
        external_rectified_distortion_ = cv::Mat::zeros(1, 5, CV_64F);
    }

    fiducial_config_client_ =
        this->create_client<orbslam3_msgs::srv::GetFiducialConfig>(
            kFiducialConfigService);
    fiducial_next_retry_ = std::chrono::steady_clock::now();
    fiducial_config_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&StereoSlamNode::ManageFiducialConfig, this));
    fiducial_worker_thread_ =
        std::thread(&StereoSlamNode::FiducialWorkerLoop, this);
    if (debug_fiducial_visualization_)
    {
        fiducial_debug_image_pub_ =
            this->create_publisher<sensor_msgs::msg::Image>(
                "orbslam/fiducial_debug/image",
                rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    }

    // ============================================================
    // Suscriptores estéreo sincronizados
    // ============================================================

    auto node_ptr =
        std::shared_ptr<rclcpp::Node>(
            this,
            [](rclcpp::Node*) {});

    left_sub =
        std::make_shared<message_filters::Subscriber<ImageMsg>>(
            node_ptr,
            "camera/left");

    right_sub =
        std::make_shared<message_filters::Subscriber<ImageMsg>>(
            node_ptr,
            "camera/right");

    syncApproximate =
        std::make_shared<
            message_filters::Synchronizer<approximate_sync_policy>>(
                approximate_sync_policy(10),
                *left_sub,
                *right_sub);

    syncApproximate->registerCallback(
        &StereoSlamNode::GrabStereo,
        this);

    const double baseline_est_m =
        (camera_fx_ > 0.0f)
            ? static_cast<double>(camera_bf_) / static_cast<double>(camera_fx_)
            : std::numeric_limits<double>::quiet_NaN();

    RCLCPP_WARN(
        this->get_logger(),
        "[CALIB0-WRAPPER-INIT] drone_id=%u drone_name=%s local_map_frame=%s "
        "odom_frame=%s body_frame=%s "
        "delta_period_frames=%d rectify=%s camera_valid=%s "
        "fx=%.3f fy=%.3f cx=%.3f cy=%.3f bf=%.3f width=%u height=%u baseline_est_m=%.6f",
        drone_id_,
        drone_name_.c_str(),
        local_map_frame_.c_str(),
        odom_frame_.c_str(),
        body_frame_.c_str(),
        delta_publish_period_frames_,
        doRectify ? "true" : "false",
        has_camera_info_ ? "true" : "false",
        camera_fx_,
        camera_fy_,
        camera_cx_,
        camera_cy_,
        camera_bf_,
        image_width_,
        image_height_,
        baseline_est_m);

    RCLCPP_INFO(
        this->get_logger(),
        "[FID-WRAPPER-INIT] service=%s queue_capacity=%d debug_visual=%s "
        "debug_topic=%s",
        kFiducialConfigService,
        fiducial_queue_capacity_,
        debug_fiducial_visualization_ ? "true" : "false",
        fiducial_debug_image_pub_
            ? "orbslam/fiducial_debug/image" : "disabled");
}


void StereoSlamNode::EmitArchitectureActivity(
    const std::string& edge_id,
    const std::string& interface_name,
    const std::string& interface_kind)
{
    if (!debug_architecture_telemetry_ || !architecture_activity_pub_)
    {
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
    std_msgs::msg::String message;
    std::ostringstream json;
    json << "{\"kind\":\"architecture_activity\",\"edge_id\":\""
         << edge_id << "\",\"interface\":\"" << interface_name
         << "\",\"interface_kind\":\"" << interface_kind << "\""
         << "\",\"source\":\"orbslam3\",\"drone_id\":" << drone_id_
         << ",\"namespace\":\"" << this->get_namespace()
         << "\",\"timestamp\":" << this->get_clock()->now().seconds() << "}";
    message.data = json.str();
    architecture_activity_pub_->publish(message);
}


void StereoSlamNode::ManageFiducialConfig()
{
    const auto state = fiducial_config_state_.load();
    if (state == FiducialConfigState::READY ||
        state == FiducialConfigState::DISABLED)
    {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (state == FiducialConfigState::REQUEST_CONFIG)
    {
        if (now >= fiducial_request_deadline_)
        {
            ++fiducial_request_generation_;
            fiducial_config_state_.store(FiducialConfigState::WAIT_SERVICE);
            fiducial_next_retry_ = now + std::chrono::seconds(1);
            RCLCPP_WARN(
                this->get_logger(),
                "[FID-CONFIG-TIMEOUT] drone_id=%u timeout_sec=2 retry_sec=1",
                drone_id_);
        }
        return;
    }
    if (now < fiducial_next_retry_)
    {
        return;
    }
    if (!fiducial_config_client_->service_is_ready())
    {
        fiducial_next_retry_ = now + std::chrono::seconds(1);
        RCLCPP_INFO(
            this->get_logger(),
            "[FID-CONFIG-WAIT] drone_id=%u service=%s retry_sec=1",
            drone_id_, kFiducialConfigService);
        return;
    }

    auto request =
        std::make_shared<orbslam3_msgs::srv::GetFiducialConfig::Request>();
    request->drone_id = drone_id_;
    request->drone_name = drone_name_;
    const uint64_t generation = ++fiducial_request_generation_;
    fiducial_request_deadline_ = now + std::chrono::seconds(2);
    fiducial_config_state_.store(FiducialConfigState::REQUEST_CONFIG);
    fiducial_config_client_->async_send_request(
        request,
        [this, generation](
            rclcpp::Client<orbslam3_msgs::srv::GetFiducialConfig>::SharedFuture
                future)
        {
            HandleFiducialConfigResponse(generation, future);
        });
    EmitArchitectureActivity(
        "fiducial_config_server_to_wrapper",
        kFiducialConfigService,
        "service");
    RCLCPP_INFO(
        this->get_logger(),
        "[FID-CONFIG-REQUEST] drone_id=%u generation=%lu",
        drone_id_, static_cast<unsigned long>(generation));
}


void StereoSlamNode::HandleFiducialConfigResponse(
    uint64_t generation,
    rclcpp::Client<orbslam3_msgs::srv::GetFiducialConfig>::SharedFuture future)
{
    if (generation != fiducial_request_generation_ ||
        fiducial_config_state_.load() != FiducialConfigState::REQUEST_CONFIG)
    {
        return;
    }
    try
    {
        const auto response = future.get();
        if (!response->success)
        {
            throw std::runtime_error(response->message);
        }
        orbslam3_ros2::FiducialDetectorConfig config;
        config.family = response->family;
        config.corner_refinement = response->corner_refinement;
        config.pose_solver = response->pose_solver;
        config.max_reprojection_error_px =
            response->max_reprojection_error_px;
        for (const auto& tag : response->tags)
        {
            const auto inserted = config.tag_sizes_m.emplace(
                static_cast<int>(tag.tag_id), tag.size_m);
            if (!inserted.second)
            {
                throw std::runtime_error("tag_id duplicado en respuesta");
            }
        }
        fiducial_detector_.Configure(config);
        if (config.tag_sizes_m.empty())
        {
            fiducial_config_state_.store(FiducialConfigState::DISABLED);
            RCLCPP_WARN(
                this->get_logger(),
                "[FID-CONFIG-DISABLED] drone_id=%u reason=empty_config",
                drone_id_);
            return;
        }
        fiducial_config_state_.store(FiducialConfigState::READY);
        RCLCPP_INFO(
            this->get_logger(),
            "[FID-CONFIG-READY] drone_id=%u schema=%u family=%s tags=%zu "
            "max_reprojection_error_px=%.3f",
            drone_id_, response->schema_version, response->family.c_str(),
            config.tag_sizes_m.size(), response->max_reprojection_error_px);
    }
    catch (const std::exception& error)
    {
        fiducial_config_state_.store(FiducialConfigState::WAIT_SERVICE);
        fiducial_next_retry_ =
            std::chrono::steady_clock::now() + std::chrono::seconds(1);
        RCLCPP_ERROR(
            this->get_logger(),
            "[FID-CONFIG-INVALID] drone_id=%u error=%s retry_sec=1",
            drone_id_, error.what());
    }
}


void StereoSlamNode::EnqueueFiducialJob(
    const ORB_SLAM3::System::StereoTrackingReceipt& receipt,
    const std::string& camera_optical_frame)
{
    if (fiducial_config_state_.load() != FiducialConfigState::READY)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "[FID-KF-SKIP] drone_id=%u keyframe_id=%lu reason=config_not_ready",
            drone_id_,
            static_cast<unsigned long>(receipt.keyframe_event.keyframe_id));
        return;
    }
    if (!receipt.camera.valid || receipt.image_left_effective.empty())
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "[FID-KF-SKIP] drone_id=%u keyframe_id=%lu "
            "reason=invalid_effective_camera_or_image",
            drone_id_,
            static_cast<unsigned long>(receipt.keyframe_event.keyframe_id));
        return;
    }

    FiducialJob job;
    job.drone_id = drone_id_;
    job.map_epoch = map_epoch_;
    job.event = receipt.keyframe_event;
    job.image = receipt.image_left_effective;
    job.camera_optical_frame = camera_optical_frame.empty()
        ? drone_name_ + "/camera_left_optical_frame"
        : camera_optical_frame;
    receipt.camera.camera_matrix.convertTo(job.camera.camera_matrix, CV_64F);
    receipt.camera.distortion_coefficients.convertTo(
        job.camera.distortion, CV_64F);
    job.camera.width = static_cast<int>(receipt.camera.image_width);
    job.camera.height = static_cast<int>(receipt.camera.image_height);
    job.camera.rectified = receipt.camera.is_rectified;
    if (doRectify)
    {
        external_rectified_camera_matrix_.copyTo(job.camera.camera_matrix);
        external_rectified_distortion_.copyTo(job.camera.distortion);
        job.camera.width = job.image.cols;
        job.camera.height = job.image.rows;
        job.camera.rectified = true;
    }

    std::lock_guard<std::mutex> lock(fiducial_jobs_mutex_);
    if (fiducial_jobs_.size() >=
        static_cast<std::size_t>(fiducial_queue_capacity_))
    {
        const auto dropped_id = fiducial_jobs_.front().event.keyframe_id;
        fiducial_jobs_.pop_front();
        RCLCPP_WARN(
            this->get_logger(),
            "[FID-QUEUE-DROP-OLDEST] drone_id=%u dropped_keyframe_id=%lu "
            "capacity=%d",
            drone_id_, static_cast<unsigned long>(dropped_id),
            fiducial_queue_capacity_);
    }
    fiducial_jobs_.push_back(std::move(job));
    fiducial_jobs_cv_.notify_one();
}


void StereoSlamNode::FiducialWorkerLoop()
{
    while (true)
    {
        FiducialJob job;
        {
            std::unique_lock<std::mutex> lock(fiducial_jobs_mutex_);
            fiducial_jobs_cv_.wait(lock, [this]() {
                return fiducial_worker_stop_ || !fiducial_jobs_.empty();
            });
            if (fiducial_worker_stop_)
            {
                return;
            }
            job = std::move(fiducial_jobs_.front());
            fiducial_jobs_.pop_front();
        }
        try
        {
            const auto result = fiducial_detector_.Detect(job.image, job.camera);
            std::size_t valid_count = 0;
            for (const auto& detection : result.decoded_tags)
            {
                valid_count += detection.valid ? 1U : 0U;
                RCLCPP_INFO(
                    this->get_logger(),
                    "[FID-TAG] drone_id=%u epoch=%lu keyframe_id=%lu tag_id=%d "
                    "valid=%s reason=%s reprojection_error_px=%.6f "
                    "quality=%.6f z_m=%.6f area_px2=%.3f ambiguity_px=%.6f",
                    job.drone_id, static_cast<unsigned long>(job.map_epoch),
                    static_cast<unsigned long>(job.event.keyframe_id),
                    detection.tag_id, detection.valid ? "true" : "false",
                    detection.valid ? "accepted" :
                        detection.rejection_reason.c_str(),
                    detection.reprojection_error_px, detection.quality,
                    detection.translation_vector[2],
                    detection.marker_area_px2, detection.pose_ambiguity_px);
            }
            RCLCPP_INFO(
                this->get_logger(),
                "[FID-KF-DONE] drone_id=%u epoch=%lu keyframe_id=%lu "
                "decoded=%zu valid=%zu undecoded=%zu detect_ms=%.3f "
                "pose_ms=%.3f total_ms=%.3f",
                job.drone_id, static_cast<unsigned long>(job.map_epoch),
                static_cast<unsigned long>(job.event.keyframe_id),
                result.decoded_tags.size(), valid_count,
                result.undecoded_candidates, result.detection_ms,
                result.pose_ms, result.total_ms);
            PublishFiducialObservations(job, result);
            if (fiducial_debug_image_pub_ &&
                !result.decoded_tags.empty())
            {
                PublishFiducialDebugImage(job, result);
            }
        }
        catch (const std::exception& error)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "[FID-WORKER-ERROR] drone_id=%u keyframe_id=%lu error=%s",
                job.drone_id,
                static_cast<unsigned long>(job.event.keyframe_id),
                error.what());
        }
    }
}


void StereoSlamNode::PublishFiducialObservations(
    const FiducialJob& job,
    const orbslam3_ros2::FiducialDetectionResult& result)
{
    std::vector<const orbslam3_ros2::FiducialDetection*> valid;
    valid.reserve(result.decoded_tags.size());
    for (const auto& detection : result.decoded_tags)
    {
        if (detection.valid)
        {
            valid.push_back(&detection);
        }
    }
    if (valid.empty())
    {
        return;
    }
    std::sort(valid.begin(), valid.end(), [](const auto* left, const auto* right) {
        return left->tag_id < right->tag_id;
    });

    orbslam3_msgs::msg::FiducialKeyFrameObservations message;
    message.header.stamp = TimestampToRosTime(job.event.timestamp);
    message.header.frame_id = job.camera_optical_frame;
    message.drone_id = job.drone_id;
    message.drone_name = drone_name_;
    message.map_epoch = job.map_epoch;
    message.local_keyframe_id = job.event.keyframe_id;
    message.source_frame_id = job.event.source_frame_id;
    message.observations.reserve(valid.size());
    for (const auto* detection : valid)
    {
        orbslam3_msgs::msg::FiducialTagObservation observation;
        observation.tag_id = static_cast<uint32_t>(detection->tag_id);
        observation.camera_t_tag.translation.x = detection->translation_vector[0];
        observation.camera_t_tag.translation.y = detection->translation_vector[1];
        observation.camera_t_tag.translation.z = detection->translation_vector[2];
        observation.camera_t_tag.rotation =
            RotationVectorToQuaternion(detection->rotation_vector);
        observation.quality_score = detection->quality;
        observation.reprojection_error_px = detection->reprojection_error_px;
        observation.tag_area_px2 = detection->marker_area_px2;
        observation.pose_ambiguity = detection->pose_ambiguity_px;
        message.observations.push_back(std::move(observation));
    }
    fiducial_observations_pub_->publish(message);
    EmitArchitectureActivity(
        "wrapper_to_server_fiducial_observations",
        "orbslam/fiducial_keyframe_observations",
        "topic");
    RCLCPP_INFO(
        this->get_logger(),
        "[FID-BATCH-PUB] drone=%u epoch=%lu kf=%lu tags=%zu",
        job.drone_id, static_cast<unsigned long>(job.map_epoch),
        static_cast<unsigned long>(job.event.keyframe_id), valid.size());
}


void StereoSlamNode::PublishFiducialDebugImage(
    const FiducialJob& job,
    const orbslam3_ros2::FiducialDetectionResult& result)
{
    cv::Mat annotated;
    if (job.image.channels() == 1)
    {
        cv::cvtColor(job.image, annotated, cv::COLOR_GRAY2BGR);
    }
    else
    {
        annotated = job.image.clone();
    }
    for (const auto& detection : result.decoded_tags)
    {
        const cv::Scalar color = detection.valid
            ? cv::Scalar(0, 220, 0)
            : cv::Scalar(0, 0, 255);
        std::vector<std::vector<cv::Point>> contours(1);
        for (const auto& corner : detection.corners)
        {
            contours[0].emplace_back(cvRound(corner.x), cvRound(corner.y));
        }
        cv::polylines(annotated, contours, true, color, 2, cv::LINE_AA);
        const std::string label = detection.valid
            ? "tag_id=" + std::to_string(detection.tag_id) + " accepted"
            : "tag_id=" + std::to_string(detection.tag_id) + " " +
                detection.rejection_reason;
        const cv::Point origin = contours[0].empty()
            ? cv::Point(8, 24)
            : contours[0].front() + cv::Point(0, -8);
        cv::putText(
            annotated, label, origin, cv::FONT_HERSHEY_SIMPLEX,
            0.55, color, 2, cv::LINE_AA);
    }
    std_msgs::msg::Header header;
    header.stamp = TimestampToRosTime(job.event.timestamp);
    header.frame_id = drone_name_ + "/epoch_" +
        std::to_string(job.map_epoch) + "/kf_" +
        std::to_string(job.event.keyframe_id);
    auto message = cv_bridge::CvImage(header, "bgr8", annotated).toImageMsg();
    fiducial_debug_image_pub_->publish(*message);
    RCLCPP_INFO(
        this->get_logger(),
        "[FID-VISUAL-PUB] drone_id=%u epoch=%lu keyframe_id=%lu decoded=%zu",
        job.drone_id, static_cast<unsigned long>(job.map_epoch),
        static_cast<unsigned long>(job.event.keyframe_id),
        result.decoded_tags.size());
}


// ============================================================
// Destructor
// ============================================================

StereoSlamNode::~StereoSlamNode()
{
    fiducial_config_timer_.reset();
    {
        std::lock_guard<std::mutex> lock(fiducial_jobs_mutex_);
        fiducial_worker_stop_ = true;
        fiducial_jobs_.clear();
    }
    fiducial_jobs_cv_.notify_all();
    if (fiducial_worker_thread_.joinable())
    {
        fiducial_worker_thread_.join();
    }
    m_SLAM->Shutdown();
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}


// ============================================================
// Callback estéreo principal
// ============================================================

void StereoSlamNode::GrabStereo(
    const ImageMsg::SharedPtr msgLeft,
    const ImageMsg::SharedPtr msgRight)
{
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "cv_bridge exception: %s",
            e.what());

        return;
    }

    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "cv_bridge exception: %s",
            e.what());

        return;
    }

    if (debug_architecture_telemetry_)
    {
        EmitArchitectureActivity(
            "sim_to_orbslam_stereo",
            "camera/left + camera/right");
    }

    cv::Mat imLeftForTracking;
    cv::Mat imRightForTracking;

    if (doRectify)
    {
        cv::remap(
            cv_ptrLeft->image,
            imLeftForTracking,
            M1l,
            M2l,
            cv::INTER_LINEAR);

        cv::remap(
            cv_ptrRight->image,
            imRightForTracking,
            M1r,
            M2r,
            cv::INTER_LINEAR);
    }
    else
    {
        imLeftForTracking = cv_ptrLeft->image;
        imRightForTracking = cv_ptrRight->image;
    }

    const double input_timestamp =
        Utility::StampToSec(msgLeft->header.stamp);

    ORB_SLAM3::System::StereoTrackingReceipt tracking_receipt;
    Sophus::SE3f Tcw =
        m_SLAM->TrackStereo(
            imLeftForTracking,
            imRightForTracking,
            input_timestamp,
            {},
            "",
            &tracking_receipt);

    // Detectar cambio de mapa una sola vez. Antes se llamaba aquí y de
    // nuevo más abajo, de forma que la primera llamada podía consumir el
    // cambio y hacer que epoch_changed fuese false en la segunda.
    const bool epoch_changed =
        UpdateMapEpochFromCurrentMap();

    if (tracking_receipt.keyframe_event.created)
    {
        const double timestamp_delta_sec =
            std::abs(
                tracking_receipt.keyframe_event.timestamp - input_timestamp);

        RCLCPP_WARN(
            this->get_logger(),
            "[KF-EVENT-CREATED] drone_id=%u epoch=%lu keyframe_id=%lu "
            "source_frame_id=%lu event_timestamp=%.9f input_timestamp=%.9f "
            "timestamp_delta_sec=%.9f image_width=%d image_height=%d",
            drone_id_,
            static_cast<unsigned long>(map_epoch_),
            static_cast<unsigned long>(
                tracking_receipt.keyframe_event.keyframe_id),
            static_cast<unsigned long>(
                tracking_receipt.keyframe_event.source_frame_id),
            tracking_receipt.keyframe_event.timestamp,
            input_timestamp,
            timestamp_delta_sec,
            tracking_receipt.image_left_effective.cols,
            tracking_receipt.image_left_effective.rows);

        const bool geometry_matches =
            tracking_receipt.camera.valid &&
            tracking_receipt.image_left_effective.cols ==
                static_cast<int>(tracking_receipt.camera.image_width) &&
            tracking_receipt.image_left_effective.rows ==
                static_cast<int>(tracking_receipt.camera.image_height);
        if (timestamp_delta_sec > 1.0e-6 || !geometry_matches)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "[KF-EVENT-TIMESTAMP-MISMATCH] drone_id=%u keyframe_id=%lu "
                "source_frame_id=%lu delta_sec=%.9f geometry_matches=%s",
                drone_id_,
                static_cast<unsigned long>(
                    tracking_receipt.keyframe_event.keyframe_id),
                static_cast<unsigned long>(
                    tracking_receipt.keyframe_event.source_frame_id),
                timestamp_delta_sec,
                geometry_matches ? "true" : "false");
        }
        EnqueueFiducialJob(tracking_receipt, msgLeft->header.frame_id);
    }
    else
    {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "[KF-EVENT-NONE] drone_id=%u epoch=%lu frame_counter=%lu "
            "input_timestamp=%.6f",
            drone_id_,
            static_cast<unsigned long>(map_epoch_),
            static_cast<unsigned long>(frame_counter_),
            input_timestamp);
    }

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "[PIPE0-WRAPPER-TRACK] drone_id=%u epoch=%lu frame_counter=%lu "
        "tracking_state=%d stamp=%.6f",
        drone_id_,
        map_epoch_,
        frame_counter_,
        tracking_receipt.tracking_state,
        input_timestamp);

    PublishNavigationState(msgLeft->header.stamp, tracking_receipt, Tcw);

    // Publicación legacy de cámara; el estado de navegación publica O_T_B.
    if (tracking_receipt.tracking_state == ORB_SLAM3::Tracking::OK)
    {
        PublishLocalPose(msgLeft->header.stamp, Tcw);
    }

    // Publicación incremental del mapa cada N frames.
    frame_counter_++;

    if (epoch_changed)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "[PIPE0-WRAPPER-EPOCH-PUBLISH] drone_id=%u new_epoch=%lu "
            "frame_counter=%lu action=force_immediate_delta_publish",
            drone_id_,
            map_epoch_,
            frame_counter_);

        RCLCPP_WARN(
            this->get_logger(),
            "[WRAPPER-EPOCH] forcing immediate OrbMap publish after map_epoch change. drone_id=%u epoch=%lu",
            drone_id_,
            map_epoch_);

        PublishOrbMapDelta();
        return;
    }

    if (frame_counter_ % static_cast<uint64_t>(delta_publish_period_frames_) == 0)
    {
        PublishOrbMapDelta();
    }
}


// ============================================================
// Publicar pose local Twc en el frame local ORB
// ============================================================

void StereoSlamNode::PublishLocalPose(
    const builtin_interfaces::msg::Time& stamp,
    const Sophus::SE3f& Tcw)
{
    Sophus::SE3f Twc = Tcw.inverse();

    Eigen::Matrix3f R = Twc.rotationMatrix();
    Eigen::Vector3f t = Twc.translation();

    Eigen::Quaternionf q(R);
    q.normalize();

    geometry_msgs::msg::PoseStamped msg;

    msg.header.stamp = stamp;
    msg.header.frame_id = local_map_frame_;

    msg.pose.position.x = t.x();
    msg.pose.position.y = t.y();
    msg.pose.position.z = t.z();

    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();

    pose_local_pub_->publish(msg);
}

void StereoSlamNode::PublishNavigationState(
    const builtin_interfaces::msg::Time& stamp,
    const ORB_SLAM3::System::StereoTrackingReceipt& receipt,
    const Sophus::SE3f& Tcw)
{
    const bool tracking_valid =
        receipt.tracking_state == ORB_SLAM3::Tracking::OK;
    const Sophus::SE3f local_t_camera = Tcw.inverse();
    const orbslam3_ros2::ContinuousPoseResult pose_result =
        navigation_state_estimator_.Update(
            map_epoch_,
            tracking_valid,
            receipt.reference_keyframe_valid,
            receipt.reference_keyframe_id,
            receipt.tcr_valid,
            receipt.Tcr,
            local_t_camera);
    if (pose_result.local_valid && pose_result.active_reference_valid &&
        (!global_pose_request_valid_ || requested_global_epoch_ != map_epoch_ ||
        requested_global_keyframe_id_ != pose_result.active_reference_keyframe_id))
    {
        RequestGlobalPose(map_epoch_, pose_result.active_reference_keyframe_id);
    }

    orbslam3_msgs::msg::NavigationState message;
    message.header.stamp = stamp;
    message.header.frame_id = odom_frame_;
    message.child_frame_id = body_frame_;
    message.drone_id = drone_id_;
    message.sample_sequence = navigation_sample_sequence_;
    message.map_epoch = map_epoch_;
    message.tracking_state = static_cast<int8_t>(receipt.tracking_state);
    message.pose_source = pose_result.local_valid
        ? orbslam3_msgs::msg::NavigationState::POSE_SOURCE_ORB
        : orbslam3_msgs::msg::NavigationState::POSE_SOURCE_INVALID;
    message.global_status = static_cast<uint8_t>(pose_result.global_state);
    message.pose_revision = pose_result.pose_revision;
    message.local_valid = pose_result.local_valid;
    message.local_continuity_valid = pose_result.continuity_valid;
    message.global_valid =
        pose_result.local_valid &&
        pose_result.global_state == orbslam3_ros2::GlobalPoseState::Authoritative;
    message.velocity_valid = false;
    message.reference_keyframe_valid = pose_result.active_reference_valid;
    message.reference_keyframe_id = pose_result.active_reference_keyframe_id;
    message.tcr.orientation.w = 1.0;
    message.o_t_body.orientation.w = 1.0;
    message.w_t_body.orientation.w = 1.0;
    if (pose_result.reference_pending)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 500,
            "[F5H-REFERENCE-GATE] event=%s reported_ref=%lu active_ref=%lu "
            "step_translation_m=%.6f step_rotation_rad=%.6f",
            pose_result.reference_rejected ? "rejected" : "pending",
            static_cast<unsigned long>(receipt.reference_keyframe_id),
            static_cast<unsigned long>(pose_result.active_reference_keyframe_id),
            pose_result.step_translation_m,
            pose_result.step_rotation_rad);
    }
    if (pose_result.reference_gate_timed_out)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "[F5H-REFERENCE-GATE] event=timeout reported_ref=%lu",
            static_cast<unsigned long>(receipt.reference_keyframe_id));
    }
    if (pose_result.active_reference_valid)
    {
        message.tcr = SophusToPoseMsg(pose_result.active_tcr);
    }
    if (pose_result.local_valid)
    {
        orbslam3_ros2::PredictedOrbPoseState predicted;
        if (pose_result.measurement_accepted)
        {
            const Sophus::SE3f raw_o_t_body =
                pose_result.o_t_camera * body_t_camera_.inverse();
            if (pose_result.reference_changed)
            {
                frames_since_reference_change_ = 0;
            }
            const orbslam3_ros2::OrbMeasurementContext measurement_context{
                map_epoch_,
                receipt.tracking_state,
                pose_result.active_reference_keyframe_id,
                pose_result.reference_changed,
                frames_since_reference_change_};
            predicted = orb_pose_predictor_.UpdateMeasurement(
                raw_o_t_body, RosTimeToSeconds(stamp), measurement_context);
            latest_orb_measurement_input_stamp_sec_ = RosTimeToSeconds(stamp);
            latest_orb_measurement_stamp_sec_ = this->get_clock()->now().seconds();
            ++orb_measurement_count_;
            LogOrbMeasurementDiagnostics(orb_pose_predictor_.last_diagnostics());
            if (frames_since_reference_change_ != 0xFFFFFFFFU)
            {
                ++frames_since_reference_change_;
            }
            if (orb_pose_predictor_.last_update_limited())
            {
                ++orb_limited_measurement_count_;
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "[F5H-ORB-STATE-FILTER] limited=true "
                    "orientation_rejected=%s consecutive_rejections=%u "
                    "position_innovation_m=%.6f rotation_innovation_rad=%.6f "
                    "rotation_step_rad=%.6f",
                    orb_pose_predictor_.last_orientation_rejected() ? "true" : "false",
                    orb_pose_predictor_.consecutive_angular_rejections(),
                    orb_pose_predictor_.last_position_innovation_m(),
                    orb_pose_predictor_.last_rotation_innovation_rad(),
                    orb_pose_predictor_.last_rotation_step_rad());
            }
        }
        else
        {
            predicted = orb_pose_predictor_.Predict(RosTimeToSeconds(stamp));
        }
        if (!predicted.valid || !orb_pose_predictor_.healthy())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "[F5H-ORB-STATE-REJECTED] consecutive_angular_rejections=%u "
                "reference_pending=%s gate_timeout=%s",
                orb_pose_predictor_.consecutive_angular_rejections(),
                pose_result.reference_pending ? "true" : "false",
                pose_result.reference_gate_timed_out ? "true" : "false");
            message.local_valid = false;
            message.local_continuity_valid = false;
            message.global_valid = false;
            message.velocity_valid = false;
            message.pose_source =
                orbslam3_msgs::msg::NavigationState::POSE_SOURCE_INVALID;
            orb_pose_predictor_.Reset();
            o_t_world_valid_ = false;
            frames_since_reference_change_ = 0xFFFFFFFFU;
            last_published_orb_pose_valid_ = false;
        }
        else
        {
            message.o_t_body = SophusToPoseMsg(predicted.pose);
            message.velocity_valid = predicted.velocity_valid;
            if (predicted.velocity_valid)
            {
                message.velocity.linear.x = predicted.linear_velocity.x();
                message.velocity.linear.y = predicted.linear_velocity.y();
                message.velocity.linear.z = predicted.linear_velocity.z();
                message.velocity.angular.x = predicted.angular_velocity.x();
                message.velocity.angular.y = predicted.angular_velocity.y();
                message.velocity.angular.z = predicted.angular_velocity.z();
            }
            if (
                pose_result.measurement_accepted &&
                pose_result.global_state != orbslam3_ros2::GlobalPoseState::Invalid)
            {
                const Sophus::SE3f raw_w_t_body =
                    pose_result.w_t_camera * body_t_camera_.inverse();
                o_t_world_ = predicted.pose * raw_w_t_body.inverse();
                o_t_world_valid_ = true;
            }
            else if (
                pose_result.global_state == orbslam3_ros2::GlobalPoseState::Invalid)
            {
                o_t_world_valid_ = false;
            }
            if (
                o_t_world_valid_ &&
                pose_result.global_state != orbslam3_ros2::GlobalPoseState::Invalid)
            {
                message.w_t_body = SophusToPoseMsg(
                    o_t_world_.inverse() * predicted.pose);
            }
        }
    }
    else
    {
        orb_pose_predictor_.Reset();
        o_t_world_valid_ = false;
        frames_since_reference_change_ = 0xFFFFFFFFU;
        last_published_orb_pose_valid_ = false;
    }
    if (message.global_valid)
    {
        const geometry_msgs::msg::Pose o_t_camera =
            SophusToPoseMsg(pose_result.o_t_camera);
        const geometry_msgs::msg::Pose w_t_camera =
            SophusToPoseMsg(pose_result.w_t_camera);
        const geometry_msgs::msg::Pose body_t_camera =
            SophusToPoseMsg(body_t_camera_);
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "[F5H-WRAPPER-FRAME-DIAG] part=inputs drone=%u epoch=%lu "
            "ref_kf=%lu raw_sample=%lu "
            "o_c_p=(%.6f,%.6f,%.6f) o_c_q=(%.6f,%.6f,%.6f,%.6f) "
            "w_c_p=(%.6f,%.6f,%.6f) w_c_q=(%.6f,%.6f,%.6f,%.6f) "
            "b_c_p=(%.6f,%.6f,%.6f) b_c_q=(%.6f,%.6f,%.6f,%.6f)",
            drone_id_, static_cast<unsigned long>(map_epoch_),
            static_cast<unsigned long>(receipt.reference_keyframe_id),
            static_cast<unsigned long>(message.sample_sequence),
            o_t_camera.position.x, o_t_camera.position.y, o_t_camera.position.z,
            o_t_camera.orientation.x, o_t_camera.orientation.y,
            o_t_camera.orientation.z, o_t_camera.orientation.w,
            w_t_camera.position.x, w_t_camera.position.y, w_t_camera.position.z,
            w_t_camera.orientation.x, w_t_camera.orientation.y,
            w_t_camera.orientation.z, w_t_camera.orientation.w,
            body_t_camera.position.x, body_t_camera.position.y,
            body_t_camera.position.z, body_t_camera.orientation.x,
            body_t_camera.orientation.y, body_t_camera.orientation.z,
            body_t_camera.orientation.w);
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "[F5H-WRAPPER-FRAME-DIAG] part=outputs drone=%u epoch=%lu "
            "ref_kf=%lu raw_sample=%lu "
            "o_b_p=(%.6f,%.6f,%.6f) o_b_q=(%.6f,%.6f,%.6f,%.6f) "
            "w_b_p=(%.6f,%.6f,%.6f) w_b_q=(%.6f,%.6f,%.6f,%.6f) "
            "tcr_p=(%.6f,%.6f,%.6f) tcr_q=(%.6f,%.6f,%.6f,%.6f)",
            drone_id_, static_cast<unsigned long>(map_epoch_),
            static_cast<unsigned long>(receipt.reference_keyframe_id),
            static_cast<unsigned long>(message.sample_sequence),
            message.o_t_body.position.x, message.o_t_body.position.y,
            message.o_t_body.position.z, message.o_t_body.orientation.x,
            message.o_t_body.orientation.y, message.o_t_body.orientation.z,
            message.o_t_body.orientation.w, message.w_t_body.position.x,
            message.w_t_body.position.y, message.w_t_body.position.z,
            message.w_t_body.orientation.x, message.w_t_body.orientation.y,
            message.w_t_body.orientation.z, message.w_t_body.orientation.w,
            message.tcr.position.x, message.tcr.position.y,
            message.tcr.position.z, message.tcr.orientation.x,
            message.tcr.orientation.y, message.tcr.orientation.z,
            message.tcr.orientation.w);
    }
    latest_navigation_state_ = message;
    navigation_state_ready_ = true;

    if (receipt.tracking_state != last_navigation_tracking_state_)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "[F5B-TRACKING] drone_id=%u epoch=%lu previous=%d current=%d "
            "local_valid=%s continuity_valid=%s ref_valid=%s ref_kf=%lu",
            drone_id_,
            static_cast<unsigned long>(map_epoch_),
            last_navigation_tracking_state_,
            receipt.tracking_state,
            pose_result.local_valid ? "true" : "false",
            pose_result.continuity_valid ? "true" : "false",
            receipt.reference_keyframe_valid ? "true" : "false",
            static_cast<unsigned long>(receipt.reference_keyframe_id));
        last_navigation_tracking_state_ = receipt.tracking_state;
    }
    if (pose_result.reference_changed)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "[F5H-REFERENCE-GATE] event=accepted active_ref=%lu",
            static_cast<unsigned long>(pose_result.active_reference_keyframe_id));
        RCLCPP_INFO(
            this->get_logger(),
            "[F5B-REFERENCE-KF] drone_id=%u epoch=%lu ref_kf=%lu "
            "step_translation_m=%.6f step_rotation_rad=%.6f continuity_valid=true",
            drone_id_,
            static_cast<unsigned long>(map_epoch_),
            static_cast<unsigned long>(receipt.reference_keyframe_id),
            pose_result.step_translation_m,
            pose_result.step_rotation_rad);
    }
    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "[F5B-O-CONTINUITY] drone_id=%u epoch=%lu tracking=%d ref_kf=%lu "
        "local_valid=%s continuity_valid=%s step_translation_m=%.6f "
        "step_rotation_rad=%.6f",
        drone_id_,
        static_cast<unsigned long>(map_epoch_),
        receipt.tracking_state,
        static_cast<unsigned long>(receipt.reference_keyframe_id),
        pose_result.local_valid ? "true" : "false",
        pose_result.continuity_valid ? "true" : "false",
        pose_result.step_translation_m,
        pose_result.step_rotation_rad);
    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "[F5E-POSE-STATE] drone_id=%u epoch=%lu ref_kf=%lu local_valid=%s "
        "global_status=%u global_valid=%s pose_revision=%lu pose_source=%u",
        drone_id_, static_cast<unsigned long>(map_epoch_),
        static_cast<unsigned long>(receipt.reference_keyframe_id),
        message.local_valid ? "true" : "false", message.global_status,
        message.global_valid ? "true" : "false",
        static_cast<unsigned long>(message.pose_revision), message.pose_source);
}

void StereoSlamNode::LogOrbMeasurementDiagnostics(
    const orbslam3_ros2::OrbPosePredictorDiagnostics& diagnostics)
{
    if (!debug_orb_control_state_ || !diagnostics.measurement_processed)
    {
        return;
    }

    const bool relevant =
        diagnostics.classification !=
            orbslam3_ros2::AngularCorrectionClass::Small &&
        diagnostics.classification !=
            orbslam3_ros2::AngularCorrectionClass::Initializing;
    if (!relevant)
    {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 200,
            "[F5H-ORB-MEASUREMENT] classification=%s stamp=%.6f dt=%.6f "
            "drone_id=%u epoch=%lu tracking=%d ref_kf=%lu "
            "raw_step_rotation_rad=%.6f rotation_innovation_rad=%.6f "
            "published_pose_rotation_step_rad=%.6f "
            "omega=(%.6f,%.6f,%.6f)",
            orbslam3_ros2::AngularCorrectionClassName(diagnostics.classification),
            diagnostics.measurement_stamp_sec, diagnostics.dt_sec, drone_id_,
            static_cast<unsigned long>(diagnostics.context.map_epoch),
            diagnostics.context.tracking_state,
            static_cast<unsigned long>(diagnostics.context.reference_keyframe_id),
            diagnostics.raw_step_rotation_rad,
            diagnostics.rotation_innovation_rad,
            diagnostics.published_pose_rotation_step_rad,
            diagnostics.angular_velocity_after_limits.x(),
            diagnostics.angular_velocity_after_limits.y(),
            diagnostics.angular_velocity_after_limits.z());
        return;
    }

    orb_diagnostic_window_until_sec_ = std::max(
        orb_diagnostic_window_until_sec_, this->get_clock()->now().seconds() + 2.0);
    RCLCPP_WARN(
        this->get_logger(),
        "[F5H-ORB-MEASUREMENT] classification=%s stamp=%.6f dt=%.6f "
        "drone_id=%u epoch=%lu tracking=%d ref_kf=%lu reference_changed=%s "
        "frames_since_reference_change=%u post_reference_switch=%s "
        "raw_step_translation_m=%.6f raw_step_rotation_rad=%.6f "
        "position_innovation_m=%.6f "
        "rotation_innovation_vector=(%.6f,%.6f,%.6f) "
        "rotation_innovation_rad=%.6f small_limit_rad=%.6f "
        "previous_v=(%.6f,%.6f,%.6f) previous_omega=(%.6f,%.6f,%.6f) "
        "implied_omega=(%.6f,%.6f,%.6f) implied_alpha=(%.6f,%.6f,%.6f) "
        "pending_id=%lu pending_good=%u pending_total=%u "
        "consistency_cosine=%.6f magnitude_ratio=%.6f "
        "correction_fraction=%.6f applied_rotation_correction_rad=%.6f "
        "published_pose_translation_step_m=%.6f "
        "published_pose_rotation_step_rad=%.6f "
        "linear_velocity=(%.6f,%.6f,%.6f) "
        "angular_velocity=(%.6f,%.6f,%.6f) healthy=%s rejections=%u",
        orbslam3_ros2::AngularCorrectionClassName(diagnostics.classification),
        diagnostics.measurement_stamp_sec, diagnostics.dt_sec, drone_id_,
        static_cast<unsigned long>(diagnostics.context.map_epoch),
        diagnostics.context.tracking_state,
        static_cast<unsigned long>(diagnostics.context.reference_keyframe_id),
        diagnostics.context.reference_changed ? "true" : "false",
        diagnostics.context.frames_since_reference_change,
        diagnostics.post_reference_switch ? "true" : "false",
        diagnostics.raw_step_translation_m, diagnostics.raw_step_rotation_rad,
        diagnostics.position_innovation_m,
        diagnostics.rotation_innovation.x(), diagnostics.rotation_innovation.y(),
        diagnostics.rotation_innovation.z(), diagnostics.rotation_innovation_rad,
        diagnostics.small_rotation_limit_rad,
        diagnostics.previous_linear_velocity.x(),
        diagnostics.previous_linear_velocity.y(),
        diagnostics.previous_linear_velocity.z(),
        diagnostics.previous_angular_velocity.x(),
        diagnostics.previous_angular_velocity.y(),
        diagnostics.previous_angular_velocity.z(),
        diagnostics.implied_angular_velocity.x(),
        diagnostics.implied_angular_velocity.y(),
        diagnostics.implied_angular_velocity.z(),
        diagnostics.implied_angular_acceleration.x(),
        diagnostics.implied_angular_acceleration.y(),
        diagnostics.implied_angular_acceleration.z(),
        static_cast<unsigned long>(diagnostics.pending_correction_id),
        diagnostics.pending_good_frames, diagnostics.pending_total_frames,
        diagnostics.consistency_cosine, diagnostics.magnitude_ratio,
        diagnostics.correction_fraction_applied,
        diagnostics.applied_rotation_correction_rad,
        diagnostics.published_pose_translation_step_m,
        diagnostics.published_pose_rotation_step_rad,
        diagnostics.linear_velocity_after_limits.x(),
        diagnostics.linear_velocity_after_limits.y(),
        diagnostics.linear_velocity_after_limits.z(),
        diagnostics.angular_velocity_after_limits.x(),
        diagnostics.angular_velocity_after_limits.y(),
        diagnostics.angular_velocity_after_limits.z(),
        diagnostics.predictor_healthy ? "true" : "false",
        diagnostics.consecutive_rejections);
}

void StereoSlamNode::PublishPredictedNavigationState()
{
    if (!navigation_state_ready_)
    {
        return;
    }

    orbslam3_msgs::msg::NavigationState message = latest_navigation_state_;
    const rclcpp::Time now = this->get_clock()->now();
    float published_translation_step_m = 0.0f;
    float published_rotation_step_rad = 0.0f;
    message.header.stamp = now;
    message.sample_sequence = navigation_sample_sequence_++;
    if (message.local_valid)
    {
        const auto predicted = orb_pose_predictor_.Predict(now.seconds());
        if (!predicted.valid)
        {
            return;
        }
        message.o_t_body = SophusToPoseMsg(predicted.pose);
        if (last_published_orb_pose_valid_)
        {
            const Sophus::SE3f published_step =
                last_published_orb_pose_.inverse() * predicted.pose;
            published_translation_step_m = published_step.translation().norm();
            published_rotation_step_rad = published_step.so3().log().norm();
        }
        last_published_orb_pose_ = predicted.pose;
        last_published_orb_pose_valid_ = true;
        message.velocity_valid = predicted.velocity_valid;
        message.velocity = geometry_msgs::msg::Twist();
        if (predicted.velocity_valid)
        {
            message.velocity.linear.x = predicted.linear_velocity.x();
            message.velocity.linear.y = predicted.linear_velocity.y();
            message.velocity.linear.z = predicted.linear_velocity.z();
            message.velocity.angular.x = predicted.angular_velocity.x();
            message.velocity.angular.y = predicted.angular_velocity.y();
            message.velocity.angular.z = predicted.angular_velocity.z();
        }
        if (
            o_t_world_valid_ &&
            message.global_status !=
            orbslam3_msgs::msg::NavigationState::GLOBAL_STATUS_INVALID)
        {
            message.w_t_body = SophusToPoseMsg(
                o_t_world_.inverse() * predicted.pose);
        }
    }
    else
    {
        message.velocity_valid = false;
        message.velocity = geometry_msgs::msg::Twist();
        last_published_orb_pose_valid_ = false;
    }

    navigation_state_pub_->publish(message);
    ++orb_prediction_count_;
    if (debug_orb_control_state_)
    {
        const double state_age_sec = latest_orb_measurement_stamp_sec_ > 0.0 ?
            std::max(0.0, now.seconds() - latest_orb_measurement_stamp_sec_) : -1.0;
        const bool diagnostic_window =
            now.seconds() <= orb_diagnostic_window_until_sec_;
        if (diagnostic_window)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "[F5H-ORB-PUBLISH] publish_stamp=%.6f measurement_input_stamp=%.6f "
                "measurement_receive_stamp=%.6f "
                "state_age_sec=%.6f drone_id=%u sample=%lu source=%u "
                "local_valid=%s continuity_valid=%s velocity_valid=%s "
                "epoch=%lu ref_kf=%lu tracking=%d "
                "pose_p=(%.6f,%.6f,%.6f) pose_q=(%.6f,%.6f,%.6f,%.6f) "
                "v=(%.6f,%.6f,%.6f) omega=(%.6f,%.6f,%.6f) "
                "published_translation_step_m=%.6f "
                "published_rotation_step_rad=%.6f",
                now.seconds(), latest_orb_measurement_input_stamp_sec_,
                latest_orb_measurement_stamp_sec_, state_age_sec,
                drone_id_, static_cast<unsigned long>(message.sample_sequence),
                message.pose_source, message.local_valid ? "true" : "false",
                message.local_continuity_valid ? "true" : "false",
                message.velocity_valid ? "true" : "false",
                static_cast<unsigned long>(message.map_epoch),
                static_cast<unsigned long>(message.reference_keyframe_id),
                message.tracking_state,
                message.o_t_body.position.x, message.o_t_body.position.y,
                message.o_t_body.position.z, message.o_t_body.orientation.x,
                message.o_t_body.orientation.y, message.o_t_body.orientation.z,
                message.o_t_body.orientation.w, message.velocity.linear.x,
                message.velocity.linear.y, message.velocity.linear.z,
                message.velocity.angular.x, message.velocity.angular.y,
                message.velocity.angular.z, published_translation_step_m,
                published_rotation_step_rad);
        }
        else
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 200,
                "[F5H-ORB-PUBLISH] publish_stamp=%.6f measurement_input_stamp=%.6f "
                "measurement_receive_stamp=%.6f "
                "state_age_sec=%.6f drone_id=%u source=%u local_valid=%s "
                "velocity_valid=%s epoch=%lu ref_kf=%lu tracking=%d "
                "published_rotation_step_rad=%.6f omega=(%.6f,%.6f,%.6f)",
                now.seconds(), latest_orb_measurement_input_stamp_sec_,
                latest_orb_measurement_stamp_sec_, state_age_sec,
                drone_id_, message.pose_source,
                message.local_valid ? "true" : "false",
                message.velocity_valid ? "true" : "false",
                static_cast<unsigned long>(message.map_epoch),
                static_cast<unsigned long>(message.reference_keyframe_id),
                message.tracking_state, published_rotation_step_rad,
                message.velocity.angular.x, message.velocity.angular.y,
                message.velocity.angular.z);
        }
    }
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 10000,
        "[F5H-ORB-STATE-PREDICTOR] measurements=%lu predictions=%lu "
        "limited_measurements=%lu publish_rate_hz=%.3f local_valid=%s",
        static_cast<unsigned long>(orb_measurement_count_),
        static_cast<unsigned long>(orb_prediction_count_),
        static_cast<unsigned long>(orb_limited_measurement_count_),
        orb_state_publish_rate_hz_,
        message.local_valid ? "true" : "false");
}

void StereoSlamNode::RequestGlobalPose(uint64_t map_epoch, uint64_t keyframe_id)
{
    if (!global_pose_client_->service_is_ready())
    {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "[F5D-KF-REQUEST-WAIT] drone_id=%u epoch=%lu kf=%lu",
            drone_id_, static_cast<unsigned long>(map_epoch),
            static_cast<unsigned long>(keyframe_id));
        return;
    }
    auto request =
        std::make_shared<orbslam3_msgs::srv::GetGlobalKeyFramePose::Request>();
    request->drone_id = drone_id_;
    request->map_epoch = map_epoch;
    request->keyframe_id = keyframe_id;
    global_pose_request_valid_ = true;
    requested_global_epoch_ = map_epoch;
    requested_global_keyframe_id_ = keyframe_id;
    const uint64_t generation = ++global_pose_request_generation_;
    global_pose_client_->async_send_request(
        request,
        [this, generation, map_epoch, keyframe_id](
            rclcpp::Client<orbslam3_msgs::srv::GetGlobalKeyFramePose>::SharedFuture
                future)
        {
            HandleGlobalPoseResponse(
                generation, map_epoch, keyframe_id, std::move(future));
        });
    RCLCPP_INFO(
        this->get_logger(),
        "[F5D-KF-REQUEST] drone_id=%u epoch=%lu kf=%lu generation=%lu",
        drone_id_, static_cast<unsigned long>(map_epoch),
        static_cast<unsigned long>(keyframe_id),
        static_cast<unsigned long>(generation));
}

void StereoSlamNode::HandleGlobalPoseResponse(
    uint64_t generation,
    uint64_t map_epoch,
    uint64_t keyframe_id,
    rclcpp::Client<orbslam3_msgs::srv::GetGlobalKeyFramePose>::SharedFuture future)
{
    if (generation != global_pose_request_generation_ ||
        map_epoch != requested_global_epoch_ ||
        keyframe_id != requested_global_keyframe_id_)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "[F5D-STALE-RESPONSE] drone_id=%u epoch=%lu kf=%lu generation=%lu",
            drone_id_, static_cast<unsigned long>(map_epoch),
            static_cast<unsigned long>(keyframe_id),
            static_cast<unsigned long>(generation));
        return;
    }
    ApplyGlobalPoseMessage(future.get()->result, "service");
}

void StereoSlamNode::HandleGlobalPosePush(
    orbslam3_msgs::msg::GlobalKeyFramePose::ConstSharedPtr message)
{
    ApplyGlobalPoseMessage(*message, "push");
}

void StereoSlamNode::ApplyGlobalPoseMessage(
    const orbslam3_msgs::msg::GlobalKeyFramePose& message,
    const char* source)
{
    if (message.drone_id != drone_id_ || !global_pose_request_valid_ ||
        message.map_epoch != requested_global_epoch_ ||
        message.keyframe_id != requested_global_keyframe_id_)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "[F5D-STALE-REVISION] drone_id=%u msg_drone=%u epoch=%lu kf=%lu "
            "revision=%lu source=%s",
            drone_id_, message.drone_id,
            static_cast<unsigned long>(message.map_epoch),
            static_cast<unsigned long>(message.keyframe_id),
            static_cast<unsigned long>(message.pose_revision), source);
        return;
    }
    if (message.status == orbslam3_msgs::msg::GlobalKeyFramePose::STATUS_AVAILABLE)
    {
        const bool accepted = navigation_state_estimator_.ApplyAuthoritativeGlobalPose(
            message.map_epoch, message.keyframe_id, message.pose_revision,
            PoseMsgToSophus(message.w_t_keyframe));
        RCLCPP_WARN(
            this->get_logger(),
            "[F5E-GLOBAL-AUTHORITY] drone_id=%u epoch=%lu kf=%lu revision=%lu "
            "source=%s accepted=%s",
            drone_id_, static_cast<unsigned long>(message.map_epoch),
            static_cast<unsigned long>(message.keyframe_id),
            static_cast<unsigned long>(message.pose_revision), source,
            accepted ? "true" : "false");
        return;
    }
    if (message.status ==
        orbslam3_msgs::msg::GlobalKeyFramePose::STATUS_INVALID_EPOCH)
    {
        navigation_state_estimator_.InvalidateGlobalPose(
            message.map_epoch, message.keyframe_id);
    }
    RCLCPP_INFO(
        this->get_logger(),
        "[F5D-KF-RESPONSE] drone_id=%u epoch=%lu kf=%lu status=%u source=%s",
        drone_id_, static_cast<unsigned long>(message.map_epoch),
        static_cast<unsigned long>(message.keyframe_id), message.status, source);
}


// ============================================================
// Publicar delta incremental del mapa local ORB
// ============================================================

void StereoSlamNode::PublishOrbMapDelta()
{
    orbslam3_msgs::msg::OrbMap delta_msg =
        BuildOrbMap(false, true);

    if (delta_msg.mappoints.empty() &&
        delta_msg.keyframes.empty())
    {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            3000,
            "[PIPE0-WRAPPER-DELTA-SKIP] drone_id=%u epoch=%lu seq=%lu "
            "reason=empty_delta",
            drone_id_,
            map_epoch_,
            map_sequence_);

        return;
    }

    orb_map_delta_pub_->publish(delta_msg);

    RCLCPP_WARN(
        this->get_logger(),
        "[PIPE0-WRAPPER-DELTA-PUB] drone_id=%u epoch=%lu seq=%lu "
        "mps=%zu kfs=%zu frame_id=%s camera_valid=%s",
        drone_id_,
        delta_msg.map_epoch,
        delta_msg.map_sequence,
        delta_msg.mappoints.size(),
        delta_msg.keyframes.size(),
        delta_msg.header.frame_id.c_str(),
        has_camera_info_ ? "true" : "false");
}


// ============================================================
// Servicio snapshot completo
// ============================================================

void StereoSlamNode::GetFullMapServiceCallback(
    const std::shared_ptr<orbslam3_msgs::srv::GetOrbMap::Request> request,
    std::shared_ptr<orbslam3_msgs::srv::GetOrbMap::Response> response)
{
    (void)request;

    response->map =
        BuildOrbMap(true, true);

    RCLCPP_WARN(
        this->get_logger(),
        "[PIPE0-WRAPPER-FULL-SNAPSHOT] drone_id=%u epoch=%lu seq=%lu "
        "mps=%zu kfs=%zu",
        drone_id_,
        response->map.map_epoch,
        response->map.map_sequence,
        response->map.mappoints.size(),
        response->map.keyframes.size());
}


// ============================================================
// Construir OrbMap.
// full_snapshot = true  -> envía todo.
// full_snapshot = false -> solo nuevos/cambiados/borrados.
// update_cache = true   -> actualiza hashes enviados.
// ============================================================

orbslam3_msgs::msg::OrbMap StereoSlamNode::BuildOrbMap(
    bool full_snapshot,
    bool update_cache)
{
    UpdateMapEpochFromCurrentMap();

    orbslam3_msgs::msg::OrbMap map_msg;

    ORB_SLAM3::Map* active_map_ptr =
        current_orb_map_ptr_;

    if (!active_map_ptr)
    {
        active_map_ptr =
            GetCurrentMapPointerFromKeyFrames();
    }

    if (!active_map_ptr)
    {
        return map_msg;
    }

    map_msg.header.stamp = this->now();
    map_msg.header.frame_id = local_map_frame_;

    map_msg.drone_id = drone_id_;
    map_msg.drone_name = drone_name_;
    map_msg.map_frame = local_map_frame_;
    map_msg.map_sequence = map_sequence_++;

    map_msg.map_epoch = map_epoch_;

    map_msg.fx = camera_fx_;
    map_msg.fy = camera_fy_;
    map_msg.cx = camera_cx_;
    map_msg.cy = camera_cy_;
    map_msg.bf = camera_bf_;

    map_msg.image_width = image_width_;
    map_msg.image_height = image_height_;

    std::unordered_map<uint64_t, std::size_t> next_mappoint_hashes =
        full_snapshot ?
        std::unordered_map<uint64_t, std::size_t>() :
        sent_mappoint_hashes_;

    std::unordered_map<uint64_t, std::size_t> next_keyframe_hashes =
        full_snapshot ?
        std::unordered_map<uint64_t, std::size_t>() :
        sent_keyframe_hashes_;

    // ============================================================
    // MapPoints
    // ============================================================

    auto all_mps = m_SLAM->GetAllMapPoints();

    map_msg.mappoints.reserve(all_mps.size());

    for (auto* pMP : all_mps)
    {
        if (!pMP)
            continue;
        if (!MapPointBelongsToMap(pMP, active_map_ptr))
        continue;

        const uint64_t id =
            static_cast<uint64_t>(pMP->mnId);

        if (pMP->isBad())
        {
            if (!full_snapshot &&
                sent_mappoint_hashes_.find(id) != sent_mappoint_hashes_.end())
            {
                orbslam3_msgs::msg::OrbMapPoint bad_msg;

                bad_msg.id = id;
                bad_msg.is_bad = true;
                bad_msg.descriptor.data.fill(0);

                map_msg.mappoints.push_back(bad_msg);

                next_mappoint_hashes.erase(id);
            }

            continue;
        }

        const std::size_t hash =
            HashMapPoint(pMP);

        auto old_it =
            sent_mappoint_hashes_.find(id);

        const bool is_new =
            old_it == sent_mappoint_hashes_.end();

        const bool changed =
            !is_new && old_it->second != hash;

        if (full_snapshot || is_new || changed)
        {
            orbslam3_msgs::msg::OrbMapPoint mp_msg;

            FillMapPointMsg(pMP, mp_msg);

            map_msg.mappoints.push_back(mp_msg);
        }

        next_mappoint_hashes[id] = hash;
    }

    // ============================================================
    // KeyFrames
    // ============================================================

    auto all_kfs = m_SLAM->GetAllKeyFrames();

    map_msg.keyframes.reserve(all_kfs.size());

    for (auto* pKF : all_kfs)
    {
        if (!pKF)
            continue;
        if (!KeyFrameBelongsToMap(pKF, active_map_ptr))
            continue;

        const uint64_t id =
            static_cast<uint64_t>(pKF->mnId);

        if (pKF->isBad())
        {
            if (!full_snapshot &&
                sent_keyframe_hashes_.find(id) != sent_keyframe_hashes_.end())
            {
                orbslam3_msgs::msg::OrbKeyFrame bad_kf_msg;

                bad_kf_msg.id = id;
                bad_kf_msg.frame_id = local_map_frame_;
                bad_kf_msg.is_bad = true;

                map_msg.keyframes.push_back(bad_kf_msg);

                next_keyframe_hashes.erase(id);
            }

            continue;
        }

        const std::size_t hash =
            HashKeyFrame(pKF);

        auto old_it =
            sent_keyframe_hashes_.find(id);

        const bool is_new =
            old_it == sent_keyframe_hashes_.end();

        const bool changed =
            !is_new && old_it->second != hash;

        if (full_snapshot || is_new || changed)
        {
            orbslam3_msgs::msg::OrbKeyFrame kf_msg;

            FillKeyFrameMsg(pKF, kf_msg);

            map_msg.keyframes.push_back(kf_msg);
        }

        next_keyframe_hashes[id] = hash;
    }

    if (update_cache)
    {
        sent_mappoint_hashes_ =
            std::move(next_mappoint_hashes);

        sent_keyframe_hashes_ =
            std::move(next_keyframe_hashes);
    }

    return map_msg;
}


// ============================================================
// Rellenar MapPoint con información útil para servidor global.
// Exporta:
// - posición
// - descriptor representativo
// - calidad
// - normal
// - distancias invariantes
// - referencia
// - observaciones KeyFrame-feature
// ============================================================

void StereoSlamNode::FillMapPointMsg(
    ORB_SLAM3::MapPoint* pMP,
    orbslam3_msgs::msg::OrbMapPoint& mp_msg)
{
    mp_msg.id =
        static_cast<uint64_t>(pMP->mnId);

    Eigen::Vector3f pos =
        pMP->GetWorldPos();

    mp_msg.position.x = pos.x();
    mp_msg.position.y = pos.y();
    mp_msg.position.z = pos.z();

    mp_msg.descriptor.data.fill(0);

    cv::Mat desc =
        pMP->GetDescriptor();

    if (!desc.empty() && desc.cols >= 32)
    {
        for (int j = 0; j < 32; ++j)
        {
            mp_msg.descriptor.data[j] =
                desc.at<unsigned char>(0, j);
        }
    }

    mp_msg.is_bad = false;

    mp_msg.observations_count =
        static_cast<uint32_t>(pMP->Observations());

    mp_msg.found_ratio =
        static_cast<float>(pMP->GetFoundRatio());

    Eigen::Vector3f normal =
        pMP->GetNormal();

    mp_msg.normal_vector.x = normal.x();
    mp_msg.normal_vector.y = normal.y();
    mp_msg.normal_vector.z = normal.z();

    mp_msg.min_distance =
        static_cast<float>(pMP->GetMinDistanceInvariance());

    mp_msg.max_distance =
        static_cast<float>(pMP->GetMaxDistanceInvariance());

    ORB_SLAM3::KeyFrame* ref_kf =
        pMP->GetReferenceKeyFrame();

    if (ref_kf && !ref_kf->isBad())
    {
        mp_msg.reference_keyframe_id =
            static_cast<uint64_t>(ref_kf->mnId);
    }
    else
    {
        mp_msg.reference_keyframe_id = 0;
    }

    // Observaciones del MapPoint:
    // qué keyframes lo ven y en qué índice de feature.
    mp_msg.observations.clear();

    auto observations =
        pMP->GetObservations();

    mp_msg.observations.reserve(observations.size());

    for (const auto& obs_it : observations)
    {
        ORB_SLAM3::KeyFrame* pKF =
            obs_it.first;

        if (!pKF || pKF->isBad())
            continue;

        orbslam3_msgs::msg::OrbObservation obs_msg;

        obs_msg.keyframe_id =
            static_cast<uint64_t>(pKF->mnId);

        const int left_idx =
            std::get<0>(obs_it.second);

        const int right_idx =
            std::get<1>(obs_it.second);

        if (left_idx < 0)
            continue;

        obs_msg.keypoint_index =
            static_cast<uint32_t>(left_idx);

        obs_msg.right_keypoint_index =
            right_idx;

        mp_msg.observations.push_back(obs_msg);
    }
}


// ============================================================
// Rellenar KeyFrame con la información que ORB-SLAM3 usa:
// - pose
// - keypoints + descriptors + stereo info
// - asociaciones a MapPoints
// - covisibilidad
// - BoW vector
// - FeatureVector
// - spanning tree
// - loop edges locales
// ============================================================

void StereoSlamNode::FillKeyFrameMsg(
    ORB_SLAM3::KeyFrame* pKF,
    orbslam3_msgs::msg::OrbKeyFrame& kf_msg)
{
    kf_msg.id =
        static_cast<uint64_t>(pKF->mnId);

    kf_msg.frame_id =
        local_map_frame_;

    kf_msg.camera_id = 0;
    kf_msg.is_bad = false;

    Sophus::SE3f Twc =
        pKF->GetPoseInverse();

    kf_msg.pose =
        SophusToPoseMsg(Twc);

    kf_msg.stamp = TimestampToRosTime(pKF->mTimeStamp);

    const std::vector<cv::KeyPoint>& keypoints =
        pKF->mvKeysUn;

    const cv::Mat& descriptors =
        pKF->mDescriptors;

    std::vector<ORB_SLAM3::MapPoint*> map_points =
        pKF->GetMapPointMatches();

    kf_msg.keypoints.clear();
    kf_msg.mappoint_ids.clear();

    kf_msg.keypoints.reserve(keypoints.size());
    kf_msg.mappoint_ids.reserve(keypoints.size());

    // ============================================================
    // Keypoints, descriptores, stereo info y asociación a MapPoint
    // ============================================================

    for (size_t i = 0; i < keypoints.size(); ++i)
    {
        orbslam3_msgs::msg::OrbKeyPoint kp_msg;

        kp_msg.u = keypoints[i].pt.x;
        kp_msg.v = keypoints[i].pt.y;
        kp_msg.size = keypoints[i].size;
        kp_msg.angle = keypoints[i].angle;
        kp_msg.response = keypoints[i].response;
        kp_msg.octave = keypoints[i].octave;
        kp_msg.class_id = keypoints[i].class_id;

        kp_msg.descriptor.data.fill(0);

        if (!descriptors.empty() &&
            descriptors.rows > static_cast<int>(i) &&
            descriptors.cols >= 32)
        {
            for (int j = 0; j < 32; ++j)
            {
                kp_msg.descriptor.data[j] =
                    descriptors.at<unsigned char>(
                        static_cast<int>(i),
                        j);
            }
        }

        if (i < pKF->mvuRight.size())
        {
            kp_msg.u_right =
                static_cast<float>(pKF->mvuRight[i]);
        }
        else
        {
            kp_msg.u_right = -1.0f;
        }

        if (i < pKF->mvDepth.size())
        {
            kp_msg.depth =
                static_cast<float>(pKF->mvDepth[i]);
        }
        else
        {
            kp_msg.depth = -1.0f;
        }

        kf_msg.keypoints.push_back(kp_msg);

        if (i < map_points.size() &&
            map_points[i] &&
            !map_points[i]->isBad())
        {
            kf_msg.mappoint_ids.push_back(
                static_cast<uint64_t>(map_points[i]->mnId));
        }
        else
        {
            kf_msg.mappoint_ids.push_back(0);
        }
    }


    // ============================================================
    // Covisibilidad
    // ============================================================

    kf_msg.connected_keyframe_ids.clear();
    kf_msg.connected_keyframe_weights.clear();

    std::vector<ORB_SLAM3::KeyFrame*> connected_kfs =
        pKF->GetVectorCovisibleKeyFrames();

    for (auto* pConnectedKF : connected_kfs)
    {
        if (!pConnectedKF || pConnectedKF->isBad())
            continue;

        int weight =
            pKF->GetWeight(pConnectedKF);

        kf_msg.connected_keyframe_ids.push_back(
            static_cast<uint64_t>(pConnectedKF->mnId));

        kf_msg.connected_keyframe_weights.push_back(
            static_cast<uint32_t>(std::max(0, weight)));
    }

    // ============================================================
    // BoW vector usado por KeyFrameDatabase / place recognition
    // ============================================================

    kf_msg.bow_word_ids.clear();
    kf_msg.bow_word_values.clear();

    kf_msg.bow_word_ids.reserve(pKF->mBowVec.size());
    kf_msg.bow_word_values.reserve(pKF->mBowVec.size());

    for (const auto& bow_it : pKF->mBowVec)
    {
        kf_msg.bow_word_ids.push_back(
            static_cast<uint32_t>(bow_it.first));

        kf_msg.bow_word_values.push_back(
            static_cast<float>(bow_it.second));
    }

    // ============================================================
    // FeatureVector usado por ORBmatcher::SearchByBoW
    // ============================================================

    kf_msg.feat_node_ids.clear();
    kf_msg.feat_node_start_indices.clear();
    kf_msg.feat_node_sizes.clear();
    kf_msg.feat_indices.clear();

    uint32_t offset = 0;

    for (const auto& feat_it : pKF->mFeatVec)
    {
        kf_msg.feat_node_ids.push_back(
            static_cast<uint32_t>(feat_it.first));

        kf_msg.feat_node_start_indices.push_back(offset);

        kf_msg.feat_node_sizes.push_back(
            static_cast<uint32_t>(feat_it.second.size()));

        for (size_t idx : feat_it.second)
        {
            kf_msg.feat_indices.push_back(
                static_cast<uint32_t>(idx));

            offset++;
        }
    }

    // ============================================================
    // Spanning tree: parent / children
    // ============================================================

    ORB_SLAM3::KeyFrame* parent =
        pKF->GetParent();

    if (parent && !parent->isBad())
        kf_msg.parent_keyframe_id =
            static_cast<uint64_t>(parent->mnId);
    else
        kf_msg.parent_keyframe_id = 0;

    kf_msg.child_keyframe_ids.clear();

    std::set<ORB_SLAM3::KeyFrame*> children =
        pKF->GetChilds();

    for (auto* child : children)
    {
        if (child && !child->isBad())
            kf_msg.child_keyframe_ids.push_back(
                static_cast<uint64_t>(child->mnId));
    }


    // ============================================================
    // Loop edges locales detectadas por ORB-SLAM3 local
    // ============================================================

    kf_msg.loop_edge_keyframe_ids.clear();

    std::set<ORB_SLAM3::KeyFrame*> loop_edges =
        pKF->GetLoopEdges();

    for (auto* loop_kf : loop_edges)
    {
        if (loop_kf && !loop_kf->isBad())
        {
            kf_msg.loop_edge_keyframe_ids.push_back(
                static_cast<uint64_t>(loop_kf->mnId));
        }
    }
}


// ============================================================
// Conversión Sophus SE3 -> geometry_msgs/Pose
// ============================================================

geometry_msgs::msg::Pose StereoSlamNode::SophusToPoseMsg(
    const Sophus::SE3f& T)
{
    geometry_msgs::msg::Pose pose_msg;

    Eigen::Matrix3f R =
        T.rotationMatrix();

    Eigen::Vector3f t =
        T.translation();

    Eigen::Quaternionf q(R);
    q.normalize();

    pose_msg.position.x = t.x();
    pose_msg.position.y = t.y();
    pose_msg.position.z = t.z();

    pose_msg.orientation.x = q.x();
    pose_msg.orientation.y = q.y();
    pose_msg.orientation.z = q.z();
    pose_msg.orientation.w = q.w();

    return pose_msg;
}

Sophus::SE3f StereoSlamNode::PoseMsgToSophus(
    const geometry_msgs::msg::Pose& pose)
{
    Eigen::Quaternionf quaternion(
        static_cast<float>(pose.orientation.w),
        static_cast<float>(pose.orientation.x),
        static_cast<float>(pose.orientation.y),
        static_cast<float>(pose.orientation.z));
    if (quaternion.norm() < 1e-6f)
    {
        quaternion = Eigen::Quaternionf::Identity();
    }
    else
    {
        quaternion.normalize();
    }
    return Sophus::SE3f(
        quaternion.toRotationMatrix(),
        Eigen::Vector3f(
            static_cast<float>(pose.position.x),
            static_cast<float>(pose.position.y),
            static_cast<float>(pose.position.z)));
}


// ============================================================
// Hash MapPoint.
// Si ORB-SLAM3 cambia posición, descriptor, observaciones,
// normal, distancias o referencia, el servidor recibirá update.
// ============================================================

std::size_t StereoSlamNode::HashMapPoint(
    ORB_SLAM3::MapPoint* pMP)
{
    std::size_t seed = 0;

    HashCombine(seed, static_cast<std::size_t>(pMP->mnId));

    Eigen::Vector3f pos =
        pMP->GetWorldPos();

    const int64_t qx =
        static_cast<int64_t>(std::llround(pos.x() * 1000.0f));

    const int64_t qy =
        static_cast<int64_t>(std::llround(pos.y() * 1000.0f));

    const int64_t qz =
        static_cast<int64_t>(std::llround(pos.z() * 1000.0f));

    HashCombine(seed, std::hash<int64_t>{}(qx));
    HashCombine(seed, std::hash<int64_t>{}(qy));
    HashCombine(seed, std::hash<int64_t>{}(qz));

    cv::Mat desc =
        pMP->GetDescriptor();

    if (!desc.empty() && desc.cols >= 32)
    {
        for (int j = 0; j < 32; ++j)
        {
            HashCombine(
                seed,
                static_cast<std::size_t>(
                    desc.at<unsigned char>(0, j)));
        }
    }

    HashCombine(
        seed,
        static_cast<std::size_t>(pMP->Observations()));

    HashCombine(
        seed,
        std::hash<int64_t>{}(
            static_cast<int64_t>(
                std::llround(pMP->GetFoundRatio() * 1000000.0f))));

    Eigen::Vector3f normal =
        pMP->GetNormal();

    HashCombine(
        seed,
        std::hash<int64_t>{}(
            static_cast<int64_t>(
                std::llround(normal.x() * 1000000.0f))));

    HashCombine(
        seed,
        std::hash<int64_t>{}(
            static_cast<int64_t>(
                std::llround(normal.y() * 1000000.0f))));

    HashCombine(
        seed,
        std::hash<int64_t>{}(
            static_cast<int64_t>(
                std::llround(normal.z() * 1000000.0f))));

    HashCombine(
        seed,
        std::hash<int64_t>{}(
            static_cast<int64_t>(
                std::llround(pMP->GetMinDistanceInvariance() * 1000.0f))));

    HashCombine(
        seed,
        std::hash<int64_t>{}(
            static_cast<int64_t>(
                std::llround(pMP->GetMaxDistanceInvariance() * 1000.0f))));

    ORB_SLAM3::KeyFrame* ref_kf =
        pMP->GetReferenceKeyFrame();

    if (ref_kf && !ref_kf->isBad())
    {
        HashCombine(
            seed,
            static_cast<std::size_t>(ref_kf->mnId));
    }
    else
    {
        HashCombine(seed, 0);
    }

    auto observations =
        pMP->GetObservations();

    HashCombine(seed, observations.size());

    for (const auto& obs_it : observations)
    {
        ORB_SLAM3::KeyFrame* pKF =
            obs_it.first;

        if (!pKF || pKF->isBad())
            continue;

        HashCombine(
            seed,
            static_cast<std::size_t>(pKF->mnId));

        HashCombine(
            seed,
            std::hash<int64_t>{}(
                static_cast<int64_t>(std::get<0>(obs_it.second))));

        HashCombine(
            seed,
            std::hash<int64_t>{}(
                static_cast<int64_t>(std::get<1>(obs_it.second))));
    }

    return seed;
}


// ============================================================
// Hash KeyFrame.
// Incluye pose, asociaciones, covisibilidad, BoW, FeatureVector,
// información estéreo, spanning tree y loop edges locales.
// ============================================================

std::size_t StereoSlamNode::HashKeyFrame(
    ORB_SLAM3::KeyFrame* pKF)
{
    std::size_t seed = 0;

    HashCombine(seed, static_cast<std::size_t>(pKF->mnId));

    Sophus::SE3f Twc =
        pKF->GetPoseInverse();

    Eigen::Vector3f t =
        Twc.translation();

    Eigen::Quaternionf q(
        Twc.rotationMatrix());

    q.normalize();

    const int64_t tx =
        static_cast<int64_t>(std::llround(t.x() * 1000.0f));

    const int64_t ty =
        static_cast<int64_t>(std::llround(t.y() * 1000.0f));

    const int64_t tz =
        static_cast<int64_t>(std::llround(t.z() * 1000.0f));
    const int64_t qx =
        static_cast<int64_t>(std::llround(q.x() * 1000000.0f));

    const int64_t qy =
        static_cast<int64_t>(std::llround(q.y() * 1000000.0f));

    const int64_t qz =
        static_cast<int64_t>(std::llround(q.z() * 1000000.0f));

    const int64_t qw =
        static_cast<int64_t>(std::llround(q.w() * 1000000.0f));

    HashCombine(seed, std::hash<int64_t>{}(tx));
    HashCombine(seed, std::hash<int64_t>{}(ty));
    HashCombine(seed, std::hash<int64_t>{}(tz));

    HashCombine(seed, std::hash<int64_t>{}(qx));
    HashCombine(seed, std::hash<int64_t>{}(qy));
    HashCombine(seed, std::hash<int64_t>{}(qz));
    HashCombine(seed, std::hash<int64_t>{}(qw));

    std::vector<ORB_SLAM3::MapPoint*> map_points =
        pKF->GetMapPointMatches();

    HashCombine(seed, map_points.size());

    for (auto* pMP : map_points)
    {
        if (pMP && !pMP->isBad())
        {
            HashCombine(
                seed,
                static_cast<std::size_t>(pMP->mnId));
        }
        else
        {
            HashCombine(seed, 0);
        }
    }

    std::vector<ORB_SLAM3::KeyFrame*> connected_kfs =
        pKF->GetVectorCovisibleKeyFrames();

    HashCombine(seed, connected_kfs.size());

    for (auto* pConnectedKF : connected_kfs)
    {
        if (!pConnectedKF || pConnectedKF->isBad())
            continue;

        HashCombine(
            seed,
            static_cast<std::size_t>(pConnectedKF->mnId));

        HashCombine(
            seed,
            static_cast<std::size_t>(
                std::max(0, pKF->GetWeight(pConnectedKF))));
    }

    // BoW
    HashCombine(seed, pKF->mBowVec.size());

    for (const auto& bow_it : pKF->mBowVec)
    {
        HashCombine(seed, static_cast<std::size_t>(bow_it.first));
        HashCombine(seed, std::hash<int64_t>{}(
            static_cast<int64_t>(
                std::llround(bow_it.second * 1000000.0))));
    }

    // FeatureVector
    HashCombine(seed, pKF->mFeatVec.size());

    for (const auto& feat_it : pKF->mFeatVec)
    {
        HashCombine(seed, static_cast<std::size_t>(feat_it.first));
        HashCombine(seed, feat_it.second.size());

        for (size_t idx : feat_it.second)
            HashCombine(seed, static_cast<std::size_t>(idx));
    }

    // Stereo
    HashCombine(seed, pKF->mvuRight.size());

    for (float ur : pKF->mvuRight)
    {
        HashCombine(seed, std::hash<int64_t>{}(
            static_cast<int64_t>(std::llround(ur * 1000.0f))));
    }

    HashCombine(seed, pKF->mvDepth.size());

    for (float d : pKF->mvDepth)
    {
        HashCombine(seed, std::hash<int64_t>{}(
            static_cast<int64_t>(std::llround(d * 1000.0f))));
    }

    // Parent
    ORB_SLAM3::KeyFrame* parent = pKF->GetParent();

    if (parent && !parent->isBad())
        HashCombine(seed, static_cast<std::size_t>(parent->mnId));
    else
        HashCombine(seed, 0);

    // Children
    std::set<ORB_SLAM3::KeyFrame*> children = pKF->GetChilds();

    HashCombine(seed, children.size());

    for (auto* child : children)
    {
        if (child && !child->isBad())
            HashCombine(seed, static_cast<std::size_t>(child->mnId));
    }

    // Local loop edges
    std::set<ORB_SLAM3::KeyFrame*> loop_edges = pKF->GetLoopEdges();

    HashCombine(seed, loop_edges.size());

    for (auto* loop_kf : loop_edges)
    {
        if (loop_kf && !loop_kf->isBad())
            HashCombine(seed, static_cast<std::size_t>(loop_kf->mnId));
    }

    return seed;
}


double StereoSlamNode::CvMatAtDouble(
    const cv::Mat& mat,
    int r,
    int c) const
{
    if (mat.empty())
        return 0.0;

    if (mat.type() == CV_32F)
        return static_cast<double>(mat.at<float>(r, c));

    if (mat.type() == CV_64F)
        return mat.at<double>(r, c);

    cv::Mat tmp;
    mat.convertTo(tmp, CV_64F);

    return tmp.at<double>(r, c);
}


void StereoSlamNode::LoadCameraInfoFromSettings(
    const std::string& settings_file)
{
    cv::FileStorage fsSettings(
        settings_file,
        cv::FileStorage::READ);

    if (!fsSettings.isOpened())
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Could not open settings file to load camera info: %s",
            settings_file.c_str());

        has_camera_info_ = false;
        return;
    }

    // ============================================================
    // 1. Leer formato ORB-SLAM3 estándar:
    //
    // Camera.fx
    // Camera.fy
    // Camera.cx
    // Camera.cy
    // Camera.bf
    // Camera.width
    // Camera.height
    //
    // Tu YAML usa este formato.
    // ============================================================

    if (!fsSettings["Camera.fx"].empty())
    {
        camera_fx_ =
            static_cast<float>(
                static_cast<double>(fsSettings["Camera.fx"]));
    }

    if (!fsSettings["Camera.fy"].empty())
    {
        camera_fy_ =
            static_cast<float>(
                static_cast<double>(fsSettings["Camera.fy"]));
    }

    if (!fsSettings["Camera.cx"].empty())
    {
        camera_cx_ =
            static_cast<float>(
                static_cast<double>(fsSettings["Camera.cx"]));
    }

    if (!fsSettings["Camera.cy"].empty())
    {
        camera_cy_ =
            static_cast<float>(
                static_cast<double>(fsSettings["Camera.cy"]));
    }

    if (!fsSettings["Camera.bf"].empty())
    {
        camera_bf_ =
            static_cast<float>(
                static_cast<double>(fsSettings["Camera.bf"]));
    }

    if (!fsSettings["Camera.width"].empty())
    {
        const int width =
            static_cast<int>(fsSettings["Camera.width"]);

        image_width_ =
            static_cast<uint32_t>(std::max(0, width));
    }

    if (!fsSettings["Camera.height"].empty())
    {
        const int height =
            static_cast<int>(fsSettings["Camera.height"]);

        image_height_ =
            static_cast<uint32_t>(std::max(0, height));
    }

    // ============================================================
    // 2. Fallback para YAMLs rectificados tipo LEFT.K / LEFT.width.
    //
    // Si algún parámetro Camera.* no existe, intentamos cargar LEFT.*.
    // Esto mantiene compatibilidad con otros datasets.
    // ============================================================

    if (camera_fx_ <= 0.0f ||
        camera_fy_ <= 0.0f ||
        camera_cx_ < 0.0f ||
        camera_cy_ < 0.0f)
    {
        cv::Mat K_l;
        fsSettings["LEFT.K"] >> K_l;

        if (!K_l.empty())
        {
            camera_fx_ =
                static_cast<float>(CvMatAtDouble(K_l, 0, 0));

            camera_fy_ =
                static_cast<float>(CvMatAtDouble(K_l, 1, 1));

            camera_cx_ =
                static_cast<float>(CvMatAtDouble(K_l, 0, 2));

            camera_cy_ =
                static_cast<float>(CvMatAtDouble(K_l, 1, 2));
        }
    }

    if (image_width_ == 0 &&
        !fsSettings["LEFT.width"].empty())
    {
        const int width =
            static_cast<int>(fsSettings["LEFT.width"]);

        image_width_ =
            static_cast<uint32_t>(std::max(0, width));
    }

    if (image_height_ == 0 &&
        !fsSettings["LEFT.height"].empty())
    {
        const int height =
            static_cast<int>(fsSettings["LEFT.height"]);

        image_height_ =
            static_cast<uint32_t>(std::max(0, height));
    }

    // ============================================================
    // 3. Validación final.
    // ============================================================

    has_camera_info_ =
        camera_fx_ > 0.0f &&
        camera_fy_ > 0.0f &&
        camera_cx_ >= 0.0f &&
        camera_cy_ >= 0.0f &&
        camera_bf_ > 0.0f &&
        image_width_ > 0 &&
        image_height_ > 0;

    RCLCPP_INFO(
        this->get_logger(),
        "Camera info loaded from %s: fx=%.3f fy=%.3f cx=%.3f cy=%.3f bf=%.3f width=%u height=%u valid=%s",
        settings_file.c_str(),
        camera_fx_,
        camera_fy_,
        camera_cx_,
        camera_cy_,
        camera_bf_,
        image_width_,
        image_height_,
        has_camera_info_ ? "true" : "false");
}


bool StereoSlamNode::KeyFrameBelongsToMap(
    ORB_SLAM3::KeyFrame* pKF,
    ORB_SLAM3::Map* pMap) const
{
    if (!pKF || !pMap || pKF->isBad())
        return false;

    return pKF->GetMap() == pMap;
}


bool StereoSlamNode::MapPointBelongsToMap(
    ORB_SLAM3::MapPoint* pMP,
    ORB_SLAM3::Map* pMap) const
{
    if (!pMP || !pMap || pMP->isBad())
        return false;

    ORB_SLAM3::KeyFrame* ref_kf =
        pMP->GetReferenceKeyFrame();

    if (ref_kf && !ref_kf->isBad() && ref_kf->GetMap() == pMap)
        return true;

    auto observations =
        pMP->GetObservations();

    for (const auto& obs_it : observations)
    {
        ORB_SLAM3::KeyFrame* pKF =
            obs_it.first;

        if (pKF && !pKF->isBad() && pKF->GetMap() == pMap)
            return true;
    }

    return false;
}


void StereoSlamNode::GetMapKeyFrameStats(
    ORB_SLAM3::Map* pMap,
    size_t& count_out,
    uint64_t& min_kf_id_out,
    uint64_t& max_kf_id_out) const
{
    count_out = 0;
    min_kf_id_out = std::numeric_limits<uint64_t>::max();
    max_kf_id_out = 0;

    if (!pMap)
    {
        min_kf_id_out = 0;
        return;
    }

    auto all_kfs =
        m_SLAM->GetAllKeyFrames();

    for (auto* pKF : all_kfs)
    {
        if (!KeyFrameBelongsToMap(pKF, pMap))
            continue;

        const uint64_t id =
            static_cast<uint64_t>(pKF->mnId);

        min_kf_id_out =
            std::min(min_kf_id_out, id);

        max_kf_id_out =
            std::max(max_kf_id_out, id);

        count_out++;
    }

    if (count_out == 0)
    {
        min_kf_id_out = 0;
        max_kf_id_out = 0;
    }
}

ORB_SLAM3::Map* StereoSlamNode::GetCurrentMapPointerFromKeyFrames()
{
    auto all_kfs =
        m_SLAM->GetAllKeyFrames();

    ORB_SLAM3::KeyFrame* newest_kf = nullptr;
    double newest_time = -1.0;

    for (auto* pKF : all_kfs)
    {
        if (!pKF || pKF->isBad())
            continue;

        if (pKF->mTimeStamp > newest_time)
        {
            newest_time = pKF->mTimeStamp;
            newest_kf = pKF;
        }
    }

    if (!newest_kf)
        return nullptr;

    return newest_kf->GetMap();
}


bool StereoSlamNode::UpdateMapEpochFromCurrentMap()
{
    ORB_SLAM3::Map* map_ptr =
        GetCurrentMapPointerFromKeyFrames();

    if (!map_ptr)
        return false;

    size_t kf_count = 0;
    uint64_t min_kf_id = 0;
    uint64_t max_kf_id = 0;

    GetMapKeyFrameStats(
        map_ptr,
        kf_count,
        min_kf_id,
        max_kf_id);

    if (kf_count == 0)
        return false;

    if (!has_current_orb_map_ptr_)
    {
        current_orb_map_ptr_ = map_ptr;
        has_current_orb_map_ptr_ = true;

        current_map_kf_count_ = kf_count;
        current_map_min_kf_id_ = min_kf_id;
        current_map_max_kf_id_ = max_kf_id;
        has_current_map_signature_ = true;

        RCLCPP_INFO(
            this->get_logger(),
            "[WRAPPER-EPOCH] initial active map detected. epoch=%lu ptr=%p kfs=%zu min_kf=%lu max_kf=%lu",
            map_epoch_,
            static_cast<void*>(current_orb_map_ptr_),
            current_map_kf_count_,
            current_map_min_kf_id_,
            current_map_max_kf_id_);

        return false;
    }

    bool active_map_changed = false;
    std::string reason;

    if (map_ptr != current_orb_map_ptr_)
    {
        active_map_changed = true;
        reason = "map pointer changed";
    }
    else if (has_current_map_signature_)
    {
        // ORB-SLAM3 puede resetear/reiniciar el mapa activo sin que
        // nuestra detección por puntero sea suficiente.
        //
        // Si el mínimo KF del mapa activo salta hacia delante y el número
        // de KFs cae mucho, lo tratamos como submapa nuevo.
        const bool min_kf_jump =
            min_kf_id > current_map_min_kf_id_;

        const bool count_dropped =
            kf_count + 2 < current_map_kf_count_;

        const bool new_first_kf_after_previous_map =
            min_kf_id > current_map_max_kf_id_;

        const bool very_small_new_map =
            kf_count <= 3;

        // Caso importante:
        // ORB-SLAM3 puede resetear el mapa activo y crear un nuevo mapa con el
        // mismo puntero o sin que nuestra detección por puntero sea suficiente.
        // En ese caso, el primer KF del mapa activo salta hacia delante.
        // Si además el mapa activo es pequeño, o su primer KF es posterior al
        // max KF conocido del mapa anterior, lo tratamos como epoch nuevo.
        const bool looks_like_reset_same_pointer =
            min_kf_jump &&
            (count_dropped ||
            new_first_kf_after_previous_map ||
            very_small_new_map);

        if (looks_like_reset_same_pointer)
        {
            active_map_changed = true;
            reason =
                "same map pointer but active map first KF changed/reset";
        }
    }

    if (!active_map_changed)
    {
        current_map_kf_count_ = kf_count;
        current_map_min_kf_id_ = min_kf_id;
        current_map_max_kf_id_ = max_kf_id;
        has_current_map_signature_ = true;

        return false;
    }

    map_epoch_++;

    current_orb_map_ptr_ = map_ptr;

    current_map_kf_count_ = kf_count;
    current_map_min_kf_id_ = min_kf_id;
    current_map_max_kf_id_ = max_kf_id;
    has_current_map_signature_ = true;

    sent_mappoint_hashes_.clear();
    sent_keyframe_hashes_.clear();

    // Resetear secuencia ayuda al servidor a separar claramente el nuevo submapa.
    map_sequence_ = 0;

    RCLCPP_WARN(
        this->get_logger(),
        "[PIPE0-WRAPPER-EPOCH] drone_id=%u new_epoch=%lu reason=%s "
        "ptr=%p kfs=%zu min_kf=%lu max_kf=%lu action=clear_sent_caches_reset_sequence",
        drone_id_,
        map_epoch_,
        reason.c_str(),
        static_cast<void*>(current_orb_map_ptr_),
        current_map_kf_count_,
        current_map_min_kf_id_,
        current_map_max_kf_id_);

    return true;
}


// ============================================================
// Hash combine
// ============================================================

void StereoSlamNode::HashCombine(
    std::size_t& seed,
    std::size_t value)
{
    seed ^=
        value +
        0x9e3779b97f4a7c15ULL +
        (seed << 6) +
        (seed >> 2);
}
