#include "stereo-slam-node.hpp"

#include <opencv2/core/core.hpp>

#include <sstream>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <tuple>

using std::placeholders::_1;
using std::placeholders::_2;


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
    this->declare_parameter<int>("delta_publish_period_frames", 30);
    this->declare_parameter<bool>("debug_architecture_telemetry", false);

    drone_id_ =
        static_cast<uint32_t>(
            this->get_parameter("drone_id").as_int());

    drone_name_ =
        this->get_parameter("drone_name").as_string();

    local_map_frame_ =
        this->get_parameter("local_map_frame").as_string();

    delta_publish_period_frames_ =
        this->get_parameter("delta_publish_period_frames").as_int();

    debug_architecture_telemetry_ =
        this->get_parameter("debug_architecture_telemetry").as_bool();

    if (delta_publish_period_frames_ <= 0)
    {
        delta_publish_period_frames_ = 30;
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

    // ============================================================
    // Publicador de pose local actual de cámara
    // ============================================================

    pose_local_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "orbslam/pose_local",
            rclcpp::QoS(20));

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
        "delta_period_frames=%d rectify=%s camera_valid=%s "
        "fx=%.3f fy=%.3f cx=%.3f cy=%.3f bf=%.3f width=%u height=%u baseline_est_m=%.6f",
        drone_id_,
        drone_name_.c_str(),
        local_map_frame_.c_str(),
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
}


void StereoSlamNode::EmitArchitectureActivity(
    const std::string& edge_id,
    const std::string& interface_name)
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
         << "\",\"interface_kind\":\"topic\""
         << "\",\"source\":\"orbslam3\",\"drone_id\":" << drone_id_
         << ",\"namespace\":\"" << this->get_namespace()
         << "\",\"timestamp\":" << this->get_clock()->now().seconds() << "}";
    message.data = json.str();
    architecture_activity_pub_->publish(message);
}


// ============================================================
// Destructor
// ============================================================

StereoSlamNode::~StereoSlamNode()
{
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

    // ============================================================
    // FASE 4C - Asociación exacta frame -> KeyFrame.
    //
    // Guardamos en variables locales exactamente las imágenes que este
    // callback entrega a System::TrackStereo(). La futura 4D usará
    // imLeftForTracking únicamente si esta llamada crea un KeyFrame.
    // ============================================================
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

    const ORB_SLAM3::System::LastKeyFrameInfo kf_before =
        m_SLAM->GetLastKeyFrameInfo();

    Sophus::SE3f Tcw =
        m_SLAM->TrackStereo(
            imLeftForTracking,
            imRightForTracking,
            input_timestamp);

    // Detectar cambio de mapa una sola vez. Antes se llamaba aquí y de
    // nuevo más abajo, de forma que la primera llamada podía consumir el
    // cambio y hacer que epoch_changed fuese false en la segunda.
    const bool epoch_changed =
        UpdateMapEpochFromCurrentMap();

    const ORB_SLAM3::System::LastKeyFrameInfo kf_after =
        m_SLAM->GetLastKeyFrameInfo();

    const bool keyframe_created =
        kf_after.valid &&
        (!kf_before.valid ||
         kf_after.keyframe_id != kf_before.keyframe_id ||
         kf_after.source_frame_id != kf_before.source_frame_id ||
         std::abs(kf_after.timestamp - kf_before.timestamp) > 1.0e-9);

    if (keyframe_created)
    {
        const double timestamp_delta_sec =
            std::abs(kf_after.timestamp - input_timestamp);

        RCLCPP_WARN(
            this->get_logger(),
            "[KF-EVENT-CREATED] drone_id=%u epoch=%lu keyframe_id=%lu "
            "source_frame_id=%lu event_timestamp=%.9f input_timestamp=%.9f "
            "timestamp_delta_sec=%.9f image_width=%d image_height=%d",
            drone_id_,
            static_cast<unsigned long>(map_epoch_),
            static_cast<unsigned long>(kf_after.keyframe_id),
            static_cast<unsigned long>(kf_after.source_frame_id),
            kf_after.timestamp,
            input_timestamp,
            timestamp_delta_sec,
            imLeftForTracking.cols,
            imLeftForTracking.rows);

        if (timestamp_delta_sec > 1.0e-6)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "[KF-EVENT-TIMESTAMP-MISMATCH] drone_id=%u keyframe_id=%lu "
                "source_frame_id=%lu delta_sec=%.9f",
                drone_id_,
                static_cast<unsigned long>(kf_after.keyframe_id),
                static_cast<unsigned long>(kf_after.source_frame_id),
                timestamp_delta_sec);
        }
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
        m_SLAM->GetTrackingState(),
        input_timestamp);

    // Publicamos pose local de cámara solo si tracking está OK.
    if (m_SLAM->GetTrackingState() == ORB_SLAM3::Tracking::OK)
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

    double t =
        pKF->mTimeStamp;

    int32_t sec =
        static_cast<int32_t>(std::floor(t));

    uint32_t nanosec =
        static_cast<uint32_t>((t - sec) * 1e9);

    kf_msg.stamp.sec = sec;
    kf_msg.stamp.nanosec = nanosec;

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
