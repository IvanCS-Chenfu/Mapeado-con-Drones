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

/* AÑADIDO */
#include "orbslam3_msgs/msg/orb_map.hpp"
#include "orbslam3_msgs/srv/get_orb_map.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "MapPoint.h"
#include "KeyFrame.h"

#include <unordered_map>
#include <unordered_set>
#include <chrono>
#include <cmath>

#include <set>
#include <tuple>
#include <algorithm>
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

        cv_bridge::CvImageConstPtr cv_ptrLeft;
        cv_bridge::CvImageConstPtr cv_ptrRight;

        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > left_sub;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > right_sub;

        std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy> > syncApproximate;


        /* AÑADIDO */
        rclcpp::Publisher<orbslam3_msgs::msg::OrbMap>::SharedPtr orb_map_delta_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_local_pub_;

        rclcpp::Service<orbslam3_msgs::srv::GetOrbMap>::SharedPtr full_map_service_;

        uint32_t drone_id_;
        std::string drone_name_;
        std::string local_map_frame_;

        uint64_t map_sequence_ = 0;
        uint64_t frame_counter_ = 0;

        int delta_publish_period_frames_ = 10;

        std::unordered_map<uint64_t, std::size_t> sent_mappoint_hashes_;
        std::unordered_map<uint64_t, std::size_t> sent_keyframe_hashes_;

        void PublishLocalPose(const builtin_interfaces::msg::Time& stamp, const Sophus::SE3f& Tcw);

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
