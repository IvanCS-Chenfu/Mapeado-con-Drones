#include "orbslam3_server/primary_queue.hpp"
#include "orbslam3_server/ground_truth_buffer.hpp"
#include "orbslam3_server/secondary_queue.hpp"
#include "orbslam3_server/submap_color.hpp"

#include "orbslam3_multi/sparse_global_backend.hpp"
#include "orbslam3_msgs/msg/orb_map.hpp"
#include "orbslam3_msgs/srv/get_orb_map.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <set>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

using namespace std::chrono_literals;

namespace orbslam3_server
{
namespace
{

struct FiducialConfig
{
  int32_t id = 0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double radius_m = 0.0;
};

struct ReplayVisitState
{
  int32_t fiducial_id = 0;
  uint64_t visit_id = 0;
  double last_stamp_sec = 0.0;
};

int64_t StampToNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

double StampToSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

bool PoseToMatrix(const geometry_msgs::msg::Pose & pose, Eigen::Matrix4d * matrix)
{
  const Eigen::Quaterniond quaternion(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
    !std::isfinite(pose.position.z) || !std::isfinite(quaternion.w()) ||
    !std::isfinite(quaternion.x()) || !std::isfinite(quaternion.y()) ||
    !std::isfinite(quaternion.z()) || quaternion.norm() < 1e-9)
  {
    return false;
  }
  *matrix = Eigen::Matrix4d::Identity();
  matrix->block<3, 3>(0, 0) = quaternion.normalized().toRotationMatrix();
  (*matrix)(0, 3) = pose.position.x;
  (*matrix)(1, 3) = pose.position.y;
  (*matrix)(2, 3) = pose.position.z;
  return matrix->allFinite();
}

geometry_msgs::msg::Pose MatrixToPose(const Eigen::Matrix4d & matrix)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = matrix(0, 3);
  pose.position.y = matrix(1, 3);
  pose.position.z = matrix(2, 3);
  const Eigen::Quaterniond quaternion(matrix.block<3, 3>(0, 0));
  const auto normalized = quaternion.normalized();
  pose.orientation.x = normalized.x();
  pose.orientation.y = normalized.y();
  pose.orientation.z = normalized.z();
  pose.orientation.w = normalized.w();
  return pose;
}

float PackRgb(float red, float green, float blue)
{
  const uint32_t packed =
    (static_cast<uint32_t>(std::clamp(red, 0.0F, 1.0F) * 255.0F) << 16U) |
    (static_cast<uint32_t>(std::clamp(green, 0.0F, 1.0F) * 255.0F) << 8U) |
    static_cast<uint32_t>(std::clamp(blue, 0.0F, 1.0F) * 255.0F);
  float value = 0.0F;
  std::memcpy(&value, &packed, sizeof(value));
  return value;
}

float ScoreRgb(float score)
{
  const float normalized = std::clamp(score, 0.0F, 1.0F);
  const float red = normalized <= 0.5F ? 1.0F : 2.0F * (1.0F - normalized);
  const float green = normalized <= 0.5F ? 2.0F * normalized : 1.0F;
  return PackRgb(red, green, 0.0F);
}

sensor_msgs::msg::PointField MakePointField(
  const std::string & name, uint32_t offset, uint8_t datatype)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1;
  return field;
}

template<typename T>
void WritePointField(std::vector<uint8_t> * data, size_t offset, const T & value)
{
  std::memcpy(data->data() + offset, &value, sizeof(T));
}

Eigen::Matrix4d BuildBodyToCamera(
  double x, double y, double z, double roll_deg, double pitch_deg,
  double yaw_deg, bool optical_convention)
{
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  if (optical_convention) {
    Eigen::Matrix3d rotation;
    rotation << 0.0, 0.0, 1.0,
      -1.0, 0.0, 0.0,
      0.0, -1.0, 0.0;
    transform.block<3, 3>(0, 0) = rotation;
  } else {
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    const Eigen::AngleAxisd roll(roll_deg * kDegToRad, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch(pitch_deg * kDegToRad, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw(yaw_deg * kDegToRad, Eigen::Vector3d::UnitZ());
    transform.block<3, 3>(0, 0) = (yaw * pitch * roll).toRotationMatrix();
  }
  transform(0, 3) = x;
  transform(1, 3) = y;
  transform(2, 3) = z;
  return transform;
}

}  // namespace

/// Adaptador ROS y único owner de workers, colas, publicación y backpressure.
/// El principal publica; el secundario solo prepara/compromete cambios y marca dirty.
class GlobalMapServer final : public rclcpp::Node
{
public:
  GlobalMapServer()
  : Node("global_map_server")
  {
    const int64_t drone_count = declare_parameter<int64_t>("drone_count", 2);
    const std::string namespace_base =
      declare_parameter<std::string>("drone_namespace_base", "dron");
    high_watermark_ = static_cast<size_t>(
      declare_parameter<int64_t>("primary_queue_high_watermark", 8));
    low_watermark_ = static_cast<size_t>(
      declare_parameter<int64_t>("primary_queue_low_watermark", 2));
    secondary_high_watermark_ = static_cast<size_t>(
      declare_parameter<int64_t>("secondary_queue_high_watermark", 64));
    secondary_low_watermark_ = static_cast<size_t>(
      declare_parameter<int64_t>("secondary_queue_low_watermark", 16));
    if (secondary_high_watermark_ == 0 ||
      secondary_low_watermark_ >= secondary_high_watermark_)
    {
      throw std::invalid_argument("secondary watermarks invalidos");
    }
    declare_parameter<int64_t>("primary_worker_debug_delay_ms", 0);
    record_enabled_ = declare_parameter<bool>("rawdb_record_enabled", false);
    record_path_ = declare_parameter<std::string>("rawdb_record_path", "");
    replay_path_ = declare_parameter<std::string>("rawdb_replay_path", "");
    replay_entry_delay_ms_ = std::max<int64_t>(
      0, declare_parameter<int64_t>("rawdb_replay_entry_delay_ms", 0));
    full_snapshot_enabled_ = declare_parameter<bool>("full_snapshot_enabled", true);
    full_snapshot_startup_delay_sec_ = std::max(
      0.1, declare_parameter<double>("full_snapshot_startup_delay_sec", 35.0));
    full_snapshot_period_sec_ = std::max(
      5.0, declare_parameter<double>("full_snapshot_period_sec", 35.0));
    debug_drop_one_delta_ = declare_parameter<bool>(
      "debug_drop_one_delta_for_snapshot_test", false);
    debug_drop_drone_id_ = static_cast<uint32_t>(declare_parameter<int64_t>(
        "debug_drop_delta_drone_id", 1));
    debug_anchor_enabled_ = declare_parameter<bool>("pose_store_debug_anchor_enabled", false);
    debug_anchor_drone_id_ = static_cast<uint32_t>(
      declare_parameter<int64_t>("pose_store_debug_anchor_drone_id", 1));
    debug_anchor_map_epoch_ = static_cast<uint64_t>(
      declare_parameter<int64_t>("pose_store_debug_anchor_map_epoch", 0));
    debug_anchor_x_ = declare_parameter<double>("pose_store_debug_anchor_x", 10.0);
    debug_anchor_y_ = declare_parameter<double>("pose_store_debug_anchor_y", 0.0);
    debug_anchor_z_ = declare_parameter<double>("pose_store_debug_anchor_z", 0.0);
    fiducial_sim_enabled_ = declare_parameter<bool>("fiducial_sim_enabled", true);
    fiducial_gt_max_dt_sec_ = declare_parameter<double>("fiducial_gt_max_dt_sec", 1.0);
    orbslam3_multi::FiducialOptimizationConfig optimization_config;
    optimization_config.translation_threshold_m = declare_parameter<double>(
      "fiducial_translation_threshold_m", 0.35);
    optimization_config.rotation_threshold_rad = declare_parameter<double>(
      "fiducial_rotation_threshold_rad", 0.35);
    optimization_config.yaw_threshold_rad = declare_parameter<double>(
      "fiducial_yaw_threshold_rad", 0.25);
    optimization_config.control_vertex_ratio = declare_parameter<double>(
      "pose_graph_control_vertex_ratio", 0.30);
    optimization_config.endpoint_neighborhood_ratio = declare_parameter<double>(
      "pose_graph_endpoint_neighborhood_ratio", 0.20);
    optimization_config.covisibility_min_support = static_cast<uint32_t>(
      declare_parameter<int>("pose_graph_covisibility_min_support", 15));
    optimization_config.max_correction_fraction_per_pass = declare_parameter<double>(
      "fiducial_max_correction_fraction_per_pass", 1.0);
    optimization_config.max_refinement_passes = static_cast<uint32_t>(std::max<int64_t>(
        1, declare_parameter<int64_t>("fiducial_max_refinement_passes", 4)));
    fiducial_optimization_config_ = optimization_config;
    backend_.ConfigureFiducialOptimization(optimization_config);
    orbslam3_multi::LoopPipelineConfig loop_config;
    loop_config.min_query_mappoints = static_cast<size_t>(std::max<int64_t>(
        6, declare_parameter<int64_t>("loop_min_query_mappoints", 12)));
    loop_config.max_bow_candidates = static_cast<size_t>(std::max<int64_t>(
        1, declare_parameter<int64_t>("loop_max_bow_candidates", 10)));
    loop_config.max_candidate_regions = static_cast<size_t>(std::max<int64_t>(
        1, declare_parameter<int64_t>("loop_max_candidate_regions", 3)));
    loop_config.max_candidate_window_keyframes = static_cast<size_t>(std::max<int64_t>(
        1, declare_parameter<int64_t>("loop_max_candidate_window_keyframes", 12)));
    loop_config.max_subcloud_points = static_cast<size_t>(std::max<int64_t>(
        32, declare_parameter<int64_t>("loop_max_subcloud_points", 320)));
    loop_config.strong_covisibility_support = static_cast<uint64_t>(std::max<int64_t>(
        1, declare_parameter<int64_t>("loop_strong_covisibility_support", 15)));
    loop_config.temporal_window_radius = static_cast<uint64_t>(std::max<int64_t>(
        0, declare_parameter<int64_t>("loop_temporal_window_radius", 8)));
    loop_config.min_bow_score = declare_parameter<double>("loop_min_bow_score", 0.01);
    loop_config.max_hamming_distance = static_cast<uint32_t>(std::max<int64_t>(
        1, declare_parameter<int64_t>("loop_max_hamming_distance", 80)));
    loop_config.descriptor_ratio = declare_parameter<double>("loop_descriptor_ratio", 0.90);
    loop_config.min_ransac_matches = static_cast<size_t>(std::max<int64_t>(
        3, declare_parameter<int64_t>("loop_min_ransac_matches", 6)));
    loop_config.min_ransac_inliers = static_cast<size_t>(std::max<int64_t>(
        3, declare_parameter<int64_t>("loop_min_ransac_inliers", 6)));
    loop_config.max_ransac_iterations = static_cast<size_t>(std::max<int64_t>(
        1, declare_parameter<int64_t>("loop_max_ransac_iterations", 80)));
    loop_config.ransac_inlier_threshold_m = declare_parameter<double>(
      "loop_ransac_inlier_threshold_m", 0.30);
    loop_config.min_inlier_ratio = declare_parameter<double>(
      "loop_min_inlier_ratio", 0.25);
    loop_config.max_mean_residual_m = declare_parameter<double>(
      "loop_max_mean_residual_m", 0.20);
    loop_config.max_residual_m = declare_parameter<double>(
      "loop_max_residual_m", 0.75);
    loop_config.fusion_translation_threshold_m = declare_parameter<double>(
      "loop_fusion_translation_threshold_m", 0.35);
    loop_config.fusion_rotation_threshold_rad = declare_parameter<double>(
      "loop_fusion_rotation_threshold_rad", 0.25);
    loop_config.independent_translation_m = declare_parameter<double>(
      "loop_independent_translation_m", 0.20);
    loop_config.independent_yaw_rad = declare_parameter<double>(
      "loop_independent_yaw_rad", 0.0872664626);
    loop_config.hypothesis_translation_tolerance_m = declare_parameter<double>(
      "loop_hypothesis_translation_tolerance_m", 0.50);
    loop_config.hypothesis_rotation_tolerance_rad = declare_parameter<double>(
      "loop_hypothesis_rotation_tolerance_rad", 0.35);
    loop_config.hypothesis_min_support = static_cast<size_t>(std::max<int64_t>(
        2, declare_parameter<int64_t>("loop_hypothesis_min_support", 2)));
    loop_config.ambiguity_margin = static_cast<size_t>(std::max<int64_t>(
        1, declare_parameter<int64_t>("loop_ambiguity_margin", 2)));
    loop_config.structural_temporal_increase_m = declare_parameter<double>(
      "loop_structural_temporal_increase_m", 2.0);
    loop_config.structural_temporal_increase_rad = declare_parameter<double>(
      "loop_structural_temporal_increase_rad", 0.70);
    loop_config.structural_covisibility_increase_m = declare_parameter<double>(
      "loop_structural_covisibility_increase_m", 1.0);
    loop_config.structural_covisibility_increase_rad = declare_parameter<double>(
      "loop_structural_covisibility_increase_rad", 0.50);
    loop_config.structural_prior_loop_increase_m = declare_parameter<double>(
      "loop_structural_prior_increase_m", 0.50);
    loop_config.structural_prior_loop_increase_rad = declare_parameter<double>(
      "loop_structural_prior_increase_rad", 0.35);
    loop_config.hard_corridor_max_translation_m = declare_parameter<double>(
      "loop_hard_corridor_max_translation_m", 5.0);
    loop_config.hard_corridor_max_rotation_rad = declare_parameter<double>(
      "loop_hard_corridor_max_rotation_rad", 0.3490658503988659);
    loop_config.recent_loss_base_translation_m = declare_parameter<double>(
      "loop_recent_loss_base_translation_m", 2.0);
    loop_config.recent_loss_path_drift_ratio = declare_parameter<double>(
      "loop_recent_loss_path_drift_ratio", 0.20);
    loop_config.recent_loss_base_rotation_rad = declare_parameter<double>(
      "loop_recent_loss_base_rotation_rad", 0.35);
    loop_config.recent_loss_rotation_drift_ratio = declare_parameter<double>(
      "loop_recent_loss_rotation_drift_ratio", 0.20);
    backend_.ConfigureLoopPipeline(loop_config);
    orbslam3_multi::FusedLandmarkConfig fused_config;
    fused_config.max_track_dispersion_m = declare_parameter<double>(
      "fusion_max_track_dispersion_m", 0.50);
    fused_config.visibility_depth_tolerance_m = declare_parameter<double>(
      "fusion_visibility_depth_tolerance_m", 0.30);
    fused_config.visibility_cell_size_px = static_cast<uint32_t>(std::max<int64_t>(
        1, declare_parameter<int64_t>("fusion_visibility_cell_size_px", 8)));
    fused_config.inlier_reward = static_cast<float>(declare_parameter<double>(
        "fusion_score_inlier_reward", 0.04));
    fused_config.member_bonus = static_cast<float>(declare_parameter<double>(
        "fusion_score_member_bonus", 0.04));
    backend_.ConfigureFusedLandmarks(fused_config);
    orbslam3_multi::LandmarkScoreConfig score_config;
    score_config.isolation_radius_m = declare_parameter<double>(
      "score_isolation_radius_m", 0.35);
    score_config.isolation_min_neighbors = static_cast<uint32_t>(declare_parameter<int>(
        "score_isolation_min_neighbors", 2));
    score_config.isolation_min_observations = static_cast<uint32_t>(declare_parameter<int>(
        "score_isolation_min_observations", 3));
    score_config.isolation_min_factor = static_cast<float>(declare_parameter<double>(
        "score_isolation_min_factor", 0.35));
    score_config.suspicious_near_distance_m = declare_parameter<double>(
      "score_suspicious_near_distance_m", 1.0);
    score_config.suspicious_near_min_factor = static_cast<float>(declare_parameter<double>(
        "score_suspicious_near_min_factor", 0.05));
    score_config.far_baseline_multiplier = declare_parameter<double>(
      "score_far_baseline_multiplier", 83.33333333333333);
    score_config.far_distance_fallback_m = declare_parameter<double>(
      "score_far_distance_fallback_m", 5.0);
    score_config.far_min_factor = static_cast<float>(declare_parameter<double>(
        "score_far_min_factor", 0.25));
    backend_.ConfigureLandmarkScores(score_config);
    const double body_camera_x = declare_parameter<double>("body_T_camera_x", 0.10);
    const double body_camera_y = declare_parameter<double>("body_T_camera_y", 0.03);
    const double body_camera_z = declare_parameter<double>("body_T_camera_z", 0.03);
    const double body_camera_roll =
      declare_parameter<double>("body_T_camera_roll_deg", 0.0);
    const double body_camera_pitch =
      declare_parameter<double>("body_T_camera_pitch_deg", -90.0);
    const double body_camera_yaw =
      declare_parameter<double>("body_T_camera_yaw_deg", 90.0);
    const bool optical_convention =
      declare_parameter<bool>("use_camera_optical_frame_convention", true);
    body_T_camera_ = BuildBodyToCamera(
      body_camera_x, body_camera_y, body_camera_z, body_camera_roll,
      body_camera_pitch, body_camera_yaw, optical_convention);
    fiducials_ = {
      {
        1, declare_parameter<double>("fiducial_1_x", 0.0),
        declare_parameter<double>("fiducial_1_y", 9.0),
        declare_parameter<double>("fiducial_1_z", 1.0),
        declare_parameter<double>("fiducial_1_radius", 2.0)},
      {
        2, declare_parameter<double>("fiducial_2_x", 0.0),
        declare_parameter<double>("fiducial_2_y", -9.0),
        declare_parameter<double>("fiducial_2_z", 1.0),
        declare_parameter<double>("fiducial_2_radius", 2.0)}};

    if (drone_count <= 0) {
      throw std::invalid_argument("drone_count debe ser positivo");
    }
    if (record_enabled_) {
      if (record_path_.empty()) {
        throw std::invalid_argument("rawdb_record_path no puede estar vacio al grabar");
      }
      std::string error;
      if (!backend_.StartRawRecord(record_path_, &error)) {
        throw std::runtime_error("no se pudo iniciar record incremental: " + error);
      }
      RCLCPP_INFO(
        get_logger(),
        "[F3C-RECORD-STREAM-START] path=%s format=v2 delta_only=true resident=0",
        record_path_.c_str());
    } else {
      backend_.DisableRawJournalRetention();
    }
    backpressure_ = std::make_unique<BackpressureHysteresis>(high_watermark_, low_watermark_);

    rclcpp::QoS flow_qos(rclcpp::KeepLast(256));
    flow_qos.best_effort();
    flow_publisher_ = create_publisher<std_msgs::msg::String>(
      "/global_mapping/flow_events", flow_qos);

    rclcpp::QoS backpressure_qos(rclcpp::KeepLast(1));
    backpressure_qos.reliable().transient_local();
    backpressure_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/global_mapping/backpressure_active", backpressure_qos);
    PublishBackpressure(false, 0, true);

    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.reliable().transient_local();
    sparse_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/global_sparse_cloud", map_qos);
    keyframes_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/global_keyframes", map_qos);

    worker_ = std::thread(&GlobalMapServer::WorkerLoop, this);
    secondary_worker_ = std::thread(&GlobalMapServer::SecondaryWorkerLoop, this);

    if (!replay_path_.empty()) {
      StartReplay();
    } else {
      for (int64_t drone_id = 1; drone_id <= drone_count; ++drone_id) {
        const std::string topic = "/" + namespace_base + "_" +
          std::to_string(drone_id) + "/orbslam/orb_map_delta";
        subscriptions_.push_back(
          create_subscription<orbslam3_msgs::msg::OrbMap>(
            topic, rclcpp::QoS(10).reliable(),
            [this, drone_id, topic](orbslam3_msgs::msg::OrbMap::ConstSharedPtr map) {
              OnDelta(static_cast<uint32_t>(drone_id), topic, std::move(map));
            }));
        RCLCPP_INFO(
          get_logger(), "[F3C-SERVER-SUBSCRIBED] drone_id=%ld topic=%s",
          drone_id, topic.c_str());

        const std::string snapshot_service = "/" + namespace_base + "_" +
          std::to_string(drone_id) + "/orbslam/get_full_map";
        snapshot_services_[static_cast<uint32_t>(drone_id)] = snapshot_service;
        snapshot_clients_[static_cast<uint32_t>(drone_id)] =
          create_client<orbslam3_msgs::srv::GetOrbMap>(snapshot_service);
        snapshot_deferred_[static_cast<uint32_t>(drone_id)] = false;
        RCLCPP_INFO(
          get_logger(),
          "[F3G-SNAPSHOT-CLIENT-READY] drone_id=%ld service=%s",
          drone_id, snapshot_service.c_str());

        const std::string gt_topic = "/" + namespace_base + "_" +
          std::to_string(drone_id) + "/sensor/GT/pose";
        rclcpp::SubscriptionOptions gt_options;
        if (!gt_callback_group_) {
          gt_callback_group_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        }
        gt_options.callback_group = gt_callback_group_;
        gt_subscriptions_.push_back(
          create_subscription<geometry_msgs::msg::PoseStamped>(
            gt_topic, rclcpp::QoS(rclcpp::KeepLast(50)).reliable(),
            [this, drone_id](geometry_msgs::msg::PoseStamped::ConstSharedPtr pose) {
              OnGroundTruth(static_cast<uint32_t>(drone_id), std::move(pose));
            }, gt_options));
        RCLCPP_INFO(
          get_logger(), "[F3E-GT-SUBSCRIBED] drone_id=%ld topic=%s capacity=50",
          drone_id, gt_topic.c_str());
      }

      if (full_snapshot_enabled_) {
        snapshot_startup_timer_ = create_wall_timer(
          std::chrono::duration<double>(full_snapshot_startup_delay_sec_),
          [this]() {
            if (snapshot_startup_timer_) {
              snapshot_startup_timer_->cancel();
            }
            RequestAllSnapshots("startup_resync");
          });
        snapshot_periodic_timer_ = create_wall_timer(
          std::chrono::duration<double>(full_snapshot_period_sec_),
          [this]() {RequestAllSnapshots("periodic");});
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "[F3C-SERVER-INIT] mode=%s high=%zu low=%zu record_enabled=%s",
      replay_path_.empty() ? "live" : "replay", high_watermark_, low_watermark_,
      record_enabled_ ? "true" : "false");
    RCLCPP_WARN(
      get_logger(),
      "[F3E-FID-CONFIG] enabled=%s max_dt=%.3f ring=50 "
      "fid1=(%.2f,%.2f,%.2f,r=%.2f) fid2=(%.2f,%.2f,%.2f,r=%.2f) "
      "body_T_camera=(%.2f,%.2f,%.2f) optical=%s",
      fiducial_sim_enabled_ ? "true" : "false", fiducial_gt_max_dt_sec_,
      fiducials_[0].x, fiducials_[0].y, fiducials_[0].z, fiducials_[0].radius_m,
      fiducials_[1].x, fiducials_[1].y, fiducials_[1].z, fiducials_[1].radius_m,
      body_camera_x, body_camera_y, body_camera_z, optical_convention ? "true" : "false");
    RCLCPP_WARN(
      get_logger(),
      "[F3H-SECONDARY-CONFIG] high=%zu low=%zu priorities=MAX,HIGH,NORMAL "
      "fid_thresholds=(%.3f,%.3f,%.3f) graph_ratio=%.3f neighborhood=%.3f "
      "covis_min_support=%u max_passes=%u",
      secondary_high_watermark_, secondary_low_watermark_,
      optimization_config.translation_threshold_m,
      optimization_config.rotation_threshold_rad,
      optimization_config.yaw_threshold_rad,
      optimization_config.control_vertex_ratio,
      optimization_config.endpoint_neighborhood_ratio,
      optimization_config.covisibility_min_support,
      optimization_config.max_refinement_passes);
  }

  ~GlobalMapServer() override
  {
    shutting_down_.store(true);
    if (snapshot_startup_timer_) {
      snapshot_startup_timer_->cancel();
    }
    if (snapshot_periodic_timer_) {
      snapshot_periodic_timer_->cancel();
    }
    snapshot_clients_.clear();
    subscriptions_.clear();
    gt_subscriptions_.clear();
    primary_queue_.Close();
    if (worker_.joinable()) {
      worker_.join();
    }
    secondary_queue_.Close();
    if (secondary_worker_.joinable()) {
      secondary_worker_.join();
    }

    const auto pose_stats = backend_.GetPoseStats();
    const auto raw_stats = backend_.GetRawStats();
    RCLCPP_INFO(
      get_logger(),
      "[F3E-FID-FINAL] observations=%lu anchors_created=%lu deferred=%lu "
      "anchors=%lu poses=%lu active=%lu hard=%lu journal=%lu",
      fiducial_observations_processed_, fiducial_anchors_created_,
      fiducial_observations_deferred_, pose_stats.anchors, pose_stats.poses,
      pose_stats.active_poses, pose_stats.hard_fiducial_keyframes,
      raw_stats.fiducial_observations);

    if (record_enabled_ && !record_path_.empty()) {
      std::string error;
      const bool saved = backend_.FinalizeRawRecord(&error);
      const auto stats = backend_.GetRawStats();
      const auto storage = backend_.GetRawJournalStorageStats();
      if (saved) {
        RCLCPP_INFO(
          get_logger(),
          "[F3C-RECORD-SAVE] path=%s entries=%lu submaps=%lu kfs=%lu mps=%lu "
          "fiducial_observations=%lu resident_entries=%lu bytes=%lu incremental=true",
          record_path_.c_str(), stats.journal_entries, stats.submaps,
          stats.keyframes, stats.mappoints, stats.fiducial_observations,
          storage.resident_entries, storage.record_bytes_written);
        RCLCPP_INFO(
          get_logger(),
          "[F3E-FID-JOURNAL-SAVE] path=%s observations=%lu success=true",
          record_path_.c_str(), stats.fiducial_observations);
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "[F3C-RECORD-SAVE] path=%s success=false logical_entries=%lu "
          "resident_entries=%lu bytes=%lu error=%s",
          record_path_.c_str(), storage.logical_entries, storage.resident_entries,
          storage.record_bytes_written, error.c_str());
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "[F3C-PRIMARY-SHUTDOWN] pending=%zu processed=%lu active=%u max_active=%u",
      primary_queue_.Pending(), processed_inputs_.load(), active_primary_tasks_.load(),
      max_active_primary_tasks_.load());
    RCLCPP_INFO(
      get_logger(),
      "[F3H-SECONDARY-SHUTDOWN] pending=%zu processed=%lu stale=%lu committed=%lu "
      "hard_failed=%lu max_active=%u",
      secondary_queue_.Pending(), secondary_processed_.load(), secondary_stale_.load(),
      secondary_committed_.load(), secondary_hard_failed_.load(),
      max_active_secondary_tasks_.load());
  }

private:
  void RequestAllSnapshots(const std::string & reason)
  {
    for (const auto & [drone_id, client] : snapshot_clients_) {
      (void)client;
      RequestSnapshot(drone_id, reason);
    }
  }

  void RequestSnapshot(uint32_t drone_id, const std::string & reason)
  {
    if (!full_snapshot_enabled_ || !replay_path_.empty() || shutting_down_.load()) {
      return;
    }
    const auto client_it = snapshot_clients_.find(drone_id);
    if (client_it == snapshot_clients_.end()) {
      return;
    }
    if (backpressure_->Active()) {
      std::lock_guard<std::mutex> lock(snapshot_state_mutex_);
      snapshot_deferred_[drone_id] = true;
      RCLCPP_INFO(
        get_logger(),
        "[F3G-SNAPSHOT-REQUEST-SKIP] drone_id=%u reason=%s cause=backpressure",
        drone_id, reason.c_str());
      return;
    }

    size_t waiting = 0;
    {
      std::lock_guard<std::mutex> lock(snapshot_state_mutex_);
      const bool already_waiting = std::any_of(
        snapshot_waiting_.begin(), snapshot_waiting_.end(),
        [drone_id](const auto & request) {return request.first == drone_id;});
      if (snapshot_global_active_drone_ == drone_id || already_waiting) {
        RCLCPP_INFO(
          get_logger(),
          "[F3G-SNAPSHOT-REQUEST-SKIP] drone_id=%u reason=%s "
          "cause=global_active_or_queued",
          drone_id, reason.c_str());
        return;
      }
      snapshot_waiting_.emplace_back(drone_id, reason);
      snapshot_deferred_[drone_id] = false;
      waiting = snapshot_waiting_.size();
    }

    RCLCPP_INFO(
      get_logger(),
      "[F3G-SNAPSHOT-GLOBAL-QUEUED] drone_id=%u reason=%s waiting=%zu",
      drone_id, reason.c_str(), waiting);
    TryStartNextSnapshot();
  }

  void TryStartNextSnapshot()
  {
    if (!full_snapshot_enabled_ || !replay_path_.empty() || shutting_down_.load() ||
      backpressure_->Active())
    {
      return;
    }

    uint32_t drone_id = 0;
    std::string reason;
    std::string service;
    rclcpp::Client<orbslam3_msgs::srv::GetOrbMap>::SharedPtr client;
    std::vector<std::pair<uint32_t, std::string>> unavailable;
    size_t waiting = 0;

    {
      std::lock_guard<std::mutex> lock(snapshot_state_mutex_);
      if (snapshot_global_active_drone_ != 0U) {
        return;
      }
      while (!snapshot_waiting_.empty()) {
        const auto request = std::move(snapshot_waiting_.front());
        snapshot_waiting_.pop_front();
        const auto client_it = snapshot_clients_.find(request.first);
        if (client_it == snapshot_clients_.end() ||
          !client_it->second->service_is_ready())
        {
          snapshot_deferred_[request.first] = true;
          unavailable.push_back(request);
          continue;
        }
        drone_id = request.first;
        reason = request.second;
        service = snapshot_services_.at(drone_id);
        client = client_it->second;
        snapshot_global_active_drone_ = drone_id;
        waiting = snapshot_waiting_.size();
        break;
      }
    }

    for (const auto & request : unavailable) {
      RCLCPP_WARN(
        get_logger(),
        "[F3G-SNAPSHOT-REQUEST-SKIP] drone_id=%u reason=%s cause=service_unavailable",
        request.first, request.second.c_str());
    }
    if (drone_id == 0U || !client) {
      return;
    }

    RCLCPP_WARN(
      get_logger(),
      "[F3G-SNAPSHOT-REQUEST] drone_id=%u service=%s reason=%s "
      "global_active=true waiting=%zu",
      drone_id, service.c_str(), reason.c_str(), waiting);
    auto request = std::make_shared<orbslam3_msgs::srv::GetOrbMap::Request>();
    try {
      client->async_send_request(
        request,
        [this, drone_id, service](
          rclcpp::Client<orbslam3_msgs::srv::GetOrbMap>::SharedFuture future)
        {
          try {
            const auto response = future.get();
            auto map = std::shared_ptr<const orbslam3_msgs::msg::OrbMap>(
              response, &response->map);
            if (map->drone_id == 0U ||
            (map->keyframes.empty() && map->mappoints.empty()))
            {
              RCLCPP_WARN(
                get_logger(),
                "[F3G-SNAPSHOT-REJECT] drone_id=%u service=%s "
                "reason=empty_or_uninitialized",
                drone_id, service.c_str());
              CompleteSnapshotRequest(drone_id, "empty_or_uninitialized");
              return;
            }
            if (map->drone_id != drone_id) {
              RCLCPP_ERROR(
                get_logger(),
                "[F3G-SNAPSHOT-REJECT] expected_drone=%u payload_drone=%u reason=identity",
                drone_id, map->drone_id);
              CompleteSnapshotRequest(drone_id, "identity_rejected");
              return;
            }

            const auto enqueued = primary_queue_.PushLive(
              map, PrimaryInputKind::FullSnapshot);
            {
              std::lock_guard<std::mutex> lock(snapshot_state_mutex_);
              last_map_sequence_[drone_id] = std::max(
                last_map_sequence_[drone_id], map->map_sequence);
            }
            EmitFlowEvent(
              "wrapper_server_snapshot", "snapshot_received", enqueued.arrival_id,
              "live", map->drone_id, map->map_epoch, enqueued.pending);
            EmitFlowEvent(
              "server_primary_queue", "enqueue_snapshot", enqueued.arrival_id,
              "live", map->drone_id, map->map_epoch, enqueued.pending);
            RCLCPP_WARN(
              get_logger(),
              "[F3G-SNAPSHOT-RX] arrival_id=%lu drone_id=%u epoch=%lu seq=%lu "
              "kfs=%zu mps=%zu pending=%zu zero_copy_rx=true",
              enqueued.arrival_id, map->drone_id, map->map_epoch, map->map_sequence,
              map->keyframes.size(), map->mappoints.size(), enqueued.pending);
            UpdateBackpressure();
            primary_queue_.MarkReady(enqueued.arrival_id);
          } catch (const std::exception & ex) {
            RCLCPP_ERROR(
              get_logger(),
              "[F3G-SNAPSHOT-REJECT] drone_id=%u service=%s reason=%s",
              drone_id, service.c_str(), ex.what());
            CompleteSnapshotRequest(drone_id, "response_error");
          }
        });
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        get_logger(),
        "[F3G-SNAPSHOT-REJECT] drone_id=%u service=%s reason=%s",
        drone_id, service.c_str(), ex.what());
      CompleteSnapshotRequest(drone_id, "request_error");
    }
  }

  void CompleteSnapshotRequest(uint32_t drone_id, const std::string & reason)
  {
    bool released = false;
    size_t waiting = 0;
    {
      std::lock_guard<std::mutex> lock(snapshot_state_mutex_);
      if (snapshot_global_active_drone_ == drone_id) {
        snapshot_global_active_drone_ = 0U;
        released = true;
      }
      waiting = snapshot_waiting_.size();
    }
    if (!released) {
      return;
    }
    RCLCPP_INFO(
      get_logger(),
      "[F3G-SNAPSHOT-GLOBAL-RELEASE] drone_id=%u reason=%s waiting=%zu",
      drone_id, reason.c_str(), waiting);
    TryStartNextSnapshot();
  }

  void OnDelta(
    uint32_t expected_drone_id,
    const std::string & topic,
    orbslam3_msgs::msg::OrbMap::ConstSharedPtr map)
  {
    if (!map || map->drone_id == 0U) {
      RCLCPP_ERROR(get_logger(), "[F3C-PRIMARY-REJECT] reason=invalid_map topic=%s", topic.c_str());
      return;
    }
    if (map->drone_id != expected_drone_id) {
      RCLCPP_WARN(
        get_logger(),
        "[F3C-PRIMARY-IDENTITY-WARN] expected_drone=%u payload_drone=%u topic=%s",
        expected_drone_id, map->drone_id, topic.c_str());
    }

    bool sequence_gap = false;
    uint64_t previous_sequence = 0;
    {
      std::lock_guard<std::mutex> lock(snapshot_state_mutex_);
      const auto previous = last_map_sequence_.find(map->drone_id);
      if (previous != last_map_sequence_.end()) {
        previous_sequence = previous->second;
        sequence_gap = map->map_sequence > previous_sequence + 1U;
      }
      last_map_sequence_[map->drone_id] = std::max(
        last_map_sequence_[map->drone_id], map->map_sequence);
    }

    if (debug_drop_one_delta_ && !debug_delta_dropped_ &&
      map->drone_id == debug_drop_drone_id_)
    {
      debug_delta_dropped_ = true;
      RCLCPP_WARN(
        get_logger(),
        "[F3G-DEBUG-DELTA-DROPPED] drone_id=%u epoch=%lu seq=%lu kfs=%zu mps=%zu",
        map->drone_id, map->map_epoch, map->map_sequence,
        map->keyframes.size(), map->mappoints.size());
      RequestSnapshot(map->drone_id, "debug_dropped_delta");
      return;
    }

    try {
      const auto enqueued = primary_queue_.PushLive(map, PrimaryInputKind::Delta);
      EmitFlowEvent(
        "wrapper_server", "delta_received", enqueued.arrival_id, "live",
        map->drone_id, map->map_epoch, enqueued.pending);
      EmitFlowEvent(
        "server_primary_queue", "enqueue", enqueued.arrival_id, "live",
        map->drone_id, map->map_epoch, enqueued.pending);
      RCLCPP_INFO(
        get_logger(),
        "[F3C-PRIMARY-ENQUEUE] arrival_id=%lu source=live pending=%zu topic=%s",
        enqueued.arrival_id, enqueued.pending, topic.c_str());
      UpdateBackpressure();
      primary_queue_.MarkReady(enqueued.arrival_id);
      if (sequence_gap) {
        RCLCPP_INFO(
          get_logger(),
          "[F3G-SEQUENCE-GAP-HINT] drone_id=%u previous=%lu current=%lu action=request_snapshot",
          map->drone_id, previous_sequence, map->map_sequence);
        RequestSnapshot(map->drone_id, "sequence_gap_hint");
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "[F3C-PRIMARY-REJECT] reason=%s", ex.what());
    }
  }

  void OnGroundTruth(
    uint32_t drone_id,
    geometry_msgs::msg::PoseStamped::ConstSharedPtr pose)
  {
    if (!pose || !fiducial_sim_enabled_) {
      return;
    }
    Eigen::Matrix4d world_T_body;
    if (!PoseToMatrix(pose->pose, &world_T_body)) {
      RCLCPP_WARN(
        get_logger(), "[F3E-GT-BUFFER] drone_id=%u accepted=false reason=invalid_pose",
        drone_id);
      return;
    }

    GroundTruthSample sample;
    sample.stamp_ns = StampToNanoseconds(pose->header.stamp);
    sample.world_T_body = pose->pose;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (const auto & fiducial : fiducials_) {
      const double dx = pose->pose.position.x - fiducial.x;
      const double dy = pose->pose.position.y - fiducial.y;
      const double dz = pose->pose.position.z - fiducial.z;
      const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        sample.distance_to_fiducial_m = distance;
        sample.fiducial_id = distance <= fiducial.radius_m ? fiducial.id : 0;
      }
    }
    const int32_t previous_fiducial = live_fiducial_id_by_drone_[drone_id];
    if (sample.fiducial_id > 0) {
      if (sample.fiducial_id != previous_fiducial) {
        live_visit_id_by_drone_[drone_id] = next_fiducial_visit_id_++;
        RCLCPP_INFO(
          get_logger(),
          "[F3H-FID-VISIT-START] drone_id=%u fid=%d visit=%lu stamp=%.6f",
          drone_id, sample.fiducial_id, live_visit_id_by_drone_[drone_id],
          static_cast<double>(sample.stamp_ns) * 1e-9);
      }
      sample.fiducial_visit_id = live_visit_id_by_drone_[drone_id];
    } else {
      live_visit_id_by_drone_[drone_id] = 0;
    }
    live_fiducial_id_by_drone_[drone_id] = sample.fiducial_id;
    gt_buffer_.Push(drone_id, sample);
    const auto size = gt_buffer_.Size(drone_id);
    const uint64_t samples_received = ++gt_samples_received_[drone_id];
    if (samples_received == 1U || samples_received == GroundTruthBuffer::kCapacityPerDrone ||
      samples_received % 500U == 0U)
    {
      RCLCPP_INFO(
        get_logger(),
        "[F3E-GT-BUFFER] drone_id=%u accepted=true received=%lu size=%zu capacity=50 "
        "stamp=%.6f fid=%d distance=%.3f",
        drone_id, samples_received, size, static_cast<double>(sample.stamp_ns) * 1e-9,
        sample.fiducial_id, sample.distance_to_fiducial_m);
    }
  }

  void StartReplay()
  {
    orbslam3_multi::RawRecordMetadata metadata;
    std::string error;
    if (!orbslam3_multi::RawMapDatabase::ReadRecordMetadata(
        replay_path_, &metadata, &error))
    {
      primary_queue_.Close();
      if (worker_.joinable()) {
        worker_.join();
      }
      secondary_queue_.Close();
      if (secondary_worker_.joinable()) {
        secondary_worker_.join();
      }
      throw std::runtime_error("no se pudo cargar replay: " + error);
    }

    auto replay_observations = metadata.fiducial_observations;
    std::stable_sort(
      replay_observations.begin(), replay_observations.end(),
      [](const auto & lhs, const auto & rhs) {
        return std::tie(
          lhs.keyframe_id.drone_id, lhs.keyframe_id.map_epoch,
          lhs.observation_stamp_sec, lhs.keyframe_id.local_kf_id) <
        std::tie(
          rhs.keyframe_id.drone_id, rhs.keyframe_id.map_epoch,
          rhs.observation_stamp_sec, rhs.keyframe_id.local_kf_id);
      });

    for (auto observation : replay_observations) {
      if (observation.fiducial_visit_id == 0) {
        const orbslam3_multi::RawSubmapId submap_id{
          observation.keyframe_id.drone_id, observation.keyframe_id.map_epoch};
        auto & state = replay_visit_state_[submap_id];
        const bool new_visit = state.visit_id == 0 ||
          state.fiducial_id != observation.fiducial_id ||
          observation.observation_stamp_sec<state.last_stamp_sec ||
            observation.observation_stamp_sec - state.last_stamp_sec>2.0;
        if (new_visit) {
          state.visit_id = next_fiducial_visit_id_++;
        }
        state.fiducial_id = observation.fiducial_id;
        state.last_stamp_sec = observation.observation_stamp_sec;
        observation.fiducial_visit_id = state.visit_id;
      }
      replay_fiducials_by_arrival_[observation.arrival_id].push_back(observation);
    }

    replay_total_ = metadata.entry_count;
    RCLCPP_INFO(
      get_logger(),
      "[F3C-REPLAY-LOAD] path=%s version=%u entries=%lu fiducial_observations=%zu "
      "mode=stream queue_capacity=%zu",
      replay_path_.c_str(), metadata.version, metadata.entry_count,
      metadata.fiducial_observations.size(), high_watermark_);

    uint64_t streamed_entries = 0;
    size_t max_pending = 0;
    const bool streamed = orbslam3_multi::RawMapDatabase::StreamRecordEntries(
      replay_path_,
      [this, &streamed_entries, &max_pending](orbslam3_multi::RawJournalEntry && entry) {
        if (!primary_queue_.WaitUntilPendingBelow(high_watermark_)) {
          return false;
        }
        const uint32_t drone_id = entry.map->drone_id;
        const uint64_t map_epoch = entry.map->map_epoch;
        const uint64_t arrival_id = entry.arrival_id;
        const auto enqueued = primary_queue_.PushReplay(arrival_id, std::move(entry.map));
        max_pending = std::max(max_pending, enqueued.pending);
        ++streamed_entries;
        EmitFlowEvent(
          "server_primary_queue", "enqueue", arrival_id, "replay",
          drone_id, map_epoch, enqueued.pending);
        RCLCPP_INFO(
          get_logger(),
          "[F3C-REPLAY-ENQUEUE] arrival_id=%lu pending=%zu drone_id=%u epoch=%lu",
          arrival_id, enqueued.pending, drone_id, map_epoch);
        UpdateBackpressure();
        primary_queue_.MarkReady(arrival_id);
        if (replay_entry_delay_ms_ > 0) {
          std::this_thread::sleep_for(std::chrono::milliseconds(replay_entry_delay_ms_));
        }
        return true;
      }, &error);
    if (!streamed || streamed_entries != replay_total_) {
      primary_queue_.Close();
      if (worker_.joinable()) {
        worker_.join();
      }
      secondary_queue_.Close();
      if (secondary_worker_.joinable()) {
        secondary_worker_.join();
      }
      throw std::runtime_error(
              "no se pudo reproducir record secuencial: " + error);
    }
    RCLCPP_INFO(
      get_logger(), "[F3C-REPLAY-FEED-DONE] entries=%lu max_pending=%zu capacity=%zu",
      streamed_entries, max_pending, high_watermark_);
  }

  // Preserva el orden arrival_id: ingesta raw, derivados baratos y publicación coherente.
  void WorkerLoop()
  {
    PrimaryInput input;
    while (primary_queue_.WaitPop(&input)) {
      UpdateBackpressure();
      const uint32_t active = active_primary_tasks_.fetch_add(1) + 1;
      uint32_t observed_max = max_active_primary_tasks_.load();
      while (active > observed_max &&
        !max_active_primary_tasks_.compare_exchange_weak(observed_max, active)) {}

      EmitFlowEvent(
        "primary_queue_worker", "dequeue_start", input.arrival_id,
        ToString(input.source), input.map->drone_id, input.map->map_epoch,
        primary_queue_.Pending());
      RCLCPP_INFO(
        get_logger(),
        "[F3C-PRIMARY-START] arrival_id=%lu source=%s kind=%s active=%u pending=%zu",
        input.arrival_id, ToString(input.source), ToString(input.kind), active,
        primary_queue_.Pending());

      bool success = false;
      try {
        const int64_t delay_ms = std::max<int64_t>(
          0, get_parameter("primary_worker_debug_delay_ms").as_int());
        if (delay_ms > 0) {
          std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        }

        const auto result = input.kind == PrimaryInputKind::FullSnapshot ?
          backend_.InsertFullSnapshot(input.arrival_id, input.map) :
          backend_.InsertDelta(input.arrival_id, input.map);
        const auto & raw = result.raw_result;
        const auto journal_storage = backend_.GetRawJournalStorageStats();
        EmitFlowEvent(
          input.kind == PrimaryInputKind::FullSnapshot ?
          "primary_worker_raw_db_snapshot" : "primary_worker_raw_db",
          input.kind == PrimaryInputKind::FullSnapshot ? "full_commit" : "raw_commit",
          input.arrival_id,
          ToString(input.source), input.map->drone_id, input.map->map_epoch,
          primary_queue_.Pending());
        RCLCPP_INFO(
          get_logger(),
          "[F3C-RAW-COMMIT] arrival_id=%lu source=%s submap=(%u,%lu) "
          "revision=%lu material=%s new_kfs=%zu updated_kfs=%zu new_mps=%zu "
          "updated_mps=%zu journal=%lu resident_journal=%lu record_bytes=%lu "
          "submaps=%lu kfs=%lu mps=%lu",
          input.arrival_id, ToString(input.source), raw.submap_id.drone_id,
          raw.submap_id.map_epoch, raw.submap_revision,
          raw.has_material_changes ? "true" : "false",
          raw.new_keyframe_ids.size(), raw.updated_keyframe_ids.size(),
          raw.new_mappoint_ids.size(), raw.updated_mappoint_ids.size(),
          raw.stats.journal_entries, journal_storage.resident_entries,
          journal_storage.record_bytes_written, raw.stats.submaps,
          raw.stats.keyframes, raw.stats.mappoints);

        if (input.kind == PrimaryInputKind::FullSnapshot) {
          RCLCPP_WARN(
            get_logger(),
            "[F3G-SNAPSHOT-DIFF] arrival_id=%lu submap=(%u,%lu) material=%s "
            "pose_kfs=%zu assoc_kfs=%zu geometry_mps=%zu score_mps=%zu "
            "invalid_kfs=%zu invalid_mps=%zu normalized_record=%s",
            input.arrival_id, raw.submap_id.drone_id, raw.submap_id.map_epoch,
            raw.has_material_changes ? "true" : "false",
            raw.pose_changed_keyframe_ids.size(),
            raw.association_changed_keyframe_ids.size(),
            raw.geometry_changed_mappoint_ids.size(),
            raw.score_input_changed_mappoint_ids.size(),
            raw.invalidated_keyframe_ids.size(), raw.invalidated_mappoint_ids.size(),
            raw.normalized_delta_appended ? "true" : "false");
        }

        const auto & scores = result.score_changes;
        if (scores.HasStoreChanges()) {
          std::ostringstream score_detail;
          score_detail << " created=" << scores.created_ids.size()
                       << " updated=" << scores.updated_ids.size()
                       << " invalidated=" << scores.invalidated_ids.size()
                       << " input_updated=" << scores.input_updated_ids.size()
                       << " score_revision=" << scores.score_revision_after;
          EmitFlowEvent(
            "raw_db_landmark_score_manager", "score_raw_commit", input.arrival_id,
            ToString(input.source), raw.submap_id.drone_id, raw.submap_id.map_epoch,
            primary_queue_.Pending(), score_detail.str());
        }
        if (scores.HasChanges()) {
          std::ostringstream score_detail;
          score_detail << " created=" << scores.created_ids.size()
                       << " updated=" << scores.updated_ids.size()
                       << " invalidated=" << scores.invalidated_ids.size()
                       << " input_updated=" << scores.input_updated_ids.size()
                       << " score_revision=" << scores.score_revision_after;
          EmitFlowEvent(
            "landmark_score_manager_global_map_builder", "score_dirty", input.arrival_id,
            ToString(input.source), raw.submap_id.drone_id, raw.submap_id.map_epoch,
            primary_queue_.Pending(), score_detail.str());
          RCLCPP_INFO(
            get_logger(),
            "[F3R-RAW-SCORE-COMMIT] arrival_id=%lu created=%zu updated=%zu "
            "invalidated=%zu input_updated=%zu fused_updated=%zu revision=%lu",
            input.arrival_id, scores.created_ids.size(), scores.updated_ids.size(),
            scores.invalidated_ids.size(), scores.input_updated_ids.size(),
            scores.fused_created_ids.size() + scores.fused_updated_ids.size(),
            scores.score_revision_after);
        } else if (!scores.input_updated_ids.empty()) {
          RCLCPP_INFO(
            get_logger(),
            "[F3R-SCORE-NOOP] arrival_id=%lu input_updated=%zu revision=%lu "
            "builder_dirty=0",
            input.arrival_id, scores.input_updated_ids.size(),
            scores.score_revision_after);
        }
        if (input.arrival_id % 25U == 0U) {
          const auto score_stats = backend_.GetScoreStats();
          RCLCPP_INFO(
            get_logger(),
            "[F3R-SCORE-STATS] revision=%lu tracked=%lu bad=%lu anchored=%lu "
            "isolated=%lu near=%lu far=%lu min=%.4f mean=%.4f max=%.4f",
            score_stats.score_revision, score_stats.tracked_points,
            score_stats.bad_points, score_stats.anchored_points,
            score_stats.isolated_points, score_stats.suspicious_near_points,
            score_stats.far_points, score_stats.score_min, score_stats.score_mean,
            score_stats.score_max);
        }

        if (result.pose_stage_executed) {
          const auto & poses = result.pose_changes;
          std::ostringstream detail;
          detail << " status=" << orbslam3_multi::ToString(poses.status)
                 << " pose_inputs=" << raw.pose_changes.size()
                 << " created=" << poses.created_ids.size()
                 << " updated=" << poses.updated_ids.size()
                 << " invalidated=" << poses.invalidated_ids.size()
                 << " control_propagated=" << poses.control_propagated_ids.size()
                 << " skipped=" << poses.skipped_unanchored_ids.size()
                 << " pose_revision=" << poses.store_revision_after;
          EmitFlowEvent(
            input.kind == PrimaryInputKind::FullSnapshot ?
            "raw_db_global_pose_store_snapshot" : "raw_db_global_pose_store",
            input.kind == PrimaryInputKind::FullSnapshot ?
            "snapshot_reconcile" : "pose_changes", input.arrival_id,
            ToString(input.source), raw.submap_id.drone_id, raw.submap_id.map_epoch,
            primary_queue_.Pending(), detail.str());
          if (poses.status == orbslam3_multi::PoseCommitStatus::Applied) {
            EmitFlowEvent(
              "global_pose_store_global_map_builder", "pose_dirty", input.arrival_id,
              ToString(input.source), raw.submap_id.drone_id, raw.submap_id.map_epoch,
              primary_queue_.Pending(), detail.str());
          }
          RCLCPP_INFO(
            get_logger(),
            "[F3D-POSE-STAGE] arrival_id=%lu source=%s submap=(%u,%lu) "
            "status=%s inputs=%zu created=%zu updated=%zu invalidated=%zu "
            "preserved=%zu control_propagated=%zu skipped_unanchored=%zu "
            "commit=%lu revision=%lu",
            input.arrival_id, ToString(input.source), raw.submap_id.drone_id,
            raw.submap_id.map_epoch, orbslam3_multi::ToString(poses.status),
            raw.pose_changes.size(), poses.created_ids.size(), poses.updated_ids.size(),
            poses.invalidated_ids.size(), poses.preserved_ids.size(),
            poses.control_propagated_ids.size(), poses.skipped_unanchored_ids.size(),
            poses.commit_id,
            poses.store_revision_after);
          if (!poses.control_propagated_ids.empty()) {
            RCLCPP_WARN(
              get_logger(),
              "[F3K-FUTURE-KF-PROPAGATE] arrival_id=%lu submap=(%u,%lu) count=%zu",
              input.arrival_id, raw.submap_id.drone_id, raw.submap_id.map_epoch,
              poses.control_propagated_ids.size());
          }
          if (input.kind == PrimaryInputKind::FullSnapshot &&
            !poses.preserved_ids.empty())
          {
            RCLCPP_WARN(
              get_logger(),
              "[F3G-POSE-WORLD-PRESERVED] arrival_id=%lu count=%zu pose_revision=%lu",
              input.arrival_id, poses.preserved_ids.size(), poses.store_revision_after);
          }
        } else {
          RCLCPP_INFO(
            get_logger(),
            "[F3D-POSE-SKIP] arrival_id=%lu source=%s submap=(%u,%lu) "
            "reason=no_pose_relevant_kf_changes",
            input.arrival_id, ToString(input.source), raw.submap_id.drone_id,
            raw.submap_id.map_epoch);
        }

        EnqueueSecondaryWork(raw, ToString(input.source));

        if (input.kind == PrimaryInputKind::FullSnapshot) {
          if (!raw.has_material_changes) {
            RCLCPP_INFO(
              get_logger(),
              "[F3G-SNAPSHOT-NOOP] arrival_id=%lu submap=(%u,%lu) action=end",
              input.arrival_id, raw.submap_id.drone_id, raw.submap_id.map_epoch);
          } else {
            RCLCPP_WARN(
              get_logger(),
              "[F3G-SNAPSHOT-DIRTY-DEFERRED] arrival_id=%lu kfs=%zu mps=%zu "
              "builder_executed=false publish=false",
              input.arrival_id,
              raw.pose_changed_keyframe_ids.size() +
              raw.association_changed_keyframe_ids.size() +
              raw.invalidated_keyframe_ids.size(),
              raw.geometry_changed_mappoint_ids.size() +
              raw.score_input_changed_mappoint_ids.size() +
              raw.invalidated_mappoint_ids.size());
            if (raw.normalized_delta_appended) {
              RCLCPP_INFO(
                get_logger(),
                "[F3G-NORMALIZED-DELTA-RECORD] arrival_id=%lu journal=%lu "
                "kfs=%zu mps=%zu",
                input.arrival_id, raw.stats.journal_entries,
                raw.new_keyframe_ids.size() + raw.updated_keyframe_ids.size(),
                raw.new_mappoint_ids.size() + raw.updated_mappoint_ids.size());
            }
          }
        } else {
          if (input.source == PrimaryInputSource::Replay) {
            ProcessReplayFiducials(input.arrival_id, raw);
          } else {
            ProcessLiveFiducials(input.arrival_id, raw);
          }
          MaybeCommitSyntheticAnchor(input.arrival_id, ToString(input.source), raw);
          BuildAndPublishGlobalMap(input.arrival_id, ToString(input.source), raw);
          if (result.had_deferred_snapshot_dirty) {
            RCLCPP_WARN(
              get_logger(),
              "[F3G-DIRTY-CONSUMED-BY-DELTA] arrival_id=%lu source=%s",
              input.arrival_id, ToString(input.source));
          }
        }
        success = true;
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(
          get_logger(), "[F3C-PRIMARY-ERROR] arrival_id=%lu source=%s error=%s",
          input.arrival_id, ToString(input.source), ex.what());
      }

      active_primary_tasks_.fetch_sub(1);
      const uint64_t processed = processed_inputs_.fetch_add(1) + 1;
      RCLCPP_INFO(
        get_logger(),
        "[F3C-PRIMARY-END] arrival_id=%lu source=%s kind=%s success=%s active=0 pending=%zu",
        input.arrival_id, ToString(input.source), ToString(input.kind),
        success ? "true" : "false", primary_queue_.Pending());

      if (input.source == PrimaryInputSource::Live &&
        input.kind == PrimaryInputKind::FullSnapshot)
      {
        const uint32_t completed_drone_id = input.map->drone_id;
        input.map.reset();
        CompleteSnapshotRequest(
          completed_drone_id, success ? "commit" : "worker_error");
      }

      if (input.source == PrimaryInputSource::Replay && processed == replay_total_) {
        const auto stats = backend_.GetRawStats();
        const auto pose_stats = backend_.GetPoseStats();
        RCLCPP_INFO(
          get_logger(),
          "[F3C-REPLAY-DONE] entries=%lu journal=%lu submaps=%lu kfs=%lu mps=%lu "
          "max_active=%u",
          processed, stats.journal_entries, stats.submaps, stats.keyframes,
          stats.mappoints, max_active_primary_tasks_.load());
        RCLCPP_INFO(
          get_logger(),
          "[F3D-REPLAY-POSE-DONE] anchors=%lu poses=%lu active=%lu inactive=%lu "
          "commits=%lu revision=%lu debug_anchor=%s",
          pose_stats.anchors, pose_stats.poses, pose_stats.active_poses,
          pose_stats.inactive_poses, pose_stats.commits, pose_stats.store_revision,
          debug_anchor_committed_ ? "true" : "false");
        RCLCPP_INFO(
          get_logger(),
          "[F3E-REPLAY-FID-DONE] observations=%lu anchors=%lu deferred=%lu "
          "hard=%lu poses=%lu gt_live_subscriptions=0",
          fiducial_observations_processed_, fiducial_anchors_created_,
          fiducial_observations_deferred_, pose_stats.hard_fiducial_keyframes,
          pose_stats.poses);
        const auto score_stats = backend_.GetScoreStats();
        RCLCPP_INFO(
          get_logger(),
          "[F3R-SCORE-STATS] revision=%lu tracked=%lu bad=%lu anchored=%lu "
          "isolated=%lu near=%lu far=%lu min=%.4f mean=%.4f max=%.4f",
          score_stats.score_revision, score_stats.tracked_points, score_stats.bad_points,
          score_stats.anchored_points, score_stats.isolated_points,
          score_stats.suspicious_near_points, score_stats.far_points,
          score_stats.score_min, score_stats.score_mean, score_stats.score_max);
      }
    }
  }

  // Ejecuta una tarea cada vez. La barrera por tarea convierte excepciones en fallo bloqueante.
  void SecondaryWorkerLoop()
  {
    SecondaryTask queued;
    while (secondary_queue_.WaitPop(&queued)) {
      const uint32_t active = active_secondary_tasks_.fetch_add(1) + 1;
      uint32_t previous_max = max_active_secondary_tasks_.load();
      while (active > previous_max &&
        !max_active_secondary_tasks_.compare_exchange_weak(previous_max, active)) {}

      EmitSecondaryLifecycleEvent(
        "start", queued, secondary_queue_.Pending(), "");
      EmitSecondaryFlowEvent(
        "secondary_queue_secondary_worker", "dequeue_start", queued,
        secondary_queue_.Pending(), " priority=" + std::string(ToString(queued.priority)));
      RCLCPP_WARN(
        get_logger(),
        "[F3H-SECONDARY-START] task=%lu kind=%s priority=%s sequence=%lu "
        "intent=%s pending=%zu",
        queued.task_id, ToString(queued.kind), ToString(queued.priority),
        queued.enqueue_sequence,
        queued.loop.has_value() ? orbslam3_multi::ToString(queued.loop->intent) : "n/a",
        secondary_queue_.Pending());

      bool full_success = false;
      bool stale = false;
      bool hard_failure = false;
      std::string final_reason;
      std::optional<orbslam3_multi::RawKeyFrameId> fusion_retry_keyframe;
      uint64_t fusion_retry_arrival = 0;
      std::string fusion_retry_cause;
      std::optional<orbslam3_multi::FiducialOptimizationTask> fiducial_retry_task;
      std::vector<orbslam3_multi::RawKeyFrameId> post_optimization_rerun_ids;
      uint64_t post_optimization_arrival = 0U;
      try {
        if (queued.kind == SecondaryTaskKind::DatabaseUpdate &&
          queued.database_update.has_value())
        {
          EmitSecondaryFlowEvent(
            "secondary_worker_covisibility_database", "database_update", queued,
            secondary_queue_.Pending(), "");
          const auto update = backend_.ProcessDatabaseUpdate(*queued.database_update);
          stale = update.stale;
          full_success = update.committed;
          final_reason = update.reason;
          RCLCPP_WARN(
            get_logger(),
            "[F3M-DATABASE-UPDATE] task=%lu arrival=%lu submap=(%u,%lu) "
            "committed=%s stale=%s added=%zu updated=%zu removed=%zu unchanged=%zu "
            "revision=%lu reason=%s",
            queued.task_id, queued.database_update->source_arrival_id,
            queued.database_update->submap_id.drone_id,
            queued.database_update->submap_id.map_epoch,
            update.committed ? "true" : "false", update.stale ? "true" : "false",
            update.added, update.updated, update.removed, update.unchanged,
            update.revision_after, update.reason.c_str());
          if (update.committed) {
            auto loops = backend_.CreateLoopTasks(
              queued.database_update->source_arrival_id,
              queued.database_update->loop_keyframe_ids);
            const size_t enqueued = EnqueueLoopTasks(
              &loops, "database_update_complete");
            EmitSecondaryFlowEvent(
              "covisibility_database_secondary_queue", "loop_enqueue", queued,
              secondary_queue_.Pending(), " loops=" + std::to_string(enqueued));
          }
        } else if (queued.kind == SecondaryTaskKind::Loop && queued.loop.has_value()) {
          EmitSecondaryFlowEvent(
            "secondary_worker_loop_detector", "bow_search", queued,
            secondary_queue_.Pending(), "");
          auto loop = backend_.ProcessLoopTask(*queued.loop);
          if (loop.decision ==
            orbslam3_multi::LoopTaskDecisionKind::OptimizationEvidence)
          {
            optimization_active_.store(true);
            UpdateBackpressure();
            EmitSecondaryFlowEvent(
              "loop_decision_pose_graph_builder", "opt_loop", queued,
              secondary_queue_.Pending(),
              " evidence=" + std::to_string(loop.geometry_results.size()));
            RCLCPP_WARN(
              get_logger(),
              "[F3Q-OPT-START] task=%lu query=(%u,%lu,%lu) priority=NORMAL "
              "intent=%s stop_drones=true geometry=%zu selected_regions=%zu",
              queued.task_id, queued.loop->query_keyframe_id.drone_id,
              queued.loop->query_keyframe_id.map_epoch,
              queued.loop->query_keyframe_id.local_kf_id,
              orbslam3_multi::ToString(queued.loop->intent),
              loop.geometry_results.size(), loop.optimization_geometry_indices.size());
            loop = backend_.ProcessLoopOptimization(std::move(loop));
            if (loop.optimization.committed) {
              post_optimization_rerun_ids = loop.rerun_keyframe_ids;
              post_optimization_arrival = queued.loop->source_arrival_id;
            }
            optimization_active_.store(false);
            UpdateBackpressure();
          }
          stale = loop.decision == orbslam3_multi::LoopTaskDecisionKind::Stale;
          hard_failure = loop.decision == orbslam3_multi::LoopTaskDecisionKind::Error;
          full_success = !stale && !hard_failure;
          final_reason = loop.reason;
          if (stale && (loop.fusion.attempted || loop.optimization.attempted)) {
            fusion_retry_keyframe = queued.loop->query_keyframe_id;
            fusion_retry_arrival = queued.loop->source_arrival_id;
            fusion_retry_cause = loop.optimization.attempted ?
              "loop_optimization_stale_retry" :
              (loop.fusion.rolled_back ?
              "fusion_rollback_retry" : "fusion_stale_retry");
          }
          EmitSecondaryFlowEvent(
            "loop_detector_loop_bow_index", "upsert_search", queued,
            secondary_queue_.Pending(),
            " candidates=" + std::to_string(loop.bow_candidates));
          EmitSecondaryFlowEvent(
            "loop_bow_index_subcloud_loop_verifier", "geometry", queued,
            secondary_queue_.Pending(),
            " regions=" + std::to_string(loop.regions.size()) +
            " results=" + std::to_string(loop.geometry_results.size()));
          EmitSecondaryFlowEvent(
            "subcloud_loop_verifier_loop_decision", "decision", queued,
            secondary_queue_.Pending(),
            " result=" + std::string(orbslam3_multi::ToString(loop.decision)));
          if (loop.optimization.attempted) {
            EmitSecondaryFlowEvent(
              "pose_graph_builder_optimization_manager", "solve_loop", queued,
              secondary_queue_.Pending(),
              " controls=" + std::to_string(loop.optimization.controls) +
              " iterations=" + std::to_string(loop.optimization.iterations));
            EmitSecondaryFlowEvent(
              "optimization_manager_validation", "validate_loop", queued,
              secondary_queue_.Pending(),
              " accepted=" + std::string(
                loop.optimization.accepted ? "true" : "false"));
            if (loop.optimization.committed) {
              EmitSecondaryFlowEvent(
                "validation_global_pose_store", "commit_loop", queued,
                secondary_queue_.Pending(),
                " moved=" + std::to_string(loop.optimization.moved_keyframes));
              EmitSecondaryFlowEvent(
                "global_pose_store_global_map_builder", "pose_dirty", queued,
                secondary_queue_.Pending(),
                " kfs=" + std::to_string(loop.optimization.moved_keyframes));
            }
            RCLCPP_WARN(
              get_logger(),
              "[F3Q-LOOP-OPT] task=%lu graph=%s optimized=%s accepted=%s committed=%s "
              "stale=%s submaps=%zu window=%zu controls=%zu edges=(temporal=%zu,covis=%zu,loop=%zu) "
              "iterations=%zu error_before=(%.6f,%.6f) error_after=(%.6f,%.6f) "
              "structure=(edges=%zu,corridor_kfs=%zu,max_increase=%.6fm/%.6frad,"
              "corridor_excess=%.6fm/%.6frad->%.6fm/%.6frad) "
              "cost=(%.6f->%.6f) time_ms=(graph=%.3f,solve=%.3f,validation=%.3f,commit=%.3f) "
              "rebuilds=%zu discarded_regions=%zu moved=%zu propagated=%zu "
              "rebased_skipped=%zu rebased_inactive=%zu fusion_after=%s reason=%s",
              queued.task_id, loop.optimization.graph_built ? "true" : "false",
              loop.optimization.optimized ? "true" : "false",
              loop.optimization.accepted ? "true" : "false",
              loop.optimization.committed ? "true" : "false",
              loop.optimization.stale ? "true" : "false",
              loop.optimization.submaps, loop.optimization.window_keyframes,
              loop.optimization.controls, loop.optimization.temporal_edges,
              loop.optimization.covisibility_edges, loop.optimization.loop_edges,
              loop.optimization.iterations,
              loop.optimization.initial_translation_error_m,
              loop.optimization.initial_rotation_error_rad,
              loop.optimization.final_translation_error_m,
              loop.optimization.final_rotation_error_rad,
              loop.optimization.structural_edges_checked,
              loop.optimization.hard_corridor_keyframes_checked,
              loop.optimization.max_structural_translation_increase_m,
              loop.optimization.max_structural_rotation_increase_rad,
              loop.optimization.max_corridor_translation_excess_before_m,
              loop.optimization.max_corridor_rotation_excess_before_rad,
              loop.optimization.max_corridor_translation_excess_after_m,
              loop.optimization.max_corridor_rotation_excess_after_rad,
              loop.optimization.initial_cost, loop.optimization.final_cost,
              loop.optimization.graph_ms, loop.optimization.solve_ms,
              loop.optimization.validation_ms, loop.optimization.commit_ms,
              loop.optimization.rebuilds,
              loop.optimization.discarded_loop_regions,
              loop.optimization.moved_keyframes,
              loop.optimization.propagated_keyframes,
              loop.optimization.rebased_skipped_controls,
              loop.optimization.rebased_inactive_controls,
              loop.optimization.fusion_after_optimization ? "true" : "false",
              loop.optimization.reason.c_str());
            RCLCPP_WARN(
              get_logger(),
              "[F3Q-OPT-END] task=%lu stop_drones=false committed=%s reason=%s",
              queued.task_id, loop.optimization.committed ? "true" : "false",
              loop.reason.c_str());
          }
          if (loop.fusion.attempted) {
            EmitSecondaryFlowEvent(
              loop.optimization.committed ?
              "validation_fused_landmark_manager" :
              "loop_decision_fused_landmark_manager",
              loop.optimization.committed ? "fusion_after_opt" : "fusion_prepare", queued,
              secondary_queue_.Pending(),
              " pairs=" + std::to_string(loop.fusion.pair_results));
          }
          if (loop.fusion.committed) {
            EmitSecondaryFlowEvent(
              "fused_landmark_manager_covisibility_database", "server_covisibility", queued,
              secondary_queue_.Pending(),
              " added=" + std::to_string(loop.fusion.covisibility_added));
            EmitSecondaryFlowEvent(
              "fused_landmark_manager_landmark_score_manager", "score_evidence", queued,
              secondary_queue_.Pending(),
              " positive=" + std::to_string(loop.fusion.score_positive_events) +
              " negative=" + std::to_string(loop.fusion.score_negative_events) +
              " visibility_diagnostics=" +
              std::to_string(loop.fusion.score_visibility_diagnostics));
            EmitSecondaryFlowEvent(
              "fused_landmark_manager_global_map_builder", "fusion_dirty", queued,
              secondary_queue_.Pending(),
              " hidden=" + std::to_string(loop.fusion.hidden_raw_members));
          }
          if (loop.anchor_commit.status == orbslam3_multi::PoseCommitStatus::Applied) {
            EmitSecondaryFlowEvent(
              "loop_decision_loop_anchor_store", "anchor_evidence", queued,
              secondary_queue_.Pending(),
              " submaps=" + std::to_string(loop.anchor_commit.anchored_submaps.size()));
            EmitSecondaryFlowEvent(
              "loop_anchor_store_global_pose_store", "anchor_commit", queued,
              secondary_queue_.Pending(),
              " submaps=" + std::to_string(loop.anchor_commit.anchored_submaps.size()) +
              " kfs=" + std::to_string(loop.anchor_commit.dirty_keyframe_ids.size()));
            EmitSecondaryFlowEvent(
              "global_pose_store_global_map_builder", "pose_dirty", queued,
              secondary_queue_.Pending(),
              " kfs=" + std::to_string(loop.anchor_commit.dirty_keyframe_ids.size()));
            auto reruns = backend_.CreateLoopTasks(
              queued.loop->source_arrival_id, loop.rerun_keyframe_ids);
            EnqueueLoopTasks(&reruns, "anchor_revision_changed");
          }
          RCLCPP_WARN(
            get_logger(),
            "[F3O-LOOP-DONE] task=%lu query=(%u,%lu,%lu) decision=%s fast=%s bow=%zu "
            "regions=%zu geometry=%zu opt_regions=%zu fusion_pairs=%zu anchors=%zu dirty=%zu "
            "support=(observed=%s,compatible=%s,independent=%s,count=%zu/%zu,competing=%zu,"
            "ambiguity=%s,margin=%zu,separation=%.3fm/%.3frad,accepted=%s) "
            "recent_loss=(checked=%s,passed=%s,error=%.3fm/%.3frad,limit=%.3fm/%.3frad) "
            "protected=(checked=%s,query=%s,candidate=%s,rejected=%s,ledger=%s,"
            "error=%.3fm/%.3frad) refresh_spatial=(accepted=%zu,rejected=%zu) "
            "reason=%s",
            queued.task_id, queued.loop->query_keyframe_id.drone_id,
            queued.loop->query_keyframe_id.map_epoch,
            queued.loop->query_keyframe_id.local_kf_id,
            orbslam3_multi::ToString(loop.decision),
            loop.used_fast_overlap ? "true" : "false", loop.bow_candidates,
            loop.regions.size(), loop.geometry_results.size(),
            loop.optimization_geometry_indices.size(), loop.fusion_pairs.size(),
            loop.anchor_commit.anchored_submaps.size(),
            loop.anchor_commit.dirty_keyframe_ids.size(),
            loop.hypothesis_support.observed ? "true" : "false",
            loop.hypothesis_support.compatible_hypothesis ? "true" : "false",
            loop.hypothesis_support.independent ? "true" : "false",
            loop.hypothesis_support.support,
            loop.hypothesis_support.required_support,
            loop.hypothesis_support.competing_support,
            loop.hypothesis_support.ambiguity_satisfied ? "true" : "false",
            loop.hypothesis_support.ambiguity_margin,
            loop.hypothesis_support.nearest_translation_separation_m,
            loop.hypothesis_support.nearest_yaw_separation_rad,
            loop.hypothesis_support.accepted ? "true" : "false",
            loop.recent_loss_gate_checked ? "true" : "false",
            loop.recent_loss_gate_passed ? "true" : "false",
            loop.recent_loss_translation_m,
            loop.recent_loss_rotation_rad,
            loop.recent_loss_translation_limit_m,
            loop.recent_loss_rotation_limit_rad,
            loop.protected_region_checked ? "true" : "false",
            loop.protected_query_stable ? "true" : "false",
            loop.protected_candidate_stable ? "true" : "false",
            loop.protected_region_rejected ? "true" : "false",
            loop.rejection_ledger_hit ? "true" : "false",
            loop.protected_translation_error_m,
            loop.protected_rotation_error_rad,
            loop.refresh_spatial_candidates,
            loop.refresh_spatial_rejected,
            loop.reason.c_str());
          if (loop.fusion.attempted) {
            RCLCPP_INFO(
              get_logger(),
              "[F3R-FUSED-SCORE-COMMIT] task=%lu committed=%s positive=%zu "
              "negative=%zu visibility_diagnostics=%zu dirty=%zu",
              queued.task_id, loop.fusion.committed ? "true" : "false",
              loop.fusion.score_positive_events, loop.fusion.score_negative_events,
              loop.fusion.score_visibility_diagnostics,
              loop.fusion.score_raw_updates);
            RCLCPP_WARN(
              get_logger(),
              "[F3P-FUSION] task=%lu prepared=%s committed=%s stale=%s rollback=%s "
              "pairs=%zu tracks=(%zu,%zu,%zu) hidden=%zu score=(+%zu,-%zu,dirty=%zu) "
              "covis=(+%zu,~%zu) visibility=(regions=%zu/%zu,projected=%zu,time_ms=%.3f) "
              "timing_ms=(prepare=%.3f,commit=%.3f) reason=%s",
              queued.task_id, loop.fusion.prepared ? "true" : "false",
              loop.fusion.committed ? "true" : "false",
              loop.fusion.stale ? "true" : "false",
              loop.fusion.rolled_back ? "true" : "false",
              loop.fusion.pair_results, loop.fusion.tracks_created,
              loop.fusion.tracks_updated, loop.fusion.tracks_retired,
              loop.fusion.hidden_raw_members, loop.fusion.score_positive_events,
              loop.fusion.score_negative_events, loop.fusion.score_raw_updates,
              loop.fusion.covisibility_added, loop.fusion.covisibility_updated,
              loop.fusion.visibility_regions_completed,
              loop.fusion.visibility_regions_started,
              loop.fusion.visibility_projected_points, loop.fusion.visibility_ms,
              loop.fusion.prepare_ms, loop.fusion.commit_ms, loop.fusion.reason.c_str());
          }
          for (const auto & geometry : loop.geometry_results) {
            RCLCPP_INFO(
              get_logger(),
              "[F3O-RANSAC] task=%lu query=%lu candidate=%lu accepted=%s matches=%zu "
              "inliers=%zu ratio=%.3f mean=%.4f max=%.4f pose_error=(%.4f,%.4f) reason=%s",
              queued.task_id, geometry.query_keyframe_id.local_kf_id,
              geometry.candidate_keyframe_id.local_kf_id,
              geometry.accepted ? "true" : "false", geometry.matches,
              geometry.inliers, geometry.inlier_ratio, geometry.mean_residual_m,
              geometry.max_residual_m, geometry.current_translation_error_m,
              geometry.current_rotation_error_rad, geometry.reason.c_str());
          }
        } else if (queued.kind == SecondaryTaskKind::FiducialOptimization &&
          queued.fiducial.has_value())
        {
          auto revalidation = backend_.RevalidateFiducialTask(*queued.fiducial);
          RCLCPP_WARN(
            get_logger(),
            "[F3H-FID-REVALIDATE] task=%lu decision=%s translation=%.6f "
            "rotation=%.6f yaw=%.6f reason=%s",
            queued.task_id, orbslam3_multi::ToString(revalidation.decision),
            revalidation.error.translation_m, revalidation.error.rotation_rad,
            revalidation.error.yaw_rad, revalidation.reason.c_str());
          if (revalidation.decision == orbslam3_multi::FiducialTaskDecision::Stale) {
            stale = true;
            final_reason = revalidation.reason;
            EmitSecondaryFlowEvent(
              "secondary_worker_validation", "stale", queued,
              secondary_queue_.Pending(), " reason=" + final_reason);
          } else if (revalidation.decision != orbslam3_multi::FiducialTaskDecision::Ready) {
            hard_failure = true;
            final_reason = revalidation.reason;
          } else {
            optimization_active_.store(true);
            UpdateBackpressure();
            auto active_task = revalidation.task;
            for (uint32_t pass = 1;
              pass <= fiducial_optimization_config_.max_refinement_passes; ++pass)
            {
              const auto graph = backend_.BuildFiducialPoseGraph(active_task);
              if (!graph.success) {
                hard_failure = true;
                final_reason = graph.reason;
                break;
              }
              EmitSecondaryFlowEvent(
                "secondary_worker_pose_graph_builder", "graph_build", queued,
                secondary_queue_.Pending(),
                " pass=" + std::to_string(pass) +
                " window=" + std::to_string(graph.problem.keyframes.size()) +
                " controls=" + std::to_string(graph.problem.control_indices.size()));
              RCLCPP_WARN(
                get_logger(),
                "[F3I-GRAPH-BUILD] task=%lu pass=%u submap=(%u,%lu) "
                "control=%lu target=%lu window=%zu controls=%zu edges=%zu propagation=%zu",
                queued.task_id, pass, active_task.submap_id.drone_id,
                active_task.submap_id.map_epoch, active_task.control_keyframe_id.local_kf_id,
                active_task.keyframe_id.local_kf_id, graph.problem.keyframes.size(),
                graph.problem.control_indices.size(), graph.problem.temporal_edges.size(),
                graph.problem.propagation_plan.size());

              const auto proposal = backend_.OptimizeFiducialPoseGraph(graph.problem);
              EmitSecondaryFlowEvent(
                "pose_graph_builder_optimization_manager", "solve", queued,
                secondary_queue_.Pending(),
                " pass=" + std::to_string(pass) +
                " status=" + orbslam3_multi::ToString(proposal.status));
              RCLCPP_WARN(
                get_logger(),
                "[F3J-OPTIMIZE] task=%lu pass=%u status=%s correction_fraction=%.3f "
                "before=(%.6f,%.6f,%.6f) after=(%.6f,%.6f,%.6f) reason=%s",
                queued.task_id, pass, orbslam3_multi::ToString(proposal.status),
                proposal.correction_fraction, proposal.initial_error.translation_m,
                proposal.initial_error.rotation_rad, proposal.initial_error.yaw_rad,
                proposal.final_error.translation_m, proposal.final_error.rotation_rad,
                proposal.final_error.yaw_rad, proposal.reason.c_str());

              const auto validation = backend_.ValidateFiducialProposal(
                graph.problem, proposal);
              EmitSecondaryFlowEvent(
                "optimization_manager_validation", "validate", queued,
                secondary_queue_.Pending(),
                " pass=" + std::to_string(pass) +
                " decision=" + orbslam3_multi::ToString(validation.decision));
              RCLCPP_WARN(
                get_logger(),
                "[F3L-VALIDATE] task=%lu pass=%u decision=%s final=(%.6f,%.6f,%.6f) "
                "reason=%s",
                queued.task_id, pass, orbslam3_multi::ToString(validation.decision),
                validation.final_error.translation_m, validation.final_error.rotation_rad,
                validation.final_error.yaw_rad, validation.reason.c_str());
              if (validation.decision == orbslam3_multi::ValidationDecision::HardFailure) {
                hard_failure = true;
                final_reason = validation.reason;
                break;
              }

              const auto commit = backend_.CommitFiducialProposal(
                graph.problem, proposal, validation);
              if (!commit.committed) {
                if (commit.reason == "scoped_revision_conflict" ||
                  commit.reason == "revision_conflict")
                {
                  RCLCPP_WARN(
                    get_logger(),
                    "[F3K-COMMIT-STALE] task=%lu pass=%u reason=%s action=revalidate",
                    queued.task_id, pass, commit.reason.c_str());
                  revalidation = backend_.RevalidateFiducialTask(active_task);
                  if (revalidation.decision ==
                    orbslam3_multi::FiducialTaskDecision::Stale)
                  {
                    stale = true;
                    final_reason = "stale_after_revision_conflict";
                    break;
                  }
                  if (revalidation.decision ==
                    orbslam3_multi::FiducialTaskDecision::Ready)
                  {
                    fiducial_retry_task = revalidation.task;
                    stale = true;
                    final_reason = "fiducial_revision_conflict_retry";
                    break;
                  }
                }
                hard_failure = true;
                final_reason = commit.reason;
                break;
              }
              EmitSecondaryFlowEvent(
                "validation_global_pose_store",
                commit.full_accept ? "commit_full" : "commit_partial", queued,
                secondary_queue_.Pending(),
                " pass=" + std::to_string(pass) +
                " moved=" + std::to_string(commit.pose_changes.updated_ids.size()) +
                " propagated=" +
                std::to_string(commit.pose_changes.control_propagated_ids.size()));
              EmitSecondaryFlowEvent(
                "global_pose_store_global_map_builder", "pose_dirty", queued,
                secondary_queue_.Pending(),
                " kfs=" + std::to_string(commit.pose_changes.updated_ids.size()) +
                " propagated=" +
                std::to_string(commit.pose_changes.control_propagated_ids.size()));
              RCLCPP_WARN(
                get_logger(),
                "[F3K-ATOMIC-COMMIT] task=%lu pass=%u full=%s commit=%lu revision=%lu "
                "window=%zu late_window=%zu tail=%zu moved_kfs=%zu "
                "control_propagated=%zu hard_added=%zu",
                queued.task_id, pass, commit.full_accept ? "true" : "false",
                commit.pose_changes.commit_id, commit.pose_changes.store_revision_after,
                commit.window_keyframes, commit.late_window_keyframes,
                commit.tail_keyframes, commit.pose_changes.updated_ids.size(),
                commit.pose_changes.control_propagated_ids.size(),
                commit.pose_changes.hard_fiducial_ids.size());
              post_optimization_rerun_ids.insert(
                post_optimization_rerun_ids.end(), commit.rerun_keyframe_ids.begin(),
                commit.rerun_keyframe_ids.end());
              post_optimization_arrival = active_task.observation_arrival_id;
              if (commit.full_accept) {
                RCLCPP_WARN(
                  get_logger(),
                  "[F3K-CONTINUATION-UPDATE] task=%lu submap=(%u,%lu) control=%lu "
                  "commit=%lu revision=%lu",
                  queued.task_id, active_task.submap_id.drone_id,
                  active_task.submap_id.map_epoch, active_task.keyframe_id.local_kf_id,
                  commit.pose_changes.commit_id, commit.pose_changes.store_revision_after);
              }
              if (commit.full_accept) {
                full_success = true;
                final_reason = commit.reason;
                break;
              }

              revalidation = backend_.RevalidateFiducialTask(active_task);
              if (revalidation.decision == orbslam3_multi::FiducialTaskDecision::Stale) {
                full_success = true;
                final_reason = "partial_commit_revalidated_within_threshold";
                break;
              }
              if (revalidation.decision != orbslam3_multi::FiducialTaskDecision::Ready) {
                hard_failure = true;
                final_reason = revalidation.reason;
                break;
              }
              active_task = revalidation.task;
            }
            if (!full_success && !stale && !hard_failure) {
              hard_failure = true;
              final_reason = "max_refinement_passes_exceeded";
            }
            optimization_active_.store(false);
          }
        } else {
          hard_failure = true;
          final_reason = "secondary_payload_missing";
        }
      } catch (const std::exception & ex) {
        optimization_active_.store(false);
        hard_failure = true;
        final_reason = std::string("secondary_exception:") + ex.what();
        RCLCPP_ERROR(
          get_logger(), "[F3H-SECONDARY-EXCEPTION] task=%lu kind=%s error=%s",
          queued.task_id, ToString(queued.kind), ex.what());
      } catch (...) {
        optimization_active_.store(false);
        hard_failure = true;
        final_reason = "secondary_exception:unknown";
        RCLCPP_ERROR(
          get_logger(), "[F3H-SECONDARY-EXCEPTION] task=%lu kind=%s error=unknown",
          queued.task_id, ToString(queued.kind));
      }

      if (stale) {
        ++secondary_stale_;
      } else if (full_success) {
        ++secondary_committed_;
      } else if (hard_failure) {
        ++secondary_hard_failed_;
        secondary_blocking_failure_.store(true);
        RCLCPP_ERROR(
          get_logger(), "[F3L-HARD-FAILURE] task=%lu reason=%s stop_drones=true",
          queued.task_id, final_reason.c_str());
        EmitSecondaryFlowEvent(
          "optimization_manager_validation", "hard_failure", queued,
          secondary_queue_.Pending(), " reason=" + final_reason);
      }
      ++secondary_processed_;
      secondary_queue_.Complete(queued);
      active_secondary_tasks_.fetch_sub(1);
      if (fiducial_retry_task.has_value()) {
        try {
          const auto enqueue = secondary_queue_.PushFiducial(*fiducial_retry_task);
          RCLCPP_WARN(
            get_logger(),
            "[F3H-FID-RETRY] previous_task=%lu retry_task=%lu submap=(%u,%lu) "
            "kf=%lu enqueued=%s duplicate=%s sequence=%lu pending=%zu "
            "cause=revision_conflict",
            queued.task_id, fiducial_retry_task->task_id,
            fiducial_retry_task->submap_id.drone_id,
            fiducial_retry_task->submap_id.map_epoch,
            fiducial_retry_task->keyframe_id.local_kf_id,
            enqueue.enqueued ? "true" : "false",
            enqueue.duplicate ? "true" : "false",
            enqueue.enqueue_sequence, enqueue.pending);
          if (enqueue.enqueued) {
            EmitSecondaryFlowEvent(
              "secondary_worker_secondary_queue_retry", "fiducial_retry_enqueue",
              queued, enqueue.pending,
              " retry_task=" + std::to_string(fiducial_retry_task->task_id));
          }
        } catch (const std::exception & ex) {
          RCLCPP_WARN(
            get_logger(),
            "[F3H-FID-RETRY-SKIP] previous_task=%lu reason=%s",
            queued.task_id, ex.what());
        }
      }
      UpdateBackpressure();
      RCLCPP_WARN(
        get_logger(),
        "[F3H-SECONDARY-DONE] task=%lu stale=%s committed=%s hard_failed=%s "
        "reason=%s pending=%zu",
        queued.task_id, stale ? "true" : "false", full_success ? "true" : "false",
        hard_failure ? "true" : "false", final_reason.c_str(),
        secondary_queue_.Pending());
      EmitSecondaryLifecycleEvent(
        "done", queued, secondary_queue_.Pending(),
        " reason=" + final_reason +
        " stale=" + (stale ? std::string("true") : std::string("false")) +
        " committed=" + (full_success ? std::string("true") : std::string("false")));
      if (!post_optimization_rerun_ids.empty()) {
        std::sort(
          post_optimization_rerun_ids.begin(), post_optimization_rerun_ids.end());
        post_optimization_rerun_ids.erase(
          std::unique(
            post_optimization_rerun_ids.begin(), post_optimization_rerun_ids.end()),
          post_optimization_rerun_ids.end());
        auto reruns = backend_.CreateFusionRefreshTasks(
          post_optimization_arrival, post_optimization_rerun_ids);
        const size_t created = reruns.size();
        const size_t enqueued = EnqueueLoopTasks(
          &reruns, "post_optimization_pose_change", true);
        RCLCPP_WARN(
          get_logger(),
          "[F3Q-POST-OPT-LOOPS] previous_task=%lu moved=%zu grouped=%zu created=%zu "
          "enqueued=%zu pending=%zu",
          queued.task_id, post_optimization_rerun_ids.size(),
          post_optimization_rerun_ids.size() >= created ?
          post_optimization_rerun_ids.size() - created : 0U,
          created, enqueued, secondary_queue_.Pending());
      }
      if (fusion_retry_keyframe.has_value()) {
        try {
          auto retries = backend_.CreateLoopTasks(
            fusion_retry_arrival, {*fusion_retry_keyframe});
          if (queued.loop.has_value()) {
            for (auto & retry : retries) {
              retry.intent = queued.loop->intent;
            }
          }
          const size_t created = retries.size();
          const size_t enqueued = EnqueueLoopTasks(
            &retries, fusion_retry_cause, true);
          const uint64_t retry_task = retries.empty() ? 0U : retries.front().task_id;
          RCLCPP_WARN(
            get_logger(),
            "[F3P-FUSION-RETRY] previous_task=%lu retry_task=%lu query=(%u,%lu,%lu) "
            "created=%zu enqueued=%zu cause=%s pending=%zu",
            queued.task_id, retry_task, fusion_retry_keyframe->drone_id,
            fusion_retry_keyframe->map_epoch, fusion_retry_keyframe->local_kf_id,
            created, enqueued, fusion_retry_cause.c_str(), secondary_queue_.Pending());
          if (enqueued != 0U) {
            EmitSecondaryFlowEvent(
              "secondary_worker_secondary_queue_retry", "retry_enqueue", queued,
              secondary_queue_.Pending(),
              " retry_task=" + std::to_string(retry_task));
          }
        } catch (const std::exception & ex) {
          RCLCPP_WARN(
            get_logger(),
            "[F3P-FUSION-RETRY-SKIP] previous_task=%lu reason=%s",
            queued.task_id, ex.what());
        }
      }
    }
  }

  size_t EnqueueLoopTasks(
    std::vector<orbslam3_multi::LoopTask> * tasks,
    const std::string & cause,
    bool retry_completed_revision = false)
  {
    if (tasks == nullptr) {
      return 0;
    }
    size_t enqueued_count = 0;
    for (auto & task : *tasks) {
      if (task.task_id == 0U) {
        task.task_id = next_pipeline_task_id_.fetch_add(1);
      }
      const auto enqueue = secondary_queue_.PushLoop(task, retry_completed_revision);
      if (enqueue.enqueued) {
        ++enqueued_count;
      }
      RCLCPP_INFO(
        get_logger(),
        "[F3N-LOOP-ENQUEUE] task=%lu arrival=%lu query=(%u,%lu,%lu) "
        "revision=(raw=%lu,appearance=%lu,geometry=%lu,validation=%lu,anchor=%lu) "
        "intent=%s priority=LOW sequence=%lu pending=%zu "
        "enqueued=%s duplicate=%s retry=%s cause=%s",
        task.task_id, task.source_arrival_id, task.query_keyframe_id.drone_id,
        task.query_keyframe_id.map_epoch, task.query_keyframe_id.local_kf_id,
        task.revision.raw_revision, task.revision.appearance_revision,
        task.revision.geometry_revision, task.revision.validation_revision,
        task.revision.anchor_revision,
        orbslam3_multi::ToString(task.intent),
        enqueue.enqueue_sequence, enqueue.pending,
        enqueue.enqueued ? "true" : "false",
        enqueue.duplicate ? "true" : "false",
        retry_completed_revision ? "true" : "false", cause.c_str());
    }
    UpdateBackpressure();
    return enqueued_count;
  }

  void EnqueueSecondaryWork(
    const orbslam3_multi::RawInsertResult & raw,
    const std::string & source)
  {
    auto plan = backend_.PlanSecondaryWork(raw);
    if (plan.database_update.has_value()) {
      plan.database_update->task_id = next_pipeline_task_id_.fetch_add(1);
      const auto enqueue = secondary_queue_.PushDatabaseUpdate(*plan.database_update);
      std::ostringstream detail;
      detail << " task=" << plan.database_update->task_id
             << " priority=MEDIUM covis_kfs="
             << plan.database_update->covisibility_keyframe_ids.size()
             << " loop_kfs=" << plan.database_update->loop_keyframe_ids.size();
      EmitFlowEvent(
        "raw_db_secondary_queue_database", "database_update_enqueue",
        raw.arrival_id, source, raw.submap_id.drone_id, raw.submap_id.map_epoch,
        enqueue.pending, detail.str());
      RCLCPP_WARN(
        get_logger(),
        "[F3M-DATABASE-ENQUEUE] task=%lu arrival=%lu submap=(%u,%lu) "
        "revision=%lu priority=MEDIUM covis_kfs=%zu loop_kfs=%zu sequence=%lu pending=%zu",
        plan.database_update->task_id, raw.arrival_id, raw.submap_id.drone_id,
        raw.submap_id.map_epoch, raw.submap_revision,
        plan.database_update->covisibility_keyframe_ids.size(),
        plan.database_update->loop_keyframe_ids.size(), enqueue.enqueue_sequence,
        enqueue.pending);
      UpdateBackpressure();
      return;
    }
    EnqueueLoopTasks(&plan.direct_loop_tasks, "raw_without_covisibility_update");
  }

  orbslam3_multi::FiducialObservation ToLiveObservation(
    uint64_t arrival_id,
    const orbslam3_multi::RawKeyFrameId & keyframe_id,
    const orbslam3_msgs::msg::OrbKeyFrame & keyframe,
    const GroundTruthMatch & match) const
  {
    Eigen::Matrix4d world_T_body;
    if (!PoseToMatrix(match.sample.world_T_body, &world_T_body)) {
      throw std::invalid_argument("muestra GT normalizada invalida");
    }
    orbslam3_multi::FiducialObservation observation;
    observation.arrival_id = arrival_id;
    observation.keyframe_id = keyframe_id;
    observation.fiducial_id = match.sample.fiducial_id;
    observation.fiducial_visit_id = match.sample.fiducial_visit_id;
    observation.world_T_camera_target = MatrixToPose(world_T_body * body_T_camera_);
    observation.keyframe_stamp_sec = StampToSeconds(keyframe.stamp);
    observation.observation_stamp_sec =
      static_cast<double>(match.sample.stamp_ns) * 1e-9;
    observation.association_dt_sec = match.dt_sec;
    observation.distance_to_fiducial_m = match.sample.distance_to_fiducial_m;
    observation.source = "simulated_gt";
    observation.quality = match.dt_sec <= 0.5 * fiducial_gt_max_dt_sec_ ? "ok" : "low";
    return observation;
  }

  static orbslam3_multi::FiducialObservation ToReplayObservation(
    const orbslam3_multi::RecordedFiducialObservation & recorded)
  {
    orbslam3_multi::FiducialObservation observation;
    observation.arrival_id = recorded.arrival_id;
    observation.keyframe_id = recorded.keyframe_id;
    observation.fiducial_id = recorded.fiducial_id;
    observation.fiducial_visit_id = recorded.fiducial_visit_id;
    observation.world_T_camera_target = recorded.world_T_camera_target;
    observation.keyframe_stamp_sec = recorded.keyframe_stamp_sec;
    observation.observation_stamp_sec = recorded.observation_stamp_sec;
    observation.association_dt_sec = recorded.association_dt_sec;
    observation.distance_to_fiducial_m = recorded.distance_to_fiducial_m;
    observation.source = recorded.source;
    observation.quality = recorded.quality;
    return observation;
  }

  void ProcessLiveFiducials(
    uint64_t arrival_id,
    const orbslam3_multi::RawInsertResult & raw)
  {
    if (!fiducial_sim_enabled_) {
      return;
    }
    auto keyframe_ids = raw.new_keyframe_ids;
    std::sort(
      keyframe_ids.begin(), keyframe_ids.end(),
      [](const auto & lhs, const auto & rhs) {
        return lhs.local_kf_id < rhs.local_kf_id;
      });
    for (const auto & keyframe_id : keyframe_ids) {
      const auto keyframe = backend_.GetRawKeyFrame(keyframe_id);
      if (!keyframe.has_value()) {
        RCLCPP_WARN(
          get_logger(),
          "[F3E-FID-ASSOC-REJECT] arrival_id=%lu submap=(%u,%lu) kf=%lu "
          "reason=raw_keyframe_missing",
          arrival_id, keyframe_id.drone_id, keyframe_id.map_epoch,
          keyframe_id.local_kf_id);
        continue;
      }

      const auto samples = gt_buffer_.Snapshot(keyframe_id.drone_id);
      const auto match = GroundTruthBuffer::FindNearest(
        samples, StampToNanoseconds(keyframe->stamp), fiducial_gt_max_dt_sec_);
      if (!match.matched) {
        RCLCPP_INFO(
          get_logger(),
          "[F3E-FID-ASSOC-REJECT] arrival_id=%lu submap=(%u,%lu) kf=%lu "
          "kf_stamp=%.6f nearest_dt=%.6f samples=%zu reason=%s",
          arrival_id, keyframe_id.drone_id, keyframe_id.map_epoch,
          keyframe_id.local_kf_id, StampToSeconds(keyframe->stamp), match.dt_sec,
          samples.size(), match.reason.c_str());
        continue;
      }

      const auto observation = ToLiveObservation(arrival_id, keyframe_id, *keyframe, match);
      RCLCPP_WARN(
        get_logger(),
        "[F3E-FID-KF-ASSOC] arrival_id=%lu submap=(%u,%lu) kf=%lu fid=%d "
        "kf_stamp=%.6f gt_stamp=%.6f nearest_dt=%.6f distance=%.3f quality=%s",
        arrival_id, keyframe_id.drone_id, keyframe_id.map_epoch,
        keyframe_id.local_kf_id, observation.fiducial_id,
        observation.keyframe_stamp_sec, observation.observation_stamp_sec,
        observation.association_dt_sec, observation.distance_to_fiducial_m,
        observation.quality.c_str());
      HandleFiducialObservation(observation, true, "live");
    }
  }

  void ProcessReplayFiducials(
    uint64_t arrival_id,
    const orbslam3_multi::RawInsertResult & raw)
  {
    const auto found = replay_fiducials_by_arrival_.find(arrival_id);
    if (found == replay_fiducials_by_arrival_.end()) {
      return;
    }
    auto observations = found->second;
    std::sort(
      observations.begin(), observations.end(),
      [](const auto & lhs, const auto & rhs) {
        if (lhs.keyframe_stamp_sec != rhs.keyframe_stamp_sec) {
          return lhs.keyframe_stamp_sec < rhs.keyframe_stamp_sec;
        }
        return lhs.keyframe_id.local_kf_id < rhs.keyframe_id.local_kf_id;
      });
    for (const auto & recorded : observations) {
      const auto observation = ToReplayObservation(recorded);
      RCLCPP_WARN(
        get_logger(),
        "[F3E-FID-REPLAY-OBS] arrival_id=%lu submap=(%u,%lu) kf=%lu fid=%d "
        "nearest_dt=%.6f raw_new_kfs=%zu",
        arrival_id, observation.keyframe_id.drone_id,
        observation.keyframe_id.map_epoch, observation.keyframe_id.local_kf_id,
        observation.fiducial_id, observation.association_dt_sec,
        raw.new_keyframe_ids.size());
      HandleFiducialObservation(observation, false, "replay");
    }
  }

  void HandleFiducialObservation(
    const orbslam3_multi::FiducialObservation & observation,
    bool append_to_journal,
    const std::string & execution_source)
  {
    EmitFlowEvent(
      "server_fiducial_anchor_manager", "fiducial_observation",
      observation.arrival_id, execution_source, observation.keyframe_id.drone_id,
      observation.keyframe_id.map_epoch, primary_queue_.Pending(),
      " fid=" + std::to_string(observation.fiducial_id) +
      " kf=" + std::to_string(observation.keyframe_id.local_kf_id));
    const auto result = backend_.ProcessFiducialObservation(
      observation, append_to_journal);
    ++fiducial_observations_processed_;
    if (result.status == orbslam3_multi::FiducialProcessStatus::AnchorCreated) {
      const bool replaced_loop_anchor =
        result.reason == "loop_anchor_replaced_by_first_hard_fiducial";
      if (!replaced_loop_anchor) {
        ++fiducial_anchors_created_;
      }
      std::ostringstream detail;
      detail << " status=" << orbslam3_multi::ToString(result.status)
             << " fid=" << observation.fiducial_id
             << " visit=" << observation.fiducial_visit_id
             << " kf=" << observation.keyframe_id.local_kf_id
             << " created=" << result.pose_changes.created_ids.size()
             << " hard=" << result.pose_changes.hard_fiducial_ids.size()
             << " pose_revision=" << result.pose_changes.store_revision_after;
      EmitFlowEvent(
        "fiducial_anchor_manager_global_pose_store",
        replaced_loop_anchor ? "first_hard_reanchor" : "first_anchor_commit",
        observation.arrival_id, execution_source, observation.keyframe_id.drone_id,
        observation.keyframe_id.map_epoch, primary_queue_.Pending(), detail.str());
      EmitFlowEvent(
        "global_pose_store_global_map_builder",
        replaced_loop_anchor ? "first_hard_reanchor_dirty" : "first_anchor_dirty",
        observation.arrival_id, execution_source, observation.keyframe_id.drone_id,
        observation.keyframe_id.map_epoch, primary_queue_.Pending(), detail.str());
      RCLCPP_WARN(
        get_logger(),
        "[F3E-FID-FIRST-ANCHOR] arrival_id=%lu source=%s submap=(%u,%lu) "
        "kf=%lu fid=%d status=%s created=%zu commit=%lu revision=%lu "
        "world_T_local_t=(%.3f,%.3f,%.3f)",
        observation.arrival_id, execution_source.c_str(),
        observation.keyframe_id.drone_id, observation.keyframe_id.map_epoch,
        observation.keyframe_id.local_kf_id, observation.fiducial_id,
        orbslam3_multi::ToString(result.pose_changes.status),
        result.pose_changes.created_ids.size(), result.pose_changes.commit_id,
        result.pose_changes.store_revision_after, result.world_T_local.position.x,
        result.world_T_local.position.y, result.world_T_local.position.z);
      if (replaced_loop_anchor) {
        RCLCPP_WARN(
          get_logger(),
          "[F3O-FID-LOOP-REANCHOR] arrival_id=%lu submap=(%u,%lu) kf=%lu "
          "fid=%d updated=%zu propagated=%zu hard=%zu commit=%lu revision=%lu",
          observation.arrival_id, observation.keyframe_id.drone_id,
          observation.keyframe_id.map_epoch, observation.keyframe_id.local_kf_id,
          observation.fiducial_id, result.pose_changes.updated_ids.size(),
          result.pose_changes.control_propagated_ids.size(),
          result.pose_changes.hard_fiducial_ids.size(), result.pose_changes.commit_id,
          result.pose_changes.store_revision_after);
      }
      RCLCPP_WARN(
        get_logger(),
        "[F3E-FID-KF-HARD] submap=(%u,%lu) kf=%lu hard=%s total_hard=%zu",
        observation.keyframe_id.drone_id, observation.keyframe_id.map_epoch,
        observation.keyframe_id.local_kf_id, result.hard_keyframe ? "true" : "false",
        result.pose_changes.hard_fiducial_ids.size());
    } else if (
      result.status == orbslam3_multi::FiducialProcessStatus::RevisitWithinThreshold)
    {
      ++fiducial_observations_deferred_;
      RCLCPP_INFO(
        get_logger(),
        "[F3H-FID-POSE-ERROR] arrival_id=%lu source=%s submap=(%u,%lu) kf=%lu "
        "fid=%d visit=%lu translation=%.6f rotation=%.6f yaw=%.6f "
        "status=within_threshold promote_control=%s reason=%s",
        observation.arrival_id, execution_source.c_str(),
        observation.keyframe_id.drone_id, observation.keyframe_id.map_epoch,
        observation.keyframe_id.local_kf_id, observation.fiducial_id,
        observation.fiducial_visit_id, result.error.translation_m,
        result.error.rotation_rad, result.error.yaw_rad,
        result.promote_control ? "true" : "false",
        result.reason.c_str());
    } else if (
      result.status == orbslam3_multi::FiducialProcessStatus::OptimizationRequired &&
      result.optimization_task.has_value())
    {
      RCLCPP_WARN(
        get_logger(),
        "[F3H-FID-POSE-ERROR] arrival_id=%lu source=%s submap=(%u,%lu) kf=%lu "
        "fid=%d visit=%lu translation=%.6f rotation=%.6f yaw=%.6f "
        "status=optimization_required",
        observation.arrival_id, execution_source.c_str(),
        observation.keyframe_id.drone_id, observation.keyframe_id.map_epoch,
        observation.keyframe_id.local_kf_id, observation.fiducial_id,
        observation.fiducial_visit_id, result.error.translation_m,
        result.error.rotation_rad, result.error.yaw_rad);
      const auto enqueue = secondary_queue_.PushFiducial(*result.optimization_task);
      std::ostringstream detail;
      detail << " task=" << result.optimization_task->task_id
             << " priority=MAX fid=" << observation.fiducial_id
             << " visit=" << observation.fiducial_visit_id
             << " kf=" << observation.keyframe_id.local_kf_id
             << " enqueued=" << (enqueue.enqueued ? "true" : "false")
             << " duplicate=" << (enqueue.duplicate ? "true" : "false");
      EmitFlowEvent(
        "fiducial_anchor_manager_secondary_queue", "opt_fid_enqueue",
        observation.arrival_id, execution_source, observation.keyframe_id.drone_id,
        observation.keyframe_id.map_epoch, enqueue.pending, detail.str());
      RCLCPP_WARN(
        get_logger(),
        "[F3H-FID-TASK-ENQUEUE] task=%lu sequence=%lu priority=MAX "
        "submap=(%u,%lu) kf=%lu fid=%d visit=%lu pending=%zu enqueued=%s duplicate=%s",
        result.optimization_task->task_id, enqueue.enqueue_sequence,
        observation.keyframe_id.drone_id, observation.keyframe_id.map_epoch,
        observation.keyframe_id.local_kf_id, observation.fiducial_id,
        observation.fiducial_visit_id, enqueue.pending,
        enqueue.enqueued ? "true" : "false", enqueue.duplicate ? "true" : "false");
      UpdateBackpressure();
    } else {
      RCLCPP_ERROR(
        get_logger(),
        "[F3E-FID-OBS] arrival_id=%lu source=%s submap=(%u,%lu) kf=%lu "
        "fid=%d status=%s reason=%s",
        observation.arrival_id, execution_source.c_str(),
        observation.keyframe_id.drone_id, observation.keyframe_id.map_epoch,
        observation.keyframe_id.local_kf_id, observation.fiducial_id,
        orbslam3_multi::ToString(result.status), result.reason.c_str());
    }

    const auto stats = backend_.GetPoseStats();
    RCLCPP_INFO(
      get_logger(),
      "[F3E-FID-STATS] observations=%lu anchors_created=%lu deferred=%lu "
      "anchors=%lu poses=%lu active=%lu hard=%lu journaled=%s",
      fiducial_observations_processed_, fiducial_anchors_created_,
      fiducial_observations_deferred_, stats.anchors, stats.poses,
      stats.active_poses, stats.hard_fiducial_keyframes,
      result.journaled ? "true" : "false");
  }

  sensor_msgs::msg::PointCloud2 BuildPointCloud(
    const orbslam3_multi::GlobalMapBuildResult & build,
    const rclcpp::Time & stamp) const
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "world";
    cloud.header.stamp = stamp;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(build.points.size());
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = 32;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.push_back(MakePointField("x", 0, sensor_msgs::msg::PointField::FLOAT32));
    cloud.fields.push_back(MakePointField("y", 4, sensor_msgs::msg::PointField::FLOAT32));
    cloud.fields.push_back(MakePointField("z", 8, sensor_msgs::msg::PointField::FLOAT32));
    cloud.fields.push_back(MakePointField("score", 12, sensor_msgs::msg::PointField::FLOAT32));
    cloud.fields.push_back(MakePointField("rgb", 16, sensor_msgs::msg::PointField::FLOAT32));
    cloud.fields.push_back(MakePointField("drone_id", 20, sensor_msgs::msg::PointField::UINT32));
    cloud.fields.push_back(
      MakePointField(
        "map_epoch_low", 24, sensor_msgs::msg::PointField::UINT32));
    cloud.fields.push_back(
      MakePointField(
        "map_epoch_high", 28, sensor_msgs::msg::PointField::UINT32));
    cloud.data.resize(cloud.row_step);
    for (size_t index = 0; index < build.points.size(); ++index) {
      const auto & point = build.points[index];
      const size_t offset = index * cloud.point_step;
      const float rgb = ScoreRgb(point.score);
      const uint32_t epoch_low = static_cast<uint32_t>(point.mappoint_id.map_epoch);
      const uint32_t epoch_high = static_cast<uint32_t>(point.mappoint_id.map_epoch >> 32U);
      WritePointField(&cloud.data, offset + 0, point.x);
      WritePointField(&cloud.data, offset + 4, point.y);
      WritePointField(&cloud.data, offset + 8, point.z);
      WritePointField(&cloud.data, offset + 12, point.score);
      WritePointField(&cloud.data, offset + 16, rgb);
      WritePointField(&cloud.data, offset + 20, point.mappoint_id.drone_id);
      WritePointField(&cloud.data, offset + 24, epoch_low);
      WritePointField(&cloud.data, offset + 28, epoch_high);
    }
    return cloud;
  }

  visualization_msgs::msg::MarkerArray BuildKeyFrameMarkers(
    const orbslam3_multi::GlobalMapBuildResult & build,
    const rclcpp::Time & stamp)
  {
    visualization_msgs::msg::MarkerArray array;
    std::set<orbslam3_multi::RawKeyFrameId> current_ids;
    for (const auto & keyframe : build.keyframes) {
      current_ids.insert(keyframe.keyframe_id);
      auto marker_id = keyframe_marker_ids_.find(keyframe.keyframe_id);
      if (marker_id == keyframe_marker_ids_.end()) {
        marker_id = keyframe_marker_ids_.emplace(
          keyframe.keyframe_id, next_keyframe_marker_id_++).first;
      }

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = stamp;
      marker.ns = "global_keyframes";
      marker.id = marker_id->second;
      marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = keyframe.world_pose;
      marker.scale.x = 0.025;
      marker.color = orbslam3_server::SubmapColor(
        keyframe.keyframe_id.drone_id, keyframe.keyframe_id.map_epoch);

      geometry_msgs::msg::Point origin;
      geometry_msgs::msg::Point a;
      geometry_msgs::msg::Point b;
      geometry_msgs::msg::Point c;
      geometry_msgs::msg::Point d;
      a.x = -0.18; a.y = -0.12; a.z = 0.30;
      b.x = 0.18; b.y = -0.12; b.z = 0.30;
      c.x = 0.18; c.y = 0.12; c.z = 0.30;
      d.x = -0.18; d.y = 0.12; d.z = 0.30;
      const auto add_line = [&marker](
        const geometry_msgs::msg::Point & from,
        const geometry_msgs::msg::Point & to)
        {
          marker.points.push_back(from);
          marker.points.push_back(to);
        };
      add_line(origin, a);
      add_line(origin, b);
      add_line(origin, c);
      add_line(origin, d);
      add_line(a, b);
      add_line(b, c);
      add_line(c, d);
      add_line(d, a);
      array.markers.push_back(std::move(marker));
    }

    for (const auto & previous : published_keyframes_) {
      if (current_ids.find(previous) != current_ids.end()) {
        continue;
      }
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = stamp;
      marker.ns = "global_keyframes";
      marker.id = keyframe_marker_ids_.at(previous);
      marker.action = visualization_msgs::msg::Marker::DELETE;
      array.markers.push_back(std::move(marker));
    }
    published_keyframes_ = std::move(current_ids);
    return array;
  }

  // Única frontera de publicación de cloud/KFs; consume la revisión conjunta del builder.
  void BuildAndPublishGlobalMap(
    uint64_t arrival_id,
    const std::string & source,
    const orbslam3_multi::RawInsertResult & raw)
  {
    if (raw.has_material_changes) {
      std::ostringstream raw_detail;
      raw_detail << " kfs=" << raw.new_keyframe_ids.size() + raw.updated_keyframe_ids.size()
                 << " mps=" << raw.new_mappoint_ids.size() + raw.updated_mappoint_ids.size();
      EmitFlowEvent(
        "raw_db_global_map_builder", "raw_dirty", arrival_id, source,
        raw.submap_id.drone_id, raw.submap_id.map_epoch, primary_queue_.Pending(),
        raw_detail.str());
    }

    const auto build = backend_.BuildGlobalMap();
    if (!build.changed) {
      RCLCPP_INFO(
        get_logger(),
        "[F3F-BUILDER-SKIP] arrival_id=%lu reason=no_public_dirty dirty_kfs=%zu "
        "dirty_mps=%zu deferred_submaps=%zu deferred_kfs=%zu deferred_mps=%zu "
        "backfilled_submaps=%zu backfilled_kfs=%zu backfilled_mps=%zu "
        "cached_kfs=%zu cached_mps=%zu",
        arrival_id, build.dirty_keyframes, build.dirty_mappoints,
        build.deferred_unanchored_submaps, build.deferred_unanchored_keyframes,
        build.deferred_unanchored_mappoints, build.backfilled_submaps,
        build.backfilled_keyframes, build.backfilled_mappoints,
        build.keyframes.size(), build.points.size());
      return;
    }

    std::ostringstream detail;
    detail << " revision=" << build.publication_revision
           << " points=" << build.points.size()
           << " kfs=" << build.keyframes.size()
           << " recalculated_kfs=" << build.recalculated_keyframes
           << " recalculated_mps=" << build.recalculated_mappoints
           << " recalculated_tracks=" << build.recalculated_fused_tracks
           << " fusion_revision=" << build.fusion_revision;
    EmitFlowEvent(
      "global_map_builder_server", "global_map_build", arrival_id, source,
      raw.submap_id.drone_id, raw.submap_id.map_epoch, primary_queue_.Pending(),
      detail.str());

    const auto stamp = get_clock()->now();
    const auto cloud = BuildPointCloud(build, stamp);
    const auto markers = BuildKeyFrameMarkers(build, stamp);
    sparse_cloud_publisher_->publish(cloud);
    keyframes_publisher_->publish(markers);
    EmitFlowEvent(
      "server_rviz_cloud", "pointcloud2_publish", arrival_id, source,
      raw.submap_id.drone_id, raw.submap_id.map_epoch, primary_queue_.Pending(),
      detail.str());
    EmitFlowEvent(
      "server_rviz_keyframes", "marker_array_publish", arrival_id, source,
      raw.submap_id.drone_id, raw.submap_id.map_epoch, primary_queue_.Pending(),
      detail.str());

    RCLCPP_WARN(
      get_logger(),
      "[F3F-GLOBALMAP-PUBLISH] arrival_id=%lu revision=%lu raw_revision=%lu "
      "pose_revision=%lu score_revision=%lu points=%zu keyframes=%zu "
      "dirty_kfs=%zu dirty_mps=%zu recalculated_kfs=%zu recalculated_mps=%zu "
      "recalculated_tracks=%zu fusion_revision=%lu "
      "deferred_submaps=%zu deferred_kfs=%zu deferred_mps=%zu "
      "backfilled_submaps=%zu backfilled_kfs=%zu backfilled_mps=%zu "
      "skip_unanchored=%zu skip_bad=%zu skip_invalid=%zu skip_no_world_kf=%zu "
      "reference_assoc=%zu fallback_assoc=%zu fallback_submap=%zu "
      "frame=world score_field=true rgb_field=true identity_fields=true",
      arrival_id, build.publication_revision, raw.submap_revision,
      build.pose_revision, build.score_revision, build.points.size(), build.keyframes.size(),
      build.dirty_keyframes, build.dirty_mappoints, build.recalculated_keyframes,
      build.recalculated_mappoints, build.recalculated_fused_tracks,
      build.fusion_revision, build.deferred_unanchored_submaps,
      build.deferred_unanchored_keyframes, build.deferred_unanchored_mappoints,
      build.backfilled_submaps, build.backfilled_keyframes, build.backfilled_mappoints,
      build.skipped_unanchored, build.skipped_bad,
      build.skipped_invalid, build.skipped_without_world_keyframe,
      build.reference_associations, build.fallback_associations,
      build.fallback_submap_points);
  }

  void UpdateBackpressure()
  {
    const size_t primary_pending = primary_queue_.Pending();
    const auto secondary = secondary_queue_.PendingStats();
    backpressure_->Update(primary_pending);
    std::optional<bool> transition;
    {
      std::lock_guard<std::mutex> lock(backpressure_state_mutex_);
      if (!secondary_pressure_active_ && secondary.critical >= secondary_high_watermark_) {
        secondary_pressure_active_ = true;
      } else if (secondary_pressure_active_ &&
        secondary.critical <= secondary_low_watermark_)
      {
        secondary_pressure_active_ = false;
      }
      const bool active = backpressure_->Active() || secondary_pressure_active_ ||
        optimization_active_.load() || secondary_blocking_failure_.load();
      if (active != combined_backpressure_active_) {
        combined_backpressure_active_ = active;
        transition = active;
      }
    }
    if (transition.has_value()) {
      PublishBackpressure(*transition, primary_pending, false);
      if (!*transition && full_snapshot_enabled_ && replay_path_.empty()) {
        std::vector<uint32_t> deferred;
        {
          std::lock_guard<std::mutex> lock(snapshot_state_mutex_);
          for (const auto & [drone_id, pending_snapshot] : snapshot_deferred_) {
            if (pending_snapshot) {
              deferred.push_back(drone_id);
            }
          }
        }
        for (const uint32_t drone_id : deferred) {
          RequestSnapshot(drone_id, "backpressure_release");
        }
        TryStartNextSnapshot();
      }
    }
  }

  void PublishBackpressure(bool active, size_t pending, bool initial)
  {
    const auto secondary = secondary_queue_.PendingStats();
    std_msgs::msg::Bool message;
    message.data = active;
    backpressure_publisher_->publish(message);
    RCLCPP_WARN(
      get_logger(),
      "[F3C-BACKPRESSURE] active=%s primary_pending=%zu primary_high=%zu "
      "primary_low=%zu secondary_pending=%zu secondary_critical=%zu "
      "secondary_maintenance=%zu secondary_high=%zu secondary_low=%zu "
      "optimization_active=%s blocking_failure=%s initial=%s",
      active ? "true" : "false", pending, high_watermark_, low_watermark_,
      secondary.total, secondary.critical, secondary.maintenance,
      secondary_high_watermark_, secondary_low_watermark_,
      optimization_active_.load() ? "true" : "false",
      secondary_blocking_failure_.load() ? "true" : "false",
      initial ? "true" : "false");
    if (!initial) {
      EmitFlowEvent(
        "server_mission_gate", active ? "backpressure_on" : "backpressure_off",
        0, replay_path_.empty() ? "live" : "replay", 0, 0, pending);
    }
  }

  void MaybeCommitSyntheticAnchor(
    uint64_t arrival_id,
    const std::string & source,
    const orbslam3_multi::RawInsertResult & raw)
  {
    if (!debug_anchor_enabled_ || debug_anchor_committed_ ||
      raw.submap_id.drone_id != debug_anchor_drone_id_ ||
      raw.submap_id.map_epoch != debug_anchor_map_epoch_ ||
      raw.new_keyframe_ids.empty())
    {
      return;
    }

    geometry_msgs::msg::Pose world_T_local;
    world_T_local.position.x = debug_anchor_x_;
    world_T_local.position.y = debug_anchor_y_;
    world_T_local.position.z = debug_anchor_z_;
    world_T_local.orientation.w = 1.0;
    const auto changes = backend_.CommitAnchor(raw.submap_id, world_T_local, arrival_id);
    debug_anchor_committed_ =
      changes.status == orbslam3_multi::PoseCommitStatus::Applied ||
      changes.status == orbslam3_multi::PoseCommitStatus::AlreadyAnchored;

    std::ostringstream detail;
    detail << " status=" << orbslam3_multi::ToString(changes.status)
           << " anchor=synthetic"
           << " created=" << changes.created_ids.size()
           << " invalidated=" << changes.invalidated_ids.size()
           << " pose_revision=" << changes.store_revision_after;
    EmitFlowEvent(
      "raw_db_global_pose_store", "synthetic_anchor", arrival_id, source,
      raw.submap_id.drone_id, raw.submap_id.map_epoch, primary_queue_.Pending(),
      detail.str());
    RCLCPP_WARN(
      get_logger(),
      "[F3D-SYNTHETIC-ANCHOR] arrival_id=%lu source=%s submap=(%u,%lu) "
      "status=%s created=%zu invalidated=%zu commit=%lu revision=%lu "
      "translation=(%.3f,%.3f,%.3f)",
      arrival_id, source.c_str(), raw.submap_id.drone_id, raw.submap_id.map_epoch,
      orbslam3_multi::ToString(changes.status), changes.created_ids.size(),
      changes.invalidated_ids.size(), changes.commit_id, changes.store_revision_after,
      debug_anchor_x_, debug_anchor_y_, debug_anchor_z_);
  }

  void EmitFlowEvent(
    const std::string & edge_id,
    const std::string & stage,
    uint64_t arrival_id,
    const std::string & source,
    uint32_t drone_id,
    uint64_t map_epoch,
    size_t pending,
    const std::string & extra_detail = "")
  {
    std::ostringstream json;
    json << "{\"phase\":\"3G\",\"edge_id\":\"" << edge_id
         << "\",\"stage\":\"" << stage
         << "\",\"flow_id\":\"primary:" << arrival_id
         << "\",\"source\":\"" << source
         << "\",\"arrival_id\":" << arrival_id
         << ",\"drone_id\":" << drone_id
         << ",\"map_epoch\":" << map_epoch
         << ",\"pending\":" << pending
         << ",\"detail\":\"" << stage << " arrival=" << arrival_id
         << " source=" << source << " pending=" << pending << extra_detail << "\"}";
    std_msgs::msg::String message;
    message.data = json.str();
    flow_publisher_->publish(message);
  }

  void EmitSecondaryFlowEvent(
    const std::string & edge_id,
    const std::string & stage,
    const SecondaryTask & task,
    size_t pending,
    const std::string & extra_detail = "")
  {
    uint32_t drone_id = 0;
    uint64_t map_epoch = 0;
    if (task.fiducial.has_value()) {
      drone_id = task.fiducial->submap_id.drone_id;
      map_epoch = task.fiducial->submap_id.map_epoch;
    } else if (task.database_update.has_value()) {
      drone_id = task.database_update->submap_id.drone_id;
      map_epoch = task.database_update->submap_id.map_epoch;
    } else if (task.loop.has_value()) {
      drone_id = task.loop->query_keyframe_id.drone_id;
      map_epoch = task.loop->query_keyframe_id.map_epoch;
    }
    std::ostringstream json;
    json << "{\"phase\":\"3L\",\"edge_id\":\"" << edge_id
         << "\",\"stage\":\"" << stage
         << "\",\"flow_id\":\"secondary:" << task.task_id
         << "\",\"source\":\"secondary\",\"arrival_id\":" << task.task_id
         << ",\"drone_id\":" << drone_id
         << ",\"map_epoch\":" << map_epoch
         << ",\"pending\":" << pending
         << ",\"detail\":\"" << stage << " task=" << task.task_id
         << " priority=" << ToString(task.priority) << " pending=" << pending
         << extra_detail << "\"}";
    std_msgs::msg::String message;
    message.data = json.str();
    flow_publisher_->publish(message);
  }

  void EmitSecondaryLifecycleEvent(
    const std::string & task_state,
    const SecondaryTask & task,
    size_t pending,
    const std::string & extra_detail)
  {
    uint32_t drone_id = 0;
    uint64_t map_epoch = 0;
    if (task.fiducial.has_value()) {
      drone_id = task.fiducial->submap_id.drone_id;
      map_epoch = task.fiducial->submap_id.map_epoch;
    } else if (task.database_update.has_value()) {
      drone_id = task.database_update->submap_id.drone_id;
      map_epoch = task.database_update->submap_id.map_epoch;
    } else if (task.loop.has_value()) {
      drone_id = task.loop->query_keyframe_id.drone_id;
      map_epoch = task.loop->query_keyframe_id.map_epoch;
    }
    std::ostringstream json;
    json << "{\"phase\":\"3L\",\"kind\":\"secondary_task_lifecycle\""
         << ",\"task_state\":\"" << task_state
         << "\",\"flow_id\":\"secondary:" << task.task_id
         << "\",\"task_id\":" << task.task_id
         << ",\"source\":\"secondary\",\"drone_id\":" << drone_id
         << ",\"map_epoch\":" << map_epoch
         << ",\"pending\":" << pending
         << ",\"detail\":\"secondary_" << task_state
         << " task=" << task.task_id << " priority=" << ToString(task.priority)
         << " pending=" << pending << extra_detail << "\"}";
    std_msgs::msg::String message;
    message.data = json.str();
    flow_publisher_->publish(message);
  }

  PrimaryQueue primary_queue_;
  SecondaryTaskQueue secondary_queue_;
  std::unique_ptr<BackpressureHysteresis> backpressure_;
  orbslam3_multi::SparseGlobalBackend backend_;
  std::thread worker_;
  std::thread secondary_worker_;

  std::vector<rclcpp::Subscription<orbslam3_msgs::msg::OrbMap>::SharedPtr> subscriptions_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr>
  gt_subscriptions_;
  rclcpp::CallbackGroup::SharedPtr gt_callback_group_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr flow_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr backpressure_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sparse_cloud_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr keyframes_publisher_;
  std::map<uint32_t, rclcpp::Client<orbslam3_msgs::srv::GetOrbMap>::SharedPtr>
  snapshot_clients_;
  std::map<uint32_t, std::string> snapshot_services_;
  rclcpp::TimerBase::SharedPtr snapshot_startup_timer_;
  rclcpp::TimerBase::SharedPtr snapshot_periodic_timer_;
  // Protege únicamente scheduling de snapshots; nunca se mantiene durante llamadas ROS.
  std::mutex snapshot_state_mutex_;
  std::deque<std::pair<uint32_t, std::string>> snapshot_waiting_;
  uint32_t snapshot_global_active_drone_ = 0;
  std::map<uint32_t, bool> snapshot_deferred_;
  std::map<uint32_t, uint64_t> last_map_sequence_;

  std::atomic<uint32_t> active_primary_tasks_{0};
  std::atomic<uint32_t> max_active_primary_tasks_{0};
  std::atomic<uint32_t> active_secondary_tasks_{0};
  std::atomic<uint32_t> max_active_secondary_tasks_{0};
  std::atomic<uint64_t> processed_inputs_{0};
  std::atomic<uint64_t> secondary_processed_{0};
  std::atomic<uint64_t> secondary_stale_{0};
  std::atomic<uint64_t> secondary_committed_{0};
  std::atomic<uint64_t> secondary_hard_failed_{0};
  std::atomic<uint64_t> next_pipeline_task_id_{1000000000000ULL};
  std::atomic<bool> optimization_active_{false};
  std::atomic<bool> secondary_blocking_failure_{false};
  std::atomic<bool> shutting_down_{false};
  uint64_t replay_total_ = 0;
  size_t high_watermark_ = 8;
  size_t low_watermark_ = 2;
  size_t secondary_high_watermark_ = 64;
  size_t secondary_low_watermark_ = 16;
  // Combina presión principal/secundaria y optimización sin mezclarla con locks del backend.
  std::mutex backpressure_state_mutex_;
  bool secondary_pressure_active_ = false;
  bool combined_backpressure_active_ = false;
  bool record_enabled_ = false;
  std::string record_path_;
  std::string replay_path_;
  int64_t replay_entry_delay_ms_ = 0;
  bool full_snapshot_enabled_ = true;
  double full_snapshot_startup_delay_sec_ = 35.0;
  double full_snapshot_period_sec_ = 35.0;
  bool debug_drop_one_delta_ = false;
  bool debug_delta_dropped_ = false;
  uint32_t debug_drop_drone_id_ = 1;
  bool debug_anchor_enabled_ = false;
  bool debug_anchor_committed_ = false;
  uint32_t debug_anchor_drone_id_ = 1;
  uint64_t debug_anchor_map_epoch_ = 0;
  double debug_anchor_x_ = 10.0;
  double debug_anchor_y_ = 0.0;
  double debug_anchor_z_ = 0.0;
  bool fiducial_sim_enabled_ = true;
  double fiducial_gt_max_dt_sec_ = 1.0;
  orbslam3_multi::FiducialOptimizationConfig fiducial_optimization_config_;
  Eigen::Matrix4d body_T_camera_ = Eigen::Matrix4d::Identity();
  std::vector<FiducialConfig> fiducials_;
  GroundTruthBuffer gt_buffer_;
  std::map<uint32_t, uint64_t> gt_samples_received_;
  std::map<uint32_t, int32_t> live_fiducial_id_by_drone_;
  std::map<uint32_t, uint64_t> live_visit_id_by_drone_;
  uint64_t next_fiducial_visit_id_ = 1;
  std::map<orbslam3_multi::RawSubmapId, ReplayVisitState> replay_visit_state_;
  std::map<uint64_t, std::vector<orbslam3_multi::RecordedFiducialObservation>>
  replay_fiducials_by_arrival_;
  uint64_t fiducial_observations_processed_ = 0;
  uint64_t fiducial_anchors_created_ = 0;
  uint64_t fiducial_observations_deferred_ = 0;
  std::map<orbslam3_multi::RawKeyFrameId, int32_t> keyframe_marker_ids_;
  std::set<orbslam3_multi::RawKeyFrameId> published_keyframes_;
  int32_t next_keyframe_marker_id_ = 1;
};

}  // namespace orbslam3_server

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<orbslam3_server::GlobalMapServer>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
