#include "orbslam3_server/fiducial_object_interpreter.hpp"

#include "orbslam3_msgs/msg/fiducial_tag_observation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

#include <Eigen/Eigenvalues>
#include <yaml-cpp/yaml.h>

namespace orbslam3_server
{
namespace
{

constexpr double kWeightEpsilon = 1e-6;
constexpr double kDominantWeightRatio = 2.0;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

Eigen::Vector3d ParseVector3(const YAML::Node & node, const std::string & name)
{
  if (!node.IsSequence() || node.size() != 3U) {
    throw std::invalid_argument(name + " debe contener tres valores");
  }
  const Eigen::Vector3d value(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
  if (!value.allFinite()) {
    throw std::invalid_argument(name + " contiene valores no finitos");
  }
  return value;
}

Eigen::Matrix3d RotationFromRpyDegrees(const Eigen::Vector3d & rpy_deg)
{
  const Eigen::AngleAxisd roll(rpy_deg.x() * kDegToRad, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch(rpy_deg.y() * kDegToRad, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw(rpy_deg.z() * kDegToRad, Eigen::Vector3d::UnitZ());
  return (yaw * pitch * roll).toRotationMatrix();
}

Eigen::Isometry3d FaceTransform(const std::string & face, const Eigen::Vector3d & size)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  Eigen::Vector3d x_axis;
  Eigen::Vector3d y_axis;
  Eigen::Vector3d z_axis;
  if (face == "pos_x") {
    transform.translation() = Eigen::Vector3d(0.5 * size.x(), 0.0, 0.0);
    x_axis = Eigen::Vector3d::UnitY();
    y_axis = Eigen::Vector3d::UnitZ();
    z_axis = Eigen::Vector3d::UnitX();
  } else if (face == "neg_x") {
    transform.translation() = Eigen::Vector3d(-0.5 * size.x(), 0.0, 0.0);
    x_axis = -Eigen::Vector3d::UnitY();
    y_axis = Eigen::Vector3d::UnitZ();
    z_axis = -Eigen::Vector3d::UnitX();
  } else if (face == "pos_y") {
    transform.translation() = Eigen::Vector3d(0.0, 0.5 * size.y(), 0.0);
    x_axis = -Eigen::Vector3d::UnitX();
    y_axis = Eigen::Vector3d::UnitZ();
    z_axis = Eigen::Vector3d::UnitY();
  } else if (face == "neg_y") {
    transform.translation() = Eigen::Vector3d(0.0, -0.5 * size.y(), 0.0);
    x_axis = Eigen::Vector3d::UnitX();
    y_axis = Eigen::Vector3d::UnitZ();
    z_axis = -Eigen::Vector3d::UnitY();
  } else if (face == "pos_z") {
    transform.translation() = Eigen::Vector3d(0.0, 0.0, 0.5 * size.z());
    x_axis = Eigen::Vector3d::UnitX();
    y_axis = Eigen::Vector3d::UnitY();
    z_axis = Eigen::Vector3d::UnitZ();
  } else if (face == "neg_z") {
    transform.translation() = Eigen::Vector3d(0.0, 0.0, -0.5 * size.z());
    x_axis = -Eigen::Vector3d::UnitX();
    y_axis = Eigen::Vector3d::UnitY();
    z_axis = -Eigen::Vector3d::UnitZ();
  } else {
    throw std::invalid_argument("cara fiducial desconocida: " + face);
  }
  transform.linear().col(0) = x_axis;
  transform.linear().col(1) = y_axis;
  transform.linear().col(2) = z_axis;
  if (!transform.matrix().allFinite() ||
    std::abs(transform.linear().determinant() - 1.0) > 1e-9)
  {
    throw std::invalid_argument("base no dextrogira para cara " + face);
  }
  return transform;
}

bool TransformToIsometry(
  const geometry_msgs::msg::Transform & message, Eigen::Isometry3d * transform)
{
  const Eigen::Quaterniond quaternion(
    message.rotation.w, message.rotation.x, message.rotation.y, message.rotation.z);
  const Eigen::Vector3d translation(
    message.translation.x, message.translation.y, message.translation.z);
  if (!translation.allFinite() || !std::isfinite(quaternion.w()) ||
    !std::isfinite(quaternion.x()) || !std::isfinite(quaternion.y()) ||
    !std::isfinite(quaternion.z()) || quaternion.norm() < 1e-9)
  {
    return false;
  }
  *transform = Eigen::Isometry3d::Identity();
  transform->linear() = quaternion.normalized().toRotationMatrix();
  transform->translation() = translation;
  return true;
}

geometry_msgs::msg::Pose ToPose(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  const Eigen::Quaterniond quaternion(transform.linear());
  const auto normalized = quaternion.normalized();
  pose.orientation.x = normalized.x();
  pose.orientation.y = normalized.y();
  pose.orientation.z = normalized.z();
  pose.orientation.w = normalized.w();
  return pose;
}

double RotationDistance(const Eigen::Matrix3d & lhs, const Eigen::Matrix3d & rhs)
{
  return Eigen::AngleAxisd(lhs * rhs.transpose()).angle();
}

Eigen::Quaterniond WeightedQuaternion(
  const std::vector<Eigen::Isometry3d> & candidates, const std::vector<double> & weights)
{
  Eigen::Matrix4d accumulator = Eigen::Matrix4d::Zero();
  for (size_t index = 0; index < candidates.size(); ++index) {
    Eigen::Quaterniond quaternion(candidates[index].linear());
    quaternion.normalize();
    const Eigen::Vector4d coefficients(
      quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z());
    accumulator += weights[index] * coefficients * coefficients.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(accumulator);
  if (solver.info() != Eigen::Success) {
    throw std::runtime_error("no se pudo fusionar rotacion fiducial");
  }
  const Eigen::Vector4d result = solver.eigenvectors().col(3);
  return Eigen::Quaterniond(result[0], result[1], result[2], result[3]).normalized();
}

double RobustFactor(double normalized_residual)
{
  const double residual = std::max(0.0, normalized_residual);
  return 1.0 / (1.0 + residual * residual);
}

double StampToSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

}  // namespace

void FiducialObjectInterpreter::Load(const std::string & yaml_path)
{
  const YAML::Node root = YAML::LoadFile(yaml_path);
  if (root["schema_version"].as<int>(0) <= 0) {
    throw std::invalid_argument("schema_version fiducial debe ser positivo");
  }
  FiducialInterpretationConfig loaded_config = config_;
  const auto eligibility = root["anchor_eligibility"];
  loaded_config.min_distance_m = eligibility["min_distance_m"].as<double>();
  loaded_config.max_distance_m = eligibility["max_distance_m"].as<double>();

  std::map<int32_t, ObjectConfig> objects;
  std::map<uint32_t, TagConfig> tags;
  for (const auto & object_node : root["objects"]) {
    ObjectConfig object;
    object.object_id = object_node["object_id"].as<int32_t>();
    if (object.object_id <= 0 || objects.count(object.object_id) != 0U) {
      throw std::invalid_argument("object_id fiducial invalido o duplicado");
    }
    if (object_node["shape"].as<std::string>() != "box") {
      throw std::invalid_argument("solo se soportan objetos fiduciales box");
    }
    const auto size_node = object_node["size_m"];
    const Eigen::Vector3d size(
      size_node["x"].as<double>(), size_node["y"].as<double>(),
      size_node["z"].as<double>());
    if (!size.allFinite() || (size.array() <= 0.0).any()) {
      throw std::invalid_argument("dimensiones fiduciales invalidas");
    }
    const auto pose_node = object_node["world_T_object"];
    object.world_T_object.translation() = ParseVector3(
      pose_node["translation_m"], "world_T_object.translation_m");
    object.world_T_object.linear() = RotationFromRpyDegrees(
      ParseVector3(
        pose_node["rotation_rpy_deg"], "world_T_object.rotation_rpy_deg"));
    objects.emplace(object.object_id, object);

    const auto faces = object_node["faces"];
    for (const auto & face_entry : faces) {
      const std::string face_name = face_entry.first.as<std::string>();
      const auto face = face_entry.second;
      if (!face["enabled"].as<bool>(false)) {
        continue;
      }
      TagConfig tag;
      tag.tag_id = face["tag_id"].as<uint32_t>();
      tag.object_id = object.object_id;
      tag.object_T_tag = FaceTransform(face_name, size);
      if (!tags.emplace(tag.tag_id, tag).second) {
        throw std::invalid_argument("tag_id fiducial duplicado");
      }
    }
  }
  if (!(loaded_config.min_distance_m >= 0.0) ||
    !(loaded_config.max_distance_m > loaded_config.min_distance_m))
  {
    throw std::invalid_argument("rango fiducial invalido");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  config_ = loaded_config;
  objects_ = std::move(objects);
  tags_ = std::move(tags);
  visits_.clear();
  recent_by_drone_.clear();
  next_visit_id_ = 1;
}

void FiducialObjectInterpreter::Configure(const FiducialInterpretationConfig & config)
{
  if (!(config.min_distance_m >= 0.0) ||
    !(config.max_distance_m > config.min_distance_m) ||
    !(config.consistency_translation_m > 0.0) ||
    !(config.consistency_rotation_rad > 0.0) || !(config.visit_gap_sec > 0.0) ||
    config.recent_capacity_per_drone == 0U)
  {
    throw std::invalid_argument("configuracion del interpretador fiducial invalida");
  }
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
  for (auto & item : recent_by_drone_) {
    while (item.second.size() > config_.recent_capacity_per_drone) {
      item.second.pop_front();
    }
  }
}

FiducialInterpretationConfig FiducialObjectInterpreter::GetConfig() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return config_;
}

FiducialKeyFrameInterpretation FiducialObjectInterpreter::Interpret(
  const orbslam3_multi::SynchronizedFiducialBatch & match)
{
  std::lock_guard<std::mutex> lock(mutex_);
  FiducialKeyFrameInterpretation result;
  result.keyframe_id = match.keyframe_id;
  result.raw_arrival_id = match.raw_first_arrival_id;
  result.stamp_sec = StampToSeconds(match.raw_keyframe.stamp);

  struct Candidate
  {
    const orbslam3_msgs::msg::FiducialTagObservation * observation = nullptr;
    const TagConfig * config = nullptr;
    Eigen::Isometry3d world_T_camera = Eigen::Isometry3d::Identity();
    double distance_m = 0.0;
    double base_weight = 0.0;
  };
  std::map<int32_t, std::vector<Candidate>> grouped;
  double max_area = kWeightEpsilon;
  for (const auto & observation : match.batch.observations) {
    max_area = std::max(max_area, observation.tag_area_px2);
  }
  for (const auto & observation : match.batch.observations) {
    const auto tag = tags_.find(observation.tag_id);
    if (tag == tags_.end()) {
      result.unknown_tag_ids.push_back(observation.tag_id);
      continue;
    }
    Eigen::Isometry3d camera_T_tag;
    if (!TransformToIsometry(observation.camera_t_tag, &camera_T_tag)) {
      continue;
    }
    const auto object = objects_.find(tag->second.object_id);
    if (object == objects_.end()) {
      continue;
    }
    Candidate candidate;
    candidate.observation = &observation;
    candidate.config = &tag->second;
    candidate.distance_m = camera_T_tag.translation().norm();
    candidate.base_weight = std::max(observation.quality_score, kWeightEpsilon) *
      std::sqrt(std::max(observation.tag_area_px2, 0.0) / max_area);
    const Eigen::Isometry3d camera_T_object =
      camera_T_tag * tag->second.object_T_tag.inverse();
    candidate.world_T_camera = object->second.world_T_object * camera_T_object.inverse();
    grouped[tag->second.object_id].push_back(std::move(candidate));
  }

  for (auto & group : grouped) {
    auto & candidates = group.second;
    InterpretedFiducialObject interpreted;
    interpreted.object_id = group.first;
    const bool all_in_range = std::all_of(
      candidates.begin(), candidates.end(), [this](const Candidate & candidate) {
        return candidate.distance_m >= config_.min_distance_m &&
        candidate.distance_m <= config_.max_distance_m;
      });

    const auto seed = std::max_element(
      candidates.begin(), candidates.end(), [](const Candidate & lhs, const Candidate & rhs) {
        if (lhs.base_weight != rhs.base_weight) {
          return lhs.base_weight < rhs.base_weight;
        }
        return lhs.config->tag_id > rhs.config->tag_id;
      });
    Eigen::Isometry3d fused = seed->world_T_camera;
    std::vector<double> effective_weights(candidates.size(), 0.0);
    for (int iteration = 0; iteration < 3; ++iteration) {
      double weight_sum = 0.0;
      Eigen::Vector3d translation = Eigen::Vector3d::Zero();
      for (size_t index = 0; index < candidates.size(); ++index) {
        const double translation_residual =
          (candidates[index].world_T_camera.translation() - fused.translation()).norm();
        const double rotation_residual = RotationDistance(
          candidates[index].world_T_camera.linear(), fused.linear());
        const double normalized = std::max(
          translation_residual / config_.consistency_translation_m,
          rotation_residual / config_.consistency_rotation_rad);
        effective_weights[index] = candidates[index].base_weight * RobustFactor(normalized);
        weight_sum += effective_weights[index];
        translation += effective_weights[index] * candidates[index].world_T_camera.translation();
      }
      if (weight_sum <= kWeightEpsilon) {
        break;
      }
      fused.translation() = translation / weight_sum;
      std::vector<Eigen::Isometry3d> poses;
      poses.reserve(candidates.size());
      for (const auto & candidate : candidates) {
        poses.push_back(candidate.world_T_camera);
      }
      fused.linear() = WeightedQuaternion(poses, effective_weights).toRotationMatrix();
    }

    double effective_sum = 0.0;
    double base_sum = 0.0;
    double quality_sum = 0.0;
    double distance_sum = 0.0;
    size_t coherent_count = 0U;
    double dominant_base = 0.0;
    for (size_t index = 0; index < candidates.size(); ++index) {
      const auto & candidate = candidates[index];
      InterpretedFiducialTag tag;
      tag.tag_id = candidate.config->tag_id;
      tag.object_id = candidate.config->object_id;
      tag.distance_m = candidate.distance_m;
      tag.base_weight = candidate.base_weight;
      tag.robust_weight = effective_weights[index];
      tag.translation_residual_m =
        (candidate.world_T_camera.translation() - fused.translation()).norm();
      tag.rotation_residual_rad = RotationDistance(
        candidate.world_T_camera.linear(), fused.linear());
      tag.in_range = candidate.distance_m >= config_.min_distance_m &&
        candidate.distance_m <= config_.max_distance_m;
      tag.world_T_camera = ToPose(candidate.world_T_camera);
      if (tag.translation_residual_m <= config_.consistency_translation_m &&
        tag.rotation_residual_rad <= config_.consistency_rotation_rad)
      {
        ++coherent_count;
      }
      effective_sum += tag.robust_weight;
      base_sum += tag.base_weight;
      dominant_base = std::max(dominant_base, tag.base_weight);
      quality_sum += tag.robust_weight * candidate.observation->quality_score;
      distance_sum += tag.robust_weight * tag.distance_m;
      interpreted.tags.push_back(std::move(tag));
    }
    const bool dominant_single = candidates.size() == 1U ||
      dominant_base >= kDominantWeightRatio * std::max(kWeightEpsilon, base_sum - dominant_base);
    const bool stable = coherent_count >=
      std::min<size_t>(2U, candidates.size()) || dominant_single;
    interpreted.world_T_camera_fused = ToPose(fused);
    interpreted.quality = effective_sum > kWeightEpsilon ?
      std::clamp(quality_sum / effective_sum, 0.0, 1.0) *
      std::clamp(effective_sum / std::max(base_sum, kWeightEpsilon), 0.0, 1.0) : 0.0;
    interpreted.distance_m = effective_sum > kWeightEpsilon ?
      distance_sum / effective_sum : seed->distance_m;
    interpreted.anchor_eligible = all_in_range && stable;
    interpreted.reason = !all_in_range ? "tag_out_of_range" :
      (stable ? "eligible" : "multiface_no_stable_solution");
    result.objects.push_back(std::move(interpreted));
  }

  std::sort(
    result.objects.begin(), result.objects.end(), [](const auto & lhs, const auto & rhs) {
      return lhs.object_id < rhs.object_id;
    });
  for (size_t index = 0; index < result.objects.size(); ++index) {
    if (!result.objects[index].anchor_eligible) {
      continue;
    }
    if (!result.primary_index.has_value() ||
      result.objects[index].quality > result.objects[*result.primary_index].quality ||
      (result.objects[index].quality == result.objects[*result.primary_index].quality &&
      result.objects[index].object_id < result.objects[*result.primary_index].object_id))
    {
      result.primary_index = index;
    }
  }
  if (result.primary_index.has_value()) {
    result.primary_visit_id = AssignVisitLocked(
      result.keyframe_id, result.objects[*result.primary_index].object_id, result.stamp_sec);
  }
  AppendRecentLocked(result);
  return result;
}

uint64_t FiducialObjectInterpreter::AssignVisitLocked(
  const orbslam3_multi::RawKeyFrameId & keyframe_id, int32_t object_id,
  double stamp_sec)
{
  const auto key = std::make_tuple(keyframe_id.drone_id, keyframe_id.map_epoch, object_id);
  auto & visits = visits_[key];
  VisitState * closest = nullptr;
  double closest_distance = std::numeric_limits<double>::infinity();
  for (auto & visit : visits) {
    const double distance = stamp_sec < visit.first_stamp_sec ?
      visit.first_stamp_sec - stamp_sec :
      (stamp_sec > visit.last_stamp_sec ? stamp_sec - visit.last_stamp_sec : 0.0);
    if (distance <= config_.visit_gap_sec && distance < closest_distance) {
      closest = &visit;
      closest_distance = distance;
    }
  }
  if (closest == nullptr) {
    visits.push_back(VisitState{next_visit_id_++, stamp_sec, stamp_sec});
    return visits.back().visit_id;
  }
  closest->first_stamp_sec = std::min(closest->first_stamp_sec, stamp_sec);
  closest->last_stamp_sec = std::max(closest->last_stamp_sec, stamp_sec);
  return closest->visit_id;
}

void FiducialObjectInterpreter::AppendRecentLocked(
  const FiducialKeyFrameInterpretation & interpretation)
{
  auto & recent = recent_by_drone_[interpretation.keyframe_id.drone_id];
  recent.push_back(interpretation);
  while (recent.size() > config_.recent_capacity_per_drone) {
    recent.pop_front();
  }
}

std::vector<FiducialKeyFrameInterpretation> FiducialObjectInterpreter::GetRecent(
  uint32_t drone_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = recent_by_drone_.find(drone_id);
  return found == recent_by_drone_.end() ?
         std::vector<FiducialKeyFrameInterpretation>{} :
         std::vector<FiducialKeyFrameInterpretation>(found->second.begin(), found->second.end());
}

size_t FiducialObjectInterpreter::ObjectCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return objects_.size();
}

size_t FiducialObjectInterpreter::TagCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return tags_.size();
}

}  // namespace orbslam3_server
