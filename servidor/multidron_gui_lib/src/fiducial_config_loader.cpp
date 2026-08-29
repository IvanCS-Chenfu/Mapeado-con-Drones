#include "multidron_gui_lib/fiducial_config_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <stdexcept>

namespace multidron_gui_lib
{
namespace
{

QQuaternion QuaternionFromRpyDeg(double roll_deg, double pitch_deg, double yaw_deg)
{
  constexpr double kPi = 3.14159265358979323846;
  const double roll = roll_deg * kPi / 180.0;
  const double pitch = pitch_deg * kPi / 180.0;
  const double yaw = yaw_deg * kPi / 180.0;

  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);

  const double w = cr * cp * cy + sr * sp * sy;
  const double x = sr * cp * cy - cr * sp * sy;
  const double y = cr * sp * cy + sr * cp * sy;
  const double z = cr * cp * sy - sr * sp * cy;
  return QQuaternion(
    static_cast<float>(w), static_cast<float>(x),
    static_cast<float>(y), static_cast<float>(z)).normalized();
}

bool ReadVec3(const YAML::Node & node, QVector3D * value)
{
  if (!node || !node.IsSequence() || node.size() != 3U || value == nullptr) {
    return false;
  }
  *value = QVector3D(
    node[0].as<float>(), node[1].as<float>(), node[2].as<float>());
  return true;
}

}  // namespace

bool FiducialConfigLoader::Load(
  const std::string & path,
  FiducialVector * objects,
  std::string * error)
{
  if (objects == nullptr) {
    if (error != nullptr) {
      *error = "output_null";
    }
    return false;
  }

  try {
    const YAML::Node root = YAML::LoadFile(path);
    const YAML::Node yaml_objects = root["objects"];
    if (!yaml_objects || !yaml_objects.IsSequence()) {
      if (error != nullptr) {
        *error = "objects_missing_or_not_sequence";
      }
      return false;
    }

    FiducialVector parsed;
    parsed.reserve(yaml_objects.size());
    for (const auto & yaml_object : yaml_objects) {
      FiducialObject object;
      object.object_id = yaml_object["object_id"].as<std::int32_t>();
      if (yaml_object["shape"]) {
        object.shape = yaml_object["shape"].as<std::string>();
      }

      const YAML::Node size = yaml_object["size_m"];
      if (!size || !size.IsMap()) {
        throw std::runtime_error("size_m invalido en objeto " + std::to_string(object.object_id));
      }
      object.size_m = QVector3D(
        size["x"].as<float>(), size["y"].as<float>(), size["z"].as<float>());

      const YAML::Node transform = yaml_object["world_T_object"];
      if (!transform || !ReadVec3(transform["translation_m"], &object.position)) {
        throw std::runtime_error(
                "world_T_object.translation_m invalido en objeto " +
                std::to_string(object.object_id));
      }
      QVector3D rpy_deg;
      if (!ReadVec3(transform["rotation_rpy_deg"], &rpy_deg)) {
        throw std::runtime_error(
                "world_T_object.rotation_rpy_deg invalido en objeto " +
                std::to_string(object.object_id));
      }
      object.orientation = QuaternionFromRpyDeg(rpy_deg.x(), rpy_deg.y(), rpy_deg.z());

      const YAML::Node faces = yaml_object["faces"];
      if (faces && faces.IsMap()) {
        for (const auto & face : faces) {
          const YAML::Node definition = face.second;
          const bool enabled = definition["enabled"] && definition["enabled"].as<bool>();
          if (enabled && definition["tag_id"]) {
            object.tag_ids.push_back(definition["tag_id"].as<std::int32_t>());
          }
        }
      }
      std::sort(object.tag_ids.begin(), object.tag_ids.end());
      parsed.push_back(std::move(object));
    }

    *objects = std::move(parsed);
    if (error != nullptr) {
      error->clear();
    }
    return true;
  } catch (const std::exception & ex) {
    if (error != nullptr) {
      *error = ex.what();
    }
    return false;
  }
}

}  // namespace multidron_gui_lib
