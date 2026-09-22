#include "fiducial-ground-truth.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>

#include <cmath>
#include <stdexcept>
#include <utility>

namespace orbslam3_ros2
{
namespace
{

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kHalfPi = 0.5f * kPi;

Eigen::Vector3f ParseVector3(const YAML::Node& node, const std::string& name)
{
    if (!node.IsSequence() || node.size() != 3U)
    {
        throw std::invalid_argument(name + " debe contener tres valores");
    }
    const Eigen::Vector3f value(
        node[0].as<float>(), node[1].as<float>(), node[2].as<float>());
    if (!value.allFinite())
    {
        throw std::invalid_argument(name + " contiene valores no finitos");
    }
    return value;
}

Eigen::Matrix3f RotationFromRpyRadians(const Eigen::Vector3f& rpy)
{
    return (Eigen::AngleAxisf(rpy.z(), Eigen::Vector3f::UnitZ()) *
            Eigen::AngleAxisf(rpy.y(), Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(rpy.x(), Eigen::Vector3f::UnitX()))
        .toRotationMatrix();
}

Sophus::SE3f FaceTransform(
    const std::string& face,
    const Eigen::Vector3f& size,
    float surface_offset_m)
{
    Eigen::Vector3f translation;
    Eigen::Vector3f rpy;
    if (face == "pos_x")
    {
        translation = {size.x() * 0.5f + surface_offset_m, 0.0f, 0.0f};
        rpy = {kHalfPi, 0.0f, kHalfPi};
    }
    else if (face == "neg_x")
    {
        translation = {-size.x() * 0.5f - surface_offset_m, 0.0f, 0.0f};
        rpy = {kHalfPi, 0.0f, -kHalfPi};
    }
    else if (face == "pos_y")
    {
        translation = {0.0f, size.y() * 0.5f + surface_offset_m, 0.0f};
        rpy = {kHalfPi, 0.0f, kPi};
    }
    else if (face == "neg_y")
    {
        translation = {0.0f, -size.y() * 0.5f - surface_offset_m, 0.0f};
        rpy = {kHalfPi, 0.0f, 0.0f};
    }
    else if (face == "pos_z")
    {
        translation = {0.0f, 0.0f, size.z() * 0.5f + surface_offset_m};
        rpy = Eigen::Vector3f::Zero();
    }
    else if (face == "neg_z")
    {
        translation = {0.0f, 0.0f, -size.z() * 0.5f - surface_offset_m};
        rpy = {0.0f, kPi, 0.0f};
    }
    else
    {
        throw std::invalid_argument("cara fiducial desconocida: " + face);
    }
    return Sophus::SE3f(RotationFromRpyRadians(rpy), translation);
}

}  // namespace

void FiducialGroundTruth::Load(
    const std::string& objects_config_path,
    const std::string& rendering_config_path)
{
    if (objects_config_path.empty() || rendering_config_path.empty())
    {
        throw std::invalid_argument("rutas YAML de diagnostico fiducial vacias");
    }
    const YAML::Node objects_root = YAML::LoadFile(objects_config_path);
    const YAML::Node rendering_root = YAML::LoadFile(rendering_config_path);
    if (objects_root["schema_version"].as<int>(0) <= 0 ||
        rendering_root["schema_version"].as<int>(0) <= 0)
    {
        throw std::invalid_argument("schema YAML fiducial invalido");
    }
    const float surface_offset_m = rendering_root["surface_offset_m"].as<float>();
    if (!std::isfinite(surface_offset_m) || surface_offset_m < 0.0f)
    {
        throw std::invalid_argument("surface_offset_m invalido");
    }

    std::map<int, Sophus::SE3f> loaded_tags;
    for (const auto& object_node : objects_root["objects"])
    {
        if (object_node["shape"].as<std::string>() != "box")
        {
            throw std::invalid_argument("solo se soportan objetos fiduciales box");
        }
        const YAML::Node size_node = object_node["size_m"];
        const Eigen::Vector3f size(
            size_node["x"].as<float>(), size_node["y"].as<float>(),
            size_node["z"].as<float>());
        if (!size.allFinite() || (size.array() <= 0.0f).any())
        {
            throw std::invalid_argument("tamano de objeto fiducial invalido");
        }
        const YAML::Node pose_node = object_node["world_T_object"];
        const Eigen::Vector3f rpy_deg = ParseVector3(
            pose_node["rotation_rpy_deg"], "world_T_object.rotation_rpy_deg");
        const Sophus::SE3f world_t_object(
            RotationFromRpyRadians(rpy_deg * static_cast<float>(kDegToRad)),
            ParseVector3(
                pose_node["translation_m"], "world_T_object.translation_m"));

        for (const auto& face_entry : object_node["faces"])
        {
            const std::string face_name = face_entry.first.as<std::string>();
            const YAML::Node face = face_entry.second;
            if (!face["enabled"].as<bool>(false))
            {
                continue;
            }
            const int tag_id = face["tag_id"].as<int>();
            if (tag_id < 0 || !loaded_tags.emplace(
                    tag_id,
                    world_t_object * FaceTransform(
                        face_name, size, surface_offset_m)).second)
            {
                throw std::invalid_argument("tag_id fiducial invalido o duplicado");
            }
        }
    }
    if (loaded_tags.empty())
    {
        throw std::invalid_argument("no hay tags fiduciales habilitados");
    }
    world_t_tags_ = std::move(loaded_tags);
}

bool FiducialGroundTruth::WorldTTag(
    int tag_id, Sophus::SE3f* world_t_tag) const
{
    const auto found = world_t_tags_.find(tag_id);
    if (found == world_t_tags_.end())
    {
        return false;
    }
    *world_t_tag = found->second;
    return true;
}

}  // namespace orbslam3_ros2
