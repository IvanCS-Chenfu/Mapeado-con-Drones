#include "task_lib/mission_config.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>

namespace task_lib
{
namespace
{

Vec3 ReadVec3(const YAML::Node & node, const std::string & name)
{
  if (!node.IsSequence() || node.size() != 3U) {
    throw std::invalid_argument(name + " debe ser una secuencia de tres números");
  }
  Vec3 value{node[0].as<double>(), node[1].as<double>(), node[2].as<double>()};
  if (!std::isfinite(value.x) || !std::isfinite(value.y) || !std::isfinite(value.z)) {
    throw std::invalid_argument(name + " contiene un valor no finito");
  }
  return value;
}

void ValidateConfig(const MissionConfig & config)
{
  if (config.mission_id.empty()) {
    throw std::invalid_argument("mission_id no puede estar vacío");
  }
  if (config.frame_id != "world") {
    throw std::invalid_argument("mapping_roi.frame_id debe ser world");
  }
  if (config.drones.empty()) {
    throw std::invalid_argument("drones debe contener al menos un drone_id");
  }
  std::set<std::uint32_t> unique_drones;
  for (const std::uint32_t drone_id : config.drones) {
    if (drone_id == 0U || !unique_drones.insert(drone_id).second) {
      throw std::invalid_argument("drones contiene un ID cero o duplicado");
    }
  }
  const auto & min = config.mapping_roi.min;
  const auto & max = config.mapping_roi.max;
  if (!(min.x < max.x && min.y < max.y && min.z < max.z)) {
    throw std::invalid_argument("mapping_roi debe tener volumen positivo");
  }
  const auto & h = config.mapping_hysteresis;
  if (h.x < 0.0 || h.y < 0.0 || h.z < 0.0) {
    throw std::invalid_argument("mapping_hysteresis no puede ser negativa");
  }
  if (!std::isfinite(config.level_height) || config.level_height <= 0.0) {
    throw std::invalid_argument("level_height debe ser finito y positivo");
  }
}

std::uint64_t HashConfig(const MissionConfig & config)
{
  std::ostringstream normalized;
  normalized << std::setprecision(17) << config.mission_id << '|' << config.frame_id << '|';
  for (const auto drone_id : config.drones) {
    normalized << drone_id << ',';
  }
  normalized << '|' << config.mapping_roi.min.x << ',' << config.mapping_roi.min.y << ','
             << config.mapping_roi.min.z << '|' << config.mapping_roi.max.x << ','
             << config.mapping_roi.max.y << ',' << config.mapping_roi.max.z << '|'
             << config.mapping_hysteresis.x << ',' << config.mapping_hysteresis.y << ','
             << config.mapping_hysteresis.z << '|' << config.level_height;
  std::uint64_t hash = 1469598103934665603ULL;
  for (const unsigned char character : normalized.str()) {
    hash ^= character;
    hash *= 1099511628211ULL;
  }
  return hash == 0U ? 1U : hash;
}

}  // namespace

MissionConfig LoadMissionConfig(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  MissionConfig config;
  config.mission_id = root["mission_id"].as<std::string>();
  if (!root["drones"].IsSequence()) {
    throw std::invalid_argument("drones debe ser una secuencia");
  }
  for (const auto & drone : root["drones"]) {
    config.drones.push_back(drone.as<std::uint32_t>());
  }
  const YAML::Node roi = root["mapping_roi"];
  config.frame_id = roi["frame_id"].as<std::string>();
  config.mapping_roi.min = ReadVec3(roi["min"], "mapping_roi.min");
  config.mapping_roi.max = ReadVec3(roi["max"], "mapping_roi.max");
  config.mapping_hysteresis = ReadVec3(
    root["mapping_hysteresis"], "mapping_hysteresis");
  config.level_height = root["level_height"].as<double>();
  ValidateConfig(config);
  return config;
}

MissionGeometry BuildMissionGeometry(const MissionConfig & config)
{
  ValidateConfig(config);
  MissionGeometry geometry;
  geometry.config_revision = HashConfig(config);
  geometry.mapping_roi = config.mapping_roi;
  geometry.mapping_hysteresis = config.mapping_hysteresis;
  geometry.level_height = config.level_height;
  geometry.hard_flight_volume = AxisAlignedBox{
    Vec3{
      config.mapping_roi.min.x - config.mapping_hysteresis.x,
      config.mapping_roi.min.y - config.mapping_hysteresis.y,
      config.mapping_roi.min.z - config.mapping_hysteresis.z},
    Vec3{
      config.mapping_roi.max.x + config.mapping_hysteresis.x,
      config.mapping_roi.max.y + config.mapping_hysteresis.y,
      config.mapping_roi.max.z + config.mapping_hysteresis.z}};

  const double total_height = config.mapping_roi.max.z - config.mapping_roi.min.z;
  const auto level_count = std::max<std::uint32_t>(
    1U, static_cast<std::uint32_t>(std::floor(total_height / config.level_height)));
  const double x_mid = 0.5 * (config.mapping_roi.min.x + config.mapping_roi.max.x);
  const double y_mid = 0.5 * (config.mapping_roi.min.y + config.mapping_roi.max.y);

  for (std::uint32_t index = 0; index < level_count; ++index) {
    const double z_min = config.mapping_roi.min.z + index * config.level_height;
    const double z_max = index + 1U == level_count ?
      config.mapping_roi.max.z : z_min + config.level_height;
    geometry.levels.push_back(MappingLevel{index, z_min, z_max});
    const std::string prefix = "level_" + std::to_string(index) + "_";
    geometry.regions.push_back(BaseSubRoi{
      prefix + "AB", index, BaseSide::AB,
      AxisAlignedBox{{config.mapping_roi.min.x, config.mapping_roi.min.y, z_min},
        {config.mapping_roi.max.x, y_mid, z_max}}});
    geometry.regions.push_back(BaseSubRoi{
      prefix + "BC", index, BaseSide::BC,
      AxisAlignedBox{{x_mid, config.mapping_roi.min.y, z_min},
        {config.mapping_roi.max.x, config.mapping_roi.max.y, z_max}}});
    geometry.regions.push_back(BaseSubRoi{
      prefix + "CD", index, BaseSide::CD,
      AxisAlignedBox{{config.mapping_roi.min.x, y_mid, z_min},
        {config.mapping_roi.max.x, config.mapping_roi.max.y, z_max}}});
    geometry.regions.push_back(BaseSubRoi{
      prefix + "DA", index, BaseSide::DA,
      AxisAlignedBox{{config.mapping_roi.min.x, config.mapping_roi.min.y, z_min},
        {x_mid, config.mapping_roi.max.y, z_max}}});
  }
  return geometry;
}

std::string BaseSideName(BaseSide side)
{
  switch (side) {
    case BaseSide::AB: return "AB";
    case BaseSide::BC: return "BC";
    case BaseSide::CD: return "CD";
    case BaseSide::DA: return "DA";
  }
  throw std::invalid_argument("BaseSide desconocido");
}

}  // namespace task_lib
