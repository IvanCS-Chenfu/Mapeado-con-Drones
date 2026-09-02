#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace task_lib
{

struct Vec3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct AxisAlignedBox
{
  Vec3 min;
  Vec3 max;
};

struct MissionConfig
{
  std::string mission_id;
  std::vector<std::uint32_t> drones;
  std::string frame_id = "world";
  AxisAlignedBox mapping_roi;
  Vec3 mapping_hysteresis;
  double level_height = 0.0;
};

struct MappingLevel
{
  std::uint32_t level_index = 0;
  double z_min = 0.0;
  double z_max = 0.0;
};

enum class BaseSide : std::uint8_t
{
  AB = 0,
  BC = 1,
  CD = 2,
  DA = 3,
};

struct BaseSubRoi
{
  std::string region_id;
  std::uint32_t level_index = 0;
  BaseSide side = BaseSide::AB;
  AxisAlignedBox bounds;
};

struct MissionGeometry
{
  std::uint64_t config_revision = 0;
  AxisAlignedBox mapping_roi;
  AxisAlignedBox hard_flight_volume;
  Vec3 mapping_hysteresis;
  double level_height = 0.0;
  std::vector<MappingLevel> levels;
  std::vector<BaseSubRoi> regions;
};

MissionConfig LoadMissionConfig(const std::string & path);
MissionGeometry BuildMissionGeometry(const MissionConfig & config);
std::string BaseSideName(BaseSide side);

}  // namespace task_lib
