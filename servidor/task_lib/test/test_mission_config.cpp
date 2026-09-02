#include "task_lib/mission_config.hpp"

#include <gtest/gtest.h>

#include <fstream>
#include <stdexcept>
#include <string>

namespace
{

std::string WriteConfig(const std::string & name, const std::string & content)
{
  const std::string path = "/tmp/task_lib_" + name + ".yaml";
  std::ofstream stream(path);
  stream << content;
  return path;
}

const char * kValidConfig = R"(
mission_id: test_house
drones: [1, 2]
mapping_roi:
  frame_id: world
  min: [-10.0, -8.0, 0.0]
  max: [10.0, 8.0, 5.0]
mapping_hysteresis: [2.0, 1.0, 0.5]
level_height: 2.0
)";

TEST(MissionConfig, BuildsHalfRegionsAndAddsRemainderToLastLevel)
{
  const auto config = task_lib::LoadMissionConfig(WriteConfig("valid", kValidConfig));
  const auto geometry = task_lib::BuildMissionGeometry(config);
  ASSERT_EQ(geometry.levels.size(), 2U);
  EXPECT_DOUBLE_EQ(geometry.levels[0].z_min, 0.0);
  EXPECT_DOUBLE_EQ(geometry.levels[0].z_max, 2.0);
  EXPECT_DOUBLE_EQ(geometry.levels[1].z_min, 2.0);
  EXPECT_DOUBLE_EQ(geometry.levels[1].z_max, 5.0);
  ASSERT_EQ(geometry.regions.size(), 8U);
  EXPECT_DOUBLE_EQ(geometry.regions[0].bounds.max.y, 0.0);
  EXPECT_DOUBLE_EQ(geometry.regions[1].bounds.min.x, 0.0);
  EXPECT_DOUBLE_EQ(geometry.regions[2].bounds.min.y, 0.0);
  EXPECT_DOUBLE_EQ(geometry.regions[3].bounds.max.x, 0.0);

  const double slice_area = 20.0 * 16.0;
  for (std::size_t index = 0; index < 4U; ++index) {
    const auto & bounds = geometry.regions[index].bounds;
    const double area = (bounds.max.x - bounds.min.x) * (bounds.max.y - bounds.min.y);
    EXPECT_DOUBLE_EQ(area, 0.5 * slice_area);
  }

  const auto & ab = geometry.regions[0].bounds;
  const auto & bc = geometry.regions[1].bounds;
  const double adjacent_area =
    (std::min(ab.max.x, bc.max.x) - std::max(ab.min.x, bc.min.x)) *
    (std::min(ab.max.y, bc.max.y) - std::max(ab.min.y, bc.min.y));
  EXPECT_DOUBLE_EQ(adjacent_area, 0.25 * slice_area);
}

TEST(MissionConfig, ExactMultipleProducesUniformLevels)
{
  std::string exact = kValidConfig;
  exact.replace(exact.find("[10.0, 8.0, 5.0]"), std::string("[10.0, 8.0, 5.0]").size(),
    "[10.0, 8.0, 4.0]");
  const auto geometry = task_lib::BuildMissionGeometry(
    task_lib::LoadMissionConfig(WriteConfig("exact", exact)));
  ASSERT_EQ(geometry.levels.size(), 2U);
  EXPECT_DOUBLE_EQ(geometry.levels[0].z_max - geometry.levels[0].z_min, 2.0);
  EXPECT_DOUBLE_EQ(geometry.levels[1].z_max - geometry.levels[1].z_min, 2.0);
}

TEST(MissionConfig, DerivesHardFlightVolumeFromThreeAxisHysteresis)
{
  const auto config = task_lib::LoadMissionConfig(WriteConfig("hard", kValidConfig));
  const auto geometry = task_lib::BuildMissionGeometry(config);
  EXPECT_DOUBLE_EQ(geometry.hard_flight_volume.min.x, -12.0);
  EXPECT_DOUBLE_EQ(geometry.hard_flight_volume.min.y, -9.0);
  EXPECT_DOUBLE_EQ(geometry.hard_flight_volume.min.z, -0.5);
  EXPECT_DOUBLE_EQ(geometry.hard_flight_volume.max.x, 12.0);
  EXPECT_DOUBLE_EQ(geometry.hard_flight_volume.max.y, 9.0);
  EXPECT_DOUBLE_EQ(geometry.hard_flight_volume.max.z, 5.5);
  EXPECT_NE(geometry.config_revision, 0U);
}

TEST(MissionConfig, RejectsDuplicateDroneAndInvalidGeometry)
{
  std::string duplicate = kValidConfig;
  duplicate.replace(duplicate.find("[1, 2]"), 6, "[1, 1]");
  EXPECT_THROW(task_lib::LoadMissionConfig(WriteConfig("duplicate", duplicate)),
    std::invalid_argument);

  std::string inverted = kValidConfig;
  inverted.replace(inverted.find("[10.0, 8.0, 5.0]"),
    std::string("[10.0, 8.0, 5.0]").size(), "[-10.0, 8.0, 5.0]");
  EXPECT_THROW(task_lib::LoadMissionConfig(WriteConfig("inverted", inverted)),
    std::invalid_argument);

  std::string negative_hysteresis = kValidConfig;
  negative_hysteresis.replace(
    negative_hysteresis.find("[2.0, 1.0, 0.5]"),
    std::string("[2.0, 1.0, 0.5]").size(), "[-1.0, 1.0, 0.5]");
  EXPECT_THROW(
    task_lib::LoadMissionConfig(WriteConfig("negative_hysteresis", negative_hysteresis)),
    std::invalid_argument);
}

TEST(MissionConfig, RoiShorterThanLevelHeightProducesOneLevel)
{
  std::string short_roi = kValidConfig;
  short_roi.replace(short_roi.find("[10.0, 8.0, 5.0]"),
    std::string("[10.0, 8.0, 5.0]").size(), "[10.0, 8.0, 1.0]");
  const auto geometry = task_lib::BuildMissionGeometry(
    task_lib::LoadMissionConfig(WriteConfig("short", short_roi)));
  ASSERT_EQ(geometry.levels.size(), 1U);
  EXPECT_DOUBLE_EQ(geometry.levels[0].z_max, 1.0);
  EXPECT_EQ(geometry.regions.size(), 4U);
}

}  // namespace
