#include "multidron_gui_lib/fiducial_config_loader.hpp"

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>

TEST(FiducialConfigLoader, LoadsBoxAndEnabledTags)
{
  const std::string path = "/tmp/multidron_gui_fiducial_test.yaml";
  {
    std::ofstream file(path);
    file <<
      "schema_version: 1\n"
      "objects:\n"
      "  - object_id: 9\n"
      "    shape: box\n"
      "    size_m: {x: 0.4, y: 0.5, z: 0.6}\n"
      "    world_T_object:\n"
      "      translation_m: [1.0, 2.0, 3.0]\n"
      "      rotation_rpy_deg: [0.0, 0.0, 90.0]\n"
      "    faces:\n"
      "      pos_x: {enabled: true, tag_id: 91}\n"
      "      neg_x: {enabled: false, tag_id: 92}\n";
  }

  multidron_gui_lib::FiducialVector objects;
  std::string error;
  ASSERT_TRUE(multidron_gui_lib::FiducialConfigLoader::Load(path, &objects, &error)) << error;
  ASSERT_EQ(objects.size(), 1U);
  EXPECT_EQ(objects.front().object_id, 9);
  EXPECT_EQ(objects.front().tag_ids.size(), 1U);
  EXPECT_EQ(objects.front().tag_ids.front(), 91);
  EXPECT_NEAR(objects.front().position.x(), 1.0F, 1e-6F);
  EXPECT_NEAR(objects.front().position.z(), 3.0F, 1e-6F);
  std::remove(path.c_str());
}
