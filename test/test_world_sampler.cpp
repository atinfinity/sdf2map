#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>

#include "sdf2map/options.hpp"
#include "sdf2map/world_sampler.hpp"

namespace
{

// A yaw-rotated collision under a translated model: if pose composition
// order is wrong, the model translation gets rotated and the box lands
// near (-5, 10) instead of (10, 5).
constexpr const char kWorld[] = R"(<?xml version="1.0"?>
<sdf version="1.9">
  <world name="t">
    <model name="offset_rotated">
      <static>true</static>
      <pose>10 5 0 0 0 0</pose>
      <link name="link">
        <pose>1 0 0.5 0 0 0</pose>
        <collision name="collision">
          <pose>0 0 0 0 0 1.5708</pose>
          <geometry>
            <box><size>4 0.5 1</size></box>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
)";

}  // namespace

TEST(WorldSampler, PoseCompositionIsParentTimesChild)
{
  const auto path =
    std::filesystem::temp_directory_path() / "sdf2map_pose_test.sdf";
  {
    std::ofstream out(path);
    out << kWorld;
  }

  sdf2map::Options opts;
  opts.input = path.string();
  opts.density = 500.0;
  sdf2map::SampleStats stats;
  const auto cloud = sdf2map::SampleWorld(opts, stats);
  std::filesystem::remove(path);

  ASSERT_GT(cloud->size(), 100u);
  EXPECT_EQ(stats.geometries, 1u);

  // Box 4 x 0.5 rotated 90 deg -> world footprint 0.5 x 4 centered at
  // (11, 5); z in [0, 1].
  float min_x = 1e9f, max_x = -1e9f, min_y = 1e9f, max_y = -1e9f;
  float min_z = 1e9f, max_z = -1e9f;
  for (const auto & p : *cloud) {
    min_x = std::min(min_x, p.x);
    max_x = std::max(max_x, p.x);
    min_y = std::min(min_y, p.y);
    max_y = std::max(max_y, p.y);
    min_z = std::min(min_z, p.z);
    max_z = std::max(max_z, p.z);
  }
  EXPECT_NEAR(min_x, 11.0 - 0.25, 0.01);
  EXPECT_NEAR(max_x, 11.0 + 0.25, 0.01);
  EXPECT_NEAR(min_y, 5.0 - 2.0, 0.01);
  EXPECT_NEAR(max_y, 5.0 + 2.0, 0.01);
  EXPECT_NEAR(min_z, 0.0, 0.01);
  EXPECT_NEAR(max_z, 1.0, 0.01);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
