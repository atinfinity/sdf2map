// Copyright 2026 atinfinity
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
constexpr const char kWorld[] =
  R"(<?xml version="1.0"?>
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

namespace
{

std::filesystem::path WriteTempWorld(
  const std::string & name, const std::string & content)
{
  const auto path = std::filesystem::temp_directory_path() / name;
  std::ofstream out(path);
  out << content;
  return path;
}

constexpr const char kTwoBoxWorld[] =
  R"(<?xml version="1.0"?>
<sdf version="1.9">
  <world name="t">
    <model name="keep_box">
      <static>true</static>
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>1 1 1</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>1 1 1</size></box></geometry></visual>
      </link>
    </model>
    <model name="person_walking">
      <static>true</static>
      <pose>5 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>1 1 1</size></box></geometry></collision>
        <visual name="v">
          <transparency>1.0</transparency>
          <geometry><box><size>1 1 1</size></box></geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
)";

size_t CountNear(const sdf2map::CloudXYZ & cloud, float x0)
{
  size_t n = 0;
  for (const auto & p : cloud) {
    n += std::abs(p.x - x0) < 1.0 ? 1 : 0;
  }
  return n;
}

}  // namespace

TEST(WorldSampler, ExcludeByNamePattern)
{
  const auto path = WriteTempWorld("sdf2map_exclude_test.sdf", kTwoBoxWorld);
  sdf2map::Options opts;
  opts.input = path.string();
  opts.density = 200.0;
  opts.exclude_patterns = {"person.*"};
  sdf2map::SampleStats stats;
  const auto cloud = sdf2map::SampleWorld(opts, stats);
  std::filesystem::remove(path);

  EXPECT_EQ(stats.excluded_models, 1u);
  EXPECT_GT(CountNear(*cloud, 0.0f), 500u);
  EXPECT_EQ(CountNear(*cloud, 5.0f), 0u);
}

TEST(WorldSampler, TransparentVisualSkipped)
{
  const auto path = WriteTempWorld("sdf2map_transp_test.sdf", kTwoBoxWorld);
  sdf2map::Options opts;
  opts.input = path.string();
  opts.density = 200.0;
  opts.use_visual = true;
  sdf2map::SampleStats stats;
  const auto cloud = sdf2map::SampleWorld(opts, stats);

  EXPECT_GT(CountNear(*cloud, 0.0f), 500u);
  EXPECT_EQ(CountNear(*cloud, 5.0f), 0u);  // transparency 1.0 -> skipped
  EXPECT_EQ(stats.skipped, 1u);

  // Collision mode is unaffected by transparency
  sdf2map::Options copts = opts;
  copts.use_visual = false;
  sdf2map::SampleStats cstats;
  const auto ccloud = sdf2map::SampleWorld(copts, cstats);
  std::filesystem::remove(path);
  EXPECT_GT(CountNear(*ccloud, 5.0f), 500u);
}

TEST(WorldSampler, HeightmapConeHill)
{
  // cone_hill.png: normalized height 1 - r/R, radial. size 10x10x2 ->
  // z(r) = 2 * (1 - r/5) for r < 5 m
  const std::string world =
    std::string(
    R"(<?xml version="1.0"?>
<sdf version="1.9">
  <world name="t">
    <model name="terrain">
      <static>true</static>
      <link name="link">
        <collision name="c">
          <geometry>
            <heightmap>
              <uri>)")
    +
    TEST_WORLDS_DIR +
    R"(/heightmaps/cone_hill.png</uri>
              <size>10 10 2</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
)";
  const auto path = WriteTempWorld("sdf2map_heightmap_test.sdf", world);
  sdf2map::Options opts;
  opts.input = path.string();
  opts.density = 100.0;
  sdf2map::SampleStats stats;
  const auto cloud = sdf2map::SampleWorld(opts, stats);
  std::filesystem::remove(path);

  ASSERT_GT(cloud->size(), 5000u);
  EXPECT_EQ(stats.geometries, 1u);
  float max_z = 0;
  for (const auto & p : *cloud) {
    ASSERT_LE(std::abs(p.x), 5.001f);
    ASSERT_LE(std::abs(p.y), 5.001f);
    const double r = std::hypot(p.x, p.y);
    const double expected = 2.0 * std::max(0.0, 1.0 - r / 5.0);
    ASSERT_NEAR(p.z, expected, 0.06);
    max_z = std::max(max_z, p.z);
  }
  EXPECT_GT(max_z, 1.8f);  // the summit is represented
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
