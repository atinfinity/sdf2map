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

// End-to-end geometric verification: samples worlds/test_world.sdf and
// checks every object's point cluster against its analytically expected
// placement (ported from the original standalone verify.py).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>

#include "sdf2map/options.hpp"
#include "sdf2map/world_sampler.hpp"

namespace
{

class E2EGeometry : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    sdf2map::Options opts;
    opts.input = std::string(TEST_WORLDS_DIR) + "/test_world.sdf";
    opts.density = 400.0;
    sdf2map::SampleStats stats;
    cloud_ = sdf2map::SampleWorld(opts, stats);
    ASSERT_GT(cloud_->size(), 100000u);
  }

  /// Points with |x-cx|<hx, |y-cy|<hy, z in [z_lo, z_hi]
  static sdf2map::CloudXYZ Select(
    double cx, double cy, double hx, double hy,
    double z_lo = 0.02, double z_hi = 1e9)
  {
    sdf2map::CloudXYZ out;
    for (const auto & p : *cloud_) {
      if (std::abs(p.x - cx) < hx && std::abs(p.y - cy) < hy &&
        p.z > z_lo && p.z < z_hi)
      {
        out.push_back(p);
      }
    }
    return out;
  }

  static sdf2map::CloudXYZ::Ptr cloud_;
};

sdf2map::CloudXYZ::Ptr E2EGeometry::cloud_;

}  // namespace

TEST_F(E2EGeometry, PillarCylinder)
{
  // r=0.3, l=2 at (-2, 2, 1)
  const auto sel = Select(-2, 2, 0.5, 0.5, -0.05);
  ASSERT_GT(sel.size(), 500u);
  double max_r = 0, min_z = 1e9, max_z = -1e9;
  for (const auto & p : sel) {
    // Radius on the lateral band only; the window floor also catches
    // ground-plane points which would skew the radius
    if (std::abs(p.z - 1.0) < 0.9) {
      max_r = std::max(
        max_r, static_cast<double>(std::hypot(p.x + 2, p.y - 2)));
    }
    min_z = std::min(min_z, static_cast<double>(p.z));
    max_z = std::max(max_z, static_cast<double>(p.z));
  }
  EXPECT_NEAR(max_r, 0.3, 0.01);
  EXPECT_NEAR(min_z, 0.0, 0.05);
  EXPECT_NEAR(max_z, 2.0, 0.05);
}

TEST_F(E2EGeometry, BallSphere)
{
  // r=0.5 at (-2,-2,0.5)
  const auto sel = Select(-2, -2, 0.8, 0.8);
  ASSERT_GT(sel.size(), 500u);
  for (const auto & p : sel) {
    const double r = std::sqrt(
      std::pow(p.x + 2, 2) + std::pow(p.y + 2, 2) + std::pow(p.z - 0.5, 2));
    ASSERT_NEAR(r, 0.5, 0.01);
  }
}

TEST_F(E2EGeometry, BoxWallRotatedFootprint)
{
  // 4 x 0.2 x 1 at (3,0,0.5), yaw 30 deg; window excludes neighbours
  const auto sel = Select(3, 0, 2.0, 1.15);
  ASSERT_GT(sel.size(), 1000u);
  const double c = std::cos(-0.5236), s = std::sin(-0.5236);
  double max_z = -1e9;
  for (const auto & p : sel) {
    const double lx = c * (p.x - 3) - s * p.y;
    const double ly = s * (p.x - 3) + c * p.y;
    ASSERT_LT(std::abs(lx), 2.001);
    ASSERT_LT(std::abs(ly), 0.101);
    max_z = std::max(max_z, static_cast<double>(p.z));
  }
  EXPECT_NEAR(max_z, 1.0, 0.01);
}

TEST_F(E2EGeometry, PyramidMesh)
{
  // Base 1x1 at z=0.5, apex z=1.5, at (0,-3)
  const auto sel = Select(0, -3, 0.7, 0.7, 0.4);
  ASSERT_GT(sel.size(), 500u);
  double min_z = 1e9, max_z = -1e9;
  for (const auto & p : sel) {
    min_z = std::min(min_z, static_cast<double>(p.z));
    max_z = std::max(max_z, static_cast<double>(p.z));
    // Half-width shrinks linearly toward the apex
    const double hw = std::max(std::abs(p.x), std::abs(p.y + 3));
    ASSERT_LT(hw, 0.5 * (1 - (p.z - 0.5)) + 0.01);
  }
  EXPECT_NEAR(min_z, 0.5, 0.05);
  EXPECT_NEAR(max_z, 1.5, 0.06);
}

TEST_F(E2EGeometry, NestedShelfWithRelativeToFrame)
{
  // Model (2,3) yaw90: boxes 2x0.6x0.5; base spans z [0,0.5], top box via
  // frame relative_to spans z [0.75,1.25]; footprint rotated to x +-0.3,
  // y +-1.0
  const auto sel = Select(2, 3, 1.5, 1.5);
  ASSERT_GT(sel.size(), 1000u);
  double max_dx = 0, max_dy = 0, max_z = -1e9;
  size_t gap_points = 0;
  for (const auto & p : sel) {
    max_dx = std::max(max_dx, std::abs(p.x - 2.0));
    max_dy = std::max(max_dy, std::abs(p.y - 3.0));
    max_z = std::max(max_z, static_cast<double>(p.z));
    if (p.z > 0.55 && p.z < 0.70) {
      ++gap_points;
    }
  }
  EXPECT_NEAR(max_dx, 0.3, 0.01);
  EXPECT_NEAR(max_dy, 1.0, 0.01);
  EXPECT_NEAR(max_z, 1.25, 0.01);
  EXPECT_EQ(gap_points, 0u);
}

TEST_F(E2EGeometry, CapsuleRolled)
{
  // r=0.2, l=1 at (4,-2,0.8) roll 90: axis along y, z in [0.6,1.0]
  const auto sel = Select(4, -2, 0.5, 1.2, 0.3);
  ASSERT_GT(sel.size(), 500u);
  double min_z = 1e9, max_z = -1e9, max_dy = 0;
  for (const auto & p : sel) {
    min_z = std::min(min_z, static_cast<double>(p.z));
    max_z = std::max(max_z, static_cast<double>(p.z));
    max_dy = std::max(max_dy, std::abs(p.y + 2.0));
  }
  EXPECT_NEAR(min_z, 0.6, 0.02);
  EXPECT_NEAR(max_z, 1.0, 0.02);
  EXPECT_NEAR(max_dy, 0.7, 0.02);
}

TEST_F(E2EGeometry, EllipsoidSurfaceEquation)
{
  // radii 0.6 0.4 0.3 at (-4,0,0.3)
  const auto sel = Select(-4, 0, 0.8, 0.6);
  ASSERT_GT(sel.size(), 500u);
  for (const auto & p : sel) {
    const double q = std::pow((p.x + 4) / 0.6, 2) +
      std::pow(p.y / 0.4, 2) + std::pow((p.z - 0.3) / 0.3, 2);
    ASSERT_NEAR(q, 1.0, 0.05);
  }
}

TEST_F(E2EGeometry, OffsetRotatedCollision)
{
  // Regression for gz-math7 pose order: box 2x0.5x1, collision yaw90,
  // model at (5,5,0.5) -> world footprint x +-0.25, y +-1.0
  const auto sel = Select(5, 5, 1.5, 1.5);
  ASSERT_GT(sel.size(), 500u);
  for (const auto & p : sel) {
    ASSERT_LT(std::abs(p.x - 5.0), 0.251);
    ASSERT_LT(std::abs(p.y - 5.0), 1.001);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
