#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include <sdf/Box.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Geometry.hh>
#include <sdf/Sphere.hh>

#include "sdf2map/samplers.hpp"

namespace
{

constexpr double kDensity = 1000.0;  // points / m^2
constexpr double kEps = 1e-6;

sdf2map::CloudXYZ Sample(
  const sdf::Geometry & geom, const gz::math::Pose3d & pose)
{
  sdf2map::ResourceResolver resolver(".");
  sdf2map::Sampler sampler(kDensity, 42, &resolver);
  sdf2map::CloudXYZ cloud;
  EXPECT_TRUE(sampler.SampleGeometry(geom, pose, cloud));
  return cloud;
}

}  // namespace

TEST(Samplers, BoxPointsLieOnSurface)
{
  sdf::Geometry geom;
  geom.SetType(sdf::GeometryType::BOX);
  sdf::Box box;
  box.SetSize({2.0, 1.0, 0.5});
  geom.SetBoxShape(box);

  const auto cloud = Sample(geom, gz::math::Pose3d::Zero);
  const double expected_area = 2 * (2 * 1 + 1 * 0.5 + 2 * 0.5);
  EXPECT_NEAR(cloud.size(), expected_area * kDensity, expected_area * 10);

  for (const auto & p : cloud) {
    EXPECT_LE(std::abs(p.x), 1.0 + kEps);
    EXPECT_LE(std::abs(p.y), 0.5 + kEps);
    EXPECT_LE(std::abs(p.z), 0.25 + kEps);
    // At least one coordinate must be on a face
    const bool on_face = std::abs(std::abs(p.x) - 1.0) < kEps ||
      std::abs(std::abs(p.y) - 0.5) < kEps ||
      std::abs(std::abs(p.z) - 0.25) < kEps;
    EXPECT_TRUE(on_face);
  }
}

TEST(Samplers, SpherePointsAtRadius)
{
  sdf::Geometry geom;
  geom.SetType(sdf::GeometryType::SPHERE);
  sdf::Sphere sphere;
  sphere.SetRadius(0.7);
  geom.SetSphereShape(sphere);

  const gz::math::Pose3d pose(1.0, 2.0, 3.0, 0, 0, 0);
  const auto cloud = Sample(geom, pose);
  ASSERT_GT(cloud.size(), 100u);

  for (const auto & p : cloud) {
    const double r = gz::math::Vector3d(
      p.x - 1.0, p.y - 2.0, p.z - 3.0).Length();
    EXPECT_NEAR(r, 0.7, 1e-5);
  }
}

TEST(Samplers, CylinderPointsOnSurface)
{
  sdf::Geometry geom;
  geom.SetType(sdf::GeometryType::CYLINDER);
  sdf::Cylinder cyl;
  cyl.SetRadius(0.5);
  cyl.SetLength(2.0);
  geom.SetCylinderShape(cyl);

  // Rotate the cylinder axis onto world x
  const gz::math::Pose3d pose(0, 0, 0, 0, GZ_PI / 2, 0);
  const auto cloud = Sample(geom, pose);
  ASSERT_GT(cloud.size(), 100u);

  for (const auto & p : cloud) {
    // After rotation the axis is x, so radial distance is in the y-z plane
    const double radial = std::hypot(p.y, p.z);
    const bool lateral = std::abs(radial - 0.5) < 1e-5;
    const bool cap = std::abs(std::abs(p.x) - 1.0) < 1e-5 &&
      radial <= 0.5 + kEps;
    EXPECT_TRUE(lateral || cap);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
