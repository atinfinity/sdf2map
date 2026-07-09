#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include <sdf/Box.hh>
#include <sdf/Cone.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Geometry.hh>
#include <sdf/Polyline.hh>
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

TEST(Samplers, ConePointsOnSurface)
{
  sdf::Geometry geom;
  geom.SetType(sdf::GeometryType::CONE);
  sdf::Cone cone;
  cone.SetRadius(0.5);
  cone.SetLength(1.0);
  geom.SetConeShape(cone);

  const auto cloud = Sample(geom, gz::math::Pose3d::Zero);
  ASSERT_GT(cloud.size(), 500u);

  for (const auto & p : cloud) {
    const double radial = std::hypot(p.x, p.y);
    ASSERT_GE(p.z, -0.5 - kEps);
    ASSERT_LE(p.z, 0.5 + kEps);
    const bool base = std::abs(p.z + 0.5) < 1e-5 && radial <= 0.5 + kEps;
    // Lateral surface: r = 0.5 * (1 - t), t = (z + 0.5) / 1.0
    const double expected_r = 0.5 * (1.0 - (p.z + 0.5));
    const bool lateral = std::abs(radial - expected_r) < 1e-5;
    ASSERT_TRUE(base || lateral);
  }
}

TEST(Samplers, PolylineExtrusion)
{
  sdf::Geometry geom;
  geom.SetType(sdf::GeometryType::POLYLINE);
  sdf::Polyline poly;
  poly.SetHeight(2.0);
  // Non-convex L-shape
  poly.AddPoint({0.0, 0.0});
  poly.AddPoint({2.0, 0.0});
  poly.AddPoint({2.0, 1.0});
  poly.AddPoint({1.0, 1.0});
  poly.AddPoint({1.0, 2.0});
  poly.AddPoint({0.0, 2.0});
  geom.SetPolylineShape({poly});

  const auto cloud = Sample(geom, gz::math::Pose3d::Zero);
  ASSERT_GT(cloud.size(), 1000u);

  size_t caps = 0;
  for (const auto & p : cloud) {
    ASSERT_GE(p.z, -kEps);
    ASSERT_LE(p.z, 2.0 + kEps);
    // Inside the L-shape outline (with tolerance): the notch
    // x>1, y>1 must stay empty
    ASSERT_LE(p.x, 2.0 + kEps);
    ASSERT_LE(p.y, 2.0 + kEps);
    ASSERT_GE(p.x, -kEps);
    ASSERT_GE(p.y, -kEps);
    ASSERT_FALSE(p.x > 1.0 + 1e-5 && p.y > 1.0 + 1e-5)
      << "point in the concave notch: " << p.x << ", " << p.y;
    if (std::abs(p.z - 2.0) < 1e-6 || std::abs(p.z) < 1e-6) {
      ++caps;
    }
  }
  // Caps (2 x 3 m^2) should carry a large share of the points
  EXPECT_GT(caps, cloud.size() / 4);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
