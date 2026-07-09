#ifndef SDF2MAP__SAMPLERS_HPP_
#define SDF2MAP__SAMPLERS_HPP_

#include <random>
#include <string>

#include <gz/math/Pose3.hh>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sdf/Geometry.hh>

#include "sdf2map/resource_resolver.hpp"

namespace sdf2map
{

using CloudXYZ = pcl::PointCloud<pcl::PointXYZ>;

/// Samples points uniformly on the surface of SDF geometries.
class Sampler
{
public:
  Sampler(double density, unsigned int seed, const ResourceResolver * resolver);

  /// Sample the given geometry, transform the points by world_pose and
  /// append them to out. Returns false if the geometry type is unsupported
  /// or the mesh could not be loaded.
  bool SampleGeometry(
    const sdf::Geometry & geom, const gz::math::Pose3d & world_pose,
    CloudXYZ & out);

private:
  void SampleBox(const gz::math::Vector3d & size,
    const gz::math::Pose3d & pose, CloudXYZ & out);
  void SampleCylinder(double radius, double length,
    const gz::math::Pose3d & pose, CloudXYZ & out);
  void SampleSphere(double radius,
    const gz::math::Pose3d & pose, CloudXYZ & out);
  void SampleCapsule(double radius, double length,
    const gz::math::Pose3d & pose, CloudXYZ & out);
  void SampleEllipsoid(const gz::math::Vector3d & radii,
    const gz::math::Pose3d & pose, CloudXYZ & out);
  void SamplePlane(const gz::math::Vector3d & normal,
    const gz::math::Vector2d & size,
    const gz::math::Pose3d & pose, CloudXYZ & out);
  void SampleCone(double radius, double length,
    const gz::math::Pose3d & pose, CloudXYZ & out);
  void SamplePolyline(const sdf::Geometry & geom,
    const gz::math::Pose3d & pose, CloudXYZ & out);
  bool SampleMesh(const sdf::Geometry & geom,
    const gz::math::Pose3d & pose, CloudXYZ & out);
  bool SampleHeightmap(const sdf::Geometry & geom,
    const gz::math::Pose3d & pose, CloudXYZ & out);

  void SampleRect(double sx, double sy, const gz::math::Pose3d & rect_pose,
    const gz::math::Pose3d & pose, CloudXYZ & out);
  void AddPoint(const gz::math::Pose3d & pose,
    const gz::math::Vector3d & local, CloudXYZ & out);
  size_t CountFor(double area) const;
  double Uniform(double lo, double hi);

  double density_;
  std::mt19937 rng_;
  const ResourceResolver * resolver_;
};

}  // namespace sdf2map

#endif  // SDF2MAP__SAMPLERS_HPP_
