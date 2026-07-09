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
  void SampleBox(
    const gz::math::Vector3d & size,
    const gz::math::Pose3d & pose, CloudXYZ & out);
  void SampleCylinder(
    double radius, double length,
    const gz::math::Pose3d & pose, CloudXYZ & out);
  void SampleSphere(
    double radius,
    const gz::math::Pose3d & pose, CloudXYZ & out);
  void SampleCapsule(
    double radius, double length,
    const gz::math::Pose3d & pose, CloudXYZ & out);
  void SampleEllipsoid(
    const gz::math::Vector3d & radii,
    const gz::math::Pose3d & pose, CloudXYZ & out);
  void SamplePlane(
    const gz::math::Vector3d & normal,
    const gz::math::Vector2d & size,
    const gz::math::Pose3d & pose, CloudXYZ & out);
  void SampleCone(
    double radius, double length,
    const gz::math::Pose3d & pose, CloudXYZ & out);
  void SamplePolyline(
    const sdf::Geometry & geom,
    const gz::math::Pose3d & pose, CloudXYZ & out);
  bool SampleMesh(
    const sdf::Geometry & geom,
    const gz::math::Pose3d & pose, CloudXYZ & out);
  bool SampleHeightmap(
    const sdf::Geometry & geom,
    const gz::math::Pose3d & pose, CloudXYZ & out);

  void SampleRect(
    double sx, double sy, const gz::math::Pose3d & rect_pose,
    const gz::math::Pose3d & pose, CloudXYZ & out);
  void AddPoint(
    const gz::math::Pose3d & pose,
    const gz::math::Vector3d & local, CloudXYZ & out);
  size_t CountFor(double area) const;
  double Uniform(double lo, double hi);

  double density_;
  std::mt19937 rng_;
  const ResourceResolver * resolver_;
};

}  // namespace sdf2map

#endif  // SDF2MAP__SAMPLERS_HPP_
