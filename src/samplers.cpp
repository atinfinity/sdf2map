#include "sdf2map/samplers.hpp"

#include <array>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <vector>

#include <gz/common/Image.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/SubMesh.hh>
#include <sdf/Box.hh>
#include <sdf/Capsule.hh>
#include <sdf/Cone.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Element.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Heightmap.hh>
#include <sdf/Mesh.hh>
#include <sdf/Plane.hh>
#include <sdf/Polyline.hh>
#include <sdf/Sphere.hh>

namespace sdf2map
{

Sampler::Sampler(
  double density, unsigned int seed, const ResourceResolver * resolver)
: density_(density), rng_(seed), resolver_(resolver)
{
}

bool Sampler::SampleGeometry(
  const sdf::Geometry & geom, const gz::math::Pose3d & world_pose,
  CloudXYZ & out)
{
  switch (geom.Type()) {
    case sdf::GeometryType::BOX:
      SampleBox(geom.BoxShape()->Size(), world_pose, out);
      return true;
    case sdf::GeometryType::CYLINDER:
      SampleCylinder(
        geom.CylinderShape()->Radius(), geom.CylinderShape()->Length(),
        world_pose, out);
      return true;
    case sdf::GeometryType::SPHERE:
      SampleSphere(geom.SphereShape()->Radius(), world_pose, out);
      return true;
    case sdf::GeometryType::CAPSULE:
      SampleCapsule(
        geom.CapsuleShape()->Radius(), geom.CapsuleShape()->Length(),
        world_pose, out);
      return true;
    case sdf::GeometryType::ELLIPSOID:
      SampleEllipsoid(geom.EllipsoidShape()->Radii(), world_pose, out);
      return true;
    case sdf::GeometryType::PLANE:
      SamplePlane(
        geom.PlaneShape()->Normal(), geom.PlaneShape()->Size(),
        world_pose, out);
      return true;
    case sdf::GeometryType::CONE:
      SampleCone(
        geom.ConeShape()->Radius(), geom.ConeShape()->Length(),
        world_pose, out);
      return true;
    case sdf::GeometryType::POLYLINE:
      SamplePolyline(geom, world_pose, out);
      return true;
    case sdf::GeometryType::MESH:
      return SampleMesh(geom, world_pose, out);
    case sdf::GeometryType::HEIGHTMAP:
      return SampleHeightmap(geom, world_pose, out);
    default:
      return false;
  }
}

size_t Sampler::CountFor(double area) const
{
  return static_cast<size_t>(std::ceil(std::max(area, 0.0) * density_));
}

double Sampler::Uniform(double lo, double hi)
{
  return std::uniform_real_distribution<double>(lo, hi)(rng_);
}

void Sampler::AddPoint(
  const gz::math::Pose3d & pose, const gz::math::Vector3d & local,
  CloudXYZ & out)
{
  const auto w = pose.Rot().RotateVector(local) + pose.Pos();
  out.emplace_back(
    static_cast<float>(w.X()), static_cast<float>(w.Y()),
    static_cast<float>(w.Z()));
}

void Sampler::SampleRect(
  double sx, double sy, const gz::math::Pose3d & rect_pose,
  const gz::math::Pose3d & pose, CloudXYZ & out)
{
  const size_t n = CountFor(sx * sy);
  for (size_t i = 0; i < n; ++i) {
    gz::math::Vector3d p(
      Uniform(-sx / 2, sx / 2), Uniform(-sy / 2, sy / 2), 0.0);
    AddPoint(pose, rect_pose.Rot().RotateVector(p) + rect_pose.Pos(), out);
  }
}

void Sampler::SampleBox(
  const gz::math::Vector3d & size, const gz::math::Pose3d & pose,
  CloudXYZ & out)
{
  using gz::math::Pose3d;
  using gz::math::Vector3d;
  const double hx = size.X() / 2, hy = size.Y() / 2, hz = size.Z() / 2;
  const double half_pi = GZ_PI / 2;
  // +z / -z faces
  SampleRect(size.X(), size.Y(), Pose3d(0, 0, hz, 0, 0, 0), pose, out);
  SampleRect(size.X(), size.Y(), Pose3d(0, 0, -hz, GZ_PI, 0, 0), pose, out);
  // +x / -x faces (rotate local z to +/-x)
  SampleRect(size.Z(), size.Y(), Pose3d(hx, 0, 0, 0, half_pi, 0), pose, out);
  SampleRect(size.Z(), size.Y(), Pose3d(-hx, 0, 0, 0, -half_pi, 0), pose, out);
  // +y / -y faces
  SampleRect(size.X(), size.Z(), Pose3d(0, hy, 0, -half_pi, 0, 0), pose, out);
  SampleRect(size.X(), size.Z(), Pose3d(0, -hy, 0, half_pi, 0, 0), pose, out);
}

void Sampler::SampleCylinder(
  double radius, double length, const gz::math::Pose3d & pose, CloudXYZ & out)
{
  // Lateral surface
  const size_t n_side = CountFor(2 * GZ_PI * radius * length);
  for (size_t i = 0; i < n_side; ++i) {
    const double th = Uniform(0, 2 * GZ_PI);
    AddPoint(
      pose,
      {radius * std::cos(th), radius * std::sin(th),
        Uniform(-length / 2, length / 2)},
      out);
  }
  // Two caps
  const size_t n_cap = CountFor(GZ_PI * radius * radius);
  for (double sign : {1.0, -1.0}) {
    for (size_t i = 0; i < n_cap; ++i) {
      const double th = Uniform(0, 2 * GZ_PI);
      const double r = radius * std::sqrt(Uniform(0, 1));
      AddPoint(
        pose, {r * std::cos(th), r * std::sin(th), sign * length / 2}, out);
    }
  }
}

void Sampler::SampleSphere(
  double radius, const gz::math::Pose3d & pose, CloudXYZ & out)
{
  const size_t n = CountFor(4 * GZ_PI * radius * radius);
  std::normal_distribution<double> gauss(0.0, 1.0);
  for (size_t i = 0; i < n; ++i) {
    gz::math::Vector3d d(gauss(rng_), gauss(rng_), gauss(rng_));
    if (d.Length() < 1e-9) {
      continue;
    }
    AddPoint(pose, d.Normalized() * radius, out);
  }
}

void Sampler::SampleCapsule(
  double radius, double length, const gz::math::Pose3d & pose, CloudXYZ & out)
{
  // Lateral cylinder part
  const size_t n_side = CountFor(2 * GZ_PI * radius * length);
  for (size_t i = 0; i < n_side; ++i) {
    const double th = Uniform(0, 2 * GZ_PI);
    AddPoint(
      pose,
      {radius * std::cos(th), radius * std::sin(th),
        Uniform(-length / 2, length / 2)},
      out);
  }
  // Two hemispherical ends, sampled as one full sphere split at z=0
  const size_t n_sphere = CountFor(4 * GZ_PI * radius * radius);
  std::normal_distribution<double> gauss(0.0, 1.0);
  for (size_t i = 0; i < n_sphere; ++i) {
    gz::math::Vector3d d(gauss(rng_), gauss(rng_), gauss(rng_));
    if (d.Length() < 1e-9) {
      continue;
    }
    d = d.Normalized() * radius;
    d.Z(d.Z() + (d.Z() >= 0 ? length / 2 : -length / 2));
    AddPoint(pose, d, out);
  }
}

void Sampler::SampleEllipsoid(
  const gz::math::Vector3d & radii, const gz::math::Pose3d & pose,
  CloudXYZ & out)
{
  const double a = radii.X(), b = radii.Y(), c = radii.Z();
  // Thomsen's approximation of the ellipsoid surface area
  const double p = 1.6075;
  const double area = 4 * GZ_PI *
    std::pow(
    (std::pow(a * b, p) + std::pow(b * c, p) + std::pow(a * c, p)) / 3.0,
    1.0 / p);
  const size_t n = CountFor(area);
  // Rejection sampling: sample the unit sphere, scale, and accept with
  // probability proportional to the local area stretch factor so the
  // result stays uniform on the ellipsoid surface.
  const double max_factor =
    std::max({b * c, a * c, a * b});
  std::normal_distribution<double> gauss(0.0, 1.0);
  size_t added = 0;
  size_t guard = 0;
  while (added < n && guard < n * 20) {
    ++guard;
    gz::math::Vector3d d(gauss(rng_), gauss(rng_), gauss(rng_));
    if (d.Length() < 1e-9) {
      continue;
    }
    d.Normalize();
    const double factor = std::sqrt(
      std::pow(b * c * d.X(), 2) + std::pow(a * c * d.Y(), 2) +
      std::pow(a * b * d.Z(), 2));
    if (Uniform(0, 1) * max_factor > factor) {
      continue;
    }
    AddPoint(pose, {a * d.X(), b * d.Y(), c * d.Z()}, out);
    ++added;
  }
}

void Sampler::SamplePlane(
  const gz::math::Vector3d & normal, const gz::math::Vector2d & size,
  const gz::math::Pose3d & pose, CloudXYZ & out)
{
  gz::math::Quaterniond rot;
  rot.SetFrom2Axes(gz::math::Vector3d::UnitZ, normal.Normalized());
  SampleRect(
    size.X(), size.Y(), gz::math::Pose3d(gz::math::Vector3d::Zero, rot),
    pose, out);
}

void Sampler::SampleCone(
  double radius, double length, const gz::math::Pose3d & pose, CloudXYZ & out)
{
  // Apex at +length/2, base disc at -length/2 (SDF convention)
  const double slant = std::sqrt(radius * radius + length * length);
  const size_t n_side = CountFor(GZ_PI * radius * slant);
  for (size_t i = 0; i < n_side; ++i) {
    // Uniform surface density: pdf over t (0=base, 1=apex) proportional
    // to the local radius (1-t)
    const double t = 1.0 - std::sqrt(Uniform(0, 1));
    const double r = radius * (1.0 - t);
    const double th = Uniform(0, 2 * GZ_PI);
    AddPoint(
      pose,
      {r * std::cos(th), r * std::sin(th), -length / 2 + t * length}, out);
  }
  const size_t n_base = CountFor(GZ_PI * radius * radius);
  for (size_t i = 0; i < n_base; ++i) {
    const double th = Uniform(0, 2 * GZ_PI);
    const double r = radius * std::sqrt(Uniform(0, 1));
    AddPoint(pose, {r * std::cos(th), r * std::sin(th), -length / 2}, out);
  }
}

namespace
{

/// Ear-clipping triangulation of a simple (hole-free) polygon.
/// Returns index triples into pts.
std::vector<std::array<size_t, 3>> Triangulate(
  std::vector<gz::math::Vector2d> pts)
{
  std::vector<std::array<size_t, 3>> tris;
  // Ensure counter-clockwise order
  double signed_area = 0;
  for (size_t i = 0; i < pts.size(); ++i) {
    const auto & a = pts[i];
    const auto & b = pts[(i + 1) % pts.size()];
    signed_area += a.X() * b.Y() - b.X() * a.Y();
  }
  std::vector<size_t> idx(pts.size());
  for (size_t i = 0; i < idx.size(); ++i) {
    idx[i] = signed_area >= 0 ? i : pts.size() - 1 - i;
  }

  auto cross = [&pts](size_t a, size_t b, size_t c) {
      return (pts[b].X() - pts[a].X()) * (pts[c].Y() - pts[a].Y()) -
             (pts[b].Y() - pts[a].Y()) * (pts[c].X() - pts[a].X());
    };
  auto inside = [&pts, &cross](size_t a, size_t b, size_t c, size_t p) {
      return cross(a, b, p) >= 0 && cross(b, c, p) >= 0 &&
             cross(c, a, p) >= 0;
    };

  size_t guard = 0;
  while (idx.size() > 3 && guard++ < 10000) {
    bool clipped = false;
    for (size_t i = 0; i < idx.size(); ++i) {
      const size_t prev = idx[(i + idx.size() - 1) % idx.size()];
      const size_t cur = idx[i];
      const size_t next = idx[(i + 1) % idx.size()];
      if (cross(prev, cur, next) <= 0) {
        continue;  // reflex vertex
      }
      bool ear = true;
      for (size_t p : idx) {
        if (p != prev && p != cur && p != next &&
          inside(prev, cur, next, p))
        {
          ear = false;
          break;
        }
      }
      if (ear) {
        tris.push_back({prev, cur, next});
        idx.erase(idx.begin() + i);
        clipped = true;
        break;
      }
    }
    if (!clipped) {
      break;  // degenerate polygon; keep what we have
    }
  }
  if (idx.size() == 3) {
    tris.push_back({idx[0], idx[1], idx[2]});
  }
  return tris;
}

}  // namespace

void Sampler::SamplePolyline(
  const sdf::Geometry & geom, const gz::math::Pose3d & pose, CloudXYZ & out)
{
  for (const sdf::Polyline & poly : geom.PolylineShape()) {
    const double height = poly.Height();
    const auto & pts = poly.Points();
    if (pts.size() < 3) {
      continue;
    }

    // Extruded side walls (the outline is closed)
    for (size_t i = 0; i < pts.size(); ++i) {
      const auto & a = pts[i];
      const auto & b = pts[(i + 1) % pts.size()];
      const double len = (b - a).Length();
      const size_t n = CountFor(len * height);
      for (size_t k = 0; k < n; ++k) {
        const double t = Uniform(0, 1);
        AddPoint(
          pose,
          {a.X() + t * (b.X() - a.X()), a.Y() + t * (b.Y() - a.Y()),
            Uniform(0, height)},
          out);
      }
    }

    // Top and bottom caps via triangulation
    const auto tris = Triangulate(pts);
    for (const auto & tri : tris) {
      const auto & a = pts[tri[0]];
      const auto & b = pts[tri[1]];
      const auto & c = pts[tri[2]];
      const double area = 0.5 * std::abs(
        (b.X() - a.X()) * (c.Y() - a.Y()) -
        (b.Y() - a.Y()) * (c.X() - a.X()));
      for (double z : {0.0, height}) {
        const size_t n = CountFor(area);
        for (size_t k = 0; k < n; ++k) {
          double u = Uniform(0, 1), v = Uniform(0, 1);
          if (u + v > 1.0) {
            u = 1.0 - u;
            v = 1.0 - v;
          }
          AddPoint(
            pose,
            {a.X() + u * (b.X() - a.X()) + v * (c.X() - a.X()),
              a.Y() + u * (b.Y() - a.Y()) + v * (c.Y() - a.Y()), z},
            out);
        }
      }
    }
  }
}

bool Sampler::SampleHeightmap(
  const sdf::Geometry & geom, const gz::math::Pose3d & pose, CloudXYZ & out)
{
  const sdf::Heightmap * hm = geom.HeightmapShape();
  std::string declaring_dir;
  if (!hm->FilePath().empty()) {
    declaring_dir =
      std::filesystem::path(hm->FilePath()).parent_path().string();
  }
  const std::string path = resolver_->Resolve(hm->Uri(), declaring_dir);
  if (path.empty()) {
    std::cerr << "[sdf2map] WARNING: could not resolve heightmap URI: "
              << hm->Uri() << std::endl;
    return false;
  }

  gz::common::Image image;
  if (image.Load(path) != 0 || image.Width() < 2 || image.Height() < 2) {
    std::cerr << "[sdf2map] WARNING: failed to load heightmap image "
              << "(DEM/GeoTIFF is not supported yet): " << path << std::endl;
    return false;
  }

  const unsigned int w = image.Width();
  const unsigned int h = image.Height();
  const gz::math::Vector3d size = hm->Size();
  const gz::math::Vector3d offset = hm->Position();

  // Normalized elevation grid; image row 0 maps to +y (viewed from above)
  std::vector<double> z_grid(static_cast<size_t>(w) * h);
  for (unsigned int r = 0; r < h; ++r) {
    for (unsigned int c = 0; c < w; ++c) {
      z_grid[static_cast<size_t>(r) * w + c] =
        image.Pixel(c, r).R() * size.Z();
    }
  }
  const double cell_x = size.X() / (w - 1);
  const double cell_y = size.Y() / (h - 1);

  // Weight cells by their true (sloped) surface area
  std::vector<double> weights;
  weights.reserve(static_cast<size_t>(w - 1) * (h - 1));
  double total_area = 0;
  for (unsigned int r = 0; r + 1 < h; ++r) {
    for (unsigned int c = 0; c + 1 < w; ++c) {
      const double z00 = z_grid[static_cast<size_t>(r) * w + c];
      const double z10 = z_grid[static_cast<size_t>(r) * w + c + 1];
      const double z01 = z_grid[static_cast<size_t>(r + 1) * w + c];
      const double dzdx = (z10 - z00) / cell_x;
      const double dzdy = (z01 - z00) / cell_y;
      const double area = cell_x * cell_y *
        std::sqrt(1.0 + dzdx * dzdx + dzdy * dzdy);
      weights.push_back(area);
      total_area += area;
    }
  }
  std::discrete_distribution<size_t> pick(weights.begin(), weights.end());

  const size_t n = CountFor(total_area);
  for (size_t i = 0; i < n; ++i) {
    const size_t cell = pick(rng_);
    const unsigned int r = cell / (w - 1);
    const unsigned int c = cell % (w - 1);
    const double fx = Uniform(0, 1), fy = Uniform(0, 1);
    // Bilinear interpolation inside the cell
    const double z00 = z_grid[static_cast<size_t>(r) * w + c];
    const double z10 = z_grid[static_cast<size_t>(r) * w + c + 1];
    const double z01 = z_grid[static_cast<size_t>(r + 1) * w + c];
    const double z11 = z_grid[static_cast<size_t>(r + 1) * w + c + 1];
    const double z = z00 * (1 - fx) * (1 - fy) + z10 * fx * (1 - fy) +
      z01 * (1 - fx) * fy + z11 * fx * fy;
    // Image row 0 = +y edge, growing r moves toward -y
    const double x = -size.X() / 2 + (c + fx) * cell_x;
    const double y = size.Y() / 2 - (r + fy) * cell_y;
    AddPoint(pose, offset + gz::math::Vector3d(x, y, z), out);
  }
  return true;
}

bool Sampler::SampleMesh(
  const sdf::Geometry & geom, const gz::math::Pose3d & pose, CloudXYZ & out)
{
  const sdf::Mesh * mesh_sdf = geom.MeshShape();
  // Relative mesh URIs are relative to the SDF file that declared them
  // (e.g. an included Fuel model's model.sdf), not to the world file.
  std::string declaring_dir;
  if (!mesh_sdf->FilePath().empty()) {
    declaring_dir =
      std::filesystem::path(mesh_sdf->FilePath()).parent_path().string();
  }
  const std::string path =
    resolver_->Resolve(mesh_sdf->Uri(), declaring_dir);
  if (path.empty()) {
    std::cerr << "[sdf2map] WARNING: could not resolve mesh URI: "
              << mesh_sdf->Uri() << std::endl;
    return false;
  }
  auto * manager = gz::common::MeshManager::Instance();
  const gz::common::Mesh * mesh = manager->Load(path);
  if (!mesh) {
    std::cerr << "[sdf2map] WARNING: failed to load mesh: " << path
              << std::endl;
    return false;
  }

  const gz::math::Vector3d scale = mesh_sdf->Scale();
  const std::string submesh_filter = mesh_sdf->Submesh();

  struct Tri
  {
    gz::math::Vector3d v0, v1, v2;
    double area;
  };
  std::vector<Tri> tris;
  double total_area = 0.0;

  for (unsigned int s = 0; s < mesh->SubMeshCount(); ++s) {
    auto submesh = mesh->SubMeshByIndex(s).lock();
    if (!submesh) {
      continue;
    }
    if (!submesh_filter.empty() && submesh->Name() != submesh_filter) {
      continue;
    }
    if (submesh->SubMeshPrimitiveType() != gz::common::SubMesh::TRIANGLES) {
      continue;
    }
    for (unsigned int i = 0; i + 2 < submesh->IndexCount(); i += 3) {
      Tri t;
      t.v0 = submesh->Vertex(submesh->Index(i)) * scale;
      t.v1 = submesh->Vertex(submesh->Index(i + 1)) * scale;
      t.v2 = submesh->Vertex(submesh->Index(i + 2)) * scale;
      t.area = 0.5 * (t.v1 - t.v0).Cross(t.v2 - t.v0).Length();
      if (t.area > 1e-12) {
        total_area += t.area;
        tris.push_back(t);
      }
    }
  }
  if (tris.empty()) {
    std::cerr << "[sdf2map] WARNING: mesh has no triangles: " << path
              << std::endl;
    return false;
  }

  std::vector<double> weights;
  weights.reserve(tris.size());
  for (const auto & t : tris) {
    weights.push_back(t.area);
  }
  std::discrete_distribution<size_t> pick(weights.begin(), weights.end());

  const size_t n = CountFor(total_area);
  for (size_t i = 0; i < n; ++i) {
    const Tri & t = tris[pick(rng_)];
    // Uniform point in a triangle via barycentric coordinates
    double u = Uniform(0, 1), v = Uniform(0, 1);
    if (u + v > 1.0) {
      u = 1.0 - u;
      v = 1.0 - v;
    }
    AddPoint(pose, t.v0 + (t.v1 - t.v0) * u + (t.v2 - t.v0) * v, out);
  }
  return true;
}

}  // namespace sdf2map
