#include "sdf2map/exporter.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

namespace fs = std::filesystem;

namespace sdf2map
{

std::size_t ExportPCD(const CloudXYZ::Ptr & cloud, const Options & opts)
{
  CloudXYZ::Ptr work = cloud;

  if (std::isfinite(opts.z_min) || std::isfinite(opts.z_max)) {
    auto cropped = std::make_shared<CloudXYZ>();
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(work);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(
      static_cast<float>(opts.z_min), static_cast<float>(opts.z_max));
    pass.filter(*cropped);
    work = cropped;
  }

  if (opts.voxel > 0.0) {
    auto filtered = std::make_shared<CloudXYZ>();
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(work);
    const float leaf = static_cast<float>(opts.voxel);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.filter(*filtered);
    work = filtered;
  }

  if (work->empty()) {
    throw std::runtime_error(
            "point cloud is empty after filtering; nothing to write");
  }

  int ret = opts.ascii_pcd ?
    pcl::io::savePCDFileASCII(opts.output_pcd, *work) :
    pcl::io::savePCDFileBinary(opts.output_pcd, *work);
  if (ret < 0) {
    throw std::runtime_error("failed to write PCD file: " + opts.output_pcd);
  }
  return work->size();
}

void ExportOccupancyGrid(const CloudXYZ & cloud, const Options & opts)
{
  if (cloud.empty()) {
    throw std::runtime_error("point cloud is empty; cannot build grid");
  }
  const double res = opts.grid_resolution;

  // Grid bounds from the xy extent of the full cloud (the ground plane
  // included, so floor area becomes free space) or, with
  // grid_bounds_slice, from the obstacle band only.
  bool first = true;
  float min_x = 0, max_x = 0, min_y = 0, max_y = 0;
  for (const auto & p : cloud) {
    if (opts.grid_bounds_slice &&
      (p.z < opts.slice_z_min || p.z > opts.slice_z_max))
    {
      continue;
    }
    if (first) {
      min_x = max_x = p.x;
      min_y = max_y = p.y;
      first = false;
      continue;
    }
    min_x = std::min(min_x, p.x);
    max_x = std::max(max_x, p.x);
    min_y = std::min(min_y, p.y);
    max_y = std::max(max_y, p.y);
  }
  if (first) {
    throw std::runtime_error(
            "no points in the occupancy slice band; check --slice-z-min/max");
  }
  const double margin =
    opts.grid_bounds_slice ? std::max(opts.grid_bounds_margin, res) : res;
  const double origin_x = min_x - margin;
  const double origin_y = min_y - margin;
  const int width =
    static_cast<int>(std::ceil((max_x + margin - origin_x) / res));
  const int height =
    static_cast<int>(std::ceil((max_y + margin - origin_y) / res));
  if (width <= 0 || height <= 0 ||
    static_cast<long>(width) * height > 100'000'000L)
  {
    throw std::runtime_error(
            "occupancy grid would be " + std::to_string(width) + "x" +
            std::to_string(height) + " cells; check --grid-resolution");
  }

  constexpr uint8_t kFree = 254;
  constexpr uint8_t kOccupied = 0;
  std::vector<uint8_t> grid(
    static_cast<size_t>(width) * height, kFree);

  size_t occupied_cells = 0;
  for (const auto & p : cloud) {
    if (p.z < opts.slice_z_min || p.z > opts.slice_z_max) {
      continue;
    }
    const int cx = static_cast<int>((p.x - origin_x) / res);
    const int cy = static_cast<int>((p.y - origin_y) / res);
    if (cx < 0 || cx >= width || cy < 0 || cy >= height) {
      continue;
    }
    uint8_t & cell = grid[static_cast<size_t>(cy) * width + cx];
    if (cell != kOccupied) {
      cell = kOccupied;
      ++occupied_cells;
    }
  }

  // Morphological closing (dilate then erode) to fill speckle holes the
  // stochastic sampling leaves inside obstacle footprints
  if (opts.grid_close > 0) {
    auto pass = [&](std::vector<uint8_t> & g, uint8_t from, uint8_t to) {
        std::vector<uint8_t> src = g;
        for (int y = 0; y < height; ++y) {
          for (int x = 0; x < width; ++x) {
            if (src[static_cast<size_t>(y) * width + x] != from) {
              continue;
            }
            bool hit = false;
            for (int dy = -1; dy <= 1 && !hit; ++dy) {
              for (int dx = -1; dx <= 1 && !hit; ++dx) {
                const int nx = x + dx, ny = y + dy;
                hit = nx >= 0 && nx < width && ny >= 0 && ny < height &&
                  src[static_cast<size_t>(ny) * width + nx] == to;
              }
            }
            if (hit) {
              g[static_cast<size_t>(y) * width + x] = to;
            }
          }
        }
      };
    for (int i = 0; i < opts.grid_close; ++i) {
      pass(grid, kFree, kOccupied);  // dilate obstacles
    }
    for (int i = 0; i < opts.grid_close; ++i) {
      pass(grid, kOccupied, kFree);  // erode back
    }
    occupied_cells = 0;
    for (uint8_t v : grid) {
      occupied_cells += v == kOccupied ? 1 : 0;
    }
  }

  const fs::path yaml_path = fs::path(opts.grid_yaml);
  fs::path pgm_path = yaml_path;
  pgm_path.replace_extension(".pgm");

  // PGM: row 0 is the top of the image = maximum y.
  std::ofstream pgm(pgm_path, std::ios::binary);
  if (!pgm) {
    throw std::runtime_error("cannot open " + pgm_path.string());
  }
  pgm << "P5\n" << width << " " << height << "\n255\n";
  for (int row = height - 1; row >= 0; --row) {
    pgm.write(
      reinterpret_cast<const char *>(&grid[static_cast<size_t>(row) * width]),
      width);
  }
  pgm.close();

  std::ofstream yaml(yaml_path);
  if (!yaml) {
    throw std::runtime_error("cannot open " + yaml_path.string());
  }
  yaml << "image: " << pgm_path.filename().string() << "\n"
       << "mode: trinary\n"
       << "resolution: " << res << "\n"
       << "origin: [" << origin_x << ", " << origin_y << ", 0.0]\n"
       << "negate: 0\n"
       << "occupied_thresh: 0.65\n"
       << "free_thresh: 0.25\n";
  yaml.close();

  std::cout << "[sdf2map] occupancy grid: " << width << "x" << height
            << " cells (" << res << " m/cell), " << occupied_cells
            << " occupied -> " << pgm_path.string() << " + "
            << yaml_path.string() << std::endl;
}

}  // namespace sdf2map
