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

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "sdf2map/exporter.hpp"
#include "sdf2map/options.hpp"
#include "sdf2map/publisher.hpp"
#include "sdf2map/world_sampler.hpp"

namespace
{

void PrintUsage()
{
  std::cout <<
    "sdf2map - convert a Gazebo SDF world into a PCD point cloud map\n"
    "          (and optionally a Nav2 occupancy grid)\n\n"
    "Usage: sdf2map --input <world.sdf> [options]\n\n"
    "Options:\n"
    "  --input <file>          input SDF world or model file (required)\n"
    "  --model-path <dirs>     extra model:// search directories\n"
    "                          (colon-separated, repeatable; searched\n"
    "                          before GZ_SIM_RESOURCE_PATH)\n"
    "  --output <file>         output PCD file (default: map.pcd)\n"
    "  --density <n>           surface sampling density in points/m^2\n"
    "                          (default: 400)\n"
    "  --geometry <c|v>        sample 'collision' or 'visual' geometry\n"
    "                          (default: collision)\n"
    "  --voxel <m>             VoxelGrid leaf size for the PCD output;\n"
    "                          0 disables (default: 0)\n"
    "  --z-min <m>             crop PCD points below this height\n"
    "  --z-max <m>             crop PCD points above this height\n"
    "  --exclude-ground        skip plane geometries (ground planes)\n"
    "  --exclude <regex>       skip models whose name matches (repeatable,\n"
    "                          e.g. --exclude 'person|actor')\n"
    "  --transparency-threshold <t>  skip visuals with transparency >= t\n"
    "                          in visual mode (default: 0.95)\n"
    "  --ascii                 write ASCII PCD instead of binary\n"
    "  --seed <n>              random seed (default: 42)\n"
    "  --no-download           do not download missing Fuel models\n"
    "  --publish               publish the cloud as PointCloud2 on\n"
    "                          /sdf2map/map_cloud and spin (RViz preview)\n"
    "  --verbose               print resolved model poses\n\n"
    "Actors (<actor> animated entities) are intentionally ignored: they\n"
    "are dynamic objects that do not belong in a localization map.\n\n"
    "Occupancy grid options:\n"
    "  --occupancy-grid <file.yaml>  also write a Nav2 map (YAML + PGM)\n"
    "  --grid-resolution <m>   grid cell size (default: 0.05)\n"
    "  --slice-z-min <m>       lower bound of the obstacle height band\n"
    "                          (default: 0.1)\n"
    "  --slice-z-max <m>       upper bound of the obstacle height band\n"
    "                          (default: 1.8)\n"
    "  --grid-bounds <full|slice>  grid extent from the full cloud\n"
    "                          (default) or the obstacle slice only\n"
    "  --grid-margin <m>       extra margin around slice bounds\n"
    "                          (default: 1.0, used with --grid-bounds slice)\n"
    "  --grid-close <n>        morphological closing radius in cells to\n"
    "                          fill speckle holes; 0 disables (default: 1)\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  sdf2map::Options opts;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string {
        if (i + 1 >= argc) {
          throw std::runtime_error("missing value for " + arg);
        }
        return argv[++i];
      };
    try {
      if (arg == "--help" || arg == "-h") {
        PrintUsage();
        return 0;
      } else if (arg == "--input") {
        opts.input = next();
      } else if (arg == "--model-path") {
        // May be given multiple times; each value may be colon-separated.
        std::stringstream ss(next());
        std::string dir;
        while (std::getline(ss, dir, ':')) {
          if (!dir.empty()) {
            opts.model_paths.push_back(dir);
          }
        }
      } else if (arg == "--output") {
        opts.output_pcd = next();
      } else if (arg == "--density") {
        opts.density = std::stod(next());
      } else if (arg == "--geometry") {
        const std::string v = next();
        if (v == "visual" || v == "v") {
          opts.use_visual = true;
        } else if (v == "collision" || v == "c") {
          opts.use_visual = false;
        } else {
          throw std::runtime_error("--geometry must be collision or visual");
        }
      } else if (arg == "--voxel") {
        opts.voxel = std::stod(next());
      } else if (arg == "--z-min") {
        opts.z_min = std::stod(next());
      } else if (arg == "--z-max") {
        opts.z_max = std::stod(next());
      } else if (arg == "--exclude-ground") {
        opts.exclude_ground = true;
      } else if (arg == "--exclude") {
        opts.exclude_patterns.push_back(next());
      } else if (arg == "--transparency-threshold") {
        opts.transparency_threshold = std::stod(next());
      } else if (arg == "--ascii") {
        opts.ascii_pcd = true;
      } else if (arg == "--verbose") {
        opts.verbose = true;
      } else if (arg == "--no-download") {
        opts.download = false;
      } else if (arg == "--publish") {
        opts.publish = true;
      } else if (arg == "--seed") {
        opts.seed = static_cast<unsigned int>(std::stoul(next()));
      } else if (arg == "--occupancy-grid") {
        opts.grid_yaml = next();
      } else if (arg == "--grid-resolution") {
        opts.grid_resolution = std::stod(next());
      } else if (arg == "--slice-z-min") {
        opts.slice_z_min = std::stod(next());
      } else if (arg == "--slice-z-max") {
        opts.slice_z_max = std::stod(next());
      } else if (arg == "--grid-bounds") {
        const std::string v = next();
        if (v == "slice") {
          opts.grid_bounds_slice = true;
        } else if (v == "full") {
          opts.grid_bounds_slice = false;
        } else {
          throw std::runtime_error("--grid-bounds must be full or slice");
        }
      } else if (arg == "--grid-margin") {
        opts.grid_bounds_margin = std::stod(next());
      } else if (arg == "--grid-close") {
        opts.grid_close = std::stoi(next());
      } else {
        std::cerr << "Unknown option: " << arg << "\n\n";
        PrintUsage();
        return 1;
      }
    } catch (const std::exception & e) {
      std::cerr << "Error parsing arguments: " << e.what() << std::endl;
      return 1;
    }
  }

  if (opts.input.empty()) {
    std::cerr << "Error: --input is required\n\n";
    PrintUsage();
    return 1;
  }
  if (opts.density <= 0.0) {
    std::cerr << "Error: --density must be > 0" << std::endl;
    return 1;
  }

  try {
    sdf2map::SampleStats stats;
    auto cloud = sdf2map::SampleWorld(opts, stats);
    std::cout << "[sdf2map] sampled " << cloud->size() << " points from "
              << stats.geometries << " geometries in " << stats.models
              << " models";
    if (stats.skipped > 0) {
      std::cout << " (" << stats.skipped << " geometries skipped)";
    }
    if (stats.excluded_models > 0) {
      std::cout << " (" << stats.excluded_models << " models excluded)";
    }
    std::cout << std::endl;

    const std::size_t written = sdf2map::ExportPCD(cloud, opts);
    std::cout << "[sdf2map] wrote " << written << " points -> "
              << opts.output_pcd << std::endl;

    if (!opts.grid_yaml.empty()) {
      sdf2map::ExportOccupancyGrid(*cloud, opts);
    }
    if (opts.publish) {
      sdf2map::PublishCloud(*cloud, argc, argv);
    }
  } catch (const std::exception & e) {
    std::cerr << "[sdf2map] ERROR: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}
