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

#ifndef SDF2MAP__OPTIONS_HPP_
#define SDF2MAP__OPTIONS_HPP_

#include <limits>
#include <string>
#include <vector>

namespace sdf2map
{

struct Options
{
  std::string input;
  std::string output_pcd{"map.pcd"};
  // Extra model:// search directories, tried before the environment
  // variables (GZ_SIM_RESOURCE_PATH etc.)
  std::vector<std::string> model_paths;
  // Download Fuel models missing from the local cache
  bool download{true};
  // Publish the exported cloud as sensor_msgs/PointCloud2 and keep
  // spinning so RViz can inspect it
  bool publish{false};

  // Surface sampling density [points / m^2]
  double density{400.0};
  // Sample <visual> geometry instead of <collision>
  bool use_visual{false};
  // VoxelGrid leaf size [m] applied to the PCD output; 0 disables
  double voxel{0.0};
  // Height crop applied to the PCD output
  double z_min{-std::numeric_limits<double>::infinity()};
  double z_max{std::numeric_limits<double>::infinity()};
  // Skip plane geometries (ground)
  bool exclude_ground{false};
  // Skip whole models whose (possibly nested) name matches any of these
  // ECMAScript regexes (regex_search semantics)
  std::vector<std::string> exclude_patterns;
  // Visuals with transparency >= this are skipped in --geometry visual
  // mode (glass etc. that a lidar would not return)
  double transparency_threshold{0.95};
  bool ascii_pcd{false};
  unsigned int seed{42};
  // Print resolved world poses of every model/link while sampling
  bool verbose{false};

  // Occupancy grid output ("" disables). Path of the YAML file;
  // the PGM is written next to it with the same basename.
  std::string grid_yaml;
  double grid_resolution{0.05};
  // Height band whose points become obstacles in the grid
  double slice_z_min{0.1};
  double slice_z_max{1.8};
  // Grid bounds source: full cloud xy extent (default), or only the
  // obstacle slice's extent (compact maps for worlds with huge ground
  // planes), padded by grid_bounds_margin.
  bool grid_bounds_slice{false};
  double grid_bounds_margin{1.0};
  // Morphological closing radius in cells applied to occupied cells;
  // fills speckle holes left by stochastic sampling. 0 disables.
  int grid_close{1};
};

}  // namespace sdf2map

#endif  // SDF2MAP__OPTIONS_HPP_
