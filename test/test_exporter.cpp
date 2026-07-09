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

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "sdf2map/exporter.hpp"
#include "sdf2map/options.hpp"

namespace
{

struct Pgm
{
  int width{0};
  int height{0};
  std::vector<uint8_t> data;
};

Pgm ReadPgm(const std::filesystem::path & path)
{
  std::ifstream in(path, std::ios::binary);
  std::string magic;
  Pgm pgm;
  int maxval;
  in >> magic >> pgm.width >> pgm.height >> maxval;
  in.get();  // single whitespace after the header
  pgm.data.resize(static_cast<size_t>(pgm.width) * pgm.height);
  in.read(reinterpret_cast<char *>(pgm.data.data()), pgm.data.size());
  return pgm;
}

size_t OccupiedCells(const Pgm & pgm)
{
  size_t n = 0;
  for (uint8_t v : pgm.data) {
    n += v == 0 ? 1 : 0;
  }
  return n;
}

/// A 2 m wall of one point per 5 cm cell at z=1, with one missing cell in
/// the middle, plus low corner points that define the grid bounds without
/// entering the obstacle slice.
sdf2map::CloudXYZ MakeWallWithHole()
{
  sdf2map::CloudXYZ cloud;
  for (int k = 0; k <= 40; ++k) {
    if (k == 20) {
      continue;  // the hole
    }
    cloud.emplace_back(0.025f + 0.05f * k, 0.025f, 1.0f);
  }
  cloud.emplace_back(-1.0f, -1.0f, 0.0f);
  cloud.emplace_back(3.0f, 1.0f, 0.0f);
  return cloud;
}

}  // namespace

TEST(Exporter, GridClosingFillsSpeckleHoles)
{
  const auto cloud = MakeWallWithHole();
  const auto dir = std::filesystem::temp_directory_path();

  sdf2map::Options opts;
  opts.grid_resolution = 0.05;
  opts.slice_z_min = 0.5;
  opts.slice_z_max = 1.5;

  opts.grid_close = 0;
  opts.grid_yaml = (dir / "sdf2map_grid_open.yaml").string();
  sdf2map::ExportOccupancyGrid(cloud, opts);
  const auto open = ReadPgm(dir / "sdf2map_grid_open.pgm");

  opts.grid_close = 1;
  opts.grid_yaml = (dir / "sdf2map_grid_closed.yaml").string();
  sdf2map::ExportOccupancyGrid(cloud, opts);
  const auto closed = ReadPgm(dir / "sdf2map_grid_closed.pgm");

  for (const char * base :
    {"sdf2map_grid_open", "sdf2map_grid_closed"})
  {
    std::filesystem::remove(dir / (std::string(base) + ".yaml"));
    std::filesystem::remove(dir / (std::string(base) + ".pgm"));
  }

  EXPECT_EQ(OccupiedCells(open), 40u);
  // Closing fills exactly the hole and does not grow the wall
  EXPECT_EQ(OccupiedCells(closed), 41u);
  EXPECT_EQ(open.width, closed.width);
  EXPECT_EQ(open.height, closed.height);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
