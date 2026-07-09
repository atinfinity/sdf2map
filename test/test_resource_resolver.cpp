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

#include <filesystem>
#include <fstream>
#include <string>

#include "sdf2map/resource_resolver.hpp"

namespace
{

namespace fs = std::filesystem;

class ResourceResolverTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    base_ = fs::temp_directory_path() / "sdf2map_resolver_test";
    fs::create_directories(base_ / "meshes");
    std::ofstream(base_ / "meshes" / "part.stl") << "solid s\nendsolid s\n";
  }

  void TearDown() override
  {
    fs::remove_all(base_);
  }

  fs::path base_;
};

}  // namespace

TEST_F(ResourceResolverTest, MalformedPercentEncodingDoesNotThrow)
{
  // Regression: "%zz" used to reach std::stoi and abort the whole run
  // with an uncaught exception.
  sdf2map::ResourceResolver resolver(base_.string(), {}, false);
  std::string result;
  EXPECT_NO_THROW(
    result = resolver.Resolve(
      "https://fuel.gazebosim.org/1.0/x/models/nope%zz/1/files/m.dae"));
  EXPECT_TRUE(result.empty());
  EXPECT_NO_THROW(
    result = resolver.Resolve(
      "https://fuel.gazebosim.org/1.0/x/models/trailing%2"));
  EXPECT_TRUE(result.empty());
}

TEST_F(ResourceResolverTest, RelativePathAgainstBaseDir)
{
  sdf2map::ResourceResolver resolver(base_.string(), {}, false);
  const std::string found = resolver.Resolve("meshes/part.stl");
  EXPECT_EQ(found, (base_ / "meshes" / "part.stl").string());
  EXPECT_TRUE(resolver.Resolve("meshes/missing.stl").empty());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
