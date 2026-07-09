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

#ifndef SDF2MAP__RESOURCE_RESOLVER_HPP_
#define SDF2MAP__RESOURCE_RESOLVER_HPP_

#include <set>
#include <string>
#include <vector>

namespace sdf2map
{

/// Resolves SDF resource URIs (model://, file://, https:// Fuel URIs,
/// absolute and relative paths) to local filesystem paths.
///
/// Search order for model:// URIs:
///   1. Directories in GZ_SIM_RESOURCE_PATH / IGN_GAZEBO_RESOURCE_PATH /
///      GAZEBO_MODEL_PATH / SDF_PATH
///   2. The Fuel cache (~/.gz/fuel), picking the highest cached version
class ResourceResolver
{
public:
  /// \param base_dir directory of the SDF file being processed, used to
  /// resolve plain relative paths.
  /// \param extra_dirs additional model:// search directories, tried
  /// before the ones from the environment.
  /// \param download when true, Fuel URIs missing from the local cache
  /// are downloaded on demand (like gz sim does).
  explicit ResourceResolver(
    const std::string & base_dir,
    const std::vector<std::string> & extra_dirs = {},
    bool download = true);

  /// Resolve a URI to an existing file or directory path. Returns "" on
  /// failure. Plain relative URIs are resolved against base_dir_override
  /// when given (e.g. the directory of an included model.sdf), then
  /// against the base directory passed to the constructor.
  std::string Resolve(
    const std::string & uri, const std::string & base_dir_override = "") const;

private:
  std::string ResolveModelUri(const std::string & uri) const;
  std::string ResolveFuelUrl(const std::string & uri) const;
  std::string FindInFuelCache(
    const std::string & model_name, const std::string & version,
    const std::string & sub_path) const;

  std::string base_dir_;
  std::string fuel_cache_;
  std::vector<std::string> search_dirs_;
  bool download_;
  // Base model URIs a download was already attempted for
  mutable std::set<std::string> download_attempts_;
};

}  // namespace sdf2map

#endif  // SDF2MAP__RESOURCE_RESOLVER_HPP_
