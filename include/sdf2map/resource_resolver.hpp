#ifndef SDF2MAP__RESOURCE_RESOLVER_HPP_
#define SDF2MAP__RESOURCE_RESOLVER_HPP_

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
  explicit ResourceResolver(
    const std::string & base_dir,
    const std::vector<std::string> & extra_dirs = {});

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
};

}  // namespace sdf2map

#endif  // SDF2MAP__RESOURCE_RESOLVER_HPP_
