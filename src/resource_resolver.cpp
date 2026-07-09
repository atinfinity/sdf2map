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

#include "sdf2map/resource_resolver.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <sstream>

#include <gz/fuel_tools/Interface.hh>

namespace fs = std::filesystem;

namespace sdf2map
{
namespace
{

std::vector<std::string> SplitPaths(const char * env)
{
  std::vector<std::string> out;
  const char * val = env ? std::getenv(env) : nullptr;
  if (!val) {
    return out;
  }
  std::stringstream ss(val);
  std::string item;
  while (std::getline(ss, item, ':')) {
    if (!item.empty()) {
      out.push_back(item);
    }
  }
  return out;
}

std::string Lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(),
    [](unsigned char c) {return std::tolower(c);});
  return s;
}

std::string UrlDecode(const std::string & in)
{
  std::string out;
  for (size_t i = 0; i < in.size(); ++i) {
    // Decode %XX only for two valid hex digits; anything else (including
    // malformed sequences like "%zz") is copied through literally so a
    // broken URI never aborts the conversion.
    if (in[i] == '%' && i + 2 < in.size() &&
      std::isxdigit(static_cast<unsigned char>(in[i + 1])) &&
      std::isxdigit(static_cast<unsigned char>(in[i + 2])))
    {
      out += static_cast<char>(std::stoi(in.substr(i + 1, 2), nullptr, 16));
      i += 2;
    } else {
      out += in[i];
    }
  }
  return out;
}

/// Within a cached model directory, pick the highest numeric version subdir.
std::string HighestVersion(const fs::path & model_dir)
{
  int best = -1;
  fs::path best_path;
  std::error_code ec;
  for (const auto & entry : fs::directory_iterator(model_dir, ec)) {
    if (!entry.is_directory()) {
      continue;
    }
    try {
      int v = std::stoi(entry.path().filename().string());
      if (v > best) {
        best = v;
        best_path = entry.path();
      }
    } catch (const std::exception &) {
    }
  }
  return best >= 0 ? best_path.string() : std::string();
}

}  // namespace

ResourceResolver::ResourceResolver(
  const std::string & base_dir, const std::vector<std::string> & extra_dirs,
  bool download)
: base_dir_(base_dir), download_(download)
{
  search_dirs_ = extra_dirs;
  for (const char * env :
    {"GZ_SIM_RESOURCE_PATH", "IGN_GAZEBO_RESOURCE_PATH",
      "GAZEBO_MODEL_PATH", "SDF_PATH"})
  {
    for (auto & p : SplitPaths(env)) {
      search_dirs_.push_back(p);
    }
  }
  const char * home = std::getenv("HOME");
  if (home) {
    fuel_cache_ = std::string(home) + "/.gz/fuel";
  }
  const char * fuel_env = std::getenv("GZ_FUEL_CACHE_PATH");
  if (fuel_env) {
    fuel_cache_ = fuel_env;
  }
}

std::string ResourceResolver::Resolve(
  const std::string & uri, const std::string & base_dir_override) const
{
  if (uri.rfind("model://", 0) == 0) {
    return ResolveModelUri(uri.substr(8));
  }
  if (uri.rfind("file://", 0) == 0) {
    std::string path = uri.substr(7);
    return fs::exists(path) ? path : std::string();
  }
  if (uri.rfind("https://", 0) == 0 || uri.rfind("http://", 0) == 0) {
    return ResolveFuelUrl(uri);
  }
  if (fs::path(uri).is_absolute()) {
    return fs::exists(uri) ? uri : std::string();
  }
  // Plain relative path: relative to the directory of the file that
  // referenced it (an included model.sdf), then to the input SDF's dir.
  if (!base_dir_override.empty()) {
    fs::path candidate = fs::path(base_dir_override) / uri;
    if (fs::exists(candidate)) {
      return candidate.string();
    }
  }
  fs::path candidate = fs::path(base_dir_) / uri;
  return fs::exists(candidate) ? candidate.string() : std::string();
}

std::string ResourceResolver::ResolveModelUri(const std::string & rest) const
{
  const auto slash = rest.find('/');
  const std::string model_name = rest.substr(0, slash);
  const std::string sub_path =
    slash == std::string::npos ? "" : rest.substr(slash + 1);

  for (const auto & dir : search_dirs_) {
    fs::path candidate = fs::path(dir) / model_name / sub_path;
    if (fs::exists(candidate)) {
      return candidate.string();
    }
    // Some setups put the model contents directly on the search path.
    candidate = fs::path(dir) / sub_path;
    if (!sub_path.empty() && fs::exists(candidate)) {
      return candidate.string();
    }
  }
  return FindInFuelCache(model_name, "", sub_path);
}

std::string ResourceResolver::ResolveFuelUrl(const std::string & uri) const
{
  // https://fuel.gazebosim.org/1.0/<owner>/models/<name>
  //   [/<version>[/files]/<sub_path>]
  const auto scheme_end = uri.find("://");
  std::string rest = UrlDecode(uri.substr(scheme_end + 3));
  const auto models_pos = Lower(rest).find("/models/");
  if (models_pos == std::string::npos) {
    return std::string();
  }
  std::string after = rest.substr(models_pos + 8);
  std::vector<std::string> segments;
  {
    std::stringstream ss(after);
    std::string item;
    while (std::getline(ss, item, '/')) {
      if (!item.empty()) {
        segments.push_back(item);
      }
    }
  }
  if (segments.empty()) {
    return std::string();
  }
  const std::string model_name = segments[0];
  size_t idx = 1;
  std::string version;
  if (idx < segments.size() &&
    std::all_of(segments[idx].begin(), segments[idx].end(),
    [](unsigned char c) {return std::isdigit(c);}))
  {
    version = segments[idx++];
    if (idx < segments.size() && segments[idx] == "files") {
      ++idx;
    }
  }
  std::string sub_path;
  for (; idx < segments.size(); ++idx) {
    sub_path += (sub_path.empty() ? "" : "/") + segments[idx];
  }
  std::string found = FindInFuelCache(model_name, version, sub_path);
  if (!found.empty() || !download_) {
    return found;
  }

  // Cache miss: download the whole model to the Fuel cache, then retry.
  const std::string base_uri = uri.substr(0, scheme_end + 3) +
    rest.substr(0, models_pos + 8) + model_name;
  if (!download_attempts_.insert(Lower(base_uri)).second) {
    return std::string();  // already tried and failed
  }
  std::cerr << "[sdf2map] downloading " << base_uri << " ..." << std::endl;
  const std::string fetched = gz::fuel_tools::fetchResource(base_uri);
  if (fetched.empty()) {
    std::cerr << "[sdf2map] WARNING: download failed: " << base_uri
              << std::endl;
    return std::string();
  }
  if (sub_path.empty()) {
    return fetched;
  }
  found = FindInFuelCache(model_name, version, sub_path);
  if (!found.empty()) {
    return found;
  }
  fs::path candidate = fs::path(fetched) / sub_path;
  return fs::exists(candidate) ? candidate.string() : std::string();
}

std::string ResourceResolver::FindInFuelCache(
  const std::string & model_name, const std::string & version,
  const std::string & sub_path) const
{
  if (fuel_cache_.empty() || !fs::exists(fuel_cache_)) {
    return std::string();
  }
  const std::string wanted = Lower(model_name);
  std::error_code ec;
  // Layout: <cache>/<server>/<owner>/models/<name>/<version>/...
  for (const auto & server : fs::directory_iterator(fuel_cache_, ec)) {
    if (!server.is_directory()) {
      continue;
    }
    std::error_code ec2;
    for (const auto & owner : fs::directory_iterator(server.path(), ec2)) {
      fs::path models_dir = owner.path() / "models";
      if (!fs::is_directory(models_dir)) {
        continue;
      }
      std::error_code ec3;
      for (const auto & model : fs::directory_iterator(models_dir, ec3)) {
        if (Lower(model.path().filename().string()) != wanted) {
          continue;
        }
        // Prefer the exact requested version, fall back to the highest.
        std::string version_dir;
        if (!version.empty() && fs::exists(model.path() / version)) {
          version_dir = (model.path() / version).string();
        } else {
          version_dir = HighestVersion(model.path());
        }
        if (version_dir.empty()) {
          continue;
        }
        fs::path candidate = fs::path(version_dir) / sub_path;
        if (fs::exists(candidate)) {
          return candidate.string();
        }
      }
    }
  }
  return std::string();
}

}  // namespace sdf2map
