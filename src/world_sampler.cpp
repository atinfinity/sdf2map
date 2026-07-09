#include "sdf2map/world_sampler.hpp"

#include <filesystem>
#include <iostream>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

#include <sdf/Collision.hh>
#include <sdf/Error.hh>
#include <sdf/Geometry.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/ParserConfig.hh>
#include <sdf/Root.hh>
#include <sdf/SemanticPose.hh>
#include <sdf/Visual.hh>
#include <sdf/World.hh>

#include "sdf2map/resource_resolver.hpp"

namespace sdf2map
{
namespace
{

/// Resolve an entity pose relative to its default parent frame, falling
/// back to the raw pose on frame-graph errors.
template<typename EntityT>
gz::math::Pose3d ResolvePose(const EntityT & entity)
{
  gz::math::Pose3d pose;
  sdf::Errors errors = entity.SemanticPose().Resolve(pose);
  if (!errors.empty()) {
    std::cerr << "[sdf2map] WARNING: pose resolution failed ("
              << errors[0].Message() << "), using raw pose" << std::endl;
    pose = entity.SemanticPose().RawPose();
  }
  return pose;
}

class WorldSampler
{
public:
  WorldSampler(const Options & opts, Sampler & sampler, SampleStats & stats)
  : opts_(opts), sampler_(sampler), stats_(stats)
  {
    for (const auto & pattern : opts.exclude_patterns) {
      exclude_.emplace_back(pattern);
    }
  }

  void ProcessModel(
    const sdf::Model & model, const gz::math::Pose3d & parent_world,
    CloudXYZ & out)
  {
    for (const auto & re : exclude_) {
      if (std::regex_search(model.Name(), re)) {
        if (opts_.verbose) {
          std::cout << "[sdf2map] model '" << model.Name()
                    << "' excluded by pattern" << std::endl;
        }
        ++stats_.excluded_models;
        return;
      }
    }
    ++stats_.models;
    // gz-math7 Pose3 composes matrix-style: X_OQ = X_OP * X_PQ
    // (parent-to-world on the LEFT, child-in-parent on the RIGHT).
    const gz::math::Pose3d model_world = parent_world * ResolvePose(model);
    if (opts_.verbose) {
      std::cout << "[sdf2map] model '" << model.Name() << "' world pose: "
                << model_world << std::endl;
    }

    for (uint64_t l = 0; l < model.LinkCount(); ++l) {
      const sdf::Link * link = model.LinkByIndex(l);
      const gz::math::Pose3d link_world = model_world * ResolvePose(*link);

      if (opts_.use_visual) {
        for (uint64_t v = 0; v < link->VisualCount(); ++v) {
          const sdf::Visual * vis = link->VisualByIndex(v);
          if (vis->Transparency() >= opts_.transparency_threshold) {
            ++stats_.skipped;
            continue;
          }
          ProcessGeometry(*vis->Geom(), link_world * ResolvePose(*vis), out);
        }
      } else {
        for (uint64_t c = 0; c < link->CollisionCount(); ++c) {
          const sdf::Collision * col = link->CollisionByIndex(c);
          ProcessGeometry(*col->Geom(), link_world * ResolvePose(*col), out);
        }
      }
    }
    for (uint64_t m = 0; m < model.ModelCount(); ++m) {
      ProcessModel(*model.ModelByIndex(m), model_world, out);
    }
  }

private:
  void ProcessGeometry(
    const sdf::Geometry & geom, const gz::math::Pose3d & world_pose,
    CloudXYZ & out)
  {
    if (opts_.exclude_ground && geom.Type() == sdf::GeometryType::PLANE) {
      ++stats_.skipped;
      return;
    }
    if (sampler_.SampleGeometry(geom, world_pose, out)) {
      ++stats_.geometries;
    } else {
      ++stats_.skipped;
    }
  }

  const Options & opts_;
  Sampler & sampler_;
  SampleStats & stats_;
  std::vector<std::regex> exclude_;
};

}  // namespace

CloudXYZ::Ptr SampleWorld(const Options & opts, SampleStats & stats)
{
  const std::string base_dir =
    std::filesystem::absolute(opts.input).parent_path().string();
  ResourceResolver resolver(base_dir, opts.model_paths);

  sdf::ParserConfig config = sdf::ParserConfig::GlobalConfig();
  config.SetFindCallback(
    [&resolver](const std::string & uri) {
      return resolver.Resolve(uri);
    });

  sdf::Root root;
  sdf::Errors errors = root.Load(opts.input, config);
  bool fatal = false;
  for (const auto & e : errors) {
    std::cerr << "[sdf2map] SDF: " << e.Message() << std::endl;
    fatal = true;
  }
  if (fatal && !root.WorldByIndex(0) && !root.Model()) {
    throw std::runtime_error("failed to load SDF file: " + opts.input);
  }

  Sampler sampler(opts.density, opts.seed, &resolver);
  auto cloud = std::make_shared<CloudXYZ>();
  WorldSampler world_sampler(opts, sampler, stats);

  if (root.WorldCount() > 0) {
    const sdf::World * world = root.WorldByIndex(0);
    if (root.WorldCount() > 1) {
      std::cerr << "[sdf2map] NOTE: file contains " << root.WorldCount()
                << " worlds, using the first: " << world->Name() << std::endl;
    }
    for (uint64_t i = 0; i < world->ModelCount(); ++i) {
      world_sampler.ProcessModel(
        *world->ModelByIndex(i), gz::math::Pose3d::Zero, *cloud);
    }
  } else if (root.Model()) {
    world_sampler.ProcessModel(*root.Model(), gz::math::Pose3d::Zero, *cloud);
  } else {
    throw std::runtime_error("SDF file contains no world or model");
  }

  cloud->width = cloud->size();
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

}  // namespace sdf2map
