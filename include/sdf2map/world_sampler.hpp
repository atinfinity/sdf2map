#ifndef SDF2MAP__WORLD_SAMPLER_HPP_
#define SDF2MAP__WORLD_SAMPLER_HPP_

#include <cstddef>

#include "sdf2map/options.hpp"
#include "sdf2map/samplers.hpp"

namespace sdf2map
{

struct SampleStats
{
  std::size_t models{0};
  std::size_t geometries{0};
  std::size_t skipped{0};
};

/// Load the SDF world/model file named in opts.input and sample every
/// collision (or visual) geometry into one cloud in world coordinates.
/// Throws std::runtime_error on load failure.
CloudXYZ::Ptr SampleWorld(const Options & opts, SampleStats & stats);

}  // namespace sdf2map

#endif  // SDF2MAP__WORLD_SAMPLER_HPP_
