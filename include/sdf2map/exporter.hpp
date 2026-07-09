#ifndef SDF2MAP__EXPORTER_HPP_
#define SDF2MAP__EXPORTER_HPP_

#include "sdf2map/options.hpp"
#include "sdf2map/samplers.hpp"

namespace sdf2map
{

/// Apply z-crop and voxel filter, then write the PCD file.
/// Returns the number of points written.
std::size_t ExportPCD(const CloudXYZ::Ptr & cloud, const Options & opts);

/// Rasterize a z-band slice of the raw cloud into a Nav2 occupancy grid
/// (PGM + YAML). Grid bounds are taken from the xy extent of the full
/// cloud; every in-bounds cell is free unless a slice point occupies it.
void ExportOccupancyGrid(const CloudXYZ & cloud, const Options & opts);

}  // namespace sdf2map

#endif  // SDF2MAP__EXPORTER_HPP_
