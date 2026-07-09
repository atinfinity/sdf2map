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
