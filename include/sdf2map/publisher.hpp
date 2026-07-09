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

#ifndef SDF2MAP__PUBLISHER_HPP_
#define SDF2MAP__PUBLISHER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace sdf2map
{

/// Publish the cloud once on /sdf2map/map_cloud (frame "map", transient
/// local QoS so RViz can late-subscribe) and spin until Ctrl-C.
void PublishCloud(
  const pcl::PointCloud<pcl::PointXYZ> & cloud, int argc, char ** argv);

}  // namespace sdf2map

#endif  // SDF2MAP__PUBLISHER_HPP_
