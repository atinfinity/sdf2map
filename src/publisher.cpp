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

#include "sdf2map/publisher.hpp"

#include <iostream>
#include <memory>

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace sdf2map
{

void PublishCloud(
  const pcl::PointCloud<pcl::PointXYZ> & cloud, int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("sdf2map");
  auto qos = rclcpp::QoS(1).transient_local();
  auto pub =
    node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/sdf2map/map_cloud", qos);

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = "map";
  msg.header.stamp = node->now();
  pub->publish(msg);

  std::cout << "[sdf2map] publishing " << cloud.size()
            << " points on /sdf2map/map_cloud (frame 'map', latched); "
            << "press Ctrl-C to exit" << std::endl;
  rclcpp::spin(node);
  rclcpp::shutdown();
}

}  // namespace sdf2map
