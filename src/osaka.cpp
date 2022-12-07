// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#include "reference_system/osaka.hpp"
#include "utils.hpp"

namespace reference_system
{

Osaka::Osaka(rclcpp::NodeOptions options)
: Node("osaka", options)
{
  salween_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("salween", 10);
  godavari_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>("godavari", 10);

  parana_subscription_ = create_subscription<std_msgs::msg::String>(
    "parana",
    10,
    [](std_msgs::msg::String::UniquePtr msg) {
      (void) msg;
    });

  columbia_subscription_ = create_subscription<sensor_msgs::msg::Image>(
    "columbia",
    10,
    [](sensor_msgs::msg::Image::UniquePtr msg) {
      (void) msg;
    });

  colorado_subscription_ = create_subscription<sensor_msgs::msg::Image>(
    "colorado",
    10,
    [this](sensor_msgs::msg::Image::UniquePtr msg) {
      (void) msg;

      sensor_msgs::msg::PointCloud2 pc_msg = random_pointcloud(64*480);
      sensor_msgs::msg::LaserScan scan_msg = random_laserscan();

      salween_publisher_->publish(pc_msg);
      godavari_publisher_->publish(scan_msg);
    });
}

}  // namespace reference_system

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(reference_system::Osaka)
