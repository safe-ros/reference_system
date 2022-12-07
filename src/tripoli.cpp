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

#include "reference_system/tripoli.hpp"

namespace reference_system
{

Tripoli::Tripoli(rclcpp::NodeOptions options)
: Node("tripoli", options)
{
  loire_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("loire", 10);

  godavari_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "godavari",
    10,
    [this](sensor_msgs::msg::LaserScan::UniquePtr msg) {
      (void) msg;
      sensor_msgs::msg::PointCloud2 pc_msg;
      loire_publisher_->publish(pc_msg);
    });

  columbia_subscription_ = create_subscription<sensor_msgs::msg::Image>(
    "columbia",
    10,
    [](sensor_msgs::msg::Image::UniquePtr msg) {
      (void) msg;
    });
}
}  // namespace reference_system

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(reference_system::Tripoli)
