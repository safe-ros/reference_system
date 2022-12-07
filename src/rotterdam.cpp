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

#include "reference_system/rotterdam.hpp"

namespace reference_system
{

Rotterdam::Rotterdam(rclcpp::NodeOptions options)
: Node("rotterdam", options)
{
  murray_publisher_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("murray", 10);

  mekong_subscription_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "mekong",
    10,
    [this](geometry_msgs::msg::TwistWithCovarianceStamped::UniquePtr msg) {
      (void) msg;
      geometry_msgs::msg::Vector3Stamped pub_msg;
      murray_publisher_->publish(pub_msg);
    });
}
}  // namespace reference_system

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(reference_system::Rotterdam)
