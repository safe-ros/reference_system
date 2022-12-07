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

#include "reference_system/geneva.hpp"
#include "utils.hpp"

namespace reference_system
{
Geneva::Geneva(rclcpp::NodeOptions options)
: Node("geneva", options)
{
  arkansas_publisher_ = create_publisher<std_msgs::msg::String>("arkansas", 10);

  parana_subscription_ = create_subscription<std_msgs::msg::String>(
    "parana",
    10,
    [this](std_msgs::msg::String::UniquePtr msg) {
      msg->data = random_string(32);
      arkansas_publisher_->publish(std::move(msg));
    });

  danube_subscription_ = create_subscription<std_msgs::msg::String>(
    "danube",
    10,
    [](std_msgs::msg::String::UniquePtr msg) {
      (void) msg;
    });

  tagus_subscription_ = create_subscription<geometry_msgs::msg::Pose>(
    "tagus",
    10,
    [](geometry_msgs::msg::Pose::UniquePtr msg) {
      (void) msg;
    });

  congo_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    "congo",
    10,
    [](geometry_msgs::msg::Twist::UniquePtr msg) {
      (void) msg;
    });
}
}  // namespace reference_system

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(reference_system::Geneva)
