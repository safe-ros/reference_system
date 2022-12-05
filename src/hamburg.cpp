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

#include "reference_system/hamburg.hpp"

namespace reference_system
{

Hamburg::Hamburg(rclcpp::NodeOptions options)
: Node("hamburg", options)
{
  parana_publisher_ = create_publisher<std_msgs::msg::String>("parana", 10);

  tigris_subscription_ = create_subscription<std_msgs::msg::Float32>(
    "tigris",
    10,
    [](std_msgs::msg::Float32::UniquePtr msg) {
      (void) msg;
    });

  ganges_subscription_ = create_subscription<std_msgs::msg::Int64>(
    "ganges",
    10,
    [](std_msgs::msg::Int64::UniquePtr msg) {
      (void) msg;
    });

  nile_subscription_ = create_subscription<std_msgs::msg::Int32>(
    "nile",
    10,
    [](std_msgs::msg::Int32::UniquePtr msg) {
      (void) msg;
    });

  danube_subscription_ = create_subscription<std_msgs::msg::String>(
    "danube",
    10,
    [this](std_msgs::msg::String::UniquePtr msg) {
      this->parana_publisher_->publish(std::move(msg));
    });
}

}  // namespace reference_system

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(reference_system::Hamburg)
