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

#include "reference_system/georgetown.hpp"

namespace reference_system
{

Georgetown::Georgetown(rclcpp::NodeOptions options)
: Node("georgetown", options)
{
  volga_publisher_ = create_publisher<std_msgs::msg::Float64>("volga", 10);

  murray_subscription_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "murray",
    10,
    [](geometry_msgs::msg::Vector3Stamped::UniquePtr msg) {
      (void) msg;
    });

  lena_subscription_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
    "lena",
    10,
    [](geometry_msgs::msg::WrenchStamped::UniquePtr msg) {
      (void) msg;
    });

  timer_ = create_wall_timer(
    std::chrono::milliseconds(50), [this]() {
      std_msgs::msg::Float64 msg;
      this->volga_publisher_->publish(msg);
    });
}
}  // namespace reference_system

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(reference_system::Georgetown)
