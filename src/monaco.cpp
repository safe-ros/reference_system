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

#include "reference_system/monaco.hpp"
#include "utils.hpp"

namespace reference_system
{

Monaco::Monaco(rclcpp::NodeOptions options)
: Node("monaco", options)
{
  ohio_publisher_ = create_publisher<std_msgs::msg::Float32>("ohio", 10);

  congo_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    "congo",
    10,
    [this](geometry_msgs::msg::Twist::UniquePtr msg) {
      (void) msg;
      std_msgs::msg::Float32 pub_msg;
      pub_msg.data = random_number<float>();
      ohio_publisher_->publish(pub_msg);
    });
}
}  // namespace reference_system

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(reference_system::Monaco)
