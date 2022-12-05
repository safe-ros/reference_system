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

#include "reference_system/arequipa.hpp"

namespace reference_system
{

Arequipa::Arequipa(rclcpp::NodeOptions options)
: Node("arequipa", options)
{
  arkansas_subscription_ = create_subscription<std_msgs::msg::String>(
    "arkansas",
    10,
    [](std_msgs::msg::String::UniquePtr msg) {
      (void) msg;
    });
}
}  // namespace reference_system

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(reference_system::Arequipa)
