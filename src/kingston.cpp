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

#include "reference_system/kingston.hpp"
#include "utils.hpp"

#include <chrono>

using namespace std::chrono_literals;

auto kKingstonConfig = reference_system::PublisherConfig(
{
  "kingston",
  "yamuna",
  std::chrono::milliseconds(100),
});

namespace reference_system
{

Kingston::Kingston(rclcpp::NodeOptions options)
: PublisherNode(kKingstonConfig, options)
{
}

void Kingston::populate_msg(geometry_msgs::msg::Vector3 &msg)
{
  msg = random_vector3();
}

}  // namespace reference_system

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(reference_system::Kingston)
