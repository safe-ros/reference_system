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

#include "reference_system/hebron.hpp"
#include "utils.hpp"

#include <chrono>

using namespace std::chrono_literals;

auto kHebronConfig = reference_system::PublisherConfig(
{
  "hebron",
  "chenab",
  std::chrono::milliseconds(100),
});

namespace reference_system
{

Hebron::Hebron(rclcpp::NodeOptions options)
: PublisherNode(kHebronConfig, options)
{
}

void Hebron::populate_msg(geometry_msgs::msg::Quaternion &msg)
{
  msg = random_quaternion();
}

}  // namespace reference_system

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(reference_system::Hebron)
