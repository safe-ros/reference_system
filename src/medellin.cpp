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

#include "reference_system/medellin.hpp"
#include "utils.hpp"

#include <chrono>

using namespace std::chrono_literals;

auto kMedellinConfig = reference_system::PublisherConfig(
{
  "medellin",
  "nile",
  std::chrono::milliseconds(10),
});

namespace reference_system
{

Medellin::Medellin(rclcpp::NodeOptions options)
: PublisherNode(kMedellinConfig, options)
{
}

void Medellin::populate_msg(std_msgs::msg::Int32 &msg)
{
  msg.data = random_number<int32_t>();
}
}  // namespace reference_system

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(reference_system::Medellin)
