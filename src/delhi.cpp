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

#include "reference_system/delhi.hpp"
#include "utils.hpp"

#include <chrono>

using namespace std::chrono_literals;

auto kDelhiConfig = reference_system::PublisherConfig(
{
  "delhi",
  "columbia",
  std::chrono::milliseconds(1000),
});

namespace reference_system
{

Delhi::Delhi(rclcpp::NodeOptions options)
: PublisherNode(kDelhiConfig, options)
{
}

void Delhi::populate_msg(sensor_msgs::msg::Image &msg)
{
  msg = random_image(100);
}

}  // namespace reference_system

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(reference_system::Delhi)
