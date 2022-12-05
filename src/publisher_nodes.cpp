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

#include "reference_system/publisher_nodes.hpp"

#include <chrono>

using namespace std::chrono_literals;

auto kCordobaConfig = reference_system::PublisherConfig(
{
  "cordoba",
  "amazon",
  std::chrono::milliseconds(100),
});

auto kFreeportConfig = reference_system::PublisherConfig(
{
  "freeport",
  "ganges",
  std::chrono::milliseconds(50),
});

auto kMedellinConfig = reference_system::PublisherConfig(
{
  "medellin",
  "nile",
  std::chrono::milliseconds(10),
});

auto kPortsmouthConfig = reference_system::PublisherConfig(
{
  "portsmouth",
  "danube",
  std::chrono::milliseconds(200),
});

auto kDelhiConfig = reference_system::PublisherConfig(
{
  "delhi",
  "columbia",
  std::chrono::milliseconds(1000),
});

auto kHebronConfig = reference_system::PublisherConfig(
{
  "hebron",
  "chenab",
  std::chrono::milliseconds(100),
});

auto kKingstonConfig = reference_system::PublisherConfig(
{
  "kingston",
  "yamuna",
  std::chrono::milliseconds(100),
});

namespace reference_system
{

Cordoba::Cordoba(rclcpp::NodeOptions options)
: PublisherNode(kCordobaConfig, options)
{
}

Freeport::Freeport(rclcpp::NodeOptions options)
: PublisherNode(kFreeportConfig, options)
{
}

Medellin::Medellin(rclcpp::NodeOptions options)
: PublisherNode(kMedellinConfig, options)
{
}

Portsmouth::Portsmouth(rclcpp::NodeOptions options)
: PublisherNode(kPortsmouthConfig, options)
{
}

Delhi::Delhi(rclcpp::NodeOptions options)
: PublisherNode(kDelhiConfig, options)
{
}

Hebron::Hebron(rclcpp::NodeOptions options)
: PublisherNode(kHebronConfig, options)
{
}

Kingston::Kingston(rclcpp::NodeOptions options)
: PublisherNode(kKingstonConfig, options)
{
}
}  // namespace reference_system

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(reference_system::Cordoba)
RCLCPP_COMPONENTS_REGISTER_NODE(reference_system::Freeport)
RCLCPP_COMPONENTS_REGISTER_NODE(reference_system::Medellin)
RCLCPP_COMPONENTS_REGISTER_NODE(reference_system::Portsmouth)
RCLCPP_COMPONENTS_REGISTER_NODE(reference_system::Delhi)
RCLCPP_COMPONENTS_REGISTER_NODE(reference_system::Hebron)
RCLCPP_COMPONENTS_REGISTER_NODE(reference_system::Kingston)
