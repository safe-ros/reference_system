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

#include "reference_system/taipei.hpp"

namespace reference_system
{

Taipei::Taipei(rclcpp::NodeOptions options)
: Node("taipei", options)
{
  publisher_ = create_publisher<MsgType>("colorado", 10);
  subscription_ = create_subscription<MsgType>(
    "columbia",
    10,
    [this](MsgType::UniquePtr msg) {
      this->on_receive(std::move(msg));
    });
}

void Taipei::on_receive(MsgType::UniquePtr msg)
{
  publisher_->publish(std::move(msg));
}

}  // namespace reference_system

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(reference_system::Taipei)
