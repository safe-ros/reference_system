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

#ifndef REFERENCE_SYSTEM__LYON_HPP_
#define REFERENCE_SYSTEM__LYON_HPP_

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float32.hpp"

namespace reference_system
{
class Lyon : public rclcpp::Node
{
public:
  explicit Lyon(rclcpp::NodeOptions options);

protected:
  using MsgType = std_msgs::msg::Float32;

  void on_receive(MsgType::UniquePtr msg);

  rclcpp::Subscription<MsgType>::SharedPtr subscription_;
  rclcpp::Publisher<MsgType>::SharedPtr publisher_;
};
}  // namespace reference_system

#endif  // REFERENCE_SYSTEM__LYON_HPP_
