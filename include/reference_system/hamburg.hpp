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

#ifndef REFERENCE_SYSTEM__HAMBURG_HPP_
#define REFERENCE_SYSTEM__HAMBURG_HPP_

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/string.hpp"

namespace reference_system
{
class Hamburg : public rclcpp::Node
{
public:
  explicit Hamburg(rclcpp::NodeOptions options);

protected:
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr tigris_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr ganges_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr nile_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr danube_subscription_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr parana_publisher_;
};
}  // namespace reference_system

#endif  // REFERENCE_SYSTEM__HAMBURG_HPP_
