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

#ifndef REFERENCE_SYSTEM__GENEVA_HPP_
#define REFERENCE_SYSTEM__GENEVA_HPP_

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace reference_system
{
class Geneva : public rclcpp::Node
{
public:
  explicit Geneva(rclcpp::NodeOptions options);

protected:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr parana_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr danube_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr tagus_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr congo_subscription_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arkansas_publisher_;
};
}  // namespace reference_system

#endif  // REFERENCE_SYSTEM__GENEVA_HPP_
