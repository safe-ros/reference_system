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

#ifndef REFERENCE_SYSTEM__GEORGETOWN_HPP_
#define REFERENCE_SYSTEM__GEORGETOWN_HPP_

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/float64.hpp"

namespace reference_system
{
class Georgetown : public rclcpp::Node
{
public:
  explicit Georgetown(rclcpp::NodeOptions options);

protected:
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr murray_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr lena_subscription_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr volga_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace reference_system

#endif  // REFERENCE_SYSTEM__GEORGETOWN_HPP_
