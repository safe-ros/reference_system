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

#ifndef REFERENCE_SYSTEM__ROTTERDAM_HPP_
#define REFERENCE_SYSTEM__ROTTERDAM_HPP_

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

namespace reference_system
{
class Rotterdam : public rclcpp::Node
{
public:
  explicit Rotterdam(rclcpp::NodeOptions options);

protected:
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    mekong_subscription_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr murray_publisher_;
};
}  // namespace reference_system

#endif  // REFERENCE_SYSTEM__ROTTERDAM_HPP_
