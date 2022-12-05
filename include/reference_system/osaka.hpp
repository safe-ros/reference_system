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

#ifndef REFERENCE_SYSTEM__OSAKA_HPP_
#define REFERENCE_SYSTEM__OSAKA_HPP_

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"

namespace reference_system
{
class Osaka : public rclcpp::Node
{
public:
  explicit Osaka(rclcpp::NodeOptions options);

protected:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr colorado_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr columbia_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr parana_subscription_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr salween_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr godavari_publisher_;
};
}  // namespace reference_system
#endif  // REFERENCE_SYSTEM__OSAKA_HPP_
