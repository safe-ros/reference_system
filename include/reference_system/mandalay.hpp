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

#ifndef REFERENCE_SYSTEM__MANDALAY_HPP_
#define REFERENCE_SYSTEM__MANDALAY_HPP_

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"

namespace reference_system
{
class Mandalay : public rclcpp::Node
{
public:
  explicit Mandalay(rclcpp::NodeOptions options);

protected:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr danube_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr chenab_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr salween_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr godavari_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr yamuna_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr loire_subscription_;

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr tagus_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr missouri_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr brazos_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace reference_system

#endif  // REFERENCE_SYSTEM__MANDALAY_HPP_
