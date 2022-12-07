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

#include "reference_system/ponce.hpp"

namespace reference_system
{

Ponce::Ponce(rclcpp::NodeOptions options)
: Node("ponce", options)
{
  congo_publisher_ = create_publisher<geometry_msgs::msg::Twist>("congo", 10);
  mekong_publisher_ =
    create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("mekong", 10);

  tagus_subscription_ = create_subscription<geometry_msgs::msg::Pose>(
    "tagus",
    10,
    [](geometry_msgs::msg::Pose::UniquePtr msg) {
      (void) msg;
    });

  danube_subscription_ = create_subscription<std_msgs::msg::String>(
    "danube",
    10,
    [](std_msgs::msg::String::UniquePtr msg) {
      (void) msg;
    });

  missouri_subscription_ = create_subscription<sensor_msgs::msg::Image>(
    "missouri",
    10,
    [](sensor_msgs::msg::Image::UniquePtr msg) {
      (void) msg;
    });

  brazos_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "brazos",
    10,
    [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) {
      (void) msg;
      auto cmd = geometry_msgs::msg::TwistWithCovarianceStamped();
      this->congo_publisher_->publish(cmd.twist.twist);
      this->mekong_publisher_->publish(cmd);
    });

  yamuna_subscription_ = create_subscription<geometry_msgs::msg::Vector3>(
    "yamuna",
    10,
    [](geometry_msgs::msg::Vector3::UniquePtr msg) {
      (void) msg;
    });

  godavari_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "godavari",
    10,
    [](sensor_msgs::msg::LaserScan::UniquePtr msg) {
      (void) msg;
    });

  loire_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "loire",
    10,
    [](sensor_msgs::msg::PointCloud2::UniquePtr msg) {
      (void) msg;
    });

  ohio_subscription_ = create_subscription<std_msgs::msg::Float32>(
    "ohio",
    10,
    [](std_msgs::msg::Float32::UniquePtr msg) {
      (void) msg;
    });

  volga_subscription_ = create_subscription<std_msgs::msg::Float64>(
    "volga",
    10,
    [](std_msgs::msg::Float64::UniquePtr msg) {
      (void) msg;
    });
}

}  // namespace reference_system

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(reference_system::Ponce)
