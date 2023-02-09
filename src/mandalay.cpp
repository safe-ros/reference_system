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

#include "reference_system/mandalay.hpp"
#include "utils.hpp"

namespace reference_system
{

Mandalay::Mandalay(rclcpp::NodeOptions options)
: Node("mandalay", options)
{
  tagus_publisher_ = create_publisher<geometry_msgs::msg::Pose>("tagus", 10);
  missouri_publisher_ = create_publisher<sensor_msgs::msg::Image>("missouri", 10);
  brazos_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("brazos", 10);

  danube_subscription_ = create_subscription<std_msgs::msg::String>(
    "danube",
    10,
    [](std_msgs::msg::String::UniquePtr msg) {
      (void) msg;
    });

  chenab_subscription_ = create_subscription<geometry_msgs::msg::Quaternion>(
    "chenab",
    10,
    [](geometry_msgs::msg::Quaternion::UniquePtr msg) {
      (void) msg;
    });

  salween_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "salween",
    10,
    [](sensor_msgs::msg::PointCloud2::UniquePtr msg) {
      (void) msg;
    });

  godavari_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "godavari",
    10,
    [](sensor_msgs::msg::LaserScan::UniquePtr msg) {
      (void) msg;
    });

  yamuna_subscription_ = create_subscription<geometry_msgs::msg::Vector3>(
    "yamuna",
    10,
    [](geometry_msgs::msg::Vector3::UniquePtr msg) {
      (void) msg;
    });

  loire_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "loire",
    10,
    [](sensor_msgs::msg::PointCloud2::UniquePtr msg) {
      (void) msg;
    });


  timer_ = create_wall_timer(
    std::chrono::milliseconds(100), [this]() {
      {
        geometry_msgs::msg::Pose msg = random_pose();
        tagus_publisher_->publish(msg);
      }
      {
        sensor_msgs::msg::Image msg = random_image(100);
        missouri_publisher_->publish(msg);
      }
      {
        sensor_msgs::msg::PointCloud2 msg = random_pointcloud(100);
        brazos_publisher_->publish(msg);
      }
    });
}

}  // namespace reference_system

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(reference_system::Mandalay)
