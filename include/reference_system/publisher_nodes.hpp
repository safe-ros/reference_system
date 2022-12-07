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

#ifndef REFERENCE_SYSTEM__PUBLISHER_NODES_HPP_
#define REFERENCE_SYSTEM__PUBLISHER_NODES_HPP_

#include "rclcpp/rclcpp.hpp"
#include "reference_system/publisher_node.hpp"

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace reference_system
{

class Cordoba final : public PublisherNode<std_msgs::msg::Float32>
{
public:
  Cordoba(rclcpp::NodeOptions options);
  ~Cordoba() final = default;
};

class Freeport final : public PublisherNode<std_msgs::msg::Int64>
{
public:
  Freeport(rclcpp::NodeOptions options);
  ~Freeport() final = default;
};

class Medellin final : public PublisherNode<std_msgs::msg::Int32>
{
public:
  Medellin(rclcpp::NodeOptions options);
  ~Medellin() final = default;
};

class Portsmouth final : public PublisherNode<std_msgs::msg::String>
{
public:
  Portsmouth(rclcpp::NodeOptions options);
  ~Portsmouth() final = default;
};

class Delhi final : public PublisherNode<sensor_msgs::msg::Image>
{
public:
  Delhi(rclcpp::NodeOptions options);
  ~Delhi() final = default;
};

class Hebron final : public PublisherNode<geometry_msgs::msg::Quaternion>
{
public:
  Hebron(rclcpp::NodeOptions options);
  ~Hebron() final = default;
};

class Kingston final : public PublisherNode<geometry_msgs::msg::Vector3>
{
public:
  Kingston(rclcpp::NodeOptions options);
  ~Kingston() final = default;
};
}  // namespace reference_system
#endif  // REFERENCE_SYSTEM__PUBLISHER_NODES_HPP_
