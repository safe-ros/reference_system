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

#ifndef REFERENCE_SYSTEM__HEBRON_HPP_
#define REFERENCE_SYSTEM__HEBRON_HPP_

#include "rclcpp/rclcpp.hpp"
#include "reference_system/publisher_node.hpp"

#include "geometry_msgs/msg/quaternion.hpp"

namespace reference_system
{

class Hebron final : public PublisherNode<geometry_msgs::msg::Quaternion>
{
public:
  Hebron(rclcpp::NodeOptions options);
  ~Hebron() final = default;

  void populate_msg(geometry_msgs::msg::Quaternion &msg) final;
};

}  // namespace reference_system
#endif  // REFERENCE_SYSTEM__HEBRON_HPP_