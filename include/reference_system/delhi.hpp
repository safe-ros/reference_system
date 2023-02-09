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

#ifndef REFERENCE_SYSTEM__DELHI_HPP_
#define REFERENCE_SYSTEM__DELHI_HPP_

#include "rclcpp/rclcpp.hpp"
#include "reference_system/publisher_node.hpp"

#include "sensor_msgs/msg/image.hpp"

namespace reference_system
{

class Delhi final : public PublisherNode<sensor_msgs::msg::Image>
{
public:
  Delhi(rclcpp::NodeOptions options);
  ~Delhi() final = default;

  void populate_msg(sensor_msgs::msg::Image &msg) final;
};

}  // namespace reference_system
#endif  // REFERENCE_SYSTEM__DELHI_HPP_
