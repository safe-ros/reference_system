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

#ifndef REFERENCE_SYSTEM__PUBLISHER_NODE_HPP_
#define REFERENCE_SYSTEM__PUBLISHER_NODE_HPP_

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace reference_system
{
struct PublisherConfig
{
  std::string node_name;
  std::string topic;
  std::chrono::milliseconds publish_period;
  size_t queue_depth = 10;
};

template<typename MsgType>
class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode(PublisherConfig config, rclcpp::NodeOptions options)
  : Node(config.node_name, options)
  {
    publisher_ = create_publisher<MsgType>(config.topic, config.queue_depth);
    timer_ = create_wall_timer(config.publish_period, std::bind(&PublisherNode::on_timer, this));
  }

  virtual ~PublisherNode() = default;

  virtual void populate_msg(MsgType& /*msg*/) {};

protected:
  void on_timer()
  {
    auto msg = std::make_unique<MsgType>();
    populate_msg(*msg);
    publisher_->publish(std::move(msg));
  }

  typename rclcpp::Publisher<MsgType>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

};
}  // namespace reference_system

#endif  // REFERENCE_SYSTEM__PUBLISHER_NODE_HPP_
