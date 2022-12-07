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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <geometry_msgs/msg/detail/twist_with_covariance__struct.hpp>
#include <geometry_msgs/msg/detail/twist_with_covariance_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/wrench_stamped__struct.hpp>
#include <random>
#include <sensor_msgs/msg/detail/laser_scan__struct.hpp>
#include <sensor_msgs/msg/detail/point_field__struct.hpp>
#include <vector>
#include <algorithm>
#include <limits>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>


namespace reference_system
{


template<typename T,
  typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type>
auto random_number(T min = std::numeric_limits<T>::min(), T max = std::numeric_limits<T>::max())
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dist(min, max);

  return dist(gen);
}

template<typename T,
  typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type>
auto random_number_vector(
  size_t length,
  T min = std::numeric_limits<T>::min(),
  T max = std::numeric_limits<T>::max())
{
  auto randnum = [&]() -> T
    {
      return random_number<T>(min, max);
    };

  std::vector<T> out(length);
  std::generate(out.begin(), out.end(), randnum);

  return out;
}

template<typename T, int len,
  typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type>
auto random_number_array(
  T min = std::numeric_limits<T>::min(),
  T max = std::numeric_limits<T>::max())
{
  auto randnum = [&]() -> T
    {
      return random_number<T>(min, max);
    };

  std::array<T, len> out;
  std::generate(out.begin(), out.end(), randnum);

  return out;
}

// MSG GEN =========================================================================================

auto random_string(size_t length) -> std::string;
auto random_header(size_t len = 16) -> std_msgs::msg::Header;
auto random_quaternion() -> geometry_msgs::msg::Quaternion;
auto random_point() -> geometry_msgs::msg::Point;
auto random_pose() -> geometry_msgs::msg::Pose;
auto random_vector3() -> geometry_msgs::msg::Vector3;
auto random_vector3stamped() -> geometry_msgs::msg::Vector3Stamped;
auto random_twist() -> geometry_msgs::msg::Twist;
auto random_twistwithcovariance() -> geometry_msgs::msg::TwistWithCovariance;
auto random_twistwithcovariancestamped() -> geometry_msgs::msg::TwistWithCovarianceStamped;
auto random_wrench() -> geometry_msgs::msg::Wrench;
auto random_wrenchstamped() -> geometry_msgs::msg::WrenchStamped;
auto random_image(size_t len = 0) -> sensor_msgs::msg::Image;
auto random_pointfield(size_t len = 32) -> sensor_msgs::msg::PointField;
auto random_pointcloud(size_t len = 0)  -> sensor_msgs::msg::PointCloud2;
auto random_laserscan(size_t len = 1024) -> sensor_msgs::msg::LaserScan;
}  // namespace reference_system

#endif  // UTILS_HPP_
