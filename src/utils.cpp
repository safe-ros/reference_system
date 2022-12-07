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

#include "utils.hpp"

#include <algorithm>
#include <limits>
#include <random>
#include <string>

namespace reference_system
{

// Modified from:
// https://stackoverflow.com/questions/440133/how-do-i-create-a-random-alpha-numeric-string-in-c
  std::string random_string(size_t length)
{
  auto randchar = []() -> char
    {
      const char charset[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "`~!@#$%^&*()_+"
        "[]\\;',./{}|:\"<>?";
      const size_t max_index = (sizeof(charset) - 1);
      return charset[rand() % max_index];
    };
  std::string str(length, 0);
  std::generate_n(str.begin(), length, randchar);
  return str;
}

// MSG GEN =========================================================================================
auto random_header(size_t len) -> std_msgs::msg::Header
{
  std_msgs::msg::Header header_msg;
  header_msg.stamp.sec = random_number<int32_t>();
  header_msg.stamp.nanosec = random_number<uint32_t>();
  header_msg.frame_id = random_string(len);
  return header_msg;
}

auto random_quaternion() -> geometry_msgs::msg::Quaternion
{
  geometry_msgs::msg::Quaternion q_msg;
  q_msg.x = random_number<double>();
  q_msg.y = random_number<double>();
  q_msg.z = random_number<double>();
  q_msg.w = random_number<double>();
  return q_msg;
}

auto random_point() -> geometry_msgs::msg::Point
{
  geometry_msgs::msg::Point p_msg;
  p_msg.x = random_number<double>();
  p_msg.y = random_number<double>();
  p_msg.z = random_number<double>();
  return p_msg;
}

auto random_pose() -> geometry_msgs::msg::Pose
{
  geometry_msgs::msg::Pose pose_msg;
  pose_msg.position = random_point();
  pose_msg.orientation = random_quaternion();
  return pose_msg;
}

auto random_vector3() -> geometry_msgs::msg::Vector3
{
  geometry_msgs::msg::Vector3 vec_msg;
  vec_msg.x = random_number<double>();
  vec_msg.y = random_number<double>();
  vec_msg.z = random_number<double>();
  return vec_msg;
}

auto random_vector3stamped() -> geometry_msgs::msg::Vector3Stamped
{
  geometry_msgs::msg::Vector3Stamped vec_msg;

  vec_msg.header = random_header();
  vec_msg.vector = random_vector3();

  return vec_msg;
}

auto random_twist() -> geometry_msgs::msg::Twist
{
  geometry_msgs::msg::Twist twist_msg;

  twist_msg.linear = random_vector3();
  twist_msg.angular = random_vector3();

  return twist_msg;
}

auto random_twistwithcovariance() -> geometry_msgs::msg::TwistWithCovariance
{
  geometry_msgs::msg::TwistWithCovariance twist_msg;

  twist_msg.twist = random_twist();
  twist_msg.covariance = random_number_array<double, 36>();

  return twist_msg;
}

auto random_twistwithcovariancestamped() -> geometry_msgs::msg::TwistWithCovarianceStamped
{
  geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;

  twist_msg.header = random_header();
  twist_msg.twist = random_twistwithcovariance();

  return twist_msg;
}

auto random_wrench() -> geometry_msgs::msg::Wrench
{
  geometry_msgs::msg::Wrench wrench_msg;

  wrench_msg.force = random_vector3();
  wrench_msg.torque = random_vector3();

  return wrench_msg;
}

auto random_wrenchstamped() -> geometry_msgs::msg::WrenchStamped
{
  geometry_msgs::msg::WrenchStamped wrench_msg;

  wrench_msg.header = random_header();
  wrench_msg.wrench = random_wrench();

  return wrench_msg;
}

auto random_image(size_t len) -> sensor_msgs::msg::Image
{
  sensor_msgs::msg::Image image_msg;

  image_msg.header = random_header();

  image_msg.height = random_number<uint32_t>();
  image_msg.width = random_number<uint32_t>();

  image_msg.encoding = random_string(32);
  image_msg.is_bigendian = random_number<int>(0, 1);
  image_msg.step = random_number<uint32_t>();

  image_msg.data = random_number_vector<uint8_t>(len);

  return image_msg;
}

auto random_pointfield(size_t len) -> sensor_msgs::msg::PointField
{
  sensor_msgs::msg::PointField pt_msg;

  pt_msg.name = random_string(len);
  pt_msg.offset = random_number<uint32_t>();
  pt_msg.datatype = random_number<uint8_t>();
  pt_msg.count = random_number<uint32_t>(0, 1);

  return pt_msg;
}

auto random_pointcloud(size_t len)  -> sensor_msgs::msg::PointCloud2
{
  sensor_msgs::msg::PointCloud2 pc_msg;

  pc_msg.header = random_header();

  pc_msg.height = random_number<uint32_t>();
  pc_msg.width = random_number<uint32_t>();

  std::vector<sensor_msgs::msg::PointField> pts(3);
  for (int i = 0; i < 3; ++i) {
    pts.push_back(random_pointfield());
  }

  pc_msg.fields = pts;

  pc_msg.is_bigendian = random_number<int>(0, 1);
  pc_msg.point_step = random_number<uint32_t>();
  pc_msg.row_step = random_number<uint32_t>();

  pc_msg.data = random_number_vector<uint8_t>(len);

  pc_msg.is_dense = random_number<int>(0, 1);

  return pc_msg;
}

auto random_laserscan(size_t len) -> sensor_msgs::msg::LaserScan
{
  sensor_msgs::msg::LaserScan scan_msg;

  scan_msg.header = random_header();

  scan_msg.angle_min = random_number<float>();
  scan_msg.angle_max = random_number<float>();
  scan_msg.angle_increment = random_number<float>();
  scan_msg.time_increment = random_number<float>();
  scan_msg.scan_time = random_number<float>();
  scan_msg.range_min = random_number<float>();
  scan_msg.range_max = random_number<float>();

  scan_msg.ranges = random_number_vector<float>(len);
  scan_msg.intensities = random_number_vector<float>(len);

  return scan_msg;
}
}  // namespace reference_system
