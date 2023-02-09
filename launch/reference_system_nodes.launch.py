# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch the reference system across two containers"""

from launch import LaunchDescription
from launch_ros.actions import Node

REFERENCE_NODES = [
  "arequipa",
  "barcelona",
  "cordoba",
  "delhi",
  "freeport",
  "geneva",
  "georgetown",
  "hamburg",
  "hebron",
  "kingston",
  "lyon",
  "osaka",
  "mandalay",
  "medellin",
  "monaco",
  "ponce",
  "portsmouth",
  "rotterdam",
  "taipei",
  "tripoli"
]

def generate_launch_description():
    """Generate launch description with multiple components."""

    return LaunchDescription([
        Node(package='reference_system', executable=node, output='screen')
            for node in REFERENCE_NODES])
