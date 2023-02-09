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

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""

    robot_container = ComposableNodeContainer(
            name='reference_system_robot',
            namespace='',
            package='rclcpp_components',
            executable='component_container_isolated',
            composable_node_descriptions=[
                ComposableNode(
                    package='reference_system',
                    plugin='reference_system::Cordoba',
                ),
                ComposableNode(
                    package='reference_system',
                    plugin='reference_system::Freeport',
                ),
                ComposableNode(
                    package='reference_system',
                    plugin='reference_system::Medellin',
                ),
                ComposableNode(
                    package='reference_system',
                    plugin='reference_system::Portsmouth',
                ),
                ComposableNode(
                    package='reference_system',
                    plugin='reference_system::Delhi',
                ),
                ComposableNode(
                    package='reference_system',
                    plugin='reference_system::Taipei',
                ),
                ComposableNode(
                    package='reference_system',
                    plugin='reference_system::Lyon',
                ),
                ComposableNode(
                    package='reference_system',
                    plugin='reference_system::Hebron',
                ),
                ComposableNode(
                    package='reference_system',
                    plugin='reference_system::Kingston',
                ),
                ComposableNode(
                    package='reference_system',
                    plugin='reference_system::Hamburg',
                ),
                ComposableNode(
                    package='reference_system',
                    plugin='reference_system::Osaka',
                ),
                ComposableNode(
                    package='reference_system',
                    plugin='reference_system::Tripoli',
                ),
                ComposableNode(
                    package='reference_system',
                    plugin='reference_system::Mandalay',
                ),
                ComposableNode(
                    package='reference_system',
                    plugin='reference_system::Ponce',
                )
            ],
            output='screen',
    )

    control_container = ComposableNodeContainer(
            name='reference_system_control',
            namespace='',
            package='rclcpp_components',
            executable='component_container_isolated',
            composable_node_descriptions=[
                ComposableNode(
                    package='reference_system',
                    plugin='reference_system::Geneva',
                ),
                ComposableNode(
                    package='reference_system',
                    plugin='reference_system::Monaco',
                ),
                ComposableNode(
                    package='reference_system',
                    plugin='reference_system::Rotterdam',
                ),
                ComposableNode(
                    package='reference_system',
                    plugin='reference_system::Barcelona',
                ),
                ComposableNode(
                    package='reference_system',
                    plugin='reference_system::Arequipa',
                ),
                ComposableNode(
                    package='reference_system',
                    plugin='reference_system::Georgetown',
                )
            ],
            output='screen',
    )

    return launch.LaunchDescription([robot_container, control_container])
