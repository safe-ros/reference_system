import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


'''
This requires cloning the demos submodule:
git submodule update --init --recursive

'''

def generate_launch_description():
    dummy_robot_bringup = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('dummy_robot_bringup'), 'launch'),
         '/dummy_robot_bringup_launch.py'])
      )
    dummy_robot_bringup_with_namespace = GroupAction(
     actions=[
         PushRosNamespace('dummy_robot'),
         dummy_robot_bringup,
      ]
   )

    fib_action_server = Node(
            package='action_tutorials_cpp',
            namespace='fib_actions',
            executable='fibonacci_action_server',
            name='action_server'
        )

    return LaunchDescription([
        dummy_robot_bringup_with_namespace,
        fib_action_server,
    ])