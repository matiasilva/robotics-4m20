#!/usr/bin/env _python_exercises

# Copyright 1996-2021 Cyberbotics Ltd.
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

"""Launch Webots TurtleBot3 Burger driver. Modified from webots_ros2_turtlebot package."""

import os
import pathlib

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace

from launch import LaunchDescription


def prepare_launch_nodes(context, *args, **kwargs):
    robot_dir = get_package_share_directory('webots_ros2_turtlebot')
    package_dir = get_package_share_directory('part0')
    robot_description = pathlib.Path(os.path.join(robot_dir, 'resource', 'turtlebot_webots.urdf')).read_text()
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2control.yml')  # our own file
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    robot_name = LaunchConfiguration('robot_name', default="robot0").perform(context)  # get string

    # TODO: Revert once the https://github.com/ros-controls/ros2_control/pull/444 PR gets into the release
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    # TODO: Ugly hack for namespacing controller_manager node
    controller_mgr_ns_node = ["-c", "/" + robot_name + "/controller_manager"]
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    controller_manager_update_rate = 32

    use_deprecated_spawner_py = 'ROS_DISTRO' in os.environ and os.environ['ROS_DISTRO'] == 'foxy'

    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner' if not use_deprecated_spawner_py else 'spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout + controller_mgr_ns_node,
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner' if not use_deprecated_spawner_py else 'spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout + controller_mgr_ns_node,
    )

    mappings = [('diffdrive_controller/cmd_vel_unstamped', 'cmd_vel')]
    mappings.append(('/odom', 'odom'))  # force-remove leading '/'
    if ('ROS_DISTRO' in os.environ and os.environ['ROS_DISTRO'] == 'rolling') or \
            ('ROS_REPO' in os.environ and os.environ['ROS_REPO'] == 'testing'):
        mappings.append(('diffdrive_controller/odom', 'odom'))

    turtlebot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        parameters=[
            {'robot_description': robot_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
            ros2_control_params
        ],
        remappings=mappings
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    return [
        joint_state_broadcaster_spawner,
        diffdrive_controller_spawner,
        robot_state_publisher,
        turtlebot_driver,
        footprint_publisher,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value=TextSubstitution(text="robot0")),
        GroupAction(
            actions=[PushRosNamespace(LaunchConfiguration('robot_name')),
                     OpaqueFunction(function=prepare_launch_nodes),
                     ]
        )
    ])
