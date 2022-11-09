from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='part2',
            executable='rrt_navigation',
            name='rrt_navigation',
            arguments=['--use_webots'],
            output='screen',
            namespace='robot0'),
    ])
