from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='part2',
            executable='send_clock',
            name='send_clock'),
    ])
