from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='part2',
            executable='potential_field_navigation',
            name='potential_field_navigation',
            output='screen',
            namespace='robot0'),
    ])
