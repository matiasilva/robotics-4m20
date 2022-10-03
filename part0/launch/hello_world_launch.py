from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='part0',
            executable='hello_world',
            name='hello_world'),
    ])
