import pdb
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir

from launch_ros.actions import Node


def robot_launch_setup(context, *args, **kwargs):
    # https://github.com/ROBOTIS-GIT/turtlebot3/tree/ros2/turtlebot3_bringup/launch
    os.environ['TURTLEBOT3_MODEL'] = LaunchConfiguration('model').perform(context)
    os.environ['LDS_MODEL'] = LaunchConfiguration('lds').perform(context)
    return

def generate_launch_description():

    # launch tf static transform
    map_broadcaster_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_broadcaster',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'occupancy_grid'],
    )

    turtlebot3_bringup_dir = get_package_share_directory('turtlebot3_bringup')
    robot_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            [turtlebot3_bringup_dir, '/launch/robot.launch.py']),
    )

    # occ_grid publisher
    # https://github.com/ROBOTIS-GIT/turtlebot3/blob/ros2/turtlebot3_cartographer/launch/cartographer.launch.py

    part2_share_dir = get_package_share_directory('part2')

    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='robot0')
    robot_name_config = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    open_rviz_arg = DeclareLaunchArgument('open_rviz', default_value='true')
    should_open_rviz = LaunchConfiguration('open_rviz')

    cartographer_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            [part2_share_dir, '/launch/cartographer_launch.py']),
        launch_arguments={'use_rviz': should_open_rviz, 'robot_name': robot_name_config, 'use_sim_time': use_sim_time}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='burger', description='model type [burger, waffle, waffle_pi]'),
        DeclareLaunchArgument('lds', default_value='LDS-01'),
        OpaqueFunction(function=robot_launch_setup),
        #robot_launch,
        map_broadcaster_node,
        open_rviz_arg,
        robot_name_arg,
        cartographer_launch,
    ])
