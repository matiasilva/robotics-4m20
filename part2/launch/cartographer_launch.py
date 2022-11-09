# Original: https://github.com/ROBOTIS-GIT/turtlebot3/blob/ros2/turtlebot3_cartographer/launch/cartographer.launch.py
#
# Modified to allow namespacing of cartographer and occupancy grid nodes
#
#
#
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    turtlebot3_cartographer_prefix = get_package_share_directory('turtlebot3_cartographer')

    robot_name = TextSubstitution(text="robot0")
    robot_name_launch_config = LaunchConfiguration('robot_name')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  get_package_share_directory('part2'), 'configs'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='turtlebot3_lds_2d_part2.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    #rviz_config_dir = os.path.join(get_package_share_directory('turtlebot3_cartographer'),
    #                               'rviz', 'tb3_cartographer.rviz')
    rviz_config_dir = os.path.join(get_package_share_directory('part2'),
                                   'configs', 'rviz_config.rviz')


    return LaunchDescription([

        DeclareLaunchArgument(
            'robot_name',
            default_value=robot_name),
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            namespace=robot_name_launch_config,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename]),

        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        GroupAction(
            actions=[
                # push-ros-namespace to set namespace of included nodes
                PushRosNamespace(robot_name_launch_config),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([turtlebot3_cartographer_prefix, '/launch/occupancy_grid.launch.py']),
                    launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
                                      'publish_period_sec': publish_period_sec}.items(),
                ),
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
            output='screen'),
    ])
