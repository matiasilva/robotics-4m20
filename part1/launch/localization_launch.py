from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration, TextSubstitution


def prepare_launch_nodes(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time', default=True).perform(context)
    ns = LaunchConfiguration('robot_name', default="robot0").perform(context)
    return [ExecuteProcess(
                cmd=['ros2', 'run', 'part1', 'localization', '--ros-args', '--remap', '__ns:=/'+ns, '-p', 'use_sim_time:='+use_sim_time],
                output='both',
            )]


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value=TextSubstitution(text="robot0")),
        GroupAction(
            actions=[PushRosNamespace(LaunchConfiguration('robot_name')),
                     OpaqueFunction(function=prepare_launch_nodes),
                     ]),
    ])