import pdb

import launch
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from webots_ros2_driver.webots_launcher import WebotsLauncher

import os
import shutil
import numpy as np
from scipy.spatial.transform import Rotation as R

# Constants used for indexing.
X = 0
Y = 1
YAW = 2


ROBOT_COUNT = 5
ROBOT_RADIUS = 0.178 / 2.
STARTING_AREA_SIDE_OFFSET = 1.






def generate_poses():

    def is_valid(pose, poses):
        wall_offset = STARTING_AREA_SIDE_OFFSET - ROBOT_RADIUS
        for p1 in poses:
            if not np.linalg.norm(pose[:2] - p1[:2]) >= 2 * ROBOT_RADIUS:
                return False
        return -wall_offset <= pose[X] <= wall_offset and -wall_offset <= pose[Y] <= wall_offset


    poses = []
    for _ in range(ROBOT_COUNT):
        pose = np.zeros(3, dtype=np.float32)
        wall_offset = STARTING_AREA_SIDE_OFFSET - ROBOT_RADIUS
        pose[X] = (np.random.rand() - 0.5) * 2. * wall_offset
        pose[Y] = (np.random.rand() - 0.5) * 2. * wall_offset
        pose[YAW] = np.random.rand() * np.pi * 2.
        while not is_valid(pose, poses):
            pose[X] = (np.random.rand() - 0.5) * 2. * wall_offset
            pose[Y] = (np.random.rand() - 0.5) * 2. * wall_offset
        poses.append(pose)
    return poses

def generate_launch_description():

    launch_desc = LaunchDescription()

    package_dir = get_package_share_directory('part2')

    poses = generate_poses()

    worlds_dir = os.path.join(package_dir, 'worlds')
    clean_world = os.path.join(worlds_dir, 'empty_plane_world.wbt')
    populated_world = os.path.join(worlds_dir, 'multirobot_plane_world.wbt')
    world_file = shutil.copy(clean_world, populated_world)

    with open(world_file, 'a') as fp:
        for idx, pose in enumerate(poses):

            name = 'robot'+str(idx)

            trans = ' '.join(str(x) for x in pose[:2]) + ' 0'

            rot_arr = R.from_euler('Z', pose[YAW]).as_quat()
            rot = ' '.join(str(x) for x in rot_arr)

            robot_proto_string = f'TurtleBot3Burger {{\r\n  name \"{name}\"\r\n  translation {trans}\r\n  rotation {rot}\r\n  controller \"<extern>\"\r\n  controllerArgs [\r\n    \"\"\r\n  ]\r\n  extensionSlot [\r\n    Solid {{\r\n      name \"imu_link\"\r\n    }}\r\n    GPS {{\r\n    }}\r\n    InertialUnit {{\r\n      name \"inertial_unit\"\r\n    }}\r\n    RobotisLds01 {{\r\n    }}\r\n  ]\r\n}}\r\n'
            fp.write(robot_proto_string)

    world = PathJoinSubstitution([package_dir, 'worlds', 'multirobot_plane_world.wbt'])

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        mode='pause'
    )

    webots_shutdown_action = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )

    launch_desc.add_action(webots)
    launch_desc.add_action(webots_shutdown_action)

    for i in range(ROBOT_COUNT):
        robot_name = "robot"+str(i)
        robot_launch = IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource([package_dir, '/launch/robot_launch.py']),
            launch_arguments={'robot_name': robot_name}.items()
        )
        launch_desc.add_action(robot_launch)

    return launch_desc

