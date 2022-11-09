#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import numpy as np
import rclpy
from rclpy.node import Node

# For pose information.
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry

ROBOT_RADIUS = 0.105 / 2.
ROBOT_COUNT = 5
DESTINATION_AREA_SIDE_OFFSET = 3.

X = 0
Y = 1
YAW = 2


class Robot(object):
    def __init__(self, name: str):
        self._name = name
        self._pose = [np.nan, np.nan, np.nan]

    def pose_callback(self, msg):
        self._pose[0] = msg.pose.pose.position.x
        self._pose[1] = msg.pose.pose.position.y
        _, _, yaw = R.from_quat([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w]).as_euler('XYZ')
        self._pose[2] = yaw

    @property
    def get_name(self):
        return self._name

    @property
    def pose(self):
        return self._pose

    @property
    def ready(self):
        return not np.isnan(self._pose[0])


class MultirobotDriver(Node):
    def __init__(self, args):
        super().__init__('multirobot_driver')
        self._destinations = self._generate_destinations()
        self._robots = [Robot('robot' + str(x)) for x in range(ROBOT_COUNT)]
        self._pose_subscribers = [self.create_subscription(Odometry, robot.get_name + '/odom',
                                                           robot.pose_callback, 5) for robot in self._robots]
        self._rate_limiter = self.create_timer(timer_period_sec=0.1, callback=self.timer_callback)

    @staticmethod
    def _generate_destinations():
        def is_valid(pose, poses):
            for p1 in poses:
                if not np.linalg.norm(pose[:2] - p1[:2]) >= 2 * ROBOT_RADIUS:
                    return False
            return -wall_offset <= pose[X] <= wall_offset and -wall_offset <= pose[Y] <= wall_offset

        poses = []
        for _ in range(ROBOT_COUNT):
            pose = np.zeros(2, dtype=np.float32)
            wall_offset = DESTINATION_AREA_SIDE_OFFSET - ROBOT_RADIUS
            pose[X] = (np.random.rand() - 0.5) * 2. * wall_offset
            pose[Y] = (np.random.rand() - 0.5) * 2. * wall_offset
            while not is_valid(pose, poses):
                pose[X] = (np.random.rand() - 0.5) * 2. * wall_offset
                pose[Y] = (np.random.rand() - 0.5) * 2. * wall_offset
            poses.append(pose)
        return poses

    def timer_callback(self):
        # An implementation for sending periodic commands to the robot
        # could potentially go here (as well as other periodic methods).
        # NOTE: Goal assignment should [likely] not happen here. 
        return


def run(args):
    rclpy.init()

    multirobot_driver_node = MultirobotDriver(args)

    rclpy.spin(multirobot_driver_node)

    multirobot_driver_node.destroy_node()
    rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Runs multi-robot navigation')
    args, unknown = parser.parse_known_args()
    run(args)


if __name__ == '__main__':
    main()
