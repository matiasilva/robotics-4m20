#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import pdb

import numpy as np
import rclpy
import os
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist
# Laser scan message:
# http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# For groundtruth information.
from scipy.spatial.transform import Rotation as R


def braitenberg(front, front_left, front_right, left, right):
    u = 0.  # [m/s]
    w = 0.  # [rad/s] going counter-clockwise.

    rad = 0.066
    axle = 0.16

    # limits of distance sensor from 0.5 to 3.5

    max_u = .2
    max_w = np.pi / 4.
    safety_dist = .5

    # vr, vl is a linear sum of weighted inputs
    # each sensor input needs a pre-treatment with a smooth function
    # the pre treatment 'normalizes' the inputs wrt to the safety distance
    fl = pre_treat(front_left, safety_dist)
    fr = pre_treat(front_right, safety_dist)
    l = pre_treat(left, safety_dist)
    r = pre_treat(right, safety_dist)
    f = pre_treat(front, safety_dist)

    vl = 2 + 0.4 * fl + 0.3 * l + 0.2 * f
    vr = 2 + 0.4 * fr + 0.3 * r - 0.2 * f

    u = rad * 0.5 * (vl + vr)
    w = (rad/axle) * (vr - vl)

    # Solution:
    return u, w

def pre_treat(x, safety_dist):
    # sensor limits
    if x == np.inf or x > 3.5:
        x = 3.5

    if x < 0.012:
        x = 0.012

    return np.exp(safety_dist)*np.exp(-x)


# rule base, has to be renamed so that it can be run
def braaitenberg(front, front_left, front_right, left, right):
    u = 0.  # [m/s]
    w = 0.  # [rad/s] going counter-clockwise.

    rad = 0.066
    axle = 0.16

    # MISSING: Implement a rule-based controller that avoids obstacles.
    max_u = .2
    max_w = np.pi / 4.
    safety_dist = .5

    vl = 2
    vr = 2

    # Solution:
    if (front - safety_dist) < 0.1:
        vl = vl + 3

    if (front_left - safety_dist) < 0.1:
        vl = vl + 3

    if (front_right - safety_dist) < 0.1:
        vr = vr + 3

    u = rad * 0.5 * (vl + vr)
    w = (rad/axle) * (vr - vl)

    return u, w


class SimpleLaser(Node):
    def __init__(self):
        super().__init__('simple_laser')#try subnode
        self._angles = [0., np.pi / 4., -np.pi / 4., np.pi / 2., -np.pi / 2.]
        self._width = np.pi / 180. * 10.  # 10 degrees cone of view.
        self._measurements = [float('inf')] * len(self._angles)
        self._indices = None

    def callback(self, msg):
        # Helper for angles.
        def _within(x, a, b):
            pi2 = np.pi * 2.
            x %= pi2
            a %= pi2
            b %= pi2
            if a < b:
                return a <= x <= b
            return a <= x or x <= b

        # Compute indices the first time.
        if self._indices is None:
            self._indices = [[] for _ in range(len(self._angles))]
            for i, d in enumerate(msg.ranges):
                angle = msg.angle_min + i * msg.angle_increment
                for j, center_angle in enumerate(self._angles):
                    if _within(angle, center_angle - self._width / 2., center_angle + self._width / 2.):
                        self._indices[j].append(i)

        ranges = np.array(msg.ranges)
        for i, idx in enumerate(self._indices):
            # np.percentile outputs NaN if there are less than 2 actual numbers in the input
            if np.isinf(ranges[idx]).sum() <= len(ranges[idx]) - 2:
                # We do not take the minimum range of the cone but the 10-th percentile for robustness.
                self._measurements[i] = np.nanpercentile(ranges[idx], 10)
            else:
                self._measurements[i] = np.inf

    @property
    def measurements(self):
        return self._measurements


class GroundtruthPose(Node):
    def __init__(self):
        super().__init__('groundtruth_pose')
        self._pose = [np.nan, np.nan, np.nan]

    def callback(self, msg):
        self._pose[0] = msg.pose.pose.position.x
        self._pose[1] = msg.pose.pose.position.y
        _, _, yaw = R.from_quat([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z + 1,
            msg.pose.pose.orientation.w]).as_rotvec()
        self._pose[2] = yaw

    @property
    def pose(self):
        return self._pose


class ObstacleAvoidance(Node):

    def __init__(self, args):
        super().__init__('obstacle_avoidance')
        self._avoidance_method = globals()[args.mode]
        self._laser = SimpleLaser()
        self._laser_subscriber = self.create_subscription(LaserScan, 'TurtleBot3Burger/scan', self._laser.callback, 5)
        self._publisher = self.create_publisher(Twist, 'cmd_vel', 5)
        self._groundtruth = GroundtruthPose()
        self._groundtruth_subscriber = self.create_subscription(Odometry, '/robot0/diffdrive_controller/odom',
                                                                self._groundtruth.callback, 5)
        share_tmp_dir = os.path.join(get_package_share_directory('part1'), 'tmp')
        print(share_tmp_dir)
        os.makedirs(share_tmp_dir, exist_ok=True)
        file_path = os.path.join(share_tmp_dir, 'webots_exercise.txt')
        self._temp_file = file_path
        self._pose_history = []
        self._rate_limiter = self.create_timer(timer_period_sec=0.1, callback=self.timer_callback)

        # Keep track of groundtruth position for plotting purposes.
        with open(self._temp_file, 'w+'):
            pass

    def timer_callback(self):
        u, w = self._avoidance_method(*self._laser.measurements)
        vel_msg = Twist()
        vel_msg.linear.x = u
        vel_msg.angular.z = w
        self._publisher.publish(vel_msg)

        # Log groundtruth positions in: tmp/webots_exercise.txt
        self._pose_history.append(self._groundtruth.pose)
        if len(self._pose_history) % 10:
            with open(self._temp_file, 'a') as fp:
                fp.write('\n'.join(','.join(str(v) for v in p) for p in self._pose_history) + '\n')
                self._pose_history = []


def run(args):
    rclpy.init()
    obstacle_avoidance = ObstacleAvoidance(args)

    rclpy.spin(obstacle_avoidance)

    obstacle_avoidance.destroy_node()
    rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Runs obstacle avoidance')
    parser.add_argument('--mode', action='store', default='rule_based', help='Method.',
                        choices=['braitenberg', 'rule_based'])
    args, unknown = parser.parse_known_args()
    run(args)



if __name__ == '__main__':
    main()
