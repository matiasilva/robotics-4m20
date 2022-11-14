#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import copy
import os

import numpy as np
import rclpy
from rclpy.node import Node

# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist
# Laser scan message:
# http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
from sensor_msgs.msg import LaserScan
# For groundtruth information.
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation as R
# For displaying particles.
# http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud.html
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32
from geometry_msgs.msg import Point32
from std_msgs.msg import Header
# Odometry.

from random import random
from nav_msgs.msg import Odometry

# stats
from scipy.stats import norm

# Constants used for indexing.
X = 0
Y = 1
YAW = 2

ROBOT_RADIUS = 0.105 / 2.
WALL_OFFSET = 2.
CYLINDER_POSITION = np.array([.5, .6], dtype=np.float32)  # adjust from .wbt
CYLINDER_RADIUS = .3


def braitenberg(front, front_left, front_right, left, right):
    u = 0.  # [m/s]
    w = 0.  # [rad/s] going counter-clockwise.

    # physical vehicle constants
    rad = 0.066
    axle = 0.16

    # limits of distance sensor from 0.5 to 3.5
    # chosen smooth fn lnx from 0 to 1.25

    max_u = .2
    max_w = np.pi / 4.
    safety_dist = .5

    # vr, vl is a linear sum of weighted inputs
    # each sensor input needs a pre-treatment with a smooth function
    #print(front, front_left, front_right, left, right)
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
    if x == np.inf or x > 3.5:
        x = 3.5

    if x < 0.012:
        x = 0.012

    return np.exp(safety_dist)*np.exp(-x)


class Particle(object):
    """Represents a particle."""

    def __init__(self, like_pose=None):
        self._pose = np.zeros(3, dtype=np.float32)
        self._weight = 1.

        # MISSING: Initialize a particle randomly in the arena. Set the values of
        # _pose such that it is a valid pose (i.e., inside the arena walls, but
        # outside the cylinder). Consider implementing is_valid() below and use it
        # in this function. The initialisation can take an optional arg like_pose,
        # which may be used to create a particle 'similar' to the given pose, i.e.,
        # with a random variation on the given pose. Useful if you want to sample
        # particles close to a good solution.

        # Solution:
        # if no previous pose is specified, pick a location at random
        # on the entire canvas, yaw has full 360 reach
        if like_pose is None:
            self._pose[X] = np.random.uniform(-2, 2)
            self._pose[Y] = np.random.uniform(-2, 2)
            self._pose[YAW] = np.random.uniform(-np.pi, np.pi)

            while not self.is_valid():
                self._pose[X] = np.random.uniform(-2, 2)
                self._pose[Y] = np.random.uniform(-2, 2)
                self._pose[YAW] = np.random.uniform(-np.pi, np.pi)
        else:
            # generate this particle close to the given pose
            # choose normal distribution to bunch up values at desired pose
            sigma = 0.1
            self._pose[X] = like_pose[X] + np.random.normal(0, sigma)
            self._pose[Y] = like_pose[Y] + np.random.normal(0, sigma)
            self._pose[YAW] = like_pose[YAW] + np.random.normal(0, sigma)
            while not self.is_valid():
                self._pose[X] = like_pose[X] + np.random.normal(0, sigma)
                self._pose[Y] = like_pose[Y] + np.random.normal(0, sigma)
                self._pose[YAW] = like_pose[YAW] + np.random.normal(0, sigma)


    def is_valid(self):
        # MISSING: Implement a function that returns True if the current particle
        # position is valid. You might need to use this function in __init__()
        # and compute_weight().

        # Solution:

        is_not_intersecting = ((self._pose[0]-0.5)**2 + (self._pose[1]-0.6)**2) > 0.3**2
        is_within_bounds = (self._pose[0] > -2) and (self._pose[0] < 2) and (self._pose[1] > -2) and (self._pose[1] < 2)

        return is_within_bounds and is_not_intersecting 


    def move(self, delta_pose):
        # MISSING: Update the particle pose according the motion model.
        # delta_pose is an offset in the particle frame. As motion model,
        # use roughtly 10% standard deviation with respect to the forward
        # and rotational velocity.
        #
        # In a second step, make the necessary modifications to handle the
        # kidnapped robot problem. For example, with a low probability the
        # particle can be repositioned randomly in the arena.

        # Solution:

        # decompose robot's x speed as its forward speed using particle's yaw
        # effectively a conversion from local robot coords to global coords
        u = abs(delta_pose[0])
        deltax = (delta_pose[0] + np.random.normal(0, 0.1*abs(u)))*np.cos(self._pose[YAW])
        deltay = (delta_pose[0]+  np.random.normal(0, 0.1*abs(u)))*np.sin(self._pose[YAW])
        deltayaw = delta_pose[YAW]
        w = abs(delta_pose[YAW])
        
        self._pose[X] += deltax 
        self._pose[Y] += deltay
        self._pose[YAW] += deltayaw + np.random.normal(0, 0.1*w)

        # give some particles a chance to "respawn" elsewhere
        if np.random.uniform(0,1) < 0.1:
            self._pose[X] = np.random.uniform(-2, 2)
            self._pose[Y] = np.random.uniform(-2, 2)
            self._pose[YAW] = np.random.uniform(-np.pi, np.pi)

            while not self.is_valid():
                self._pose[X] = np.random.uniform(-2, 2)
                self._pose[Y] = np.random.uniform(-2, 2)
                self._pose[YAW] = np.random.uniform(-np.pi, np.pi)        

    def compute_weight(self, front, front_left, front_right, left, right):
        # MISSING: Update the particle weight self._weight according to measurements.
        # You can use the self.ray_trace(angle) function below. Remember to reduce the
        # weight of particles that are outside the arena. As measurement model, use a
        # Gaussian error with a standard deviation of 80 [cm]. Note that the maximum
        # range of the laser-range finder is 3.5 meters (observations beyond this range
        # will show up as infinity).
        sigma = .8
        variance = sigma ** 2.

        # ensure physical limits are enforced
        if front == np.inf or front > 3.5:
            front = 3.5
        if left == np.inf or left > 3.5:
            left = 3.5
        if right == np.inf or right > 3.5:
            right = 3.5
        if front_left == np.inf or front_left > 3.5:
            front_left = 3.5
        if front_right == np.inf or front_right > 3.5:
            front_right = 3.5

        # kill off any particles outside the map or on an obstacle
        if not self.is_valid():
            self._weight = 0
        else:
            # find angles of all beams
            fronta = self._pose[YAW]
            frontlefta = fronta + np.pi/4
            frontrighta = fronta - np.pi/4
            lefta = fronta + np.pi/2
            righta = fronta - np.pi/2

            # find dist to those beams
            frontdist = self.ray_trace(fronta)
            frontleftdist = self.ray_trace(frontlefta)
            frontrightdist = self.ray_trace(frontrighta)
            leftdist = self.ray_trace(lefta)
            rightdist = self.ray_trace(righta)

            # evaluate gaussian prob (comparing two distances)
            frontp = norm.pdf(frontdist, loc = front, scale = sigma)
            frontleftp = norm.pdf(frontleftdist, loc = front_left, scale = sigma)
            frontrightp = norm.pdf(frontrightdist, loc = front_right, scale = sigma)
            leftp = norm.pdf(leftdist, loc = left, scale = sigma)
            rightp = norm.pdf(rightdist, loc = right, scale = sigma)

            # final weight is product of all probabilities
            w = frontp * frontleftp * frontrightp * leftp * rightp
            self._weight = w


    def ray_trace(self, angle):
        """Returns the distance to the first obstacle from the particle."""
        wall_off = np.pi/2.
        cyl_off = np.pi
        def intersection_segment(x1, x2, y1, y2):
            point1 = np.array([x1, y1], dtype=np.float32)
            point2 = np.array([x2, y2], dtype=np.float32)
            v1 = self._pose[:2] - point1
            v2 = point2 - point1
            v3 = np.array([np.cos(angle + self._pose[YAW] + wall_off), np.sin(angle + self._pose[YAW] + wall_off)],
                          dtype=np.float32)
            t1 = np.cross(v2, v1) / np.dot(v2, v3)
            t2 = np.dot(v1, v3) / np.dot(v2, v3)
            if t1 >= 0. and t2 >= 0. and t2 <= 1.:
                return t1
            return float('inf')

        def intersection_cylinder(x, y, r):
            center = np.array([x, y], dtype=np.float32)
            v = np.array([np.cos(angle + self._pose[YAW] + cyl_off), np.sin(angle + self._pose[YAW] + cyl_off)],
                         dtype=np.float32)

            v1 = center - self._pose[:2]
            a = v.dot(v)
            b = 2. * v.dot(v1)
            c = v1.dot(v1) - r ** 2.
            q = b ** 2. - 4. * a * c
            if q < 0.:
                return float('inf')
            g = 1. / (2. * a)
            q = g * np.sqrt(q)
            b = -b * g
            d = min(b + q, b - q)
            if d >= 0.:
                return d
            return float('inf')

        d = min(intersection_segment(-WALL_OFFSET, -WALL_OFFSET, -WALL_OFFSET, WALL_OFFSET),
                intersection_segment(WALL_OFFSET, WALL_OFFSET, -WALL_OFFSET, WALL_OFFSET),
                intersection_segment(-WALL_OFFSET, WALL_OFFSET, -WALL_OFFSET, -WALL_OFFSET),
                intersection_segment(-WALL_OFFSET, WALL_OFFSET, WALL_OFFSET, WALL_OFFSET),
                intersection_cylinder(CYLINDER_POSITION[X], CYLINDER_POSITION[Y], CYLINDER_RADIUS))
        return d

    @property
    def pose(self):
        return self._pose

    @property
    def weight(self):
        return self._weight


class SimpleLaser(Node):
    def __init__(self):
        super().__init__('simple_laser')
        self._angles = [0., np.pi / 4., -np.pi / 4., np.pi / 2., -np.pi / 2.]
        self._width = np.pi / 180. * 3.1  # 3.1 degrees cone of view (3 rays).
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
    def ready(self):
        return not np.isnan(self._measurements[0])

    @property
    def measurements(self):
        return self._measurements


class Motion(Node):
    def __init__(self):
        super().__init__('motion')
        self._previous_time = None
        self._delta_pose = np.array([0., 0., 0.], dtype=np.float32)

    def callback(self, msg):
        u = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        if self._previous_time is None:
            self._previous_time = rclpy.time.Time.from_msg(msg.header.stamp);
        current_time = rclpy.time.Time.from_msg(msg.header.stamp);
        dt = (current_time-self._previous_time).nanoseconds * 1e-9;
        self._delta_pose[X] += u * dt
        self._delta_pose[Y] += 0.
        self._delta_pose[YAW] += w * dt
        self._previous_time = current_time


    @property
    def ready(self):
        return True

    @property
    def delta_pose(self):
        ret = self._delta_pose.copy()
        self._delta_pose[:] = 0
        return ret


class GroundtruthPose(Node):
    def __init__(self):
        super().__init__('groundtruth_pose')
        self._pose = np.array([np.nan, np.nan, np.nan], dtype=np.float32)

    def callback(self, msg):
        self._pose[X] = msg.pose.pose.position.x
        self._pose[Y] = msg.pose.pose.position.y
        _, _, yaw = R.from_quat([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z ,#+ 1,
            msg.pose.pose.orientation.w]).as_euler(seq="XYZ")
        self._pose[YAW] = yaw

    @property
    def ready(self):
        return not np.isnan(self._pose[0])

    @property
    def pose(self):
        return self._pose


class Localization(Node):
    def __init__(self):
        super().__init__('localization')
        self._laser = SimpleLaser()
        self._laser_subscriber = self.create_subscription(LaserScan, 'TurtleBot3Burger/scan', self._laser.callback, 5)
        self._publisher = self.create_publisher(Twist, 'cmd_vel', 5)
        self._groundtruth = GroundtruthPose()
        self._groundtruth_subscriber = self.create_subscription(Odometry, '/robot0/diffdrive_controller/odom',
                                                                self._groundtruth.callback, 5)
        self._motion = Motion()
        self._motion_subscriber = self.create_subscription(Odometry, '/robot0/diffdrive_controller/odom',
                                                           self._motion.callback, 1)

        self._particle_publisher = self.create_publisher(PointCloud, 'particles', 1)

        share_tmp_dir = os.path.join(get_package_share_directory('part1'), 'tmp')
        os.makedirs(share_tmp_dir, exist_ok=True)
        file_path = os.path.join(share_tmp_dir, 'webots_exercise.txt')
        self._temp_file = file_path

        self._num_particles = 80
        self._particles = [Particle() for _ in range(self._num_particles)]
        self._pose_history = []
        self._rate_limiter = self.create_timer(timer_period_sec=0.1, callback=self.timer_callback)

        with open(self._temp_file, 'w+'):
            pass

    def timer_callback(self):
        if self._laser.ready and self._motion.ready and self._groundtruth.ready:

            u, w = braitenberg(*self._laser.measurements)
            vel_msg = Twist()
            vel_msg.linear.x = u
            vel_msg.angular.z = w
            self._publisher.publish(vel_msg)

            # Update particle positions and weights.
            total_weight = 0.
            delta_pose = self._motion.delta_pose
            for i, p in enumerate(self._particles):
                p.move(delta_pose)
                p.compute_weight(*self._laser.measurements)
                total_weight += p.weight

            # MISSING: Low variance re-sampling of particles.
            # Implement a resampling mechanism that keeps a subset of particles
            # for the next iteration (possibly based on their relative weight
            # compared to the mean or max weight). Ensure that the number of
            # active particles doesn't fall too low, and replenish using random
            # new particles if needed.

            sensitivity = 0.4
            particle_thresh = 0.6
            # re-sampling based on maximum but can also experiment with mean

            # sort in descending order by weight
            particles_sorted = sorted(self._particles, key=lambda p:p.weight, reverse=True)
            #Node.get_logger(self).info(f'topbottom! {sortedparticles[0].weight} {sortedparticles[-1].weight}')                                        
            new_particles = []
            mean_weight = total_weight/self._num_particles
            for i, particle in enumerate(particles_sorted):
                #if abs(particle.pose[X]) > 2 or abs(particle.pose[Y]) > 2:
                #    Node.get_logger(self).info(f'LOOK! {particle.pose}, {particle.weight}')    

                if particle.weight >= sensitivity*particles_sorted[0].weight:
                    new_particles.append(particle)
            
            curr_new_particles = len(new_particles)
            
            # If number of particles fall below 60% of total particles, resample 
            if curr_new_particles < particle_thresh* self._num_particles:
                to_add = self._num_particles - curr_new_particles
                for i in range(to_add):
                    rand_int = np.random.randint(0, curr_new_particles)
                    # copy the pose of another random particle
                    new_particles.append(Particle(new_particles[rand_int].pose))

            # Solution:
            self._particles = new_particles;
            particles = new_particles;

            # Publish particles.
            particle_msg = PointCloud()
            particle_msg.header.stamp = self.get_clock().now().to_msg()
            particle_msg.header.frame_id = 'map'
            intensity_channel = ChannelFloat32()
            intensity_channel.name = 'intensity'
            particle_msg.channels.append(intensity_channel)
            for p in particles:
                pt = Point32()
                pt.x = float(p.pose[X])
                pt.y = float(p.pose[Y])
                pt.z = .05
                particle_msg.points.append(pt)
                intensity_channel.values.append(p.weight)
            self._particle_publisher.publish(particle_msg)

            poses = np.array([p.pose for p in particles], dtype=np.float32)
            median_pose = np.mean(poses, axis=0)
            self._pose_history.append(np.concatenate([self._groundtruth.pose, median_pose], axis=0))
            if len(self._pose_history) % 10:
                with open(self._temp_file, 'a') as fp:
                    fp.write('\n'.join(','.join(str(v) for v in p) for p in self._pose_history) + '\n')
                    self._pose_history = []


def run(args):
    rclpy.init()
    np.set_printoptions(formatter={'float': '{: 0.3f}'.format})
    localization = Localization()

    rclpy.spin(localization)

    localization.destroy_node()
    rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Runs a particle filter')
    args, unknown = parser.parse_known_args()
    run(args)


if __name__ == '__main__':
    main()
