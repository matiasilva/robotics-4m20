#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import subprocess
import time


class Clock(Node):
  def __init__(self):
    super().__init__('get_time')
    self._subscriber = self.create_subscription(String, '/date_time', self.callback, 1)

    self._ready = False

    subprocess.call(['sudo echo "Authenticated"'], shell=True)

    timer_period = 0.01 # seconds
    self._timer = self.create_timer(timer_period, self.timer_cb)

  def callback(self, msg):
    if not self._ready:
      subprocess.call(['sudo date +%F%T -s "{}"'.format(msg.data)], shell=True)
      self.get_logger().info('Time set to ' + msg.data)
      self._ready = True

  def timer_cb(self):
    if self.ready:
      raise SystemExit

  @property
  def ready(self):
    return self._ready



def main(args=None):
  rclpy.init(args=args)

  clock_node = Clock()

  try:
    rclpy.spin(clock_node)
  except SystemExit:  # <--- process the exception
    rclpy.logging.get_logger("Exiting").info('Clock received')

  clock_node.destroy_node()

  rclpy.shutdown()

if __name__ == '__main__':
  main()
