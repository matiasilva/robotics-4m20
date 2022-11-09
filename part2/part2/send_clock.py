#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import rclpy
from rclpy.node import Node

import datetime
from std_msgs.msg import String


class SendClockNode(Node):

  def __init__(self):
    super().__init__('send_clock')
    self._publisher = self.create_publisher(String, '/date_time', 1)
    timer_period = .001  # seconds
    self._timer = self.create_timer(timer_period, self.timer_cb)

  def timer_cb(self):
    now, _ = self.get_clock().now().seconds_nanoseconds()
    date_time = datetime.datetime.fromtimestamp(int(now)).strftime('%Y%m%d %H:%M:%S')
    msg = String()
    msg.data = date_time
    self._publisher.publish(msg)



def main(args=None):
  rclpy.init(args=args)

  send_clock_node = SendClockNode()

  rclpy.spin(send_clock_node)
  send_clock_node.destroy_node()

  rclpy.shutdown()


if __name__ == '__main__':
  main()
