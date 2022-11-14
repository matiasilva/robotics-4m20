import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SimpleNode(Node):

    def __init__(self):
        super().__init__('hello_world')
        self.publisher_ = self.create_publisher(Twist, 'robot0/cmd_vel', 5)
        timer_period = .2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.1
        vel_msg.angular.z = 0.2
        self.publisher_.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    simple_node = SimpleNode()
    rclpy.spin(simple_node)
    simple_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
