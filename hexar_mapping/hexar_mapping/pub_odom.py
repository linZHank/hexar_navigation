import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry


class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odom_publisher')
        self.odom_pub = self.create_publisher(Odometry, 'hexar/odom', 10)
        self.odom_pub_timer = self.create_timer(0.5, self.odom_pub_timer_cb)
        self.x = 0.
        self.y = 0.
        self.th = 0.
        self.vx = 0.
        self.vy = 0.
        self.vth = 0.
        self.cur_time = self.get_clock().now()
        self.pre_time = self.get_clock().now()
        # self.i = 0

    def odom_pub_timer_cb(self):
        self.cur_time = self.get_clock().now()
        dt = (self.cur_time - self.pre_time).nanoseconds * 1e-9  # convert to seconds
        delta_th = self.vth * dt
        # msg = Odometry()
        # msg.header.stamp = current_time
        # msg.header.frame_id = "odom"
        # msg.child_frame_id = "base_link"
        # msg.pose.pose.position.x = x
        # msg.pose.pose.position.y = y
        # msg.pose.pose.position.z = z
        # msg.pose.pose.orientation = quat
        # self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    odom_publisher = OdometryPublisher()

    rclpy.spin(odom_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
