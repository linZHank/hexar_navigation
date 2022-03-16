import numpy as np
import rclpy
from rclpy.node import Node
import tf_transformations

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odom_publisher')
        # subscriber
        self.vel_sub = self.create_subscription(
            Twist, 
            '/hexar/velocity', 
            self.vel_sub_cb, 
            1
        )
        self.cmdv_sub  # prevent unused variable warning
        # publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.odom_pub_timer = self.create_timer(0.2, self.odom_pub_timer_cb)
        # variables
        self.x = 0.
        self.y = 0.
        self.th = 0.
        self.lin_x = 0.
        self.ang_z = 0.
        self.cur_time = self.get_clock().now()
        self.pre_time = self.get_clock().now()
        # self.i = 0

    def vel_sub_cb(self, msg):
        self.lin_x = msg.linear.x
        self.ang_z = msg.angular.z

    def odom_pub_timer_cb(self):
        # update pose
        self.cur_time = self.get_clock().now()
        dt = (self.cur_time - self.pre_time).nanoseconds * 1e-9  # convert to seconds
        delta_x = self.lin_x * np.cos(self.th) * dt
        delta_y = self.lin_x * np.sin(self.th) * dt
        delta_th = self.ang_z * dt
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        q = tf_transformations.quaternion_about_axis(self.th, (0, 0, 1))
        # prepare Odometry message
        msg = Odometry()
        msg.header.stamp = self.cur_time
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = self.z
        msg.pose.pose.orientation = q
        msg.twist.twist.linear.x = self.lin_x
        msg.twist.twist.angular.z = self.ang_z
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        # self.i += 1


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
