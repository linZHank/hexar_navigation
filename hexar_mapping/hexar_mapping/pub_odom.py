import serial
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry


class OdomPublish(Node):

    def __init__(self):
        super().__init__('odom_publisher')
        # setup serial comm
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        self.ser.reset_input_buffer()
        self.bytes = self.ser.readline()
        while (b'\xfe' in self.bytes or b'\xff' in self.bytes):
            continue
        self.odom_pub_ = self.create_publisher(Odometry, 'hexar/odom', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            self.bytes = self.ser.readline()
            spd_str = self.bytes.decode('utf-8').rstrip() 

            # split the string into a list ("," is where it splits)
            # leftCPS, rightCPS, linear_l, linear_r, linear, angular
            spd_list = spd_str.split(",")
            lin_ = float(spd_list[0])
            ang = float(spd_list[1])
            
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation = quat
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
