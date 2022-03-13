import serial
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry


class HexaRobot(Node):

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
            spd_list = spd_str.split(",")  # wheel speeds, rad/s
            whl_spd_l = float(spd_list[0])  
            whl_spd_r = float(spd_list[1])
        # get wheel directions
        dir_l = self.get_parameter('/hexar/lwhl_dir').get_parameter_value().integer_value
        dir_r = self.get_parameter('/hexar/rwhl_dir').get_parameter_value().integer_value
            
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

    hexar = HexaRobot()

    rclpy.spin(hexar)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hexar.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
