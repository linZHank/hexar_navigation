import serial
from gpiozero import LED, Robot
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class HexaRobot(Node):

    def __init__(self):
        super().__init__('odom_publisher')
        # setup serial comm
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        self.ser.reset_input_buffer()
        self.bytes = self.ser.readline()
        while (b'\xfe' in self.bytes or b'\xff' in self.bytes):
            continue
        # setup robot driver
        self.driver = Robot((20, 21), (6, 13))
        self.en_l = LED(26)
        self.en_r = LED(12)
        self.en_l.on()
        self.en_r.on()
        # setup velocity publisher
        self.vel_pub_ = self.create_publisher(Twist, 'hexar/velocity', 10)
        self.vel_pub_timer = self.create_timer(0.1, self.vel_pub_cb)
        # params
        self.dir_l = 0
        self.dir_r = 0
        # self.i = 0

    def vel_pub_cb(self):
        if self.ser.in_waiting > 0:
            self.bytes = self.ser.readline()
            spd_str = self.bytes.decode('utf-8').rstrip() 
            spd_list = spd_str.split(",")  # wheel speeds, rad/s
            whl_spd_l = float(spd_list[0])  
            whl_spd_r = float(spd_list[1])
        # compute wheel linear velocity
        wvel_l = self.dir_l * whl_spd_l * .08  # wheel_radius = .08
        wvel_r = self.dir_r * whl_spd_r * .08  
        # 
        msg = Twist()
        msg.linear.x = (wvel_l + wvel_r) * .5
        msg.angular.z = (wvel_r - wvel_l) / .19  # wheel_separation = .19
        self.vel_pub_.publish(msg)
        self.get_logger().debug('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    hexar = HexaRobot()

    rclpy.spin(hexar)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hexar.en_l.off()
    hexar.en_r.off()
    hexar.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
