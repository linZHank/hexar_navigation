import serial
from gpiozero import LED, Motor
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class HexaRobotCore(Node):

    def __init__(self):
        super().__init__('hexa_subr_pubr')
        # setup serial comm
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        self.ser.reset_input_buffer()
        self.bytes = self.ser.readline()
        while (b'\xfe' in self.bytes or b'\xff' in self.bytes):
            continue
        # setup robot driver
        self.left_motor = Motor(20, 21)
        self.right_motor = Motor(6, 13)
        self.en_l = LED(26)
        self.en_r = LED(12)
        self.en_l.on()
        self.en_r.on()
        # setup velocity publisher
        self.vel_pub = self.create_publisher(Twist, 'hexar/velocity', 10)
        self.vel_pub_timer = self.create_timer(0.1, self.vel_pub_cb)
        # setup twist listener
        self.cmdv_sub = self.create_subscription(
            Twist, 
            'cmd_vel', 
            self.cmdv_sub_cb, 
            10
        )
        self.cmdv_sub  # prevent unused variable warning
        # constants
        self.WHEEL_RADIUS = 0.08
        self.WHEEL_SEPARATION = 0.19
        # variables
        self.lwhl_dir = 0
        self.rwhl_dir = 0
        self.lin_x = 0.
        self.ang_z = 0.
        self.lwhl_vel = 0.
        self.rwhl_vel = 0.
        self.k_p = 0.003
        self.dutycycle_left = 0.
        self.dutycycle_right = 0.

    def cmdv_sub_cb(self, msg):
        targ_lin_x = msg.linear.x
        targ_ang_z = msg.angular.z
        targ_lwhl_vel = (targ_lin_x - (targ_ang_z *self.WHEEL_SEPARATION) * .5)
        targ_rwhl_vel = (targ_lin_x + (targ_ang_z *self.WHEEL_SEPARATION) * .5)
        # set wheel directions based on target wheel velocity
        if targ_lwhl_vel > 0:
            self.lwhl_dir = 1
        elif targ_lwhl_vel < 0:
            self.lwhl_dir = -1
        else:
            self.lwhl_dir = 0
        if targ_rwhl_vel > 0:
            self.rwhl_dir = 1
        elif targ_rwhl_vel < 0:
            self.rwhl_dir = -1
        else:
            self.rwhl_dir = 0
        # calculate duty cycle change
        lerr = abs(targ_lwhl_vel) - abs(self.lwhl_vel)
        ldcinc = self.k_p * lerr  # duty cycle increment
        self.dutycycle_left += ldcinc
        if self.dutycycle_left >= 1.:
            self.dutycycle_left = 1.
        elif self.dutycycle_left <= 0.:
            self.dutycycle_left = 0
        rerr = abs(targ_rwhl_vel) - abs(self.rwhl_vel)
        rdcinc = self.k_p * rerr
        self.dutycycle_right += rdcinc
        if self.dutycycle_right >= 1.:
            self.dutycycle_right = 1.
        elif self.dutycycle_right <= 0.:
            self.dutycycle_right = 0
        # drive motors
        if self.lwhl_dir > 0:
            self.left_motor.forward(self.dutycycle_left)
        elif self.lwhl_dir < 0: 
            self.left_motor.backward(self.dutycycle_left)
        if self.rwhl_dir > 0:
            self.right_motor.forward(self.dutycycle_right)
        elif self.lwhl_dir < 0: 
            self.right_motor.backward(self.dutycycle_right)

        self.get_logger().info('Target Velocity: "%s"' % msg)

    def vel_pub_cb(self):
        if self.ser.in_waiting > 0:
            self.bytes_ = self.ser.readline()
            spd_str = self.bytes_.decode('utf-8').rstrip() 
            spd_list = spd_str.split(",")  # wheel speeds, rad/s
            lwhl_spd = float(spd_list[0])  
            rwhl_spd = float(spd_list[1])
        # compute wheel linear velocity
        self.lwhl_vel = self.lwhl_dir * lwhl_spd
        self.rwhl_vel = self.rwhl_dir * rwhl_spd
        lwhl_vel_lin = self.lwhl_vel * self.WHEEL_RADIUS
        rwhl_vel_lin = self.rwhl_vel * self.WHEEL_RADIUS
        # 
        msg = Twist()
        self.lin_x = (lwhl_vel_lin + rwhl_vel_lin) * .5
        self.ang_z = (rwhl_vel_lin - lwhl_vel_lin) / self.WHEEL_SEPARATION
        msg.linear.x = self.lin_x
        msg.angular.z = self.ang_z
        self.vel_pub.publish(msg)
        self.get_logger().debug('Actual Velocity: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)

    hexar = HexaRobotCore()

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
