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
        # setup robot driver
        self.left_motor = Motor(20, 21)
        self.right_motor = Motor(6, 13)
        self.en_l = LED(26)
        self.en_r = LED(12)
        self.en_l.on()
        self.en_r.on()
        # setup velocity publisher
        self.vel_pub = self.create_publisher(Twist, 'hexar/velocity', 1)
        self.vel_pub_timer = self.create_timer(0.01, self.vel_pub_cb)
        # setup twist listener
        self.cmdv_sub = self.create_subscription(
            Twist, 
            'cmd_vel', 
            self.cmdv_sub_cb, 
            10
        )
        self.cmdv_sub  # prevent unused variable warning
        # setup pid controller
        self.ctrl_timer = self.create_timer(0.02, self.ctrl_cb)
        # constants
        self.WHEEL_RADIUS = 0.08
        self.WHEEL_SEPARATION = 0.19
        self.K_P = 0.01
        # variables
        self.lin_x = 0.
        self.ang_z = 0.
        self.targ_lin_x = 0.
        self.targ_ang_z = 0.
        self.lwhl_dir = 0
        self.rwhl_dir = 0
        self.lwhl_spd = 0.
        self.rwhl_spd = 0.
        self.lwhl_vel = 0.
        self.rwhl_vel = 0.
        self.targ_lwhl_vel = 0.
        self.targ_rwhl_vel = 0.
        self.dutycycle_left = 0.
        self.dutycycle_right = 0.

    def cmdv_sub_cb(self, msg):
        self.targ_lin_x = msg.linear.x
        self.targ_ang_z = msg.angular.z
        # self.get_logger().info('Target Velocity: "%s"' % msg)

    def vel_pub_cb(self):
        """
        Callback function in vel_pub_timer
        Publish robot's velocity every 0.01 seconds
        """
        if self.ser.in_waiting > 0:
            self.bytes_ = self.ser.readline()
            if not b'\xfe' in self.bytes_ and not b'\xff' in self.bytes_:
                spd_str = self.bytes_.decode('utf-8').rstrip() 
                spd_list = spd_str.split(",")  # wheel speeds, rad/s
                if len(spd_list) == 2:
                    self.lwhl_spd = float(spd_list[0])  
                    self.rwhl_spd = float(spd_list[1])
        # print(f"lwhl speed, rwhl speed: {self.lwhl_spd, self.rwhl_spd}")
        # compute wheel linear velocity
        self.lwhl_vel = self.lwhl_dir * self.lwhl_spd
        self.rwhl_vel = self.rwhl_dir * self.rwhl_spd
        lwhl_vel_lin = self.lwhl_vel * self.WHEEL_RADIUS
        rwhl_vel_lin = self.rwhl_vel * self.WHEEL_RADIUS
        # print(f"lwhl linear, rwhl linear: {lwhl_vel_lin, rwhl_vel_lin}")
        # 
        msg = Twist()
        self.lin_x = (lwhl_vel_lin + rwhl_vel_lin) * .5
        self.ang_z = (rwhl_vel_lin - lwhl_vel_lin) / self.WHEEL_SEPARATION
        msg.linear.x = self.lin_x
        msg.angular.z = self.ang_z
        self.vel_pub.publish(msg)
        self.get_logger().info(f"Actual Velocity: {msg}")

    def ctrl_cb(self):
        self.targ_lwhl_vel = (self.targ_lin_x - (self.targ_ang_z *self.WHEEL_SEPARATION) * .5)
        self.targ_rwhl_vel = (self.targ_lin_x + (self.targ_ang_z *self.WHEEL_SEPARATION) * .5)
        # print(f"target wheel speed: {self.targ_lwhl_vel, self.targ_rwhl_vel}")
        # set wheel directions based on target wheel velocity
        if self.targ_lwhl_vel > 0:
            self.lwhl_dir = 1
        elif self.targ_lwhl_vel < 0:
            self.lwhl_dir = -1
        else:
            self.lwhl_dir = 0
        if self.targ_rwhl_vel > 0:
            self.rwhl_dir = 1
        elif self.targ_rwhl_vel < 0:
            self.rwhl_dir = -1
        else:
            self.rwhl_dir = 0
        # print(f"wheel direction: {self.lwhl_dir, self.rwhl_dir}")  # debug
        # calculate duty cycle change
        lerr = abs(self.targ_lwhl_vel) - abs(self.lwhl_vel)
        ldcinc = self.K_P * lerr  # duty cycle increment
        # print(f"left error, left dutycycle increment: {lerr, ldcinc}")
        self.dutycycle_left += ldcinc
        if self.dutycycle_left >= 1.:
            self.dutycycle_left = 1.
        elif self.dutycycle_left <= 0.:
            self.dutycycle_left = 0
        rerr = abs(self.targ_rwhl_vel) - abs(self.rwhl_vel)
        rdcinc = self.K_P * rerr
        self.dutycycle_right += rdcinc
        if self.dutycycle_right >= 1.:
            self.dutycycle_right = 1.
        elif self.dutycycle_right <= 0.:
            self.dutycycle_right = 0
        print(f"left/right dutycycle: {self.dutycycle_left, self.dutycycle_right}")
        # drive motors
        if self.lwhl_dir > 0:
            self.left_motor.forward(self.dutycycle_left)
        elif self.lwhl_dir < 0:
            self.left_motor.backward(self.dutycycle_left)
        else:
            self.left_motor.stop()
        if self.rwhl_dir > 0:
            self.right_motor.forward(self.dutycycle_right)
        elif self.lwhl_dir < 0:
            self.right_motor.backward(self.dutycycle_right)
        else:
            self.right_motor.stop()
        # self.left_motor.forward(.35)  # for test only
        # self.right_motor.forward(.35)


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
