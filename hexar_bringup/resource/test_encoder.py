from time import sleep
from gpiozero import Robot, LED
import serial

# setup motors
en_l = LED(26)
en_r = LED(12)
en_l.on()
en_r.on()
bot = Robot((20, 21), (6, 13))
left_dutycycle = 0.5
right_dutycycle = 0.5
# setup serial port
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
ser.reset_input_buffer()
left_omega = 0.0
right_omega = 0.0

# main loop
for _ in range(100):  
    if ser.in_waiting > 0:
        line = ser.readline()
        if b'\xff' in line or b'\xfe' in line:
            print('cannot decode')
            continue
        speeds = line.decode('utf-8').rstrip().split(',')
        left_omega = float(speeds[0])
        right_omega = float(speeds[1])
    # drive motors
    bot.left_motor.forward(left_dutycycle)
    bot.right_motor.forward(right_dutycycle)
    sleep(.02)
    print(
        "---\n",
        f"leftwheel_speed: {left_omega} \n", \
        f"rightwheel_speed: {right_omega}"
    )

bot.stop()
en_l.off()
en_r.off()

