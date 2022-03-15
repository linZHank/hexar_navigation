from time import sleep
from gpiozero import Robot, LED

# setup motors
en_l = LED(26)
en_r = LED(12)
en_l.on()
en_r.on()
bot = Robot((20, 21), (6, 13))
left_dutycycle = 0.35
right_dutycycle = 0.4
# setup serial port
# main loop
for _ in range(400):  
    # drive motors
    bot.left_motor.forward(left_dutycycle)
    bot.right_motor.forward(right_dutycycle)
    sleep(.02)

bot.stop()
en_l.off()
en_r.off()


