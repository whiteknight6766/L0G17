import time
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
from adafruit_extended_bus import ExtendedI2C as I2C


from std_msgs.msg import String
import sys
import termios
import tty



#i2c = board.I2C()  # uses board.SCL and board.SDA
i2c = I2C(1)

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)
pca.frequency = 60

servo1 = servo.Servo(pca.channels[0])

        
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return(key)

def ServoCalibrate():
    servo1.angle = 90
    time.sleep(.5)
    a = 90
    for i in range(91):
        print("Servo Angle: " + str(a))
        servo1.angle = a
        key = get_key()
        if key == ' ':
            a = a - 1
        else:
            a = a - 1
    servo1.angle = 90
    a = 90
    for i in range(91):
        print("Servo Angle: " + str(a))
        servo1.angle = a
        key = get_key()
        if key == ' ':
            a = a + 1
        else:
            a = a + 1
    servo1.angle = 90
        
ServoCalibrate()

pca.deinit()
