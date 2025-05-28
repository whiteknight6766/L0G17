import time
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
from adafruit_extended_bus import ExtendedI2C as I2C
import sys
import termios
import tty

#i2c = board.I2C()  # uses board.SCL and board.SDA
i2c = I2C(1)

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)
pca.frequency = 60

servo1 = servo.Servo(pca.channels[14])
        
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
    callFor = 0
    
    while callFor < 101:

        key = get_key()
        if key == 'w':
            if callFor == 100:
                pass
            else:
                callFor = callFor + 1
        elif key == 's':
            if callFor == 0:
                pass
            else:
                callFor = callFor - 1
        elif key == ' ':
            servo1.fraction = 0
            time.sleep(.05)
            pca.deinit()
            sys.exit()
        else:
            callFor = callFor
        fraction = callFor/100
        servo1.fraction = fraction
        print("Motor Power: " + str(callFor))
        
ServoCalibrate()

pca.deinit()

