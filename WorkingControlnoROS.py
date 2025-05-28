import time
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
from adafruit_extended_bus import ExtendedI2C as I2C
import sys
import termios
import tty

i2c = I2C(1)
pca = PCA9685(i2c)
pca.frequency = 100
steering = servo.Servo(pca.channels[0])
forward = servo.Servo(pca.channels[14])
reverse = servo.Servo(pca.channels[15])

steering.angle = 80
time.sleep(.5)
a = 80
for i in range(70):
    steering.angle = a
    a = a - 1
    time.sleep(0.01)

steering.angle = 80
a = 80
for i in range (70):
    steering.angle = a
    a = a + 1
    time.sleep(0.01)

steering.angle = 80


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
    while True:
        key = get_key()
        if key == "q" or key == "Q":
            #Go Forward and left
            s = 150
            f = 140
            b = 0
        if key == "w" or key == "W":
            #Go Forward
            s = 80
            f = 140
            b = 0
        if key == "e" or key == "E":
            #Go Right and Forward
            s = 10
            f = 140
            b = 0
        if key == "a" or key == "A":
            #Go right and backward (turns car left)
            s = 10
            f = 0
            b = 140
        if key == "s" or key == "S":
            #Go backward
            s = 80
            f = 0
            b = 140
        if key == "d" or key == "D":
            #Go left and backward ( turns car left)
            s = 150
            f = 0
            b = 140
        if key == " " or key == " ":
            #Go left and backward ( turns car left)
            s = 80
            f = 0
            b = 0
            #pca.deinit()
            #sys.exit()
        if key == "r" or key == "R":
            steering.angle = 80
            forward.angle = 0
            reverse.angle = 0
            return()
        steering.angle = s
        forward.angle = f
        reverse.angle = b
ServoCalibrate()
pca.deinit()

