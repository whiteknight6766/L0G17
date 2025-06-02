import time
from datetime import datetime
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import adafruit_mpu6050
from adafruit_extended_bus import ExtendedI2C as I2C
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from pydub import AudioSegment
from pydub.playback import play

class MotorComms():
    def __init__(self):
        i2c = I2C(1)
        pca = PCA9685(i2c)
        pca.frequency = 100
        self.steering = servo.Servo(pca.channels[0])
        self.forward = servo.Servo(pca.channels[14])
        self.reverse = servo.Servo(pca.channels[15])
    
    def activate(self, s, f, b):
        self.steering.angle = s
        self.forward.angle = f
        self.reverse.angle = b

class Speakers():
        def __init__(self, pubName):
            super().__init__(pubName)
            self.subscription = self.create_subscription(String, 'L0G17_Mood', self.listener_callback, 10)
            self.subscription  # prevent unused variable warning
            self.get_logger().info(f'Loaded Mood Listner')
            
        def listener_callback(self, msg):    
            self.mesgTime = datetime.now()
            #self.get_logger().info(f'Key pressed: {msg}')
            
            if msg.data == "1":
                # for playing wav file
                song = AudioSegment.from_wav("/home/ubuntu/sounds/SW01_Vehicles_MSE6_MouseDroid_General_VAR_01 1 0 0.wav")
                play(song)
            if msg.data == "2":
                # for playing wav file
                song = AudioSegment.from_wav("/home/ubuntu/sounds/SW01_Vehicles_MSE6_MouseDroid_RunAway_VAR_02 2 1 0.wav") 
                play(song)
            if msg.data == "3":
                # for playing wav file
                song = AudioSegment.from_wav("/home/ubuntu/sounds/SW01_Vehicles_MSE6_MouseDroid_General_VAR_02 0 0 0.wav")
                play(song)
            if msg.data == "4":
                # for playing wav file
                song = AudioSegment.from_wav("/home/ubuntu/sounds/SW01_Vehicles_MSE6_MouseDroid_Alert_VAR_01 0 5 0.wav") #Same After this one
                play(song)
            if msg.data == "5":
                # for playing wav file
                song = AudioSegment.from_wav("/home/ubuntu/sounds/SW01_Vehicles_MSE6_MouseDroid_General_VAR_01 1 0 0.wav")
                play(song)
            if msg.data == "6":
                # for playing wav file
                song = AudioSegment.from_wav("/home/ubuntu/sounds/SW01_Vehicles_MSE6_MouseDroid_General_VAR_01 1 0 0.wav")
                play(song)
            if msg.data == "7":
                # for playing wav file
                song = AudioSegment.from_wav("/home/ubuntu/sounds/SW01_Vehicles_MSE6_MouseDroid_General_VAR_01 1 0 0.wav")
                play(song)
            if msg.data == "8":
                # for playing wav file
                song = AudioSegment.from_wav("/home/ubuntu/sounds/SW01_Vehicles_MSE6_MouseDroid_General_VAR_01 1 0 0.wav")
                play(song)
            if msg.data == "9":
                # for playing wav file
                song = AudioSegment.from_wav("/home/ubuntu/sounds/SW01_Vehicles_MSE6_MouseDroid_General_VAR_01 1 0 0.wav")
                play(song)
            if msg.data == "0":
                # for playing wav file
                song = AudioSegment.from_wav("/home/ubuntu/sounds/SW01_Vehicles_MSE6_MouseDroid_General_VAR_01 1 0 0.wav")
                play(song)
    
class HeartBeat(Node):
    def __init__(self):
        super().__init__("MSE_HB_L0G17")
        self.publisherhb = self.create_publisher(String, "MSE_HB_L0G17", 1)
        self.get_logger().info(f'loaded publisher HB')
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.timer = self.create_timer(timer_period, self.publish_HB)

    def timer_callback(self):
        msg = String()
        msg.data = "L0G17"
        self.publisherhb.publish(msg)
        #self.get_logger().info(f'Published HB')

class Gyro(Node):
    def __init__(self):
        super().__init__("MSE_GYRO_L0G17")
        self.publisherhb = self.create_publisher(Twist, "MSE_GYRO_L0G17", 10)
        self.get_logger().info(f'loaded publisher GYRO')
        i2c = I2C(1)
        self.mpu = adafruit_mpu6050.MPU6050(i2c)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.timer = self.create_timer(timer_period, self.publish_HB)

    def timer_callback(self):
        vel_L0G17 = Twist()
        accx, accy, accz = self.mpu.acceleration
        gyrox, gyroy, gyroz = self.mpu.gyro
        vel_L0G17.linear.x = accx
        vel_L0G17.linear.y = accy
        vel_L0G17.linear.z = accz
        vel_L0G17.angular.x = gyrox
        vel_L0G17.angular.y = gyroy
        vel_L0G17.angular.z = gyroz
        self.publisherhb.publish(vel_L0G17)
        
class Controller_HB(Node):
    def __init__(self, pubName):
        super().__init__(pubName)
        self.subscription = self.create_subscription(String, 'controller_hb', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info(f'Loaded Controller HB Listner')
        timer_period = 0.5  # seconds
        self.mesgTime = datetime.now()
        self.nowTime = datetime.now()
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        global stearingServo
        global moveFoward
        global moveBackward
        self.nowTime = datetime.now()
        delta = self.nowTime - self.mesgTime
        if delta.seconds >= 1:
            # Panic and stop
            stearingServo.angle = 90
            moveFoward.angle = 0
            moveBackward.angle = 0
        
    def listener_callback(self, msg):    
        self.mesgTime = datetime.now()
        #self.get_logger().info(f'Key pressed: {msg}')

class KeyboardAction(Node):

    def __init__(self, pubName):
        super().__init__(pubName)
        self.motorcomms = MotorComms()
        self.subscription = self.create_subscription(String, 'controller_cmd', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info(f'Loaded Movement')
        
    def listener_callback(self, msg):
        if msg.data == "q" or msg.data == "Q":
            #Go Forward and left
            s = 10
            f = 120
            b = 0
        if msg.data == "w" or msg.data == "W":
            #Go Forward
            s = 80
            f = 120
            b = 0
        if msg.data == "e" or msg.data == "E":
            #Go Right and Forward
            s = 150
            f = 120
            b = 0
        if msg.data == "a" or msg.data == "A":
            #Go right and backward (turns car left)
            s = 150
            f = 0
            b = 120
        if msg.data == "s" or msg.data == "S":
            #Go backward
            s = 80
            f = 0
            b = 120
        if msg.data == "d" or msg.data == "D":
            #Go left and backward ( turns car left)
            s = 10
            f = 0
            b = 120
        if msg.data == " " or msg.data == " ":
            #Go left and backward ( turns car left)
            s = 80
            f = 0
            b = 0
        if msg.data == "r" or msg.data == "r":
            #Go left and backward ( turns car left)
            s = 80
            f = 0
            b = 0
        self.get_logger().info(f'Key pressed: {msg}, setting servos to {s}, {f}, {b}')
        self.motorcomms.activate(s, f, b)

class BatteryLevel(Node):
  
    def __init__(self, pubName):
        super().__init__(pubName)
        self.publisherhb = self.create_publisher(String, pubName, 1)
        self.get_logger().info(f'Loaded Battery Level')
        timer_period = 10  # seconds
        msg = Num()
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        #talk to max14095 chip and get varialble
        self.msg.data = nothingyet #variable from i2c
        self.publisherhb.publish(msg)


def main(args=None):
    #ALSO NEED AN ODOMETER 
    rclpy.init()
    exec = SingleThreadedExecutor()
    exec.add_node(HeartBeat())
    exec.add_node(Gyro())
    exec.add_node(Controller_HB("L0g17_HB_Listner"))
    exec.add_node(KeyboardAction("L0g17_KB_Listner"))
    exec.spin() # Also blocking
    rclpy.shutdown()
    #pca.deinit() This shouldn't deinit because PCA isn't defined here, we need to find ROS Shutdown of classes and deinit there. 


if __name__ == '__main__':
    main()
