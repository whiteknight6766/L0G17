# L0G17
This is the code repo for the ROS2 scripts to drive Mr Baddelley's MSE-6 shell.

There are two calibration files. The steering calibration allows you to step through and find what angle in center as well as left and right extreemes. Mr Baddeley's files call for an ackerman steering rack which does not allow for 180 degrees of servo movement. By finding the extreemes, you can hard code those values into the control software. This prevents undue stress on the steering bar and frame as the servo throws the steering angles around slamming the two pieces together. I found there was 70 degrees of free movement on either side of center. My center was 80 and left and right are 10 and 150. 

Movement calibration was not done via the script presented. It SHOULD have been correct and technically did make the motor move and adjust the power. However, I found it was about a 10th as fast as directly connecting the battery to the motor. Instead I found that mapping the steering servo calibration script to the motor pins (14,15) improved the situation dramatically. I was able to get the motor to spin faster than I wanted by useing the servo.angle() method instead of servo.fraction() method. Documentation for python Servo library indicates these should be the same. they are not. 

Both of those sets of values were brought into the Control script. It currently runs "just fine" except its not controlled by ROS2. The internet seems to think that if I initiate the i2c and servo libraries inside the class then it should be callable in a ROS callback as "self.servo.angle = 80." This is also incorrect. The code produced no errors and would even print out the correct values that should be sent to the servos, however the values would never get sent. 

More work is required. 

TODO:
Use ROS callbacks to control servos
Code out GPS sensors
Find an odometer mounting position

Unknowns:
Camera
Lidar
SLAM
